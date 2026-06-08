#!/usr/bin/env python3
"""
Camera trigger server - listens for UDP commands from ESP32
Run: python3 camera_trigger_server.py
"""
import json
import io
import math
import mimetypes
import os
import re
import shlex
import shutil
import socket
import subprocess
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, quote, urlparse

import numpy as np
from PIL import Image

UDP_PORT = 8888
HTTP_PORT = 8080
SCRIPT_DIR = Path(__file__).resolve().parent
AUTOTUNE_SCRIPT = SCRIPT_DIR / 'camera_auto_tune.py'
AUTOTUNE_LOG_PATH = SCRIPT_DIR / 'camera_autotune.log'
CAMERA_STATUS_SETTINGS = {
        'f-number': 'f-number',
        'iso': 'iso',
        'shutterspeed': 'shutterspeed',
        'battery-level': '/main/other/d218',
        'color-temperature': '/main/imgsettings/colortemperature',
        'exposure-metering-mode': '/main/capturesettings/exposuremetermode',
        'still-capture-mode': '/main/capturesettings/capturemode',
        'exposure-program': '/main/capturesettings/expprogram',
        'focus-mode': '/main/capturesettings/focusmode',
        'image-quality': '/main/capturesettings/imagequality',
        'flash-mode': '/main/capturesettings/flashmode',
        'white-balance': '/main/imgsettings/whitebalance',
        'image-size': '/main/imgsettings/imagesize',
        'capture-target': '/main/settings/capturetarget',
}
CAMERA_WRITABLE_SETTINGS = [
        'f-number',
        'iso',
        'shutterspeed',
        'exposure-metering-mode',
        'still-capture-mode',
        'exposure-program',
        'focus-mode',
        'image-quality',
        'flash-mode',
        'white-balance',
        'image-size',
        'capture-target',
]

CAMERA_LOCK = threading.Lock()
STATUS_CACHE_LOCK = threading.Lock()
STATUS_REFRESH_LOCK = threading.Lock()
STATUS_CACHE = {}
STATUS_CACHE_AT = 0.0
STATUS_CACHE_TTL_S = 1.0

DETECT_CACHE_LOCK = threading.Lock()
DETECT_CACHE_AT = 0.0
DETECT_CACHE_TTL_S = 2.0
DETECT_CACHE_RESULT = (False, 'Camera detection has not run yet')
MOVIE_RECORDING = False
MOVIE_STATE_LOCK = threading.Lock()
AUTOTUNE_PROCESS = None
AUTOTUNE_LOG_HANDLE = None
AUTOTUNE_STATE_LOCK = threading.Lock()
AUTOTUNE_INFO_CACHE_LOCK = threading.Lock()
AUTOTUNE_INFO_CACHE = {
        'path': None,
        'mtime': None,
        'payload': None,
}
AUTOTUNE_IMAGE_EXTS = {'.jpg', '.jpeg', '.png'}
TIMELAPSE_ROOT = os.environ.get(
        'TIMELAPSE_ROOT',
        '/home/micah/projects/holy-grail/holy-grail-timelapse/timelapse',
)
ORIN_SSH_USER = os.environ.get('ORIN_SSH_USER', 'micah')
ORIN_SSH_HOST = os.environ.get('ORIN_SSH_HOST', '10.42.0.1')
ORIN_SSH_KEY = os.environ.get('ORIN_SSH_KEY', str(Path.home() / '.ssh' / 'id_ed25519_orin_nopass'))
SYNC_SCRIPT_PATH = SCRIPT_DIR / 'sync_orin_timelapse.sh'
SYNC_LOG_PATH = SCRIPT_DIR / 'sync_orin.log'
TIMELAPSE_PREVIEW_HTML_PATH = SCRIPT_DIR / 'timelapse_preview_original.html'
TIMELAPSE_EXPORT_DIR = SCRIPT_DIR / 'timelapse_exports'
TIMELAPSE_HIDDEN_DIRS_PATH = SCRIPT_DIR / '.timelapse_hidden_dirs.json'
SYNC_STATE_LOCK = threading.Lock()
SYNC_PROCESS = None
SYNC_FOLDER = None
SYNC_VIEW = 'optimal'
SYNC_STARTED_AT = None
SYNC_WATCH = False
SYNC_INTERVAL_SECONDS = 60
TIMELAPSE_AI_SUMMARY_PATH = SCRIPT_DIR / '.timelapse_ai_summary.json'
REMOTE_COUNTS_CACHE_LOCK = threading.Lock()
REMOTE_COUNTS_CACHE = {
        'optimal': {},
        'all': {},
}
REMOTE_LATEST_MTIME_CACHE_LOCK = threading.Lock()
REMOTE_LATEST_MTIME_CACHE = {
        'optimal': {},
        'all': {},
}
TIMELAPSE_METADATA_CACHE_LOCK = threading.Lock()
TIMELAPSE_METADATA_CACHE = {}

_ORIN_REACHABLE_LOCK = threading.Lock()
_ORIN_REACHABLE_STATE = {'reachable': None, 'checked_at': 0.0}
_ORIN_REACHABLE_TTL = 15.0
_ORIN_REACHABLE_PROBE_TIMEOUT = 2.0


def _orin_is_reachable():
        """2s TCP probe to port 22. Result cached 15s so a down ORIN skips all SSH calls instantly."""
        import socket as _socket
        now = time.monotonic()
        with _ORIN_REACHABLE_LOCK:
                if _ORIN_REACHABLE_STATE['reachable'] is not None and (now - _ORIN_REACHABLE_STATE['checked_at']) < _ORIN_REACHABLE_TTL:
                        return bool(_ORIN_REACHABLE_STATE['reachable'])
        try:
                with _socket.create_connection((ORIN_SSH_HOST, 22), timeout=_ORIN_REACHABLE_PROBE_TIMEOUT):
                        reachable = True
        except Exception:
                reachable = False
        with _ORIN_REACHABLE_LOCK:
                _ORIN_REACHABLE_STATE['reachable'] = reachable
                _ORIN_REACHABLE_STATE['checked_at'] = time.monotonic()
        return reachable


def build_luma_histogram(image_path, bins=64):
        with Image.open(image_path) as img:
                luma = np.asarray(img.convert('L'), dtype=np.uint8)
        hist, _ = np.histogram(luma, bins=bins, range=(0, 256))
        return hist.astype(int).tolist()


def estimate_adjustment_reason(metrics, deadband=0.15):
        if metrics.highlight_clip_pct > 1.0 and metrics.delta_ev >= -deadband:
                return 'Highlights clipping: darkening to recover detail.'
        if metrics.shadow_clip_pct > 2.0 and metrics.delta_ev <= deadband:
                return 'Shadows clipping: brightening to recover detail.'
        if metrics.delta_ev > deadband:
                return 'Scene is dark vs target: brighten exposure.'
        if metrics.delta_ev < -deadband:
                return 'Scene is bright vs target: darken exposure.'
        return 'Near target: hold unless clipping increases.'


def to_change_record(sign, text):
        if sign == '+':
                tone = 'green'
        elif sign == '-':
                tone = 'red'
        else:
                tone = 'neutral'
        return {
                'sign': sign,
                'text': text,
                'tone': tone,
        }


def build_capture_changes(iso, shutter_s, fstop, prev_iso, prev_shutter_s, prev_fstop):
        changes = {
                'iso': to_change_record('0', '0'),
                'shutter': to_change_record('0', '0 EV'),
                'fstop': to_change_record('0', '0 EV'),
        }

        if iso is not None and prev_iso is not None:
                diff_iso = int(iso) - int(prev_iso)
                if diff_iso > 0:
                        changes['iso'] = to_change_record('+', f'+{diff_iso}')
                elif diff_iso < 0:
                        changes['iso'] = to_change_record('-', str(diff_iso))

        if shutter_s is not None and prev_shutter_s is not None and shutter_s > 0 and prev_shutter_s > 0:
                ev_shutter = math.log2(float(shutter_s) / float(prev_shutter_s))
                if ev_shutter > 1e-6:
                        changes['shutter'] = to_change_record('+', f'+{ev_shutter:.2f} EV')
                elif ev_shutter < -1e-6:
                        changes['shutter'] = to_change_record('-', f'{ev_shutter:.2f} EV')

        if fstop is not None and prev_fstop is not None and fstop > 0 and prev_fstop > 0:
                ev_aperture = 2.0 * math.log2(float(prev_fstop) / float(fstop))
                if ev_aperture > 1e-6:
                        changes['fstop'] = to_change_record('+', f'+{ev_aperture:.2f} EV')
                elif ev_aperture < -1e-6:
                        changes['fstop'] = to_change_record('-', f'{ev_aperture:.2f} EV')

        return changes


def parse_numeric_value(raw_value, cast=float):
        if raw_value is None:
                return None
        text = str(raw_value).strip()
        if not text:
                return None
        try:
                return cast(text)
        except Exception:
                return None


def resolve_lowest_iso_choice():
        result = get_camera_setting('iso')
        if result.get('available'):
                choices = []
                for choice in result.get('choices', []):
                        match = ''.join(ch for ch in str(choice) if ch.isdigit())
                        if match:
                                try:
                                        choices.append(int(match))
                                except Exception:
                                        pass
                if choices:
                        return min(choices)
        return 100


def resolve_python_executable():
        candidates = [
                SCRIPT_DIR.parent / '.venv' / 'bin' / 'python',
                SCRIPT_DIR.parent / '.venv' / 'Scripts' / 'python.exe',
                Path(sys.executable),
        ]
        for candidate in candidates:
                if candidate and candidate.exists():
                        return str(candidate)
        return sys.executable


def _timelapse_name_is_valid(name):
        value = str(name or '').strip()
        if not value:
                return False
        if value == 'timelapse':
                return True
        if re.match(r'^timelapse_[0-9]{8}_[0-9]{6}$', value):
                return True
        # Also allow safe local-only timelapse folders used for ad-hoc imports.
        return _timelapse_local_dir_name_is_valid(value)


def _timelapse_local_dir_name_is_valid(name):
        value = str(name or '').strip()
        if not value:
                return False
        if value in ('.', '..'):
                return False
        if '/' in value or '\\' in value:
                return False
        return value.startswith('timelapse')


def _load_hidden_timelapse_dirs():
        try:
                if not TIMELAPSE_HIDDEN_DIRS_PATH.exists():
                        return set()
                payload = json.loads(TIMELAPSE_HIDDEN_DIRS_PATH.read_text(encoding='utf-8'))
                names = payload.get('dirs') if isinstance(payload, dict) else []
                if not isinstance(names, list):
                        return set()
                out = set()
                for raw in names:
                        name = str(raw or '').strip()
                        if _timelapse_local_dir_name_is_valid(name):
                                out.add(name)
                return out
        except Exception:
                return set()


def _save_hidden_timelapse_dirs(hidden_dirs):
        try:
                cleaned = sorted({
                        str(name or '').strip()
                        for name in (hidden_dirs or set())
                        if _timelapse_local_dir_name_is_valid(str(name or '').strip())
                })
                TIMELAPSE_HIDDEN_DIRS_PATH.write_text(
                        json.dumps({'dirs': cleaned}, indent=2),
                        encoding='utf-8',
                )
                return True
        except Exception:
                return False


def hide_timelapse_dir(dir_name):
        name = str(dir_name or '').strip()
        if not _timelapse_local_dir_name_is_valid(name):
                return False
        hidden = _load_hidden_timelapse_dirs()
        hidden.add(name)
        return _save_hidden_timelapse_dirs(hidden)


def delete_local_timelapse_dir(dir_name, source=''):
        folder = str(dir_name or '').strip()
        source_hint = str(source or '').strip().lower()
        if not _timelapse_local_dir_name_is_valid(folder):
                return {'ok': False, 'error': 'Invalid dir'}

        mirror_candidate = SCRIPT_DIR / 'timelapse_orin' / folder
        local_candidate = SCRIPT_DIR / folder

        if source_hint == 'remote':
                target = mirror_candidate
        elif source_hint == 'local':
                target = local_candidate
        else:
                if mirror_candidate.exists() and mirror_candidate.is_dir():
                        target = mirror_candidate
                else:
                        target = local_candidate

        try:
                resolved = target.resolve()
                root_resolved = SCRIPT_DIR.resolve()
                resolved.relative_to(root_resolved)
        except Exception:
                return {'ok': False, 'error': 'Path out of bounds'}

        if resolved.name != folder:
                return {'ok': False, 'error': 'Invalid target path'}

        if not resolved.exists() or not resolved.is_dir():
                hide_timelapse_dir(folder)
                return {
                        'ok': True,
                        'deleted': folder,
                        'source': source_hint or 'auto',
                        'path': str(resolved),
                        'hidden_only': True,
                        'message': 'Local directory already absent; folder hidden from list',
                }

        try:
                shutil.rmtree(resolved)
        except Exception as exc:
                return {'ok': False, 'error': f'Failed to delete directory: {exc}'}

        hide_timelapse_dir(folder)

        return {
                'ok': True,
                'deleted': folder,
                'source': source_hint or 'auto',
                'path': str(resolved),
        }


def delete_local_timelapse_frame(dir_name, frame_name):
        folder = str(dir_name or '').strip()
        frame = str(frame_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return {'ok': False, 'error': 'Invalid dir'}
        if not _frame_name_is_valid(frame):
                return {'ok': False, 'error': 'Invalid frame name'}

        deleted_paths = []
        removed_analysis_cache = 0
        removed_analysis_sidecar = 0

        delete_dirs = []
        seen_dirs = set()
        for preferred in ('optimal', 'all'):
                for path_obj in _timelapse_frame_dir_candidates(folder, preferred_view=preferred):
                        try:
                                resolved_dir = path_obj.resolve()
                                resolved_dir.relative_to(SCRIPT_DIR.resolve())
                        except Exception:
                                continue
                        key = str(resolved_dir)
                        if key in seen_dirs:
                                continue
                        seen_dirs.add(key)
                        delete_dirs.append(resolved_dir)

        for path_obj in delete_dirs:
                try:
                        candidate = (path_obj / frame).resolve()
                        candidate.relative_to(SCRIPT_DIR.resolve())
                except Exception:
                        continue

                if not candidate.exists() or not candidate.is_file():
                        continue

                try:
                        cache_path = _frame_analysis_cache_path_for_local(candidate, frame)
                except Exception:
                        cache_path = None
                if cache_path and cache_path.exists() and cache_path.is_file():
                        try:
                                cache_path.unlink()
                                removed_analysis_cache += 1
                        except Exception:
                                pass

                try:
                        analysis_path = candidate.parent.parent / '.sync_state' / 'analysis' / f"{Path(frame).stem}.json"
                        if analysis_path.exists() and analysis_path.is_file():
                                analysis_path.unlink()
                                removed_analysis_sidecar += 1
                except Exception:
                        pass

                try:
                        candidate.unlink()
                        deleted_paths.append(str(candidate))
                except Exception:
                        continue

        if not deleted_paths:
                return {'ok': False, 'error': 'Frame not found locally'}

        removed_mem_cache = 0
        key_hint = f"/{frame}:"
        with TIMELAPSE_METADATA_CACHE_LOCK:
                for key in list(TIMELAPSE_METADATA_CACHE.keys()):
                        if key_hint in str(key):
                                del TIMELAPSE_METADATA_CACHE[key]
                                removed_mem_cache += 1

        removed_ai_entries = 0
        if TIMELAPSE_AI_SUMMARY_PATH.exists():
                try:
                        payload = json.loads(TIMELAPSE_AI_SUMMARY_PATH.read_text(encoding='utf-8'))
                except Exception:
                        payload = {}
                if not isinstance(payload, dict):
                        payload = {}
                entries = payload.get('entries') if isinstance(payload.get('entries'), dict) else {}
                key = f"{folder}/{frame}"
                if key in entries:
                        del entries[key]
                        removed_ai_entries = 1
                payload['entries'] = entries
                try:
                        TIMELAPSE_AI_SUMMARY_PATH.write_text(json.dumps(payload, indent=2), encoding='utf-8')
                except Exception:
                        pass

        return {
                'ok': True,
                'dir': folder,
                'name': frame,
                'deleted_count': len(deleted_paths),
                'deleted_paths': deleted_paths,
                'cleared': {
                        'analysis_cache': removed_analysis_cache,
                        'analysis_sidecar': removed_analysis_sidecar,
                        'metadata_mem': removed_mem_cache,
                        'ai_entries': removed_ai_entries,
                },
        }


def delete_local_timelapse_frame_range(dir_name, start_name, end_name, view='optimal'):
        folder = str(dir_name or '').strip()
        start_frame = str(start_name or '').strip()
        end_frame = str(end_name or '').strip()
        view_mode = str(view or 'optimal').strip().lower()
        if view_mode not in ('optimal', 'all'):
                view_mode = 'optimal'

        if not _timelapse_name_is_valid(folder):
                return {'ok': False, 'error': 'Invalid dir'}
        if not _frame_name_is_valid(start_frame):
                return {'ok': False, 'error': 'Invalid start frame'}
        if not _frame_name_is_valid(end_frame):
                return {'ok': False, 'error': 'Invalid end frame'}

        frame_paths = _full_frame_paths_for_timelapse_dir(folder, preferred_view=view_mode, limit=500000)
        if not frame_paths:
                return {'ok': False, 'error': 'No local frames found for this timelapse'}

        names = [p.name for p in frame_paths]
        try:
                start_idx = names.index(start_frame)
        except Exception:
                return {'ok': False, 'error': 'Start frame not found in current local view'}
        try:
                end_idx = names.index(end_frame)
        except Exception:
                return {'ok': False, 'error': 'End frame not found in current local view'}

        lo = min(start_idx, end_idx)
        hi = max(start_idx, end_idx)
        targets = frame_paths[lo:hi + 1]
        if not targets:
                return {'ok': False, 'error': 'No frames selected for deletion'}

        target_names = [p.name for p in targets]

        delete_dirs = []
        seen_dirs = set()
        for preferred in ('optimal', 'all'):
                for path_obj in _timelapse_frame_dir_candidates(folder, preferred_view=preferred):
                        try:
                                resolved_dir = path_obj.resolve()
                                resolved_dir.relative_to(SCRIPT_DIR.resolve())
                        except Exception:
                                continue
                        key = str(resolved_dir)
                        if key in seen_dirs:
                                continue
                        seen_dirs.add(key)
                        delete_dirs.append(resolved_dir)

        deleted_names = []
        removed_analysis_cache = 0
        removed_analysis_sidecar = 0

        for frame in target_names:
                deleted_this_frame = False
                for path_obj in delete_dirs:
                        try:
                                candidate = (path_obj / frame).resolve()
                                candidate.relative_to(SCRIPT_DIR.resolve())
                        except Exception:
                                continue

                        if not candidate.exists() or not candidate.is_file():
                                continue

                        try:
                                cache_path = _frame_analysis_cache_path_for_local(candidate, frame)
                        except Exception:
                                cache_path = None
                        if cache_path and cache_path.exists() and cache_path.is_file():
                                try:
                                        cache_path.unlink()
                                        removed_analysis_cache += 1
                                except Exception:
                                        pass

                        try:
                                analysis_path = candidate.parent.parent / '.sync_state' / 'analysis' / f"{Path(frame).stem}.json"
                                if analysis_path.exists() and analysis_path.is_file():
                                        analysis_path.unlink()
                                        removed_analysis_sidecar += 1
                        except Exception:
                                pass

                        try:
                                candidate.unlink()
                                deleted_this_frame = True
                        except Exception:
                                continue

                if deleted_this_frame:
                        deleted_names.append(frame)

        if not deleted_names:
                return {'ok': False, 'error': 'No frames were deleted'}

        removed_mem_cache = 0
        name_hints = {f"/{name}:" for name in deleted_names}
        with TIMELAPSE_METADATA_CACHE_LOCK:
                for key in list(TIMELAPSE_METADATA_CACHE.keys()):
                        key_text = str(key)
                        if any(hint in key_text for hint in name_hints):
                                del TIMELAPSE_METADATA_CACHE[key]
                                removed_mem_cache += 1

        removed_ai_entries = 0
        if TIMELAPSE_AI_SUMMARY_PATH.exists():
                try:
                        payload = json.loads(TIMELAPSE_AI_SUMMARY_PATH.read_text(encoding='utf-8'))
                except Exception:
                        payload = {}
                if not isinstance(payload, dict):
                        payload = {}
                entries = payload.get('entries') if isinstance(payload.get('entries'), dict) else {}
                for name in deleted_names:
                        key = f"{folder}/{name}"
                        if key in entries:
                                del entries[key]
                                removed_ai_entries += 1
                payload['entries'] = entries
                try:
                        TIMELAPSE_AI_SUMMARY_PATH.write_text(json.dumps(payload, indent=2), encoding='utf-8')
                except Exception:
                        pass

        return {
                'ok': True,
                'dir': folder,
                'view': view_mode,
                'start_name': start_frame,
                'end_name': end_frame,
                'deleted_count': len(deleted_names),
                'first_deleted': deleted_names[0],
                'last_deleted': deleted_names[-1],
                'deleted_names': deleted_names,
                'cleared': {
                        'analysis_cache': removed_analysis_cache,
                        'analysis_sidecar': removed_analysis_sidecar,
                        'metadata_mem': removed_mem_cache,
                        'ai_entries': removed_ai_entries,
                },
        }


def _count_image_files(path_obj):
        try:
                if not path_obj.exists() or not path_obj.is_dir():
                        return 0
                count = 0
                for child in path_obj.iterdir():
                        if not child.is_file():
                                continue
                        lower_name = child.name.lower()
                        if lower_name.startswith('timelapse_') or lower_name.startswith('capt_'):
                                count += 1
                return count
        except Exception:
                return 0


def _latest_local_image_info(path_obj):
        try:
                if not path_obj.exists() or not path_obj.is_dir():
                        return {'name': None, 'mtime': None, 'path': None}
                candidates = []
                for child in path_obj.iterdir():
                        if not child.is_file():
                                continue
                        lower_name = child.name.lower()
                        if not (lower_name.startswith('timelapse_') or lower_name.startswith('capt_')):
                                continue
                        try:
                                mtime = child.stat().st_mtime
                        except Exception:
                                continue
                        seq = _frame_sequence_value(child.name)
                        # Prefer explicit frame sequence when present, then mtime.
                        seq_rank = seq if seq is not None else -1
                        candidates.append((seq_rank, mtime, child.name, child))
                if not candidates:
                        return {'name': None, 'mtime': None, 'path': None}
                candidates.sort(key=lambda item: (item[0], item[1], item[2]), reverse=True)
                _, mtime, _, best = candidates[0]
                return {'name': best.name, 'mtime': mtime, 'path': str(best)}
        except Exception:
                return {'name': None, 'mtime': None, 'path': None}


def _preview_frame_urls(path_obj, limit=80):
        try:
                if not path_obj.exists() or not path_obj.is_dir():
                        return []

                images = []
                for child in path_obj.iterdir():
                        if not child.is_file():
                                continue
                        lower_name = child.name.lower()
                        if not (lower_name.startswith('timelapse_') or lower_name.startswith('capt_')):
                                continue
                        images.append(child)

                if not images:
                        return []

                # Keep natural timelapse order while limiting payload size.
                images = sorted(images, key=lambda p: p.name)
                if len(images) > limit:
                        images = images[-limit:]

                out = []
                for img in images:
                        try:
                                mtime = int(img.stat().st_mtime)
                        except Exception:
                                mtime = 0
                        out.append(f"/api/timelapse-preview?path={quote(str(img))}&ts={mtime}")
                return out
        except Exception:
                return []


def _full_frame_urls_for_timelapse_dir(dir_name, preferred_view='optimal', limit=5000):
        folder = str(dir_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return []

        preferred = str(preferred_view or 'optimal').strip().lower()
        hard_limit = max(1, min(20000, int(limit or 5000)))

        if preferred == 'all':
                frame_paths = _merged_frame_paths_for_all_view(folder, limit=hard_limit)
                out = []
                for frame_path in frame_paths:
                        try:
                                mtime = int(frame_path.stat().st_mtime)
                        except Exception:
                                mtime = 0
                        out.append(f"/api/timelapse-preview?path={quote(str(frame_path))}&ts={mtime}")
                return out

        candidate_dirs = _timelapse_frame_dir_candidates(folder, preferred_view=preferred)
        for target in candidate_dirs:
                if not target.exists() or not target.is_dir():
                        continue
                frame_names = _list_local_frame_names_in_dir(target)
                if not frame_names:
                        continue
                if len(frame_names) > hard_limit:
                        frame_names = frame_names[-hard_limit:]

                out = []
                for frame_name in frame_names:
                        frame_path = target / frame_name
                        try:
                                mtime = int(frame_path.stat().st_mtime)
                        except Exception:
                                mtime = 0
                        out.append(f"/api/timelapse-preview?path={quote(str(frame_path))}&ts={mtime}")
                return out

        return []


def _full_frame_paths_for_timelapse_dir(dir_name, preferred_view='optimal', limit=None):
        folder = str(dir_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return []

        preferred = str(preferred_view or 'optimal').strip().lower()
        if preferred == 'all':
                return _merged_frame_paths_for_all_view(folder, limit=limit)

        candidate_dirs = _timelapse_frame_dir_candidates(folder, preferred_view=preferred)

        hard_limit = None
        if limit is not None:
                try:
                        parsed_limit = int(limit)
                except Exception:
                        parsed_limit = 0
                if parsed_limit > 0:
                        hard_limit = max(1, min(500000, parsed_limit))
        for target in candidate_dirs:
                if not target.exists() or not target.is_dir():
                        continue
                frame_names = _list_local_frame_names_in_dir(target)
                if not frame_names:
                        continue
                if hard_limit is not None and len(frame_names) > hard_limit:
                        frame_names = frame_names[-hard_limit:]
                return [target / frame_name for frame_name in frame_names]
        return []


def _merged_frame_paths_for_all_view(dir_name, limit=None):
        """Build all-view frames from optimal first, then fill gaps from all folders."""
        folder = str(dir_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return []

        primary_dirs = []
        backup_dirs = []
        seen = set()
        for target in _timelapse_frame_dir_candidates(folder, preferred_view='optimal'):
                key = str(target)
                if key in seen:
                        continue
                seen.add(key)
                if str(target.name).lower() == 'all':
                        backup_dirs.append(target)
                else:
                        primary_dirs.append(target)

        selected = {}
        for target in primary_dirs:
                if not target.exists() or not target.is_dir():
                        continue
                for frame_name in _list_local_frame_names_in_dir(target):
                        if frame_name not in selected:
                                selected[frame_name] = target / frame_name

        for target in backup_dirs:
                if not target.exists() or not target.is_dir():
                        continue
                for frame_name in _list_local_frame_names_in_dir(target):
                        if frame_name not in selected:
                                selected[frame_name] = target / frame_name

        if not selected:
                return []

        frame_names = sorted(selected.keys())

        hard_limit = None
        if limit is not None:
                try:
                        parsed_limit = int(limit)
                except Exception:
                        parsed_limit = 0
                if parsed_limit > 0:
                        hard_limit = max(1, min(500000, parsed_limit))
        if hard_limit is not None and len(frame_names) > hard_limit:
                frame_names = frame_names[-hard_limit:]

        return [selected[name] for name in frame_names]


def _ffconcat_escape_path(path_obj):
        text = str(path_obj)
        text = text.replace('\\', '\\\\')
        text = text.replace("'", "'\\''")
        return text


def export_timelapse_video(
        dir_name,
        view='optimal',
        fps=12,
        resolution='source',
        quality='medium',
        fmt='mp4',
        max_frames=None,
        hide_banner=True,
        loglevel='error',
        concat_safe=0,
        codec='libx264',
        preset=None,
        crf=None,
        pix_fmt='yuv420p',
        faststart=True,
        name_tag=None,
        start_pct=None,
        end_pct=None,
):
        folder = str(dir_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return {'ok': False, 'error': 'Invalid dir'}

        view_mode = str(view or 'optimal').strip().lower()
        if view_mode not in ('optimal', 'all'):
                view_mode = 'optimal'

        try:
                fps_val = int(fps)
        except Exception:
                fps_val = 12
        fps_val = max(1, min(60, fps_val))

        resolution_mode = str(resolution or 'source').strip().lower()
        resolution_height_map = {
                'source': None,
                '2160p': 2160,
                '1440p': 1440,
                '1080p': 1080,
                '720p': 720,
                '540p': 540,
        }
        if resolution_mode not in resolution_height_map:
                resolution_mode = 'source'

        quality_mode = str(quality or 'medium').strip().lower()
        quality_map = {
                'low': {'crf': 30, 'preset': 'veryfast'},
                'medium': {'crf': 24, 'preset': 'medium'},
                'high': {'crf': 18, 'preset': 'slow'},
        }
        if quality_mode not in quality_map:
                quality_mode = 'medium'
        quality_cfg = quality_map[quality_mode]

        try:
                hide_banner_val = bool(hide_banner)
        except Exception:
                hide_banner_val = True

        loglevel_mode = str(loglevel or 'error').strip().lower()
        allowed_loglevels = {'quiet', 'panic', 'fatal', 'error', 'warning', 'info', 'verbose', 'debug', 'trace'}
        if loglevel_mode not in allowed_loglevels:
                loglevel_mode = 'error'

        try:
                concat_safe_val = 1 if int(concat_safe) else 0
        except Exception:
                concat_safe_val = 0

        codec_mode = str(codec or 'libx264').strip().lower()
        allowed_codecs = {'libx264', 'libx265'}
        if codec_mode not in allowed_codecs:
                codec_mode = 'libx264'

        allowed_presets = {'ultrafast', 'superfast', 'veryfast', 'faster', 'fast', 'medium', 'slow', 'slower', 'veryslow'}
        preset_mode = str(preset or quality_cfg.get('preset') or 'medium').strip().lower()
        if preset_mode not in allowed_presets:
                preset_mode = str(quality_cfg.get('preset') or 'medium')

        try:
                crf_val = int(crf if crf is not None else quality_cfg.get('crf'))
        except Exception:
                crf_val = int(quality_cfg.get('crf') or 24)
        crf_val = max(0, min(51, crf_val))

        pix_fmt_mode = str(pix_fmt or 'yuv420p').strip().lower()
        allowed_pix_fmts = {'yuv420p', 'yuv422p', 'yuv444p'}
        if pix_fmt_mode not in allowed_pix_fmts:
                pix_fmt_mode = 'yuv420p'

        try:
                faststart_val = bool(faststart)
        except Exception:
                faststart_val = True

        format_mode = str(fmt or 'mp4').strip().lower()
        format_ext_map = {
                'mp4': '.mp4',
                'mov': '.mov',
        }
        if format_mode not in format_ext_map:
                format_mode = 'mp4'
        format_ext = format_ext_map[format_mode]

        max_frames_val = None
        if max_frames is None:
                max_frames_val = None
        else:
                raw_max_frames = str(max_frames).strip().lower()
                if raw_max_frames in ('', 'all', 'none', '0', '-1'):
                        max_frames_val = None
                else:
                        try:
                                parsed = int(raw_max_frames)
                        except Exception:
                                parsed = 0
                        if parsed > 0:
                                max_frames_val = max(50, min(500000, parsed))
                        else:
                                max_frames_val = None

        frame_paths = _full_frame_paths_for_timelapse_dir(folder, preferred_view=view_mode, limit=max_frames_val)
        if len(frame_paths) < 2:
                return {'ok': False, 'error': 'Not enough frames to export video'}

        # Trim frame list to loop points expressed as 0-1 percentages
        try:
                sp = float(start_pct) if start_pct is not None else None
                ep = float(end_pct) if end_pct is not None else None
                total = len(frame_paths)
                start_idx = max(0, int(round(sp * (total - 1)))) if sp is not None and 0.0 <= sp <= 1.0 else 0
                end_idx = min(total - 1, int(round(ep * (total - 1)))) if ep is not None and 0.0 <= ep <= 1.0 else total - 1
                if start_idx > 0 or end_idx < total - 1:
                        frame_paths = frame_paths[start_idx:end_idx + 1]
                        if len(frame_paths) < 2:
                                return {'ok': False, 'error': 'Loop range too narrow — fewer than 2 frames'}
        except Exception:
                pass

        ffmpeg_path = shutil.which('ffmpeg')
        if not ffmpeg_path:
                return {'ok': False, 'error': 'ffmpeg is not installed or not found in PATH'}

        try:
                TIMELAPSE_EXPORT_DIR.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
                return {'ok': False, 'error': f'Failed to create export directory: {exc}'}

        stamp = int(time.time())
        raw_tag = str(name_tag or '').strip().lower()
        safe_tag = ''.join(ch for ch in raw_tag if ch.isalnum() or ch in ('-', '_'))
        tag_part = f"_{safe_tag}" if safe_tag else ''
        max_part = f"{max_frames_val}max" if max_frames_val is not None else 'allmax'
        output_name = f"{folder}_{view_mode}{tag_part}_{fps_val}fps_{resolution_mode}_{quality_mode}_{max_part}_{stamp}{format_ext}"
        output_path = TIMELAPSE_EXPORT_DIR / output_name
        list_path = TIMELAPSE_EXPORT_DIR / f".{folder}_{stamp}.ffconcat.txt"

        try:
                lines = []
                for frame_path in frame_paths:
                        lines.append(f"file '{_ffconcat_escape_path(frame_path)}'\n")
                list_path.write_text(''.join(lines), encoding='utf-8')

                scale_filter = 'scale=trunc(iw/2)*2:trunc(ih/2)*2'
                target_height = resolution_height_map.get(resolution_mode)
                if target_height:
                        scale_filter = f'scale=-2:{int(target_height)}'

                cmd = [
                        ffmpeg_path,
                        '-y',
                ]
                if hide_banner_val:
                        cmd.append('-hide_banner')
                cmd.extend([
                        '-loglevel', loglevel_mode,
                        '-f', 'concat',
                        '-safe', str(concat_safe_val),
                        '-r', str(fps_val),
                        '-i', str(list_path),
                        '-vf', scale_filter + ',format=' + pix_fmt_mode,
                        '-c:v', codec_mode,
                        '-preset', str(preset_mode),
                        '-crf', str(crf_val),
                ])
                if faststart_val:
                        cmd.extend(['-movflags', '+faststart'])
                cmd.append(str(output_path))
                probe = subprocess.run(cmd, capture_output=True, text=True, check=False)
                if probe.returncode != 0 or not output_path.exists():
                        err = (probe.stderr or probe.stdout or 'ffmpeg export failed').strip()
                        return {'ok': False, 'error': err}

                return {
                        'ok': True,
                        'dir': folder,
                        'view': view_mode,
                        'fps': fps_val,
                        'resolution': resolution_mode,
                        'quality': quality_mode,
                        'format': format_mode,
                        'max_frames': max_frames_val if max_frames_val is not None else 'all',
                        'hide_banner': hide_banner_val,
                        'loglevel': loglevel_mode,
                        'concat_safe': concat_safe_val,
                        'codec': codec_mode,
                        'preset': preset_mode,
                        'crf': crf_val,
                        'pix_fmt': pix_fmt_mode,
                        'faststart': faststart_val,
                        'frame_count': len(frame_paths),
                        'file_name': output_name,
                        'download_url': f"/api/timelapse-export-download?name={quote(output_name)}",
                }
        except Exception as exc:
                return {'ok': False, 'error': f'Video export failed: {exc}'}
        finally:
                try:
                        if list_path.exists():
                                list_path.unlink()
                except Exception:
                        pass


def get_latest_timelapse_export(dir_name, prefer_tag=None, fallback_to_any=True):
        folder = str(dir_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return {'ok': False, 'error': 'Invalid dir'}
        try:
                export_dir = TIMELAPSE_EXPORT_DIR.resolve()
                if not export_dir.exists() or not export_dir.is_dir():
                        return {'ok': True, 'available': False, 'dir': folder}
                # Require exact folder token by matching the expected export layout:
                # {folder}_{view}[_tag]_{fps}fps_{resolution}_{quality}_{max}_{stamp}.{ext}
                valid_prefixes = (
                        f"{folder}_optimal_",
                        f"{folder}_all_",
                )
                prefer_token = None
                if prefer_tag:
                        token = ''.join(ch for ch in str(prefer_tag).strip().lower() if ch.isalnum() or ch in ('-', '_'))
                        if token:
                                prefer_token = f"_{token}_"
                matches = []
                preferred_matches = []
                for child in export_dir.iterdir():
                        if not child.is_file():
                                continue
                        if child.suffix.lower() not in ('.mp4', '.mov'):
                                continue
                        if not child.name.startswith(valid_prefixes):
                                continue
                        try:
                                mtime = float(child.stat().st_mtime)
                        except Exception:
                                mtime = 0.0
                        item = (mtime, child.name, child)
                        matches.append(item)
                        if prefer_token and prefer_token in child.name:
                                preferred_matches.append(item)
                if not matches:
                        return {'ok': True, 'available': False, 'dir': folder}

                pool = preferred_matches if preferred_matches else (matches if fallback_to_any else [])
                if not pool:
                        return {'ok': True, 'available': False, 'dir': folder}

                pool.sort(key=lambda item: (item[0], item[1]), reverse=True)
                _, _, latest = pool[0]
                return {
                        'ok': True,
                        'available': True,
                        'dir': folder,
                        'file_name': latest.name,
                        'media_url': f"/api/timelapse-export-media?name={quote(latest.name)}",
                        'download_url': f"/api/timelapse-export-download?name={quote(latest.name)}",
                }
        except Exception as exc:
                return {'ok': False, 'error': f'Failed to read exports: {exc}'}


def _frame_name_is_valid(name):
        if not name:
                return False
        text = str(name).strip()
        if '/' in text or '\\' in text:
                return False
        lower = text.lower()
        return lower.startswith('timelapse_') or lower.startswith('capt_')


def _frame_sequence_value(name):
        text = str(name or '').strip()
        if not _frame_name_is_valid(text):
                return None
        stem = Path(text).stem
        match = re.search(r'(\d+)$', stem)
        if not match:
                return None
        try:
                return int(match.group(1))
        except Exception:
                return None


def _frame_name_is_fallback(name):
        text = str(name or '').strip()
        if not text:
                return False
        stem = Path(text).stem.lower()
        return stem.endswith('_fallback') or '_fallback_' in stem


def _timelapse_local_dir_candidates(dir_name):
        name = str(dir_name or '').strip()
        if not _timelapse_name_is_valid(name):
                return []
        # Reuse the same resolution order used by preview/export to avoid
        # mixing different photo sets for the same timelapse name.
        return _timelapse_frame_dir_candidates(name, preferred_view='optimal')


def _timelapse_frame_dir_candidates(dir_name, preferred_view='optimal'):
        name = str(dir_name or '').strip()
        if not _timelapse_name_is_valid(name):
                return []
        preferred = str(preferred_view or 'optimal').strip().lower()
        if preferred not in ('optimal', 'all'):
                preferred = 'optimal'

        local_root = SCRIPT_DIR / name
        local_opt = local_root / 'optimal'
        local_all = local_root / 'all'
        mirror_root = SCRIPT_DIR / 'timelapse_orin' / name
        mirror_opt = mirror_root / 'optimal'
        mirror_all = mirror_root / 'all'

        # Canonical legacy folder `timelapse` should prefer local scripts/timelapse.
        if name == 'timelapse':
                return [
                        local_opt,
                        local_root,
                        mirror_opt,
                        mirror_root,
                        local_all,
                        mirror_all,
                ]

        return [
                mirror_opt,
                mirror_root,
                local_opt,
                local_root,
                mirror_all,
                local_all,
        ]


def _list_local_frame_names_in_dir(path_obj):
        if not path_obj.exists() or not path_obj.is_dir():
                return []
        names = []
        for child in path_obj.iterdir():
                if not child.is_file():
                        continue
                if _frame_name_is_valid(child.name):
                        names.append(child.name)
        names.sort()
        return names


def _list_remote_frame_names(dir_name, preferred_subdir=None):
        folder = str(dir_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return [], None

        subdirs = []
        preferred = str(preferred_subdir or '').strip().lower()
        if preferred in ('optimal', 'all'):
                subdirs.append(preferred)
        for candidate in ('optimal', 'all'):
                if candidate not in subdirs:
                        subdirs.append(candidate)

        for subdir in subdirs:
                remote_dir = f"{TIMELAPSE_ROOT}/{folder}/{subdir}"
                remote_q = shlex.quote(remote_dir)
                cmd = (
                        f"if [ -d {remote_q} ]; then "
                        f"find {remote_q} -maxdepth 1 -type f "
                        "\\( -name 'timelapse_*' -o -name 'capt_*' \\) "
                        "-printf '%f\\n' | LC_ALL=C sort; "
                        "fi"
                )
                probe = subprocess.run(
                        [
                                'ssh',
                                '-o', 'BatchMode=yes',
                                '-o', 'ConnectTimeout=10',
                                '-o', 'IdentitiesOnly=yes',
                                '-i', ORIN_SSH_KEY,
                                f'{ORIN_SSH_USER}@{ORIN_SSH_HOST}',
                                cmd,
                        ],
                        capture_output=True,
                        text=True,
                        check=False,
                )
                if probe.returncode != 0:
                        continue
                names = [line.strip() for line in (probe.stdout or '').splitlines() if _frame_name_is_valid(line.strip())]
                if names:
                        return names, subdir
        return [], None


def _fetch_remote_frame_bytes(dir_name, frame_name, preferred_subdir=None):
        folder = str(dir_name or '').strip()
        frame = str(frame_name or '').strip()
        if not _timelapse_name_is_valid(folder) or not _frame_name_is_valid(frame):
                return None

        subdirs = []
        if preferred_subdir in ('optimal', 'all'):
                subdirs.append(preferred_subdir)
        for candidate in ('optimal', 'all'):
                if candidate not in subdirs:
                        subdirs.append(candidate)

        for subdir in subdirs:
                remote_path = f"{TIMELAPSE_ROOT}/{folder}/{subdir}/{frame}"
                remote_q = shlex.quote(remote_path)
                cmd = f"test -f {remote_q} && cat {remote_q}"
                probe = subprocess.run(
                        [
                                'ssh',
                                '-o', 'BatchMode=yes',
                                '-o', 'ConnectTimeout=12',
                                '-o', 'IdentitiesOnly=yes',
                                '-i', ORIN_SSH_KEY,
                                f'{ORIN_SSH_USER}@{ORIN_SSH_HOST}',
                                cmd,
                        ],
                        capture_output=True,
                        check=False,
                )
                if probe.returncode == 0 and probe.stdout:
                        return probe.stdout
        return None


def _as_float(value):
        if value is None:
                return None
        try:
                if isinstance(value, tuple) and len(value) == 2:
                        den = float(value[1])
                        if den == 0:
                                return None
                        return float(value[0]) / den
                if hasattr(value, 'numerator') and hasattr(value, 'denominator'):
                        den = float(getattr(value, 'denominator'))
                        if den == 0:
                                return None
                        return float(getattr(value, 'numerator')) / den
                return float(value)
        except Exception:
                return None


def _extract_capture_from_image(img_obj):
        iso = None
        shutter_s = None
        fstop = None
        try:
                exif = img_obj.getexif()
        except Exception:
                exif = None
        if exif:
                iso_val = _as_float(exif.get(34855, exif.get(41989)))
                if iso_val is not None:
                        iso = int(round(iso_val))
                shutter_s = _as_float(exif.get(33434))
                fstop = _as_float(exif.get(33437))

        shutter_text = None
        if shutter_s is not None:
                if shutter_s >= 1.0:
                        shutter_text = f"{shutter_s:.3g}s"
                else:
                        denom = round(1.0 / max(shutter_s, 1e-9))
                        shutter_text = f"1/{denom}"
        return {
                'iso': iso,
                'shutter': shutter_text,
                'shutter_s': shutter_s,
                'fstop': fstop,
        }


def _parse_shutter_text_to_seconds(shutter_text):
        text = str(shutter_text or '').strip().lower()
        if not text:
                return None
        try:
                if text.endswith('s'):
                        text = text[:-1].strip()
                if '/' in text:
                        num_s, den_s = text.split('/', 1)
                        num = float(num_s.strip())
                        den = float(den_s.strip())
                        if den == 0:
                                return None
                        return num / den
                return float(text)
        except Exception:
                return None


def _load_capture_fallback_from_analysis(frame_path, frame_name):
        try:
                analysis_path = frame_path.parent.parent / '.sync_state' / 'analysis' / f"{Path(frame_name).stem}.json"
        except Exception:
                return None
        if not analysis_path.exists() or not analysis_path.is_file():
                return None
        try:
                payload = json.loads(analysis_path.read_text(encoding='utf-8'))
        except Exception:
                return None

        current = payload.get('current') or {}
        iso = current.get('iso')
        shutter_text = current.get('shutter')
        fstop = current.get('fstop')
        shutter_s = _parse_shutter_text_to_seconds(shutter_text)

        if iso is None and shutter_text is None and fstop is None:
                return None
        return {
                'iso': int(round(iso)) if iso is not None else None,
                'shutter': str(shutter_text) if shutter_text is not None else None,
                'shutter_s': shutter_s,
                'fstop': float(fstop) if fstop is not None else None,
        }


def _load_capture_fallback_from_file_metadata(frame_path):
        path_text = str(frame_path)

        iso = None
        shutter_s = None
        fstop = None

        # Prefer exiftool when available; it reads Sony fields more reliably.
        try:
                probe = subprocess.run(
                        ['exiftool', '-s3', '-ISO', '-ExposureTime', '-FNumber', path_text],
                        capture_output=True,
                        text=True,
                        timeout=5,
                        check=False,
                )
                if probe.returncode == 0:
                        lines = [line.strip() for line in (probe.stdout or '').splitlines() if line.strip()]
                        if len(lines) >= 3:
                                iso_match = re.search(r'\d+', lines[0])
                                if iso_match:
                                        try:
                                                iso = int(iso_match.group(0))
                                        except Exception:
                                                iso = None
                                shutter_s = _parse_shutter_text_to_seconds(lines[1])
                                fstop_match = re.search(r'\d+(?:\.\d+)?', lines[2])
                                if fstop_match:
                                        try:
                                                fstop = float(fstop_match.group(0))
                                        except Exception:
                                                fstop = None
        except Exception:
                pass

        # macOS Spotlight fallback.
        if iso is None or shutter_s is None or fstop is None:
                try:
                        mdls = subprocess.run(
                                [
                                        'mdls',
                                        '-name', 'kMDItemISOSpeed',
                                        '-name', 'kMDItemExposureTimeSeconds',
                                        '-name', 'kMDItemAperture',
                                        path_text,
                                ],
                                capture_output=True,
                                text=True,
                                timeout=5,
                                check=False,
                        )
                        if mdls.returncode == 0:
                                for raw_line in (mdls.stdout or '').splitlines():
                                        line = raw_line.strip()
                                        if not line or '=' not in line or '(null)' in line:
                                                continue
                                        key, value = [part.strip() for part in line.split('=', 1)]
                                        num_match = re.search(r'-?\d+(?:\.\d+)?', value)
                                        if not num_match:
                                                continue
                                        num = float(num_match.group(0))
                                        if key == 'kMDItemISOSpeed' and iso is None:
                                                iso = int(round(num))
                                        elif key == 'kMDItemExposureTimeSeconds' and shutter_s is None:
                                                shutter_s = num
                                        elif key == 'kMDItemAperture' and fstop is None:
                                                fstop = num
                except Exception:
                        pass

        if iso is None and shutter_s is None and fstop is None:
                return None

        shutter_text = None
        if shutter_s is not None:
                if shutter_s >= 1.0:
                        shutter_text = f"{shutter_s:.3g}s"
                else:
                        denom = round(1.0 / max(shutter_s, 1e-9))
                        shutter_text = f"1/{denom}"

        return {
                'iso': int(round(iso)) if iso is not None else None,
                'shutter': shutter_text,
                'shutter_s': shutter_s,
                'fstop': float(fstop) if fstop is not None else None,
        }


def _load_remote_capture_fallback_from_analysis(dir_name, frame_name):
        folder = str(dir_name or '').strip()
        frame = str(frame_name or '').strip()
        if not _timelapse_name_is_valid(folder) or not _frame_name_is_valid(frame):
                return None

        analysis_path = f"{TIMELAPSE_ROOT}/{folder}/.sync_state/analysis/{Path(frame).stem}.json"
        analysis_q = shlex.quote(analysis_path)
        cmd = f"test -f {analysis_q} && cat {analysis_q}"
        probe = subprocess.run(
                [
                        'ssh',
                        '-o', 'BatchMode=yes',
                        '-o', 'ConnectTimeout=10',
                        '-o', 'IdentitiesOnly=yes',
                        '-i', ORIN_SSH_KEY,
                        f'{ORIN_SSH_USER}@{ORIN_SSH_HOST}',
                        cmd,
                ],
                capture_output=True,
                text=True,
                check=False,
        )
        if probe.returncode != 0 or not (probe.stdout or '').strip():
                return None

        try:
                payload = json.loads(probe.stdout)
        except Exception:
                return None

        current = payload.get('current') or {}
        iso = current.get('iso')
        shutter_text = current.get('shutter')
        fstop = current.get('fstop')
        shutter_s = _parse_shutter_text_to_seconds(shutter_text)

        if iso is None and shutter_text is None and fstop is None:
                return None
        return {
                'iso': int(round(iso)) if iso is not None else None,
                'shutter': str(shutter_text) if shutter_text is not None else None,
                'shutter_s': shutter_s,
                'fstop': float(fstop) if fstop is not None else None,
        }


def _frame_analysis_cache_path_for_local(frame_path, frame_name):
        try:
                folder_root = frame_path.parent.parent
                cache_dir = folder_root / '.sync_state' / 'frame_analysis_cache'
                cache_dir.mkdir(parents=True, exist_ok=True)
                return cache_dir / f"{str(frame_name)}.json"
        except Exception:
                return None


def _load_persisted_frame_analysis(frame_path, frame_name):
        try:
                stat = frame_path.stat()
        except Exception:
                return None
        cache_path = _frame_analysis_cache_path_for_local(frame_path, frame_name)
        if not cache_path or not cache_path.exists() or not cache_path.is_file():
                return None
        try:
                payload = json.loads(cache_path.read_text(encoding='utf-8'))
        except Exception:
                return None
        if not isinstance(payload, dict):
                return None
        if str(payload.get('image_name') or '') != str(frame_name):
                return None
        try:
                cached_mtime_ns = int(payload.get('image_mtime_ns') or 0)
                cached_size = int(payload.get('image_size') or -1)
        except Exception:
                return None
        if cached_mtime_ns != int(getattr(stat, 'st_mtime_ns', int(stat.st_mtime * 1e9))):
                return None
        if cached_size != int(stat.st_size):
                return None
        capture = payload.get('capture') if isinstance(payload.get('capture'), dict) else None
        metrics = payload.get('metrics') if isinstance(payload.get('metrics'), dict) else None
        histogram = payload.get('histogram') if isinstance(payload.get('histogram'), dict) else None
        if not capture or not metrics or not histogram:
                return None
        return {
                'capture': capture,
                'metrics': metrics,
                'histogram': histogram,
                'source': 'local',
        }


def _persist_frame_analysis(frame_path, frame_name, capture, metrics, histogram):
        cache_path = _frame_analysis_cache_path_for_local(frame_path, frame_name)
        if not cache_path:
                return
        try:
                stat = frame_path.stat()
                payload = {
                        'image_name': str(frame_name),
                        'image_mtime_ns': int(getattr(stat, 'st_mtime_ns', int(stat.st_mtime * 1e9))),
                        'image_size': int(stat.st_size),
                        'capture': capture,
                        'metrics': metrics,
                        'histogram': histogram,
                        'updated_at': time.time(),
                }
                cache_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        except Exception:
                return


def _metrics_from_image(img_obj, target_luma=135.0, bins=64):
        arr = np.asarray(img_obj.convert('RGB'), dtype=np.float32)
        luma = 0.2126 * arr[:, :, 0] + 0.7152 * arr[:, :, 1] + 0.0722 * arr[:, :, 2]

        shadow_clip = float((luma <= 5.0).mean() * 100.0)
        highlight_clip = float((luma >= 250.0).mean() * 100.0)
        median_luma = float(np.median(luma))
        mean_luma = float(np.mean(luma))
        delta_ev = math.log2(max(float(target_luma), 1e-6) / max(median_luma, 1e-6))

        hist, _ = np.histogram(luma, bins=bins, range=(0, 256))
        return {
                'metrics': {
                        'delta_ev': float(delta_ev),
                        'median_luma': float(median_luma),
                        'mean_luma': float(mean_luma),
                        'highlight_clip_pct': float(highlight_clip),
                        'shadow_clip_pct': float(shadow_clip),
                },
                'histogram': {
                        'bins': hist.astype(int).tolist(),
                        'min': 0,
                        'max': 255,
                },
        }


def _load_frame_analysis(dir_name, frame_name, preferred_subdir=None):
        folder = str(dir_name or '').strip()
        frame = str(frame_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return None
        if not _frame_name_is_valid(frame):
                return None

        for path_obj in _timelapse_local_dir_candidates(folder):
                candidate = path_obj / frame
                if not candidate.exists() or not candidate.is_file():
                        continue
                try:
                        stat = candidate.stat()
                        cache_key = f"local:{candidate}:{int(stat.st_mtime)}:{stat.st_size}"
                except Exception:
                        cache_key = None

                if cache_key:
                        with TIMELAPSE_METADATA_CACHE_LOCK:
                                cached = TIMELAPSE_METADATA_CACHE.get(cache_key)
                        if cached:
                                return dict(cached)

                persisted = _load_persisted_frame_analysis(candidate, frame)
                if persisted:
                        persisted_capture = persisted.get('capture') if isinstance(persisted.get('capture'), dict) else {}
                        needs_capture_backfill = (
                                persisted_capture.get('iso') is None
                                or persisted_capture.get('shutter') is None
                                or persisted_capture.get('fstop') is None
                        )
                        if needs_capture_backfill:
                                fallback_capture = _load_capture_fallback_from_analysis(candidate, frame)
                                if not fallback_capture:
                                        fallback_capture = _load_remote_capture_fallback_from_analysis(folder, frame)
                                if not fallback_capture:
                                        fallback_capture = _load_capture_fallback_from_file_metadata(candidate)
                                if fallback_capture:
                                        if persisted_capture.get('iso') is None:
                                                persisted_capture['iso'] = fallback_capture.get('iso')
                                        if persisted_capture.get('shutter') is None:
                                                persisted_capture['shutter'] = fallback_capture.get('shutter')
                                        if persisted_capture.get('shutter_s') is None:
                                                persisted_capture['shutter_s'] = fallback_capture.get('shutter_s')
                                        if persisted_capture.get('fstop') is None:
                                                persisted_capture['fstop'] = fallback_capture.get('fstop')
                                        persisted['capture'] = persisted_capture
                                        _persist_frame_analysis(
                                                candidate,
                                                frame,
                                                persisted_capture,
                                                persisted.get('metrics') or {},
                                                persisted.get('histogram') or {},
                                        )
                        if cache_key:
                                with TIMELAPSE_METADATA_CACHE_LOCK:
                                        TIMELAPSE_METADATA_CACHE[cache_key] = dict(persisted)
                        return dict(persisted)

                try:
                        with Image.open(candidate) as img:
                                capture = _extract_capture_from_image(img)
                                metric_pack = _metrics_from_image(img)
                except Exception:
                        continue

                if capture.get('iso') is None or capture.get('shutter') is None or capture.get('fstop') is None:
                        fallback_capture = _load_capture_fallback_from_analysis(candidate, frame)
                        if not fallback_capture:
                                # Local mirror frames may not have a local analysis sidecar yet.
                                # Reuse remote analysis if available to populate capture values.
                                fallback_capture = _load_remote_capture_fallback_from_analysis(folder, frame)
                        if not fallback_capture:
                                fallback_capture = _load_capture_fallback_from_file_metadata(candidate)
                        if fallback_capture:
                                if capture.get('iso') is None:
                                        capture['iso'] = fallback_capture.get('iso')
                                if capture.get('shutter') is None:
                                        capture['shutter'] = fallback_capture.get('shutter')
                                if capture.get('shutter_s') is None:
                                        capture['shutter_s'] = fallback_capture.get('shutter_s')
                                if capture.get('fstop') is None:
                                        capture['fstop'] = fallback_capture.get('fstop')

                result = {
                        'capture': capture,
                        'metrics': metric_pack['metrics'],
                        'histogram': metric_pack['histogram'],
                        'source': 'local',
                }
                _persist_frame_analysis(candidate, frame, result['capture'], result['metrics'], result['histogram'])
                if cache_key:
                        with TIMELAPSE_METADATA_CACHE_LOCK:
                                TIMELAPSE_METADATA_CACHE[cache_key] = dict(result)
                return result

        names, remote_subdir = _list_remote_frame_names(folder, preferred_subdir=preferred_subdir)
        if frame not in names:
                return None
        raw = _fetch_remote_frame_bytes(folder, frame, preferred_subdir=remote_subdir)
        if not raw:
                return None
        try:
                with Image.open(io.BytesIO(raw)) as img:
                        capture = _extract_capture_from_image(img)
                        metric_pack = _metrics_from_image(img)
        except Exception:
                return None

        if capture.get('iso') is None or capture.get('shutter') is None or capture.get('fstop') is None:
                fallback_capture = _load_remote_capture_fallback_from_analysis(folder, frame)
                if fallback_capture:
                        if capture.get('iso') is None:
                                capture['iso'] = fallback_capture.get('iso')
                        if capture.get('shutter') is None:
                                capture['shutter'] = fallback_capture.get('shutter')
                        if capture.get('shutter_s') is None:
                                capture['shutter_s'] = fallback_capture.get('shutter_s')
                        if capture.get('fstop') is None:
                                capture['fstop'] = fallback_capture.get('fstop')

        return {
                'capture': capture,
                'metrics': metric_pack['metrics'],
                'histogram': metric_pack['histogram'],
                'source': 'remote',
        }


def _compute_comparison_delta(current, previous):
        cur_m = current.get('metrics') or {}
        prv_m = previous.get('metrics') or {}
        cur_c = current.get('capture') or {}
        prv_c = previous.get('capture') or {}

        cur_shutter = cur_c.get('shutter_s')
        prv_shutter = prv_c.get('shutter_s')
        shutter_ev = None
        if cur_shutter and prv_shutter and cur_shutter > 0 and prv_shutter > 0:
                shutter_ev = math.log2(float(cur_shutter) / float(prv_shutter))

        cur_fstop = cur_c.get('fstop')
        prv_fstop = prv_c.get('fstop')
        fstop_ev = None
        if cur_fstop and prv_fstop and cur_fstop > 0 and prv_fstop > 0:
                fstop_ev = 2.0 * math.log2(float(prv_fstop) / float(cur_fstop))

        return {
                'delta_ev': (cur_m.get('delta_ev') or 0.0) - (prv_m.get('delta_ev') or 0.0),
                'median_luma': (cur_m.get('median_luma') or 0.0) - (prv_m.get('median_luma') or 0.0),
                'highlight_clip_pct': (cur_m.get('highlight_clip_pct') or 0.0) - (prv_m.get('highlight_clip_pct') or 0.0),
                'shadow_clip_pct': (cur_m.get('shadow_clip_pct') or 0.0) - (prv_m.get('shadow_clip_pct') or 0.0),
                'iso': ((cur_c.get('iso') or 0) - (prv_c.get('iso') or 0)) if cur_c.get('iso') is not None and prv_c.get('iso') is not None else None,
                'shutter_ev': shutter_ev,
                'fstop_ev': fstop_ev,
        }


def build_timelapse_frame_metadata_payload(dir_name, frame_name, include_histogram=False):
        folder = str(dir_name or '').strip()
        frame = str(frame_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return {'available': False, 'reason': 'Invalid dir'}
        if not _frame_name_is_valid(frame):
                return {'available': False, 'reason': 'Invalid frame name'}

        current = _load_frame_analysis(folder, frame)
        if not current:
                return {'available': False, 'reason': 'Frame not available locally or remotely'}

        names = []
        for path_obj in _timelapse_local_dir_candidates(folder):
                names = _list_local_frame_names_in_dir(path_obj)
                if names:
                        break
        if not names:
                names, _ = _list_remote_frame_names(folder)

        comparisons = {}
        offsets = [5, 10, 20, 40]
        try:
                idx = names.index(frame)
        except Exception:
                idx = -1

        for off in offsets:
                key = str(off)
                if idx < 0:
                        comparisons[key] = {'available': False, 'reason': 'Frame not found in sequence'}
                        continue
                prev_idx = idx - off
                if prev_idx < 0:
                        comparisons[key] = {'available': False, 'reason': 'Not enough prior frames'}
                        continue
                prev_name = names[prev_idx]
                previous = _load_frame_analysis(folder, prev_name)
                if not previous:
                        comparisons[key] = {'available': False, 'reason': 'Prior frame unavailable'}
                        continue
                comparisons[key] = {
                        'available': True,
                        'frame': {'name': prev_name},
                        'delta': _compute_comparison_delta(current, previous),
                }

        capture = current.get('capture') or {}
        capture_out = {
                'iso': capture.get('iso'),
                'shutter': capture.get('shutter'),
                'fstop': capture.get('fstop'),
        }
        capture_included = any(capture_out.get(k) is not None and capture_out.get(k) != '' for k in ('iso', 'shutter', 'fstop'))
        current_obj = {
                'name': frame,
                'capture': capture_out,
                'capture_included': bool(capture_included),
                'metrics': current.get('metrics') or {},
                'source': current.get('source') or 'local',
        }
        if include_histogram:
                current_obj['histogram'] = current.get('histogram')

        return {
                'available': True,
                'dir': folder,
                'current': current_obj,
                'comparisons': comparisons,
        }


def build_latest_frame_meta_summary(dir_name, local_dir, preferred_subdir='optimal'):
        folder = str(dir_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return None

        names = []
        try:
                names = _list_local_frame_names_in_dir(Path(local_dir))
        except Exception:
                names = []

        source = 'local'
        current_name = None
        previous_name = None

        if names:
                current_name = names[-1]
                previous_name = names[-2] if len(names) > 1 else None
        else:
                remote_names, _resolved_subdir = _list_remote_frame_names(folder, preferred_subdir=preferred_subdir)
                if not remote_names:
                        return None
                source = 'remote'
                current_name = remote_names[-1]
                previous_name = remote_names[-2] if len(remote_names) > 1 else None

        current = _load_frame_analysis(folder, current_name, preferred_subdir=preferred_subdir)
        if not current:
                return None

        previous = _load_frame_analysis(folder, previous_name, preferred_subdir=preferred_subdir) if previous_name else None
        delta = _compute_comparison_delta(current, previous) if previous else {}

        capture = current.get('capture') or {}
        metrics = current.get('metrics') or {}

        return {
                'available': True,
                'source': source,
                'current_name': current_name,
                'previous_name': previous_name,
                'current': {
                        'capture': {
                                'iso': capture.get('iso'),
                                'shutter': capture.get('shutter'),
                                'fstop': capture.get('fstop'),
                        },
                        'metrics': {
                                'delta_ev': metrics.get('delta_ev'),
                                'median_luma': metrics.get('median_luma'),
                                'highlight_clip_pct': metrics.get('highlight_clip_pct'),
                                'shadow_clip_pct': metrics.get('shadow_clip_pct'),
                        },
                },
                'delta': {
                        'iso': delta.get('iso'),
                        'shutter_ev': delta.get('shutter_ev'),
                        'fstop_ev': delta.get('fstop_ev'),
                        'delta_ev': delta.get('delta_ev'),
                        'median_luma': delta.get('median_luma'),
                        'highlight_clip_pct': delta.get('highlight_clip_pct'),
                        'shadow_clip_pct': delta.get('shadow_clip_pct'),
                },
        }


def _clamp(value, low, high):
        try:
                v = float(value)
        except Exception:
                return low
        return max(low, min(high, v))


def _ensure_local_preview_frame(dir_name, frame_name, preferred_subdir='optimal'):
        folder = str(dir_name or '').strip()
        frame = str(frame_name or '').strip()
        subdir = 'all' if str(preferred_subdir or '').strip().lower() == 'all' else 'optimal'
        if not _timelapse_name_is_valid(folder) or not _frame_name_is_valid(frame):
                return None

        target_dir = SCRIPT_DIR / 'timelapse_orin' / folder / subdir
        target_path = target_dir / frame
        try:
                if target_path.exists() and target_path.is_file() and target_path.stat().st_size > 0:
                        return str(target_path)
        except Exception:
                pass

        raw = _fetch_remote_frame_bytes(folder, frame, preferred_subdir=subdir)
        if not raw:
                return None
        try:
                target_dir.mkdir(parents=True, exist_ok=True)
                target_path.write_bytes(raw)
                return str(target_path)
        except Exception:
                return None


def _estimate_horizon_score_from_image(arr_u8):
        try:
                if arr_u8 is None or arr_u8.size == 0:
                        return None
                h, w = arr_u8.shape[:2]
                if h < 20 or w < 20:
                        return None
                step = max(1, int(max(h, w) / 320))
                arr = arr_u8[::step, ::step, :].astype(np.float32)
                gray = (0.299 * arr[:, :, 0]) + (0.587 * arr[:, :, 1]) + (0.114 * arr[:, :, 2])
                gx = gray[:, 2:] - gray[:, :-2]
                gy = gray[2:, :] - gray[:-2, :]
                gx = gx[1:-1, :]
                gy = gy[:, 1:-1]
                mag = np.abs(gx) + np.abs(gy)
                mask = mag > 18.0
                if not np.any(mask):
                        return None
                gxm = gx[mask]
                gym = gy[mask]
                sxx = float(np.sum(gxm * gxm))
                syy = float(np.sum(gym * gym))
                sxy = float(np.sum(gxm * gym))
                total = sxx + syy
                if total <= 1e-6:
                        return None
                phi = 0.5 * math.atan2((2.0 * sxy), (sxx - syy))
                line_ang = phi + (math.pi / 2.0)
                while line_ang > (math.pi / 2.0):
                        line_ang -= math.pi
                while line_ang < (-math.pi / 2.0):
                        line_ang += math.pi
                tilt_deg = abs(line_ang * (180.0 / math.pi))
                score = int(round(_clamp(100.0 - (max(0.0, tilt_deg - 2.0) * 2.8), 0.0, 100.0)))
                return score
        except Exception:
                return None


def _estimate_obstruction_score_from_image(arr_u8):
        try:
                if arr_u8 is None or arr_u8.size == 0:
                        return None
                h, w = arr_u8.shape[:2]
                if h < 20 or w < 20:
                        return None
                step = max(1, int(max(h, w) / 320))
                arr = arr_u8[::step, ::step, :].astype(np.float32)
                gray = (0.299 * arr[:, :, 0]) + (0.587 * arr[:, :, 1]) + (0.114 * arr[:, :, 2])
                std = float(np.std(gray))
                dark_ratio = float(np.mean(gray < 35.0))
                gx = gray[:, 2:] - gray[:, :-2]
                gy = gray[2:, :] - gray[:-2, :]
                edge = float(np.mean(np.abs(gx[1:-1, :]) + np.abs(gy[:, 1:-1])))
                risk = 0.0
                if std < 22.0:
                        risk += 25.0
                if edge < 11.0:
                        risk += 25.0
                if dark_ratio > 0.55:
                        risk += 20.0
                score = int(round(_clamp(100.0 - risk, 0.0, 100.0)))
                return score
        except Exception:
                return None


def _load_frame_rgb_array(dir_name, frame_name, preferred_subdir='optimal'):
        folder = str(dir_name or '').strip()
        frame = str(frame_name or '').strip()
        if not _timelapse_name_is_valid(folder) or not _frame_name_is_valid(frame):
                return None

        for path_obj in _timelapse_frame_dir_candidates(folder, preferred_view=preferred_subdir):
            candidate = path_obj / frame
            if not candidate.exists() or not candidate.is_file():
                    continue
            try:
                    with Image.open(candidate) as img:
                            return np.asarray(img.convert('RGB'), dtype=np.uint8)
            except Exception:
                    continue

        raw = _fetch_remote_frame_bytes(folder, frame, preferred_subdir=preferred_subdir)
        if not raw:
                return None
        try:
                with Image.open(io.BytesIO(raw)) as img:
                        return np.asarray(img.convert('RGB'), dtype=np.uint8)
        except Exception:
                return None


def _compute_preview_ai_summary(dir_name, frame_name, preferred_subdir='optimal'):
        payload = build_timelapse_frame_metadata_payload(dir_name, frame_name, include_histogram=False)
        if not payload.get('available'):
                return None

        current = payload.get('current') or {}
        metrics = current.get('metrics') or {}
        comparisons = payload.get('comparisons') or {}

        luma = float(metrics.get('median_luma') or 0.0)
        hi = max(0.0, float(metrics.get('highlight_clip_pct') or 0.0))
        sh = max(0.0, float(metrics.get('shadow_clip_pct') or 0.0))
        keys = ['5', '10', '20', '40']

        jump_penalty = 0.0
        windows = 0
        ev_penalty = 0.0
        luma_penalty = 0.0
        risk = 0.0

        for key in keys:
                cmp_item = comparisons.get(key) if isinstance(comparisons, dict) else None
                if not cmp_item or not cmp_item.get('available'):
                        continue
                d = cmp_item.get('delta') or {}
                d_ev = abs(float(d.get('delta_ev') or 0.0))
                d_luma = abs(float(d.get('median_luma') or 0.0))
                d_hi = max(0.0, float(d.get('highlight_clip_pct') or 0.0))
                d_sh = max(0.0, float(d.get('shadow_clip_pct') or 0.0))

                jump_penalty += _clamp((d_ev * 60.0) + (d_luma * 1.2), 0.0, 60.0)
                ev_penalty += _clamp(d_ev * 70.0, 0.0, 45.0)
                luma_penalty += _clamp(d_luma * 1.1, 0.0, 35.0)
                risk += _clamp(d_ev * 55.0, 0.0, 50.0) + _clamp(d_luma * 1.6, 0.0, 30.0) + _clamp((d_hi + d_sh) * 1.8, 0.0, 20.0)
                windows += 1

        jump_score = _clamp(100.0 - (jump_penalty / windows), 0.0, 100.0) if windows else 70.0
        target_luma = 112.0
        luma_score = _clamp(100.0 - _clamp(abs(luma - target_luma) * 0.9, 0.0, 45.0), 0.0, 100.0)
        clip_score = _clamp(100.0 - _clamp((hi * 4.0) + (sh * 2.5), 0.0, 60.0), 0.0, 100.0)
        trend_score = int(round(_clamp((0.45 * jump_score) + (0.35 * luma_score) + (0.20 * clip_score), 0.0, 100.0)))

        ev_score = _clamp(100.0 - (ev_penalty / windows), 0.0, 100.0) if windows else 70.0
        luma_stability = _clamp(100.0 - (luma_penalty / windows), 0.0, 100.0) if windows else 70.0
        smooth_score = int(round(_clamp((0.6 * ev_score) + (0.4 * luma_stability), 0.0, 100.0)))

        transition_score = int(round(_clamp(100.0 - (risk / windows), 0.0, 100.0))) if windows else 70

        recommendation = 'Hold'
        if hi > 6.0:
                recommendation = 'Reduce exposure'
        elif sh > 15.0 and hi < 2.0:
                recommendation = 'Lift shadows'
        elif smooth_score < 70:
                recommendation = 'Smooth ramp'
        elif luma < 70 and hi < 1.0 and smooth_score >= 70:
                recommendation = 'Nudge brighter'
        elif luma > 170 and sh < 5.0 and smooth_score >= 70:
                recommendation = 'Nudge darker'

        arr = _load_frame_rgb_array(dir_name, frame_name, preferred_subdir=preferred_subdir)
        horizon_score = _estimate_horizon_score_from_image(arr)
        obstruction_score = _estimate_obstruction_score_from_image(arr)

        score_parts = [trend_score, smooth_score, transition_score]
        if horizon_score is not None:
                score_parts.append(int(horizon_score))
        if obstruction_score is not None:
                score_parts.append(int(obstruction_score))

        summary = {
                'trend_score': int(trend_score),
                'smooth_score': int(smooth_score),
                'transition_score': int(transition_score),
                'recommendation': recommendation,
                'overall_score': int(round(sum(score_parts) / max(1, len(score_parts)))),
        }
        if horizon_score is not None:
                summary['horizon_score'] = int(horizon_score)
        if obstruction_score is not None:
                summary['obstruction_score'] = int(obstruction_score)
        return summary


def append_timelapse_ai_summary(dir_name, frame_name, summary):
        folder = str(dir_name or '').strip()
        frame = str(frame_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return {'ok': False, 'error': 'Invalid dir'}
        if not _frame_name_is_valid(frame):
                return {'ok': False, 'error': 'Invalid frame name'}
        if not isinstance(summary, dict):
                return {'ok': False, 'error': 'Invalid summary payload'}

        payload = {}
        if TIMELAPSE_AI_SUMMARY_PATH.exists():
                try:
                        payload = json.loads(TIMELAPSE_AI_SUMMARY_PATH.read_text(encoding='utf-8'))
                except Exception:
                        payload = {}
        if not isinstance(payload, dict):
                payload = {}

        entries = payload.get('entries') if isinstance(payload.get('entries'), dict) else {}
        payload['entries'] = entries
        key = f"{folder}/{frame}"
        entries[key] = {
                'dir': folder,
                'name': frame,
                'summary': summary,
                'updated_at': time.time(),
        }

        TIMELAPSE_AI_SUMMARY_PATH.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        return {'ok': True}


def recompute_timelapse_dir_stats(dir_name):
        folder = str(dir_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return {'ok': False, 'error': 'Invalid dir'}

        cleared_analysis = 0
        cleared_hist_persist = 0
        with TIMELAPSE_METADATA_CACHE_LOCK:
                cleared_analysis = len(TIMELAPSE_METADATA_CACHE)
                TIMELAPSE_METADATA_CACHE.clear()

        for path_obj in _timelapse_local_dir_candidates(folder):
                try:
                        if not path_obj.exists() or not path_obj.is_dir():
                                continue
                        cache_dir = path_obj.parent / '.sync_state' / 'frame_analysis_cache'
                        if not cache_dir.exists() or not cache_dir.is_dir():
                                continue
                        for entry in cache_dir.glob('*.json'):
                                try:
                                        entry.unlink()
                                        cleared_hist_persist += 1
                                except Exception:
                                        continue
                        break
                except Exception:
                        continue

        cleared_ai = 0
        if TIMELAPSE_AI_SUMMARY_PATH.exists():
                try:
                        payload = json.loads(TIMELAPSE_AI_SUMMARY_PATH.read_text(encoding='utf-8'))
                except Exception:
                        payload = {}
                entries = payload.get('entries') if isinstance(payload.get('entries'), dict) else {}
                keep = {}
                for key, item in entries.items():
                        item_dir = ''
                        if isinstance(item, dict):
                                item_dir = str(item.get('dir') or '').strip()
                        if item_dir == folder:
                                cleared_ai += 1
                        else:
                                keep[key] = item
                payload['entries'] = keep
                try:
                        TIMELAPSE_AI_SUMMARY_PATH.write_text(json.dumps(payload, indent=2), encoding='utf-8')
                except Exception:
                        pass

        return {
                'ok': True,
                'cleared': {
                        'analysis_entries': cleared_analysis,
                        'hist_mem_entries': 0,
                        'hist_persist_entries': cleared_hist_persist,
                        'ai_summary_entries': cleared_ai,
                },
        }


def _load_timelapse_ai_summary():
        try:
                if not TIMELAPSE_AI_SUMMARY_PATH.exists():
                        return {}
                payload = json.loads(TIMELAPSE_AI_SUMMARY_PATH.read_text(encoding='utf-8'))
                entries = payload.get('entries', {}) if isinstance(payload, dict) else {}
                per_dir = {}
                metric_keys = [
                        'trend_score',
                        'smooth_score',
                        'transition_score',
                        'horizon_score',
                        'obstruction_score',
                        'overall_score',
                ]
                for data in entries.values():
                        if not isinstance(data, dict):
                                continue
                        dir_name = str(data.get('dir') or '').strip()
                        summary = data.get('summary') if isinstance(data.get('summary'), dict) else {}
                        recommendation = summary.get('recommendation')
                        updated_at = data.get('updated_at')
                        if not dir_name:
                                continue
                        bucket = per_dir.setdefault(
                                dir_name,
                                {
                                        'metrics': {k: [] for k in metric_keys},
                                        'recommendation': None,
                                        'updated_at': 0,
                                },
                        )
                        for key in metric_keys:
                                value = summary.get(key)
                                if value is None:
                                        continue
                                try:
                                        bucket['metrics'][key].append(float(value))
                                except Exception:
                                        pass
                        if recommendation and not bucket['recommendation']:
                                bucket['recommendation'] = str(recommendation)
                        try:
                                upd = float(updated_at or 0)
                        except Exception:
                                upd = 0
                        if upd > bucket['updated_at']:
                                bucket['updated_at'] = upd
                                if recommendation:
                                        bucket['recommendation'] = str(recommendation)

                reduced = {}
                for dir_name, bucket in per_dir.items():
                        metrics = bucket.get('metrics', {})
                        averaged = {}
                        for key in metric_keys:
                                vals = metrics.get(key, [])
                                if vals:
                                        averaged[key] = round(sum(vals) / len(vals), 1)

                        if not averaged:
                                continue
                        reduced[dir_name] = {
                                'scores': averaged,
                                'overall_score': averaged.get('overall_score'),
                                'recommendation': bucket.get('recommendation') or 'n/a',
                                'updated_at': bucket.get('updated_at') or 0,
                        }
                return reduced
        except Exception:
                return {}


def _remote_count_image_files(remote_dir):
        remote_q = shlex.quote(str(remote_dir))
        cmd = (
                f"test -d {remote_q} && "
                f"find {remote_q} -maxdepth 1 -type f | wc -l || echo 0"
        )
        probe = subprocess.run(
                [
                        'ssh',
                        '-o', 'BatchMode=yes',
                        '-o', 'ConnectTimeout=10',
                        '-o', 'IdentitiesOnly=yes',
                        '-i', ORIN_SSH_KEY,
                        f'{ORIN_SSH_USER}@{ORIN_SSH_HOST}',
                        cmd,
                ],
                capture_output=True,
                text=True,
                check=False,
        )
        if probe.returncode != 0:
                return 0
        try:
                return int((probe.stdout or '0').strip().splitlines()[-1])
        except Exception:
                return 0


def _remote_counts_by_subdir(subdir_name):
        subdir_key = str(subdir_name or 'optimal').strip().lower()
        if subdir_key not in ('optimal', 'all'):
                subdir_key = 'optimal'
        if not _orin_is_reachable():
                with REMOTE_COUNTS_CACHE_LOCK:
                        return dict(REMOTE_COUNTS_CACHE.get(subdir_key, {}))
        counts = {}
        subdir_q = shlex.quote(subdir_key)
        root_q = shlex.quote(str(TIMELAPSE_ROOT))
        cmd = (
                f"for d in {root_q}/timelapse_[0-9]*_[0-9]*; do "
                "[ -d \"$d\" ] || continue; "
                f"target=\"$d\"/{subdir_q}; "
                "[ -d \"$target\" ] || continue; "
                "c=$(find \"$target\" -maxdepth 1 -type f | wc -l); "
                "printf '%s\t%s\n' \"$(basename \"$d\")\" \"$c\"; "
                "done | LC_ALL=C sort"
        )
        probe = subprocess.run(
                [
                        'ssh',
                        '-o', 'BatchMode=yes',
                        '-o', 'ConnectTimeout=3',
                        '-o', 'IdentitiesOnly=yes',
                        '-i', ORIN_SSH_KEY,
                        f'{ORIN_SSH_USER}@{ORIN_SSH_HOST}',
                        cmd,
                ],
                capture_output=True,
                text=True,
                check=False,
        )
        if probe.returncode != 0:
                with REMOTE_COUNTS_CACHE_LOCK:
                        return dict(REMOTE_COUNTS_CACHE.get(subdir_key, {}))

        for raw in (probe.stdout or '').splitlines():
                parts = raw.split('\t', 1)
                if len(parts) != 2:
                        continue
                name = parts[0].strip()
                try:
                        counts[name] = int(parts[1].strip())
                except Exception:
                        continue

        if counts:
                with REMOTE_COUNTS_CACHE_LOCK:
                        REMOTE_COUNTS_CACHE[subdir_key] = dict(counts)
                return counts

        with REMOTE_COUNTS_CACHE_LOCK:
                return dict(REMOTE_COUNTS_CACHE.get(subdir_key, {}))


def _remote_latest_mtime_by_subdir(subdir_name):
        subdir_key = str(subdir_name or 'optimal').strip().lower()
        if subdir_key not in ('optimal', 'all'):
                subdir_key = 'optimal'
        if not _orin_is_reachable():
                with REMOTE_LATEST_MTIME_CACHE_LOCK:
                        return dict(REMOTE_LATEST_MTIME_CACHE.get(subdir_key, {}))
        latest_by_dir = {}
        subdir_q = shlex.quote(subdir_key)
        root_q = shlex.quote(str(TIMELAPSE_ROOT))
        cmd = (
                f"for d in {root_q}/timelapse_[0-9]*_[0-9]*; do "
                "[ -d \"$d\" ] || continue; "
                f"target=\"$d\"/{subdir_q}; "
                "[ -d \"$target\" ] || continue; "
                "last=$(find \"$target\" -maxdepth 1 -type f "
                "\\( -name 'timelapse_*' -o -name 'capt_*' \\) "
                "-printf '%T@\\n' | LC_ALL=C sort -n | tail -n 1); "
                "[ -n \"$last\" ] || continue; "
                "printf '%s\\t%s\\n' \"$(basename \"$d\")\" \"$last\"; "
                "done | LC_ALL=C sort"
        )
        probe = subprocess.run(
                [
                        'ssh',
                        '-o', 'BatchMode=yes',
                        '-o', 'ConnectTimeout=3',
                        '-o', 'IdentitiesOnly=yes',
                        '-i', ORIN_SSH_KEY,
                        f'{ORIN_SSH_USER}@{ORIN_SSH_HOST}',
                        cmd,
                ],
                capture_output=True,
                text=True,
                check=False,
        )
        if probe.returncode != 0:
                with REMOTE_LATEST_MTIME_CACHE_LOCK:
                        return dict(REMOTE_LATEST_MTIME_CACHE.get(subdir_key, {}))

        for raw in (probe.stdout or '').splitlines():
                parts = raw.split('\t', 1)
                if len(parts) != 2:
                        continue
                name = parts[0].strip()
                try:
                        latest_by_dir[name] = float(parts[1].strip())
                except Exception:
                        continue

        if latest_by_dir:
                with REMOTE_LATEST_MTIME_CACHE_LOCK:
                        REMOTE_LATEST_MTIME_CACHE[subdir_key] = dict(latest_by_dir)
                return latest_by_dir

        with REMOTE_LATEST_MTIME_CACHE_LOCK:
                return dict(REMOTE_LATEST_MTIME_CACHE.get(subdir_key, {}))


def _remote_first_image_info(folder_name, preferred_subdir='optimal'):
        folder = str(folder_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return {'path': None, 'name': None, 'url': None}

        subdirs = []
        preferred = str(preferred_subdir or 'optimal').strip().lower()
        if preferred in ('all', 'optimal'):
                subdirs.append(preferred)
        for candidate in ('optimal', 'all'):
                if candidate not in subdirs:
                        subdirs.append(candidate)

        for subdir in subdirs:
                remote_dir = f"{TIMELAPSE_ROOT}/{folder}/{subdir}"
                remote_dir_q = shlex.quote(remote_dir)
                # Prefer the latest browser-displayable image (avoids early white calibration frames).
                cmd = (
                        f"if [ -d {remote_dir_q} ]; then "
                        f"find {remote_dir_q} -maxdepth 1 -type f \\( "
                        "-iname '*.jpg' -o -iname '*.jpeg' -o -iname '*.png' "
                        "-o -iname '*.webp' -o -iname '*.gif' -o -iname '*.bmp' \\) "
                        "-printf '%T@ %p\\n' | sort -n | tail -n 1 | cut -d' ' -f2-; "
                        "fi"
                )
                probe = subprocess.run(
                        [
                                'ssh',
                                '-o', 'BatchMode=yes',
                                '-o', 'ConnectTimeout=12',
                                '-o', 'IdentitiesOnly=yes',
                                '-i', ORIN_SSH_KEY,
                                f'{ORIN_SSH_USER}@{ORIN_SSH_HOST}',
                                cmd,
                        ],
                        capture_output=True,
                        text=True,
                        check=False,
                )
                if probe.returncode != 0:
                        continue
                path = str((probe.stdout or '').strip().splitlines()[-1] if (probe.stdout or '').strip() else '')
                if not path:
                        # Fallback: latest timelapse/capt file. May still be non-renderable (e.g., RAW).
                        fallback_cmd = (
                                f"if [ -d {remote_dir_q} ]; then "
                                f"find {remote_dir_q} -maxdepth 1 -type f \\( "
                                "-name 'timelapse_*' -o -name 'capt_*' \\) "
                                "-printf '%T@ %p\\n' | sort -n | tail -n 1 | cut -d' ' -f2-; "
                                "fi"
                        )
                        fallback_probe = subprocess.run(
                                [
                                        'ssh',
                                        '-o', 'BatchMode=yes',
                                        '-o', 'ConnectTimeout=12',
                                        '-o', 'IdentitiesOnly=yes',
                                        '-i', ORIN_SSH_KEY,
                                        f'{ORIN_SSH_USER}@{ORIN_SSH_HOST}',
                                        fallback_cmd,
                                ],
                                capture_output=True,
                                text=True,
                                check=False,
                        )
                        if fallback_probe.returncode == 0:
                                path = str((fallback_probe.stdout or '').strip().splitlines()[-1] if (fallback_probe.stdout or '').strip() else '')
                if not path:
                        continue
                return {
                        'path': path,
                        'name': Path(path).name,
                        'url': f"/api/timelapse-remote-preview?path={quote(path)}",
                }

        return {'path': None, 'name': None, 'url': None}


def _remote_preview_frame_urls(folder_name, preferred_subdir='optimal', limit=80):
        folder = str(folder_name or '').strip()
        if not _timelapse_name_is_valid(folder):
                return []

        subdirs = []
        preferred = str(preferred_subdir or 'optimal').strip().lower()
        if preferred in ('all', 'optimal'):
                subdirs.append(preferred)
        for candidate in ('optimal', 'all'):
                if candidate not in subdirs:
                        subdirs.append(candidate)

        try_limit = max(1, min(500, int(limit)))

        for subdir in subdirs:
                remote_dir = f"{TIMELAPSE_ROOT}/{folder}/{subdir}"
                remote_q = shlex.quote(remote_dir)
                cmd = (
                        f"if [ -d {remote_q} ]; then "
                        f"find {remote_q} -maxdepth 1 -type f "
                        "\\( -name 'timelapse_*' -o -name 'capt_*' \\) "
                        "-printf '%f\\n' | LC_ALL=C sort; "
                        "fi"
                )
                probe = subprocess.run(
                        [
                                'ssh',
                                '-o', 'BatchMode=yes',
                                '-o', 'ConnectTimeout=12',
                                '-o', 'IdentitiesOnly=yes',
                                '-i', ORIN_SSH_KEY,
                                f'{ORIN_SSH_USER}@{ORIN_SSH_HOST}',
                                cmd,
                        ],
                        capture_output=True,
                        text=True,
                        check=False,
                )
                if probe.returncode != 0:
                        continue

                names = [line.strip() for line in (probe.stdout or '').splitlines() if line.strip()]
                if not names:
                        continue
                if len(names) > try_limit:
                        names = names[-try_limit:]

                out = []
                for name in names:
                        path = f"{remote_dir}/{name}"
                        out.append(f"/api/timelapse-remote-preview?path={quote(path)}")
                return out

        return []


def _parse_sync_log_progress(log_text):
        progress = {
                'phase': 'idle',
                'message': '',
                'percent': None,
                'speed': None,
                'eta': None,
                'folder': None,
                'view': None,
                'watch': None,
                'files_done': None,
                'files_total': None,
                'files_remaining': None,
                'new_files': None,
                'processed_new': None,
                'failed_new': None,
        }
        if not log_text:
                return progress

        normalized = log_text.replace('\r', '\n')
        lines = [line.strip() for line in normalized.splitlines() if line.strip()]
        if not lines:
                return progress

        start_idx = -1
        for i, line in enumerate(lines):
                lower = line.lower()
                if 'start sync folder=' in lower:
                        start_idx = i
                elif 'sync start:' in lower and start_idx < 0:
                        start_idx = i

        relevant_lines = lines[start_idx:] if start_idx >= 0 else lines

        start_re = re.compile(r"start\s+sync\s+folder=(?P<folder>\S+)\s+view=(?P<view>all|optimal)\s+watch=(?P<watch>true|false)", re.IGNORECASE)
        for line in reversed(relevant_lines):
                match = start_re.search(line)
                if not match:
                        continue
                progress['folder'] = match.group('folder')
                progress['view'] = str(match.group('view') or '').lower()
                progress['watch'] = str(match.group('watch') or '').lower() == 'true'
                break

        for line in reversed(relevant_lines):
                if 'sync failed' in line.lower():
                        progress['phase'] = 'error'
                        progress['message'] = line
                        return progress
                if 'process complete:' in line.lower():
                        progress['phase'] = 'processing'
                        progress['message'] = line
                        break
                if 'sync complete:' in line.lower():
                        progress['phase'] = 'processing'
                        progress['message'] = line
                        match = re.search(r'new_files\s*=\s*(\d+)', line, flags=re.IGNORECASE)
                        if match:
                                try:
                                        progress['new_files'] = int(match.group(1))
                                except Exception:
                                        progress['new_files'] = None
                        break
                if 'sync start:' in line.lower():
                        progress['phase'] = 'syncing'
                        progress['message'] = line
                        break
                if 'no remote timelapse_' in line.lower():
                        progress['phase'] = 'waiting'
                        progress['message'] = line
                        break

        for line in reversed(relevant_lines):
                if 'process complete:' not in line.lower():
                        continue
                match = re.search(r'processed_new\s*=\s*(\d+)\s+failed\s*=\s*(\d+)', line, flags=re.IGNORECASE)
                if not match:
                        continue
                try:
                        progress['processed_new'] = int(match.group(1))
                        progress['failed_new'] = int(match.group(2))
                except Exception:
                        progress['processed_new'] = None
                        progress['failed_new'] = None
                break

        # Typical rsync --info=progress2 line:
        # 12.34M  42%   3.41MB/s    0:00:11 (xfr#3, to-chk=55/100)
        prog_re = re.compile(
                r"(?P<bytes>[0-9][0-9.,]*[A-Za-z]?)\s+"
                r"(?P<pct>[0-9]{1,3})%\s+"
                r"(?P<speed>[0-9.]+[A-Za-z]+/s)\s+"
                r"(?P<eta>[0-9:]+)"
        )
        for line in reversed(relevant_lines):
                match = prog_re.search(line)
                if not match:
                        continue
                try:
                        progress['percent'] = max(0, min(100, int(match.group('pct'))))
                except Exception:
                        progress['percent'] = None
                progress['speed'] = match.group('speed')
                progress['eta'] = match.group('eta')
                progress['phase'] = 'syncing'
                progress['message'] = line
                to_chk = re.search(r'to-chk\s*=\s*(\d+)\s*/\s*(\d+)', line, flags=re.IGNORECASE)
                if to_chk:
                        try:
                                remaining = int(to_chk.group(1))
                                total = int(to_chk.group(2))
                                done = max(0, total - remaining)
                                progress['files_remaining'] = remaining
                                progress['files_total'] = total
                                progress['files_done'] = done
                        except Exception:
                                progress['files_remaining'] = None
                                progress['files_total'] = None
                                progress['files_done'] = None
                return progress

        to_chk_only_re = re.compile(r'to-chk\s*=\s*(\d+)\s*/\s*(\d+)', re.IGNORECASE)
        for line in reversed(relevant_lines):
                match = to_chk_only_re.search(line)
                if not match:
                        continue
                try:
                        remaining = int(match.group(1))
                        total = int(match.group(2))
                        done = max(0, total - remaining)
                        progress['files_remaining'] = remaining
                        progress['files_total'] = total
                        progress['files_done'] = done
                except Exception:
                        progress['files_remaining'] = None
                        progress['files_total'] = None
                        progress['files_done'] = None
                if progress['phase'] == 'idle':
                        progress['phase'] = 'syncing'
                        progress['message'] = line
                break

        return progress


def _read_sync_progress():
        if not SYNC_LOG_PATH.exists():
                return {
                        'phase': 'idle',
                        'message': '',
                        'percent': None,
                        'speed': None,
                        'eta': None,
                }
        try:
                with open(SYNC_LOG_PATH, 'rb') as fh:
                        fh.seek(0, os.SEEK_END)
                        size = fh.tell()
                        window = min(size, 128 * 1024)
                        fh.seek(max(0, size - window), os.SEEK_SET)
                        chunk = fh.read(window)
                text = chunk.decode('utf-8', errors='replace')
                return _parse_sync_log_progress(text)
        except Exception:
                return {
                        'phase': 'idle',
                        'message': '',
                        'percent': None,
                        'speed': None,
                        'eta': None,
                }


def cleanup_sync_process():
        global SYNC_PROCESS, SYNC_FOLDER, SYNC_VIEW, SYNC_STARTED_AT, SYNC_WATCH, SYNC_INTERVAL_SECONDS
        with SYNC_STATE_LOCK:
                proc = SYNC_PROCESS
                if proc is not None and proc.poll() is not None:
                        SYNC_PROCESS = None
                        SYNC_FOLDER = None
                        SYNC_VIEW = 'optimal'
                        SYNC_STARTED_AT = None
                        SYNC_WATCH = False
                        SYNC_INTERVAL_SECONDS = 60


def get_sync_state():
        cleanup_sync_process()
        with SYNC_STATE_LOCK:
                proc = SYNC_PROCESS
                running = bool(proc is not None and proc.poll() is None)
                progress = _read_sync_progress()
                if not running and progress.get('phase') == 'syncing':
                        progress['phase'] = 'complete'
                        if progress.get('percent') is None:
                                progress['percent'] = 100
                return {
                        'running': running,
                        'folder': SYNC_FOLDER,
                        'view': SYNC_VIEW,
                        'watch': bool(SYNC_WATCH),
                        'interval_seconds': int(SYNC_INTERVAL_SECONDS),
                        'started_at': SYNC_STARTED_AT,
                        'pid': proc.pid if running else None,
                        'log_path': str(SYNC_LOG_PATH),
                        'progress': progress,
                }


def list_timelapse_directories(view='optimal', profile=False):
        view_mode = 'all' if view == 'all' else 'optimal'
        results = []
        timings = []
        request_start = time.perf_counter()
        hidden_dirs = _load_hidden_timelapse_dirs()

        def mark(label, started_at):
                timings.append((label, time.perf_counter() - started_at))

        phase_start = time.perf_counter()
        ai_summary_by_dir = _load_timelapse_ai_summary()
        mark('load_ai_summary', phase_start)

        phase_start = time.perf_counter()
        remote_optimal_counts = _remote_counts_by_subdir('optimal')
        remote_all_counts = _remote_counts_by_subdir('all')
        remote_optimal_latest = _remote_latest_mtime_by_subdir('optimal')
        remote_all_latest = _remote_latest_mtime_by_subdir('all')
        mark('load_remote_counts', phase_start)

        phase_start = time.perf_counter()
        root_q = shlex.quote(TIMELAPSE_ROOT)
        list_cmd = (
                f"find {root_q} -mindepth 1 -maxdepth 1 -type d "
                "-name 'timelapse_[0-9]*_[0-9]*' -printf '%f\\n' | LC_ALL=C sort"
        )
        ssh_cmd = [
                'ssh',
                '-o', 'BatchMode=yes',
                '-o', 'ConnectTimeout=3',
                '-o', 'IdentitiesOnly=yes',
                '-i', ORIN_SSH_KEY,
                f'{ORIN_SSH_USER}@{ORIN_SSH_HOST}',
                list_cmd,
        ]
        names = []
        if _orin_is_reachable():
                try:
                        _r = subprocess.run(ssh_cmd, capture_output=True, text=True, check=False, timeout=5)
                        if _r.returncode == 0:
                                names = [line.strip() for line in _r.stdout.splitlines() if line.strip()]
                except subprocess.TimeoutExpired:
                        pass
        mark('ssh_list_timelapse_dirs', phase_start)

        phase_start = time.perf_counter()
        for name in names:
                if not _timelapse_name_is_valid(name):
                        continue
                if name in hidden_dirs:
                        continue

                base_q = shlex.quote(f'{TIMELAPSE_ROOT}/{name}')
                subdir_check = subprocess.run(
                        [
                                'ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6',
                                '-o', 'IdentitiesOnly=yes', '-i', ORIN_SSH_KEY,
                                f'{ORIN_SSH_USER}@{ORIN_SSH_HOST}',
                                f"echo $(test -d {base_q}/all && echo 1 || echo 0) $(test -d {base_q}/optimal && echo 1 || echo 0)",
                        ],
                        capture_output=True,
                        text=True,
                        check=False,
                        timeout=8,
                )

                parts = subdir_check.stdout.strip().split() if subdir_check.returncode == 0 else []
                has_all = (parts[0] == '1') if len(parts) > 0 else False
                has_optimal = (parts[1] == '1') if len(parts) > 1 else False
                if not has_optimal:
                        continue
                selected_path = f"{TIMELAPSE_ROOT}/{name}/{view_mode}"
                local_base_dir = SCRIPT_DIR / 'timelapse_orin' / name
                local_optimal_dir = local_base_dir / 'optimal'
                local_all_dir = local_base_dir / 'all'
                local_selected_dir = local_all_dir if view_mode == 'all' else local_optimal_dir
                remote_optimal_count = int(remote_optimal_counts.get(name, 0) or 0) if has_optimal else 0
                remote_all_count = int(remote_all_counts.get(name, 0) or 0) if has_all else 0
                remote_selected_count = remote_all_count if view_mode == 'all' else remote_optimal_count
                try:
                        remote_latest_mtime = float((remote_all_latest if view_mode == 'all' else remote_optimal_latest).get(name, 0) or 0)
                except Exception:
                        remote_latest_mtime = 0.0
                local_selected_count = _count_image_files(local_selected_dir)
                local_all_count = _count_image_files(local_all_dir)
                local_latest = _latest_local_image_info(local_selected_dir)
                preview_frames = _preview_frame_urls(local_selected_dir)
                const_remote_ahead = remote_selected_count > local_selected_count
                if const_remote_ahead or not preview_frames:
                        preview_frames = _remote_preview_frame_urls(name, preferred_subdir=view_mode, limit=80)
                ai_meta = ai_summary_by_dir.get(name, {})
                preview_url = None
                preview_name = None
                remote_first = {'path': None, 'name': None, 'url': None}
                if const_remote_ahead or not local_latest.get('path'):
                        remote_first = _remote_first_image_info(name, preferred_subdir=view_mode)

                if local_latest.get('path') and not const_remote_ahead:
                        preview_url = f"/api/timelapse-preview?path={quote(local_latest['path'])}&ts={int(local_latest.get('mtime') or 0)}"
                        preview_name = local_latest.get('name')
                else:
                        preview_url = remote_first.get('url')
                        preview_name = remote_first.get('name')
                        if preview_name:
                                synced_path = _ensure_local_preview_frame(name, preview_name, preferred_subdir=view_mode)
                                if synced_path:
                                        try:
                                                synced_obj = Path(synced_path)
                                                synced_mtime = int(synced_obj.stat().st_mtime)
                                                preview_url = f"/api/timelapse-preview?path={quote(str(synced_obj))}&ts={synced_mtime}"
                                        except Exception:
                                                # Keep remote preview URL when local mirror write races or path is transient.
                                                pass

                last_photo_name = preview_name if const_remote_ahead else (local_latest.get('name') or preview_name)
                last_photo_mtime = remote_latest_mtime if const_remote_ahead else local_latest.get('mtime')

                if (not ai_meta or not ai_meta.get('scores')) and preview_name:
                        computed_ai = _compute_preview_ai_summary(name, preview_name, preferred_subdir=view_mode)
                        if computed_ai:
                                append_timelapse_ai_summary(name, preview_name, computed_ai)
                                ai_meta = {
                                        'scores': computed_ai,
                                        'overall_score': computed_ai.get('overall_score'),
                                        'recommendation': computed_ai.get('recommendation') or 'n/a',
                                }
                latest_export = get_latest_timelapse_export(name, prefer_tag='thumb', fallback_to_any=False)
                export_available = bool(latest_export.get('ok') and latest_export.get('available') and latest_export.get('download_url'))

                results.append({
                        'name': name,
                        'source': 'remote',
                        'path': f"{TIMELAPSE_ROOT}/{name}",
                        'has_all': has_all,
                        'has_optimal': has_optimal,
                        'selected_view': view_mode,
                        'selected_path': selected_path,
                        'selected_exists': has_all if view_mode == 'all' else has_optimal,
                        'remote_selected_count': remote_selected_count,
                        'remote_optimal_count': remote_optimal_count,
                        'remote_all_count': remote_all_count,
                        'remote_latest_mtime': remote_latest_mtime,
                        'local_selected_count': local_selected_count,
                        'local_selected_path': str(local_selected_dir),
                        'local_optimal_path': str(local_optimal_dir),
                        'local_optimal_count': _count_image_files(local_optimal_dir),
                        'local_all_path': str(local_all_dir),
                        'local_all_count': local_all_count,
                        'last_photo_name': last_photo_name,
                        'last_photo_is_fallback': _frame_name_is_fallback(last_photo_name),
                        'last_photo_mtime': last_photo_mtime,
                        'last_photo_url': preview_url,
                        'preview_url': preview_url,
                        'preview_name': preview_name,
                        'latest_export_available': export_available,
                        'latest_export_name': latest_export.get('file_name') if export_available else None,
                        'latest_export_url': (latest_export.get('media_url') or latest_export.get('download_url')) if export_available else None,
                        'latest_export_download_url': latest_export.get('download_url') if export_available else None,
                        'preview_frames': preview_frames,
                        'ai_score': ai_meta.get('overall_score'),
                        'ai_scores': ai_meta.get('scores') or {},
                        'ai_recommendation': ai_meta.get('recommendation'),
                        'frame_meta': build_latest_frame_meta_summary(name, local_selected_dir, preferred_subdir=view_mode),
                })
        mark('build_remote_rows', phase_start)

        # Include existing local timelapse folders so prior captures remain visible.
        phase_start = time.perf_counter()
        for child in sorted(SCRIPT_DIR.iterdir(), key=lambda p: p.name):
                if not child.is_dir():
                        continue
                if not child.name.startswith('timelapse'):
                        continue
                if child.name == 'timelapse_orin':
                        continue
                if child.name in hidden_dirs:
                        continue

                if any(r.get('name') == child.name and r.get('source') == 'remote' for r in results):
                        continue

                optimal_dir = child / 'optimal'
                all_dir = child / 'all'
                selected_dir = optimal_dir if optimal_dir.is_dir() else child
                local_optimal_count = _count_image_files(optimal_dir) if optimal_dir.is_dir() else _count_image_files(selected_dir)
                local_all_count = _count_image_files(all_dir) if all_dir.is_dir() else _count_image_files(selected_dir)
                local_latest = _latest_local_image_info(selected_dir)
                preview_frames = _preview_frame_urls(selected_dir)
                ai_meta = ai_summary_by_dir.get(child.name, {})
                preview_url = None
                if local_latest.get('path'):
                        preview_url = f"/api/timelapse-preview?path={quote(local_latest['path'])}&ts={int(local_latest.get('mtime') or 0)}"
                latest_export = get_latest_timelapse_export(child.name, prefer_tag='thumb', fallback_to_any=False) if _timelapse_name_is_valid(child.name) else {'ok': True, 'available': False}
                export_available = bool(latest_export.get('ok') and latest_export.get('available') and latest_export.get('download_url'))
                results.append({
                        'name': child.name,
                        'source': 'local',
                        'path': str(child),
                        'has_all': (child / 'all').is_dir(),
                        'has_optimal': optimal_dir.is_dir(),
                        'selected_view': 'local',
                        'selected_path': str(selected_dir),
                        'selected_exists': selected_dir.is_dir(),
                        'remote_selected_count': 0,
                        'local_selected_count': _count_image_files(selected_dir),
                        'local_optimal_path': str(optimal_dir),
                        'local_optimal_count': local_optimal_count,
                        'local_all_path': str(all_dir),
                        'local_all_count': local_all_count,
                        'last_photo_name': local_latest.get('name'),
                        'last_photo_is_fallback': _frame_name_is_fallback(local_latest.get('name')),
                        'last_photo_mtime': local_latest.get('mtime'),
                        'last_photo_url': preview_url,
                        'preview_url': preview_url,
                        'preview_name': local_latest.get('name'),
                        'latest_export_available': export_available,
                        'latest_export_name': latest_export.get('file_name') if export_available else None,
                        'latest_export_url': (latest_export.get('media_url') or latest_export.get('download_url')) if export_available else None,
                        'latest_export_download_url': latest_export.get('download_url') if export_available else None,
                        'preview_frames': preview_frames,
                        'ai_score': ai_meta.get('overall_score'),
                        'ai_scores': ai_meta.get('scores') or {},
                        'ai_recommendation': ai_meta.get('recommendation'),
                        'frame_meta': build_latest_frame_meta_summary(child.name, selected_dir, preferred_subdir='optimal'),
                })
        mark('build_local_rows', phase_start)

        phase_start = time.perf_counter()
        results.sort(key=lambda item: (0 if item.get('source') == 'remote' else 1, item.get('name', '')))
        mark('sort_results', phase_start)

        total_s = time.perf_counter() - request_start
        profile_payload = {
                'total_s': round(total_s, 3),
                'rows': len(results),
                'steps': [{
                        'name': label,
                        'ms': round(duration * 1000.0, 1),
                } for label, duration in timings],
        }
        if profile or total_s >= 2.5:
                step_summary = ', '.join(f"{item['name']}={item['ms']}ms" for item in profile_payload['steps'])
                print(f"[timing] /api/timelapse-directories total={profile_payload['total_s']}s rows={profile_payload['rows']} {step_summary}")
        return results, profile_payload


def _latest_local_timelapse_image_for_folder(folder_name):
        name = str(folder_name or '').strip()
        if not _timelapse_name_is_valid(name):
                return {}

        base_candidates = [
                SCRIPT_DIR / 'timelapse_orin' / name,
                SCRIPT_DIR / name,
        ]
        latest = {}
        best_rank = (-1, -1.0, '')

        for base in base_candidates:
                if not base.is_dir():
                        continue
                for sub in ('optimal', 'all', ''):
                        target = base / sub if sub else base
                        if not target.is_dir():
                                continue
                        info = _latest_local_image_info(target)
                        if not info.get('path'):
                                continue
                        seq_rank = _frame_sequence_value(info.get('name'))
                        seq_rank = seq_rank if seq_rank is not None else -1
                        mtime = float(info.get('mtime') or 0.0)
                        rank = (seq_rank, mtime, str(info.get('name') or ''))
                        if rank >= best_rank:
                                best_rank = rank
                                latest = info

        return latest


def build_latest_photo_payload(folder_name):
        name = str(folder_name or '').strip()
        latest = _latest_local_timelapse_image_for_folder(name)
        if not latest.get('path'):
                return {
                        'ok': True,
                        'folder': name,
                        'available': False,
                }

        mtime = int(latest.get('mtime') or 0)
        return {
                'ok': True,
                'folder': name,
                'available': True,
                'last_photo_name': latest.get('name'),
                'last_photo_is_fallback': _frame_name_is_fallback(latest.get('name')),
                'last_photo_mtime': latest.get('mtime'),
                'last_photo_url': f"/api/timelapse-preview?path={quote(str(latest['path']))}&ts={mtime}",
        }


def start_timelapse_sync(folder_name, view='optimal', watch=False, interval_seconds=60):
        global SYNC_PROCESS, SYNC_FOLDER, SYNC_VIEW, SYNC_STARTED_AT, SYNC_WATCH, SYNC_INTERVAL_SECONDS

        folder = str(folder_name or '').strip()
        view_mode = 'all' if view == 'all' else 'optimal'
        watch_mode = bool(watch)
        try:
                interval_val = int(interval_seconds)
        except Exception:
                interval_val = 60
        interval_val = max(5, min(3600, interval_val))
        if not _timelapse_name_is_valid(folder):
                return {'ok': False, 'error': 'Invalid folder name'}
        if not SYNC_SCRIPT_PATH.exists():
                return {'ok': False, 'error': f'Sync script not found: {SYNC_SCRIPT_PATH}'}

        with SYNC_STATE_LOCK:
                if SYNC_PROCESS is not None and SYNC_PROCESS.poll() is None:
                        return {
                                'ok': False,
                                'error': 'A sync is already running',
                                'running': True,
                                'folder': SYNC_FOLDER,
                        }

                try:
                        log_handle = open(SYNC_LOG_PATH, 'a', buffering=1)
                        log_handle.write(
                                f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] start sync folder={folder} view={view_mode} watch={watch_mode} interval={interval_val}s\n"
                        )
                        command = [
                                'bash',
                                str(SYNC_SCRIPT_PATH),
                                '--folder',
                                folder,
                                '--list-view',
                                view_mode,
                                '--remote-user',
                                ORIN_SSH_USER,
                                '--remote-host',
                                ORIN_SSH_HOST,
                                '--remote-dir',
                                TIMELAPSE_ROOT,
                                '--ssh-key',
                                ORIN_SSH_KEY,
                        ]
                        if watch_mode:
                                command.extend(['--watch', '--interval', str(interval_val)])
                        proc = subprocess.Popen(
                                command,
                                cwd=str(SCRIPT_DIR),
                                stdout=log_handle,
                                stderr=subprocess.STDOUT,
                                start_new_session=True,
                        )
                        SYNC_PROCESS = proc
                        SYNC_FOLDER = folder
                        SYNC_VIEW = view_mode
                        SYNC_WATCH = watch_mode
                        SYNC_INTERVAL_SECONDS = interval_val
                        SYNC_STARTED_AT = time.strftime('%Y-%m-%d %H:%M:%S')
                except Exception as exc:
                        return {'ok': False, 'error': f'Failed to start sync: {exc}'}

        return {
                'ok': True,
                'running': True,
                'folder': folder,
                'view': view_mode,
                'watch': watch_mode,
                'interval_seconds': interval_val,
                'pid': proc.pid,
        }


def stop_timelapse_sync():
        global SYNC_PROCESS, SYNC_FOLDER, SYNC_VIEW, SYNC_STARTED_AT, SYNC_WATCH, SYNC_INTERVAL_SECONDS

        with SYNC_STATE_LOCK:
                proc = SYNC_PROCESS
                if proc is None or proc.poll() is not None:
                        SYNC_PROCESS = None
                        SYNC_FOLDER = None
                        SYNC_VIEW = 'optimal'
                        SYNC_STARTED_AT = None
                        SYNC_WATCH = False
                        SYNC_INTERVAL_SECONDS = 60
                        return {'ok': True, 'running': False, 'stopped': False}

                try:
                        proc.terminate()
                        proc.wait(timeout=6)
                except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=6)
                except Exception:
                        pass

                SYNC_PROCESS = None
                SYNC_FOLDER = None
                SYNC_VIEW = 'optimal'
                SYNC_STARTED_AT = None
                SYNC_WATCH = False
                SYNC_INTERVAL_SECONDS = 60
                return {'ok': True, 'running': False, 'stopped': True}


def build_autotune_command(startup_iso=None, startup_fstop=None, startup_shutter=None,
                           lock_iso=False, lock_shutter=False, lock_aperture=False):
        resolved_iso = int(startup_iso) if startup_iso is not None else resolve_lowest_iso_choice()
        resolved_fstop = float(startup_fstop) if startup_fstop is not None else 16.0
        command = [
                resolve_python_executable(),
                str(AUTOTUNE_SCRIPT),
                '--duration-minutes', '120',
                '--interval-seconds', '3',
                '--capture-timeout', '90',
                '--capture-retry-seconds', '1',
                '--max-consecutive-capture-failures', '10',
                '--keep-dir', 'timelapse',
                '--reject-log', 'reject_candidates.csv',
                '--iso-strategy', 'last',
                '--startup-priority', 'low-iso',
                '--startup-set-baseline',
                '--iso-min', str(resolved_iso),
                '--startup-iso', str(resolved_iso),
                '--startup-fstop', str(resolved_fstop),
                '--target-luma', '135',
                '--deadband-ev', '0.05',
                '--startup-deadband-ev', '0.20',
                '--shutter-min', '1/8000',
                '--max-step-ev', '1.0',
                '--aggressive-breakout-ev', '0.5',
                '--aggressive-max-step-ev', '2.0',
                '--settled-max-step-ev', '0.25',
                '--settled-breakout-ev', '0.20',
                '--startup-max-step-ev', '2.5',
                '--startup-retry-seconds', '1',
                '--startup-max-iterations', '0',
                '--shutter-max', '1/30',
                '--fstop-stops', '1.4,1.8,2,2.8,4,5.6,8,11,16',
        ]
        if startup_shutter:
                command.extend(['--startup-shutter', str(startup_shutter)])
        if lock_aperture:
                command.append('--lock-aperture')
        if lock_iso:
                command.extend(['--iso-min', str(resolved_iso), '--iso-max', str(resolved_iso)])
        if lock_shutter and startup_shutter:
                command.extend(['--shutter-min', str(startup_shutter), '--shutter-max', str(startup_shutter)])
        return command


def cleanup_autotune_process():
        global AUTOTUNE_PROCESS, AUTOTUNE_LOG_HANDLE

        with AUTOTUNE_STATE_LOCK:
                proc = AUTOTUNE_PROCESS
                if proc is not None and proc.poll() is not None:
                        AUTOTUNE_PROCESS = None
                        if AUTOTUNE_LOG_HANDLE is not None:
                                try:
                                        AUTOTUNE_LOG_HANDLE.close()
                                except Exception:
                                        pass
                                AUTOTUNE_LOG_HANDLE = None


def kill_orphan_autotune_processes(exclude_pid=None):
        """Kill stray camera_auto_tune.py processes not tracked by this server instance."""
        killed = 0
        current_pid = os.getpid()
        tracked_exclude = int(exclude_pid) if exclude_pid else None

        try:
                probe = subprocess.run(
                        ['pgrep', '-af', 'camera_auto_tune.py'],
                        capture_output=True,
                        text=True,
                        check=False,
                )
        except Exception:
                return 0

        if probe.returncode not in (0, 1):
                return 0

        candidate_pids = []
        for raw_line in probe.stdout.splitlines():
                line = raw_line.strip()
                if not line:
                        continue
                parts = line.split(' ', 1)
                if not parts or not parts[0].isdigit():
                        continue
                pid = int(parts[0])
                cmd = parts[1] if len(parts) > 1 else ''
                if pid == current_pid:
                        continue
                if tracked_exclude is not None and pid == tracked_exclude:
                        continue
                if 'camera_auto_tune.py' not in cmd:
                        continue
                candidate_pids.append(pid)

        if not candidate_pids:
                return 0

        for pid in candidate_pids:
                try:
                        os.kill(pid, 15)
                        killed += 1
                except Exception:
                        pass

        time.sleep(0.2)
        for pid in candidate_pids:
                try:
                        os.kill(pid, 0)
                except Exception:
                        continue
                try:
                        os.kill(pid, 9)
                except Exception:
                        pass

        return killed


def get_autotune_state():
        cleanup_autotune_process()
        with AUTOTUNE_STATE_LOCK:
                proc = AUTOTUNE_PROCESS
                if proc is None:
                        return {'running': False, 'pid': None}
                return {'running': True, 'pid': proc.pid}


def start_autotune(startup_iso=None, startup_fstop=None, startup_shutter=None,
                   lock_iso=False, lock_shutter=False, lock_aperture=False):
        global AUTOTUNE_PROCESS, AUTOTUNE_LOG_HANDLE

        cleanup_autotune_process()
        kill_orphan_autotune_processes()
        with AUTOTUNE_STATE_LOCK:
                if AUTOTUNE_PROCESS is not None and AUTOTUNE_PROCESS.poll() is None:
                        return {'ok': True, 'running': True, 'pid': AUTOTUNE_PROCESS.pid}

                AUTOTUNE_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
                log_handle = open(AUTOTUNE_LOG_PATH, 'a', encoding='utf-8', buffering=1)
                try:
                        proc = subprocess.Popen(
                                build_autotune_command(startup_iso=startup_iso, startup_fstop=startup_fstop, startup_shutter=startup_shutter,
                                                       lock_iso=lock_iso, lock_shutter=lock_shutter, lock_aperture=lock_aperture),
                                cwd=str(SCRIPT_DIR),
                                stdout=log_handle,
                                stderr=subprocess.STDOUT,
                                text=True,
                                start_new_session=True,
                        )
                except Exception:
                        log_handle.close()
                        raise

                AUTOTUNE_PROCESS = proc
                AUTOTUNE_LOG_HANDLE = log_handle
                return {'ok': True, 'running': True, 'pid': proc.pid}


def stop_autotune():
        global AUTOTUNE_PROCESS, AUTOTUNE_LOG_HANDLE

        with AUTOTUNE_STATE_LOCK:
                proc = AUTOTUNE_PROCESS
                if proc is None or proc.poll() is not None:
                        AUTOTUNE_PROCESS = None
                        if AUTOTUNE_LOG_HANDLE is not None:
                                try:
                                        AUTOTUNE_LOG_HANDLE.close()
                                except Exception:
                                        pass
                                AUTOTUNE_LOG_HANDLE = None
                        orphan_killed = kill_orphan_autotune_processes()
                        return {'ok': True, 'running': False, 'stopped': False, 'orphan_killed': orphan_killed}

                try:
                        proc.terminate()
                        proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=5)

                AUTOTUNE_PROCESS = None
                if AUTOTUNE_LOG_HANDLE is not None:
                        try:
                                AUTOTUNE_LOG_HANDLE.close()
                        except Exception:
                                pass
                        AUTOTUNE_LOG_HANDLE = None
                orphan_killed = kill_orphan_autotune_processes(exclude_pid=proc.pid)
                return {'ok': True, 'running': False, 'stopped': True, 'orphan_killed': orphan_killed}


def find_latest_autotune_image_path():
        search_dirs = [SCRIPT_DIR, SCRIPT_DIR / 'timelapse']
        candidates = []

        for directory in search_dirs:
                if not directory.exists() or not directory.is_dir():
                        continue

                for path in directory.iterdir():
                        if not path.is_file():
                                continue
                        if path.suffix.lower() not in AUTOTUNE_IMAGE_EXTS:
                                continue
                        if not path.name.lower().startswith('capt_'):
                                continue
                        try:
                                mtime = path.stat().st_mtime
                        except Exception:
                                continue
                        candidates.append((mtime, path))

        if not candidates:
                        return None

        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[0][1]


def find_recent_autotune_images(limit=2):
        search_dirs = [SCRIPT_DIR, SCRIPT_DIR / 'timelapse']
        candidates = []

        for directory in search_dirs:
                if not directory.exists() or not directory.is_dir():
                        continue

                for path in directory.iterdir():
                        if not path.is_file():
                                continue
                        if path.suffix.lower() not in AUTOTUNE_IMAGE_EXTS:
                                continue
                        if not path.name.lower().startswith('capt_'):
                                continue
                        try:
                                mtime = path.stat().st_mtime
                        except Exception:
                                continue
                        candidates.append((mtime, path))

        if not candidates:
                return []

        candidates.sort(key=lambda item: item[0], reverse=True)
        return [item[1] for item in candidates[:max(1, int(limit))]]


def get_latest_autotune_payload(target_luma=135.0):
        latest_path = find_latest_autotune_image_path()
        if latest_path is None:
                return {
                        'available': False,
                        'error': 'No calibration image found yet',
                }

        try:
                mtime = latest_path.stat().st_mtime
        except Exception as exc:
                return {
                        'available': False,
                        'error': str(exc),
                }

        with AUTOTUNE_INFO_CACHE_LOCK:
                if (
                        AUTOTUNE_INFO_CACHE['path'] == str(latest_path)
                        and AUTOTUNE_INFO_CACHE['mtime'] == mtime
                        and AUTOTUNE_INFO_CACHE['payload'] is not None
                ):
                        return dict(AUTOTUNE_INFO_CACHE['payload'])

        try:
                from camera_ai_analyzer import analyze_image, format_shutter, read_capture_settings_from_exif

                metrics = analyze_image(image_path=latest_path, target_luma=target_luma)
                iso, shutter_s, fstop = read_capture_settings_from_exif(latest_path)
                recent_paths = find_recent_autotune_images(limit=2)
                previous_path = recent_paths[1] if len(recent_paths) > 1 else None
                prev_iso, prev_shutter_s, prev_fstop = (None, None, None)
                if previous_path is not None:
                        prev_iso, prev_shutter_s, prev_fstop = read_capture_settings_from_exif(previous_path)
                shutter_text = format_shutter(shutter_s) if shutter_s is not None else None
                image_url = f'/api/autotune/latest-image?name={latest_path.name}&ts={int(mtime)}'
                histogram = build_luma_histogram(latest_path, bins=64)
                changes = build_capture_changes(iso, shutter_s, fstop, prev_iso, prev_shutter_s, prev_fstop)
                reason = estimate_adjustment_reason(metrics)

                if metrics.highlight_clip_pct > 1.0 or metrics.delta_ev < -0.15:
                        recommendation = 'Darken'
                elif metrics.delta_ev > 0.15:
                        recommendation = 'Lighten'
                else:
                        recommendation = 'Optimized'

                payload = {
                        'available': True,
                        'image': {
                                'name': latest_path.name,
                                'mtime': mtime,
                                'url': image_url,
                        },
                        'capture': {
                                'iso': iso,
                                'shutter': shutter_text,
                                'fstop': fstop,
                        },
                        'metrics': {
                                'delta_ev': metrics.delta_ev,
                                'median_luma': metrics.median_luma,
                                'highlight_clip_pct': metrics.highlight_clip_pct,
                                'shadow_clip_pct': metrics.shadow_clip_pct,
                        },
                        'histogram': {
                                'bins': histogram,
                                'min': 0,
                                'max': 255,
                        },
                        'changes': changes,
                        'reason': reason,
                        'recommendation': recommendation,
                }
        except Exception as exc:
                payload = {
                        'available': False,
                        'error': str(exc),
                }

        with AUTOTUNE_INFO_CACHE_LOCK:
                AUTOTUNE_INFO_CACHE['path'] = str(latest_path)
                AUTOTUNE_INFO_CACHE['mtime'] = mtime
                AUTOTUNE_INFO_CACHE['payload'] = dict(payload)

        return payload


def send_file(handler, file_path):
        try:
                data = file_path.read_bytes()
        except Exception:
                handler._send_json({'error': 'Image not available'}, status=HTTPStatus.NOT_FOUND)
                return

        mime_type, _ = mimetypes.guess_type(str(file_path))
        if not mime_type:
                mime_type = 'application/octet-stream'

        handler.send_response(HTTPStatus.OK)
        handler.send_header('Content-Type', mime_type)
        handler.send_header('Cache-Control', 'no-store')
        handler._send_cors_headers()
        handler.send_header('Content-Length', str(len(data)))
        handler.end_headers()
        try:
                handler.wfile.write(data)
        except (ConnectionResetError, BrokenPipeError):
                # Client closed connection before receiving all data; suppress error
                pass


def release_camera_lock():
        # Kill macOS camera daemons that can lock the USB camera.
        # These daemons respawn aggressively, so we need multiple kills and a longer wait.
        for _ in range(3):  # Multiple kill attempts
                subprocess.run(
                        ['pkill', '-9', 'icdd'],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                )
                subprocess.run(
                        ['pkill', '-9', 'ptpcamerad'],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                )
                subprocess.run(
                        ['pkill', '-9', 'mscamerad'],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                )
                time.sleep(0.1)  # Brief pause between kill attempts
        time.sleep(2.0)  # Longer wait to let device settle before gphoto2 access


def run_gphoto(args, timeout=15):
        cmd = ['gphoto2'] + args
        try:
                return subprocess.run(
                        cmd,
                        capture_output=True,
                        text=True,
                        timeout=timeout,
                )
        except subprocess.TimeoutExpired as exc:
                stdout = exc.stdout if isinstance(exc.stdout, str) else ''
                stderr = exc.stderr if isinstance(exc.stderr, str) else ''
                timeout_msg = f'gphoto2 timeout after {timeout}s'
                return subprocess.CompletedProcess(
                        cmd,
                        124,
                        stdout,
                        f'{timeout_msg}\n{stderr}'.strip(),
                )
        except Exception as exc:
                return subprocess.CompletedProcess(cmd, 1, '', str(exc))


def invalidate_status_cache():
        global STATUS_CACHE_AT
        with STATUS_CACHE_LOCK:
                STATUS_CACHE.clear()
                STATUS_CACHE_AT = 0.0


def detect_camera(force=False):
        global DETECT_CACHE_AT, DETECT_CACHE_RESULT

        now = time.time()
        with DETECT_CACHE_LOCK:
                if (not force) and (now - DETECT_CACHE_AT) < DETECT_CACHE_TTL_S:
                        return DETECT_CACHE_RESULT

        with CAMERA_LOCK:
                release_camera_lock()
                result = run_gphoto(['--auto-detect'], timeout=5)

        if result.returncode != 0:
                detected = False
                reason = (result.stderr or result.stdout).strip()
        else:
                lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
                # Expected body line format: "<model> usb:XXX,YYY".
                detected_lines = [line for line in lines if 'usb:' in line.lower()]
                detected = len(detected_lines) > 0
                reason = 'Camera detected' if detected else 'No camera found via gphoto2 --auto-detect'

        cache_value = (detected, reason)
        with DETECT_CACHE_LOCK:
                DETECT_CACHE_RESULT = cache_value
                DETECT_CACHE_AT = time.time()

        return cache_value


def parse_get_config(raw_output):
        current = None
        readonly = None
        choices = []

        for line in raw_output.splitlines():
                line = line.strip()
                if line.startswith('Current:'):
                        current = line.split(':', 1)[1].strip()
                elif line.startswith('Readonly:'):
                        readonly = line.split(':', 1)[1].strip() == '1'
                elif line.startswith('Choice:'):
                        parts = line.split(' ', 2)
                        if len(parts) == 3:
                                choices.append(parts[2].strip())

        return {
                'current': current,
                'readonly': readonly,
                'choices': choices,
        }


def get_camera_setting(setting_name):
        with CAMERA_LOCK:
                release_camera_lock()
                result = run_gphoto(['--get-config', setting_name])

        if result.returncode != 0:
                return {
                        'available': False,
                        'error': (result.stderr or result.stdout).strip(),
                }

        parsed = parse_get_config(result.stdout)
        parsed['available'] = True
        return parsed


def set_camera_setting(setting_name, value):
        with CAMERA_LOCK:
                release_camera_lock()
                result = run_gphoto(['--set-config', f'{setting_name}={value}'])

        if result.returncode != 0:
                return {
                        'ok': False,
                        'error': (result.stderr or result.stdout).strip(),
                }

        return {'ok': True}


def trigger_capture():
        with CAMERA_LOCK:
                release_camera_lock()
                result = run_gphoto(['--trigger-capture'])

        return result.returncode == 0, (result.stderr or result.stdout).strip()


def set_movie_recording(enable):
        global MOVIE_RECORDING

        with CAMERA_LOCK:
                release_camera_lock()
                result = run_gphoto(['--set-config', f'movie={1 if enable else 0}'])

        if result.returncode != 0:
                return {
                        'ok': False,
                        'error': (result.stderr or result.stdout).strip(),
                        'recording': MOVIE_RECORDING,
                }

        with MOVIE_STATE_LOCK:
                MOVIE_RECORDING = bool(enable)

        return {'ok': True, 'recording': MOVIE_RECORDING}


def infer_movie_recording_from_snapshot(snapshot):
        # Sony's /main/actions/movie often reports an unusable constant value.
        # Infer movie state from mode-dependent settings that reliably change.
        exposure_program = str(
                snapshot.get('exposure-program', {}).get('current', '')
        ).strip()
        image_quality_readonly = bool(
                snapshot.get('image-quality', {}).get('readonly', False)
        )

        if 'movie' in exposure_program.lower():
                return True

        # Fallback heuristic when exposure program text is not populated.
        if image_quality_readonly:
                return True

        return False


def read_status():
        global STATUS_CACHE_AT

        now = time.time()
        with STATUS_CACHE_LOCK:
                if STATUS_CACHE and (now - STATUS_CACHE_AT) < STATUS_CACHE_TTL_S:
                        return dict(STATUS_CACHE)

        # Avoid running overlapping full-status sweeps when clients poll frequently.
        if not STATUS_REFRESH_LOCK.acquire(blocking=False):
                with STATUS_CACHE_LOCK:
                        if STATUS_CACHE:
                                return dict(STATUS_CACHE)
                return {
                        name: {'available': False, 'error': 'Status refresh in progress'}
                        for name in CAMERA_STATUS_SETTINGS
                }

        try:
                # Perform the full status sweep under one camera-lock session.
                # This avoids repeated daemon-kill sleeps per setting, which can
                # starve /api/set requests and appear as UI hangs.
                with CAMERA_LOCK:
                        release_camera_lock()

                        detect_result = run_gphoto(['--auto-detect'], timeout=5)
                        if detect_result.returncode != 0:
                                detect_reason = (detect_result.stderr or detect_result.stdout).strip()
                                detected = False
                        else:
                                lines = [line.strip() for line in detect_result.stdout.splitlines() if line.strip()]
                                detected_lines = [line for line in lines if 'usb:' in line.lower()]
                                detected = len(detected_lines) > 0
                                detect_reason = (
                                        'Camera detected'
                                        if detected
                                        else 'No camera found via gphoto2 --auto-detect'
                                )

                        cache_value = (detected, detect_reason)
                        with DETECT_CACHE_LOCK:
                                global DETECT_CACHE_RESULT, DETECT_CACHE_AT
                                DETECT_CACHE_RESULT = cache_value
                                DETECT_CACHE_AT = time.time()

                        if not detected:
                                snapshot = {
                                        name: {'available': False, 'error': detect_reason}
                                        for name in CAMERA_STATUS_SETTINGS
                                }
                        else:
                                snapshot = {}
                                for name, config_path in CAMERA_STATUS_SETTINGS.items():
                                        result = run_gphoto(['--get-config', config_path])
                                        if result.returncode != 0:
                                                snapshot[name] = {
                                                        'available': False,
                                                        'error': (result.stderr or result.stdout).strip(),
                                                }
                                        else:
                                                parsed = parse_get_config(result.stdout)
                                                parsed['available'] = True
                                                snapshot[name] = parsed

                inferred_movie = infer_movie_recording_from_snapshot(snapshot)
                with MOVIE_STATE_LOCK:
                        global MOVIE_RECORDING
                        MOVIE_RECORDING = bool(inferred_movie)

                with STATUS_CACHE_LOCK:
                        STATUS_CACHE.clear()
                        STATUS_CACHE.update(snapshot)
                        STATUS_CACHE_AT = time.time()
                return snapshot
        finally:
                STATUS_REFRESH_LOCK.release()


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>Camera Control</title>
    <style>
        :root {
            --bg: #f6efe5;
            --paper: #fff9f2;
            --ink: #1f1d1a;
            --muted: #5a564f;
            --accent: #005f73;
            --ok: #2d6a4f;
            --err: #9b2226;
            --line: #e6d7c7;
        }
        * { box-sizing: border-box; }
        body {
            margin: 0;
            font-family: "Avenir Next", "Gill Sans", "Trebuchet MS", sans-serif;
            background: radial-gradient(circle at 20% 20%, #fffdf9 0%, var(--bg) 60%);
            color: var(--ink);
        }
        .wrap {
            max-width: 860px;
            margin: 28px auto;
            padding: 0 14px 32px;
        }
        h1 {
            font-size: clamp(1.6rem, 4.3vw, 2.4rem);
            margin: 0 0 8px;
            letter-spacing: 0.02em;
        }
        .subtitle {
            color: var(--muted);
            margin-bottom: 20px;
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
            gap: 12px;
        }
        .card {
            background: var(--paper);
            border: 1px solid var(--line);
            border-radius: 14px;
            padding: 14px;
            box-shadow: 0 8px 24px rgba(0,0,0,0.05);
        }
        .label {
            text-transform: uppercase;
            letter-spacing: 0.08em;
            font-size: 0.73rem;
            color: var(--muted);
        }
        .value {
            font-size: 1.35rem;
            margin: 8px 0 12px;
            min-height: 1.6em;
        }
        form {
            display: flex;
            gap: 8px;
            flex-wrap: wrap;
        }
        input {
            flex: 1;
            min-width: 110px;
            padding: 8px 9px;
            border: 1px solid #ceb89f;
            border-radius: 8px;
            background: #fffcf8;
        }
        button {
            border: none;
            border-radius: 8px;
            padding: 8px 12px;
            background: var(--accent);
            color: white;
            cursor: pointer;
            font-weight: 600;
        }
        button:hover { filter: brightness(1.05); }
        .toolbar {
            margin: 14px 0;
            display: flex;
            gap: 8px;
            flex-wrap: wrap;
        }
        .msg {
            margin-top: 14px;
            min-height: 1.2em;
            color: var(--muted);
        }
        .msg.ok { color: var(--ok); }
        .msg.err { color: var(--err); }
        .choices {
            color: var(--muted);
            font-size: 0.8rem;
            margin-top: 7px;
            line-height: 1.3;
            min-height: 1.3em;
        }
    </style>
</head>
<body>
    <div class="wrap">
        <h1>Camera Control Panel</h1>
        <div class="subtitle">Live values from gphoto2 for f-number, ISO, and shutter speed.</div>
                                                <th>AI</th>

        <div class="toolbar">
                                                <th>Viewer</th>
            <button id="triggerBtn" type="button">Trigger Capture</button>
        </div>

        <div class="grid" id="cards"></div>
        <div id="message" class="msg"></div>
    </div>

    <script>
        const SETTINGS = ['f-number', 'iso', 'shutterspeed'];
        const cards = document.getElementById('cards');
        const message = document.getElementById('message');

        function setMessage(text, kind = '') {
            message.textContent = text;
            message.className = 'msg' + (kind ? ' ' + kind : '');
        }

        function cardHtml(name, data) {
            const available = data && data.available;
            const current = available ? (data.current || '(no value)') : '(unavailable)';
            const isReadOnly = available && data.readonly === true;
            let choicesText = '';
            if (available && Array.isArray(data.choices) && data.choices.length) {
                choicesText = 'Choices: ' + data.choices.slice(0, 14).join(', ');
            }
            if (!available && data && data.error) {
                choicesText = data.error;
            }

            return `
                <div class="card" data-name="${name}">
                    <div class="label">${name}</div>
                    <div class="value">${current}</div>
                    <form>
                        <input name="value" placeholder="Set ${name}" ${isReadOnly ? 'disabled' : ''}>
                        <button type="submit" ${isReadOnly ? 'disabled' : ''}>Apply</button>
                    </form>
                    <div class="choices">${choicesText}</div>
                </div>
            `;
        }

        async function fetchStatus() {
            const r = await fetch('/api/status');
            if (!r.ok) {
                throw new Error('Status request failed');
            }
            return r.json();
        }

        async function refresh() {
            setMessage('Refreshing...');
            try {
                const status = await fetchStatus();
                cards.innerHTML = SETTINGS.map(name => cardHtml(name, status[name])).join('');
                bindForms();
                setMessage('Status updated', 'ok');
            } catch (err) {
                setMessage('Refresh failed: ' + err.message, 'err');
            }
        }

        function bindForms() {
            document.querySelectorAll('.card form').forEach(form => {
                form.addEventListener('submit', async (e) => {
                    e.preventDefault();
                    const card = e.target.closest('.card');
                    const setting = card.getAttribute('data-name');
                    const value = new FormData(form).get('value');
                    if (!value) {
                        setMessage('Enter a value for ' + setting, 'err');
                        return;
                    }

                    setMessage('Applying ' + setting + '...');
                                        const controller = new AbortController();
                                        const timeoutId = setTimeout(() => controller.abort(), 12000);

                                        try {
                                                const r = await fetch('/api/set', {
                                                        method: 'POST',
                                                        headers: {'Content-Type': 'application/json'},
                                                        body: JSON.stringify({ setting, value }),
                                                        signal: controller.signal,
                                                });
                                                const body = await r.json();
                                                if (!r.ok || !body.ok) {
                                                        setMessage('Set failed for ' + setting + ': ' + (body.error || 'unknown error'), 'err');
                                                        return;
                                                }
                                                setMessage(setting + ' set to ' + value, 'ok');
                                                await refresh();
                                        } catch (err) {
                                                if (err && err.name === 'AbortError') {
                                                        setMessage('Set request timed out for ' + setting, 'err');
                                                } else {
                                                        setMessage('Set failed for ' + setting + ': ' + err.message, 'err');
                                                }
                                        } finally {
                                                clearTimeout(timeoutId);
                    }
                });
            });
        }

        document.getElementById('refreshBtn').addEventListener('click', refresh);
        document.getElementById('triggerBtn').addEventListener('click', async () => {
            setMessage('Triggering...');
            const r = await fetch('/api/trigger', { method: 'POST' });
            const body = await r.json();
            if (r.ok && body.ok) {
                setMessage('Photo triggered', 'ok');
            } else {
                setMessage('Trigger failed: ' + (body.error || 'unknown error'), 'err');
            }
        });

        refresh();
    </script>
</body>
</html>
"""

TIMELAPSE_PAGE = """<!doctype html>
<html lang=\"en\">
<head>
        <meta charset=\"utf-8\">
        <meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">
        <title>Timelapse Directories</title>
        <style>
                * { box-sizing: border-box; }
                body {
                        margin: 0;
                        font-family: \"Avenir Next\", \"Gill Sans\", \"Trebuchet MS\", sans-serif;
                        background: #111;
                        color: #fff;
                }
                .header {
                        background: #1a1a1a;
                        border-bottom: 2px solid #ff9000;
                        padding: 15px 20px;
                        display: flex;
                        align-items: center;
                        justify-content: space-between;
                        gap: 12px;
                        flex-wrap: wrap;
                }
                .header h1 {
                        margin: 0;
                        color: #ff9000;
                        font-size: 1.5em;
                }
                .nav {
                        display: flex;
                        gap: 15px;
                        flex-wrap: wrap;
                        align-items: center;
                }
                .nav a {
                        color: #aaa;
                        text-decoration: none;
                        padding: 5px 10px;
                        border-radius: 4px;
                        transition: all .3s;
                        display: inline-flex;
                        align-items: center;
                        justify-content: center;
                        min-width: 96px;
                }
                .nav a:hover { color: #fff; background: #333; }
                .nav a.active { color: #ff9000; background: #222; }
                .nav label {
                        color: #aaa;
                        display: flex;
                        align-items: center;
                        gap: 6px;
                        min-width: 190px;
                        justify-content: center;
                }
                .nav label.nav-group { min-width: 170px; }
                .nav select {
                        background: #111;
                        border: 1px solid #333;
                        color: #fff;
                        border-radius: 4px;
                        padding: 5px 8px;
                        width: 150px;
                }
                .wrap { max-width: 1080px; margin: 0 auto; padding: 20px 14px 32px; }
                .section {
                        background: #1a1a1a;
                        border: 1px solid #333;
                        border-radius: 8px;
                        padding: 20px;
                }
                h2 {
                        margin: 0 0 12px;
                        color: #ff9000;
                        font-size: 1.3em;
                        border-bottom: 1px solid #333;
                        padding-bottom: 10px;
                }
                .sub { color: #aaa; margin-bottom: 14px; }
                .toolbar { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 10px; align-items: center; }
                .view-hint { margin: 6px 0 10px; color: #ffcf8c; font-size: 0.86rem; }
                button {
                        border: 1px solid #333;
                        border-radius: 8px;
                        padding: 8px 10px;
                        background: #ff9000;
                        color: #000;
                        font-weight: 700;
                        cursor: pointer;
                }
                .status { margin: 8px 0 10px; color: #aaa; }
                .status.ok { color: #00e676; }
                .status.err { color: #ff6b6b; }
                .stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); gap: 8px; margin: 8px 0 12px; }
                .stat {
                        background: #141414;
                        border: 1px solid #2f2f2f;
                        border-radius: 8px;
                        padding: 8px 10px;
                }
                .stat-label { color: #9c958c; font-size: 0.75rem; text-transform: uppercase; letter-spacing: 0.04em; }
                .stat-value { color: #fff; font-size: 1.1rem; font-weight: 700; margin-top: 2px; }
                .progress-wrap {
                        margin: 8px 0 14px;
                        background: #151515;
                        border: 1px solid #333;
                        border-radius: 10px;
                        padding: 10px;
                }
                .progress-title { color: #ffb45c; font-size: 0.85rem; margin-bottom: 6px; }
                .progress-track {
                        width: 100%;
                        height: 12px;
                        background: #0d0d0d;
                        border: 1px solid #2a2a2a;
                        border-radius: 999px;
                        overflow: hidden;
                }
                .progress-fill {
                        height: 100%;
                        width: 0%;
                        background: linear-gradient(90deg, #ff6f00, #ff9000, #ffba54);
                        transition: width 0.35s ease;
                }
                .progress-fill.indeterminate {
                        width: 35%;
                        animation: indeterminate 1.2s linear infinite;
                }
                .progress-meta {
                        display: flex;
                        gap: 14px;
                        flex-wrap: wrap;
                        margin-top: 8px;
                        color: #b8b1a8;
                        font-size: 0.88rem;
                }
                @keyframes indeterminate {
                        from { transform: translateX(-120%); }
                        to { transform: translateX(320%); }
                }
                .preview {
                        width: 180px;
                        height: 102px;
                        border-radius: 8px;
                        border: 1px solid #3a3a3a;
                        object-fit: cover;
                        background: #0f0f0f;
                        display: block;
                        max-width: 180px;
                        max-height: 102px;
                        cursor: default;
                }
                .preview-video {
                        cursor: pointer;
                }
                .preview-playbar {
                        width: 180px;
                        height: 4px;
                        margin-top: 4px;
                        border-radius: 999px;
                        background: #1b2730;
                        border: 1px solid #2f414f;
                        overflow: hidden;
                        cursor: ew-resize;
                }
                .preview-playbar-fill {
                        height: 100%;
                        width: 0%;
                        background: linear-gradient(90deg, #5cbcff, #9affc8);
                        transition: width 0.08s linear;
                }
                .preview-open {
                        border: none;
                        padding: 0;
                        background: transparent;
                        cursor: zoom-in;
                        display: inline-block;
                }
                .preview-empty {
                        width: 180px;
                        height: 102px;
                        border-radius: 8px;
                        border: 1px dashed #3a3a3a;
                        display: flex;
                        align-items: center;
                        justify-content: center;
                        color: #777;
                        font-size: 0.75rem;
                        max-width: 180px;
                        max-height: 102px;
                }
                .preview-actions {
                        margin-top: 6px;
                        display: flex;
                        align-items: center;
                        gap: 6px;
                        flex-wrap: wrap;
                }
                .loop-range-fields {
                        display: flex;
                        align-items: center;
                        gap: 6px;
                        flex-wrap: wrap;
                }
                .loop-range-label {
                        color: #7ba4c5;
                        font-size: 0.7rem;
                        line-height: 1.1;
                }
                .loop-time-readout {
                        border: 1px solid #2f3a46;
                        background: #121a20;
                        color: #b9d8f2;
                        border-radius: 6px;
                        padding: 2px 7px;
                        font-size: 0.7rem;
                        line-height: 1.2;
                        min-width: 64px;
                        text-align: center;
                        cursor: text;
                }
                .loop-mode-btn {
                        border: 1px solid #325a79;
                        background: #142636;
                        color: #a7c8e2;
                        border-radius: 999px;
                        padding: 2px 8px;
                        font-size: 0.68rem;
                        line-height: 1.2;
                        cursor: pointer;
                }
                .loop-mode-btn.is-active {
                        border-color: #68b6ff;
                        background: #1c3951;
                        color: #d6ebff;
                }
                .loop-mark-btn {
                        border: 1px solid #3f5f2f;
                        background: #1b2e16;
                        color: #bfe8b0;
                        border-radius: 999px;
                        padding: 2px 8px;
                        font-size: 0.68rem;
                        line-height: 1.2;
                        cursor: pointer;
                }
                .loop-timeline {
                        position: relative;
                        width: 180px;
                        height: 18px;
                        border-radius: 999px;
                        border: 1px solid #30414f;
                        background: linear-gradient(180deg, #0f171d, #0b1116);
                        cursor: crosshair;
                        overflow: hidden;
                }
                .loop-timeline-range {
                        position: absolute;
                        top: 0;
                        height: 100%;
                        border-radius: 999px;
                        background: linear-gradient(90deg, rgba(92, 190, 255, 0.38), rgba(122, 227, 255, 0.28));
                        pointer-events: none;
                }
                .loop-marker {
                        position: absolute;
                        top: -2px;
                        width: 6px;
                        height: 22px;
                        border-radius: 3px;
                        border: 1px solid #09131a;
                        background: #79cbff;
                        transform: translateX(-50%);
                        cursor: ew-resize;
                        z-index: 2;
                }
                .loop-marker-out {
                        background: #8cffbf;
                }
                .export-needed-btn {
                        border: 1px solid #5e3b12;
                        background: #2a1b0c;
                        color: #ffcc8c;
                        border-radius: 999px;
                        padding: 3px 8px;
                        font-size: 0.72rem;
                        line-height: 1.1;
                        cursor: pointer;
                }
                .export-needed-btn:hover {
                        background: #3a250f;
                        color: #ffd9aa;
                }
                .reexport-btn {
                        border: 1px solid #1f4b2f;
                        background: #12301f;
                        color: #9ee5b2;
                        border-radius: 999px;
                        padding: 3px 8px;
                        font-size: 0.72rem;
                        line-height: 1.1;
                        cursor: pointer;
                }
                .reexport-btn:hover {
                        background: #18452a;
                        color: #c8f7d6;
                }
                .thumb {
                        width: 100%;
                        max-width: 360px;
                        height: auto;
                        max-height: 240px;
                        border-radius: 8px;
                        border: 1px solid #3a3a3a;
                        object-fit: contain;
                        background: #0f0f0f;
                        display: block;
                }
                .tiny { font-size: 0.74rem; color: #888; margin-top: 4px; max-width: 220px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
                .expander-btn {
                        width: 28px;
                        height: 28px;
                        border-radius: 6px;
                        border: 1px solid #444;
                        background: #1f1f1f;
                        color: #ff9000;
                        font-size: 0.95rem;
                        line-height: 1;
                        padding: 0;
                        cursor: pointer;
                }
                .expander-btn[aria-expanded="true"] {
                        background: #2a1f10;
                        border-color: #ff9000;
                }
                .expand-cell { width: 40px; }
                .detail-row { display: none; }
                .detail-row.open { display: table-row; }
                .detail-panel {
                        background: #141414;
                        border-top: 1px solid #2b2b2b;
                        padding: 12px;
                }
                .detail-image {
                        width: 960px;
                        max-width: 92vw;
                        height: auto;
                        aspect-ratio: 16 / 9;
                        object-fit: cover;
                        border: 1px solid #3a3a3a;
                        border-radius: 10px;
                        background: #000;
                        display: block;
                }
                .detail-photo-open {
                        border: none;
                        padding: 0;
                        background: transparent;
                        cursor: zoom-in;
                        display: inline-block;
                }
                .detail-full-link {
                        display: inline-block;
                        margin-top: 8px;
                        color: #ffb45c;
                        font-size: 0.82rem;
                        text-decoration: none;
                }
                .detail-full-link:hover { text-decoration: underline; }
                .detail-content {
                        display: flex;
                        align-items: flex-start;
                        gap: 14px;
                        flex-wrap: wrap;
                }
                .detail-photo-block {
                        min-width: 320px;
                        width: 100%;
                        max-width: 960px;
                }
                .detail-meta {
                        min-width: 280px;
                        max-width: 520px;
                        display: grid;
                        gap: 8px;
                }
                .detail-dir-name {
                        color: #ffbe69;
                        font-size: 0.9rem;
                        font-weight: 700;
                        margin-bottom: 2px;
                }
                .detail-meta-top {
                        display: grid;
                        grid-template-columns: minmax(260px, 1fr) minmax(260px, 1fr);
                        align-items: stretch;
                        gap: 10px;
                }
                .ai-card {
                        min-width: 0;
                        background: #151515;
                        border: 1px solid #2f2f2f;
                        border-radius: 8px;
                        padding: 10px;
                }
                .ai-top {
                        display: flex;
                        justify-content: space-between;
                        align-items: center;
                        margin-bottom: 8px;
                        color: #ffcf8c;
                        font-size: 0.88rem;
                }
                .ai-grid {
                        display: grid;
                        grid-template-columns: 1fr 1fr;
                        gap: 6px 12px;
                        font-size: 0.82rem;
                        color: #d4cec5;
                }
                .ai-grid span { display: flex; justify-content: space-between; gap: 8px; }
                .ai-grid b { color: #ffb45c; font-weight: 600; }
                .ai-label-tip {
                        cursor: help;
                        text-decoration: underline dotted rgba(255, 180, 92, 0.7);
                        text-underline-offset: 2px;
                        position: relative;
                }
                .ai-label-tip:hover::after {
                        content: attr(data-tip);
                        position: absolute;
                        left: 0;
                        bottom: calc(100% + 6px);
                        min-width: 190px;
                        max-width: 260px;
                        z-index: 6;
                        border: 1px solid #4a3a26;
                        background: #1e1710;
                        color: #f4ddbf;
                        border-radius: 6px;
                        padding: 6px 8px;
                        font-size: 0.72rem;
                        line-height: 1.35;
                        text-transform: none;
                        letter-spacing: normal;
                        white-space: normal;
                        box-shadow: 0 8px 20px rgba(0, 0, 0, 0.38);
                }
                .ai-recommendation {
                        color: #c9bba8;
                        font-size: 0.83rem;
                        line-height: 1.35;
                }
                .meta-mini {
                        margin-top: 0;
                        border: 1px solid #2c2c2c;
                        border-radius: 8px;
                        padding: 8px;
                        background: #121212;
                        min-width: 0;
                }
                .meta-mini-title {
                        color: #9d968e;
                        font-size: 0.72rem;
                        text-transform: uppercase;
                        letter-spacing: 0.05em;
                        margin-bottom: 5px;
                }
                .meta-mini-row {
                        display: grid;
                        grid-template-columns: auto 1fr auto;
                        gap: 8px;
                        align-items: baseline;
                        font-size: 0.79rem;
                        line-height: 1.45;
                }
                .meta-mini-row .k { color: #9b948b; }
                .meta-mini-row .v { color: #f1ece5; text-align: right; font-variant-numeric: tabular-nums; }
                .meta-mini-row .d { font-variant-numeric: tabular-nums; font-size: 0.76rem; min-width: 58px; text-align: right; }
                .meta-delta-pos { color: #86efac; }
                .meta-delta-neg { color: #fca5a5; }
                .meta-delta-zero { color: #9ca3af; }
                table {
                        width: 100%;
                        border-collapse: collapse;
                        background: #1a1a1a;
                        border: 1px solid #333;
                        border-radius: 12px;
                        overflow: hidden;
                }
                th, td {
                        text-align: left;
                        padding: 10px;
                        border-bottom: 1px solid #2b2b2b;
                        vertical-align: middle;
                }
                td.preview-cell { min-width: 220px; }
                th { color: #aaa; font-size: 0.85rem; text-transform: uppercase; letter-spacing: 0.04em; }
                tr:last-child td { border-bottom: none; }
                tr.live-optimal-stalled td {
                        background: #361617;
                        border-bottom-color: #5a2a2b;
                }
                tr.live-optimal-stalled td:first-child {
                        border-left: 3px solid #e34b4b;
                }
                .dim { color: #8f8a84; }
                .folder-link {
                        color: #ffbe69;
                        text-decoration: none;
                        font-weight: 700;
                }
                .folder-link:hover {
                        text-decoration: underline;
                }
                .pill {
                        display: inline-block;
                        border: 1px solid #444;
                        border-radius: 999px;
                        padding: 2px 8px;
                        font-size: 0.78rem;
                        color: #aaa;
                        background: #222;
                }
                .pill-live {
                        border-color: #00e676;
                        color: #0e2618;
                        background: #00e676;
                        font-weight: 700;
                        margin-left: 6px;
                }
                .pill-remote-missing {
                        border-color: #f59e0b;
                        color: #fef3c7;
                        background: #3a2a0a;
                        font-weight: 700;
                        margin-left: 6px;
                }
                .pill-fallback {
                        border-color: #60a5fa;
                        color: #dbeafe;
                        background: #1e3a5f;
                        font-weight: 700;
                        margin-left: 6px;
                }
                .remote-missing-hint {
                        color: #ffcf8c;
                        max-width: 280px;
                        white-space: normal;
                        overflow: visible;
                        text-overflow: clip;
                }
                .stall-hint {
                        color: #ff9b9b;
                        font-weight: 600;
                        max-width: 320px;
                        white-space: normal;
                        overflow: visible;
                        text-overflow: clip;
                }
                .count-muted {
                        color: #9b968f;
                }
                .sync-btn {
                        background: #ff9000;
                        color: #000;
                        border: none;
                        border-radius: 8px;
                        padding: 0 12px;
                        font-weight: 600;
                        font-size: 1rem;
                        line-height: 1;
                        min-width: 132px;
                        height: 52px;
                        display: inline-flex;
                        align-items: center;
                        justify-content: center;
                        text-align: center;
                        white-space: nowrap;
                        text-decoration: none;
                        cursor: pointer;
                }
                .sync-btn:disabled { opacity: 0.5; cursor: not-allowed; }
                .sync-action-btn {
                        width: 34px;
                        min-width: 34px;
                        height: 34px;
                        padding: 0;
                        border-radius: 8px;
                        font-size: 16px;
                        line-height: 1;
                }
                .row-actions {
                        display: inline-flex;
                        align-items: center;
                        gap: 8px;
                        flex-wrap: nowrap;
                }
                .delete-dir-btn {
                        width: 28px;
                        min-width: 28px;
                        height: 28px;
                        padding: 0;
                        border: 1px solid #7a2a2a;
                        border-radius: 6px;
                        background: #5b1f1f;
                        color: #ffd5d5;
                        display: inline-flex;
                        align-items: center;
                        justify-content: center;
                        font-size: 14px;
                        line-height: 1;
                }
                .delete-dir-btn:hover:not(:disabled) {
                        background: #6f2525;
                        border-color: #9b3333;
                }
                .delete-dir-btn:disabled {
                        color: #c9a8a8;
                }
                .photo-modal {
                        position: fixed;
                        inset: 0;
                        display: none;
                        align-items: center;
                        justify-content: center;
                        background: rgba(0, 0, 0, 0.86);
                        z-index: 9999;
                        padding: 20px;
                }
                .photo-modal.open { display: flex; }
                .photo-modal-content {
                        position: relative;
                        width: min(96vw, 1280px);
                        max-height: 94vh;
                        background: #111;
                        border: 1px solid #2c2c2c;
                        border-radius: 12px;
                        padding: 12px;
                        display: grid;
                        grid-template-rows: auto 1fr;
                        gap: 10px;
                }
                .photo-modal-toolbar {
                        display: flex;
                        align-items: center;
                        justify-content: space-between;
                        gap: 10px;
                        flex-wrap: wrap;
                }
                .photo-modal-title {
                        color: #cfc7bb;
                        font-size: 0.9rem;
                        min-width: 0;
                        white-space: nowrap;
                        overflow: hidden;
                        text-overflow: ellipsis;
                }
                .photo-modal-actions {
                        display: flex;
                        align-items: center;
                        gap: 8px;
                }
                .photo-modal-live-btn {
                        border: 1px solid #555;
                        border-radius: 8px;
                        background: #1f1f1f;
                        color: #fff;
                        padding: 6px 10px;
                        font-size: 0.85rem;
                        cursor: pointer;
                }
                .photo-modal-live-btn.on {
                        border-color: #ffb45c;
                        background: #2b1c08;
                        color: #ffcf90;
                }
                .photo-modal-full {
                        color: #ffb45c;
                        text-decoration: none;
                        font-size: 0.85rem;
                }
                .photo-modal-close {
                        border: 1px solid #555;
                        border-radius: 8px;
                        background: #1f1f1f;
                        color: #fff;
                        padding: 6px 10px;
                        font-size: 0.85rem;
                        cursor: pointer;
                }
                .photo-modal-img-wrap {
                        min-height: 0;
                        display: flex;
                        align-items: center;
                        justify-content: center;
                        background: #080808;
                        border: 1px solid #222;
                        border-radius: 10px;
                        overflow: auto;
                }
                .photo-modal-img {
                        max-width: 100%;
                        max-height: calc(94vh - 120px);
                        width: auto;
                        height: auto;
                        object-fit: contain;
                        display: block;
                        cursor: zoom-in;
                }
        </style>
</head>
<body>
        <div class=\"header\">
                <h1>MOCO JIB</h1>
                <div class=\"nav\">
                        <label for=\"navControlSelect\" class=\"nav-group\">Control
                                <select id=\"navControlSelect\">
                                        <option value=\"console\" selected>Console</option>
                                        <option value=\"settings\">Settings</option>
                                </select>
                        </label>
                        <label for=\"navModeSelect\">Mode
                                <select id=\"navModeSelect\">
                                        <option value=\"waypoint\" selected>Waypoint Mode</option>
                                        <option value=\"timelapse\">Timelapse</option>
                                        <option value=\"bounce\">Bounce</option>
                                        <option value=\"drone\">Drone</option>
                                </select>
                        </label>
                        <a id=\"navViewer\" href=\"http://127.0.0.1:18080/timelapse-directories\" class=\"active\">Timelapse Dirs</a>
                </div>
        </div>
        <div class=\"wrap\">
                <div class=\"section\">
                        <h2>Timelapse Directories</h2>
                        <div class=\"sub\">Remote root: /home/micah/projects/holy-grail/holy-grail-timelapse/timelapse</div>
                        <div class=\"toolbar\">
                                <span style="color:#aaa">Showing remote timelapse directories</span>
                                <button id=\"refreshBtn\" type=\"button\">Refresh</button>
                                <label style=\"color:#aaa;display:flex;align-items:center;gap:6px;margin-left:8px;\"><input id=\"keepUpToDateChk\" type=\"checkbox\"> Keep up to date</label>
                                <label style=\"color:#aaa;display:flex;align-items:center;gap:6px;\">Every <input id=\"keepUpToDateInterval\" type=\"number\" min=\"5\" max=\"3600\" value=\"30\" style=\"width:74px;\"> s</label>
                                <label style="color:#aaa;display:flex;align-items:center;gap:6px;">Monitor view
                                        <select id="monitorViewSelect" style="background:#111;border:1px solid #333;color:#fff;border-radius:6px;padding:6px 8px;">
                                                <option value="auto" selected>Auto</option>
                                                <option value="optimal">Optimal</option>
                                                <option value="all">All</option>
                                        </select>
                                </label>
                                <button id=\"stopSyncBtn\" type=\"button\" style=\"background:#333;color:#fff;border:1px solid #555;\" disabled>Stop</button>
                        </div>
                        <div id="viewHint" class="view-hint"></div>
                        <div id=\"status\" class=\"status\"></div>
                        <div class=\"stats\">
                                <div class=\"stat\"><div class=\"stat-label\">Remote Dirs</div><div id=\"remoteDirCount\" class=\"stat-value\">0</div></div>
                                <div class=\"stat\"><div class=\"stat-label\">Local Dirs</div><div id=\"localDirCount\" class=\"stat-value\">0</div></div>
                                <div class=\"stat\"><div class=\"stat-label\">Remote Items</div><div id=\"remoteItemCount\" class=\"stat-value\">0</div></div>
                                <div class=\"stat\"><div class=\"stat-label\">Local Items</div><div id=\"localItemCount\" class=\"stat-value\">0</div></div>
                        </div>
                        <div class=\"progress-wrap\">
                                <div class=\"progress-title\" id=\"syncStage\">Sync idle</div>
                                <div class=\"progress-track\"><div class=\"progress-fill\" id=\"syncFill\"></div></div>
                                <div class=\"progress-meta\">
                                        <span id=\"syncPercent\">0%</span>
                                        <span id=\"syncSpeed\">Speed: --</span>
                                        <span id=\"syncEta\">ETA: --</span>
                                        <span id=\"syncRemaining\">Remaining: --</span>
                                </div>
                        </div>
                        <table>
                                <thead>
                                        <tr>
                                                <th></th>
                                                <th>Preview</th>
                                                <th>Folder</th>
                                                <th>AI</th>
                                                <th>Remote (opt / all)</th>
                                                <th>Local (opt / all)</th>
                                                <th>Sync</th>
                                        </tr>
                                </thead>
                                <tbody id=\"rows\"></tbody>
                        </table>
                </div>
        </div>

        <div id="photoModal" class="photo-modal" aria-hidden="true">
                <div class="photo-modal-content" role="dialog" aria-modal="true" aria-label="Last Photo Preview">
                        <div class="photo-modal-toolbar">
                                <div id="photoModalTitle" class="photo-modal-title">Photo preview</div>
                                <div class="photo-modal-actions">
                                        <button id="photoModalLiveBtn" class="photo-modal-live-btn" type="button">Live: Off</button>
                                        <a id="photoModalFull" class="photo-modal-full" href="#" target="_blank" rel="noopener">Open full size</a>
                                        <button id="photoModalClose" class="photo-modal-close" type="button">X Close</button>
                                </div>
                        </div>
                        <div class="photo-modal-img-wrap">
                                <img id="photoModalImg" class="photo-modal-img" alt="Timelapse last photo enlarged">
                        </div>
                </div>
        </div>

        <script>
                const rows = document.getElementById('rows');
                const statusEl = document.getElementById('status');
                const remoteDirCountEl = document.getElementById('remoteDirCount');
                const localDirCountEl = document.getElementById('localDirCount');
                const remoteItemCountEl = document.getElementById('remoteItemCount');
                const localItemCountEl = document.getElementById('localItemCount');
                const syncFill = document.getElementById('syncFill');
                const syncPercent = document.getElementById('syncPercent');
                const syncSpeed = document.getElementById('syncSpeed');
                const syncEta = document.getElementById('syncEta');
                const syncRemaining = document.getElementById('syncRemaining');
                const syncStage = document.getElementById('syncStage');
                const navControlSelect = document.getElementById('navControlSelect');
                const navModeSelect = document.getElementById('navModeSelect');
                const navViewer = document.getElementById('navViewer');
                const keepUpToDateChk = document.getElementById('keepUpToDateChk');
                const keepUpToDateInterval = document.getElementById('keepUpToDateInterval');
                const monitorViewSelect = document.getElementById('monitorViewSelect');
                const viewHint = document.getElementById('viewHint');
                const stopSyncBtn = document.getElementById('stopSyncBtn');
                const photoModal = document.getElementById('photoModal');
                const photoModalImg = document.getElementById('photoModalImg');
                const photoModalTitle = document.getElementById('photoModalTitle');
                const photoModalFull = document.getElementById('photoModalFull');
                const photoModalClose = document.getElementById('photoModalClose');
                const photoModalLiveBtn = document.getElementById('photoModalLiveBtn');
                const photoModalContent = photoModal ? photoModal.querySelector('.photo-modal-content') : null;
                const relayOrigin = 'http://127.0.0.1:18080';
                let mainAppOrigin = relayOrigin;
                let expandedFolderNameState = null;
                const lastRemoteCountByFolder = new Map();
                const lastLocalCountByFolder = new Map();
                const liveDetectedAtByFolder = new Map();
                const LIVE_HOLD_MS = 300000;
                let lastDirectoriesSignature = '';
                let refreshInFlight = false;
                let refreshQueued = false;
                const HIDDEN_FOLDERS_STORAGE_KEY = 'timelapseHiddenFolders';
                const hiddenFolders = new Set();
                let modalLiveFolderName = '';
                let modalLastPhotoBaseUrl = '';
                let modalBaseTitle = '';
                let modalLiveKeepUpdated = false;
                let modalLiveTimer = null;

                function loadHiddenFolders() {
                        try {
                                const raw = window.localStorage.getItem(HIDDEN_FOLDERS_STORAGE_KEY);
                                if (!raw) return;
                                const list = JSON.parse(raw);
                                if (!Array.isArray(list)) return;
                                list.forEach(name => {
                                        const n = String(name || '').trim();
                                        if (n) hiddenFolders.add(n);
                                });
                        } catch (_) {
                                // ignore malformed local storage data
                        }
                }

                function saveHiddenFolders() {
                        try {
                                window.localStorage.setItem(HIDDEN_FOLDERS_STORAGE_KEY, JSON.stringify(Array.from(hiddenFolders)));
                        } catch (_) {
                                // ignore local storage write failures
                        }
                }

                function syncPhotoModalLiveButton() {
                        if (!photoModalLiveBtn) return;
                        photoModalLiveBtn.textContent = modalLiveKeepUpdated ? 'Live: On' : 'Live: Off';
                        photoModalLiveBtn.classList.toggle('on', !!modalLiveKeepUpdated);
                }

                function stopModalLiveTimer() {
                        if (modalLiveTimer) {
                                clearInterval(modalLiveTimer);
                                modalLiveTimer = null;
                        }
                }

                function applyModalLatestPhotoUrl(baseUrl, latestPhotoName = '') {
                        const srcBase = String(baseUrl || '');
                        if (!srcBase || !photoModalImg) return;
                        photoModalImg.src = withCacheBust(srcBase, Date.now());
                        modalLastPhotoBaseUrl = srcBase;
                        if (photoModalFull) {
                                photoModalFull.href = srcBase;
                        }
                        const nextName = String(latestPhotoName || '').trim();
                        if (photoModalTitle && modalLiveFolderName && nextName) {
                                photoModalTitle.textContent = modalLiveFolderName + ' - ' + nextName;
                        }
                }

                function findDirectoryByName(directories, folderName) {
                        const dirs = Array.isArray(directories) ? directories : [];
                        return dirs.find(d => String((d && d.name) || '') === String(folderName || '')) || null;
                }

                async function refreshPhotoModalLiveNow() {
                        if (!modalLiveKeepUpdated || !modalLiveFolderName) return;
                        if (!photoModal || !photoModal.classList.contains('open')) return;
                        try {
                                const r = await fetch('/api/timelapse/latest-photo?folder=' + encodeURIComponent(modalLiveFolderName));
                                const body = await r.json();
                                if (!r.ok || !body.ok) return;
                                const baseUrl = String(body.last_photo_url || '');
                                if (!baseUrl) return;
                                const latestPhotoName = String(body.last_photo_name || '');
                                applyModalLatestPhotoUrl(baseUrl, latestPhotoName);
                        } catch (_) {}
                }

                function setModalLiveEnabled(enabled) {
                        modalLiveKeepUpdated = !!enabled;
                        syncPhotoModalLiveButton();
                        stopModalLiveTimer();
                        if (!modalLiveKeepUpdated) {
                                return;
                        }
                        refreshPhotoModalLiveNow().catch(() => {});
                        modalLiveTimer = setInterval(() => {
                                refreshPhotoModalLiveNow().catch(() => {});
                        }, 1500);
                }

                function initMainNavLinks() {
                        mainAppOrigin = relayOrigin;
                        const dirsUrl = relayOrigin + '/timelapse-directories';
                        navViewer.href = dirsUrl;
                        navViewer.addEventListener('click', (event) => {
                                event.preventDefault();
                                window.location.href = dirsUrl;
                        });
                }

                function setStatus(text, kind = '') {
                        statusEl.textContent = text;
                        statusEl.className = 'status' + (kind ? ' ' + kind : '');
                }

                function applyLiveActivityFlags(directories) {
                        const now = Date.now();
                        return (directories || []).map(item => {
                                const out = { ...item };
                                if (out.source === 'remote') {
                                        const name = String(out.name || '');
                                        const count = Number(out.remote_selected_count || 0);
                                        const localCount = Number(out.local_selected_count || 0);
                                        if (lastRemoteCountByFolder.has(name)) {
                                                const prev = Number(lastRemoteCountByFolder.get(name) || 0);
                                                if (count > prev) {
                                                        liveDetectedAtByFolder.set(name, now);
                                                }
                                        }
                                        if (lastLocalCountByFolder.has(name)) {
                                                const prevLocal = Number(lastLocalCountByFolder.get(name) || 0);
                                                if (localCount > prevLocal) {
                                                        liveDetectedAtByFolder.set(name, now);
                                                }
                                        }
                                        lastRemoteCountByFolder.set(name, count);
                                        lastLocalCountByFolder.set(name, localCount);

                                        const remoteLatestMs = Number(out.remote_latest_mtime || 0) * 1000;
                                        const recentRemoteWrite = remoteLatestMs > 0 && (now - remoteLatestMs) <= LIVE_HOLD_MS;
                                        const liveAt = Number(liveDetectedAtByFolder.get(name) || 0);
                                        const recentDelta = liveAt > 0 && (now - liveAt) <= LIVE_HOLD_MS;
                                        out.is_live = recentRemoteWrite || recentDelta;
                                } else {
                                        out.is_live = false;
                                }
                                return out;
                        });
                }

                function buildDirectoriesSignature(directories, syncState) {
                        const normalized = (directories || []).map(item => ({
                                name: item.name,
                                source: item.source,
                                preview_url: item.preview_url,
                                last_photo_url: item.last_photo_url,
                                latest_export_available: !!item.latest_export_available,
                                latest_export_url: item.latest_export_url,
                                remote_selected_count: item.remote_selected_count,
                                local_selected_count: item.local_selected_count,
                                remote_optimal_count: item.remote_optimal_count,
                                remote_all_count: item.remote_all_count,
                                ai_score: item.ai_score,
                                frame_meta_current_name: item.frame_meta && item.frame_meta.current_name,
                                frame_meta_iso: item.frame_meta && item.frame_meta.current && item.frame_meta.current.capture && item.frame_meta.current.capture.iso,
                                frame_meta_shutter: item.frame_meta && item.frame_meta.current && item.frame_meta.current.capture && item.frame_meta.current.capture.shutter,
                                frame_meta_fstop: item.frame_meta && item.frame_meta.current && item.frame_meta.current.capture && item.frame_meta.current.capture.fstop,
                                is_live: !!item.is_live,
                        }));
                        const sync = syncState || {};
                        return JSON.stringify({
                                directories: normalized,
                                sync_running: !!sync.running,
                                sync_folder: sync.folder || '',
                                sync_watch: !!sync.watch,
                        });
                }

                function withCacheBust(url, token) {
                        const src = String(url || '');
                        if (!src) return src;
                        const sep = src.includes('?') ? '&' : '?';
                        return src + sep + 'live_ts=' + encodeURIComponent(String(token || Date.now()));
                }

                function asNum(v) {
                        if (v === null || v === undefined) return null;
                        const n = Number(v);
                        return Number.isFinite(n) ? n : null;
                }

                function fmtNum(v, digits) {
                        const n = asNum(v);
                        if (n === null) return '-';
                        const d = Number.isFinite(Number(digits)) ? Number(digits) : 2;
                        return n.toFixed(Math.max(0, Math.min(6, d)));
                }

                function renderDelta(v, digits, suffix = '', eps = 0.0001) {
                        const n = asNum(v);
                        if (n === null) return '';
                        let cls = 'meta-delta-zero';
                        if (Math.abs(n) > Math.abs(Number(eps) || 0.0001)) {
                                cls = n > 0 ? 'meta-delta-pos' : 'meta-delta-neg';
                        }
                        const sign = n > 0 ? '+' : (n < 0 ? '' : '');
                        return '<span class="d ' + cls + '">' + sign + fmtNum(n, digits) + suffix + '</span>';
                }

                function renderMiniMetaRow(label, valueText, deltaHtml) {
                        return '<div class="meta-mini-row"><span class="k">' + label + '</span><span class="v">' + valueText + '</span>' + (deltaHtml || '<span class="d"></span>') + '</div>';
                }

                function renderFrameMetaMini(frameMeta, context) {
                        if (!frameMeta || !frameMeta.available || !frameMeta.current) {
                                return '<div class="meta-mini"><div class="meta-mini-title">Frame Metadata</div><div class="tiny">Unavailable</div></div>';
                        }

                        const ctx = context || {};
                        const cur = frameMeta.current || {};
                        const cap = cur.capture || {};
                        const met = cur.metrics || {};
                        const d = frameMeta.delta || {};

                        const ctxRows = [];
                        const dirName = String(ctx.dirName || '').trim();
                        const fileName = String(ctx.fileName || '').trim();
                        if (dirName) {
                                ctxRows.push(renderMiniMetaRow('Directory', '<a class="folder-link" href="/timelapse-preview?dir=' + encodeURIComponent(dirName) + '">' + dirName + '</a>', ''));
                        }
                        if (fileName) {
                                ctxRows.push(renderMiniMetaRow('File', fileName, ''));
                        }

                        const rowsHtml = [
                                ...ctxRows,
                                renderMiniMetaRow('ISO', (cap.iso ?? '-'), renderDelta(d.iso, 0, '', 0.01)),
                                renderMiniMetaRow('Shutter', (cap.shutter || '-'), renderDelta(d.shutter_ev, 2, ' EV', 0.0001)),
                                renderMiniMetaRow('F-stop', (cap.fstop !== null && cap.fstop !== undefined) ? fmtNum(cap.fstop, 1) : '-', renderDelta(d.fstop_ev, 2, ' EV', 0.0001)),
                                renderMiniMetaRow('ΔEV', fmtNum(met.delta_ev, 2), renderDelta(d.delta_ev, 2, '', 0.0001)),
                                renderMiniMetaRow('Luma', fmtNum(met.median_luma, 1), renderDelta(d.median_luma, 1, '', 0.05)),
                                renderMiniMetaRow('Hi%', fmtNum(met.highlight_clip_pct, 2) + '%', renderDelta(d.highlight_clip_pct, 2, '%', 0.01)),
                                renderMiniMetaRow('Shadow%', fmtNum(met.shadow_clip_pct, 2) + '%', renderDelta(d.shadow_clip_pct, 2, '%', 0.01)),
                        ].join('');

                        return '<div class="meta-mini"><div class="meta-mini-title">Frame Metadata</div>' + rowsHtml + '</div>';
                }

                function renderRows(directories, syncState) {
                        const sync = syncState || {};
                        const monitorPref = String((monitorViewSelect && monitorViewSelect.value) || 'auto').toLowerCase();
                        const visibleDirectories = (directories || []).filter(item => {
                                const name = String((item && item.name) || '').trim();
                                return !hiddenFolders.has(name);
                        });

                        if (!visibleDirectories.length) {
                                rows.innerHTML = '<tr><td class="dim" colspan="7">No timelapse_[timestamp] directories found.</td></tr>';
                                return;
                        }
                        const ordered = [...visibleDirectories].sort((a, b) => {
                                const aLive = a && a.is_live ? 1 : 0;
                                const bLive = b && b.is_live ? 1 : 0;
                                if (aLive !== bLive) {
                                        return bLive - aLive;
                                }
                                const aSource = (a && a.source === 'remote') ? 0 : 1;
                                const bSource = (b && b.source === 'remote') ? 0 : 1;
                                if (aSource !== bSource) {
                                        return aSource - bSource;
                                }
                                return String((b && b.name) || '').localeCompare(String((a && a.name) || ''));
                        });

                        if (viewHint) {
                                const switched = ordered.filter(item => {
                                        if (!item || !item.is_live) return false;
                                        const remoteDelta = Math.max(0, Number(item.remote_all_count || 0) - Number(item.remote_optimal_count || 0));
                                        const localDelta = Math.max(0, Number(item.local_all_count || 0) - Number(item.local_optimal_count || 0));
                                        return Math.max(remoteDelta, localDelta) > 0;
                                });
                                if (switched.length) {
                                        const first = switched[0];
                                        const remoteDelta = Math.max(0, Number(first.remote_all_count || 0) - Number(first.remote_optimal_count || 0));
                                        const localDelta = Math.max(0, Number(first.local_all_count || 0) - Number(first.local_optimal_count || 0));
                                        const delta = Math.max(remoteDelta, localDelta);
                                        const forcedLabel = monitorPref === 'auto'
                                                ? 'Auto now tracks all for this row.'
                                                : ('Monitor view is forced to ' + monitorPref + '.');
                                        viewHint.textContent = 'Optimal appears stalled for ' + first.name + '; all has +' + delta + ' newer frame(s). ' + forcedLabel;
                                } else {
                                        viewHint.textContent = '';
                                }
                        }

                        rows.innerHTML = ordered.map(item => {
                                const isRemote = item.source === 'remote';
                                const syncRunningThis = !!sync.running && sync.folder === item.name;
                                const localOptimalCount = Number(item.local_optimal_count ?? 0);
                                const localAllCount = Number(item.local_all_count ?? item.local_selected_count ?? 0);
                                const hasRemoteOptimal = Number(item.remote_optimal_count || 0) > 0;
                                const hasRemoteAll = Number(item.remote_all_count || 0) > 0;
                                const remoteOptimalCount = Number(item.remote_optimal_count || 0);
                                const remoteAllCount = Number(item.remote_all_count || 0);
                                const remoteDelta = Math.max(0, remoteAllCount - remoteOptimalCount);
                                const localDelta = Math.max(0, localAllCount - localOptimalCount);
                                const liveDelta = Math.max(remoteDelta, localDelta);
                                const liveOptimalStalled = !!item.is_live && liveDelta > 0;
                                const remoteDeletedOnOrin = isRemote && !item.is_live && remoteOptimalCount === 0 && remoteAllCount === 0 && (localOptimalCount > 0 || localAllCount > 0);
                                const disabled = (isRemote && (!!sync.running || remoteDeletedOnOrin)) ? 'disabled' : '';
                                const autoView = (hasRemoteAll && remoteAllCount > remoteOptimalCount) ? 'all' : ((!hasRemoteOptimal && hasRemoteAll) ? 'all' : 'optimal');
                                const syncView = (monitorPref === 'all' || monitorPref === 'optimal') ? monitorPref : autoView;
                                const syncTitle = remoteDeletedOnOrin
                                        ? 'Remote directory missing on ORIN (0/0)'
                                        : (syncRunningThis
                                        ? (sync.watch ? 'Live syncing...' : 'Syncing...')
                                        : (syncView === 'all' ? 'Sync all frames' : 'Sync optimal frames'));
                                const deleteDisabled = syncRunningThis ? 'disabled' : '';
                                const sourcePill = isRemote ? 'remote origin / local mirror' : 'local only';
                                const staleHint = liveOptimalStalled
                                        ? `<div class="tiny stall-hint">Optimal stalled; all is live (+${liveDelta})</div>`
                                        : '';
                                const livePill = item.is_live ? '<span class="pill pill-live">LIVE</span>' : '';
                                const remoteMissingPill = remoteDeletedOnOrin ? '<span class="pill pill-remote-missing">DELETED ON ORIN</span>' : '';
                                const fallbackPill = item.last_photo_is_fallback ? '<span class="pill pill-fallback">FALLBACK</span>' : '';
                                const remoteCellHint = remoteDeletedOnOrin
                                        ? '<div class="tiny remote-missing-hint">Remote shows 0 / 0. Directory was deleted on ORIN; local mirror remains.</div>'
                                        : staleHint;
                                const scoreText = (item.ai_score === null || item.ai_score === undefined) ? 'n/a' : String(item.ai_score);
                                const lastPhotoText = item.last_photo_name || item.preview_name || 'n/a';
                                const shouldLiveRefreshThumb = !!(sync.watch && syncRunningThis);
                                const previewSrc = shouldLiveRefreshThumb ? withCacheBust(item.preview_url, Date.now()) : item.preview_url;
                                const hasExport = !!item.latest_export_available;
                                const exportVideoSrc = String(item.latest_export_url || '').trim();
                                const showVideoThumb = !syncRunningThis && !item.is_live && hasExport && !!exportVideoSrc;
                                const showExportNeeded = !syncRunningThis && !item.is_live && !hasExport;
                                const showReexport = !syncRunningThis && !item.is_live && hasExport;
                                const exportNeededHtml = showExportNeeded
                                        ? `<div class="preview-actions"><button class="export-needed-btn" type="button" data-export-needed="${item.name}" data-view="${syncView}">Export Needed</button></div>`
                                        : '';
                                const reexportHtml = showReexport
                                        ? `<div class="preview-actions"><button class="reexport-btn" type="button" data-reexport="${item.name}" data-view="${syncView}">Re-export Thumb</button></div>`
                                        : '';
                                let previewHtml = '<div class="preview-empty">No preview</div>' + exportNeededHtml;
                                if (showVideoThumb) {
                                        const videoPoster = item.preview_url ? ` poster="${previewSrc}"` : '';
                                        previewHtml = `<video class="preview preview-video" src="${exportVideoSrc}"${videoPoster} muted playsinline preload="metadata" data-hover-play-video="1" data-loop-id="${item.name}" data-loop-view="${syncView}" data-open-preview-dir="${item.name}" data-open-preview-view="${syncView}"></video>${reexportHtml}`;
                                } else if (item.preview_url) {
                                        previewHtml = `<img class="preview" src="${previewSrc}" alt="${item.name} preview" data-open-preview-dir="${item.name}" data-open-preview-view="${syncView}">${exportNeededHtml}${reexportHtml}`;
                                }
                                const score = item.ai_scores || {};
                                const frameMetaMini = renderFrameMetaMini(item.frame_meta || null, {
                                        dirName: item.name,
                                        fileName: lastPhotoText,
                                });
                                const aiSummary = `
                                        <div class="pill">Overall ${score.overall_score ?? scoreText}</div>
                                        <div class="tiny">Recommend: ${item.ai_recommendation || 'n/a'}</div>`;
                                const aiDetails = `
                                        <div class="ai-card">
                                                <div class="ai-top"><span class="ai-label-tip" data-tip="Combined quality score from trend, smoothness, transition risk, horizon level, and obstruction checks.">Overall</span><strong>${score.overall_score ?? 'n/a'}</strong></div>
                                                <div class="ai-grid">
                                                        <span><b class="ai-label-tip" data-tip="How well exposure follows a stable direction without over/under corrections.">Trend</b><em>${score.trend_score ?? 'n/a'}</em></span>
                                                        <span><b class="ai-label-tip" data-tip="Frame-to-frame smoothness score; higher means fewer visible jumps.">Smooth</b><em>${score.smooth_score ?? 'n/a'}</em></span>
                                                        <span><b class="ai-label-tip" data-tip="Risk score for abrupt EV/luminance jumps between nearby frames.">Transition</b><em>${score.transition_score ?? 'n/a'}</em></span>
                                                        <span><b class="ai-label-tip" data-tip="Estimated camera level consistency. Higher means less tilt/drift.">Horizon</b><em>${score.horizon_score ?? 'n/a'}</em></span>
                                                        <span><b class="ai-label-tip" data-tip="Visibility/clarity signal. Lower values can indicate occlusion or lens blockage.">Obstruction</b><em>${score.obstruction_score ?? 'n/a'}</em></span>
                                                </div>
                                                <div class="ai-recommendation ai-label-tip" data-tip="Suggested next action inferred from current exposure, trend, and transition behavior." style="margin-top:8px;">Recommend: ${item.ai_recommendation || 'n/a'}</div>
                                        </div>`;
                                const drawerPhotoUrl = item.last_photo_url || item.preview_url;
                                const safePhotoTitle = `${item.name} - ${lastPhotoText}`;
                                const liveRefreshForRow = !!(sync.watch && syncRunningThis);
                                const detailPhotoSrc = liveRefreshForRow ? withCacheBust(drawerPhotoUrl, Date.now()) : drawerPhotoUrl;
                                const detailImage = drawerPhotoUrl
                                        ? `<button class="detail-photo-open" type="button" data-photo-url="${drawerPhotoUrl}" data-photo-title="${safePhotoTitle}" data-photo-folder="${item.name}" data-live-refresh="${liveRefreshForRow ? '1' : '0'}"><img class="detail-image" src="${detailPhotoSrc}" alt="${item.name} expanded last photo"></button><br><a class="detail-full-link" href="${drawerPhotoUrl}" target="_blank" rel="noopener">Open full size</a>`
                                        : `<div class="preview-empty">No photo</div>`;
                                const detailContent = `
                                        <div class="detail-content">
                                                <div class="detail-photo-block">
                                                        ${detailImage}
                                                </div>
                                                <div class="detail-meta">
                                                        <div class="detail-meta-top">${frameMetaMini}${aiDetails}</div>
                                                </div>
                                        </div>`;
                                const rowClass = liveOptimalStalled ? 'live-optimal-stalled' : '';
                                return `
                                        <tr class="${rowClass}">
                                                <td class=\"expand-cell\"><button class=\"expander-btn\" data-expand=\"${item.name}\" aria-expanded=\"false\" title=\"Expand row\">▸</button></td>
                                                <td class="preview-cell">${previewHtml}</td>
                                                <td class=\"dim\">${(livePill || remoteMissingPill || fallbackPill) ? (livePill + remoteMissingPill + fallbackPill) : '<span class="tiny">Expand row for directory details</span>'}</td>
                                                <td>${aiSummary}</td>
                                                <td>${remoteDeletedOnOrin ? '<span class="count-muted">0 / 0</span>' : (remoteOptimalCount + ' / ' + remoteAllCount)}${remoteCellHint}</td>
                                                <td>${localOptimalCount} / ${localAllCount}</td>
                                                <td>
                                                        <div class=\"row-actions\">${isRemote ? `<button class=\"sync-btn sync-action-btn\" data-folder=\"${item.name}\" data-source=\"${item.source}\" data-view=\"${syncView}\" title=\"${syncTitle}\" aria-label=\"${syncTitle}\" ${disabled}>&#x21bb;</button>` : ''}<button class=\"sync-btn delete-dir-btn\" data-folder=\"${item.name}\" data-source=\"${item.source}\" title=\"Delete local directory\" aria-label=\"Delete local directory ${item.name}\" ${deleteDisabled}>🗑</button></div>
                                                </td>
                                        </tr>
                                        <tr class=\"detail-row\" data-detail=\"${item.name}\"><td colspan=\"7\" class=\"detail-panel\">${detailContent}</td></tr>
                                `;
                        }).join('');

                        document.querySelectorAll('.sync-action-btn').forEach(btn => {
                                btn.addEventListener('click', async () => {
                                        const folder = btn.getAttribute('data-folder');
                                        const source = btn.getAttribute('data-source') || '';
                                        const syncView = (btn.getAttribute('data-view') || 'optimal').toLowerCase() === 'all' ? 'all' : 'optimal';
                                        if (!folder) {
                                                setStatus('Sync failed: missing folder name', 'err');
                                                return;
                                        }
                                        if (source === 'local') return;
                                        btn.disabled = true;
                                        const keepUp = !!keepUpToDateChk.checked;
                                        let intervalSeconds = parseInt(String(keepUpToDateInterval.value || '30'), 10);
                                        if (!Number.isFinite(intervalSeconds)) intervalSeconds = 30;
                                        intervalSeconds = Math.max(5, Math.min(3600, intervalSeconds));
                                        keepUpToDateInterval.value = String(intervalSeconds);
                                        setStatus('Starting ' + (keepUp ? 'live ' : '') + 'sync (' + syncView + ') for ' + folder + '...', '');
                                        try {
                                                const r = await fetch('/api/timelapse-sync', {
                                                        method: 'POST',
                                                        headers: {'Content-Type': 'application/json'},
                                                        body: JSON.stringify({
                                                                folder,
                                                                view: syncView,
                                                                keep_up_to_date: keepUp,
                                                                interval_seconds: intervalSeconds,
                                                        })
                                                });
                                                const body = await r.json();
                                                if (!r.ok || !body.ok) {
                                                        throw new Error(body.error || 'sync start failed');
                                                }
                                                if (body.watch) {
                                                        setStatus('Live sync started for ' + folder + ' (every ' + (body.interval_seconds || intervalSeconds) + 's)', 'ok');
                                                } else {
                                                        setStatus('Sync started for ' + folder, 'ok');
                                                }
                                                refresh(true, true).catch(() => {});
                                        } catch (err) {
                                                setStatus('Sync failed: ' + err.message, 'err');
                                        } finally {
                                                btn.disabled = false;
                                        }
                                });
                        });

                        document.querySelectorAll('.expander-btn').forEach(btn => {
                                btn.addEventListener('click', () => {
                                        const id = btn.getAttribute('data-expand');
                                        const detail = document.querySelector(`.detail-row[data-detail="${id}"]`);
                                        if (!detail) {
                                                return;
                                        }
                                        const isOpen = detail.classList.contains('open');
                                        document.querySelectorAll('.detail-row.open').forEach(r => r.classList.remove('open'));
                                        document.querySelectorAll('.expander-btn[aria-expanded="true"]').forEach(b => {
                                                b.setAttribute('aria-expanded', 'false');
                                                b.textContent = '▸';
                                        });
                                        if (!isOpen) {
                                                detail.classList.add('open');
                                                btn.setAttribute('aria-expanded', 'true');
                                                btn.textContent = '▾';
                                                expandedFolderNameState = id;
                                        } else {
                                                expandedFolderNameState = null;
                                        }
                                });
                        });

                        document.querySelectorAll('.delete-dir-btn').forEach(btn => {
                                btn.addEventListener('click', async () => {
                                        const folder = String(btn.getAttribute('data-folder') || '').trim();
                                        const source = String(btn.getAttribute('data-source') || '').trim();
                                        if (!folder) {
                                                setStatus('Delete failed: missing folder name', 'err');
                                                return;
                                        }
                                        const typed = window.prompt('Type delete to permanently remove local directory for ' + folder + ':', '');
                                        if (typed === null) {
                                                setStatus('Delete cancelled', '');
                                                return;
                                        }
                                        if (String(typed).trim().toLowerCase() !== 'delete') {
                                                setStatus("Delete cancelled: confirmation text must be 'delete'", 'err');
                                                return;
                                        }

                                        btn.disabled = true;
                                        const previousLabel = btn.textContent;
                                        btn.textContent = 'Deleting...';
                                        try {
                                                const r = await fetch('/api/timelapse/delete-dir', {
                                                        method: 'POST',
                                                        headers: {'Content-Type': 'application/json'},
                                                        body: JSON.stringify({
                                                                dir: folder,
                                                                source,
                                                                confirm_text: 'delete',
                                                        }),
                                                });
                                                const body = await r.json();
                                                if (!r.ok || !body.ok) {
                                                        throw new Error(body.error || 'delete failed');
                                                }
                                                hiddenFolders.add(folder);
                                                saveHiddenFolders();
                                                setStatus('Deleted local directory for ' + folder, 'ok');
                                                refresh(false, true).catch(() => {});
                                        } catch (err) {
                                                setStatus('Delete failed: ' + err.message, 'err');
                                        } finally {
                                                btn.textContent = previousLabel;
                                                btn.disabled = false;
                                        }
                                });
                        });

                        document.querySelectorAll('.export-needed-btn').forEach(btn => {
                                btn.addEventListener('click', async () => {
                                        const folder = String(btn.getAttribute('data-export-needed') || '').trim();
                                        const view = String(btn.getAttribute('data-view') || 'optimal').trim().toLowerCase() === 'all' ? 'all' : 'optimal';
                                        if (!folder) return;
                                        const originalLabel = btn.textContent;
                                        btn.disabled = true;
                                        btn.textContent = 'Exporting...';
                                        setStatus('Starting export for ' + folder + '...', '');
                                        try {
                                                const r = await fetch('/api/timelapse/export-video', {
                                                        method: 'POST',
                                                        headers: { 'Content-Type': 'application/json' },
                                                        body: JSON.stringify({
                                                                dir: folder,
                                                                view,
                                                                fps: 8,
                                                                resolution: '540p',
                                                                quality: 'low',
                                                                format: 'mp4',
                                                                max_frames: null,
                                                                hide_banner: true,
                                                                loglevel: 'error',
                                                                concat_safe: 0,
                                                                codec: 'libx264',
                                                                preset: 'veryfast',
                                                                crf: 30,
                                                                pix_fmt: 'yuv420p',
                                                                faststart: true,
                                                                name_tag: 'thumb',
                                                        }),
                                                });
                                                const body = await r.json();
                                                if (!r.ok || !body.ok) {
                                                        throw new Error(body.error || 'export failed');
                                                }
                                                setStatus('Export ready for ' + folder + ': ' + (body.file_name || 'video.mp4'), 'ok');
                                                refresh(false, true).catch(() => {});
                                        } catch (err) {
                                                setStatus('Export failed for ' + folder + ': ' + (err.message || 'unknown error'), 'err');
                                        } finally {
                                                btn.textContent = originalLabel;
                                                btn.disabled = false;
                                        }
                                });
                        });

                        document.querySelectorAll('.reexport-btn').forEach(btn => {
                                btn.addEventListener('click', async () => {
                                        const folder = String(btn.getAttribute('data-reexport') || '').trim();
                                        const view = String(btn.getAttribute('data-view') || 'optimal').trim().toLowerCase() === 'all' ? 'all' : 'optimal';
                                        if (!folder) return;
                                        const originalLabel = btn.textContent;
                                        btn.disabled = true;
                                        btn.textContent = 'Re-exporting...';
                                        setStatus('Starting re-export for ' + folder + '...', '');
                                        let loopStartPct = null;
                                        let loopEndPct = null;
                                        try {
                                                const lsKey = 'timelapse-loop-points:' + folder;
                                                const lsRaw = window.localStorage.getItem(lsKey) || '';
                                                if (lsRaw) {
                                                        const lsParsed = JSON.parse(lsRaw);
                                                        const viewEntry = (lsParsed && lsParsed.views && typeof lsParsed.views === 'object' && lsParsed.views[view] && typeof lsParsed.views[view] === 'object')
                                                                ? lsParsed.views[view]
                                                                : null;
                                                        const inPct = (viewEntry && viewEntry.in_pct !== undefined) ? Number(viewEntry.in_pct) : Number(lsParsed && lsParsed.in_pct);
                                                        const outPctRaw = (viewEntry && viewEntry.out_pct !== undefined) ? viewEntry.out_pct : (lsParsed && lsParsed.out_pct);
                                                        const outPct = (outPctRaw === null || outPctRaw === undefined || outPctRaw === '') ? null : Number(outPctRaw);
                                                        if (Number.isFinite(inPct) && inPct > 0) loopStartPct = inPct;
                                                        if (outPct !== null && Number.isFinite(outPct) && outPct < 0.9999) loopEndPct = outPct;
                                                }
                                        } catch (_) {}
                                        try {
                                                const r = await fetch('/api/timelapse/export-video', {
                                                        method: 'POST',
                                                        headers: { 'Content-Type': 'application/json' },
                                                        body: JSON.stringify({
                                                                dir: folder,
                                                                view,
                                                                fps: 8,
                                                                resolution: '540p',
                                                                quality: 'low',
                                                                format: 'mp4',
                                                                max_frames: null,
                                                                hide_banner: true,
                                                                loglevel: 'error',
                                                                concat_safe: 0,
                                                                codec: 'libx264',
                                                                preset: 'veryfast',
                                                                crf: 30,
                                                                pix_fmt: 'yuv420p',
                                                                faststart: true,
                                                                name_tag: 'thumb',
                                                                start_pct: loopStartPct,
                                                                end_pct: loopEndPct,
                                                        }),
                                                });
                                                const body = await r.json();
                                                if (!r.ok || !body.ok) {
                                                        throw new Error(body.error || 're-export failed');
                                                }
                                                setStatus('Re-export ready for ' + folder + ': ' + (body.file_name || 'video.mp4'), 'ok');
                                                refresh(false, true).catch(() => {});
                                        } catch (err) {
                                                setStatus('Re-export failed for ' + folder + ': ' + (err.message || 'unknown error'), 'err');
                                        } finally {
                                                btn.textContent = originalLabel;
                                                btn.disabled = false;
                                        }
                                });
                        });

                        bindPreviewScrub();
                        bindPreviewOpeners();
                        bindHoverVideoPreviews();
                        bindDetailPhotoOpeners();
                }

                function bindPreviewOpeners() {
                        const media = document.querySelectorAll('[data-open-preview-dir]');
                        media.forEach(el => {
                                if (el.dataset.openPreviewBound === '1') return;
                                el.dataset.openPreviewBound = '1';
                                el.style.cursor = 'pointer';
                                el.title = 'Open timelapse preview';
                                el.addEventListener('click', (event) => {
                                        const dir = String(el.getAttribute('data-open-preview-dir') || '').trim();
                                        const view = String(el.getAttribute('data-open-preview-view') || 'optimal').trim().toLowerCase() === 'all' ? 'all' : 'optimal';
                                        if (!dir) return;
                                        const url = '/timelapse-preview?dir=' + encodeURIComponent(dir) + '&view=' + encodeURIComponent(view);
                                        if (event.metaKey || event.ctrlKey || event.button === 1) {
                                                window.open(url, '_blank', 'noopener');
                                                return;
                                        }
                                        window.location.href = url;
                                });
                        });
                }

                function bindHoverVideoPreviews() {
                        const videos = document.querySelectorAll('video[data-hover-play-video="1"]');
                        let returnAutoplayDir = '';
                        try {
                                const raw = window.localStorage.getItem('timelapse-return-autoplay') || '';
                                if (raw) {
                                        const parsed = JSON.parse(raw);
                                        const dir = String(parsed && parsed.dir || '').trim();
                                        const at = Number(parsed && parsed.at || 0);
                                        if (dir && Number.isFinite(at) && (Date.now() - at) <= 120000) {
                                                returnAutoplayDir = dir;
                                        } else {
                                                window.localStorage.removeItem('timelapse-return-autoplay');
                                        }
                                }
                        } catch (_) {
                        }

                        const consumeReturnAutoplay = (loopId) => {
                                const normalized = String(loopId || '').trim();
                                if (!returnAutoplayDir || !normalized) return false;
                                if (normalized !== returnAutoplayDir) return false;
                                returnAutoplayDir = '';
                                try {
                                        window.localStorage.removeItem('timelapse-return-autoplay');
                                } catch (_) {
                                }
                                return true;
                        };

                        videos.forEach(video => {
                                if (video.dataset.hoverBound === '1') {
                                        return;
                                }
                                video.dataset.hoverBound = '1';
                                video.muted = true;
                                video.defaultMuted = true;
                                video.playsInline = true;
                                video.loop = false;
                                const loopId = String(video.getAttribute('data-loop-id') || '').trim();
                                const loopView = String(video.getAttribute('data-loop-view') || 'optimal').trim().toLowerCase() === 'all' ? 'all' : 'optimal';
                                const loopTrack = loopId ? document.querySelector(`[data-loop-track="${loopId}"]`) : null;
                                const loopRangeFill = loopId ? document.querySelector(`[data-loop-range-fill="${loopId}"]`) : null;
                                const loopInMarker = loopId ? document.querySelector(`[data-loop-marker-in="${loopId}"]`) : null;
                                const loopOutMarker = loopId ? document.querySelector(`[data-loop-marker-out="${loopId}"]`) : null;
                                const loopInReadout = loopId ? document.querySelector(`[data-loop-in-readout="${loopId}"]`) : null;
                                const loopOutReadout = loopId ? document.querySelector(`[data-loop-out-readout="${loopId}"]`) : null;
                                const markButtons = loopId ? document.querySelectorAll(`[data-loop-mark-target="${loopId}"]`) : [];
                                const playbar = loopId ? document.querySelector(`[data-playbar="${loopId}"]`) : null;
                                const playbarFill = loopId ? document.querySelector(`[data-playbar-fill="${loopId}"]`) : null;
                                const modeButtons = loopId ? document.querySelectorAll(`[data-loop-mode-target="${loopId}"]`) : [];
                                let hovering = false;
                                let activeMarker = 'in';

                                const clamp = (value, min, max) => {
                                        if (!Number.isFinite(value)) return min;
                                        return Math.min(max, Math.max(min, value));
                                };

                                const formatLoopTime = (seconds) => {
                                        const val = Number(seconds);
                                        if (!Number.isFinite(val) || val < 0) {
                                                return '';
                                        }
                                        const whole = Math.floor(val);
                                        const h = Math.floor(whole / 3600);
                                        const m = Math.floor((whole % 3600) / 60);
                                        const s = whole % 60;
                                        if (h > 0) {
                                                return String(h) + ':' + String(m).padStart(2, '0') + ':' + String(s).padStart(2, '0');
                                        }
                                        return String(m) + ':' + String(s).padStart(2, '0');
                                };

                                const setActiveMode = (mode) => {
                                        activeMarker = (mode === 'out') ? 'out' : 'in';
                                        Array.from(modeButtons || []).forEach(btn => {
                                                const btnMode = String(btn.getAttribute('data-loop-mode') || 'in').toLowerCase();
                                                btn.classList.toggle('is-active', btnMode === activeMarker);
                                        });
                                };

                                const parseTypedLoopTime = (rawValue) => {
                                        const raw = String(rawValue || '').trim().toLowerCase();
                                        if (!raw) return null;
                                        if (raw === 'full' || raw === 'end' || raw === 'max') return null;
                                        if (/^\\d+(?:\\.\\d+)?$/.test(raw)) {
                                                const sec = Number(raw);
                                                return Number.isFinite(sec) ? sec : null;
                                        }
                                        const parts = raw.split(':').map(v => String(v).trim());
                                        if (!parts.length || parts.length > 3) return null;
                                        if (!parts.every(v => /^\\d+(?:\\.\\d+)?$/.test(v))) return null;
                                        let total = 0;
                                        for (let i = 0; i < parts.length; i += 1) {
                                                const n = Number(parts[i]);
                                                if (!Number.isFinite(n)) return null;
                                                total = total * 60 + n;
                                        }
                                        return total;
                                };

                                const getStoredLoopPoints = () => {
                                        const rawStart = Number(video.dataset.loopStartSec || 0);
                                        const rawEndText = String(video.dataset.loopEndSec || '').trim();
                                        const rawEnd = rawEndText ? Number(rawEndText) : null;
                                        const start = Number.isFinite(rawStart) ? Math.max(0, rawStart) : 0;
                                        const end = Number.isFinite(rawEnd) ? Math.max(0, rawEnd) : null;
                                        return { start, end };
                                };

                                const setStoredLoopPoints = (startSec, endSec) => {
                                        const start = Number.isFinite(startSec) ? Math.max(0, Number(startSec)) : 0;
                                        const end = Number.isFinite(endSec) ? Math.max(0, Number(endSec)) : null;
                                        video.dataset.loopStartSec = String(start);
                                        if (end === null) {
                                                delete video.dataset.loopEndSec;
                                        } else {
                                                video.dataset.loopEndSec = String(end);
                                        }
                                };

                                const loadSavedLoopPoints = () => {
                                        if (!loopId || !window.localStorage) {
                                                return { start: 0, end: null };
                                        }
                                        try {
                                                const raw = window.localStorage.getItem('timelapse-loop-points:' + loopId);
                                                if (!raw) return { start: 0, end: null };
                                                const parsed = JSON.parse(raw);
                                                // Prefer current loopView entry; fall back to any saved view entry
                                                const views = (parsed && parsed.views && typeof parsed.views === 'object') ? parsed.views : {};
                                                let viewEntry = (views[loopView] && typeof views[loopView] === 'object') ? views[loopView] : null;
                                                if (!viewEntry) {
                                                        const otherView = loopView === 'all' ? 'optimal' : 'all';
                                                        if (views[otherView] && typeof views[otherView] === 'object') {
                                                                viewEntry = views[otherView];
                                                        }
                                                }
                                                const duration = (Number.isFinite(video.duration) && video.duration > 0) ? Number(video.duration) : null;
                                                const startRaw = Number((viewEntry && viewEntry.in !== undefined) ? viewEntry.in : (parsed && parsed.in));
                                                const endRaw = (viewEntry && viewEntry.out !== undefined) ? viewEntry.out : (parsed && parsed.out);
                                                const endSecRaw = (endRaw === null || endRaw === undefined || endRaw === '') ? null : Number(endRaw);
                                                const startPctRaw = Number((viewEntry && viewEntry.in_pct !== undefined) ? viewEntry.in_pct : (parsed && parsed.in_pct));
                                                const endPctVal = (viewEntry && viewEntry.out_pct !== undefined) ? viewEntry.out_pct : (parsed && parsed.out_pct);
                                                const endPctRaw = (endPctVal === null || endPctVal === undefined || endPctVal === '') ? null : Number(endPctVal);
                                                let start = Number.isFinite(startRaw) ? Math.max(0, startRaw) : 0;
                                                let end = Number.isFinite(endSecRaw) ? Math.max(0, endSecRaw) : null;
                                                if (duration !== null) {
                                                        if (Number.isFinite(startPctRaw)) {
                                                                const pct = clamp(startPctRaw, 0, 1);
                                                                start = duration * pct;
                                                        }
                                                        if (endPctVal === null || endPctVal === undefined || endPctVal === '') {
                                                                // Keep explicit second-based end if present, otherwise full-range.
                                                        } else if (Number.isFinite(endPctRaw)) {
                                                                const pct = clamp(endPctRaw, 0, 1);
                                                                end = pct >= 0.9999 ? null : (duration * pct);
                                                        }
                                                }
                                                return {
                                                        start: Number.isFinite(start) ? Math.max(0, start) : 0,
                                                        end: Number.isFinite(end) ? Math.max(0, end) : null,
                                                };
                                        } catch (_) {
                                                return { start: 0, end: null };
                                        }
                                };

                                const updateLoopReadouts = () => {
                                        const pts = getStoredLoopPoints();
                                        if (loopInReadout) {
                                                loopInReadout.textContent = 'In ' + (formatLoopTime(pts.start) || '0:00');
                                        }
                                        if (loopOutReadout) {
                                                loopOutReadout.textContent = 'Out ' + (pts.end === null ? 'Full' : (formatLoopTime(pts.end) || '0:00'));
                                        }
                                };

                                const setPlaybarProgress = (loopRange) => {
                                        if (!playbarFill) return;
                                        const range = loopRange || getLoopRange();
                                        const start = Number(range.start) || 0;
                                        const end = (range.end === null || range.end === undefined) ? ((Number.isFinite(video.duration) && video.duration > 0) ? Number(video.duration) : null) : Number(range.end);
                                        if (!Number.isFinite(end) || end <= start) {
                                                playbarFill.style.width = '0%';
                                                return;
                                        }
                                        const pos = clamp((Number(video.currentTime) - start) / (end - start), 0, 1);
                                        playbarFill.style.width = String(Math.round(pos * 1000) / 10) + '%';
                                };

                                const getTimelineSpan = () => {
                                        const duration = (Number.isFinite(video.duration) && video.duration > 0) ? Number(video.duration) : null;
                                        if (duration !== null) {
                                                return duration;
                                        }
                                        const points = getStoredLoopPoints();
                                        const maxKnown = Math.max(Number(points.start || 0), Number(points.end || 0), 30);
                                        return maxKnown;
                                };

                                const getLoopRange = () => {
                                        const fullEnd = (Number.isFinite(video.duration) && video.duration > 0) ? Number(video.duration) : null;
                                        const points = getStoredLoopPoints();
                                        if ((Number(points.start) || 0) <= 0 && points.end === null) {
                                                return { start: 0, end: fullEnd };
                                        }

                                        let start = Number.isFinite(points.start) ? Number(points.start) : 0;
                                        let end = Number.isFinite(points.end) ? Number(points.end) : fullEnd;

                                        if (!Number.isFinite(start) || start < 0) {
                                                start = 0;
                                        }

                                        if (end === null || end === undefined || !Number.isFinite(end)) {
                                                return { start, end: fullEnd };
                                        }

                                        if (fullEnd !== null) {
                                                end = Math.min(end, fullEnd);
                                        }

                                        if (end <= start) {
                                                if (fullEnd !== null && fullEnd > start + 0.1) {
                                                        end = fullEnd;
                                                } else {
                                                        end = start + 0.1;
                                                }
                                        }

                                        return { start, end };
                                };

                                const syncTimelineFromInputs = () => {
                                        if (!loopTrack || !loopInMarker || !loopOutMarker || !loopRangeFill) {
                                                return;
                                        }
                                        const span = Math.max(0.1, getTimelineSpan());
                                        const loopRange = getLoopRange();
                                        const start = clamp(Number(loopRange.start) || 0, 0, span);
                                        const rawEnd = (loopRange.end === null || loopRange.end === undefined) ? span : Number(loopRange.end);
                                        const end = clamp(Number.isFinite(rawEnd) ? rawEnd : span, start, span);
                                        const startPct = clamp((start / span) * 100, 0, 100);
                                        const endPct = clamp((end / span) * 100, startPct, 100);
                                        loopInMarker.style.left = String(startPct) + '%';
                                        loopOutMarker.style.left = String(endPct) + '%';
                                        loopRangeFill.style.left = String(startPct) + '%';
                                        loopRangeFill.style.width = String(Math.max(0, endPct - startPct)) + '%';
                                        updateLoopReadouts();
                                };

                                const applyMarkerToInputs = (mode, seconds) => {
                                        const span = Math.max(0.1, getTimelineSpan());
                                        const value = clamp(Number(seconds) || 0, 0, span);
                                        const points = getStoredLoopPoints();
                                        let nextStart = Number.isFinite(points.start) ? points.start : 0;
                                        let nextEnd = Number.isFinite(points.end) ? points.end : null;
                                        if (mode === 'out') {
                                                nextEnd = value;
                                        } else {
                                                nextStart = value;
                                        }

                                        if (Number.isFinite(nextEnd) && nextEnd <= nextStart) {
                                                if (mode === 'in') {
                                                        nextEnd = clamp(nextStart + 0.1, 0, span);
                                                } else {
                                                        nextStart = clamp(nextEnd - 0.1, 0, span);
                                                }
                                        }

                                        setStoredLoopPoints(nextStart, nextEnd);
                                        syncTimelineFromInputs();
                                };

                                const timelineEventToSeconds = (evt) => {
                                        if (!loopTrack) return 0;
                                        const rect = loopTrack.getBoundingClientRect();
                                        const span = Math.max(0.1, getTimelineSpan());
                                        const x = clamp(Number(evt.clientX) - Number(rect.left), 0, Math.max(1, Number(rect.width)));
                                        const ratio = x / Math.max(1, Number(rect.width));
                                        return ratio * span;
                                };

                                const playbarEventToSeconds = (evt) => {
                                        if (!playbar) return 0;
                                        const rect = playbar.getBoundingClientRect();
                                        const duration = (Number.isFinite(video.duration) && video.duration > 0) ? Number(video.duration) : Math.max(0.1, getTimelineSpan());
                                        const x = clamp(Number(evt.clientX) - Number(rect.left), 0, Math.max(1, Number(rect.width)));
                                        const ratio = x / Math.max(1, Number(rect.width));
                                        return ratio * duration;
                                };

                                const seekPreviewTo = (seconds) => {
                                        const duration = (Number.isFinite(video.duration) && video.duration > 0) ? Number(video.duration) : Math.max(0.1, getTimelineSpan());
                                        const target = clamp(Number(seconds) || 0, 0, duration);
                                        try {
                                                video.currentTime = target;
                                        } catch (_) {
                                        }
                                        setPlaybarProgress(getLoopRange());
                                };

                                if (loopTrack) {
                                        loopTrack.addEventListener('click', (evt) => {
                                                if (evt.target && evt.target.classList && evt.target.classList.contains('loop-marker')) {
                                                        return;
                                                }
                                                const sec = timelineEventToSeconds(evt);
                                                applyMarkerToInputs(activeMarker, sec);
                                                if (activeMarker === 'in') {
                                                        setActiveMode('out');
                                                }
                                        });
                                }

                                if (playbar) {
                                        let scrubbing = false;
                                        const onScrubMove = (evt) => {
                                                if (!scrubbing) return;
                                                const sec = playbarEventToSeconds(evt);
                                                applyMarkerToInputs(activeMarker, sec);
                                                seekPreviewTo(sec);
                                        };
                                        const onScrubUp = () => {
                                                if (!scrubbing) return;
                                                scrubbing = false;
                                                window.removeEventListener('pointermove', onScrubMove);
                                                window.removeEventListener('pointerup', onScrubUp);
                                        };
                                        playbar.addEventListener('pointerdown', (evt) => {
                                                evt.preventDefault();
                                                scrubbing = true;
                                                try { video.pause(); } catch (_) {}
                                                const sec = playbarEventToSeconds(evt);
                                                applyMarkerToInputs(activeMarker, sec);
                                                seekPreviewTo(sec);
                                                window.addEventListener('pointermove', onScrubMove);
                                                window.addEventListener('pointerup', onScrubUp);
                                        });
                                }

                                const bindMarkerDrag = (markerEl, mode) => {
                                        if (!markerEl) return;
                                        let dragging = false;
                                        const onMove = (evt) => {
                                                if (!dragging) return;
                                                const sec = timelineEventToSeconds(evt);
                                                applyMarkerToInputs(mode, sec);
                                        };
                                        const onUp = () => {
                                                if (!dragging) return;
                                                dragging = false;
                                                window.removeEventListener('pointermove', onMove);
                                                window.removeEventListener('pointerup', onUp);
                                        };
                                        markerEl.addEventListener('pointerdown', (evt) => {
                                                evt.preventDefault();
                                                dragging = true;
                                                setActiveMode(mode);
                                                window.addEventListener('pointermove', onMove);
                                                window.addEventListener('pointerup', onUp);
                                        });
                                };

                                bindMarkerDrag(loopInMarker, 'in');
                                bindMarkerDrag(loopOutMarker, 'out');

                                Array.from(modeButtons || []).forEach(btn => {
                                        btn.addEventListener('click', () => {
                                                const mode = String(btn.getAttribute('data-loop-mode') || 'in').toLowerCase();
                                                setActiveMode(mode);
                                        });
                                });

                                Array.from(markButtons || []).forEach(btn => {
                                        btn.addEventListener('click', () => {
                                                const mode = String(btn.getAttribute('data-loop-mark') || 'in').toLowerCase();
                                                const markSec = Number(video.currentTime) || 0;
                                                applyMarkerToInputs(mode, markSec);
                                                setActiveMode(mode === 'in' ? 'out' : 'in');
                                        });
                                });

                                if (loopInReadout) {
                                        loopInReadout.addEventListener('click', () => {
                                                const current = getStoredLoopPoints();
                                                const typed = window.prompt('Set loop In time (seconds, mm:ss, or hh:mm:ss):', formatLoopTime(current.start) || '0:00');
                                                if (typed === null) return;
                                                const parsed = parseTypedLoopTime(typed);
                                                if (!Number.isFinite(parsed)) {
                                                        setStatus('Invalid In time. Use seconds or mm:ss.', 'err');
                                                        return;
                                                }
                                                applyMarkerToInputs('in', parsed);
                                                setActiveMode('out');
                                                if (hovering) {
                                                        const range = getLoopRange();
                                                        try { video.currentTime = Number(range.start) || 0; } catch (_) {}
                                                }
                                        });
                                }

                                if (loopOutReadout) {
                                        loopOutReadout.addEventListener('click', () => {
                                                const current = getStoredLoopPoints();
                                                const defaultOut = (current.end === null) ? 'full' : (formatLoopTime(current.end) || 'full');
                                                const typed = window.prompt('Set loop Out time (seconds, mm:ss, hh:mm:ss, or "full"):', defaultOut);
                                                if (typed === null) return;
                                                const parsed = parseTypedLoopTime(typed);
                                                if (String(typed || '').trim() && parsed === null && String(typed || '').trim().toLowerCase() !== 'full' && String(typed || '').trim().toLowerCase() !== 'end' && String(typed || '').trim().toLowerCase() !== 'max') {
                                                        setStatus('Invalid Out time. Use seconds, mm:ss, or "full".', 'err');
                                                        return;
                                                }
                                                if (parsed === null) {
                                                        const pts = getStoredLoopPoints();
                                                        setStoredLoopPoints(pts.start, null);
                                                        syncTimelineFromInputs();
                                                } else {
                                                        applyMarkerToInputs('out', parsed);
                                                }
                                                setActiveMode('in');
                                        });
                                }

                                setActiveMode('in');
                                const persistedLoop = loadSavedLoopPoints();
                                setStoredLoopPoints(persistedLoop.start, persistedLoop.end);
                                try {
                                        video.pause();
                                        video.currentTime = 0;
                                } catch (_) {
                                }
                                const playPreview = () => {
                                        const persistedLoop = loadSavedLoopPoints();
                                        setStoredLoopPoints(persistedLoop.start, persistedLoop.end);
                                        syncTimelineFromInputs();
                                        hovering = true;
                                        const loopRange = getLoopRange();
                                        try {
                                                video.currentTime = Number(loopRange.start) || 0;
                                        } catch (_) {
                                        }
                                        if ((video.readyState || 0) < 2) {
                                                try { video.load(); } catch (_) {}
                                        }
                                        const playPromise = video.play();
                                        if (playPromise && typeof playPromise.catch === 'function') {
                                                playPromise.catch(() => {
                                                        try {
                                                                video.muted = true;
                                                                const retry = video.play();
                                                                if (retry && typeof retry.catch === 'function') {
                                                                        retry.catch(() => {});
                                                                }
                                                        } catch (_) {}
                                                });
                                        }
                                        setPlaybarProgress(loopRange);
                                };
                                const stopPreview = () => {
                                        hovering = false;
                                        try {
                                                video.pause();
                                                video.currentTime = 0;
                                        } catch (_) {
                                        }
                                        setPlaybarProgress({ start: 0, end: 1 });
                                        if (playbarFill) {
                                                playbarFill.style.width = '0%';
                                        }
                                };

                                const loopBack = () => {
                                        if (!hovering) return;
                                        const loopRange = getLoopRange();
                                        try {
                                                video.currentTime = Number(loopRange.start) || 0;
                                        } catch (_) {}
                                        const again = video.play();
                                        if (again && typeof again.catch === 'function') again.catch(() => {});
                                        setPlaybarProgress(loopRange);
                                };

                                const onTimeUpdate = () => {
                                        if (!hovering) return;
                                        const loopRange = getLoopRange();
                                        const effectiveEnd = (loopRange.end !== null && loopRange.end !== undefined)
                                                ? Number(loopRange.end)
                                                : (Number.isFinite(video.duration) && video.duration > 0 ? Number(video.duration) : null);
                                        if (effectiveEnd === null) return;
                                        if (video.currentTime >= effectiveEnd - 0.05) {
                                                loopBack();
                                                return;
                                        }
                                        setPlaybarProgress(loopRange);
                                };

                                video.addEventListener('mouseenter', playPreview);
                                video.addEventListener('mouseleave', stopPreview);
                                video.addEventListener('pointerenter', playPreview);
                                video.addEventListener('pointerleave', stopPreview);
                                video.addEventListener('timeupdate', onTimeUpdate);
                                video.addEventListener('ended', loopBack);

                                video.addEventListener('loadedmetadata', () => {
                                        const persisted = loadSavedLoopPoints();
                                        setStoredLoopPoints(persisted.start, persisted.end);
                                        syncTimelineFromInputs();
                                        // If already hovering when metadata arrives, seek to correct start
                                        if (hovering && persisted.start > 0.05) {
                                                try { video.currentTime = persisted.start; } catch (_) {}
                                        }
                                });
                                video.addEventListener('durationchange', syncTimelineFromInputs);
                                video.addEventListener('loadedmetadata', () => setPlaybarProgress(getLoopRange()));
                                syncTimelineFromInputs();
                                if (playbarFill) {
                                        playbarFill.style.width = '0%';
                                }
                                if (consumeReturnAutoplay(loopId)) {
                                        window.requestAnimationFrame(() => {
                                                playPreview();
                                        });
                                }
                        });
                }

                function openPhotoModal(url, title, folderName, enableLiveRefresh) {
                        if (!url || !photoModal || !photoModalImg) return;
                        photoModalImg.src = url;
                        photoModalImg.alt = title || 'Timelapse photo enlarged';
                        if (photoModalTitle) photoModalTitle.textContent = title || 'Photo preview';
                        modalBaseTitle = String(title || 'Photo preview');
                        if (photoModalFull) photoModalFull.href = url;
                        modalLiveFolderName = folderName ? String(folderName) : '';
                        modalLastPhotoBaseUrl = String(url || '');
                        setModalLiveEnabled(!!enableLiveRefresh);
                        photoModal.classList.add('open');
                        photoModal.setAttribute('aria-hidden', 'false');
                }

                function isAnyFullscreenOpen() {
                        return Boolean(document.fullscreenElement || document.webkitFullscreenElement);
                }

                async function requestElementFullscreen(el) {
                        if (!el) return;
                        if (el.requestFullscreen) {
                                await el.requestFullscreen();
                                return;
                        }
                        if (el.webkitRequestFullscreen) {
                                el.webkitRequestFullscreen();
                        }
                }

                async function exitAnyFullscreen() {
                        if (document.fullscreenElement && document.exitFullscreen) {
                                await document.exitFullscreen();
                                return;
                        }
                        if (document.webkitFullscreenElement && document.webkitExitFullscreen) {
                                document.webkitExitFullscreen();
                        }
                }

                async function closePhotoModal() {
                        if (!photoModal || !photoModalImg) return;
                        if (isAnyFullscreenOpen()) {
                                await exitAnyFullscreen();
                        }
                        photoModal.classList.remove('open');
                        photoModal.setAttribute('aria-hidden', 'true');
                        photoModalImg.removeAttribute('src');
                        if (photoModalTitle) {
                                photoModalTitle.textContent = modalBaseTitle || 'Photo preview';
                        }
                        modalLiveFolderName = '';
                        modalLastPhotoBaseUrl = '';
                        modalBaseTitle = '';
                        setModalLiveEnabled(false);
                }

                function syncPhotoModalZoomAffordance() {
                        if (!photoModalImg) return;
                        const fullscreenOpen = isAnyFullscreenOpen();
                        photoModalImg.style.cursor = fullscreenOpen ? 'zoom-out' : 'zoom-in';
                        photoModalImg.title = fullscreenOpen ? 'Click to exit fullscreen' : 'Click to fullscreen';
                }

                function bindDetailPhotoOpeners() {
                        document.querySelectorAll('.detail-photo-open').forEach(btn => {
                                btn.addEventListener('click', () => {
                                        const url = btn.getAttribute('data-photo-url') || '';
                                        const title = btn.getAttribute('data-photo-title') || 'Photo preview';
                                        const folderName = btn.getAttribute('data-photo-folder') || '';
                                        const enableLiveRefresh = (btn.getAttribute('data-live-refresh') || '') === '1';
                                        openPhotoModal(url, title, folderName, enableLiveRefresh);
                                });
                        });
                }

                function refreshOpenPhotoModalFromLiveSync(directories, syncState) {
                        if (!photoModal || !photoModal.classList.contains('open')) return;
                        if (!modalLiveFolderName) return;
                        if (!modalLiveKeepUpdated) {
                                return;
                        }
                        const hit = findDirectoryByName(directories, modalLiveFolderName);
                        if (!hit) return;
                        const baseUrl = String(hit.last_photo_url || hit.preview_url || '');
                        if (!baseUrl) return;
                        applyModalLatestPhotoUrl(baseUrl, String(hit.last_photo_name || ''));
                }

                function bindPreviewScrub() {
                        const previewEls = document.querySelectorAll('.scrub-preview');
                        previewEls.forEach(img => {
                                const framesRaw = img.getAttribute('data-frames') || '';
                                let frames = framesRaw
                                        ? framesRaw.split('||').map(s => String(s || '').trim()).filter(Boolean)
                                        : [];
                                const folder = String(img.getAttribute('data-folder') || '').trim();
                                const view = String(img.getAttribute('data-view') || 'optimal').trim().toLowerCase() === 'all' ? 'all' : 'optimal';
                                let fullRangeFetchPromise = null;
                                let fullRangeLoaded = false;

                                const ensureFullRangeFrames = async () => {
                                        if (fullRangeLoaded) {
                                                return frames;
                                        }
                                        if (fullRangeFetchPromise) {
                                                return fullRangeFetchPromise;
                                        }
                                        if (!folder) {
                                                fullRangeLoaded = true;
                                                return frames;
                                        }
                                        fullRangeFetchPromise = (async () => {
                                                try {
                                                        const r = await fetch('/api/timelapse/frames?dir=' + encodeURIComponent(folder) + '&view=' + encodeURIComponent(view), { cache: 'no-store' });
                                                        const body = await r.json();
                                                        const fullFrames = Array.isArray(body && body.frames)
                                                                ? body.frames.map(s => String(s || '').trim()).filter(Boolean)
                                                                : [];
                                                        if (fullFrames.length > frames.length) {
                                                                frames = fullFrames;
                                                                img.setAttribute('data-frames', fullFrames.join('||'));
                                                        }
                                                        fullRangeLoaded = true;
                                                        return frames;
                                                } catch (_) {
                                                        fullRangeLoaded = true;
                                                        return frames;
                                                } finally {
                                                        fullRangeFetchPromise = null;
                                                }
                                        })();
                                        return fullRangeFetchPromise;
                                };

                                let initial = img.getAttribute('src') || frames[frames.length - 1];
                                const stripHoverToken = (src) => String(src || '').replace(/[?&]live_ts=[^&]*/g, '');
                                const findFrameIndexBySrc = (src) => {
                                        const value = stripHoverToken(String(src || '').trim());
                                        if (!value) return -1;
                                        return frames.findIndex((f) => stripHoverToken(f) === value);
                                };
                                const initialIndex = findFrameIndexBySrc(initial);
                                img.dataset.frameIndex = String(initialIndex >= 0 ? initialIndex : (frames.length - 1));
                                let lastX = null;
                                let hovering = false;
                                let autoDirection = -1;
                                let rafId = 0;
                                let lastTickMs = 0;
                                const FRAME_STEP_MS = 120;

                                const setFrameAt = (nextIndex) => {
                                        const next = Math.max(0, Math.min(frames.length - 1, Number(nextIndex) || 0));
                                        img.dataset.frameIndex = String(next);
                                        img.src = frames[next];
                                };

                                const stepHoverPlayback = () => {
                                        if (frames.length < 2) {
                                                return;
                                        }
                                        if (!hovering) return;
                                        const current = parseInt(img.dataset.frameIndex || '0', 10) || 0;
                                        let next = current + autoDirection;
                                        if (next < 0 || next > (frames.length - 1)) {
                                                autoDirection = autoDirection * -1;
                                                next = current + autoDirection;
                                        }
                                        if (next !== current) {
                                                setFrameAt(next);
                                        }
                                };

                                const hoverLoop = (tickMs) => {
                                        if (!hovering) {
                                                rafId = 0;
                                                return;
                                        }
                                        if (!lastTickMs || (tickMs - lastTickMs) >= FRAME_STEP_MS) {
                                                stepHoverPlayback();
                                                lastTickMs = tickMs;
                                        }
                                        rafId = window.requestAnimationFrame(hoverLoop);
                                };

                                const startHoverPlayback = () => {
                                        if (rafId) return;
                                        lastTickMs = 0;
                                        rafId = window.requestAnimationFrame(hoverLoop);
                                };

                                const stopHoverPlayback = () => {
                                        if (!rafId) return;
                                        window.cancelAnimationFrame(rafId);
                                        rafId = 0;
                                };

                                img.addEventListener('mouseenter', async () => {
                                        try {
                                                await ensureFullRangeFrames();
                                        } catch (_) {
                                        }
                                        if (frames.length < 2) {
                                                return;
                                        }
                                        initial = img.getAttribute('src') || initial;
                                        const currentIndex = findFrameIndexBySrc(initial);
                                        if (currentIndex >= 0) {
                                                img.dataset.frameIndex = String(currentIndex);
                                        }
                                        const startIndex = parseInt(img.dataset.frameIndex || '0', 10) || 0;
                                        autoDirection = startIndex >= (frames.length - 1) ? -1 : 1;
                                        hovering = true;
                                        // Make the preview visibly react as soon as hover begins.
                                        stepHoverPlayback();
                                        startHoverPlayback();
                                        lastX = null;
                                });

                                img.addEventListener('mousemove', (event) => {
                                        if (lastX === null) {
                                                lastX = event.clientX;
                                                return;
                                        }
                                        const dx = event.clientX - lastX;
                                        lastX = event.clientX;
                                        if (Math.abs(dx) < 2) {
                                                return;
                                        }

                                        const current = parseInt(img.dataset.frameIndex || '0', 10) || 0;
                                        const direction = dx > 0 ? 1 : -1;
                                        autoDirection = direction;
                                        const magnitudeSteps = Math.max(1, Math.min(4, Math.floor(Math.abs(dx) / 14)));
                                        const next = Math.max(0, Math.min(frames.length - 1, current + (direction * magnitudeSteps)));
                                        if (next !== current) {
                                                setFrameAt(next);
                                        }
                                });

                                img.addEventListener('wheel', (event) => {
                                        if (frames.length < 2) {
                                                return;
                                        }
                                        event.preventDefault();
                                        const current = parseInt(img.dataset.frameIndex || '0', 10) || 0;
                                        const delta = event.deltaY > 0 ? 1 : -1;
                                        const next = Math.max(0, Math.min(frames.length - 1, current + delta));
                                        if (next !== current) {
                                                img.dataset.frameIndex = String(next);
                                                img.src = frames[next];
                                        }
                                }, { passive: false });

                                img.addEventListener('mouseleave', () => {
                                        hovering = false;
                                        stopHoverPlayback();
                                        img.src = initial;
                                        const initialFrameIndex = findFrameIndexBySrc(initial);
                                        img.dataset.frameIndex = String(initialFrameIndex >= 0 ? initialFrameIndex : (frames.length - 1));
                                        lastX = null;
                                });
                        });
                }

                function updateSummary(summary) {
                        const s = summary || {};
                        remoteDirCountEl.textContent = String(s.remote_dirs ?? 0);
                        localDirCountEl.textContent = String(s.local_dirs ?? 0);
                        remoteItemCountEl.textContent = String(s.remote_items ?? 0);
                        localItemCountEl.textContent = String(s.local_items ?? 0);
                }

                function updateSyncProgress(syncState, directories) {
                        const sync = syncState || {};
                        const prog = sync.progress || {};
                        const hasPercent = Number.isFinite(prog.percent);
                        const progressFolder = String(prog.folder || sync.folder || '').trim();
                        const progressView = String(prog.view || sync.view || '').trim().toLowerCase();
                        const viewLabel = progressView === 'all' ? 'all' : 'optimal';
                        const dirs = Array.isArray(directories) ? directories : [];
                        let remaining = null;
                        let remoteCount = null;
                        let localCount = null;
                        const parsedDone = Number.isFinite(Number(prog.files_done)) ? Number(prog.files_done) : null;
                        const parsedTotal = Number.isFinite(Number(prog.files_total)) ? Number(prog.files_total) : null;
                        const parsedRemaining = Number.isFinite(Number(prog.files_remaining)) ? Number(prog.files_remaining) : null;
                        if (progressFolder) {
                                const hit = dirs.find(d => String((d && d.name) || '') === String(progressFolder));
                                if (hit) {
                                        remoteCount = Number(hit.remote_selected_count || 0);
                                        localCount = Number(hit.local_selected_count || 0);
                                        remaining = Math.max(0, remoteCount - localCount);
                                }
                        }
                        stopSyncBtn.disabled = !sync.running;

                        if (!sync.running) {
                                syncRemaining.textContent = 'Files: not syncing (remote status only)';
                        } else if (parsedTotal !== null && parsedTotal > 0 && parsedDone !== null) {
                                const rem = parsedRemaining !== null ? parsedRemaining : Math.max(0, parsedTotal - parsedDone);
                                syncRemaining.textContent = 'Files: ' + parsedDone + '/' + parsedTotal + ' (remaining ' + rem + ')';
                        } else if (remaining === null) {
                                syncRemaining.textContent = 'Files: --';
                        } else {
                                syncRemaining.textContent = 'Mirror count: ' + localCount + '/' + remoteCount + ' (remaining ' + remaining + ')';
                        }

                        if (sync.running) {
                                const targetLabel = progressFolder ? (progressFolder + ' [' + viewLabel + ']') : '...';
                                if (sync.watch) {
                                        if (prog.phase === 'processing') {
                                                syncStage.textContent = 'Live processing new frames for ' + targetLabel;
                                        } else {
                                                syncStage.textContent = 'Live syncing ' + targetLabel + ' every ' + (sync.interval_seconds || 60) + 's';
                                        }
                                } else if (prog.phase === 'processing') {
                                        syncStage.textContent = 'Processing new frames for ' + targetLabel;
                                } else if (prog.phase === 'waiting') {
                                        syncStage.textContent = 'Waiting for remote frames in ' + targetLabel;
                                } else {
                                        syncStage.textContent = 'Syncing ' + targetLabel;
                                }
                        } else if (prog.phase === 'error') {
                                syncStage.textContent = 'Sync error';
                        } else if (prog.phase === 'complete') {
                                syncStage.textContent = 'Sync complete';
                        } else {
                                syncStage.textContent = 'No active sync (reading remote status only)';
                        }

                        const derivedPct = (!hasPercent && parsedTotal !== null && parsedTotal > 0 && parsedDone !== null)
                                ? Math.max(0, Math.min(100, Math.round((parsedDone / parsedTotal) * 100)))
                                : null;
                        const activePct = hasPercent ? Math.max(0, Math.min(100, Number(prog.percent))) : derivedPct;

                        if (sync.running && (activePct === null || (remaining !== null && remaining > 0 && Number(activePct || 0) >= 100))) {
                                syncFill.classList.add('indeterminate');
                                syncFill.style.width = '35%';
                                syncPercent.textContent = (parsedTotal !== null && parsedDone !== null)
                                        ? ('Working... ' + parsedDone + '/' + parsedTotal)
                                        : 'Working...';
                        } else if (!sync.running) {
                                syncFill.classList.remove('indeterminate');
                                syncFill.style.width = '0%';
                                syncPercent.textContent = 'Idle';
                        } else {
                                syncFill.classList.remove('indeterminate');
                                const pct = (activePct !== null) ? activePct : (sync.running ? 0 : 100);
                                syncFill.style.width = pct + '%';
                                syncPercent.textContent = pct + '%';
                        }

                        if (!sync.running) {
                                syncSpeed.textContent = 'Speed: n/a (idle)';
                                syncEta.textContent = 'ETA: n/a (idle)';
                        } else {
                                syncSpeed.textContent = 'Speed: ' + (prog.speed || '--');
                                syncEta.textContent = 'ETA: ' + (prog.eta || '--');
                        }
                }

                async function refresh(silent = false, forceRender = false) {
                        if (refreshInFlight) {
                                refreshQueued = true;
                                return;
                        }
                        refreshInFlight = true;
                        if (!silent) {
                                setStatus('Loading directories...');
                        }
                        try {
                                const r = await fetch('/api/timelapse-directories?view=optimal');
                                const body = await r.json();
                                if (!r.ok || !body.ok) {
                                        throw new Error(body.error || 'request failed');
                                }
                                const dirs = applyLiveActivityFlags(body.directories || []);
                                const syncState = body.sync || { running: false };
                                const signature = buildDirectoriesSignature(dirs, syncState);
                                const forceLiveRender = !!(syncState.running && syncState.watch);
                                const hoverVideoActive = !!document.querySelector('video.preview-video:hover');
                                if (forceRender || (!hoverVideoActive && (forceLiveRender || signature !== lastDirectoriesSignature))) {
                                        renderRows(dirs, syncState);
                                        lastDirectoriesSignature = signature;
                                }
                                refreshOpenPhotoModalFromLiveSync(dirs, syncState);
                                updateSummary(body.summary || {});
                                updateSyncProgress(syncState, dirs);

                                // Restore expanded state if the folder still exists.
                                if (expandedFolderNameState) {
                                        const restoredBtn = document.querySelector(`.expander-btn[data-expand="${expandedFolderNameState}"]`);
                                        if (restoredBtn) {
                                                const detail = document.querySelector(`.detail-row[data-detail="${expandedFolderNameState}"]`);
                                                if (detail) {
                                                        detail.classList.add('open');
                                                        restoredBtn.setAttribute('aria-expanded', 'true');
                                                        restoredBtn.textContent = '▾';
                                                }
                                        } else {
                                                expandedFolderNameState = null;
                                        }
                                }

                                if (syncState && syncState.running) {
                                        const prog = syncState.progress || {};
                                        const parsedDone = Number.isFinite(Number(prog.files_done)) ? Number(prog.files_done) : null;
                                        const parsedTotal = Number.isFinite(Number(prog.files_total)) ? Number(prog.files_total) : null;
                                        const folder = String(syncState.folder || prog.folder || '').trim();
                                        const view = String(syncState.view || prog.view || 'optimal').trim().toLowerCase() === 'all' ? 'all' : 'optimal';
                                        const pct = Number.isFinite(prog.percent) ? (' - ' + prog.percent + '%') : '';
                                        const fileCounter = (parsedDone !== null && parsedTotal !== null && parsedTotal > 0)
                                                ? (' - files ' + parsedDone + '/' + parsedTotal)
                                                : '';
                                        const mode = syncState.watch ? ('live every ' + (syncState.interval_seconds || 60) + 's') : 'one-shot';
                                        setStatus('Sync running (' + mode + ') for ' + folder + ' [' + view + '] (pid ' + syncState.pid + ')' + fileCounter + pct, 'ok');
                                } else if ((syncState.progress || {}).phase === 'error') {
                                        const msg = (syncState.progress || {}).message || 'Sync failed';
                                        setStatus(msg, 'err');
                                } else if (!silent) {
                                        setStatus('Loaded ' + (body.directories || []).length + ' directories from ' + body.root, 'ok');
                                }
                        } catch (err) {
                                setStatus('Load failed: ' + err.message, 'err');
                        } finally {
                                refreshInFlight = false;
                                if (refreshQueued) {
                                        refreshQueued = false;
                                        setTimeout(() => {
                                                refresh(true, false).catch(() => {});
                                        }, 0);
                                }
                        }
                }

                document.getElementById('refreshBtn').addEventListener('click', () => refresh(false, true));
                if (monitorViewSelect) {
                        monitorViewSelect.addEventListener('change', () => {
                                refresh(false, true).catch(() => {});
                        });
                }
                stopSyncBtn.addEventListener('click', async () => {
                        stopSyncBtn.disabled = true;
                        setStatus('Stopping sync...', '');
                        try {
                                const r = await fetch('/api/timelapse-sync', {
                                        method: 'POST',
                                        headers: {'Content-Type': 'application/json'},
                                        body: JSON.stringify({ action: 'stop' })
                                });
                                const body = await r.json();
                                if (!r.ok || !body.ok) {
                                        throw new Error(body.error || 'Failed to stop sync');
                                }
                                setStatus('Sync stopped', 'ok');
                                await refresh(false, true);
                        } catch (err) {
                                setStatus('Stop failed: ' + err.message, 'err');
                                stopSyncBtn.disabled = false;
                        }
                });
                navModeSelect.addEventListener('change', () => {
                        const target = String(navModeSelect.value || '').trim();
                        if (!target) return;
                        window.location.href = mainAppOrigin + '/' + target;
                });
                navControlSelect.addEventListener('change', () => {
                        const target = String(navControlSelect.value || '').trim();
                        if (!target) return;
                        window.location.href = mainAppOrigin + '/' + target;
                });
                if (photoModalClose) {
                        photoModalClose.addEventListener('click', () => {
                                closePhotoModal().catch(() => {});
                        });
                }
                if (photoModalLiveBtn) {
                        photoModalLiveBtn.addEventListener('click', () => {
                                setModalLiveEnabled(!modalLiveKeepUpdated);
                        });
                }
                if (photoModal) {
                        photoModal.addEventListener('click', (event) => {
                                if (event.target === photoModal) {
                                        closePhotoModal().catch(() => {});
                                }
                        });
                }
                if (photoModalImg) {
                        syncPhotoModalZoomAffordance();
                        photoModalImg.addEventListener('click', () => {
                                if (!photoModal || !photoModal.classList.contains('open')) {
                                        return;
                                }
                                if (isAnyFullscreenOpen()) {
                                        exitAnyFullscreen().catch(() => {});
                                        return;
                                }
                                const fullscreenTarget = photoModalContent || photoModal;
                                requestElementFullscreen(fullscreenTarget).catch(() => {});
                        });
                }
                document.addEventListener('fullscreenchange', syncPhotoModalZoomAffordance);
                document.addEventListener('webkitfullscreenchange', syncPhotoModalZoomAffordance);
                document.addEventListener('keydown', (event) => {
                        if (event.key === 'Escape' && photoModal && photoModal.classList.contains('open')) {
                                if (isAnyFullscreenOpen()) {
                                        exitAnyFullscreen().catch(() => {});
                                        return;
                                }
                                closePhotoModal().catch(() => {});
                        }
                });
                initMainNavLinks();
                loadHiddenFolders();
                refresh(false, true);
                setInterval(() => refresh(true, false), 3000);
        </script>
</body>
</html>
"""

VIEWER_PAGE = """<!doctype html>
<html lang="en">
<head>
	<meta charset="utf-8">
	<meta name="viewport" content="width=device-width,initial-scale=1">
	<title>Timelapse Viewer</title>
		* { box-sizing: border-box; }
		body { margin: 0; font-family: "Avenir Next", "Gill Sans", "Trebuchet MS", sans-serif; background: #0a0a0a; color: #fff; }
		.header { background: #1a1a1a; border-bottom: 2px solid #ff9000; padding: 15px 20px; display: flex; justify-content: space-between; align-items: center; }
		.header h1 { margin: 0; color: #ff9000; font-size: 1.25rem; }
                .header-meta { color: #c8c8c8; font-size: 0.9rem; }
		.header a { color: #ff9000; text-decoration: none; border: 1px solid #444; padding: 6px 10px; border-radius: 8px; }
		.wrap { max-width: 1600px; margin: 0 auto; padding: 18px; display: grid; grid-template-columns: 280px 1fr 300px; gap: 20px; }
		.column { display: flex; flex-direction: column; gap: 12px; }
		.card { background: #1a1a1a; border: 1px solid #2a2a2a; border-radius: 10px; padding: 14px; }
		.card-title { color: #ff9000; font-weight: 700; font-size: 0.95rem; margin-bottom: 8px; text-transform: uppercase; }
		.score-row { display: flex; justify-content: space-between; align-items: baseline; gap: 8px; margin-bottom: 8px; }
		.score-label { color: #aaa; font-size: 0.85rem; }
		.score-value { color: #fff; font-size: 1.1rem; font-weight: 700; }
		.score-status { color: #4ade80; font-size: 0.78rem; text-transform: uppercase; }
		.score-status.warn { color: #fbbf24; }
		.score-status.bad { color: #ef4444; }
		.subtitle { color: #888; font-size: 0.8rem; margin-top: 4px; }
		.center-column { display: flex; flex-direction: column; gap: 12px; }
		.stage { border: 1px solid #333; background: #000; border-radius: 12px; overflow: hidden; flex: 1; display: flex; align-items: center; justify-content: center; }
		.stage img { width: 100%; height: 100%; object-fit: contain; display: block; }
		.controls { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
		.btn { background: #ff9000; color: #000; border: none; border-radius: 8px; padding: 8px 12px; font-weight: 700; cursor: pointer; font-size: 0.9rem; }
		.btn:hover { background: #ffb030; }
		.scrub-bar { flex: 1; }
		.meta { color: #888; font-size: 0.8rem; }
		.chart-canvas { width: 100%; height: 140px; background: #0f0f0f; border: 1px solid #2a2a2a; border-radius: 8px; }
		.loading { color: #888; text-align: center; padding: 20px; }
                .fs-overlay {
                        position: fixed;
                        inset: 0;
                        display: none;
                        z-index: 10000;
                        background: rgba(0, 0, 0, 0.95);
                        padding: 14px;
                }
                .fs-overlay.open {
                        display: grid;
                        grid-template-rows: auto 1fr;
                        gap: 10px;
                }
                .fs-toolbar {
                        display: flex;
                        justify-content: flex-end;
                        align-items: center;
                }
                .fs-close {
                        background: #1f1f1f;
                        color: #fff;
                        border: 1px solid #575757;
                        border-radius: 8px;
                        padding: 8px 12px;
                        font-weight: 700;
                        cursor: pointer;
                }
                .fs-image-wrap {
                        min-height: 0;
                        display: flex;
                        align-items: center;
                        justify-content: center;
                }
                .fs-image {
                        max-width: 100%;
                        max-height: calc(100vh - 80px);
                        object-fit: contain;
                        display: block;
                }
	</style>
</head>
<body>
	<div class="header">
		<h1>MOCO JIB Viewer</h1>
                <div id="viewerFrameCount" class="header-meta">Frames: 0</div>
		<a href="/timelapse-directories">Back To Directories</a>
	</div>
	<div class="wrap">
		<!-- LEFT COLUMN: AI SCORES -->
		<div class="column" id="leftColumn">
			<div class="card">
				<div class="card-title">AI Analysis</div>
				<div id="aiScores" class="loading">Loading...</div>
			</div>
		</div>
		<!-- CENTER COLUMN: IMAGE & CONTROLS -->
		<div class="center-column">
			<div class="stage"><img id="frame" alt="Timelapse frame"></div>
			<div class="controls">
				<button id="prev" class="btn">← Prev</button>
				<button id="play" class="btn">Play</button>
				<button id="next" class="btn">Next →</button>
				<input id="scrub" class="scrub-bar" type="range" min="0" max="0" step="1" value="0">
				<span id="index" class="meta">0 / 0</span>
			</div>
			<div class="meta" id="frameInfo" style="text-align:center;">Loading...</div>
		</div>
		<!-- RIGHT COLUMN: CHARTS -->
		<div class="column">
			<div class="card">
				<div class="card-title">Charts</div>
				<div style="font-size:0.85rem;color:#aaa;margin-bottom:8px;">Luma Histogram</div>
				<canvas id="histogramCanvas" class="chart-canvas"></canvas>
			</div>
		</div>
	</div>
        <div id="frameFullscreen" class="fs-overlay" aria-hidden="true">
                <div class="fs-toolbar">
                        <button id="frameFullscreenClose" class="fs-close" type="button">X Close</button>
                </div>
                <div class="fs-image-wrap">
                        <img id="frameFullscreenImg" class="fs-image" alt="Fullscreen frame preview">
                </div>
        </div>
	<script>
		const q = new URLSearchParams(location.search);
		const wanted = q.get('dir') || '';
		const frame = document.getElementById('frame');
		const scrub = document.getElementById('scrub');
		const indexEl = document.getElementById('index');
		const frameInfo = document.getElementById('frameInfo');
                const viewerFrameCount = document.getElementById('viewerFrameCount');
		const playBtn = document.getElementById('play');
		const aiScoresEl = document.getElementById('aiScores');
		const histogramCanvas = document.getElementById('histogramCanvas');
                const frameFullscreen = document.getElementById('frameFullscreen');
                const frameFullscreenImg = document.getElementById('frameFullscreenImg');
                const frameFullscreenClose = document.getElementById('frameFullscreenClose');
		
		let frames = [];
		let idx = 0;
		let timer = null;
		let directory = null;

		function drawHistogram(bins) {
			const ctx = histogramCanvas.getContext('2d');
			const w = histogramCanvas.width;
			const h = histogramCanvas.height;
			ctx.fillStyle = '#0f0f0f';
			ctx.fillRect(0, 0, w, h);
			
			if (!bins || bins.length === 0) return;
			
			const maxVal = Math.max(...bins);
			const barWidth = w / bins.length;
			ctx.fillStyle = '#888888';
			bins.forEach((val, i) => {
				const barHeight = (val / maxVal) * (h * 0.85);
				ctx.fillRect(i * barWidth, h - barHeight, barWidth - 1, barHeight);
			});
		}

		function renderAIScores(item) {
			if (!item || !item.ai_scores) {
				aiScoresEl.innerHTML = '<div class="loading">No AI data</div>';
				return;
			}
			const scores = item.ai_scores;
			const html = [
				`<div class="score-row">
					<span class="score-label">Exposure Trend</span>
					<span class="score-value">${scores.trend_score ?? 'n/a'}</span>
				</div>`,
				`<div class="score-row">
					<span class="score-label">Smoothness</span>
					<span class="score-value">${scores.smooth_score ?? 'n/a'}</span>
				</div>`,
				`<div class="score-row">
					<span class="score-label">Transition</span>
					<span class="score-value">${scores.transition_score ?? 'n/a'}</span>
				</div>`,
				`<div class="score-row">
					<span class="score-label">Horizon</span>
					<span class="score-value">${scores.horizon_score ?? 'n/a'}</span>
				</div>`,
				`<div class="score-row">
					<span class="score-label">Obstruction</span>
					<span class="score-value">${scores.obstruction_score ?? 'n/a'}</span>
				</div>`,
			].join('');
			aiScoresEl.innerHTML = html;
		}

		function render() {
			if (!frames.length) {
				frame.removeAttribute('src');
				indexEl.textContent = '0 / 0';
				frameInfo.textContent = 'No frames';
                                if (viewerFrameCount) viewerFrameCount.textContent = 'Frames: 0';
				return;
			}
			idx = Math.max(0, Math.min(frames.length - 1, idx));
			frame.src = frames[idx];
			scrub.max = String(frames.length - 1);
			scrub.value = String(idx);
			indexEl.textContent = (idx + 1) + ' / ' + frames.length;
			frameInfo.textContent = 'Frame ' + (idx + 1) + ' of ' + frames.length;
                        if (viewerFrameCount) viewerFrameCount.textContent = 'Frames: ' + frames.length;
			
			// Load histogram for current frame
			if (frames[idx]) {
				fetch('/api/frame-metrics?path=' + encodeURIComponent(frames[idx]))
					.then(r => r.json())
					.then(body => {
						if (body.ok && body.histogram) {
							drawHistogram(body.histogram);
						}
					})
					.catch(e => console.log('Histogram load failed:', e.message));
			}
		}

		function togglePlay() {
			if (timer) {
				clearInterval(timer); timer = null; playBtn.textContent = 'Play'; return;
			}
			timer = setInterval(() => { idx = (idx + 1) % Math.max(1, frames.length); render(); }, 90);
                                        if (source === 'local') return;
                        frameFullscreenImg.src = frame.src;
                }

                function closeFrameFullscreen() {
                        if (!frameFullscreen || !frameFullscreenImg) return;
                        frameFullscreen.classList.remove('open');
                        frameFullscreen.setAttribute('aria-hidden', 'true');
                        frameFullscreenImg.removeAttribute('src');
                }

		document.getElementById('prev').onclick = () => { idx--; render(); };
		document.getElementById('next').onclick = () => { idx++; render(); };
		playBtn.onclick = togglePlay;
		scrub.oninput = () => { idx = parseInt(scrub.value || '0', 10) || 0; render(); };
                frame.onclick = openFrameFullscreen;
                if (frameFullscreenClose) {
                        frameFullscreenClose.onclick = closeFrameFullscreen;
                }
                if (frameFullscreen) {
                        frameFullscreen.addEventListener('click', (event) => {
                                if (event.target === frameFullscreen) {
                                        closeFrameFullscreen();
                                }
                        });
                }
                document.addEventListener('keydown', (event) => {
                        if (event.key === 'Escape' && frameFullscreen && frameFullscreen.classList.contains('open')) {
                                closeFrameFullscreen();
                        }
                });

		frame.addEventListener('mousemove', (ev) => {
			if (!frames.length) return;
			const rect = frame.getBoundingClientRect();
			const x = Math.max(0, Math.min(rect.width, ev.clientX - rect.left));
			idx = Math.round((x / Math.max(1, rect.width)) * (frames.length - 1));
			render();
		});
		frame.addEventListener('wheel', (ev) => {
			if (!frames.length) return;
			ev.preventDefault();
			idx += (ev.deltaY > 0 ? 1 : -1);
			render();
		}, { passive: false });

		(async function init(){
			try {
                                const r = await fetch('/api/timelapse-directories?view=all');
				const body = await r.json();
				const dirs = body.directories || [];
                                const hit = dirs.find(d => d.name === wanted) || dirs[0];
				if (!hit) {
					frameInfo.textContent = 'No directories available.';
					return;
				}
				directory = hit;
                                const selectedView = String(hit.selected_view || 'all').toLowerCase();
                                const fr = await fetch('/api/timelapse/frames?dir=' + encodeURIComponent(hit.name) + '&view=' + encodeURIComponent(selectedView));
                                const fb = await fr.json();
                                frames = (fr.ok && fb.ok && Array.isArray(fb.frames) && fb.frames.length)
                                        ? fb.frames
                                        : (hit.preview_frames || []);
				renderAIScores(hit);
				render();
			} catch (e) {
				frameInfo.textContent = 'Viewer load failed: ' + e.message;
                })();
</body>
</html>
"""


class CameraHTTPHandler(BaseHTTPRequestHandler):
        def _send_cors_headers(self):
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')

        def handle_one_request(self):
                try:
                        super().handle_one_request()
                except (ConnectionResetError, BrokenPipeError):
                        # Client closed the connection mid-response; ignore noisy transport errors.
                        return
                except Exception as exc:
                        try:
                                print(f"[http] unhandled request error: {exc}", file=sys.stderr)
                        except Exception:
                                pass
                        try:
                                body = json.dumps({'ok': False, 'error': 'Internal server error'}).encode('utf-8')
                                self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                                self.send_header('Content-Type', 'application/json; charset=utf-8')
                                self._send_cors_headers()
                                self.send_header('Content-Length', str(len(body)))
                                self.end_headers()
                                self._write_body_safe(body)
                        except Exception:
                                return

        def _write_body_safe(self, body):
                try:
                        self.wfile.write(body)
                        return True
                except (ConnectionResetError, BrokenPipeError):
                        return False

        def _send_json(self, payload, status=HTTPStatus.OK):
                body = json.dumps(payload).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self._send_cors_headers()
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self._write_body_safe(body)

        def do_OPTIONS(self):
                self.send_response(HTTPStatus.NO_CONTENT)
                self._send_cors_headers()
                self.end_headers()

        def do_HEAD(self):
                if self.path.startswith('/api/timelapse-export-download') or self.path.startswith('/api/timelapse-export-media'):
                        parsed = urlparse(self.path)
                        query = parse_qs(parsed.query)
                        file_name = str((query.get('name') or [''])[0]).strip()
                        lower_file = file_name.lower()
                        if not file_name or '/' in file_name or '\\' in file_name or not (lower_file.endswith('.mp4') or lower_file.endswith('.mov')):
                                self.send_response(HTTPStatus.BAD_REQUEST)
                                self._send_cors_headers()
                                self.end_headers()
                                return

                        file_path = (TIMELAPSE_EXPORT_DIR / file_name).resolve()
                        try:
                                file_path.relative_to(TIMELAPSE_EXPORT_DIR.resolve())
                        except Exception:
                                self.send_response(HTTPStatus.FORBIDDEN)
                                self._send_cors_headers()
                                self.end_headers()
                                return

                        if not file_path.exists() or not file_path.is_file():
                                self.send_response(HTTPStatus.NOT_FOUND)
                                self._send_cors_headers()
                                self.end_headers()
                                return

                        try:
                                file_size = int(file_path.stat().st_size)
                        except Exception:
                                self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                                self._send_cors_headers()
                                self.end_headers()
                                return

                        content_type = 'video/quicktime' if lower_file.endswith('.mov') else 'video/mp4'
                        is_media = self.path.startswith('/api/timelapse-export-media')
                        self.send_response(HTTPStatus.OK)
                        self.send_header('Content-Type', content_type)
                        self.send_header('Cache-Control', 'no-store')
                        if is_media:
                                self.send_header('Accept-Ranges', 'bytes')
                                self.send_header('Content-Disposition', f'inline; filename="{file_name}"')
                        else:
                                self.send_header('Content-Disposition', f'attachment; filename="{file_name}"')
                        self._send_cors_headers()
                        self.send_header('Content-Length', str(file_size))
                        self.end_headers()
                        return

                self.send_response(HTTPStatus.NOT_FOUND)
                self._send_cors_headers()
                self.end_headers()

        def do_GET(self):
                if self.path == '/':
                        body = HTML_PAGE.encode('utf-8')
                        self.send_response(HTTPStatus.OK)
                        self.send_header('Content-Type', 'text/html; charset=utf-8')
                        self._send_cors_headers()
                        self.send_header('Content-Length', str(len(body)))
                        self.end_headers()
                        self._write_body_safe(body)
                        return

                if self.path == '/timelapse-directories':
                        body = TIMELAPSE_PAGE.encode('utf-8')
                        self.send_response(HTTPStatus.OK)
                        self.send_header('Content-Type', 'text/html; charset=utf-8')
                        self._send_cors_headers()
                        self.send_header('Content-Length', str(len(body)))
                        self.end_headers()
                        self._write_body_safe(body)
                        return

                if self.path.startswith('/timelapse-preview'):
                        if TIMELAPSE_PREVIEW_HTML_PATH.exists() and TIMELAPSE_PREVIEW_HTML_PATH.is_file():
                                body = TIMELAPSE_PREVIEW_HTML_PATH.read_text(encoding='utf-8').encode('utf-8')
                        else:
                                body = VIEWER_PAGE.encode('utf-8')
                        self.send_response(HTTPStatus.OK)
                        self.send_header('Content-Type', 'text/html; charset=utf-8')
                        self._send_cors_headers()
                        self.send_header('Content-Length', str(len(body)))
                        self.end_headers()
                        self._write_body_safe(body)
                        return

                if self.path.startswith('/viewer'):
                        parsed = urlparse(self.path)
                        location = '/timelapse-preview'
                        if parsed.query:
                                location += '?' + parsed.query
                        self.send_response(HTTPStatus.FOUND)
                        self.send_header('Location', location)
                        self._send_cors_headers()
                        self.end_headers()
                        return

                if self.path.startswith('/api/timelapse-directories'):
                        parsed = urlparse(self.path)
                        query = parse_qs(parsed.query)
                        view_mode = (query.get('view') or ['optimal'])[0].strip().lower()
                        if view_mode not in ('all', 'optimal'):
                                view_mode = 'optimal'
                        profile_mode = str((query.get('profile') or ['0'])[0]).strip().lower() in ('1', 'true', 'yes', 'on')
                        directories, profile_payload = list_timelapse_directories(view=view_mode, profile=profile_mode)
                        summary = {
                                'remote_dirs': sum(1 for d in directories if d.get('source') == 'remote'),
                                'local_dirs': sum(1 for d in directories if d.get('source') == 'local'),
                                'remote_items': sum(int(d.get('remote_selected_count', 0) or 0) for d in directories),
                                'local_items': sum(int(d.get('local_selected_count', 0) or 0) for d in directories),
                        }
                        payload = {
                                'ok': True,
                                'root': str(TIMELAPSE_ROOT),
                                'view': view_mode,
                                'summary': summary,
                                'directories': directories,
                                'sync': get_sync_state(),
                        }
                        if profile_mode:
                                payload['profile'] = profile_payload
                        self._send_json(payload)
                        return

                if self.path.startswith('/api/timelapse/latest-photo'):
                        parsed = urlparse(self.path)
                        query = parse_qs(parsed.query)
                        folder_name = str((query.get('folder') or [''])[0]).strip()
                        if not folder_name:
                                self._send_json({'ok': False, 'error': 'Missing folder'}, status=HTTPStatus.BAD_REQUEST)
                                return
                        if not _timelapse_name_is_valid(folder_name):
                                self._send_json({'ok': False, 'error': 'Invalid folder'}, status=HTTPStatus.BAD_REQUEST)
                                return
                        self._send_json(build_latest_photo_payload(folder_name))
                        return

                if self.path.startswith('/api/timelapse/frames'):
                        parsed = urlparse(self.path)
                        query = parse_qs(parsed.query)
                        dir_name = str((query.get('dir') or [''])[0]).strip()
                        view_mode = str((query.get('view') or ['optimal'])[0]).strip().lower()
                        if not dir_name:
                                self._send_json({'ok': False, 'error': 'Missing dir'}, status=HTTPStatus.BAD_REQUEST)
                                return
                        if view_mode not in ('optimal', 'all'):
                                view_mode = 'optimal'
                        frames = _full_frame_urls_for_timelapse_dir(dir_name, preferred_view=view_mode, limit=5000)
                        self._send_json({
                                'ok': True,
                                'dir': dir_name,
                                'view': view_mode,
                                'count': len(frames),
                                'frames': frames,
                        })
                        return

                if self.path.startswith('/api/timelapse-export-download'):
                        parsed = urlparse(self.path)
                        query = parse_qs(parsed.query)
                        file_name = str((query.get('name') or [''])[0]).strip()
                        lower_file = file_name.lower()
                        if not file_name or '/' in file_name or '\\' in file_name or not (lower_file.endswith('.mp4') or lower_file.endswith('.mov')):
                                self._send_json({'ok': False, 'error': 'Invalid export file'}, status=HTTPStatus.BAD_REQUEST)
                                return
                        file_path = (TIMELAPSE_EXPORT_DIR / file_name).resolve()
                        try:
                                file_path.relative_to(TIMELAPSE_EXPORT_DIR.resolve())
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Path out of bounds'}, status=HTTPStatus.FORBIDDEN)
                                return
                        if not file_path.exists() or not file_path.is_file():
                                self._send_json({'ok': False, 'error': 'Export file not found'}, status=HTTPStatus.NOT_FOUND)
                                return
                        try:
                                data = file_path.read_bytes()
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Failed to read export file'}, status=HTTPStatus.INTERNAL_SERVER_ERROR)
                                return
                        self.send_response(HTTPStatus.OK)
                        content_type = 'video/quicktime' if lower_file.endswith('.mov') else 'video/mp4'
                        self.send_header('Content-Type', content_type)
                        self.send_header('Cache-Control', 'no-store')
                        self.send_header('Content-Disposition', f'attachment; filename="{file_name}"')
                        self._send_cors_headers()
                        self.send_header('Content-Length', str(len(data)))
                        self.end_headers()
                        self._write_body_safe(data)
                        return

                if self.path.startswith('/api/timelapse-export-media'):
                        parsed = urlparse(self.path)
                        query = parse_qs(parsed.query)
                        file_name = str((query.get('name') or [''])[0]).strip()
                        lower_file = file_name.lower()
                        if not file_name or '/' in file_name or '\\' in file_name or not (lower_file.endswith('.mp4') or lower_file.endswith('.mov')):
                                self._send_json({'ok': False, 'error': 'Invalid export file'}, status=HTTPStatus.BAD_REQUEST)
                                return
                        file_path = (TIMELAPSE_EXPORT_DIR / file_name).resolve()
                        try:
                                file_path.relative_to(TIMELAPSE_EXPORT_DIR.resolve())
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Path out of bounds'}, status=HTTPStatus.FORBIDDEN)
                                return
                        if not file_path.exists() or not file_path.is_file():
                                self._send_json({'ok': False, 'error': 'Export file not found'}, status=HTTPStatus.NOT_FOUND)
                                return
                        try:
                                file_size = int(file_path.stat().st_size)
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Failed to read export file'}, status=HTTPStatus.INTERNAL_SERVER_ERROR)
                                return

                        content_type = 'video/quicktime' if lower_file.endswith('.mov') else 'video/mp4'
                        range_header = str(self.headers.get('Range') or '').strip()

                        start = 0
                        end = max(0, file_size - 1)
                        partial = False
                        if range_header.startswith('bytes='):
                                partial = True
                                spec = range_header[6:].strip()
                                first, _, last = spec.partition('-')
                                try:
                                        if first and last:
                                                start = int(first)
                                                end = int(last)
                                        elif first:
                                                start = int(first)
                                                end = file_size - 1
                                        elif last:
                                                suffix_len = int(last)
                                                if suffix_len <= 0:
                                                        raise ValueError('invalid suffix range')
                                                start = max(0, file_size - suffix_len)
                                                end = file_size - 1
                                        else:
                                                raise ValueError('invalid range')
                                except Exception:
                                        self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                                        self.send_header('Content-Range', f'bytes */{file_size}')
                                        self._send_cors_headers()
                                        self.end_headers()
                                        return
                                if start < 0 or end < start or start >= file_size:
                                        self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                                        self.send_header('Content-Range', f'bytes */{file_size}')
                                        self._send_cors_headers()
                                        self.end_headers()
                                        return
                                end = min(end, file_size - 1)

                        content_length = max(0, end - start + 1)
                        self.send_response(HTTPStatus.PARTIAL_CONTENT if partial else HTTPStatus.OK)
                        self.send_header('Content-Type', content_type)
                        self.send_header('Cache-Control', 'no-store')
                        self.send_header('Accept-Ranges', 'bytes')
                        self.send_header('Content-Disposition', f'inline; filename="{file_name}"')
                        if partial:
                                self.send_header('Content-Range', f'bytes {start}-{end}/{file_size}')
                        self._send_cors_headers()
                        self.send_header('Content-Length', str(content_length))
                        self.end_headers()

                        try:
                                with open(file_path, 'rb') as f:
                                        if start:
                                                f.seek(start)
                                        remaining = content_length
                                        while remaining > 0:
                                                chunk = f.read(min(64 * 1024, remaining))
                                                if not chunk:
                                                        break
                                                if not self._write_body_safe(chunk):
                                                        break
                                                remaining -= len(chunk)
                        except Exception:
                                pass
                        return

                if self.path.startswith('/api/timelapse/latest-export'):
                        parsed = urlparse(self.path)
                        query = parse_qs(parsed.query)
                        dir_name = str((query.get('dir') or [''])[0]).strip()
                        if not dir_name:
                                self._send_json({'ok': False, 'error': 'Missing dir'}, status=HTTPStatus.BAD_REQUEST)
                                return
                        result = get_latest_timelapse_export(dir_name)
                        if result.get('ok'):
                                self._send_json(result)
                        else:
                                self._send_json(result, status=HTTPStatus.BAD_REQUEST)
                        return

                parsed = urlparse(self.path)
                if parsed.path == '/api/timelapse/frame-metadata':
                        query = parse_qs(parsed.query)
                        frame_name = str((query.get('name') or [''])[0]).strip()
                        dir_name = str((query.get('dir') or [''])[0]).strip()
                        include_hist = str((query.get('hist') or ['0'])[0]).strip() in ('1', 'true', 'yes')
                        if not frame_name or not dir_name:
                                self._send_json({'available': False, 'reason': 'Missing name or dir'}, status=HTTPStatus.BAD_REQUEST)
                                return
                        payload = build_timelapse_frame_metadata_payload(dir_name, frame_name, include_histogram=include_hist)
                        self._send_json(payload)
                        return

                if self.path.startswith('/api/timelapse-preview'):
                        parsed = urlparse(self.path)
                        query = parse_qs(parsed.query)
                        requested = str((query.get('path') or [''])[0]).strip()
                        if not requested:
                                self._send_json({'error': 'Missing path'}, status=HTTPStatus.BAD_REQUEST)
                                return
                        try:
                                preview_path = Path(requested).resolve()
                        except Exception:
                                self._send_json({'error': 'Invalid path'}, status=HTTPStatus.BAD_REQUEST)
                                return
                        try:
                                preview_path.relative_to(SCRIPT_DIR.resolve())
                        except Exception:
                                self._send_json({'error': 'Path out of bounds'}, status=HTTPStatus.FORBIDDEN)
                                return
                        if not preview_path.exists() or not preview_path.is_file():
                                self._send_json({'error': 'Image not found'}, status=HTTPStatus.NOT_FOUND)
                                return
                        send_file(self, preview_path)
                        return

                if self.path.startswith('/api/timelapse-remote-preview'):
                        parsed = urlparse(self.path)
                        query = parse_qs(parsed.query)
                        requested = str((query.get('path') or [''])[0]).strip()
                        if not requested:
                                self._send_json({'error': 'Missing path'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        root_prefix = str(TIMELAPSE_ROOT).rstrip('/') + '/timelapse_'
                        if not requested.startswith(root_prefix):
                                self._send_json({'error': 'Path out of bounds'}, status=HTTPStatus.FORBIDDEN)
                                return

                        remote_q = shlex.quote(requested)
                        cmd = f"test -f {remote_q} && cat {remote_q}"
                        probe = subprocess.run(
                                [
                                        'ssh',
                                        '-o', 'BatchMode=yes',
                                        '-o', 'ConnectTimeout=12',
                                        '-o', 'IdentitiesOnly=yes',
                                        '-i', ORIN_SSH_KEY,
                                        f'{ORIN_SSH_USER}@{ORIN_SSH_HOST}',
                                        cmd,
                                ],
                                capture_output=True,
                                check=False,
                        )
                        if probe.returncode != 0 or not probe.stdout:
                                self._send_json({'error': 'Image not found'}, status=HTTPStatus.NOT_FOUND)
                                return

                        mime_type, _ = mimetypes.guess_type(requested)
                        if not mime_type:
                                mime_type = 'application/octet-stream'

                        body = probe.stdout
                        self.send_response(HTTPStatus.OK)
                        self.send_header('Content-Type', mime_type)
                        self.send_header('Cache-Control', 'no-store')
                        self._send_cors_headers()
                        self.send_header('Content-Length', str(len(body)))
                        self.end_headers()
                        self._write_body_safe(body)
                        return

                if self.path.startswith('/api/frame-metrics'):
                        parsed = urlparse(self.path)
                        query = parse_qs(parsed.query)
                        requested = str((query.get('path') or [''])[0]).strip()
                        if not requested:
                                self._send_json({'error': 'Missing path'}, status=HTTPStatus.BAD_REQUEST)
                                return
                        try:
                                frame_path = Path(requested).resolve()
                        except Exception:
                                self._send_json({'error': 'Invalid path'}, status=HTTPStatus.BAD_REQUEST)
                                return
                        try:
                                frame_path.relative_to(SCRIPT_DIR.resolve())
                        except Exception:
                                self._send_json({'error': 'Path out of bounds'}, status=HTTPStatus.FORBIDDEN)
                                return
                        if not frame_path.exists() or not frame_path.is_file():
                                self._send_json({'error': 'Image not found'}, status=HTTPStatus.NOT_FOUND)
                                return
                        try:
                                histogram = build_luma_histogram(frame_path, bins=64)
                                payload = {
                                        'ok': True,
                                        'path': str(frame_path),
                                        'histogram': histogram,
                                }
                                self._send_json(payload)
                        except Exception as e:
                                self._send_json({'error': str(e)}, status=HTTPStatus.INTERNAL_SERVER_ERROR)
                        return

                if self.path == '/api/status':
                        payload = read_status()
                        with MOVIE_STATE_LOCK:
                                payload['movie-recording'] = MOVIE_RECORDING
                        payload['autotune'] = get_autotune_state()
                        self._send_json(payload)
                        return

                if self.path.startswith('/api/autotune/latest-image'):
                        latest_path = find_latest_autotune_image_path()
                        if latest_path is None:
                                self._send_json({'error': 'Image not available'}, status=HTTPStatus.NOT_FOUND)
                                return
                        send_file(self, latest_path)
                        return

                if self.path.startswith('/api/autotune/latest'):
                        payload = get_latest_autotune_payload()
                        self._send_json(payload)
                        return

                self._send_json({'error': 'Not found'}, status=HTTPStatus.NOT_FOUND)

        def do_POST(self):
                if self.path in ('/api/timelapse/frame-metadata-batch', '/api/timelapse/frame-metadata-lite-batch'):
                        try:
                                content_length = int(self.headers.get('Content-Length', '0'))
                                raw_body = self.rfile.read(content_length)
                                body = json.loads(raw_body.decode('utf-8')) if raw_body else {}
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Invalid JSON body'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        names = body.get('names') if isinstance(body, dict) else []
                        dir_name = str((body or {}).get('dir') or '').strip()
                        include_hist = self.path.endswith('-lite-batch') or bool((body or {}).get('hist', False))

                        if not _timelapse_name_is_valid(dir_name):
                                self._send_json({'ok': False, 'error': 'Invalid dir'}, status=HTTPStatus.BAD_REQUEST)
                                return
                        if not isinstance(names, list):
                                self._send_json({'ok': False, 'error': 'names must be a list'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        items = {}
                        for raw_name in names[:250]:
                                frame_name = str(raw_name or '').strip()
                                if not _frame_name_is_valid(frame_name):
                                        continue
                                items[frame_name] = build_timelapse_frame_metadata_payload(
                                        dir_name,
                                        frame_name,
                                        include_histogram=include_hist,
                                )

                        self._send_json({'ok': True, 'dir': dir_name, 'items': items})
                        return

                if self.path == '/api/timelapse/ai-summary':
                        try:
                                content_length = int(self.headers.get('Content-Length', '0'))
                                raw_body = self.rfile.read(content_length)
                                body = json.loads(raw_body.decode('utf-8')) if raw_body else {}
                                dir_name = str(body.get('dir') or '').strip()
                                frame_name = str(body.get('name') or '').strip()
                                summary = body.get('summary')
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Invalid JSON body'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        result = append_timelapse_ai_summary(dir_name, frame_name, summary)
                        if result.get('ok'):
                                self._send_json(result)
                        else:
                                self._send_json(result, status=HTTPStatus.BAD_REQUEST)
                        return

                if self.path == '/api/timelapse/recompute-dir-stats':
                        try:
                                content_length = int(self.headers.get('Content-Length', '0'))
                                raw_body = self.rfile.read(content_length)
                                body = json.loads(raw_body.decode('utf-8')) if raw_body else {}
                                dir_name = str(body.get('dir') or '').strip()
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Invalid JSON body'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        result = recompute_timelapse_dir_stats(dir_name)
                        if result.get('ok'):
                                self._send_json(result)
                        else:
                                self._send_json(result, status=HTTPStatus.BAD_REQUEST)
                        return

                if self.path == '/api/timelapse/delete-dir':
                        try:
                                content_length = int(self.headers.get('Content-Length', '0'))
                                raw_body = self.rfile.read(content_length)
                                body = json.loads(raw_body.decode('utf-8')) if raw_body else {}
                                dir_name = str(body.get('dir') or '').strip()
                                source_hint = str(body.get('source') or '').strip().lower()
                                confirm_text = str(body.get('confirm_text') or '').strip().lower()
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Invalid JSON body'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        if confirm_text != 'delete':
                                self._send_json({'ok': False, 'error': "Type 'delete' to confirm"}, status=HTTPStatus.BAD_REQUEST)
                                return

                        result = delete_local_timelapse_dir(dir_name, source=source_hint)
                        if result.get('ok'):
                                self._send_json(result)
                        else:
                                self._send_json(result, status=HTTPStatus.BAD_REQUEST)
                        return

                if self.path == '/api/timelapse/delete-frame':
                        try:
                                content_length = int(self.headers.get('Content-Length', '0'))
                                raw_body = self.rfile.read(content_length)
                                body = json.loads(raw_body.decode('utf-8')) if raw_body else {}
                                dir_name = str(body.get('dir') or '').strip()
                                frame_name = str(body.get('name') or '').strip()
                                confirm_text = str(body.get('confirm_text') or '').strip().lower()
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Invalid JSON body'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        if confirm_text != 'delete':
                                self._send_json({'ok': False, 'error': "Type 'delete' to confirm"}, status=HTTPStatus.BAD_REQUEST)
                                return

                        result = delete_local_timelapse_frame(dir_name, frame_name)
                        if result.get('ok'):
                                self._send_json(result)
                        else:
                                self._send_json(result, status=HTTPStatus.BAD_REQUEST)
                        return

                if self.path == '/api/timelapse/delete-frame-range':
                        try:
                                content_length = int(self.headers.get('Content-Length', '0'))
                                raw_body = self.rfile.read(content_length)
                                body = json.loads(raw_body.decode('utf-8')) if raw_body else {}
                                dir_name = str(body.get('dir') or '').strip()
                                start_name = str(body.get('start_name') or '').strip()
                                end_name = str(body.get('end_name') or '').strip()
                                view_mode = str(body.get('view') or 'optimal').strip().lower()
                                confirm_text = str(body.get('confirm_text') or '').strip().lower()
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Invalid JSON body'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        if confirm_text != 'delete':
                                self._send_json({'ok': False, 'error': "Type 'delete' to confirm"}, status=HTTPStatus.BAD_REQUEST)
                                return

                        result = delete_local_timelapse_frame_range(dir_name, start_name, end_name, view=view_mode)
                        if result.get('ok'):
                                self._send_json(result)
                        else:
                                self._send_json(result, status=HTTPStatus.BAD_REQUEST)
                        return

                if self.path == '/api/trigger':
                        ok, error = trigger_capture()
                        if ok:
                                self._send_json({'ok': True})
                        else:
                                self._send_json({'ok': False, 'error': error}, status=HTTPStatus.BAD_REQUEST)
                        return

                if self.path == '/api/movie':
                        try:
                                content_length = int(self.headers.get('Content-Length', '0'))
                                raw_body = self.rfile.read(content_length)
                                body = json.loads(raw_body.decode('utf-8')) if raw_body else {}
                                action = str(body.get('action', 'toggle')).strip().lower()
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Invalid JSON body'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        with MOVIE_STATE_LOCK:
                                current_state = MOVIE_RECORDING

                        if action == 'start':
                                result = set_movie_recording(True)
                        elif action == 'stop':
                                result = set_movie_recording(False)
                        else:
                                result = set_movie_recording(not current_state)

                        if result.get('ok'):
                                self._send_json({'ok': True, 'recording': result.get('recording', False)})
                        else:
                                self._send_json(result, status=HTTPStatus.BAD_REQUEST)
                        return

                if self.path == '/api/autotune':
                        try:
                                content_length = int(self.headers.get('Content-Length', '0'))
                                raw_body = self.rfile.read(content_length)
                                body = json.loads(raw_body.decode('utf-8')) if raw_body else {}
                                action = str(body.get('action', 'toggle')).strip().lower()
                                startup_iso = parse_numeric_value(body.get('iso'), cast=int)
                                startup_fstop = parse_numeric_value(body.get('fstop'), cast=float)
                                startup_shutter = str(body.get('shutter', '')).strip() or None
                                lock_iso = bool(body.get('lock_iso', False))
                                lock_shutter = bool(body.get('lock_shutter', False))
                                lock_fstop = bool(body.get('lock_fstop', False))
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Invalid JSON body'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        state = get_autotune_state()
                        lock_kwargs = dict(lock_iso=lock_iso, lock_shutter=lock_shutter, lock_aperture=lock_fstop)
                        if action == 'start':
                                result = start_autotune(startup_iso=startup_iso, startup_fstop=startup_fstop, startup_shutter=startup_shutter, **lock_kwargs)
                        elif action == 'stop':
                                result = stop_autotune()
                        else:
                                result = stop_autotune() if state.get('running') else start_autotune(startup_iso=startup_iso, startup_fstop=startup_fstop, startup_shutter=startup_shutter, **lock_kwargs)

                        if result.get('ok'):
                                self._send_json(result)
                        else:
                                self._send_json(result, status=HTTPStatus.BAD_REQUEST)
                        return

                if self.path == '/api/set':
                        try:
                                content_length = int(self.headers.get('Content-Length', '0'))
                                raw_body = self.rfile.read(content_length)
                                body = json.loads(raw_body.decode('utf-8'))
                                setting = str(body.get('setting', '')).strip()
                                value = str(body.get('value', '')).strip()
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Invalid JSON body'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        if setting not in CAMERA_WRITABLE_SETTINGS:
                                self._send_json({'ok': False, 'error': 'Unsupported setting'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        if not value:
                                self._send_json({'ok': False, 'error': 'Value is required'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        result = set_camera_setting(CAMERA_STATUS_SETTINGS[setting], value)
                        if result.get('ok'):
                                invalidate_status_cache()
                                self._send_json({'ok': True})
                        else:
                                self._send_json(result, status=HTTPStatus.BAD_REQUEST)
                        return

                if self.path == '/api/timelapse-sync':
                        try:
                                content_length = int(self.headers.get('Content-Length', '0'))
                                raw_body = self.rfile.read(content_length)
                                body = json.loads(raw_body.decode('utf-8')) if raw_body else {}
                                action = str(body.get('action', 'start')).strip().lower()
                                folder = str(body.get('folder', '')).strip()
                                view_mode = str(body.get('view', 'optimal')).strip().lower()
                                keep_up_to_date = bool(body.get('keep_up_to_date', False))
                                interval_seconds = int(body.get('interval_seconds', 30))
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Invalid JSON body'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        if action == 'stop':
                                result = stop_timelapse_sync()
                        else:
                                result = start_timelapse_sync(
                                        folder,
                                        view=view_mode,
                                        watch=keep_up_to_date,
                                        interval_seconds=interval_seconds,
                                )
                        if result.get('ok'):
                                self._send_json(result)
                        else:
                                self._send_json(result, status=HTTPStatus.BAD_REQUEST)
                        return

                if self.path == '/api/timelapse/export-video':
                        try:
                                content_length = int(self.headers.get('Content-Length', '0'))
                                raw_body = self.rfile.read(content_length)
                                body = json.loads(raw_body.decode('utf-8')) if raw_body else {}
                                dir_name = str(body.get('dir') or '').strip()
                                view_mode = str(body.get('view') or 'optimal').strip().lower()
                                fps = int(body.get('fps') or 12)
                                resolution = str(body.get('resolution') or 'source').strip().lower()
                                quality = str(body.get('quality') or 'medium').strip().lower()
                                format_mode = str(body.get('format') or 'mp4').strip().lower()
                                max_frames_raw = body.get('max_frames')
                                if max_frames_raw is None:
                                        max_frames = None
                                else:
                                        max_frames_text = str(max_frames_raw).strip().lower()
                                        if max_frames_text in ('', 'all', 'none', '0', '-1'):
                                                max_frames = None
                                        else:
                                                max_frames = int(max_frames_raw)
                                hide_banner = bool(body.get('hide_banner', True))
                                loglevel = str(body.get('loglevel') or 'error').strip().lower()
                                concat_safe = int(body.get('concat_safe', 0) or 0)
                                codec = str(body.get('codec') or 'libx264').strip().lower()
                                preset = str(body.get('preset') or '').strip().lower() or None
                                crf = int(body.get('crf')) if body.get('crf') is not None else None
                                pix_fmt = str(body.get('pix_fmt') or 'yuv420p').strip().lower()
                                faststart = bool(body.get('faststart', True))
                                name_tag = str(body.get('name_tag') or '').strip() or None
                                start_pct_raw = body.get('start_pct')
                                end_pct_raw = body.get('end_pct')
                                start_pct = float(start_pct_raw) if start_pct_raw is not None else None
                                end_pct = float(end_pct_raw) if end_pct_raw is not None else None
                        except Exception:
                                self._send_json({'ok': False, 'error': 'Invalid JSON body'}, status=HTTPStatus.BAD_REQUEST)
                                return

                        result = export_timelapse_video(
                                dir_name,
                                view=view_mode,
                                fps=fps,
                                resolution=resolution,
                                quality=quality,
                                fmt=format_mode,
                                max_frames=max_frames,
                                hide_banner=hide_banner,
                                loglevel=loglevel,
                                concat_safe=concat_safe,
                                codec=codec,
                                preset=preset,
                                crf=crf,
                                pix_fmt=pix_fmt,
                                faststart=faststart,
                                name_tag=name_tag,
                                start_pct=start_pct,
                                end_pct=end_pct,
                        )
                        if result.get('ok'):
                                self._send_json(result)
                        else:
                                self._send_json(result, status=HTTPStatus.BAD_REQUEST)
                        return

                self._send_json({'error': 'Not found'}, status=HTTPStatus.NOT_FOUND)

        def log_message(self, _format, *args):
                # Keep HTTP request logs concise.
                return


def run_http_server():
        server = ThreadingHTTPServer(('0.0.0.0', HTTP_PORT), CameraHTTPHandler)
        print(f'Web UI available at http://0.0.0.0:{HTTP_PORT}')
        server.serve_forever()


def run_udp_server():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', UDP_PORT))

        print(f'Camera trigger server listening on UDP port {UDP_PORT}...')
        print('Waiting for trigger commands from ESP32...')

        while True:
                try:
                        data, addr = sock.recvfrom(1024)
                        cmd = data.decode('utf-8').strip()

                        if cmd == 'TRIGGER':
                                print(f'Trigger received from {addr[0]} - firing camera...')
                                ok, error = trigger_capture()
                                if ok:
                                        print('Photo triggered successfully')
                                else:
                                        print(f'Error: {error}')

                        elif cmd == 'PING':
                                sock.sendto(b'PONG', addr)
                                print(f'Ping from {addr[0]}')

                except Exception as e:
                        print(f'Error: {e}')


if __name__ == '__main__':
        # Start background daemon suppression to prevent system daemons from reclaiming USB
        def suppress_daemons():
                while True:
                        for daemon in ['icdd', 'ptpcamerad', 'mscamerad']:
                                subprocess.run(
                                        ['pkill', '-9', daemon],
                                        stdout=subprocess.DEVNULL,
                                        stderr=subprocess.DEVNULL,
                                )
                        time.sleep(0.2)  # Keep daemons suppressed every 0.2s
        
        daemon_suppressor = threading.Thread(target=suppress_daemons, daemon=True)
        daemon_suppressor.start()
        
        http_thread = threading.Thread(target=run_http_server, daemon=True)
        http_thread.start()
        run_udp_server()