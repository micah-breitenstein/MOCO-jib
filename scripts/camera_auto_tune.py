#!/usr/bin/env python3
"""
Capture -> analyze -> auto-adjust loop using gphoto2 and camera_ai_analyzer.

Workflow per iteration:
1) Capture and download one frame via gphoto2 --capture-image-and-download --keep
2) Resolve downloaded filename
3) Analyze frame and compute recommended next settings
4) Apply gphoto2 --set-config changes
5) Stop when exposure converges, otherwise repeat
"""

from __future__ import annotations

import argparse
import csv
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Sequence, Tuple

from camera_ai_analyzer import (
    DEFAULT_FSTOP_STOPS,
    DEFAULT_ISO_STOPS,
    DEFAULT_SHUTTER_STOPS,
    AnalysisMetrics,
    analyze_image,
    format_shutter,
    parse_shutter,
    parse_numeric_list,
    read_capture_settings_from_exif,
    recommend_settings,
)


def wait_for_file_stable(path: Path, stable_for_s: float = 1.5, max_wait_s: float = 120.0, poll_interval_s: float = 0.5) -> bool:
    """Wait until a file exists and its size has not changed for stable_for_s seconds.
    Returns True when stable, False if max_wait_s exceeded."""
    deadline = time.time() + max_wait_s
    last_size = -1
    stable_since: Optional[float] = None
    while time.time() < deadline:
        try:
            size = path.stat().st_size
        except FileNotFoundError:
            time.sleep(poll_interval_s)
            continue
        if size != last_size:
            last_size = size
            stable_since = time.time()
        elif stable_since is not None and (time.time() - stable_since) >= stable_for_s:
            return True
        time.sleep(poll_interval_s)
    return False


def wait_for_camera_ready(max_wait_s: float = 120.0, poll_interval_s: float = 2.0) -> bool:
    """Poll gphoto2 with a trivial get-config until the camera responds cleanly.
    Returns True when ready, False if max_wait_s exceeded."""
    deadline = time.time() + max_wait_s
    while time.time() < deadline:
        try:
            result = subprocess.run(
                ["gphoto2", "--get-config", "batterylevel"],
                capture_output=True, text=True, timeout=10, check=False,
            )
            returncode = result.returncode
            combined = (result.stdout + result.stderr).lower()
        except subprocess.TimeoutExpired:
            returncode = -1
            combined = "timeout"
        if returncode == 0:
            return True
        # Camera still busy with NR, writing, or probe timed out - keep waiting
        if any(m in combined for m in ("busy", "unavailable", "could not claim", "io-library", "ptp", "timeout")):
            time.sleep(poll_interval_s)
            continue
        # Unexpected error - stop waiting
        return False
    return False


def run_gphoto(args: Sequence[str], timeout: int = 45) -> subprocess.CompletedProcess[str]:
    cmd = ["gphoto2", *args]
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired as exc:
        timeout_msg = f"gphoto2 command timed out after {timeout}s: {' '.join(cmd)}"
        stdout_text = exc.stdout.decode(errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
        stderr_text = exc.stderr.decode(errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or "")
        combined_stderr = "\n".join(part for part in [stderr_text, timeout_msg] if part).strip()
        return subprocess.CompletedProcess(cmd, returncode=124, stdout=stdout_text, stderr=combined_stderr)


def parse_downloaded_filename(output_text: str) -> Optional[Path]:
    # gphoto2 typically emits lines like:
    # "Saving file as capt_MJB09396.JPG"
    match = re.search(r"Saving file as\s+(.+)$", output_text, flags=re.MULTILINE)
    if not match:
        return None
    raw_name = match.group(1).strip().strip('"')
    return Path(raw_name)


def newest_image_in_dir(workdir: Path) -> Optional[Path]:
    exts = {".jpg", ".jpeg", ".png", ".arw", ".cr2", ".cr3", ".nef", ".dng"}
    candidates = [p for p in workdir.iterdir() if p.is_file() and p.suffix.lower() in exts]
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


SHUTTER_CHOICES_CACHE: Optional[list[tuple[str, float]]] = None


def load_shutter_choices() -> list[tuple[str, float]]:
    global SHUTTER_CHOICES_CACHE
    if SHUTTER_CHOICES_CACHE is not None:
        return SHUTTER_CHOICES_CACHE

    result = run_gphoto(["--get-config", "shutterspeed"], timeout=20)
    if result.returncode != 0:
        SHUTTER_CHOICES_CACHE = []
        return SHUTTER_CHOICES_CACHE

    choices: list[tuple[str, float]] = []
    for line in result.stdout.splitlines():
        match = re.match(r"^Choice:\s+\d+\s+(.+)$", line.strip())
        if not match:
            continue
        raw_choice = match.group(1).strip()
        try:
            parsed_choice = parse_shutter(raw_choice)
        except Exception:
            continue
        choices.append((raw_choice, parsed_choice))

    SHUTTER_CHOICES_CACHE = choices
    return SHUTTER_CHOICES_CACHE


def resolve_shutter_choice(value: str) -> str:
    try:
        target_seconds = parse_shutter(value)
    except Exception:
        return value

    choices = load_shutter_choices()
    if not choices:
        return value

    best_label, best_seconds = min(
        choices,
        key=lambda item: abs(item[1] - target_seconds),
    )
    tolerance = max(1e-6, target_seconds * 0.02)
    if abs(best_seconds - target_seconds) <= tolerance:
        return best_label
    return value


def apply_setting(name: str, value: str, retries: int = 6, retry_seconds: float = 1.5) -> Tuple[bool, str]:
    attempt = 0
    max_attempts = max(1, int(retries) + 1)
    last_message = ""

    while attempt < max_attempts:
        attempt += 1

        # Some cameras expose settings as temporarily read-only while they are
        # still flushing buffers or finishing in-camera processing.
        wait_for_camera_ready(max_wait_s=15.0, poll_interval_s=1.0)

        requested_value = resolve_shutter_choice(value) if name == "shutterspeed" else value
        result = run_gphoto(["--set-config", f"{name}={requested_value}"], timeout=20)
        combined = (result.stdout + "\n" + result.stderr).strip()
        if result.returncode == 0:
            return True, combined

        last_message = combined
        combined_l = combined.lower()
        read_only = ("property" in combined_l and "read only" in combined_l)
        transient = (
            is_transient_capture_error(combined)
            or ("could not claim the usb device" in combined_l)
            or read_only
            or ("busy" in combined_l)
            or ("unavailable" in combined_l)
        )
        if read_only and attempt < max_attempts:
            # Camera accepted capture but is still locking writable controls.
            # Back off longer before retrying the same setting.
            extra_wait = min(45.0, 8.0 * attempt)
            print(
                f"{name} is temporarily read-only (attempt {attempt}/{max_attempts}); "
                f"waiting {extra_wait:.1f}s before retry."
            )
            wait_for_camera_ready(max_wait_s=extra_wait + 15.0, poll_interval_s=1.0)
            time.sleep(extra_wait)
            continue
        if transient and attempt < max_attempts:
            transient_wait = min(20.0, max(1.0, float(retry_seconds)) * (2 ** (attempt - 1)))
            print(
                f"{name} set transient failure (attempt {attempt}/{max_attempts}); "
                f"waiting {transient_wait:.1f}s before retry."
            )
            wait_for_camera_ready(max_wait_s=20.0, poll_interval_s=1.0)
            time.sleep(transient_wait)
            continue

        break

    return False, last_message


def converged(metrics: AnalysisMetrics, deadband_ev: float) -> bool:
    return (
        abs(metrics.delta_ev) <= deadband_ev
        and metrics.highlight_clip_pct <= 1.0
        and metrics.shadow_clip_pct <= 2.0
    )


def append_reject_log(
    log_path: Path,
    image_name: str,
    metrics: AnalysisMetrics,
    current_iso: int,
    current_shutter_s: float,
    current_fstop: float,
    note: str,
) -> None:
    is_new = not log_path.exists()
    with log_path.open("a", newline="", encoding="utf-8") as fp:
        writer = csv.writer(fp)
        if is_new:
            writer.writerow(
                [
                    "timestamp",
                    "image",
                    "iso",
                    "shutter",
                    "fstop",
                    "delta_ev",
                    "median_luma",
                    "highlight_clip_pct",
                    "shadow_clip_pct",
                    "blur_score",
                    "note",
                ]
            )
        writer.writerow(
            [
                datetime.now().isoformat(timespec="seconds"),
                image_name,
                current_iso,
                format_shutter(current_shutter_s),
                current_fstop,
                f"{metrics.delta_ev:.4f}",
                f"{metrics.median_luma:.4f}",
                f"{metrics.highlight_clip_pct:.4f}",
                f"{metrics.shadow_clip_pct:.4f}",
                f"{metrics.blur_score:.4f}",
                note,
            ]
        )


def copy_keeper_image(image_path: Path, keep_dir: Path, tag: Optional[str] = None) -> Path:
    keep_dir.mkdir(parents=True, exist_ok=True)
    stem = image_path.stem
    if tag:
        clean_tag = str(tag).strip().replace(" ", "_")
        if clean_tag and not stem.endswith(f"_{clean_tag}"):
            stem = f"{stem}_{clean_tag}"
    dst = keep_dir / f"{stem}{image_path.suffix}"
    if dst.exists():
        suffix = image_path.suffix
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        dst = keep_dir / f"{stem}_{timestamp}{suffix}"
    shutil.copy2(image_path, dst)
    return dst


def is_transient_capture_error(output_text: str) -> bool:
    text = (output_text or "").lower()
    markers = [
        "ptp timeout",
        "timeout reading from or writing to the port",
        "could not capture image",
        "could not capture",
        "io-library",
    ]
    return any(marker in text for marker in markers)


def is_read_only_config_error(output_text: str, config_name: Optional[str] = None) -> bool:
    text = (output_text or "").lower()
    if "read only" not in text:
        return False
    if config_name is None:
        return True
    return config_name.lower() in text


def next_stop_value(current: float, stops: Sequence[float], brighten: bool) -> Optional[float]:
    ordered = sorted({float(v) for v in stops})
    if brighten:
        for value in ordered:
            if value > current * (1.0 + 1e-9):
                return value
        return None
    for value in reversed(ordered):
        if value < current * (1.0 - 1e-9):
            return value
    return None


def stepped_stop_value(current: float, stops: Sequence[float], brighten: bool, steps: int) -> Optional[float]:
    """Move by up to `steps` stop entries in the requested direction."""
    steps = max(1, int(steps))
    value = current
    moved = False
    for _ in range(steps):
        nxt = next_stop_value(value, stops, brighten=brighten)
        if nxt is None:
            break
        value = nxt
        moved = True
    if not moved:
        return None
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Auto-tune camera exposure using iterative captures.")
    parser.add_argument("--workdir", default=".", help="Directory where captures are downloaded.")
    parser.add_argument("--max-iterations", type=int, default=6, help="Max capture/adjust iterations.")
    parser.add_argument(
        "--keep-on-camera",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Keep each captured image on camera storage after download.",
    )
    parser.add_argument(
        "--capture-timeout",
        type=int,
        default=90,
        help="Timeout in seconds for each gphoto2 capture/download command.",
    )
    parser.add_argument("--duration-minutes", type=float, default=None, help="Optional run duration in minutes.")
    parser.add_argument(
        "--interval-seconds",
        type=int,
        default=0,
        help="Seconds between captures. Any value > 0 runs timed mode continuously (unless --duration-minutes is set).",
    )
    parser.add_argument(
        "--startup-tune-shot",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use the first timed capture as a calibration frame and do not save it to keep-dir.",
    )
    parser.add_argument(
        "--startup-max-iterations",
        type=int,
        default=0,
        help="Maximum fast calibration attempts at launch before switching cadence (0 = unlimited until calibrated).",
    )
    parser.add_argument(
        "--startup-max-step-ev",
        type=float,
        default=1.0,
        help="Max EV step during startup calibration mode for faster convergence.",
    )
    parser.add_argument(
        "--startup-priority",
        choices=["low-iso", "fast-brighten", "star-hunt"],
        default="low-iso",
        help=(
            "Startup tuning priority: 'low-iso' keeps ISO as low as possible and adjusts shutter first; "
            "'fast-brighten' raises ISO first to converge faster in very dark scenes; "
            "'star-hunt' jumps to a long shutter first, then sweeps aperture before ISO."
        ),
    )
    parser.add_argument(
        "--star-hunt-aperture-jump-stops",
        type=int,
        default=2,
        help="Aperture stop entries to jump per brighten step in startup-priority=star-hunt.",
    )
    parser.add_argument(
        "--startup-retry-seconds",
        type=float,
        default=5.0,
        help="Seconds between startup calibration attempts when frame is not yet optimal.",
    )
    parser.add_argument(
        "--capture-retry-seconds",
        type=float,
        default=5.0,
        help="Seconds to wait before retrying after a transient capture transport error.",
    )
    parser.add_argument(
        "--max-consecutive-capture-failures",
        type=int,
        default=6,
        help="Abort after this many consecutive transient capture failures.",
    )
    parser.add_argument(
        "--reject-log",
        default="reject_candidates.csv",
        help="CSV file to log frames that are not yet optimal.",
    )
    parser.add_argument(
        "--keep-dir",
        default="timelapse",
        help="Directory where optimal keeper frames are copied.",
    )
    parser.add_argument(
        "--keep-mode",
        choices=["optimal", "all", "best-effort"],
        default="optimal",
        help=(
            "Frame copy policy for keep-dir: "
            "'optimal' copies only converged frames, "
            "'all' copies every frame, "
            "'best-effort' guarantees one saved keeper per timed interval "
            "and tags non-optimal frames as fallback."
        ),
    )

    parser.add_argument("--target-luma", type=float, default=118.0)
    parser.add_argument("--deadband-ev", type=float, default=0.15)
    parser.add_argument(
        "--startup-deadband-ev",
        type=float,
        default=0.20,
        help="Wider EV deadband used only during startup calibration to avoid oscillation.",
    )
    parser.add_argument("--max-step-ev", type=float, default=1.0)
    parser.add_argument(
        "--aggressive-breakout-ev",
        type=float,
        default=1.0,
        help="If |deltaEV| exceeds this, use aggressive EV step for faster convergence.",
    )
    parser.add_argument(
        "--aggressive-max-step-ev",
        type=float,
        default=1.0,
        help="Max EV step while in aggressive convergence mode.",
    )
    parser.add_argument(
        "--settled-max-step-ev",
        type=float,
        default=0.125,
        help="Max EV step after calibration has settled; lower values reduce visible brightness jumps.",
    )
    parser.add_argument(
        "--settled-breakout-ev",
        type=float,
        default=0.5,
        help=(
            "If |deltaEV| exceeds this in settled mode, temporarily bypass smoothing "
            "and use normal max-step-ev to recover from rapid light changes."
        ),
    )
    parser.add_argument("--blur-min", type=float, default=120.0)
    parser.add_argument(
        "--iso-strategy",
        choices=["last", "first-when-blur-low"],
        default="last",
        help="ISO adjustment strategy. 'last' minimizes ISO changes for timelapse.",
    )

    parser.add_argument("--shutter-min", default="1/1000")
    parser.add_argument("--shutter-max", default="1/30")
    parser.add_argument("--iso-min", type=int, default=100)
    parser.add_argument("--iso-max", type=int, default=6400)
    parser.add_argument("--lock-aperture", action="store_true", default=False)
    parser.add_argument(
        "--startup-set-baseline",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Set startup baseline camera values before first capture (lowest ISO and max f-stop by default).",
    )
    parser.add_argument(
        "--startup-iso",
        type=int,
        default=None,
        help="Optional startup ISO override. Defaults to --iso-min.",
    )
    parser.add_argument(
        "--startup-fstop",
        type=float,
        default=None,
        help="Optional startup f-stop override. Defaults to maximum value in --fstop-stops.",
    )
    parser.add_argument(
        "--startup-shutter",
        default=None,
        help="Optional startup shutter override (e.g. 1/30).",
    )

    parser.add_argument("--iso-stops", default=None)
    parser.add_argument("--shutter-stops", default=None)
    parser.add_argument("--fstop-stops", default=None)
    return parser


def main() -> int:
    args = build_parser().parse_args()

    workdir = Path(args.workdir).expanduser().resolve()
    if not workdir.exists() or not workdir.is_dir():
        print(f"error: invalid --workdir: {workdir}", file=sys.stderr)
        return 2

    try:
        shutter_min_s = parse_shutter(args.shutter_min)
        shutter_max_s = parse_shutter(args.shutter_max)
    except Exception as exc:
        print(f"error: invalid shutter bounds: {exc}", file=sys.stderr)
        return 2

    if shutter_min_s > shutter_max_s:
        print("error: --shutter-min must be faster/smaller than --shutter-max", file=sys.stderr)
        return 2

    iso_stops_override = parse_numeric_list(args.iso_stops, as_int=True)
    shutter_stops_override = parse_numeric_list(args.shutter_stops, as_int=False)
    fstop_stops_override = parse_numeric_list(args.fstop_stops, as_int=False)

    iso_stops = iso_stops_override if iso_stops_override else [float(v) for v in DEFAULT_ISO_STOPS]
    shutter_stops = shutter_stops_override if shutter_stops_override else list(DEFAULT_SHUTTER_STOPS)
    fstop_stops = fstop_stops_override if fstop_stops_override else list(DEFAULT_FSTOP_STOPS)
    fstop_stops = sorted({float(f) for f in fstop_stops if float(f) >= 1.4})
    if not fstop_stops:
        print("error: no usable f-stop values remain after filtering lens minimum", file=sys.stderr)
        return 2

    if args.startup_set_baseline:
        startup_iso = args.startup_iso if args.startup_iso is not None else int(args.iso_min)
        # Default: widest aperture (min f-stop) and longest shutter for best light gathering.
        startup_fstop = args.startup_fstop if args.startup_fstop is not None else float(min(fstop_stops))
        startup_shutter = args.startup_shutter if args.startup_shutter is not None else format_shutter(shutter_max_s)

        print(f"Applying startup baseline: ISO={startup_iso}, shutter={startup_shutter}, f/{startup_fstop}")
        ok, msg = apply_setting("iso", str(startup_iso))
        print("Set startup ISO:", "ok" if ok else "failed")
        if not ok and msg:
            print(msg)

        ok, msg = apply_setting("f-number", f"{startup_fstop}")
        print("Set startup f-stop:", "ok" if ok else "failed")
        if not ok and msg:
            print(msg)

        ok, msg = apply_setting("shutterspeed", str(startup_shutter))
        print("Set startup shutter:", "ok" if ok else "failed")
        if not ok and msg:
            print(msg)

    last_iso: Optional[int] = None
    last_shutter_s: Optional[float] = None
    last_fstop: Optional[float] = None
    last_recommendation: Optional[tuple[int, float, float]] = None
    repeated_recommendation_count = 0
    # Oscillation detection: track last N (iso, shutter, fstop) tuples applied.
    settings_history: list = []
    OSCILLATION_WINDOW = 4
    reject_log_path = (workdir / args.reject_log).resolve()
    # Auto-generate a timestamped keep-dir nested inside the timelapse root so
    # it appears in the timelapse-directories UI.
    # Structure: timelapse/timelapse_YYYYMMDD_HHMMSS/optimal/
    keep_dir_name = args.keep_dir
    if keep_dir_name == "timelapse":
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        keep_dir_path = (workdir / "timelapse" / f"timelapse_{ts}" / "optimal").resolve()
    else:
        keep_dir_path = (workdir / keep_dir_name).resolve()

    calibration_only_mode = args.duration_minutes is not None and args.duration_minutes <= 0
    timed_mode = args.interval_seconds > 0 or (args.duration_minutes is not None and args.duration_minutes > 0)
    if args.duration_minutes is not None and args.duration_minutes > 0:
        end_time = time.time() + (args.duration_minutes * 60.0)
    else:
        end_time = None
    startup_tune_pending = calibration_only_mode or (timed_mode and bool(args.startup_tune_shot))
    startup_attempts = 0
    consecutive_capture_failures = 0
    settled_mode = False
    best_effort_mode = args.keep_mode == "best-effort"
    pre_capture_busy_failures = 0
    last_capture_fingerprint: Optional[tuple[str, int, int]] = None
    stale_capture_retries = 0

    next_capture_at = time.time()
    iteration = 0
    first_capture_pending = True
    print("Starting auto-tune loop...")
    while True:
        if end_time is not None and time.time() >= end_time:
            print("Run duration reached. Stopping.")
            break

        if not timed_mode and not calibration_only_mode and iteration >= args.max_iterations:
            print("Reached max iterations. More adjustment may still be needed.")
            break

        now = time.time()
        if not first_capture_pending and now < next_capture_at:
            wait_s = next_capture_at - now
            print(f"Waiting {wait_s:.1f}s before next capture...")
            time.sleep(wait_s)

        # Guard capture start: if the camera is still flushing long-exposure
        # buffers or in-camera processing, do not trigger a new frame yet.
        if last_shutter_s is not None and last_shutter_s > 1.0:
            pre_cap_wait = max(8.0, last_shutter_s + 8.0)
            if not wait_for_camera_ready(max_wait_s=pre_cap_wait, poll_interval_s=1.0):
                pre_capture_busy_failures += 1
                if pre_capture_busy_failures >= 3:
                    print(
                        "Camera readiness probe failed repeatedly, but this can be a false busy state. "
                        "Attempting capture anyway."
                    )
                    pre_capture_busy_failures = 0
                else:
                    retry_wait = max(3.0, float(args.capture_retry_seconds))
                    print(
                        f"Camera still busy before capture after {pre_cap_wait:.0f}s; "
                        f"retrying in {retry_wait:.1f}s."
                    )
                    next_capture_at = time.time() + retry_wait
                    first_capture_pending = False
                    continue
            else:
                pre_capture_busy_failures = 0

        iteration += 1
        if timed_mode:
            print(f"\nIteration {iteration} (timed mode)")
        else:
            print(f"\nIteration {iteration}/{args.max_iterations}")

        before_capture_files = {p.name for p in workdir.iterdir() if p.is_file()}
        print("Starting capture...")
        capture_args = ["--capture-image-and-download"]
        if args.keep_on_camera:
            capture_args.append("--keep")
        cap = run_gphoto(capture_args, timeout=args.capture_timeout)
        first_capture_pending = False
        cap_output = (cap.stdout + "\n" + cap.stderr).strip()
        if cap.returncode != 0:
            print("Capture failed:")
            print(cap_output)
            if is_transient_capture_error(cap_output):
                consecutive_capture_failures += 1
                if args.max_consecutive_capture_failures > 0 and consecutive_capture_failures >= args.max_consecutive_capture_failures:
                    print(
                        "error: too many consecutive transient capture failures; aborting.",
                        file=sys.stderr,
                    )
                    return 1
                wait_s = max(0.0, float(args.capture_retry_seconds))
                next_capture_at = time.time() + wait_s
                print(f"Transient camera transport error. Retrying capture in {wait_s:.1f}s...")
                continue
            return 1

        consecutive_capture_failures = 0

        image_path = parse_downloaded_filename(cap_output)
        if image_path is None:
            after_capture_files = [p for p in workdir.iterdir() if p.is_file()]
            new_candidates = [
                p for p in after_capture_files
                if p.name not in before_capture_files
                and p.suffix.lower() in {".jpg", ".jpeg", ".png", ".arw", ".cr2", ".cr3", ".nef", ".dng"}
            ]
            if new_candidates:
                image_path = max(new_candidates, key=lambda p: p.stat().st_mtime)
            else:
                image_path = newest_image_in_dir(workdir)
        elif not image_path.is_absolute():
            image_path = workdir / image_path

        if image_path is None or not image_path.exists():
            print("error: could not resolve downloaded filename", file=sys.stderr)
            print("gphoto2 output:")
            print(cap_output)
            if is_transient_capture_error(cap_output):
                consecutive_capture_failures += 1
                if args.max_consecutive_capture_failures > 0 and consecutive_capture_failures >= args.max_consecutive_capture_failures:
                    print(
                        "error: too many consecutive transient capture failures; aborting.",
                        file=sys.stderr,
                    )
                    return 1
                wait_s = max(0.0, float(args.capture_retry_seconds))
                next_capture_at = time.time() + wait_s
                print(f"Transient capture/transfer error. Retrying in {wait_s:.1f}s...")
                continue
            return 1

        print(f"Captured: {image_path.name}")

        # Wait until the downloaded file has fully landed on disk (size stable
        # for 1.5s). This covers USB transfer completion.
        max_file_wait = max(10.0, (last_shutter_s or 0) * 2.0 + 10.0)
        stable = wait_for_file_stable(image_path, stable_for_s=1.5, max_wait_s=max_file_wait)
        if not stable:
            print(f"File did not stabilise in {max_file_wait:.0f}s, proceeding anyway.")

        # Guard against stale frame reuse: if the downloaded file is identical
        # to the previous iteration (same name/size/mtime), retry capture
        # instead of re-analyzing old pixels.
        try:
            st = image_path.stat()
            capture_fingerprint = (image_path.name, int(st.st_size), int(st.st_mtime_ns))
        except FileNotFoundError:
            capture_fingerprint = None

        use_last_applied_settings = False
        if capture_fingerprint is not None and capture_fingerprint == last_capture_fingerprint:
            stale_capture_retries += 1
            if stale_capture_retries < 3:
                retry_wait = max(3.0, float(args.capture_retry_seconds))
                print(
                    "Downloaded frame appears unchanged from previous capture; "
                    f"retrying capture in {retry_wait:.1f}s."
                )
                next_capture_at = time.time() + retry_wait
                continue
            print(
                "Downloaded frame is still unchanged after repeated retries; "
                "proceeding with analysis and using last applied camera settings."
            )
            stale_capture_retries = 0
            use_last_applied_settings = True
        else:
            stale_capture_retries = 0

        if capture_fingerprint is not None:
            last_capture_fingerprint = capture_fingerprint

        # For long exposures, also wait until the camera itself is responsive
        # (in-camera NR processing finishes after file transfer).
        if last_shutter_s is not None and last_shutter_s > 2.0:
            nr_max = last_shutter_s + 5.0
            ready = wait_for_camera_ready(max_wait_s=nr_max, poll_interval_s=1.5)
            if not ready:
                print(
                    f"Camera still reports busy after {nr_max:.0f}s; proceeding with analysis/tuning anyway."
                )

        suppress_keeper_this_frame = startup_tune_pending and timed_mode
        if startup_tune_pending:
            startup_attempts += 1
        copied_this_iteration = False
        if args.keep_mode == "all" and not suppress_keeper_this_frame:
            copied_path = copy_keeper_image(image_path, keep_dir_path)
            copied_this_iteration = True
            print(f"Keeper saved: {copied_path.name}")

        metrics = analyze_image(image_path=image_path, target_luma=args.target_luma)
        exif_iso, exif_shutter_s, exif_fstop = read_capture_settings_from_exif(image_path)

        if use_last_applied_settings and last_iso is not None and last_shutter_s is not None:
            current_iso = last_iso
            current_shutter_s = last_shutter_s
            current_fstop = last_fstop
        else:
            current_iso = exif_iso if exif_iso is not None else last_iso
            current_shutter_s = exif_shutter_s if exif_shutter_s is not None else last_shutter_s
            current_fstop = exif_fstop if exif_fstop is not None else last_fstop

        if current_iso is None or current_shutter_s is None:
            print("error: missing ISO or shutter metadata and no previous value to fall back to", file=sys.stderr)
            return 1

        effective_lock_aperture = bool(args.lock_aperture)
        if current_fstop is None:
            current_fstop = 2.8
            effective_lock_aperture = True

        print(
            "Current settings: "
            f"ISO={current_iso}, shutter={format_shutter(current_shutter_s)}, f/{current_fstop}"
        )
        print(
            "Metrics: "
            f"deltaEV={metrics.delta_ev:.3f}, "
            f"median={metrics.median_luma:.1f}, "
            f"high_clip={metrics.highlight_clip_pct:.3f}%, "
            f"shadow_clip={metrics.shadow_clip_pct:.3f}%"
        )

        if suppress_keeper_this_frame:
            print("Startup tune shot: using this frame to calibrate; not saving to keep-dir.")

        active_deadband_ev = args.startup_deadband_ev if startup_tune_pending else args.deadband_ev
        if startup_tune_pending:
            # Startup should settle quickly to usable exposure and then switch to cadence.
            # Strict clip constraints can keep startup stuck in calibration indefinitely.
            optimal_now = abs(metrics.delta_ev) <= active_deadband_ev
        else:
            optimal_now = converged(metrics, active_deadband_ev)

        # If star-hunt startup is clearly seeing dawn/daylight, stop treating
        # frames as startup tuning shots and switch to normal daytime control.
        if (
            startup_tune_pending
            and args.startup_priority == "star-hunt"
            and (
                metrics.highlight_clip_pct >= 25.0
                or metrics.median_luma >= 245.0
                or (
                    metrics.delta_ev < -0.75
                    and current_shutter_s >= 8.0
                    and float(current_fstop) <= 2.0
                )
            )
        ):
            startup_tune_pending = False
            active_deadband_ev = args.deadband_ev
            optimal_now = converged(metrics, active_deadband_ev)
            print(
                "Dawn/daylight detected during star startup; switching to normal exposure control."
            )

        if startup_tune_pending and optimal_now:
            if calibration_only_mode:
                startup_tune_pending = False
                if not copied_this_iteration:
                    copied_path = copy_keeper_image(image_path, keep_dir_path)
                    print(f"Keeper saved: {copied_path.name}")
                print("Calibration complete.")
                return 0
            startup_tune_pending = False
            print(
                "Startup calibration complete (luma settled). Taking first keeper shot now, "
                "then switching to normal timed capture cadence."
            )
            continue

        if not optimal_now:
            append_reject_log(
                log_path=reject_log_path,
                image_name=image_path.name,
                metrics=metrics,
                current_iso=current_iso,
                current_shutter_s=current_shutter_s,
                current_fstop=current_fstop,
                note="not_optimal",
            )
            print(f"Logged non-optimal frame to {reject_log_path.name}")

        if optimal_now and not timed_mode:
            if not copied_this_iteration:
                copied_path = copy_keeper_image(image_path, keep_dir_path)
                print(f"Keeper saved: {copied_path.name}")
            print("Adjusted, all set.")
            return 0
        if optimal_now and timed_mode:
            if not copied_this_iteration and not suppress_keeper_this_frame:
                copied_path = copy_keeper_image(image_path, keep_dir_path)
                print(f"Keeper saved: {copied_path.name}")
                settled_mode = True
            if args.interval_seconds > 0:
                next_capture_at = time.time() + args.interval_seconds
            print("Frame is optimal. Continuing timed capture loop.")

        if timed_mode and best_effort_mode and not suppress_keeper_this_frame and not copied_this_iteration:
            keeper_tag = None if optimal_now else "fallback"
            copied_path = copy_keeper_image(image_path, keep_dir_path, tag=keeper_tag)
            copied_this_iteration = True
            if keeper_tag:
                print(f"Keeper saved (fallback): {copied_path.name}")
            else:
                print(f"Keeper saved (optimal): {copied_path.name}")

        if (timed_mode or calibration_only_mode) and not optimal_now:
            if startup_tune_pending:
                wait_s = max(0.0, float(args.startup_retry_seconds))
                next_capture_at = time.time() + wait_s
                print(f"Startup calibration in progress. Retrying in {wait_s:.1f}s.")
            elif timed_mode and best_effort_mode and args.interval_seconds > 0:
                next_capture_at = time.time() + args.interval_seconds
                print(
                    "Best-effort cadence active. Saved fallback keeper for this interval "
                    "and continuing fixed interval capture."
                )
            elif (
                settled_mode
                and abs(metrics.delta_ev) <= args.settled_breakout_ev
                and args.interval_seconds > 0
            ):
                next_capture_at = time.time() + args.interval_seconds
                print(
                    "Post-settle smoothing active. Keeping cadence and applying small exposure nudges "
                    "on the next timed frame."
                )
            elif args.interval_seconds > 0:
                print("Frame is not optimal. Retrying immediately until a keeper is found.")

        if optimal_now and timed_mode:
            continue

        if startup_tune_pending:
            active_max_step_ev = args.startup_max_step_ev
        elif settled_mode and abs(metrics.delta_ev) <= args.settled_breakout_ev:
            active_max_step_ev = min(args.max_step_ev, args.settled_max_step_ev)
        else:
            active_max_step_ev = args.max_step_ev

        rec = recommend_settings(
            metrics=metrics,
            iso=current_iso,
            shutter_s=current_shutter_s,
            fstop=current_fstop,
            iso_stops=iso_stops,
            shutter_stops=shutter_stops,
            fstop_stops=fstop_stops,
            shutter_min_s=shutter_min_s,
            shutter_max_s=shutter_max_s,
            iso_min=args.iso_min,
            iso_max=args.iso_max,
            deadband_ev=active_deadband_ev,
            max_step_ev=active_max_step_ev,
            lock_aperture=effective_lock_aperture,
            # Disable blur-based shutter speed-up during startup to prevent oscillation.
            blur_min=(-1.0 if startup_tune_pending else args.blur_min),
            prefer_iso_first_when_blur_low=(
                (startup_tune_pending and args.startup_priority == "fast-brighten")
                or (args.iso_strategy == "first-when-blur-low")
            ),
            force_iso_first_when_brightening=(
                startup_tune_pending and args.startup_priority == "fast-brighten"
            ),
            force_aperture_first_when_brightening=(
                startup_tune_pending and args.startup_priority in ("low-iso", "star-hunt")
            ),
            # Brightening priority: shutter longer → aperture wider → ISO last.
            prefer_aperture_last=False,
        )

        if startup_tune_pending and args.startup_priority == "star-hunt" and not optimal_now:
            bounded_shutter_stops = [s for s in shutter_stops if shutter_min_s <= s <= shutter_max_s]
            star_target_shutter = shutter_max_s
            min_fstop = min(float(f) for f in fstop_stops) if fstop_stops else float(current_fstop)

            if metrics.delta_ev > active_deadband_ev:
                # For night startup, snap directly to max shutter and widest
                # available aperture before touching ISO.
                if current_shutter_s + 1e-9 < star_target_shutter:
                    rec.suggested_shutter_s = star_target_shutter
                    print(
                        "Star-hunt override: jumping shutter to "
                        f"{format_shutter(star_target_shutter)} before aperture/ISO sweep."
                    )
                if not effective_lock_aperture and float(current_fstop) - min_fstop > 1e-6:
                    rec.suggested_fstop = min_fstop
                    print(
                        "Star-hunt override: opening aperture to minimum available "
                        f"f/{current_fstop} -> f/{rec.suggested_fstop}."
                    )
                if rec.suggested_shutter_s == current_shutter_s and rec.suggested_fstop == current_fstop:
                    nudged_iso = next_stop_value(float(current_iso), iso_stops, brighten=True)
                    if nudged_iso is not None and nudged_iso <= float(args.iso_max):
                        rec.suggested_iso = int(round(nudged_iso))
                        print(f"Star-hunt override: raising ISO one stop to {rec.suggested_iso}.")
            elif metrics.delta_ev < -active_deadband_ev or (
                metrics.highlight_clip_pct > 1.0 and metrics.delta_ev <= 0.0
            ):
                # After overshoot, back down aperture first, then shutter/ISO.
                if not effective_lock_aperture:
                    current_f = float(current_fstop)
                    higher_f_candidates = sorted([f for f in fstop_stops if f > current_f])
                    if higher_f_candidates:
                        rec.suggested_fstop = float(higher_f_candidates[0])
                        print(
                            "Star-hunt override: backing down aperture one stop "
                            f"f/{current_fstop} -> f/{rec.suggested_fstop}."
                        )
                if rec.suggested_fstop == current_fstop:
                    darker_shutter = next_stop_value(current_shutter_s, bounded_shutter_stops, brighten=False)
                    if darker_shutter is not None:
                        rec.suggested_shutter_s = darker_shutter
                        print(
                            "Star-hunt override: backing down shutter one stop "
                            f"to {format_shutter(darker_shutter)}."
                        )
                if rec.suggested_fstop == current_fstop and rec.suggested_shutter_s == current_shutter_s:
                    darker_iso = next_stop_value(float(current_iso), iso_stops, brighten=False)
                    if darker_iso is not None and darker_iso >= float(args.iso_min):
                        rec.suggested_iso = int(round(darker_iso))
                        print(f"Star-hunt override: lowering ISO one stop to {rec.suggested_iso}.")

        print(
            "Recommend: "
            f"action={rec.action}, "
            f"ISO={rec.suggested_iso}, "
            f"shutter={format_shutter(rec.suggested_shutter_s)}, "
            f"f/{rec.suggested_fstop}, "
            f"ev_step={rec.applied_ev_step:.3f}"
        )

        current_recommendation = (
            int(rec.suggested_iso),
            float(rec.suggested_shutter_s),
            float(rec.suggested_fstop),
        )
        if last_recommendation == current_recommendation:
            repeated_recommendation_count += 1
        else:
            repeated_recommendation_count = 0

        # If we recommend the exact same tuple repeatedly, skip the extra
        # retries and force a one-stop aperture move immediately.
        if (
            startup_tune_pending
            and args.startup_priority == "star-hunt"
            and repeated_recommendation_count >= 1
            and not optimal_now
            and not effective_lock_aperture
        ):
            if rec.action in ('darken', 'speed_up_shutter'):
                f_candidate = next_stop_value(rec.suggested_fstop, fstop_stops, brighten=True)
            else:
                f_candidate = next_stop_value(rec.suggested_fstop, fstop_stops, brighten=False)
            if f_candidate is not None and f_candidate != rec.suggested_fstop:
                print(
                    "Repeated recommendation detected: forcing immediate f-stop step "
                    f"f/{rec.suggested_fstop} -> f/{f_candidate}."
                )
                rec.suggested_fstop = f_candidate
                current_recommendation = (
                    int(rec.suggested_iso),
                    float(rec.suggested_shutter_s),
                    float(rec.suggested_fstop),
                )

        if startup_tune_pending and not optimal_now and abs(rec.applied_ev_step) < 1e-6:
            bounded_shutter_stops = [s for s in shutter_stops if shutter_min_s <= s <= shutter_max_s]
            if metrics.delta_ev > active_deadband_ev:
                nudged_shutter = next_stop_value(current_shutter_s, bounded_shutter_stops, brighten=True)
                if nudged_shutter is not None:
                    rec.suggested_shutter_s = nudged_shutter
                    print(
                        "Startup nudge: forcing brighter shutter stop "
                        f"{format_shutter(nudged_shutter)}"
                    )
                else:
                    if args.startup_priority == "low-iso" and not effective_lock_aperture:
                        current_f = float(current_fstop)
                        lower_f_candidates = sorted([f for f in fstop_stops if f < current_f])
                        if lower_f_candidates:
                            rec.suggested_fstop = float(lower_f_candidates[-1])
                            print(f"Startup nudge: forcing wider aperture f/{rec.suggested_fstop}")
                    if rec.suggested_fstop == current_fstop:
                        nudged_iso = next_stop_value(float(current_iso), iso_stops, brighten=True)
                        if nudged_iso is not None and nudged_iso <= float(args.iso_max):
                            rec.suggested_iso = int(round(nudged_iso))
                            print(f"Startup nudge: forcing brighter ISO stop {rec.suggested_iso}")
            elif metrics.delta_ev < -active_deadband_ev or (
                metrics.highlight_clip_pct > 1.0 and metrics.delta_ev <= 0.0
            ):
                nudged_shutter = next_stop_value(current_shutter_s, bounded_shutter_stops, brighten=False)
                if nudged_shutter is not None:
                    rec.suggested_shutter_s = nudged_shutter
                    print(
                        "Startup nudge: forcing darker shutter stop "
                        f"{format_shutter(nudged_shutter)}"
                    )
                else:
                    nudged_iso = next_stop_value(float(current_iso), iso_stops, brighten=False)
                    if nudged_iso is not None and nudged_iso >= float(args.iso_min):
                        rec.suggested_iso = int(round(nudged_iso))
                        print(f"Startup nudge: forcing darker ISO stop {rec.suggested_iso}")

        changed = False
        if rec.suggested_iso != current_iso:
            print(f"Applying: gphoto2 --set-config iso={rec.suggested_iso}")
            ok, msg = apply_setting("iso", str(rec.suggested_iso))
            print("Set ISO:", "ok" if ok else "failed")
            if not ok:
                print(msg)
                return 1
            changed = True

        shutter_text = format_shutter(rec.suggested_shutter_s)
        current_shutter_text = format_shutter(current_shutter_s)
        if shutter_text != current_shutter_text:
            print(f"Applying: gphoto2 --set-config shutterspeed={shutter_text}")
            ok, msg = apply_setting("shutterspeed", shutter_text)
            print("Set shutter:", "ok" if ok else "failed")
            if not ok:
                print(msg)
                return 1
            else:
                changed = True

        if not effective_lock_aperture and abs(rec.suggested_fstop - current_fstop) > 1e-6:
            print(f"Applying: gphoto2 --set-config f-number={rec.suggested_fstop}")
            ok, msg = apply_setting("f-number", f"{rec.suggested_fstop}")
            print("Set f-stop:", "ok" if ok else "failed")
            if not ok:
                # Lens may not support this f-stop — treat as a hard lens limit
                # and compensate with shutter speed / ISO rather than aborting.
                print(
                    f"f/{rec.suggested_fstop} rejected by camera (likely lens limit). "
                    "Compensating via shutter / ISO."
                )
                bounded_shutter_stops = [s for s in shutter_stops if shutter_min_s <= s <= shutter_max_s]
                if rec.action in ("darken", "speed_up_shutter"):
                    # Darken: ISO first for better quality, shutter second.
                    comp_iso = next_stop_value(float(current_iso), iso_stops, brighten=False)
                    if comp_iso is not None and int(round(comp_iso)) >= args.iso_min:
                        comp_iso_i = int(round(comp_iso))
                        print(f"Applying: gphoto2 --set-config iso={comp_iso_i}")
                        ok2, msg2 = apply_setting("iso", str(comp_iso_i))
                        print("Set ISO (lens-limit compensation):", "ok" if ok2 else "failed")
                        if ok2:
                            rec.suggested_iso = comp_iso_i
                            last_iso = comp_iso_i
                            changed = True
                    if not changed:
                        comp_shutter = next_stop_value(current_shutter_s, bounded_shutter_stops, brighten=False)
                        if comp_shutter is not None:
                            comp_shutter_text = format_shutter(comp_shutter)
                            print(f"Applying: gphoto2 --set-config shutterspeed={comp_shutter_text}")
                            ok2, msg2 = apply_setting("shutterspeed", comp_shutter_text)
                            print("Set shutter (lens-limit compensation):", "ok" if ok2 else "failed")
                            if ok2:
                                rec.suggested_shutter_s = comp_shutter
                                last_shutter_s = comp_shutter
                                changed = True
            else:
                changed = True

        if not changed:
            print("No setting changes needed.")
            if startup_tune_pending and not optimal_now:
                print("Startup calibration stalled at stop boundary. Forcing one-stop adjustment.")
                forced_change = False
                bounded_shutter_stops = [s for s in shutter_stops if shutter_min_s <= s <= shutter_max_s]

                if metrics.delta_ev > args.deadband_ev:
                    forced_shutter = next_stop_value(current_shutter_s, bounded_shutter_stops, brighten=True)
                    if forced_shutter is not None:
                        forced_shutter_text = format_shutter(forced_shutter)
                        print(f"Applying: gphoto2 --set-config shutterspeed={forced_shutter_text}")
                        ok, msg = apply_setting("shutterspeed", forced_shutter_text)
                        print("Set shutter:", "ok" if ok else "failed")
                        if not ok:
                            print(msg)
                            return 1
                        rec.suggested_shutter_s = forced_shutter
                        forced_change = True
                    else:
                        if args.startup_priority == "low-iso" and not effective_lock_aperture:
                            current_f = float(current_fstop)
                            lower_f_candidates = sorted([f for f in fstop_stops if f < current_f])
                            if lower_f_candidates:
                                forced_f = float(lower_f_candidates[-1])
                                print(f"Applying: gphoto2 --set-config f-number={forced_f}")
                                ok, msg = apply_setting("f-number", f"{forced_f}")
                                print("Set f-stop:", "ok" if ok else "failed")
                                if not ok:
                                    print(msg)
                                    return 1
                                rec.suggested_fstop = forced_f
                                forced_change = True
                        if not forced_change:
                            forced_iso = next_stop_value(float(current_iso), iso_stops, brighten=True)
                            if forced_iso is not None and forced_iso <= float(args.iso_max):
                                forced_iso_i = int(round(forced_iso))
                                print(f"Applying: gphoto2 --set-config iso={forced_iso_i}")
                                ok, msg = apply_setting("iso", str(forced_iso_i))
                                print("Set ISO:", "ok" if ok else "failed")
                                if not ok:
                                    print(msg)
                                    return 1
                                rec.suggested_iso = forced_iso_i
                                forced_change = True
                else:
                    forced_shutter = next_stop_value(current_shutter_s, bounded_shutter_stops, brighten=False)
                    if forced_shutter is not None:
                        forced_shutter_text = format_shutter(forced_shutter)
                        print(f"Applying: gphoto2 --set-config shutterspeed={forced_shutter_text}")
                        ok, msg = apply_setting("shutterspeed", forced_shutter_text)
                        print("Set shutter:", "ok" if ok else "failed")
                        if not ok:
                            print(msg)
                            return 1
                        rec.suggested_shutter_s = forced_shutter
                        forced_change = True
                    else:
                        forced_iso = next_stop_value(float(current_iso), iso_stops, brighten=False)
                        if forced_iso is not None and forced_iso >= float(args.iso_min):
                            forced_iso_i = int(round(forced_iso))
                            print(f"Applying: gphoto2 --set-config iso={forced_iso_i}")
                            ok, msg = apply_setting("iso", str(forced_iso_i))
                            print("Set ISO:", "ok" if ok else "failed")
                            if not ok:
                                print(msg)
                                return 1
                            rec.suggested_iso = forced_iso_i
                            forced_change = True

                if forced_change:
                    changed = True

            if timed_mode and not startup_tune_pending and not optimal_now and not changed and abs(rec.applied_ev_step) < 1e-6:
                if not copied_this_iteration:
                    copied_path = copy_keeper_image(image_path, keep_dir_path)
                    print(f"Keeper saved (bounded exposure): {copied_path.name}")
                    settled_mode = True
                if args.interval_seconds > 0:
                    next_capture_at = time.time() + args.interval_seconds
                print(
                    "Exposure is bounded by camera limits; accepting best-possible frame "
                    "and continuing normal timed cadence."
                )
                continue

            if not timed_mode:
                print("Adjusted, all set.")
                return 0

        last_iso = rec.suggested_iso
        last_shutter_s = rec.suggested_shutter_s
        last_fstop = rec.suggested_fstop
        last_recommendation = current_recommendation

        # Oscillation detection: if we keep alternating between the same two
        # setting combinations, force an aperture step to break the loop.
        settings_history.append((rec.suggested_iso, rec.suggested_shutter_s, rec.suggested_fstop))
        if len(settings_history) > OSCILLATION_WINDOW:
            settings_history.pop(0)
        if len(settings_history) >= OSCILLATION_WINDOW:
            unique_states = set(settings_history)
            if (
                startup_tune_pending
                and args.startup_priority == "star-hunt"
                and len(unique_states) <= 2
                and not effective_lock_aperture
            ):
                print(
                    "Oscillation detected: stuck between the same settings. "
                    "Forcing one f-stop step to break out."
                )
                # Step aperture one stop in the direction that matches the
                # current recommendation (darken → higher f-stop, else lower).
                if rec.action in ('darken', 'speed_up_shutter'):
                    # Higher f-number = darker (smaller aperture)
                    f_candidate = next_stop_value(rec.suggested_fstop, fstop_stops, brighten=True)
                else:
                    # Lower f-number = brighter (wider aperture)
                    f_candidate = next_stop_value(rec.suggested_fstop, fstop_stops, brighten=False)
                if f_candidate is not None and f_candidate != rec.suggested_fstop:
                    print(f"Oscillation break: f/{rec.suggested_fstop} -> f/{f_candidate}")
                    ok, msg = apply_setting('f-number', str(f_candidate))
                    if ok:
                        last_fstop = f_candidate
                        settings_history.clear()
                    else:
                        # Camera rejected the f-stop (lens limit); fall back to shutter / ISO.
                        print(
                            f"f/{f_candidate} rejected (lens limit). "
                            "Oscillation break via shutter / ISO."
                        )
                        bounded_shutter_stops = [s for s in shutter_stops if shutter_min_s <= s <= shutter_max_s]
                        if rec.action in ('darken', 'speed_up_shutter'):
                            # Darken: ISO first for better quality, shutter second.
                            fb_iso = next_stop_value(float(rec.suggested_iso), iso_stops, brighten=False)
                            if fb_iso is not None and int(round(fb_iso)) >= args.iso_min:
                                fb_iso_i = int(round(fb_iso))
                                print(f"Oscillation break fallback: ISO -> {fb_iso_i}")
                                ok2, _ = apply_setting('iso', str(fb_iso_i))
                                if ok2:
                                    last_iso = fb_iso_i
                                    settings_history.clear()
                            if settings_history:
                                fb_shutter = next_stop_value(rec.suggested_shutter_s, bounded_shutter_stops, brighten=False)
                                if fb_shutter is not None:
                                    fb_text = format_shutter(fb_shutter)
                                    print(f"Oscillation break fallback: shutter -> {fb_text}")
                                    ok2, _ = apply_setting('shutterspeed', fb_text)
                                    if ok2:
                                        last_shutter_s = fb_shutter
                                        settings_history.clear()
                        else:
                            fb_shutter = next_stop_value(rec.suggested_shutter_s, bounded_shutter_stops, brighten=True)
                            if fb_shutter is not None:
                                fb_text = format_shutter(fb_shutter)
                                print(f"Oscillation break fallback: shutter -> {fb_text}")
                                ok2, _ = apply_setting('shutterspeed', fb_text)
                                if ok2:
                                    last_shutter_s = fb_shutter
                                    settings_history.clear()
                else:
                    # f-stop already at boundary — apply shutter / ISO to break out directly.
                    print("Oscillation: aperture at limit. Breaking out via shutter / ISO.")
                    bounded_shutter_stops = [s for s in shutter_stops if shutter_min_s <= s <= shutter_max_s]
                    if rec.action in ('darken', 'speed_up_shutter'):
                        # Darken: ISO first for better quality, shutter second.
                        fb_iso = next_stop_value(float(rec.suggested_iso), iso_stops, brighten=False)
                        if fb_iso is not None and int(round(fb_iso)) >= args.iso_min:
                            fb_iso_i = int(round(fb_iso))
                            print(f"Oscillation break (aperture limit): ISO -> {fb_iso_i}")
                            ok2, _ = apply_setting('iso', str(fb_iso_i))
                            if ok2:
                                last_iso = fb_iso_i
                                settings_history.clear()
                        if settings_history:
                            fb_shutter = next_stop_value(rec.suggested_shutter_s, bounded_shutter_stops, brighten=False)
                            if fb_shutter is not None:
                                fb_text = format_shutter(fb_shutter)
                                print(f"Oscillation break (aperture limit): shutter -> {fb_text}")
                                ok2, _ = apply_setting('shutterspeed', fb_text)
                                if ok2:
                                    last_shutter_s = fb_shutter
                                    settings_history.clear()
                    else:
                        fb_shutter = next_stop_value(rec.suggested_shutter_s, bounded_shutter_stops, brighten=True)
                        if fb_shutter is not None:
                            fb_text = format_shutter(fb_shutter)
                            print(f"Oscillation break (aperture limit): shutter -> {fb_text}")
                            ok2, _ = apply_setting('shutterspeed', fb_text)
                            if ok2:
                                last_shutter_s = fb_shutter
                                settings_history.clear()

        if startup_tune_pending and args.startup_max_iterations > 0 and startup_attempts >= args.startup_max_iterations:
            startup_tune_pending = False
            print(
                "Startup calibration attempt limit reached. "
                "Taking first keeper shot now, then switching to normal timed capture cadence."
            )

    print("Auto-tune run complete.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
