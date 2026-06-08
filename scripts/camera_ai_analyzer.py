#!/usr/bin/env python3
"""
Analyze a JPG and recommend next camera settings (ISO/shutter/f-stop).

This script is designed to be a practical first step toward a closed-loop
capture pipeline. It focuses on deterministic metrics so behavior is
explainable and easy to tune before adding ML.

Examples:
  python3 camera_ai_analyzer.py \
    --image "./capt_MJB09384.JPG" \
    --iso 400 --shutter 1/125 --fstop 2.8

  python3 camera_ai_analyzer.py \
    --image "./capt_MJB09384.JPG" \
    --iso 800 --shutter 1/60 --fstop 4 \
    --target-luma 118 --max-step-ev 0.67 --json
"""

from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image


DEFAULT_ISO_STOPS: List[int] = [
    50, 64, 80, 100, 125, 160, 200, 250, 320, 400, 500, 640,
    800, 1000, 1250, 1600, 2000, 2500, 3200, 4000, 5000, 6400,
    8000, 10000, 12800, 16000, 20000, 25600, 32000, 40000, 51200,
]

DEFAULT_SHUTTER_STOPS: List[float] = [
    30.0, 25.0, 20.0, 15.0, 13.0, 10.0, 8.0, 6.0, 5.0, 4.0, 3.2, 2.5,
    2.0, 1.6, 1.3, 1.0,
    1 / 1.3, 1 / 1.6, 1 / 2.0, 1 / 2.5, 1 / 3.2, 1 / 4.0, 1 / 5.0, 1 / 6.0,
    1 / 8.0, 1 / 10.0, 1 / 13.0, 1 / 15.0, 1 / 20.0, 1 / 25.0, 1 / 30.0,
    1 / 40.0, 1 / 50.0, 1 / 60.0, 1 / 80.0, 1 / 100.0, 1 / 125.0, 1 / 160.0,
    1 / 200.0, 1 / 250.0, 1 / 320.0, 1 / 400.0, 1 / 500.0, 1 / 640.0,
    1 / 800.0, 1 / 1000.0, 1 / 1250.0, 1 / 1600.0, 1 / 2000.0, 1 / 2500.0,
    1 / 3200.0, 1 / 4000.0, 1 / 5000.0, 1 / 6400.0, 1 / 8000.0,
]

DEFAULT_FSTOP_STOPS: List[float] = [
    1.4, 1.6, 1.8,
    2.0, 2.2, 2.5, 2.8,
    3.2, 3.5, 4.0, 4.5,
    5.0, 5.6, 6.3,
    7.1, 8.0, 9.0,
    10.0, 11.0, 13.0,
    14.0, 16.0,
    18.0, 20.0, 22.0, 25.0, 29.0, 32.0,
]


@dataclass
class AnalysisMetrics:
    median_luma: float
    mean_luma: float
    shadow_clip_pct: float
    highlight_clip_pct: float
    blur_score: float
    delta_ev: float


@dataclass
class Recommendation:
    action: str
    reason: str
    suggested_iso: int
    suggested_shutter_s: float
    suggested_fstop: float
    applied_ev_step: float


def _to_float(value: object) -> Optional[float]:
    if value is None:
        return None
    try:
        if isinstance(value, tuple) and len(value) == 2:
            den = float(value[1])
            if den == 0.0:
                return None
            return float(value[0]) / den
        if hasattr(value, "numerator") and hasattr(value, "denominator"):
            den = float(getattr(value, "denominator"))
            if den == 0.0:
                return None
            return float(getattr(value, "numerator")) / den
        return float(value)
    except Exception:
        return None


def read_capture_settings_from_exif(image_path: Path) -> Tuple[Optional[int], Optional[float], Optional[float]]:
    # Prefer exiftool first when available because Sony JPG metadata can expose
    # aperture/ISO more reliably there than through Pillow EXIF readers.
    iso, shutter_s, fstop = read_capture_settings_with_exiftool(image_path)

    try:
        with Image.open(image_path) as img:
            exif = img.getexif()
    except Exception:
        exif = None

    if exif:
        # Common EXIF tags for capture settings.
        # 34855: ISOSpeedRatings, 33434: ExposureTime, 33437: FNumber,
        # 41989: PhotographicSensitivity (newer naming).
        iso_raw = exif.get(34855, exif.get(41989))
        iso_float = _to_float(iso_raw)
        if iso is None and iso_float is not None:
            iso = int(round(iso_float))

        shutter_raw = exif.get(33434)
        shutter_val = _to_float(shutter_raw)
        if shutter_s is None and shutter_val is not None:
            shutter_s = shutter_val

        fstop_raw = exif.get(33437)
        fstop_val = _to_float(fstop_raw)
        if fstop is None and fstop_val is not None:
            fstop = fstop_val

    # macOS fallback: Spotlight metadata often includes ISO/exposure.
    if iso is None or shutter_s is None or fstop is None:
        mdls_iso, mdls_shutter_s, mdls_fstop = read_capture_settings_with_mdls(image_path)
        if iso is None:
            iso = mdls_iso
        if shutter_s is None:
            shutter_s = mdls_shutter_s
        if fstop is None:
            fstop = mdls_fstop

    return iso, shutter_s, fstop


def read_capture_settings_with_exiftool(image_path: Path) -> Tuple[Optional[int], Optional[float], Optional[float]]:
    try:
        result = subprocess.run(
            ["exiftool", "-s3", "-ISO", "-ExposureTime", "-FNumber", str(image_path)],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
    except Exception:
        return None, None, None

    if result.returncode != 0:
        return None, None, None

    lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if len(lines) < 3:
        return None, None, None

    iso: Optional[int] = None
    shutter_s: Optional[float] = None
    fstop: Optional[float] = None

    # exiftool output for -s3 follows request order: ISO, ExposureTime, FNumber
    iso_match = re.search(r"\d+", lines[0])
    if iso_match:
        iso = int(iso_match.group(0))

    try:
        shutter_s = parse_shutter(lines[1])
    except Exception:
        shutter_s = None

    fstop_match = re.search(r"\d+(?:\.\d+)?", lines[2])
    if fstop_match:
        fstop = float(fstop_match.group(0))

    return iso, shutter_s, fstop


def read_capture_settings_with_mdls(image_path: Path) -> Tuple[Optional[int], Optional[float], Optional[float]]:
    try:
        result = subprocess.run(
            [
                "mdls",
                "-name",
                "kMDItemISOSpeed",
                "-name",
                "kMDItemExposureTimeSeconds",
                "-name",
                "kMDItemAperture",
                str(image_path),
            ],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
    except Exception:
        return None, None, None

    if result.returncode != 0:
        return None, None, None

    iso: Optional[int] = None
    shutter_s: Optional[float] = None
    fstop: Optional[float] = None

    for raw_line in result.stdout.splitlines():
        line = raw_line.strip()
        if not line or "=" not in line:
            continue
        key, value = [part.strip() for part in line.split("=", 1)]
        if value == "(null)":
            continue

        number_match = re.search(r"-?\d+(?:\.\d+)?", value)
        if not number_match:
            continue

        num = float(number_match.group(0))
        if key == "kMDItemISOSpeed":
            iso = int(round(num))
        elif key == "kMDItemExposureTimeSeconds":
            shutter_s = num
        elif key == "kMDItemAperture":
            fstop = num

    return iso, shutter_s, fstop


def parse_shutter(value: str) -> float:
    raw = value.strip().lower().replace("sec", "").replace("s", "")
    if "/" in raw:
        num, den = raw.split("/", 1)
        return float(num) / float(den)
    return float(raw)


def format_shutter(seconds: float) -> str:
    if seconds >= 1.0:
        return f"{seconds:.3g}s"
    denom = round(1.0 / max(seconds, 1e-9))
    return f"1/{denom}"


def parse_numeric_list(raw: Optional[str], as_int: bool) -> Optional[List[float]]:
    if not raw:
        return None
    parts = [p.strip() for p in re.split(r"[;,\s]+", raw.strip()) if p.strip()]
    values: List[float] = []
    for part in parts:
        if as_int:
            values.append(float(int(part)))
        else:
            if "/" in part:
                values.append(parse_shutter(part))
            else:
                values.append(float(part))
    return values


def nearest_stop(value: float, stops: Sequence[float]) -> float:
    return min(stops, key=lambda s: abs(math.log(max(value, 1e-12)) - math.log(max(s, 1e-12))))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def analyze_image(image_path: Path, target_luma: float) -> AnalysisMetrics:
    img = Image.open(image_path).convert("RGB")
    arr = np.asarray(img, dtype=np.float32)

    # Rec.709 luma approximation for JPGs.
    luma = 0.2126 * arr[:, :, 0] + 0.7152 * arr[:, :, 1] + 0.0722 * arr[:, :, 2]

    shadow_clip = float((luma <= 5.0).mean() * 100.0)
    highlight_clip = float((luma >= 250.0).mean() * 100.0)
    median_luma = float(np.median(luma))
    mean_luma = float(np.mean(luma))

    # 5-point Laplacian variance as a simple blur proxy.
    lap = (
        -4.0 * luma[1:-1, 1:-1]
        + luma[:-2, 1:-1]
        + luma[2:, 1:-1]
        + luma[1:-1, :-2]
        + luma[1:-1, 2:]
    )
    blur_score = float(np.var(lap))

    delta_ev = math.log2(max(target_luma, 1e-6) / max(median_luma, 1e-6))

    return AnalysisMetrics(
        median_luma=median_luma,
        mean_luma=mean_luma,
        shadow_clip_pct=shadow_clip,
        highlight_clip_pct=highlight_clip,
        blur_score=blur_score,
        delta_ev=delta_ev,
    )


def recommend_settings(
    metrics: AnalysisMetrics,
    iso: int,
    shutter_s: float,
    fstop: float,
    iso_stops: Sequence[float],
    shutter_stops: Sequence[float],
    fstop_stops: Sequence[float],
    shutter_min_s: float,
    shutter_max_s: float,
    iso_min: int,
    iso_max: int,
    deadband_ev: float,
    max_step_ev: float,
    lock_aperture: bool,
    blur_min: float,
    prefer_iso_first_when_blur_low: bool,
    force_iso_first_when_brightening: bool = False,
    force_aperture_first_when_brightening: bool = False,
    prefer_aperture_last: bool = False,
) -> Recommendation:
    suggested_iso = int(iso)
    suggested_shutter = float(shutter_s)
    suggested_fstop = float(fstop)
    reason = ""

    # Decision priority:
    # 1) If frame is dark, always brighten first.
    # 2) If frame is bright/clipping, darken.
    # 3) Blur-only correction applies only when exposure is near target.
    if metrics.highlight_clip_pct > 2.0:
        action = "darken"
        raw_step = -max_step_ev
        reason = "highlight clipping above 2%"
    elif metrics.shadow_clip_pct > 2.0:
        action = "brighten"
        raw_step = max_step_ev
        reason = "shadow clipping above 2%"
    elif abs(metrics.delta_ev) < deadband_ev:
        action = "hold"
        raw_step = 0.0
        reason = "within EV deadband"
    elif metrics.delta_ev > deadband_ev:
        action = "brighten"
        raw_step = clamp(metrics.delta_ev, -max_step_ev, max_step_ev)
        reason = "underexposed relative to target luminance"
    elif metrics.blur_score < blur_min:
        action = "speed_up_shutter"
        raw_step = -min(max_step_ev, 0.67)
        reason = "blur score below threshold"
    else:
        action = "darken"
        raw_step = clamp(metrics.delta_ev, -max_step_ev, max_step_ev)
        reason = "median luminance offset from target"

    if raw_step == 0.0:
        return Recommendation(
            action=action,
            reason=reason,
            suggested_iso=suggested_iso,
            suggested_shutter_s=suggested_shutter,
            suggested_fstop=suggested_fstop,
            applied_ev_step=0.0,
        )

    achieved_shutter_ev = 0.0
    remaining_ev = raw_step

    # Optional strategy: when brightening and aperture control is allowed,
    # open aperture first to preserve lower ISO.
    if (
        force_aperture_first_when_brightening
        and not prefer_aperture_last
        and raw_step > 0.0
        and not lock_aperture
        and abs(remaining_ev) > 0.05
    ):
        f_candidate = suggested_fstop / (2.0 ** (remaining_ev / 2.0))
        f_candidate = nearest_stop(f_candidate, fstop_stops)
        aperture_achieved_ev = -2.0 * math.log2(max(f_candidate, 1e-9) / max(suggested_fstop, 1e-9))
        suggested_fstop = float(f_candidate)
        remaining_ev = remaining_ev - aperture_achieved_ev

    # Optional strategy: when brightening and blur is already low,
    # you may prefer ISO before slowing shutter.
    prefer_iso_first = (
        (force_iso_first_when_brightening and raw_step > 0.0)
        or (
            prefer_iso_first_when_blur_low
            and raw_step > 0.0
            and metrics.blur_score < blur_min
        )
    )

    if prefer_iso_first:
        iso_candidate = suggested_iso * (2.0 ** remaining_ev)
        iso_candidate = clamp(iso_candidate, float(iso_min), float(iso_max))
        iso_candidate = nearest_stop(iso_candidate, iso_stops)
        iso_achieved_ev = math.log2(max(iso_candidate, 1.0) / max(suggested_iso, 1.0))
        suggested_iso = int(round(iso_candidate))
        remaining_ev = raw_step - iso_achieved_ev

    # EV model for shutter: shorter exposure darkens, longer brightens.
    if abs(remaining_ev) > 0.05:
        shutter_candidate = suggested_shutter * (2.0 ** remaining_ev)
        shutter_candidate = clamp(shutter_candidate, shutter_min_s, shutter_max_s)
        shutter_candidate = nearest_stop(shutter_candidate, shutter_stops)

        achieved_shutter_ev = math.log2(max(shutter_candidate, 1e-9) / max(suggested_shutter, 1e-9))
        remaining_ev = remaining_ev - achieved_shutter_ev
        suggested_shutter = shutter_candidate

    # If shutter-first path was used, compensate remainder with aperture first,
    # then ISO by default. In aperture-last mode, do ISO first and only open
    # aperture if shutter/ISO could not finish the requested EV step.
    if not prefer_iso_first and abs(remaining_ev) > 0.05:
        if prefer_aperture_last:
            iso_candidate = suggested_iso * (2.0 ** remaining_ev)
            iso_candidate = clamp(iso_candidate, float(iso_min), float(iso_max))
            iso_candidate = nearest_stop(iso_candidate, iso_stops)
            iso_achieved_ev = math.log2(max(iso_candidate, 1.0) / max(suggested_iso, 1.0))
            suggested_iso = int(round(iso_candidate))
            remaining_ev = remaining_ev - iso_achieved_ev

            if not lock_aperture and abs(remaining_ev) > 0.05:
                f_candidate = suggested_fstop / (2.0 ** (remaining_ev / 2.0))
                f_candidate = nearest_stop(f_candidate, fstop_stops)
                aperture_achieved_ev = -2.0 * math.log2(max(f_candidate, 1e-9) / max(suggested_fstop, 1e-9))
                suggested_fstop = float(f_candidate)
                remaining_ev = remaining_ev - aperture_achieved_ev
        else:
            if not lock_aperture:
                f_candidate = suggested_fstop / (2.0 ** (remaining_ev / 2.0))
                f_candidate = nearest_stop(f_candidate, fstop_stops)
                aperture_achieved_ev = -2.0 * math.log2(max(f_candidate, 1e-9) / max(suggested_fstop, 1e-9))
                suggested_fstop = float(f_candidate)
                remaining_ev = remaining_ev - aperture_achieved_ev

            if abs(remaining_ev) > 0.05:
                iso_candidate = suggested_iso * (2.0 ** remaining_ev)
                iso_candidate = clamp(iso_candidate, float(iso_min), float(iso_max))
                iso_candidate = nearest_stop(iso_candidate, iso_stops)
                suggested_iso = int(round(iso_candidate))

    # Optionally use aperture only if caller allows and ISO hit limits.
    if not lock_aperture and (suggested_iso in (iso_min, iso_max)):
        residual_from_iso = math.log2(max(suggested_iso, 1) / max(iso, 1))
        total_achieved = achieved_shutter_ev + residual_from_iso
        aperture_remaining_ev = raw_step - total_achieved
        if abs(aperture_remaining_ev) > 0.1:
            # For f-stop, exposure ~ 1 / N^2. Positive EV means smaller N.
            f_candidate = suggested_fstop / (2.0 ** (aperture_remaining_ev / 2.0))
            f_candidate = nearest_stop(f_candidate, fstop_stops)
            suggested_fstop = float(f_candidate)

    total_applied_ev = (
        math.log2(max(suggested_shutter, 1e-9) / max(shutter_s, 1e-9))
        + math.log2(max(suggested_iso, 1) / max(iso, 1))
    )
    if not lock_aperture:
        total_applied_ev += -2.0 * math.log2(max(suggested_fstop, 1e-9) / max(fstop, 1e-9))

    return Recommendation(
        action=action,
        reason=reason,
        suggested_iso=suggested_iso,
        suggested_shutter_s=suggested_shutter,
        suggested_fstop=suggested_fstop,
        applied_ev_step=total_applied_ev,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Analyze JPG and recommend next camera settings.")
    parser.add_argument("--image", required=True, help="Path to JPG image.")

    parser.add_argument("--iso", required=False, type=int, help="Current ISO value. If omitted, uses EXIF.")
    parser.add_argument("--shutter", required=False, help="Current shutter (e.g. 1/125 or 0.008). If omitted, uses EXIF.")
    parser.add_argument("--fstop", required=False, type=float, help="Current f-stop value (e.g. 2.8). If omitted, uses EXIF.")

    parser.add_argument("--target-luma", type=float, default=118.0, help="Target median luminance (0-255).")
    parser.add_argument("--deadband-ev", type=float, default=0.15, help="No-change EV deadband.")
    parser.add_argument("--max-step-ev", type=float, default=0.67, help="Max EV step per frame.")

    parser.add_argument("--shutter-min", default="1/1000", help="Fastest allowed shutter (smallest time).")
    parser.add_argument("--shutter-max", default="1/30", help="Slowest allowed shutter (largest time).")
    parser.add_argument("--iso-min", type=int, default=100, help="Minimum allowed ISO.")
    parser.add_argument("--iso-max", type=int, default=6400, help="Maximum allowed ISO.")
    parser.add_argument("--lock-aperture", action="store_true", default=False, help="Do not change f-stop.")
    parser.add_argument("--blur-min", type=float, default=120.0, help="Minimum blur score threshold.")
    parser.add_argument(
        "--iso-strategy",
        choices=["last", "first-when-blur-low"],
        default="last",
        help="ISO adjustment strategy. 'last' is timelapse-friendly and minimizes ISO changes.",
    )

    parser.add_argument(
        "--iso-stops",
        default=None,
        help="Optional CSV/space-separated ISO stops to override defaults.",
    )
    parser.add_argument(
        "--shutter-stops",
        default=None,
        help="Optional CSV/space-separated shutter stops (e.g. '1/125,1/100,1/80').",
    )
    parser.add_argument(
        "--fstop-stops",
        default=None,
        help="Optional CSV/space-separated f-stop stops.",
    )

    parser.add_argument("--json", action="store_true", help="Print JSON output.")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    image_path = Path(args.image)
    if not image_path.exists():
        print(f"error: image not found: {image_path}", file=sys.stderr)
        return 2

    exif_iso, exif_shutter_s, exif_fstop = read_capture_settings_from_exif(image_path)

    iso = args.iso if args.iso is not None else exif_iso
    if iso is None:
        print("error: ISO not provided and not found in EXIF (use --iso)", file=sys.stderr)
        return 2

    if args.shutter is not None:
        try:
            shutter_s = parse_shutter(args.shutter)
        except Exception as exc:
            print(f"error: invalid --shutter value '{args.shutter}': {exc}", file=sys.stderr)
            return 2
    else:
        shutter_s = exif_shutter_s
        if shutter_s is None:
            print("error: shutter not provided and not found in EXIF (use --shutter)", file=sys.stderr)
            return 2

    fstop_source = "cli" if args.fstop is not None else "exif"
    fstop = args.fstop if args.fstop is not None else exif_fstop
    effective_lock_aperture = bool(args.lock_aperture)
    if fstop is None:
        if effective_lock_aperture:
            # With aperture locked, f-stop is not used for control decisions.
            # Keep a placeholder for reporting consistency.
            fstop = 2.8
            fstop_source = "default"
        else:
            # Auto-fallback: if aperture metadata is missing, continue in
            # aperture-locked mode so no manual --fstop is required.
            fstop = 2.8
            fstop_source = "default"
            effective_lock_aperture = True

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

    metrics = analyze_image(image_path=image_path, target_luma=args.target_luma)

    rec = recommend_settings(
        metrics=metrics,
        iso=iso,
        shutter_s=shutter_s,
        fstop=fstop,
        iso_stops=iso_stops,
        shutter_stops=shutter_stops,
        fstop_stops=fstop_stops,
        shutter_min_s=shutter_min_s,
        shutter_max_s=shutter_max_s,
        iso_min=args.iso_min,
        iso_max=args.iso_max,
        deadband_ev=args.deadband_ev,
        max_step_ev=args.max_step_ev,
        lock_aperture=effective_lock_aperture,
        blur_min=args.blur_min,
        prefer_iso_first_when_blur_low=(args.iso_strategy == "first-when-blur-low"),
        force_iso_first_when_brightening=False,
    )

    result = {
        "image": str(image_path),
        "current": {
            "iso": iso,
            "shutter": format_shutter(shutter_s),
            "fstop": fstop,
        },
        "capture_settings_source": {
            "iso": "cli" if args.iso is not None else "exif",
            "shutter": "cli" if args.shutter is not None else "exif",
            "fstop": fstop_source,
        },
        "effective_lock_aperture": effective_lock_aperture,
        "metrics": asdict(metrics),
        "recommendation": {
            **asdict(rec),
            "suggested_shutter": format_shutter(rec.suggested_shutter_s),
        },
    }

    if args.json:
        print(json.dumps(result, indent=2))
        return 0

    print(f"Image: {result['image']}")
    print("Current:")
    print(f"  ISO:      {result['current']['iso']}")
    print(f"  Shutter:  {result['current']['shutter']}")
    print(f"  F-stop:   f/{result['current']['fstop']}")
    print(
        "  Source:   "
        f"ISO={result['capture_settings_source']['iso']}, "
        f"Shutter={result['capture_settings_source']['shutter']}, "
        f"F-stop={result['capture_settings_source']['fstop']}"
    )
    print(f"  Aperture control locked: {result['effective_lock_aperture']}")

    print("Metrics:")
    print(f"  Median luma:        {metrics.median_luma:.2f}")
    print(f"  Mean luma:          {metrics.mean_luma:.2f}")
    print(f"  Shadow clip %:      {metrics.shadow_clip_pct:.3f}")
    print(f"  Highlight clip %:   {metrics.highlight_clip_pct:.3f}")
    print(f"  Blur score:         {metrics.blur_score:.2f}")
    print(f"  Delta EV:           {metrics.delta_ev:.3f}")

    print("Recommendation:")
    print(f"  Action:             {rec.action}")
    print(f"  Reason:             {rec.reason}")
    print(f"  Suggested ISO:      {rec.suggested_iso}")
    print(f"  Suggested Shutter:  {format_shutter(rec.suggested_shutter_s)}")
    print(f"  Suggested F-stop:   f/{rec.suggested_fstop}")
    print(f"  Applied EV step:    {rec.applied_ev_step:.3f}")

    print("\nNext gphoto2 commands (example):")
    print(f"  gphoto2 --set-config iso={rec.suggested_iso}")
    print(f"  gphoto2 --set-config shutterspeed={format_shutter(rec.suggested_shutter_s)}")
    if not result["effective_lock_aperture"]:
        print(f"  gphoto2 --set-config f-number={rec.suggested_fstop}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
