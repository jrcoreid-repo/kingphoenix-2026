#!/usr/bin/env python3
"""Kingphoenix GCS V2.2.3.1 - final GUI telemetry binding fix.

Keeps the frozen final layout. This patch binds the already-received V2.2.3
telemetry to the existing final GUI tags (especially Current Waypoint and
LiDAR) and adds a low-rate telemetry diagnostic print.
"""

from __future__ import annotations

import time

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final as gcs
import kingphoenix_gcs_final_v2_2_3 as v223


_original_v223_update = v223._update_info_panel_v223
_last_diag_s = 0.0


def _safe_set(tag, value):
    try:
        if dpg.does_item_exist(tag):
            dpg.set_value(tag, value)
    except Exception:
        pass


def _freshest_raw_metadata():
    """Prefer exact active metadata, then newest raw camera metadata.

    Telemetry is common FC state, so it must not disappear just because one
    camera's exact-frame metadata is temporarily unavailable.
    """
    candidates = []

    try:
        meta = gcs.get_active_metadata()
        if isinstance(meta, dict):
            candidates.append((3, meta))
    except Exception:
        pass

    for camera in (gcs.front, gcs.bottom):
        try:
            meta = gcs.get_latest_raw_metadata(camera)
            if isinstance(meta, dict):
                ts = int(meta.get("timestamp_ns") or 0)
                candidates.append((ts, meta))
        except Exception:
            pass

    if not candidates:
        return None

    return max(candidates, key=lambda item: item[0])[1]


def _bind_existing_final_tags(meta):
    telemetry = (meta or {}).get("telemetry") or {}
    if not telemetry:
        return

    # Existing final waypoint field in SETUP / WAYPOINT card.
    wp = telemetry.get("current_mission_item")
    wp_text = "--" if wp is None else str(int(wp))
    _safe_set("waypoint_current_value", wp_text)
    _safe_set("mav_waypoint", wp_text)
    _safe_set("mav_wp_v223", "WP --" if wp is None else f"WP {int(wp)}")

    # Existing final SYSTEM / RANGE LiDAR field.
    lidar = telemetry.get("rangefinder_m")
    lidar_age = telemetry.get("rangefinder_age_s")
    lidar_fresh = bool(telemetry.get("rangefinder_fresh", False))
    source = telemetry.get("rangefinder_source") or "--"

    if lidar is None:
        lidar_text = "--"
    elif lidar_fresh:
        lidar_text = f"{float(lidar):.2f} m"
        if lidar_age is not None:
            lidar_text += f" / {float(lidar_age) * 1000.0:.0f} ms"
    else:
        lidar_text = f"STALE {float(lidar):.2f} m"
        if lidar_age is not None:
            lidar_text += f" / {float(lidar_age):.2f}s"
    _safe_set("value_rangefinder", lidar_text)

    # Existing hidden compatibility fields are also kept current.
    lat = telemetry.get("gps_lat_deg")
    lon = telemetry.get("gps_lon_deg")
    if lat is not None and lon is not None:
        _safe_set("mav_position", f"{float(lat):.7f}, {float(lon):.7f}")
        _safe_set("mav_gps_pos", f"GPS {float(lat):.6f}, {float(lon):.6f}")

    fix_type = telemetry.get("gps_fix_type")
    sats = telemetry.get("gps_satellites")
    hdop = telemetry.get("gps_hdop")
    parts = [v223._format_fix_type(fix_type)]
    if sats is not None:
        parts.append(f"{int(sats)} SAT")
    if hdop is not None:
        parts.append(f"HDOP {float(hdop):.2f}")
    _safe_set("mav_gps", " / ".join(parts))

    return {
        "wp": wp,
        "lidar": lidar,
        "lidar_age": lidar_age,
        "lidar_fresh": lidar_fresh,
        "lidar_source": source,
        "fix": fix_type,
        "sats": sats,
        "lat": lat,
        "lon": lon,
        "rates": telemetry.get("rx_rate_hz") or {},
    }


def _update_info_panel_v2231():
    global _last_diag_s

    # Preserve all V2.2.3 UI updates first.
    _original_v223_update()

    try:
        meta = _freshest_raw_metadata()
        diag = _bind_existing_final_tags(meta)
        if not diag:
            return

        now = time.monotonic()
        if now - _last_diag_s >= 2.0:
            _last_diag_s = now
            rates = diag.get("rates") or {}
            print(
                "[GCS TELEMETRY] "
                f"GPS fix={diag.get('fix')} sats={diag.get('sats')} "
                f"lat={diag.get('lat')} lon={diag.get('lon')} | "
                f"WP={diag.get('wp')} | "
                f"LiDAR={diag.get('lidar')}m fresh={diag.get('lidar_fresh')} "
                f"age={diag.get('lidar_age')} source={diag.get('lidar_source')} | "
                "RX GPS={:.1f}Hz GPOS={:.1f}Hz DIST={:.1f}Hz WP={:.1f}Hz".format(
                    float(rates.get("GPS_RAW_INT") or 0.0),
                    float(rates.get("GLOBAL_POSITION_INT") or 0.0),
                    float(rates.get("DISTANCE_SENSOR") or 0.0),
                    float(rates.get("MISSION_CURRENT") or 0.0),
                )
            )
    except Exception as exc:
        print(f"[GCS] V2.2.3.1 binding warning: {exc}")


def main():
    # v223.main() will install its own wrappers into gcs; replacing this
    # function before calling it makes our final binding the active updater.
    v223._update_info_panel_v223 = _update_info_panel_v2231

    print("[GCS] V2.2.3.1 FINAL GUI BINDING FIX")
    print("[GCS] Existing Current Waypoint + LiDAR fields bound to live FC telemetry")
    print("[GCS] Telemetry source falls back to freshest raw camera metadata")
    print("[GCS] CONTROL TX remains DISABLED")
    v223.main()


if __name__ == "__main__":
    main()
