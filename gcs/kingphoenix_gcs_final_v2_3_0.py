#!/usr/bin/env python3
"""Kingphoenix GCS V2.3.0 - first guarded FC command controls.

Extends V2.2.3.2 while preserving the frozen operator layout. Adds compact
bench ARM/DISARM and flight-mode controls below the existing servo-test row.
Commands travel over the existing GCS UDP control channel and are executed on
the Pi through CommandManager/VCT. Movement, takeoff, mission velocity and
servo PWM TX remain disabled in this build.
"""

from __future__ import annotations

import json
import threading
import time

import dearpygui.dearpygui as dpg

import kingphoenix_gcs_final as gcs
import kingphoenix_gcs_final_v2_2_3_2 as v2232


_original_create_gui_v2232 = v2232._create_gui_v2232
_original_set_operator_mode = gcs._set_operator_mode

BENCH_MODES = [
    "STABILIZE",
    "ALT_HOLD",
    "LOITER",
    "GUIDED",
    "AUTO",
    "LAND",
]

_command_busy = False
_command_lock = threading.Lock()


def _safe_set(tag, value):
    try:
        if dpg.does_item_exist(tag):
            dpg.set_value(tag, value)
    except Exception:
        pass


def _safe_enable(tag, enabled):
    try:
        if not dpg.does_item_exist(tag):
            return
        if enabled:
            dpg.enable_item(tag)
        else:
            dpg.disable_item(tag)
    except Exception:
        pass


def _set_operator_mode_v230(setup_mode):
    """Preserve the existing interlock and include new bench FC controls."""
    _original_set_operator_mode(setup_mode)

    # ARM and mode selection are bench/setup-only. DISARM is also kept inside
    # SETUP mode here to avoid creating a new in-flight kill control path.
    enabled = bool(setup_mode)
    for tag in (
        "fc_arm_button",
        "fc_disarm_button",
        "fc_mode_combo",
        "fc_set_mode_button",
    ):
        _safe_enable(tag, enabled)

    if dpg.does_item_exist("fc_command_state"):
        if enabled:
            _safe_set("fc_command_state", "BENCH CONTROL READY")
        else:
            _safe_set("fc_command_state", "LOCKED - ENTER SETUP MODE")


def _send_control_command(payload, status_prefix):
    """Send one request/ACK transaction without blocking the DearPyGUI loop."""
    global _command_busy

    with _command_lock:
        if _command_busy:
            _safe_set("fc_command_state", "BUSY - WAIT FOR CURRENT COMMAND")
            return
        _command_busy = True

    try:
        if not getattr(gcs, "is_connected", False):
            _safe_set("fc_command_state", "BLOCKED - GCS NOT CONNECTED")
            return

        tx_id = int(time.time_ns() & 0x7FFFFFFFFFFFFFFF)
        message = dict(payload)
        message["tx_id"] = tx_id

        _safe_set("fc_command_state", f"{status_prefix} - WAITING FC VERIFY")
        print(f"[GCS-CMD] TX {message.get('type')} tx_id={tx_id} payload={message}")

        old_timeout = None
        try:
            with gcs.control_socket_lock:
                try:
                    old_timeout = gcs.control_socket.gettimeout()
                except Exception:
                    old_timeout = None

                gcs.control_socket.settimeout(8.0)
                gcs.control_socket.sendto(
                    json.dumps(message).encode("utf-8"),
                    (gcs.drone_ip, gcs.CONTROL_PORT),
                )

                while True:
                    data, sender = gcs.control_socket.recvfrom(8192)
                    response = json.loads(data.decode("utf-8"))
                    if response.get("type") != "ACK":
                        continue
                    if response.get("tx_id") != tx_id:
                        continue
                    break

            ok = bool(response.get("ok", False))
            text = str(response.get("message") or ("VERIFIED" if ok else "FAILED"))
            detail = response.get("data") or {}

            if ok:
                _safe_set("fc_command_state", text)
            else:
                _safe_set("fc_command_state", f"FAIL - {text}")

            print(
                f"[GCS-CMD] ACK tx_id={tx_id} ok={ok} message={text} "
                f"detail={detail} sender={sender[0]}:{sender[1]}"
            )

        except Exception as exc:
            _safe_set("fc_command_state", f"COMMAND ERROR - {exc}")
            print(f"[GCS-CMD] ERROR {type(exc).__name__}: {exc}")
        finally:
            try:
                with gcs.control_socket_lock:
                    gcs.control_socket.settimeout(old_timeout)
            except Exception:
                pass

    finally:
        with _command_lock:
            _command_busy = False


def _start_command(payload, status_prefix):
    threading.Thread(
        target=_send_control_command,
        args=(payload, status_prefix),
        daemon=True,
    ).start()


def _arm_confirmed():
    try:
        dpg.configure_item("fc_arm_confirm_modal", show=False)
    except Exception:
        pass
    _start_command(
        {"type": "ARM", "bench_confirmed": True},
        "ARM REQUESTED",
    )


def _arm_requested():
    if not getattr(gcs, "edit_mode_unlocked", False):
        _safe_set("fc_command_state", "ARM BLOCKED - ENTER SETUP MODE")
        return
    if not getattr(gcs, "is_connected", False):
        _safe_set("fc_command_state", "ARM BLOCKED - GCS NOT CONNECTED")
        return

    try:
        dpg.configure_item("fc_arm_confirm_modal", show=True)
        vw = int(dpg.get_viewport_client_width())
        vh = int(dpg.get_viewport_client_height())
        dpg.configure_item(
            "fc_arm_confirm_modal",
            pos=(max(0, vw // 2 - 220), max(0, vh // 2 - 105)),
        )
    except Exception as exc:
        _safe_set("fc_command_state", f"ARM UI ERROR - {exc}")


def _disarm_requested():
    if not getattr(gcs, "edit_mode_unlocked", False):
        _safe_set("fc_command_state", "DISARM BLOCKED - ENTER SETUP MODE")
        return
    _start_command({"type": "DISARM"}, "DISARM REQUESTED")


def _set_mode_requested():
    if not getattr(gcs, "edit_mode_unlocked", False):
        _safe_set("fc_command_state", "MODE BLOCKED - ENTER SETUP MODE")
        return

    try:
        mode = str(dpg.get_value("fc_mode_combo") or "").upper()
    except Exception:
        mode = ""

    if mode not in BENCH_MODES:
        _safe_set("fc_command_state", "MODE BLOCKED - INVALID SELECTION")
        return

    _start_command(
        {"type": "SET_MODE", "mode": mode},
        f"MODE {mode} REQUESTED",
    )


def _create_gui_v230():
    _original_create_gui_v2232()

    try:
        # Compact addition under the existing servo test row. The servo buttons
        # remain placeholders until physical output channel + OPEN/CLOSE PWM
        # endpoints are explicitly confirmed.
        dpg.add_separator(parent="drop_control_card")
        dpg.add_text(
            "FC BENCH CONTROL",
            parent="drop_control_card",
            color=(145, 180, 220, 255),
        )

        with dpg.group(horizontal=True, parent="drop_control_card"):
            dpg.add_button(
                label="ARM",
                tag="fc_arm_button",
                width=80,
                height=34,
                enabled=False,
                callback=_arm_requested,
            )
            dpg.add_button(
                label="DISARM",
                tag="fc_disarm_button",
                width=90,
                height=34,
                enabled=False,
                callback=_disarm_requested,
            )
            dpg.add_combo(
                BENCH_MODES,
                default_value="STABILIZE",
                tag="fc_mode_combo",
                width=120,
                enabled=False,
            )
            dpg.add_button(
                label="SET MODE",
                tag="fc_set_mode_button",
                width=95,
                height=34,
                enabled=False,
                callback=_set_mode_requested,
            )

        dpg.add_text(
            "LOCKED - ENTER SETUP MODE",
            tag="fc_command_state",
            parent="drop_control_card",
            color=(180, 190, 200, 255),
        )

        # Update the operator footer so it is obvious that only the narrow
        # bench command subset has been enabled.
        try:
            dpg.add_text(
                "V2.3: ARM/DISARM + DISARMED MODE TX ENABLED / MOVEMENT TX DISABLED",
                parent="footer_group",
                color=(145, 180, 220, 255),
            )
        except Exception:
            pass

        # Apply the interlock once more now that the new tags exist.
        _set_operator_mode_v230(getattr(gcs, "edit_mode_unlocked", False))

    except Exception as exc:
        print(f"[GCS] V2.3 control widget warning: {exc}")

    # ARM confirmation is deliberately explicit. This is the only action in
    # this first TX build that can start the motors after FC pre-arm succeeds.
    try:
        with dpg.window(
            tag="fc_arm_confirm_modal",
            label="CONFIRM FC ARM",
            modal=True,
            show=False,
            no_resize=True,
            no_collapse=True,
            width=440,
            height=210,
        ):
            dpg.add_text("ARM WILL SEND A REAL COMMAND TO THE FLIGHT CONTROLLER")
            dpg.add_text("Use bench setup only. Ensure propellers/area are clear.")
            dpg.add_separator()
            with dpg.group(horizontal=True):
                dpg.add_button(
                    label="CANCEL",
                    width=180,
                    height=42,
                    callback=lambda: dpg.configure_item(
                        "fc_arm_confirm_modal", show=False
                    ),
                )
                dpg.add_button(
                    label="CONFIRM ARM",
                    width=180,
                    height=42,
                    callback=_arm_confirmed,
                )
    except Exception as exc:
        print(f"[GCS] ARM confirm modal warning: {exc}")


def main():
    # The V2.2.3.2 wrapper installs this hook into the frozen GCS. Replacing its
    # create function keeps all previous GPS/LiDAR/RSSI fixes intact.
    v2232._create_gui_v2232 = _create_gui_v230
    gcs._set_operator_mode = _set_operator_mode_v230

    print("[GCS] V2.3.0 GUARDED FC BENCH CONTROL")
    print("[GCS] Real TX enabled: ARM / DISARM / DISARMED flight-mode selection")
    print("[GCS] ARM requires explicit confirmation + existing CommandManager/VCT")
    print("[GCS] Servo PWM and movement/takeoff/mission TX remain DISABLED")
    print("[GCS] Field stream profile: 640x400 @20fps configured on Raspberry Pi")

    v2232.main()


if __name__ == "__main__":
    main()
