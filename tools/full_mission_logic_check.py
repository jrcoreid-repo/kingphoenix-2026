#!/usr/bin/env python3
"""Hardware-free orchestration smoke tests for Kingphoenix full mission."""

from __future__ import annotations

import sys
import threading
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pymavlink import mavutil

from core.event_bus import EventBus
from core.shared_state import SharedState
from mission.fc_mission_sync import FCMissionSync
from mission.full_mission_config import load_full_mission_config
from mission.mission_manager import MissionManager


class _Result:
    def __init__(self, ok=True, reason=""):
        self.ok = bool(ok)
        self.reason = str(reason)


class FakeCommand:
    def __init__(self):
        self.modes = []
        self.land_calls = 0

    def set_mode(self, mode):
        self.modes.append(str(mode).upper())
        return _Result(True)

    def land(self):
        self.land_calls += 1
        return _Result(True)


class FakeWaypoints:
    edit_unlocked = False

    def validate_strategy(self, strategy, total):
        return True, f"WP1-WP{total} complete"


class FakeFCSync:
    def __init__(self):
        self.ensure_calls = []
        self.current = []

    def ensure(self, waypoints, strategy, cfg):
        self.ensure_calls.append(str(strategy))
        return False, "MATCH"

    def set_current(self, seq):
        self.current.append(int(seq))


class FakeVision:
    def __init__(self):
        self.routes = []
        self.disabled = []
        self.shared_ai = SimpleNamespace(ready=True)

    def set_ai_route(self, camera, **kwargs):
        self.routes.append((str(camera), dict(kwargs)))

    def disable_ai(self, source="MISSION"):
        self.disabled.append(str(source))


class FakeServo:
    def __init__(self):
        self.logical_state = "CLOSE"
        self.open_calls = []

    def open(self, source="MISSION"):
        self.logical_state = "OPEN"
        self.open_calls.append(str(source))
        return True, f"SERVO OPEN / {source}"


class FakeTakeoff:
    def __init__(self):
        self.calls = 0

    def run(self, abort_event=None):
        assert abort_event is not None and not abort_event.is_set()
        self.calls += 1
        return True


class FakeFuzzy:
    def __init__(self, result=True):
        self.calls = 0
        self.result = bool(result)
        self.max_duration_s = None

    def run(self, abort_event=None, max_duration_s=None):
        assert abort_event is not None and not abort_event.is_set()
        self.calls += 1
        self.max_duration_s = max_duration_s
        return self.result

    def _zero(self):
        return None


class TestMissionManager(MissionManager):
    def __init__(self, *args, visual_success=False, **kwargs):
        super().__init__(*args, **kwargs)
        self.visual_success = bool(visual_success)
        self.waited = []

    def _preflight(self, strategy):
        return None

    def _wait_reached(self, seq, *, monitor_drop=False):
        self.waited.append(int(seq))
        if self.visual_success and int(seq) == int(self.cfg["mission"]["drop_wp"]):
            if not self.drop_done:
                ok, _ = self.servo.open(source="VISION_DROP_TEST")
                assert ok
                self.drop_done = True
                self.drop_armed = False
                self._disable_ai("DROP_COMPLETE_TEST")
        return None


def build_manager(visual_success, fuzzy_success=True):
    cfg = load_full_mission_config()
    state = SharedState()
    state.vehicle.connected = True
    state.vehicle.armed = False
    state.vehicle.mode = "GUIDED"

    command = FakeCommand()
    waypoints = FakeWaypoints()
    fc_sync = FakeFCSync()
    vision = FakeVision()
    servo = FakeServo()
    takeoff = FakeTakeoff()
    fuzzy = FakeFuzzy(fuzzy_success)

    manager = TestMissionManager(
        state,
        EventBus(),
        command,
        waypoints,
        fc_sync,
        vision,
        servo,
        takeoff,
        fuzzy,
        cfg,
        threading.Event(),
        visual_success=visual_success,
    )
    return manager, state, command, fc_sync, vision, servo, takeoff, fuzzy


def check_visual_drop_path():
    m, state, command, fc, vision, servo, takeoff, fuzzy = build_manager(True)
    m._run_mission("LEFT")

    assert state.mission.status == "COMPLETE"
    assert takeoff.calls == 1
    assert fc.ensure_calls == ["LEFT"]
    assert fc.current == [1]
    assert m.waited == list(range(1, 12))
    assert fuzzy.calls == 0
    assert servo.open_calls == ["VISION_DROP_TEST"]
    assert command.modes == ["AUTO"]
    assert command.land_calls == 1
    assert any(camera == "bottom" for camera, _ in vision.routes)


def check_fuzzy_fallback_path():
    m, state, command, fc, vision, servo, takeoff, fuzzy = build_manager(False)
    m._run_mission("RIGHT")

    assert state.mission.status == "COMPLETE"
    assert takeoff.calls == 1
    assert fc.ensure_calls == ["RIGHT"]
    assert fc.current == [1, 8]
    assert m.waited == list(range(1, 12))
    assert fuzzy.calls == 1
    assert servo.open_calls == ["FUZZY_DROP"]
    assert command.modes == ["AUTO", "AUTO"]
    assert command.land_calls == 1


def check_force_drop_path():
    m, state, command, fc, vision, servo, takeoff, fuzzy = build_manager(False, fuzzy_success=False)
    m._run_mission("LEFT")

    assert state.mission.status == "COMPLETE"
    assert fuzzy.calls == 1
    assert fuzzy.max_duration_s == float(m.cfg["drop"]["force_drop_after_fuzzy_s"])
    assert servo.open_calls == ["FORCE_DROP_FALLBACK"]
    assert fc.current == [1, 8]
    assert command.modes == ["AUTO", "AUTO"]
    assert command.land_calls == 1


def check_mission_compare_contract():
    cfg = load_full_mission_config()
    state = SharedState()
    sync_cfg = dict(cfg["mission_sync"])
    sync_cfg["acceptance_radius_m"] = float(cfg["waypoint"]["acceptance_radius_m"])
    sync = FCMissionSync(
        tx=None,
        state=state,
        protocol_queue=None,
        cfg=sync_cfg,
    )

    route = [{
        "wp": 1,
        "label": "LEFT/WP1",
        "lat": -7.0,
        "lon": 110.0,
        "alt_m": 1.0,
    }]
    home = {
        "seq": 0,
        "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        "lat": -7.0,
        "lon": 110.0,
        "alt_m": 0.0,
        "acceptance_radius_m": 0.0,
    }
    wp = {
        "seq": 1,
        "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        "lat": -7.0,
        "lon": 110.0,
        "alt_m": 1.0,
        "acceptance_radius_m": float(cfg["waypoint"]["acceptance_radius_m"]),
    }

    ok, _ = sync.compare(route, [home, dict(wp)])
    assert ok

    wrong_frame = dict(wp)
    wrong_frame["frame"] = mavutil.mavlink.MAV_FRAME_GLOBAL_INT
    ok, reason = sync.compare(route, [home, wrong_frame])
    assert not ok and "frame" in reason.lower()

    wrong_radius = dict(wp)
    wrong_radius["acceptance_radius_m"] += 0.50
    ok, reason = sync.compare(route, [home, wrong_radius])
    assert not ok and "acceptance" in reason.lower()


def main():
    check_visual_drop_path()
    check_fuzzy_fallback_path()
    check_force_drop_path()
    check_mission_compare_contract()
    print("[LOGIC-CHECK] visual-drop + fuzzy-center + force-drop + mission-compare PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
