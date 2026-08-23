from __future__ import annotations


class MavlinkConnection:
    """Owns the PyMAVLink connection object and local MAVLink identity."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.master = None

        self.source_system = int(cfg.get("source_system", 1))
        self.source_component = int(cfg.get("source_component", 191))
        self.target_system = int(cfg.get("target_system", 1))
        self.target_component = int(cfg.get("target_component", 1))

    def open(self):
        try:
            from pymavlink import mavutil

            kwargs = {
                "source_system": self.source_system,
                "source_component": self.source_component,
                "autoreconnect": True,
            }

            connection_string = self.cfg["connection"]

            # baud is meaningful for serial; harmless for many network links,
            # but only pass it for a likely serial device to keep intent clear.
            if str(connection_string).startswith("/dev/"):
                kwargs["baud"] = int(self.cfg.get("baud", 921600))

            self.master = mavutil.mavlink_connection(
                connection_string,
                **kwargs,
            )

            print(
                f"[MAVLINK] opening {connection_string} "
                f"source={self.source_system}:{self.source_component} "
                f"target={self.target_system}:{self.target_component}"
            )

        except Exception as e:
            print(f"[MAVLINK] unavailable: {e}")
            self.master = None

    def close(self):
        if self.master is not None:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None
