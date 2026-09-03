#!/usr/bin/env python3
"""Canonical Kingphoenix Raspberry Pi entrypoint.

`python3 main.py` intentionally launches the current validated V2.3.2 runtime
stack so operators do not have to remember versioned entrypoint filenames.
That stack includes V2.2.3 expanded MAVLink telemetry/readiness plus V2.3
GCS ARM/DISARM/disarmed-mode control and FC STATUSTEXT metadata.
"""

from main_v2_3_2 import main


if __name__ == "__main__":
    main()
