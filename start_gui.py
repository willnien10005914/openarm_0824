"""Launch the custom DaMiao GUI (live position + click-to-move dial).

On Windows this uses:
  - Zubax Babel via python-can slcan (COM port)
  - Waveshare USB-CAN-A via the usbcan_a backend

There is no native socketcan on Windows.

Usage:
    python start_gui.py
    python start_gui.py --port 5000
"""

from __future__ import annotations

import argparse

from gui_server import run_server


def main() -> None:
    parser = argparse.ArgumentParser(description="Custom DaMiao GUI for Babel/USB-CAN-A")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    args = parser.parse_args()
    run_server(host=args.host, port=args.port)


if __name__ == "__main__":
    main()
