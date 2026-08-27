"""Launch the official damiao-motor Web GUI, patched for COM adapters.

Prefer `python start_gui.py` for the click-to-move position dial.
Channel must be a COM port (Zubax Babel slcan or USB-CAN-A), not can0.
"""

from __future__ import annotations

import argparse

import patch_damiao
from can_adapter import list_adapter_ports


def main() -> None:
    parser = argparse.ArgumentParser(description="Official DaMiao GUI + COM adapter patch")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    patch_damiao.apply()
    from damiao_motor.gui.web_gui import run_server

    ports = list_adapter_ports()
    print("Official damiao-motor GUI (patched: slcan / usbcan_a)")
    print("Open http://{}:{}".format(args.host, args.port))
    if ports:
        for item in ports:
            print("  ", item["label"])
        print("Channel 請填 COM 埠，不要填 can0。")
    print("Close other programs using the COM port before connecting.")
    run_server(host=args.host, port=args.port, debug=args.debug)


if __name__ == "__main__":
    main()
