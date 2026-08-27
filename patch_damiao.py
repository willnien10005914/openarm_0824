"""Patch damiao-motor so COM adapters use slcan (Babel) or usbcan_a (Waveshare)."""

from __future__ import annotations

import os
import sys
import threading

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from can_adapter import (  # noqa: E402
    detect_kind,
    list_adapter_ports,
    list_serial_devices,
    looks_like_serial_channel,
    open_serial_bus,
)
from usbcan_a import register_backend  # noqa: E402


def apply() -> None:
    register_backend()
    _patch_controller()
    _patch_gui()


def _patch_controller() -> None:
    from damiao_motor.core.controller import DaMiaoController

    original_init = DaMiaoController.__init__

    def patched_init(self, channel: str = "can0", bustype: str = "socketcan") -> None:
        requested = bustype if bustype in ("slcan", "usbcan_a") else "auto"
        if requested != "auto" or looks_like_serial_channel(str(channel)):
            kind = detect_kind(str(channel), requested)
            self.bus = open_serial_bus(str(channel), kind)
            self.motors = {}
            self._motors_by_feedback = {}
            self._polling_thread = None
            self._polling_active = False
            self._polling_lock = threading.Lock()
            self._adapter_kind = kind
            return
        original_init(self, channel=channel, bustype=bustype)
        self._adapter_kind = bustype

    DaMiaoController.__init__ = patched_init  # type: ignore[method-assign]


def _patch_gui() -> None:
    from flask import jsonify, request

    from damiao_motor.gui import web_gui

    def list_can_interfaces():
        return jsonify(
            {
                "success": True,
                "interfaces": list_serial_devices(),
            }
        )

    web_gui.app.view_functions["list_can_interfaces"] = list_can_interfaces

    original_connect = web_gui.connect

    def connect():
        data = request.json or {}
        channel = str(data.get("channel", "")).strip()
        adapter = str(data.get("bustype") or data.get("adapter") or "auto")
        if looks_like_serial_channel(channel) or not channel:
            ports = list_adapter_ports()
            if not channel:
                if not ports:
                    return jsonify(
                        {
                            "success": False,
                            "error": "找不到 COM 埠。請插入 Zubax Babel 或 USB-CAN-A。",
                        }
                    ), 500
                channel = ports[0]["device"]
            try:
                kind = detect_kind(channel, adapter)
                web_gui.init_controller(channel=channel, bustype=kind)
                return jsonify({"success": True, "channel": channel, "adapter": kind})
            except Exception as exc:
                return jsonify({"success": False, "error": str(exc)}), 500
        return original_connect()

    web_gui.app.view_functions["connect"] = connect

    original_scan = web_gui.scan

    def scan():
        data = dict(request.get_json(silent=True) or {})
        if not data.get("motor_type") or data.get("motor_type") == "4310":
            data["motor_type"] = "6248P"
        original_get_json = request.get_json

        def patched_get_json(*_args, **_kwargs):
            return data

        request.get_json = patched_get_json  # type: ignore[method-assign]
        try:
            return original_scan()
        finally:
            request.get_json = original_get_json  # type: ignore[method-assign]

    web_gui.app.view_functions["scan"] = scan

    @web_gui.app.after_request
    def _inject_serial_defaults(response):
        content_type = response.headers.get("Content-Type", "")
        if "text/html" not in content_type:
            return response
        html = response.get_data(as_text=True)
        ports = list_adapter_ports()
        default_port = ports[0]["device"] if ports else "COM3"
        snippet = f"""
<script>
(function() {{
  const channel = document.getElementById('can_channel');
  if (channel) {{
    channel.value = {default_port!r};
    channel.placeholder = {default_port!r};
    channel.title = 'COM port: Zubax Babel (slcan) or USB-CAN-A. CAN 1 Mbps.';
  }}
  const origFetch = window.fetch.bind(window);
  window.fetch = function(url, opts) {{
    if (typeof url === 'string' && url.indexOf('/api/scan') !== -1 && opts && opts.body) {{
      try {{
        const body = JSON.parse(opts.body);
        body.motor_type = '6248P';
        opts = Object.assign({{}}, opts, {{ body: JSON.stringify(body) }});
      }} catch (e) {{}}
    }}
    return origFetch(url, opts);
  }};
}})();
</script>
</body>
"""
        if "</body>" in html and "Zubax Babel (slcan)" not in html:
            html = html.replace("</body>", snippet, 1)
            response.set_data(html)
        return response
