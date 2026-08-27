"""Detect and open Windows serial CAN adapters: Zubax Babel (slcan) or USB-CAN-A."""

from __future__ import annotations

import threading
from typing import Any, Optional

from serial.tools import list_ports

CAN_BITRATE_DEFAULT = 1_000_000
SLCAN_TTY_BAUD = 115200

CH340_VID, CH340_PID = 0x1A86, 0x7523
# OpenMoko VID used by Zubax Babel CDC ACM
BABEL_VID, BABEL_PID = 0x1D50, 0x60C7


def looks_like_serial_channel(channel: str) -> bool:
    name = (channel or "").strip()
    upper = name.upper()
    return (
        upper.startswith("COM")
        or name.startswith("/dev/tty")
        or name.startswith("/dev/cu.")
    )


def _port_kind(port: Any) -> str:
    vid = getattr(port, "vid", None)
    pid = getattr(port, "pid", None)
    desc = (port.description or "").lower()
    hwid = (port.hwid or "").upper()
    manufacturer = (getattr(port, "manufacturer", None) or "").lower()
    product = (getattr(port, "product", None) or "").lower()
    blob = " ".join((desc, hwid, manufacturer, product))

    if (vid == CH340_VID and pid == CH340_PID) or "1A86:7523" in hwid or "ch340" in blob:
        return "usbcan_a"
    if (
        (vid == BABEL_VID and pid == BABEL_PID)
        or "1D50:60C7" in hwid
        or "babel" in blob
        or "zubax" in blob
        or "canface" in blob
    ):
        return "slcan"
    if "stm32" in blob or "virtual com" in blob:
        return "slcan"
    return "slcan"


def _port_label(device: str, kind: str, description: str) -> str:
    if kind == "usbcan_a":
        name = "Waveshare USB-CAN-A"
    elif "babel" in (description or "").lower() or "zubax" in (description or "").lower():
        name = "Zubax Babel"
    else:
        name = "Zubax Babel / SLCAN"
    proto = "slcan" if kind == "slcan" else "usbcan_a"
    return f"{device} — {name} ({proto})"


def list_adapter_ports() -> list[dict[str, str]]:
    """COM ports with detected adapter kind. Babel/SLCAN first, then USB-CAN-A."""
    items: list[dict[str, str]] = []
    for port in list_ports.comports():
        kind = _port_kind(port)
        items.append(
            {
                "device": port.device,
                "kind": kind,
                "description": port.description or "",
                "label": _port_label(port.device, kind, port.description or ""),
            }
        )
    items.sort(key=lambda p: (0 if p["kind"] == "slcan" else 1, p["device"]))
    return items


def list_serial_devices() -> list[str]:
    return [item["device"] for item in list_adapter_ports()]


def detect_kind(channel: str, requested: str = "auto") -> str:
    if requested in ("slcan", "usbcan_a"):
        return requested
    for item in list_adapter_ports():
        if item["device"].upper() == (channel or "").strip().upper():
            return item["kind"]
    return "slcan"


class ThreadSafeBus:
    """Serialize send/recv. python-can slcan shares one COM port with no lock."""

    def __init__(self, bus: Any) -> None:
        self._bus = bus
        self._io_lock = threading.RLock()

    def send(self, msg: Any, timeout: Optional[float] = None) -> Any:
        with self._io_lock:
            return self._bus.send(msg, timeout=timeout)

    def recv(self, timeout: Optional[float] = None) -> Any:
        with self._io_lock:
            return self._bus.recv(timeout=timeout)

    def shutdown(self) -> Any:
        with self._io_lock:
            return self._bus.shutdown()

    def flush_tx_buffer(self) -> Any:
        flush = getattr(self._bus, "flush_tx_buffer", None)
        if flush is None:
            return None
        with self._io_lock:
            return flush()

    def __getattr__(self, name: str) -> Any:
        return getattr(self._bus, name)


def open_serial_bus(channel: str, kind: str, bitrate: int = CAN_BITRATE_DEFAULT):
    import can

    from usbcan_a import register_backend

    register_backend()
    if kind == "usbcan_a":
        bus = can.interface.Bus(
            channel=channel,
            bustype="usbcan_a",
            bitrate=bitrate,
            baudrate=2_000_000,
            ignore_config=True,
        )
    else:
        bus = can.interface.Bus(
            channel=channel,
            bustype="slcan",
            bitrate=bitrate,
            tty_baudrate=SLCAN_TTY_BAUD,
            sleep_after_open=1.0,
            ignore_config=True,
        )
    return ThreadSafeBus(bus)
