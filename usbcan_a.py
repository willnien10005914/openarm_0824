"""python-can backend for Waveshare USB-CAN-A (CH340 serial, 2 Mbps).

USB-CAN-A is USB-serial-CAN, not SocketCAN / gs_usb. It uses the same
variable-length packet protocol as the Seeed USB-CAN Analyzer:

  config: AA 55 12 ... checksum  (20 bytes)
  data:   AA <type> <id> <payload> 55
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Any, Optional

import can
from can import BusABC, CanProtocol, Message
from serial.tools import list_ports

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise can.CanInterfaceNotImplementedError("pyserial is required") from exc

SERIAL_BAUD_DEFAULT = 2_000_000
CAN_BITRATE_DEFAULT = 1_000_000
CH340_VID = 0x1A86
CH340_PID = 0x7523

CAN_BITRATE_CODES = {
    1_000_000: 0x01,
    800_000: 0x02,
    500_000: 0x03,
    400_000: 0x04,
    250_000: 0x05,
    200_000: 0x06,
    125_000: 0x07,
    100_000: 0x08,
    50_000: 0x09,
    20_000: 0x0A,
    10_000: 0x0B,
    5_000: 0x0C,
}


def list_usbcan_ports() -> list[str]:
    """Return COM / tty devices, CH340 USB-CAN-A adapters first."""
    ports = list(list_ports.comports())
    preferred = []
    others = []
    for port in ports:
        vid = getattr(port, "vid", None)
        pid = getattr(port, "pid", None)
        desc = (port.description or "").lower()
        hwid = (port.hwid or "").upper()
        is_ch340 = (
            (vid == CH340_VID and pid == CH340_PID)
            or "1A86:7523" in hwid
            or "ch340" in desc
            or "usb-serial" in desc
        )
        (preferred if is_ch340 else others).append(port.device)
    return preferred + others


def looks_like_serial_channel(channel: str) -> bool:
    name = (channel or "").strip()
    upper = name.upper()
    return upper.startswith("COM") or name.startswith("/dev/tty") or name.startswith("/dev/cu.")


def register_backend() -> None:
    """Register this module as python-can interface ``usbcan_a``."""
    import can.interfaces

    can.interfaces.BACKENDS["usbcan_a"] = ("usbcan_a", "UsbCanABus")


class UsbCanABus(BusABC):
    """Waveshare USB-CAN-A serial-to-CAN adapter."""

    def __init__(
        self,
        channel: str,
        bitrate: int = CAN_BITRATE_DEFAULT,
        baudrate: int = SERIAL_BAUD_DEFAULT,
        timeout: float = 0.05,
        **kwargs: Any,
    ) -> None:
        kwargs.pop("fd", None)
        kwargs.pop("receive_own_messages", None)
        if serial is None:
            raise can.CanInterfaceNotImplementedError("pyserial is required")
        if not channel:
            raise can.CanInitializationError("Must specify a serial port, for example COM3")
        if bitrate not in CAN_BITRATE_CODES:
            raise can.CanInitializationError(
                f"Unsupported CAN bitrate {bitrate}. "
                f"Choices: {sorted(CAN_BITRATE_CODES, reverse=True)}"
            )

        self.channel = channel
        self.bitrate = int(bitrate)
        self._can_protocol = CanProtocol.CAN_20
        self.channel_info = f"usbcan_a:{channel}@{self.bitrate}"
        self._rx_queue: queue.Queue[Message] = queue.Queue()
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._rx_buf = bytearray()

        try:
            self.ser = serial.Serial(
                port=channel,
                baudrate=int(baudrate),
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=timeout,
                write_timeout=1.0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
        except Exception as error:
            raise can.CanInitializationError(
                f"Could not open USB-CAN-A serial port {channel}: {error}"
            ) from error

        if hasattr(self.ser, "set_buffer_size"):
            try:
                self.ser.set_buffer_size(rx_size=256 * 1024, tx_size=64 * 1024)
            except Exception:
                pass

        self._configure_adapter()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="usbcan-a-rx", daemon=True
        )
        self._rx_thread.start()
        super().__init__(channel=channel, **kwargs)

    def _configure_adapter(self) -> None:
        frame = bytearray(20)
        frame[0] = 0xAA
        frame[1] = 0x55
        frame[2] = 0x12  # variable-length protocol
        frame[3] = CAN_BITRATE_CODES[self.bitrate]
        frame[4] = 0x01  # standard frame
        # filter ID / mask ID remain 0 = accept all
        frame[13] = 0x00  # normal mode
        frame[14] = 0x00  # auto retransmit enabled
        frame[19] = sum(frame[2:19]) & 0xFF
        with self._write_lock:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(frame)
            self.ser.flush()
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def send(self, msg: Message, timeout: Optional[float] = None) -> None:
        dlc = len(msg.data)
        if dlc > 8:
            raise can.CanOperationError("Classic CAN payload is limited to 8 bytes")
        frame_type = 0xC0 | dlc
        if msg.is_extended_id:
            frame_type |= 0x20
        if msg.is_remote_frame:
            frame_type |= 0x10
        id_bytes = msg.arbitration_id.to_bytes(4 if msg.is_extended_id else 2, "little")
        packet = bytes([0xAA, frame_type]) + id_bytes + bytes(msg.data) + bytes([0x55])
        try:
            with self._write_lock:
                self.ser.write(packet)
                self.ser.flush()
        except serial.PortNotOpenError as error:
            raise can.CanOperationError("writing to closed USB-CAN-A port") from error
        except serial.SerialTimeoutException as error:
            raise can.CanTimeoutError() from error

    def _recv_internal(self, timeout: Optional[float]) -> tuple[Optional[Message], bool]:
        try:
            if timeout == 0:
                msg = self._rx_queue.get_nowait()
            else:
                msg = self._rx_queue.get(timeout=timeout)
            return msg, True
        except queue.Empty:
            return None, False

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(4096)
            except Exception:
                break
            if not chunk:
                continue
            self._rx_buf.extend(chunk)
            self._parse_buffer()

    def _parse_buffer(self) -> None:
        buf = self._rx_buf
        while True:
            start = buf.find(b"\xaa")
            if start < 0:
                buf.clear()
                return
            if start > 0:
                del buf[:start]
            if len(buf) < 3:
                return
            frame_type = buf[1]
            if (frame_type & 0xC0) != 0xC0:
                del buf[0]
                continue
            dlc = frame_type & 0x0F
            if dlc > 8:
                del buf[0]
                continue
            extended = bool(frame_type & 0x20)
            remote = bool(frame_type & 0x10)
            id_len = 4 if extended else 2
            total = 2 + id_len + dlc + 1
            if len(buf) < total:
                return
            if buf[total - 1] != 0x55:
                del buf[0]
                continue
            packet = bytes(buf[:total])
            del buf[:total]
            can_id = int.from_bytes(packet[2 : 2 + id_len], "little")
            can_id &= 0x1FFFFFFF if extended else 0x7FF
            data = packet[2 + id_len : 2 + id_len + dlc]
            msg = Message(
                timestamp=time.time(),
                arbitration_id=can_id,
                is_extended_id=extended,
                is_remote_frame=remote,
                is_rx=True,
                dlc=dlc,
                data=data,
                channel=self.channel,
            )
            try:
                self._rx_queue.put_nowait(msg)
            except queue.Full:
                pass

    def shutdown(self) -> None:
        self._stop.set()
        super().shutdown()
        try:
            self.ser.close()
        except Exception:
            pass
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)

    @staticmethod
    def _detect_available_configs() -> list[dict[str, Any]]:
        return [{"interface": "usbcan_a", "channel": port} for port in list_usbcan_ports()]
