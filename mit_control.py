"""Scan DaMiao Node ID over USB-CAN-A, then move slowly in MIT mode.

Safety defaults:
  - amplitude 0.20 rad (~11 deg)
  - frequency 0.08 Hz
  - stiffness 4.0, damping 0.8
  - command rate 50 Hz

Usage:
    python mit_control.py                 # scan Node ID, then slow MIT motion
    python mit_control.py --scan-only     # only read Node ID / feedback
    python mit_control.py --port COM5
    python mit_control.py --id 1 --hold   # enable and hold current position
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import patch_damiao
from can_adapter import detect_kind, list_adapter_ports, looks_like_serial_channel

MOTOR_TYPE = "6248P"
ESC_ID_RID = 8  # receive / command Node ID
MST_ID_RID = 7  # feedback ID
CTRL_MODE_RID = 10


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Slow MIT control for DM-J6248P-2EC")
    parser.add_argument("--port", default="", help="COM port, e.g. COM5")
    parser.add_argument("--adapter", default="auto", choices=("auto", "slcan", "usbcan_a"))
    parser.add_argument("--id", type=lambda x: int(x, 0), default=0, help="Known motor ESC_ID (skip scan)")
    parser.add_argument("--feedback-id", type=lambda x: int(x, 0), default=0)
    parser.add_argument("--motor-type", default=MOTOR_TYPE)
    parser.add_argument("--scan-only", action="store_true", help="Scan and print IDs, do not enable")
    parser.add_argument("--hold", action="store_true", help="Hold current position instead of oscillating")
    parser.add_argument("--amplitude", type=float, default=0.20, help="MIT position amplitude in rad")
    parser.add_argument("--freq", type=float, default=0.08, help="Sine frequency in Hz (keep low)")
    parser.add_argument("--kp", type=float, default=4.0, help="MIT stiffness")
    parser.add_argument("--kd", type=float, default=0.8, help="MIT damping")
    parser.add_argument("--rate", type=float, default=50.0, help="Command frequency in Hz")
    parser.add_argument("--scan-end", type=lambda x: int(x, 0), default=0x10)
    return parser.parse_args()


def resolve_port(requested: str) -> str:
    ports = list_adapter_ports()
    if requested:
        if not looks_like_serial_channel(requested):
            raise SystemExit(f"Expected a COM port, got {requested!r}")
        return requested
    if not ports:
        raise SystemExit(
            "No COM port found. Plug in Zubax Babel or USB-CAN-A, "
            "and close any other program using the port."
        )
    if len(ports) > 1:
        for item in ports:
            print(item["label"])
    print(f"Using {ports[0]['device']} ({ports[0]['kind']})")
    return ports[0]["device"]


def scan_motors(controller, motor_type: str, end_id: int) -> list[int]:
    from damiao_motor.core.controller import DaMiaoController

    assert isinstance(controller, DaMiaoController)
    found: list[int] = []
    controller.flush_bus()
    for motor_id in range(0x01, end_id + 1):
        try:
            motor = controller.add_motor(
                motor_id=motor_id, feedback_id=0x00, motor_type=motor_type
            )
        except ValueError:
            motor = controller.get_motor(motor_id)
        motor.send_cmd_mit(
            target_position=0.0,
            target_velocity=0.0,
            stiffness=0.0,
            damping=0.0,
            feedforward_torque=0.0,
        )
        time.sleep(0.02)

    deadline = time.perf_counter() + 0.6
    responded: set[int] = set()
    while time.perf_counter() < deadline:
        controller.poll_feedback()
        for motor_id, motor in controller.motors.items():
            state = motor.get_states() or {}
            if state.get("can_id") is not None and motor_id not in responded:
                responded.add(motor_id)
                found.append(motor_id)
        time.sleep(0.01)
    return found


def read_ids(motor) -> dict[str, object]:
    info: dict[str, object] = {
        "esc_id_arg": motor.motor_id,
        "feedback_id_arg": motor.feedback_id,
    }
    try:
        info["ESC_ID"] = motor.get_register(ESC_ID_RID, timeout=0.8)
    except Exception as exc:
        info["ESC_ID"] = f"(read failed: {exc})"
    try:
        info["MST_ID"] = motor.get_register(MST_ID_RID, timeout=0.8)
    except Exception as exc:
        info["MST_ID"] = f"(read failed: {exc})"
    try:
        info["CTRL_MODE"] = motor.get_register(CTRL_MODE_RID, timeout=0.8)
    except Exception as exc:
        info["CTRL_MODE"] = f"(read failed: {exc})"
    return info


def print_state(motor) -> None:
    state = motor.get_states() or {}
    print(
        "  status={status} pos={pos:.4f} rad vel={vel:.4f} rad/s "
        "torq={torq:.3f} Nm Tmos={t_mos} Trotor={t_rotor}".format(
            status=state.get("status", "--"),
            pos=float(state.get("pos") or 0.0),
            vel=float(state.get("vel") or 0.0),
            torq=float(state.get("torq") or 0.0),
            t_mos=state.get("t_mos", "--"),
            t_rotor=state.get("t_rotor", "--"),
        )
    )


def main() -> None:
    args = parse_args()
    patch_damiao.apply()
    from damiao_motor import DaMiaoController

    port = resolve_port(args.port)
    kind = detect_kind(port, args.adapter)
    print(f"Opening {kind} {port} at 1 Mbps, motor type {args.motor_type}")
    print("Keep the motor mounted and the area clear. Ctrl+C stops and disables.")

    controller = DaMiaoController(channel=port, bustype=kind)
    try:
        if args.id:
            found = [args.id]
            controller.add_motor(
                motor_id=args.id,
                feedback_id=args.feedback_id,
                motor_type=args.motor_type,
            )
        else:
            print("Scanning motor IDs 0x01-0x{:02X} ...".format(args.scan_end))
            found = scan_motors(controller, args.motor_type, args.scan_end)

        if not found:
            raise SystemExit(
                "No motor replied. Check 24V power, CAN_H/CAN_L, 120 ohm switch, "
                "and that USB-CAN.exe is closed."
            )

        print("Detected motors:")
        for motor_id in found:
            motor = controller.get_motor(motor_id)
            ids = read_ids(motor)

            def _fmt_id(value: object) -> str:
                if isinstance(value, int):
                    return "0x{:02X} ({:d})".format(value, value)
                return str(value)

            print(
                "  Node/ESC_ID(scan)={}  ESC_ID(reg8)={}  MST_ID(reg7)={}  CTRL_MODE={}".format(
                    _fmt_id(motor_id),
                    _fmt_id(ids.get("ESC_ID")),
                    _fmt_id(ids.get("MST_ID")),
                    ids.get("CTRL_MODE"),
                )
            )
            print_state(motor)

        if args.scan_only:
            print("Scan-only done. Motor was not enabled.")
            return

        motor = controller.get_motor(found[0])
        print(f"Controlling motor 0x{motor.motor_id:02X} in MIT mode (slow).")
        motor.enable()
        time.sleep(0.05)
        motor.ensure_control_mode("MIT")
        time.sleep(0.05)

        # Wait for a valid position sample before moving.
        origin = 0.0
        for _ in range(20):
            state = motor.get_states() or {}
            if state.get("pos") is not None:
                origin = float(state["pos"])
                break
            motor.send_cmd_mit(origin, 0.0, args.kp, args.kd, 0.0)
            time.sleep(0.02)
        print(f"Current position origin = {origin:.4f} rad")

        period = 1.0 / max(args.rate, 1.0)
        start = time.perf_counter()
        last_print = 0.0
        omega = 2.0 * math.pi * max(args.freq, 0.0)
        try:
            while True:
                now = time.perf_counter()
                if args.hold:
                    target = origin
                    vel_ff = 0.0
                else:
                    phase = omega * (now - start)
                    target = origin + args.amplitude * math.sin(phase)
                    vel_ff = args.amplitude * omega * math.cos(phase)
                motor.send_cmd_mit(
                    target_position=target,
                    target_velocity=vel_ff,
                    stiffness=args.kp,
                    damping=args.kd,
                    feedforward_torque=0.0,
                )
                if now - last_print >= 0.25:
                    print_state(motor)
                    last_print = now
                elapsed = time.perf_counter() - now
                time.sleep(max(0.0, period - elapsed))
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            pos = (motor.get_states() or {}).get("pos")
            hold_pos = origin if pos is None else float(pos)
            for _ in range(8):
                motor.send_cmd_mit(hold_pos, 0.0, args.kp, args.kd, 0.0)
                time.sleep(0.02)
            motor.disable()
            print("Motor disabled.")
    finally:
        controller.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
