"""Custom DaMiao GUI: live position + click-to-move dial (Babel slcan / USB-CAN-A)."""

from __future__ import annotations

import math
import os
import threading
import time
import traceback
from typing import Any, Optional

from flask import Flask, jsonify, render_template, request

import patch_damiao
from can_adapter import detect_kind, list_adapter_ports, looks_like_serial_channel

MOTOR_TYPE = "6248P"
TWO_PI = 2.0 * math.pi
CMD_HZ = 25.0
MAX_SPEED_DEG_S = 60.0
DEFAULT_SPEED_DEG_S = 18.0
DEFAULT_KP = 4.0
DEFAULT_KD = 2.0
SCAN_ID_MIN = 0x01
SCAN_ID_MAX = 0x10
ESC_ID_RID = 8
MST_ID_RID = 7

_template_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "templates")
app = Flask(__name__, template_folder=_template_dir)

_lock = threading.RLock()
_controller = None
_motor = None
_motors_found: list[dict[str, Any]] = []
_enabled = False
_cmd_pos: Optional[float] = None
_target_pos: Optional[float] = None
_max_speed_rad_s = math.radians(DEFAULT_SPEED_DEG_S)
_kp = DEFAULT_KP
_kd = DEFAULT_KD
_loop_stop = threading.Event()
_loop_thread: Optional[threading.Thread] = None
_channel = ""
_adapter_kind = ""
_error_seq = 0
_errors: list[dict[str, Any]] = []
_send_fail = 0


def _note_error(msg: str) -> None:
    global _error_seq
    text = str(msg).strip() or "unknown error"
    print("[gui]", text, flush=True)
    with _lock:
        _error_seq += 1
        _errors.append({"seq": _error_seq, "msg": text, "t": time.strftime("%H:%M:%S")})
        del _errors[:-20]


def _state_payload() -> dict[str, Any]:
    motor = _motor
    raw = motor.get_states() if motor else {}
    pos = raw.get("pos")
    vel = raw.get("vel")
    return {
        "connected": _controller is not None,
        "channel": _channel,
        "adapter": _adapter_kind,
        "enabled": _enabled,
        "motor_id": None if motor is None else motor.motor_id,
        "motors": list(_motors_found),
        "status": raw.get("status"),
        "pos_rad": pos,
        "pos_deg": None if pos is None else math.degrees(pos),
        "pos_deg_mod": None if pos is None else (math.degrees(pos) % 360.0 + 360.0) % 360.0,
        "vel": vel,
        "torq": raw.get("torq"),
        "t_mos": raw.get("t_mos"),
        "t_rotor": raw.get("t_rotor"),
        "target_rad": _target_pos,
        "target_deg_mod": None
        if _target_pos is None
        else (_target_pos * 180.0 / math.pi % 360.0 + 360.0) % 360.0,
        "cmd_rad": _cmd_pos,
        "kp": _kp,
        "kd": _kd,
        "max_deg_s": math.degrees(_max_speed_rad_s),
        "errors": list(_errors),
        "error_seq": _error_seq,
    }


def _nearest_target(current: float, clicked_deg: float) -> float:
    clicked = math.radians(clicked_deg % 360.0)
    current_mod = current % TWO_PI
    delta = (clicked - current_mod + math.pi) % TWO_PI - math.pi
    return current + delta


def _send_mit(motor, pos: float, vel: float, kp: float, kd: float) -> None:
    # send_raw skips damiao's per-command enable/clear-error side effects.
    data = motor.encode_cmd_msg(pos, vel, 0.0, kp, kd)
    motor.send_raw(data)


def _state_pos(motor) -> Optional[float]:
    state = (motor.get_states() if motor is not None else None) or {}
    pos = state.get("pos")
    return None if pos is None else float(pos)


def _ensure_motor(controller, motor_id: int, feedback_id: int = 0x00):
    try:
        return controller.add_motor(
            motor_id=motor_id, feedback_id=feedback_id, motor_type=MOTOR_TYPE
        )
    except ValueError:
        return controller.get_motor(motor_id)


def _drop_motor(controller, motor_id: int) -> None:
    controller.motors.pop(motor_id, None)
    controller._motors_by_feedback.pop(motor_id, None)


def _remap_motor(controller, motor, new_id: int, new_mst: int) -> None:
    old_id = motor.motor_id
    if new_id != old_id and new_id in controller.motors and controller.motors[new_id] is not motor:
        _drop_motor(controller, new_id)
    if old_id in controller.motors and controller.motors[old_id] is motor:
        del controller.motors[old_id]
    controller._motors_by_feedback.pop(old_id, None)
    motor.motor_id = new_id
    motor.feedback_id = new_mst
    controller.motors[new_id] = motor
    controller._motors_by_feedback[new_id] = motor


def _run_as_id(motor, can_id: int, fn) -> None:
    saved = motor.motor_id
    motor.motor_id = can_id
    try:
        fn()
    finally:
        motor.motor_id = saved


def _query_status(motor) -> None:
    motor.state = {}
    motor.request_motor_feedback()


def _wait_pos(motor, timeout: float = 0.45) -> Optional[float]:
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        pos = _state_pos(motor)
        if pos is not None:
            return pos
        try:
            motor.request_motor_feedback()
        except Exception:
            pass
        time.sleep(0.02)
    return _state_pos(motor)


def _hold_at(motor, pos: float, kp: float, kd: float, times: int = 6) -> None:
    for _ in range(times):
        _send_mit(motor, pos, 0.0, kp, kd)
        time.sleep(0.015)


def _read_register_fresh(motor, rid: int, timeout: float = 0.7) -> Optional[int]:
    with motor.registers_lock:
        motor.registers.pop(rid, None)
    try:
        value = motor.get_register(rid, timeout=timeout)
    except Exception:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _control_loop() -> None:
    global _cmd_pos, _send_fail, _enabled
    dt = 1.0 / CMD_HZ
    while not _loop_stop.is_set():
        t0 = time.perf_counter()
        with _lock:
            motor = _motor
            enabled = _enabled
            cmd = _cmd_pos
            target = _target_pos
            max_speed = _max_speed_rad_s
            kp = _kp
            kd = _kd
        if motor is not None and enabled and cmd is not None and target is not None:
            err = target - cmd
            step = max_speed * dt
            if abs(err) <= step:
                cmd = target
                vel_ff = 0.0
            else:
                cmd += math.copysign(step, err)
                vel_ff = math.copysign(max_speed, err)
            try:
                _send_mit(motor, cmd, vel_ff, kp, kd)
                _send_fail = 0
                with _lock:
                    _cmd_pos = cmd
            except Exception as exc:
                _send_fail += 1
                _note_error(f"MIT 送指令失敗（{_send_fail}）：{exc}")
                if _send_fail >= 8:
                    with _lock:
                        _enabled = False
                    _note_error("連續送指令失敗，已自動 Disable。請檢查 COM / 重新連線。")
                    _send_fail = 0
        elapsed = time.perf_counter() - t0
        time.sleep(max(0.0, dt - elapsed))


def _ensure_loop() -> None:
    global _loop_thread
    if _loop_thread is not None and _loop_thread.is_alive():
        return
    _loop_stop.clear()
    _loop_thread = threading.Thread(target=_control_loop, name="mit-loop", daemon=True)
    _loop_thread.start()


def _shutdown_controller() -> None:
    global _controller, _motor, _enabled, _cmd_pos, _target_pos, _motors_found, _channel, _adapter_kind
    with _lock:
        _enabled = False
        _cmd_pos = None
        _target_pos = None
        controller = _controller
        _controller = None
        _motor = None
        _motors_found = []
        _channel = ""
        _adapter_kind = ""
    if controller is not None:
        try:
            controller.shutdown()
        except Exception as exc:
            _note_error(f"關閉轉接器失敗：{exc}")


def _motor_info(motor) -> dict[str, Any]:
    state = motor.get_states() or {}
    arb = state.get("arbitration_id")
    info: dict[str, Any] = {
        "id": motor.motor_id,
        "esc_id": motor.motor_id,
        "mst_id": motor.feedback_id if motor.feedback_id else arb,
        "arb_id": arb,
        "pos": state.get("pos"),
        "status": state.get("status"),
    }
    esc = _read_register_fresh(motor, ESC_ID_RID)
    mst = _read_register_fresh(motor, MST_ID_RID)
    if esc is not None:
        info["esc_id"] = esc
    if mst is not None:
        info["mst_id"] = mst
        if motor.feedback_id != mst:
            motor.feedback_id = mst
    return info


def _scan(controller, motor_type: str) -> list[dict[str, Any]]:
    """Probe each Node ID one-by-one. Burst scans drop replies on USB-CAN-A."""
    del motor_type  # motors are created with MOTOR_TYPE
    controller.flush_bus()
    found_ids: set[int] = set()
    for motor_id in range(SCAN_ID_MIN, SCAN_ID_MAX + 1):
        motor = _ensure_motor(controller, motor_id)
        motor.state = {}
        try:
            _query_status(motor)
        except Exception:
            _send_mit(motor, 0.0, 0.0, 0.0, 0.0)
        deadline = time.perf_counter() + 0.12
        while time.perf_counter() < deadline:
            time.sleep(0.015)
            state = motor.get_states() or {}
            if state.get("can_id") is None:
                continue
            found_ids.add(motor_id)
            break

    for motor_id in list(controller.motors):
        if motor_id not in found_ids:
            _drop_motor(controller, motor_id)

    found: list[dict[str, Any]] = []
    for motor_id in sorted(found_ids):
        motor = controller.get_motor(motor_id)
        found.append(_motor_info(motor))
    return found


def _select_scanned_motor(controller, motors: list[dict[str, Any]], prefer_id: Optional[int]):
    if not motors:
        return None, None
    chosen_id = prefer_id if prefer_id in {item["id"] for item in motors} else motors[0]["id"]
    motor = controller.get_motor(chosen_id)
    pos = _state_pos(motor)
    if pos is None:
        pos = _wait_pos(motor, timeout=0.3)
    return motor, pos


@app.route("/")
def index():
    return render_template("custom_gui.html")


@app.route("/api/ports")
def ports():
    return jsonify({"success": True, "ports": list_adapter_ports()})


@app.route("/api/connect", methods=["POST"])
def connect():
    global _controller, _motor, _motors_found, _channel, _adapter_kind, _cmd_pos, _target_pos
    data = request.get_json(silent=True) or {}
    channel = str(data.get("channel") or "").strip()
    adapter = str(data.get("adapter") or "auto")
    ports = list_adapter_ports()
    if not channel:
        if not ports:
            return jsonify({"success": False, "error": "找不到 COM 埠，請插入 Zubax Babel 或 USB-CAN-A。"}), 400
        channel = ports[0]["device"]
    if not looks_like_serial_channel(channel):
        return jsonify(
            {
                "success": False,
                "error": "請選 COM 埠（例如 COM5）。Windows 沒有 socketcan。",
            }
        ), 400

    _shutdown_controller()

    try:
        from damiao_motor import DaMiaoController

        kind = detect_kind(channel, adapter)
        controller = DaMiaoController(channel=channel, bustype=kind)
        motors = _scan(controller, MOTOR_TYPE)
        if not motors:
            try:
                controller.shutdown()
            except Exception:
                pass
            return jsonify(
                {
                    "success": False,
                    "error": "轉接器已開，但沒掃到馬達。請檢查 24V、CAN_H/CAN_L、120Ω 終端，並關閉其他佔用 COM 的程式。若 bus 上有兩顆但都是 ID 0x01，請先單顆接上寫入不同 Node ID。",
                }
            ), 400
        motor, pos = _select_scanned_motor(controller, motors, None)
        if motor is None:
            raise RuntimeError("掃到馬達但無法選取")
        if pos is None:
            pos = 0.0
            _note_error("連線時讀不到位置，暫以 0 顯示；Enable 前會再讀一次。")
        with _lock:
            _controller = controller
            _motor = motor
            _motors_found = motors
            _channel = channel
            _adapter_kind = kind
            _cmd_pos = pos
            _target_pos = pos
            payload = _state_payload()
        _ensure_loop()
        return jsonify({"success": True, "state": payload})
    except Exception as exc:
        _note_error(f"連線失敗：{exc}")
        traceback.print_exc()
        _shutdown_controller()
        return jsonify({"success": False, "error": str(exc)}), 500


@app.route("/api/disconnect", methods=["POST"])
def disconnect():
    motor = None
    enabled = False
    with _lock:
        motor = _motor
        enabled = _enabled
        _enabled = False
    if motor is not None and enabled:
        try:
            motor.disable()
        except Exception as exc:
            _note_error(f"Disable 失敗：{exc}")
    _shutdown_controller()
    return jsonify({"success": True})


@app.route("/api/select", methods=["POST"])
def select_motor():
    global _motor, _cmd_pos, _target_pos, _enabled
    data = request.get_json(silent=True) or {}
    motor_id = int(data.get("id", 0))
    with _lock:
        if _controller is None:
            return jsonify({"success": False, "error": "尚未連線"}), 400
        if _motor is not None and _motor.motor_id == motor_id:
            return jsonify({"success": True, "state": _state_payload()})
        try:
            motor = _controller.get_motor(motor_id)
        except KeyError:
            return jsonify({"success": False, "error": f"沒有馬達 {motor_id}"}), 404
        old = _motor
        was_enabled = _enabled
        _enabled = False
        _motor = motor
    if was_enabled and old is not None:
        try:
            old.disable()
        except Exception as exc:
            _note_error(f"切換馬達時 Disable 失敗：{exc}")
    pos = _state_pos(motor)
    if pos is None:
        pos = _wait_pos(motor, timeout=0.3)
    with _lock:
        _cmd_pos = pos
        _target_pos = pos if pos is not None else _target_pos
        return jsonify({"success": True, "state": _state_payload()})


@app.route("/api/enable", methods=["POST"])
def enable():
    global _enabled, _cmd_pos, _target_pos
    with _lock:
        motor = _motor
        kp = _kp
        kd = _kd
        fallback = _cmd_pos
        if motor is None:
            return jsonify({"success": False, "error": "尚未連線"}), 400
    try:
        pos = _wait_pos(motor, timeout=0.5)
        if pos is None:
            pos = fallback
        if pos is None:
            return jsonify(
                {
                    "success": False,
                    "error": "讀不到目前位置，已取消 Enable，避免馬達衝到 0。請再按一次掃描後重試。",
                }
            ), 400
        with _lock:
            _cmd_pos = pos
            _target_pos = pos
        try:
            motor.ensure_control_mode("MIT")
        except Exception as exc:
            _note_error(f"確認 MIT 模式失敗（改以目前模式 Enable）：{exc}")
        pos = _wait_pos(motor, timeout=0.25) or pos
        with _lock:
            _cmd_pos = pos
            _target_pos = pos
        motor.enable()
        _hold_at(motor, pos, kp, kd, times=8)
        pos = _wait_pos(motor, timeout=0.25) or pos
        _hold_at(motor, pos, kp, kd, times=4)
    except Exception as exc:
        _note_error(f"Enable 失敗：{exc}")
        traceback.print_exc()
        return jsonify({"success": False, "error": f"無法 Enable / 切到 MIT：{exc}"}), 500
    with _lock:
        _cmd_pos = pos
        _target_pos = pos
        _enabled = True
        payload = _state_payload()
    _ensure_loop()
    return jsonify({"success": True, "state": payload, "hold_rad": pos})


@app.route("/api/disable", methods=["POST"])
def disable():
    global _enabled
    with _lock:
        motor = _motor
        pos = _cmd_pos
        _enabled = False
    if motor is not None:
        try:
            hold = float((motor.get_states() or {}).get("pos") or pos or 0.0)
            _send_mit(motor, hold, 0.0, 0.0, 0.0)
            time.sleep(0.02)
            motor.disable()
        except Exception as exc:
            _note_error(f"Disable 失敗：{exc}")
    with _lock:
        return jsonify({"success": True, "state": _state_payload()})


@app.route("/api/target", methods=["POST"])
def set_target():
    global _target_pos, _cmd_pos
    data = request.get_json(silent=True) or {}
    with _lock:
        if _motor is None:
            return jsonify({"success": False, "error": "尚未連線"}), 400
        current = _state_pos(_motor)
        if current is None:
            current = float(_cmd_pos or 0.0)
        if _cmd_pos is None:
            _cmd_pos = current
        if "deg" in data:
            _target_pos = _nearest_target(current, float(data["deg"]))
        elif "rad" in data:
            _target_pos = float(data["rad"])
        else:
            return jsonify({"success": False, "error": "需要 deg 或 rad"}), 400
        if not _enabled:
            return jsonify(
                {
                    "success": True,
                    "state": _state_payload(),
                    "warning": "已記下目標，但馬達尚未 Enable，不會轉動。",
                }
            )
        return jsonify({"success": True, "state": _state_payload()})


@app.route("/api/settings", methods=["POST"])
def settings():
    global _kp, _kd, _max_speed_rad_s
    data = request.get_json(silent=True) or {}
    with _lock:
        if "kp" in data:
            _kp = max(0.0, min(40.0, float(data["kp"])))
        if "kd" in data:
            _kd = max(0.0, min(5.0, float(data["kd"])))
        if "max_deg_s" in data:
            deg_s = max(1.0, min(MAX_SPEED_DEG_S, float(data["max_deg_s"])))
            _max_speed_rad_s = math.radians(deg_s)
        return jsonify({"success": True, "state": _state_payload()})


@app.route("/api/state")
def state():
    with _lock:
        return jsonify({"success": True, "state": _state_payload()})


@app.route("/api/scan", methods=["POST"])
def scan_bus():
    global _motor, _motors_found, _cmd_pos, _target_pos, _enabled
    with _lock:
        controller = _controller
        motor = _motor
        prefer_id = None if motor is None else motor.motor_id
        _enabled = False
    if controller is None:
        return jsonify({"success": False, "error": "尚未連線"}), 400
    if motor is not None:
        try:
            motor.disable()
        except Exception as exc:
            _note_error(f"掃描前 Disable 失敗：{exc}")
    try:
        motors = _scan(controller, MOTOR_TYPE)
        motor, pos = _select_scanned_motor(controller, motors, prefer_id)
        with _lock:
            _motors_found = motors
            _motor = motor
            if pos is not None:
                _cmd_pos = pos
                _target_pos = pos
            payload = _state_payload()
        ids = ", ".join(
            f"0x{item['id']:02X}(MST 0x{int(item.get('mst_id') or 0):02X})" for item in motors
        )
        if not motors:
            msg = "bus 上沒掃到馬達 Node ID。若接了兩顆卻只看到一顆，通常是兩顆都還是 0x01，請先單顆寫入不同 ID。"
        else:
            msg = f"掃到 {len(motors)} 顆馬達 Node ID：{ids}"
            if len(motors) == 1:
                msg += "。若實際接了兩顆，請先單顆接上把其中一顆改成不同 Sender CAN ID 再並聯。"
        return jsonify({"success": True, "state": payload, "message": msg})
    except Exception as exc:
        _note_error(f"掃描失敗：{exc}")
        traceback.print_exc()
        return jsonify({"success": False, "error": f"掃描失敗：{exc}"}), 500


@app.route("/api/flash_id", methods=["POST"])
def flash_id():
    global _enabled, _motor, _cmd_pos, _target_pos, _motors_found
    data = request.get_json(silent=True) or {}
    joint_name = str(data.get("joint_name", ""))
    sender_can_id = int(data.get("sender_can_id", 0))
    receiver_master_id = int(data.get("receiver_master_id", 0))
    if sender_can_id < 1 or sender_can_id > 15:
        return jsonify({"success": False, "error": "Sender CAN ID 必須是 1–15（MIT 回授只帶 4-bit Node ID）"}), 400
    if receiver_master_id < 0 or receiver_master_id > 0x7FF:
        return jsonify({"success": False, "error": "Receiver / Master ID 必須是 0–0x7FF"}), 400

    with _lock:
        motor = _motor
        controller = _controller
        _enabled = False
        if motor is None or controller is None:
            return jsonify({"success": False, "error": "尚未連線"}), 400

    try:
        old_id = motor.motor_id
        try:
            motor.disable()
        except Exception:
            pass
        time.sleep(0.05)

        # MST_ID first, while the motor still answers on the old ESC_ID.
        motor.write_register(MST_ID_RID, receiver_master_id)
        time.sleep(0.08)
        motor.write_register(ESC_ID_RID, sender_can_id)
        time.sleep(0.08)
        _remap_motor(controller, motor, sender_can_id, receiver_master_id)

        # After ESC_ID writes to RAM, save must address the new ID.
        # Some firmware still accepts the old ID until reboot, so save both.
        motor.store_parameters()
        time.sleep(0.05)
        if old_id != sender_can_id:
            _run_as_id(motor, old_id, motor.store_parameters)
        time.sleep(1.5)

        with motor.registers_lock:
            motor.registers.pop(ESC_ID_RID, None)
            motor.registers.pop(MST_ID_RID, None)

        _query_status(motor)
        time.sleep(0.15)
        esc = _read_register_fresh(motor, ESC_ID_RID, timeout=1.0)
        mst = _read_register_fresh(motor, MST_ID_RID, timeout=1.0)
        if esc is None:
            # Motor may have rebooted; probe new then old ID.
            for probe_id in (sender_can_id, old_id):
                probe = _ensure_motor(controller, probe_id)
                _query_status(probe)
                time.sleep(0.12)
                if _state_pos(probe) is not None or (probe.get_states() or {}).get("can_id") is not None:
                    motor = probe
                    esc = _read_register_fresh(probe, ESC_ID_RID, timeout=0.8)
                    mst = _read_register_fresh(probe, MST_ID_RID, timeout=0.8)
                    break

        motors = _scan(controller, MOTOR_TYPE)
        if sender_can_id in {item["id"] for item in motors}:
            motor = controller.get_motor(sender_can_id)
        elif motors:
            motor = controller.get_motor(motors[0]["id"])
        pos = _state_pos(motor)
        if pos is None:
            pos = _wait_pos(motor, timeout=0.3) if motor is not None else None

        with _lock:
            _motor = motor
            _motors_found = motors
            if pos is not None:
                _cmd_pos = pos
                _target_pos = pos
            payload = _state_payload()

        persisted = esc == sender_can_id
        mst_ok = mst == receiver_master_id
        ids = ", ".join(
            f"0x{item['id']:02X}(MST 0x{int(item.get('mst_id') or 0):02X})" for item in motors
        ) or "(無)"
        if persisted and mst_ok:
            msg = (
                f"[成功] {joint_name} 已寫入並存進 flash：Sender/ESC_ID=0x{sender_can_id:02X}，"
                f"Receiver/MST_ID=0x{receiver_master_id:02X}。目前 bus：{ids}。"
                "請再斷電重上電後按「掃描 Node ID」確認沒變回 0x01。"
            )
        else:
            msg = (
                f"[警告] {joint_name} 已送出寫入，但讀回 ESC_ID={esc!r} MST_ID={mst!r}，"
                f"期望 0x{sender_can_id:02X}/0x{receiver_master_id:02X}。"
                f"目前 bus：{ids}。請斷電重上電後再掃描。"
            )
        return jsonify({"success": True, "state": payload, "message": msg})

    except Exception as exc:
        _note_error(f"燒錄/測試失敗：{exc}")
        traceback.print_exc()
        return jsonify({"success": False, "error": f"燒錄/測試失敗：{exc}"}), 500



from werkzeug.exceptions import NotFound

@app.errorhandler(NotFound)
def _handle_404(exc):
    return jsonify({"success": False, "error": "Not found"}), 404

@app.errorhandler(Exception)
def _unhandled(exc):
    _note_error(f"伺服器錯誤：{exc}")
    traceback.print_exc()
    return jsonify({"success": False, "error": str(exc)}), 500


def run_server(host: str = "127.0.0.1", port: int = 5000) -> None:
    patch_damiao.apply()
    ports = list_adapter_ports()
    print("自訂 DaMiao GUI（Zubax Babel slcan / USB-CAN-A）")
    print("Open http://{}:{}".format(host, port))
    if ports:
        for item in ports:
            print("  ", item["label"])
    else:
        print("尚未偵測到 COM 埠。請插入 Babel 後重新整理頁面。")
    print("Windows 沒有 socketcan；Babel 走 slcan。請關閉佔用 COM 的其他程式。")
    app.run(host=host, port=port, debug=False, use_reloader=False, threaded=True)
