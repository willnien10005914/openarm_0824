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
DEFAULT_KP = 8.0
DEFAULT_KD = 1.0

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


def _scan(controller, motor_type: str) -> list[dict[str, Any]]:
    controller.flush_bus()
    for motor_id in range(0x01, 0x11):
        try:
            motor = controller.add_motor(
                motor_id=motor_id, feedback_id=0x00, motor_type=motor_type
            )
        except ValueError:
            motor = controller.get_motor(motor_id)
        _send_mit(motor, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.02)
    found: list[dict[str, Any]] = []
    seen: set[int] = set()
    deadline = time.perf_counter() + 0.6
    while time.perf_counter() < deadline:
        controller.poll_feedback()
        for motor_id, motor in controller.motors.items():
            state = motor.get_states() or {}
            if state.get("can_id") is None or motor_id in seen:
                continue
            seen.add(motor_id)
            found.append(
                {
                    "id": motor_id,
                    "arb_id": state.get("arbitration_id", 0),
                    "pos": state.get("pos"),
                    "status": state.get("status"),
                }
            )
        time.sleep(0.01)
    return found


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
                    "error": "轉接器已開，但沒掃到馬達。請檢查 24V、CAN_H/CAN_L、120Ω 終端，並關閉其他佔用 COM 的程式。",
                }
            ), 400
        motor = controller.get_motor(motors[0]["id"])
        state = motor.get_states() or {}
        pos = float(state.get("pos") or 0.0)
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
    pos = float((motor.get_states() or {}).get("pos") or 0.0)
    with _lock:
        _cmd_pos = pos
        _target_pos = pos
        return jsonify({"success": True, "state": _state_payload()})


@app.route("/api/enable", methods=["POST"])
def enable():
    global _enabled, _cmd_pos, _target_pos
    with _lock:
        motor = _motor
        if motor is None:
            return jsonify({"success": False, "error": "尚未連線"}), 400
    try:
        pos = float((motor.get_states() or {}).get("pos") or 0.0)
        motor.enable()
        time.sleep(0.03)
        motor.ensure_control_mode("MIT")
        pos = float((motor.get_states() or {}).get("pos") or pos)
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
    return jsonify({"success": True, "state": payload})


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
        current = float((_motor.get_states() or {}).get("pos") or _cmd_pos or 0.0)
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


@app.route("/api/flash_id", methods=["POST"])
def flash_id():
    global _enabled, _motor, _cmd_pos, _target_pos
    data = request.get_json(silent=True) or {}
    joint_name = str(data.get("joint_name", ""))
    sender_can_id = int(data.get("sender_can_id", 0))
    receiver_master_id = int(data.get("receiver_master_id", 0))
    
    with _lock:
        motor = _motor
        controller = _controller
        if motor is None or controller is None:
            return jsonify({"success": False, "error": "尚未連線"}), 400
            
    try:
        # Step 1: Write new IDs
        motor.set_receive_id(sender_can_id)
        time.sleep(0.05)
        if hasattr(motor, 'set_feedback_id'):
            motor.set_feedback_id(receiver_master_id)
            time.sleep(0.05)
            
        # Step 2: Store parameters
        motor.store_parameters()
        time.sleep(2.0) # Give it time to flash and reboot (increased from 0.5s)
        
        # Step 3: Reconnect to new ID
        with _lock:
            old_id = motor.motor_id
            _enabled = False
            if old_id != sender_can_id and old_id in controller.motors:
                del controller.motors[old_id]
                
        try:
            new_motor = controller.add_motor(motor_id=sender_can_id, feedback_id=0x00, motor_type=MOTOR_TYPE)
        except ValueError:
            new_motor = controller.get_motor(sender_can_id)
            
        _send_mit(new_motor, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.05)
        
        with _lock:
            _motor = new_motor
            # We don't do a full scan, just clear and assume success for now
            _motors_found.clear()
        # Step 4: Verify connection on new ID
        # We just try to read a state to see if it responds, without changing modes or enabling
        try:
            state = new_motor.get_states() or {}
        except Exception:
            # If it times out, it means it's not responding on the new ID
            raise RuntimeError(f"馬達未在新的 ID (0x{sender_can_id:02X}) 上回應，請確認是否需手動斷電重啟。")
        
        with _lock:
            _enabled = False
            payload = _state_payload()
            
        msg = f"[成功] {joint_name} 已向馬達發送寫入指令 (Sender CAN ID: 0x{sender_can_id:02X}, Receiver ID: 0x{receiver_master_id:02X}) 並儲存。"
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
