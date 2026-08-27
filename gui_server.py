"""Custom DaMiao GUI: live position + click-to-move dial (Babel slcan / USB-CAN-A)."""

from __future__ import annotations

import logging
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
_tracks: dict[int, dict[str, Any]] = {}
_selected_ids: list[int] = []
_focus_id: Optional[int] = None
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


def _deg_mod(pos: Optional[float]) -> Optional[float]:
    if pos is None:
        return None
    return (math.degrees(pos) % 360.0 + 360.0) % 360.0


def _sync_focus_aliases() -> None:
    global _motor, _enabled, _cmd_pos, _target_pos
    if _controller is None or _focus_id is None or _focus_id not in _tracks:
        _motor = None
        _enabled = False
        _cmd_pos = None
        _target_pos = None
        return
    try:
        _motor = _controller.get_motor(_focus_id)
    except KeyError:
        _motor = None
    tr = _tracks.get(_focus_id, {})
    _cmd_pos = tr.get("cmd_pos")
    _target_pos = tr.get("target_pos")
    _enabled = any(
        bool(_tracks.get(mid, {}).get("enabled")) for mid in _selected_ids if mid in _tracks
    )


def _reset_tracks(
    controller,
    motors: list[dict[str, Any]],
    keep_selected: Optional[list[int]] = None,
    prefer_focus: Optional[int] = None,
) -> None:
    global _tracks, _selected_ids, _focus_id
    found_ids = [int(item["id"]) for item in motors]
    old = _tracks
    _tracks = {}
    for mid in found_ids:
        motor = controller.get_motor(mid)
        pos = _state_pos(motor)
        prev = old.get(mid, {})
        _tracks[mid] = {
            "enabled": False,
            "cmd_pos": pos if pos is not None else prev.get("cmd_pos"),
            "target_pos": pos if pos is not None else prev.get("target_pos"),
            "fail": 0,
        }
    if keep_selected:
        _selected_ids = [mid for mid in keep_selected if mid in _tracks]
    else:
        _selected_ids = list(found_ids)
    if not _selected_ids and found_ids:
        _selected_ids = [found_ids[0]]
    if prefer_focus in _tracks:
        _focus_id = prefer_focus
    elif _focus_id not in _tracks:
        _focus_id = _selected_ids[0] if _selected_ids else None
    _sync_focus_aliases()


def _parse_ids(data: dict[str, Any]) -> list[int]:
    raw = data.get("ids")
    if raw is None and data.get("id") is not None:
        raw = [data.get("id")]
    if raw is None:
        return list(_selected_ids)
    ids: list[int] = []
    seen: set[int] = set()
    for item in raw:
        mid = int(item)
        if mid in seen:
            continue
        seen.add(mid)
        ids.append(mid)
    return ids


def _motors_payload() -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for info in _motors_found:
        mid = int(info["id"])
        motor = None
        if _controller is not None:
            try:
                motor = _controller.get_motor(mid)
            except KeyError:
                motor = None
        state = (motor.get_states() if motor is not None else None) or {}
        tr = _tracks.get(mid, {})
        pos = state.get("pos")
        if pos is None:
            pos = tr.get("cmd_pos")
        tgt = tr.get("target_pos")
        item = dict(info)
        item.update(
            {
                "selected": mid in _selected_ids,
                "enabled": bool(tr.get("enabled")),
                "focused": mid == _focus_id,
                "pos": pos,
                "pos_deg_mod": _deg_mod(None if pos is None else float(pos)),
                "target_rad": tgt,
                "target_deg_mod": _deg_mod(None if tgt is None else float(tgt)),
                "status": state.get("status") or info.get("status"),
                "vel": state.get("vel"),
                "torq": state.get("torq"),
                "t_mos": state.get("t_mos"),
            }
        )
        out.append(item)
    return out


def _state_payload() -> dict[str, Any]:
    _sync_focus_aliases()
    motor = _motor
    raw = motor.get_states() if motor else {}
    pos = raw.get("pos")
    if pos is None:
        pos = _cmd_pos
    vel = raw.get("vel")
    motors = _motors_payload()
    enabled_ids = [mid for mid, tr in _tracks.items() if tr.get("enabled")]
    return {
        "connected": _controller is not None,
        "channel": _channel,
        "adapter": _adapter_kind,
        "enabled": _enabled,
        "motor_id": _focus_id,
        "selected_ids": list(_selected_ids),
        "enabled_ids": enabled_ids,
        "motors": motors,
        "status": raw.get("status"),
        "pos_rad": pos,
        "pos_deg": None if pos is None else math.degrees(pos),
        "pos_deg_mod": _deg_mod(None if pos is None else float(pos)),
        "vel": vel,
        "torq": raw.get("torq"),
        "t_mos": raw.get("t_mos"),
        "t_rotor": raw.get("t_rotor"),
        "target_rad": _target_pos,
        "target_deg_mod": _deg_mod(_target_pos),
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
    global _send_fail
    dt = 1.0 / CMD_HZ
    while not _loop_stop.is_set():
        t0 = time.perf_counter()
        with _lock:
            controller = _controller
            max_speed = _max_speed_rad_s
            kp = _kp
            kd = _kd
            jobs = []
            for mid, tr in _tracks.items():
                if not tr.get("enabled"):
                    continue
                cmd = tr.get("cmd_pos")
                target = tr.get("target_pos")
                if cmd is None or target is None or controller is None:
                    continue
                jobs.append((mid, cmd, target))
        if jobs and controller is not None:
            for mid, cmd, target in jobs:
                err = target - cmd
                step = max_speed * dt
                if abs(err) <= step:
                    cmd = target
                    vel_ff = 0.0
                else:
                    cmd += math.copysign(step, err)
                    vel_ff = math.copysign(max_speed, err)
                try:
                    motor = controller.get_motor(mid)
                    _send_mit(motor, cmd, vel_ff, kp, kd)
                    with _lock:
                        if mid in _tracks:
                            _tracks[mid]["cmd_pos"] = cmd
                            _tracks[mid]["fail"] = 0
                    _send_fail = 0
                except Exception as exc:
                    with _lock:
                        if mid in _tracks:
                            _tracks[mid]["fail"] = int(_tracks[mid].get("fail") or 0) + 1
                            fail_n = _tracks[mid]["fail"]
                        else:
                            fail_n = 0
                    _note_error(f"MIT 送指令失敗（0x{mid:02X} #{fail_n}）：{exc}")
                    if fail_n >= 8:
                        with _lock:
                            if mid in _tracks:
                                _tracks[mid]["enabled"] = False
                                _tracks[mid]["fail"] = 0
                            _sync_focus_aliases()
                        _note_error(f"馬達 0x{mid:02X} 連續送指令失敗，已自動 Disable。")
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
    global _controller, _motor, _enabled, _cmd_pos, _target_pos, _motors_found
    global _channel, _adapter_kind, _tracks, _selected_ids, _focus_id
    with _lock:
        _enabled = False
        _cmd_pos = None
        _target_pos = None
        _tracks = {}
        _selected_ids = []
        _focus_id = None
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
            _motors_found = motors
            _channel = channel
            _adapter_kind = kind
            _reset_tracks(controller, motors)
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
    motors = []
    with _lock:
        controller = _controller
        enabled_ids = [mid for mid, tr in _tracks.items() if tr.get("enabled")]
        for mid in enabled_ids:
            _tracks[mid]["enabled"] = False
        _sync_focus_aliases()
        if controller is not None:
            for mid in enabled_ids:
                try:
                    motors.append(controller.get_motor(mid))
                except KeyError:
                    pass
    for motor in motors:
        try:
            pos = _state_pos(motor)
            if pos is not None:
                _send_mit(motor, pos, 0.0, 0.0, 0.0)
            motor.disable()
        except Exception as exc:
            _note_error(f"Disable 失敗：{exc}")
    _shutdown_controller()
    return jsonify({"success": True})


@app.route("/api/select", methods=["POST"])
def select_motor():
    global _selected_ids, _focus_id
    data = request.get_json(silent=True) or {}
    with _lock:
        if _controller is None:
            return jsonify({"success": False, "error": "尚未連線"}), 400
        ids = _parse_ids(data)
        valid = [mid for mid in ids if mid in _tracks]
        if not valid and ids:
            return jsonify({"success": False, "error": "選擇的 Node ID 不在掃描結果裡"}), 400
        _selected_ids = valid
        focus = data.get("focus")
        if focus is not None:
            focus = int(focus)
            if focus in _tracks:
                _focus_id = focus
        if _selected_ids and _focus_id not in _selected_ids:
            _focus_id = _selected_ids[0]
        _sync_focus_aliases()
        return jsonify({"success": True, "state": _state_payload()})


def _enable_group(motors, kp: float, kd: float) -> dict[int, float]:
    holds: dict[int, float] = {}
    missing: list[int] = []
    for motor in motors:
        try:
            motor.ensure_control_mode("MIT")
        except Exception as exc:
            _note_error(f"0x{motor.motor_id:02X} 確認 MIT 模式失敗（繼續 Enable）：{exc}")
        pos = _wait_pos(motor, timeout=0.45)
        if pos is None:
            missing.append(motor.motor_id)
            continue
        holds[motor.motor_id] = pos
        with _lock:
            if motor.motor_id in _tracks:
                _tracks[motor.motor_id]["cmd_pos"] = pos
                _tracks[motor.motor_id]["target_pos"] = pos
    if missing and not holds:
        raise RuntimeError(
            "讀不到目前位置，已取消 Enable，避免馬達衝到 0。請再按一次掃描後重試。"
        )
    if missing:
        _note_error("部分馬達讀不到位置，已跳過：" + ", ".join(f"0x{i:02X}" for i in missing))
    ready = [m for m in motors if m.motor_id in holds]
    for motor in ready:
        motor.enable()
    for _ in range(6):
        for motor in ready:
            _send_mit(motor, holds[motor.motor_id], 0.0, kp, kd)
        time.sleep(0.015)
    for motor in ready:
        pos = _wait_pos(motor, timeout=0.2) or holds[motor.motor_id]
        holds[motor.motor_id] = pos
        with _lock:
            if motor.motor_id in _tracks:
                _tracks[motor.motor_id]["cmd_pos"] = pos
                _tracks[motor.motor_id]["target_pos"] = pos
                _tracks[motor.motor_id]["enabled"] = True
                _tracks[motor.motor_id]["fail"] = 0
    return holds


@app.route("/api/enable", methods=["POST"])
def enable():
    data = request.get_json(silent=True) or {}
    with _lock:
        controller = _controller
        kp = _kp
        kd = _kd
        if controller is None:
            return jsonify({"success": False, "error": "尚未連線"}), 400
        ids = [mid for mid in _parse_ids(data) if mid in _tracks]
        if not ids:
            return jsonify({"success": False, "error": "請先勾選要 Enable 的馬達"}), 400
    motors = []
    for mid in ids:
        try:
            motors.append(controller.get_motor(mid))
        except KeyError:
            return jsonify({"success": False, "error": f"沒有馬達 0x{mid:02X}"}), 404
    try:
        holds = _enable_group(motors, kp, kd)
    except Exception as exc:
        _note_error(f"Enable 失敗：{exc}")
        traceback.print_exc()
        return jsonify({"success": False, "error": f"無法 Enable / 切到 MIT：{exc}"}), 500
    with _lock:
        _sync_focus_aliases()
        payload = _state_payload()
    _ensure_loop()
    hold_rad = holds.get(_focus_id) if holds else None
    return jsonify({"success": True, "state": payload, "hold_rad": hold_rad, "enabled_ids": list(holds)})


@app.route("/api/disable", methods=["POST"])
def disable():
    data = request.get_json(silent=True) or {}
    with _lock:
        controller = _controller
        ids = [mid for mid in _parse_ids(data) if mid in _tracks]
        for mid in ids:
            _tracks[mid]["enabled"] = False
        _sync_focus_aliases()
    if controller is not None:
        for mid in ids:
            try:
                motor = controller.get_motor(mid)
            except KeyError:
                continue
            try:
                hold = _state_pos(motor)
                if hold is None:
                    hold = _tracks.get(mid, {}).get("cmd_pos")
                if hold is not None:
                    _send_mit(motor, float(hold), 0.0, 0.0, 0.0)
                time.sleep(0.01)
                motor.disable()
            except Exception as exc:
                _note_error(f"Disable 0x{mid:02X} 失敗：{exc}")
    with _lock:
        return jsonify({"success": True, "state": _state_payload()})


@app.route("/api/target", methods=["POST"])
def set_target():
    data = request.get_json(silent=True) or {}
    with _lock:
        if _controller is None:
            return jsonify({"success": False, "error": "尚未連線"}), 400
        ids = [mid for mid in _parse_ids(data) if mid in _tracks]
        if not ids:
            return jsonify({"success": False, "error": "請先勾選要轉動的馬達"}), 400
        if "deg" not in data and "rad" not in data:
            return jsonify({"success": False, "error": "需要 deg 或 rad"}), 400
        any_enabled = False
        for mid in ids:
            motor = _controller.get_motor(mid)
            current = _state_pos(motor)
            if current is None:
                current = _tracks[mid].get("cmd_pos")
            if current is None:
                current = 0.0
            if _tracks[mid].get("cmd_pos") is None:
                _tracks[mid]["cmd_pos"] = current
            if "deg" in data:
                _tracks[mid]["target_pos"] = _nearest_target(float(current), float(data["deg"]))
            else:
                _tracks[mid]["target_pos"] = float(data["rad"])
            if _tracks[mid].get("enabled"):
                any_enabled = True
        _sync_focus_aliases()
        if not any_enabled:
            return jsonify(
                {
                    "success": True,
                    "state": _state_payload(),
                    "warning": "已記下目標，但選取的馬達尚未 Enable，不會轉動。",
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
    global _motors_found
    with _lock:
        controller = _controller
        keep_selected = list(_selected_ids)
        keep_focus = _focus_id
        enabled_ids = [mid for mid, tr in _tracks.items() if tr.get("enabled")]
        for mid in enabled_ids:
            _tracks[mid]["enabled"] = False
        _sync_focus_aliases()
    if controller is None:
        return jsonify({"success": False, "error": "尚未連線"}), 400
    for mid in enabled_ids:
        try:
            controller.get_motor(mid).disable()
        except Exception as exc:
            _note_error(f"掃描前 Disable 0x{mid:02X} 失敗：{exc}")
    try:
        motors = _scan(controller, MOTOR_TYPE)
        with _lock:
            _motors_found = motors
            _reset_tracks(controller, motors, keep_selected, prefer_focus=keep_focus)
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
        for tr in _tracks.values():
            tr["enabled"] = False
        _sync_focus_aliases()
        motor = _motor
        controller = _controller
        if motor is None or controller is None:
            return jsonify({"success": False, "error": "尚未連線，或請先點選要燒錄的那顆馬達"}), 400

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
            _motors_found = motors
            keep = [sender_can_id] if sender_can_id in {item["id"] for item in motors} else None
            _reset_tracks(controller, motors, keep, prefer_focus=sender_can_id)
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


def _quiet_access_log() -> None:
    class _SkipPoll(logging.Filter):
        def filter(self, record: logging.LogRecord) -> bool:
            msg = record.getMessage()
            return "/api/state" not in msg

    werkzeug = logging.getLogger("werkzeug")
    werkzeug.addFilter(_SkipPoll())


def run_server(host: str = "127.0.0.1", port: int = 5000) -> None:
    patch_damiao.apply()
    _quiet_access_log()
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
