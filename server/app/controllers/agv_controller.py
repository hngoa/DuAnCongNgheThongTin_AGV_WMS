"""
===================================================================================
CONTROLLER GIAO TIẾP VỚI XE AGV_VER2 (AGV CONTROLLER)
XỬ LÝ TELEMETRY CHU KỲ (100ms), PHÁT LỆNH ĐIỀU KHIỂN & LƯU VẾT DI CHUYỂN
===================================================================================
"""

import time
from typing import Dict, Any, Tuple
from app.config import DEFAULT_AGV_ID, DEFAULT_MAP_ID, AGV_WATCHDOG_TIMEOUT_SEC

class AGVController:
    def __init__(self, workflow_manager):
        self.workflow_manager = workflow_manager
        self.agv_id = DEFAULT_AGV_ID
        self.trail = []
        self.last_seen_time = 0.0
        self.first_seen_time = 0.0
        self.last_telemetry_data = {}
        self._unlocked_while_offline = False
        # Quản lý nhật ký di chuyển (Movement Logger)
        self.is_logging_active = True
        self.last_log_time = 0.0
        self.last_logged_state = ""
        self.last_logged_station = ""
        self.last_logged_dist = -999.0
        self.camera_controller = None

    def handle_telemetry(self, body_json: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        """
        Nhận gói tin Telemetry từ ESP32-S3 qua HTTP POST /api/telemetry:
          - x, y, yaw, dist, speed, line_mask, line_err
          - voltage, current, power, energy_mah, ina_ok
          - ticks_l, ticks_r, pwm_l, pwm_r, state, ack_id, station, leg
        """
        now = time.time()
        if not self.first_seen_time or (now - self.last_seen_time > AGV_WATCHDOG_TIMEOUT_SEC):
            self.first_seen_time = now
        self.last_seen_time = now
        self.last_telemetry_data = body_json
        self._unlocked_while_offline = False

        # 1. Cập nhật vệt di chuyển (Trail) cho bản đồ Canvas 2D
        cur_x = float(body_json.get("x", 0.0))
        cur_y = float(body_json.get("y", 0.0))
        if not self.trail:
            self.trail.append([round(cur_x, 3), round(cur_y, 3)])
        else:
            last_pt = self.trail[-1]
            dx = cur_x - last_pt[0]
            dy = cur_y - last_pt[1]
            if (dx * dx + dy * dy) > 0.0004: # Di chuyển > 2cm thì thêm điểm mới
                self.trail.append([round(cur_x, 3), round(cur_y, 3)])
                if len(self.trail) > 1000:
                    self.trail.pop(0)

        # 2. Chuyển tiếp tới Bộ điều phối Workflow & FSM theo bản đồ hiện hoạt
        active_map = self.workflow_manager.map_repo.get_active_map()
        map_id = body_json.get("map_id") or (active_map["map_id"] if active_map else DEFAULT_MAP_ID)
        body_json["map_id"] = map_id
        resp_obj = self.workflow_manager.handle_telemetry(body_json)

        # Tự động kiểm tra và kích hoạt Shot 1 nếu AGV đang đỗ an toàn ở Trạm A
        if hasattr(self, "camera_controller") and self.camera_controller:
            self.camera_controller.check_auto_inbound_capture(self)

        # 3. Ghi vết lịch sử di chuyển vào Database (Data Logger)
        if self.is_logging_active:
            now = self.last_seen_time
            state = str(body_json.get("state", "STOPPED"))
            speed = float(body_json.get("speed", 0.0))
            dist = float(body_json.get("dist", 0.0))
            station = str(body_json.get("station", ""))
            leg = str(body_json.get("leg", ""))

            # Điều kiện ghi:
            # - Khi đổi trạng thái hoặc đổi trạm
            # - Hoặc khi xe đang chạy và cách lần ghi trước >= 0.25s
            # - Hoặc khi xe đứng yên thì ghi định kỳ mỗi 5 giây (Heartbeat)
            state_changed = (state != self.last_logged_state) or (station != self.last_logged_station)
            is_moving = (abs(speed) > 1.0) or (state not in ("STOPPED", "OFFLINE"))
            dist_moved = abs(dist - self.last_logged_dist) >= 0.02
            time_elapsed = now - self.last_log_time

            should_log = False
            if state_changed:
                should_log = True
            elif is_moving and (time_elapsed >= 0.25 or dist_moved):
                should_log = True
            elif not is_moving and time_elapsed >= 5.0:
                should_log = True

            if should_log:
                self.last_log_time = now
                self.last_logged_state = state
                self.last_logged_station = station
                self.last_logged_dist = dist
                try:
                    self.workflow_manager.telemetry_repo.log(
                        agv_id=self.agv_id,
                        map_id=map_id,
                        x=cur_x,
                        y=cur_y,
                        yaw=float(body_json.get("yaw", 0.0)),
                        dist=dist,
                        speed=speed,
                        mask=int(body_json.get("line_mask", 0)),
                        err=float(body_json.get("line_err", 0.0)),
                        v=float(body_json.get("voltage", 0.0)),
                        a=float(body_json.get("current", 0.0)),
                        w=float(body_json.get("power", 0.0)),
                        mah=float(body_json.get("energy_mah", 0.0)),
                        state=state,
                        task_id=None,
                        station=station,
                        leg=leg
                    )
                except Exception:
                    pass

        return 200, resp_obj

    def handle_command(self, body_json: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
        """Nhận lệnh điều khiển thủ công từ Web WMS Dashboard qua POST /api/command"""
        action = str(body_json.get("action", "")).strip().lower()

        if action in ("capture", "capture_now", "trigger_camera"):
            if hasattr(self, "camera_controller") and self.camera_controller:
                st = str(body_json.get("station", "A")).upper()
                mode = str(body_json.get("mode", "CAPTURE")).upper()
                cmd_id = self.camera_controller.queue_command("CAPTURE_NOW", metadata={"station": st, "mode": mode})
                return 200, {
                    "status": "success",
                    "action": "capture_now",
                    "cmd": "CAPTURE_NOW",
                    "cmd_id": cmd_id,
                    "cam_cmd_id": cmd_id,
                    "message": f"Đã phát lệnh chụp khay hàng #{cmd_id} xuống điện thoại"
                }
            else:
                return 400, {"status": "error", "message": "Camera controller chưa được kích hoạt"}
        elif action == "reset_odom":
            self.trail = []
            self.workflow_manager.abort_active_missions(
                reason="RESET_ODOM",
                clear_agv_cmd=False,
                clear_camera=True
            )
            self.workflow_manager.last_reported_station = "A"
            cmd_obj = self.workflow_manager.agv_fsm.queue_command("RESET_ODOM")
        elif action == "reset_energy":
            cmd_obj = self.workflow_manager.agv_fsm.queue_command("RESET_ENERGY")
        elif action == "start":
            cmd_obj = self.workflow_manager.agv_fsm.queue_command("START")
        elif action == "stop":
            self.workflow_manager.abort_active_missions(
                reason="MANUAL_STOP",
                clear_agv_cmd=False,
                clear_camera=True
            )
            cmd_obj = self.workflow_manager.agv_fsm.queue_command("STOP")
        elif action in ("abort_missions", "abort_mission", "reset_wms"):
            recovery = self.workflow_manager.abort_active_missions(
                reason="MANUAL_ABORT_MISSION",
                clear_agv_cmd=True,
                clear_camera=True
            )
            return 200, {
                "status": "mission_reset",
                "cmd_id": 0,
                "cmd": "ABORT_MISSIONS",
                "aborted_task_ids": recovery["aborted_task_ids"],
                "message": f"Đã hủy nhiệm vụ kẹt ({len(recovery['aborted_task_ids'])} task) và đưa WMS về trạng thái sẵn sàng."
            }
        elif action in ("go_to", "return_home"):
            st = "A" if action == "return_home" else str(body_json.get("station", "A")).upper()
            active_map = self.workflow_manager.map_repo.get_active_map()
            map_id = active_map["map_id"] if active_map else DEFAULT_MAP_ID
            cur_st = str(self.last_telemetry_data.get("station", "")).strip().upper() or None
            dispatch_res = self.workflow_manager.dispatch_manual_goto(
                map_id=map_id,
                target_node=st,
                current_station=cur_st,
                clear_missions=(action == "return_home" or st == "A")
            )
            cmd_obj = dispatch_res["command"]
        elif action == "auto_loop_on":
            cmd_obj = self.workflow_manager.agv_fsm.queue_command("AUTO_LOOP:ON")
        elif action == "auto_loop_reverse":
            cmd_obj = self.workflow_manager.agv_fsm.queue_command("AUTO_LOOP:REVERSE")
        elif action == "auto_loop_off":
            cmd_obj = self.workflow_manager.agv_fsm.queue_command("AUTO_LOOP:OFF")
        elif action == "turn_left_90":
            cmd_obj = self.workflow_manager.agv_fsm.queue_command("TURN_LEFT_90")
        elif action == "turn_right_90":
            cmd_obj = self.workflow_manager.agv_fsm.queue_command("TURN_RIGHT_90")
        elif action == "turn_180":
            cmd_obj = self.workflow_manager.agv_fsm.queue_command("TURN_180")
        elif action == "calibrate_gyro":
            cmd_obj = self.workflow_manager.agv_fsm.queue_command("CALIBRATE_GYRO")
        elif action == "simulate_gap":
            cmd_obj = self.workflow_manager.agv_fsm.queue_command("SIMULATE_GAP")
        else:
            cmd_str = action.upper()
            cmd_obj = self.workflow_manager.agv_fsm.queue_command(cmd_str)

        return 200, {
            "status": "command_queued",
            "cmd_id": cmd_obj["cmd_id"],
            "cmd": cmd_obj["cmd"]
        }

    def get_dashboard_state(self, camera_info: Dict[str, Any]) -> Dict[str, Any]:
        """Tổng hợp toàn bộ trạng thái hệ thống trả về cho Web Dashboard"""
        now = time.time()
        is_connected = (now - self.last_seen_time) < AGV_WATCHDOG_TIMEOUT_SEC
        
        agv_record = self.workflow_manager.fleet_repo.get_agv(self.agv_id) or {}
        last_t = self.last_telemetry_data

        if not is_connected:
            if self.last_seen_time > 0.0:
                agv_fsm = self.workflow_manager.agv_fsm
                agv_fsm.last_heartbeat = min(agv_fsm.last_heartbeat or self.last_seen_time, self.last_seen_time)
                agv_fsm.check_watchdog(AGV_WATCHDOG_TIMEOUT_SEC)
                if agv_record.get("status") != "OFFLINE":
                    self.workflow_manager.fleet_repo.update_telemetry(
                        agv_id=self.agv_id,
                        x=float(last_t.get("x", agv_record.get("x", 0.0))),
                        y=float(last_t.get("y", agv_record.get("y", 0.0))),
                        yaw=float(last_t.get("yaw", agv_record.get("yaw_deg", 90.0))),
                        speed=0.0,
                        voltage=float(last_t.get("voltage", agv_record.get("voltage_v", 0.0))),
                        current=0.0,
                        battery_pct=float(agv_record.get("battery_pct", 0.0)),
                        status="OFFLINE",
                        ack_id=int(agv_record.get("last_ack_id", 0))
                    )
            if not self._unlocked_while_offline:
                self.workflow_manager.map_repo.unlock_all_by_agv(self.agv_id)
                self.workflow_manager.manual_route_target = None
                self._unlocked_while_offline = True
            return {
                "robot_id": self.agv_id,
                "connected": False,
                "last_seen_str": "Chưa kết nối" if self.last_seen_time == 0.0 else "Mất kết nối",
                "odom_x": float(last_t.get("x", agv_record.get("x", 0.0))),
                "odom_y": float(last_t.get("y", agv_record.get("y", 0.0))),
                "yaw_deg": float(last_t.get("yaw", agv_record.get("yaw_deg", 90.0))),
                "total_distance_m": float(last_t.get("dist", 0.0)),
                "speed_cm_s": 0.0,
                "trail": self.trail,
                "line_mask": 0,
                "line_error": 0.0,
                "robot_state": "OFFLINE",
                "station": str(last_t.get("station", self.workflow_manager.last_reported_station)),
                "leg": str(last_t.get("leg", "")),
                "ticks_l": 0,
                "ticks_r": 0,
                "pwm_l": 0,
                "pwm_r": 0,
                "voltage": 0.0,
                "current": 0.0,
                "power": 0.0,
                "energy_mah": 0.0,
                "battery_pct": 0.0,
                "ina_ok": False,
                "camera": camera_info,
                "pending_command": self.workflow_manager.agv_fsm.pending_command,
                "active_mission": self.workflow_manager.active_mission
            }

        # KHI ĐANG KẾT NỐI (TELEMETRY THỜI GIAN THỰC TỪ ESP32-S3)
        v = float(last_t.get("voltage", agv_record.get("voltage_v", 0.0)))
        
        # Tính % pin dựa trên điện áp bus INA226 thực tế (Tự động nhận diện Pin 2S 6.4V-8.4V và 3S 9.9V-12.6V)
        b_pct = last_t.get("battery_pct")
        if b_pct is None:
            if v <= 3.0:
                b_pct = 0.0
            elif v <= 8.8:
                b_pct = max(0.0, min(100.0, round(((v - 6.4) / (8.4 - 6.4)) * 100.0, 1)))
            else:
                b_pct = max(0.0, min(100.0, round(((v - 9.9) / (12.6 - 9.9)) * 100.0, 1)))
        else:
            b_pct = float(b_pct)

        return {
            "robot_id": self.agv_id,
            "connected": True,
            "last_seen_str": f"Đang kết nối ({(now - self.last_seen_time)*1000:.0f}ms)",
            "odom_x": float(last_t.get("x", agv_record.get("x", 0.0))),
            "odom_y": float(last_t.get("y", agv_record.get("y", 0.0))),
            "yaw_deg": float(last_t.get("yaw", agv_record.get("yaw_deg", 90.0))),
            "total_distance_m": float(last_t.get("dist", 0.0)),
            "speed_cm_s": float(last_t.get("speed", agv_record.get("speed_cm_s", 0.0))),
            "trail": self.trail,
            "line_mask": int(last_t.get("line_mask", 0)),
            "line_error": float(last_t.get("line_err", 0.0)),
            "robot_state": str(last_t.get("state", agv_record.get("status", "IDLE"))),
            "station": str(last_t.get("station", self.workflow_manager.last_reported_station)),
            "leg": str(last_t.get("leg", "")),
            "ticks_l": int(last_t.get("ticks_l", 0)),
            "ticks_r": int(last_t.get("ticks_r", 0)),
            "pwm_l": int(last_t.get("pwm_l", 0)),
            "pwm_r": int(last_t.get("pwm_r", 0)),
            "voltage": v,
            "current": float(last_t.get("current", agv_record.get("current_a", 0.0))),
            "power": float(last_t.get("power", 0.0)),
            "energy_mah": float(last_t.get("energy_mah", 0.0)),
            "battery_pct": b_pct,
            "ina_ok": bool(last_t.get("ina_ok", True)),
            "camera": camera_info,
            "pending_command": self.workflow_manager.agv_fsm.pending_command,
            "active_mission": self.workflow_manager.active_mission
        }

    def get_logs_data(self, limit: int = 100, offset: int = 0, state: str = "ALL") -> Dict[str, Any]:
        """Lấy danh sách log di chuyển và thông số thống kê phục vụ trang /logs"""
        logs = self.workflow_manager.telemetry_repo.get_logs(limit=limit, offset=offset, state=state)
        total_count = self.workflow_manager.telemetry_repo.count(state=state)
        return {
            "status": "success",
            "is_logging_active": self.is_logging_active,
            "total_count": total_count,
            "limit": limit,
            "offset": offset,
            "logs": logs
        }

    def clear_logs(self) -> Dict[str, Any]:
        """Xóa toàn bộ nhật ký di chuyển trong database"""
        self.workflow_manager.telemetry_repo.clear()
        return {"status": "success", "message": "Đã xóa toàn bộ nhật ký di chuyển thành công"}

    def toggle_logging(self, enabled: bool) -> Dict[str, Any]:
        """Bật hoặc tạm dừng ghi nhật ký di chuyển"""
        self.is_logging_active = bool(enabled)
        return {
            "status": "success", 
            "is_logging_active": self.is_logging_active,
            "message": f"Đã {'bật' if self.is_logging_active else 'tạm dừng'} ghi nhật ký di chuyển"
        }

    def export_logs_csv(self) -> Tuple[int, str, bytes]:
        """Xuất toàn bộ nhật ký di chuyển ra file CSV"""
        import io, csv
        from datetime import datetime
        logs = self.workflow_manager.telemetry_repo.get_recent(limit=10000)
        output = io.StringIO()
        writer = csv.writer(output)
        writer.writerow([
            "ID", "Timestamp", "AGV_ID", "Map_ID", "Station", "Leg", "State", 
            "X_m", "Y_m", "Yaw_deg", "TotalDist_m", "Speed_cm_s", 
            "LineMask", "LineErr", "Voltage_V", "Current_A", "Power_W", "Energy_mAh"
        ])
        for r in logs:
            writer.writerow([
                r.get("log_id", ""), r.get("timestamp", ""), r.get("agv_id", ""), r.get("map_id", ""),
                r.get("station", ""), r.get("leg", ""), r.get("state", ""),
                r.get("x", 0.0), r.get("y", 0.0), r.get("yaw", 0.0),
                r.get("dist", 0.0), r.get("speed", 0.0), r.get("line_mask", 0),
                r.get("line_err", 0.0), r.get("voltage", 0.0), r.get("current", 0.0),
                r.get("power", 0.0), r.get("energy_mah", 0.0)
            ])
        csv_bytes = output.getvalue().encode("utf-8-sig") # utf-8-sig để Excel mở chuẩn tiếng Việt
        filename = f"agv_movement_logs_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        return 200, filename, csv_bytes
