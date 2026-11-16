"""
===================================================================================
MÁY TRẠNG THÁI VẬN HÀNH & AN TOÀN XE AGV (AGV MOTION & SAFETY FSM)
THEO DÕI TRẠNG THÁI REAL-TIME, CẦU CHÌ AN TOÀN WATCHDOG & BẢO VỆ PIN INA226
===================================================================================
"""

from typing import Dict, Any, Optional
import time
try:
    from .fsm_definitions import AGVState
except ImportError:
    from fsm_definitions import AGVState

class AGVStateMachine:
    """Quản lý trạng thái logic và an toàn của xe AGV trên Server"""
    def __init__(self, agv_id: str, fleet_repo):
        self.agv_id = agv_id
        self.fleet_repo = fleet_repo
        self.current_state = AGVState.OFFLINE
        self.last_heartbeat = 0.0
        agv_rec = fleet_repo.get_agv(agv_id) if fleet_repo else None
        self.cmd_id_counter = int(agv_rec.get("last_ack_id", 0)) if agv_rec else 0
        self.pending_command: Optional[Dict[str, Any]] = None
        self.ina_ok: Optional[bool] = None

    def update_from_telemetry(self, data: Dict[str, Any]) -> AGVState:
        """
        Xử lý gói tin Telemetry nhận từ ESP32-S3 (Chu kỳ 100ms - 200ms):
          - Cập nhật thời điểm heartbeat
          - Kiểm tra ngưỡng điện áp pin INA226
          - Cập nhật trạng thái chuyển động
          - Kiểm tra ACK id để xóa pending command
        """
        now = time.time()
        self.last_heartbeat = now

        voltage = float(data.get("voltage", 0.0))
        speed = float(data.get("speed", 0.0))
        reported_state = str(data.get("state", "IDLE")).upper()
        ack_id = int(data.get("ack_id", 0))

        # 1. Đồng bộ bộ đếm cmd_id nếu ESP32 có ack_id cao hơn (tránh mất lệnh khi khởi động lại Server)
        if ack_id > self.cmd_id_counter:
            self.cmd_id_counter = ack_id
            if self.pending_command and self.pending_command["cmd_id"] <= ack_id:
                self.cmd_id_counter = ack_id + 1
                self.pending_command["cmd_id"] = self.cmd_id_counter

        # Kiểm tra ACK từ xe để xóa lệnh chờ
        if self.pending_command and ack_id == self.pending_command["cmd_id"]:
            self.pending_command = None

        # 2. Kiểm tra an toàn Điện áp Pin (INA226 - Tự động thích ứng Pin 2S 7.4V-8.4V và 3S 11.1V-12.6V)
        battery_fault = False
        ina_ok = data.get("ina_ok")
        if "ina_ok" in data and ina_ok is not None:
            self.ina_ok = bool(ina_ok)
        if ina_ok is not False:
            if (3.0 < voltage < 6.4) or (8.8 < voltage < 9.9):
                self.current_state = AGVState.EMERGENCY_STOP
                battery_fault = True
            elif (6.4 <= voltage < 6.8) or (9.9 <= voltage < 10.5):
                if self.current_state not in (AGVState.LINE_FOLLOWING, AGVState.DECELERATING):
                    self.current_state = AGVState.LOW_BATTERY
                    battery_fault = True

        # 3. Đồng bộ trạng thái từ firmware nếu không bị sự cố pin
        if not battery_fault:
            if reported_state in ("LOST_LINE_STOP", "GAP_TIMEOUT_STOP", "EMERGENCY_STOP"):
                self.current_state = AGVState.EMERGENCY_STOP
            elif "GAP" in reported_state:
                self.current_state = AGVState.GAP_CROSSING
            elif reported_state.startswith("AT_STATION"):
                self.current_state = AGVState.AT_STATION
            elif (
                speed > 5.0
                or "RUNNING" in reported_state
                or "TURNING" in reported_state
                or "APPROACH" in reported_state
                or "DECELERATING" in reported_state
                or reported_state == "PAUSED_TRANSIT"
            ):
                self.current_state = AGVState.LINE_FOLLOWING
            else:
                self.current_state = AGVState.IDLE

        return self.current_state

    def check_watchdog(self, timeout_sec: float = 6.0) -> bool:
        """Kiểm tra Watchdog: Nếu quá timeout_sec không nhận được telemetry -> OFFLINE"""
        if time.time() - self.last_heartbeat > timeout_sec:
            self.current_state = AGVState.OFFLINE
            return True
        return False

    def queue_command(self, action: str, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Tạo lệnh mới mang cmd_id tự tăng duy nhất (Idempotent Command)"""
        self.cmd_id_counter += 1
        cmd_obj = {
            "cmd_id": self.cmd_id_counter,
            "cmd": action,
            "params": params or {},
            "timestamp": time.time()
        }
        self.pending_command = cmd_obj
        return cmd_obj
