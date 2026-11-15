"""
===================================================================================
AGV_Ver2 - WMS WEB DASHBOARD & DATA LOGGER SERVER (SA BÀN 1.54m x 1.51m)
TÍCH HỢP QUẢN LÝ NĂNG LƯỢNG & ĐO DÒNG/ÁP INA226 (I2C1: GPIO 8, 9)
-----------------------------------------------------------------------------------
TỌA ĐỘ BẢN ĐỒ THỰC TẾ (Gốc O tại Trạm A - Tâm trục 2 bánh xe):
  - Trạm A: (X = 0.00m, Y = 0.00m) - Điểm xuất phát chuẩn
  - Trạm B: (X = 0.00m, Y = 1.51m) - Chiều dài cạnh AB = 151cm
  - Trạm C: (X = 1.54m, Y = 1.51m) - Chiều dài cạnh BC = 154cm
  - Trạm D: (X = 1.54m, Y = 0.00m) - Chiều dài cạnh CD = 154cm

TÍNH NĂNG CHÍNH:
  1. Giám sát thời gian thực 2D Canvas: Vị trí (X, Y) và góc quay Yaw của xe.
  2. Bảng quản lý năng lượng & Pin INA226: VBUS (V), Dòng tải (A), Công suất (W), 
     Dung lượng tiêu thụ (mAh/Wh), % Pin LiPo (3S/2S/4S/12V PB) & Đồ thị sóng trực tiếp.
  3. Hiển thị trạng thái 5 mắt Line TCRT5000, 2 Encoder, MPU-6050, xung PWM.
  4. Bộ thu thập dữ liệu (Data Logger): Ghi nhật ký từng frame và Xuất file CSV đầy đủ năng lượng.
  5. Điều phối Mission 8 chặng 2 chiều & Ma trận 3 nút & Lập lộ trình Điểm - Điểm.
  6. Tự động phát sóng UDP Discovery (Port 5005) để ESP32 tự tìm thấy Server.
===================================================================================
"""

from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import json
import time
import math
from datetime import datetime
import threading
import socket
import csv
import io
import os

PORT = 5000
HOST = "0.0.0.0"
UDP_PORT = 5005

# --- CẤU HÌNH KÍCH THƯỚC BẢN ĐỒ (SA BÀN THỰC TẾ) ---
MAP_WIDTH_X  = 1.54   # Chiều ngang (A -> D) = 154 cm
MAP_HEIGHT_Y = 1.51   # Chiều dọc (A -> B)   = 151 cm

STATIONS = [
    {"id": "A", "name": "Trạm A (0.00m, 0.00m)", "x": 0.00, "y": 0.00, "color": "#10b981"},
    {"id": "B", "name": "Trạm B (0.00m, 1.51m)", "x": 0.00, "y": 1.51, "color": "#38bdf8"},
    {"id": "C", "name": "Trạm C (1.54m, 1.51m)", "x": 1.54, "y": 1.51, "color": "#a855f7"},
    {"id": "D", "name": "Trạm D (1.54m, 0.00m)", "x": 1.54, "y": 0.00, "color": "#f59e0b"}
]

# Ước lượng % Dung lượng Pin dựa trên điện áp
def calculate_battery_soc(voltage, btype="3S_LIPO"):
    if btype == "3S_LIPO":
        v_min, v_max = 9.9, 12.6
    elif btype == "2S_LIPO":
        v_min, v_max = 6.6, 8.4
    elif btype == "4S_LIPO":
        v_min, v_max = 13.2, 16.8
    else: # 12V PB (Ắc quy chì)
        v_min, v_max = 10.5, 12.8

    if voltage <= v_min:
        return 0.0
    if voltage >= v_max:
        return 100.0
    return round(((voltage - v_min) / (v_max - v_min)) * 100.0, 1)

# Trạng thái toàn cục của hệ thống
state = {
    "robot_id": "AGV_Ver2",
    "connected": False,
    "last_seen": 0.0,
    "last_seen_str": "Chưa kết nối",

    # Vị trí tâm trục bánh xe (X, Y) và góc hướng
    "odom_x": 0.0,
    "odom_y": 0.0,
    "yaw_deg": 90.0,
    "total_distance_m": 0.0,
    "speed_cm_s": 0.0,
    "trail": [[0.0, 0.0]],

    # Cảm biến dò line 5 mắt
    "line_mask": 0,
    "line_error": 0.0,
    "line_state": "ON_LINE",
    "robot_state": "STOPPED",

    # Encoder & Động cơ
    "ticks_l": 0,
    "ticks_r": 0,
    "pwm_l": 0,
    "pwm_r": 0,

    # MPU-6050
    "mpu_ok": True,
    "gyro_rate": 0.0,

    # Quản lý Năng lượng & Cảm biến INA226
    "voltage": 0.0,
    "current": 0.0,
    "power": 0.0,
    "energy_mah": 0.0,
    "energy_wh": 0.0,
    "battery_pct": 0.0,
    "battery_type": "3S_LIPO",
    "ina_ok": False,

    # Mission điều phối
    "current_mission": "NONE",
    "mission_status": "IDLE",
    "auto_loop": False,

    # Lệnh chờ gửi xuống ESP32
    "cmd_id_counter": 0,
    "pending_command": None,
    "last_ack_id": 0,
    "last_cmd_action": "NONE"
}

# Nhật ký telemetry lưu trữ để phân tích & hiệu chuẩn
MAX_LOG_RECORDS = 5000
log_records = []
log_lock = threading.Lock()

def get_local_ips():
    ips = []
    try:
        hostname = socket.gethostname()
        for ip in socket.gethostbyname_ex(hostname)[2]:
            if not ip.startswith("127."):
                ips.append(ip)
    except Exception:
        pass
    return ips

# --- THREAD PHÁT SÓNG UDP TỰ ĐỘNG DÒ TÌM SERVER ---
def udp_broadcaster():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    msg = json.dumps({"service": "AGV_SERVER", "port": PORT, "version": "Ver2"}).encode('utf-8')
    while True:
        try:
            sock.sendto(msg, ("<broadcast>", UDP_PORT))
        except Exception:
            pass
        time.sleep(1.0)

# --- XỬ LÝ HTTP REQUEST ---
class AGVHttpHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def _send_json(self, data, code=200):
        body = json.dumps(data).encode('utf-8')
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?")[0]

        if path == "/" or path == "/index.html":
            self._serve_dashboard_html()
        elif path == "/api/state":
            now = time.time()
            state["connected"] = (now - state["last_seen"]) < 2.5
            if not state["connected"]:
                state["last_seen_str"] = "Mất kết nối (" + str(int(now - state["last_seen"])) + "s)"
            else:
                state["last_seen_str"] = "Đang kết nối (" + str(int((now - state["last_seen"])*1000)) + "ms)"
            self._send_json(state)
        elif path == "/api/logs":
            with log_lock:
                recent = log_records[-200:]
            self._send_json({"total": len(log_records), "logs": recent})
        elif path == "/api/export_csv":
            self._export_logs_csv()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        path = self.path.split("?")[0]
        content_len = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_len).decode("utf-8") if content_len > 0 else "{}"

        if path == "/api/telemetry":
            try:
                data = json.loads(body)
                now = time.time()
                state["last_seen"] = now
                state["connected"] = True

                # Cập nhật thông số di chuyển & odometry
                state["odom_x"] = float(data.get("x", state["odom_x"]))
                state["odom_y"] = float(data.get("y", state["odom_y"]))
                state["yaw_deg"] = float(data.get("yaw", state["yaw_deg"]))
                state["total_distance_m"] = float(data.get("dist", state["total_distance_m"]))
                state["speed_cm_s"] = float(data.get("speed", state["speed_cm_s"]))
                state["line_mask"] = int(data.get("line_mask", state["line_mask"]))
                state["line_error"] = float(data.get("line_err", state["line_error"]))
                state["robot_state"] = str(data.get("state", state["robot_state"]))
                state["ticks_l"] = int(data.get("ticks_l", state["ticks_l"]))
                state["ticks_r"] = int(data.get("ticks_r", state["ticks_r"]))
                state["pwm_l"] = int(data.get("pwm_l", state["pwm_l"]))
                state["pwm_r"] = int(data.get("pwm_r", state["pwm_r"]))
                state["auto_loop"] = bool(data.get("auto_loop", state["auto_loop"]))

                # Cập nhật thông số năng lượng INA226
                state["voltage"] = float(data.get("voltage", state["voltage"]))
                state["current"] = float(data.get("current", state["current"]))
                state["power"] = float(data.get("power", state["power"]))
                state["energy_mah"] = float(data.get("energy_mah", state["energy_mah"]))
                state["energy_wh"] = round(state["energy_mah"] * (state["voltage"] / 1000.0), 3)
                state["ina_ok"] = bool(data.get("ina_ok", state["ina_ok"]))
                state["battery_pct"] = calculate_battery_soc(state["voltage"], state.get("battery_type", "3S_LIPO"))

                # Lưu vệt di chuyển (Trail)
                last_trail = state["trail"][-1]
                dx = state["odom_x"] - last_trail[0]
                dy = state["odom_y"] - last_trail[1]
                if (dx*dx + dy*dy) > 0.0004:
                    state["trail"].append([round(state["odom_x"], 3), round(state["odom_y"], 3)])
                    if len(state["trail"]) > 1000:
                        state["trail"].pop(0)

                # Ghi log telemetry kèm năng lượng
                with log_lock:
                    record = {
                        "t": datetime.now().strftime("%H:%M:%S.%f")[:-3],
                        "x": round(state["odom_x"], 3),
                        "y": round(state["odom_y"], 3),
                        "yaw": round(state["yaw_deg"], 1),
                        "dist": round(state["total_distance_m"], 3),
                        "speed": round(state["speed_cm_s"], 1),
                        "mask": f"{state['line_mask']:05b}",
                        "err": round(state["line_error"], 2),
                        "v": round(state["voltage"], 3),
                        "a": round(state["current"], 3),
                        "w": round(state["power"], 3),
                        "mah": round(state["energy_mah"], 1),
                        "pwmL": state["pwm_l"],
                        "pwmR": state["pwm_r"],
                        "ticksL": state["ticks_l"],
                        "ticksR": state["ticks_r"],
                        "state": state["robot_state"]
                    }
                    log_records.append(record)
                    if len(log_records) > MAX_LOG_RECORDS:
                        log_records.pop(0)

                # Kiểm tra ACK xác nhận lệnh từ ESP32
                ack_id = int(data.get("ack_id", 0))
                state["last_ack_id"] = ack_id
                if state["pending_command"] and ack_id >= state["pending_command"]["id"]:
                    state["pending_command"] = None

                # Phản hồi lệnh chờ gửi xuống xe nếu có
                cmd_obj = state["pending_command"]
                self._send_json({
                    "status": "ok",
                    "cmd": cmd_obj["cmd"] if cmd_obj else None,
                    "cmd_id": cmd_obj["id"] if cmd_obj else 0
                })
            except Exception as e:
                self._send_json({"error": str(e)}, 400)

        elif path == "/api/command":
            try:
                cmd_data = json.loads(body)
                action = cmd_data.get("action", "")
                state["cmd_id_counter"] = state.get("cmd_id_counter", 0) + 1
                new_id = state["cmd_id_counter"]
                cmd_str = action.upper()

                if action == "reset_odom":
                    state["odom_x"] = 0.0
                    state["odom_y"] = 0.0
                    state["yaw_deg"] = 90.0
                    state["trail"] = [[0.0, 0.0]]
                    cmd_str = "RESET_ODOM"
                elif action == "reset_energy":
                    state["energy_mah"] = 0.0
                    state["energy_wh"] = 0.0
                    cmd_str = "RESET_ENERGY"
                elif action == "start":
                    cmd_str = "START"
                elif action == "stop":
                    cmd_str = "STOP"
                elif action == "test_forward":
                    cmd_str = "TEST_FORWARD"
                elif action == "calibrate_gyro":
                    cmd_str = "CALIBRATE_GYRO"
                elif action == "turn_left_90":
                    cmd_str = "TURN_LEFT_90"
                elif action == "turn_right_90":
                    cmd_str = "TURN_RIGHT_90"
                elif action == "turn_180":
                    cmd_str = "TURN_180"
                elif action == "simulate_gap":
                    cmd_str = "SIMULATE_GAP"
                elif action == "mission_next":
                    cmd_str = "MISSION:NEXT"
                elif action == "mission_prev":
                    cmd_str = "MISSION:PREV"
                elif action == "go_to":
                    st = cmd_data.get("station", "A").upper()
                    cmd_str = f"GO_TO:{st}"
                elif action == "auto_loop_on":
                    state["auto_loop"] = True
                    cmd_str = "AUTO_LOOP:ON"
                elif action == "auto_loop_reverse":
                    state["auto_loop"] = True
                    cmd_str = "AUTO_LOOP:REVERSE"
                elif action == "auto_loop_off":
                    state["auto_loop"] = False
                    cmd_str = "AUTO_LOOP:OFF"
                elif action == "mission":
                    m_id = cmd_data.get("mission_id", "AB")
                    state["current_mission"] = m_id
                    cmd_str = f"MISSION:{m_id}"
                elif action == "set_battery_type":
                    btype = cmd_data.get("battery_type", "3S_LIPO")
                    state["battery_type"] = btype
                    state["battery_pct"] = calculate_battery_soc(state["voltage"], btype)
                    self._send_json({"status": "battery_type_updated", "type": btype})
                    return

                state["pending_command"] = {"id": new_id, "cmd": cmd_str}
                state["last_cmd_action"] = cmd_str
                self._send_json({"status": "command_queued", "cmd_id": new_id, "cmd": cmd_str})
            except Exception as e:
                self._send_json({"error": str(e)}, 400)

        elif path == "/api/clear_logs":
            with log_lock:
                log_records.clear()
            self._send_json({"status": "logs_cleared"})

        else:
            self.send_response(404)
            self.end_headers()

    def _export_logs_csv(self):
        output = io.StringIO()
        writer = csv.writer(output)
        writer.writerow(["Timestamp", "X_m", "Y_m", "Yaw_deg", "TotalDist_m", "Speed_cm_s", "LineMask", "LineErr", "Voltage_V", "Current_A", "Power_W", "Energy_mAh", "Pwm_L", "Pwm_R", "Ticks_L", "Ticks_R", "State"])
        with log_lock:
            for r in log_records:
                writer.writerow([
                    r["t"], r["x"], r["y"], r["yaw"], r["dist"], r["speed"], 
                    r["mask"], r["err"], r.get("v", 0.0), r.get("a", 0.0), 
                    r.get("w", 0.0), r.get("mah", 0.0), r["pwmL"], r["pwmR"], 
                    r["ticksL"], r["ticksR"], r["state"]
                ])
        
        csv_data = output.getvalue().encode("utf-8")
        filename = f"agv_telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.send_response(200)
        self.send_header("Content-Type", "text/csv; charset=utf-8")
        self.send_header("Content-Disposition", f"attachment; filename={filename}")
        self.send_header("Content-Length", str(len(csv_data)))
        self.end_headers()
        self.wfile.write(csv_data)

    def _serve_dashboard_html(self):
        html = """<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AGV_Ver2 - WMS Digital Twin & Energy Management</title>
<style>
  :root {
    --bg: #0b1120; --panel: #111827; --border: #1f2937; --text: #f8fafc;
    --text-dim: #94a3b8; --accent: #38bdf8; --success: #10b981; --warning: #f59e0b; --danger: #ef4444;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
  body { background: var(--bg); color: var(--text); padding: 16px; font-size: 14px; }
  header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 14px; padding-bottom: 12px; border-bottom: 1px solid var(--border); flex-wrap: wrap; gap: 8px; }
  h1 { font-size: 19px; font-weight: 700; color: var(--accent); display: flex; align-items: center; gap: 8px; }
  .badge { padding: 4px 10px; border-radius: 999px; font-size: 11px; font-weight: 600; display: inline-flex; align-items: center; gap: 6px; }
  .badge.online { background: rgba(16, 185, 129, 0.2); color: var(--success); border: 1px solid var(--success); }
  .badge.offline { background: rgba(239, 68, 68, 0.2); color: var(--danger); border: 1px solid var(--danger); }
  .badge.sensor { background: rgba(56, 189, 248, 0.15); color: var(--accent); border: 1px solid var(--accent); }
  
  .layout-grid { display: grid; grid-template-columns: 1fr 390px; gap: 16px; }
  .card { background: var(--panel); border: 1px solid var(--border); border-radius: 12px; padding: 14px 16px; box-shadow: 0 4px 6px -1px rgba(0,0,0,0.3); margin-bottom: 16px; }
  .card h2 { font-size: 14px; font-weight: 600; margin-bottom: 10px; color: var(--text); display: flex; justify-content: space-between; align-items: center; }
  
  /* Canvas Map vừa vặn màn hình */
  .canvas-wrapper { position: relative; width: 100%; height: 420px; max-height: 48vh; background: #070b14; border-radius: 8px; border: 1px solid var(--border); overflow: hidden; }
  canvas { width: 100%; height: 100%; display: block; }

  /* Sensor Bars */
  .ir-container { display: flex; gap: 6px; justify-content: center; margin: 8px 0; }
  .ir-sensor { flex: 1; padding: 8px 3px; text-align: center; background: #070b14; border: 1px solid var(--border); border-radius: 6px; font-weight: bold; font-size: 11px; }
  .ir-sensor.active { background: #e11d48; color: #fff; box-shadow: 0 0 10px #e11d48; border-color: #fda4af; }

  /* Data Value Grid */
  .val-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .val-box { background: #070b14; padding: 9px 12px; border-radius: 8px; border: 1px solid var(--border); }
  .val-box .label { font-size: 11px; color: var(--text-dim); text-transform: uppercase; margin-bottom: 3px; }
  .val-box .number { font-size: 17px; font-weight: 700; color: var(--accent); font-family: 'Consolas', monospace; }

  /* Controls */
  .btn-group { display: flex; gap: 6px; flex-wrap: wrap; margin-top: 8px; }
  button { padding: 7px 12px; border-radius: 6px; border: none; font-weight: 600; cursor: pointer; transition: all 0.2s; font-size: 12px; }
  .btn-primary { background: var(--accent); color: #090d16; }
  .btn-success { background: var(--success); color: #fff; }
  .btn-danger  { background: var(--danger); color: #fff; }
  .btn-secondary { background: #1f2937; color: var(--text); border: 1px solid #374151; }
  button:hover { opacity: 0.9; transform: translateY(-1px); }

  /* Log Table */
  .log-table-wrap { max-height: 200px; overflow-y: auto; font-family: monospace; font-size: 11px; }
  table { width: 100%; border-collapse: collapse; text-align: left; }
  th { background: #070b14; padding: 5px 6px; position: sticky; top: 0; color: var(--text-dim); font-weight: 600; border-bottom: 1px solid var(--border); }
  td { padding: 4px 6px; border-bottom: 1px solid #1f2937; color: var(--text); }
  tr:hover td { background: rgba(56, 189, 248, 0.05); }

  /* Pin & Năng lượng Style */
  .bat-level-box { width: 44px; height: 22px; border: 2px solid #64748b; border-radius: 4px; position: relative; padding: 2px; }
  .bat-level-box::after { content: ''; position: absolute; right: -5px; top: 5px; width: 3px; height: 8px; background: #64748b; border-radius: 0 2px 2px 0; }
  .bat-level-fill { height: 100%; border-radius: 2px; width: 0%; background: #10b981; transition: width 0.3s ease; }
</style>
</head>
<body>

<header>
  <h1>🚚 AGV_Ver2 Digital Twin & Power Management</h1>
  <div style="display:flex; align-items:center; gap:8px;">
    <span id="inaBadge" class="badge sensor">⚡ INA226: CHƯA KẾT NỐI</span>
    <span id="connBadge" class="badge offline">● Chưa kết nối</span>
    <span id="connPing" style="font-size:12px; color:var(--text-dim);">-- ms</span>
  </div>
</header>

<!-- ========================================================================= -->
<!-- BẢNG TÍCH HỢP QUẢN LÝ NĂNG LƯỢNG & PIN INA226 (I2C1: SDA 8, SCL 9)         -->
<!-- ========================================================================= -->
<div class="card" style="border-top:3px solid #38bdf8; margin-bottom:14px;">
  <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:10px; flex-wrap:wrap; gap:8px;">
    <h2 style="margin:0; font-size:14px; font-weight:700; color:#38bdf8; display:flex; align-items:center; gap:8px;">
      <span>⚡ Trung Tâm Quản Lý Năng Lượng & Pin (INA226 - GPIO 8, 9)</span>
    </h2>
    <div style="display:flex; align-items:center; gap:8px; flex-wrap:wrap;">
      <select id="selBatteryType" onchange="changeBatteryType()" style="background:#070b14; color:#fff; border:1px solid var(--border); border-radius:6px; padding:4px 8px; font-size:11px; outline:none;">
        <option value="3S_LIPO" selected>Pin 3S LiPo (9.9V - 12.6V)</option>
        <option value="2S_LIPO">Pin 2S LiPo (6.6V - 8.4V)</option>
        <option value="4S_LIPO">Pin 4S LiPo (13.2V - 16.8V)</option>
        <option value="12V_PB">Ắc quy 12V (10.5V - 12.8V)</option>
      </select>
      <button class="btn-secondary" style="font-size:11px; padding:4px 8px;" onclick="sendCmd('reset_energy')">🔄 Reset 0.0 mAh</button>
      <button class="btn-secondary" style="font-size:11px; padding:4px 8px;" onclick="togglePowerChart()">📈 Ẩn/Hiện Sóng</button>
    </div>
  </div>

  <!-- Băng trạng thái Dung lượng Pin (SOC) -->
  <div style="display:flex; align-items:center; gap:14px; background:#070b14; border:1px solid var(--border); border-radius:8px; padding:8px 12px; margin-bottom:10px; flex-wrap:wrap;">
    <div style="display:flex; align-items:center; gap:8px;">
      <div class="bat-level-box">
        <div id="batLevelMini" class="bat-level-fill"></div>
      </div>
      <div>
        <span id="batPctText" style="font-size:16px; font-weight:800; color:#10b981;">0.0%</span>
        <span id="batVolText" style="font-size:11px; color:var(--text-dim); margin-left:4px;">(0.00V)</span>
      </div>
    </div>
    <div style="flex:1; min-width:160px;">
      <div style="width:100%; height:7px; background:#1f2937; border-radius:4px; overflow:hidden;">
        <div id="batBarFill" style="height:100%; width:0%; background:#10b981; transition:width 0.3s ease;"></div>
      </div>
    </div>
    <div id="batStatusAlert" style="font-size:11px; font-weight:700; color:#38bdf8;">Đang kết nối cảm biến...</div>
  </div>

  <!-- 4 Thẻ Số Liệu Năng Lượng Thực Đo -->
  <div style="display:grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); gap:8px;">
    <div class="val-box" style="border-left:3px solid #38bdf8;">
      <div class="label">Điện Áp Nguồn (VBUS)</div>
      <div class="number" id="valBusV" style="color:#38bdf8;">0.000 V</div>
    </div>
    <div class="val-box" style="border-left:3px solid #f59e0b;">
      <div class="label">Dòng Điện Tải (CURRENT)</div>
      <div class="number" id="valCurrentA" style="color:#f59e0b;">0.000 A</div>
      <div id="valCurrentMa" style="font-size:11px; color:var(--text-dim); margin-top:2px;">0.0 mA (R_shunt 0.1Ω)</div>
    </div>
    <div class="val-box" style="border-left:3px solid #10b981;">
      <div class="label">Công Suất Tiêu Thụ (POWER)</div>
      <div class="number" id="valPowerW" style="color:#10b981;">0.000 W</div>
      <div id="valPowerMw" style="font-size:11px; color:var(--text-dim); margin-top:2px;">0.0 mW</div>
    </div>
    <div class="val-box" style="border-left:3px solid #c084fc;">
      <div class="label">Dung Lượng Tiêu Hao (ENERGY)</div>
      <div class="number" id="valEnergyMah" style="color:#c084fc;">0.0 mAh</div>
      <div id="valEnergyWh" style="font-size:11px; color:var(--text-dim); margin-top:2px;">Điện năng: 0.000 Wh</div>
    </div>
  </div>

  <!-- Đồ thị sóng năng lượng thời gian thực (Live Rolling Waveform) -->
  <div id="powerChartWrap" style="margin-top:10px; display:block;">
    <div style="display:flex; justify-content:space-between; font-size:11px; color:var(--text-dim); margin-bottom:4px;">
      <span>📈 Đồ thị sóng thời gian thực (Xanh cyan: Điện áp VBUS | Cam: Dòng điện tải A)</span>
      <span id="chartPointsCount">60 mẫu gần nhất (10Hz)</span>
    </div>
    <div style="position:relative; width:100%; height:95px; background:#070b14; border-radius:6px; border:1px solid var(--border); overflow:hidden;">
      <canvas id="powerCanvas"></canvas>
    </div>
  </div>
</div>

<div class="layout-grid">
  <!-- CỘT TRÁI: BẢN ĐỒ 2D CANVAS & DATA LOGGER -->
  <div>
    <div class="card">
      <h2>
        <span>🗺️ Bản đồ Sa bàn Thực tế (Tọa độ Digital Twin)</span>
        <div style="display:flex; align-items:center; gap:6px;">
          <button class="btn-secondary" style="font-size:11px; padding:4px 8px;" onclick="resetOdom()">📍 Ghim Gốc A (0,0)</button>
          <button class="btn-secondary" style="font-size:11px; padding:4px 8px;" onclick="toggleMapSettings()">⚙️ Kích Thước</button>
        </div>
      </h2>

      <!-- Bảng điều chỉnh kích thước sa bàn tức thời -->
      <div id="mapSettingsBox" style="display:none; background:#070b14; border:1px solid var(--border); border-radius:8px; padding:10px 14px; margin-bottom:10px; font-size:12px;">
        <div style="display:flex; gap:12px; align-items:center; flex-wrap:wrap;">
          <div>
            <label style="color:var(--text-dim); font-weight:600;">Dài X (A ➔ D):</label>
            <input type="number" id="cfgMapW" value="1.54" step="0.05" min="0.5" max="5.0" style="width:70px; padding:3px 6px; font-size:11px; font-family:monospace;" onchange="applyMapDimensions()"> m
          </div>
          <div>
            <label style="color:var(--text-dim); font-weight:600;">Rộng Y (A ➔ B):</label>
            <input type="number" id="cfgMapH" value="1.51" step="0.05" min="0.5" max="5.0" style="width:70px; padding:3px 6px; font-size:11px; font-family:monospace;" onchange="applyMapDimensions()"> m
          </div>
          <div>
            <label style="color:var(--text-dim); font-weight:600;">Lề (Margin):</label>
            <input type="number" id="cfgMargin" value="0.38" step="0.05" min="0.2" max="1.0" style="width:65px; padding:3px 6px; font-size:11px; font-family:monospace;" onchange="applyMapDimensions()"> m
          </div>
          <button class="btn-primary" style="font-size:11px; padding:4px 8px;" onclick="applyMapDimensions()">Áp dụng</button>
          <button class="btn-secondary" style="font-size:11px; padding:4px 8px;" onclick="resetMapDefaults()">Mặc định (1.54x1.51)</button>
        </div>
      </div>

      <div class="canvas-wrapper">
        <canvas id="mapCanvas"></canvas>
      </div>
      <div style="margin-top:8px; display:flex; justify-content:space-between; font-size:11px; color:var(--text-dim);">
        <span id="footerStationA">Gốc O ≡ Trạm A (0.00, 0.00)</span>
        <span id="footerStationD">Trục X ➔ Điểm D (1.54m)</span>
        <span id="footerStationB">Trục Y ➔ Điểm B (1.51m)</span>
        <span>Xe: Tâm 2 bánh tại (X, Y)</span>
      </div>
    </div>

    <!-- BẢNG NHẬT KÝ THU THẬP LOG KÈM ĐIỆN NĂNG -->
    <div class="card">
      <h2>
        <span>📋 Nhật ký Thu thập Dữ liệu Telemetry (Data Logger)</span>
        <div style="display:flex; gap:6px;">
          <button class="btn-primary" style="font-size:11px; padding:4px 8px;" onclick="exportCsv()">📥 Xuất File CSV</button>
          <button class="btn-secondary" style="font-size:11px; padding:4px 8px;" onclick="clearLogs()">🗑️ Xóa Log</button>
        </div>
      </h2>
      <div class="log-table-wrap">
        <table>
          <thead>
            <tr>
              <th>Giờ</th><th>X (m)</th><th>Y (m)</th><th>Yaw</th><th>Quãng đường</th><th>Vận tốc</th><th>Line</th><th>VBUS</th><th>Dòng/W</th><th>mAh</th><th>PWM</th><th>Trạng thái</th>
            </tr>
          </thead>
          <tbody id="logTbody">
            <tr><td colspan="12" style="text-align:center; color:var(--text-dim);">Đang chờ dữ liệu...</td></tr>
          </tbody>
        </table>
      </div>
    </div>
  </div>

  <!-- CỘT PHẢI: BẢNG ĐIỀU KHIỂN & CẢM BIẾN -->
  <div>
    <!-- TỌA ĐỘ VÀ VẬN HÀNH -->
    <div class="card">
      <h2>Tọa độ Thực tế (Tâm 2 Bánh)</h2>
      <div class="val-grid">
        <div class="val-box"><div class="label">Tọa độ X</div><div class="number" id="valX">0.000 m</div></div>
        <div class="val-box"><div class="label">Tọa độ Y</div><div class="number" id="valY">0.000 m</div></div>
        <div class="val-box"><div class="label">Góc Hướng Yaw</div><div class="number" id="valYaw">+90.0°</div></div>
        <div class="val-box"><div class="label">Vận tốc</div><div class="number" id="valSpeed">0.0 cm/s</div></div>
      </div>
      <div style="margin-top:8px;">
        <div class="val-box"><div class="label">Tổng Quãng Đường Đi Được</div><div class="number" id="valDist" style="font-size:20px; color:var(--success);">0.000 m</div></div>
      </div>
    </div>

    <!-- TRẠNG THÁI CẢM BIẾN LINE -->
    <div class="card">
      <h2>Thanh Dò Line TCRT5000 (5 Mắt)</h2>
      <div class="ir-container">
        <div class="ir-sensor" id="irL2">L2 (-2)</div>
        <div class="ir-sensor" id="irL1">L1 (-1)</div>
        <div class="ir-sensor" id="irC">C (0)</div>
        <div class="ir-sensor" id="irR1">R1 (+1)</div>
        <div class="ir-sensor" id="irR2">R2 (+2)</div>
      </div>
      <div class="val-grid" style="margin-top:8px;">
        <div class="val-box"><div class="label">Sai số Lệch Tâm (e)</div><div class="number" id="valLineErr">0.00</div></div>
        <div class="val-box"><div class="label">Trạng thái Line</div><div class="number" id="valLineState" style="font-size:13px;">ON_LINE</div></div>
      </div>
    </div>

    <!-- ĐỘNG CƠ & ENCODER -->
    <div class="card">
      <h2>Động cơ & Bộ đếm Encoder</h2>
      <div class="val-grid">
        <div class="val-box"><div class="label">PWM Trái / Phải</div><div class="number" id="valPwm">0 | 0</div></div>
        <div class="val-box"><div class="label">Trạng thái Xe</div><div class="number" id="valRobotState" style="font-size:13px; color:var(--warning);">STOPPED</div></div>
        <div class="val-box"><div class="label">Encoder Trái</div><div class="number" id="valTicksL">0</div></div>
        <div class="val-box"><div class="label">Encoder Phải</div><div class="number" id="valTicksR">0</div></div>
      </div>
    </div>

    <!-- ĐIỀU PHỐI MISSION -->
    <div class="card">
      <h2>Điều phối Nhiệm vụ Đa Trạm (Mission Dispatch)</h2>
      <div class="btn-group">
        <button class="btn-success" onclick="sendCmd('start')">▶️ Chạy Tiếp</button>
        <button class="btn-danger"  onclick="sendCmd('stop')">⏹️ Dừng Khẩn Cấp</button>
        <button class="btn-primary" onclick="sendCmd('test_forward')">⚡ Test Tiến 1s</button>
        <button class="btn-secondary" onclick="sendCmd('calibrate_gyro')">⚖️ Calib Gyro</button>
      </div>

      <div style="margin-top:10px; font-size:11px; font-weight:bold; color:#38bdf8; margin-bottom:4px;">① MA TRẬN 3 NÚT ĐIỀU HƯỚNG NHANH:</div>
      <div class="btn-group">
        <button class="btn-primary" style="background:#0284c7; color:#fff;" onclick="sendCmd('mission_next')">⏩ Đến Trạm Kế Tiếp</button>
        <button class="btn-primary" style="background:#0369a1; color:#fff;" onclick="sendCmd('mission_prev')">⏪ Lùi Về Trạm Trước</button>
        <button class="btn-secondary" style="border-color:#a855f7; color:#c084fc;" onclick="sendCmd('turn_180')">🔄 Quay Đầu 180°</button>
      </div>

      <div style="margin-top:10px; font-size:11px; font-weight:bold; color:#10b981; margin-bottom:4px;">② ĐIỀU PHỐI ĐIỂM - ĐIỂM (GO TO TARGET):</div>
      <div class="btn-group">
        <button class="btn-secondary" style="border-color:#10b981; color:#34d399;" onclick="sendGoTo('A')">📍 Trạm A</button>
        <button class="btn-secondary" style="border-color:#38bdf8; color:#38bdf8;" onclick="sendGoTo('B')">📍 Trạm B</button>
        <button class="btn-secondary" style="border-color:#a855f7; color:#c084fc;" onclick="sendGoTo('C')">📍 Trạm C</button>
        <button class="btn-secondary" style="border-color:#f59e0b; color:#fbbf24;" onclick="sendGoTo('D')">📍 Trạm D</button>
      </div>

      <div style="margin-top:10px; font-size:11px; font-weight:bold; color:#f59e0b; margin-bottom:4px;">③ CHẠY 8 CHẶNG 2 CHIỀU ĐẶC TẢ:</div>
      <div style="font-size:10px; color:var(--text-dim); margin-bottom:3px;">Chạy Xuôi (Chiều kim đồng hồ):</div>
      <div class="btn-group">
        <button class="btn-secondary" onclick="sendMission('AB')">A ➔ B (1.51m)</button>
        <button class="btn-secondary" onclick="sendMission('BC')">B ➔ C (1.54m)</button>
        <button class="btn-secondary" onclick="sendMission('CD')">C ➔ D (1.51m)</button>
        <button class="btn-secondary" onclick="sendMission('DA')">D ➔ A (1.54m)</button>
      </div>
      <div style="font-size:10px; color:var(--text-dim); margin-top:5px; margin-bottom:3px;">Chạy Ngược (Đang ở C quay lại B):</div>
      <div class="btn-group">
        <button class="btn-secondary" style="border-color:#ec4899; color:#f472b6;" onclick="sendMission('CB')">↩️ C ➔ B (1.54m)</button>
        <button class="btn-secondary" style="border-color:#ec4899; color:#f472b6;" onclick="sendMission('BA')">↩️ B ➔ A (1.51m)</button>
        <button class="btn-secondary" style="border-color:#ec4899; color:#f472b6;" onclick="sendMission('AD')">↩️ A ➔ D (1.54m)</button>
        <button class="btn-secondary" style="border-color:#ec4899; color:#f472b6;" onclick="sendMission('DC')">↩️ D ➔ C (1.51m)</button>
      </div>

      <div style="margin-top:10px; font-size:11px; color:var(--text-dim); margin-bottom:4px;">Tuần Hoàn Tự Động & Thử Nghiệm:</div>
      <div class="btn-group">
        <button id="btnAutoLoop" class="btn-success" onclick="toggleAutoLoop()">🔄 Bật Vòng Lặp A-B-C-D-A</button>
        <button class="btn-secondary" style="border-color:#f59e0b; color:#fbbf24;" onclick="sendCmd('simulate_gap')">⚡ Test Đứt Vạch 25cm</button>
      </div>

      <div id="cmdNotification" style="margin-top:10px; padding:7px 10px; background:#070b14; border:1px solid var(--border); border-radius:6px; font-size:11px; color:var(--accent);">
        Trạng thái điều khiển: SẴN SÀNG
      </div>
    </div>
  </div>
</div>

<script>
  const canvas = document.getElementById('mapCanvas');
  const ctx = canvas.getContext('2d');

  const pCanvas = document.getElementById('powerCanvas');
  const pctx = pCanvas.getContext('2d');

  // Kích thước sa bàn thực tế
  let MAP_W = 1.54;
  let MAP_H = 1.51;
  let MARGIN_M = 0.38;

  let currentState = null;
  const powerHistory = [];
  const MAX_POWER_POINTS = 60;
  let currentBatteryType = '3S_LIPO';

  function toggleMapSettings() {
    const box = document.getElementById('mapSettingsBox');
    box.style.display = (box.style.display === 'none') ? 'block' : 'none';
  }

  function togglePowerChart() {
    const box = document.getElementById('powerChartWrap');
    box.style.display = (box.style.display === 'none') ? 'block' : 'none';
  }

  function applyMapDimensions() {
    MAP_W = parseFloat(document.getElementById('cfgMapW').value) || 1.54;
    MAP_H = parseFloat(document.getElementById('cfgMapH').value) || 1.51;
    MARGIN_M = parseFloat(document.getElementById('cfgMargin').value) || 0.38;
    
    document.getElementById('footerStationD').innerText = 'Trục X ➔ Điểm D (' + MAP_W.toFixed(2) + 'm)';
    document.getElementById('footerStationB').innerText = 'Trục Y ➔ Điểm B (' + MAP_H.toFixed(2) + 'm)';
    drawMap();
  }

  function resetMapDefaults() {
    document.getElementById('cfgMapW').value = "1.54";
    document.getElementById('cfgMapH').value = "1.51";
    document.getElementById('cfgMargin').value = "0.38";
    applyMapDimensions();
  }

  function resizeAllCanvases() {
    if (canvas && canvas.parentElement) {
      canvas.width = canvas.parentElement.clientWidth * window.devicePixelRatio;
      canvas.height = canvas.parentElement.clientHeight * window.devicePixelRatio;
      drawMap();
    }
    if (pCanvas && pCanvas.parentElement) {
      pCanvas.width = pCanvas.parentElement.clientWidth * window.devicePixelRatio;
      pCanvas.height = pCanvas.parentElement.clientHeight * window.devicePixelRatio;
      drawPowerChart();
    }
  }
  window.addEventListener('resize', resizeAllCanvases);

  function worldToCanvas(wx, wy) {
    const totalW = MAP_W + 2 * MARGIN_M;
    const totalH = MAP_H + 2 * MARGIN_M;
    const scale = Math.min(canvas.width / totalW, canvas.height / totalH);
    const offsetX = (canvas.width - totalW * scale) / 2 + MARGIN_M * scale;
    const offsetY = (canvas.height - totalH * scale) / 2 + MARGIN_M * scale;
    return { x: offsetX + wx * scale, y: offsetY + wy * scale, scale: scale };
  }

  function drawMap() {
    if (!canvas.width) return;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const pA = worldToCanvas(0.00, 0.00);
    const pB = worldToCanvas(0.00, MAP_H);
    const pC = worldToCanvas(MAP_W, MAP_H);
    const pD = worldToCanvas(MAP_W, 0.00);
    const scale = pA.scale;

    // 1. Gridlines mỗi 20cm
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    for (let x = 0; x <= MAP_W + 0.05; x += 0.2) {
      const top = worldToCanvas(x, 0);
      const bot = worldToCanvas(x, MAP_H);
      ctx.beginPath(); ctx.moveTo(top.x, top.y); ctx.lineTo(bot.x, bot.y); ctx.stroke();
    }
    for (let y = 0; y <= MAP_H + 0.05; y += 0.2) {
      const left = worldToCanvas(0, y);
      const right = worldToCanvas(MAP_W, y);
      ctx.beginPath(); ctx.moveTo(left.x, left.y); ctx.lineTo(right.x, right.y); ctx.stroke();
    }

    // 2. Đường line sa bàn
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = Math.max(3, 0.025 * scale);
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    ctx.beginPath();
    ctx.moveTo(pA.x, pA.y); ctx.lineTo(pB.x, pB.y); ctx.lineTo(pC.x, pC.y); ctx.lineTo(pD.x, pD.y);
    ctx.closePath();
    ctx.stroke();

    // 3. Vệt đường đi (Trail)
    if (currentState && currentState.trail && currentState.trail.length > 1) {
      ctx.strokeStyle = '#10b981';
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      const startP = worldToCanvas(currentState.trail[0][0], currentState.trail[0][1]);
      ctx.moveTo(startP.x, startP.y);
      for (let i = 1; i < currentState.trail.length; i++) {
        const pt = worldToCanvas(currentState.trail[i][0], currentState.trail[i][1]);
        ctx.lineTo(pt.x, pt.y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // 4. 4 Trạm A, B, C, D
    const stations = [
      { id: "A (0,0)", p: pA, color: "#10b981", offX: -55, offY: -10 },
      { id: `B (0, ${MAP_H}m)`, p: pB, color: "#38bdf8", offX: -65, offY: 18 },
      { id: `C (${MAP_W}m, ${MAP_H}m)`, p: pC, color: "#a855f7", offX: 10, offY: 18 },
      { id: `D (${MAP_W}m, 0)`, p: pD, color: "#f59e0b", offX: 10, offY: -10 }
    ];
    stations.forEach(st => {
      ctx.fillStyle = st.color;
      ctx.beginPath(); ctx.arc(st.p.x, st.p.y, 7, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = '#f8fafc';
      ctx.font = 'bold 11px sans-serif';
      ctx.fillText(st.id, st.p.x + st.offX, st.p.y + st.offY);
    });

    // 5. AGV Robot
    if (currentState) {
      const rx = currentState.odom_x;
      const ry = currentState.odom_y;
      const yawRad = (currentState.yaw_deg * Math.PI) / 180.0;
      const rPos = worldToCanvas(rx, ry);

      ctx.save();
      ctx.translate(rPos.x, rPos.y);
      ctx.rotate(yawRad);

      const trackW = 0.29 * scale;
      const distL = 0.23 * scale;
      const wheelR = 0.0325 * scale;
      const wheelW = 0.024 * scale;
      const bodyW = 0.24 * scale;
      const bodyL = (0.23 + 0.07) * scale;

      // Bánh xe
      ctx.fillStyle = '#0284c7';
      ctx.fillRect(-wheelR, -trackW/2 - wheelW/2, wheelR*2, wheelW);
      ctx.fillRect(-wheelR, trackW/2 - wheelW/2, wheelR*2, wheelW);

      // Thân xe
      ctx.fillStyle = 'rgba(56, 189, 248, 0.85)';
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.roundRect(-0.06 * scale, -bodyW/2, bodyL, bodyW, 6);
      ctx.fill(); ctx.stroke();

      // Cảm biến line TCRT5000 tại L = 23cm
      ctx.fillStyle = '#e11d48';
      ctx.fillRect(distL - 0.008 * scale, -0.075 * scale, 0.016 * scale, 0.15 * scale);

      // Mũi tên hướng
      ctx.strokeStyle = '#facc15'; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(distL, 0); ctx.stroke();

      // Tâm trục xoay
      ctx.fillStyle = '#ef4444';
      ctx.beginPath(); ctx.arc(0, 0, 5, 0, Math.PI * 2); ctx.fill();

      ctx.restore();
    }
  }

  function drawPowerChart() {
    if (!pCanvas || !pCanvas.width) return;
    const w = pCanvas.width;
    const h = pCanvas.height;
    pctx.clearRect(0, 0, w, h);

    if (powerHistory.length < 2) return;

    let maxV = 13.0;
    let maxA = 2.5;
    for (let pt of powerHistory) {
      if (pt.v > maxV) maxV = pt.v + 0.5;
      if (pt.a > maxA) maxA = pt.a + 0.5;
    }

    // Gridlines ngang
    pctx.strokeStyle = '#1e293b';
    pctx.lineWidth = 1;
    for (let i = 1; i <= 3; i++) {
      let y = h - (i * (h / 4));
      pctx.beginPath(); pctx.moveTo(0, y); pctx.lineTo(w, y); pctx.stroke();
    }

    const n = powerHistory.length;
    const dx = w / (MAX_POWER_POINTS - 1);
    const startX = (MAX_POWER_POINTS - n) * dx;

    // Đường Voltage (Xanh Cyan)
    pctx.strokeStyle = '#38bdf8';
    pctx.lineWidth = 2;
    pctx.beginPath();
    for (let i = 0; i < n; i++) {
      let x = startX + i * dx;
      let y = h - ((powerHistory[i].v / maxV) * (h - 16)) - 8;
      if (i === 0) pctx.moveTo(x, y);
      else pctx.lineTo(x, y);
    }
    pctx.stroke();

    // Đường Current (Cam)
    pctx.strokeStyle = '#f59e0b';
    pctx.lineWidth = 2;
    pctx.beginPath();
    for (let i = 0; i < n; i++) {
      let x = startX + i * dx;
      let y = h - ((Math.max(0, powerHistory[i].a) / maxA) * (h - 16)) - 8;
      if (i === 0) pctx.moveTo(x, y);
      else pctx.lineTo(x, y);
    }
    pctx.stroke();
  }

  function calcBatterySoc(v, btype) {
    let v_min = 9.9, v_max = 12.6;
    if (btype === '2S_LIPO') { v_min = 6.6; v_max = 8.4; }
    else if (btype === '4S_LIPO') { v_min = 13.2; v_max = 16.8; }
    else if (btype === '12V_PB') { v_min = 10.5; v_max = 12.8; }
    if (v <= v_min) return 0.0;
    if (v >= v_max) return 100.0;
    return Math.round(((v - v_min) / (v_max - v_min)) * 1000) / 10;
  }

  async function updateDashboard() {
    try {
      const t0 = performance.now();
      const res = await fetch('/api/state');
      const data = await res.json();
      currentState = data;

      // Cập nhật kết nối Server
      const badge = document.getElementById('connBadge');
      if (data.connected) {
        badge.className = 'badge online';
        badge.innerText = '● ' + data.last_seen_str;
      } else {
        badge.className = 'badge offline';
        badge.innerText = '● ' + data.last_seen_str;
      }
      document.getElementById('connPing').innerText = Math.round(performance.now() - t0) + ' ms';

      // Cập nhật trạng thái cảm biến INA226
      const inaBadge = document.getElementById('inaBadge');
      if (data.ina_ok) {
        inaBadge.className = 'badge online';
        inaBadge.innerText = '⚡ INA226: HOẠT ĐỘNG (GPIO 8, 9)';
      } else {
        inaBadge.className = 'badge offline';
        inaBadge.innerText = '⚡ INA226: CHƯA PHÁT HIỆN';
      }

      // CẬP NHẬT BẢNG NĂNG LƯỢNG
      const v = data.voltage || 0.0;
      const a = data.current || 0.0;
      const w = data.power || 0.0;
      const mah = data.energy_mah || 0.0;
      const wh = data.energy_wh || 0.0;

      document.getElementById('valBusV').innerText = v.toFixed(3) + ' V';
      document.getElementById('valCurrentA').innerText = a.toFixed(3) + ' A';
      document.getElementById('valCurrentMa').innerText = (a * 1000.0).toFixed(1) + ' mA (R_shunt 0.1Ω)';
      document.getElementById('valPowerW').innerText = w.toFixed(3) + ' W';
      document.getElementById('valPowerMw').innerText = (w * 1000.0).toFixed(1) + ' mW';
      document.getElementById('valEnergyMah').innerText = mah.toFixed(1) + ' mAh';
      document.getElementById('valEnergyWh').innerText = 'Điện năng: ' + wh.toFixed(3) + ' Wh';

      // Cập nhật % Pin (SOC)
      const soc = calcBatterySoc(v, currentBatteryType);
      document.getElementById('batPctText').innerText = soc.toFixed(1) + '%';
      document.getElementById('batVolText').innerText = '(' + v.toFixed(2) + 'V)';
      document.getElementById('batLevelMini').style.width = soc + '%';
      document.getElementById('batBarFill').style.width = soc + '%';

      let socColor = '#10b981';
      let alertMsg = 'Nguồn ổn định (' + currentBatteryType + ')';
      if (v < 5.0) {
        socColor = '#64748b';
        alertMsg = 'Chưa cấp nguồn động cơ / INA226 chờ tín hiệu';
      } else if (soc <= 15.0) {
        socColor = '#ef4444';
        alertMsg = '⚠️ CẢNH BÁO: PIN YẾU! CẦN SẠC GẤP!';
      } else if (soc <= 30.0) {
        socColor = '#f59e0b';
        alertMsg = 'Pin sắp hết, chuẩn bị nạp';
      } else if (a > 4.5) {
        socColor = '#ef4444';
        alertMsg = '⚠️ CẢNH BÁO QUÁ DÒNG TẢI (> 4.5A)!';
      }

      document.getElementById('batPctText').style.color = socColor;
      document.getElementById('batLevelMini').style.background = socColor;
      document.getElementById('batBarFill').style.background = socColor;
      const alertElem = document.getElementById('batStatusAlert');
      alertElem.innerText = alertMsg;
      alertElem.style.color = socColor;

      // Cập nhật mảng sóng năng lượng
      powerHistory.push({ v: v, a: a, w: w });
      if (powerHistory.length > MAX_POWER_POINTS) powerHistory.shift();
      drawPowerChart();

      // Cập nhật giá trị di chuyển
      document.getElementById('valX').innerText = data.odom_x.toFixed(3) + ' m';
      document.getElementById('valY').innerText = data.odom_y.toFixed(3) + ' m';
      document.getElementById('valYaw').innerText = (data.yaw_deg >= 0 ? '+' : '') + data.yaw_deg.toFixed(1) + '°';
      document.getElementById('valSpeed').innerText = data.speed_cm_s.toFixed(1) + ' cm/s';
      document.getElementById('valDist').innerText = data.total_distance_m.toFixed(3) + ' m';
      document.getElementById('valLineErr').innerText = data.line_error.toFixed(2);
      document.getElementById('valLineState').innerText = data.line_state;
      document.getElementById('valPwm').innerText = data.pwm_l + ' | ' + data.pwm_r;
      document.getElementById('valRobotState').innerText = data.robot_state;
      document.getElementById('valTicksL').innerText = data.ticks_l;
      document.getElementById('valTicksR').innerText = data.ticks_r;

      // Cập nhật 5 mắt IR
      const mask = data.line_mask;
      document.getElementById('irL2').className = 'ir-sensor' + ((mask & 16) ? ' active' : '');
      document.getElementById('irL1').className = 'ir-sensor' + ((mask & 8)  ? ' active' : '');
      document.getElementById('irC').className  = 'ir-sensor' + ((mask & 4)  ? ' active' : '');
      document.getElementById('irR1').className = 'ir-sensor' + ((mask & 2)  ? ' active' : '');
      document.getElementById('irR2').className = 'ir-sensor' + ((mask & 1)  ? ' active' : '');

      // Thông báo điều khiển
      const notif = document.getElementById('cmdNotification');
      if (data.pending_command) {
        notif.innerText = '📡 Đang phát lệnh #' + data.pending_command.id + ' (' + data.pending_command.cmd + ') chờ xe nhận...';
        notif.style.color = 'var(--warning)';
      } else if (data.last_cmd_action && data.last_cmd_action !== 'NONE') {
        notif.innerText = '✅ Xe đã nhận & thực thi lệnh: ' + data.last_cmd_action;
        notif.style.color = 'var(--success)';
      }

      // Nút Auto Loop
      const btnLoop = document.getElementById('btnAutoLoop');
      if (btnLoop && data.auto_loop !== undefined) {
        if (data.auto_loop) {
          btnLoop.className = 'btn-danger';
          btnLoop.innerText = '⏹️ Tắt Vòng Lặp Tự Động';
        } else {
          btnLoop.className = 'btn-success';
          btnLoop.innerText = '🔄 Bật Vòng Lặp A-B-C-D-A';
        }
      }

      drawMap();
    } catch (e) {
      console.error(e);
    }
  }

  async function updateLogTable() {
    try {
      const res = await fetch('/api/logs');
      const data = await res.json();
      const tbody = document.getElementById('logTbody');
      if (!data.logs || data.logs.length === 0) return;

      let html = '';
      const logs = data.logs.slice(-20).reverse();
      logs.forEach(l => {
        html += `<tr>
          <td>${l.t}</td>
          <td>${l.x.toFixed(3)}</td>
          <td>${l.y.toFixed(3)}</td>
          <td>${l.yaw.toFixed(1)}°</td>
          <td>${l.dist.toFixed(2)}m</td>
          <td>${l.speed.toFixed(1)}</td>
          <td><code>${l.mask}</code></td>
          <td style="color:#38bdf8">${l.v ? l.v.toFixed(2)+'V' : '--'}</td>
          <td style="color:#f59e0b">${l.a ? l.a.toFixed(2)+'A' : '--'}</td>
          <td style="color:#c084fc">${l.mah ? l.mah.toFixed(1) : '--'}</td>
          <td>${l.pwmL}/${l.pwmR}</td>
          <td><span style="color:#f59e0b">${l.state}</span></td>
        </tr>`;
      });
      tbody.innerHTML = html;
    } catch(e) {}
  }

  async function sendCmd(act) {
    document.getElementById('cmdNotification').innerText = '⏳ Đang gửi lệnh: ' + act.toUpperCase() + '...';
    try {
      const res = await fetch('/api/command', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: act})
      });
      const data = await res.json();
      document.getElementById('cmdNotification').innerText = '📡 Đã phát lệnh #' + data.cmd_id + ' (' + data.cmd + ') -> Chờ xe nhận...';
    } catch(e) {
      document.getElementById('cmdNotification').innerText = '❌ Lỗi gửi lệnh: ' + e;
    }
  }

  async function sendMission(mId) {
    document.getElementById('cmdNotification').innerText = '⏳ Đang gửi nhiệm vụ: ' + mId + '...';
    try {
      const res = await fetch('/api/command', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: 'mission', mission_id: mId})
      });
      const data = await res.json();
      document.getElementById('cmdNotification').innerText = '📡 Đã phát nhiệm vụ #' + data.cmd_id + ' (' + data.cmd + ') -> Chờ xe nhận...';
    } catch(e) {
      document.getElementById('cmdNotification').innerText = '❌ Lỗi gửi nhiệm vụ: ' + e;
    }
  }

  async function sendGoTo(st) {
    document.getElementById('cmdNotification').innerText = '⏳ Đang lập lộ trình đến Trạm ' + st + '...';
    try {
      const res = await fetch('/api/command', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: 'go_to', station: st})
      });
      const data = await res.json();
      document.getElementById('cmdNotification').innerText = '📡 Đã phát lệnh #' + data.cmd_id + ' (ĐIỂM-ĐIỂM: ' + data.cmd + ') -> Chờ xe nhận...';
    } catch(e) {
      document.getElementById('cmdNotification').innerText = '❌ Lỗi gửi lệnh: ' + e;
    }
  }

  async function changeBatteryType() {
    currentBatteryType = document.getElementById('selBatteryType').value;
    await fetch('/api/command', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({action: 'set_battery_type', battery_type: currentBatteryType})
    });
  }

  async function resetOdom() {
    await sendCmd('reset_odom');
  }

  let autoLoopActive = false;
  async function toggleAutoLoop() {
    autoLoopActive = !autoLoopActive;
    if (autoLoopActive) {
      await sendCmd('auto_loop_on');
    } else {
      await sendCmd('auto_loop_off');
    }
  }

  function exportCsv() {
    window.location.href = '/api/export_csv';
  }

  async function clearLogs() {
    await fetch('/api/clear_logs', {method: 'POST'});
    document.getElementById('logTbody').innerHTML = '<tr><td colspan="12" style="text-align:center;">Đã xóa log</td></tr>';
  }

  resizeAllCanvases();
  setInterval(updateDashboard, 100);
  setInterval(updateLogTable, 500);
</script>
</body>
</html>
"""
        body = html.encode('utf-8')
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

def main():
    print("=" * 75)
    print("    AGV_VER2 - WMS DIGITAL TWIN & POWER MANAGEMENT SERVER (PORT 5000)")
    print("=" * 75)
    ips = get_local_ips()
    print(f">> Server đang chạy tại:")
    print(f"   [Cục bộ]  : http://localhost:{PORT}")
    for ip in ips:
        print(f"   [Mạng Lan]: http://{ip}:{PORT}")
    print(f">> Sa bàn thực tế: Dài {MAP_WIDTH_X}m (X) x Rộng {MAP_HEIGHT_Y}m (Y)")
    print(f">> Cảm biến năng lượng: INA226 (I2C1: SDA=GPIO 8, SCL=GPIO 9)")
    print(f">> Cổng UDP Discovery: Port {UDP_PORT}")
    print("=" * 75)

    # Chạy UDP Broadcaster ở chế độ nền
    threading.Thread(target=udp_broadcaster, daemon=True).start()

    # Chạy HTTP Server
    server = ThreadingHTTPServer((HOST, PORT), AGVHttpHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n>> Đã tắt Server.")

if __name__ == "__main__":
    main()
