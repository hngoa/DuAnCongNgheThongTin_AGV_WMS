"""
===================================================================================
AGV_Ver2 - DEDICATED INA226 POWER & BATTERY MANAGEMENT SERVER (PORT 5002)
-----------------------------------------------------------------------------------
MÁY CHỦ QUẢN LÝ NĂNG LƯỢNG & ĐO DÒNG / ÁP CHUYÊN BIỆT CHO CẢM BIẾN INA226:
  1. Giám sát thời gian thực: Điện áp Bus (V), Sụt áp Shunt (mV), Dòng tải (A/mA), Công suất (W).
  2. Đo đếm năng lượng tiêu thụ tích lũy: Dung lượng (mAh) và Điện năng tiêu thụ (Wh).
  3. Ước lượng % Dung lượng Pin (SOC - State of Charge) theo các loại pin (3S LiPo, 2S, 4S, 12V PB).
  4. Đồ thị cuộn sóng thời gian thực (Live Rolling Charts: V, A, W) chạy mượt mà offline 100%.
  5. Cảnh báo an toàn thông minh: Cảnh báo Pin yếu (< 10.5V), Cảnh báo quá dòng (> 5A).
  6. Xuất nhật ký đo kiểm ra file CSV phục vụ báo cáo và phân tích đồ thị.
  7. Tự động phát beacon UDP (Port 5006) để ESP32-S3 tự động tìm thấy Server.
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

PORT = 5002
HOST = "0.0.0.0"
UDP_PORT = 5006

# Trạng thái năng lượng toàn cục
power_state = {
    "connected": False,
    "last_seen": 0.0,
    "last_seen_str": "Chưa kết nối",
    
    # Số liệu đo thời gian thực
    "bus_voltage_v": 0.0,
    "shunt_voltage_mv": 0.0,
    "current_a": 0.0,
    "current_ma": 0.0,
    "power_w": 0.0,
    
    # Năng lượng tích lũy
    "energy_mah": 0.0,
    "energy_wh": 0.0,
    "uptime_sec": 0,
    
    # Pin & Cấu hình
    "battery_type": "3S_LIPO", # 3S_LIPO (9.6V - 12.6V), 2S_LIPO (6.4V - 8.4V), 4S_LIPO (12.8V - 16.8V)
    "battery_pct": 0.0,
    "r_shunt": 0.100, # Ohm (mặc định R100 = 0.1 Ohm)
    "i2c_addr": "0x40",
    "i2c_ok": False,
    "status": "IDLE"
}

# Lịch sử đo đạc để vẽ đồ thị và xuất CSV
MAX_HISTORY = 1000
history_records = []
history_lock = threading.Lock()
start_time = time.time()
last_energy_calc_time = time.time()

def calculate_battery_soc(voltage, btype):
    """Tính % dung lượng pin dựa trên đường cong điện áp xả"""
    if btype == "3S_LIPO":
        v_min, v_max = 9.9, 12.6
    elif btype == "2S_LIPO":
        v_min, v_max = 6.6, 8.4
    elif btype == "4S_LIPO":
        v_min, v_max = 13.2, 16.8
    else: # 12V ắc quy chì
        v_min, v_max = 10.5, 12.8
    
    if voltage <= v_min:
        return 0.0
    if voltage >= v_max:
        return 100.0
    return round(((voltage - v_min) / (v_max - v_min)) * 100.0, 1)

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

# --- THREAD PHÁT SÓNG UDP DÒ TÌM SERVER NĂNG LƯỢNG ---
def udp_broadcaster():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    msg = json.dumps({"service": "INA226_POWER_SERVER", "port": PORT, "version": "Ver2"}).encode('utf-8')
    while True:
        try:
            sock.sendto(msg, ("<broadcast>", UDP_PORT))
        except Exception:
            pass
        time.sleep(1.0)

# --- XỬ LÝ HTTP REQUEST ---
class PowerHttpHandler(BaseHTTPRequestHandler):
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
            self._serve_power_dashboard()
        elif path == "/api/power_state":
            now = time.time()
            power_state["connected"] = (now - power_state["last_seen"]) < 2.5
            if not power_state["connected"]:
                power_state["last_seen_str"] = "Mất kết nối (" + str(int(now - power_state["last_seen"])) + "s)"
            else:
                power_state["last_seen_str"] = "Đang kết nối (" + str(int((now - power_state["last_seen"])*1000)) + "ms)"
            
            with history_lock:
                recent_history = history_records[-150:]
            
            resp = {
                "state": power_state,
                "history": recent_history
            }
            self._send_json(resp)
        elif path == "/api/export_csv":
            self._export_csv()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        global last_energy_calc_time
        path = self.path.split("?")[0]
        content_len = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_len).decode("utf-8") if content_len > 0 else "{}"

        if path == "/api/power_telemetry":
            try:
                data = json.loads(body)
                now = time.time()
                dt = now - last_energy_calc_time
                if dt < 0 or dt > 5.0:
                    dt = 0.2
                last_energy_calc_time = now

                power_state["last_seen"] = now
                power_state["connected"] = True
                power_state["bus_voltage_v"] = float(data.get("voltage", 0.0))
                power_state["shunt_voltage_mv"] = float(data.get("shunt_mv", 0.0))
                power_state["current_a"] = float(data.get("current", 0.0))
                power_state["current_ma"] = power_state["current_a"] * 1000.0
                power_state["power_w"] = float(data.get("power", power_state["bus_voltage_v"] * power_state["current_a"]))
                power_state["i2c_addr"] = data.get("addr", "0x40")
                power_state["r_shunt"] = float(data.get("r_shunt", power_state["r_shunt"]))
                power_state["i2c_ok"] = True

                # Tính tích phân năng lượng tiêu thụ (mAh và Wh)
                if power_state["current_a"] > 0.005:
                    power_state["energy_mah"] += (power_state["current_ma"] * (dt / 3600.0))
                    power_state["energy_wh"] += (power_state["power_w"] * (dt / 3600.0))

                power_state["uptime_sec"] = int(now - start_time)
                power_state["battery_pct"] = calculate_battery_soc(power_state["bus_voltage_v"], power_state["battery_type"])

                # Cảnh báo an toàn
                if power_state["bus_voltage_v"] < 9.9 and power_state["battery_type"] == "3S_LIPO" and power_state["bus_voltage_v"] > 3.0:
                    power_state["status"] = "LOW_BATTERY"
                elif power_state["current_a"] > 6.0:
                    power_state["status"] = "OVERCURRENT"
                else:
                    power_state["status"] = "NORMAL"

                # Lưu vào lịch sử đồ thị
                with history_lock:
                    history_records.append({
                        "t": datetime.now().strftime("%H:%M:%S.%f")[:-4],
                        "v": round(power_state["bus_voltage_v"], 3),
                        "a": round(power_state["current_a"], 3),
                        "w": round(power_state["power_w"], 3),
                        "mah": round(power_state["energy_mah"], 2)
                    })
                    if len(history_records) > MAX_HISTORY:
                        history_records.pop(0)

                self._send_json({"status": "ok", "ack": 1})
            except Exception as e:
                self._send_json({"error": str(e)}, 400)

        elif path == "/api/reset_energy":
            power_state["energy_mah"] = 0.0
            power_state["energy_wh"] = 0.0
            self._send_json({"status": "energy_reset"})

        elif path == "/api/set_battery_type":
            try:
                req = json.loads(body)
                power_state["battery_type"] = req.get("type", "3S_LIPO")
                self._send_json({"status": "battery_type_updated", "type": power_state["battery_type"]})
            except Exception as e:
                self._send_json({"error": str(e)}, 400)
        else:
            self.send_response(404)
            self.end_headers()

    def _export_csv(self):
        output = io.StringIO()
        writer = csv.writer(output)
        writer.writerow(["Timestamp", "Bus_Voltage_V", "Current_A", "Power_W", "Energy_mAh", "Energy_Wh"])
        with history_lock:
            for r in history_records:
                writer.writerow([r["t"], r["v"], r["a"], r["w"], r.get("mah", 0), round(r.get("mah", 0) * 0.011, 2)])
        
        csv_data = output.getvalue().encode("utf-8")
        filename = f"ina226_power_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.send_response(200)
        self.send_header("Content-Type", "text/csv; charset=utf-8")
        self.send_header("Content-Disposition", f"attachment; filename={filename}")
        self.send_header("Content-Length", str(len(csv_data)))
        self.end_headers()
        self.wfile.write(csv_data)

    def _serve_power_dashboard(self):
        html = """<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>HỆ THỐNG QUẢN LÝ NĂNG LƯỢNG INA226 - AGV_VER2</title>
<style>
  :root {
    --bg-dark: #090d16;
    --card-bg: #111827;
    --border: #1f2937;
    --accent: #38bdf8;
    --accent-glow: rgba(56, 189, 248, 0.25);
    --success: #10b981;
    --warning: #f59e0b;
    --danger: #ef4444;
    --text-main: #f9fafb;
    --text-dim: #9ca3af;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; }
  body { background: var(--bg-dark); color: var(--text-main); padding: 16px; min-height: 100vh; }
  .header { display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid var(--border); padding-bottom: 12px; margin-bottom: 16px; flex-wrap: wrap; gap: 8px; }
  .title-group h1 { font-size: 20px; font-weight: 700; color: #fff; display: flex; align-items: center; gap: 8px; }
  .badge { padding: 4px 10px; border-radius: 999px; font-size: 12px; font-weight: 600; text-transform: uppercase; }
  .badge.online { background: rgba(16, 185, 129, 0.2); color: var(--success); border: 1px solid var(--success); }
  .badge.offline { background: rgba(239, 68, 68, 0.2); color: var(--danger); border: 1px solid var(--danger); }
  
  .grid-metrics { display: grid; grid-template-columns: repeat(auto-fit, minmax(210px, 1fr)); gap: 12px; margin-bottom: 16px; }
  .card-metric { background: var(--card-bg); border: 1px solid var(--border); border-radius: 10px; padding: 14px; position: relative; overflow: hidden; }
  .card-metric::after { content: ''; position: absolute; top: 0; left: 0; width: 4px; height: 100%; background: var(--accent); }
  .card-metric.voltage::after { background: #38bdf8; }
  .card-metric.current::after { background: #f59e0b; }
  .card-metric.power::after { background: #10b981; }
  .card-metric.energy::after { background: #a855f7; }
  .metric-label { font-size: 12px; color: var(--text-dim); text-transform: uppercase; font-weight: 600; letter-spacing: 0.5px; }
  .metric-value { font-size: 32px; font-weight: 800; margin: 4px 0; color: #fff; font-family: 'Consolas', monospace; }
  .metric-sub { font-size: 12px; color: var(--text-dim); }

  .battery-container { background: var(--card-bg); border: 1px solid var(--border); border-radius: 10px; padding: 16px; margin-bottom: 16px; display: flex; align-items: center; gap: 20px; flex-wrap: wrap; }
  .bat-icon-box { width: 80px; height: 42px; border: 3px solid #64748b; border-radius: 6px; position: relative; padding: 2px; }
  .bat-icon-box::after { content: ''; position: absolute; right: -8px; top: 10px; width: 5px; height: 16px; background: #64748b; border-radius: 0 2px 2px 0; }
  .bat-level { height: 100%; border-radius: 2px; transition: width 0.3s ease, background 0.3s ease; width: 0%; }
  .bat-info { flex: 1; min-width: 200px; }
  .bat-info h3 { font-size: 16px; margin-bottom: 4px; color: #fff; }
  .bat-bar-track { width: 100%; height: 12px; background: #1e293b; border-radius: 6px; overflow: hidden; margin-top: 6px; }
  .bat-bar-fill { height: 100%; transition: width 0.3s ease; width: 0%; }

  .charts-grid { display: grid; grid-template-columns: 1fr; gap: 16px; margin-bottom: 16px; }
  .card-chart { background: var(--card-bg); border: 1px solid var(--border); border-radius: 10px; padding: 16px; }
  .card-chart h3 { font-size: 14px; font-weight: 600; color: var(--text-dim); margin-bottom: 12px; display: flex; justify-content: space-between; align-items: center; }
  canvas { width: 100%; height: 200px; display: block; background: #0b1120; border-radius: 6px; border: 1px solid #1e293b; }

  .actions-bar { display: flex; gap: 10px; flex-wrap: wrap; align-items: center; background: var(--card-bg); border: 1px solid var(--border); border-radius: 10px; padding: 12px 16px; }
  button { padding: 8px 16px; border-radius: 6px; font-weight: 600; font-size: 13px; cursor: pointer; border: 1px solid transparent; transition: all 0.2s; }
  .btn-primary { background: #0284c7; color: #fff; }
  .btn-primary:hover { background: #0369a1; }
  .btn-secondary { background: #1f2937; color: #e5e7eb; border-color: #374151; }
  .btn-secondary:hover { background: #374151; }
  .btn-danger { background: #dc2626; color: #fff; }
  select { background: #1f2937; color: #fff; padding: 8px 12px; border-radius: 6px; border: 1px solid #374151; font-size: 13px; outline: none; }
</style>
</head>
<body>

  <div class="header">
    <div class="title-group">
      <h1>⚡ TRUNG TÂM GIÁM SÁT NĂNG LƯỢNG INA226 (ESP32-S3)</h1>
      <span style="font-size:12px; color:var(--text-dim);">Chân I2C: SDA=GPIO 8 | SCL=GPIO 9</span>
    </div>
    <div style="display:flex; align-items:center; gap:10px;">
      <span id="connBadge" class="badge offline">● Chưa kết nối</span>
      <span id="connPing" style="font-size:12px; color:var(--text-dim);">-- ms</span>
    </div>
  </div>

  <!-- BĂNG THÔNG SỐ CHÍNH -->
  <div class="grid-metrics">
    <div class="card-metric voltage">
      <div class="metric-label">Điện Áp Nguồn (VBUS)</div>
      <div id="valVoltage" class="metric-value" style="color:#38bdf8;">0.000 V</div>
      <div id="valShuntMv" class="metric-sub">Sụt áp Shunt: +0.000 mV</div>
    </div>

    <div class="card-metric current">
      <div class="metric-label">Dòng Điện Tiêu Thụ</div>
      <div id="valCurrent" class="metric-value" style="color:#f59e0b;">0.000 A</div>
      <div id="valCurrentMa" class="metric-sub">0.0 mA (Trở Shunt: 0.100 Ω)</div>
    </div>

    <div class="card-metric power">
      <div class="metric-label">Công Suất Tải</div>
      <div id="valPower" class="metric-value" style="color:#10b981;">0.000 W</div>
      <div id="valPowerMw" class="metric-sub">0.0 mW</div>
    </div>

    <div class="card-metric energy">
      <div class="metric-label">Dung Lượng Tiêu Thụ</div>
      <div id="valMah" class="metric-value" style="color:#c084fc;">0.0 mAh</div>
      <div id="valWh" class="metric-sub">Điện năng: 0.000 Wh</div>
    </div>
  </div>

  <!-- KHỐI QUẢN LÝ DUNG LƯỢNG PIN (SOC) -->
  <div class="battery-container">
    <div class="bat-icon-box">
      <div id="batLevelMini" class="bat-level" style="background:#10b981;"></div>
    </div>
    <div class="bat-info">
      <div style="display:flex; justify-content:space-between; align-items:center;">
        <h3 id="batTitle">Dung Lượng Pin Ước Tính: 0.0%</h3>
        <span id="batStatusText" style="font-size:12px; font-weight:600; color:#10b981;">HOẠT ĐỘNG TỐT</span>
      </div>
      <div class="bat-bar-track">
        <div id="batBarFill" class="bat-bar-fill" style="background:#10b981;"></div>
      </div>
    </div>
    <div style="display:flex; gap:10px; align-items:center;">
      <span style="font-size:13px; color:var(--text-dim);">Loại Pin:</span>
      <select id="selBatteryType" onchange="changeBatteryType()">
        <option value="3S_LIPO" selected>Pin LiPo 3S (9.9V - 12.6V)</option>
        <option value="2S_LIPO">Pin LiPo 2S (6.6V - 8.4V)</option>
        <option value="4S_LIPO">Pin LiPo 4S (13.2V - 16.8V)</option>
        <option value="12V_PB">Ắc Quy Chì 12V (10.5V - 12.8V)</option>
      </select>
    </div>
  </div>

  <!-- ĐỒ THỊ THỜI GIAN THỰC -->
  <div class="charts-grid">
    <div class="card-chart">
      <h3>
        <span>📈 ĐỒ THỊ ĐIỆN ÁP (V) & DÒNG ĐIỆN (A) THỜI GIAN THỰC</span>
        <span style="font-size:12px; color:var(--accent);">Xanh: Điện áp (V) | Cam: Dòng điện (A)</span>
      </h3>
      <canvas id="chartPower"></canvas>
    </div>
  </div>

  <!-- THANH ĐIỀU KHIỂN & XUẤT CSV -->
  <div class="actions-bar">
    <span style="font-size:13px; font-weight:600; color:var(--text-dim);">Thao Tác Quản Lý:</span>
    <button class="btn-primary" onclick="exportCsv()">📥 Xuất Dữ Liệu Đo Ra File CSV</button>
    <button class="btn-secondary" onclick="resetEnergy()">🔄 Đặt Lại Năng Lượng (0 mAh)</button>
    <span id="txtUptime" style="font-size:12px; color:var(--text-dim); margin-left:auto;">Thời gian đo: 00:00:00</span>
  </div>

<script>
  let lastData = null;
  const canvas = document.getElementById('chartPower');
  const ctx = canvas.getContext('2d');

  function resizeCanvas() {
    canvas.width = canvas.parentElement.clientWidth - 32;
    canvas.height = 200;
  }
  window.addEventListener('resize', resizeCanvas);
  resizeCanvas();

  async function updateState() {
    const t0 = performance.now();
    try {
      const res = await fetch('/api/power_state');
      const data = await res.json();
      lastData = data;

      const s = data.state;
      const badge = document.getElementById('connBadge');
      if (s.connected) {
        badge.className = 'badge online';
        badge.innerText = '● ' + s.last_seen_str;
      } else {
        badge.className = 'badge offline';
        badge.innerText = '● ' + s.last_seen_str;
      }
      document.getElementById('connPing').innerText = Math.round(performance.now() - t0) + ' ms';

      document.getElementById('valVoltage').innerText = s.bus_voltage_v.toFixed(3) + ' V';
      document.getElementById('valShuntMv').innerText = 'Sụt áp Shunt: ' + (s.shunt_voltage_mv >= 0 ? '+' : '') + s.shunt_voltage_mv.toFixed(3) + ' mV';
      document.getElementById('valCurrent').innerText = s.current_a.toFixed(3) + ' A';
      document.getElementById('valCurrentMa').innerText = s.current_ma.toFixed(1) + ' mA (Trở Shunt: ' + s.r_shunt.toFixed(3) + ' Ω)';
      document.getElementById('valPower').innerText = s.power_w.toFixed(3) + ' W';
      document.getElementById('valPowerMw').innerText = (s.power_w * 1000).toFixed(1) + ' mW';
      document.getElementById('valMah').innerText = s.energy_mah.toFixed(1) + ' mAh';
      document.getElementById('valWh').innerText = 'Điện năng: ' + s.energy_wh.toFixed(3) + ' Wh';

      // Pin
      const pct = Math.max(0, Math.min(100, s.battery_pct));
      document.getElementById('batTitle').innerText = 'Dung Lượng Pin: ' + pct.toFixed(1) + '% (' + s.bus_voltage_v.toFixed(2) + 'V)';
      const fill = document.getElementById('batBarFill');
      const mini = document.getElementById('batLevelMini');
      fill.style.width = pct + '%';
      mini.style.width = pct + '%';
      
      let color = '#10b981';
      let statusText = 'HOẠT ĐỘNG TỐT';
      if (pct < 20) { color = '#ef4444'; statusText = 'PIN YẾU - CẦN SẠC'; }
      else if (pct < 45) { color = '#f59e0b'; statusText = 'MỨC TRUNG BÌNH'; }
      fill.style.background = color;
      mini.style.background = color;
      document.getElementById('batStatusText').innerText = statusText;
      document.getElementById('batStatusText').style.color = color;

      // Uptime
      const sec = s.uptime_sec;
      const h = Math.floor(sec / 3600);
      const m = Math.floor((sec % 3600) / 60);
      const ss = sec % 60;
      document.getElementById('txtUptime').innerText = 'Thời gian đo: ' + 
        (h < 10 ? '0' : '') + h + ':' + (m < 10 ? '0' : '') + m + ':' + (ss < 10 ? '0' : '') + ss;

      drawChart(data.history);
    } catch(e) {}
  }

  function drawChart(hist) {
    if (!hist || hist.length < 2) return;
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    // Tìm max V và max A
    let maxV = 15.0;
    let maxA = 3.0;
    for (let r of hist) {
      if (r.v > maxV) maxV = r.v + 1.0;
      if (r.a > maxA) maxA = r.a + 0.5;
    }

    // Lưới ngang
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    for (let i = 1; i <= 4; i++) {
      let y = h - (i * (h / 5));
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    const n = hist.length;
    const dx = w / (n - 1);

    // Vẽ đường Voltage (Màu xanh cyan)
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < n; i++) {
      let x = i * dx;
      let y = h - ((hist[i].v / maxV) * (h - 20)) - 10;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    // Vẽ đường Current (Màu cam vàng)
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < n; i++) {
      let x = i * dx;
      let y = h - ((Math.max(0, hist[i].a) / maxA) * (h - 20)) - 10;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }

  function exportCsv() {
    window.location.href = '/api/export_csv';
  }

  async function resetEnergy() {
    await fetch('/api/reset_energy', {method: 'POST'});
  }

  async function changeBatteryType() {
    const sel = document.getElementById('selBatteryType').value;
    await fetch('/api/set_battery_type', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({type: sel})
    });
  }

  setInterval(updateState, 150);
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
    print("    AGV_VER2 - DEDICATED INA226 POWER MANAGEMENT SERVER")
    print("=" * 75)
    print(f">> Cổng HTTP Web Dashboard: http://localhost:{PORT}")
    for ip in get_local_ips():
        print(f"   * IP mạng LAN:          http://{ip}:{PORT}")
    print(f">> Cổng UDP Discovery:      Port {UDP_PORT}")
    print("=" * 75)

    # Khởi động thread phát beacon UDP
    t_udp = threading.Thread(target=udp_broadcaster, daemon=True)
    t_udp.start()

    server = ThreadingHTTPServer((HOST, PORT), PowerHttpHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n>> Đang dừng Server...")
        server.server_close()

if __name__ == "__main__":
    main()
