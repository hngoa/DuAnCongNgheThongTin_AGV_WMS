"""
===================================================================================
AGV_Ver2 - SERVER CHUYÊN DỤNG CALIB ENCODER 1 MÉT (TÂM TRỤC 2 BÁNH XE)
-----------------------------------------------------------------------------------
Mục đích:
  - Thu thập chính xác số xung Encoder Trái & Phải khi đẩy xe 1 mét (hoặc khoảng cách tùy chọn).
  - Hoạt động độc lập tại Port 5001 (không trùng với Dashboard Port 5000).
  - Tự động tính toán: Ticks/mét, Ticks/cm, Đường kính bánh hiệu dụng (mm), Độ lệch L/R.
  - Hỗ trợ lưu nhiều lần đo (Sample 1, 2, 3...) và tự động lấy giá trị trung bình chuẩn nhất.
===================================================================================
"""

from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import json
import time
from datetime import datetime
import threading
import socket
import csv
import io
import math

PORT = 5001
HOST = "0.0.0.0"

# Trạng thái toàn cục bộ đo Encoder
state = {
    "connected": False,
    "last_seen": 0,
    "last_seen_str": "Chưa kết nối",
    "ticks_l": 0,
    "ticks_r": 0,
    "ticks_avg": 0,
    "delta_lr": 0,
    "pending_command": None,
    "cmd_id": 0,
    "last_ack_id": 0
}

# Lịch sử các lần đo mẫu
samples = []
sample_lock = threading.Lock()

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

class EncoderCalibHttpHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass # Tắt log console mặc định

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
                state["last_seen_str"] = "Mất kết nối (" + str(int(now - state["last_seen"])) + "s)" if state["last_seen"] > 0 else "Chưa kết nối"
            else:
                state["last_seen_str"] = "Đang kết nối (" + str(int((now - state["last_seen"])*1000)) + "ms)"
            with sample_lock:
                samples_copy = list(samples)
            self._send_json({"state": state, "samples": samples_copy})
        elif path == "/api/export_csv":
            self._export_samples_csv()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        path = self.path.split("?")[0]
        content_len = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_len).decode("utf-8") if content_len > 0 else "{}"

        if path == "/api/encoder_data":
            try:
                data = json.loads(body)
                now = time.time()
                state["last_seen"] = now
                state["connected"] = True

                tL = int(data.get("ticks_l", 0))
                tR = int(data.get("ticks_r", 0))
                state["ticks_l"] = tL
                state["ticks_r"] = tR
                state["ticks_avg"] = round((tL + tR) / 2.0, 1)
                state["delta_lr"] = tL - tR

                ack_id = int(data.get("ack_id", 0))
                state["last_ack_id"] = ack_id
                if state["pending_command"] and ack_id >= state["pending_command"]["id"]:
                    state["pending_command"] = None

                self._send_json({
                    "status": "ok",
                    "cmd": state["pending_command"]["cmd"] if state["pending_command"] else None,
                    "cmd_id": state["pending_command"]["id"] if state["pending_command"] else 0
                })
            except Exception as e:
                self._send_json({"error": str(e)}, 400)

        elif path == "/api/command":
            try:
                cmd_data = json.loads(body)
                action = cmd_data.get("action", "")
                if action == "reset":
                    state["cmd_id"] += 1
                    state["pending_command"] = {"id": state["cmd_id"], "cmd": "RESET"}
                    state["ticks_l"] = 0
                    state["ticks_r"] = 0
                    state["ticks_avg"] = 0
                    state["delta_lr"] = 0
                self._send_json({"status": "command_queued", "cmd_id": state["cmd_id"]})
            except Exception as e:
                self._send_json({"error": str(e)}, 400)

        elif path == "/api/save_sample":
            try:
                item = json.loads(body)
                with sample_lock:
                    samples.append({
                        "id": len(samples) + 1,
                        "time": datetime.now().strftime("%H:%M:%S"),
                        "distance_m": float(item.get("distance_m", 1.0)),
                        "ticks_l": int(item.get("ticks_l", state["ticks_l"])),
                        "ticks_r": int(item.get("ticks_r", state["ticks_r"])),
                        "ticks_avg": float(item.get("ticks_avg", state["ticks_avg"])),
                        "ticks_per_m": float(item.get("ticks_per_m", 0.0)),
                        "wheel_diam_mm": float(item.get("wheel_diam_mm", 0.0)),
                        "delta_pct": float(item.get("delta_pct", 0.0)),
                        "note": str(item.get("note", "Không tải"))
                    })
                self._send_json({"status": "sample_saved", "total": len(samples)})
            except Exception as e:
                self._send_json({"error": str(e)}, 400)

        elif path == "/api/clear_samples":
            with sample_lock:
                samples.clear()
            self._send_json({"status": "cleared"})
        else:
            self.send_response(404)
            self.end_headers()

    def _export_samples_csv(self):
        output = io.StringIO()
        writer = csv.writer(output)
        writer.writerow(["Lan_Do", "Gio", "Quang_Duong_m", "Xung_Trai", "Xung_Phai", "Xung_Trung_Binh", "Ticks_Per_Meter", "Duong_Kinh_Banh_mm", "Do_Lech_Phan_Tram", "Ghi_Chu"])
        with sample_lock:
            for s in samples:
                writer.writerow([s["id"], s["time"], s["distance_m"], s["ticks_l"], s["ticks_r"], s["ticks_avg"], s["ticks_per_m"], s["wheel_diam_mm"], s["delta_pct"], s["note"]])
        csv_bytes = output.getvalue().encode('utf-8')
        filename = f"encoder_calib_1m_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.send_response(200)
        self.send_header("Content-Type", "text/csv; charset=utf-8")
        self.send_header("Content-Disposition", f"attachment; filename={filename}")
        self.send_header("Content-Length", str(len(csv_bytes)))
        self.end_headers()
        self.wfile.write(csv_bytes)

    def _serve_dashboard_html(self):
        html = """<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AGV_Ver2 - Calib Encoder 1 Mét Chuẩn Xác</title>
<style>
  :root {
    --bg: #090d16; --card: #131c2e; --border: #23334d;
    --text: #f1f5f9; --text-dim: #94a3b8;
    --accent: #38bdf8; --success: #10b981; --warning: #f59e0b; --danger: #ef4444;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
  body { background: var(--bg); color: var(--text); padding: 18px; max-width: 1200px; margin: 0 auto; }
  
  header { display: flex; justify-content: space-between; align-items: center; padding-bottom: 14px; border-bottom: 1px solid var(--border); margin-bottom: 18px; }
  h1 { font-size: 20px; font-weight: 700; color: var(--accent); display: flex; align-items: center; gap: 8px; }
  .badge { padding: 4px 12px; border-radius: 999px; font-size: 12px; font-weight: 600; display: inline-flex; align-items: center; gap: 6px; }
  .badge.online { background: rgba(16, 185, 129, 0.2); color: var(--success); border: 1px solid var(--success); }
  .badge.offline { background: rgba(239, 68, 68, 0.2); color: var(--danger); border: 1px solid var(--danger); }

  .hero-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 14px; margin-bottom: 18px; }
  .counter-box { background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 18px; text-align: center; position: relative; }
  .counter-box.main { border-color: var(--accent); box-shadow: 0 0 20px rgba(56, 189, 248, 0.15); }
  .counter-title { font-size: 13px; color: var(--text-dim); text-transform: uppercase; font-weight: 600; margin-bottom: 6px; }
  .counter-num { font-size: 42px; font-weight: 800; font-family: 'Consolas', monospace; color: var(--accent); }
  .counter-sub { font-size: 12px; color: var(--text-dim); margin-top: 4px; }
  
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 18px; margin-bottom: 18px; }
  .card h2 { font-size: 16px; font-weight: 600; margin-bottom: 14px; display: flex; justify-content: space-between; align-items: center; }

  .calc-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px; }
  .calc-box { background: #0c1424; padding: 12px; border-radius: 8px; border: 1px solid var(--border); }
  .calc-label { font-size: 11px; color: var(--text-dim); text-transform: uppercase; margin-bottom: 4px; }
  .calc-val { font-size: 20px; font-weight: 700; color: var(--success); font-family: monospace; }
  .calc-hint { font-size: 11px; color: var(--text-dim); margin-top: 2px; }

  /* Input & Controls */
  .control-row { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; margin-top: 14px; padding-top: 14px; border-top: 1px solid var(--border); }
  input, select { background: #0c1424; border: 1px solid var(--border); color: var(--text); padding: 9px 12px; border-radius: 6px; font-size: 14px; }
  button { padding: 9px 16px; border-radius: 6px; border: none; font-weight: 600; cursor: pointer; transition: all 0.2s; font-size: 13px; display: inline-flex; align-items: center; gap: 6px; }
  button:hover { opacity: 0.9; transform: translateY(-1px); }
  .btn-primary { background: var(--accent); color: #090d16; }
  .btn-success { background: var(--success); color: #fff; }
  .btn-danger  { background: var(--danger); color: #fff; }
  .btn-secondary { background: #23334d; color: var(--text); }

  /* Table */
  table { width: 100%; border-collapse: collapse; font-size: 13px; text-align: left; }
  th { background: #0c1424; padding: 10px 12px; color: var(--text-dim); font-weight: 600; border-bottom: 1px solid var(--border); }
  td { padding: 9px 12px; border-bottom: 1px solid #1a273e; font-family: monospace; }
  tr:hover td { background: rgba(56, 189, 248, 0.04); }
  .avg-row { background: rgba(16, 185, 129, 0.1); font-weight: bold; border-top: 2px solid var(--success); }
  .avg-row td { color: var(--success); }

  .guide-box { background: #0f1c30; border-left: 4px solid var(--accent); padding: 12px 16px; border-radius: 0 8px 8px 0; font-size: 13px; line-height: 1.6; color: #cbd5e1; margin-bottom: 16px; }
  .guide-box ol { margin-left: 20px; margin-top: 6px; }
  .code-pill { background: #1e293b; color: #38bdf8; padding: 2px 6px; border-radius: 4px; font-family: monospace; font-size: 12px; }
</style>
</head>
<body>

<header>
  <h1>🎯 AGV_Ver2 - Bộ Hiệu Chuẩn Encoder 1 Mét (Zero-Slip)</h1>
  <div style="display:flex; align-items:center; gap:12px;">
    <span id="connBadge" class="badge offline">● Đang tìm ESP32...</span>
    <span id="connPing" style="font-size:12px; color:var(--text-dim);">-- ms</span>
  </div>
</header>

<div class="guide-box">
  <strong>📋 QUY TRÌNH THỰC HIỆN ĐO 1 MÉT CHUẨN XÁC:</strong>
  <ol>
    <li>Tắt nguồn Buck 2 (L298N) để động cơ không bị hãm điện từ. Giữ nguồn ESP32 (cáp USB).</li>
    <li>Đặt xe sao cho <strong>TÂM TRỤC 2 BÁNH XE</strong> nằm ngay vạch <span class="code-pill">0.00m</span> trên thước dây.</li>
    <li>Bấm nút <span class="code-pill">🔄 RESET SỐ XUNG VỀ 0</span> bên dưới.</li>
    <li>Dùng tay đẩy xe <strong>chậm và đều tay</strong> theo đường thẳng cho tới khi <strong>Tâm trục 2 bánh</strong> chạm đúng vạch <span class="code-pill">1.00m</span> (hoặc vạch tùy chọn).</li>
    <li>Bấm nút <span class="code-pill">💾 LƯU LẦN ĐO NÀY</span> để lưu vào bảng. Đẩy lặp lại 2-3 lần để lấy kết quả trung bình chuẩn nhất!</li>
  </ol>
</div>

<!-- 3 Ô SỐ XUNG KHỔ LỚN -->
<div class="hero-grid">
  <div class="counter-box">
    <div class="counter-title">Xung Bánh Trái (Left Ticks)</div>
    <div class="counter-num" id="ticksL" style="color:#38bdf8;">0</div>
    <div class="counter-sub">Ngắt kênh A + B (Quad 4x)</div>
  </div>

  <div class="counter-box main">
    <div class="counter-title">⚡ TRUNG BÌNH 2 BÁNH (TICKS AVERAGE)</div>
    <div class="counter-num" id="ticksAvg" style="color:#10b981;">0.0</div>
    <div class="counter-sub" id="deltaSub">Độ lệch (L - R): 0 xung (0.0%)</div>
  </div>

  <div class="counter-box">
    <div class="counter-title">Xung Bánh Phải (Right Ticks)</div>
    <div class="counter-num" id="ticksR" style="color:#a855f7;">0</div>
    <div class="counter-sub">Ngắt kênh A + B (Quad 4x)</div>
  </div>
</div>

<!-- BẢNG TÍNH TOÁN KẾT QUẢ NỘI SUY TỨC THỜI -->
<div class="card">
  <h2>
    <span>🧮 Kết Quả Nội Suy & Thông Số Cơ Khí Tương Ứng</span>
    <span id="statusToast" style="font-size:12px; font-weight:normal; color:var(--text-dim);">Chờ đo...</span>
  </h2>
  
  <div class="calc-grid">
    <div class="calc-box">
      <div class="calc-label">Hệ số Xung Chuẩn (Ticks / Mét)</div>
      <div class="calc-val" id="resTicksPerM">0.0</div>
      <div class="calc-hint">Dùng cho code: <span class="code-pill">ticksPerMeter</span></div>
    </div>
    
    <div class="calc-box">
      <div class="calc-label">Tỷ lệ Xung (Ticks / cm)</div>
      <div class="calc-val" id="resTicksPerCm">0.00</div>
      <div class="calc-hint">Số xung trên mỗi centimet</div>
    </div>

    <div class="calc-box">
      <div class="calc-label">Đường kính lốp suy ra (mm)</div>
      <div class="calc-val" id="resWheelDiam">0.0 mm</div>
      <div class="calc-hint">Lốp 65mm có tải (Ratio ~57.2:1)</div>
    </div>

    <div class="calc-box">
      <div class="calc-label">Độ Cân Bằng 2 Bánh</div>
      <div class="calc-val" id="resBalance">100%</div>
      <div class="calc-hint" id="resBalanceHint">Tuyệt đối cân bằng</div>
    </div>
  </div>

  <!-- Thanh công cụ điều khiển -->
  <div class="control-row">
    <div style="display:flex; align-items:center; gap:8px;">
      <label style="font-size:13px; color:var(--text-dim);">Khoảng cách đẩy thực tế:</label>
      <input type="number" id="inpDist" value="1.000" step="0.05" min="0.1" max="10.0" style="width:100px; font-weight:bold; font-family:monospace;" onchange="recalculate()">
      <span style="font-size:13px;">mét</span>
    </div>

    <div style="display:flex; align-items:center; gap:8px; margin-left:12px;">
      <label style="font-size:13px; color:var(--text-dim);">Tải trọng:</label>
      <select id="inpNote">
        <option value="Không tải (0kg)">Không tải (0kg)</option>
        <option value="Có tải 1.0kg">Có tải 1.0kg</option>
        <option value="Có tải 2.0kg">Có tải 2.0kg</option>
        <option value="Có tải 3.0kg">Có tải 3.0kg</option>
        <option value="Chở hàng đầy (Full load)">Chở hàng đầy (Full load)</option>
      </select>
    </div>

    <div style="margin-left:auto; display:flex; gap:8px;">
      <button class="btn-danger" onclick="sendReset()"><span style="font-size:16px;">🔄</span> RESET VỀ 0</button>
      <button class="btn-success" onclick="saveSample()"><span style="font-size:16px;">💾</span> LƯU LẦN ĐO NÀY</button>
    </div>
  </div>
</div>

<!-- BẢNG LỊCH SỬ CÁC LẦN ĐO MẪU -->
<div class="card">
  <h2>
    <span>📑 Bảng Lịch Sử Đo Mẫu & Giá Trị Trung Bình Chuẩn</span>
    <div style="display:flex; gap:6px;">
      <button class="btn-primary" style="font-size:12px; padding:6px 12px;" onclick="exportCsv()">📥 Xuất File CSV</button>
      <button class="btn-secondary" style="font-size:12px; padding:6px 12px;" onclick="clearSamples()">🗑️ Xóa Lịch Sử</button>
    </div>
  </h2>

  <div style="overflow-x:auto;">
    <table id="sampleTable">
      <thead>
        <tr>
          <th>Lần đo</th>
          <th>Giờ</th>
          <th>Khoảng cách</th>
          <th>Xung Trái (L)</th>
          <th>Xung Phải (R)</th>
          <th>Trung bình (Avg)</th>
          <th>Hệ số (Ticks/m)</th>
          <th>Đ.Kính Lốp</th>
          <th>Lệch L/R</th>
          <th>Tải trọng / Ghi chú</th>
        </tr>
      </thead>
      <tbody id="sampleTbody">
        <tr><td colspan="10" style="text-align:center; color:var(--text-dim); padding:16px;">Chưa có lần đo nào được lưu. Hãy đẩy xe 1m và bấm "Lưu Lần Đo Này"!</td></tr>
      </tbody>
    </table>
  </div>
</div>

<!-- KHUNG COPY THÔNG SỐ VÀO CODE -->
<div class="card" style="background:#0c1424; border-color:var(--accent);">
  <h2 style="color:var(--accent);">📋 THÔNG SỐ COPY DÁN THẲNG VÀO CODE CỦA ESP32</h2>
  <div style="background:#060a12; padding:12px; border-radius:6px; border:1px solid #1e293b; font-family:'Consolas', monospace; font-size:13px; color:#38bdf8;">
    <span style="color:#64748b;">// Hệ số chuẩn thực nghiệm sau khi đẩy xe (Tâm 2 bánh):</span><br>
    <span style="color:#f59e0b;">float</span> ticksPerMeter = <span id="codeTicksVal" style="color:#10b981; font-weight:bold;">12325.0f</span>; <span style="color:#64748b;">// Cập nhật từ kết quả đo</span>
  </div>
</div>

<script>
  let liveTicksL = 0;
  let liveTicksR = 0;
  let liveTicksAvg = 0;

  function recalculate() {
    const dist = parseFloat(document.getElementById('inpDist').value) || 1.0;
    const ticksM = dist > 0 ? (liveTicksAvg / dist) : 0;
    const ticksCm = ticksM / 100.0;
    
    document.getElementById('resTicksPerM').innerText = ticksM.toFixed(1);
    document.getElementById('resTicksPerCm').innerText = ticksCm.toFixed(2);
    document.getElementById('codeTicksVal').innerText = ticksM.toFixed(1) + 'f';

    // Suy ra đường kính bánh: ticks/rev = ticksM * (PI * D).
    // Với JGA25-370 170RPM (44 xung/vòng motor * 57.2 ratio ~ 2516.8 xung/vòng bánh):
    // D = 2516.8 / (PI * ticksM) * 1000 (mm)
    if (ticksM > 1000) {
      const diamMm = (2516.8 / (Math.PI * ticksM)) * 1000.0;
      document.getElementById('resWheelDiam').innerText = diamMm.toFixed(1) + ' mm';
    } else {
      document.getElementById('resWheelDiam').innerText = '-- mm';
    }

    // Độ cân bằng
    const maxT = Math.max(Math.abs(liveTicksL), Math.abs(liveTicksR));
    const delta = Math.abs(liveTicksL - liveTicksR);
    if (maxT > 50) {
      const diffPct = (delta / maxT) * 100.0;
      const balPct = Math.max(0, 100.0 - diffPct);
      document.getElementById('resBalance').innerText = balPct.toFixed(1) + '%';
      if (diffPct < 1.5) {
        document.getElementById('resBalance').style.color = 'var(--success)';
        document.getElementById('resBalanceHint').innerText = 'Rất tốt (Lệch ' + diffPct.toFixed(1) + '%)';
      } else if (diffPct < 4.0) {
        document.getElementById('resBalance').style.color = 'var(--warning)';
        document.getElementById('resBalanceHint').innerText = 'Chấp nhận được (Lệch ' + diffPct.toFixed(1) + '%)';
      } else {
        document.getElementById('resBalance').style.color = 'var(--danger)';
        document.getElementById('resBalanceHint').innerText = 'Cần kiểm tra (Lệch ' + diffPct.toFixed(1) + '%)';
      }
      document.getElementById('deltaSub').innerText = 'Độ lệch (L - R): ' + (liveTicksL - liveTicksR) + ' xung (' + diffPct.toFixed(1) + '%)';
    } else {
      document.getElementById('resBalance').innerText = '100%';
      document.getElementById('resBalance').style.color = 'var(--success)';
      document.getElementById('resBalanceHint').innerText = 'Chờ lăn bánh...';
      document.getElementById('deltaSub').innerText = 'Độ lệch (L - R): 0 xung';
    }
  }

  async function pollState() {
    try {
      const t0 = performance.now();
      const res = await fetch('/api/state');
      const data = await res.json();
      const st = data.state;
      const samples = data.samples;

      // Kết nối
      const badge = document.getElementById('connBadge');
      if (st.connected) {
        badge.className = 'badge online';
        badge.innerText = '● ' + st.last_seen_str;
      } else {
        badge.className = 'badge offline';
        badge.innerText = '● ' + st.last_seen_str;
      }
      document.getElementById('connPing').innerText = Math.round(performance.now() - t0) + ' ms';

      // Số xung
      liveTicksL = st.ticks_l;
      liveTicksR = st.ticks_r;
      liveTicksAvg = st.ticks_avg;

      document.getElementById('ticksL').innerText = (liveTicksL >= 0 ? '+' : '') + liveTicksL;
      document.getElementById('ticksR').innerText = (liveTicksR >= 0 ? '+' : '') + liveTicksR;
      document.getElementById('ticksAvg').innerText = (liveTicksAvg >= 0 ? '+' : '') + liveTicksAvg.toFixed(1);

      recalculate();
      renderSamplesTable(samples);
    } catch (e) {}
  }

  function renderSamplesTable(samples) {
    const tbody = document.getElementById('sampleTbody');
    if (!samples || samples.length === 0) {
      tbody.innerHTML = '<tr><td colspan="10" style="text-align:center; color:var(--text-dim); padding:16px;">Chưa có lần đo nào được lưu. Hãy đẩy xe 1m và bấm "Lưu Lần Đo Này"!</td></tr>';
      return;
    }

    let html = '';
    let sumTicksPerM = 0;
    let sumDiam = 0;
    let sumDelta = 0;

    samples.forEach(s => {
      sumTicksPerM += s.ticks_per_m;
      sumDiam += s.wheel_diam_mm;
      sumDelta += s.delta_pct;
      html += `<tr>
        <td>#${s.id}</td>
        <td>${s.time}</td>
        <td>${s.distance_m.toFixed(3)} m</td>
        <td>${s.ticks_l}</td>
        <td>${s.ticks_r}</td>
        <td>${s.ticks_avg.toFixed(1)}</td>
        <td style="color:#10b981; font-weight:bold;">${s.ticks_per_m.toFixed(1)}</td>
        <td>${s.wheel_diam_mm.toFixed(1)} mm</td>
        <td>${s.delta_pct.toFixed(1)}%</td>
        <td>${s.note}</td>
      </tr>`;
    });

    const avgTicksPerM = (sumTicksPerM / samples.length);
    const avgDiam = (sumDiam / samples.length);
    const avgDelta = (sumDelta / samples.length);

    html += `<tr class="avg-row">
      <td colspan="2">⭐ TRUNG BÌNH (${samples.length} lần đo)</td>
      <td>1.000 m</td>
      <td colspan="3" style="text-align:center;">Dữ liệu đã loại trừ 100% trượt bánh</td>
      <td style="font-size:16px; font-weight:bold; color:#10b981;">${avgTicksPerM.toFixed(1)}</td>
      <td>${avgDiam.toFixed(1)} mm</td>
      <td>${avgDelta.toFixed(1)}%</td>
      <td>KẾT QUẢ VÀNG ĐỂ NẠP CODE</td>
    </tr>`;

    tbody.innerHTML = html;
    document.getElementById('codeTicksVal').innerText = avgTicksPerM.toFixed(1) + 'f';
  }

  async function sendReset() {
    document.getElementById('statusToast').innerText = '⏳ Đang phát lệnh Reset Ticks...';
    try {
      await fetch('/api/command', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: 'reset'})
      });
      document.getElementById('statusToast').innerText = '✅ Đã Reset số xung về 0!';
    } catch(e) {
      document.getElementById('statusToast').innerText = '❌ Lỗi: ' + e;
    }
  }

  async function saveSample() {
    const dist = parseFloat(document.getElementById('inpDist').value) || 1.0;
    const ticksM = dist > 0 ? (liveTicksAvg / dist) : 0;
    const maxT = Math.max(Math.abs(liveTicksL), Math.abs(liveTicksR));
    const delta = Math.abs(liveTicksL - liveTicksR);
    const diffPct = maxT > 0 ? (delta / maxT) * 100.0 : 0.0;
    const diamMm = ticksM > 1000 ? (2516.8 / (Math.PI * ticksM)) * 1000.0 : 65.0;
    const note = document.getElementById('inpNote').value;

    if (liveTicksAvg < 100) {
      alert("Số xung hiện tại quá nhỏ (< 100 xung). Hãy đẩy xe ít nhất 50cm - 1m trước khi lưu!");
      return;
    }

    try {
      await fetch('/api/save_sample', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
          distance_m: dist,
          ticks_l: liveTicksL,
          ticks_r: liveTicksR,
          ticks_avg: liveTicksAvg,
          ticks_per_m: ticksM,
          wheel_diam_mm: diamMm,
          delta_pct: diffPct,
          note: note
        })
      });
      document.getElementById('statusToast').innerText = '✅ Đã lưu mẫu đo thành công!';
    } catch(e) {
      alert("Lỗi lưu mẫu: " + e);
    }
  }

  async function clearSamples() {
    if (!confirm("Bạn có chắc chắn muốn xóa toàn bộ lịch sử đo mẫu?")) return;
    await fetch('/api/clear_samples', {method: 'POST'});
  }

  function exportCsv() {
    window.location.href = '/api/export_csv';
  }

  setInterval(pollState, 150);
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
    print("    AGV_VER2 - SERVER CHUYÊN DỤNG CALIB ENCODER 1 MÉT (ZERO-SLIP)")
    print("=" * 75)
    ips = get_local_ips()
    print(f">> Server đang chạy tại CỔNG RIÊNG {PORT}:")
    print(f"   [Cục bộ]  : http://localhost:{PORT}")
    for ip in ips:
        print(f"   [Mạng Lan]: http://{ip}:{PORT}")
    print("=" * 75)
    print(">> Đang chờ kết nối từ ESP32...")

    server = ThreadingHTTPServer((HOST, PORT), EncoderCalibHttpHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n>> Đã tắt Server Calib.")

if __name__ == "__main__":
    main()
