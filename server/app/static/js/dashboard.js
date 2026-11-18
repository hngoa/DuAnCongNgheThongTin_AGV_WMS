/**
 * ===================================================================================
 * AGV_VER2 WMS DASHBOARD & REAL-TIME DIGITAL TWIN (V2.1)
 * INTEGRATED WITH STITCH INDUSTRIAL UI & TAILWIND DESIGN SYSTEM
 * ===================================================================================
 */

// Mã khóa bảo mật quản trị viên (Admin Security Token)
const ADMIN_TOKEN = "agv_sec_admin_v2_1a2b3c";

// Tự động đính kèm X-Device-Token cho mọi yêu cầu fetch API tới server
const _origFetch = window.fetch;
window.fetch = function(url, options = {}) {
  options.headers = options.headers || {};
  if (typeof options.headers.set === 'function') {
    options.headers.set('X-Device-Token', ADMIN_TOKEN);
  } else if (Array.isArray(options.headers)) {
    options.headers.push(['X-Device-Token', ADMIN_TOKEN]);
  } else {
    options.headers['X-Device-Token'] = ADMIN_TOKEN;
  }
  return _origFetch(url, options);
};

let mapConfig = null;
let mapNodes = [];
let mapSegments = [];
let currentState = null;
let allMapsList = [];

const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");

// ===================================================================================
// 1. KHỞI TẠO VÀ RESIZE CANVAS CHUẨN ĐỘ PHÂN GIẢI CAO (HiDPI)
// ===================================================================================
function resizeCanvas() {
  if (!canvas) return;
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  ctx.resetTransform ? ctx.resetTransform() : ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.scale(dpr, dpr);
  drawMap();
}

// ===================================================================================
// 2. NẠP CẤU HÌNH BẢN ĐỒ LINE TÔ-PÔ TỪ SERVER
// ===================================================================================
async function loadMapData() {
  try {
    const res = await fetch("/api/wms/map");
    const data = await res.json();
    if (data.status === "success") {
      mapConfig = data.map;
      mapNodes = data.nodes;
      mapSegments = data.segments;
      
      const titleEl = document.getElementById("mapTitle");
      if (titleEl) {
        titleEl.innerText = `Bản Đồ Mạng Lưới: ${mapConfig.name}`;
      }
      const badgeEl = document.getElementById("lblActiveMapBadge");
      if (badgeEl) {
        badgeEl.innerText = mapConfig.map_id;
      }
      const hudX = document.getElementById("lblHudX");
      const hudY = document.getElementById("lblHudY");
      if (hudX) hudX.innerText = `X: 0.00m → ${mapConfig.width_m.toFixed(2)}m`;
      if (hudY) hudY.innerText = `Y: 0.00m → ${mapConfig.length_m.toFixed(2)}m`;

      renderDynamicMapNodesUI(mapNodes);
      resizeCanvas();
    }
  } catch (e) {
    console.error("Lỗi nạp bản đồ:", e);
  }
}

function renderDynamicMapNodesUI(nodes) {
  if (!Array.isArray(nodes) || !nodes.length) return;
  const grid = document.getElementById("quickDispatchGrid");
  if (grid) {
    const palette = [
      "bg-slate-50 hover:bg-slate-100 border-slate-200 text-slate-700",
      "bg-blue-50 hover:bg-blue-100 border-blue-200 text-primary",
      "bg-purple-50 hover:bg-purple-100 border-purple-200 text-purple-700",
      "bg-amber-50 hover:bg-amber-100 border-amber-200 text-amber-700",
      "bg-emerald-50 hover:bg-emerald-100 border-emerald-200 text-emerald-700",
      "bg-rose-50 hover:bg-rose-100 border-rose-200 text-rose-700"
    ];
    grid.className = nodes.length <= 4 ? "grid grid-cols-4 gap-2 mb-3" : "grid grid-cols-3 sm:grid-cols-4 gap-2 mb-3";
    grid.innerHTML = nodes.map((n, idx) => {
      const nid = String(n.node_id || "").toUpperCase();
      const label = (n.node_type === "INBOUND" || nid === "A") ? `Đến Trạm ${nid}` : `Đến Kho ${nid}`;
      const cls = palette[idx % palette.length];
      return `<button onclick="sendGoTo('${nid}')" class="py-1.5 px-2 rounded-md border text-center transition-colors font-sans text-xs font-semibold ${cls}" type="button">${label}</button>`;
    }).join("");
  }

  const targetNodes = nodes.filter(n => n.node_type !== "INBOUND");
  const listToShow = targetNodes.length ? targetNodes : nodes;
  ["selProdTargetNode", "selRuleTargetNode"].forEach(selId => {
    const selTarget = document.getElementById(selId);
    if (!selTarget) return;
    const prevVal = selTarget.value;
    selTarget.innerHTML = listToShow.map(n => {
      const nid = String(n.node_id || "").toUpperCase();
      return `<option value="${nid}">Kho ${nid} (${n.name || ("Trạm " + nid)})</option>`;
    }).join("");
    if ([...selTarget.options].some(o => o.value === prevVal)) {
      selTarget.value = prevVal;
    }
  });
}

// ===================================================================================
// 3. VẼ BẢN ĐỒ CANVAS 2D CHUẨN THỰC THỂ DIGITAL TWIN
// ===================================================================================
function drawMap() {
  if (!canvas || !mapConfig || !mapNodes.length) return;
  const rect = canvas.getBoundingClientRect();
  const w = rect.width;
  const h = rect.height;
  ctx.clearRect(0, 0, w, h);

  const pad = 42;
  const mapW = mapConfig.width_m || 1.54;
  const mapH = mapConfig.length_m || 1.51;
  const scaleX = (w - pad * 2) / mapW;
  const scaleY = (h - pad * 2) / mapH;
  const scale = Math.min(scaleX, scaleY);

  const offsetX = pad + (w - pad * 2 - mapW * scale) / 2;
  const offsetY = pad + (h - pad * 2 - mapH * scale) / 2;

  // Đổi từ hệ tọa độ thế giới (X, Y) sang hệ tọa độ màn hình (Gốc dưới trái)
  function toCanvas(x, y) {
    return {
      cx: offsetX + x * scale,
      cy: (h - offsetY) - y * scale
    };
  }

  // (A) VẼ CÁC ĐOẠN ĐƯỜNG LINE (TRACK SEGMENTS)
  const lockedSegments = [];
  mapSegments.forEach(seg => {
    const nFrom = mapNodes.find(n => n.node_id === seg.from_node);
    const nTo = mapNodes.find(n => n.node_id === seg.to_node);
    if (!nFrom || !nTo) return;

    const p1 = toCanvas(nFrom.x, nFrom.y);
    const p2 = toCanvas(nTo.x, nTo.y);

    ctx.beginPath();
    ctx.moveTo(p1.cx, p1.cy);
    ctx.lineTo(p2.cx, p2.cy);

    if (seg.is_locked) {
      lockedSegments.push(seg);
      // Vùng highlight cảnh báo an toàn
      ctx.lineWidth = 10;
      ctx.strokeStyle = "rgba(245, 158, 11, 0.18)";
      ctx.stroke();

      // Vạch kẻ đứt nét màu cam cảnh báo khóa đường
      ctx.beginPath();
      ctx.moveTo(p1.cx, p1.cy);
      ctx.lineTo(p2.cx, p2.cy);
      ctx.lineWidth = 4;
      ctx.strokeStyle = "#f59e0b";
      ctx.setLineDash([7, 4]);
      ctx.stroke();
      ctx.setLineDash([]);
    } else {
      // Vạch đường line chuẩn công nghiệp
      ctx.lineWidth = 8;
      ctx.strokeStyle = "#e2e8f0";
      ctx.stroke();

      ctx.beginPath();
      ctx.moveTo(p1.cx, p1.cy);
      ctx.lineTo(p2.cx, p2.cy);
      ctx.lineWidth = 2.5;
      ctx.strokeStyle = "#94a3b8";
      ctx.stroke();
    }

    // Nhãn kích thước chiều dài mét
    const midX = (p1.cx + p2.cx) / 2;
    const midY = (p1.cy + p2.cy) / 2;
    ctx.fillStyle = seg.is_locked ? "#b45309" : "#64748b";
    ctx.font = "bold 9px 'JetBrains Mono', monospace";
    ctx.fillText(`${seg.distance_m}m`, midX - 12, midY - 6);
  });

  // Cập nhật thẻ hiển thị khóa an toàn giao thông
  const lockBadge = document.getElementById("trafficLockBadge");
  const lockText = document.getElementById("trafficLockText");
  if (lockBadge && lockText) {
    if (lockedSegments.length > 0) {
      const segLabels = lockedSegments.map(s => `${s.from_node}→${s.to_node}`).join(", ");
      const owner = lockedSegments[0].locked_by_agv || "AGV_Ver2";
      lockBadge.classList.remove("hidden");
      lockText.innerText = lockedSegments.length === 1
        ? `ĐOẠN ${segLabels} ĐANG KHÓA (${owner} Chiếm Dụng)`
        : `ĐANG KHÓA ${lockedSegments.length} ĐOẠN: ${segLabels} (${owner})`;
    } else {
      lockBadge.classList.add("hidden");
    }
  }

  // (B) VẼ VỆT QUỸ ĐẠO DI CHUYỂN CỦA XE (MOTION TRAIL)
  if (currentState && currentState.trail && currentState.trail.length > 1) {
    ctx.beginPath();
    const startP = toCanvas(currentState.trail[0][0], currentState.trail[0][1]);
    ctx.moveTo(startP.cx, startP.cy);
    for (let i = 1; i < currentState.trail.length; i++) {
      const pt = toCanvas(currentState.trail[i][0], currentState.trail[i][1]);
      ctx.lineTo(pt.cx, pt.cy);
    }
    ctx.strokeStyle = "rgba(51, 102, 204, 0.45)";
    ctx.lineWidth = 2.5;
    ctx.setLineDash([4, 2]);
    ctx.stroke();
    ctx.setLineDash([]);

    // Vẽ các chấm sáng định vị gần nhất
    const trailLen = currentState.trail.length;
    for (let j = Math.max(0, trailLen - 6); j < trailLen; j++) {
      const dotP = toCanvas(currentState.trail[j][0], currentState.trail[j][1]);
      ctx.beginPath();
      ctx.arc(dotP.cx, dotP.cy, 2.5, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(51, 102, 204, 0.8)";
      ctx.fill();
    }
  }

  // (C) VẼ CÁC TRẠM DỪNG (STATION WAYPOINTS)
  mapNodes.forEach(node => {
    const p = toCanvas(node.x, node.y);
    const nodeColor = node.color || "#3366cc";

    // Vòng phát sáng ngoại vi
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, 13, 0, Math.PI * 2);
    ctx.fillStyle = nodeColor + "22";
    ctx.fill();

    // Vòng tròn trạm
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, 8, 0, Math.PI * 2);
    ctx.fillStyle = "#ffffff";
    ctx.fill();
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = nodeColor;
    ctx.stroke();

    // Điểm tâm trạm
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, 3.5, 0, Math.PI * 2);
    ctx.fillStyle = nodeColor;
    ctx.fill();

    // Nhãn Tên Trạm & Tọa độ
    ctx.fillStyle = "#1e293b";
    ctx.font = "bold 10px 'JetBrains Mono', monospace";
    const labelY = (node.y > mapH / 2) ? p.cy - 12 : p.cy + 18;
    const namePart = node.name.split(" ")[0];
    ctx.fillText(`${node.node_id}: ${namePart} (${node.x.toFixed(2)}, ${node.y.toFixed(2)})`, p.cx - 25, labelY);
  });

  // (D) VẼ THỰC THỂ ROBOT AGV THỜI GIAN THỰC (ROBOT VEHICLE SPRITE)
  if (currentState) {
    const rx = currentState.odom_x || 0.0;
    const ry = currentState.odom_y || 0.0;
    const yawDeg = currentState.yaw_deg || 90.0;
    const yawRad = yawDeg * Math.PI / 180.0;
    const rp = toCanvas(rx, ry);
    const isConn = currentState.connected;

    ctx.save();
    ctx.translate(rp.cx, rp.cy);
    ctx.rotate(-yawRad + Math.PI / 2); // Khớp góc định hướng với trục Canvas

    if (isConn) {
      // 1. Thân xe chính (Trắng viền xanh, bóng đổ nhẹ)
      ctx.fillStyle = "#ffffff";
      ctx.strokeStyle = "#3366cc";
      ctx.lineWidth = 2;
      roundRect(ctx, -14, -18, 28, 36, 4);
      ctx.fill();
      ctx.stroke();

      // 2. Bốn bánh xe cơ cấu kéo vi sai
      ctx.fillStyle = "#475569";
      ctx.fillRect(-16, -13, 2.5, 8);
      ctx.fillRect(13.5, -13, 2.5, 8);
      ctx.fillRect(-16, 5, 2.5, 8);
      ctx.fillRect(13.5, 5, 2.5, 8);

      // 3. Mũi tên định hướng tiến (Forward Direction)
      ctx.beginPath();
      ctx.moveTo(0, -13);
      ctx.lineTo(-5, -4);
      ctx.lineTo(5, -4);
      ctx.closePath();
      ctx.fillStyle = "#3366cc";
      ctx.fill();

      // 4. Đèn tín hiệu tải trọng trung tâm
      ctx.beginPath();
      ctx.arc(0, 4, 3.5, 0, Math.PI * 2);
      ctx.fillStyle = "#10b981";
      ctx.fill();

      // 5. Thẻ tên AGV_VER2
      ctx.fillStyle = "#3366cc";
      roundRect(ctx, -24, -28, 48, 11, 2);
      ctx.fill();
      ctx.fillStyle = "#ffffff";
      ctx.font = "bold 7px 'JetBrains Mono', monospace";
      ctx.textAlign = "center";
      ctx.fillText("AGV_VER2", 0, -20);
    } else {
      // Trạng thái OFFLINE: Render mờ fail-safe
      ctx.fillStyle = "rgba(241, 245, 249, 0.75)";
      ctx.strokeStyle = "#94a3b8";
      ctx.lineWidth = 2;
      ctx.setLineDash([3, 3]);
      roundRect(ctx, -14, -18, 28, 36, 4);
      ctx.fill();
      ctx.stroke();
      ctx.setLineDash([]);

      // Bánh xe mờ
      ctx.fillStyle = "#94a3b8";
      ctx.fillRect(-16, -13, 2.5, 8);
      ctx.fillRect(13.5, -13, 2.5, 8);
      ctx.fillRect(-16, 5, 2.5, 8);
      ctx.fillRect(13.5, 5, 2.5, 8);

      // Đèn cảnh báo đỏ offline
      ctx.beginPath();
      ctx.arc(0, 4, 3, 0, Math.PI * 2);
      ctx.fillStyle = "#dc2626";
      ctx.fill();

      // Thẻ tên OFFLINE
      ctx.fillStyle = "#ffffff";
      ctx.strokeStyle = "#cbd5e1";
      ctx.lineWidth = 1;
      roundRect(ctx, -28, -28, 56, 11, 2);
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = "#64748b";
      ctx.font = "bold 7px 'JetBrains Mono', monospace";
      ctx.textAlign = "center";
      ctx.fillText("AGV (OFFLINE)", 0, -20);
    }

    ctx.restore();
  }
}

// Hàm vẽ hình chữ nhật bo tròn góc
function roundRect(ctx, x, y, width, height, radius) {
  ctx.beginPath();
  ctx.moveTo(x + radius, y);
  ctx.lineTo(x + width - radius, y);
  ctx.quadraticCurveTo(x + width, y, x + width, y + radius);
  ctx.lineTo(x + width, y + height - radius);
  ctx.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
  ctx.lineTo(x + radius, y + height);
  ctx.quadraticCurveTo(x, y + height, x, y + height - radius);
  ctx.lineTo(x, y + radius);
  ctx.quadraticCurveTo(x, y, x + radius, y);
  ctx.closePath();
}

// ===================================================================================
// 4. POLLING TELEMETRY THỜI GIAN THỰC (200ms - CÓ KHÓA CHỐNG CHỒNG CHÉO REQUEST)
// ===================================================================================
let isUpdatingState = false;
async function updateState() {
  if (isUpdatingState) return;
  isUpdatingState = true;
  try {
    const res = await fetch("/api/state");
    if (!res.ok) return;
    const data = await res.json();
    currentState = data;

    const connBadge = document.getElementById("connBadge");
    const valStateBadge = document.getElementById("valStateBadge");
    const valState = document.getElementById("valState");
    const offlineBanner = document.getElementById("offlineAlertBanner");

    if (data.connected) {
      // 1. TRẠNG THÁI ONLINE
      if (offlineBanner) offlineBanner.style.display = "none";

      if (connBadge) {
        connBadge.className = "flex items-center gap-1.5 px-2.5 py-1 rounded bg-slate-50 border border-slate-200";
        connBadge.innerHTML = `<span class="w-2 h-2 rounded-full bg-emerald-500 animate-pulse"></span>
                               <span class="font-mono text-xs text-slate-700 tracking-tight">${data.last_seen_str}</span>`;
      }

      if (valStateBadge && valState) {
        valStateBadge.className = "flex items-center px-2.5 py-1 rounded bg-blue-50 border border-blue-200";
        valState.className = "font-mono text-xs text-primary font-semibold tracking-wide uppercase";
        valState.innerText = `TRẠNG THÁI: ${data.robot_state}`;
      }

      // Động học
      setText("valOdomX", data.odom_x.toFixed(2));
      setText("valOdomY", data.odom_y.toFixed(2));
      setText("valYaw", data.yaw_deg.toFixed(1) + "°");
      setText("valSpeed", data.speed_cm_s.toFixed(1));
      
      const yaw = data.yaw_deg;
      let yawDir = "Bắc (North)";
      if (yaw >= 45 && yaw < 135) yawDir = "Bắc (North)";
      else if (yaw >= 135 && yaw < 225) yawDir = "Tây (West)";
      else if (yaw >= 225 && yaw < 315) yawDir = "Nam (South)";
      else yawDir = "Đông (East)";
      setText("valYawDir", yawDir);

      // Cảm biến Line IR
      updateIrSensors(data.line_mask);
      setText("valLineErr", (data.line_error >= 0 ? "+" : "") + data.line_error.toFixed(1));
      setText("valPidStatus", Math.abs(data.line_error) <= 0.3 ? "PID: Ổn định" : "PID: Đang bù góc");

      // Năng lượng INA226
      setText("valVolt", data.voltage.toFixed(2));
      setText("valCurr", data.current.toFixed(2));
      setText("valPower", data.power.toFixed(2));
      setText("valEnergy", data.energy_mah.toFixed(1));
      setText("valBatteryPct", data.battery_pct.toFixed(0) + "%");
      setText("valVoltHeader", `(${data.voltage.toFixed(2)}V)`);

      setText("valInaStatusBadge", data.ina_ok ? "I2C: 0x40 (INA226 OK)" : "I2C: 0x40 (Chờ cảm biến)");
      setText("valBatteryText", `Sức khỏe: 98% • ${data.battery_pct.toFixed(0)}% Còn lại`);
      
      const dotEl = document.getElementById("valBatteryDot");
      if (dotEl) dotEl.className = "w-2 h-2 rounded-full bg-emerald-500";

      updateBatteryTicks(data.battery_pct);

    } else {
      // 2. TRẠNG THÁI OFFLINE (KHÔNG TẠO DATA ẢO)
      if (offlineBanner) offlineBanner.style.display = "flex";

      if (connBadge) {
        connBadge.className = "flex items-center gap-1.5 px-2.5 py-1 rounded bg-red-50 border border-red-200";
        connBadge.innerHTML = `<span class="w-2 h-2 rounded-full bg-red-500 animate-ping"></span>
                               <span class="font-mono text-xs text-red-600 font-medium tracking-tight">${data.last_seen_str || "Chưa kết nối"}</span>`;
      }

      if (valStateBadge && valState) {
        valStateBadge.className = "flex items-center px-2.5 py-1 rounded bg-slate-100 border border-slate-200";
        valState.className = "font-mono text-xs text-slate-500 font-semibold tracking-wide uppercase";
        valState.innerText = "TRẠNG THÁI: OFFLINE";
      }

      setText("valOdomX", "--");
      setText("valOdomY", "--");
      setText("valYaw", "--");
      setText("valYawDir", "Chờ tín hiệu");
      setText("valSpeed", "--");

      updateIrSensors(0);
      setText("valLineErr", "--");
      setText("valPidStatus", "PID: Chờ");

      setText("valVolt", "--");
      setText("valCurr", "--");
      setText("valPower", "--");
      setText("valEnergy", "--");
      setText("valBatteryPct", "--%");
      setText("valVoltHeader", "(-- V)");

      setText("valInaStatusBadge", "I2C: 0x40 (Chờ kết nối)");
      setText("valBatteryText", "Chưa kết nối nguồn pin robot");

      const dotEl = document.getElementById("valBatteryDot");
      if (dotEl) dotEl.className = "w-2 h-2 rounded-full bg-slate-300";

      updateBatteryTicks(0);
    }

    // Cập nhật Thị giác Camera & AI
    updateCameraVision(data.camera);

    // Vẽ lại canvas
    drawMap();

  } catch (e) {
    // Bỏ qua lỗi kết nối khi browser chuyển tab/abort request
  } finally {
    isUpdatingState = false;
  }
}

// Cập nhật hiển thị 5 bóng cảm biến line IR (L2, L1, C, R1, R2)
function updateIrSensors(mask) {
  const container = document.getElementById("irDotsContainer");
  const maskStr = document.getElementById("valLineMaskStr");
  if (!container) return;

  const bits = [];
  for (let i = 4; i >= 0; i--) {
    bits.push((mask >> i) & 1);
  }

  let dotsHtml = "";
  bits.forEach(b => {
    if (b === 1) {
      dotsHtml += `<span class="w-2 h-2 rounded-full bg-primary ring-2 ring-primary/30 animate-pulse"></span>`;
    } else {
      dotsHtml += `<span class="w-2 h-2 rounded-full bg-slate-200 border border-slate-300"></span>`;
    }
  });
  container.innerHTML = dotsHtml;
  if (maskStr) maskStr.innerText = `[ ${bits.join(" ")} ]`;
}

// Cập nhật thước đo 10 đoạn pin Li-ion
function updateBatteryTicks(pct) {
  const container = document.getElementById("batteryTicksContainer");
  if (!container) return;

  const activeTicks = Math.round((pct / 100) * 10);
  let ticksHtml = "";
  for (let i = 0; i < 10; i++) {
    if (i < activeTicks) {
      ticksHtml += `<div class="bg-primary rounded-xs"></div>`;
    } else {
      ticksHtml += `<div class="bg-slate-200 rounded-xs"></div>`;
    }
  }
  container.innerHTML = ticksHtml;
}

// Cập nhật khối thị giác PhoneCamera & AI
function updateCameraVision(cam) {
  const onlineDot = document.getElementById("camOnlineDot");
  const camStatus = document.getElementById("camStatus");
  const camImg = document.getElementById("camImg");
  const placeholder = document.getElementById("camPlaceholder");
  const reasonEl = document.getElementById("autoCaptureReason");

  if (reasonEl) {
    const reason = (cam && cam.auto_capture_reason) ? cam.auto_capture_reason : "Chờ camera điện thoại kết nối...";
    reasonEl.innerText = reason;
    const isReady = reason.includes("Đang phát lệnh") || reason.includes("chụp kiểm tra");
    const isBlocked = reason.includes("kẹt") || reason.includes("Task #") || reason.includes("mất") || reason.includes("chưa");
    reasonEl.className = isReady
      ? "text-emerald-700 truncate font-semibold"
      : isBlocked
        ? "text-amber-700 truncate font-semibold"
        : "text-slate-700 truncate font-medium";
  }

  if (cam && cam.online) {
    if (onlineDot) onlineDot.className = "w-1.5 h-1.5 rounded-full bg-emerald-500 animate-ping";
    const recText = (cam.received_at && cam.received_at !== "Chưa nhận ảnh")
      ? `Nhận lúc: ${cam.received_at} @ Wi-Fi`
      : "CameraX đã kết nối (chờ ảnh) @ Wi-Fi";
    if (camStatus) camStatus.innerText = recText;
    
    setText("aiDetectedCode", cam.product_code || "--");
    setText("aiConfidenceBadge", `AI CONF: ${(Number(cam.confidence || 0) * 100).toFixed(1)}%`);
    
    const disp = cam.dispatch || {};
    setText("aiCategory", disp.category || "ELECTRONIC");
    setText("aiTargetStation", disp.target_station_name || `Trạm ${disp.target_node || "--"}`);
    setText("aiTimestamp", `${cam.received_at} - CameraX Android`);

    if (cam.image_url) {
      if (camImg) {
        camImg.src = cam.image_url;
        camImg.classList.remove("hidden");
      }
      if (placeholder) placeholder.classList.add("hidden");
    }
  } else {
    if (onlineDot) onlineDot.className = "w-1.5 h-1.5 rounded-full bg-slate-400";
    if (camStatus) camStatus.innerText = "Chờ camera điện thoại kết nối...";
    
    setText("aiDetectedCode", "--");
    setText("aiConfidenceBadge", "AI CONF: --");
    setText("aiCategory", "--");
    setText("aiTargetStation", "--");
    setText("aiTimestamp", "--");

    if (camImg) camImg.classList.add("hidden");
    if (placeholder) placeholder.classList.remove("hidden");
  }
}

function setText(id, text) {
  const el = document.getElementById(id);
  if (el) el.innerText = text;
}

// ===================================================================================
// 5. POLLING TỒN KHO VÀ NHIỆM VỤ WMS (1s - CÓ KHÓA CHỐNG CHỒNG CHÉO REQUEST)
// ===================================================================================
let isUpdatingWms = false;
async function updateWmsData() {
  if (isUpdatingWms) return;
  isUpdatingWms = true;
  try {
    // (0) Đồng bộ trạng thái bản đồ & khóa đường line (Segments Lock) thời gian thực
    const resMap = await fetch("/api/wms/map");
    const dataMap = await resMap.json();
    if (dataMap.status === "success" && dataMap.segments) {
      const mapChanged = dataMap.map && (!mapConfig || mapConfig.map_id !== dataMap.map.map_id);
      if (dataMap.map) mapConfig = dataMap.map;
      if (dataMap.nodes) mapNodes = dataMap.nodes;
      mapSegments = dataMap.segments;
      if (mapChanged) {
        setText("mapTitle", `Bản Đồ Mạng Lưới: ${mapConfig.name}`);
        setText("lblActiveMapBadge", mapConfig.map_id);
        setText("lblHudX", `X: 0.00m → ${mapConfig.width_m.toFixed(2)}m`);
        setText("lblHudY", `Y: 0.00m → ${mapConfig.length_m.toFixed(2)}m`);
        renderDynamicMapNodesUI(mapNodes);
      }
      drawMap();
    }

    // (A) Nạp tồn kho theo trạm
    const resInv = await fetch("/api/wms/inventory");
    const dataInv = await resInv.json();
    if (dataInv.status === "success") {
      let invHtml = "";
      for (const [nid, st] of Object.entries(dataInv.inventory)) {
        const cap = st.capacity || { current_stock: 0, max_capacity: 30 };
        const pct = Math.min(100, Math.round((cap.current_stock / cap.max_capacity) * 100));
        const color = st.color || "#3366cc";

        invHtml += `
          <div class="bg-slate-50 border border-slate-200/80 p-2.5 rounded-md flex flex-col gap-1.5">
            <div class="flex items-center justify-between font-mono text-[11px]">
              <span class="text-slate-800 font-bold flex items-center gap-1.5">
                <span class="w-2 h-2 rounded-full" style="background-color: ${color}"></span>
                Trạm ${nid} (${st.node_name})
              </span>
              <span class="font-semibold" style="color: ${color}">
                ${cap.current_stock}/${cap.max_capacity} kiện (${pct}%)
              </span>
            </div>
            <div class="w-full bg-slate-200 h-1.5 rounded-full overflow-hidden">
              <div class="h-full rounded-full transition-all duration-300" style="width: ${pct}%; background-color: ${color}"></div>
            </div>
            <div class="flex flex-wrap items-center gap-1.5 mt-0.5">`;

        if (st.items && st.items.length > 0) {
          st.items.forEach(it => {
            invHtml += `<span class="font-mono text-[10px] bg-white border border-slate-200 px-1.5 py-0.5 rounded font-medium text-slate-700">
              ${it.quantity}x ${it.product_code}
            </span>`;
          });
        } else {
          invHtml += `<span class="font-mono text-[10px] text-slate-400 italic">Kho đang trống</span>`;
        }

        invHtml += `</div></div>`;
      }
      const invContainer = document.getElementById("inventoryContainer");
      if (invContainer) invContainer.innerHTML = invHtml;
    }

    // (B) Nạp danh sách Tasks gần nhất
    const resTasks = await fetch("/api/wms/tasks");
    const dataTasks = await resTasks.json();
    if (dataTasks.status === "success") {
      const taskTbody = document.getElementById("taskTableBody");
      const countBadge = document.getElementById("taskQueueCountBadge");
      if (countBadge) countBadge.innerText = `Queue: ${dataTasks.tasks.length} Task`;

      if (dataTasks.tasks.length > 0) {
        let taskHtml = "";
        dataTasks.tasks.slice(0, 5).forEach(t => {
          let badgeClass = "bg-slate-100 text-slate-600 border border-slate-200";
          let icon = "";
          if (t.status === "COMPLETED") {
            badgeClass = "bg-emerald-50 text-emerald-700 border border-emerald-200";
          } else if (t.status === "IN_TRANSIT") {
            badgeClass = "bg-blue-50 text-primary border border-blue-200 font-bold";
            icon = `<span class="w-1.5 h-1.5 rounded-full bg-primary animate-ping mr-1"></span>`;
          } else if (t.status === "PENDING") {
            badgeClass = "bg-amber-50 text-amber-800 border border-amber-200";
          }

          taskHtml += `
            <tr class="hover:bg-slate-50 transition-colors">
              <td class="py-2 px-2.5 text-primary font-bold">#${t.task_id}</td>
              <td class="py-2 px-2.5 text-slate-800 font-semibold">${t.product_code}</td>
              <td class="py-2 px-2.5 text-slate-600">Trạm ${t.from_node} → ${t.to_node}</td>
              <td class="py-2 px-2.5 text-right">
                <span class="${badgeClass} px-2 py-0.5 rounded text-[10px] inline-flex items-center">
                  ${icon}${t.status}
                </span>
              </td>
            </tr>`;
        });
        if (taskTbody) taskTbody.innerHTML = taskHtml;
      } else {
        if (taskTbody) {
          taskTbody.innerHTML = `<tr><td colspan="4" class="py-4 text-center text-slate-400 font-mono">Chưa có nhiệm vụ vận chuyển</td></tr>`;
        }
      }
    }
  } catch (e) {
    // Bỏ qua lỗi kết nối tạm thời
  } finally {
    isUpdatingWms = false;
  }
}

// ===================================================================================
// 6. PHÁT LỆNH ĐIỀU KHIỂN & ĐIỀU HƯỚNG XE
// ===================================================================================
async function sendCmd(action, payload = {}) {
  const notifyEl = document.getElementById("cmdNotify");
  try {
    if (notifyEl) notifyEl.innerText = `⏳ Đang gửi lệnh '${action}'...`;
    const res = await fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action, ...payload })
    });
    const d = await res.json();
    if (notifyEl) {
      notifyEl.innerText = d.message
        ? `✅ ${d.message}`
        : `📡 Đã phát lệnh #${d.cmd_id}: ${d.cmd}`;
    }
    if (action === "abort_missions" || action === "stop" || action === "reset_odom") {
      updateWmsData();
    }
  } catch (e) {
    if (notifyEl) notifyEl.innerText = `❌ Lỗi phát lệnh: ${e}`;
  }
}

function sendGoTo(st) {
  sendCmd("go_to", { station: st });
}

// Phát lệnh từ xa yêu cầu Android PhoneCamera chụp ảnh khay hàng (Shot 1 hoặc Kiểm tra khay)
async function triggerShot1() {
  const notifyEl = document.getElementById("cmdNotify");
  try {
    if (notifyEl) notifyEl.innerText = "⏳ Đang phát lệnh chụp khay hàng tới điện thoại...";
    const res = await fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: "capture_now", station: "A", mode: "CAPTURE" })
    });
    const d = await res.json();
    if (notifyEl) {
      notifyEl.innerText = d.message || `📡 Đã phát lệnh chụp khay hàng #${d.cam_cmd_id || d.cmd_id}`;
    }
  } catch (e) {
    if (notifyEl) notifyEl.innerText = `❌ Lỗi phát lệnh: ${e}`;
  }
}

function exportCsv() {
  window.location.href = "/api/export_csv";
}

// ===================================================================================
// 7. QUẢN TRỊ CẤU HÌNH WMS (MODAL: ĐỔI BẢN ĐỒ & QUẢN LÝ SẢN PHẨM)
// ===================================================================================
function openConfigModal() {
  const modal = document.getElementById("configModal");
  if (modal) {
    modal.classList.remove("hidden");
    renderDynamicMapNodesUI(mapNodes);
    loadMapsList();
    loadProductsList();
    loadRulesList();
  }
}

function closeConfigModal() {
  const modal = document.getElementById("configModal");
  if (modal) modal.classList.add("hidden");
  if (typeof resetProductForm === "function") resetProductForm();
}

function switchModalTab(tab) {
  const isMap = tab === 'map';
  const mapTab = document.getElementById("modalTabMap");
  const prodTab = document.getElementById("modalTabProduct");
  const tabBtnMap = document.getElementById("tabBtnMap");
  const tabBtnProd = document.getElementById("tabBtnProduct");

  if (isMap) {
    if (mapTab) mapTab.classList.remove("hidden");
    if (prodTab) prodTab.classList.add("hidden");
    if (tabBtnMap) {
      tabBtnMap.className = "flex-1 py-2.5 px-3 border-b-2 border-primary text-primary font-semibold bg-white flex items-center justify-center gap-1.5 transition-colors shadow-[0_1px_0_0_#ffffff]";
    }
    if (tabBtnProd) {
      tabBtnProd.className = "flex-1 py-2.5 px-3 border-b-2 border-transparent text-slate-500 hover:text-slate-800 hover:bg-white/60 flex items-center justify-center gap-1.5 transition-colors";
    }
  } else {
    if (mapTab) mapTab.classList.add("hidden");
    if (prodTab) prodTab.classList.remove("hidden");
    if (tabBtnProd) {
      tabBtnProd.className = "flex-1 py-2.5 px-3 border-b-2 border-primary text-primary font-semibold bg-white flex items-center justify-center gap-1.5 transition-colors shadow-[0_1px_0_0_#ffffff]";
    }
    if (tabBtnMap) {
      tabBtnMap.className = "flex-1 py-2.5 px-3 border-b-2 border-transparent text-slate-500 hover:text-slate-800 hover:bg-white/60 flex items-center justify-center gap-1.5 transition-colors";
    }
    renderDynamicMapNodesUI(mapNodes);
    loadProductsList();
    loadRulesList();
  }
}

// (A) Nạp danh sách bản đồ
async function loadMapsList() {
  try {
    const res = await fetch("/api/wms/maps");
    const data = await res.json();
    if (data.status === "success") {
      allMapsList = data.maps;
      const sel = document.getElementById("selMapList");
      if (sel) {
        sel.innerHTML = "";
        allMapsList.forEach(m => {
          const opt = document.createElement("option");
          opt.value = m.map_id;
          opt.innerText = `${m.name} (${m.width_m}m x ${m.length_m}m)${m.is_active ? " [Đang dùng]" : ""}`;
          if (m.is_active) opt.selected = true;
          sel.appendChild(opt);
        });
        previewMapDetails();
      }
    }
  } catch (e) {
    console.error("Lỗi nạp danh sách bản đồ:", e);
  }
}

function previewMapDetails() {
  const sel = document.getElementById("selMapList");
  if (!sel) return;
  const selVal = sel.value;
  const m = allMapsList.find(x => x.map_id === selVal);
  if (m) {
    setText("lblMapName", m.name);
    setText("lblMapDim", `${m.width_m}m (X) × ${m.length_m}m (Y)`);
    setText("lblMapDesc", m.description || "Bản đồ tuyến định tuyến tự động Dijkstra");
  }
}

// (B) Kích hoạt chuyển đổi bản đồ
async function applySwitchMap() {
  const sel = document.getElementById("selMapList");
  if (!sel) return;
  const selVal = sel.value;

  try {
    const res = await fetch("/api/wms/map/switch", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ map_id: selVal })
    });
    const data = await res.json();
    if (data.status === "success") {
      mapConfig = data.map;
      mapNodes = data.nodes;
      mapSegments = data.segments;
      
      setText("mapTitle", `Bản Đồ Mạng Lưới: ${mapConfig.name}`);
      setText("lblActiveMapBadge", mapConfig.map_id);
      setText("lblHudX", `X: 0.00m → ${mapConfig.width_m.toFixed(2)}m`);
      setText("lblHudY", `Y: 0.00m → ${mapConfig.length_m.toFixed(2)}m`);

      renderDynamicMapNodesUI(mapNodes);
      resizeCanvas();
      updateWmsData();
      alert(`🎉 Đã kích hoạt bản đồ thành công: ${mapConfig.name}`);
      closeConfigModal();
    } else {
      alert("❌ Lỗi kích hoạt bản đồ: " + data.message);
    }
  } catch (e) {
    alert("❌ Lỗi kết nối máy chủ: " + e);
  }
}

// (C) Nạp bản đồ tùy biến linh hoạt JSON
function toggleJsonPasteBox() {
  const box = document.getElementById("jsonPasteBox");
  if (box) box.classList.toggle("hidden");
}

async function downloadMapTemplate() {
  try {
    const res = await fetch("/api/wms/map/template");
    const d = await res.json();
    const blob = new Blob([JSON.stringify(d.template, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "map_template.json";
    a.click();
  } catch (e) {
    alert("❌ Lỗi tải template: " + e);
  }
}

async function exportCurrentMap() {
  const sel = document.getElementById("selMapList");
  const selVal = sel ? sel.value : "";
  try {
    const res = await fetch(`/api/wms/map/export?map_id=${selVal}`);
    const d = await res.json();
    if (d.status === "success") {
      const blob = new Blob([JSON.stringify(d.config, null, 2)], { type: "application/json" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = `${d.config.map_id.toLowerCase()}_config.json`;
      a.click();
    } else {
      alert("❌ Lỗi xuất map: " + d.message);
    }
  } catch (e) {
    alert("❌ Lỗi xuất bản đồ: " + e);
  }
}

async function handleMapFileUpload(event) {
  const file = event.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = async (e) => {
    try {
      const json = JSON.parse(e.target.result);
      await sendImportMap(json);
    } catch (err) {
      alert("❌ Lỗi cú pháp JSON: " + err);
    }
  };
  reader.readAsText(file);
}

async function applyJsonMapPaste() {
  const txtEl = document.getElementById("txtMapJson");
  const txt = txtEl ? txtEl.value.trim() : "";
  if (!txt) {
    alert("⚠️ Vui lòng dán đoạn mã JSON cấu hình bản đồ!");
    return;
  }
  try {
    const json = JSON.parse(txt);
    await sendImportMap(json);
  } catch (err) {
    alert("❌ Lỗi cú pháp JSON: " + err);
  }
}

async function sendImportMap(mapJson) {
  try {
    const res = await fetch("/api/wms/map/import", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(mapJson)
    });
    const data = await res.json();
    if (data.status === "success") {
      mapConfig = data.map;
      mapNodes = data.nodes;
      mapSegments = data.segments;
      
      setText("mapTitle", `Bản Đồ Mạng Lưới: ${mapConfig.name}`);
      setText("lblActiveMapBadge", mapConfig.map_id);
      setText("lblHudX", `X: 0.00m → ${mapConfig.width_m.toFixed(2)}m`);
      setText("lblHudY", `Y: 0.00m → ${mapConfig.length_m.toFixed(2)}m`);

      renderDynamicMapNodesUI(mapNodes);
      resizeCanvas();
      updateWmsData();
      alert(`🎉 Đã nạp và kích hoạt bản đồ mới thành công: ${mapConfig.name}`);
      closeConfigModal();
    } else {
      alert("❌ Lỗi nạp bản đồ: " + data.message);
    }
  } catch (e) {
    alert("❌ Lỗi kết nối máy chủ: " + e);
  }
}

// (D) Quản lý Danh mục Sản phẩm & Phân bổ Kho
let cachedProductsList = [];
let editingProductCode = null;

function escHtml(v) {
  return String(v ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#39;");
}

function resetProductForm() {
  editingProductCode = null;
  const inpCode = document.getElementById("inpProdCode");
  const inpName = document.getElementById("inpProdName");
  const inpYolo = document.getElementById("inpProdYoloId");
  const selCat = document.getElementById("selProdCategory");
  const lblTitle = document.getElementById("lblProdFormTitle");
  const lblBtn = document.getElementById("lblSaveProductBtn");
  const btnCancel = document.getElementById("btnCancelEditProduct");

  if (inpCode) {
    inpCode.value = "";
    inpCode.readOnly = false;
    inpCode.classList.remove("bg-slate-100", "cursor-not-allowed");
  }
  if (inpName) inpName.value = "";
  if (inpYolo) {
    const usedIds = new Set(cachedProductsList.map(p => Number(p.yolo_class_id)));
    let nextId = 3;
    while (usedIds.has(nextId)) nextId++;
    inpYolo.value = String(nextId);
  }
  if (selCat) selCat.value = "FOOD";
  if (lblTitle) lblTitle.innerText = "Thêm Mặt Hàng & Gán Trạm Đích";
  if (lblBtn) lblBtn.innerText = "Lưu Mặt Hàng & Trạm Đích";
  if (btnCancel) btnCancel.classList.add("hidden");
}

function editProductByIndex(idx) {
  const p = cachedProductsList[idx];
  if (!p) return;
  editingProductCode = p.product_code;

  const inpCode = document.getElementById("inpProdCode");
  const inpName = document.getElementById("inpProdName");
  const inpYolo = document.getElementById("inpProdYoloId");
  const selCat = document.getElementById("selProdCategory");
  const selTarget = document.getElementById("selProdTargetNode");
  const lblTitle = document.getElementById("lblProdFormTitle");
  const lblBtn = document.getElementById("lblSaveProductBtn");
  const btnCancel = document.getElementById("btnCancelEditProduct");

  if (inpCode) {
    inpCode.value = p.product_code;
    inpCode.readOnly = true;
    inpCode.classList.add("bg-slate-100", "cursor-not-allowed");
  }
  if (inpName) inpName.value = p.product_name || "";
  if (inpYolo) inpYolo.value = String(p.yolo_class_id ?? 0);
  if (selCat) {
    const cat = String(p.category || "GENERAL").toUpperCase();
    if (![...selCat.options].some(o => o.value === cat)) {
      const opt = document.createElement("option");
      opt.value = cat;
      opt.innerText = cat;
      selCat.appendChild(opt);
    }
    selCat.value = cat;
  }
  if (selTarget && p.target_node_id) {
    if ([...selTarget.options].some(o => o.value === p.target_node_id)) {
      selTarget.value = p.target_node_id;
    }
  }
  if (lblTitle) lblTitle.innerText = `Chỉnh Sửa '${p.product_code}' & Đổi Trạm Đích`;
  if (lblBtn) lblBtn.innerText = `Cập Nhật '${p.product_code}' & Kho Đích`;
  if (btnCancel) btnCancel.classList.remove("hidden");
}

async function loadProductsList() {
  try {
    const res = await fetch("/api/wms/products");
    const data = await res.json();
    if (data.status === "success") {
      cachedProductsList = data.products || [];
      setText("lblProdActiveMap", data.map_id || (mapConfig && mapConfig.map_id) || "--");

      let html = "";
      cachedProductsList.forEach((p, idx) => {
        const nodes = Array.isArray(p.target_nodes) && p.target_nodes.length
          ? p.target_nodes
          : (p.target_node_id ? [p.target_node_id] : []);
        let stationHtml = '<span class="text-slate-400 font-mono">--</span>';
        if (nodes.length === 1) {
          stationHtml = `<span class="bg-emerald-50 text-emerald-700 border border-emerald-200 px-1.5 py-0.5 rounded text-[10px] font-mono font-bold">Kho ${escHtml(nodes[0])}</span>`;
        } else if (nodes.length > 1) {
          stationHtml = `<span class="bg-emerald-50 text-emerald-700 border border-emerald-200 px-1.5 py-0.5 rounded text-[10px] font-mono font-bold">Kho ${escHtml(nodes[0])}</span> <span class="text-[10px] text-slate-500 font-mono">+${escHtml(nodes.slice(1).join(","))}</span>`;
        }

        html += `
          <tr class="hover:bg-slate-50 transition-colors">
            <td class="p-2 font-mono text-primary font-bold">${escHtml(p.product_code)}</td>
            <td class="p-2 font-medium text-slate-800">${escHtml(p.product_name)}</td>
            <td class="p-2 font-mono text-slate-500">Class: ${escHtml(p.yolo_class_id)}</td>
            <td class="p-2">
              <span class="bg-blue-50 text-primary border border-blue-200 px-1.5 py-0.5 rounded text-[10px] font-mono font-semibold">
                ${escHtml(p.category)}
              </span>
            </td>
            <td class="p-2">${stationHtml}</td>
            <td class="p-2 text-right whitespace-nowrap">
              <button onclick="editProductByIndex(${idx})" class="text-primary hover:underline text-[11px] font-semibold mr-2" type="button">
                Sửa
              </button>
              <button onclick="deleteProductByIndex(${idx})" class="text-error hover:underline text-[11px] font-medium" type="button">
                Xóa
              </button>
            </td>
          </tr>`;
      });
      const tbl = document.getElementById("tblProductList");
      if (tbl) {
        tbl.innerHTML = html || '<tr><td colspan="6" class="p-3 text-center text-slate-400">Chưa có sản phẩm trong kho</td></tr>';
      }
      onRuleScopeChange();
    }
  } catch (e) {
    console.error("Lỗi nạp danh mục sản phẩm:", e);
  }
}

async function submitNewProduct() {
  const code = document.getElementById("inpProdCode").value.trim().toUpperCase();
  const name = document.getElementById("inpProdName").value.trim();
  const rawYolo = document.getElementById("inpProdYoloId").value.trim();
  const yoloId = rawYolo === "" ? NaN : parseInt(rawYolo, 10);
  const category = document.getElementById("selProdCategory").value;
  const targetNode = document.getElementById("selProdTargetNode").value;

  if (!code || !name) {
    alert("⚠️ Vui lòng nhập đầy đủ Mã hàng và Tên sản phẩm!");
    return;
  }
  if (isNaN(yoloId) || yoloId < 0) {
    alert("⚠️ Model Class ID phải là số nguyên >= 0!");
    return;
  }

  try {
    const res = await fetch("/api/wms/products/save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        product_code: code,
        product_name: name,
        yolo_class_id: yoloId,
        category: category,
        target_node_id: targetNode
      })
    });
    const d = await res.json();
    if (d.status === "success") {
      alert("✅ " + d.message);
      resetProductForm();
      await loadProductsList();
      await loadRulesList();
      updateWmsData();
    } else {
      alert("❌ " + d.message);
    }
  } catch (e) {
    alert("❌ Lỗi kết nối: " + e);
  }
}

function deleteProductByIndex(idx) {
  const p = cachedProductsList[idx];
  if (p) deleteProduct(p.product_code);
}

async function deleteProduct(pCode) {
  if (!confirm(`Bạn có chắc muốn xóa mặt hàng '${pCode}' khỏi danh mục kho?`)) return;
  try {
    const res = await fetch("/api/wms/products/delete", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ product_code: pCode })
    });
    const d = await res.json();
    if (d.status === "success") {
      if (editingProductCode === pCode) resetProductForm();
      await loadProductsList();
      await loadRulesList();
    } else {
      alert("❌ " + d.message);
    }
  } catch (e) {
    alert("❌ Lỗi kết nối: " + e);
  }
}

function onRuleScopeChange() {
  const selScope = document.getElementById("selRuleScope");
  const selVal = document.getElementById("selRuleTargetVal");
  if (!selScope || !selVal) return;

  const scope = selScope.value;
  if (scope === "PRODUCT") {
    selVal.disabled = false;
    selVal.innerHTML = cachedProductsList.map(p =>
      `<option value="${escHtml(p.product_code)}">${escHtml(p.product_code)} (${escHtml(p.category)})</option>`
    ).join("") || '<option value="PKG_ELEC">PKG_ELEC</option>';
  } else if (scope === "CATEGORY") {
    selVal.disabled = false;
    const cats = [...new Set(["ELECTRONIC", "MECHANICAL", "TOOL", "FOOD", "MEDICAL", "GENERAL", ...cachedProductsList.map(p => p.category)])];
    selVal.innerHTML = cats.map(c => `<option value="${escHtml(c)}">${escHtml(c)}</option>`).join("");
  } else {
    selVal.innerHTML = '<option value="*">* (Tất cả - Dự phòng)</option>';
    selVal.disabled = true;
  }
}

async function loadRulesList() {
  try {
    const res = await fetch("/api/wms/rules");
    const data = await res.json();
    if (data.status === "success") {
      const rules = data.rules || [];
      let html = "";
      rules.forEach(r => {
        const targetLabel = r.product_code
          ? `<span class="font-mono font-bold text-primary">Mã: ${escHtml(r.product_code)}</span>`
          : (r.product_category === "*"
            ? `<span class="font-mono text-amber-700 font-semibold">* Dự phòng</span>`
            : `<span class="font-mono text-slate-700 font-semibold">Nhóm: ${escHtml(r.product_category)}</span>`);
        html += `
          <tr class="hover:bg-slate-50 transition-colors">
            <td class="p-2">${targetLabel}</td>
            <td class="p-2"><span class="bg-emerald-50 text-emerald-700 border border-emerald-200 px-1.5 py-0.5 rounded text-[10px] font-mono font-bold">Kho ${escHtml(r.target_node_id)}</span></td>
            <td class="p-2 font-mono text-slate-500">Prio ${escHtml(r.priority)}</td>
            <td class="p-2 text-right">
              <button onclick="deleteRule(${Number(r.rule_id)})" class="text-error hover:underline text-[11px] font-medium" type="button">
                Xóa
              </button>
            </td>
          </tr>`;
      });
      const tbl = document.getElementById("tblRulesList");
      if (tbl) {
        tbl.innerHTML = html || '<tr><td colspan="4" class="p-3 text-center text-slate-400">Chưa có quy tắc phân bổ cho bản đồ này</td></tr>';
      }
    }
  } catch (e) {
    console.error("Lỗi nạp quy tắc phân bổ kho:", e);
  }
}

async function submitNewRule() {
  const scope = document.getElementById("selRuleScope")?.value || "PRODUCT";
  const val = document.getElementById("selRuleTargetVal")?.value || "*";
  const targetNode = document.getElementById("selRuleTargetNode")?.value || "";
  const prioRaw = parseInt(document.getElementById("inpRulePriority")?.value || "1", 10);
  const priority = (!isNaN(prioRaw) && prioRaw >= 1) ? prioRaw : 1;

  if (!targetNode) {
    alert("⚠️ Vui lòng chọn Kho đích!");
    return;
  }

  let payload = { target_node_id: targetNode, priority: priority };
  if (scope === "PRODUCT") {
    const prod = cachedProductsList.find(p => p.product_code === val);
    payload.product_code = val;
    payload.product_category = prod ? prod.category : "GENERAL";
  } else if (scope === "CATEGORY") {
    payload.product_code = null;
    payload.product_category = val;
  } else {
    payload.product_code = null;
    payload.product_category = "*";
  }

  try {
    const res = await fetch("/api/wms/rules/save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload)
    });
    const d = await res.json();
    if (d.status === "success") {
      await loadProductsList();
      await loadRulesList();
    } else {
      alert("❌ " + d.message);
    }
  } catch (e) {
    alert("❌ Lỗi lưu quy tắc: " + e);
  }
}

async function deleteRule(ruleId) {
  if (!confirm(`Xóa quy tắc phân bổ #${ruleId}?`)) return;
  try {
    const res = await fetch("/api/wms/rules/delete", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ rule_id: ruleId })
    });
    const d = await res.json();
    if (d.status === "success") {
      await loadProductsList();
      await loadRulesList();
    } else {
      alert("❌ " + d.message);
    }
  } catch (e) {
    alert("❌ Lỗi xóa quy tắc: " + e);
  }
}

// ===================================================================================
// 8. TỰ ĐỘNG KHỞI CHẠY KHI TẢI TRANG
// ===================================================================================
window.addEventListener("resize", resizeCanvas);
loadMapData();
setInterval(updateState, 200);
setInterval(updateWmsData, 1000);
