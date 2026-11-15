-- ===================================================================================
-- SCHEMA DATABASE SQLite NÂNG CẤP CHO HỆ THỐNG WMS & ĐIỀU PHỐI AGV_VER2 (V2.1)
-- THIẾT KẾ ĐỘC LẬP BẢN ĐỒ & QUẢN LÝ TỒN KHO ĐA MẶT HÀNG LINH HOẠT:
--   1. Quản lý Đa Bản đồ Mạng lưới Line (Topological Line Graph Maps)
--   2. Các Điểm Nút trên Line (Map Nodes: Stations, Intersections, Inbound, Charging)
--   3. Các Đoạn Line Kết nối Thực tế (Line Segments: Khoảng cách thật, góc hướng)
--   4. Quản lý Tồn kho Chi tiết theo Mặt hàng (Inventory per Station)
--   5. Bộ quy tắc Phân bổ Vị trí Kho Động (Dynamic Allocation Rules)
--   6. Quản lý Nhiệm vụ Điều phối (Dispatch Tasks) & Đội xe (Fleet Management)
--   7. Data Logger Telemetry & Năng lượng INA226
-- ===================================================================================

PRAGMA foreign_keys = ON;

-- 1. BẢNG QUẢN LÝ BẢN ĐỒ SA BÀN / NHÀ KHO (MAPS)
CREATE TABLE IF NOT EXISTS maps (
    map_id TEXT PRIMARY KEY,                           -- 'SABAN_154_151', 'FACTORY_ZONE_A'...
    name TEXT NOT NULL,                                -- 'Sa bàn Thực nghiệm 1.54m x 1.51m'
    width_m REAL NOT NULL,                             -- Chiều dài sa bàn thực tế (m)
    length_m REAL NOT NULL,                            -- Chiều rộng sa bàn thực tế (m)
    description TEXT,
    is_active INTEGER NOT NULL DEFAULT 1,              -- 1: Bản đồ đang kích hoạt sử dụng
    created_at TEXT DEFAULT (datetime('now', 'localtime'))
);

-- 2. BẢNG CÁC ĐIỂM NÚT TRÊN ĐƯỜNG LINE (MAP_NODES)
CREATE TABLE IF NOT EXISTS map_nodes (
    map_id TEXT NOT NULL REFERENCES maps(map_id) ON DELETE CASCADE,
    node_id TEXT NOT NULL,                             -- 'A', 'B', 'C', 'D', 'INT_01', 'CHG_01'...
    name TEXT NOT NULL,                                -- 'Trạm Nhập Hàng A', 'Kho Kệ 1 B'...
    node_type TEXT NOT NULL DEFAULT 'STATION',         -- 'STATION', 'INBOUND', 'INTERSECTION', 'CHARGING', 'PARKING'
    x REAL NOT NULL,                                   -- Tọa độ X thực tế trên line (m)
    y REAL NOT NULL,                                   -- Tọa độ Y thực tế trên line (m)
    color TEXT DEFAULT '#38bdf8',                      -- Mã màu hiển thị trên Web UI
    max_capacity INTEGER DEFAULT 30,                   -- Sức chứa tối đa nếu là trạm kho
    is_occupied INTEGER DEFAULT 0,                     -- 1 nếu đang có xe đỗ tại điểm này
    occupied_by_agv TEXT DEFAULT NULL,
    created_at TEXT DEFAULT (datetime('now', 'localtime')),
    PRIMARY KEY (map_id, node_id)
);

-- 3. BẢNG CÁC ĐOẠN LINE KẾT NỐI (LINE_SEGMENTS - TRAFFIC & ZONE LOCKING)
CREATE TABLE IF NOT EXISTS line_segments (
    map_id TEXT NOT NULL REFERENCES maps(map_id) ON DELETE CASCADE,
    segment_id TEXT NOT NULL,                          -- 'SEG_AB', 'SEG_BC', 'SEG_CD'...
    from_node TEXT NOT NULL,
    to_node TEXT NOT NULL,
    distance_m REAL NOT NULL,                          -- Khoảng cách đo thực tế dọc theo đường line (m)
    direction TEXT NOT NULL DEFAULT 'TWO_WAY',         -- 'ONE_WAY', 'TWO_WAY'
    heading_deg REAL NOT NULL,                         -- Góc hướng la bàn MPU-6050 trên đoạn line (độ)
    speed_limit_cm_s REAL DEFAULT 25.0,                -- Giới hạn tốc độ trên đoạn line này
    has_gap INTEGER DEFAULT 0,                         -- 1 nếu đoạn line này có vạch đứt / mờ cần Dead Reckoning
    is_locked INTEGER NOT NULL DEFAULT 0,              -- 1 nếu đang bị khóa bởi xe đang di chuyển
    locked_by_agv TEXT DEFAULT NULL,
    updated_at TEXT DEFAULT (datetime('now', 'localtime')),
    PRIMARY KEY (map_id, segment_id),
    FOREIGN KEY (map_id, from_node) REFERENCES map_nodes(map_id, node_id) ON DELETE CASCADE,
    FOREIGN KEY (map_id, to_node) REFERENCES map_nodes(map_id, node_id) ON DELETE CASCADE
);

-- 4. BẢNG DANH MỤC SẢN PHẨM & MÔ HÌNH AI YOLO (PRODUCTS)
CREATE TABLE IF NOT EXISTS products (
    product_code TEXT PRIMARY KEY,                     -- 'PKG_ELEC', 'PKG_MECH', 'PKG_TOOL'
    product_name TEXT NOT NULL,                        -- 'Bo Mạch Vi Điều Khiển', 'Hộp Bánh Răng'...
    yolo_class_id INTEGER NOT NULL UNIQUE,             -- Class ID trong model YOLO (0, 1, 2...)
    category TEXT NOT NULL,                            -- 'ELECTRONIC', 'MECHANICAL', 'TOOL'
    description TEXT,
    created_at TEXT DEFAULT (datetime('now', 'localtime'))
);

-- 5. BẢNG TỒN KHO CHI TIẾT TỪNG TRẠM (INVENTORY - ĐA SẢN PHẨM)
CREATE TABLE IF NOT EXISTS inventory (
    inventory_id INTEGER PRIMARY KEY AUTOINCREMENT,
    map_id TEXT NOT NULL,
    node_id TEXT NOT NULL,
    product_code TEXT NOT NULL REFERENCES products(product_code) ON DELETE RESTRICT,
    quantity INTEGER NOT NULL DEFAULT 0,               -- Số lượng kiện hàng loại này tại trạm
    updated_at TEXT DEFAULT (datetime('now', 'localtime')),
    FOREIGN KEY (map_id, node_id) REFERENCES map_nodes(map_id, node_id) ON DELETE CASCADE,
    UNIQUE (map_id, node_id, product_code)
);

-- 6. BẢNG BỘ QUY TẮC PHÂN BỔ KHO ĐÍCH ĐỘNG (ALLOCATION_RULES)
CREATE TABLE IF NOT EXISTS allocation_rules (
    rule_id INTEGER PRIMARY KEY AUTOINCREMENT,
    map_id TEXT NOT NULL REFERENCES maps(map_id) ON DELETE CASCADE,
    product_category TEXT,                             -- 'ELECTRONIC', 'MECHANICAL', '*' (Tất cả)
    product_code TEXT,                                 -- Hoặc chỉ định rõ mặt hàng, NULL nếu áp dụng theo category
    target_node_id TEXT NOT NULL,                      -- Node trạm kho đích
    priority INTEGER NOT NULL DEFAULT 1,               -- Độ ưu tiên (1 là cao nhất)
    is_active INTEGER NOT NULL DEFAULT 1,
    FOREIGN KEY (map_id, target_node_id) REFERENCES map_nodes(map_id, node_id) ON DELETE CASCADE,
    UNIQUE (map_id, product_category, product_code, target_node_id)
);

-- 7. BẢNG QUẢN LÝ ĐỘI XE AGV (AGV_FLEET)
CREATE TABLE IF NOT EXISTS agv_fleet (
    agv_id TEXT PRIMARY KEY,                           -- 'AGV_Ver2'
    map_id TEXT REFERENCES maps(map_id) ON DELETE SET NULL,
    status TEXT NOT NULL DEFAULT 'OFFLINE',            -- 'OFFLINE', 'IDLE', 'RUNNING', 'UNLOADING', 'CHARGING', 'EMERGENCY'
    current_node TEXT DEFAULT 'A',                     -- Vị trí nút hiện tại xe đang đỗ
    target_node TEXT,                                  -- Vị trí nút đích xe đang hướng tới
    current_task_id INTEGER DEFAULT NULL,
    x REAL DEFAULT 0.0,
    y REAL DEFAULT 0.0,
    yaw_deg REAL DEFAULT 90.0,
    speed_cm_s REAL DEFAULT 0.0,
    voltage_v REAL DEFAULT 0.0,
    current_a REAL DEFAULT 0.0,
    battery_pct REAL DEFAULT 0.0,
    battery_type TEXT DEFAULT '3S_LIPO',
    last_ack_id INTEGER DEFAULT 0,
    last_heartbeat TEXT DEFAULT (datetime('now', 'localtime')),
    created_at TEXT DEFAULT (datetime('now', 'localtime'))
);

-- 8. BẢNG NHIỆM VỤ ĐIỀU PHỐI (DISPATCH_TASKS)
CREATE TABLE IF NOT EXISTS dispatch_tasks (
    task_id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_code TEXT UNIQUE NOT NULL,                    -- 'TSK-20261105-001'
    map_id TEXT NOT NULL REFERENCES maps(map_id) ON DELETE RESTRICT,
    product_code TEXT REFERENCES products(product_code) ON DELETE RESTRICT,
    product_name TEXT,
    from_node TEXT NOT NULL,                           -- Trạm nạp hàng
    to_node TEXT NOT NULL,                             -- Trạm đích phân bổ
    route_path TEXT NOT NULL,                          -- Chuỗi lộ trình line: 'A->B', 'A->B->C'...
    assigned_agv TEXT DEFAULT 'AGV_Ver2' REFERENCES agv_fleet(agv_id) ON DELETE SET NULL,
    ai_confidence REAL DEFAULT 0.0,
    image_path TEXT,
    status TEXT NOT NULL DEFAULT 'PENDING',            -- 'PENDING', 'ASSIGNED', 'IN_TRANSIT', 'DELIVERED', 'COMPLETED', 'REJECTED_UNKNOWN', 'REJECTED_STOCK_FULL', 'ABORTED'
    dwell_time_sec REAL DEFAULT 5.0,
    error_message TEXT,
    created_at TEXT DEFAULT (datetime('now', 'localtime')),
    started_at TEXT,
    completed_at TEXT
);

-- 9. BẢNG NHẬT KÝ TELEMETRY & NĂNG LƯỢNG (TELEMETRY_LOGS)
CREATE TABLE IF NOT EXISTS telemetry_logs (
    log_id INTEGER PRIMARY KEY AUTOINCREMENT,
    agv_id TEXT NOT NULL REFERENCES agv_fleet(agv_id) ON DELETE CASCADE,
    map_id TEXT,
    timestamp TEXT DEFAULT (datetime('now', 'localtime')),
    station TEXT,                                      -- Trạm hiện tại ('A', 'B', 'C', 'D')
    leg TEXT,                                          -- Chặng hiện tại ('A ➔ B', 'B ➔ C'...)
    x REAL,
    y REAL,
    yaw REAL,
    dist REAL,
    speed REAL,
    line_mask INTEGER,
    line_err REAL,
    voltage REAL,
    current REAL,
    power REAL,
    energy_mah REAL,
    state TEXT,
    task_id INTEGER REFERENCES dispatch_tasks(task_id) ON DELETE SET NULL
);

-- 10. BẢNG SỰ KIỆN HỆ THỐNG (SYSTEM_EVENTS)
CREATE TABLE IF NOT EXISTS system_events (
    event_id INTEGER PRIMARY KEY AUTOINCREMENT,
    event_type TEXT NOT NULL,                          -- 'INFO', 'WARNING', 'ERROR', 'SAFETY'
    source TEXT NOT NULL,                              -- 'AI_ENGINE', 'WMS_ALLOCATOR', 'AGV_FIRMWARE', 'POWER_GUARD'
    message TEXT NOT NULL,
    details TEXT,
    created_at TEXT DEFAULT (datetime('now', 'localtime'))
);

-- ===================================================================================
-- CÁC CHỈ MỤC TỐI ƯU TRUY VẤN
-- ===================================================================================
CREATE INDEX IF NOT EXISTS idx_nodes_map ON map_nodes(map_id);
CREATE INDEX IF NOT EXISTS idx_segments_map ON line_segments(map_id);
CREATE INDEX IF NOT EXISTS idx_inventory_lookup ON inventory(map_id, node_id, product_code);
CREATE INDEX IF NOT EXISTS idx_tasks_status ON dispatch_tasks(status);
CREATE INDEX IF NOT EXISTS idx_telemetry_agv_time ON telemetry_logs(agv_id, timestamp);
