"""
===================================================================================
SCRIPT NẠP DỮ LIỆU BAN ĐẦU (SEED DATA) CHO BẢN ĐỒ & TỒN KHO ĐA MẶT HÀNG (V2.1)
===================================================================================
"""

from pathlib import Path
import sys
SERVER_DIR = Path(__file__).resolve().parent.parent
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

from database.db_connection import get_db, init_db, DB_PATH

def seed_database(db_file: Path = DB_PATH, reset: bool = True):
    """Nạp dữ liệu cấu hình bản đồ động và quy tắc phân bổ kho thông minh.

    - Khi reset=False (mặc định khi server khởi động lại): giữ nguyên toàn bộ bản đồ,
      danh mục sản phẩm, quy tắc phân bổ trạm, tồn kho và lịch sử nếu DB đã khởi tạo.
    - Khi reset=True hoặc DB mới tinh: nạp dữ liệu mẫu ban đầu.
    """
    # 1. Đảm bảo cấu trúc bảng đã tồn tại
    init_db(db_file)

    with get_db(db_file) as conn:
        cursor = conn.cursor()

        if not reset:
            maps_cnt = cursor.execute("SELECT COUNT(*) FROM maps;").fetchone()[0]
            prods_cnt = cursor.execute("SELECT COUNT(*) FROM products;").fetchone()[0]
            if maps_cnt > 0 and prods_cnt > 0:
                # Chỉ mở khóa line treo (nếu server tắt đột ngột trước đó) và giữ nguyên dữ liệu WMS
                cursor.execute("UPDATE line_segments SET is_locked = 0, locked_by_agv = NULL;")
                cursor.execute(
                    "UPDATE agv_fleet SET status = 'OFFLINE', speed_cm_s = 0.0, "
                    "voltage_v = 0.0, current_a = 0.0, battery_pct = 0.0;"
                )
                print(f"💾 Đã phát hiện Database hiện có ({maps_cnt} bản đồ, {prods_cnt} mặt hàng) -> Giữ nguyên cấu hình & tồn kho: {db_file}")
                return

        # 2. NẠP CÁC BẢN ĐỒ MẪU (BẢN ĐỒ 1: SA BÀN LAB 1.54m x 1.51m & BẢN ĐỒ 2: KHO MỞ RỘNG 6 TRẠM)
        cursor.execute("""
            INSERT INTO maps (map_id, name, width_m, length_m, description, is_active)
            VALUES ('SABAN_154_151', 'Sa bàn Thực nghiệm Lab 1.54m x 1.51m', 1.54, 1.51, 
                    'Mạng lưới line 4 trạm hình chữ nhật 8 chặng 2 chiều', 1)
            ON CONFLICT(map_id) DO UPDATE SET
                name=excluded.name, width_m=excluded.width_m, length_m=excluded.length_m;
        """)

        cursor.execute("""
            INSERT INTO maps (map_id, name, width_m, length_m, description, is_active)
            VALUES ('MAP_EXP_6NODES', 'Kho Mở Rộng 6 Trạm 2.50m x 1.80m', 2.50, 1.80, 
                    'Mô hình kho vận mở rộng 6 trạm trung chuyển (A, B, C, D, E, F)', 0)
            ON CONFLICT(map_id) DO UPDATE SET
                name=excluded.name, width_m=excluded.width_m, length_m=excluded.length_m;
        """)

        # 3. NẠP CÁC ĐIỂM NÚT TRÊN LINE (MAP_NODES)
        nodes_data = [
            # Bản đồ 1: Sa bàn Lab
            ("SABAN_154_151", "A", "Trạm Nhập Hàng A (0.00m, 0.00m)", "INBOUND", 0.00, 0.00, "#10b981", 100, 1, "AGV_Ver2"),
            ("SABAN_154_151", "B", "Kho Kệ 1 B (0.00m, 1.51m)", "STATION", 0.00, 1.51, "#38bdf8", 30, 0, None),
            ("SABAN_154_151", "C", "Kho Kệ 2 C (1.54m, 1.51m)", "STATION", 1.54, 1.51, "#a855f7", 30, 0, None),
            ("SABAN_154_151", "D", "Kho Kệ 3 D (1.54m, 0.00m)", "STATION", 1.54, 0.00, "#f59e0b", 30, 0, None),
            
            # Bản đồ 2: Kho mở rộng 6 trạm
            ("MAP_EXP_6NODES", "A", "Trạm Nhập Hàng A (0.00m, 0.00m)", "INBOUND", 0.00, 0.00, "#10b981", 100, 1, "AGV_Ver2"),
            ("MAP_EXP_6NODES", "B", "Kho Kệ 1 B (0.00m, 1.80m)", "STATION", 0.00, 1.80, "#38bdf8", 30, 0, None),
            ("MAP_EXP_6NODES", "C", "Kho Kệ 2 C (1.25m, 1.80m)", "STATION", 1.25, 1.80, "#a855f7", 30, 0, None),
            ("MAP_EXP_6NODES", "D", "Kho Kệ 3 D (2.50m, 1.80m)", "STATION", 2.50, 1.80, "#f59e0b", 30, 0, None),
            ("MAP_EXP_6NODES", "E", "Kho Kệ 4 E (2.50m, 0.00m)", "STATION", 2.50, 0.00, "#ec4899", 30, 0, None),
            ("MAP_EXP_6NODES", "F", "Trạm Đệm F (1.25m, 0.00m)", "STATION", 1.25, 0.00, "#06b6d4", 30, 0, None)
        ]
        cursor.executemany("""
            INSERT INTO map_nodes (map_id, node_id, name, node_type, x, y, color, max_capacity, is_occupied, occupied_by_agv)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT(map_id, node_id) DO UPDATE SET
                name=excluded.name, node_type=excluded.node_type, x=excluded.x, y=excluded.y,
                color=excluded.color, max_capacity=excluded.max_capacity;
        """, nodes_data)

        # 4. NẠP CÁC ĐOẠN LINE KẾT NỐI ĐO ĐẠC THỰC TẾ (LINE_SEGMENTS)
        segments_data = [
            # Bản đồ 1: Sa bàn Lab
            ("SABAN_154_151", "SEG_AB", "A", "B", 1.51, "TWO_WAY", 90.0, 25.0, 0, 0, None),
            ("SABAN_154_151", "SEG_BC", "B", "C", 1.54, "TWO_WAY", 0.0, 25.0, 0, 0, None),
            ("SABAN_154_151", "SEG_CD", "C", "D", 1.51, "TWO_WAY", -90.0, 25.0, 0, 0, None),
            ("SABAN_154_151", "SEG_DA", "D", "A", 1.54, "TWO_WAY", 180.0, 25.0, 0, 0, None),
            ("SABAN_154_151", "SEG_BA", "B", "A", 1.51, "TWO_WAY", -90.0, 25.0, 0, 0, None),
            ("SABAN_154_151", "SEG_CB", "C", "B", 1.54, "TWO_WAY", 180.0, 25.0, 0, 0, None),
            ("SABAN_154_151", "SEG_DC", "D", "C", 1.51, "TWO_WAY", 90.0, 25.0, 0, 0, None),
            ("SABAN_154_151", "SEG_AD", "A", "D", 1.54, "TWO_WAY", 0.0, 25.0, 0, 0, None),

            # Bản đồ 2: Kho mở rộng 6 trạm
            ("MAP_EXP_6NODES", "SEG_AB", "A", "B", 1.80, "TWO_WAY", 90.0, 25.0, 0, 0, None),
            ("MAP_EXP_6NODES", "SEG_BC", "B", "C", 1.25, "TWO_WAY", 0.0, 25.0, 0, 0, None),
            ("MAP_EXP_6NODES", "SEG_CD", "C", "D", 1.25, "TWO_WAY", 0.0, 25.0, 0, 0, None),
            ("MAP_EXP_6NODES", "SEG_DE", "D", "E", 1.80, "TWO_WAY", -90.0, 25.0, 0, 0, None),
            ("MAP_EXP_6NODES", "SEG_EF", "E", "F", 1.25, "TWO_WAY", 180.0, 25.0, 0, 0, None),
            ("MAP_EXP_6NODES", "SEG_FA", "F", "A", 1.25, "TWO_WAY", 180.0, 25.0, 0, 0, None),
            ("MAP_EXP_6NODES", "SEG_BA", "B", "A", 1.80, "TWO_WAY", -90.0, 25.0, 0, 0, None),
            ("MAP_EXP_6NODES", "SEG_CB", "C", "B", 1.25, "TWO_WAY", 180.0, 25.0, 0, 0, None),
            ("MAP_EXP_6NODES", "SEG_DC", "D", "C", 1.25, "TWO_WAY", 180.0, 25.0, 0, 0, None),
            ("MAP_EXP_6NODES", "SEG_ED", "E", "D", 1.80, "TWO_WAY", 90.0, 25.0, 0, 0, None),
            ("MAP_EXP_6NODES", "SEG_FE", "F", "E", 1.25, "TWO_WAY", 0.0, 25.0, 0, 0, None),
            ("MAP_EXP_6NODES", "SEG_AF", "A", "F", 1.25, "TWO_WAY", 0.0, 25.0, 0, 0, None)
        ]
        cursor.executemany("""
            INSERT INTO line_segments (map_id, segment_id, from_node, to_node, distance_m, direction, heading_deg, speed_limit_cm_s, has_gap, is_locked, locked_by_agv)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT(map_id, segment_id) DO UPDATE SET
                distance_m=excluded.distance_m, heading_deg=excluded.heading_deg,
                is_locked=0, locked_by_agv=NULL;
        """, segments_data)

        # 5. NẠP DANH MỤC SẢN PHẨM & MÔ HÌNH AI MOBILENETV3 (PRODUCTS)
        # Khớp class_names.json của best.pth: 0 = cylinder (PKG_MECH), 1 = rectangular_box (PKG_ELEC), 2 = triangular_prism (PKG_TOOL)
        cursor.execute(
            "UPDATE products SET yolo_class_id = -(yolo_class_id + 1000) "
            "WHERE product_code IN ('PKG_MECH', 'PKG_ELEC', 'PKG_TOOL') AND yolo_class_id >= 0;"
        )
        products_data = [
            ("PKG_MECH", "Chi Tiết Bánh Răng & Khớp Nối", 0, "MECHANICAL", "Hàng chi tiết cơ khí chính xác"),
            ("PKG_ELEC", "Bo Mạch Vi Điều Khiển", 1, "ELECTRONIC", "Hàng linh kiện điện tử nhạy cảm ESD"),
            ("PKG_TOOL", "Bộ Dụng Cụ Đo & Kiểm Thử", 2, "TOOL", "Hàng thiết bị đo lường kiểm thử")
        ]
        cursor.executemany("""
            INSERT INTO products (product_code, product_name, yolo_class_id, category, description)
            VALUES (?, ?, ?, ?, ?)
            ON CONFLICT(product_code) DO UPDATE SET
                product_name=excluded.product_name, yolo_class_id=excluded.yolo_class_id,
                category=excluded.category;
        """, products_data)

        # 6. KHỞI TẠO TỒN KHO THỰC TẾ (SẠCH SẼ, KHÔNG NẠP DATA ẢO)
        # Các trạm kho bắt đầu từ 0 kiện hàng; hàng chỉ tăng khi Camera quét và AGV giao tới nơi!
        cursor.execute("DELETE FROM inventory WHERE map_id IN ('SABAN_154_151', 'MAP_EXP_6NODES');")

        # 7. NẠP BỘ QUY TẮC PHÂN BỔ KHO ĐÍCH ĐỘNG (ALLOCATION_RULES)
        # Xóa các luật cũ trước khi nạp để tránh nhân bản trùng lặp bản ghi
        cursor.execute("DELETE FROM allocation_rules WHERE map_id IN ('SABAN_154_151', 'MAP_EXP_6NODES');")
        rules_data = [
            # Quy tắc cho Bản đồ 1 (Sa bàn Lab):
            ("SABAN_154_151", "ELECTRONIC", None, "B", 1), # Nhóm Điện tử -> Ưu tiên Kho B
            ("SABAN_154_151", "MECHANICAL", None, "C", 1), # Nhóm Cơ khí -> Ưu tiên Kho C
            ("SABAN_154_151", "TOOL", None, "D", 1),       # Nhóm Dụng cụ -> Ưu tiên Kho D
            ("SABAN_154_151", "*", None, "D", 9),          # Dự phòng nếu các kho kia đầy -> Đưa về Kho D

            # Quy tắc cho Bản đồ 2 (Kho mở rộng 6 trạm):
            ("MAP_EXP_6NODES", "ELECTRONIC", None, "B", 1), # Nhóm Điện tử -> Kho B
            ("MAP_EXP_6NODES", "MECHANICAL", None, "C", 1), # Nhóm Cơ khí -> Kho C
            ("MAP_EXP_6NODES", "TOOL", None, "D", 1),       # Nhóm Dụng cụ -> Kho D
            ("MAP_EXP_6NODES", "*", None, "E", 9)          # Dự phòng -> Kho E
        ]
        cursor.executemany("""
            INSERT INTO allocation_rules (map_id, product_category, product_code, target_node_id, priority)
            VALUES (?, ?, ?, ?, ?);
        """, rules_data)

        # 8. NẠP THÔNG TIN XE AGV_VER2 (TRẠNG THÁI BAN ĐẦU: NGOẠI TUYẾN 0V, 0% PIN KHI CHƯA KẾT NỐI)
        cursor.execute("""
            INSERT INTO agv_fleet (agv_id, map_id, status, current_node, x, y, yaw_deg, voltage_v, current_a, battery_pct, battery_type, last_ack_id)
            VALUES ('AGV_Ver2', 'SABAN_154_151', 'OFFLINE', 'A', 0.00, 0.00, 90.0, 0.0, 0.0, 0.0, '3S_LIPO', 0)
            ON CONFLICT(agv_id) DO UPDATE SET
                map_id='SABAN_154_151', current_node='A', x=0.00, y=0.00, yaw_deg=90.0, 
                status='OFFLINE', voltage_v=0.0, current_a=0.0, battery_pct=0.0, last_ack_id=0;
        """)

        # 9. DỌN SẠCH DỮ LIỆU NHIỆM VỤ & TELEMETRY TEST ẢO
        cursor.execute("DELETE FROM dispatch_tasks;")
        cursor.execute("DELETE FROM telemetry_logs;")

        # 10. GHI NHẬT KÝ SỰ KIỆN
        cursor.execute("""
            INSERT INTO system_events (event_type, source, message, details)
            VALUES ('INFO', 'WMS_INIT', 'Khởi tạo thành công Bản đồ động & Làm sạch Database', 
                    'Map: SABAN_154_151, Nodes: 4, Segments: 8, Products: 3, Dynamic Rules: 4, AGV: OFFLINE');
        """)

    print(f"🎉 Đã nạp thành công dữ liệu Bản đồ động & Tồn kho đa sản phẩm vào: {db_file}")

if __name__ == "__main__":
    seed_database()
