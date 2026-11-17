"""
===================================================================================
UNIT TEST KIỂM THỬ BẢN ĐỒ LINE ĐỘNG & QUẢN LÝ TỒN KHO ĐA MẶT HÀNG (V2.1)
===================================================================================
"""

import sys
from pathlib import Path

# Thêm đường dẫn thư mục server và database vào sys.path
SERVER_DIR = Path(__file__).resolve().parent.parent
DB_DIR = SERVER_DIR / "database"

for p in (SERVER_DIR, DB_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from seed_data import seed_database
from repositories import (
    MapRepository,
    InventoryRepository,
    AllocationEngineRepository,
    ProductRepository,
    TaskRepository,
    FleetRepository,
    TelemetryRepository
)

def run_tests():
    print("=" * 75)
    print("🚀 BẮT ĐẦU KIỂM THỬ V2.1: BẢN ĐỒ ĐỘNG & QUẢN LÝ TỒN KHO ĐA MẶT HÀNG")
    print("=" * 75)

    # Sử dụng database kiểm thử độc lập, tránh ghi đè wms_database.db
    test_db = DB_DIR / "test_repo_wms.db"
    if test_db.exists():
        test_db.unlink()

    # 1. Khởi tạo và nạp dữ liệu mẫu
    print(">> [BƯỚC 1]: Nạp dữ liệu cấu hình bản đồ động & kho đa mặt hàng...")
    seed_database(test_db)

    map_repo = MapRepository(test_db)
    inv_repo = InventoryRepository(test_db)
    alloc_repo = AllocationEngineRepository(test_db)
    prod_repo = ProductRepository(test_db)
    task_repo = TaskRepository(test_db)
    fleet_repo = FleetRepository(test_db)
    telemetry_repo = TelemetryRepository(test_db)

    # 2. Kiểm tra Bản đồ Động (Dynamic Topological Map)
    print(">> [BƯỚC 2]: Kiểm tra Bản đồ mạng lưới Line động...")
    active_map = map_repo.get_active_map()
    assert active_map is not None and active_map["map_id"] == "SABAN_154_151", "Lỗi: Không tìm thấy bản đồ active"
    nodes = map_repo.get_nodes(active_map["map_id"])
    segments = map_repo.get_segments(active_map["map_id"])
    assert len(nodes) == 4, f"Lỗi: Số lượng node phải là 4, thực tế: {len(nodes)}"
    assert len(segments) == 8, f"Lỗi: Số lượng đoạn line phải là 8, thực tế: {len(segments)}"
    print(f"   ✅ Bản đồ Active: '{active_map['name']}' ({active_map['width_m']}m x {active_map['length_m']}m)")
    print(f"   ✅ Danh sách Điểm nút Line: {[n['node_id'] for n in nodes]}")
    print(f"   ✅ Danh sách Đoạn line kết nối đo thực tế: {[s['segment_id'] for s in segments]}")

    # 3. Kiểm tra Tồn kho Đa mặt hàng (Multi-Product Inventory tại 1 trạm)
    print(">> [BƯỚC 3]: Kiểm chứng Trạm B chứa ĐA MẶT HÀNG (Không bị gán cứng)...")
    # Thêm thử nghiệm 2 mặt hàng vào trạm B để chứng minh trạm B lưu được nhiều chủng loại
    inv_repo.update_stock(active_map["map_id"], "B", "PKG_ELEC", 5)
    inv_repo.update_stock(active_map["map_id"], "B", "PKG_MECH", 2)

    inv_b = inv_repo.get_node_inventory(active_map["map_id"], "B")
    cap_b = inv_repo.get_node_capacity(active_map["map_id"], "B")
    assert len(inv_b) >= 2, "Lỗi: Trạm B phải chứa ít nhất 2 loại sản phẩm khác nhau"
    print(f"   ✅ Trạm B đang chứa:")
    for item in inv_b:
        print(f"      - {item['product_name']} ({item['product_code']}): {item['quantity']} kiện")
    print(f"   ✅ Tổng sức chứa Trạm B: {cap_b['current_stock']}/{cap_b['max_capacity']} kiện (Còn trống: {cap_b['available_slots']})")

    # 4. Kiểm tra Bộ quy tắc Phân bổ Kho Động (Dynamic Allocation Engine)
    print(">> [BƯỚC 4]: Kiểm tra Phân bổ Kho Động theo AI Detection...")
    p_elec = prod_repo.get_by_code("PKG_ELEC")
    p_mech = prod_repo.get_by_code("PKG_MECH")
    
    alloc_elec = alloc_repo.allocate_target_node(active_map["map_id"], p_elec["product_code"], p_elec["category"])
    assert alloc_elec is not None and alloc_elec["target_node_id"] == "B", "Lỗi: PKG_ELEC phải phân bổ về B"
    print(f"   ✅ AI Detect 'Bo Mạch Vi Điều Khiển' -> Bộ quy tắc tự động cấp kho đích: Trạm {alloc_elec['target_node_id']} (Rule #{alloc_elec['rule_id']})")

    alloc_mech = alloc_repo.allocate_target_node(active_map["map_id"], p_mech["product_code"], p_mech["category"])
    assert alloc_mech is not None and alloc_mech["target_node_id"] == "C", "Lỗi: PKG_MECH phải phân bổ về C"
    print(f"   ✅ AI Detect 'Bánh Răng Cơ Khí' -> Bộ quy tắc tự động cấp kho đích: Trạm {alloc_mech['target_node_id']} (Rule #{alloc_mech['rule_id']})")

    # 5. Tạo Task điều phối động gắn liền với bản đồ và kho đích vừa cấp
    print(">> [BƯỚC 5]: Tạo Task điều phối trên Bản đồ hiện tại...")
    task_id = task_repo.create_task(
        map_id=active_map["map_id"],
        product_code=p_elec["product_code"],
        product_name=p_elec["product_name"],
        from_node="A",
        to_node=alloc_elec["target_node_id"],
        route_path="A->B",
        ai_confidence=0.965,
        image_path="/uploads/test_product.jpg",
        assigned_agv="AGV_Ver2"
    )
    assert task_id > 0, "Lỗi tạo Task"
    print(f"   ✅ Đã tạo Task #{task_id}: Giao hàng từ A -> {alloc_elec['target_node_id']} trên Bản đồ {active_map['map_id']}")

    # 6. Khóa đoạn line kết nối thực tế (SEG_AB & SEG_BA)
    print(">> [BƯỚC 6]: Khóa đoạn line thực tế SEG_AB / SEG_BA...")
    map_repo.lock_segments(active_map["map_id"], ["SEG_AB", "SEG_BA"], agv_id="AGV_Ver2")
    segs = {s["segment_id"]: s for s in map_repo.get_segments(active_map["map_id"])}
    assert segs["SEG_AB"]["is_locked"] == 1, "Lỗi: Đoạn line SEG_AB chưa khóa"
    print("   ✅ Đoạn line thực tế SEG_AB và SEG_BA đã khóa chống xung đột đa xe")

    # 7. Xe hoàn thành dỡ hàng -> Tăng tồn kho mặt hàng cụ thể tại Trạm B
    print(">> [BƯỚC 7]: Dỡ hàng và cập nhật tồn kho mặt hàng cụ thể...")
    task_repo.update_status(task_id, "COMPLETED")
    inv_repo.update_stock(active_map["map_id"], "B", product_code="PKG_ELEC", delta=1)
    map_repo.unlock_segments(active_map["map_id"], ["SEG_AB", "SEG_BA"])

    updated_inv_b = {i["product_code"]: i["quantity"] for i in inv_repo.get_node_inventory(active_map["map_id"], "B")}
    assert updated_inv_b["PKG_ELEC"] == 6, f"Lỗi: Số lượng PKG_ELEC phải tăng lên 6, thực tế: {updated_inv_b['PKG_ELEC']}"
    assert updated_inv_b["PKG_MECH"] == 2, "Lỗi: Số lượng PKG_MECH tại Trạm B phải giữ nguyên là 2"
    print(f"   ✅ Tồn kho PKG_ELEC tại Trạm B đã tăng thành: {updated_inv_b['PKG_ELEC']} kiện (PKG_MECH vẫn giữ nguyên {updated_inv_b['PKG_MECH']} kiện)")
    print("   ✅ Đã mở khóa an toàn đoạn line SEG_AB")

    # 8. Ghi log Telemetry gắn với bản đồ
    print(">> [BƯỚC 8]: Ghi Telemetry kèm ID Bản đồ...")
    telemetry_repo.log(
        agv_id="AGV_Ver2", map_id=active_map["map_id"], x=0.00, y=1.51, yaw=90.0,
        dist=1.51, speed=0.0, mask=4, err=0.0, v=11.75, a=0.40, w=4.7, mah=15.2,
        state="AT_STATION_B", task_id=task_id
    )
    print("   ✅ Ghi thành công Log Telemetry có map_id")

    if test_db.exists():
        try:
            test_db.unlink()
        except Exception:
            pass

    print("=" * 75)
    print("🎉 TẤT CẢ CÁC BÀI TEST V2.1 ĐÃ THÀNH CÔNG VƯỢT TRỘI (100% PASS)!")
    print("=" * 75)

if __name__ == "__main__":
    run_tests()
