"""
BỘ CHẠY KIỂM THỬ HỆ THỐNG TOÀN DIỆN (UNIFIED TEST SUITE)
AGV & WMS Smart Warehouse System
======================================================
Tập hợp và thực thi tất cả các bài test từ các module:
1. Database Repositories (test_db.py)
2. Finite State Machine (test_fsm.py)
3. HTTP Controllers & Business APIs (test_controllers.py)
"""

import sys
import time
from pathlib import Path

# Đảm bảo đường dẫn server nằm trong sys.path
TESTS_DIR = Path(__file__).resolve().parent
SERVER_DIR = TESTS_DIR.parent
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

# Import các hàm chạy test
from tests.test_db import run_tests as run_db_tests
from tests.test_fsm import run_tests as run_fsm_tests
from tests.test_controllers import run_controller_tests
from tests.test_two_shot_workflow import run_tests as run_two_shot_tests

def main():
    start_time = time.time()
    print("=" * 70)
    print("          CHẠY TOÀN BỘ BÀI KIỂM THỬ HỆ THỐNG AGV_VER2 SERVER")
    print("=" * 70)
    
    test_results = {}
    
    # 1. Database Repositories Test
    print("\n[1/4] ĐANG KIỂM THỬ TẦNG DATABASE & REPOSITORIES...")
    try:
        run_db_tests()
        test_results["Database Repositories"] = "PASSED"
    except Exception as e:
        print(f"[-] LỖI TẦNG DATABASE: {e}")
        test_results["Database Repositories"] = f"FAILED: {e}"

    # 2. Finite State Machine (FSM) Test
    print("\n[2/4] ĐANG KIỂM THỬ TẦNG TRẠNG THÁI FSM (AGV & TASK)...")
    try:
        run_fsm_tests()
        test_results["Finite State Machine"] = "PASSED"
    except Exception as e:
        print(f"[-] LỖI TẦNG FSM: {e}")
        test_results["Finite State Machine"] = f"FAILED: {e}"

    # 3. HTTP Controllers Test
    print("\n[3/4] ĐANG KIỂM THỬ TẦNG CONTROLLERS & ENDPOINTS...")
    try:
        run_controller_tests()
        test_results["HTTP Controllers & APIs"] = "PASSED"
    except Exception as e:
        print(f"[-] LỖI TẦNG CONTROLLERS: {e}")
        test_results["HTTP Controllers & APIs"] = f"FAILED: {e}"

    # 4. Two-Shot AI & Nearest Warehouse Test
    print("\n[4/4] ĐANG KIỂM THỬ QUY TRÌNH 2-SHOT AI & CHỌN KHO GẦN NHẤT...")
    try:
        run_two_shot_tests()
        test_results["2-Shot Vision & Nearest Warehouse"] = "PASSED"
    except Exception as e:
        print(f"[-] LỖI 2-SHOT AI: {e}")
        test_results["2-Shot Vision & Nearest Warehouse"] = f"FAILED: {e}"

    elapsed = time.time() - start_time
    print("\n" + "=" * 70)
    print("                   BÁO CÁO KẾT QUẢ KIỂM THỬ TỔNG HỢP")
    print("=" * 70)
    all_passed = True
    for module_name, status in test_results.items():
        symbol = "[OK]" if status == "PASSED" else "[FAIL]"
        print(f"  {symbol} {module_name.ljust(35)}: {status}")
        if status != "PASSED":
            all_passed = False
            
    print("-" * 70)
    print(f"  Thời gian hoàn thành: {elapsed:.2f}s")
    if all_passed:
        print("  => TOÀN BỘ 4 MODULE ĐẠT TIÊU CHUẨN 100% (READY FOR DEPLOYMENT)")
    else:
        print("  => CÓ MODULE BỊ LỖI, VUI LÒNG KIỂM TRA LẠI LOG")
    print("=" * 70)
    
    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())
