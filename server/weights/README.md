# GÓI MÔ HÌNH THỊ GIÁC MÁY TÍNH HYBRID CV (SLAM_KLTN — AGV_VER2 WMS)
> Thư mục lưu trữ trọng số và thuật toán Computer Vision cho hệ thống AGV_Ver2 WMS.

---

## 1. CÁC TỆP TRỌNG SỐ & MODULE ĐÃ TÍCH HỢP

```
server/weights/
├── best.pth               # Trọng số MobileNetV3-Small phân loại 3 lớp hình học
├── predict.py             # Giao diện suy luận HybridPredictor (không dùng YOLO)
├── hybrid_classifier.py   # Động cơ hợp nhất MobileNetV3-Small + OpenCV Geometry
├── geometry_analyzer.py   # Trích xuất đặc trưng hình học (Độ tròn, góc nhọn, rect_extent, sống lưng)
├── preprocessing.py       # Tiền xử lý đệm và rà soát bề mặt trống
├── roi_extraction.py      # Định vị vùng quan tâm ROI đa/đơn vật thể và phát hiện khay trống bằng OpenCV
├── model_config.json      # Siêu tham số và cấu hình ngưỡng quyết định
├── class_names.json       # Danh mục 3 lớp: cylinder (0), rectangular_box (1), triangular_prism (2)
└── README.md
```

---

## 2. BẢNG ÁNH XẠ NHÃN AI VÀO KHO WMS AGV_VER2

| Lớp CV (`CV_DuAn`) | Kiểu dáng | Mã WMS | Tên sản phẩm WMS | Phân loại | Kho đích ưu tiên |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`cylinder`** (Class 0) | Hình trụ | `PKG_MECH` | Chi Tiết Bánh Răng & Khớp Nối | `MECHANICAL` | **Kho Kệ 2 C** |
| **`rectangular_box`** (Class 1) | Hộp chữ nhật | `PKG_ELEC` | Bo Mạch Vi Điều Khiển | `ELECTRONIC` | **Kho Kệ 1 B** |
| **`triangular_prism`** (Class 2) | Lăng trụ tam giác | `PKG_TOOL` | Bộ Dụng Cụ Đo & Kiểm Thử | `TOOL` | **Kho Kệ 3 D** |

---

## 3. CỔNG AN TOÀN (SAFETY GATES)
1. **Khay trống (`NO_OBJECT`)**: Tự động phát hiện khay rỗng (nền đen hoặc sáng không có vân), từ chối tạo nhiệm vụ chuyển hàng rỗng, dừng AGV an toàn.
2. **Cần kiểm tra (`NEEDS_REVIEW`)**: Khi độ tin cậy $< 0.50$ hoặc phát hiện nhiều vật thể mâu thuẫn, máy trạng thái FSM chuyển sang `REJECTED_UNKNOWN` và yêu cầu xác nhận.
3. **Thành công (`CLASSIFIED`)**: Tự động điều phối AGV theo thuật toán Dijkstra và vẽ Bounding Box lên ảnh bằng chứng.
