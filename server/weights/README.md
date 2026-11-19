# AI Vision Engine — CV Ver3 (YOLO11n + MobileNetV3-Small + OpenCV Geometric Reasoning)

Thư mục `server/weights/` chứa toàn bộ mô-đun và trọng số thị giác **CV Ver3** đồng bộ từ `Ver3_CV` cho hệ thống `AGV_Ver2`.

## Cấu trúc thư mục
```text
server/weights/
├── slam_cv_recognizer.py   # Engine CV Ver3 hợp nhất (SLAMCVRecognizer, HybridObjectClassifier, GeometricAnalyzer)
├── yolo11n_v5b_best.pt     # Trọng số YOLO11n định vị đa vật thể trên khay hàng (Stage 1)
├── best.pth                # Trọng số MobileNetV3-Small phân loại 3 hình khối (Stage 2)
├── class_names.json        # Ánh xạ ID -> nhãn lớp chuẩn
└── README.md               # Tài liệu cấu trúc CV Ver3
```

## Kiến trúc xử lý ảnh CV Ver3 (3 giai đoạn)
1. **Stage 1 — Định vị đối tượng (`YOLO11n` — `yolo11n_v5b_best.pt`)**:
   - Phát hiện đồng thời 1 hoặc nhiều vật thể trên khay hàng (`agnostic_nms=True`, `iou=0.45`, `conf_threshold=0.05`).
   - Áp dụng tách box gộp màu (`_split_merged_box_by_color`), giải quyết box lồng nhau (`_resolve_nested_boxes`) và lọc trùng khớp trên khay (`_prune_duplicate_classes_in_tray`).
2. **Stage 2 — Phân loại & Cân bằng Hình học (`MobileNetV3-Small` — `best.pth` + `GeometricAnalyzer`)**:
   - Crop từng ROI (mặc định `pad_ratio=0.0` trong `recognize_scene`), chuẩn hóa kích thước 224x224 với viền letterbox RGB `(114, 114, 114)` và chuẩn hóa ImageNet.
   - `GeometricAnalyzer` trích xuất circularity, extent bất biến xoay (`cv2.minAreaRect`), góc đa giác (`approxPolyDP`), đường thẳng HoughLinesP và sống lưng apex ridge để xác nhận hoặc hiệu chỉnh các trường hợp hình khối dễ nhầm lẫn.
   - `_reconcile_scene_objects` kiểm tra tính nhất quán toàn cục khi khung hình có 2 hoặc 3 vật thể.
3. **Stage 3 — Tích hợp WMS (`server/app/services/ai_service.py`)**:
   - Đọc ảnh từ camera qua PIL + hiệu chỉnh hướng EXIF (`exif_transpose`) -> chuyển OpenCV **BGR** (`uint8`).
   - Gọi `SLAMCVRecognizer.recognize_scene(img_bgr)` lấy `detected_objects`, `vis_image`, `latency_ms`.
   - Kiểm tra Cổng an toàn (`NO_OBJECT` khi không có vật thể, `NEEDS_REVIEW` khi độ tin cậy `< 0.50` hoặc lớp không hợp lệ) và tra cứu quy tắc kho động từ SQLite (`products`, `allocation_rules`).

## Ánh xạ Class ID chuẩn
| ID (`class_names.json`) | Nhãn CV | Mã hàng mặc định (DB) | Trạm kho mặc định (Map `SABAN_154_151`) |
| :---: | :--- | :--- | :--- |
| `0` | `cylinder` (Trụ tròn) | `PKG_MECH` | Kho `C` |
| `1` | `rectangular_box` (Hộp chữ nhật) | `PKG_ELEC` | Kho `B` |
| `2` | `triangular_prism` (Lăng trụ tam giác) | `PKG_TOOL` | Kho `D` |

## Cài đặt thư viện cần thiết
```bash
pip install -r server/requirements.txt
```
*(Nếu thiếu `torch` / `ultralytics` / `cv2` hoặc thiếu file trọng số, `ai_service.py` sẽ tự động chuyển về `Placeholder_Mode` / Heuristic Fallback phục vụ chạy server & test không có GPU/ML libs.)*
