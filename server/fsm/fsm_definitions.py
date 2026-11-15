"""
===================================================================================
ĐỊNH NGHĨA CÁC ENUM TRẠNG THÁI CHO HỆ THỐNG AGV_VER2 WMS (FSM V2.1)
===================================================================================
"""

from enum import Enum

class TaskState(str, Enum):
    """Trạng thái vòng đời của một nhiệm vụ vận chuyển (Dispatch Task)"""
    PENDING = "PENDING"                           # Mới khởi tạo sau khi nhận ảnh từ điện thoại
    ALLOCATING = "ALLOCATING"                     # Đang tra cứu quy tắc cấp kho đích
    DISPATCHED = "DISPATCHED"                     # Đã lập lộ trình, đang chờ xe nhận lệnh
    IN_TRANSIT = "IN_TRANSIT"                     # Xe đã gửi ACK và đang chạy trên đường line
    DELIVERED = "DELIVERED"                       # Xe đã đến mỏ neo trạm đích an toàn
    UNLOADING = "UNLOADING"                       # Đang dừng đỗ dỡ hàng (Dwell time 5.0s)
    COMPLETED = "COMPLETED"                       # Đã hoàn tất, số lượng tồn kho đã tăng +1
    REJECTED_UNKNOWN = "REJECTED_UNKNOWN"         # AI không nhận diện được sản phẩm (Conf < 0.50)
    REJECTED_STOCK_FULL = "REJECTED_STOCK_FULL"   # Các kho phù hợp đều đã đầy chỗ
    ABORTED = "ABORTED"                           # Bị hủy do sự cố (Lạc line, Pin yếu, Lỗi khẩn cấp)

class AGVState(str, Enum):
    """Trạng thái vận hành và an toàn của xe AGV (ESP32-S3)"""
    OFFLINE = "OFFLINE"                           # Mất tín hiệu quá 2.5s
    IDLE = "IDLE"                                 # Đang đỗ chờ việc, sẵn sàng nhận lệnh
    STARTING = "STARTING"                         # Khởi động mềm động cơ, khóa góc Yaw
    LINE_FOLLOWING = "LINE_FOLLOWING"             # Đang bám vạch line bằng thuật toán PD
    GAP_CROSSING = "GAP_CROSSING"                 # Mất vạch, đang chạy Dead Reckoning quán tính
    DECELERATING = "DECELERATING"                 # Cách đích <= 35cm, đang giảm tốc mềm
    AT_STATION = "AT_STATION"                     # Đã dừng chính xác tại vạch mỏ neo trạm
    UNLOADING = "UNLOADING"                       # Giữ phanh cứng chờ dỡ hàng
    RETURNING_HOME = "RETURNING_HOME"             # Tự động quay về trạm A sau khi dỡ hàng
    LOW_BATTERY = "LOW_BATTERY"                   # Pin yếu < 10.5V, từ chối nhận việc mới
    EMERGENCY_STOP = "EMERGENCY_STOP"             # Phanh dừng khẩn cấp (Lạc line quá 35cm / E-Stop)

class SegmentState(str, Enum):
    """Trạng thái đoạn line sa bàn (Traffic & Zone Locking)"""
    UNLOCKED = "UNLOCKED"                         # Đoạn line tự do, mọi xe có thể đăng ký
    RESERVED = "RESERVED"                         # Đã được cấp cho xe chuẩn bị chạy vào
    ACTIVE_OCCUPIED = "ACTIVE_OCCUPIED"           # Xe đang thực sự lăn bánh trên đoạn line
    EMERGENCY_LOCKED = "EMERGENCY_LOCKED"         # Bị phong tỏa do sự cố xe đỗ chết trên đường
