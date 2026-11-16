"""
===================================================================================
MÁY TRẠNG THÁI VÒNG ĐỜI NHIỆM VỤ (TASK LIFECYCLE FSM)
QUẢN LÝ QUÁ TRÌNH TỪ KHI CHỤP ẢNH, AI PHÂN LOẠI, ĐIỀU PHỐI ĐẾN TĂNG KHO
===================================================================================
"""

from typing import Optional, Dict, Any
from datetime import datetime
import time
try:
    from .fsm_definitions import TaskState
except ImportError:
    from fsm_definitions import TaskState

class TaskStateMachine:
    """Quản lý các bước chuyển trạng thái và tính toàn vẹn của Task"""
    def __init__(self, task_repo, inv_repo, map_repo):
        self.task_repo = task_repo
        self.inv_repo = inv_repo
        self.map_repo = map_repo

    def create_pending_task(self, map_id: str, product_code: str, product_name: str,
                            from_node: str, ai_confidence: float, image_path: str,
                            assigned_agv: str = "AGV_Ver2") -> int:
        """Tạo task ở trạng thái ban đầu PENDING"""
        return self.task_repo.create_task(
            map_id=map_id,
            product_code=product_code,
            product_name=product_name,
            from_node=from_node,
            to_node="UNKNOWN", # Sẽ được cấp sau khi qua Allocation Engine
            route_path="PENDING_ALLOCATION",
            ai_confidence=ai_confidence,
            image_path=image_path,
            assigned_agv=assigned_agv
        )

    def transition_to_dispatched(self, task_id: int, to_node: str, route_str: str,
                                 assigned_agv: Optional[str] = None) -> bool:
        """Chuyển sang DISPATCHED khi Allocation Engine đã cấp kho đích và Router đã lập lộ trình"""
        task = self.task_repo.get_by_id(task_id)
        if not task or task["status"] not in (TaskState.PENDING.value, TaskState.DISPATCHED.value):
            return False
        return self.task_repo.update_dispatch_target(task_id, to_node, route_str, assigned_agv)

    def transition_to_in_transit(self, task_id: int) -> bool:
        """Chuyển sang IN_TRANSIT khi xe AGV đã phản hồi ACK nhận lệnh"""
        task = self.task_repo.get_by_id(task_id)
        if not task or task["status"] != TaskState.DISPATCHED.value:
            return False
        return self.task_repo.update_status(task_id, TaskState.IN_TRANSIT.value)

    def transition_to_delivered(self, task_id: int) -> bool:
        """Chuyển sang DELIVERED khi xe AGV báo đã dừng chính xác tại mỏ neo kho đích"""
        task = self.task_repo.get_by_id(task_id)
        if not task or task["status"] not in (TaskState.IN_TRANSIT.value, TaskState.DISPATCHED.value):
            return False
        return self.task_repo.update_status(task_id, TaskState.DELIVERED.value)

    def transition_to_completed(self, task_id: int) -> bool:
        """Hết thời gian đỗ dỡ hàng (Dwell Time) -> Chuyển COMPLETED và TĂNG TỒN KHO (+1)"""
        task = self.task_repo.get_by_id(task_id)
        if not task or task["status"] not in (TaskState.DELIVERED.value, TaskState.IN_TRANSIT.value):
            return False
        
        map_id = task["map_id"]
        to_node = task["to_node"]
        product_code = task["product_code"]

        # 1. Cập nhật trạng thái Task thành COMPLETED
        self.task_repo.update_status(task_id, TaskState.COMPLETED.value)

        # 2. Tự động tăng tồn kho mặt hàng này tại trạm đích (+1)
        self.inv_repo.update_stock(map_id, to_node, product_code, delta=1)

        return True

    def transition_to_rejected(self, task_id: int, reason: str = "UNKNOWN_PRODUCT") -> bool:
        """Từ chối nhiệm vụ do AI không nhận diện được hoặc kho đã đầy"""
        new_state = TaskState.REJECTED_UNKNOWN.value if "UNKNOWN" in reason else TaskState.REJECTED_STOCK_FULL.value
        return self.task_repo.update_status(task_id, new_state, error_msg=reason)

    def transition_to_aborted(self, task_id: int, reason: str = "SAFETY_STOP") -> bool:
        """Hủy nhiệm vụ do sự cố an toàn (lạc vạch, pin yếu)"""
        return self.task_repo.update_status(task_id, TaskState.ABORTED.value, error_msg=reason)
