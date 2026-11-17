"""
===================================================================================
DỊCH VỤ TÊN MIỀN CỤC BỘ mDNS / ZEROCONF (MULTICAST DNS RFC 6762)
CHO PHÉP TRUY CẬP SERVER QUA: http://agv-wms.local:5000
KHÔNG CẦN NHỚ HAY SỬA IP, KHÔNG PHỤ THUỘC VÀO THƯ VIỆN NGOÀI (ZERO-DEPENDENCY)
===================================================================================
"""

import socket
import struct
import threading
import time
from app.config import get_local_ips, MDNS_HOSTNAME, PORT

MDNS_GROUP = "224.0.0.251"
MDNS_PORT = 5353

class MDNSService:
    """
    Bộ phản hồi mDNS chuẩn RFC 6762 chạy hoàn toàn bằng Python socket chuẩn:
    - Lắng nghe yêu cầu phân giải tên miền 'agv-wms.local' trên cổng 5353
    - Tự động phản hồi bản ghi DNS Type A chứa IP Wi-Fi hiện tại của máy tính
    - Phát gói tin thông báo ban đầu (Gratuitous Announcement) để router cập nhật cache
    """
    def __init__(self, hostname: str = MDNS_HOSTNAME, port: int = PORT):
        self.hostname = hostname.lower()
        self.domain_name = f"{self.hostname}.local"
        self.port = port
        self.running = False
        self._thread = None
        self._sock = None

    def start(self):
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        print(f"🌐 [mDNS Service] Đã kích hoạt tên miền cục bộ: http://{self.domain_name}:{self.port}")

    def stop(self):
        self.running = False
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass

    def _get_primary_ip(self) -> str:
        local_ips = get_local_ips()
        wifi_ips = [ip for ip in local_ips if not ip.startswith("26.") and not ip.startswith("127.")]
        return wifi_ips[0] if wifi_ips else (local_ips[0] if local_ips else "127.0.0.1")

    def _build_dns_wire_name(self, name: str) -> bytes:
        parts = name.strip(".").split(".")
        out = b""
        for p in parts:
            b = p.encode("ascii")
            out += bytes([len(b)]) + b
        return out + b"\x00"

    def _build_response_packet(self, ip_str: str, query_id: int = 0) -> bytes:
        """Tạo gói tin phản hồi DNS chuẩn RFC 6762 (Authoritative Answer A Record)"""
        # Header: ID=query_id, Flags=0x8400 (Response, Authoritative), QDCOUNT=0, ANCOUNT=1, NSCOUNT=0, ARCOUNT=0
        header = struct.pack("!HHHHHH", query_id, 0x8400, 0, 1, 0, 0)
        wire_name = self._build_dns_wire_name(self.domain_name)
        ip_bytes = socket.inet_aton(ip_str)
        # Type A (1), Class IN (1) | Cache-Flush (0x8000) = 0x8001, TTL=120s, RDLENGTH=4, RDATA=IP
        answer_rr = wire_name + struct.pack("!HHIH", 1, 0x8001, 120, 4) + ip_bytes
        return header + answer_rr

    def _run_loop(self):
        # 1. Khởi tạo UDP Socket lắng nghe Multicast trên cổng 5353
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # Gắn socket vào cổng 5353
            self._sock.bind(("", MDNS_PORT))

            # Vô hiệu hóa loopback để socket không tự nhận lại các gói tin multicast của chính nó
            try:
                self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
            except Exception:
                pass

            # Thiết lập phát multicast qua card Wi-Fi chính (tránh bị Radmin VPN chiếm quyền)
            primary_ip = self._get_primary_ip()
            try:
                self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(primary_ip))
            except Exception:
                pass

            # Tham gia nhóm Multicast 224.0.0.251 trên tất cả các IP LAN (Wi-Fi)
            for ip in get_local_ips():
                if not ip.startswith("127."):
                    try:
                        mreq = socket.inet_aton(MDNS_GROUP) + socket.inet_aton(ip)
                        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                    except Exception:
                        pass

            # Dự phòng tham gia mặc định
            try:
                mreq_any = struct.pack("4sl", socket.inet_aton(MDNS_GROUP), socket.INADDR_ANY)
                self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq_any)
            except Exception:
                pass

            self._sock.settimeout(1.5)
        except Exception as e:
            print(f"⚠️ [mDNS Service] Không thể liên kết cổng 5353 ({e}). Đang dùng chế độ phát song song.")
            return

        target_bytes = self._build_dns_wire_name(self.domain_name)
        target_name_lower = self.domain_name.encode("ascii")

        # 2. Phát 2 gói tin công bố ban đầu (Gratuitous Announcement)
        primary_ip = self._get_primary_ip()
        announcement = self._build_response_packet(primary_ip)
        for _ in range(2):
            try:
                self._sock.sendto(announcement, (MDNS_GROUP, MDNS_PORT))
            except Exception:
                pass
            time.sleep(0.3)

        # 3. Vòng lặp phản hồi truy vấn từ ESP32, Android và máy tính trong LAN
        last_respond_time = {}
        last_announce_time = time.time()
        while self.running:
            try:
                # Định kỳ phát gói Gratuitous Announcement mỗi 3 giây để ESP32 luôn cập nhật cache mDNS
                # ngay cả khi Windows Dnscache chiếm quyền nhận gói truy vấn trên cổng 5353
                if time.time() - last_announce_time >= 3.0:
                    last_announce_time = time.time()
                    current_ip = self._get_primary_ip()
                    ann_pkt = self._build_response_packet(current_ip, query_id=0)
                    try:
                        self._sock.sendto(ann_pkt, (MDNS_GROUP, MDNS_PORT))
                    except Exception:
                        pass

                data, addr = self._sock.recvfrom(2048)
                if not data or len(data) < 12:
                    continue

                # Chỉ xử lý GÓI TIN TRUY VẤN (Query: QR bit = 0, QDCOUNT > 0)
                # Tuyệt đối bỏ qua gói phản hồi (Response: QR bit = 1) để tránh bão multicast phản xạ lặp vô hạn
                flags, qdcount = struct.unpack("!HH", data[2:6])
                is_response = (flags & 0x8000) != 0
                if is_response or qdcount == 0:
                    continue

                # Kiểm tra xem gói tin có hỏi về 'agv-wms' hoặc 'agv-wms.local' không
                if target_name_lower in data.lower() or target_bytes in data or self.hostname.encode("ascii") in data.lower():
                    # Chống dội gói tin lặp từ cùng 1 IP trong khoảng 300ms
                    client_ip = addr[0]
                    now = time.time()
                    if client_ip in last_respond_time and (now - last_respond_time[client_ip]) < 0.3:
                        continue
                    last_respond_time[client_ip] = now

                    query_id = struct.unpack("!H", data[:2])[0]
                    current_ip = self._get_primary_ip()
                    
                    # 1. Gói unicast giữ nguyên Transaction ID của thiết bị gửi
                    resp_unicast = self._build_response_packet(current_ip, query_id=query_id)
                    try:
                        self._sock.sendto(resp_unicast, addr)
                    except Exception:
                        pass
                    
                    # 2. Gói multicast cho toàn mạng (ID = 0 theo chuẩn RFC 6762)
                    resp_multicast = self._build_response_packet(current_ip, query_id=0)
                    try:
                        self._sock.sendto(resp_multicast, (MDNS_GROUP, MDNS_PORT))
                    except Exception:
                        pass
                    
                    print(f"🎯 [mDNS Service] Phân giải thành công 'agv-wms.local' cho {addr[0]} -> IP: {current_ip}")
            except socket.timeout:
                continue
            except Exception:
                if not self.running:
                    break
