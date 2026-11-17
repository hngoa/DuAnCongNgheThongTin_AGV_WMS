"""
===================================================================================
QUẢN LÝ KẾT NỐI DATABASE SQLite THREAD-SAFE (AGV_VER2 WMS)
===================================================================================
"""

import sqlite3
from pathlib import Path
from contextlib import contextmanager

# Đường dẫn mặc định đến file database và file schema
CURRENT_DIR = Path(__file__).resolve().parent
DB_PATH = CURRENT_DIR / "wms_database.db"
SCHEMA_PATH = CURRENT_DIR / "schema.sql"

_wal_initialized_files = set()

def get_connection(db_file: Path = DB_PATH) -> sqlite3.Connection:
    """
    Tạo kết nối SQLite tối ưu cho ứng dụng IoT / Web:
      - Bật Foreign Keys (Ràng buộc toàn vẹn quan hệ)
      - Bật chế độ WAL (Write-Ahead Logging) 1 lần duy nhất cho mỗi file DB, tránh nghẽn khóa I/O
      - Row factory trả về đối tượng giống dict để dễ serialize JSON
    """
    db_str = str(db_file)
    conn = sqlite3.connect(db_str, timeout=10.0, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys = ON;")
    if db_str not in _wal_initialized_files:
        conn.execute("PRAGMA journal_mode = WAL;")
        _wal_initialized_files.add(db_str)
    conn.execute("PRAGMA synchronous = NORMAL;")
    return conn

@contextmanager
def get_db(db_file: Path = DB_PATH):
    """Context Manager cho phép dùng cú pháp 'with get_db() as db:' an toàn"""
    conn = get_connection(db_file)
    try:
        yield conn
        conn.commit()
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()

def init_db(db_file: Path = DB_PATH, schema_file: Path = SCHEMA_PATH) -> bool:
    """Khởi tạo toàn bộ các bảng và chỉ mục từ schema.sql"""
    if not schema_file.is_file():
        raise FileNotFoundError(f"Không tìm thấy file schema tại: {schema_file}")
    
    with open(schema_file, "r", encoding="utf-8") as f:
        schema_sql = f.read()

    with get_db(db_file) as conn:
        conn.executescript(schema_sql)
    
    print(f"✅ Đã khởi tạo cấu trúc Database thành công tại: {db_file}")
    return True
