import argparse
import socket
import threading
import time
from typing import Optional, Tuple, Dict, Set

from serial.tools import list_ports
from pymavlink import mavutil


def detect_px4_serial(preferred_bauds: Tuple[int, ...], heartbeat_timeout_s: float) -> Tuple[str, int]:
    candidates = list(list_ports.comports())
    if not candidates:
        raise RuntimeError("未找到任何串口。请连接飞控后重试。")

    # 尝试所有串口与波特率组合，直到收到 PX4 心跳
    for port in candidates:
        for baud in preferred_bauds:
            try:
                conn = mavutil.mavlink_connection(device=port.device, baud=baud, autoreconnect=False)
                hb = conn.wait_heartbeat(timeout=heartbeat_timeout_s)
                conn.close()
                if hb and getattr(hb, "autopilot", None) == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                    return port.device, baud
            except Exception:
                continue
    raise RuntimeError("未检测到 PX4 心跳。请确认连接与波特率。")


class UdpPeers:
    def __init__(self, ttl_s: float = 120.0):
        self._peers_last_seen: Dict[Tuple[str, int], float] = {}
        self._lock = threading.Lock()
        self._ttl_s = ttl_s

    def update(self, addr: Tuple[str, int]) -> None:
        with self._lock:
            self._peers_last_seen[addr] = time.time()

    def all(self) -> Set[Tuple[str, int]]:
        now = time.time()
        with self._lock:
            # 清理过期节点
            expired = [a for a, ts in self._peers_last_seen.items() if (now - ts) > self._ttl_s]
            for a in expired:
                self._peers_last_seen.pop(a, None)
            return set(self._peers_last_seen.keys())


def serial_to_udp_loop(conn: mavutil.mavfile, udp_sock: socket.socket, peers: UdpPeers, stop: threading.Event):
    while not stop.is_set():
        try:
            msg = conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            buf = msg.get_msgbuf()
            if not buf:
                continue
            for addr in peers.all():
                try:
                    udp_sock.sendto(buf, addr)
                except Exception:
                    pass
        except Exception:
            # 轻量容错，继续尝试
            time.sleep(0.1)


def udp_to_serial_loop(conn: mavutil.mavfile, udp_sock: socket.socket, peers: UdpPeers, stop: threading.Event):
    udp_sock.settimeout(1.0)
    while not stop.is_set():
        try:
            data, addr = udp_sock.recvfrom(65535)
            if not data:
                continue
            peers.update(addr)
            try:
                # 直接将 UDP 载荷（MAVLink 帧）写入串口
                conn.port.write(data)
            except Exception:
                pass
        except socket.timeout:
            continue
        except Exception:
            time.sleep(0.1)


def open_serial_for_forwarding(device: str, baud: int) -> mavutil.mavfile:
    # 使用 pymavlink 打开串口，便于读消息并可直接写入底层串口
    conn = mavutil.mavlink_connection(device=device, baud=baud, autoreconnect=True)
    return conn


def main():
    parser = argparse.ArgumentParser(description="自动检测串口 PX4 并双向转发 MAVLink 到 UDP 端口")
    parser.add_argument("--bauds", default="921600,57600,115200", help="检测波特率列表，逗号分隔")
    parser.add_argument("--heartbeat-timeout", type=float, default=2.0, help="串口心跳检测超时（秒）")
    parser.add_argument("--bind", default="0.0.0.0", help="UDP 绑定地址")
    parser.add_argument("--port", type=int, default=14540, help="UDP 监听端口（MavSDK 常用 14540）")
    args = parser.parse_args()

    baud_list = tuple(int(x.strip()) for x in args.bauds.split(",") if x.strip())
    print(f"🔎 自动检测 PX4 串口，波特率候选: {baud_list}")
    device, baud = detect_px4_serial(preferred_bauds=baud_list, heartbeat_timeout_s=args.heartbeat_timeout)
    print(f"✅ 已检测到 PX4: {device} @ {baud}")

    conn = open_serial_for_forwarding(device, baud)

    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_sock.bind((args.bind, args.port))
    print(f"🌐 UDP 已绑定在 {args.bind}:{args.port}，等待客户端（QGC/MavSDK）...")

    peers = UdpPeers(ttl_s=120.0)
    stop = threading.Event()

    t1 = threading.Thread(target=serial_to_udp_loop, args=(conn, udp_sock, peers, stop), daemon=True)
    t2 = threading.Thread(target=udp_to_serial_loop, args=(conn, udp_sock, peers, stop), daemon=True)
    t1.start()
    t2.start()

    print("🔁 开始转发（Ctrl+C 退出）")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n🛑 停止中...")
        stop.set()
        t1.join(timeout=2.0)
        t2.join(timeout=2.0)
        try:
            conn.close()
        except Exception:
            pass
        try:
            udp_sock.close()
        except Exception:
            pass
        print("✅ 已退出。")


if __name__ == "__main__":
    main()

