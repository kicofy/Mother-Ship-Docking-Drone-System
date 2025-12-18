import argparse
import time
from typing import Optional

from pymavlink import mavutil


def wait_heartbeat_with_keepalive(conn: mavutil.mavfile, timeout_s: float, keepalive_hz: float = 2.0) -> bool:
    """
    等待飞控心跳；期间持续发送 GCS 心跳作为 keepalive，确保转发器记录本客户端地址后回传数据。
    """
    deadline = time.time() + timeout_s
    period = 1.0 / max(keepalive_hz, 0.1)
    next_keepalive = 0.0
    while time.time() < deadline:
        now = time.time()
        if now >= next_keepalive:
            send_gcs_heartbeat(conn)
            next_keepalive = now + period
        try:
            msg = conn.recv_match(type="HEARTBEAT", blocking=False)
        except Exception:
            msg = None
        if msg is not None:
            sid = getattr(msg, "get_srcSystem", lambda: conn.target_system)()
            cid = getattr(msg, "get_srcComponent", lambda: conn.target_component)()
            print(f"✅ 收到飞控心跳 sys={sid} comp={cid}")
            return True
        time.sleep(0.02)
    print("⌛ 未在超时时间内收到心跳")
    return False


def send_gcs_heartbeat(conn: mavutil.mavfile) -> None:
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,  # base_mode
        0,  # custom_mode
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    # 立即flush，确保发送
    try:
        conn.mav.flush()
    except Exception:
        pass


def request_autopilot_version(conn: mavutil.mavfile) -> None:
    target_sys = conn.target_system or 1
    target_comp = conn.target_component or 1
    conn.mav.command_long_send(
        target_sys,
        target_comp,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    try:
        conn.mav.flush()
    except Exception:
        pass
    print("📤 已请求 AUTOPILOT_VERSION（安全指令，仅用于通信验证）。")


def main():
    parser = argparse.ArgumentParser(description="使用 pymavlink 测试通过 serial_px4_udp_router.py 的双向通信")
    parser.add_argument("--host", default="127.0.0.1", help="转发器主机，默认 127.0.0.1")
    parser.add_argument("--port", type=int, default=14540, help="转发器 UDP 端口，默认 14540")
    parser.add_argument("--listen-seconds", type=float, default=5.0, help="接收窗口时长（秒）")
    parser.add_argument("--print-params", type=int, default=0, help="可选：打印前N个 PARAM_VALUE 用于验证下行")
    args = parser.parse_args()

    # 以 udpout 方式连到转发器（发送到转发器绑定端口，同时接收回传）
    url = f"udpout:{args.host}:{args.port}"
    print(f"🔗 连接: {url}")
    conn = mavutil.mavlink_connection(url, autoreconnect=True)

    # 先持续发送 GCS 心跳，促使转发器登记本客户端，再等待飞控心跳
    if not wait_heartbeat_with_keepalive(conn, timeout_s=10.0, keepalive_hz=2.0):
        print("❌ 未能收到心跳，请确认转发器与串口链路是否正常。")
        return
    print("📤 keepalive GCS 心跳已发送并收到对端心跳响应。")

    # 请求一次 AUTOPILOT_VERSION，验证“发送后收到响应”
    request_autopilot_version(conn)

    printed_params = 0
    end_ts = time.time() + args.listen_seconds
    print("📡 开始接收窗口...")
    while time.time() < end_ts:
        try:
            msg = conn.recv_match(blocking=False)
        except Exception:
            msg = None
        if msg is None:
            time.sleep(0.02)
            continue

        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            continue

        if mtype == "HEARTBEAT":
            print(f"📥 HEARTBEAT from sys={msg.get_srcSystem()} comp={msg.get_srcComponent()}")
        elif mtype == "AUTOPILOT_VERSION":
            # 仅打印一次概要
            flight_sw = getattr(msg, "flight_sw_version", None)
            middleware_sw = getattr(msg, "middleware_sw_version", None)
            os_sw = getattr(msg, "os_sw_version", None)
            print(f"📥 AUTOPILOT_VERSION flight={flight_sw} middleware={middleware_sw} os={os_sw}")
        elif mtype == "STATUSTEXT":
            print(f"📥 STATUSTEXT[{msg.severity}]: {msg.text}")
        elif mtype == "PARAM_VALUE" and args.print_params > 0 and printed_params < args.print_params:
            print(f"📥 PARAM {msg.param_id} = {msg.param_value} ({msg.param_type}) idx={msg.param_index}/{msg.param_count}")
            printed_params += 1

    print("✅ 通信测试完成。")


if __name__ == "__main__":
    main()

