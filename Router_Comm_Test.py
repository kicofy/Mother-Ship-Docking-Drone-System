import asyncio
from mavsdk import System


async def read_n_samples_with_timeout(async_generator, num_samples, formatter, per_item_timeout_s):
    received_count = 0
    while received_count < num_samples:
        try:
            item = await asyncio.wait_for(async_generator.__anext__(), timeout=per_item_timeout_s)
        except asyncio.TimeoutError:
            print("⏱ 流读取超时，结束该流")
            break
        except StopAsyncIteration:
            break
        print("📥", formatter(item))
        received_count += 1


def format_battery(sample):
    remaining = getattr(sample, "remaining_percent", None)
    if remaining is None:
        return "电量: N/A"
    percent = remaining if remaining > 1.0 else remaining * 100.0
    return f"电量: {percent:.1f}%"


def format_position(sample):
    return (
        f"相对高: {sample.relative_altitude_m:.2f} m, "
        f"纬度: {sample.latitude_deg:.6f}, 经度: {sample.longitude_deg:.6f}"
    )


def format_gps(sample):
    return f"卫星数: {sample.num_satellites}, Fix: {sample.fix_type}"


def format_health(sample):
    return f"GPS OK: {sample.is_global_position_ok}, Home OK: {sample.is_home_position_ok}"


async def try_send_safe_requests(drone: System):
    try:
        await drone.telemetry.set_rate_battery(1.0)
        await drone.telemetry.set_rate_health(1.0)
        try:
            await drone.telemetry.set_rate_gps_info(1.0)
        except Exception:
            pass
        print("📤 已尝试设置遥测速率（安全请求，不改变姿态/位置）。")
    except Exception as e:
        print(f"⚠️ 遥测速率设置可能未生效: {e}")

    try:
        version = await drone.info.get_version()
        print(f"ℹ️ 版本信息: {version}")
    except Exception as e:
        print(f"⚠️ 版本信息未获取: {e}")


async def main():
    # 固定默认参数（仅依赖 asyncio 与 mavsdk）
    host = "127.0.0.1"
    port = 14540
    samples = 3
    timeout_s = 5.0
    connect_timeout_s = 10.0
    include_position = False

    system_address = f"udp://{host}:{port}"
    print(f"🔗 连接转发器: {system_address}")

    drone = System()
    await drone.connect(system_address=system_address)

    print("⏳ 等待无人机连接...")
    async def _wait_connected():
        async for state in drone.core.connection_state():
            if state.is_connected:
                return
    try:
        await asyncio.wait_for(_wait_connected(), timeout=connect_timeout_s)
        print("✅ 已通过转发器连接到无人机。")
    except asyncio.TimeoutError:
        print("❌ 连接超时，请确认转发器正在运行，且本脚本可达该端口。")
        return

    # 发送安全请求以验证下行链路
    await try_send_safe_requests(drone)

    print("📡 接收遥测样本（电量/GPS/健康）...")
    tasks = [
        asyncio.create_task(read_n_samples_with_timeout(
            drone.telemetry.battery(), samples, format_battery, timeout_s)),
        asyncio.create_task(read_n_samples_with_timeout(
            drone.telemetry.gps_info(), samples, format_gps, timeout_s)),
        asyncio.create_task(read_n_samples_with_timeout(
            drone.telemetry.health(), samples, format_health, timeout_s)),
    ]

    if include_position:
        print("🧭 已启用位置流。")
        tasks.append(asyncio.create_task(read_n_samples_with_timeout(
            drone.telemetry.position(), samples, format_position, timeout_s)))

    await asyncio.gather(*tasks, return_exceptions=True)

    print("✅ 通信测试完成（未解锁、未起飞）。")


if __name__ == "__main__":
    asyncio.run(main())

