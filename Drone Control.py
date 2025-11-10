import asyncio
from mavsdk import System

async def main():
    print("🔗 正在连接 QGroundControl 转发端口 (udpin://0.0.0.0:14445)...")
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14445")

    print("⏳ 等待无人机连接...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ 已连接到无人机！")
            break

    # 等待 GPS / Home 位置锁定
    print("📡 等待 GPS 和 Home 锁定...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✅ GPS 和 Home 已锁定，可以起飞。")
            break

    # 解锁并起飞
    print("🛫 解锁 (Arm)...")
    await drone.action.arm()

    print("🚀 起飞至 5 米高度...")
    await drone.action.takeoff()

    await asyncio.sleep(8)  # 等待爬升到高度

    # 打印一次当前位置
    async for pos in drone.telemetry.position():
        print(f"📍 当前高度: {pos.relative_altitude_m:.2f} m")
        break

    # 悬停 5 秒
    print("⏸ 悬停 5 秒...")
    await asyncio.sleep(5)

    # 降落
    print("🛬 开始降落...")
    await drone.action.land()

    await asyncio.sleep(10)
    print("✅ 任务完成。")

asyncio.run(main())
