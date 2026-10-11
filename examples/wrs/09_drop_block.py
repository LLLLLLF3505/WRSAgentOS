r"""方块自由下落到桌面。独立运行，不连接 AgentOS 节点。

复制到项目 examples/wrs/ 后运行：
    .\scripts\run.ps1 examples/wrs/09_drop_block.py
无窗口验证：在命令末尾添加 --check。
"""

import argparse
import json
import time

from wrs_agent.env.wrs import load_wrs


DT = 0.02               # 每次推进 0.02 秒的物理时间
BLOCK_SIZE = 0.06       # 方块边长 6 厘米
START_HEIGHT = 0.40     # 初始中心高度 40 厘米
VIEWER_PORT = 8001      # 与原场景的 8000 端口分开


def build_scene(wrs, scene):
    """创建固定桌面和可运动方块，然后把场景交给 MuJoCo。"""
    from wrs.physics.mj_env import MJEnv

    table = wrs.wssop.box(
        name="drop_table",
        pos=(0.0, 0.0, -0.03),
        xyz_lengths=(0.8, 0.6, 0.06),  # 桌面上表面正好位于 Z=0
        rgb=(0.72, 0.52, 0.30),
        collision_type=wrs.wuc.CollisionType.OBB,
        is_floating=False,             # 固定，不能被重力带走
    )
    table.add_to_scene(scene)

    block = wrs.wssop.box(
        name="falling_block",
        pos=(0.0, 0.0, START_HEIGHT),
        xyz_lengths=(BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE),
        rgb=(0.95, 0.95, 0.95),
        mass=0.10,                     # 质量：0.1 千克
        collision_type=wrs.wuc.CollisionType.OBB,
        is_floating=True,              # 自由物体，允许物理引擎改变位姿
    )
    block.add_to_scene(scene)
    physics = MJEnv(scene=scene)        # 默认重力为 [0, 0, -9.81]
    # 缩短接触响应时间，减少方块撞击桌面时的数值穿透。
    physics.model.geom_solref[:, 0] = 0.006
    return table, block, physics


def check_physics(wrs):
    """运行真实物理计算，检查下落、接触和静止；无需浏览器。"""
    import mujoco

    table, block, physics = build_scene(wrs, wrs.wss.Scene())
    start_qpos = physics.data.qpos.copy()
    reports = []
    for trial in range(2):
        # 重置同一个仿真，验证可重复播放。
        mujoco.mj_resetData(physics.model, physics.data)
        physics.data.qpos[:] = start_qpos
        physics.runtime.forward()
        physics.sync.pull_all_sobj_pose()
        heights = []
        for _ in range(150):           # 3 秒仿真时间，不需要真的等 3 秒
            physics.step(DT)
            heights.append(float(block.pos[2]))
        assert abs(heights[4] - (START_HEIGHT - 0.5 * 9.81 * 0.1**2)) < 0.003
        assert abs(heights[-1] - BLOCK_SIZE / 2) < 0.001
        assert min(heights) > BLOCK_SIZE / 2 - 0.01, min(heights)
        assert abs(float(table.pos[2]) + 0.03) < 1e-6
        assert physics.data.ncon > 0
        assert float(wrs.np.linalg.norm(physics.data.qvel)) < 0.001
        reports.append({"trial": trial + 1, "height_at_0.1s": heights[4],
                        "final_center_height": heights[-1], "minimum_height": min(heights),
                        "contacts": int(physics.data.ncon)})
    print(json.dumps({"result": "PASS", "gravity": physics.model.opt.gravity.tolist(),
                      "checks": reports}, indent=2))


def show_demo(wrs):
    """每轮先悬停 2 秒，再自由下落；每 6 秒重新演示。"""
    import mujoco
    from wrs_agent.env.viewer import viewer_hub

    world = wrs.wvw.World(
        cam_pos=(0.9, 1.0, 0.8), cam_lookat_pos=(0.0, 0.0, 0.15),
        port=VIEWER_PORT, hz=50, auto_start_hub=False,
    )
    try:
        world.set_caption("WRS + MuJoCo: falling block")
        _, block, physics = build_scene(wrs, world.scene)
        initial_qpos = physics.data.qpos.copy()
        cycle_start = time.monotonic()
        reported = False

        def update(dt):
            nonlocal cycle_start, reported
            elapsed = time.monotonic() - cycle_start
            if elapsed >= 6.0:
                mujoco.mj_resetData(physics.model, physics.data)
                physics.data.qpos[:] = initial_qpos
                physics.runtime.forward()
                physics.sync.pull_all_sobj_pose()
                physics.contact_viz.update_from_data(physics.model, physics.data)
                cycle_start, reported = time.monotonic(), False
            elif elapsed >= 2.0:
                physics.step(DT)       # MuJoCo 求解重力与接触，随后同步回 WRS
                if not reported and physics.data.time >= 2.0:
                    print(f"方块中心高度：{float(block.pos[2]):.4f} 米；"
                          f"接触数：{physics.data.ncon}", flush=True)
                    reported = True

        world.schedule_interval(update, DT)
        with viewer_hub(VIEWER_PORT):
            print(f"打开 http://127.0.0.1:{VIEWER_PORT} 查看。", flush=True)
            print("每 6 秒重置一次；悬停 2 秒后释放。Ctrl+C 退出。", flush=True)
            world.run()
    finally:
        world.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="只验证物理计算，不打开页面")
    args = parser.parse_args()
    wrs = load_wrs()
    try:
        if args.check:
            check_physics(wrs)
        else:
            show_demo(wrs)
    except KeyboardInterrupt:
        pass
