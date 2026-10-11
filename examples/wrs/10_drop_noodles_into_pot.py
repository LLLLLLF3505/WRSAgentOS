r"""从 cooking_scene.toml 创建物理场景，让面条块从锅口上方自由下落。

从项目根目录运行：
    .\scripts\run.ps1 examples/wrs/10_drop_noodles_into_pot.py --check
    .\scripts\run.ps1 examples/wrs/10_drop_noodles_into_pot.py
本例不连接 AgentOS，不创建机械臂，不修改 TOML；只验证刚体落锅。
"""

import argparse
import json
import time
from pathlib import Path

from wrs_agent.env.wrs import load_wrs
from wrs_agent.scene import load_scene


DT = 0.02
VIEWER_PORT = 8002
RELEASE_GAP = 0.12                 # 面条底部距锅沿 12 厘米


def prepare(wrs, scene, scene_path):
    """TOML 提供外形和位置；物理属性由本示例额外设置。"""
    from wrs.physics.mj_env import MJEnv

    data = load_scene(scene_path)
    walls = [v for k, v in data.items() if k.startswith("pot_wall_")]
    bottoms = {k: v for k, v in data.items() if k.startswith("pot_bottom_")}
    if "noodles" not in data or not walls or not bottoms:
        raise ValueError("场景必须包含 noodles、pot_wall_* 和 pot_bottom_*。")
    for name, obj in data.items():
        if not obj.valid or obj.pos is None or obj.rotmat is None or obj.geometry is None:
            raise ValueError(f"{name} 缺少有效位置、朝向或盒体尺寸。")
        if not wrs.np.allclose(obj.rotmat[2], [0, 0, 1], atol=1e-6):
            raise ValueError("本入门示例要求物体保持水平，只允许绕 Z 轴旋转。")

    center = wrs.np.mean([v.pos[:2] for v in walls], axis=0)
    rim = max(v.pos[2] + v.geometry.xyz_lengths[2] / 2 for v in walls)
    floor_name = max(bottoms, key=lambda k: bottoms[k].pos[2] + bottoms[k].geometry.xyz_lengths[2] / 2)
    floor = bottoms[floor_name]
    floor_top = floor.pos[2] + floor.geometry.xyz_lengths[2] / 2
    noodle_spec = data["noodles"]
    sizes = noodle_spec.geometry.xyz_lengths
    # 检查面条四角位于最高锅底拼板上方，限定本例的释放位置。
    for sx in (-1, 1):
        for sy in (-1, 1):
            offset = wrs.np.asarray(noodle_spec.rotmat) @ [sx*sizes[0]/2, sy*sizes[1]/2, 0]
            xy = center + offset[:2]
            local = wrs.np.asarray(floor.rotmat).T @ [*(xy-wrs.np.asarray(floor.pos[:2])), 0]
            if any(abs(local[j]) >= floor.geometry.xyz_lengths[j]/2 for j in (0, 1)):
                raise ValueError("面条超出中心锅底拼板范围，需要重新选择释放位置。")
    # 面条的包围圆应位于每块锅壁内侧，避免一开始就撞到锅沿。
    noodle_radius = float(wrs.np.linalg.norm(sizes[:2])) / 2
    for wall in walls:
        clearance = float(wrs.np.linalg.norm(wrs.np.asarray(wall.pos[:2])-center)) - wall.geometry.xyz_lengths[0]/2
        if noodle_radius >= clearance:
            raise ValueError("面条尺寸超过锅内可用空间。")

    release = [float(center[0]), float(center[1]), rim + RELEASE_GAP + sizes[2]/2]
    objects = {}
    for name, spec in data.items():
        dynamic = name == "noodles"
        kwargs = {"mass": 0.10} if dynamic else {}
        obj = wrs.wssop.box(
            name=name, pos=release if dynamic else spec.pos, rotmat=spec.rotmat,
            xyz_lengths=spec.geometry.xyz_lengths, rgb=spec.rgb,
            collision_type=wrs.wuc.CollisionType.OBB,
            is_floating=dynamic, **kwargs,
        )
        obj.add_to_scene(scene)
        objects[name] = obj
    physics = MJEnv(scene=scene)
    physics.model.geom_solref[:, 0] = 0.006
    info = {"release_position": release, "pot_rim_height": rim,
            "pot_floor_top": floor_top, "expected_center_height": floor_top + sizes[2]/2,
            "original_noodle_position": noodle_spec.pos}
    return objects, physics, info


def reset(physics, initial_qpos):
    import mujoco
    mujoco.mj_resetData(physics.model, physics.data)
    physics.data.qpos[:] = initial_qpos
    physics.runtime.forward()
    physics.sync.pull_all_sobj_pose()
    physics.contact_viz.update_from_data(physics.model, physics.data)


def contact_partners(physics, objects):
    """把 MuJoCo 内部的接触体编号转换为 TOML 物体名称。"""
    body_names = {physics.model.body(physics.sync.sobj2bdy[obj].name).id: name
                  for name, obj in objects.items()}
    partners = set()
    for contact in physics.data.contact[:physics.data.ncon]:
        a = body_names.get(int(physics.model.geom_bodyid[contact.geom1]))
        b = body_names.get(int(physics.model.geom_bodyid[contact.geom2]))
        if a == "noodles" and b:
            partners.add(b)
        elif b == "noodles" and a:
            partners.add(a)
    return sorted(partners)


def check(wrs, scene_path):
    objects, physics, info = prepare(wrs, wrs.wss.Scene(), scene_path)
    initial_qpos = physics.data.qpos.copy()
    fixed_poses = {k: v.tf.copy() for k, v in objects.items() if k != "noodles"}
    reports = []
    for trial in (1, 2):
        reset(physics, initial_qpos)
        heights = []
        for _ in range(150):
            physics.step(DT)
            heights.append(float(objects["noodles"].pos[2]))
        position = objects["noodles"].pos
        partners = contact_partners(physics, objects)
        assert abs(heights[4] - (info["release_position"][2] - 0.5*9.81*0.1**2)) < 0.003
        assert abs(position[2] - info["expected_center_height"]) < 0.001
        assert wrs.np.linalg.norm(position[:2] - wrs.np.asarray(info["release_position"][:2])) < 0.003
        assert min(heights) > info["expected_center_height"] - 0.01
        assert float(wrs.np.linalg.norm(physics.data.qvel)) < 0.001
        assert any(name.startswith("pot_bottom_") for name in partners), partners
        assert "table" not in partners, "面条穿过锅底，错误落到了桌面。"
        assert all(wrs.np.allclose(objects[k].tf, pose, atol=1e-6) for k, pose in fixed_poses.items())
        reports.append({"trial": trial, "final_position": position.tolist(),
                        "minimum_center_height": min(heights), "contact_objects": partners})
    print(json.dumps({"result": "PASS", "scene": str(scene_path.resolve()),
                      **info, "checks": reports}, ensure_ascii=False, indent=2))


def show(wrs, scene_path):
    from wrs_agent.env.viewer import viewer_hub
    world = wrs.wvw.World(cam_pos=(1.0, -0.6, 1.1), cam_lookat_pos=(0.5, 0.15, 0.10),
                          port=VIEWER_PORT, hz=50, auto_start_hub=False)
    try:
        world.set_caption("WRS + MuJoCo: noodles falling into pot")
        objects, physics, info = prepare(wrs, world.scene, scene_path)
        initial_qpos = physics.data.qpos.copy()
        cycle_start, reported = time.monotonic(), False

        def update(dt):
            nonlocal cycle_start, reported
            elapsed = time.monotonic() - cycle_start
            if elapsed >= 7.0:
                reset(physics, initial_qpos)
                cycle_start, reported = time.monotonic(), False
            elif elapsed >= 2.0:
                physics.step(DT)
                if not reported and physics.data.time >= 2.0:
                    print("面条位置：", objects["noodles"].pos.tolist(),
                          "接触物体：", contact_partners(physics, objects), flush=True)
                    reported = True

        world.schedule_interval(update, DT)
        with viewer_hub(VIEWER_PORT):
            print(f"打开 http://127.0.0.1:{VIEWER_PORT}；每 7 秒重播，先悬停 2 秒。", flush=True)
            print("本例仅在内存中把面条移到锅上方，不修改 TOML，也不包含机械臂。", flush=True)
            print("释放位置：", info["release_position"], "；Ctrl+C 退出。", flush=True)
            world.run()
    finally:
        world.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--scene", type=Path, default=Path(__file__).with_name("cooking_scene.toml"))
    args = parser.parse_args()
    wrs = load_wrs()
    try:
        if args.check:
            check(wrs, args.scene)
        else:
            show(wrs, args.scene)
    except KeyboardInterrupt:
        pass
