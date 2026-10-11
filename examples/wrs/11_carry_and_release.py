r"""UR7e/DH50 教学演示：程序化夹持跟随，松开后由 MuJoCo 计算下落。

与 10_drop_noodles_into_pot.py、cooking_scene.toml 放在同一目录。
运行：.\scripts\run.ps1 examples/wrs/11_carry_and_release.py [--check]

本例不连接 AgentOS/真实硬件。机械臂由 IK/FK 驱动，不参与 MuJoCo 接触计算；
附着不等于摩擦抓取，轨迹没有经过机械臂碰撞检查。TOML 不会被修改。
"""

import argparse
import importlib.util
import json
import math
from pathlib import Path

from wrs_agent.env.wrs import load_wrs
from wrs_agent.scene import load_scene


DT = 0.02
PORT = 8003
HERE = Path(__file__).resolve().parent


def load_drop_helpers():
    path = HERE / "10_drop_noodles_into_pot.py"
    if not path.is_file():
        raise FileNotFoundError(f"请把上一步示例放在同一目录：{path}")
    spec = importlib.util.spec_from_file_location("cooking_drop_helpers", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CarryDemo:
    def __init__(self, wrs, scene, scene_path):
        self.wrs, self.np = wrs, wrs.np
        self.helpers = load_drop_helpers()
        self.objects, self.physics, self.info = self.helpers.prepare(wrs, scene, scene_path)
        self.noodles = self.objects["noodles"]
        specs = load_scene(scene_path)
        source = specs["noodles"]
        self.noodle_sizes = self.np.asarray(source.geometry.xyz_lengths)
        self.source_tf = self.np.eye(4)
        self.source_tf[:3, :3] = source.rotmat
        self.source_tf[:3, 3] = source.pos

        # 物理场景在上面已编译完成；此后添加的机械臂只用于运动学与显示。
        self.arm, self.gripper = wrs.ur_ur7e.ur7e_with_gripper()
        self.arm.add_to_scene(scene)
        self.open_width = float(self.gripper.jaw_range[1])
        self.hold_width = float(source.geometry.xyz_lengths[1])
        if not 0 < self.hold_width < self.open_width - 0.005:
            raise ValueError("面条宽度需要小于夹爪最大开口，并留出至少 5 毫米余量。")

        # 由实际模型计算法兰到抓取中心的变换，不手写 13.9 厘米偏移。
        self.flange_to_grasp = self.np.linalg.inv(self.arm.tcp("flange").tf) @ self.grasp_tf()
        self.grasp_to_flange = self.np.linalg.inv(self.flange_to_grasp)
        self.grasp_rotation = self.np.asarray(source.rotmat) @ self.np.array(
            [[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
        self.q_low, self.q_high = self.arm.chain_joint_limits("main")

        pick = self.np.asarray(source.pos, dtype=float)
        release = self.np.asarray(self.info["release_position"], dtype=float)
        above_pick = pick.copy()
        above_pick[2] = max(release[2], pick[2] + 0.15)
        if abs(above_pick[2] - release[2]) > 1e-6:
            raise ValueError("当前面条位置过高；请为此场景重新规划搬运高度。")
        retreat = release + [0, 0, 0.10]

        print("正在计算抓取中心轨迹……", flush=True)
        self.ready_q = self.solve(above_pick, self.arm.qs.copy())
        self.frames = []
        q_pick = self.segment("下降到面条", above_pick, pick, self.ready_q, 3.0, self.open_width)
        # 前置等待帧放在下降前；这里只是示例初始化在面条上方，不播放 home→这里。
        ready_frames = [("准备", self.ready_q.copy(), self.open_width)] * 100
        self.frames = ready_frames + self.frames
        self.jaw_segment("闭合夹爪", q_pick, self.open_width, self.hold_width)
        q_lift = self.segment("夹持抬升", pick, above_pick, q_pick, 3.0, self.hold_width)
        q_pot = self.segment("搬到锅上方", above_pick, release, q_lift, 5.0, self.hold_width)
        self.frames.extend([("锅上方停稳", q_pot.copy(), self.hold_width)] * 35)
        self.jaw_segment("打开夹爪", q_pot, self.hold_width, self.open_width)
        q_retreat = self.segment("释放并撤离", release, retreat, q_pot, 2.0, self.open_width)
        self.frames.extend([("等待落稳", q_retreat.copy(), self.open_width)] * 150)
        self.fixed_poses = {k: v.tf.copy() for k, v in self.objects.items() if k != "noodles"}
        self.reset()

    def grasp_tf(self):
        return self.gripper.tcp("grasp_center").tf.copy()

    def solve(self, pos, previous):
        target = self.np.eye(4)
        target[:3, :3], target[:3, 3] = self.grasp_rotation, pos
        flange = target @ self.grasp_to_flange
        candidates = self.arm.ik(flange[:3, 3], flange[:3, :3],
                                 tcp=self.arm.tcp("flange"), ref_qs=previous)
        candidates = [q for q in candidates if self.np.isfinite(q).all()
                      and (q >= self.q_low).all() and (q <= self.q_high).all()]
        if not candidates:
            raise RuntimeError(f"目标无法求解，演示未启动：{pos.tolist()}")
        q = min(candidates, key=lambda v: float(self.np.linalg.norm(v-previous)))
        self.arm.fk(q)
        if not self.np.allclose(self.grasp_tf(), target, atol=0.0001):
            raise RuntimeError("逆运动学结果未通过抓取中心位姿检查。")
        return q.copy()

    def segment(self, stage, start, end, previous, seconds, width):
        # 每 2 厘米求一个 IK 锚点，锚点之间插值关节角；并非碰撞规划。
        count = max(1, math.ceil(float(self.np.linalg.norm(end-start))/0.02))
        anchors = [previous.copy()]
        for i in range(1, count+1):
            q = self.solve(start + (end-start)*i/count, anchors[-1])
            if float(self.np.max(self.np.abs(q-anchors[-1]))) > 0.35:
                raise RuntimeError(f"{stage} 出现过大的 IK 分支跳变，停止生成轨迹。")
            anchors.append(q)
        for i in range(1, round(seconds/DT)+1):
            u = i/round(seconds/DT)
            smooth = u*u*(3-2*u)
            x = smooth*count
            index = min(int(x), count-1)
            q = anchors[index] + (anchors[index+1]-anchors[index])*(x-index)
            self.frames.append((stage, q, width))
        return anchors[-1]

    def jaw_segment(self, stage, q, start, end):
        for width in self.np.linspace(start, end, 50):
            self.frames.append((stage, q.copy(), float(width)))

    def sync_noodles(self, tf):
        self.noodles.set_pos_rotmat(tf[:3, 3], tf[:3, :3])
        self.physics.sync.push_one_sobj_qpos(self.noodles, self.noodles.quat, self.noodles.pos)
        self.physics.data.qvel[:] = 0     # 附着阶段不积分重力；释放前机械臂已停稳
        self.physics.runtime.forward()
        self.physics.sync.pull_all_sobj_pose()

    def reset(self):
        import mujoco
        mujoco.mj_resetData(self.physics.model, self.physics.data)
        self.arm.fk(self.ready_q)
        self.gripper.set_opening(self.open_width)
        self.sync_noodles(self.source_tf)
        self.physics.contact_viz.update_from_data(self.physics.model, self.physics.data)
        self.index, self.stage = 0, None
        self.held, self.released = False, False
        self.attachment = None
        self.release_jump = None
        self.max_follow_error = 0.0
        self.follow_samples = 0
        self.min_held_bottom = float("inf")
        self.min_free_center = float("inf")
        self.max_joint_step = 0.0
        self.events = []

    def tick(self, verbose=False):
        if self.index >= len(self.frames):
            return False
        stage, q, width = self.frames[self.index]
        if stage != self.stage:
            self.stage = stage
            self.events.append(stage)
            if verbose:
                print(stage, flush=True)
            if stage == "夹持抬升":
                # 记录物体相对抓取中心的位姿，后续用坐标变换保持相对关系。
                if self.np.linalg.norm(self.noodles.pos-self.grasp_tf()[:3, 3]) > 0.001:
                    raise RuntimeError("夹爪没有到达面条中心，不能建立附着关系。")
                self.attachment = self.np.linalg.inv(self.grasp_tf()) @ self.noodles.tf
                self.held = True
            elif stage == "释放并撤离":
                before = self.noodles.tf.copy()
                self.sync_noodles(before)
                self.release_jump = float(self.np.max(self.np.abs(self.noodles.tf-before)))
                self.release_pos = self.noodles.pos.copy()
                self.held, self.released = False, True

        self.max_joint_step = max(self.max_joint_step, float(self.np.max(self.np.abs(q-self.arm.qs))))
        self.arm.fk(q)
        self.gripper.set_opening(width)
        if self.held:
            target = self.grasp_tf() @ self.attachment
            self.sync_noodles(target)
            self.max_follow_error = max(self.max_follow_error, float(self.np.max(self.np.abs(self.noodles.tf-target))))
            self.follow_samples += 1
            half = self.np.abs(self.noodles.rotmat[2]) @ (self.noodle_sizes/2)
            self.min_held_bottom = min(self.min_held_bottom, float(self.noodles.pos[2]-half))
        elif self.released:
            self.physics.step(DT)
            self.min_free_center = min(self.min_free_center, float(self.noodles.pos[2]))
        self.index += 1
        return True

    def report(self):
        p = self.noodles.pos
        partners = self.helpers.contact_partners(self.physics, self.objects)
        if not self.released or self.follow_samples == 0:
            raise AssertionError("未完成夹持和释放。")
        assert self.max_follow_error < 0.0001
        assert self.release_jump < 0.00001
        assert self.max_joint_step < 0.05
        assert abs(p[2]-self.info["expected_center_height"]) < 0.001
        assert self.np.linalg.norm(p[:2]-self.release_pos[:2]) < 0.003
        assert float(self.np.linalg.norm(self.physics.data.qvel)) < 0.001
        assert any(k.startswith("pot_bottom_") for k in partners)
        assert "table" not in partners
        assert self.min_free_center > self.info["expected_center_height"]-0.01
        assert all(self.np.allclose(self.objects[k].tf, tf, atol=1e-6) for k, tf in self.fixed_poses.items())
        return {"result": "PASS", "stages": self.events, "attachment_samples": self.follow_samples,
                "max_attachment_transform_error": self.max_follow_error,
                "release_transform_jump": self.release_jump,
                "max_joint_step_rad": self.max_joint_step,
                "release_position": self.release_pos.tolist(), "final_position": p.tolist(),
                "expected_center_height": self.info["expected_center_height"],
                "contact_objects": partners,
                "mode": "kinematic_arm + programmed_attachment + MuJoCo_release",
                "robot_collision_checked": False, "friction_grasp_verified": False}


def main(args):
    wrs = load_wrs()
    if args.check:
        demo = CarryDemo(wrs, wrs.wss.Scene(), args.scene)
        reports = []
        for trial in (1, 2):
            demo.reset()
            while demo.tick():
                pass
            reports.append({"trial": trial, **demo.report()})
        print(json.dumps({"result": "PASS", "checks": reports}, ensure_ascii=False, indent=2))
        return

    from wrs_agent.env.viewer import viewer_hub
    world = wrs.wvw.World(cam_pos=(1.35, -1.15, 1.15), cam_lookat_pos=(0.30, 0, 0.22),
                          port=PORT, hz=50, auto_start_hub=False)
    try:
        world.set_caption("UR7e: programmed carry + MuJoCo release")
        demo = CarryDemo(wrs, world.scene, args.scene)

        def update(dt):
            if not demo.tick(verbose=True):
                print(json.dumps(demo.report(), ensure_ascii=False), flush=True)
                demo.reset()

        world.schedule_interval(update, DT)
        with viewer_hub(PORT):
            print(f"打开 http://127.0.0.1:{PORT}；整套动作循环播放，Ctrl+C 退出。", flush=True)
            print("程序化附着演示；不含机械臂碰撞检查或摩擦抓取验证。", flush=True)
            world.run()
    finally:
        world.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--scene", type=Path, default=HERE / "cooking_scene.toml")
    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        pass
