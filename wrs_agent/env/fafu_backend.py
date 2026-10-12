"""Six active FAFU joints; fixed tool joint stays internal to WRS."""

import time
import tomllib
from pathlib import Path

from wrs_agent.env.fafu import load_model
from wrs_agent.env.wrs import URVirtualModel
from wrs_agent.executor import SkillFailure


class FafuModel(URVirtualModel):
    profile = "fafu"

    def __init__(self):

        config = tomllib.loads(
            (Path(__file__).parents[1] / "assets/fafu/profile.toml").read_text(encoding="utf-8")
        )
        model = load_model()
        from wrs.robots.base.kine.numik import NumIKSolver

        self.wrs, self.robot = model[:2]
        self.apply, self.limits = model[4], model[7]
        self.base_pos = config["base_pos"]
        self.poses = config["poses"]
        self.robot.set_pos_rotmat(pos=self.base_pos)
        self.indices = [
            next(i for i, j in enumerate(self.robot.structure.jnts) if j.name == f"joint{n}")
            for n in range(1, 7)
        ]
        links = self.robot.structure.lnk_map
        self.chain = self.robot.add_chain(
            "main", links["base_link"], links["tool_link"], solver=NumIKSolver
        )
        if [self.robot.structure.jnts[i].name for i in self.chain.active_jnt_ids] != [
            f"joint{n}" for n in range(1, 7)
        ]:
            raise ValueError("fafu_chain_joint_order_mismatch")
        tip = next(link for link in self.robot.runtime_lnks if link.name == "tool_link")
        self.tcp = self.robot.add_tcp("tool_link", tip)
        self.gripper = None
        self.apply(self.poses["home"])
        self.scene = self.wrs.wss.Scene()
        self.robot.add_to_scene(self.scene)
        self.objects = {}
        self.path = self.jaw_path = self.goal = self.goal_tcp_tf = None
        self.goal_width = None

    @property
    def width(self):
        return None

    def read(self):
        tf = self.tcp.tf
        return dict(
            qs=self.robot.qs[self.indices].tolist(),
            tcp_name=self.tcp.name,
            tcp_pos=tf[:3, 3].tolist(),
            tcp_rotmat=tf[:3, :3].tolist(),
            gripper_width=None,
            observed_at_ns=time.time_ns(),
            valid=bool(self.wrs.np.isfinite(self.robot.gl_lnk_tfarr).all()),
        )

    def begin(self, pose, count):
        if pose not in self.poses:
            raise SkillFailure("unknown_fafu_pose")
        self.goal = self.wrs.np.asarray(self.poses[pose])
        if self.wrs.np.any(self.goal < self.limits[:, 0]) or self.wrs.np.any(
            self.goal > self.limits[:, 1]
        ):
            raise SkillFailure("fafu_pose_out_of_limits")
        self.goal_tcp_tf = None
        self.path = self.wrs.np.linspace(self.robot.qs[self.indices], self.goal, count)

    def begin_gripper(self, command, count):
        raise SkillFailure("fafu_gripper_not_supported")

    def begin_relative(self, displacement, count):
        np = self.wrs.np
        current = self.robot.qs[self.indices].copy()
        target = self.tcp.tf.copy()
        target[:3, 3] += np.asarray(displacement)
        # Explicit active-joint seed: NumIKSolver does not consume ref_qs.
        solutions = self.robot.ik(
            target[:3, 3], target[:3, :3], tcp=self.tcp, qs_active_init=current, max_iter=200
        )
        candidates = [
            q[self.indices]
            for q in solutions
            if np.isfinite(q).all()
            and (q[self.indices] >= self.limits[:, 0]).all()
            and (q[self.indices] <= self.limits[:, 1]).all()
            and np.max(np.abs(q[self.indices] - current)) <= 1.0
        ]
        if not candidates:
            raise SkillFailure("relative_target_unreachable")
        goal = min(candidates, key=lambda q: np.linalg.norm(q - current))
        self.goal, self.goal_tcp_tf = goal, target
        self.path = np.linspace(current, goal, count)

    def step(self, index):
        self.apply(self.path[index])
        return self.read()

    def sync(self, qs, width=None):
        if width is not None:
            raise ValueError("fafu_gripper_not_supported")
        self.apply(qs)
        return self.read()

    def verify(self):
        np = self.wrs.np
        return bool(
            np.isfinite(self.robot.gl_lnk_tfarr).all()
            and np.allclose(self.robot.qs[self.indices], self.goal, atol=1e-6)
            and (self.goal_tcp_tf is None or np.allclose(self.tcp.tf, self.goal_tcp_tf, atol=1e-3))
        )
