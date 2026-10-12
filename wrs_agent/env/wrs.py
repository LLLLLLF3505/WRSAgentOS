"""WRS virtual models: default FAFU; explicit UR7e hardware mirror, each thread-owned."""

import asyncio
import site
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from math import pi
from pathlib import Path

from wrs_agent.executor import ActionExecutor, ExecutionUnknown, SkillFailure
from wrs_agent.scene import load_scene
from wrs_agent.schemas import ObjectData, RobotData, SceneData
from wrs_agent.skills.robot import SKILLS

ROOT = Path(__file__).resolve().parents[2]
# "home" must equal UR7E.home_qs: the model starts there and reports pose="home".
POSES = {
    "home": [0.0, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0.0],
    "B": [0.3, -1.2, 1.8, -2.2, -1.57, 0.0],
    "C": [-0.3, -1.4, 1.5, -1.7, -1.57, 0.0],
}


def load_wrs():
    # Append platform-specific scientific package paths without executing .pth files.
    prefixes = [str(Path(sys.executable).parents[1]), sys.base_prefix]
    for package_path in site.getsitepackages(prefixes):
        if package_path not in sys.path:
            sys.path.append(package_path)
    source = ROOT / "third_party/wrs"
    sys.path.insert(0, str(source))
    import wrs

    if not Path(wrs.__file__).resolve().is_relative_to(source.resolve()):
        raise RuntimeError("unexpected_wrs_source")
    return wrs


def make_scene_object(wrs, object_id, data):
    """Construct only known, valid geometry; never invent an unknown pose."""
    if not data.valid or data.pos is None or data.rotmat is None or data.geometry is None:
        return None
    if data.geometry.kind == "mesh":
        from wrs.scene.scene_object import SceneObject

        asset_root = (ROOT / "wrs_agent/assets").resolve()
        path = (asset_root / data.geometry.asset).resolve()
        if not path.is_relative_to(asset_root) or not path.is_file():
            raise ValueError("mesh_asset_missing_or_outside_asset_root")
        obj = SceneObject.from_file(str(path), rgb=data.rgb, alpha=data.alpha)
        obj.name = object_id
        obj.set_pos_rotmat(pos=data.pos, rotmat=data.rotmat)
        return obj
    return wrs.wssop.box(
        pos=data.pos, rotmat=data.rotmat, xyz_lengths=data.geometry.xyz_lengths,
        rgb=data.rgb, alpha=data.alpha, name=object_id,
    )


def sync_scene_objects(wrs, scene, objects, displayed):
    """Apply serialized objects to a local WRS scene; no remote motion."""
    for name in set(displayed) - set(objects):
        displayed.pop(name)[1].remove_from_scene(scene)
    for name, data in objects.items():
        previous = displayed.get(name)
        if previous is not None and previous[0] == data:
            continue
        if previous is not None:
            previous[1].remove_from_scene(scene)
            del displayed[name]
        obj = make_scene_object(wrs, name, data)
        if obj is not None:
            obj.add_to_scene(scene)
            displayed[name] = (data.model_copy(deep=True), obj)


class URVirtualModel:
    """All methods, including construction, run on one owner thread."""

    def __init__(self):
        self.wrs = load_wrs()
        self.robot, self.gripper = self.wrs.ur_ur7e.ur7e_with_gripper()
        self.tcp = self.robot.tcp("flange")
        self.scene = self.wrs.wss.Scene()
        self.robot.add_to_scene(self.scene)
        self.objects = {}
        self.path = None
        self.jaw_path = None
        self.goal = None
        self.goal_tcp_tf = None
        self.goal_width = self.width

    @property
    def width(self):
        # Both DH50 fingers travel half the opening.
        return float(self.gripper.qs[0] * 2.0)

    def load_scene(self, objects):
        sync_scene_objects(self.wrs, self.scene, objects, self.objects)
        # Static configuration is read back from the actual WRS SceneObjects.
        result = {}
        for name, data in objects.items():
            values = data.model_dump()
            if name in self.objects:
                obj = self.objects[name][1]
                values.update(pos=obj.pos.tolist(), rotmat=obj.rotmat.tolist())
            result[name] = ObjectData.model_validate(values)
        return result

    def read(self):
        tcp_tf = self.tcp.tf
        return {
            "qs": self.robot.qs.tolist(),
            "tcp_name": self.tcp.name,
            "tcp_pos": tcp_tf[:3, 3].tolist(),
            "tcp_rotmat": tcp_tf[:3, :3].tolist(),
            "gripper_width": self.width,
            "observed_at_ns": time.time_ns(),
            "valid": bool(
                self.wrs.np.isfinite(self.robot.gl_lnk_tfarr).all()
                and self.wrs.np.isfinite(self.gripper.gl_lnk_tfarr).all()
                and self.wrs.np.isfinite(tcp_tf).all()
            ),
        }

    def begin(self, pose, count):
        self.goal_tcp_tf, self.jaw_path, self.goal_width = None, None, self.width
        self.goal = self.wrs.np.asarray(POSES[pose], dtype=self.wrs.np.float32)
        self.path = self.wrs.wmij.interp_by_n(self.robot.qs.copy(), self.goal, count)

    def begin_gripper(self, command, count):
        low, high = (float(v) for v in self.gripper.jaw_range)
        target = high if command == "open" else low
        # Only the jaw moves; the arm goal is where it already is.
        self.path, self.goal, self.goal_tcp_tf = None, self.robot.qs.copy(), None
        self.jaw_path = self.wrs.np.linspace(self.width, target, count).clip(low, high)
        self.goal_width = target

    def begin_relative(self, displacement, count):
        np = self.wrs.np
        current = self.robot.qs.copy()
        target = self.tcp.tf.copy()
        target[:3, 3] += np.asarray(displacement)
        solutions = self.robot.ik(target[:3, 3], target[:3, :3], tcp=self.tcp, ref_qs=current)
        lower, upper = self.robot.chain_joint_limits("main")
        # Reject distant IK branches before changing the model or its published state.
        candidates = [
            q
            for q in solutions
            if np.isfinite(q).all()
            and (q >= lower).all()
            and (q <= upper).all()
            and np.max(np.abs(q - current)) <= 1.0
        ]
        if not candidates:
            raise SkillFailure("relative_target_unreachable")
        goal = min(candidates, key=lambda q: np.linalg.norm(q - current))
        self.path, self.jaw_path = self.wrs.wmij.interp_by_n(current, goal, count), None
        self.goal, self.goal_tcp_tf, self.goal_width = goal, target, self.width

    def step(self, index):
        if self.jaw_path is not None:
            self.gripper.set_opening(float(self.jaw_path[index]))
        else:
            self.robot.fk(self.path[index])
        return self.read()

    def sync(self, qs, width=None):
        """Mirror a measured robot; the model then plans from where the robot really is."""
        self.robot.fk(self.wrs.np.asarray(qs, dtype=self.wrs.np.float32))
        if width is not None:
            low, high = (float(v) for v in self.gripper.jaw_range)
            self.gripper.set_opening(min(max(float(width), low), high))
        return self.read()

    def target(self):
        return self.goal.tolist(), self.goal_width

    def verify_measured(self, joint_tolerance=0.01, position_tolerance=0.003):
        """Postcondition on synced measurements; real joints never match a goal exactly."""
        np = self.wrs.np
        return bool(
            np.isfinite(self.robot.gl_lnk_tfarr).all()
            and np.max(np.abs(self.robot.qs - self.goal)) <= joint_tolerance
            and (
                self.goal_tcp_tf is None
                or np.linalg.norm(self.tcp.tf[:3, 3] - self.goal_tcp_tf[:3, 3])
                <= position_tolerance
            )
        )

    def verify(self):
        # Read the model, not a timer or a commanded pose label.
        before = self.robot.gl_lnk_tfarr.copy()
        after = self.robot.fk(self.robot.qs.copy())
        return bool(
            self.wrs.np.allclose(self.robot.qs, self.goal, atol=1e-6)
            and self.wrs.np.allclose(before, after, atol=1e-6)
            and self.wrs.np.isfinite(after).all()
            and abs(self.width - self.goal_width) <= 1e-6
            and (
                self.goal_tcp_tf is None
                or self.wrs.np.allclose(self.tcp.tf, self.goal_tcp_tf, atol=1e-4)
            )
        )


def VirtualModel():
    from wrs_agent.env.fafu_backend import FafuModel

    return FafuModel()


class VirtualState:
    def __init__(self, kinematics, objects=None):
        self.version = 0
        self.kinematics = kinematics
        self.objects = objects or {}
        self.pose = "home"
        self.facts = {}  # Backend-owned extras, e.g. which device the model mirrors.

    def snapshot(self):
        return SceneData(
            robot=RobotData(pose=self.pose, kinematics=self.kinematics),
            objects={name: obj.model_copy(deep=True) for name, obj in self.objects.items()},
            facts={"calibration": "wrs2-ur7e-dh50-v1", "collision_checked": False, **self.facts},
        )


ROBOT_SKILLS = ("observe", "move_named_pose", "move_relative", "set_gripper")
UNSUPPORTED = {
    "pick": "DH50 jaw is modeled, but there is no validated grasp/contact scene.",
    "place": "No verified held-object/contact state.",
    "verify": "Object placement verifier requires a grasp scene.",
    "collision_planning": "No collision-checked path in this kinematic profile.",
}


async def open_model(scene=None, *, model_factory=None):
    """One WRS model on its own owner thread; returns (call, model, state, close)."""
    objects = await asyncio.to_thread(load_scene, scene) if scene is not None else {}
    owner = ThreadPoolExecutor(max_workers=1, thread_name_prefix="wrs-model")
    loop = asyncio.get_running_loop()

    async def call(fn, *args):
        # Never cancel this future to claim a physical/model stop.
        return await loop.run_in_executor(owner, fn, *args)

    try:
        model = await call(model_factory or VirtualModel)
        if scene is None and getattr(model, "profile", None) == "fafu":
            scene = ROOT / "wrs_agent/assets/kitchen/scene.toml"
            objects = await asyncio.to_thread(load_scene, scene)
        if scene is not None:
            objects = await call(model.load_scene, objects)
        state = VirtualState(await call(model.read), objects)
        if getattr(model, "profile", None) == "fafu":
            state.facts.update(robot_model="fafu_baseV1", calibration="fafu-baseV1-tool-link-v1",
                               robot_base_x=model.base_pos[0], robot_base_y=model.base_pos[1],
                               robot_base_z=model.base_pos[2], physics_enabled=False)
    except BaseException:
        owner.shutdown(wait=False)
        raise

    async def close():
        await asyncio.to_thread(owner.shutdown, wait=True)

    return call, model, state, close


async def advance_virtual(call, model, duration, skill, state, args, stop, progress):
    try:
        if skill == "observe":
            state.kinematics = await call(model.read)
            state.version += 1
            return state.kinematics["valid"]
        count = max(2, round(duration / 0.02) + 1)
        if skill == "move_relative":
            await call(model.begin_relative, (args.dx, args.dy, args.dz), count)
        elif skill == "set_gripper":
            await call(model.begin_gripper, args.command, count)
        else:
            await call(model.begin, args.pose, count)
        for index in range(1, count):
            if stop.is_set():
                return False
            if skill != "set_gripper":
                state.pose = None
            # One in-flight FK only. No model object is read by the control thread.
            state.kinematics = await call(model.step, index)
            state.version += 1
            if stop.is_set():
                return False
            progress(index / (count - 1))
            try:
                await asyncio.wait_for(stop.wait(), duration / (count - 1))
            except TimeoutError:
                pass
        if stop.is_set():
            return False
        verified = await call(model.verify)
        if not stop.is_set() and verified and skill == "move_named_pose":
            state.pose = args.pose
        return verified
    except SkillFailure:
        # An unreachable target has caused no model update.
        raise
    except Exception:
        state.kinematics = {**state.kinematics, "valid": False}
        # A model update may have happened; no blind allow_actions or retry.
        raise ExecutionUnknown("wrs_model_state_unknown") from None


async def make_wrs_environment(journal_path, *, duration=0.4, scene=None, allow_hardware=False):
    if allow_hardware:
        raise ValueError("hardware_unsupported")
    if not 0 < duration <= 30:
        raise ValueError("invalid_duration")
    call, model, state, close = await open_model(scene)
    return ActionExecutor(
        journal_path,
        state=state,
        close_backend=close,
        skills=[
            SKILLS[name].bind(partial(advance_virtual, call, model, duration, name))
            for name in ("observe", "move_named_pose", "move_relative")
        ],
        backend="wrs",
        features_extra={
            "verification": "wrs_fk",
            "stop_scope": "virtual_fk_boundary",
            "controller_flush": False,
            "unsupported": {
                **UNSUPPORTED,
                "set_gripper": "FAFU gripper is not modeled; no jaw control or grasp physics.",
                "pick": "FAFU gripper/contact physics are not implemented.",
                "hardware": "Virtual profile; the ur_rtde backend drives the real UR7e.",
            },
        },
    )


def probe_virtual():
    from importlib.metadata import version

    model = VirtualModel()
    return {
        "import": "PASS",
        "dependencies": {name: version(name) for name in
                         ("numpy", "scipy", "mujoco", "wgpu", "websockets")},
        "module": model.wrs.__file__, "robot_class": type(model.robot).__name__,
        "gripper_class": None, "hardware": False, "virtual_runtime_adapter": "fafu_ik_fk",
        "fk_shape": list(model.robot.gl_lnk_tfarr.shape), **model.read(),
    }


async def serve_viewer_hub(port):
    # Keep all WRS imports in this adapter; no detached server survives the viewer.
    load_wrs()
    from wrs.viewer.server import serve

    await serve(host="127.0.0.1", port=port, idle_timeout=0, auto_open=False)
