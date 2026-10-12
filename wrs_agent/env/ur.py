"""UR7e + DH50 backend: the WRS model plans and checks every motion. The node starts
virtual; the real robot moves only after an explicit switch, and only when the node
was started with hardware allowed.
"""

import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor
from functools import partial

from wrs_agent.env import ur_rtde
from wrs_agent.env.wrs import ROBOT_SKILLS, UNSUPPORTED, URVirtualModel, advance_virtual, open_model
from wrs_agent.errors import error_info
from wrs_agent.executor import ActionExecutor, ExecutionUnknown, SkillFailure
from wrs_agent.schemas import ModeReceipt, ModeRequest
from wrs_agent.skills.robot import SKILLS

MONITOR_PERIOD = 0.2  # Idle readback for display; never used as a motion authorization.


async def make_ur_environment(
    journal_path,
    *,
    robot_ip=None,
    gripper_port=None,
    allow_hardware=False,
    duration=2.0,
    scene=None,
    max_joint_speed=0.3,
    gripper_width=0.05,
    payload_kg=None,
    connect=None,
):
    """Return (executor, switch_mode). Nothing connects to the robot here."""
    if not 0 < duration <= 30:
        raise ValueError("invalid_duration")
    if not 0 < max_joint_speed <= 1.0:
        raise ValueError("invalid_max_joint_speed")
    if allow_hardware and connect is None and not (robot_ip and gripper_port):
        raise ValueError("hardware_requires_robot_ip_and_gripper_port")
    connect = connect or partial(
        ur_rtde.connect, robot_ip, gripper_port, gripper_width=gripper_width, payload_kg=payload_kg
    )
    call, model, state, close_model = await open_model(scene, model_factory=URVirtualModel)
    loop = asyncio.get_running_loop()
    owner = ThreadPoolExecutor(max_workers=1, thread_name_prefix="ur-device")

    async def device(fn, *args):
        # Like the model owner: a device call is never cancelled to claim a stop.
        return await loop.run_in_executor(owner, fn, *args)

    link = {"driver": None, "busy": False}
    switches = {}
    state.facts.update(execution="virtual", robot_status="disconnected")

    async def mirror(qs, width):
        state.kinematics = await call(model.sync, qs, width)
        state.version += 1

    async def track(driver, job, halt, stop, progress):
        """Relay stop into the device thread and mirror its readback until the job ends."""
        job = asyncio.ensure_future(job)
        stopping = asyncio.ensure_future(stop.wait())
        try:
            while not job.done():
                await asyncio.wait({job} if stopping.done() else {job, stopping}, timeout=0.05)
                if stop.is_set():
                    halt.set()
                latest = driver.latest()
                if latest is not None:
                    qs, width, fraction = latest
                    await mirror(qs, width)
                    if fraction is not None and not stop.is_set():
                        progress(fraction)
            return await job
        finally:
            stopping.cancel()
            if not job.done():
                halt.set()  # A cancelled worker must still stop the device thread.

    async def advance(skill, state, args, stop, progress):
        driver = link["driver"]
        if driver is None:
            return await advance_virtual(call, model, duration, skill, state, args, stop, progress)
        try:
            sample = await device(driver.read)
            state.facts["robot_status"] = sample.status
            await mirror(sample.qs, sample.width)
            if skill == "observe":
                return sample.ready and state.kinematics["valid"]
            if not sample.ready:
                raise SkillFailure(f"robot_not_ready_{sample.status}")
            halt = threading.Event()
            if skill == "set_gripper":
                await call(model.begin_gripper, args.command, 2)
                _, width = await call(model.target)
                job = device(driver.move_gripper, width, halt)
            else:
                if skill == "move_relative":
                    await call(model.begin_relative, (args.dx, args.dy, args.dz), 2)
                else:
                    await call(model.begin, args.pose, 2)
                goal, _ = await call(model.target)
                path = ur_rtde.min_jerk_path(
                    sample.qs, goal, dt=driver.dt, max_speed=max_joint_speed, min_duration=duration
                )
                state.pose = None
                job = device(driver.follow, path, halt)
            result = await track(driver, job, halt, stop, progress)
            final = await device(driver.read)
            state.facts["robot_status"] = final.status
            await mirror(final.qs, final.width)
            if result == "stopped" or stop.is_set():
                return False
            if skill == "set_gripper":
                # CAUGHT is a jaw stop on contact; it never claims a verified grasp.
                expected = {ur_rtde.GRIP_ARRIVED} if args.command == "open" else {
                    ur_rtde.GRIP_ARRIVED, ur_rtde.GRIP_CAUGHT
                }
                return final.ready and result in expected
            verified = final.ready and await call(model.verify_measured)
            if verified and skill == "move_named_pose":
                state.pose = args.pose
            return verified
        except SkillFailure:
            raise  # Rejected before any device command.
        except Exception:
            state.kinematics = {**state.kinematics, "valid": False}
            state.facts["robot_status"] = "unknown"
            # The arm or jaw may have moved; observe on site, no blind allow_actions or retry.
            raise ExecutionUnknown("ur_device_state_unknown") from None

    executor = ActionExecutor(
        journal_path,
        state=state,
        skills=[SKILLS[name].bind(partial(advance, name)) for name in ROBOT_SKILLS],
        backend="ur_rtde",
        features_extra={
            "hardware": True,
            "verification": "rtde_joint_readback",
            "stop_scope": "ur_servo_stop_then_observed_rest",
            # servoStop should clear the lookahead window; not yet measured on the robot.
            "controller_flush": False,
            "unsupported": {
                **UNSUPPORTED,
                "collision_planning": "Joint-space min-jerk path; no collision checking.",
            },
        },
    )

    def receipt(accepted, reason=""):
        return ModeReceipt(
            accepted=accepted,
            reason=reason,
            mode="virtual" if link["driver"] is None else "real",
            control_epoch=executor.epoch,
            error=None
            if accepted
            else error_info(reason, node_id=executor.node_id, stage="control"),
        )

    async def switch(request):
        if request.boot_id != executor.boot_id or request.control_epoch != executor.epoch:
            return receipt(False, "stale_control")
        if link["busy"]:
            return receipt(False, "mode_switch_busy")
        real = request.mode == "real"
        if real == (link["driver"] is not None):
            return receipt(True)
        if real and not allow_hardware:
            return receipt(False, "hardware_not_enabled")
        if (
            executor.active is not None
            or executor.admission == "UNKNOWN"
            or not executor.stop_confirmed
        ):
            return receipt(False, "mode_switch_not_ready")
        # Fence like hold: old leases die; allow_actions waits for a confirmed switch.
        executor.epoch += 1
        executor.leases.clear()
        executor.admission = "HELD"
        executor.stop_confirmed = False
        fenced = executor.epoch
        link["busy"] = True
        try:
            if real:
                driver = await device(connect)
                sample = await device(driver.read)
                if not sample.ready or executor.epoch != fenced:
                    await device(driver.close)
                    reason = "mode_switch_interrupted" if sample.ready else sample.status
                    return receipt(False, f"robot_not_ready_{reason}")
                link["driver"] = driver
                state.facts.update(execution="real", robot_status=sample.status)
                await mirror(sample.qs, sample.width)
            else:
                driver, link["driver"] = link["driver"], None
                await device(driver.close)
                # The model keeps the last measured pose; nothing jumps back.
                state.facts.update(execution="virtual", robot_status="disconnected")
                state.version += 1
            state.pose = None
            return receipt(True)
        except Exception:
            return receipt(False, "hardware_connect_failed")
        finally:
            link["busy"] = False
            executor.stop_confirmed = executor.admission != "UNKNOWN"
            executor.on_event(
                "events/control",
                {
                    "boot_id": executor.boot_id,
                    "control_epoch": executor.epoch,
                    "stop_confirmed": executor.stop_confirmed,
                    "admission": executor.admission,
                    "execution": state.facts["execution"],
                },
            )

    async def switch_mode(payload):
        request = ModeRequest.model_validate(payload)
        if request.interrupt_id in switches:
            old, result = switches[request.interrupt_id]
            if old != request:
                return receipt(False, "interrupt_id_conflict").model_dump()
            return result.model_dump()
        result = await switch(request)
        if len(switches) < 4096:
            switches[request.interrupt_id] = (request, result)
        return result.model_dump()

    async def monitor():
        while True:
            await asyncio.sleep(MONITOR_PERIOD)
            driver = link["driver"]
            if driver is None or link["busy"] or executor.active is not None:
                continue
            try:
                sample = await device(driver.read)
            except Exception:
                state.facts["robot_status"] = "read_failed"
                continue
            # Display freshness only: no version bump, so leases and plans stay valid.
            if executor.active is None and link["driver"] is driver:
                state.facts["robot_status"] = sample.status
                state.kinematics = await call(model.sync, sample.qs, sample.width)

    watcher = asyncio.create_task(monitor())

    async def close():
        watcher.cancel()
        await asyncio.gather(watcher, return_exceptions=True)
        if link["driver"] is not None:
            driver, link["driver"] = link["driver"], None
            await device(driver.close)
        await asyncio.to_thread(owner.shutdown, wait=True)
        await close_model()

    executor.close_backend = close
    return executor, switch_mode
