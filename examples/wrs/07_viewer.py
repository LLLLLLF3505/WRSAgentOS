"""读取 WRS 节点的关节状态，在 WRS 场景中显示；与控制脚本分开运行。"""

from pathlib import Path

from examples._session import use_local_token
from wrs_agent import connect
from wrs_agent.env.viewer import viewer_hub
from wrs_agent.env.wrs import VirtualModel, sync_scene_objects

CONFIG = Path(__file__).with_name("bindings.toml")
VIEWER_PORT = 8000

if __name__ == "__main__":
    use_local_token("wrs")
    model = VirtualModel()
    wrs = model.wrs
    world = wrs.wvw.World(
        cam_pos=(1.65, -1.85, 1.55),
        cam_lookat_pos=(0.3, 0, 0.35),
        port=VIEWER_PORT,
        hz=10,
        auto_start_hub=False,
    )
    world.set_caption("WRS FAFU — 节点状态")
    model.robot.add_to_scene(world.scene)
    wrs.wssop.frame().add_to_scene(world.scene)

    with connect("tcp/127.0.0.1:7449", env_id="wrs-demo", bindings=CONFIG) as system:
        initial = system.snapshot(node="wrs")
        if initial.data.robot.kinematics is None or not initial.data.robot.kinematics.valid:
            raise RuntimeError("WRS 节点没有有效的关节状态。")
        if initial.data.facts.get("robot_model") != "fafu_baseV1":
            raise RuntimeError("该观察器要求 FAFU 节点，请检查部署配置。")
        model.robot.set_pos_rotmat(pos=[initial.data.facts["robot_base_" + axis] for axis in "xyz"])
        model.sync(initial.data.robot.kinematics.qs)
        displayed = {}
        sync_scene_objects(wrs, world.scene, initial.data.objects, displayed)

        def update(dt):
            snapshot = system.snapshot(node="wrs")
            if snapshot.boot_id != initial.boot_id:
                raise RuntimeError("WRS 节点已重启，请重新连接 viewer。")
            state = snapshot.data.robot.kinematics
            if state is None or not state.valid:
                raise RuntimeError("WRS 关节状态无法确认，停止显示。")
            # 只更新显示模型；运动命令由 06_control_arm 或 voice/06_push_to_talk 发给执行节点。
            model.sync(state.qs)
            sync_scene_objects(wrs, world.scene, snapshot.data.objects, displayed)

        world.schedule_interval(update, 0.1)
        try:
            # 这个帮助函数只管理页面服务，场景和刷新逻辑都在本文件。
            with viewer_hub(VIEWER_PORT):
                print(f"打开 http://127.0.0.1:{VIEWER_PORT}，Ctrl+C 退出观察。", flush=True)
                world.run()
        finally:
            world.close()  # 关闭显示不会取消远端任务。
