"""Every teaching file and the observable outcome checked by verify.py."""

WRS = {
    "nodes/06_late_node.py": ["直接调用： SUCCEEDED", "Agent 调度： SUCCEEDED"],
    "beginner/01_action.py": ["动作结果： SUCCEEDED", "当前位置： B"],
    "beginner/02_skills.py": ["move_named_pose 版本 1", "speak 版本 1"],
    "beginner/04_invalid_input.py": ["错误代码： invalid_arguments"],
    "tasks/01_sequence.py": ["任务结果： SUCCEEDED", "TCP 位置："],
    "tasks/02_parallel.py": ["正在执行的动作数量： 2", "任务结果： SUCCEEDED"],
    "tasks/03_cancel.py": ["旧任务： CANCELLED", "新任务： SUCCEEDED", "旧句柄仍指向： CANCELLED"],
    "tasks/04_watch.py": ["任务状态： RUNNING", "任务状态： SUCCEEDED"],
    "voice/01_text_stop_task.py": ["停止受理： True", "任务结果： CANCELLED"],
    "voice/02_text_stop_speech.py": ["播报结果： CANCELLED", "机器人结果： SUCCEEDED"],
    "voice/03_text_query.py": ["识别意图： query", "任务结果： SUCCEEDED"],
    "transport/01_query.py": ["节点： wrs", "当前位置： home"],
    "transport/02_events.py": ["收到事件：", "动作结果： SUCCEEDED"],
    "transport/03_submit.py": ["动作编号：", "动作结果： SUCCEEDED"],
    "transport/04_timeout.py": ["查询超时，没有取消任何动作。", "仍可查询机器人： home"],
    "wrs/01_move.py": ["动作结果： SUCCEEDED", "关节角：", "TCP 位置："],
    "wrs/02_cancel.py": ["动作结果： CANCELLED", "停止已确认： True"],
    "wrs/03_new_action_after_cancel.py": [
        "旧动作： CANCELLED",
        "允许新动作： True",
        "新动作： SUCCEEDED",
    ],
    "wrs/08_scene.py": ["场景坐标系： world", "物体： table", "物体： A", "碰撞规划已验证： False"],
    "wrs/04_move_relative.py": ["上 SUCCEEDED", "下 SUCCEEDED", "左 SUCCEEDED", "右 SUCCEEDED"],
}
# Real service/client groups are exercised together in test_developer_examples.py.
PAIRED = {
    "wrs/05_start_node.py",
    "wrs/06_control_arm.py",
    "wrs/07_viewer.py",
    "nodes/00_start_router.py",
    "nodes/01_start_speaker.py",
    "nodes/02_start_agent.py",
    "nodes/03_call_skill.py",
    "nodes/04_task.py",
    "nodes/05_cancel.py",
}
# Explicit audio/model or interactive examples; never run by the default verifier.
LIVE = {
    "wrs/15_imported_kitchen.py",
    "models/01_plan.py", "models/02_execute.py",
    "voice/05_start_wrs_voice.py", "voice/06_push_to_talk.py",
    "voice/07_start_llm_voice.py", "voice/08_listen_goals.py", "voice/09_viewer.py",
    "tts/01_start_node.py", "tts/02_speak.py", "tts/03_cancel.py",
}
SUPPORT = {"nodes/greet_skill.py", "voice/commands.py", "_session.py"}


def check_catalog(root):
    actual = {str(path.relative_to(root)).replace("\\", "/") for path in root.rglob("*.py")}
    expected = set(WRS) | PAIRED | LIVE | SUPPORT
    if actual != expected:
        raise ValueError(f"Example catalog mismatch: {actual ^ expected}")
