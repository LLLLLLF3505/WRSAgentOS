"""用真实 WRS FAFU 模型做虚拟 FK 运动，不连接硬件。"""

from wrs_agent import launch

if __name__ == "__main__":
    with launch(backend="wrs") as system:
        motion = system.action("move_named_pose", pose="B")
        result = motion.wait()
        state = system.snapshot()

        print("动作结果：", result.state)
        print("关节角：", state.data.robot.kinematics.qs)
        print("TCP 位置：", state.data.robot.kinematics.tcp_pos)
