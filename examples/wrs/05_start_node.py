"""启动独立 WRS 仿真节点与任务服务，供其他进程连接；Ctrl+C 关闭。"""

import asyncio
from pathlib import Path

from examples._session import use_local_token
from wrs_agent import System
from wrs_agent.env.wrs import ROOT

CONFIG = Path(__file__).with_name("bindings.toml")

SCENE = ROOT / "wrs_agent/assets/kitchen/scene.toml"

async def main():
    async with System.launch(
        backend="wrs", bindings=CONFIG, scene=SCENE, port=7449, env_id="wrs-demo", duration=1.0
    ) as system:
        print("WRS 节点已上线：tcp/127.0.0.1:7449", flush=True)
        snapshot = await system.snapshot()
        print("动作准入：", snapshot.admission, flush=True)
        print("场景物体：", list(snapshot.data.objects), flush=True)
        await asyncio.Event().wait()


if __name__ == "__main__":
    # 启动前准备共享口令，供 06_control_arm / 07_viewer 连接本服务。
    use_local_token("wrs", create=True)
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
