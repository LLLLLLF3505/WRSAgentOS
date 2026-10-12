"""静态查看正式默认 FAFU 厨具场景；与 05/07 共用模型和配置。"""

import argparse
import json
from pathlib import Path

from wrs_agent.env.wrs import ROOT, VirtualModel
from wrs_agent.scene import load_scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=ROOT / "wrs_agent/assets/kitchen/scene.toml")
    parser.add_argument("--port", type=int, default=8007)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    model = VirtualModel()
    objects = model.load_scene(load_scene(args.scene))
    print(
        json.dumps(
            {
                "result": "PASS",
                "scene": str(args.scene.resolve()),
                "objects": list(objects),
                "kinematics": model.read(),
                "physics_enabled": False,
                "collision_checked": False,
            },
            ensure_ascii=False,
            indent=2,
        ),
        flush=True,
    )
    if args.check:
        return
    from wrs_agent.env.viewer import viewer_hub

    world = model.wrs.wvw.World(
        cam_pos=(1.65, -1.85, 1.55),
        cam_lookat_pos=(0.3, 0, 0.23),
        port=args.port,
        hz=30,
        auto_start_hub=False,
    )
    for _, obj in model.objects.values():
        obj.add_to_scene(world.scene)
    model.robot.add_to_scene(world.scene)
    world.set_caption("FAFU | Default AgentOS kitchen | Static preview")
    try:
        with viewer_hub(args.port):
            print(f"打开 http://127.0.0.1:{args.port}；Ctrl+C 退出。", flush=True)
            world.run()
    finally:
        world.close()


if __name__ == "__main__":
    main()
