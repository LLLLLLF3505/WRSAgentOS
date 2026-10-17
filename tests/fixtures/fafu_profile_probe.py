"""Headless default scene and motion regression; no hardware."""

import asyncio
import json

from wrs_agent.env.wrs import open_model
from wrs_agent.schemas import KinematicState, SceneData


async def main():
    call, model, state, close = await open_model()
    try:
        assert state.facts["robot_model"] == "fafu_baseV1"
        assert set(state.objects) == {
            "table",
            "pot",
            "pot_lid",
            "induction_cooker",
            "measuring_cup",
            "skimmer",
            "plate",
            "plate_block",
            "skimmer_holder",
            "bowl_support",
            "bowl_rack",
            "bowl_support_2",
            "bowl_rack_2",
            "bowl_food_1",
            "bowl_food_2",
            "bowl_food_3",
            "bowl_food_4",
        }
        assert state.objects["measuring_cup"].alpha == 0.28
        SceneData.model_validate_json(state.snapshot().model_dump_json())
        kin = KinematicState.model_validate(await call(model.read))
        assert len(kin.qs) == 6 and kin.gripper_width is None and kin.tcp_name == "tool_link"
        np = model.wrs.np
        for pose in ("B", "C", "home", "B"):
            await call(model.begin, pose, 5)
            for i in range(1, 5):
                await call(model.step, i)
            assert await call(model.verify)
        for delta in ([0, 0, 0.02], [0, 0, -0.02], [0, 0.02, 0], [0, -0.02, 0]):
            before = await call(model.read)
            await call(model.begin_relative, delta, 5)
            for i in range(1, 5):
                after = await call(model.step, i)
            assert await call(model.verify)
            np.testing.assert_allclose(
                after["tcp_pos"], np.array(before["tcp_pos"]) + delta, atol=2e-4
            )
            np.testing.assert_allclose(after["tcp_rotmat"], before["tcp_rotmat"], atol=1e-3)
        print(
            json.dumps(
                {
                    "result": "PASS",
                    "objects": len(state.objects),
                    "active_joints": 6,
                    "named_moves": 4,
                    "relative_moves": 4,
                }
            )
        )
    finally:
        await close()


asyncio.run(main())
