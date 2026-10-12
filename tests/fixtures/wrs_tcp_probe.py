"""Check real WRS TCP offsets without loading WRS in the pytest process."""

import json

from wrs_agent.env.wrs import POSES, URVirtualModel
from wrs_agent.schemas import KinematicState

model = URVirtualModel()
np = model.wrs.np
model.robot.fk(np.asarray(POSES["B"]))
# Give the named TCP a real tool offset and an orientation different from its link.
offset_rotmat = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
model.tcp.set_loc_rotmat_pos(rotmat=offset_rotmat, pos=np.array([0.0, 0.0, 0.08]))
before = KinematicState.model_validate(model.read())
assert before.tcp_name == "flange"
assert not np.allclose(before.tcp_pos, model.robot.gl_lnk_tfarr[-1, :3, 3])
assert not np.allclose(before.tcp_rotmat, model.robot.gl_lnk_tfarr[-1, :3, :3])

delta = np.array([0.0, 0.01, 0.0])
model.begin_relative(delta, count=4)
for index in range(1, 4):
    model.step(index)
assert model.verify()
after = KinematicState.model_validate(model.read())
np.testing.assert_allclose(after.tcp_pos, np.asarray(before.tcp_pos) + delta, atol=1e-4)
np.testing.assert_allclose(after.tcp_rotmat, before.tcp_rotmat, atol=1e-4)
np.testing.assert_allclose(after.qs, model.robot.qs, atol=1e-6)

# The joints can still match while the TCP target is invalid after a tool change.
model.tcp.set_loc_rotmat_pos(rotmat=offset_rotmat, pos=np.array([0.0, 0.0, 0.10]))
assert not model.verify()
# The hardware mirror retains the UR-specific gripper and six-joint model.
for command, width in (("close", 0.0), ("open", 0.05)):
    model.begin_gripper(command, 3)
    model.step(2)
    assert model.verify()
    assert abs(model.width - width) < 1e-6
assert len(model.robot.qs) == 6
print(json.dumps({"offset_tcp_motion": "PASS", "changed_tcp_rejected": "PASS", "hardware": False}))
