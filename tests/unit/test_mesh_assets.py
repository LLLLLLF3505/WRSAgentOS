import pytest
from pydantic import ValidationError

from wrs_agent.env.wrs import ROOT
from wrs_agent.scene import load_scene
from wrs_agent.schemas import MeshGeometry, ObjectData, RobotData, SceneData, encode


@pytest.mark.parametrize(
    "asset",
    ["../secret.stl", "/tmp/a.stl", "C:/a.stl", "kitchen/../a.stl", "https://example/a.stl"],
)
def test_mesh_asset_rejects_external_paths(asset):
    with pytest.raises(ValidationError):
        MeshGeometry(asset=asset)


def test_default_kitchen_is_portable_and_bounded():
    objects = load_scene(ROOT / "wrs_agent/assets/kitchen/scene.toml")
    assert len(objects) == 8
    for obj in objects.values():
        if obj.geometry.kind == "mesh":
            assert (ROOT / "wrs_agent/assets" / obj.geometry.asset).is_file()
    assert objects["measuring_cup"].alpha == 0.28
    snapshot = SceneData(robot=RobotData(), objects=objects)
    assert len(encode(snapshot)) < 32768
    assert SceneData.model_validate_json(snapshot.model_dump_json()) == snapshot


def test_opacity_is_bounded():
    with pytest.raises(ValidationError):
        ObjectData(source="configuration", alpha=1.1)
