"""FAFU model assets only: never imports the hardware SDK or opens a device."""

import xml.etree.ElementTree as ET
from pathlib import Path

from wrs_agent.env.wrs import load_wrs


def load_model(sdk_root=None, urdf_path=None):
    """读取模型文件；不 import FAFU 控制 SDK，也不会打开串口。"""
    wrs = load_wrs()
    import numpy as np
    from urdf_parser_py.urdf import URDF
    from wrs.robots.base.mech_base import MechBase
    from wrs.robots.base.urdf_loader import urdf_to_mechstruct

    description = (
        Path(sdk_root) / "fafu_robot_python" / "fafu_robot_description"
        if sdk_root is not None
        else Path(__file__).parents[1] / "assets/fafu"
    )
    path = (urdf_path or description / "urdf" / "fafu_baseV1.urdf").resolve()
    if not path.is_file():
        raise FileNotFoundError(f"找不到 URDF：{path}；请用 --sdk-root 指定 FAFU 仓库。")
    xml = ET.parse(path).getroot()
    # MuJoCo 编译扩展不是 URDF 关节结构；只从内存副本移除，源文件保持原样。
    for extra in list(xml.findall("mujoco")):
        xml.remove(extra)

    mesh_files = set()

    def resolve_mesh(filename):
        prefix = "package://fafu_robot_description/"
        if filename.startswith(prefix):
            resolved = description / filename[len(prefix) :]
        elif filename.startswith("package://"):
            raise ValueError(f"尚未适配该模型包：{filename}")
        else:
            resolved = path.parent / filename
        resolved = resolved.resolve()
        if not resolved.is_file():
            raise FileNotFoundError(f"模型缺少网格文件：{resolved}")
        mesh_files.add(str(resolved))
        return str(resolved)

    for mesh in xml.findall(".//mesh"):
        resolve_mesh(mesh.attrib["filename"])
        if not np.allclose(np.fromstring(mesh.get("scale", "1 1 1"), sep=" "), 1):
            raise ValueError("此示例尚未处理非单位 mesh scale，请先适配模型缩放。")
    urdf = URDF.from_xml_string(ET.tostring(xml, encoding="unicode"))
    structure = urdf_to_mechstruct(
        urdf,
        str(path.parent),
        mesh_resolver=resolve_mesh,
        res_dir=str(path.parent),
        collision_type=None,
    )

    # 每次加载创建独立子类，避免 MechBase 的类缓存混用不同 URDF。
    class FafuViewModel(MechBase):
        _structure = structure

    robot = FafuViewModel(is_floating=False)
    joints = [j for j in urdf.joints if j.type != "fixed"]
    if len(joints) != 6 or any(j.type != "revolute" for j in joints):
        raise ValueError("此示例要求 6 个旋转关节，请检查选择的 URDF。")
    # 按名称写入 WRS 的完整关节向量，不假定库内关节排序。
    index = {j.name: i for i, j in enumerate(structure.jnts)}
    joint_indices = np.array([index[j.name] for j in joints])
    names = [j.name for j in joints]
    if names != [f"joint{i}" for i in range(1, 7)]:
        raise ValueError(f"关节命名与示例初始姿态不一致：{names}")
    limits = np.array([[j.limit.lower, j.limit.upper] for j in joints], dtype=float)
    # 仅用于显示的展开姿态，不是硬件回零姿态；单位为弧度。
    rest = np.array([0, 1.57, 2.4, 0, 0, 0], dtype=float)
    amplitudes = np.minimum(0.2, np.minimum(rest - limits[:, 0], limits[:, 1] - rest) * 0.5)
    if not np.isfinite(limits).all() or np.any(amplitudes <= 0):
        raise ValueError("初始姿态不适合该 URDF 的关节范围，需要单独调整。")

    def apply(q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("关节角必须是 6 个有限数值。")
        if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
            raise ValueError("示例关节角超出了 URDF 限位。")
        full = np.zeros_like(robot.qs)
        full[joint_indices] = q
        robot.fk(full)

    apply(rest)
    return wrs, robot, urdf, xml, apply, rest, amplitudes, limits, path, sorted(mesh_files)
