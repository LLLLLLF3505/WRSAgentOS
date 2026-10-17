# FAFU 默认厨具场景：团队交接说明

默认 `System.launch(backend="wrs")` 现在加载项目内 FAFU baseV1 和厨具场景。05 启动节点、07 观察器和 15 静态预览共用同一套模型和场景配置，不再依赖桌面文件、外部 fafu_arm_sdk 目录或 13 示例。

## 启动

继续使用已配置好的 Python 3.12 `wrs` 环境，并保持 `third_party/wrs` 子模块完整。新机器需按项目原安装说明准备 WRS 依赖，其中包括 `urdf_parser_py`；无需安装 FAFU 硬件控制 SDK。

在项目根目录打开三个终端，每个终端都激活环境：

```powershell
conda activate wrs
```

终端一，启动 AgentOS 节点：

```powershell
.\scripts\run.ps1 examples/wrs/05_start_node.py
```

终端二，显示节点的真实状态，浏览器打开 http://127.0.0.1:8000 ：

```powershell
.\scripts\run.ps1 examples/wrs/07_viewer.py
```

终端三，执行命名姿态以及上、下、左、右各 2 厘米的运动学演示：

```powershell
.\scripts\run.ps1 examples/wrs/06_control_arm.py
```

只看静态场景不需要启动节点：

```powershell
.\scripts\run.ps1 examples/wrs/15_imported_kitchen.py
```

静态预览端口是 8007，它不接收 06 的动作，也不反映节点运动。结束程序请在对应终端 Ctrl+C；同一服务不要重复启动。节点重启后显示 HELD 属于原项目的恢复机制，06 会在确认已停止后申请允许新动作。

## 文件分工

| 文件或目录 | 用途 |
|---|---|
| `wrs_agent/env/fafu.py` | 正式 URDF 加载器；只加载模型，不连接硬件 |
| `wrs_agent/env/fafu_backend.py` | 六活动关节映射、TCP、命名姿态、全位姿相对 IK/FK |
| `wrs_agent/env/wrs.py` | 默认选择 FAFU；加载和同步 box/STL 场景 |
| `wrs_agent/assets/fafu/profile.toml` | 底座位置、home/B/C 关节姿态；角度单位弧度 |
| `wrs_agent/assets/fafu/` | 项目内 URDF 和 7 个连杆 STL |
| `wrs_agent/assets/kitchen/scene.toml` | 唯一正式默认厨具配置，位置单位米 |
| `wrs_agent/assets/kitchen/meshes/` | 10 个已归一化到米的场景网格 |
| `wrs_agent/schemas.py` | 支持 mesh asset ID 和 alpha 的场景协议 |
| `design/kitchen/FAFU_kitchen.blend` | 可编辑 Blender 场景；不参与 AgentOS 运行 |
| `design/kitchen/preview.png` | Blender 场景预览 |
| `wrs_agent/assets/SOURCES.md` | 模型来源与单位说明 |

`scene.toml` 包含桌面、电磁炉、大锅、大锅盖、量杯、漏勺、盘子、盘上长方体、捞网架、两套碗与支架整体及碗架，以及第二套碗内 4 个淡黄色食物块，共 17 个对象。2026-10-17 的替换与新增资源说明见 [KITCHEN_ASSET_UPDATE.md](KITCHEN_ASSET_UPDATE.md)。量杯 alpha=0.28；机械臂基座 x=-0.125 米；长方体为 100×60×30 毫米。

改变位置请编辑物体 `pos`；改变颜色编辑 `rgb`；改变透明度编辑 `alpha`（0 透明，1 不透明）。STL 的 `asset` 相对 `wrs_agent/assets`，不接受磁盘绝对路径或 `../`。文件已经以米保存，不要再乘 0.001。Blender 文件与 TOML 不会自动双向同步。

## 能力和边界

- 支持 `observe`、`move_named_pose`、`move_relative`。TCP 是 URDF 的 `tool_link`，不是夹爪抓取中心。
- 向外只报告 6 个活动关节，内部固定 tool 关节不作为第七个控制量。
- home/B/C 是仿真演示姿态，不是实物标定或保证无碰撞的姿态。
- `set_gripper`、`pick`、`place` 不开放；没有 FAFU 夹爪、碰撞路径规划、重力或接触抓取。相对运动保持 TCP 朝向，但未验证路径碰撞。
- `ur_rtde` 后端显式使用原 UR7e/DH50 模型，不因默认仿真改为 FAFU 而切换硬件模型。
- 协议增加了 mesh/alpha 字段，节点和观察器需更新到同一版本。

## 旧示例清理

旧的 09、10、11、13、14 示例、旧做面 TOML 和旧 JSON/重复网格目录已经移除，示例清单已同步更新。当前 05、06、07、15 使用正式模块与 assets 资源。基础 scene.toml 仍供 08 示例和回归测试使用，继续保留。

## 验证

```powershell
.\scripts\run.ps1 examples/wrs/15_imported_kitchen.py --check
.\scripts\run.ps1 tests/fixtures/fafu_profile_probe.py
.\scripts\run.ps1 -m pytest tests/unit -q
.\scripts\run.ps1 -m pytest tests/integration/test_wrs.py tests/integration/test_wrs_examples.py tests/integration/test_wrs_scene.py tests/integration/test_wrs_tcp.py tests/integration/test_fafu_profile.py -q
```

最初 8 对象版本的历史验证：691 项单元测试通过；11 个不同集成测试通过（节点/客户端/观察器、UR TCP 与夹爪镜像、FAFU 默认配置）。实际 FAFU 8 对象、6 关节、4 次命名运动、4 次相对运动检查通过。未测试真实硬件或物理抓取。

## 提交到自己的 fork

先 `git status` 和 `git diff` 检查；在 VS Code 中按文件暂存本次改动，不使用“全部暂存”混入其他工作。需要提交的范围：

- `wrs_agent/env/fafu.py`、`fafu_backend.py`、`wrs.py`、`ur.py`
- `wrs_agent/schemas.py`、整个 `wrs_agent/assets/`、`pyproject.toml`
- `examples/wrs/01_move.py`、`05_start_node.py`、`07_viewer.py`、`15_imported_kitchen.py`
- `examples/voice/09_viewer.py`、`scripts/example_catalog.py`
- 本次修改/新增的 `tests/` 文件、本文、`design/kitchen/`
- Git 已跟踪的旧示例和旧配置删除记录也需暂存，让队员同步收到清理结果

不提交 `.local`、缓存、口令、测试临时日志。不要把子模块指针误改为未共享的本地提交。模型和 blend 都是二进制文件，会增加仓库大小；不要重复提交旧目录中的同一份网格。

建议提交说明：`集成 FAFU 默认后端和 STL 厨具场景`。本次未自动执行 git add、commit 或 push。
