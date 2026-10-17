# 新增厨具与大锅替换（2026-10-17）

默认场景已换成大锅、大锅盖，并增加捞网架、碗与支架整体、碗架。
原来的对象 ID `pot`、`pot_lid` 保留，只有它们的模型资源、标签发生变化。
旧的 `pot.stl`、`pot_lid.stl` 已由用户删除，不需要恢复。

## 文件位置

- 运行配置：`wrs_agent/assets/kitchen/scene.toml`
- 运行网格：`wrs_agent/assets/kitchen/meshes/`
- Blender 场景：`design/kitchen/FAFU_kitchen.blend`
- 场景预览：`design/kitchen/preview.png`、`top_view.png`
- 原始毫米制 STL（含旧支架设计备份）：`design/kitchen/source_stl/`
- 来源、缩放、尺寸和摆放检查：`design/kitchen/import_manifest.json`
- 最新摆放、圆环支承高度和水平距离检查：`design/kitchen/layout_check.json`

运行只依赖项目内的配置和网格，不需要桌面“厨具”目录。`source_stl/` 为设计备份，不直接用于 WRS。

| 场景 ID | 厨具 | 运行 STL | 原始尺寸（毫米） | 摆放位置（米） |
|---|---|---|---|---|
| pot | 大锅 | large_pot.stl | 369 × 251.49 × 125（含把手） | [0.02, -0.29, 0.0450001359] |
| pot_lid | 大锅盖 | large_pot_lid.stl | 246 × 246 × 62 | [0.75, -0.35, 0] |
| skimmer_holder | 捞网架 | skimmer_holder.stl | 170 × 170 × 105 | [0.05, 0.36, 0] |
| bowl_support | 碗与支架整体 | bowl_support.stl | 228.6 × 167.2 × 84.5（含突出部分） | [0.36, -0.12, 0.040010003] |
| bowl_rack | 碗架 | bowl_rack.stl | 180 × 180 × 75 | [0.36, -0.12, 0] |
| bowl_support_2 | 碗与支架整体（第二套） | bowl_support.stl | 同第一套 | [0.36, 0.19, 0.040010003] |
| bowl_rack_2 | 碗架（第二套） | bowl_rack.stl | 同第一套 | [0.36, 0.19, 0] |

新增模型均从毫米缩放到米，系数为 0.001。保留 CAD 的水平设计原点，底部归零；运行 STL 不要再缩放一次。
按机械臂朝工作台内部（+X）定义前方，右侧为 -Y。锅和电磁炉一起放在机械臂右侧，水平中心为 [0.02, -0.29]。锅仍放在电磁炉上，锅盖平放在远离机械臂的桌角。漏勺已转为竖直姿态，篮筐放入捞网架圆环，篮底距桌面约 37.4 毫米；这是静态几何摆放，未建立物理约束或绑定。
碗与支架整体和碗架作为一套，放在机械臂前方 [0.36, -0.12]；第二套并排放在 [0.36, 0.19]，两套中心相距 0.31 米。
第二套的对象 ID 为 `bowl_support_2`、`bowl_rack_2`，分别复用原来的两个 STL 文件，独立配置位置和朝向，不复制网格文件。
新 STL 原始底部 z=-35 毫米，转换为米后将网格底部归零；运行位置 z≈0.04001 米。支架支承面落在碗架顶面 z=0.075 米处，碗身伸进圆环，碗底距桌面约 40 毫米。
对象 ID 和运行文件名仍为 `bowl_support` / `bowl_support.stl`；新原件保存为 `source_stl/bowl_with_support.stl`。旧 `source_stl/bowl_support.stl` 仅为历史设计备份，不再用于当前场景。
量筒移至 [0.69, -0.12, 0]，为锅盖腾出空间。
盘子与盘上长方体一起移动到 x=0.65、y=0.16，为两套碗架留出空间。
场景现在共 17 个对象，包含第二套的碗与支架整体、碗架和碗内 4 个食物示意块。
靠近捞网的第二套碗位于 [0.36, 0.19]，碗内新增 `bowl_food_1` 至 `bowl_food_4`。每块尺寸为 32 × 18 × 22 毫米，RGB 为 [0.95, 0.82, 0.42]。使用 TOML 的 box 几何体定义，无需额外 STL。
方块高度根据碗内表面采样设置，位于碗沿以下；这是静态食物示意，没有重力或接触物理。细节预览见 `design/kitchen/food_detail.png`。

## 查看

在项目根目录激活 `wrs` 环境后运行：

```powershell
.\scripts\run.ps1 examples/wrs/15_imported_kitchen.py --check
.\scripts\run.ps1 examples/wrs/15_imported_kitchen.py
```

浏览器打开 http://127.0.0.1:8007 。端口占用时，可先结束自己先前启动的预览，或追加 `--port 8017`。
正式 AgentOS 节点和观察器仍使用 `05_start_node.py`、`07_viewer.py`，重启它们后加载新场景。
修改 TOML 后需要重启相关进程；Blender 与 TOML 不会自动双向同步。

## 检查范围

导入时检查了所有厨具在桌面范围内、模型底部高度及水平方向包围盒。
厨具之间仅有五组预期叠放的水平方向包围盒重叠；新增的食物块在水平投影上还与盛放它们的第二套碗和碗架重叠。四个食物块之间留有间隙。
这不是精确网格碰撞或机械臂运动路径安全验证。当前场景仍不提供物理重力、接触响应或真实抓取。

## 与队员同步

提交运行 STL、默认 TOML、设计源 STL、更新后的 Blender 文件、预览、导入清单、本说明、来源说明和相关测试。
也提交旧 `pot.stl`、`pot_lid.stl` 的删除记录，避免队员继续使用旧文件。
不需要提交桌面路径、临时缓存或本地测试日志。

## 本次验证结果

- WRS 默认场景加载检查通过，返回 17 个场景对象和 6 个活动关节。
- `test_mesh_assets.py`、`test_scene.py`、`test_fafu_profile.py` 共 28 项测试通过。
- FAFU 集成测试包含 4 次命名姿态及 4 次相对移动检查。
- 实际 WRS 浏览器预览已检查，截图保存在 `design/kitchen/wrs_preview.png`。
- 验证仅涉及可视化和运动学，未测试真实硬件、重力、接触抓取或运动路径碰撞。
