---
name: lab-cobot-project-conventions
description: lab_cobot_ws project constitution — read before touching any file. Use when starting any session in this repo, before modifying launch defaults or tests, when colcon test fails (missing_result, D400/D415, E501, uncrustify), when E2E hangs at NAV_TO_PICK or service calls time out, or when deciding whether a dependency/parameter/assertion change is allowed.
---

# lab_cobot_ws 项目宪法

> **权威源**：`docs/MVP-SPEC-for-codex.md` §0.2–§0.8。本技能是其可检索快照，
> 冲突时以规格为准；规格更新时同步本文件（双库 `.claude/skills/` 与 `.codex/skills/` 各一份）。
> 快照日期：2026-07-08。

## 一、禁区（§0.5，违反即返工）

1. `src/pymoveit2/` 是 vendored 第三方——**不许动**。
2. 不给 ament_python 包（`lab_cobot_manipulation`/`lab_cobot_perception`）加 `ament_add_pytest_test`（colcon 自动跑其 test/，加注册会重复执行）。
3. `lab_cobot_gazebo` 既有测试注册行勿动，新增测试按现有模式**追加**行。
4. **默认值锁死**（`lab_cobot.launch.py`）：`llm_enabled=false`、`use_truth_pose=false`、`use_sim_attach=false`、`launch_mission=true`、`launch_voice=false` 不许改；**2026-07-10 T-5 起 `require_finger_contact=true`、`use_tactile_grasp=true` 为新锁死默认**（触觉门控抓取，两开关必须同值——只开门控不开触觉时固定闭合 0.009 指面永不接触，正常抓取全失败；回退开关保留但不许改回默认）；`use_gazebo_model_pose` 由 `use_truth_pose` 透传的接线不许断。新功能开关一律**新增参数**。
5. **诚实 E2E 断言只增不减**（`test_honest_e2e_launch.py`）：DONE 达成、物块落 B 台面盒（x∈[-2.4,-1.6]、y∈[1.2,1.8]、z>0.70）、`gravity_mode=True` 全程、`use_gazebo_model_pose=False`、`gripper_attach_bridge` 不在节点列表、base_link 系相机检测≥1——任何一条不许放松，只能增强。
6. **地图与来源链不许动**：`maps/{map.pgm,map.yaml,map_provenance.yaml}`、`check_map.py` 判据。
7. **抓取安全参数锁死**：`breakaway_force` 默认 0.0（有测试锁定，位姿驱动下力阈值不可分离——实测教训）；悬空释放 `PLACE_RELEASE_CLEARANCE=0.02` 不许回退。
8. **底盘不许动**：`control_mode=pose_from_wheel_commands` 默认不变；不得回退 planar_move；文档措辞禁用"力模型/真实滚子物理"（`docs/STATUS_HONEST.md` 的"不应再使用的旧说法"是硬约束）。
9. **诚实措辞纪律**：新文档不得宣称未实证能力；`STATUS_HONEST.md` 只能随实证更新。
10. `.claude/skills/`、`.codex/skills/` 不许动（技能正文追加踩坑条目除外：只改 `.codex` 侧并在报告声明）。
11. `nav2_params.yaml`/`ekf.yaml`/`waypoints.py` 除非条目明确涉及，不许顺手改。
12. **语言纪律**：不许把中文注释/文档英译；新代码 docstring **单行英文、ASCII 句点结尾**，中文解释放 `#` 注释。
13. **手指闭合几何**：`CLOSED_ON_SAMPLE_POSITIONS=[0.009,0.009]` 的非接触路径在 `use_tactile_grasp=false` 时必须逐字节保留（回归锁）；`setup.cfg` 的 anyio 配置不许删。
14. **colcon test 禁网络、禁 GPU、禁模型下载**（见 §四）。
15. 不升级 numpy / 不 `pip install opencv-*`（会牵连 cv_bridge，本机 cv2 双版本共存是已验证的现状）。

另（§0.2）：**禁止新写"读源码断言子串"的假测试**；launch 断言一律用 introspection 模式（范本 `src/lab_cobot_bringup/test/test_lab_cobot_launch.py`，方法见技能 `ros2-launch-introspection-testing`）。

## 二、环境已知故障（§0.6，遇到勿误判为自己改错）

1. **pep257 中文句号不认（D400/D415）+ manipulation 包 D213**：docstring 一律单行英文 `.` 结尾。自查 `python3 -m pytest src/<pkg>/test/test_pep257.py -p no:anyio -q`。
2. **flake8 E501 行长 99 / F401 残留**：中文注释拆行；删代码同步清 import。
3. **所有 pytest 注册必须 `ENV "PYTEST_ADDOPTS=-p no:anyio"`，colcon build/test 前 `export PYTEST_ADDOPTS='-p no:anyio'`**。根因：~/.local anyio 插件与系统 pytest 6 不兼容（降级 anyio 3.7.1 已根治，但 pip 依赖漂移可能复发——装 DL 依赖时 httpx 链曾把它升回 4.x，导致测试**被静默跳过**）。**`set(ENV{...})` 写法无效**（configure 期赋值，实测）。colcon test 出现 `missing_result` 先查此项。
4. **E2E/SIM 前必须无残留进程（不只 gzserver）**：`pkill -9 -x gzserver; pkill -9 -x gzclient` 只清 Gazebo；**残留的 move_group/bt_navigator 会造成同名双 action server**，pymoveit2/BasicNavigator 报 "more than one action server"、goal response 错乱、任务秒败（2026-07-10 实测，连续污染多轮排查）。三条铁律：① **禁用 `-f 'gzserver|gzclient'`**——自匹配 wrapper shell 自杀（exit 137，双方实测）；② **`pkill -x` 对 >15 字符进程名失效**（内核 comm 截断：`lifecycle_manager`/`controller_server`/`velocity_smoother`/`robot_state_publisher` 全打不中，实测清不干净的元凶）——全家桶清理用拆串路径模式 `PAT='/opt/ros/''humble/lib'; pgrep -f "$PAT" | xargs -r kill -9`（模式字符串必须拆开写防自匹配）；③ 后台 `ros2 launch ... &` 用完必须杀其根 pid，否则整栈存活污染后续实验。清完 `ps aux | grep /opt/ros` 确认为零再开跑。并行多 gzserver 需 `GAZEBO_MASTER_URI` 隔离。
5. **DDS 启动竞态**：节点刚起时 service call 单次 20s 假超时（实测）——启动期参数/服务读取一律套 3 次重试（范本 `test_honest_e2e_launch.py::_assert_truth_pose_disabled`）。
6. **改完必须 `colcon build --symlink-install --packages-select <pkg>` 再验证**：新增 .py 模块/entry point 不 build 就 import 不到（现象"改了没生效"）。
7. **长时测试单独注册**：>60s 的测试必须单独 `ament_add_pytest_test` 并加进目录注册的 `--ignore=`（范本 bringup CMakeLists 的 honest_e2e，TIMEOUT 600）。
8. **position 接口无限刚度**（源码级实证：gazebo_ros2_control 每物理步 `SetPosition`+`SetVelocity(0)`，无插值）：任何方案不得让手指**一步到位**挤压物块。
9. **新 C++ 文件 uncrustify**：不手改风格，`ament_uncrustify --reformat <files>` 一条命令解决；cppcheck skip ≠ 通过，关键逻辑必须有 gtest（范本 `test_grasp_envelope.cpp`）。
10. **首帧 GPU 推理 1.5–2.1s 是 CUDA 上下文正常现象**（实测），勿当 bug 修，勿设 <2s 看门狗。
11. `import open3d` 触发 scipy/numpy 版本 UserWarning（实测）——纯警告，不修、不动 numpy。
12. `rosdep resolve` 失败先 `rosdep update`；手跑 E2E 用 `ROS_LOCALHOST_ONLY=1`。
13. 测试几何常量：焊接偏移实测 **-0.065**（勿信旧值 -0.027，见 `test_mission_place_pose.py` 注释）。
14. **任何动启动链（launch/lifecycle/节点编排）的改动，验收必须含演示配置（`gui:=true`）下实测起停**：E2E 全部无头跑，gzclient 在 WSLg 下的高负载曾让 lifecycle_manager 编排超时挂死整栈（bt_navigator 停在 unconfigured，8 连跑无头全绿也没暴露——2026-07-10 演示实测）。防御配置（respawn/bond_timeout）缺失不会让测试变红，审查时必须核对。

## 三、完成判据分级（§0.7）

`[UT]` 单测通过　`[BLD]` colcon build 通过　`[TEST]` colcon test 全绿　`[SIM]` 起 Gazebo 按判据确认。
执行 agent 自主完成 [UT][BLD][TEST]；[SIM] 把命令与关键输出贴进报告，由人复核。

## 四、依赖与离线纪律（§0.8，违反即破坏全绿基线）

- **pip 包不进 package.xml**（ultralytics/open3d/torch/faster-whisper 无 Humble rosdep key）。本机已装：`ultralytics==8.4.90 / open3d==0.19.0 / torch==2.12.1+cu130 / faster_whisper==1.2.1`【实测】。
- **被测模块顶层禁止 import 重依赖**（`import ultralytics` 1276ms、`import open3d` 1247ms 实测）：一律函数内 lazy import；每个新模块配 ast 顶层依赖守卫测试。
- **模型权重不进 git、不在测试期下载**：`.gitignore` 有 `*.pt`；权重路径 `~/lab_cobot_models/`；whisper 缓存 `~/.cache/huggingface/`。
- **colcon test 在无 DL 包、无网络、无 GPU 的机器上也必须全绿**——推理经注入 fake 测试；GPU/真模型验证只在 `scripts/`/`tools/` 手动探针与 [SIM]。
- package.xml 唯一允许新增：`<exec_depend>vision_msgs</exec_depend>`（已验证）。

## 五、commit 与切分（§0.3/0.4）

- `<type>: <中文简述>`（feat/fix/refactor/docs/test/chore），body 用 `-` 列点、标注实测依据，**不加任何署名**。
- 每 Goal 按"依赖声明 → 纯逻辑 → 接线 → 加固 → 文档"切 commit，每个 commit 后 build+test 必须绿。
- **同文件合并组**（一次改完组内全部项再 commit，避免行号漂移）：`lab_cobot_grasp_fix.cpp`（S 先 T 后）、`parallel_gripper.xacro`（T-1 一次改完）、`lab_cobot.launch.py`（每 Goal 参数段单独 commit，改前 grep 锚点）、`aruco_sample/model.sdf`（T-1 一次改完）、`gripper_driver.py`（S 先 T 后）。

## 标准验证命令

```bash
export PYTEST_ADDOPTS='-p no:anyio'
colcon build --symlink-install --packages-select <pkg>
colcon test --packages-select <pkg> && colcon test-result --verbose
pkill -9 -x gzserver; pkill -9 -x gzclient   # E2E 前（禁用 -f 匹配，会自杀）
```
