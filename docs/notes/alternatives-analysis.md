# 替代移植方案分析

> 日期:2026-06-28 夜
> ⚠️ 限制:本轮 WebSearch / WebFetch / GitHub API 均触发 429 / 速率限制,**无法实时拉取候选 README 与元数据做新验证**。以下基于:① 已有知识库调研 `CS-202618_类似开源项目调研.md`(已读,信息详细)② `git ls-remote` 可达性实测 ③ eyrc 实测与本项目自建经验。**建议限流恢复后让我重跑实时验证**(见文末)。

## 目标匹配维度
理想蓝本 = **麦克纳姆全向底盘 + 一体化(臂装底盘)+ Gazebo Classic 11 + ROS2 Humble + Nav2 + MoveIt2 + UR/类似臂**。

## 候选对比(均 git ls-remote 可达 ✓)

| 项目 | 一体化移动臂 | 底盘驱动 | Gazebo | ROS | 与目标差距 |
|---|---|---|---|---|---|
| **eyrc**(当前蓝本) | ✗ 分离(车+固定臂) | 差速 | **Classic** ✓ | **Humble** ✓ | 组件齐全;需自建一体化(**已完成**) |
| Spartan-Velanjeri/**UR-MiR** | ✓ 是(MiR+UR5e) | 差速(MiR) | Classic ✓ | Humble ✓ | 一体化✓但差速;含 Nav2+SLAM+AMCL+ArUco |
| aayush-rath/Nolon-stack | ✓ 是 | 未知 | 未知 | 未知 | Nav2+MoveIt 合并;细节待验证 |
| SeyeongW/ROS2-Mobile-Manip | ✓ 是 | 未知 | 未知 | Humble | 宠物照护场景;YOLOv8 |
| 9527114/ros2-gazebo-mm | ✓ 是 | 未知 | 未知 | 未知 | 含语音/抓取;license 不明 |
| darshmenon/multi-robot-fleet | ✓ 多机 | 未知 | 未知 | 未知 | 车队/Open-RMF;过重 |

## 结论:**坚持当前方案,不切换**

1. **没有"麦轮 + 一体化 + Gazebo Classic"完全匹配的开源项目**。麦克纳姆全向移动机械臂在开源里少见,绝大多数移动操作项目是差速/MiR 底盘。
2. **最接近的是 UR-MiR**(真一体化 + Classic + Humble + Nav2/AMCL/SLAM/ArUco),但:
   - 它是**差速 MiR 底盘**,切换后仍需"麦轮化"(和 eyrc 一样的工作);
   - 它的核心优势(臂装底盘的一体化组装)**我们已基于 eyrc 自建实现并验证**(26-link URDF,check_urdf 通过)。
3. **切换的净收益为负**:当前已完成一体化麦轮 URDF + 全栈 7 包 build + 25 单元测试 + MoveIt 配置加载验证。切换到任何替代 = 推倒重来,且替代项目均未实测可编译。

## 可借鉴(无需切换,作为后续增强参考)
- **UR-MiR**:MiR+UR 一体化 URDF 的关节/TF 组织;它的 SLAM Toolbox 建图流程(我们已用同源 eyrc 配置)。
- **Nolon-simulation-stack**(MIT):Nav2+MoveIt 移动操作的 launch 组织,可对比我们的 bringup。
- 这些可在运行时验证阶段、或冲刺增强时按需吸收单点,不影响主线。

## 待实时验证(API 限流恢复后,可让我重跑)
- UR-MiR / Nolon 的**实际可编译性**与 Gazebo 版本确认(clone + colcon build)。
- 搜索是否有更新的**麦轮全向移动机械臂 + Humble**项目(本轮 WebSearch 被限流未能执行)。
- 重跑 `alt-mobile-manip-research` workflow(脚本已存于 session,串行+重试版)。

## 实证验证更新(2026-06-28,git clone 实测 —— API 限流但 git 协议可用)

> nav-port subagent 恢复提示 Claude API 已恢复;GitHub API 仍限流(0/60),但 `git clone`(git 协议)可用,故对 top2 候选做了**实证**(depth-1 clone + 文件分析)。

### Nolon-simulation-stack → ❌ 排除
实测:**ROS Jazzy + Gazebo Sim/Ignition**(NewGz 信号 3、Classic 0),与本项目 **Humble + Gazebo Classic 不匹配**;且仅 `nolon_bot_description` 1 个包(无导航/抓取/集成)。不可用。

### Spartan-Velanjeri/UR-MiR → ✅ 真一体化 Classic 蓝本,但仍不建议切换
实测(212M,22 包):
- **Gazebo Classic** ✓(gazebo_ros 11 文件 vs NewGz 4)
- **MiR250(差速)底盘 + UR5e 一体化** ✓(ur5e/ur5/ur3 + mir),含 `mir_navigation`(Nav2)、`ur_moveit_config`、`ros2_aruco`、`realsense_gazebo_plugin`、`robotiq_description`
- **确实比 eyrc 更"一体化"**(eyrc 是分离式小车+固定臂;UR-MiR 是真·底盘+臂一体)

**但仍不切换,理由**:
1. 底盘是**差速 MiR**,切换后仍需麦轮化(与 eyrc 同等工作)。
2. **包极多极重**(含 mir_driver/mir_calibration/mir_restapi/ur_robot_driver 等真实硬件栈,仿真用需大量裁剪)。
3. **我们已基于 eyrc 自建麦轮一体化并验证通过**(26-link URDF + 全栈 build + 25 测试)——UR-MiR 的"一体化"优势我们已实现。切换 = 用差速重栈替换已验证的麦轮一体化 = **净损失**。

### 🔑 高价值可借鉴单点(无需切换,实证发现)
1. **UR-MiR 的 `realsense_gazebo_plugin` 是完整的**——而 eyrc 版缺 `gazebo_ros_realsense.cpp`(我们当前用标准 `gazebo_ros_camera` 替代)。**若要真 RealSense 仿真,可从 UR-MiR 取完整插件**。
2. **`ros2_aruco`**(标准 ArUco ROS2 包)——可替代我们自写的 `aruco_detector`,更成熟。
3. MiR+UR 一体化 URDF 的 TF/关节组织思路。

### 最终结论(实证后,不变)
**坚持"eyrc 组件 + 自建麦轮一体化"。** UR-MiR 经实测确认是优质参考但非切换对象(差速+重栈,且我们的一体化已自建验证);已锁定 2 个高价值可借鉴单点供后续按需吸收。Nolon 因版本不匹配排除。

---

## 一句话给用户
**当前"eyrc 组件 + 自建麦轮一体化"是现状下的最优路径**:我们已把别人没有的"麦轮一体化移动机械臂"做出来并验证通过。替代项目要么差速、要么未验证、要么场景不符,切换得不偿失。它们的可取之处(一体化思路)我们已实现。若你想我进一步**实测** UR-MiR 等是否真能编译,限流恢复后说一声即可。
