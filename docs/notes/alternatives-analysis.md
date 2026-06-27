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

## 一句话给用户
**当前"eyrc 组件 + 自建麦轮一体化"是现状下的最优路径**:我们已把别人没有的"麦轮一体化移动机械臂"做出来并验证通过。替代项目要么差速、要么未验证、要么场景不符,切换得不偿失。它们的可取之处(一体化思路)我们已实现。若你想我进一步**实测** UR-MiR 等是否真能编译,限流恢复后说一声即可。
