# Chapter 6 Experiment Notes

- Frozen contact-feedback baseline: results/chapter6/final_baseline/。该目录用于复杂物体抓取成功率指标，结果为 28/30 = 93.33%。
- Force-control prototype: results/chapter6/force_control/。该实验用于验证夹爪力反馈闭环控制逻辑，不替代 frozen baseline。
- Force source: VIRTUAL_ESTIMATE，不是 RAW_GAZEBO_WRENCH、FSR402 或实体硬件触觉力。
- Arm control: MoveIt2 / Cartesian trajectory position control。Force feedback 只作用于 gripper closure DOF。
- 当前 virtual force 具有离散估计特性；Probe/IGBT 主要出现 0 N 和 20 N，Fixture 约 13.8 N。因此 target=8N 不能解释为精确力跟踪目标。
- 正确表述：基于虚拟触觉力估计的夹爪力反馈闭环控制仿真原型。
- 禁止表述：真实触觉传感器力控、FSR402 闭环、Gazebo raw wrench 闭环、全机械臂笛卡尔力/位混合控制、精确跟踪 8N。
