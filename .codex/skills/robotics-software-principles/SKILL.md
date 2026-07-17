---
name: robotics-software-principles
description: >
  Foundational software design principles applied specifically to robotics module development.
  Use this skill when designing robot software modules, structuring codebases, making architecture
  decisions, reviewing robotics code, or building reusable robotics libraries. Trigger whenever the
  user mentions SOLID principles for robots, modular robotics software, clean architecture for robots,
  dependency injection in robotics, interface design for hardware, real-time design constraints, error
  handling strategies for robots, configuration management, separation of concerns in
  perception-planning-control, composability of robot behaviors, or any discussion of software
  craftsmanship in a robotics context. Also trigger for code reviews of robotics code, refactoring
  robot software, or designing APIs for robotics libraries. Also covers robot architecture patterns:
  behavior trees vs finite state machines, the layered robot software stack, sensor fusion
  architecture, safety systems with watchdogs and heartbeats, graceful degradation, hardware
  abstraction layers (HAL), sim-to-real architecture, and episode data recording for learning-based
  robotics.
---

# Robotics Software Design Principles

## Why Robotics Software Is Different

Robotics code operates under constraints that most software never faces:

1. **Physical consequences** — A bug doesn't just crash a process, it crashes a robot into a wall
2. **Real-time deadlines** — Missing a 1ms control loop deadline can cause oscillation or damage
3. **Sensor uncertainty** — All inputs are noisy, delayed, and occasionally wrong
4. **Hardware diversity** — Same algorithm must work on 10 different grippers from 5 vendors
5. **Sim-to-real gap** — Code must run identically in simulation and on real hardware
6. **Long-running operation** — Robots run for hours/days; memory leaks and drift matter
7. **Safety criticality** — Some failures must NEVER happen, regardless of software state

These constraints demand disciplined design. Below are principles that account for them.

---

## Principle 1: Single Responsibility — One Module, One Job

Every module (node, class, function) should have exactly ONE reason to change.

**Why it matters in robotics**: A perception module that also does control means a camera driver update can break your arm controller. In safety-critical systems, this coupling is unacceptable.

```python
# ❌ BAD: God module — perception + planning + control + logging
class RobotController:
    def __init__(self):
        self.camera = RealSenseCamera()
        self.detector = YOLODetector()
        self.planner = RRTPlanner()
        self.arm = UR5Driver()
        self.logger = DataLogger()

    def run(self):
        image = self.camera.capture()
        objects = self.detector.detect(image)
        path = self.planner.plan(objects[0].pose)
        self.arm.execute(path)
        self.logger.log(image, objects, path)
        # If ANY of these changes, you touch this class

# ✅ GOOD: Separated responsibilities with clear interfaces
class PerceptionModule:
    """ONLY responsibility: raw sensor data → detected objects"""
    def __init__(self, camera: CameraInterface, detector: DetectorInterface):
        self.camera = camera
        self.detector = detector

    def get_detections(self) -> List[Detection]:
        image = self.camera.capture()
        return self.detector.detect(image)

class PlanningModule:
    """ONLY responsibility: goal + world state → trajectory"""
    def __init__(self, planner: PlannerInterface):
        self.planner = planner

    def plan_to(self, target: Pose, obstacles: List[Obstacle]) -> Trajectory:
        return self.planner.plan(target, obstacles)

class ExecutionModule:
    """ONLY responsibility: trajectory → hardware commands"""
    def __init__(self, arm: ArmInterface):
        self.arm = arm

    def execute(self, trajectory: Trajectory) -> ExecutionResult:
        return self.arm.follow_trajectory(trajectory)
```

**Test**: Can you describe what a module does WITHOUT using "and"? If not, split it.

---

## Principle 2: Dependency Inversion — Depend on Abstractions, Not Hardware

High-level modules (planning, behavior) should never depend on low-level modules (drivers, hardware). Both should depend on abstractions.

**Why it matters in robotics**: This is the foundation of sim-to-real. If your planner imports `UR5Driver` directly, it can't run in simulation. If it depends on `ArmInterface`, you swap implementations freely.

```python
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional
import numpy as np

# ─── ABSTRACTIONS (the contracts) ────────────────────────────

class ArmInterface(ABC):
    """Abstract arm — every arm implementation must honor this contract"""

    @abstractmethod
    def get_joint_positions(self) -> np.ndarray:
        """Returns current joint positions in radians"""
        ...

    @abstractmethod
    def get_ee_pose(self) -> Pose:
        """Returns current end-effector pose"""
        ...

    @abstractmethod
    def move_to_joints(self, positions: np.ndarray,
                        velocity: float = 0.5) -> bool:
        """Move to joint positions. Returns True on success."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """Immediately stop all motion"""
        ...

    @property
    @abstractmethod
    def joint_limits(self) -> List[tuple]:
        """Returns [(min, max)] for each joint"""
        ...


class CameraInterface(ABC):
    """Abstract camera — any RGB camera must honor this"""

    @abstractmethod
    def capture(self) -> np.ndarray:
        """Returns (H, W, 3) uint8 RGB image"""
        ...

    @abstractmethod
    def get_intrinsics(self) -> CameraIntrinsics:
        """Returns camera intrinsic parameters"""
        ...

    @property
    @abstractmethod
    def resolution(self) -> tuple:
        """Returns (width, height)"""
        ...


class GripperInterface(ABC):
    @abstractmethod
    def open(self, width: float = 1.0) -> bool: ...

    @abstractmethod
    def close(self, force: float = 0.5) -> bool: ...

    @abstractmethod
    def get_width(self) -> float: ...

    @abstractmethod
    def is_grasping(self) -> bool: ...


# ─── CONCRETE IMPLEMENTATIONS ────────────────────────────────

class UR5Arm(ArmInterface):
    """Real UR5 via RTDE protocol"""
    def __init__(self, ip: str):
        self.rtde = RTDEControl(ip)
        self.rtde_receive = RTDEReceive(ip)

    def get_joint_positions(self) -> np.ndarray:
        return np.array(self.rtde_receive.getActualQ())

    def move_to_joints(self, positions, velocity=0.5):
        self.rtde.moveJ(positions.tolist(), velocity)
        return True

    def stop(self):
        self.rtde.stopScript()

    @property
    def joint_limits(self):
        return [(-2*np.pi, 2*np.pi)] * 6


class MuJoCoArm(ArmInterface):
    """Simulated arm in MuJoCo — SAME interface"""
    def __init__(self, model_path: str, joint_names: List[str]):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                          for n in joint_names]

    def get_joint_positions(self) -> np.ndarray:
        return np.array([self.data.qpos[jid] for jid in self.joint_ids])

    def move_to_joints(self, positions, velocity=0.5):
        # Simulate motion with position control
        self.data.ctrl[:len(positions)] = positions
        for _ in range(100):
            mujoco.mj_step(self.model, self.data)
        return True

    def stop(self):
        self.data.ctrl[:] = 0


# ─── HIGH-LEVEL CODE DEPENDS ONLY ON ABSTRACTIONS ────────────

class PickPlaceTask:
    """This class works with ANY arm + gripper + camera.
    It never knows or cares if it's sim or real."""

    def __init__(self, arm: ArmInterface, gripper: GripperInterface,
                 camera: CameraInterface, detector: DetectorInterface):
        self.arm = arm
        self.gripper = gripper
        self.camera = camera
        self.detector = detector

    def execute(self, target_class: str) -> bool:
        image = self.camera.capture()
        detections = self.detector.detect(image)
        target = next((d for d in detections if d.label == target_class), None)
        if target is None:
            return False

        self.arm.move_to_joints(self.ik(target.pose))
        self.gripper.close()
        self.arm.move_to_joints(self.place_joints)
        self.gripper.open()
        return True
```

**The Dependency Rule in Robotics**:
```
Application / Tasks
    ↓ depends on
Interfaces (ABC)
    ↑ implements
Hardware Drivers / Simulators
```

Arrows point inward. High-level policy never imports low-level drivers.

---

## Principle 3: Open-Closed — Extend Without Modifying

Modules should be open for extension but closed for modification. Add new capabilities by adding new code, not changing existing code.

**Why it matters in robotics**: You constantly add new sensors, new robots, new tasks. If adding a new camera requires modifying your perception pipeline, you'll break existing deployments.

```python
# ❌ BAD: Adding a new sensor requires modifying existing code
class PerceptionPipeline:
    def process(self, sensor_type: str, data):
        if sensor_type == 'realsense':
            return self._process_realsense(data)
        elif sensor_type == 'zed':
            return self._process_zed(data)
        elif sensor_type == 'oakd':    # New sensor = modify this class
            return self._process_oakd(data)

# ✅ GOOD: Plugin architecture — add sensors without touching core
class SensorPlugin(ABC):
    """Base class for all sensor plugins"""
    @abstractmethod
    def name(self) -> str: ...

    @abstractmethod
    def process(self, raw_data) -> ProcessedData: ...

    @abstractmethod
    def get_intrinsics(self) -> dict: ...


class RealSensePlugin(SensorPlugin):
    def name(self): return 'realsense'
    def process(self, raw_data):
        # RealSense-specific processing
        return ProcessedData(...)


class ZEDPlugin(SensorPlugin):
    def name(self): return 'zed'
    def process(self, raw_data):
        # ZED-specific processing
        return ProcessedData(...)


# Core pipeline never changes when you add sensors
class PerceptionPipeline:
    def __init__(self):
        self._plugins: dict[str, SensorPlugin] = {}

    def register_sensor(self, plugin: SensorPlugin):
        """Extend the pipeline without modifying it"""
        self._plugins[plugin.name()] = plugin

    def process(self, sensor_name: str, data):
        if sensor_name not in self._plugins:
            raise ValueError(f"Unknown sensor: {sensor_name}")
        return self._plugins[sensor_name].process(data)


# Adding OAK-D = add a file, register at startup. Zero changes to core.
class OAKDPlugin(SensorPlugin):
    def name(self): return 'oakd'
    def process(self, raw_data):
        return ProcessedData(...)

pipeline = PerceptionPipeline()
pipeline.register_sensor(RealSensePlugin())
pipeline.register_sensor(OAKDPlugin())  # No core code changed
```

---

## Principle 4: Interface Segregation — Small, Focused Interfaces

Don't force modules to depend on interfaces they don't use. Many small interfaces beat one large one.

**Why it matters in robotics**: A simple 1-DOF gripper shouldn't implement a 6-DOF dexterous hand interface. A fixed camera shouldn't implement pan-tilt methods.

```python
# ❌ BAD: Fat interface — every camera must implement ALL of these
class CameraInterface(ABC):
    @abstractmethod
    def capture_rgb(self) -> np.ndarray: ...
    @abstractmethod
    def capture_depth(self) -> np.ndarray: ...
    @abstractmethod
    def capture_pointcloud(self) -> np.ndarray: ...
    @abstractmethod
    def set_exposure(self, value: float): ...
    @abstractmethod
    def set_pan_tilt(self, pan: float, tilt: float): ...
    @abstractmethod
    def stream_video(self) -> Iterator[np.ndarray]: ...
    # A simple USB webcam can't do half of these!

# ✅ GOOD: Segregated interfaces — implement only what you support
class RGBCamera(ABC):
    """Any camera that produces RGB images"""
    @abstractmethod
    def capture_rgb(self) -> np.ndarray: ...

    @property
    @abstractmethod
    def resolution(self) -> tuple: ...

class DepthCamera(ABC):
    """Cameras that also produce depth"""
    @abstractmethod
    def capture_depth(self) -> np.ndarray: ...

    @abstractmethod
    def get_depth_intrinsics(self) -> DepthIntrinsics: ...

class ControllableCamera(ABC):
    """Cameras with adjustable settings"""
    @abstractmethod
    def set_exposure(self, value: float): ...

    @abstractmethod
    def set_white_balance(self, value: float): ...

class PTZCamera(ABC):
    """Pan-tilt-zoom cameras"""
    @abstractmethod
    def set_pan_tilt(self, pan: float, tilt: float): ...

    @abstractmethod
    def set_zoom(self, level: float): ...


# A RealSense implements RGB + Depth, but not PTZ
class RealSenseD435(RGBCamera, DepthCamera, ControllableCamera):
    def capture_rgb(self): ...
    def capture_depth(self): ...
    def set_exposure(self, value): ...
    # No PTZ methods — it's not a PTZ camera!

# A webcam implements only RGB
class USBWebcam(RGBCamera):
    def capture_rgb(self): ...
    # Nothing else required

# Perception code that only needs RGB doesn't pull in depth dependencies
class ObjectDetector:
    def __init__(self, camera: RGBCamera):  # Only needs RGB
        self.camera = camera

    def detect(self) -> List[Detection]:
        image = self.camera.capture_rgb()
        return self.model.predict(image)
```

---

## Principle 5: Liskov Substitution — Replaceable Implementations

Any implementation of an interface must be substitutable without the caller knowing. If your code works with `ArmInterface`, it must work with ANY arm that implements it.

**Why it matters in robotics**: Sim-to-real transfer, hardware swaps, and multi-robot support all depend on this.

```python
# ❌ BAD: Violates substitution — caller must know the implementation
class FrankaArm(ArmInterface):
    def move_to_joints(self, positions, velocity=0.5):
        if len(positions) != 7:
            raise ValueError("Franka has 7 joints!")  # Franka-specific
        # ...

class UR5Arm(ArmInterface):
    def move_to_joints(self, positions, velocity=0.5):
        if len(positions) != 6:
            raise ValueError("UR5 has 6 joints!")  # UR5-specific
        # ...

# Caller must know which arm it's using to pass correct joint count!
# This breaks substitutability.

# ✅ GOOD: Self-describing implementations
class ArmInterface(ABC):
    @property
    @abstractmethod
    def num_joints(self) -> int: ...

    @property
    @abstractmethod
    def joint_limits(self) -> List[tuple]: ...

    @abstractmethod
    def move_to_joints(self, positions: np.ndarray, velocity: float = 0.5) -> bool:
        """Positions must have length == self.num_joints"""
        ...

class FrankaArm(ArmInterface):
    @property
    def num_joints(self): return 7

    def move_to_joints(self, positions, velocity=0.5):
        assert len(positions) == self.num_joints
        # ...

# Caller code is generic — works with any arm
def move_to_home(arm: ArmInterface):
    home = np.zeros(arm.num_joints)  # Queries the arm, doesn't assume
    arm.move_to_joints(home)
```

**Substitution test**: Take every line of caller code. Replace `UR5` with `Franka` with `SimArm`. Does it still work? If not, your abstraction leaks.

---

## Principle 6: Separation of Rates — Respect Timing Boundaries

Different subsystems run at different rates. Never couple them.

```
Component          Typical Rate     Criticality
─────────────────────────────────────────────────
Safety monitor     1000 Hz          HARD real-time
Joint controller   500-1000 Hz      HARD real-time
Trajectory exec    100-200 Hz       Firm real-time
State estimation   50-200 Hz        Firm real-time
Perception         10-30 Hz         Soft real-time
Planning           1-10 Hz          Best effort
Task management    0.1-1 Hz         Best effort
Logging            1-30 Hz          Best effort
UI/Dashboard       1-10 Hz          Best effort
```

```python
# ❌ BAD: Perception blocks the control loop
class Robot:
    def control_loop(self):  # Must run at 100Hz = 10ms budget
        image = self.camera.capture()           # 5ms
        objects = self.detector.detect(image)    # 200ms ← BLOCKS!
        pose = self.estimate_pose(objects)       # 2ms
        cmd = self.controller.compute(pose)      # 0.1ms
        self.arm.send_command(cmd)               # 0.5ms
        # Total: 207ms. Control runs at 5Hz instead of 100Hz!

# ✅ GOOD: Decoupled rates with async boundaries
class Robot:
    def __init__(self):
        self.latest_detections = []
        self.detection_lock = threading.Lock()

        # Perception runs in its own thread at its own rate
        self.perception_thread = threading.Thread(
            target=self._perception_loop, daemon=True)
        self.perception_thread.start()

    def _perception_loop(self):
        """Runs at ~10Hz — as fast as the detector allows"""
        while self.running:
            image = self.camera.capture()
            detections = self.detector.detect(image)
            with self.detection_lock:
                self.latest_detections = detections

    def control_loop(self):
        """Runs at 100Hz — NEVER blocked by perception"""
        rate = Rate(100)  # 10ms period
        while self.running:
            with self.detection_lock:
                detections = self.latest_detections  # Latest available

            pose = self.estimate_pose(detections)
            cmd = self.controller.compute(pose)
            self.arm.send_command(cmd)
            rate.sleep()
```

**Rule**: If subsystem A is slower than subsystem B, A must communicate to B via a buffer (topic, shared variable, queue) — never by direct call.

---

## Principle 7: Fail-Safe Defaults — Safe Until Proven Otherwise

Every module should default to the safest possible behavior. Safety is not a feature you add — it's the default you degrade from.

```python
# ❌ BAD: Unsafe defaults
class ArmController:
    def __init__(self):
        self.max_velocity = 3.14       # Full speed by default!
        self.collision_check = False    # Off by default!
        self.workspace_limits = None    # No limits by default!

# ✅ GOOD: Safe defaults — must explicitly opt into danger
class ArmController:
    def __init__(self):
        self.max_velocity = 0.1              # Crawl speed by default
        self.collision_check = True           # Always on
        self.workspace_limits = DEFAULT_SAFE_WORKSPACE  # Conservative box
        self.require_enable = True            # Must be explicitly enabled
        self._enabled = False

    def enable(self, operator_confirmed: bool = False):
        """Explicit enable step — requires operator confirmation for real hardware"""
        if not operator_confirmed and not self.is_simulation:
            raise SafetyError(
                "Real hardware requires operator confirmation to enable")
        self._enabled = True

    def move_to(self, target: np.ndarray, velocity: float = None):
        if not self._enabled:
            raise SafetyError("Controller not enabled")

        velocity = velocity or self.max_velocity

        # Clamp velocity to safe range
        velocity = min(velocity, self.max_velocity)

        # Check workspace limits BEFORE moving
        if not self.workspace_limits.contains(target):
            raise WorkspaceViolation(f"Target {target} outside safe workspace")

        # Check for collisions BEFORE moving
        if self.collision_check:
            if self.collision_detector.would_collide(target):
                raise CollisionRisk(f"Collision predicted for target {target}")

        return self._execute_move(target, velocity)
```

**The rule**: What happens when a module receives no input, invalid input, or loses communication? It should stop safely, not continue blindly.

```python
class SafetyDefaults:
    """Centralized safe defaults for the entire system"""

    # Communication loss → stop
    HEARTBEAT_TIMEOUT_MS = 500
    ACTION_ON_TIMEOUT = 'stop'           # Not 'continue_last_command'

    # Unknown state → stop
    ACTION_ON_UNKNOWN_STATE = 'stop'     # Not 'assume_safe'

    # Sensor failure → stop
    ACTION_ON_SENSOR_LOSS = 'stop'       # Not 'use_last_reading'

    # Joint limit approach → slow down
    JOINT_LIMIT_MARGIN_RAD = 0.05        # Stop 0.05 rad before limit
    VELOCITY_NEAR_LIMITS = 0.05          # Crawl near limits

    # Default workspace (conservative bounding box)
    WORKSPACE_MIN = np.array([-0.5, -0.5, 0.0])   # meters
    WORKSPACE_MAX = np.array([0.5, 0.5, 0.8])      # meters
```

---

## Principle 8: Configuration Over Code — Externalize Everything That Changes

Anything that might differ between deployments, robots, or environments should be in configuration, not code.

```python
# ❌ BAD: Hardcoded values scattered across files
class GraspPlanner:
    def plan(self, object_pose):
        approach_height = 0.15          # Magic number
        grasp_depth = 0.02              # Magic number
        if object_pose.z < 0.05:        # Magic number
            return None

# ✅ GOOD: Configuration-driven
# config/grasp_planner.yaml
# grasp_planner:
#   approach_height_m: 0.15
#   grasp_depth_m: 0.02
#   min_object_height_m: 0.05
#   max_grasp_width_m: 0.08
#   approach_velocity: 0.1
#   grasp_force_n: 10.0

@dataclass
class GraspConfig:
    approach_height_m: float = 0.15
    grasp_depth_m: float = 0.02
    min_object_height_m: float = 0.05
    max_grasp_width_m: float = 0.08
    approach_velocity: float = 0.1
    grasp_force_n: float = 10.0

    @classmethod
    def from_yaml(cls, path: str) -> 'GraspConfig':
        with open(path) as f:
            data = yaml.safe_load(f)
        return cls(**data.get('grasp_planner', {}))

    def validate(self):
        assert self.approach_height_m > 0, "Approach height must be positive"
        assert 0 < self.grasp_force_n < 100, "Force out of safe range"


class GraspPlanner:
    def __init__(self, config: GraspConfig):
        config.validate()
        self.config = config

    def plan(self, object_pose):
        if object_pose.z < self.config.min_object_height_m:
            return None
        # ...
```

**What goes in config**: robot IP addresses, joint limits, sensor parameters, safety thresholds, workspace boundaries, task-specific constants, file paths, feature flags.

**What stays in code**: algorithms, control logic, data structures, interface definitions, error handling.

---

## Principle 9: Idempotent Operations — Safe to Retry

Every command should be safe to send twice. Network drops, message duplicates, and retries are facts of life in robotics.

```python
# ❌ BAD: Non-idempotent — sending twice moves the robot twice as far
def move_relative(self, delta: np.ndarray):
    current = self.get_position()
    self.move_to(current + delta)
    # If this message is sent twice due to a retry,
    # the robot moves 2x the intended distance!

# ✅ GOOD: Idempotent — sending twice has the same effect as once
def move_to_absolute(self, target: np.ndarray, command_id: str):
    if command_id == self._last_executed_command:
        return  # Already executed this command, skip
    self._last_executed_command = command_id
    self.move_to(target)
    # Sending this twice is harmless — same target, same result

# ✅ GOOD: Idempotent gripper commands
def set_gripper(self, width: float):
    """Set gripper to absolute width — not open/close toggle"""
    self.gripper.move_to_width(width)
    # Calling set_gripper(0.04) ten times still results in 0.04m width
```

---

## Principle 10: Observe Everything — You Can't Debug What You Can't See

Every module should emit structured telemetry. When a robot behaves unexpectedly at 2 AM, logs are all you have.

```python
import structlog
from dataclasses import dataclass, asdict

logger = structlog.get_logger()

@dataclass
class PerceptionEvent:
    timestamp: float
    num_detections: int
    processing_time_ms: float
    frame_id: str
    detector_confidence: float

class PerceptionModule:
    def process(self, image):
        t_start = time.monotonic()
        detections = self.detector.detect(image)
        t_elapsed = (time.monotonic() - t_start) * 1000

        # Structured logging — machine-parseable
        event = PerceptionEvent(
            timestamp=time.time(),
            num_detections=len(detections),
            processing_time_ms=t_elapsed,
            frame_id=image.header.frame_id,
            detector_confidence=max(
                (d.confidence for d in detections), default=0.0),
        )
        logger.info("perception.processed", **asdict(event))

        # Performance warnings
        if t_elapsed > 100:
            logger.warning("perception.slow",
                processing_time_ms=t_elapsed,
                threshold_ms=100)

        # Anomaly detection
        if len(detections) == 0 and self._expected_objects > 0:
            logger.warning("perception.no_detections",
                expected=self._expected_objects,
                image_mean=float(image.data.mean()))

        return detections
```

**What to log**: state transitions, command executions, safety events, performance metrics, sensor health, error conditions, configuration changes.

**How to log**: structured key-value pairs (not printf strings), with timestamps, severity levels, and module identifiers.

---

## Principle 11: Composability — Build Complex Behaviors from Simple Ones

Design modules as composable building blocks. Complex robot behaviors should emerge from combining simple, tested primitives.

```python
# Primitive skills — simple, tested, reusable
class MoveTo(Skill):
    """Move end-effector to a target pose"""
    def execute(self, target: Pose) -> bool: ...

class Grasp(Skill):
    """Close gripper with force control"""
    def execute(self, force: float = 10.0) -> bool: ...

class Release(Skill):
    """Open gripper"""
    def execute(self) -> bool: ...

class LookAt(Skill):
    """Point camera at a target"""
    def execute(self, target: Point) -> bool: ...

class Detect(Skill):
    """Detect objects of a given class"""
    def execute(self, target_class: str) -> List[Detection]: ...


# Composite skills — built from primitives
class Pick(CompositeSkill):
    """Pick = Detect + MoveTo + Grasp"""
    def __init__(self, detect: Detect, move: MoveTo, grasp: Grasp):
        self.detect = detect
        self.move = move
        self.grasp = grasp

    def execute(self, object_class: str) -> bool:
        detections = self.detect.execute(object_class)
        if not detections:
            return False
        approach = compute_approach_pose(detections[0].pose)
        if not self.move.execute(approach):
            return False
        if not self.move.execute(detections[0].pose):
            return False
        return self.grasp.execute()


class Place(CompositeSkill):
    """Place = MoveTo + Release"""
    def __init__(self, move: MoveTo, release: Release):
        self.move = move
        self.release = release

    def execute(self, target: Pose) -> bool:
        if not self.move.execute(target):
            return False
        return self.release.execute()


class PickAndPlace(CompositeSkill):
    """PickAndPlace = Pick + Place — composed from compositions"""
    def __init__(self, pick: Pick, place: Place):
        self.pick = pick
        self.place = place

    def execute(self, object_class: str, target: Pose) -> bool:
        if not self.pick.execute(object_class):
            return False
        return self.place.execute(target)


# Dependency injection wires everything together at startup
def build_skill_library(arm, gripper, camera, detector):
    move = MoveTo(arm)
    grasp = Grasp(gripper)
    release = Release(gripper)
    look = LookAt(arm)
    detect = Detect(camera, detector)
    pick = Pick(detect, move, grasp)
    place = Place(move, release)
    pick_and_place = PickAndPlace(pick, place)
    return {
        'move': move, 'grasp': grasp, 'release': release,
        'pick': pick, 'place': place,
        'pick_and_place': pick_and_place,
    }
```

---

## Principle 12: Graceful Degradation — Work With What You Have

When components fail, the robot should degrade gracefully rather than stop entirely.

```python
class DegradedModeManager:
    """Manages capability degradation as components fail"""

    def __init__(self):
        self.capabilities = {
            'full_autonomy': {'requires': ['camera', 'lidar', 'arm', 'gripper']},
            'blind_manipulation': {'requires': ['arm', 'gripper']},
            'perception_only': {'requires': ['camera', 'lidar']},
            'safe_stop': {'requires': []},
        }
        self.active_components = set()

    def component_online(self, name: str):
        self.active_components.add(name)
        self._update_mode()

    def component_offline(self, name: str):
        self.active_components.discard(name)
        logger.warning(f"Component offline: {name}")
        self._update_mode()

    def _update_mode(self):
        """Find the best mode we can support with available components"""
        for mode, spec in self.capabilities.items():
            if set(spec['requires']).issubset(self.active_components):
                if mode != self.current_mode:
                    logger.info(f"Mode change: {self.current_mode} → {mode}")
                    self.current_mode = mode
                return
        self.current_mode = 'safe_stop'
        self._execute_safe_stop()
```

---

## Quick Reference: Principle Checklist

Use this during code reviews:

| # | Principle | Check |
|---|-----------|-------|
| 1 | Single Responsibility | Can you describe the module without "and"? |
| 2 | Dependency Inversion | Does high-level code import hardware drivers? |
| 3 | Open-Closed | Does adding a new sensor require modifying existing code? |
| 4 | Interface Segregation | Are implementations forced to stub out unused methods? |
| 5 | Liskov Substitution | Can you swap sim/real without changing caller code? |
| 6 | Separation of Rates | Does perception block the control loop? |
| 7 | Fail-Safe Defaults | What happens on communication loss? |
| 8 | Configuration Over Code | Are there magic numbers in the source? |
| 9 | Idempotent Operations | Is it safe to send every command twice? |
| 10 | Observe Everything | Can you diagnose a 2 AM failure from logs alone? |
| 11 | Composability | Can you build new tasks from existing skills? |
| 12 | Graceful Degradation | What's the robot's behavior when a sensor fails? |

---

## Architecture Patterns (merged from robotics-design-patterns)

### The Robot Software Stack

Every robot system follows this layered architecture, regardless of complexity:

```
┌─────────────────────────────────────────────┐
│               APPLICATION LAYER              │
│    Mission planning, task allocation, UI     │
├─────────────────────────────────────────────┤
│              BEHAVIORAL LAYER                │
│  Behavior trees, FSMs, decision-making       │
├─────────────────────────────────────────────┤
│             FUNCTIONAL LAYER                 │
│  Perception, Planning, Control, Estimation   │
├─────────────────────────────────────────────┤
│           COMMUNICATION LAYER                │
│     ROS2, DDS, shared memory, IPC            │
├─────────────────────────────────────────────┤
│          HARDWARE ABSTRACTION LAYER          │
│    Drivers, sensor interfaces, actuators     │
├─────────────────────────────────────────────┤
│              HARDWARE LAYER                  │
│    Cameras, LiDARs, motors, grippers, IMUs   │
└─────────────────────────────────────────────┘
```

**Design Rule**: Information flows UP through perception, decisions flow DOWN through control. Never let the application layer directly command hardware.

### Behavior Trees (BT)

Behavior trees are the **recommended default** for robot decision-making. They're modular, reusable, and easier to debug than FSMs for complex behaviors.

#### Core Node Types

```
Sequence (→)     : Execute children left-to-right, FAIL on first failure
Fallback (?)     : Execute children left-to-right, SUCCEED on first success
Parallel (⇉)     : Execute all children simultaneously
Decorator        : Modify a single child's behavior
Action (leaf)    : Execute a robot action
Condition (leaf) : Check a condition (no side effects)
```

#### Example: Pick-and-Place BT

```
                    → Sequence
                   /    |      \
            → Check     → Pick     → Place
           /    \      /   |  \     /  |  \
       Battery  Obj  Open  Move  Close Move Open Release
       OK?    Found? Grip  To    Grip  To   Grip
                      per  Obj   per   Goal per
```

#### Implementation Pattern

```python
import py_trees

class MoveToTarget(py_trees.behaviour.Behaviour):
    """Action node: Move robot to a target pose"""

    def __init__(self, name, target_key="target_pose"):
        super().__init__(name)
        self.target_key = target_key
        self.action_client = None

    def setup(self, **kwargs):
        """Called once when tree is set up — initialize resources"""
        self.node = kwargs.get('node')  # ROS2 node
        self.action_client = ActionClient(
            self.node, MoveBase, 'move_base')

    def initialise(self):
        """Called when this node first ticks — send the goal"""
        bb = self.blackboard
        target = bb.get(self.target_key)
        self.goal_handle = self.action_client.send_goal(target)
        self.logger.info(f"Moving to {target}")

    def update(self):
        """Called every tick — check progress"""
        if self.goal_handle is None:
            return py_trees.common.Status.FAILURE

        status = self.goal_handle.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        elif status == GoalStatus.STATUS_ABORTED:
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Called when node exits — cancel if preempted"""
        if new_status == py_trees.common.Status.INVALID:
            if self.goal_handle:
                self.goal_handle.cancel_goal()
                self.logger.info("Movement cancelled")

# Build the tree
def create_pick_place_tree():
    root = py_trees.composites.Sequence("PickAndPlace", memory=True)

    # Safety checks (Fallback: if any fails, abort)
    safety = py_trees.composites.Sequence("SafetyChecks", memory=False)
    safety.add_children([
        CheckBattery("BatteryOK", threshold=20.0),
        CheckEStop("EStopClear"),
    ])

    pick = py_trees.composites.Sequence("Pick", memory=True)
    pick.add_children([
        DetectObject("FindObject"),
        MoveToTarget("ApproachObject", target_key="object_pose"),
        GripperCommand("CloseGripper", action="close"),
    ])

    place = py_trees.composites.Sequence("Place", memory=True)
    place.add_children([
        MoveToTarget("MoveToPlace", target_key="place_pose"),
        GripperCommand("OpenGripper", action="open"),
    ])

    root.add_children([safety, pick, place])
    return root
```

#### Blackboard Pattern

```python
# The Blackboard is the shared memory for BT nodes
bb = py_trees.blackboard.Blackboard()

# Perception nodes WRITE to blackboard
class DetectObject(py_trees.behaviour.Behaviour):
    def update(self):
        detections = self.perception.detect()
        if detections:
            self.blackboard.set("object_pose", detections[0].pose)
            self.blackboard.set("object_class", detections[0].label)
            return Status.SUCCESS
        return Status.FAILURE

# Action nodes READ from blackboard
class MoveToTarget(py_trees.behaviour.Behaviour):
    def initialise(self):
        target = self.blackboard.get("object_pose")
        self.send_goal(target)
```

### Finite State Machines (FSM)

Use FSMs for **simple, well-defined sequential behaviors** with clear states. Prefer BTs for anything complex.

```python
from enum import Enum, auto
import smach  # ROS state machine library

class RobotState(Enum):
    IDLE = auto()
    NAVIGATING = auto()
    PICKING = auto()
    PLACING = auto()
    ERROR = auto()
    CHARGING = auto()

# SMACH implementation
class NavigateState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['target_pose'],
            output_keys=['final_pose'])

    def execute(self, userdata):
        # Navigation logic
        result = navigate_to(userdata.target_pose)
        if result.success:
            userdata.final_pose = result.pose
            return 'succeeded'
        return 'aborted'

# Build state machine
sm = smach.StateMachine(outcomes=['done', 'failed'])
with sm:
    smach.StateMachine.add('NAVIGATE', NavigateState(),
        transitions={'succeeded': 'PICK', 'aborted': 'ERROR'})
    smach.StateMachine.add('PICK', PickState(),
        transitions={'succeeded': 'PLACE', 'aborted': 'ERROR'})
    smach.StateMachine.add('PLACE', PlaceState(),
        transitions={'succeeded': 'done', 'aborted': 'ERROR'})
    smach.StateMachine.add('ERROR', ErrorRecovery(),
        transitions={'recovered': 'NAVIGATE', 'fatal': 'failed'})
```

**When to use FSM vs BT**:
- FSM: Linear workflows, simple devices, UI states, protocol implementations
- BT: Complex robots, reactive behaviors, many conditional branches, reusable sub-behaviors

### Perception Pipeline

```
Raw Sensors → Preprocessing → Detection/Estimation → Fusion → World Model
```

#### Sensor Fusion Architecture

```python
class SensorFusion:
    """Multi-sensor fusion using a central world model"""

    def __init__(self):
        self.world_model = WorldModel()
        self.filters = {
            'pose': ExtendedKalmanFilter(state_dim=6),
            'objects': MultiObjectTracker(),
        }

    def update_from_camera(self, detections, timestamp):
        """Camera provides object detections with high latency"""
        for det in detections:
            self.filters['objects'].update(
                det, sensor='camera',
                uncertainty=det.confidence,
                timestamp=timestamp
            )

    def update_from_lidar(self, points, timestamp):
        """LiDAR provides precise geometry with lower latency"""
        clusters = self.segment_points(points)
        for cluster in clusters:
            self.filters['objects'].update(
                cluster, sensor='lidar',
                uncertainty=0.02,  # 2cm typical LiDAR accuracy
                timestamp=timestamp
            )

    def update_from_imu(self, imu_data, timestamp):
        """IMU provides high-frequency attitude estimates"""
        self.filters['pose'].predict(imu_data, dt=timestamp - self.last_imu_t)
        self.last_imu_t = timestamp

    def get_world_state(self):
        """Query the fused world model"""
        return WorldState(
            robot_pose=self.filters['pose'].state,
            objects=self.filters['objects'].get_tracked_objects(),
            confidence=self.filters['objects'].get_confidence_map()
        )
```

#### The Perception-Action Loop Timing

```
Camera (30Hz)  ─┐
LiDAR (10Hz)   ─┼──→ Fusion (50Hz) ──→ Planner (10Hz) ──→ Controller (100Hz+)
IMU (200Hz)    ─┘

RULE: Controller frequency > Planner frequency > Sensor frequency
      This ensures smooth execution despite variable perception latency.
```

### Safety Systems

#### The Safety Hierarchy

```
Level 0: Hardware E-Stop (physical button, cuts power)
Level 1: Safety-rated controller (SIL2/SIL3, hardware watchdog)
Level 2: Software watchdog (monitors heartbeats, enforces limits)
Level 3: Application safety (collision avoidance, workspace limits)
```

#### Software Watchdog Pattern

```python
import threading
import time

class SafetyWatchdog:
    """Monitors system health and triggers safe stop on failures"""

    def __init__(self, timeout_ms=500):
        self.timeout = timeout_ms / 1000.0
        self.heartbeats = {}
        self.lock = threading.Lock()
        self.safe_stop_triggered = False

        # Start monitoring thread
        self.monitor_thread = threading.Thread(
            target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()

    def register_component(self, name: str, critical: bool = True):
        """Register a component that must send heartbeats"""
        with self.lock:
            self.heartbeats[name] = {
                'last_beat': time.monotonic(),
                'critical': critical,
                'alive': True
            }

    def heartbeat(self, name: str):
        """Called by components to signal they're alive"""
        with self.lock:
            if name in self.heartbeats:
                self.heartbeats[name]['last_beat'] = time.monotonic()
                self.heartbeats[name]['alive'] = True

    def _monitor_loop(self):
        while True:
            now = time.monotonic()
            with self.lock:
                for name, info in self.heartbeats.items():
                    elapsed = now - info['last_beat']
                    if elapsed > self.timeout and info['alive']:
                        info['alive'] = False
                        if info['critical']:
                            self._trigger_safe_stop(
                                f"Critical component '{name}' "
                                f"timed out ({elapsed:.1f}s)")
            time.sleep(self.timeout / 4)

    def _trigger_safe_stop(self, reason: str):
        if not self.safe_stop_triggered:
            self.safe_stop_triggered = True
            logger.critical(f"SAFE STOP: {reason}")
            self._execute_safe_stop()

    def _execute_safe_stop(self):
        """Bring robot to a safe state"""
        # 1. Stop all motion (zero velocity command)
        # 2. Engage brakes
        # 3. Publish emergency state to all nodes
        # 4. Log the event
        pass
```

#### Workspace Limits

```python
class WorkspaceMonitor:
    """Enforce that robot stays within safe operational bounds"""

    def __init__(self, limits: dict):
        self.joint_limits = limits['joints']    # {joint: (min, max)}
        self.cartesian_bounds = limits['cartesian']  # AABB or convex hull
        self.velocity_limits = limits['velocity']
        self.force_limits = limits['force']

    def check_command(self, command) -> SafetyResult:
        """Validate a command BEFORE sending to hardware"""
        violations = []

        # Joint limit check
        for joint, value in command.joint_positions.items():
            lo, hi = self.joint_limits[joint]
            if not (lo <= value <= hi):
                violations.append(
                    f"Joint {joint}={value:.3f} outside [{lo:.3f}, {hi:.3f}]")

        # Velocity check
        for joint, vel in command.joint_velocities.items():
            if abs(vel) > self.velocity_limits[joint]:
                violations.append(
                    f"Joint {joint} velocity {vel:.3f} exceeds limit")

        if violations:
            return SafetyResult(safe=False, violations=violations)
        return SafetyResult(safe=True)
```

### Sim-to-Real Architecture

```
┌────────────────────────────────────┐
│         Application Code           │
│  (Same code runs in sim AND real)  │
├──────────────┬─────────────────────┤
│   Sim HAL    │     Real HAL        │
│  (MuJoCo/    │  (Hardware          │
│   Gazebo/    │   drivers)          │
│   Isaac)     │                     │
└──────────────┴─────────────────────┘
```

**Key Principles**:
1. Application code NEVER knows if it's in sim or real
2. Same message types, same topic names, same interfaces
3. Use `use_sim_time` parameter to switch clock sources
4. Domain randomization happens INSIDE the sim HAL
5. Transfer learning adapters sit at the HAL boundary

```python
# Config-driven sim/real switching
class RobotDriver:
    def __init__(self, config):
        if config['mode'] == 'simulation':
            self.arm = SimulatedArm(config['sim'])
            self.camera = SimulatedCamera(config['sim'])
        elif config['mode'] == 'real':
            self.arm = UR5Driver(config['real']['arm_ip'])
            self.camera = RealSenseDriver(config['real']['camera_serial'])

        # Application code uses the same interface regardless
        self.perception = PerceptionPipeline(self.camera)
        self.planner = MotionPlanner(self.arm)
```

### Data Recording Architecture

**Critical for learning-based robotics** — designed for the ForgeIR ecosystem:

```
┌─────────────────────────────────────────────┐
│              Event-Based Recorder            │
│  Triggers: action boundaries, anomalies,     │
│  task completions, operator signals           │
├─────────────────────────────────────────────┤
│           Multimodal Data Streams            │
│  Camera (30Hz) | Joint State (100Hz) |       │
│  Force/Torque (1kHz) | Language Annotations  │
├─────────────────────────────────────────────┤
│            Storage Layer                     │
│  Episode-based structure with metadata       │
│  Format: MCAP / Zarr / HDF5 / RLDS          │
├─────────────────────────────────────────────┤
│           Quality Assessment                 │
│  Completeness checks, trajectory validation  │
│  Anomaly detection, diversity analysis       │
└─────────────────────────────────────────────┘
```

```python
class EpisodeRecorder:
    """Records robot episodes with event-based boundaries"""

    def __init__(self, config):
        self.streams = {}
        self.episode_active = False
        self.current_episode = None
        self.storage = StorageBackend(config['format'])  # Zarr, MCAP, etc.

    def register_stream(self, name, msg_type, frequency_hz):
        self.streams[name] = StreamConfig(
            name=name, type=msg_type, freq=frequency_hz)

    def start_episode(self, metadata: dict):
        """Begin recording an episode with metadata"""
        self.current_episode = Episode(
            id=uuid4(),
            start_time=time.monotonic(),
            metadata=metadata,  # task, operator, environment, etc.
            streams={name: [] for name in self.streams}
        )
        self.episode_active = True

    def record_step(self, stream_name, data, timestamp):
        if self.episode_active:
            self.current_episode.streams[stream_name].append(
                DataPoint(data=data, timestamp=timestamp))

    def end_episode(self, outcome: str, annotations: dict = None):
        """Finalize and store the episode"""
        self.episode_active = False
        self.current_episode.end_time = time.monotonic()
        self.current_episode.outcome = outcome
        self.current_episode.annotations = annotations

        # Validate before saving
        quality = self.validate_episode(self.current_episode)
        self.current_episode.quality_score = quality

        self.storage.save(self.current_episode)
        return self.current_episode.id
```

### Anti-Patterns to Avoid

#### 1. Polling Instead of Events
**Problem**: `while True: check_sensor(); sleep(0.01)`
**Fix**: Use callbacks, subscribers, event-driven architecture.

#### 2. No Error Recovery
**Problem**: Robot stops forever on first error.
**Fix**: Every action node needs a failure mode. Behavior trees with fallbacks.

#### 3. No Timestamps
**Problem**: Sensor data without timestamps — impossible to fuse or replay.
**Fix**: Timestamp EVERYTHING at the source. Use monotonic clocks for control.

#### 4. No Data Logging
**Problem**: Can't reproduce bugs, can't train models, can't audit behavior.
**Fix**: Always record. Event-based recording is cheap. Use MCAP format.

### Architecture Decision Checklist

When designing a new robot system, answer these questions:

1. **What's the safety architecture?** (E-stop, watchdog, workspace limits)
2. **What are the real-time requirements?** (Control at 100Hz+, perception at 10-30Hz)
3. **What's the behavioral framework?** (BT for complex, FSM for simple)
4. **How does sim-to-real work?** (HAL pattern, same interfaces)
5. **How is data recorded?** (Episode-based, event-triggered, with metadata)
6. **How are failures handled?** (Graceful degradation, recovery behaviors)
7. **What's the communication middleware?** (ROS2 for most cases)
8. **How is the system deployed?** (Docker, snap, direct install)
9. **How is it tested?** (Unit, integration, hardware-in-the-loop, field)
10. **How is it monitored?** (Heartbeats, metrics, dashboards)
