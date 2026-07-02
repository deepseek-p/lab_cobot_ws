# Parallel Gripper Backend Design

## Objective

Replace the current vacuum suction end effector with a visually clear two-finger parallel gripper while keeping the Stage 5 mission stable. The first implementation uses simulated attach/detach for reliable demonstrations, but the manipulation code must keep a backend boundary so a future physical-contact gripper can replace attach/detach without rewriting navigation, perception, or mission orchestration.

## Context

The current robot mounts `vacuum_gripper.xacro` on `ur_tool0` and `PickPlace` calls `/suction/switch`. Runtime evidence suggests the suction plugin can report service success without a visually convincing grasp. Local reuse candidates were checked:

- `robot_lab_demo` has a practical parallel gripper model in `parallel_gripper_macro.xacro`, a `gripper_position_controller`, and a `soft_gripper_attach.py` bridge.
- The CS-202618 manipulation knowledge base recommends a staged flow: open gripper, approach, close gripper, attach object, lift, place, detach, open.

## Design

### End Effector Model

Create a parallel gripper macro in `lab_cobot_description` derived from `robot_lab_demo`:

- `gripper_base` fixed to `ur_tool0`
- `gripper_left_finger` and `gripper_right_finger` as prismatic joints
- `gripper_tcp` fixed frame between the fingertips

The vacuum macro should be replaced or no longer included by `lab_cobot.urdf.xacro`. Existing references to `suction_link`, `/suction/switch`, and `/suction/grasping` should be removed from the active runtime path.

### Controllers

Extend `lab_cobot_description/config/lab_cobot_controllers.yaml` with:

- `gripper_position_controller`
- two position-controlled finger joints

The existing UR arm `joint_trajectory_controller` remains unchanged. The gripper controller is command-only from the manipulation node using `std_msgs/Float64MultiArray` on `/gripper_position_controller/commands`.

### Grasp Backend Boundary

Introduce a small backend boundary in `lab_cobot_manipulation`:

- `GripperDriver.open()`
- `GripperDriver.close()`
- `GripperDriver.acquire_object()`
- `GripperDriver.release_object()`

The initial backend is `SimAttachGripperDriver`:

- `open()` publishes open finger positions.
- `close()` publishes closed-on-sample positions.
- `acquire_object()` attaches the sample to `gripper_tcp`.
- `release_object()` detaches the sample.

A future `PhysicalGripperDriver` can keep `open()` and `close()` but make `acquire_object()` a contact/state check and `release_object()` a no-op or state check after opening.

### Attach/Detach Implementation

Prefer a local Python bridge adapted from `robot_lab_demo` over relying on Gazebo contact physics. The bridge tracks the configured object pose and TCP transform, then keeps the object pose fixed relative to `gripper_tcp` while attached.

For this project the initial object is `aruco_sample`. Topics:

- `/gripper/attach/aruco_sample`
- `/gripper/detach/aruco_sample`

The bridge should launch with the full bringup stack and use Gazebo Classic state services/topics already available in this project. If a Gazebo state service is missing, the bridge should fail clearly in logs.

### Pick/Place Flow

`PickPlace.pick(pos)` becomes:

1. Open gripper.
2. Move to `above`.
3. Move to grasp pose.
4. Close gripper.
5. Acquire object through the active backend.
6. Move back to `above`.

`PickPlace.place(pos)` becomes:

1. Move to `above`.
2. Move to place pose.
3. Release object through the active backend.
4. Open gripper.
5. Move back to `above`.

Mission-level station retreat remains unchanged.

## Tests

Add or update focused tests:

- Description contract: gripper links, finger joints, and `gripper_tcp` exist in generated URDF.
- Description contract: vacuum plugin is absent from the active URDF.
- Controller contract: `gripper_position_controller` exists and commands both finger joints.
- Bringup contract: attach bridge node is launched.
- Manipulation unit tests: pick/place sequence calls open/close/acquire/release in the expected order.

Runtime validation:

1. Build affected packages.
2. Launch full stack with GUI or headless.
3. Trigger `把样件从A送到B`.
4. Confirm finger motion is visible, the sample follows after close/acquire, detaches at B, and mission ends `DONE`.

## Future Physical Gripper Migration

The future physical-contact version should only replace the backend implementation and tune model physics:

- finger collision geometry and friction
- sample mass/friction/contact parameters
- close distance/force
- grasp pose offsets
- object-acquired check

The mission state machine, Nav2 integration, perception TF lookup, station retreat, and MoveIt arm movement should not need structural changes.
