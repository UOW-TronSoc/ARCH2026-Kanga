# kanga_description

Robot description package for Kanga platforms and payload assemblies.

This package contains:
- Core rover models (`kanga_core`, `kanga_core_simple`)
- Reusable payloads (autonomous payloads, arm payload, arm gripper payload)
- Reusable sensor models (UniLidarL2, D435i via `realsense2_description`, ZED via `zed_wrapper`)
- Top-level assemblies that compose core + payloads
- RViz and launch files for quick `robot_state_publisher` visualization

## Package Layout

- `urdf/core`
  - core model macros and standalone wrappers
- `urdf/payloads`
  - reusable payload macros and standalone wrappers
- `urdf/sensors`
  - reusable sensor macros/wrappers
- `urdf/assemblies`
  - composed systems (core + payloads)
- `meshes`
  - runtime mesh assets used by package URIs (`package://kanga_description/...`)
- `onshape_export`
  - source CAD exports from `onshape-to-robot`
- `launch`
  - RSP + RViz launch files
- `rviz`
  - RViz display configs

## Xacro Convention

This package follows a 2-layer xacro pattern.

1. `*_macro.urdf.xacro`
- Defines reusable macros with parameters (typically `parent`, optional `xyz/rpy`, optional `prefix`)
- No standalone root robot usage intent

2. `*_descr.urdf.xacro`
- Standalone wrapper for visualization/testing
- Includes the corresponding macro file and instantiates it

3. `urdf/assemblies/*_descr.urdf.xacro`
- Top-level compositions across core/payload/sensor macros

## Main Launch Files

Core:
- `ros2 launch kanga_description rsp_core.launch.py`
- `ros2 launch kanga_description rsp_core_simple.launch.py`

Autonomous:
- `ros2 launch kanga_description rsp_autonomous.launch.py`
- `ros2 launch kanga_description rsp_autonomous_simple.launch.py`
- `ros2 launch kanga_description rsp_autonomous_lidar_payload.launch.py`
- `ros2 launch kanga_description rsp_autonomous_zed_payload.launch.py`

Arm:
- `ros2 launch kanga_description rsp_arm_payload.launch.py`
- `ros2 launch kanga_description rsp_arm_gripper_payload.launch.py`
- `ros2 launch kanga_description rsp_arm_gripper.launch.py`
- `ros2 launch kanga_description rsp_kanga_arm_gripper.launch.py`

## Current Assemblies

- `urdf/assemblies/kanga_autonomous_descr.urdf.xacro`
  - `kanga_core` + autonomous lidar payload + autonomous zed payload
- `urdf/assemblies/kanga_autonomous_simple_descr.urdf.xacro`
  - `kanga_core_simple` + autonomous lidar payload + autonomous zed payload
- `urdf/assemblies/arm_gripper_descr.urdf.xacro`
  - arm payload + arm gripper payload (with D435i)
- `urdf/assemblies/kanga_arm_gripper_descr.urdf.xacro`
  - `kanga_core` + arm payload + arm gripper payload (with D435i)

## Dependencies

From `package.xml`:
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `xacro`
- `realsense2_description`
- `zed_wrapper`
- `zed_msgs`

If `zed_wrapper` or `realsense2_description` are missing, xacros that include those macros will fail.

## Build

From workspace root:

```bash
colcon build --packages-select kanga_description
source install/setup.bash
```

## Validate a Model Quickly

```bash
xacro src/kanga_description/urdf/assemblies/kanga_arm_gripper_descr.urdf.xacro > /tmp/model.urdf
check_urdf /tmp/model.urdf
```

## Onshape Export Update Workflow

Typical update process after re-export:

1. Export to `onshape_export/<model_name>/`
2. Copy updated meshes into runtime mesh path under `meshes/...`
3. Update the corresponding `*_macro.urdf.xacro` transforms/inertials/joints from exported URDF
4. Rebuild package
5. Validate with `xacro + check_urdf`
6. Launch related RSP file and verify TF/visual alignment in RViz

## Notes

- `rsp_` prefix means `robot_state_publisher` launch set.
- Payload and assembly names are intentionally separated:
  - payload xacros live in `urdf/payloads/...`
  - composed systems live in `urdf/assemblies/...`
- Use `prefix` macro args if composing multiple instances of the same module in one robot.
