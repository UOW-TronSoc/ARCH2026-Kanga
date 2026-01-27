# kanga_arm_bringup

The `kanga_arm_bringup` package provides launch and configuration files to initialize and bring up the kanga arm robot system in simulation or on hardware. It handles launching the necessary ROS 2 nodes, setting parameters, and configuring interfaces for control,and visualization.

## Features

- Centralized launch file to start the kanga arm system
- Support for both simulation (e.g. Raisim) and real hardware
- Modular structure for including sensors, controllers, and visualization tools
- Parameterization for robot model, initial pose, and control frequency

## Launch Files

### `bringup.launch.py`

Main entry point for bringing up the kanga arm robot

#### Usage

**Simulation:**
```bash
ros2 launch kanga_arm_bringup bringup.launch.py
```

## Configs

#### `operation.yaml`

| Parameter      | Unit | Description                                                                                                            |
| -------------- | ---- | ---------------------------------------------------------------------------------------------------------------------- |
| `time_step_ms` | ms   | Duration of each control loop or simulation step. Lower values increase update frequency. Default is 1.0 ms (1000 Hz). |
