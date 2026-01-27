# kanga_arm_simulation

The `kanga_arm_simulation` package integrates the kanga arm robot with the [RaiSim](https://raisim.com/) physics engine for real-time, high-fidelity simulation. It provides a bridge between ROS 2 and Raisim to simulate robot dynamics, publish joint states, and apply joint-level control inputs.

## Features

This package includes:
- Integration with [RaiSim](https://raisim.com/)
- Full-body or fixed-base simulation modes
- Joint-level effort control with PD + feedforward torque input
- Publishes real-time joint states to ROS 2
- Configurable initial positions and robot parameters
- Includes collision filtering for realistic articulation
- Visual center of mass (CoM) feedback in simulation


## Node: `raisim_bridge`

A ROS 2 node that bridges control messages and physics simulation using the Raisim API.

### Subscribed Topics

| Topic                   | Type                           | Description                                 |
|-------------------------|--------------------------------|---------------------------------------------|
| `/joint_desired_control`| `sensor_msgs/msg/JointState`   | Target joint positions, velocities, efforts |

### Published Topics

| Topic           | Type                           | Description                                  |
|-----------------|--------------------------------|----------------------------------------------|
| `/joint_states` | `sensor_msgs/msg/JointState`   | Current simulated joint state for all 12 DOF |

---

## Launch Files

### `raisim.launch.py`

Entry point for launching the kanga arm robot in a RaiSim physics simulation environment.

#### Usage

**Simulation:**
```bash
ros2 launch kanga_arm_simulation raisim.launch.py
```

## Configs

#### `config/simulation.yaml`

| Parameter                  | Unit    | Description                                                        |
| -------------------------- | ------- | ------------------------------------------------------------------ |
