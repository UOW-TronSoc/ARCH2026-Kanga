# kanga_arm_description

The `kanga_arm_description` package defines the physical configuration, URDF model, and associated resources for the kanga arm robot. It provides mesh files, a robot description, and configurable parameters to be used by simulation or control packages.

## Features

This package contains:
- A URDF model of the robot with articulated joints
- Mesh files for visualization and collision
- Configuration parameters for initial pose and link geometry

## Configs

#### `kanga_arm_config.yaml`   

| Parameter                  | Unit    | Description                                                        |
| -------------------------- | ------- | ------------------------------------------------------------------ |
| `joint_initial_positions`  | radians | Initial joint angles |
| `link_lengths`             | meters  | Length of each link               |
