# kanga_arm_drive

Velocity-only ODrive mapping for the first four arm DOFs.

## Inputs
- `joint_desired_control` (`sensor_msgs/msg/JointState`)

## Outputs
- `/<odrive namespace>/control_message` (`kanga_interfaces/msg/ControlMessage`)

## Config
- `config/odrive_node_ids_arm.yaml`
  - `namespace`
  - `node_id`
  - `interface`
  - `invert`
  - `reduction` (joint velocity to motor velocity scale)
