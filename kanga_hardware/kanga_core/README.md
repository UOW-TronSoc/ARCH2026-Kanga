# kanga_core

Core rover package providing suspension joint state publishing and (optionally) a CAN bridge for the core ESP32 microcontroller.

## Nodes

### suspension_node

Publishes dummy joint states for the rover suspension joints to `/joint_states` once on startup:

- `left_suspension_joint` = 0.0
- `right_suspension_joint` = 0.0

Use when `robot_state_publisher` needs suspension joint values but no hardware driver provides them (e.g. visualization with `use_joint_state_publisher:=false`).

```bash
ros2 run kanga_core suspension_node
```

### core_can_node (optional)

Fixed-configuration bridge for the core ESP32 microcontroller riding on the shared CAN bus. Reuses the generic `kanga_microcontroller` implementation but forces the node to monitor node id `0x15`.

- publishes `microcontroller_rx` frames from node id `0x15`
- consumes outbound frames from `microcontroller_tx` and pushes them back onto the CAN bus

`launch/core.launch.yaml` starts the bridge in the `core` namespace with the correct node id and `can1` interface.
