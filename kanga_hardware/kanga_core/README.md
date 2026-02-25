# kanga_core

Fixed-configuration bridge for the core ESP32 microcontroller riding on the shared CAN bus. This package reuses the generic `kanga_microcontroller` implementation but forces the node to monitor node id `0x15` so the higher-level core stack only sees traffic meant for that device.

## Node

Executable: `core_can_node`

- wraps `MicrocontrollerCanNode` from `kanga_microcontroller`
- publishes `microcontroller_rx` frames from node id `0x15`
- consumes outbound frames from `microcontroller_tx` and pushes them back onto the CAN bus

## Launch

`launch/core.launch.yaml` starts the bridge in the `core` namespace with the correct node id and `can1` interface. Duplicate the block or override parameters as needed if the interface name or addressing ever change.
