# kanga_microcontroller

Generic SocketCAN bridge for ESP32-based microcontrollers using the shared `kanga_canbus` module. The node mirrors the behaviour of the battery and ODrive drivers while remaining node-ID agnostic so that any number of microcontrollers can coexist on the same bus as the BMS.

## Node

Executable: `microcontroller_can_node`

Publishes `microcontroller_rx` (`kanga_microcontroller/msg/MicrocontrollerFrame`) for every frame that matches the configured CAN node id. Subscribes to `microcontroller_tx` with the same message type and forwards the payload onto the bus.

## Parameters

- `interface` (`string`, default `can1`): SocketCAN interface to bind to. Matches the BMS configuration.
- `node_id` (`int`, default `0`): Logical node identifier to filter incoming frames and to populate outgoing frames when no explicit CAN id is supplied.
- `node_id_mask` (`int`, default `65535`): Bitmask applied after removing CAN flags to extract the node id field.
- `node_id_shift` (`int`, default `0`): Bit shift applied after masking when extracting the node id field.
- `filter_by_node` (`bool`, default `true`): If true only frames matching `node_id` are published on `microcontroller_rx`.
- `expect_extended_id` (`bool`, default `true`): Reject standard frames when the bus is expected to use 29-bit identifiers like the BMS line.
- `force_extended_tx` (`bool`, default `true`): Force outgoing frames to use extended identifiers unless the bus is configured otherwise.
- `allow_tx_id_override` (`bool`, default `true`): Permit messages on `microcontroller_tx` to provide a raw `can_id`. Disable to always build the id from `tx_base_id` and `node_id`.
- `tx_base_id` (`int`, default `0`): Base identifier used when synthesising CAN ids for transmissions. Combine with the mask/shift fields for multi-node deployments.
- `auto_header_stamp` (`bool`, default `true`): Stamp published messages with the current ROS time.
- `drop_remote_frames` (`bool`, default `true`): Ignore CAN remote frames on reception.

## Launch

A sample launch file `launch/microcontroller.launch.yaml` shows the typical configuration. Clone the block per microcontroller and set a unique namespace and node id. All bridges can share the same `can1` interface alongside the `kanga_battery` node.

## Message

`MicrocontrollerFrame.msg` wraps a raw CAN payload, including the node id, data length (`dlc`), flags (extended/remote), and the unflagged CAN identifier. This mirrors the existing hardware driver pattern so higher-level packages can consume microcontroller data without dealing with raw socket structures.

## Usage Notes

- Keep the CAN timing identical to the BMS line when wiring ESP32 devices through a TJA1051T/3 transceiver.
- For simple deployments, leave `allow_tx_id_override` enabled and publish to `microcontroller_tx` with `can_id` set to `0`. The node will build the identifier from `tx_base_id` and `node_id` automatically.
- To run multiple bridges, launch the executable multiple times (or duplicate the YAML entry) with distinct namespaces and `node_id` values while pointing to the same `can1` interface.
