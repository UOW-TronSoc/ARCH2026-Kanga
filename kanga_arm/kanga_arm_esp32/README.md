# kanga_arm_esp32

The .ino file to upload to the ARM ESP32. It is made using the FreeRTOS task based structure.

Cores are separated into CAN and non-CAN so that CAN will always be operating. Inside codebase tasks are arranged by Core and Priority

## Warnings
Limit Switch and ENcoder Code is somewhat structures out but NOT TESTED!!!, leave commented out for now.

## Features

### Current Features
- CAN Read Task
- CAN Write Task
- Wrist Motor Control Task
- End Effector Control Task

### Next/Planned Features
- Limit Switch Detection + Sending to Orin
- Differential Bar Encoder Reading + Sending to Orin