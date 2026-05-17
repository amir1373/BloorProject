# Bloor Project Firmware

STM32 embedded firmware project for the Bloor device/workflow.

## What This Repository Contains

- `Core/` - application code generated/organized by STM32Cube-style projects.
- `Drivers/` - MCU and board driver files.
- `Middlewares/` - middleware dependencies.
- `EWARM/` - IAR Embedded Workbench project files.
- `bloor_code.ioc` - STM32CubeMX configuration.
- `.mxproject` - STM32Cube project metadata.

## Development

Open `bloor_code.ioc` in STM32CubeMX to inspect pinout, clock, peripheral, and middleware configuration. Use the IAR project under `EWARM/` if that is the active build path.

## Hardware Notes

Document the target board revision, connected sensors/actuators, communication interfaces, and flashing/debugging procedure before using this as a production reference.

## Safety

Embedded firmware should be tested on a bench setup first. Verify power, I/O direction, communication wiring, and recovery/debug access before connecting to a deployed device.