# Bloor Project

STM32 embedded firmware project for the Bloor device/workflow.

## Contents

- `Core/` - application code generated/organized by STM32Cube-style projects.
- `Drivers/` - MCU and board driver files.
- `Middlewares/` - middleware dependencies.
- `EWARM/` - IAR Embedded Workbench project files.
- `bloor_code.ioc` - STM32CubeMX configuration.
- `.mxproject` - STM32Cube project metadata.

## Development

Open `bloor_code.ioc` in STM32CubeMX or use the IAR project under `EWARM/` if that is the active toolchain. Regenerate code carefully so custom code sections are preserved.

## Notes

This repository appears to contain firmware source and generated embedded project files. Hardware pinout, peripheral configuration, and board revision should be documented before using this as a production reference.