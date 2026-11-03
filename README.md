# stm32_motorcontrol

Five-motor cascaded position/velocity control firmware for STM32F767ZI (NUCLEO-F767ZI, STM32CubeIDE).

## Overview

This repository holds a single STM32CubeIDE project, `0514all`, generated with STM32CubeMX 6.8.1 (STM32Cube FW_F7 V1.17.0) for the NUCLEO-F767ZI board. The firmware reads five quadrature encoders and drives five motors with a PWM signal plus a direction pin. Each joint uses a cascaded loop: a PD position controller produces a velocity reference for a PI velocity controller (with saturation and anti-windup), whose output sets the PWM duty cycle.

## Features

As implemented in `0514all/Core/Src/main.c`:

- Encoder inputs on TIM1, TIM2, TIM3, TIM4 and TIM8 (encoder mode); joint angles are computed from 1024-line encoders with 4x decoding and per-joint gear ratios of 126, 126, 100, 105 and 132.
- PWM outputs on TIM5 CH4, TIM11 CH1, TIM12 CH2, TIM13 CH1 and TIM14 CH1, each paired with a direction GPIO.
- The control loop runs in the TIM9 update interrupt; position (Kp, Kd) and velocity (Kp, Ki) gains are per-joint arrays in `main.c`.
- The user button (PC13) enables control and adds 90° to every joint's target angle on each press; LED LD1 toggles.

## Repository layout

- `0514all/` – STM32CubeIDE project
  - `0514all.ioc` – STM32CubeMX configuration (pins, timers, clocks)
  - `Core/` – application code (`Src/main.c`), interrupt handlers, HAL MSP setup and startup file
  - `Drivers/` – ST-provided CMSIS and STM32F7xx HAL drivers
  - `STM32F767ZITX_FLASH.ld`, `STM32F767ZITX_RAM.ld` – linker scripts
  - `0514all Debug.launch` – STM32CubeIDE debug configuration (ST-LINK)
  - `Debug/` – STM32CubeIDE build outputs (object files, `.elf`, `.map`, ...); not needed to rebuild the project

## Requirements

- STM32CubeIDE
- NUCLEO-F767ZI board, with motor drivers and encoders wired to the pins defined in `0514all.ioc`

## Usage

Import the `0514all` folder into STM32CubeIDE as an existing project, build it, and flash/debug it with the included `0514all Debug` launch configuration. After reset, press the user button to start position control.

## Status

2023 project (single commit, May 2023), kept as a record and no longer maintained.

## License

No license file is included. The ST-provided files under `0514all/Drivers/` carry their own license files.
