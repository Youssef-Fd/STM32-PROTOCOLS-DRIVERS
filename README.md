# STM32 Protocol Drivers (Bare-Metal)

A bare-metal driver library for STM32 microcontrollers implementing core communication protocols (UART, SPI, I2C) without relying on HAL or external libraries.

This project is designed to provide a deep understanding of low-level embedded systems programming by directly interacting with hardware registers and building reusable driver APIs.

## Overview

Embedded systems development often relies on abstraction layers such as STM32 HAL. While these libraries simplify development, they hide many low-level details of the hardware.

This project focuses on:

- Direct register-level programming
- Understanding peripheral behavior
- Building reusable and modular driver architectures
- Implementing interrupt-driven communication

The goal is to bridge the gap between theoretical knowledge and real embedded systems design.

## Objectives

- Develop communication protocol drivers from scratch
- Avoid HAL and external abstraction layers
- Implement interrupt-based communication
- Design reusable APIs similar to HAL structure
- Ensure efficient and non-blocking data transfer
- Follow embedded software best practices (MISRA-oriented)

## Features

- **Bare-metal implementation**
  - No HAL or standard peripheral libraries
  - Full control over hardware registers

- **Supported Protocols**
  - UART (USART)
  - SPI
  - I2C

These protocols are fundamental in embedded systems and widely used for communication between microcontrollers and peripherals. :contentReference[oaicite:0]{index=0}

- **Interrupt-driven communication**
  - Non-blocking TX/RX
  - NVIC configuration
  - Event-based handling

- **Reusable driver architecture**
  - Clean API design inspired by HAL
  - Modular and scalable code structure

- **Static analysis & safety**
  - MISRA C guidelines
  - Code analysis using Clang Static Analyzer

## System Architecture

The driver architecture is structured into multiple layers:

- **Application Layer**
  - Uses driver APIs for communication

- **Driver Layer**
  - Implements protocol logic (UART, SPI, I2C)

- **Hardware Abstraction Layer (custom)**
  - Register definitions
  - Peripheral configuration

- **MCU Hardware**
  - STM32 microcontroller (ARM Cortex-M)

STM32 microcontrollers support multiple communication interfaces and low-level drivers, making them ideal for such implementations. :contentReference[oaicite:1]{index=1}

## Driver Design

Each driver follows a structured approach:

- Peripheral initialization (clock, GPIO, configuration)
- Data transmission and reception functions
- Interrupt handling routines
- State management for communication

Example:

- UART driver supports:
  - TX/RX in polling and interrupt modes
  - Configuration of baud rate, parity, stop bits

- SPI driver supports:
  - Full-duplex communication
  - Master mode configuration

- I2C driver supports:
  - Addressing and data transfer
  - Event-based communication

## Project Structure

```text
STM32-PROTOCOLS-DRIVERS/
├── drivers/
│   ├── uart/
│   ├── spi/
│   └── i2c/
├── inc/
├── src/
├── examples/
└── README.md
