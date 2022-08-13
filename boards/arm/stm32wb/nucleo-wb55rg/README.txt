NUCLEO-WB55RG README
======================

  This README file discusses the port of NuttX to the STMicroelectronics
  NUCLEO-WB55RG board.  That board features the multi-protocol wireless and
  ultra-low-power STM32WB55RGV6 MCU with 1MiB of Flash and 256KiB of SRAM.
  A dedicated M0+ coprocessor is responsible for performing the real-time
  low layer operations via one of the available wireless stacks distributed
  as binaries in STM32CubeWB package.

Contents
========

  - Status
  - LEDs
  - Buttons
  - Serial Console
  - Configurations

Status
======

  April 2022: The nucleo-wb55rg board minimal setup compiles successfully.

  June 2022: All STM32WB chip family is defined, many of peripherals are
    supported - GPIO, EXTI, DMA, timers, flash, PWR, RTC, USART/LPUART, SPI,
    IPCC. SRAM2 heap allocation works. Builtin apps work and ostest passed.

  July 2022: Added BLE support with mailbox IPC driver.

LEDs
====

  The board has 3 user leds:
    LED1 (Blue)      PB5
    LED2 (Green)     PB0
    LED3 (Red)       PB1

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/stm32_autoleds.c.

Buttons
=======

  The board has 3 user buttons:
    SW1     PC4 (needs SB47 close)
    SW2     PD0
    SW3     PD1

Serial Consoles
===============

  The MCU's USART1 is connected to the on-board ST-LINK/V2-1 and exposed as
  a Virtual COM Port over the same Micro-USB Type B connection used for
  programming/debugging.

Configurations
==============

  nsh:

    Configures the NuttShell (nsh) located at examples/nsh.  This
    configuration is focused on low level, command-line driver testing.

  ble:

    Besides the NuttShell this configuration also enables BLE support.
    It includes btsak application for testing BLE applications.

  nimble:

    Besides the NuttShell it includes nimble example application which
    uses NimBLE host stack.

      nsh> ifup bnep0
      nsh> nimble
