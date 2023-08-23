================
32F3348DISCOVERY
================

This README discusses issues unique to NuttX configurations for the
STMicro 32F3348DISCOVERY development board featuring the STM32F334C8
MCU. The STM32F334C8 is a 72MHz Cortex-M4 operation with 64kB Flash
memory and 16KB RAM. The board features:

- On-board ST-LINK/V2 for programming and debugging,
- High brightness LED dimming with buck converter
- One buck/boost converter
- Four user LEDs and two push-buttons,
- Easy access to most MCU pins.

Refer to http://www.st.com/en/evaluation-tools/32f3348discovery.html for
further information about this board.

Configurations
==============

nsh:
----

Configures the NuttShell (nsh) located at apps/examples/nsh.

powerled:
---------

This is a configuration for onboard high brightness LED dimming.

buckboost:
----------

This configuration uses apps/examples/smps and onboard buck/boost converter.

WARNING: current limit is not implemented!
