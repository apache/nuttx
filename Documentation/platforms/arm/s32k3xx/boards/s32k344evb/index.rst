==============
NXP S32K344EVB
==============

`NXP S32K3X4EVB-Q257 <https://www.nxp.com/design/development-boards/automotive-development-platforms/s32k-mcu-platforms/s32k3x4-q257-full-featured-general-purpose-development-board:S32K3X4EVB-Q257>`_ is an evaluation board for the `NXP S32K344 MCU <https://www.nxp.com/products/processors-and-microcontrollers/s32-automotive-platform/s32k-general-purpose-mcus/s32k3-microcontrollers-for-general-purpose:S32K3>`_ based on a Arm Cortex-M7 core (Lock-Step).

Features
========

- NXP FS32K344
    - Lock-Step Configuration
    - 160 MHz Clock (Max.)
    - 4000 Kb Flash
    -  512 Kb RAM

- NXP FS26 Safety System Basis Chip

- Connectivity:
    - OpenSDA UART (Console @ 115,200 baud)
    - 2x Secure CAN transceivers
    - I/O headers with GPIO, I2C, SPI, etc.

Serial Console
==============

By default, the serial console will be provided on the OpenSDA VCOM port:

===============  =====  ==========
OpenSDA UART RX  PTA15  LPUART6_RX
OpenSDA UART TX  PTA16  LPUART6_TX
===============  =====  ==========

USB drivers for the PEmicro CDC Serial Port are available here: http://www.pemicro.com/opensda/

LEDs and Buttons
================

Leds
----

The S32K344EVB has two RGB LEDs:

=========  =====  =========================
RedLED0    PTA29  EMIOS1 CH12 / EMIOS2 CH12
GreenLED0  PTA30  EMIOS1 CH13 / EMIOS2 CH13
BlueLED0   PTA31  EMIOS1 CH14 / FXIO D0
=========  =====  =========================

=========  =====  ===================================
RedLED1    PTB18  EMIOS1 CH15 / EMIOS2 CH14 / FXIO D1
GreenLED1  PTB25  EMIOS1 CH21 / EMIOS2 CH21 / FXIO D6
BlueLED1   PTE12  EMIOS1 CH5  / FXIO D8
=========  =====  ===================================

An output of '1' illuminates the LED.

If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any way. The following definitions are used to access individual RGB components (see s32k344evb.h):

- GPIO_LED0_R
- GPIO_LED0_G
- GPIO_LED0_B

- GPIO_LED1_R
- GPIO_LED1_G
- GPIO_LED1_B

The RGB components could, alternatively, be controlled through PWM using the common RGB LED driver.

If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board the S32K344EVB. The following definitions describe how NuttX controls the LEDs:

================  =======================  =====  =====  ====
State             Description              RED    GREEN  BLUE
================  =======================  =====  =====  ====
LED_STARTED       NuttX has been started   OFF    OFF    OFF
LED_HEAPALLOCATE  Heap has been allocated  OFF    OFF    ON
LED_IRQSENABLED   Interrupts enabled       OFF    OFF    ON
LED_STACKCREATED  Idle stack created       OFF    ON     OFF
LED_INIRQ         In an interrupt
LED_SIGNAL        In a signal handler
LED_ASSERTION     An assertion failed
LED_PANIC         The system has crashed   FLASH  OFF    OFF
LED_IDLE          S32K344 in sleep mode
================  =======================  =====  =====  ====

Buttons
-------

The S32K344EVB supports two buttons:

===  =====  ===============
SW0  PTB26  EIRQ13 / WKPU41
SW1  PTB19  WKPU38
===  =====  ===============

OpenSDA Notes
=============

- USB drivers for the PEmicro CDC Serial Port are available here: http://www.pemicro.com/opensda/
- The drag'n'drog interface expects files in .srec format.

Configurations
==============

Each S32K344EVB configuration is maintained in a sub-directory and can be selected as follows::

    tools/configure.sh s32k344evb:<subdir>

Where <subdir> is one of the sub-directories listed in the next paragraph.

NOTES (common for all configurations):

1. This configuration uses the mconf-based configuration tool. To change this configuration using that tool, you should:
    a. Build and install the kconfig-mconf tool. See nuttx/README.txt. Also see additional README.txt files in the NuttX tools repository.
    b. Execute 'make menuconfig' in nuttx/ in order to start the reconfiguration process.

2. Unless otherwise stated, the serial console used is LPUART6 at 115,200 8N1.

nsh
---
Configures the NuttShell (nsh) located at apps/examples/nsh. Support for builtin applications is enabled, but in the base configuration the only application selected is the "Hello, World!" example.
