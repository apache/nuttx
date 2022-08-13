==============
NXP S32K144EVB
==============

`NXP S32K144EVB-Q100 <https://www.nxp.com/design/development-boards/automotive-development-platforms/s32k-mcu-platforms/s32k144-q100-general-purpose-evaluation-board:S32K144EVB>`_ is an evaluation board for the `NXP S32K144 MCU <https://www.nxp.com/products/processors-and-microcontrollers/s32-automotive-platform/s32k-general-purpose-mcus/s32k1-microcontrollers-for-general-purpose:S32K1>`_ based on an Arm Cortex-M4F core.

Features
========

- NXP FS32K144HFT0VLLT MCU
    - 80 MHz Clock (Max.)
    - 512 Kb Flash
    -  64 Kb SRAM
    -   4 Kb EEPROM

- NXP UJA1169TK/F System Basis Chip

- Connectivity:
    - OpenSDA UART (Console @ 115,200 baud)
    - CAN 2.0 (Transceiver part of UJA1169 SBC)
    - I/O headers with GPIO, I2C, SPI, etc.

Serial Console
==============

By default, the serial console will be provided on the OpenSDA VCOM port:

===============  ====  ==========
OpenSDA UART RX  PTC6  LPUART1_RX
OpenSDA UART TX  PTC7  LPUART1_TX
===============  ====  ==========

USB drivers for the PEmicro CDC Serial Port are available here: http://www.pemicro.com/opensda/

LEDs and Buttons
================

Leds
----

The S32K144EVB has one RGB LED:

========  =====  ========
RedLED    PTD15  FTM0 CH0
GreenLED  PTD16  FTM0 CH1
BlueLED   PTD0   FTM0 CH2
========  =====  ========

An output of '0' illuminates the LED.

If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any way. The following definitions are used to access individual RGB components (see s32k144evb.h):

- GPIO_LED_R
- GPIO_LED_G
- GPIO_LED_B

The RGB components could, alternatively, be controlled through PWM using the common RGB LED driver.

If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board the S32K144EVB. The following definitions describe how NuttX controls the LEDs:

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
LED_IDLE          S32K144 in sleep mode
================  =======================  =====  =====  ====

Buttons
-------

The S32K144EVB supports two buttons:

===  =====
SW2  PTC12
SW3  PTC13
===  =====

OpenSDA Notes
=============

- USB drivers for the PEmicro CDC Serial Port are available here: http://www.pemicro.com/opensda/
- The drag'n'drog interface expects files in .srec format.
- Using Segger J-Link:  Easy... but remember to use the SWD connector J14 near the touch electrodes and not the OpenSDA connector near the OpenSDA USB connector J7.

Configurations
==============

Each S32K144EVB configuration is maintained in a sub-directory and can be selected as follows::

    tools/configure.sh s32k144evb:<subdir>

Where <subdir> is one of the sub-directories listed in the next paragraph.

NOTES (common for all configurations):

1. This configuration uses the mconf-based configuration tool. To change this configuration using that tool, you should:
    a. Build and install the kconfig-mconf tool. See nuttx/README.txt. Also see additional README.txt files in the NuttX tools repository.
    b. Execute 'make menuconfig' in nuttx/ in order to start the reconfiguration process.

2. Unless otherwise stated, the serial console used is LPUART1 at 115,200 8N1.

nsh
---
Configures the NuttShell (nsh) located at apps/examples/nsh. Support for builtin applications is enabled, but in the base configuration the only application selected is the "Hello, World!" example.
