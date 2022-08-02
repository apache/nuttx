===============
NXP MR-CANHUBK3
===============

s a CAN Node reference design and development platform for mobile robotics applications, such as drones and rovers

NXP MR-CANHUBK3 is a reference design and development platform for redundant networking and functional safety applications in Mobile Robotics, such as drones and rovers. It features the `NXP S32K344 MCU <https://www.nxp.com/products/processors-and-microcontrollers/s32-automotive-platform/s32k-general-purpose-mcus/s32k3-microcontrollers-for-general-purpose:S32K3>`_ based on a Arm Cortex-M7 core (Lock-Step).

Features
========

- NXP FS32K344
    - Lock-Step Configuration
    - 160 MHz Clock (Max.)
    - 4000 Kb Flash
    -  512 Kb RAM

- NXP FS26 Safety System Basis Chip

- Connectivity:
    - Console UART (Console @ 115,200 baud)
    - 6x CAN FD
    - 100Base-T1 Ethernet
    - Connectors and I/O headers with GPIO, I2C, SPI, etc.

Serial Console
==============

By default, the serial console will be provided on the DCD-LZ UART (available on the 7-pin DCD-LZ debug connector P6):

==============  =====  ==========
DCD-LZ UART RX  PTA8   LPUART2_RX
DCD-LZ UART TX  PTA9   LPUART2_TX
==============  =====  ==========

LEDs and Buttons
================

Leds
----

The MR-CANHUBK3 has one RGB LED:

========  =====  ===================================
RedLED    PTE14  FXIO D7 / EMIOS0 CH19
GreenLED  PTA27  FXIO D5 / EMIOS1 CH10 / EMIOS2 CH10
BlueLED   PTE12  FXIO D8 / EMIOS1 CH5
========  =====  ===================================

An output of '0' illuminates the LED.

If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any way. The following definitions are used to access individual RGB components (see mr-canhubk3.h):

- GPIO_LED_R
- GPIO_LED_G
- GPIO_LED_B

The RGB components could, alternatively, be controlled through PWM using the common RGB LED driver.

If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board the MR-CANHUBK3. The following definitions describe how NuttX controls the LEDs:

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

The MR-CANHUBK3 supports two buttons:

===  =====  ==============
SW1  PTD15  EIRQ31
SW2  PTA25  EIRQ5 / WKPU34
===  =====  ==============

Configurations
==============

Each MR-CANHUBK3 configuration is maintained in a sub-directory and can be selected as follows::

    tools/configure.sh mr-canhubk3:<subdir>

Where <subdir> is one of the sub-directories listed in the next paragraph.

NOTES (common for all configurations):

1. This configuration uses the mconf-based configuration tool. To change this configuration using that tool, you should:
    a. Build and install the kconfig-mconf tool. See nuttx/README.txt. Also see additional README.txt files in the NuttX tools repository.
    b. Execute 'make menuconfig' in nuttx/ in order to start the reconfiguration process.

2. Unless otherwise stated, the serial console used is LPUART2 at 115,200 8N1.

nsh
---
Configures the NuttShell (nsh) located at apps/examples/nsh. Support for builtin applications is enabled, but in the base configuration the only application selected is the "Hello, World!" example.

net
---
Besides the NuttShell this configuration also enables networking and (Socket)CAN support, as well as I2C and SPI support. It includes the SLCAN and can-utils applications for monitoring and debugging CAN applications.
