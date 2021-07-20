==========
Teensy-4.x
==========

The `Teensy 4.0 <https://www.pjrc.com/store/teensy40.html>`_ and `Teensy 4.1 <https://www.pjrc.com/store/teensy41.html>`_
are development boards from PJRC. Both boards use i.MX RT1060 MCU, feature several I/Os pins and on-board LED.

Features
========

- Processor
    - MIMXRT1062DVL6A processor
- Memory
    - 1 MB RAM memory
    - 2 MB Flash (Teensy 4.0)
    - 8 MB Flash (Teensy 4.1)
    - 1 SDIO (4 bit) native SD
- Connectivity
    - Micro USB host
    - CAN transceivers
    - UART, SPI, I2C
    - PWM output pins
    - 10/100 Mb Ethernet (Teensy 4.1 only)
    - Digital audio in/out

LEDs
====

There are two LED status indicators located on the Teensy-4.x board.
The functions of these LEDs include:

- RED LED (loading status)
    - dim:    ready
    - bright: writing
    - blink:  no USB
- USER LED (D3)

Only a single LED, D3, is under software control.

This LED is not used by the board port unless CONFIG_ARCH_LEDS is
defined. In that case, the usage by the board port is defined in
include/board.h and src/imxrt_autoleds.c. The LED is used to encode
OS-related events as follows:

================ ======================= =====
SYMBOL           Meaning                 LED
================ ======================= =====
LED_STARTED      NuttX has been started  OFF
LED_HEAPALLOCATE Heap has been allocated OFF
LED_IRQSENABLED  Interrupts enabled      OFF
LED_STACKCREATED Idle stack created      ON
LED_INIRQ        In an interrupt         N/C
LED_SIGNAL       In a signal handler     N/C
LED_ASSERTION    An assertion failed     N/C
LED_PANIC        The system has crashed  FLASH
================ ======================= =====

Thus if the LED is statically on, NuttX has successfully booted and is,
apparently, running normally. If the LED is flashing at approximately
2Hz, then a fatal error has been detected and the system has halted.

Configurations
==============

nsh-4.0
-------

Configures the NuttShell (nsh) located at examples/nsh for Teensy 4.0.
This NSH configuration is focused on low level, command-line driver testing.
Built-in applications are supported, but none are enabled. NutShells then
runs on USB console.

nsh-4.1
-------

Configures the NuttShell (nsh) located at examples/nsh for Teensy 4.1.
This NSH configuration is focused on low level, command-line driver testing.
Built-in applications are supported, but none are enabled. NutShells then
runs USB console.

can-4.1
-------

This is an nsh configuration (see above) for Teensy-4.x with added support of
CAN driver. All FlexCANs (CAN1, CAN2, CAN3) are chosen as default. FlexCAN3
is FD capable. Please note that device driver name if counted from zero.
That means for CAN1 -> can0, CAN2 -> can1 and CAN3 -> can2

Bitrate and sample point can be also changed at System type peripheral selection,
basic values are 1 MHz for bitrate and 0.80 for sample point. The FlexCAN driver
for imxrt runs at 80 MHz clock frequency.

The configuration also includes CAN utilities as candump and cansend.

This configuration can be easily changed to work with Teensy 4.0 by
selecting ``CONFIG_TEENSY_40=y``.

This configuration runs over LPUART1 (pins 24 and 25 on Teensy). Communication
over USB console can be turn on, but it causes problems with FlexCAN.

enc-4.1
-------

This is an nsh configuration (see above) with added support of incremental
encoder. Phase A is connected to GPIO_EMC_07 (pin 33), phase B to GPIO_EMC_06
(pin 4) and INDEX to GPIO_B0_12 (pin 32). Only encoder 1 is connected to those
pins.

Function of the encoder can be tested by application "qe".

netnsh-4.1
----------

This configuration is similar to the nsh configuration except that is
has networking enabled, both IPv4 and IPv6. This NSH configuration is
focused on network-related testing.

This configuration cannot be changed to Teensy 4.0 as this board does
not have Ethernet capability.

pikron-bb
---------

This is a configuration that compiles the NuttX for use with
open source/hardware `Base Board for Teensy 4.1
<https://gitlab.com/pikron/projects/imxrt-devel/-/wikis/teensy_bb>`_.
It includes CAN drivers, communication over serial port, Ethernet
support, support for 240 x 320 pixels LCD display and configuration
options for using NuttX with pysimCoder. NuttX also runs in
tickless mode with the resolution 10 usec.

This configuration cannot be changed to Teensy 4.0 as base board
is not designed for that.

pwm-4.1
-------

This configuration is similar to the nsh configuration with enabled
FlexPWM driver. Submodules 1 (pin 4) and 2 (pin 5) of FlexPWM2 are turn
on aswell as ultiple channel PWM output. Functionality can be tested
with example application "pwm". Each channel runs different duty cycle.

This configuration can be easily changed to work with Teensy 4.0 by
selecting ``CONFIG_TEENSY_40=y``.

sd-4.1
------

This is an nsh configuration (see above) for Teensy-4.x with added support of
connecting micro SD card.

You can mount micro SD card by:

.. code-block:: console

    $ mount -t vfat /dev/mmcsd0 /mnt

This configuration cannot be changed to Teensy 4.0 as this board does
not have micro SD card slot.

lcd-4.1
-------

This is an nsh configuration (see above) for Teensy-4.x with added support of
connecting LCD TFT display with ST7789 controller. You can run framebuffer demo
by starting "fb" in console. The LCD display is connected via SPI4.

This configuration can be easily changed to work with Teensy 4.0 by
selecting ``CONFIG_TEENSY_40=y``.


Flash
=====

Teensy 4.x boards does not have debugger therefore external firmware has to be used to load NuttX.
`Teensy Loader <https://www.pjrc.com/teensy/loader_cli.html>`_ can be installed and then NuttX can be loaded by:

.. code-block:: console

    $ teensy_loader_cli --mcu=TEENSY41 -v -w nuttx.hex

For Teensy 4.0 board, switch ``--mcu=TEENSY41`` to ``--mcu=TEENSY40``.
