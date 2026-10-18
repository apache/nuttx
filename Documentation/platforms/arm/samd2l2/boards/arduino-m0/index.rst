==========
Arduino M0
==========

.. tags:: arch:arm, chip:samd2l2, vendor:microchip

The Arduino M0 is a development board featuring the Microchip (formerly Atmel)
ATSAMD21G18A ARM Cortex-M0+ microcontroller. It is designed in the classic Arduino
Uno form factor, operating with 3.3V logic.

This documentation applies to the **Arduino M0**, as well as closely related
boards and compatible third-party variants:

- **Arduino M0**: Features the ATSAMD21G18A MCU with native micro-USB connector
  and a 6-pin SWD header (no onboard EDBG debugger chip).
- **Arduino M0 Pro / Arduino Zero**: Includes an onboard Atmel Embedded Debugger
  (EDBG) chip providing a hardware programming/debugging port and virtual COM port,
  alongside the native USB port.
- **Wemos SAMD21 M0 / RobotDyn SAMD21 M0**: Cost-effective compatible boards
  utilizing the ATSAMD21G18A in the Arduino form factor.

.. warning::
   All I/O pins on the Arduino M0 operate at **3.3V**. Unlike 8-bit AVR Arduino
   boards (such as Uno or Mega) which run at 5V, applying more than 3.3V to any
   I/O pin will damage the microcontroller.

Features
========

- **Microcontroller**: Microchip ATSAMD21G18A
  - 32-bit ARM Cortex-M0+ running at up to 48 MHz
  - 256 KB In-System Self-Programmable Flash memory
  - 32 KB SRAM
  - Hardware single-cycle (32x32) multiplier
- **Clock Architecture**:
  - Internal 8 MHz RC oscillator (OSC8M) driving GCLK1
  - Digital Frequency Locked Loop (DFLL48M) generating the 48 MHz main system clock (GCLK0)
  - 32.768 kHz RTC oscillator footprint
- **Operating Voltage**: 3.3V
- **Digital I/O Pins**: 14 (with 12 PWM channels and UART)
- **Analog Input Pins**: 6 (12-bit ADC channels)
- **Analog Output**: 1 (10-bit DAC)
- **USB**: Full-Speed (12 Mbps) USB 2.0 Device/Host port (Native USB)
- **Interfaces**:
  - Up to 6 SERCOM modules configurable as UART/USART, SPI, or I2C
  - Real-Time Counter (RTC) and calendar

LEDs
====

The Arduino M0 provides one user-controllable yellow LED (labeled ``L`` or ``STATUS``)
connected to port pin **PA17**. Driving PA17 high illuminates the LED.

When ``CONFIG_ARCH_LEDS`` is enabled in NuttX, the system controls
the LED to indicate operational states:

================== ======================= ======
Symbol             Meaning                 LED
================== ======================= ======
LED_STARTED        NuttX has been started  OFF
LED_HEAPALLOCATE   Heap has been allocated OFF
LED_IRQSENABLED    Interrupts enabled      OFF
LED_STACKCREATED   Idle stack created      ON
LED_INIRQ          In an interrupt         N/C
LED_SIGNAL         In a signal handler     N/C
LED_ASSERTION      An assertion failed     N/C
LED_PANIC          The system has crashed  FLASH
================== ======================= ======

When NuttX boots successfully, the LED stays statically **ON** (indicating normal
system execution). If the LED blinks rapidly at approximately 2 Hz, a fatal error or
panic has occurred.

If ``CONFIG_ARCH_LEDS`` is disabled, the LED is accessible to user
applications via the standard user LED driver (``/dev/userleds``).

Serial Consoles
===============

SERCOM5 UART
------------

By default, the ``nsh`` configuration routes the serial console to **SERCOM5**
at 115200 baud, 8 data bits, no parity, and 1 stop bit (8N1).

SERCOM5 USART signals are exposed on two solder pads labeled **RXD** and **TXD**
near the ICSP pin header:

==== ==== ==================
Pin  GPIO Function
==== ==== ==================
37   PB22 SERCOM5 / USART TX
38   PB23 SERCOM5 / USART RX
==== ==== ==================

To use this console:

1. Solder a 2-pin male header to the RXD and TXD pads.
2. Connect a 3.3V USB-to-UART TTL adapter:
   - Adapter **TX** -> Board **RXD** (PB23)
   - Adapter **RX** -> Board **TXD** (PB22)
   - Adapter **GND** -> Board **GND**

.. note::
   Using an external USB-to-UART adapter on SERCOM5 allows monitoring the system
   console without resetting the terminal session when the board or native USB
   port resets.

Native USB Port
---------------

The ATSAMD21 features an integrated Full-Speed USB controller. The ``usbnsh``
configuration enables the USB CDC/ACM serial device class, mapping the NuttShell
(NSH) console directly to the native micro-USB connector.

Using the ``usbnsh`` profile eliminates the need to solder external wires or use a
USB-to-serial converter. When connected to a host computer, the board appears as:

- **Linux**: ``/dev/ttyACM0``
- **macOS**: ``/dev/cu.usbmodem*``
- **Windows**: Virtual COM port (e.g., ``COM3``)

Configurations
==============

The following board configurations are supported:

nsh
---

Basic NuttShell (NSH) interactive shell over the SERCOM5 UART serial console
(115200 8N1).

usbnsh
------

NuttShell (NSH) console accessible directly over the native micro-USB port
using USB CDC/ACM serial class emulation.

Building NuttX
==============

To configure and build NuttX for the Arduino M0:

1. Configure the build for your selected profile:

   .. code-block:: console

      $ ./tools/configure.sh arduino-m0:nsh

   Or for the USB NSH console:

   .. code-block:: console

      $ ./tools/configure.sh arduino-m0:usbnsh

2. (Optional) Customize settings using menuconfig:

   .. code-block:: console

      $ make menuconfig

3. Build the NuttX binary:

   .. code-block:: console

      $ make

Upon completion, the build produces:

- ``nuttx``: ELF binary with debug symbols.
- ``nuttx.bin``: Raw binary image for flashing.

Flashing and Debugging
======================

SWD Programming with OpenOCD
----------------------------

Currently, only **SWD programming** is supported because the default board
linker script (``scripts/flash.ld``) links NuttX starting at flash origin
``0x00000000``. Flashing via SWD overwrites the flash starting from address
zero, which replaces any pre-installed Arduino SAM-BA bootloader.

.. note::
   Flashing via USB bootloader utilities (such as BOSSA at offset ``0x2000``)
   is not currently supported because the default binary is linked for origin
   ``0x00000000``. Booting from an offset bootloader requires adjusting the flash
   origin and vector table offset in a custom linker script.

The Arduino M0 exposes SWD pins on the 6-pin SWD connector or solder pads.
Connect your SWD programmer (**SWCLK**, **SWDIO**, **RESET**, **GND**, **3.3V**)
and flash the ``nuttx.bin`` binary using OpenOCD:

CMSIS-DAP / Atmel-ICE
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: console

   $ openocd -f interface/cmsis-dap.cfg -f target/at91samdXX.cfg \
       -c "program nuttx.bin 0x00000000 verify reset exit"

ST-Link (ST-LINK/V2)
~~~~~~~~~~~~~~~~~~~~

.. code-block:: console

   $ openocd -f interface/stlink.cfg -f target/at91samdXX.cfg \
       -c "program nuttx.bin 0x00000000 verify reset exit"

Segger J-Link
~~~~~~~~~~~~~

.. code-block:: console

   $ openocd -f interface/jlink.cfg -c "transport select swd" \
       -f target/at91samdXX.cfg \
       -c "program nuttx.bin 0x00000000 verify reset exit"

Debugging with GDB
------------------

To debug NuttX on target hardware:

1. Start OpenOCD as a GDB server:

   .. code-block:: console

      $ openocd -f interface/cmsis-dap.cfg -f target/at91samdXX.cfg

2. Connect GDB to the target:

   .. code-block:: console

      $ arm-none-eabi-gdb nuttx
      (gdb) target extended-remote :3333
      (gdb) monitor reset halt
      (gdb) load
      (gdb) continue
