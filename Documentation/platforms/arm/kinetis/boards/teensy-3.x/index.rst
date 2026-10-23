===============
PJRC Teensy 3.x
===============

.. tags:: arch:arm, chip:kinetis, vendor:nxp

The Teensy 3.1 and 3.2 are compact, breadboard-friendly development boards
designed by PJRC (Paul Stoffregen), powered by the NXP (formerly Freescale)
Kinetis MK20DX256VLH7 ARM Cortex-M4 microcontroller.

This board configuration also supports the earlier Teensy 3.0 board (which
features the Kinetis MK20DX128VLH5 microcontroller).

.. note::
   The initial port of NuttX to the Teensy 3.1 was contributed by Jakob
   Odersky (`jodersky <https://github.com/jodersky>`_).

Hardware Overview
=================

Comparison: Teensy 3.0 vs. Teensy 3.1 / 3.2
-------------------------------------------

The Teensy 3.1 and 3.2 share identical pinouts and schematics with the Teensy
3.0, but feature a significantly enhanced MCU:

============= ===================== ===================== =====================================
Feature       Teensy 3.0            Teensy 3.1 / 3.2      Teensy 3.0 Configuration
============= ===================== ===================== =====================================
Processor     NXP MK20DX128VLH5     NXP MK20DX256VLH7     ``CONFIG_ARCH_CHIP_MK20DX128VLH5=y``
Core          ARM Cortex-M4         ARM Cortex-M4         ARM Cortex-M4
Rated Clock   48 MHz                72 MHz                Selected via board clock header
Overclockable Up to 96 MHz          Up to 96 MHz          ``CONFIG_TEENSY_3X_OVERCLOCK=y``
Flash Memory  128 KB                256 KB                Selected via linker script
SRAM          16 KB                 64 KB                 ``CONFIG_RAM_SIZE=16384``
5V Tolerance  No (3.3V only)        Yes (Digital pins)    No (3.3V only)
Linker Script mk20dx128vlh5.ld      mk20dx256vlh7.ld      Automatic based on chip selection
============= ===================== ===================== =====================================

.. warning::
   While digital I/O pins on the Teensy 3.1/3.2 are 5V tolerant, the analog
   inputs (A10, A11, AREF, and the internal ADC reference) are strictly **3.3V
   maximum**. On the Teensy 3.0, **all** pins are 3.3V only. Exceeding 3.3V on
   non-tolerant pins will permanently damage the microcontroller.

Key Features (Teensy 3.1 / 3.2)
-------------------------------

- **Microcontroller**: NXP Kinetis MK20DX256VLH7 (64-pin LQFP)
  - 32-bit ARM Cortex-M4 running at 72 MHz (overclockable up to 96 MHz)
  - 256 KB Flash memory, 64 KB SRAM
  - Hardware CRC and Random Number Generator
- **Clocking**:
  - 16 MHz external crystal (XTAL on PTA18/PTA19)
  - On-chip PLL generating 72 MHz system clock (or 96 MHz when overclocked)
- **USB Interface**:
  - Full-Speed (12 Mbps) USB 2.0 Device/Host/OTG port with Micro-B connector
  - Dedicated USB regulator providing 3.3V (up to 100 mA) to external circuits
- **Serial Communication**:
  - 3 UARTs with FIFO and hardware CTS/RTS support
  - 2 SPI modules with FIFO
  - 2 I2C modules with SMBus support
- **Analog & Timers**:
  - 2 16-bit ADCs with differential mode support
  - 1 12-bit DAC
  - 3 FlexTimer modules (FTM) supporting up to 12 PWM channels
  - Carrier Modulator Transmitter (CMT)
  - Real-Time Clock (RTC) with external 32.768 kHz crystal footprint
- **Form Factor & Bootloader**:
  - 24-pin DIP footprint (0.1" pitch, breadboard friendly)
  - Onboard Nuvoton Mini54TAN processor running PJRC HalfKay USB bootloader
  - Pushbutton for entering bootloader mode

Pin Configuration
=================

Nearly all MCU port pins are routed to the outer header pins or underside pads.
The dedicated onboard hardware connections are:

======== ==============================================
Port Pin Board Usage
======== ==============================================
PTA0     Mini54TAN / Bootloader communication
PTA1     Mini54TAN / Bootloader communication
PTA2     Mini54TAN / Bootloader communication
PTA3     Mini54TAN / Bootloader communication
PTA18    16 MHz Crystal (XTAL)
PTA19    16 MHz Crystal (XTAL)
PTB1     Mini54TAN / Bootloader communication
PTB2     Mini54TAN / Bootloader communication
PTC5     User LED (Orange)
======== ==============================================

Serial Console
==============

The Kinetis K20 provides three UART modules with pins exposed on the board
headers:

============= ======== ============= =====================================
UART Module   Port Pin Board Label   PJRC Pinout Description
============= ======== ============= =====================================
**UART0_RX**  PTB16    Pin 0         RX1 / Touch
\             PTD6     Pin 21 / A7   RX1 / CS / PWM
\             PTA1     (Internal)    Mini54TAN / Bootloader
**UART0_TX**  PTB17    Pin 1         TX1 / Touch
\             PTD7     Pin 5         TX1 / PWM
\             PTA2     (Internal)    Mini54TAN / Bootloader
**UART1_RX**  PTC3     Pin 9         RX2 / CS / PWM
\             PTE1     Pad 26        Pad on underside of board
**UART1_TX**  PTC4     Pin 10        TX2 / CS / PWM
\             PTE0     Pad 31        Pad on underside of board
**UART2_RX**  PTD2     Pin 7         RX3 / DOUT
**UART2_TX**  PTD3     Pin 8         TX3 / DIN
============= ======== ============= =====================================

The default serial console in the ``nsh`` configuration is **UART0** on:

- **RX**: Pin 0 (PTB16)
- **TX**: Pin 1 (PTB17)
- **Baud rate**: 115200 8N1

Connect a 3.3V USB-to-UART serial adapter (TX to Pin 0, RX to Pin 1, GND to GND)
to access the NuttShell prompt.

LEDs
====

A single orange LED is connected to port pin **PTC5**. The cathode is connected
to ground, so driving PTC5 high illuminates the LED.

When ``CONFIG_ARCH_LEDS`` is enabled, NuttX indicates system status as
follows:

==================== ======================= ======
Symbol               Meaning                 LED
==================== ======================= ======
``LED_STARTED``      NuttX has started       OFF
``LED_HEAPALLOCATE`` Heap has been allocated OFF
``LED_IRQSENABLED``  Interrupts enabled      OFF
``LED_STACKCREATED`` Idle stack created      ON
``LED_INIRQ``        Handling an interrupt   No Change
``LED_SIGNAL``       Inside signal handler   No Change
``LED_ASSERTION``    Assertion failed        No Change
``LED_PANIC``        System has crashed      Flashing (2 Hz)
==================== ======================= ======

When NuttX boots successfully and enters normal idle operation, the LED
remains solidly **ON**. If a kernel panic occurs, the LED flashes at
approximately 2 Hz.

Flashing and Debugging
======================

Using the Teensy Loader
-----------------------

The Teensy 3.x comes pre-programmed with PJRC's proprietary HalfKay
bootloader managed by the onboard Mini54TAN chip. The bootloader communicates
over USB via the `Teensy Loader <https://www.pjrc.com/teensy/loader.html>`_
application or the command-line tool `teensy_loader_cli
<https://www.pjrc.com/teensy/loader_cli.html>`_.

NuttX configurations for the Teensy 3.x enable Intel HEX binary generation
(``CONFIG_INTELHEX_BINARY=y``), which produces ``nuttx.hex`` in the root
directory after compilation.

Flashing with ``teensy_loader_cli``:

1. Install ``teensy_loader_cli`` on your host system:

   .. code-block:: bash

      # Ubuntu / Debian
      sudo apt-get install libusb-dev
      git clone https://github.com/PaulStoffregen/teensy_loader_cli.git
      cd teensy_loader_cli && make && sudo make install

2. Build NuttX and upload the generated ``nuttx.hex``:

   .. code-block:: bash

      # For Teensy 3.1 / 3.2 (MK20DX256):
      teensy_loader_cli --mcu=mk20dx256 -w -v nuttx.hex

      # For Teensy 3.0 (MK20DX128):
      teensy_loader_cli --mcu=mk20dx128 -w -v nuttx.hex

3. Press the pushbutton on the Teensy 3.x board to initiate the programming
   cycle. Once flashing completes, the board reboots into NuttX automatically.

Hardware Debugging (SWD / JTAG)
-------------------------------

Standard hardware debugging via SWD is not directly available out of the box
because the onboard Mini54TAN bootloader chip controls the Reset and SWD
lines of the K20 microcontroller.

To use an external SWD debugger (such as J-Link, ST-Link, or CMSIS-DAP), the
hardware traces between the Mini54TAN and the MK20 chip must be severed or
the Mini54TAN held in permanent reset. For detailed instructions on this
modification, refer to the `Hacking the Teensy 3.1 for SWD Debugging
<https://mcuoneclipse.com/2014/08/09/hacking-the-teensy-v3-1-for-swd-debugging/>`_
guide on MCU on Eclipse.

Supported Configurations
========================

The board configurations for the Teensy 3.x are located under
``boards/arm/kinetis/teensy-3.x/configs/``.

nsh
---

The default NuttShell (NSH) configuration provides an interactive command-line
interface over the primary serial port.

- **Console**: UART0 (Pin 0 RX, Pin 1 TX) at 115200 8N1
- **Binary formats**: Raw binary (``nuttx.bin``) and Intel HEX (``nuttx.hex``)
- **Builtin applications**: Enabled

Building:

.. code-block:: bash

   ./tools/configure.sh teensy-3.x:nsh
   make -j$(nproc)
   teensy_loader_cli --mcu=mk20dx256 -w -v nuttx.hex  # Use --mcu=mk20dx128 for Teensy 3.0

usbnsh
------

Configures the NuttShell console to run over the native USB port using the
USB CDC/ACM class driver (virtual COM port).

- **Console**: USB CDC/ACM (``/dev/ttyACM0`` on Linux, ``COMx`` on Windows)
- **Driver**: Kinetis Full-Speed USB Device controller (``CONFIG_KINETIS_USBOTG=y``)
- **Hardware requirement**: Standard Micro-USB cable connected directly to the host PC

Building:

.. code-block:: bash

   ./tools/configure.sh teensy-3.x:usbnsh
   make -j$(nproc)
   teensy_loader_cli --mcu=mk20dx256 -w -v nuttx.hex  # Use --mcu=mk20dx128 for Teensy 3.0

After flashing and rebooting, connect to the USB serial device:

.. code-block:: bash

   # Linux
   minicom -D /dev/ttyACM0

   # Or using picocom
   picocom -b 115200 /dev/ttyACM0
