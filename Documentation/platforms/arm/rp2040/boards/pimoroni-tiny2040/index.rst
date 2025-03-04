=================
Pimoroni Tiny2040
=================

The Tiny2040 is a general purpose RP2040 board supplied by Pimoroni. 

.. figure:: Tiny2040.png
   :align: center

The Pimoroni Tiny 2040 has two buttons (RESET and BOOT) allowing to boot
from ROM without disconnecting the device.

See the `Pimoroni website
<https://shop.pimoroni.com/products/tiny-2040?variant=39560012234835/>`_ for
information about the Pimoroni Tiny 2040.

Features
========

* RP2040 microcontroller chip
* Dual-core ARM Cortex M0+ processor, flexible clock running up to 133 MHz
* 264kB of SRAM, and 2MB or 8MB of on-board Flash memory
* Castellated module allows soldering direct to carrier boards
* USB Host and Device support via type C connector
* Low-power sleep and dormant modes
* Drag & drop programming using mass storage over USB
* 12 multi-function GPIO pins
* 2× SPI, 2× I2C, 2× UART, 3× 12-bit ADC, 16× controllable PWM channels
* Accurate clock and timer on-chip
* Temperature sensor
* Accelerated floating point libraries on-chip
* 8 × Programmable IO (PIO) state machines for custom peripheral support

Serial Console
==============

By default a serial console appears on pins 15 (RX GPIO0) and 
pin 16 (TX GPIO1).  This console runs a 115200-8N1.

The board can be configured to use the USB connection as the serial console.

Buttons and LEDs
================

There is a single onboard RGB LED controlled by pins
GPIO18 (red), GPIO19 (green), and GPIO20 (blue).

The is a User/BOOT button readable as GPIO23. If held down when power
is first applied the RP2040 will boot into program mode and appear to
a computer connected via USB as a storage device.  Saving
a .UF2 file to this device will replace the Flash ROM contents 
on the RP2040.

Pin Mapping
===========
Pads numbered anticlockwise from USB connector.

===== ========== ==========
Pad   Signal     Notes
===== ========== ==========
1     VBUS       Connected to USB +5V
2     Ground
3     3V3        Out to peripherals
4     GPIO29     ADC3
5     GPIO28     ADC2
6     GPIO27     ADC1
7     GPIO26     ADC0
8     Ground
9     GPIO7
10    GPIO6
11    GPIO5
12    GPIO4
13    GPIO3
14    GPIO2
15    GPIO1      Default RX for UART0 serial console
16    GPIO0      Default TX for UART0 serial console
===== ========== ==========

Power Supply 
============

The Raspberry Pi Pico can be powered via the USB connector,
or by supplying +5V to pin 1. 

The Raspberry Pi Pico chip run on 3.3 volts.  This is supplied
by an onboard voltage regulator.

Supported Capabilities
======================

NuttX supports the following RP2040 capabilities:

* UART  (console port)

  * GPIO 0 (UART0 TX) and GPIO 1 (UART0 RX) are used for the console.

* I2C
* SPI (master only)
* DMAC
* PWM
* ADC
* Watchdog
* USB device

  * MSC, CDC/ACM serial and these composite device are supported.
  * CDC/ACM serial device can be used for the console.

* PIO (RP2040 Programmable I/O)
* Flash ROM Boot
* SRAM Boot

  * If Pico SDK is available, nuttx.uf2 file which can be used in BOOTSEL mode will be created.

* Persistent flash filesystem in unused flash ROM
* WiFi wireless communication

There is currently no direct user mode access to these RP2040 hardware features:

* SPI Slave Mode
* SSI
* RTC
* Timers

NuttX also provide support for these external devices:

* WS2812 smart pixel support

Installation
============

1. Download Raspberry Pi Pico SDK

.. code-block:: console

  $ git clone -b 2.0.0 https://github.com/raspberrypi/pico-sdk.git

2. Download and install picotool

  Instructions can be found here: https://github.com/raspberrypi/picotool

  If you are on Arch Linux, you can install the picotool through the AUR:

.. code-block:: console

  $ yay -S picotool

3. Set PICO_SDK_PATH environment variable

.. code-block:: console

  $ export PICO_SDK_PATH=<absolute_path_to_pico-sdk_directory>

4. Configure and build NuttX

.. code-block:: console

  $ git clone https://github.com/apache/nuttx.git nuttx
  $ git clone https://github.com/apache/nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh pimoroni-tiny2040:nsh
  $ make V=1

5. Connect Pimoroni Tiny 2040 board to USB port. While pressing the
   BOOT button, shortly press the RESET button. On releasing the BOOT
   button the board boots from internal ROM and will be detected as
   USB Mass Storage Device. Then copy "nuttx.uf2" into the device.
   (Same manner as the standard Pico SDK applications installation.)

6. To access the console, GPIO 0 and 1 pins must be connected to the
   device such as USB-serial converter.

   `usbnsh` configuration provides the console access by USB CDC/ACM serial
   decive.  The console is available by using a terminal software on the USB
   host.

Configurations
==============

composite
---------

NuttShell configuration (console enabled in UART0, at 115200 bps) with support for
CDC/ACM with MSC USB composite driver. ``conn`` command enables the composite
device.

gpio
--------

NuttShell configuration (console enabled in UART0, at 115200 bps) with GPIO examples.

.. list-table:: GPIO pin options
   :widths: auto
   :header-rows: 1

   * - GPIO
     - Function
   * - GPIO18
     - Onboard RGB LED (red, out)
   * - GPIO19
     - Onboard RGB LED (green, out)
   * - GPIO20
     - Onboard RGB LED (blue, out)
   * - GPIO23
     - Onboard BOOT button (user)

No interrupt pin configured.

nsh
---

Basic NuttShell configuration (console enabled in UART0, at 115200 bps).

nsh-flash
---------

Basic NuttShell configuration (console enabled in UART0, at 115200 bps
with SMART flash filesystem.

nshsram
-------

NuttShell configuration (console enabled in UART0, at 115200 bps) with interrupt
vectors in RAM.

smp
---

Basic NuttShell configuration (console enabled in UART0, at 115200 bps) with
both ARM cores enabled.

spisd
-----

NuttShell configuration (console enabled in UART0, at 115200 bps) with SPI SD
card support enabled.

.. list-table:: spisd connections
   :widths: auto
   :header-rows: 1

   * - SD card slot
     - Pimoroni Tiny 2040
   * - DAT2
     - Not connected
   * - DAT3/CS
     - GP5 (SPI0 CSn) (Pin 11)
   * - CMD /DI
     - GP7 (SPI0 TX)  (Pin 9)
   * - VDD
     - 3V3 OUT (Pin 3)
   * - CLK/SCK
     - GP6 (SPI0 SCK) (Pin 10)
   * - VSS
     - GND (Pin 2 or 8)
   * - DAT0/DO
     - GP4 (SPI0 RX)  (Pin 12)
   * - DAT1          
     - Not connected

Card hot swapping is not supported.

usbmsc
------

NuttShell configuration (console enabled in UART0, at 115200 bps) with support for
USB MSC and CDC/ACM.

``msconn`` and ``sercon`` commands enable the MSC and CDC/ACM devices. The MSC
support provides the interface to the SD card with SPI, so the SD card slot
connection like spisd configuration is required.

usbnsh
------

Basic NuttShell configuration using CDC/ACM serial (console enabled in USB Port,
at 115200 bps).

License exceptions
==================

The following files are originated from the files in Pico SDK.
So, the files are licensed under 3-Clause BSD same as Pico SDK.

* arch/arm/src/rp2040/rp2040_clock.c
* arch/arm/src/rp2040/rp2040_pll.c
* arch/arm/src/rp2040/rp2040_xosc.c

  * These are created by referring the Pico SDK clock initialization.

* arch/arm/src/rp2040/rp2040_pio.c
* arch/arm/src/rp2040/rp2040_pio.h
* arch/arm/src/rp2040/rp2040_pio_instructions.h

  * These provide the similar APIs to Pico SDK's hardware_pio APIs.

* arch/arm/src/rp2040/hardware/\*.h

  * These are generated from rp2040.svd originally provided in Pico SDK.
