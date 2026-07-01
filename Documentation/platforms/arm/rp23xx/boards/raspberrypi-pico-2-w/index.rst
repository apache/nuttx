==============================
Raspberry Pi Pico 2 W
==============================

.. tags:: chip:rp2350, wifi

The `Raspberry Pi Pico 2 W <https://www.raspberrypi.com/products/raspberry-pi-pico-2/>`_
is a general purpose board supplied by Raspberry Pi.  The W variant adds an
Infineon CYW43439 for built-in 2.4 GHz WiFi and Bluetooth.

Features
========

* RP2350 microcontroller chip
* Dual-core ARM Cortex-M33 processor, flexible clock running up to 150 MHz
* 520 kB of SRAM, and 4 MB of on-board Flash memory
* Castellated module allows soldering direct to carrier boards
* USB 1.1 Host and Device support
* Low-power sleep and dormant modes
* Drag & drop programming using mass storage over USB
* 26 multi-function GPIO pins
* 2x SPI, 2x I2C, 2x UART, 3x 12-bit ADC, 16x controllable PWM channels
* Accurate clock and timer on-chip
* Temperature sensor
* Accelerated floating point libraries on-chip
* 12 x Programmable IO (PIO) state machines for custom peripheral support
* Built in WiFi radio (Infineon CYW43439)

Serial Console
==============

By default a serial console appears on pin 1 (TX GPIO0) and pin 2
(RX GPIO1).  This console runs at 115200-8N1.

The board can be configured to use the USB connection as the serial console;
see the ``usbnsh`` configuration of the base ``raspberrypi-pico-2`` board.

Buttons and LEDs
================

The user LED is controlled by the CYW43439 wireless chip (not directly by an
RP2350 GPIO), so networking must be brought up before it can be driven.

A BOOTSEL button, which if held down when power is first applied to the board,
will cause the RP2350 to boot into programming mode and appear as a storage
device to a computer connected via USB.  Saving a .UF2 file to this device
will replace the Flash ROM contents on the Pico 2 W.

Wireless Communication
======================

The on board Infineon CYW43439 supports 2.4 GHz WiFi 4 (802.11n).  It is
driven over a PIO-based gSPI link by the NuttX ``bcmf`` FullMAC driver, ported
from the RP2040 ``raspberrypi-pico-w`` support.

WiFi firmware blob
------------------

The CYW43439 requires a firmware/CLM blob that is **not** bundled with NuttX.
It is fetched from the Raspberry Pi Pico SDK at build time; set
``PICO_SDK_PATH`` (see Installation) or point
``CONFIG_CYW43439_FIRMWARE_BIN_PATH`` at a local copy.  When the blob is
absent the build substitutes a dummy so that CI still passes, but an image
built with the dummy will not associate to an access point.

The board's NVRAM calibration data (``src/rp23xx_firmware.c``) is the same
Broadcom-supplied table used by the RP2040 ``raspberrypi-pico-w`` board.

RP2350 Wireless Pins
====================

The CYW43439 is wired to the RP2350 in the same arrangement as the RP2040
Pico W:

* GPIO23 Output - WiFi controller power / on-off control.
* GPIO24 I/O    - WiFi controller data line (gSPI).
* GPIO25 Output - WiFi controller chip select line.
* GPIO29 Output - WiFi controller clock line.

Supported Capabilities
======================

NuttX supports the following Pico 2 W capabilities:

* UART (console port)

  * GPIO 0 (UART0 TX) and GPIO 1 (UART0 RX) are used for the console.

* WiFi station mode (CYW43439, 2.4 GHz 802.11n) via the ``bcmf`` driver
* I2C
* SPI (master only)
* DMAC
* PWM
* ADC
* Watchdog
* USB device
* PIO (RP2350 Programmable I/O)
* Flash ROM Boot / SRAM Boot
* Persistent flash filesystem in unused flash ROM

Installation
============

The Pico 2 W shares the RP2350 tool-chain and SDK setup of the base
``raspberrypi-pico-2`` board; follow the Installation steps of the main
:doc:`RP2350 documentation <../../index>` to install the Pico SDK and
``picotool`` and to set ``PICO_SDK_PATH`` (which also provides the CYW43439
firmware blob).

Configurations
==============

All configurations listed below can be configured using the following command
in the ``nuttx`` directory (consult the main :doc:`RP2350 documentation
<../../index>`):

.. code:: console

   $ ./tools/configure.sh raspberrypi-pico-2-w:<configname>

telnet
------

NuttShell configuration (console enabled on UART0, at 115200 bps) with WiFi
station mode brought up at boot and a telnet server enabled, so the shell is
reachable over the network once the board associates.

Set your network credentials before building via ``make menuconfig`` under
Application Configuration -> Network Utilities -> Network initialization ->
WAPI Configuration (SSID and passphrase), and the country code under Device
Drivers -> Wireless Device Support -> IEEE 802.11 Device Support.  The shipped
defconfig carries only placeholder credentials.
