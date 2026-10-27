=====================
Raspberry Pi Pico 2 W
=====================

.. tags:: arch:arm, chip:rp23xx, part:rp2350, vendor:raspberry-pi

The `Raspberry Pi Pico 2 W <https://www.raspberrypi.com/products/raspberry-pi-pico-2/>`_
is the Raspberry Pi Pico 2 with an Infineon CYW43439 added for 2.4 GHz WiFi and
Bluetooth.

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
* Infineon CYW43439 for 2.4 GHz WiFi 4 (802.11n) and Bluetooth

Serial Console
==============

By default the serial console is UART0 on GPIO 0 (TX, pin 1) and GPIO 1
(RX, pin 2), at 115200-8N1.

The ``composite`` configuration instead puts the console on the USB
connection, as a CDC-ACM serial device.

Wireless Communication
======================

The on-board CYW43439 is driven over a half-duplex gSPI bus by a PIO state
machine, implemented in ``arch/arm/src/rp23xx/rp23xx_cyw43439.c`` (the same
driver as the :doc:`../pimoroni-pico-plus-2-w/index`).  It appears as the
``wlan0`` network device and supports 2.4 GHz WiFi 4 (802.11n) in station
mode.  Bluetooth is not supported.  The chip's firmware is downloaded when
``wlan0`` is brought up.

The CYW43439 is wired to the RP2350 as on the RP2040 Pico W:

* GPIO23 Output - power / on-off control
* GPIO24 I/O    - gSPI data, also the interrupt line
* GPIO25 Output - chip select
* GPIO29 Output - gSPI clock

CYW43439 firmware
-----------------

The wireless chip's firmware and CLM blob are linked into the NuttX binary
from ``CONFIG_CYW43439_FIRMWARE_BIN_PATH``, which defaults to the copy in the
Raspberry Pi Pico SDK::

  ${PICO_SDK_PATH}/lib/cyw43-driver/firmware/43439A0-7.95.49.00.combined

pico-sdk 2.x ships that file only as a C header; the
:doc:`../pimoroni-pico-plus-2-w/index` page shows how to convert it back.  If
the file is missing the build still succeeds, but the wireless chip will not
come up.

User LED
========

The board's LED is wired to GPIO 0 of the CYW43439, not to a pin of the
RP2350, so setting it means a request over the gSPI bus:

* The LED does not respond until ``wlan0`` has been brought up.
* It cannot be driven from interrupt context, so ``CONFIG_ARCH_LEDS`` is not
  supported; a build with it enabled is rejected with an explicit error.

With ``CONFIG_USERLED`` the LED appears as ``/dev/userleds`` and can be
driven with the ``leds`` example.

Buttons
=======

The BOOTSEL button, if held down when power is first applied to the board,
makes the RP2350 boot into programming mode and appear as a storage device to
a computer connected via USB.  Saving a .UF2 file to this device replaces the
Flash ROM contents.  BOOTSEL is not readable as a GPIO.

Supported Capabilities
======================

NuttX supports the following Pico 2 W capabilities:

* UART (console port)

  * GPIO 0 (UART0 TX) and GPIO 1 (UART0 RX) are used for the console.

* WiFi station mode (CYW43439, 2.4 GHz 802.11n)
* I2C
* SPI (master only)
* DMAC
* PWM
* ADC
* Watchdog
* USB device

  * CDC/ACM serial, CDC-NCM network and a composite of the two.
  * CDC/ACM serial device can be used for the console.

* PIO (RP2350 Programmable I/O)
* Flash ROM Boot
* SRAM Boot

Installation
============

The Pico 2 W shares the tool-chain and SDK setup of the
:doc:`../raspberrypi-pico-2/index`; follow the Installation steps of the main
:doc:`RP2350 documentation <../../index>` to install the Pico SDK and
``picotool`` and to set ``PICO_SDK_PATH``, which also provides the CYW43439
firmware.

Configurations
==============

All configurations listed below can be configured using the following command
in the ``nuttx`` directory:

.. code:: console

   $ ./tools/configure.sh raspberrypi-pico-2-w:<configname>

telnet
------

NuttShell on UART0 (115200 bps), with WiFi station mode brought up at boot and
a telnet server, so the shell is reachable over the network once the board
has associated.

Set the network credentials before building, with ``make menuconfig`` under
Application Configuration -> Network Utilities -> Network initialization ->
WAPI Configuration; the shipped defconfig carries placeholders only.

composite
---------

NuttShell on a USB CDC-ACM console, plus a CDC-NCM network interface
(``eth0``) on the same USB connection, and a telnet server.  ``eth0`` only
exists once the USB console has connected, so give it an address from the
console, for example:

.. code:: console

   nsh> ifconfig eth0 192.168.7.1

and put the host's end of the USB link in the same subnet.
