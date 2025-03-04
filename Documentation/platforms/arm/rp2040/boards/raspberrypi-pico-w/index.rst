===============================
Raspberry Pi Pico W
===============================

The `Raspberry Pi Pico <https://www.raspberrypi.com/products/raspberry-pi-pico/>`_ is a general purpose board supplied by
the Raspberry Pi Foundation. The W variant adds built in WiFi communications.

.. figure:: RaspberryPiPicoW.png
   :align: center

Features
========

* RP2040 microcontroller chip
* Dual-core ARM Cortex M0+ processor, flexible clock running up to 133 MHz
* 264kB of SRAM, and 2MB of on-board Flash memory
* Castellated module allows soldering direct to carrier boards
* USB 1.1 Host and Device support
* Low-power sleep and dormant modes
* Drag & drop programming using mass storage over USB
* 26 multi-function GPIO pins
* 2× SPI, 2× I2C, 2× UART, 3× 12-bit ADC, 16× controllable PWM channels
* Accurate clock and timer on-chip
* Temperature sensor
* Accelerated floating point libraries on-chip
* 8 × Programmable IO (PIO) state machines for custom peripheral support
* Built in WiFi radio (Infineon CYW43439)

Serial Console
==============

By default a serial console appears on pins 1 (TX GPIO0) and pin 2
(RX GPIO1).  This console runs a 115200-8N1.

The board can be configured to use the USB connection as the serial console.

Buttons and LEDs
================

LED controlled by GPIO0 of the wireless chip (not the RP2040 processor).
Use rp2040_extra_gpio_put(0,value) to control this LED.

A BOOTSEL button, which if held down when power is first
applied to the board, will cause the RP2040 to boot into programming
mode and appear  as a storage device to a computer connected via USB.
Saving a .UF2 file to this device will replace the Flash ROM contents
on the RP2040.

Wireless Communication
======================

The on board Infineon CYW43439 supports 2.4 GHz WiFi 4 communications
(802.11n), WPS3 and SoftAP with up to four clients.

Pin Mapping
===========

===== ========== ==========
Pin   Signal     Notes
===== ========== ==========
1     GPIO0      Default TX for UART0 serial console
2     GPIO1      Default RX for UART0 serial console
3     Ground
4     GPIO2
5     GPIO3
6     GPIO4      Default SDA for I2C0
7     GPIO5      Default SCL for I2C0
8     Ground
9     GPIO6      Default SDA for I2C1
10    GPIO7      Default SCL for I2C1
11    GPIO8      Default RX for SPI1
12    GPIO9      Default CSn for SPI1
13    Ground
14    GPIO10     Default SCK for SPI1
15    GPIO11     Default TX for SPI1
16    GPIO12
17    GPIO13
18    Ground
19    GPIO14
20    GPIO15
21    GPIO16     Default RX for SPI0
22    GPIO17     Default CSn for SPI0
23    Ground
24    GPIO18     Default SCK for SPI0
25    GPIO19     Default TX for SPI0
26    GPIO20     Default TX for UART1 serial console
27    GPIO21     Default RX for UART1 serial console
28    Ground
29    GPIO22
30    Run
31    GPIO26     ADC0
32    GPIO27     ADC1
33    AGND       Analog Ground
34    GPIO28     ADC2
35    ADC_VREF   Analog reference voltage
36    3V3        Power output to peripherals
37    3V3_EN     Pull to ground to turn off.
38    Ground
39    VSYS       +5V Supply to board
40    VBUS       Connected to USB +5V
===== ========== ==========

Other RP2040 Pins
=================

GPIO23 Output - WiFi controller enable.
GPIO24 I/O    - WiFi controller data line.
GPIO25 Output - WiFi controller chip select line.
GPIO29 Output - WiFi controller clock line.
ADC3   Input  - Analog voltage equal to one third of VSys voltage.

Note: ADC3 and GPIO29 share the same pin on the RP2040. If the
GPIO25 line is held high (Wifi controller NOT selected) then a voltage
equal to one third of the VSys voltage with appear on this line and
can be read with ADC3.  When the WiFi chip is selected this voltage
will be removed so the line can be used as a clock for data exchange
with the WiFi controller.

Separate pins for the Serial Debug Port (SDB) are available

WiFi Controller GPIO
====================

* GPIO0 - Output - On board LED.
* GPIO1 - Output - Power supply control.
* GPIO2 - Input  - High if USB port or Pad 40 supplying power.

Power Supply
============

The Raspberry Pi Pico can be powered via the USB connector,
or by supplying +5V to pin 39.  The board had a diode that prevents
power from pin 39 from flowing back to the USB socket, although
the socket can be power via pin 30.

The Raspberry Pi Pico chip run on 3.3 volts.  This is supplied
by an onboard voltage regulator.  This regulator can be disabled
by pulling pin 37 to ground.

The regulator can run in two modes.  By default the regulator runs
in PFM mode which provides the best efficiency, but may be
switched to PWM mode for improved ripple by outputting a one
on the wireless chip's GPIO1 (not the RP2040's GPIO1).

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

* BMP180 sensor at I2C0 (don't forget to define I2C0 GPIOs at "I2C0 GPIO pin assign" in Board Selection menu)
* INA219 sensor / module (don't forget to define I2C0 GPIOs at "I2C0 GPIO pin assign" in Board Selection menu)
* SHT4X sensor at I2C0
* Pico Display Pack (ST7789 LCD)

  * RGB leds and buttons are not supported yet.

* Pico Audio Pack (PCM5100A I2S DAC)

  * I2S interface is realized by PIO.

* WS2812 smart pixel support

Installation
============

1. Download Raspberry Pi Pico SDK and update submodule (cyw43-driver)

.. code-block:: console

  $ git clone -b 2.0.0 https://github.com/raspberrypi/pico-sdk.git
  $ cd pico-sdk
  $ git submodule update --init --recursive lib/cyw43-driver

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
  $ ./tools/configure.sh raspberrypi-pico-w:nsh
  $ make V=1

5. Connect Raspberry Pi Pico board to USB port while pressing BOOTSEL.
   The board will be detected as USB Mass Storage Device.
   Then copy "nuttx.uf2" into the device.
   (Same manner as the standard Pico SDK applications installation.)

6. To access the console, GPIO 0 and 1 pins must be connected to the
   device such as USB-serial converter.

   `usbnsh` configuration provides the console access by USB CDC/ACM serial
   device.  The console is available by using a terminal software on the USB
   host.

Configurations
==============

audiopack
---------

NuttShell configuration (console enabled in UART0, at 115200 bps) with
support for NXPlayer audio player.

Pico Audio Pack support.
See the following page for connection:
https://shop.pimoroni.com/products/pico-audio-pack
SD card interface is also enabled.

composite
---------

NuttShell configuration (console enabled in UART0, at 115200 bps) with support for
CDC/ACM with MSC USB composite driver. ``conn`` command enables the composite
device.

displaypack
-----------

NuttShell configuration (console enabled in USB Port, at 115200 bps) supporting
ST7789 video display.

See the following page for connection:
https://shop.pimoroni.com/products/pico-display-pack

enc28j60
--------

NuttShell configuration (console enabled in UART0, at 115200 bps) with support for
ENC28J60.

ENC28J60 SPI ethernet controller supports:

  - IP address is configured by DHCP.
  - DNS address is 8.8.8.8 (CONFIG_NETINIT_DNSIPADDR)
  - NTP client is enabled.

.. list-table:: ENC28J60 connections
   :widths: auto
   :header-rows: 1

   * - ENC28J60
     - Raspberry Pi Pico W
   * - GND
     - GND (Pin 3 or 38 or ...)
   * - 3.3V
     - 3V3 OUT (Pin 36)
   * - SI
     - GP15 (SPI1 TX) (Pin 20)
   * - SCK
     - GP14 (SPI1 SCK) (Pin 19)
   * - CS
     - GP13 (SPI1 CSn) (Pin 17)
   * - SO
     - GP12 (SPI1 RX) (Pin 16)
   * - INT
     - GP11 (Pin 15)
   * - RESET
     - GP10 (Pin 14)

lcd1602
-------

NuttShell configuration (console enabled in UART0, at 115200 bps) with support for
LCD1602 Segment LCD Display (I2C).

.. list-table:: LCD1602 connections
   :widths: auto
   :header-rows: 1

   * - PCF8574 BackPack 
     - Raspberry Pi Pico W
   * - GND
     - GND (Pin 3 or 38 or ...)
   * - VCC
     - 5V Vbus (Pin 40)
   * - SDA
     - GP4 (I2C0 SDA) (Pin 6)
   * - SCL
     - GP5 (I2C0 SCL) (Pin 7)

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
     - Raspberry Pi Pico W
   * - DAT2          
     - Not connected
   * - DAT3/CS
     - GP17 (SPI0 CSn) (Pin 22)
   * - CMD /DI
     - GP19 (SPI0 TX)  (Pin 25)
   * - VDD
     - 3V3 OUT (Pin 36)
   * - CLK/SCK
     - GP18 (SPI0 SCK) (Pin 24)
   * - VSS
     - GND (Pin 3 or 38 or ...)
   * - DAT0/DO
     - GP16 (SPI0 RX)  (Pin 21)
   * - DAT1          
     - Not connected

Card hot swapping is not supported.

ssd1306
-------

NuttShell configuration (console enabled in UART0, at 115200 bps) with support
for SSD1306 OLED display (I2C) test configuration.

.. list-table:: SSD1306 connections
   :widths: auto
   :header-rows: 1

   * - SSD1306
     - Raspberry Pi Pico W
   * - GND
     - GND (Pin 3 or 38 or ...)
   * - VCC
     - 3V3 OUT (Pin 36)
   * - SDA
     - GP4 (I2C0 SDA) (Pin 6)
   * - SCL   
     - GP5 (I2C0 SCL) (Pin 7)

st7735
------

NuttShell configuration (console enabled in UART0, at 115200 bps) with support for
ST7735 SPI LCD.

.. list-table:: ST7735 connections
   :widths: auto
   :header-rows: 1

   * - st7735
     - Raspberry Pi Pico W
   * - GND
     - GND (Pin 3 or 38 or ...)
   * - VCC
     - 5V Vbus (Pin 40)
   * - SDA
     - GP15 (SPI1 TX) (Pin 20)
   * - SCK
     - GP14 (SPI1 SCK) (Pin 19)
   * - CS
     - GP13 (SPI1 CSn) (Pin 17)
   * - AO(D/C)
     - GP12 (SPI1 RX) (Pin 16)
   * - BL
     - GP11 (Pin 15)
   * - RESET
     - GP10 (Pin 14)

telnet
------

NuttShell configuration (console enabled in UART0, at 115200 bps) with WiFi client
mode and both telnet server and client enabled.

After loading this configuration use make menuconfig to change the country code in
Device Drivers->Wireless Device Support->IEEE 802.11 Device Support and the 
wireless configuration in Application Configuration->Network Utilities->Network 
initialization->WAPI Configuration to match your wireless network.

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

waveshare-lcd-1.14
------------------

NuttShell configuration (console enabled in UART0, at 115200 bps) with support for
st7789.

waveshare-lcd-1.3
-----------------

NuttShell configuration (console enabled in UART0, at 115200 bps) with support for
usbmsc.

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
