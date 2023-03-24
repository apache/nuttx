==============
ESP-WROVER-KIT
==============

The `ESP-WROVER-KIT <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-wrover-kit.html>`_ is a development board for the ESP32 SoC from Espressif, based on a ESP32-WROVER-B module.

.. list-table::
   :align: center

   * - .. figure:: esp-wrover-kit-v4.1-layout-back.png
          :align: center

          ESP-WROVER-KIT board layout - front

     - .. figure:: esp-wrover-kit-v4.1-layout-front.png
          :align: center

          ESP-WROVER-KIT board layout - back

Features
========

  - ESP32-WROVER-B module
  - LCD screen
  - MicroSD card slot

Its another distinguishing feature is the embedded FTDI FT2232HL chip,
an advanced multi-interface USB bridge. This chip enables to use JTAG
for direct debugging of ESP32 through the USB interface without a separate
JTAG debugger. ESP-WROVER-KIT makes development convenient, easy, and
cost-effective.

Most of the ESP32 I/O pins are broken out to the board's pin headers for easy access.

Serial Console
==============

UART0 is, by default, the serial console. It connects to the on-board
FT2232HL converter and is available on the USB connector USB CON8 (J5).

It will show up as /dev/ttyUSB[n] where [n] will probably be 1, since
the first interface ([n] == 0) is dedicated to the USB-to-JTAG interface.

Buttons and LEDs
================

Board Buttons
-------------

There are two buttons labeled Boot and EN. The EN button is not available
to software. It pulls the chip enable line that doubles as a reset line.

The BOOT button is connected to IO0. On reset it is used as a strapping
pin to determine whether the chip boots normally or into the serial
bootloader. After reset, however, the BOOT button can be used for software
input.

Board LEDs
----------

There are several on-board LEDs for that indicate the presence of power
and USB activity.

There is an RGB LED available for software.

Pin Mapping
===========

===== ========================= ==========
Pin   Signal                    Notes
===== ========================= ==========
0     RGB LED Red / BOOT Button
2     RGB LED Green
4     RGB LED Blue
5     LCD Backlight
18    LCD Reset
19    LCD Clock
21    LCD D/C
22    LCD CS
23    LCD MOSI
25    LCD MISO
===== ========================= ==========

Configurations
==============

All of the configurations presented below can be tested by running the following commands::

    $ ./tools/configure.sh esp32-wrover-kit:<config_name>
    $ make flash ESPTOOL_PORT=/dev/ttyUSB1 -j

Where <config_name> is the name of board configuration you want to use, i.e.: nsh, buttons, wifi...
Then use a serial console terminal like ``picocom`` configured to 115200 8N1.

autopm
------

This configuration makes the device automatically enter the low power consumption mode
when in the idle state, powering off the cpu and other peripherals.

In minimum power save mode, the station wakes up every DTIM to receive a beacon. The broadcast
data will not be lost because it is transmitted after DTIM. However, it can not save much more
power if DTIM is short as the DTIM is determined by the access point.

bmp180
------

This configuration enables the use of the BMP180 pressure sensor over I2C.
You can check that the sensor is working by using the ``bmp180`` application::

    nsh> bmp180
    Pressure value = 91531
    Pressure value = 91526
    Pressure value = 91525

buttons
-------

This configuration shows the use of the buttons subsystem. It can be used by executing
the ``buttons`` application and pressing on any of the available board buttons::

    nsh> buttons
    buttons_main: Starting the button_daemon
    buttons_main: button_daemon started
    button_daemon: Running
    button_daemon: Opening /dev/buttons
    button_daemon: Supported BUTTONs 0x01
    nsh> Sample = 1
    Sample = 0

gpio
----

This is a test for the GPIO driver. It includes the 3 LEDs and one, arbitrary, GPIO.
For this example, GPIO22 was used (defined by the board implementation).
At the nsh, we can turn LEDs on and off with the following::

    nsh> gpio -o 1 /dev/gpio0
    nsh> gpio -o 0 /dev/gpio0

We can use the interrupt pin to send a signal when the interrupt fires::

    nsh> gpio -w 14 /dev/gpio2

The pin is configured to as a rising edge interrupt, so after issuing the
above command, connect it to 3.3V.

lcd1602
-------

This configuration is used to demonstrate the use of an LCD1602 display with
the ESP32-WROVER-KIT. You can run an example by executing the following commands::

    nsh> slcd
    Opening /dev/slcd0 for read/write access
    Attributes:
    rows: 2 columns: 16 nbars: 0
    max contrast: 0 max brightness: 1
    Clear screen
    WRITING:
    0000: 1b5b46                                                            .[F
    Set brightness to 1
    Print [Hello]
    WRITING:
    0000: 1b5b471b5b30304c1b5b4548656c6c6f                                  .[G.[00L.[EHello

leds
----

This configuration demonstrates the use of the on-board RGB LED with the
`userleds` subsystem. To check the included example, you can execute the
following application::

    nsh> leds
    leds_main: Starting the led_daemon
    leds_main: led_daemon started
    led_daemon (pid# 3): Running
    led_daemon: Opening /dev/userleds
    led_daemon: Supported LEDs 0x07
    led_daemon: LED set 0x01
    nsh> led_daemon: LED set 0x02
    led_daemon: LED set 0x03
    led_daemon: LED set 0x04
    led_daemon: LED set 0x05

lua
---

This configuration demonstrates the use of the of the Lua interpreter on NuttX.
To execute it, just run the ``lua`` application.

lvgl
----

This is a demonstration of the LVGL graphics library running on the NuttX LCD
driver. You can find LVGL here::

    https://www.lvgl.io/
    https://github.com/lvgl/lvgl

This configuration uses the LVGL demonstration at `apps/examples/lvgldemo`.

mmcsdspi
--------

This configuration is used to mount a FAT/FAT32 SD Card into the OS' filesystem.
To access the card's files, execute the following commands::

    nsh> mount -t vfat /dev/mmcsd0 /mnt
    nsh> ls /mnt/
    /mnt:
    song_16_88200_2ch.wav
    song_16_96000_2ch.wav
    song_24_44100_2ch.wav
    song_32_44100_2ch.wav

nsh
---

Basic NuttShell configuration (console enabled in UART0, exposed via
USB connection by means of FT2232HL converter, at 115200 bps).

nx
--

This config adds a set of tests using the graphic examples at ``apps/example/nx``.

This configuration illustrates the use of the LCD with the lower performance
SPI interface.

oneshot
-------

This config demonstrate the use of oneshot timers present on the ESP32.
To test it, just run the ``oneshot`` example::

    nsh> oneshot
    Opening /dev/oneshot
    Maximum delay is 4294967295999999
    Starting oneshot timer with delay 2000000 microseconds
    Waiting...
    Finished

rtc
---

This configuration demonstrates the use of the RTC driver through alarms.
You can set an alarm, check its progress and receive a notification after it expires::

    nsh> alarm 10
    alarm_daemon started
    alarm_daemon: Running
    Opening /dev/rtc0
    Alarm 0 set in 10 seconds
    nsh> alarm -r
    Opening /dev/rtc0
    Alarm 0 is active with 10 seconds to expiration
    nsh> alarm_daemon: alarm 0 received

tickless
--------

This configuration enables the support for tickless scheduler mode.

wifi
----

Enables Wi-Fi support. You can define your credentials this way::

    $ make menuconfig
    -> Application Configuration
        -> Network Utilities
            -> Network initialization (NETUTILS_NETINIT [=y])
                -> WAPI Configuration

Or if you don't want to keep it saved in the firmware you can do it
at runtime::

    nsh> wapi psk wlan0 mypasswd 3
    nsh> wapi essid wlan0 myssid 1
    nsh> renew wlan0