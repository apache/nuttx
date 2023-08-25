================
ESP32-S2-Saola-1
================

The `ESP32-S2-Saola-1 <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/hw-reference/esp32s2/user-guide-saola-1-v1.2.html>`_
is a development board for the ESP32-S2 SoC from Espressif, based on the following modules:

  - ESP32-S2-WROVER
  - ESP32-S2-WROVER-I
  - ESP32-S2-WROOM
  - ESP32-S2-WROOM-I

In this guide, we take ESP32-S2-Saola-1 equipped with ESP32-S2-WROVER as an example.

.. figure:: esp32-s2-saola-1-v1.2-isometric.png
    :alt:  ESP32-S2-Saola-1
    :figclass: align-center

    ESP32-S2-Saola-1

Features
========

  - ESP32-S2-WROVER
    - 4 MB external SPI flash + 2 MB PSRAM
  - USB-to-UART bridge via micro USB port
  - Power LED
  - EN and BOOT buttons
  - RGB LED (Addressable RGB LED (WS2812), driven by GPIO18)

Serial Console
==============

UART0 is, by default, the serial console.  It connects to the on-board
CP2102 converter and is available on the micro-USB connector (J1).

It will show up as /dev/ttyUSB[n] where [n] will probably be 0.

Buttons and LEDs
================

Board Buttons
-------------

There are two buttons labeled Boot and EN.  The EN button is not available
to the software.  It pulls the chip enable line that doubles as a reset line.

The BOOT button is connected to IO0.  On reset, it is used as a strapping
pin to determine whether the chip boots normally or into the serial
bootloader.  After resetting, however, the BOOT button can be used for
software input.

Board LEDs
----------

There are two on-board LEDs. RED_LED (D5) indicates the presence of 3.3V
power and is not controlled by software. RGB LED (U6) is a WS2812 addressable
LED and is driven by GPIO18.

I2S
===

ESP32-S2 has an I2S peripheral accessible using either the generic I2S audio
driver or a specific audio codec driver
(`CS4344 <https://www.cirrus.com/products/cs4344-45-48/>`__ bindings are
available at the moment). The generic I2S audio driver enables using both
the receiver module (RX) and the transmitter module (TX) without using any
specific codec. Also, it's possible to use the I2S character device driver
to bypass the audio subsystem and write directly to the I2S peripheral.

.. note:: When using the audio system, sample rate and data width are
  automatically set by the upper half audio driver.

.. note:: The above statement is not valid when using the I2S character
  device driver.
  It's possible to use 8, 16, 24, and 32-bit-widths writing directly to the
  I2S character device. Just make sure to set the bit-width::

    $ make menuconfig
    -> System Type
        -> ESP32-S2 Peripheral Selection
            -> I2S
                -> Bit Witdh

The following configurations use the I2S peripheral::
  * :ref:`platforms/xtensa/esp32s2/boards/esp32s2-saola-1/index:audio`
  * :ref:`platforms/xtensa/esp32s2/boards/esp32s2-saola-1/index:i2schar`
  * :ref:`platforms/xtensa/esp32s2/boards/esp32s2-saola-1/index:nxlooper`

Configurations
==============

All of the configurations presented below can be tested by running the following commands::

    $ ./tools/configure.sh esp32s2-saola-1:<config_name>
    $ make flash ESPTOOL_PORT=/dev/ttyUSB0 -j

Where <config_name> is the name of board configuration you want to use, i.e.: nsh, buttons, wifi...
Then use a serial console terminal like ``picocom`` configured to 115200 8N1.

audio
-----

This configuration uses the I2S peripheral and an externally connected audio
codec to play an audio file. The easiest way of playing an uncompressed file
is embedding into the firmware. This configuration selects
`romfs example <https://github.com/apache/nuttx-apps/tree/master/examples/romfs>`__
to allow that.

**Audio Codec Setup**

The CS4344 audio codec is connected to the following pins:

============ ========== =========================================
ESP32-S2 Pin CS4344 Pin Description
============ ========== =========================================
33           MCLK       Master Clock
35           SCLK       Serial Clock
34           LRCK       Left Right Clock (Word Select)
36           SDIN       Serial Data In on CS4344. (DOUT on ESP32)
============ ========== =========================================

**ROMFS example**

Prepare and build the ``audio`` defconfig::

  $ make -j distclean && ./tools/configure.sh esp32s2-saola-1:audio && make

This will create a temporary folder in ``apps/examples/romfs/testdir``. Move
a PCM-encoded (``.wav``) audio file with 16 or 24 bits/sample (sampled at 16~48kHz)
to this folder.

.. note:: You can use :download:`this 440 Hz sinusoidal tone <tone.wav>`.
   The audio file should be located at ``apps/examples/romfs/testdir/tone.wav``

Build the project again and flash it (make sure not to clean it, just build)

After successfully built and flashed, load the romfs and play it::

    nsh> romfs
    nsh> nxplayer
    nxplayer> play /usr/share/local/tone.wav

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

coremark
--------

This configuration sets the CoreMark benchmark up for running on the maximum
number of cores for this system. It also enables some optimization flags and
disables the NuttShell to get the best possible score.

.. note:: As the NSH is disabled, the application will start as soon as the
  system is turned on.

cxx
---

Development enviroment ready for C++ applications. You can check if the setup
was successfull by running ``cxxtest``::

    nsh> cxxtest
    Test ofstream ================================
    printf: Starting test_ostream
    printf: Successfully opened /dev/console
    cout: Successfully opened /dev/console
    Writing this to /dev/console
    Test iostream ================================
    Hello, this is only a test
    Print an int: 190
    Print a char: d
    Test std::vector =============================
    v1=1 2 3
    Hello World Good Luck
    Test std::map ================================
    Test C++17 features ==========================
    File /proc/meminfo exists!
    Invalid file! /invalid
    File /proc/version exists!

gpio
----

This is a test for the GPIO driver. It includes one arbitrary GPIO.
For this example, GPIO1 was used (defined by the board implementation).
At the nsh, we can turn the GPIO output on and off with the following::

    nsh> gpio -o 1 /dev/gpio0
    nsh> gpio -o 0 /dev/gpio0

i2c
---

This configuration can be used to scan and manipulate I2C devices.
You can scan for all I2C devices using the following command::

    nsh> i2c dev 0x00 0x7f

i2schar
-------

This configuration enables the I2S character device and the i2schar example
app, which provides an easy-to-use way of testing the I2S peripheral,
enabling both the TX and the RX for those peripherals.

**I2S pinout**

============ ========== =========================================
ESP32-S2 Pin Signal Pin Description
============ ========== =========================================
33           MCLK       Master Clock
35           SCLK       Bit Clock (SCLK)
34           LRCK       Word Select (LRCLK)
36           DOUT       Data Out
37           DIN        Data In
============ ========== =========================================

After successfully built and flashed, run on the boards's terminal::

    nsh> i2schar

The corresponding output should show related debug information.

mcuboot_nsh
-----------

This configuration is the same as the ``nsh`` configuration, but it generates the application
image in a format that can be used by MCUboot. It also makes the ``make bootloader`` command to
build the MCUboot bootloader image using the Espressif HAL.

nsh
---

Basic NuttShell configuration (console enabled in UART0, exposed via
USB connection by means of CP2102 converter, at 115200 bps).

nxlooper
--------

This configuration uses the I2S peripheral as an I2S receiver and
transmitter at the same time. The idea is to capture an I2S data frame
using the RX module and reproduce the captured data on the TX module.

**Receiving and transmitting data on I2S**

The I2S will act as a receiver (master mode), capturing data from DIN, which
needs to be connected to an external source as follows:

============ ========== =========================================
ESP32-S2 Pin Signal Pin Description
============ ========== =========================================
33           MCLK       Master Clock
35           SCLK       Bit Clock (SCLK) Output
34           LRCK       Word Select (LRCLK) Output
36           DOUT       Data Out
37           DIN        Data In
============ ========== =========================================

The DOUT pin will output the captured data frame.

.. note:: The ESP32-S2 contains a single I2S peripheral, so the peripheral
  works on "full-duplex" mode. The `SCLK` and `LRCK` signals are connected
  internally and the TX module is set-up as slave and the RX as master.

**nxlooper**

The ``nxlooper`` application captures data from the audio device with receiving
capabilities and forwards the audio data frame to the audio device with
transmitting capabilities.

After successfully built and flashed, run on the boards's terminal::

    nsh> nxlooper
    nxlooper> loopback

.. note:: ``loopback`` command default arguments for the channel configuration,
  the data width and the sample rate are, respectively, 2 channels,
  16 bits/sample and 48KHz. These arguments can be supplied to select
  different audio formats, for instance::

    nxlooper> loopback 2 8 44100

oneshot
-------

This config demonstrate the use of oneshot timers present on the ESP32-S2.
To test it, just run the ``oneshot`` example::

    nsh> oneshot
    Opening /dev/oneshot
    Maximum delay is 4294967295999999
    Starting oneshot timer with delay 2000000 microseconds
    Waiting...
    Finished

ostest
------

This is the NuttX test at apps/testing/ostest that is run against all new
architecture ports to assure a correct implementation of the OS.

pwm
------

This configuration demonstrates the use of PWM through a LED connected to GPIO2.
To test it, just execute the ``pwm`` application::

    nsh> pwm
    pwm_main: starting output with frequency: 10000 duty: 00008000
    pwm_main: stopping output

random
------

This configuration shows the use of the ESP32-S2's True Random Number Generator with
entropy sourced from Wi-Fi and Bluetooth noise.
To test it, just run ``rand`` to get 32 randomly generated bytes::

    nsh> rand
    Reading 8 random numbers
    Random values (0x3ffe0b00):
    0000  98 b9 66 a2 a2 c0 a2 ae 09 70 93 d1 b5 91 86 c8  ..f......p......
    0010  8f 0e 0b 04 29 64 21 72 01 92 7c a2 27 60 6f 90  ....)d!r..|.'`o.

timer
-----

This config tests the general-use purpose timers. It includes the 4 timers,
adds driver support, registers the timers as devices and includes the timer
example.

To test it, just run the following::

    nsh> timer -d /dev/timerx

Where x in the timer instance.

twai
----

This configuration enables the support for the TWAI (Two-Wire Automotive Interface) driver.
You can test it by connecting TWAI RX and TWAI TX pins which are GPIO0 and GPIO2 by default
to a external transceiver or connecting TWAI RX to TWAI TX pin by enabling
the ``Device Drivers -> CAN Driver Support -> CAN loopback mode`` option and running the ``can`` example::

    nsh> can
    nmsgs: 0
    min ID: 1 max ID: 2047
    Bit timing:
      Baud: 1000000
      TSEG1: 15
      TSEG2: 4
        SJW: 3
      ID:    1 DLC: 1

watchdog
--------

This config test the watchdog timers. It includes the 2 MWDTs,
adds driver support, registers the WDTs as devices and includes the watchdog
example.

To test it, just run the following::

    nsh> wdog -i /dev/watchdogx

Where x is the watchdog instance.
