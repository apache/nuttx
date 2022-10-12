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

Buttons
-------

There are two buttons labeled Boot and EN.  The EN button is not available
to the software.  It pulls the chip enable line that doubles as a reset line.

The BOOT button is connected to IO0.  On reset, it is used as a strapping
pin to determine whether the chip boots normally or into the serial
bootloader.  After resetting, however, the BOOT button can be used for
software input.

LEDs
----

There are two on-board LEDs. RED_LED (D5) indicates the presence of 3.3V
power and is not controlled by software. RGB LED (U6) is a WS2812 addressable
LED and is driven by GPIO18.

I2S
===

ESP32-S2 has an I2S peripheral accessible using either the generic I2S audio
driver or a specific audio codec driver
(`CS4344 <https://www.cirrus.com/products/cs4344-45-48/>`__ bindings are
available at the moment). Also, it's possible to use the I2S character device
driver to bypass audio systems and write directly to the I2S peripheral.

.. note:: The I2S peripheral is able to work on two functional modes
  internally: 16 and 32-bit width.
  That limits using the I2S peripheral to play audio files other than 16/32
  bit-widths as the internal buffer allocated for the audio content does not
  consider the operation modes of the peripheral. This limitation is planned
  to be removed soon by copying the buffers internally and making the
  necessary adjustments.

.. note:: The above statement is not valid when using the I2S character
  device driver.
  It's possible to use 8, 16, 24, and 32-bit-widths writing directly to the
  I2S character device. Just make sure to set the bit-width::

    $ make menuconfig
    -> System Type
        -> ESP32-S2 Peripheral Selection
            -> I2S
                -> I2S0/1
                    -> Bit Witdh

  And make sure the data stream buffer being written to the I2S peripheral is
  aligned to the next boundary i.e. 16 bits for the 8 and 16-bit-widths and
  32 bits for 24 and 32-bit-widths.

Configurations
==============

audio
-----

This configuration uses the I2S0 peripheral and an externally connected audio
codec to play an audio file. The easiest way of playing an uncompressed file
is embedding into the firmware. This configuration selects
`romfs example <https://github.com/apache/incubator-nuttx-apps/tree/master/examples/romfs>`_`
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

Prepare and build the `audio` defconfig::

  $ make -j distclean && ./tools/configure.sh esp32s2-saola-1:audio && make

This will create a temporary folder in `apps/examples/romfs/testdir`. Move
a PCM-encoded (`.wav`) audio file with 16 bits/sample (sampled at 8~48kHz)
to this folder.

.. note:: You can use :download:`this 440 Hz sinusoidal tone <tone.wav>`.
   The audio file should be located at `apps/examples/romfs/testdir/tone.wav`

Build the project again and flash it (make sure not to clean it, just build)

After successfully built and flashed, load the romfs and play it::

  $ nsh> romfs
  $ nsh> nxplayer
  $ nxplayer> play /usr/share/local/tone.wav

i2schar
-------

This configuration enables the I2S character device and the i2schar example
app, which provides an easy-to-use way of testing the I2S peripheral.

After successfully built and flashed, run on the board's terminal::

  $ i2schar

The corresponding output should show related debug information.

nsh
---

Basic NuttShell configuration (console enabled in UART0, exposed via
USB connection by means of CP2102 converter, at 115200 bps).

timer
-----

This config tests the general-use purpose timers. It includes the 4 timers,
adds driver support, registers the timers as devices and includes the timer
example.

To test it, just run the following::

  nsh> timer -d /dev/timerx

Where x in the timer instance.

watchdog
--------

This config test the watchdog timers. It includes the 2 MWDTs,
adds driver support, registers the WDTs as devices and includes the watchdog
example.

To test it, just run the following::

  nsh> wdog -d /dev/watchdogx

Where x in the watchdog instance.
