..
    Ported from ESP-ADF documentation https://espressif-docs.readthedocs-hosted.com/projects/esp-adf/en/latest/design-guide/dev-boards/board-esp32-lyrat-v4.3.html

================
ESP32-LyraT V4.3
================

The ESP32-LyraT development board is a hardware platform designed for the
dual-core ESP32 audio applications, e.g., Wi-Fi or BT audio speakers,
speech-based remote controllers, smart-home appliances with audio
functionality(ies), etc. You can find the board schematic `here <https://dl.espressif.com/dl/schematics/ESP32-LYRAT_V4.3-20220119.pdf>`_.

.. figure:: esp32-lyrat-v4.3-layout-with-wrover-e-module.jpg
    :alt: ESP32 LyraT V4.3 Board Layout
    :figclass: align-center

    ESP32-LyraT V4.3 Board Layout

The block diagram below presents main components of the ESP32-LyraT.

.. figure:: esp32-lyrat-v4.3-electrical-block-diagram-with-wrover-e-module.png
    :alt: ESP32 LyraT V4.3 Electrical Block Diagram
    :figclass: align-center

    ESP32-LyraT V4.3 Electrical Block Diagram

Features
========

    - ESP32-WROVER-E Module
    - JTAG Interface
    - MicroSD Slot
    - Audio Codec Chip
    - Battery Charger Chip
    - Touch Pad Buttons

Serial Console
==============

UART0 is, by default, the serial console. It connects to the on-board
CP2102N bridge and is available on the USB connector.

It will show up as /dev/ttyUSB[n] where [n] will probably be 0.

Buttons and LEDs
================

Buttons
-------

Two key labeled *Rec* and *Mode*. They are routed to **ESP32-WROVER-E Module**
and intended for developing and testing a UI for audio applications using
dedicated API.

Four touch pads labeled *Play*, *Sel*,  *Vol+* and *Vol-*.
They are routed to **ESP32-WROVER-E Module** and intended for development and
testing of a UI for audio applications using dedicated API. **They are currently
not supported as the driver is in development.**

There are two buttons labeled Boot and EN. The EN button pulls the chip enable
line that doubles as a reset line. The BOOT button is connected to IO0. On
reset it is used as a strapping pin to determine whether the chip boots
normally or into the serial bootloader.

Entering of the ESP32 into upload mode may be done in two ways:

* Manually by pressing both **Boot** and **RST** keys and then releasing first
  **RST** and then **Boot** key.
* Automatically by software performing the upload. The software is using
  **DTR** and **RTS** signals of the serial interface to control states of
  **EN**, **IO0** and **IO2** pins of the ESP32. This functionality is enabled
  by installing jumpers in three headers **JP23**, **JP24** and **JP25**.
  Remove all jumpers after upload is complete.

LEDs
----

A general purpose green LED controlled by the **ESP32-WROVER-E Module** to
indicate certain operation states of the audio application using dedicated
API. It can also be used by the user for other purposes.

The **Standby** green LED indicates that power has been applied to the
**Micro USB Port**. The **Charging** red LED indicates that a battery
connected to the **Battery Socket** is being charged.

Audio Codec
===========

**This is currently unsupported. Drivers still in development.**

The Audio Codec Chip, `ES8388 <http://www.everest-semi.com/pdf/ES8388%20DS.pdf>`_,
is a low power stereo audio codec with a headphone amplifier. It consists
of 2-channel ADC, 2-channel DAC, microphone amplifier, headphone amplifier
, digital sound effects, analog mixing and gain functions. It is
interfaced with **ESP32-WROVER-E Module** over I2S and I2S buses to
provide audio processing in hardware independently from the audio
application.

It also provides:

    - Onboard microphone connected to IN1 of the **Audio Codec Chip**.
    - Auxiliary input socket connected to IN2 (left and right channel) of the
      **Audio Codec Chip**. Use a 3.5 mm stereo jack to connect to this socket.
    - Output socket to connect headphones with a 3.5 mm stereo jack.

    .. note::
        The socket may be used with mobile phone headsets and is compatible
        with OMPT standard headsets only. It does work with CTIA headsets.
        Please refer to `Phone connector (audio) <https://en.wikipedia.org/wiki/Phone_connector_(audio)#TRRS_standards>`_
        on Wikipedia.

    - Output socket to connect a speaker. The 4-ohm and 3-watt speaker is
      recommended. The pins have a 2.00 mm / 0.08" pitch.

The development board uses two mono Class D amplifier ICs, model number NS4150
with maximum output power of 3W and operating voltage from 3.0V to 5.25V.
The audio input source is the digital-to-analog converter (DAC) output of the
ES8388. Audio output supports two external speakers.
An optional audio output is a pair of headphones feed from the same DACs as
the amplifier ICs.

To switch between using headphones and speakers, the board provides a digital
input signal to detect when a headphone jack is inserted and a digital output
signal to enable or disable the amplifier ICs. In other words selection
between speakers and headphones is under software control instead of using
mechanical contacts that would disconnect speakers once a headphone jack is
inserted.

SD card
=======

The development board supports a MicroSD card in SPI/1-bit/4-bit modes,
and can store or play audio files in the MicroSD card. Note that **JTAG**
cannot be used and should be disconnected by setting **Function DIP
Switch** when **MicroSD Card** is in operation, because some of signals
are shared by both devices.

Enable MicroSD Card in 1-wire Mode
""""""""""""""""""""""""""""""""""

Set **Function DIP Switch** to:

+---------+-----------------+
|  DIP SW | Position        |
+=========+=================+
|    1    |    OFF          |
+---------+-----------------+
|    2    |    OFF          |
+---------+-----------------+
|    3    |    OFF          |
+---------+-----------------+
|    4    |    OFF          |
+---------+-----------------+
|    5    |    OFF          |
+---------+-----------------+
|    6    |    OFF          |
+---------+-----------------+
|    7    |    OFF :sup:`1` |
+---------+-----------------+
|    8    |    n/a          |
+---------+-----------------+

1. **AUX Input** detection may be enabled by toggling the DIP SW 7 *ON*.
   Note that the **AUX Input** signal pin should not be be plugged in when the
   system powers up. Otherwise the ESP32 may not be able to boot correctly.

In this mode:

* **JTAG** functionality is not available
* *Vol-* touch button is available for use with the API

Enable MicroSD Card in 4-wire Mode
""""""""""""""""""""""""""""""""""

Set **Function DIP Switch** to:

+---------+-----------+
|  DIP SW | Position  |
+=========+===========+
|    1    |    ON     |
+---------+-----------+
|    2    |    ON     |
+---------+-----------+
|    3    |    OFF    |
+---------+-----------+
|    4    |    OFF    |
+---------+-----------+
|    5    |    OFF    |
+---------+-----------+
|    6    |    OFF    |
+---------+-----------+
|    7    |    OFF    |
+---------+-----------+
|    8    |    n/a    |
+---------+-----------+

In this mode:

* **JTAG** functionality is not available
* *Vol-* touch button is not available for use with the API
* **AUX Input** detection from the API is not available

JTAG
====

Provides access to the **JTAG** interface of **ESP32-WROVER-E Module**.
It may be used for debugging, application upload, as well as implementing
several other functions.

Enable JTAG
"""""""""""

Set **Function DIP Switch** to:

+---------+-----------+
|  DIP SW | Position  |
+=========+===========+
|    1    |    OFF    |
+---------+-----------+
|    2    |    OFF    |
+---------+-----------+
|    3    |    ON     |
+---------+-----------+
|    4    |    ON     |
+---------+-----------+
|    5    |    ON     |
+---------+-----------+
|    6    |    ON     |
+---------+-----------+
|    7    |    ON     |
+---------+-----------+
|    8    |    n/a    |
+---------+-----------+

In this mode:

* **MicroSD Card** functionality is not available, remove the card from the slot
* *Vol-* touch button is not available for use with the API
* **AUX Input** detection from the API is not available

Battery
=======

The board has a constant current & constant voltage linear charger for single cell
lithium-ion batteries AP5056. Used for charging of a battery connected to
the **Battery Socket** over the **Micro USB Port**.

.. note::
    Please verify if polarity on the battery plug matches polarity of the
    socket as marked on the board's soldermask besides the socket.

.. note::
    The **Power On Switch** does not affect/disconnect the Li-ion
    battery charging.

Pin Mapping
===========

Several pins ESP32 module are allocated to the on board hardware. Some of
them, like GPIO0 or GPIO2, have multiple functions. Please refer to the table
below.

+-----------+------+-------------------------------------------------------+
| GPIO Pin  | Type | Function Definition                                   |
+===========+======+=======================================================+
| SENSOR_VP | I    | Audio **Rec** (PB)                                    |
+-----------+------+-------------------------------------------------------+
| SENSOR_VN | I    | Audio **Mode** (PB)                                   |
+-----------+------+-------------------------------------------------------+
| IO32      | I/O  | Audio **Set** (TP)                                    |
+-----------+------+-------------------------------------------------------+
| IO33      | I/O  | Audio **Play** (TP)                                   |
+-----------+------+-------------------------------------------------------+
| IO27      | I/O  | Audio **Vol+** (TP)                                   |
+-----------+------+-------------------------------------------------------+
| IO13      | I/O  | JTAG **MTCK**, MicroSD **D3**, Audio **Vol-** (TP)    |
+-----------+------+-------------------------------------------------------+
| IO14      | I/O  | JTAG **MTMS**, MicroSD **CLK**                        |
+-----------+------+-------------------------------------------------------+
| IO12      | I/O  | JTAG **MTDI**, MicroSD **D2**, Aux signal **detect**  |
+-----------+------+-------------------------------------------------------+
| IO15      | I/O  | JTAG **MTDO**, MicroSD **CMD**                        |
+-----------+------+-------------------------------------------------------+
| IO2       | I/O  | Automatic Upload, MicroSD **D0**                      |
+-----------+------+-------------------------------------------------------+
| IO4       | I/O  | MicroSD **D1**                                        |
+-----------+------+-------------------------------------------------------+
| IO34      | I    | MicroSD insert **detect**                             |
+-----------+------+-------------------------------------------------------+
| IO0       | I/O  | Automatic Upload, I2S **MCLK**                        |
+-----------+------+-------------------------------------------------------+
| IO5       | I/O  | I2S **SCLK**                                          |
+-----------+------+-------------------------------------------------------+
| IO25      | I/O  | I2S **LRCK**                                          |
+-----------+------+-------------------------------------------------------+
| IO26      | I/O  | I2S **DSDIN**                                         |
+-----------+------+-------------------------------------------------------+
| IO35      | I    | I2S **ASDOUT**                                        |
+-----------+------+-------------------------------------------------------+
| IO19      | I/O  | Headphone jack insert **detect**                      |
+-----------+------+-------------------------------------------------------+
| IO22      | I/O  | Green LED indicator                                   |
+-----------+------+-------------------------------------------------------+
| IO21      | I/O  | PA Enable output                                      |
+-----------+------+-------------------------------------------------------+
| IO18      | I/O  | I2C **SDA**                                           |
+-----------+------+-------------------------------------------------------+
| IO23      | I/O  | I2C **SCL**                                           |
+-----------+------+-------------------------------------------------------+

* (TP) - touch pad
* (PB) - push button

There are several pin headers available to connect external components, check
the state of particular signal bus or debug operation of ESP32. Note that some
signals are shared.

UART Header / JP2
"""""""""""""""""

+---+-------------+
|   | Header Pin  |
+===+=============+
| 1 | 3.3V        |
+---+-------------+
| 2 | TX          |
+---+-------------+
| 3 | RX          |
+---+-------------+
| 4 | GND         |
+---+-------------+


I2S Header / JP4
""""""""""""""""

+---+----------------+-------------+
|   | I2C Header Pin | ESP32 Pin   |
+===+================+=============+
| 1 | MCLK           | GPIO0       |
+---+----------------+-------------+
| 2 | SCLK           | GPIO5       |
+---+----------------+-------------+
| 1 | LRCK           | GPIO25      |
+---+----------------+-------------+
| 2 | DSDIN          | GPIO26      |
+---+----------------+-------------+
| 3 | ASDOUT         | GPIO35      |
+---+----------------+-------------+
| 3 | GND            | GND         |
+---+----------------+-------------+


I2C Header / JP5
""""""""""""""""

+---+----------------+-------------+
|   | I2C Header Pin | ESP32 Pin   |
+===+================+=============+
| 1 | SCL            | GPIO23      |
+---+----------------+-------------+
| 2 | SDA            | GPIO18      |
+---+----------------+-------------+
| 3 | GND            | GND         |
+---+----------------+-------------+


JTAG Header / JP7
"""""""""""""""""

+---+---------------+-------------+
|   | ESP32 Pin     | JTAG Signal |
+===+===============+=============+
| 1 | MTDO / GPIO15 | TDO         |
+---+---------------+-------------+
| 2 | MTCK / GPIO13 | TCK         |
+---+---------------+-------------+
| 3 | MTDI / GPIO12 | TDI         |
+---+---------------+-------------+
| 4 | MTMS / GPIO14 | TMS         |
+---+---------------+-------------+

.. note::
    **JTAG** cannot be used if **MicroSD Card** is enabled.

Configurations
==============

nsh
---

Basic NuttShell configuration (console enabled in UART0, exposed via
USB connection by means of the CP2102N bridge, at 115200 bps).

wapi
----

Enables Wi-Fi support.