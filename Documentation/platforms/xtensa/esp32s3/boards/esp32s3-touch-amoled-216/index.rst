====================================
Waveshare ESP32-S3-Touch-AMOLED-2.16
====================================

.. tags:: arch:xtensa, chip:esp32s3, vendor:espressif

This page discusses issues unique to NuttX configurations for the
Waveshare ESP32-S3-Touch-AMOLED-2.16 board.

.. figure:: esp32s3-touch-amoled-216.jpg
   :align: center

   ESP32-S3-Touch-AMOLED-2.16 (image: Waveshare, Apache-2.0)

The ESP32-S3-Touch-AMOLED-2.16 is a compact development board with a
2.16 inch square AMOLED touch display, power management with a Li-ion
battery charger, motion sensing, a real time clock, audio input and output
and a microSD card slot.

Features
========

  - ESP32-S3R8, dual core Xtensa LX7 at up to 240 MHz, with 8MB octal PSRAM
  - 16MB quad SPI flash
  - 2.16 inch 480x480 AMOLED, CO5300 controller on a QSPI bus
  - CST9220 capacitive touch controller
  - AXP2101 power management IC with Li-ion battery charger
  - QMI8658 6-axis IMU (accelerometer and gyroscope)
  - PCF85063 real time clock with backup supply
  - ES8311 audio codec driving a speaker amplifier
  - ES7210 four channel audio ADC with two microphones
  - microSD card slot
  - BOOT, PWR and one user (IO18) key
  - USB Type-C connected to the ESP32-S3 USB Serial/JTAG controller

Board documentation: https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-2.16

Schematic and examples: https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-2.16

Serial Console
==============

The board has no USB to UART bridge.  The USB Type-C connector goes to the
ESP32-S3 USB Serial/JTAG controller, which is used both to flash the board
and as the NuttShell console (``/dev/console``, enumerated by the host as
``/dev/ttyACM0`` on Linux).  UART0 (GPIO43 TX, GPIO44 RX) is available on
the side header.

Board Buttons
=============

The board has three keys:

  ====== ======== ====================================================
  Key    GPIO     Description
  ====== ======== ====================================================
  BOOT   GPIO0    Active low. Held at reset, enters the ROM bootloader
  IO18   GPIO18   Active low, free for the application
  PWR    GPIO16   AXP2101 PWRON key, mirrored to GPIO16 (active high)
  ====== ======== ====================================================

The PWR key is wired to the PWRON pin of the AXP2101.  A MOSFET mirrors
its level to GPIO16, so the application can see it as a normal key, but
the AXP2101 still acts on it: a long press powers the board off and a
short press powers it back on.

I2C Bus
=======

All of the on-board peripherals except the display and the SD card share
one I2C bus, I2C0, on GPIO15 (SDA) and GPIO14 (SCL):

  ======== ======== ===============================
  Address  Device   Function
  ======== ======== ===============================
  0x18     ES8311   Audio codec (playback)
  0x34     AXP2101  Power management
  0x40     ES7210   Audio ADC (microphones)
  0x51     PCF85063 Real time clock
  0x5A     CST9220  Touch controller
  0x6B     QMI8658  IMU
  ======== ======== ===============================

Pin Mapping
===========

  ========== ==================================================
  GPIO       Function
  ========== ==================================================
  GPIO0      BOOT key
  GPIO1      microSD MOSI
  GPIO2      microSD SCLK
  GPIO3      microSD MISO
  GPIO4-7    AMOLED QSPI data 0-3
  GPIO8      I2S data out (to ES8311)
  GPIO9      I2S bit clock
  GPIO10     I2S data in (from ES7210)
  GPIO11     Touch interrupt
  GPIO12     AMOLED QSPI chip select
  GPIO13     RTC interrupt
  GPIO14     I2C SCL
  GPIO15     I2C SDA
  GPIO16     PWR key (SYS_OUT)
  GPIO17     IMU interrupt 1
  GPIO18     IO18 key
  GPIO19/20  USB D-/D+
  GPIO21     IMU interrupt 2
  GPIO38     AMOLED QSPI clock
  GPIO39     AMOLED reset
  GPIO40     Touch reset
  GPIO41     microSD chip select
  GPIO42     I2S master clock
  GPIO43/44  UART0 TX/RX
  GPIO45     I2S word select
  GPIO46     Speaker amplifier enable
  ========== ==================================================

Configurations
==============

All of the configurations presented below can be tested by running the
following commands::

    $ ./tools/configure.sh esp32s3-touch-amoled-216:<config_name>
    $ make flash ESPTOOL_PORT=/dev/ttyACM0 -j

Where ``<config_name>`` is the name of the board configuration you want to
use, e.g. ``nsh``.  To access the console, open ``/dev/ttyACM0`` with a
serial terminal such as ``picocom`` or ``minicom``.

buttons
-------

This configuration shows the use of the buttons subsystem.  It can be used
by executing the ``buttons`` application and pressing the BOOT, IO18 or
PWR key::

    nsh> buttons
    buttons_main: Starting the button_daemon
    buttons_main: button_daemon started
    button_daemon: Running
    button_daemon: Opening /dev/buttons
    button_daemon: Supported BUTTONs 0x07
    BOOT was pressed
    BOOT was released
    IO18 was pressed
    IO18 was released
    PWR was pressed
    PWR was released

i2c
---

This configuration enables the I2C bus driver and the ``i2c`` tool, which
can be used to inspect the on-board peripherals::

    nsh> i2c dev 0x08 0x77
         0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
    00:                         -- -- -- -- -- -- -- --
    10: -- -- -- -- -- -- -- -- 18 -- -- -- -- -- -- --
    20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    30: -- -- -- -- 34 -- -- -- -- -- -- -- -- -- -- --
    40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    50: -- 51 -- -- -- -- -- -- -- -- 5a -- -- -- -- --
    60: -- -- -- -- -- -- -- -- -- -- -- 6b -- -- -- --
    70: -- -- -- -- -- -- -- --

nsh
---

Basic NuttShell configuration (console enabled on the USB Serial/JTAG
controller).

qmi8658
-------

This configuration registers the QMI8658 6-axis IMU through the uORB
sensor framework, as ``/dev/uorb/sensor_accel0`` and
``/dev/uorb/sensor_gyro0``.  The ``uorb_listener`` application can be used
to read it.  With the board lying flat, the Z axis of the accelerometer
reads about 1 g::

    nsh> uorb_listener -n 3 sensor_accel0

    Monitor objects num:1
    object_name:sensor_accel, object_instance:0
    sensor_accel(now:6650000):timestamp:6650000,x:0.024414,y:0.012695,z:0.995728,temperature:35.105469
    sensor_accel(now:7660000):timestamp:7660000,x:0.021851,y:0.016724,z:0.998169,temperature:35.089844
    sensor_accel(now:8670000):timestamp:8670000,x:0.022583,y:0.017944,z:0.998291,temperature:35.062500
    Object name:sensor_accel0, received:3
    Total number of received Message:3/3

psram
-----

This configuration enables the 8MB octal PSRAM and adds it to the heap.
The ``free`` command shows the extra memory, and the ``heap`` test can be
used to exercise it::

    nsh> free
          total       used       free    maxused    maxfree  nused  nfree name
        8775692      23020    8752672      23400    8388592     75      4 Umem

sdmmc
-----

This configuration binds the microSD card slot, on SPI3 (GPIO2 SCLK, GPIO1
MOSI, GPIO3 MISO, GPIO41 CS), to ``/dev/mmcsd0`` and enables the FAT file
system.  The slot has no card detect switch, so the card must be inserted
before the board boots::

    nsh> mount -t vfat /dev/mmcsd0 /mnt
    nsh> echo hello-from-nuttx > /mnt/nxtest.txt
    nsh> cat /mnt/nxtest.txt
    hello-from-nuttx
    nsh> dd if=/dev/zero of=/mnt/big.bin bs=4096 count=256
    1048576 bytes (256 blocks) copied, 800000 usec, 1280 KB/s
    nsh> df -h
      Filesystem      Size      Used  Available Mounted on
      vfat           3580M     1040K      3579M /mnt

wifi
----

Enables Wi-Fi support (station mode) with the ``wapi`` tool.  You can scan
for networks and connect to an access point with::

    nsh> wapi scan wlan0
    nsh> wapi psk wlan0 mypasswd 3
    nsh> wapi essid wlan0 myssid 1
    nsh> renew wlan0

``wapi save_config wlan0`` stores the current settings on the SPI flash,
where ``CONFIG_WIRELESS_WAPI_INITCONF`` picks them up at boot.
