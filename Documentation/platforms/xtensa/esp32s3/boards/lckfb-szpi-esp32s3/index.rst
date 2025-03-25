===================
LCKFB SZPI ESP32-S3
===================

The `LCKFB SZPI ESP32-S3 <https://wiki.lckfb.com/zh-hans/szpi-esp32s3/>`_ is a development board for the ESP32-S3 SoC from Jialichuang, based on a ESP32-S3-WROOM-1 module.

.. list-table::
   :align: center

   * - .. figure:: lckfb-szpi-esp32s3-white.png
          :align: center

Features
========

The development board almost includes all the achievable functions of the ESP32-S3, maximizing the performance of the ESP32-S3. It features a color screen display (2 inches), complete audio input (2 microphones) and output (speaker) functions. Combined with its integrated motion sensors, as well as Wi-Fi and Bluetooth capabilities, it can be used to create more practical and interesting IoT applications. There are two expansion interfaces available for connecting additional external sensor modules and actuators. The development board is compact in size (69x41x14mm), and the shell is designed without screws, allowing for easy opening and installation by hand without the need for tools such as a screwdriver.

Serial Console
==============

UART0 is, by default, the serial console.  It connects to the on-board
CP2102 converter and is available on the USB connector USB CON8 (J1).

It will show up as /dev/ttyUSB[n] where [n] will probably be 0.

Buttons
================

Board Buttons
-------------

There are two buttons labeled Boot and EN.  The EN button is not available
to software.  It pulls the chip enable line that doubles as a reset line.

The BOOT button is connected to IO0.  On reset it is used as a strapping
pin to determine whether the chip boots normally or into the serial
bootloader.  After reset, however, the BOOT button can be used for software
input.

I2S
===

ESP32-S3 has two I2S peripherals accessible using either the generic I2S audio
driver or a specific audio codec driver
(`CS4344 <https://www.cirrus.com/products/cs4344-45-48/>`__ bindings are
available at the moment). The generic I2S audio driver enables the use of both
the receiver module (RX) and the transmitter module (TX) without using any
specific codec. Also, it's possible to use the I2S character device driver
to bypass the audio subsystem and write directly to the I2S peripheral.

The following configurations use the I2S peripheral::
  * :ref:`platforms/xtensa/esp32s3/boards/esp32s3-devkit/index:audio`
  * :ref:`platforms/xtensa/esp32s3/boards/esp32s3-devkit/index:nxlooper`

Configurations
==============

All of the configurations presented below can be tested by running the following commands::

    $ ./tools/configure.sh lckfb-szpi-esp32s3:<config_name>
    $ make flash ESPTOOL_PORT=/dev/ttyUSB0 -j

Where <config_name> is the name of board configuration you want to use, i.e.: nsh, buttons, wifi...
Then use a serial console terminal like ``picocom`` configured to 115200 8N1.

nsh
---

Basic NuttShell configuration (console enabled in UART0, exposed via
USB connection by means of CP2102 converter, at 115200 bps).

usb_device
----------

Basic NuttShell configuration console and USB CDCACM enabled.

You can run the configuration and compilation procedure::

  $ ./tools/configure.sh lckfb-szpi-esp32s3:usb_device
  $ make -j16
  $ make flash ESPTOOL_PORT=/dev/ttyUSBx

And then run the usbserial command at device side::

  nsh> usbserial &

Finally check dmesg and content at host side::

  $ sudo dmesg -c
  [1768234.376415] usb 1-9.3.3: new full-speed USB device number 87 using xhci_hcd
  [1768234.468015] usb 1-9.3.3: New USB device found, idVendor=0525, idProduct=a4a7, bcdDevice= 1.01
  [1768234.468020] usb 1-9.3.3: New USB device strings: Mfr=1, Product=2, SerialNumber=3
  [1768234.468021] usb 1-9.3.3: Product: CDC/ACM Serial
  [1768234.468022] usb 1-9.3.3: Manufacturer: NuttX
  [1768234.468023] usb 1-9.3.3: SerialNumber: 0
  [1768234.478806] cdc_acm 1-9.3.3:1.0: ttyACM5: USB ACM device

  $ sudo minicom -D /dev/ttyACM5 -b 115200

adb
---

Basic NuttShell configuration console enabled over USB Device (USB ADB).

You can run the configuration and compilation procedure::

  $ ./tools/configure.sh lckfb-szpi-esp32s3:adb
  $ make -j16
  $ make flash ESPTOOL_PORT=/dev/ttyUSBx

Then run the adb command::

  $ adb -s 1234 shell
  nsh> uname -a
  NuttX 0.0.0  Mar 21 2025 14:25:36 xtensa lckfb-szpi-esp32s3

txtable
-------

Basic TXTABLE(Text based Partition Table) configuration console enabled over USB ADB.

You can run the configuration and compilation procedure::

  $ ./tools/configure.sh -l lckfb-szpi-esp32s3:txtable
  $ make -j16
  $ make flash ESPTOOL_PORT=/dev/ttyUSBx

Then check the partition::

  nsh> ls -l /dev/
  /dev:
   dr--r--r--           0 adb0/
   crw-rw-rw-           0 console
   frw-rw-rw-     1044480 data
   frw-rw-rw-     1048576 esp32s3flash
   c-w--w--w-           0 log
   crw-rw-rw-           0 null
   crw-rw-rw-           0 ptmx
   dr--r--r--           0 pts/
   brw-rw-rw-        1024 ram0
   crw-rw-rw-           0 ttyS0
   frw-rw-rw-        4096 txtable
   crw-rw-rw-           0 zero

fastboot
--------

The basic Fastboot configuration is based on lckfb-szpi-esp32s3:usb_device.
More details about usage of fastboot, please refer to `fastbootd — NuttX latest documentation <https://nuttx.apache.org/docs/latest/applications/system/fastboot/index.html>`_.

You can run the configuration and compilation procedure::

  $ ./tools/configure.sh -l lckfb-szpi-esp32s3:fastboot
  $ make flash ESPTOOL_PORT=/dev/ttyUSBx -j

To test it, just run the following (**Default is host side**):

1. Install fastboot tool::

    sudo apt install fastboot

2. List devices running fastboot::

    fastboot devices

  Example::

    $ fastboot devices
    1234    fastboot

3. Display given variable::

    fastboot getvar <NAME>

  Example::

    # Display the "kernel" variable::
    $ fastboot -s 1234 getvar kernel
    Kernel: NuttX
    Finished. Total time: 0.000s

4. Flash given partition::

    fastboot flash PARTITION FILENAME

  Example (Flash test.img to partition ram10)::

    # 1. Generate a test image
    $ dd if=/dev/random of=test.img bs=1 count=128

    # 2. Create a RAM disk (Device side)
    nsh> mkrd -m 10 -s 512 640
    nsh> ls -l /dev/ram10
     brw-rw-rw-      327680 /dev/ram10

    # 3. Flash test.img to partition ram10
    $ fastboot flash ram10 ./test.img
    Sending 'ram10' (0 KB)                             OKAY [  0.001s]
    Writing 'ram10'                                    OKAY [  0.001s]
    Finished. Total time: 0.003s

    # 4. Hexdump the test.img and partition ram10, and compare

    ## Host side
    $ hexdump test.img
    0000000 b1e8 b297 4ac5 9dfa d170 244e 4f83 0f93
    0000010 1bf7 0b19 7bde 5543 0520 9719 746d 54fc
    0000020 369d 72b3 f2e6 f463 c8e9 24c8 c876 e820
    0000030 384d 07ab 52ca 2b24 dee7 0404 2663 91e4
    0000040 6752 3611 aece b543 5194 2224 d1d5 8144
    0000050 ff44 3bc9 5155 b393 1efb 9e88 2de9 3669
    0000060 d010 2770 9192 2532 ccf5 591f 39ea 2431
    0000070 2e3f feb0 87ef 9bdf 7dd4 2e79 64de edf6
    0000080

    ## Device side
    nsh> hexdump /dev/ram10 count=128
    /dev/ram10 at 00000000:
    0000: e8 b1 97 b2 c5 4a fa 9d 70 d1 4e 24 83 4f 93 0f .....J..p.N$.O..
    0010: f7 1b 19 0b de 7b 43 55 20 05 19 97 6d 74 fc 54 .....{CU ...mt.T
    0020: 9d 36 b3 72 e6 f2 63 f4 e9 c8 c8 24 76 c8 20 e8 .6.r..c....$v. .
    0030: 4d 38 ab 07 ca 52 24 2b e7 de 04 04 63 26 e4 91 M8...R$+....c&..
    0040: 52 67 11 36 ce ae 43 b5 94 51 24 22 d5 d1 44 81 Rg.6..C..Q$"..D.
    0050: 44 ff c9 3b 55 51 93 b3 fb 1e 88 9e e9 2d 69 36 D..;UQ.......-i6
    0060: 10 d0 70 27 92 91 32 25 f5 cc 1f 59 ea 39 31 24 ..p'..2%...Y.91$
    0070: 3f 2e b0 fe ef 87 df 9b d4 7d 79 2e de 64 f6 ed ?........}y..d..

pca9557
-------

Basic NuttShell configuration console and I/O expander driver PCA9557 enabled.

You can run the configuration and compilation procedure::

  $ ./tools/configure.sh lckfb-szpi-esp32s3:pca9557
  $ make flash -j$(nproc) ESPTOOL_PORT=/dev/ttyUSB0

Then test gpio2(pin9(P2) of PCA9557)::

  # With hardware check, the pin levels meet the expected requirements.

  # Output low
  nsh> echo 0 > /dev/gpio2
  nsh> cat /dev/gpio2
  0

  # Output high
  nsh> echo 1 > /dev/gpio2
  nsh> cat /dev/gpio2
  1
