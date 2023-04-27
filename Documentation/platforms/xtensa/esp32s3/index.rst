==================
Espressif ESP32-S3
==================

The ESP32-S3 is a series of single and dual-core SoCs from Espressif
based on Harvard architecture Xtensa LX7 CPUs and with on-chip support
for Bluetooth and Wi-Fi.

All embedded memory, external memory and peripherals are located on the
data bus and/or the instruction bus of these CPUs. With some minor
exceptions, the address mapping of two CPUs is symmetric, meaning they
use the same addresses to access the same memory. Multiple peripherals in
the system can access embedded memory via DMA.

On dual-core SoCs, the two CPUs are typically named "PRO_CPU" and "APP_CPU"
(for "protocol" and "application"), however for most purposes the
two CPUs are interchangeable.

Toolchain
=========

You can use the prebuilt `toolchain <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/tools/idf-tools.html#xtensa-esp32s3-elf>`__
for Xtensa architecture and `OpenOCD <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/tools/idf-tools.html#openocd-esp32>`__
for ESP32-S3 by Espressif.

For flashing firmware, you will need to install ``esptool.py`` by running::

    $ pip install esptool

Building from source
--------------------

You can also build the toolchain yourself. The steps to
build the toolchain with crosstool-NG on Linux are as follows

.. code-block:: console

  $ git clone https://github.com/espressif/crosstool-NG.git
  $ cd crosstool-NG
  $ git submodule update --init

  $ ./bootstrap && ./configure --enable-local && make

  $ ./ct-ng xtensa-esp32s3-elf
  $ ./ct-ng build

  $ chmod -R u+w builds/xtensa-esp32s3-elf

  $ export PATH="crosstool-NG/builds/xtensa-esp32-elf/bin:$PATH"

These steps are given in the setup guide in
`ESP-IDF documentation <https://docs.espressif.com/projects/esp-idf/en/latest/get-started/linux-setup-scratch.html>`_.

Flashing
========

Firmware for ESP32-S3 is flashed via the USB/UART or internal USB DEVICE JTAG interface using the
``esptool.py`` tool.
It's a two step process where the first converts the ELF file into a ESP32-S3 compatible binary
and the second flashes it to the board.  These steps are included into the build system and you can
flash your NuttX firmware simply by running::

    $ make flash ESPTOOL_PORT=<port>

where ``<port>`` is typically ``/dev/ttyUSB0`` or similar. You can change the baudrate by passing ``ESPTOOL_BAUD``.

Bootloader and partitions
-------------------------

ESP32-S3 requires a bootloader to be flashed as well as a set of FLASH partitions. This is only needed the first time
(or any time you which to modify either of these). An easy way is to use prebuilt binaries for NuttX `from here <https://github.com/espressif/esp-nuttx-bootloader>`_. In there you will find instructions to rebuild these if necessary.
Once you downloaded both binaries, you can flash them by adding an ``ESPTOOL_BINDIR`` parameter, pointing to the directory where these binaries were downloaded:

.. code-block:: console

   $ make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=<dir>

.. note:: It is recommended that if this is the first time you are using the board with NuttX that you perform a complete SPI FLASH erase.

   .. code-block:: console

      $ esptool.py erase_flash

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX:

========== ======= =====
Peripheral Support NOTES
========== ======= =====
ADC          No
AES          No
Bluetooth    No
CAMERA       No
CAN/TWAI     No
DMA          Yes
eFuse        No
GPIO         Yes
I2C          No
I2S          No
LCD          No
LED_PWM      No
MCPWM        No
Pulse_CNT    No
RMT          No
RNG          No
RSA          No
RTC          No
SD/MMC       No
SDIO         No
SHA          No
SPI          Yes
SPIFLASH     Yes
SPIRAM       Yes
Timers       Yes
Touch        Yes
UART         Yes
USB OTG      No
USB SERIAL   Yes
Watchdog     Yes
Wi-Fi        Yes   WPA3-SAE supported
========== ======= =====

.. _esp32s3_peripheral_support:

Wi-Fi
-----

.. tip:: Boards usually expose a ``wifi`` defconfig which enables Wi-Fi

A standard network interface will be configured and can be initialized such as::

    nsh> ifup wlan0
    nsh> wapi psk wlan0 mypasswd 3
    nsh> wapi essid wlan0 myssid 1
    nsh> renew wlan0

In this case a connection to AP with SSID ``myssid`` is done, using ``mypasswd`` as
password. IP address is obtained via DHCP using ``renew`` command. You can check
the result by running ``ifconfig`` afterwards.

.. tip:: Please refer to :ref:`ESP32 Wi-Fi Station Mode <esp32_wi-fi_sta>`
  for more information.

Wi-Fi SoftAP
------------

It is possible to use ESP32 as an Access Point (SoftAP).

.. tip:: Boards usually expose a ``sta_softap`` defconfig which enables Wi-Fi
   (STA + SoftAP)

If you are using this board config profile you can run these commands to be able
to connect your smartphone or laptop to your board::

    nsh> ifup wlan1
    nsh> dhcpd_start wlan1
    nsh> wapi psk wlan1 mypasswd 3
    nsh> wapi essid wlan1 nuttxap 1

In this case, you are creating the access point ``nuttxapp`` in your board and to
connect to it on your smartphone you will be required to type the password ``mypasswd``
using WPA2.

.. tip:: Please refer to :ref:`ESP32 Wi-Fi SoftAP Mode <esp32_wi-fi_softap>`
  for more information.

The ``dhcpd_start`` is necessary to let your board to associate an IP to your smartphone.

Memory Map
==========

Address Mapping
---------------

================== ========== ========== =============== ===============
BUS TYPE           START      LAST       DESCRIPTION     NOTES
================== ========== ========== =============== ===============
To be added
================== ========== ========== =============== ===============

Embedded Memory
---------------

=========== ========== ========== =============== ===============
BUS TYPE    START      LAST       DESCRIPTION     NOTES
=========== ========== ========== =============== ===============
To be added
=========== ========== ========== =============== ===============

Boundary Address (Embedded)
---------------------------

====================== ========== ========== =============== ===============
BUS TYPE               START      LAST       DESCRIPTION     NOTES
====================== ========== ========== =============== ===============
To be added
====================== ========== ========== =============== ===============

External Memory
---------------

=========== ========== ========== =============== ===============
BUS TYPE    START      LAST       DESCRIPTION     NOTES
=========== ========== ========== =============== ===============
To be added
=========== ========== ========== =============== ===============

Boundary Address (External)
---------------------------

To be added

Linker Segments
---------------

+---------------------+------------+------------+------+------------------------------+
| DESCRIPTION         | START      | END        | ATTR | LINKER SEGMENT NAME          |
+=====================+============+============+======+==============================+
| To be added         |            |            |      |                              |
+---------------------+------------+------------+------+------------------------------+

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
