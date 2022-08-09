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
GPIO         Yes
UART         Yes
SPI          Yes
I2C          No
CAN/TWAI     No
DMA          Yes
Wifi         No
SPIFLASH     Yes
SPIRAM       Yes
Timers       Yes
Watchdog     Yes
RTC          No
RNG          No
AES          No
eFuse        No
ADC          No
Bluetooth    No
SDIO         No
SD/MMC       No
I2S          No
LCD          No
CAMERA       No
LED_PWM      No
RMT          No
MCPWM        No
Pulse_CNT    No
SHA          No
RSA          No
USB SERIAL   Yes
USB OTG      No
========== ======= =====

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
