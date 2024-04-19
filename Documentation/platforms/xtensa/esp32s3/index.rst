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

ESP32-S3 Toolchain
==================

The toolchain used to build ESP32-S3 firmware can be either downloaded or built from the sources.
It is **highly** recommended to use (download or build) the same toolchain version that is being
used by the NuttX CI.

Please refer to the Docker
`container <https://github.com/apache/nuttx/tree/master/tools/ci/docker/linux/Dockerfile>`_ and
check for the current compiler version being used. For instance:

.. code-block::

  ###############################################################################
  # Build image for tool required by ESP32 builds
  ###############################################################################
  FROM nuttx-toolchain-base AS nuttx-toolchain-esp32
  # Download the latest ESP32 GCC toolchain prebuilt by Espressif
  RUN mkdir -p xtensa-esp32-elf-gcc && \
    curl -s -L "https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/xtensa-esp32-elf-12.2.0_20230208-x86_64-linux-gnu.tar.xz" \
    | tar -C xtensa-esp32-elf-gcc --strip-components 1 -xJ

  RUN mkdir -p xtensa-esp32s2-elf-gcc && \
    curl -s -L "https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/xtensa-esp32s2-elf-12.2.0_20230208-x86_64-linux-gnu.tar.xz" \
    | tar -C xtensa-esp32s2-elf-gcc --strip-components 1 -xJ

  RUN mkdir -p xtensa-esp32s3-elf-gcc && \
    curl -s -L "https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/xtensa-esp32s3-elf-12.2.0_20230208-x86_64-linux-gnu.tar.xz" \
    | tar -C xtensa-esp32s3-elf-gcc --strip-components 1 -xJ

For ESP32-S3, the toolchain version is based on GGC 12.2.0 (``xtensa-esp32s3-elf-12.2.0_20230208``)

The prebuilt Toolchain (Recommended)
------------------------------------

First, create a directory to hold the toolchain:

.. code-block:: console

  $ mkdir -p /path/to/your/toolchain/xtensa-esp32s3-elf-gcc

Download and extract toolchain:

.. code-block:: console

  $ curl -s -L "https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/xtensa-esp32s3-elf-12.2.0_20230208-x86_64-linux-gnu.tar.xz" \
  | tar -C xtensa-esp32s3-elf-gcc --strip-components 1 -xJ

Add the toolchain to your `PATH`:

.. code-block:: console

  $ echo "export PATH=/path/to/your/toolchain/xtensa-esp32s3-elf-gcc/bin:$PATH" >> ~/.bashrc

You can edit your shell's rc files if you don't use bash.

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

Building and flashing NuttX
===========================

Bootloader and partitions
-------------------------

Nuttx can boot the ESP32-S3 directly using the so-called "Simple Boot". An externally-built
2nd stage bootloader is not required in this case as all functions required to boot the device
are built within Nuttx. Simple boot does not require any specific configuration (it is selectable
by default if no other 2nd stage bootloader is used).

If other features are required, an externally-built 2nd stage bootloader is needed. The bootloader
is built using the ``make bootloader`` command. This command generates the firmware in the
``nuttx`` folder. The ``ESPTOOL_BINDIR`` is used in the ``make flash`` command to specify the path
to the bootloader. For compatibility among other SoCs and future options of 2nd stage bootloaders,
the commands ``make bootloader`` and the ``ESPTOOL_BINDIR`` option (for the ``make flash``) can be
used even if no externally-built 2nd stage bootloader is being built (they will be ignored if
Simple Boot is used, for instance)::

  $ make bootloader

.. note:: It is recommended that if this is the first time you are using the board with NuttX to
   perform a complete SPI FLASH erase.

    .. code-block:: console

      $ esptool.py erase_flash

Building and Flashing
---------------------

First, make sure that ``esptool.py`` is installed.  This tool is used to convert the ELF to a
compatible ESP32-S3 image and to flash the image into the board.
It can be installed with: ``pip install esptool``.

It's a two-step process where the first converts the ELF file into an ESP32-S3 compatible binary
and the second flashes it to the board. These steps are included in the build system and it is
possible to build and flash the NuttX firmware simply by running::

    $ make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

where ``<port>`` is typically ``/dev/ttyUSB0`` or similar. ``ESPTOOL_BINDIR=./`` is the path of the
externally-built 2nd stage bootloader and the partition table (if applicable): when built using the
``make bootloader``, these files are placed into ``nuttx`` folder. ``ESPTOOL_BAUD`` is able to
change the flash baudrate if desired.

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX:

========== ======= =====
Peripheral Support NOTES
========== ======= =====
ADC          No
AES          YES
Bluetooth    No
CAMERA       No
CAN/TWAI     Yes
DMA          Yes
eFuse        No
GPIO         Yes
I2C          No
I2S          Yes
LCD          No
LED_PWM      No
MCPWM        No
Pulse_CNT    No
RMT          No
RNG          No
RSA          No
RTC          Yes
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

.. tip:: Boards usually expose a ``wifi`` defconfig which enables Wi-Fi. On ESP32-S3,
   SMP is enabled to enhance Wi-Fi performance.

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

It is possible to use ESP32-S3 as an Access Point (SoftAP).

.. tip:: Boards usually expose a ``sta_softap`` defconfig which enables Wi-Fi
   (STA + SoftAP). On ESP32-S3, SMP is enabled to enhance Wi-Fi performance.

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

Supported Boards
================

.. toctree::
  :glob:
  :maxdepth: 1

  boards/*/*
