==================
Espressif ESP32-S2
==================

The ESP32-S2 is a series of single-core SoCs from Espressif based on Harvard
architecture Xtensa LX7 CPU and with on-chip support for Wi-Fi.

All embedded memory, external memory and peripherals are located on the
data bus and/or the instruction bus of the CPU. Multiple peripherals in
the system can access embedded memory via DMA.

ESP32-S2 Toolchain
==================

The toolchain used to build ESP32-S2 firmware can be either downloaded or built from the sources.
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

For ESP32-S2, the toolchain version is based on GGC 12.2.0 (``xtensa-esp32s2-elf-12.2.0_20230208``)

The prebuilt Toolchain (Recommended)
------------------------------------

First, create a directory to hold the toolchain:

.. code-block:: console

  $ mkdir -p /path/to/your/toolchain/xtensa-esp32s2-elf-gcc

Download and extract toolchain:

.. code-block:: console

  $ curl -s -L "https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/xtensa-esp32s2-elf-12.2.0_20230208-x86_64-linux-gnu.tar.xz" \
  | tar -C xtensa-esp32s2-elf-gcc --strip-components 1 -xJ

Add the toolchain to your `PATH`:

.. code-block:: console

  $ echo "export PATH=/path/to/your/toolchain/xtensa-esp32s2-elf-gcc/bin:$PATH" >> ~/.bashrc

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

  $ ./ct-ng xtensa-esp32s2-elf
  $ ./ct-ng build

  $ chmod -R u+w builds/xtensa-esp32s2-elf

  $ export PATH="crosstool-NG/builds/xtensa-esp32-elf/bin:$PATH"

These steps are given in the setup guide in
`ESP-IDF documentation <https://docs.espressif.com/projects/esp-idf/en/latest/get-started/linux-setup-scratch.html>`_.

Building and flashing NuttX
===========================

Bootloader and partitions
-------------------------

NuttX can boot the ESP32-S2 directly using the so-called "Simple Boot". An externally-built
2nd stage bootloader is not required in this case as all functions required to boot the device
are built within NuttX. Simple boot does not require any specific configuration (it is selectable
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
compatible ESP32-S2 image and to flash the image into the board.
It can be installed with: ``pip install esptool==4.8.dev4``.

It's a two-step process where the first converts the ELF file into an ESP32-S2 compatible binary
and the second flashes it to the board. These steps are included in the build system and it is
possible to build and flash the NuttX firmware simply by running::

    $ make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

where ``<port>`` is typically ``/dev/ttyUSB0`` or similar. ``ESPTOOL_BINDIR=./`` is the path of the
externally-built 2nd stage bootloader and the partition table (if applicable): when built using the
``make bootloader``, these files are placed into ``nuttx`` folder. ``ESPTOOL_BAUD`` is able to
change the flash baud rate if desired.

Debugging with ``openocd`` and ``gdb``
======================================

Espressif uses a specific version of OpenOCD to support ESP32-S2: `openocd-esp32 <https://github.com/espressif/>`_.

Please check `Building OpenOCD from Sources <https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32s2/api-guides/jtag-debugging/index.html#jtag-debugging-building-openocd>`_
for more information on how to build OpenOCD for ESP32-S2.

ESP32-S2 has dedicated pins for JTAG debugging. The following pins are used for JTAG debugging:

============= ===========
ESP32-S2 Pin  JTAG Signal
============= ===========
MTDO / GPIO40 TDO
MTDI / GPIO41 TDI
MTCK / GPIO39 TCK
MTMS / GPIO42 TMS
============= ===========

Some boards, like :ref:`ESP32-S2-Kaluga-1 Kit v1.3 <platforms/xtensa/esp32s2/boards/esp32s2-kaluga-1/index:ESP32-S2-Kaluga-1 Kit v1.3>` have a built-in JTAG debugger.

Other boards that don't have any built-in JTAG debugger can be debugged using an external JTAG debugger being connected
directly to the ESP32-S2 JTAG pins.

.. note:: One must configure the USB drivers to enable JTAG communication. Please check
  `Configure USB Drivers <https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32s2/api-guides/jtag-debugging/configure-ft2232h-jtag.html?highlight=udev#configure-usb-drivers>`_
  for configuring the JTAG adapter of the :ref:`ESP32-S2-Kaluga-1 <platforms/xtensa/esp32s2/boards/esp32s2-kaluga-1/index:ESP32-S2-Kaluga-1 Kit v1.3>` board
  and other FT2232-based JTAG adapters.

OpenOCD can then be used::

  openocd -c 'set ESP_RTOS hwthread; set ESP_FLASH_SIZE 0' -f board/esp32s2-kaluga-1.cfg

Once OpenOCD is running, you can use GDB to connect to it and debug your application::

  xtensa-esp32s2-elf-gdb -x gdbinit nuttx

whereas the content of the ``gdbinit`` file is::

  target remote :3333
  set remote hardware-watchpoint-limit 2
  mon reset halt
  flushregs
  monitor reset halt
  thb nsh_main
  c

.. note:: ``nuttx`` is the ELF file generated by the build process. Please note that ``CONFIG_DEBUG_SYMBOLS`` must be enabled in the ``menuconfig``.

Please refer to :doc:`/quickstart/debugging` for more information about debugging techniques.

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX:

========== ======= =====
Peripheral Support NOTES
========== ======= =====
ADC          No
AES          No
CAN/TWAI     Yes
DMA          Yes
eFuse        No
Ethernet     No
GPIO         Yes
I2C          Yes
I2S          Yes
LED_PWM      No
Pulse_CNT    No
RMT          No
RNG          Yes
RSA          No
RTC          Yes
SHA          No
SPI          Yes
SPIFLASH     Yes
SPIRAM       Yes
Timers       Yes
Touch        Yes
UART         Yes
Watchdog     Yes
Wifi         No
========== ======= =====

Memory Map
==========

Address Mapping
---------------

================== ========== ========== =============== ===============
BUS TYPE           START      LAST       DESCRIPTION     NOTES
================== ========== ========== =============== ===============
.                  0x00000000 0x3EFFFFFF                 Reserved
Data               0x3F000000 0x3F3FFFFF External Memory
Data               0x3F400000 0x3F4FFFFF Peripheral
Data               0x3F500000 0x3FF7FFFF External Memory
.                  0x3FF80000 0x3FF9DFFF                 Reserved
Data               0x3FF9E000 0x3FFFFFFF Embedded Memory
Instruction        0x40000000 0x40071FFF Embedded Memory
.                  0x40072000 0x4007FFFF                 Reserved
Instruction        0x40080000 0x407FFFFF External Memory
.                  0x40800000 0x4FFFFFFF                 Reserved
Data / Instruction 0x50000000 0x50001FFF Embedded Memory
.                  0x50002000 0x5FFFFFFF                 Reserved
Data / Instruction 0x60000000 0x600BFFFF Peripheral
.                  0x600C0000 0x617FFFFF                 Reserved
Data / Instruction 0x61800000 0x61803FFF Peripheral
.                  0x61804000 0xFFFFFFFF                 Reserved
================== ========== ========== =============== ===============

Embedded Memory
---------------

=========== ========== ========== =============== ================== =====
BUS TYPE    START      LAST       DESCRIPTION     PERMISSION CONTROL NOTES
=========== ========== ========== =============== ================== =====
Data        0x3FF9E000 0x3FF9FFFF RTC FAST Memory YES
Data        0x3FFA0000 0x3FFAFFFF Internal ROM 1  NO
Data        0x3FFB0000 0x3FFB7FFF Internal SRAM 0 YES                DMA
Data        0x3FFB8000 0x3FFFFFFF Internal SRAM 1 YES                DMA
=========== ========== ========== =============== ================== =====

Boundary Address (Embedded)
---------------------------

====================== ========== ========== =============== ================== ===============
BUS TYPE               START      LAST       DESCRIPTION     PERMISSION CONTROL NOTES
====================== ========== ========== =============== ================== ===============
Instruction            0x40000000 0x4000FFFF Internal ROM 0  NO
Instruction            0x40010000 0x4001FFFF Internal ROM 1  NO
Instruction            0x40020000 0x40027FFF Internal SRAM 0 YES
Instruction            0x40028000 0x4006FFFF Internal SRAM 1 YES
Instruction            0x40070000 0x40071FFF RTC FAST Memory YES
Data / Instruction     0x50000000 0x50001FFF RTC SLOW Memory YES
====================== ========== ========== =============== ================== ===============

External Memory
---------------

=========== ========== ========== =============== ================== ===============
BUS TYPE    START      LAST       DESCRIPTION     PERMISSION CONTROL NOTES
=========== ========== ========== =============== ================== ===============
Data        0x3F000000 0x3F3FFFFF ICache          YES                Read
Data        0x3F500000 0x3FF7FFFF DCache          YES                Read and Write
=========== ========== ========== =============== ================== ===============

Boundary Address (External)
---------------------------

=========== ========== ========== =============== ================== ===============
BUS TYPE    START      LAST       DESCRIPTION     PERMISSION CONTROL NOTES
=========== ========== ========== =============== ================== ===============
Instruction 0x40080000 0x407FFFFF ICache          YES                Read
=========== ========== ========== =============== ================== ===============

Linker Segments
---------------

+---------------------+------------+-------------------+------+------------------------------+
| DESCRIPTION         | START      | END               | ATTR | LINKER SEGMENT NAME          |
+=====================+============+===================+======+==============================+
| FLASH mapped data:  | 0X3F000020 | 0X3F000020 +      | R    | drom0_0_seg (NOTE 1)         |
|     - .rodata       |            | FLASH_SIZE - 0x20 |      |                              |
|     - Constructors  |            |                   |      |                              |
|       /destructors  |            |                   |      |                              |
+---------------------+------------+-------------------+------+------------------------------+
| COMMON data RAM:    | 0X3FFB0000 | 0x3FFDE000        | RW   | dram0_0_seg (NOTE 2)         |
|  - .bss/.data       |            |                   |      |                              |
+---------------------+------------+-------------------+------+------------------------------+
| IRAM for PRO cpu:   | 0x40022000 | 0x40050000        | RX   | iram0_0_seg                  |
|  - Interrupt Vectors|            |                   |      |                              |
|  - Low level        |            |                   |      |                              |
|    handlers         |            |                   |      |                              |
|  - Xtensa/Espressif |            |                   |      |                              |
|    libraries        |            |                   |      |                              |
+---------------------+------------+-------------------+------+------------------------------+
| RTC fast memory:    | 0x40070000 | 0x40072000        | RWX  | rtc_iram_seg                 |
|  - .rtc.text        |            |                   |      |                              |
|    (unused?)        |            |                   |      |                              |
+---------------------+------------+-------------------+------+------------------------------+
| FLASH:              | 0x40080020 | 0x40080020 +      | RX   | irom0_0_seg (actually FLASH) |
|  - .text            |            | FLASH_SIZE        |      |                              |
|                     |            | (NOTE 3)          |      |                              |
+---------------------+------------+-------------------+------+------------------------------+
| RTC slow memory:    | 0x50000000 | 0x50002000        | RW   | rtc_slow_seg (NOTE 4)        |
|  - .rtc.data/rodata |            |                   |      |                              |
|    (unused?)        |            |                   |      |                              |
+---------------------+------------+-------------------+------+------------------------------+

.. note::
  (1) The linker script will reserve space at the beginning of the segment
      for MCUboot header if ESP32S2_APP_FORMAT_MCUBOOT flag is active.
  (2) Heap starts at the end of dram_0_seg.
  (3) Subtract 0x20 if ESP32S2_APP_FORMAT_MCUBOOT is not active.
  (4) Linker script will reserve space at the beginning and at the end
      of the segment for ULP coprocessor reserve memory.

64-bit Timers
=============

ESP32-S2 has 4 generic timers of 64 bits (2 from Group 0 and 2 from Group 1).
They're accessible as character drivers, the configuration along with a
guidance on how to run the example and the description of the application level
interface can be found in the :doc:`timer documentation </components/drivers/character/timers/timer>`.

Watchdog Timers
===============

ESP32-S2 has 3 WDTs. 2 MWDTs from the Timers Module and 1 RWDT from the RTC Module
(Currently not supported yet). They're accessible as character drivers,
The configuration along with a guidance on how to run the example and the description
of the application level interface can be found in the
:doc:`watchdog documentation </components/drivers/character/timers/watchdog>`.

I2S
===

The I2S peripheral is accessible using either the generic I2S audio driver or a specific
audio codec driver. Also, it's possible to use the I2S character driver to bypass the
audio subsystem and develop specific usages of the I2S peripheral.

.. note:: Note that the bit-width and sample rate can be modified "on-the-go" when using
   audio-related drivers. That is not the case for the I2S character device driver and
   such parameters are set on compile time through `make menuconfig`.

Please check for usage examples using the :doc:`ESP32-S2-Saola-1 </platforms/xtensa/esp32s2/boards/esp32s2-saola-1/index>`.

Secure Boot and Flash Encryption
================================

Secure Boot
-----------

Secure Boot protects a device from running any unauthorized (i.e., unsigned) code by checking that
each piece of software that is being booted is signed. On an ESP32-S2, these pieces of software include
the second stage bootloader and each application binary. Note that the first stage bootloader does not
require signing as it is ROM code thus cannot be changed. This is achieved using specific hardware in
conjunction with MCUboot (read more about MCUboot `here <https://docs.mcuboot.com/>`__).

The Secure Boot process on the ESP32-S2 involves the following steps performed:

1. The first stage bootloader verifies the second stage bootloader's RSA-PSS signature. If the verification is successful,
   the first stage bootloader loads and executes the second stage bootloader.

2. When the second stage bootloader loads a particular application image, the application's signature (RSA, ECDSA or ED25519) is verified
   by MCUboot.
   If the verification is successful, the application image is executed.

.. warning:: Once enabled, Secure Boot will not boot a modified bootloader. The bootloader will only boot an
   application firmware image if it has a verified digital signature. There are implications for reflashing
   updated images once Secure Boot is enabled. You can find more information about the ESP32-S2's Secure boot
   `here <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/security/secure-boot-v2.html>`__.

.. note:: As the bootloader image is built on top of the Hardware Abstraction Layer component
   of `ESP-IDF <https://github.com/espressif/esp-idf>`_, the
   `API port by Espressif <https://docs.mcuboot.com/readme-espressif.html>`_ will be used
   by MCUboot rather than the original NuttX port.

Flash Encryption
----------------

Flash encryption is intended for encrypting the contents of the ESP32-S2's off-chip flash memory. Once this feature is enabled,
firmware is flashed as plaintext, and then the data is encrypted in place on the first boot. As a result, physical readout
of flash will not be sufficient to recover most flash contents.

.. warning::  After enabling Flash Encryption, an encryption key is generated internally by the device and
   cannot be accessed by the user for re-encrypting data and re-flashing the system, hence it will be permanently encrypted.
   Re-flashing an encrypted system is complicated and not always possible. You can find more information about the ESP32-S2's Flash Encryption
   `here <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/security/flash-encryption.html>`__.

Prerequisites
-------------

First of all, we need to install ``imgtool`` (a MCUboot utility application to manipulate binary
images) and ``esptool`` (the ESP32-S2 toolkit)::

    $ pip install imgtool esptool==4.8.dev4

We also need to make sure that the python modules are added to ``PATH``::

    $ echo "PATH=$PATH:/home/$USER/.local/bin" >> ~/.bashrc

Now, we will create a folder to store the generated keys (such as ``~/signing_keys``)::

    $ mkdir ~/signing_keys && cd ~/signing_keys

With all set up, we can now generate keys to sign the bootloader and application binary images,
respectively, of the compiled project::

    $ espsecure.py generate_signing_key --version 2 bootloader_signing_key.pem
    $ imgtool keygen --key app_signing_key.pem --type rsa-3072

.. important:: The contents of the key files must be stored securely and kept secret.

Enabling Secure Boot and Flash Encryption
-----------------------------------------

To enable Secure Boot for the current project, go to the project's NuttX directory, execute ``make menuconfig`` and the following steps:

   1. Enable experimental features in :menuselection:`Build Setup --> Show experimental options`;

   2. Enable MCUboot in :menuselection:`Application Configuration --> Bootloader Utilities --> MCUboot`;

   3. Change image type to ``MCUboot-bootable format`` in :menuselection:`System Type --> Application Image Configuration --> Application Image Format`;

   4. Enable building MCUboot from the source code by selecting ``Build binaries from source``;
      in :menuselection:`System Type --> Application Image Configuration --> Source for bootloader binaries`;

   5. Enable Secure Boot in :menuselection:`System Type --> Application Image Configuration --> Enable hardware Secure Boot in bootloader`;

   6. If you want to protect the SPI Bus against data sniffing, you can enable Flash Encryption in
      :menuselection:`System Type --> Application Image Configuration --> Enable Flash Encryption on boot`.

Now you can design an update and confirm agent to your application. Check the `MCUboot design guide <https://docs.mcuboot.com/design.html>`_ and the
`MCUboot Espressif port documentation <https://docs.mcuboot.com/readme-espressif.html>`_ for
more information on how to apply MCUboot. Also check some `notes about the NuttX MCUboot port <https://github.com/mcu-tools/mcuboot/blob/main/docs/readme-nuttx.md>`_,
the `MCUboot porting guide <https://github.com/mcu-tools/mcuboot/blob/main/docs/PORTING.md>`_ and some
`examples of MCUboot applied in NuttX applications <https://github.com/apache/nuttx-apps/tree/master/examples/mcuboot>`_.

After you developed an application which implements all desired functions, you need to flash it into the primary image slot
of the device (it will automatically be in the confirmed state, you can learn more about image
confirmation `here <https://docs.mcuboot.com/design.html#image-swapping>`__).
To flash to the primary image slot, select ``Application image primary slot`` in
:menuselection:`System Type --> Application Image Configuration --> Target slot for image flashing`
and compile it using ``make -j ESPSEC_KEYDIR=~/signing_keys``.

When creating update images, make sure to change :menuselection:`System Type --> Application Image Configuration --> Target slot for image flashing`
to ``Application image secondary slot``.

.. important:: When deploying your application, make sure to disable UART Download Mode by selecting ``Permanently disabled`` in
   :menuselection:`System Type --> Application Image Configuration --> UART ROM download mode`
   and change usage mode to ``Release`` in `System Type --> Application Image Configuration --> Enable usage mode`.
   **After disabling UART Download Mode you will not be able to flash other images through UART.**

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
