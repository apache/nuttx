.. _esp32c2:

==================
Espressif ESP32-C2
==================

The ESP32-C2 (also sold as ESP8684) is a highly integrated, low-power SoC
with a RISC-V core. It supports 2.4 GHz Wi-Fi 4 (802.11b/g/n) and
Bluetooth 5 (LE).

* Internal Memory
  - 576 KB ROM
  - 272 KB SRAM (16 KB can be configured as cache)
  - No RTC retention SRAM (saved RTC time does not survive deep sleep)
* External Memory
  - SPI flash in the module (typically 1 MB, 2 MB or 4 MB)
* Connectivity
  - 2.4 GHz Wi-Fi
  - Bluetooth Low Energy 5
* GPIO
  - 14 user GPIOs (GPIO0–GPIO10, GPIO18–GPIO20)
* Clock
  - Main XTAL is 26 MHz on ESP8684-MINI-1 modules (40 MHz is also supported)

ESP32-C2 Toolchain
==================

A generic RISC-V toolchain can be used to build ESP32-C2 projects. It's recommended to use the same
toolchain used by NuttX CI. Please refer to the Docker
`container <https://github.com/apache/nuttx/tree/master/tools/ci/docker/linux/Dockerfile>`_ and
check for the current compiler version being used. For instance:

.. code-block::

  ###############################################################################
  # Build image for tool required by RISCV builds
  ###############################################################################
  FROM nuttx-toolchain-base AS nuttx-toolchain-riscv
  # Download the latest RISCV GCC toolchain prebuilt by xPack
  RUN mkdir riscv-none-elf-gcc && \
  curl -s -L "https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v13.2.0-2/xpack-riscv-none-elf-gcc-13.2.0-2-linux-x64.tar.gz" \
  | tar -C riscv-none-elf-gcc --strip-components 1 -xz

It uses the xPack's prebuilt toolchain based on GCC 13.2.0-2.

Installing
----------

First, create a directory to hold the toolchain:

.. code-block:: console

  $ mkdir -p /path/to/your/toolchain/riscv-none-elf-gcc

Download and extract toolchain:

.. code-block:: console

  $ curl -s -L "https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v13.2.0-2/xpack-riscv-none-elf-gcc-13.2.0-2-linux-x64.tar.gz" \
  | tar -C /path/to/your/toolchain/riscv-none-elf-gcc --strip-components 1 -xz

Add the toolchain to your `PATH`:

.. code-block:: console

  $ echo "export PATH=/path/to/your/toolchain/riscv-none-elf-gcc/bin:$PATH" >> ~/.bashrc

You can edit your shell's rc files if you don't use bash.

Building and flashing NuttX
===========================

Installing esptool
------------------

Make sure that ``esptool.py`` is installed and up-to-date.
This tool is used to convert the ELF to a compatible ESP32-C2 image and to flash the image into the board.

It can be installed with: ``pip install esptool>=4.8.1``.

.. warning::
    Installing ``esptool.py`` may required a Python virtual environment on newer systems.
    This will be the case if the ``pip install`` command throws an error such as:
    ``error: externally-managed-environment``.

    If you are not familiar with virtual environments, refer to `Managing esptool on virtual environment`_ for instructions on how to install ``esptool.py``.

Bootloader and partitions
-------------------------

NuttX can boot the ESP32-C2 directly using the so-called "Simple Boot".
An externally-built 2nd stage bootloader is not required in this case as all
functions required to boot the device are built within NuttX. Simple boot does not
require any specific configuration (it is selectable by default if no other
2nd stage bootloader is used).

If features like `Flash Encryption`_ are required, an externally-built
2nd stage bootloader is needed. The MCUBoot bootloader is built using
the ``make bootloader`` command. This command generates the firmware in the
``nuttx`` folder. The ``ESPTOOL_BINDIR`` is used in the ``make flash`` command
to specify the path to the bootloader. For compatibility among other SoCs and
future options of 2nd stage bootloaders, the commands ``make bootloader`` and
the ``ESPTOOL_BINDIR`` option (for the ``make flash``) can be used even if no
externally-built 2nd stage bootloader is being built (they will be ignored if
Simple Boot is used, for instance)::

  $ make bootloader

.. note::
   MCUBoot support for ESP32-C2 on NuttX is still in progress. The
   ``mcuboot_nsh`` board configuration can build an MCUBoot-format image,
   but there is no ``mcuboot_update_agent`` configuration yet. The default
   MCUBoot slot map in Kconfig assumes at least 4 MB of flash and does
   **not** fit the 2 MB modules used on many ESP8684-DevKitM-1 boards.

.. note:: It is recommended that if this is the first time you are using the board with NuttX to
   perform a complete SPI FLASH erase.

    .. code-block:: console

      $ esptool.py erase_flash

Building and Flashing
---------------------

This is a two-step process where the first step converts the ELF file into an ESP32-C2 compatible binary
and the second step flashes it to the board. These steps are included in the build system and it is
possible to build and flash the NuttX firmware simply by running::

    $ make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

where:

* ``ESPTOOL_PORT`` is typically ``/dev/ttyUSB0`` or similar.
* ``ESPTOOL_BINDIR=./`` is the path of the externally-built 2nd stage bootloader and the partition table (if applicable): when built using the ``make bootloader``, these files are placed into ``nuttx`` folder.
* ``ESPTOOL_BAUD`` is able to change the flash baud rate if desired.

The ESP32-C2 port defaults to **2 MB** flash and **60 MHz** flash clock.

Flashing NSH Example
--------------------

This example shows how to build and flash the ``nsh`` defconfig for the ESP8684-DevKitM-1 board::

    $ cd nuttx
    $ make distclean
    $ ./tools/configure.sh esp8684-devkitm:nsh
    $ make -j$(nproc)

When the build is complete, the firmware can be flashed to the board using the command::

    $ make -j$(nproc) flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

where ``<port>`` is the serial port where the board is connected::

  $ make flash ESPTOOL_PORT=/dev/ttyUSB0 ESPTOOL_BINDIR=./
  CP: nuttx.hex
  MKIMAGE: NuttX binary
  esptool.py -c esp32c2 elf2image --ram-only-header -fs 2MB -fm dio -ff 60m -o nuttx.bin nuttx
  [...]
  Generated: nuttx.bin
  esptool.py -c esp32c2 -p /dev/ttyUSB0 -b 921600  write_flash -fs 2MB -fm dio -ff 60m 0x0000 nuttx.bin
  [...]
  Hard resetting via RTS pin...

Now opening the serial port with a terminal emulator should show the NuttX console::

  $ picocom -b 115200 /dev/ttyUSB0
  NuttShell (NSH) NuttX-12.8.0
  nsh> uname -a
  NuttX 12.8.0 ... risc-v esp8684-devkitm

The USB-to-UART bridge on the DevKit exposes UART0. The default UART0 pins
are GPIO20 (TX) and GPIO19 (RX). Use a USB cable that carries data lines;
charge-only cables will not enumerate the bridge.

Building with CMake
-------------------

General CMake usage (out-of-tree build, ``menuconfig`` target, and so on) is described in
:doc:`/quickstart/compiling_cmake`. The ESP32-C2 common arch enables post-build steps that
produce ``nuttx.bin`` (and related images) under the **CMake binary directory**; the build
log also prints suggested ``esptool.py`` command lines for your layout.

Example (NuttX shell defconfig, Ninja generator)::

  $ cd nuttx
  $ cmake -B build -DBOARD_CONFIG=esp8684-devkitm:nsh -GNinja
  $ cmake --build build

To reconfigure the tree after changing options (same as other NuttX CMake boards)::

  $ cmake --build build -t menuconfig
  $ cmake --build build

Persistent HAL cache (``NXTMPDIR``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Pass ``-DNXTMPDIR=ON`` at **configure** time to reuse a persistent clone of the
``esp-hal-3rdparty`` repository under ``nuttx/../nxtmpdir/esp-hal-3rdparty``. CMake checks
the expected revision; if it does not match, the cache directory is refreshed. This cuts
repeat configure/build time when the HAL checkout would otherwise be re-fetched into the
binary directory.

Example::

  $ cmake -B build -DBOARD_CONFIG=esp8684-devkitm:nsh -DNXTMPDIR=ON -GNinja
  $ cmake --build build

MCUBoot: building the 2nd-stage bootloader (``-t bootloader``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For configurations that use MCUboot, build the bootloader the same way as
with Make, but via the CMake target::

  $ cmake --build build -t bootloader

The image is installed as ``mcuboot-esp32c2.bin`` in the NuttX **source** directory (not
inside ``build/``).

.. note::

   Flashing paths differ from the pure-Make flow: the application image is under your CMake
   build directory (for example ``build/nuttx.bin``), while MCUboot binaries live next to
   ``nuttx`` sources.

Target flashing (``-t flash``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After a successful CMake build, you can flash the chip with the ``flash`` custom target.
This is the CMake-side equivalent of the Make ``FLASH`` logic in
``tools/espressif/Config.mk``.

**Serial port:** you must set ``ESPTOOL_PORT`` to a non-empty value (for example
``/dev/ttyUSB0``). If it is unset or empty, the flash step fails.

Example::

  $ export ESPTOOL_PORT=/dev/ttyUSB0
  $ cmake --build build -t flash

Or for a single invocation::

  $ ESPTOOL_PORT=/dev/ttyUSB0 cmake --build build -t flash

Debugging
=========

This section describes debugging techniques for the ESP32-C2.

Debugging with ``openocd`` and ``gdb``
--------------------------------------

Espressif uses a specific version of OpenOCD to support ESP32-C2: `openocd-esp32 <https://github.com/espressif/openocd-esp32>`_.

Please check `Building OpenOCD from Sources <https://docs.espressif.com/projects/esp-idf/en/latest/esp32c2/api-guides/jtag-debugging/index.html#jtag-debugging-building-openocd>`_
for more information on how to build OpenOCD for ESP32-C2.

The ESP32-C2 does **not** integrate a USB-to-JTAG adapter. An external JTAG
adapter is required and can be connected as follows:

============ ===========
ESP32-C2 Pin JTAG Signal
============ ===========
GPIO4        TMS
GPIO5        TDI
GPIO6        TCK
GPIO7        TDO
============ ===========

These pins are also the default MTMS / MTDI / MTCK / MTDO strapping functions
on the ESP8684-DevKitM-1 header.

OpenOCD can then be used::

  openocd -c 'set ESP_RTOS hwthread; set ESP_FLASH_SIZE 0' -f board/esp32c2-ftdi.cfg

Once OpenOCD is running, you can use GDB to connect to it and debug your application::

  riscv-none-elf-gdb -x gdbinit nuttx

whereas the content of the ``gdbinit`` file is::

  target remote :3333
  set remote hardware-watchpoint-limit 2
  mon reset halt
  flushregs
  monitor reset halt
  thb nsh_main
  c

.. note:: ``nuttx`` is the ELF file generated by the build process. Please note that ``CONFIG_DEBUG_SYMBOLS`` must be enabled in the ``menuconfig``.

.. note::
  ``appimage_offset`` should be set to ``0x0`` when ``Simple Boot`` is used. For MCUboot, this value should be set to
  ``CONFIG_ESPRESSIF_OTA_PRIMARY_SLOT_OFFSET`` (``0x20000`` by default).

Please refer to :doc:`/quickstart/debugging` for more information about debugging techniques.

Stack Dump and Backtrace Dump
-----------------------------

NuttX has a feature to dump the stack of a task and to dump the backtrace of it (and of all
the other tasks). This feature is useful to debug the system when it is not behaving as expected,
especially when it is crashing.

In order to enable this feature, the following options must be enabled in the NuttX configuration:
``CONFIG_SCHED_BACKTRACE``, ``CONFIG_DEBUG_SYMBOLS`` and, optionally, ``CONFIG_ALLSYMS``.

.. note::
   The first two options enable the backtrace dump. The third option enables the backtrace dump
   with the associated symbols, but increases the size of the generated NuttX binary.

Espressif also provides a tool to translate the backtrace dump into a human-readable format.
This tool is called ``btdecode.sh`` and is available at ``tools/espressif/btdecode.sh`` of NuttX
repository.

.. note::
   This tool is not necessary if ``CONFIG_ALLSYMS`` is enabled. In this case, the backtrace dump
   contains the function names.

Save a crash dump that contains ``sched_dumpstack`` lines to a file and decode it with::

  ./tools/espressif/btdecode.sh esp32c2 /tmp/backtrace.txt

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX:

=========== ======= ====================
Peripheral  Support NOTES
=========== ======= ====================
ADC          Yes    Oneshot
AES          Yes
Bluetooth    Yes
CAN/TWAI     No
DMA          Yes
eFuse        Yes    Also virtual mode supported
GPIO         Yes    Dedicated GPIO supported
HMAC         No
I2C          Yes    Master and Slave mode supported
I2S          Yes
LED/PWM      Yes
RMT          Yes
RNG          Yes
RSA          No
RTC          Yes    No RTC retention SRAM
SHA          Yes
SPI          Yes
SPIFLASH     Yes
SPIRAM       No
Timers       Yes    One timer group
UART         Yes
USB Serial   No     No USB-Serial-JTAG on this SoC
Watchdog     Yes
Wi-Fi        Yes    WPA3-SAE supported
=========== ======= ====================

Analog-to-digital converter (ADC)
---------------------------------

Two ADC units are available for the ESP32-C2:

* ADC1 with 5 channels.
* ADC2 with 1 channel. **This unit is not implemented.**

Those units are independent and can be used simultaneously. During bringup, GPIOs for selected channels are
configured automatically to be used as ADC inputs.
If available, ADC calibration is automatically applied (see
`this page <https://docs.espressif.com/projects/esp-idf/en/latest/esp32c2/api-reference/peripherals/adc_calibration.html>`__ for more details).
Otherwise, a simple conversion is applied based on the attenuation and resolution.

The ADC unit is accessible using the ADC character driver, which returns data for the enabled channels.

The ADC1 unit can be enabled in the menu :menuselection:`System Type --> Peripheral Support --> Analog-to-digital converter (ADC)`.

Then, it can be customized in the menu :menuselection:`System Type --> ADC Configuration`, which includes operating mode, gain and channels.

========== ===========
 Channel    ADC1 GPIO
========== ===========
0           0
1           1
2           2
3           3
4           4
========== ===========

ADC2 channel 0 is GPIO5.

.. warning:: Maximum measurable voltage may saturate around 2900 mV.

.. _MCUBoot C2:

MCUBoot
=======

The ESP32-C2 can use MCUBoot as a 2nd stage bootloader. NuttX integration is
still marked as in progress upstream (see
`MCUBoot Espressif port <https://docs.mcuboot.com/readme-espressif.html>`__).

The ``esp8684-devkitm:mcuboot_nsh`` configuration produces an MCUBoot-compatible
application image and enables ``make bootloader``. There is no
``mcuboot_update_agent`` defconfig for this board yet.

.. warning::
   Default MCUBoot Kconfig offsets (primary ``0x20000``, secondary ``0x170000``,
   scratch ``0x2C0000``, optional storage ``0x300000``) assume **4 MB or more**
   of flash. ESP8684-DevKitM-1 boards commonly ship with **2 MB**. Using those
   defaults on 2 MB flash will place partitions past the end of the device.
   Override ``ESPRESSIF_OTA_*`` and ``ESPRESSIF_STORAGE_MTD_*`` before enabling
   MCUBoot on 2 MB parts.

For Simple Boot on 2 MB flash, the storage MTD defaults to offset ``0x110000``
and size ``0xf0000``.

Flash Encryption
----------------

Flash encryption is intended for encrypting the contents of the ESP32-C2's off-chip flash memory. Once this feature is enabled,
firmware is flashed as plaintext, and then the data is encrypted in place on the first boot. As a result, physical readout
of flash will not be sufficient to recover most flash contents.

The current state of flash encryption for ESP32-C2 allows the use of Virtual E-Fuses and development mode, which permit users to evaluate and test the firmware before making definitive changes such as burning E-Fuses.

Flash encryption supports the following features:

  .. list-table::
    :header-rows: 1

    * - Feature
      - Description
    * - **Flash Encryption with Virtual E-Fuses**
      - Use flash encryption without burning E-Fuses. Default selection when flash encryption is enabled.
    * - **Flash Encryption in Development mode**
      - Allows reflashing an encrypted device by appending the ``--encrypt`` argument to the ``esptool.py write_flash`` command. This is done automatically if ``ESPRESSIF_SECURE_FLASH_ENC_FLASH_DEVICE_ENCRYPTED`` is set.
    * - **Flash Encryption in Release mode**
      - Does not allow reflashing the device. This is a permanent setting.
    * - **Flash Encryption key**
      - A user-generated key is required by default. Alternatively, a device-generated key is possible, but it will not be recoverable by the user (not recommended). See ``ESPRESSIF_SECURE_FLASH_ENC_USE_HOST_KEY``.
    * - **Encrypted MTD Partition**
      - If SPI Flash is enabled, an empty user MTD partition will be automatically encrypted on first flash.

.. note::

   It is **strongly suggested** to read the following before working on flash encryption:

   - `MCUBoot Flash Encryption <https://docs.mcuboot.com/readme-espressif.html#flash-encryption>`_
   - `General E-Fuse documentation <https://docs.espressif.com/projects/esp-idf/en/latest/esp32c2/api-reference/system/efuse.html>`_
   - `Flash Encryption Relevant E-Fuses <https://docs.espressif.com/projects/esp-idf/en/latest/esp32c2/security/flash-encryption.html#relevant-efuses>`_

   ESP32-C2 Secure Boot V2 uses **ECDSA**, not RSA.

Flash Encryption Requirements
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Flash encryption requires burning E-Fuses to enable it on chip. This is not a reversible operation and should be done with caution.
There is, however, a way to test the flash encryption by simulating them on flash.

Build System Features
'''''''''''''''''''''

The build system contains some safeguards to avoid accidentally burning E-Fuses and automations for convenience. Those are summarized below:

  1. A yellow warning will show up during build alerting that flash encryption is enabled (same for Virtual E-Fuses).
  2. If ``ESPRESSIF_SECURE_FLASH_ENC_USE_HOST_KEY`` is set, build will fail if the flash encryption key is not found.
  3. If SPI Flash is enabled, the user MTD partition is automatically encrypted with the provided encryption key.
  4. ``make flash`` command will prompt the user for confirmation before burning the E-Fuse, if Virtual E-Fuses are disabled.

Simulating Flash Encryption with Virtual E-Fuses
'''''''''''''''''''''''''''''''''''''''''''''''''

It is highly recommended to use this method for testing the flash encryption before actually burning the E-Fuses.
The E-Fuses are stored in flash and persist between reboots. No real E-Fuses are changed.

To enable virtual E-Fuses for flash encryption testing, open ``menuconfig`` and:
  1. Enable flash encryption on boot on: :menuselection:`System Type --> Bootloader and Image Configuration`
  2. Verify Virtual E-Fuses are enabled (this is done by default): :menuselection:`System Type --> Peripheral Support --> E-Fuse support`

Actual encryption and burning E-Fuses
'''''''''''''''''''''''''''''''''''''

E-Fuses are burned by esptool and the bootloader on the first boot after flashing with encryption enabled.
This process is automated on NuttX build system.

.. warning::  Burning E-Fuses is NOT a reversible operation and should be done with caution.

To build a firmware with E-Fuse support and flash encryption enabled, open ``menuconfig`` and:
  1. Enable flash encryption on boot on: :menuselection:`System Type --> Bootloader and Image Configuration`
  2. Disable Virtual E-Fuses :menuselection:`System Type --> Peripheral Support --> E-Fuse support`
  3. Check usage mode is Development (this allows reflashing, while Release mode does not).

.. note::  If using development mode of flash encryption (see menuconfig and documentation above), it is still possible to re-flash the device with esptool by
  setting ``ESPRESSIF_SECURE_FLASH_ENC_FLASH_DEVICE_ENCRYPTED`` which adds ``--encrypt`` argument to the ``esptool.py write_flash`` command.
  This will apply the burned encryption key to the image while flashing.

Flash Allocation for MCUBoot
----------------------------

When MCUBoot is enabled, the **default** Kconfig layout is the same as on other
Espressif RISC-V chips (4 MB class). Do not use it unchanged on 2 MB flash.

**Default flash layout (MCUBoot enabled, 4 MB+)**

.. list-table::
   :header-rows: 1
   :widths: 40 20 20
   :align: left

   * - Region
     - Offset
     - Size
   * - Bootloader
     - 0x000000
     - 64KB
   * - E-Fuse Virtual (see Note)
     - 0x010000
     - 64KB
   * - Primary Application Slot (/dev/ota0)
     - 0x020000
     - 1.4MB
   * - Secondary Application Slot (/dev/ota1)
     - 0x170000
     - 1.4MB
   * - Scratch Partition (/dev/otascratch)
     - 0x2C0000
     - 256KB
   * - Storage MTD (optional)
     - 0x300000
     - 1MB
   * - Available Flash
     - 0x400000+
     - Remaining

.. raw:: html

   <div style="clear: both"></div>

**Note**: The E-Fuse Virtual region is optional and only used when
``ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH`` is enabled. However, this 64KB
location is always allocated in the memory layout to prevent accidental
erasure during board flashing operations, ensuring data preservation if
virtual E-Fuses are later enabled.

The key KConfig options that control this layout:

- ``ESPRESSIF_OTA_PRIMARY_SLOT_OFFSET`` (default: 0x20000)
- ``ESPRESSIF_OTA_SECONDARY_SLOT_OFFSET`` (default: 0x170000)
- ``ESPRESSIF_OTA_SLOT_SIZE`` (default: 0x150000)
- ``ESPRESSIF_OTA_SCRATCH_OFFSET`` (default: 0x2C0000)
- ``ESPRESSIF_OTA_SCRATCH_SIZE`` (default: 0x40000)
- ``ESPRESSIF_STORAGE_MTD_OFFSET`` (default: 0x300000 when MCUBoot enabled)
- ``ESPRESSIF_STORAGE_MTD_SIZE`` (default: 0x100000)

For MCUBoot operation:

- The **Primary Slot** contains the currently running application
- The **Secondary Slot** receives OTA updates
- The **Scratch Partition** is used by MCUBoot for image swapping during updates
- MCUBoot manages image validation, confirmation, and rollback functionality

_`Managing esptool on virtual environment`
==========================================

This section describes how to install ``esptool``, ``imgtool`` or any other Python packages in a
proper environment.

Normally, a Linux-based OS would already have Python 3 installed by default. Up to a few years ago,
you could simply call ``pip install`` to install packages globally. However, this is no longer recommended
as it can lead to conflicts between packages and versions. The recommended way to install Python packages
is to use a virtual environment.

A virtual environment is a self-contained directory that contains a Python installation for a particular
version of Python, plus a number of additional packages. You can create a virtual environment for each
project you are working on, and install the required packages in that environment.

Two alternatives are explained below, you can select any one of those.

Using pipx (recommended)
------------------------

``pipx`` is a tool that makes it easy to install Python packages in a virtual environment. To install
``pipx``, you can run the following command (using apt as example)::

    $ apt install pipx

Once you have installed ``pipx``, you can use it to install Python packages in a virtual environment. For
example, to install the ``esptool`` package, you can run the following command::

    $ pipx install esptool

This will create a new virtual environment in the ``~/.local/pipx/venvs`` directory, which contains the
``esptool`` package. You can now use the ``esptool`` command as normal, and so will the build system.

Make sure to run ``pipx ensurepath`` to add the ``~/.local/bin`` directory to your ``PATH``. This will
allow you to run the ``esptool`` command from any directory.

Using venv (alternative)
------------------------
To create a virtual environment, you can use the ``venv`` module, which is included in the Python standard
library. To create a virtual environment, you can run the following command::

    $ python3 -m venv myenv

This will create a new directory called ``myenv`` in the current directory, which contains a Python
installation and a copy of the Python standard library. To activate the virtual environment, you can run
the following command::

    $ source myenv/bin/activate

This will change your shell prompt to indicate that you are now working in the virtual environment. You can
now install packages using ``pip``. For example, to install the ``esptool`` package, you can run the following
command::

    $ pip install esptool

This will install the ``esptool`` package in the virtual environment. You can now use the ``esptool`` command as
normal. When you are finished working in the virtual environment, you can deactivate it by running the following
command::

    $ deactivate

This will return your shell prompt to its normal state. You can reactivate the virtual environment at any time by
running the ``source myenv/bin/activate`` command again. You can also delete the virtual environment by deleting
the directory that contains it.

Supported Boards
================

.. toctree::
  :glob:
  :maxdepth: 1

  boards/*/*
