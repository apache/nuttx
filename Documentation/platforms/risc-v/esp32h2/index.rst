==================
Espressif ESP32-H2
==================

The ESP32-H2 is an ultra-low-power and highly integrated SoC with a RISC-V
core and supports 2.4 GHz transceiver, Bluetooth 5 (LE) and the 802.15.4 protocol.

* Address Space
  - 452 KB of internal memory address space accessed from the instruction bus
  - 452 KB of internal memory address space accessed from the data bus
  - 832 KB of peripheral address space
  - 16 MB of external memory virtual address space accessed from the instruction bus
  - 16 MB of external memory virtual address space accessed from the data bus
  - 260 KB of internal DMA address space
* Internal Memory
  - 128 KB ROM
  - 320 KB SRAM (16 KB can be configured as Cache)
  - 4 KB of SRAM in RTC
* External Memory
  - Up to 16 MB of external flash
* Peripherals
  - Multiple peripherals
* GDMA
  - 7 modules are capable of DMA operations.

ESP32-H2 Toolchain
==================

A generic RISC-V toolchain can be used to build ESP32-H2 projects. It's recommended to use the same
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

Bootloader and partitions
-------------------------

NuttX can boot the ESP32-H2 directly using the so-called "Simple Boot".
An externally-built 2nd stage bootloader is not required in this case as all
functions required to boot the device are built within NuttX. Simple boot does not
require any specific configuration (it is selectable by default if no other
2nd stage bootloader is used). For compatibility among other SoCs and future options
of 2nd stage bootloaders, the commands ``make bootloader`` and the ``ESPTOOL_BINDIR``
option (for the ``make flash``) are kept (and ignored if Simple Boot is used).

If other features are required, an externally-built 2nd stage bootloader is needed.
The bootloader is built using the ``make bootloader`` command. This command generates
the firmware in the ``nuttx`` folder. The ``ESPTOOL_BINDIR`` is used in the
``make flash`` command to specify the path to the bootloader. For compatibility
among other SoCs and future options of 2nd stage bootloaders, the commands
``make bootloader`` and the ``ESPTOOL_BINDIR`` option (for the ``make flash``)
can be used even if no externally-built 2nd stage bootloader
is being built (they will be ignored if Simple Boot is used, for instance)::

  $ make bootloader

.. note:: It is recommended that if this is the first time you are using the board with NuttX to
   perform a complete SPI FLASH erase.

    .. code-block:: console

      $ esptool.py erase_flash

Building and flashing
---------------------

First, make sure that ``esptool.py`` is installed.  This tool is used to convert
the ELF to a compatible ESP32-H2 image and to flash the image into the board.
It can be installed with: ``pip install esptool==4.8.dev4``.

Configure the NuttX project: ``./tools/configure.sh esp32h2-devkit:nsh``
Run ``make`` to build the project.  Note that the conversion mentioned above is
included in the build process.
The ``esptool.py`` is used to flash all the binaries. However, this is also
included in the build process and we can build and flash with::

  make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

Where ``<port>`` is typically ``/dev/ttyUSB0`` or similar and ``./`` is
the path to the folder containing the externally-built 2nd stage bootloader for
the ESP32-H2 as explained above.

Debugging with ``openocd`` and ``gdb``
======================================

Espressif uses a specific version of OpenOCD to support ESP32-H2: `openocd-esp32 <https://github.com/espressif/>`_.

Please check `Building OpenOCD from Sources <https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32h2/api-guides/jtag-debugging/index.html#jtag-debugging-building-openocd>`_
for more information on how to build OpenOCD for ESP32-H2.

You do not need an external JTAG to debug, the ESP32-H2 integrates a
USB-to-JTAG adapter.

.. note:: One must configure the USB drivers to enable JTAG communication. Please check
  `Configure USB Drivers <https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32h2/api-guides/jtag-debugging/configure-builtin-jtag.html#configure-usb-drivers>`_
  for more information.

OpenOCD can then be used::

  openocd -c 'set ESP_RTOS hwthread; set ESP_FLASH_SIZE 0' -f board/esp32h2-builtin.cfg

If you want to debug with an external JTAG adapter it can
be connected as follows:

============ ===========
ESP32-H2 Pin JTAG Signal
============ ===========
GPIO2        TMS
GPIO5        TDI
GPIO4        TCK
GPIO3        TDO
============ ===========

Furthermore, an efuse needs to be burnt to be able to debug::

  espefuse.py -p <port> burn_efuse DIS_USB_JTAG

.. warning:: Burning eFuses is an irreversible operation, so please
  consider the above option before starting the process.

OpenOCD can then be used::

  openocd  -c 'set ESP_RTOS hwthread; set ESP_FLASH_SIZE 0' -f board/esp32h2-ftdi.cfg

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

Please refer to :doc:`/quickstart/debugging` for more information about debugging techniques.

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX:

==============  =======
Peripheral      Support
==============  =======
ADC              No
AES              No
Bluetooth        No
CAN/TWAI         Yes
DMA              Yes
ECC              No
eFuse            No
GPIO             Yes
HMAC             No
I2C              Yes
I2S              No
Int. Temp.       No
LED              No
LED_PWM          Yes
MCPWM            No
Pulse Counter    No
RMT              No
RNG              No
RSA              No
RTC              Yes
SD/MMC           No
SDIO             No
SHA              No
SPI              Yes
SPIFLASH         Yes
Timers           Yes
UART             Yes
Watchdog         Yes
Wifi             No
XTS              No
==============  =======

Supported Boards
================

.. toctree::
  :glob:
  :maxdepth: 1

  boards/*/*
