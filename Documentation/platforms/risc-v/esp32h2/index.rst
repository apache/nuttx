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

Second stage bootloader
=======================

Nuttx can boot the ESP32-H2 directly using the so-called "Simple Boot". 
An externally-built 2nd stage bootloader is not required in this case as all
functions required to boot the device are built within Nuttx. Simple boot does not
require any specific configuration (it is selectable by default if no other
2nd stage bootloader is used). For compatibility among other SoCs and future options
of 2nd stage bootloaders, the commands ``make bootloader`` and the ``ESPTOOL_BINDIR``
option (for the ``make flash``) are kept (and ignored if Simple Boot is used).

Building and flashing
=====================

First, make sure that ``esptool.py`` is installed.  This tool is used to convert
the ELF to a compatible ESP32-H2 image and to flash the image into the board.
It can be installed with: ``pip install esptool``.

Configure the NuttX project: ``./tools/configure.sh esp32h2-devkit:nsh``
Run ``make`` to build the project.  Note that the conversion mentioned above is
included in the build process.
The ``esptool.py`` is used to flash all the binaries. However, this is also
included in the build process and we can build and flash with::

  make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=./

Where ``<port>`` is typically ``/dev/ttyUSB0`` or similar and ``./`` is
the path to the folder containing the externally-built 2nd stage bootloader for
the ESP32-H2 as explained above.

Debugging with OpenOCD
======================

Download and build OpenOCD from Espressif, that can be found in
https://github.com/espressif/openocd-esp32

You don not need an external JTAG is to debug, the ESP32-H2 integrates a
USB-to-JTAG adapter.

OpenOCD can then be used::

  openocd -c 'set ESP_RTOS none' -f board/esp32h2-builtin.cfg

If you want to debug with an external JTAG adapter it can
be connected as follows::

  TMS -> GPIO2
  TDI -> GPIO5
  TCK -> GPIO5
  TDO -> GPIO3

Furthermore, an efuse needs to be burnt to be able to debug::

  espefuse.py -p <port> burn_efuse DIS_USB_JTAG

OpenOCD can then be used::

  openocd  -c 'set ESP_RTOS none' -f board/esp32h2-ftdi.cfg

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
DMA              No
ECC              No
eFuse            No
GPIO             Yes
HMAC             No
I2C              No
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
SPI              No
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
