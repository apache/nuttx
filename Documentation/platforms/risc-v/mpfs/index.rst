===========================
Microchip Polarfire (MPFS)
===========================

RV64 64-bit RISC-V multiprocessor-based Microcontroller Subsystem
(MPFS025T, MPFS095T, MPFS160T, MPFS250T, MPFS460T)

MPFS Toolchain
==============

A generic RISC-V toolchain can be used to build MPFS projects.
Like: https://xpack.github.io/riscv-none-embed-gcc or https://github.com/sifive/freedom-tools/releases

Booting
=======

The NuttX works as a standalone operating system that may initialize all the required clocks and
peripherals including DDR memory. Alternatively, the vendor's HSS bootloader may be used instead to
perform all the initialization steps.

Building and flashing
=====================

Configure the NuttX project: ``./tools/configure.sh icicle:nsh``
Run ``make`` to build the project.

If NuttX is built as a standalone OS with the config option CONFIG_MPFS_BOOTLOADER set, it's important
to note that the image size is very limited: only (128K - 256) bytes. 256 bytes are reserved for the
header that must be prepended into the NuttX binary. This binary is called hss-envm-wrapper-bm1-dummySbic.bin
and may be found from the vendor's HSS implementation. Moreover, after prepending the 256-byte header,
the LMA section needs to be adjusted, for example::

   riscv64-unknown-elf-objcopy -I binary -O ihex --change-section-lma *+0x20220000 nuttx.bin nuttx.hex

The output binary nuttx.hex may be flashed with the LiberoSoc tool.
The tool is available from: https://www.microchip.com/en-us/products/fpgas-and-plds/fpga-and-soc-design-tools/fpga/libero-software-later-versions#Documents%20and%20Downloads

There's an alternative way to use NuttX, without the CONFIG_MPFS_BOOTLOADER option set. This expects the
HSS bootloader sets up the system (memories, caches, initializes clocks etc.).
First make sure that ``hss-payload-generator`` is installed.
Available from: https://github.com/polarfire-soc/hart-software-services

This tool is used to convert the ELF/bin to a compatible HSS payload image

Create HSS payload bin::

   hss-payload-generator -v -c hss-nuttx.yml payload.bin

Debugging with OpenOCD
======================

Compatible OpenOCD and configs can be downloaded from:
https://www.microsemi.com/product-directory/design-tools/4879-softconsole#downloads

OpenOCD can then be used::

   openocd -c "set DEVICE MPFS" --file board/microsemi-riscv.cfg

Peripheral Support
==================

The following list indicates the state of peripherals' support in NuttX:

============   =======  =====
Peripheral     Support  NOTES
============   =======  =====
GPIO           Yes
MMUART         Yes      Uart mode only
SPI            Yes
I2C            Yes
eMMC SD/SDIO   Yes      No PHY training
USB            Yes
Ethernet MAC   Yes
Timers         No
Watchdog       No
RTC            No
CAN            No
eNVM           No
============   =======  =====

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*


