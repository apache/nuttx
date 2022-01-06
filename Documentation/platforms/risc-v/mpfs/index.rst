===========================
Microchip Polarfile (MPFS)
===========================

RV64 64-bit RISC-V multiprocessor-based Microcontroller Subsystem
(MPFS025T, MPFS095T, MPFS160T, MPFS250T, MPFS460T)


MPFS Toolchain
==============

A generic RISC-V toolchain can be used to build MPFS projects.
Like: https://xpack.github.io/riscv-none-embed-gcc or https://github.com/sifive/freedom-tools/releases


Booting
=======

The NuttX port for now relies on HSS bootloader to carry on some hardware initializations.


Building and flashing
=====================

First make sure that ``hss-payload-generator`` is installed.
Available from: https://github.com/polarfire-soc/hart-software-services

This tool is used to convert the ELF/bin to a compatible HSS payload image

Configure the NuttX project: ``./tools/configure.sh icicle:nsh``
Run ``make`` to build the project.

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
eMMC SD/SDIO   Yes      eMMC not fully tested
Timers         No
Watchdog       No       
RTC            No       
CAN            No       
eNVM           No       
USB            No
============   =======  =====



Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

   
