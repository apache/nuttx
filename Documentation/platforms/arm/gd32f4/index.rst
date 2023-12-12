======
GD32F4
======

The devices of GD32F4xx series are 32-bit general-purpose microcontrollers
based on the Cortex-M4 processor. The Cortex-M4 processor includes three
AHB buses known as I-Code, D-Code and System buses. All memory accesses of
the Cortex-M4 processor are executed on the three buses according to the
different purposes and the target memory spaces. The memory organization
uses a Harvard architecture, pre-defined memory map and up to 4 GB of
memory space, making the system flexible and extendable.

Supported MCUs
==============

TODO

Peripheral Support
==================

The following list indicates peripherals now supported
in NuttX:

==========  =======  =====
Peripheral  Support  Notes
==========  =======  =====
SYSCFG      Yes
FMC         Yes
PMU         yes
RCU         Yes      
GPIO        Yes
DMA         Yes
IPA         no
EXTI        Yes
SPI         Yes
TLI         no
I2C         Yes
USART       Yes
I2S         no
SDIO        yes
ENET        Yes
==========  =======  =====

Memory
------

- CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case)

- CONFIG_RAM_START - The start address of installed DRAM

- CONFIG_GD32_TCMEXCLUDE - Exclude TCM SRAM from the HEAP

- CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
  stack. If defined, this symbol is the size of the interrupt
  stack in bytes.  If not defined, the user task stacks will be
  used during interrupt handling.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
