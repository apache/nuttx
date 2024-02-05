==========
ST STM32H7
==========

Supported MCUs
==============

Dual-core lines:

===========  ======= ================
MCU          Support Note
===========  ======= ================
STM32H747    Partial Only STM32H747XI
STM32H757    No
STM32H745    Yes
STM32H755    No
===========  ======= ================

Single-core lines:

===========  ======= ================
MCU          Support Note
===========  ======= ================
STM32H7A3    No
STM32H7B3    Partial Only STM32H7B3LI
STM32H743    Yes
STM32H753    Yes
STM32H742    No
STM32H725    No
STM32H735    No
STM32H723    No
STM32H733    No
===========  ======= ================

Value lines:

===========  ======= ================
MCU          Support Note
===========  ======= ================
STM32H7B0    No
STM32H750    No
STM32H730    No
===========  ======= ================


Peripheral Support
==================


The following list indicates peripherals supported in NuttX:

==========  =======  =====
Peripheral  Support  Notes
==========  =======  =====
RAMECC      No
FLASH       Yes
SMM         No
PM          ?
RCC         Yes
CRS         No
HSEM        Yes
GPIO        Yes
SYSCFG      Yes
MDMA        ?
DMA         Yes
BDMA        Yes
DMA2D       Yes
EXTI        Yes
CRC         Yes
FMC         Yes
QUADSPI     Yes
DLYB        No
ADC         Yes
DAC         Yes
VREFBUF     No
COMP        No
OPAMP       No
DFSDM       No
DCMI        No
LTDC        Yes
JPEG        No
RNG         Yes
CRYP        No
HASH        ?
HRTIM       No
TIM         Yes
LPTIM       No
IWDG        Yes
WWDG        Yes
RTC         Yes
I2C         Yes
USART       Yes
SPI         Yes
I2S         ?
SAI         No
SPIDIFRX    No
SWPMI       No
MDIOS       ?
SDMMC       Yes
FDCAN       Yes
OTG_FS      Yes
OTG_HS      Yes
ETH         Yes
HDMI_CEC    No
==========  =======  =====

Dual-core support
=================

Some of the STM32H7 chips have an additional Cortex-M4 core built-in.
The selection of the core for which the image is build is made using options:

  - ``CONFIG_ARCH_CHIP_STM32H7_CORTEXM7`` - selects Cortex-M7 core
  - ``CONFIG_ARCH_CHIP_STM32H7_CORTEXM4`` - selects Cortex-M4 core

Support for the CM7 core is always enabled, support for the CM4 core is controlled
with the ``CONFIG_STM32H7_CORTEXM4_ENABLED`` option.

Interprocessor communication between cores is realized with the NuttX RPTUN
device based on the OpenAMP framework. ``HSEM`` is used for synchronization and
notification between cores.

32kB of the SRAM3 is reserved for shared memory and this is the only available
option at the moment.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
