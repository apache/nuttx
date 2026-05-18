==========
ST STM32N6
==========

This is a port of NuttX to the STM32N6 family.
The STM32N6 is a chip based on the Arm Cortex-M55.

Development is performed on the Nucleo-N657X0-Q.  At this time only the
STM32N657X0 is supported.  Kconfig will need updates to support other
MCUs in the family.

Supported MCUs
==============

===========  ======= ================
MCU          Support Note
===========  ======= ================
STM32N645     No
STM32N647     No
STM32N655     No
STM32N657     Yes    STM32N657X0 only
===========  ======= ================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  ===============================================
Peripheral  Support  Notes
==========  =======  ===============================================
GPIO        Yes
PWR         Yes      Partial.
RCC         Yes      PLL1 clock tree.
USART       Yes      USART1 only.

ADC         No
DCACHE      No
DCMIPP      No
DMA         No
ETH         No
I2C         No
ICACHE      No
IWDG        No
LPTIM       No
LTDC        No
MPU         No
NPU         No
RNG         No
RTC         No
SAI         No
SDMMC       No
SPI         No
TIM         No
USB         No
XSPI        No
==========  =======  ===============================================

References
==========

[RM0486] STMicroelectronics, STM32N647/657xx Arm®-based 32-bit MCUs

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
