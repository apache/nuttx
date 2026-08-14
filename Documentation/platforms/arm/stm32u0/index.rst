==========
ST STM32U0
==========

Supported MCUs
==============

=========  ======= =======================
MCU        Support Note
=========  ======= =======================
STM32U031  No
STM32U073  Yes     USB not supported yet
STM32U083  Yes     USB not supported yet
=========  ======= =======================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  =====
Peripheral  Support  Notes
==========  =======  =====
FLASH       No
PM          No
RCC         Yes
GPIO        Yes
SYSCFG      No
DMA         Yes
DMAMUX      Yes
EXTI        Yes
CRC         No
ADC         Yes
DAC         No
COMP        No
OPAMP       No
RNG         Yes
AES         Yes
TSC         No
LCD         No
TIM         Yes
LPTIM       No
IWDG        Yes
WWDG        Yes
RTC         No
I2C         Yes
USART       Yes
LPUART      No
SPI         Yes
USB         No
==========  =======  =====

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
