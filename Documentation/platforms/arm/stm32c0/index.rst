==========
ST STM32C0
==========

Supported MCUs
==============

=========  ======= =======================
MCU        Support Note
=========  ======= =======================
STM32C051  Yes     
STM32C071  Yes     USB not supported yet
STM32C091  Yes     
STM32C092  Yes     FDCAN not supported yet
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
CSR         No
GPIO        Yes
SYSCFG      No
DMA         Yes
DMAMUX      Yes
EXTI        Yes
CRC         No
ADC         Yes
TIM         Yes
IRTIM       No
IWDG        No
WWDG        No
I2C         Yes
USART       Yes
SPI         Yes
FDCAN       No
USB         No
==========  =======  =====

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
