==========
ST STM32G0
==========

Supported MCUs
==============

=========  ======= ================
MCU        Support Note
=========  ======= ================
STM32G0x0  Yes     Value line
STM32G0x1  Yes     Access line
=========  ======= ================

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
SYSCFG      Yes      
DMA         Yes
DMAMUX      Yes
EXTI        Yes
CRC         Yes
ADC         Yes
DAC         No
VREFBUF     ?
COMP        No
RNG         Yes
AES         Yes
TIM         Yes
LPTIM       No
IRTIM       No
IWDG        No
WWDG        No
RTC         No
TAMP        No
I2C         Yes
USART       Yes
LPUSART     No
SPI         Yes
UCPD        No
USB         ?
HDIM_CEC    No
==========  =======  =====

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
