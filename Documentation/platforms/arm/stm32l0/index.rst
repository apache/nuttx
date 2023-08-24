==========
ST STM32L0
==========

Supported MCUs
==============

STM32L071, STM32L072 and STM32L073

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  =====
Peripheral  Support  Notes
==========  =======  =====
FLASH       No
CRC         No
FIREWALL    No
PM          No
RCC         Yes      
CSR         No
GPIO        Yes
SYSCFG      Yes      
DMA         Yes
EXTI        Yes
ADC         Yes
DAC         No
COMP        No
LCD         No 
TSC         No
AES         Yes
RNG         Yes
TIM         Yes
LPTIM       No
IWDG        No
WWDG        No
RTC         No
I2C         Yes
USART       Yes
LPUSART     No
SPI         Yes
I2S         No
USB         Yes
==========  =======  =====

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
