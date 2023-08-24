==========
ST STM32F0
==========

Supported MCUs
==============

The following list includes MCUs from STM32F0 series and indicates whether
they are supported in NuttX

=========  ======= ================
MCU        Support Note
=========  ======= ================
STM32F0x0  Yes     Value line
STM32F0x1  Yes     Access line
STM32F0x2  Yes     USB line
STM32F0x8  Yes     Low-voltage line
=========  ======= ================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  =====
Peripheral  Support  Notes
==========  =======  =====
FLASH       No
CRC         No
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
TSC         No
TIM         Yes
IRTIM       No
IWDG        No
WWDG        No
RTC         No
I2C         Yes
USART       Yes
SPI         Yes
I2S         No
CAN         No
USB         Yes
HDMI-CEC    No
==========  =======  =====

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
