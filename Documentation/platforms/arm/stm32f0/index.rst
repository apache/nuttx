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
IRQs        Yes
GPIO        Yes
EXTI        Yes
HSE         Yes
PLL         Yes      
HSI         Yes      
MSI         Yes      
LSE         Yes      
RCC         Yes      
SYSCFG      Yes      
USART       Yes
FLASH       No
DMA         Yes
SPI         Yes
I2S         No
I2C         Yes
RTC         No
Timers      Yes
IRTIM       No
PM          No
RNG         Yes
CRC         No
ADC         Yes
DAC         No
COMP        No
WWDG        No
IWDG        No
CAN         No
HDMI-CEC    No
USB         Yes
==========  =======  =====

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
