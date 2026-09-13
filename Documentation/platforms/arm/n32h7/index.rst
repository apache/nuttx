============
Nations N32H7
============

Supported MCUs
==============

The following N32H7 series MCUs are supported in NuttX:

============  ======= ================
MCU           Support Note
============  ======= ================
N32H762IIL7   Yes     Cortex-M7 @ 600MHz, 1.5MB SRAM, 2MB Flash
N32H76x       Yes     Currently supported via the N32H762IIL7 BSP
N32H78x       No      (No plan yet)
============  ======= ================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX for N32H7 family:

==========  =======  ======================
Peripheral  Support  Notes
==========  =======  ======================
GPIO        Yes
EXTI        Yes
RCC         Yes
UART        Yes
TIM         Yes
PWM         Yes
ONESHOT     Yes
TICKLESS    Yes
DMA         Yes
USBHS       Yes      Device and Host modes
SDMMC       Yes      ADMA2 supported
CORDIC      Yes
FLASH       Yes      MTD driver
UID         Yes
WDT         No       Not yet ported
RTC         No       Not yet ported
I2C         No       Not yet ported
SPI         No       Not yet ported
ADC         No       Not yet ported
DAC         No       Not yet ported
CAN/FDCAN   No       Not yet ported
ETH         No       Not yet ported
==========  =======  ======================

.. note::
   The SIP Flash on N32H76x has a limitation that prevents effective cache usage.
   Therefore, code running from Flash may have lower performance than expected.
   It is recommended to place critical code in ITCM for better performance.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
