=============
Renesas RA4M1
=============

Supported MCUs
==============

The following list includes MCUs from RA4M1 series and indicates whether
they are supported in NuttX

=============  ======= ================
MCU            Support Note
=============  ======= ================
R7FA4M1ABxCFP  Yes
R7FA4M1ABxCLJ  No
R7FA4M1ABxCFM  Yes
R7FA4M1ABxCNB  No
R7FA4M1ABxCFL  Yes
R7FA4M1ABxCNE  No
R7FA4M1ABxCNF  No
=============  ======= ================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  =====================================
Peripheral  Support  Notes
==========  =======  =====================================
FLASH       No
CLOCK       Yes      Partially, just internal clock (HOCO)
ICU         Yes
KINT        No
ELC         No
DTC         No
DMAC        No
GPT         Yes      Just PWM
AGT         No
RTC         No
WDT         No
IWDT        No
SCI         Yes      Just UART
IIC         No
SPI         No
SSIE        No
QSPI        No
SDHI        No
CAN         No
USBFS       No
ADC14       No
DAC12       No
DAC8        No
ACMPLP      No
OPAMP       No
TSN         No
SLCDC       No
CTSU        No
CRC         No
DOC         No
GPIO        Yes
==========  =======  =====================================

SCI
----

The Serial Communications Interface (SCI) is configurable to support several serial communication modes: Asynchronous (UART), Clock synchronous, Simple SPI
Smart card interface, Simple IIC (master-only).
Nuttx driver support UART mode (No-FIFO).

GPIO
-----

Pins can be configured/operated using ``ra_gpio_*`` functions.

GPT
----
General Purpose Timer (GPT) is a multi-function timer that can be used in various modes such as PWM output, input capture, output compare, and interval timer.
Nuttx driver support PWM mode.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
