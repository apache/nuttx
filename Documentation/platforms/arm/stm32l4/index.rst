==========
ST STM32L4
==========

Supported MCUs
==============

This is a port of NuttX to the STM32L4 Family

Used development boards are the Nucleo L476RG, Nucleo L496ZG,
Nucleo L452RE, Nucleo L432KC, STM32L4VG Discovery and
Motorola MDK.

Most code is copied and adapted from the STM32 and STM32F7 ports.

The various supported STM32L4 families are:


============  ======= ====== ================================
MCU           Support Manual Note
============  ======= ====== ================================
STM32L471xx   No      RM0392
STM32L4X1     Yes     RM0394 Subset of STM32L4_STM32L4X3 [1]
STM32L4X2     Yes     RM0394 Subset of STM32L4_STM32L4X3 [1]
STM32L4X3     Yes     RM0394
STM32L4X5     Yes     RM0351 (was RM0395 in past)
STM32L4X6     Yes     RM0351
STM32L4XR     Yes     RM0432 (STM32L4+)
============  ======= ====== ================================

[1]: Please avoid depending on CONFIG_STM32L4_STM32L4X1 and
CONFIG_STM32L4_STM32L4X2 as the MCUs are of the same subfamily
as CONFIG_STM32L4_STM32L4X3.

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  ==============================
Peripheral  Support  Notes
==========  =======  ==============================
IRQs        Yes
GPIO        Yes
EXTI        Yes
HSI         Yes
HSE         Yes
PLL         Yes      Works @ 80 MHz
MSI         Yes
LSE         Yes
RCC         Yes
SYSCTL      Yes
USART       Yes
DMA         Yes
SRAM2       Yes
SPI         Yes
I2C         Yes
RTC         Yes
QSPI        Yes      
CAN         Yes
OTGFS       Yes 
Timers      Yes
PM          Yes
FSMC        No
AES         No
RNG         Yes
CRC         No       configurable polynomial
WWDG        No
IWDG        Yes
SDMMC       Yes
ADC         Yes
DAC         Yes
DMA2D       No
==========  =======  ==============================

==========  =======  ==============================
Peripheral  Support  Notes
==========  =======  ==============================
FIREWALL    Yes      requires support from ldscript
TSC         No
SWP         No
LPUART      Yes
LPTIM       Yes
OPAMP       No
COMP        Yes
DFSDM       Yes
LCD         No
SAIPLL      Yes
SAI         Yes
HASH        No
DCMI        No
==========  =======  ==============================

New peripherals only in STM32L4+:

==========  =======  ==============================
Peripheral  Support  Notes
==========  =======  ==============================
DMAMUX1     Yes
DSI         No
GFXMMU      No
LTDC        No
OCTOSPI     No
OCTOSPIIOM  No
==========  =======  ==============================

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
