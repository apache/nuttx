==========
ST STM32U3
==========

The STM32U3 family consists of ultra-low-power microcontrollers based on the
Arm Cortex-M33 core.  NuttX currently supports the STM32U3C5 with TrustZone
disabled.

Supported MCUs
==============

===========  =======  =====================
MCU          Support  Notes
===========  =======  =====================
STM32U3C5    Yes      2 MiB Flash, 640 KiB SRAM
===========  =======  =====================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======
Peripheral  Support
==========  =======
RCC         Yes
GPIO        Yes
EXTI        Yes
USART/UART  Yes
LPUART      Yes

ADC         No
ADF         No
AES         No
CCB         No
COMP        No
CRC         No
CRS         No
DAC         No
DBGMCU      No
DLYB        No
FDCAN       No
FLASH       No
GPDMA       No
GTZC        No
HASH        No
HSP         No
I2C         No
I3C         No
ICACHE      No
IWDG        No
LPTIM       No
OCTOSPI     No
OPAMP       No
PKA         No
PWR         No
RAMCFG      No
RNG         No
RTC         No
SAES        No
SAI         No
SDMMC       No
SPI/I2S     No
SYSCFG      No
TAMP        No
TIM         No
TSC         No
USB_DRD_FS  No
VREFBUF     No
WWDG        No
==========  =======

References
==========

[RM0487] STMicroelectronics, STM32U3 series Arm-based 32-bit MCUs.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
