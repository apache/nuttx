==========
ST STM32H5
==========

This is a port of the STM32H5 family.
The STM32H5 is a chip based on the ARM Cortex-M33.
Most code is adapted from legacy STM32 and STM32H7.

Development primarily using the Nucleo-H563ZI as of Feb 5th, 2025.
Therefore, at this time only the STM32H563 is truly supported. However,
much of the current support should work for all MCUs. Kconfig will need
updates to support MCUs besides the STM32H563.

Supported MCUs
==============

===========  ======= ================
MCU          Support Note
===========  ======= ================
STM32H503     No
STM32H523     No
STM32H533     No
STM32H562     No
STM32H563     Yes
STM32H573     No
===========  ======= ================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  =====
Peripheral  Support  Notes
==========  =======  =====
ADC         Yes
ETH         Yes
DTS         Yes      Software trigger only.
FLASH       Yes      Hardware defines only.
FDCAN       Yes
GPDMA       Yes
GPIO        Yes
I2C         Yes
ICACHE      Yes
RCC         Yes
USART       Yes
LPUART      Yes
OCTOSPI     Yes      Implemented as QSPI.
PWR         Yes      Partial.
SPI         Yes
TIM         Yes
USB_FS      Yes      USB Device and Host Support.

AES         No
CEC         No
CORDIC      No
CRC         No
CRS         No
DAC         No
DBG         No
DCACHE      No
DCMI        No
DLYB        No
EXTI        No
FMAC        No
FSMC        No
GTZC        No
HASH        No
I3C         No
IWDG        No
LPTIM       No
OTFDEC      No
PKA         No
PSSI        No
RAMCFG      No
SBS         No
SDMMC       No
RNG         No
RTC         No
SAES        No
SAI         No
TAMP        No
UCPD        No
VREFBUF     No
WWDG        No

==========  =======  =====

USB FS Host
-----------

STM32 USB FS Host Driver Support. The STM32H5 is equipped with a Dual Role USB device
capable of operating as a device or host. 

Pre-requisites:

- CONFIG_USBHOST         - Enable USB host support
- CONFIG_STM32H5_USBFS_HOST  - Enable the STM32 USB OTG FS block in host mode

USB host requires a stable 48MHz clock. This should come from a PLL driven by the HSE.
HSI48 cannot be reliably used in host mode due to drift. It can only be used in device mode.

Options:

- STM32H5_USBDRD_NCHANNELS - Number of host channels. Default 8

- STM32H5_USBDRD_DESCSIZE - Maximum size of a descriptor.  Default: 128

References
=================
[RM0481] Reference Manual: STM32H523/33xx, STM32H562/63xx, and STM32H573xx Arm® -based 32-bit MCUs

Support
=================

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
