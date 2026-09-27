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
STM32H503     Yes
STM32H523     No
STM32H533     Yes
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
CRC         Yes
CRS         No
DAC         No
DBG         No
DCACHE      No
DCMI        No
DLYB        No
EXTI        Yes
FMAC        No
FSMC        No
GTZC        No
HASH        No
I3C         No
IWDG        Yes
LPTIM       Yes
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
WWDG        Yes
OTP         Yes

==========  =======  =====

USB FS Host
-----------

STM32 USB FS Host Driver Support. The STM32H5 is equipped with a Dual Role USB device
capable of operating as a device or host. 

Pre-requisites:

- CONFIG_USBHOST         - Enable USB host support
- CONFIG_STM32_USBFS_HOST  - Enable the STM32 USB OTG FS block in host mode

USB host requires a stable 48MHz clock. This should come from a PLL driven by the HSE.
HSI48 cannot be reliably used in host mode due to drift. It can only be used in device mode.

Options:

- STM32H5_USBDRD_NCHANNELS - Number of host channels. Default 8

- STM32H5_USBDRD_DESCSIZE - Maximum size of a descriptor.  Default: 128

OTP
---

STM32H5 parts have a 2 KiB one-time programmable (OTP) area. It's organized
into 32 blocks of 32 16-bit words. Each word can be successfully programmed once.
Each block may be permanently locked at any point. Written words may be read.

Writing the same word more than once is unsupported. Doing so may cause corruption.
Reading an unwritten word raises an exception.
To simplify the programming model, the OTP API
locks blocks after any part is written. The user may check which blocks are locked.
Blocks are tagged as "written" using this lock status
without the need for out-of-band metadata. Some of the words within a locked
block may be left unwritten, so the exception may still be raised if an unwritten word
within a locked block is read.

.. code:: c

   int stm32_otp_write(const uint16_t *data, uint16_t len, uint32_t offset);
   int stm32_otp_read(uint16_t *data, uint16_t len, uint32_t offset);
   uint32_t stm32_otp_getlockstatus(void);

The API allows cross-block reads/writes that don't necessarily start/end at block boundaries.
Any block affected by ``stm32_otp_write`` will be locked. The user should be aware
of the block size and count when partitioning the OTP area for their needs.
``len`` is the number of bytes - not words. It has no alignment requirement. ``offset`` is
the offset in bytes. It must be a multiple of 4.

Clocks
------

``STM32_BOARD_HSIKERON_ENABLE`` can be defined in board.h to keep HSI running in
STOP mode. This can be used to keep a peripheral clocked by HSI running in
STOP mode.

``STM32_RCC_CCIPR1_U[S]ARTxSEL`` (e.g. ``STM32_RCC_CCIPR1_USART3SEL``) can be defined as one of

- ``RCC_CCIPR1_U[S]ARTxSEL_RCCPCLK1``
- ``RCC_CCIPR1_U[S]ARTxSEL_PLL2QCK``
- ``RCC_CCIPR1_U[S]ARTxSEL_PLL3QCK``
- ``RCC_CCIPR1_U[S]ARTxSEL_HSIKERCK``
- ``RCC_CCIPR1_U[S]ARTxSEL_CSIKERCK``
- ``RCC_CCIPR1_U[S]ARTxSEL_LSECK``

E.g. ``RCC_CCIPR1_USART3SEL_HSIKERCK`` in board.h to select the clock source for that USART.
The clock source is set in RCC initialization. Only stm32_serial.c is aware of this setting.
TODO: Make stm32_lowputc.c aware of this clock source setting too.

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
