==========
ST STM32H5
==========

This is a port of the STM32H5 family.
The STM32H5 is a chip based on the ARM Cortex-M33.
Most code is adapted from legacy STM32 and STM32H7.

Board ports are available for STM32H503, STM32H533, STM32H563 and
STM32H573 devices.  Peripheral support varies by board configuration.

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
STM32H573     Yes
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
ICACHE      Yes      Uses the MPU to keep OTP, RO and EDATA flash uncached.
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

ICACHE
------

With ``CONFIG_STM32_ICACHE`` the OTP, read-only (UID, flash size, package) and
EDATA flash areas (0x08fff000-0x09017fff) are mapped non-cacheable with an MPU
region, so they can be read like any other memory.

The MPU is not applied in the HardFault and NMI handlers (``HFNMIENA=0``), so
these areas must not be read from NMI or HardFault context: with the ICACHE
enabled such a read raises a bus fault.

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

There are two APIs for it: a block-oriented API with locking, and a lower-level
word API. An optional eFuse character device is built on top of the word API.

Block API
~~~~~~~~~

This API is organized into 32 blocks of 32 16-bit words. Each word can be
successfully programmed once. Each block may be permanently locked at any
point. Written words may be read.

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

Word API
~~~~~~~~

``CONFIG_STM32H5_OTP_WORD`` builds direct 16- or 32-bit word access to the
OTP area, independent of the block API above -- there is no locking, and
no relation between a "word" index here and the block API's byte
``offset``:

.. code:: c

   int stm32_otp_word_read16(uint32_t word, uint16_t *value);
   int stm32_otp_word_read32(uint32_t word, uint32_t *value);

Unlike the block API, reading a blank (never programmed) word does not
raise an exception: it legitimately reads back as
``0xffff``/``0xffffffff``, which is returned through ``*value`` either
way. Since whether a word has been written is known, that is reported
through the return value: ``-ENODATA`` for a blank word, ``OK`` for one
that holds real data. ``-EIO`` is returned only when a word's ECC does
not check out at all, i.e. it is neither blank nor the value its own
program operation wrote.

With ``CONFIG_STM32H5_OTP_WRITE`` also set:

.. code:: c

   int stm32_otp_word_write16(uint32_t word, uint16_t value);
   int stm32_otp_word_write32(uint32_t word, uint32_t value);

Each word may be programmed once. Writing a word that already holds
exactly the value requested is a harmless no-op that returns ``OK``.
Writing a word that already holds a different value returns ``-EEXIST``.
A hardware programming failure, or a post-write readback mismatch,
returns ``-EIO``.

eFuse Character Device
~~~~~~~~~~~~~~~~~~~~~~

``CONFIG_STM32H5_EFUSE`` (which selects ``CONFIG_STM32H5_OTP_WORD``)
registers the OTP area as a NuttX efuse character device, by default
``/dev/efuse``, built on the word API above. See
:doc:`/os/drivers/character/efuse` for the ``EFUSEIOC_READ_FIELD``/
``EFUSEIOC_WRITE_FIELD`` ioctl interface. Field bit offsets index into the
flat bit space of the OTP area at 16 bits per word, the same as the word
API's ``word`` index. Writing a field requires ``CONFIG_STM32H5_OTP_WRITE``;
without it, writes are refused with ``-EPERM``.

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
