==========
ST STM32F2
==========

Supported MCUs
==============

TODO

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  =====
Peripheral  Support  Notes
==========  =======  =====
FLASH       Yes
CRC         Yes
PM          ?
RCC         Yes      
GPIO        Yes
SYSCFG      Yes
EXTI        Yes
DMA         Yes
ADC         Yes
DAC         Yes
DCMI        No
TIM         Yes
IWDG        Yes
WWDG        Yes
CRYP        Yes
RNG         Yes
HASH        ?
RTC         Yes
I2C         Yes
USART       Yes
SPI         Yes
SDIO        Yes
CAN         Yes
ETH         Yes
OTG_FS      Yes
OTG_HS      Yes
FSMC        Yes
==========  =======  =====

Memory
------

- CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case)

- CONFIG_RAM_START - The start address of installed DRAM

In addition to internal SRAM, SRAM may also be available through the FSMC.
In order to use FSMC SRAM, the following additional things need to be
present in the NuttX configuration file:

- CONFIG_STM32_EXTERNAL_RAM - Indicates that SRAM is available via the
  FSMC (as opposed to an LCD or FLASH).

- CONFIG_HEAP2_BASE - The base address of the SRAM in the FSMC address space (hex)

- CONFIG_HEAP2_SIZE - The size of the SRAM in the FSMC address space (decimal)

- CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that have LEDs

- CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
  stack. If defined, this symbol is the size of the interrupt
  stack in bytes.  If not defined, the user task stacks will be
  used during interrupt handling.

- CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

- CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

Clock
-----

- CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
  configuration features.::

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

- CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
  of delay loops

CAN
---

- CONFIG_CAN - Enables CAN support (one or both of CONFIG_STM32_CAN1 or
  CONFIG_STM32_CAN2 must also be defined)

- CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.
  Default Standard 11-bit IDs.

- CONFIG_CAN_FIFOSIZE - The size of the circular buffer of CAN messages.
  Default: 8

- CONFIG_CAN_NPENDINGRTR - The size of the list of pending RTR requests.
  Default: 4

- CONFIG_STM32_CAN1 - Enable support for CAN1

- CONFIG_STM32_CAN2 - Enable support for CAN2

- CONFIG_STM32_CAN1_BAUD - CAN1 BAUD rate.
  Required if CONFIG_STM32_CAN1 is defined.

- CONFIG_STM32_CAN2_BAUD - CAN1 BAUD rate.
  Required if CONFIG_STM32_CAN2 is defined.

- CONFIG_STM32_CAN_TSEG1 - The number of CAN time quanta in segment 1.
  Default: 6

- CONFIG_STM32_CAN_TSEG2 - the number of CAN time quanta in segment 2.
  Default: 7

- CONFIG_STM32_CAN_REGDEBUG - If CONFIG_DEBUG_FEATURES is set, this will generate an
  dump of all CAN registers.

FSMC SRAM
---------

Internal SRAM is available in all members of the STM32 family. In addition
to internal SRAM, SRAM may also be available through the FSMC.  In order to
use FSMC SRAM, the following additional things need to be present in the
NuttX configuration file:

- CONFIG_STM32_FSMC=y - Enables the FSMC
- CONFIG_STM32_EXTERNAL_RAM=y - Indicates that SRAM is available via the
  FSMC (as opposed to an LCD or FLASH).
- CONFIG_HEAP2_BASE - The base address of the SRAM in the FSMC
  address space
- CONFIG_HEAP2_SIZE - The size of the SRAM in the FSMC
  address space
- CONFIG_MM_REGIONS - Must be set to a large enough value to
  include the FSMC SRAM

Timers
------

Timer devices may be used for different purposes.  One special purpose is
to generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
is defined (as above) then the following may also be defined to indicate that
the timer is intended to be used for pulsed output modulation, ADC conversion,
or DAC conversion. Note that ADC/DAC require two definition:  Not only do you have
to assign the timer (n) for used by the ADC or DAC, but then you also have to
configure which ADC or DAC (m) it is assigned to.:

- CONFIG_STM32_TIMn_PWM   Reserve timer n for use by PWM, n=1,..,14
- CONFIG_STM32_TIMn_ADC   Reserve timer n for use by ADC, n=1,..,14
- CONFIG_STM32_TIMn_ADCm  Reserve timer n to trigger ADCm, n=1,..,14, m=1,..,3
- CONFIG_STM32_TIMn_DAC   Reserve timer n for use by DAC, n=1,..,14
- CONFIG_STM32_TIMn_DACm  Reserve timer n to trigger DACm, n=1,..,14, m=1,..,2

For each timer that is enabled for PWM usage, we need the following additional
configuration settings:

- CONFIG_STM32_TIMx_CHANNEL - Specifies the timer output channel {1,..,4}

NOTE: The STM32 timers are each capable of generating different signals on
each of the four channels with different duty cycles.  That capability is
not supported by this driver:  Only one output channel per timer.

JTAG
----

JTAG Enable settings (by default JTAG-DP and SW-DP are disabled):

- CONFIG_STM32_JTAG_FULL_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)

- CONFIG_STM32_JTAG_NOJNTRST_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
  but without JNTRST.

- CONFIG_STM32_JTAG_SW_ENABLE - Set JTAG-DP disabled and SW-DP enabled

USART
-----

Options:

- CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=1,2,3) or UARTm (m=4,5)
  for the console and ttys0 (default is the USART1).

- CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
  This specific the size of the receive buffer

- CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
  being sent.  This specific the size of the transmit buffer

- CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be

- CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.

- CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity

- CONFIG_U[S]ARTn_2STOP - Two stop bits

SPI
---

- CONFIG_STM32_SPI_INTERRUPTS - Select to enable interrupt driven SPI
  support. Non-interrupt-driven, poll-waiting is recommended if the
  interrupt rate would be to high in the interrupt driven case.

- CONFIG_STM32_SPIx_DMA - Use DMA to improve SPIx transfer performance.
  Cannot be used with CONFIG_STM32_SPI_INTERRUPT.

SDIO
----

Options:

- CONFIG_SDIO_DMA - Support DMA data transfers.  Requires CONFIG_STM32_SDIO
  and CONFIG_STM32_DMA2.

- CONFIG_STM32_SDIO_PRI - Select SDIO interrupt priority.  Default: 128

- CONFIG_STM32_SDIO_DMAPRIO - Select SDIO DMA interrupt priority.
  Default:  Medium

- CONFIG_STM32_SDIO_WIDTH_D1_ONLY - Select 1-bit transfer mode.  Default:
  4-bit transfer mode.

ETH
---

Options:

- CONFIG_STM32_PHYADDR - The 5-bit address of the PHY on the board

- CONFIG_STM32_MII - Support Ethernet MII interface

- CONFIG_STM32_MII_MCO1 - Use MCO1 to clock the MII interface

- CONFIG_STM32_MII_MCO2 - Use MCO2 to clock the MII interface

- CONFIG_STM32_RMII - Support Ethernet RMII interface

- CONFIG_STM32_AUTONEG - Use PHY autonegotiation to determine speed and mode

- CONFIG_STM32_ETHFD - If CONFIG_STM32_AUTONEG is not defined, then this
  may be defined to select full duplex mode. Default: half-duplex

- CONFIG_STM32_ETH100MBPS - If CONFIG_STM32_AUTONEG is not defined, then this
  may be defined to select 100 MBps speed.  Default: 10 Mbps

- CONFIG_STM32_PHYSR - This must be provided if CONFIG_STM32_AUTONEG is
  defined.  The PHY status register address may diff from PHY to PHY.  This
  configuration sets the address of the PHY status register.

- CONFIG_STM32_PHYSR_SPEED - This must be provided if CONFIG_STM32_AUTONEG is
  defined.  This provides bit mask indicating 10 or 100MBps speed.

- CONFIG_STM32_PHYSR_100MBPS - This must be provided if CONFIG_STM32_AUTONEG is
  defined.  This provides the value of the speed bit(s) indicating 100MBps speed.

- CONFIG_STM32_PHYSR_MODE - This must be provided if CONFIG_STM32_AUTONEG is
  defined.  This provide bit mask indicating full or half duplex modes.

- CONFIG_STM32_PHYSR_FULLDUPLEX - This must be provided if CONFIG_STM32_AUTONEG is
  defined.  This provides the value of the mode bits indicating full duplex mode.

- CONFIG_STM32_ETH_PTP - Precision Time Protocol (PTP).  Not supported
  but some hooks are indicated with this condition.

USB OTG FS
----------

STM32 USB OTG FS Host Driver Support

Pre-requisites:

- CONFIG_USBHOST      - Enable general USB host support
- CONFIG_STM32_OTGFS  - Enable the STM32 USB OTG FS block
- CONFIG_STM32_SYSCFG - Needed

- CONFIG_STM32_OTGFS_RXFIFO_SIZE - Size of the RX FIFO in 32-bit words.
  Default 128 (512 bytes)

- CONFIG_STM32_OTGFS_NPTXFIFO_SIZE - Size of the non-periodic Tx FIFO
  in 32-bit words.  Default 96 (384 bytes)

- CONFIG_STM32_OTGFS_PTXFIFO_SIZE - Size of the periodic Tx FIFO in 32-bit
  words.  Default 96 (384 bytes)

- CONFIG_STM32_OTGFS_DESCSIZE - Maximum size of a descriptor.  Default: 128

- CONFIG_STM32_OTGFS_SOFINTR - Enable SOF interrupts.  Why would you ever
  want to do that?

- CONFIG_STM32_USBHOST_REGDEBUG - Enable very low-level register access
  debug.  Depends on CONFIG_DEBUG_FEATURES.

- CONFIG_STM32_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
  packets. Depends on CONFIG_DEBUG_FEATURES.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
