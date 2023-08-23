==========
ST STM32L1
==========

Supported MCUs
==============

TODO

Peripheral Support
==================

TODO

Memory
------

CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

CONFIG_RAM_SIZE=16384 (16Kb)

CONFIG_RAM_START - The start address of installed DRAM

CONFIG_RAM_START=0x20000000

CONFIG_STM32_CCMEXCLUDE - Exclude CCM SRAM from the HEAP

CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
stack. If defined, this symbol is the size of the interrupt
stack in bytes.  If not defined, the user task stacks will be
used during interrupt handling.

Clock
-----

CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
configuration features.::

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

TIMER
-----

Timer devices may be used for different purposes.  One special purpose is
to generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
is defined (as above) then the following may also be defined to indicate that
the timer is intended to be used for pulsed output modulation, ADC conversion,
or DAC conversion. Note that ADC/DAC require two definition:  Not only do you have
to assign the timer (n) for used by the ADC or DAC, but then you also have to
configure which ADC or DAC (m) it is assigned to.

    CONFIG_STM32_TIMn_PWM   Reserve timer n for use by PWM, n=1,..,14
    CONFIG_STM32_TIMn_ADC   Reserve timer n for use by ADC, n=1,..,14
    CONFIG_STM32_TIMn_ADCm  Reserve timer n to trigger ADCm, n=1,..,14, m=1,..,3
    CONFIG_STM32_TIMn_DAC   Reserve timer n for use by DAC, n=1,..,14
    CONFIG_STM32_TIMn_DACm  Reserve timer n to trigger DACm, n=1,..,14, m=1,..,2

For each timer that is enabled for PWM usage, we need the following additional
configuration settings:

    CONFIG_STM32_TIMx_CHANNEL - Specifies the timer output channel {1,..,4}

NOTE: The STM32 timers are each capable of generating different signals on
each of the four channels with different duty cycles.  That capability is
not supported by this driver:  Only one output channel per timer.

JTAG
----

CONFIG_STM32_JTAG_FULL_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)

CONFIG_STM32_JTAG_NOJNTRST_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
but without JNTRST.

CONFIG_STM32_JTAG_SW_ENABLE - Set JTAG-DP disabled and SW-DP enabled


USART
-----

CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=1,2,3) or UART
m (m=4,5) for the console and ttys0 (default is the USART1).

CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
This specific the size of the receive buffer

CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
being sent.  This specific the size of the transmit buffer

CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be

CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.

CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity

CONFIG_U[S]ARTn_2STOP - Two stop bits

CAN
---

CONFIG_CAN - Enables CAN support (one or both of CONFIG_STM32_CAN1 or
CONFIG_STM32_CAN2 must also be defined)

CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
Standard 11-bit IDs.

CONFIG_CAN_FIFOSIZE - The size of the circular buffer of CAN messages.
Default: 8

CONFIG_CAN_NPENDINGRTR - The size of the list of pending RTR requests.
Default: 4

CONFIG_CAN_LOOPBACK - A CAN driver may or may not support a loopback
mode for testing. The STM32 CAN driver does support loopback mode.

CONFIG_STM32_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN1
is defined.

CONFIG_STM32_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN2
is defined.

CONFIG_STM32_CAN_TSEG1 - The number of CAN time quanta in segment 1.
Default: 6

CONFIG_STM32_CAN_TSEG2 - the number of CAN time quanta in segment 2.
Default: 7

CONFIG_STM32_CAN_REGDEBUG - If CONFIG_DEBUG_FEATURES is set, this will generate an
dump of all CAN registers.

SPI
---

CONFIG_STM32_SPI_INTERRUPTS - Select to enable interrupt driven SPI
      support. Non-interrupt-driven, poll-waiting is recommended if the
      interrupt rate would be to high in the interrupt driven case.

CONFIG_STM32_SPIx_DMA - Use DMA to improve SPIx transfer performance.
      Cannot be used with CONFIG_STM32_SPI_INTERRUPT.

SLCD
----

To enable SLCD support::

  Board Selection:
    CONFIG_ARCH_LEDS=n                      : Disable board LED support

  Library Routines:
    CONFIG_LIBC_SLCDCODEC=y                  : Enable the SLCD CODEC

  System Type -> STM32 Peripheral Support:
    CONFIG_STM32_LCD=y                      : Enable the Segment LCD

To enable LCD debug output::

  Device Drivers:
    CONFIG_LCD=y                            : (Needed to enable LCD debug)

  Build Setup -> Debug Options:
    CONFIG_DEBUG_FEATURES=y                 : Enable debug features
    CONFIG_DEBUG_INFO=y                     : Enable LCD debug

NOTE:  At this point in time, testing of the SLCD is very limited because
there is not much in apps/examples/slcd.  Certainly there are more bugs
to be found.  There are also many segment-encoded glyphs in stm32_lcd.c
But there is a basically functional driver with a working test setup
that can be extended if you want a fully functional SLCD driver.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
