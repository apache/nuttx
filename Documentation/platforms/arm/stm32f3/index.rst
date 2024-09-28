==========
ST STM32F3
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
DMA         Yes
EXTI        Yes
ADC         Yes
SDADC       Yes
DAC         Yes
COMP        Yes
OPAMP       Yes
TSC         No
TIM         Yes
HRTIM       Yes
IRTIM       No
IWDG        ?
WWDG        ?
RTC         Yes
I2C         Yes
USART       Yes
SPI         Yes
I2S         ?
CAN         Yes
USB         Yes
HDMI-CEC    No
==========  =======  =====

Memory
------

- CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case)

- CONFIG_RAM_START - The start address of installed DRAM

- CONFIG_STM32_CCMEXCLUDE - Exclude CCM SRAM from the HEAP

- CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
  stack. If defined, this symbol is the size of the interrupt
  stack in bytes.  If not defined, the user task stacks will be
  used during interrupt handling.

- CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

Clock
-----

- CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
  configuration features.::

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

- CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
  of delay loops

Timers
------

Timer devices may be used for different purposes.  One special purpose is
to generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
is defined (as above) then the following may also be defined to indicate that
the timer is intended to be used for pulsed output modulation, ADC conversion,
or DAC conversion. Note that ADC/DAC require two definition:  Not only do you have
to assign the timer (n) for used by the ADC or DAC, but then you also have to
configure which ADC or DAC (m) it is assigned to.

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

CAN character device
--------------------

- CONFIG_CAN - Enables CAN support (one or both of CONFIG_STM32_CAN1 or
  CONFIG_STM32_CAN2 must also be defined)

- CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
  Standard 11-bit IDs.

- CONFIG_CAN_FIFOSIZE - The size of the circular buffer of CAN messages.
  Default: 8

- CONFIG_CAN_NPENDINGRTR - The size of the list of pending RTR requests.
  Default: 4

- CONFIG_CAN_LOOPBACK - A CAN driver may or may not support a loopback
  mode for testing. The STM32 CAN driver does support loopback mode.

- CONFIG_STM32_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN1
  is defined.

- CONFIG_STM32_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN2
  is defined.

- CONFIG_STM32_CAN_TSEG1 - The number of CAN time quanta in segment 1.
  Default: 6

- CONFIG_STM32_CAN_TSEG2 - the number of CAN time quanta in segment 2.
  Default: 7

- CONFIG_STM32_CAN_REGDEBUG - If CONFIG_DEBUG_FEATURES is set, this will generate an
  dump of all CAN registers.

CAN SocketCAN
-------------

TODO

SPI
---

- CONFIG_STM32_SPI_INTERRUPTS - Select to enable interrupt driven SPI
  support. Non-interrupt-driven, poll-waiting is recommended if the
  interrupt rate would be to high in the interrupt driven case.

- CONFIG_STM32_SPIx_DMA - Use DMA to improve SPIx transfer performance.
  Cannot be used with CONFIG_STM32_SPI_INTERRUPT.

USB FS
------

TODO

FPU
===

FPU Configuration Options
-------------------------

There are two version of the FPU support built into the STM32 port.

1. Non-Lazy Floating Point Register Save

   In this configuration floating point register save and restore is
   implemented on interrupt entry and return, respectively.  In this
   case, you may use floating point operations for interrupt handling
   logic if necessary.  This FPU behavior logic is enabled by default
   with::

     CONFIG_ARCH_FPU=y

2. Lazy Floating Point Register Save.

   An alternative mplementation only saves and restores FPU registers only
   on context switches.  This means: (1) floating point registers are not
   stored on each context switch and, hence, possibly better interrupt
   performance.  But, (2) since floating point registers are not saved,
   you cannot use floating point operations within interrupt handlers.

   This logic can be enabled by simply adding the following to your .config
   file::

     CONFIG_ARCH_FPU=y

Flashing and Debugging
======================

NuttX firmware Flashing with STLink probe and OpenOCD::

   openocd -f  interface/stlink.cfg -f target/stm32f3x.cfg -c 'program nuttx.bin 0x08000000; reset run; exit'

Remote target Reset with STLink probe and OpenOCD::

   openocd -f interface/stlink.cfg -f target/stm32f3x.cfg -c 'init; reset run; exit'

Remote target Debug with STLink probe and OpenOCD:

 1. You need to have NuttX built with debug symbols, see :ref:`debugging`.

 2. Launch the OpenOCD GDB server::

     openocd -f interface/stlink.cfg -f target/stm32f3x.cfg -c 'init; reset halt'

 3. You can now attach to remote OpenOCD GDB server with your favorite debugger,
    for instance gdb::

     arm-none-eabi-gdb --tui nuttx -ex 'target extended-remote localhost:3333'
     (gdb) monitor reset halt
     (gdb) breakpoint nsh_main
     (gdb) continue

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
