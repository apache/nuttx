==========
ST STM32F7
==========

Supported MCUs
==============

TODO

Peripheral Support
==================

..
   Individual subsystems can be enabled:

       =========================  ==========
       APB1                       Peripheral
       =========================  ==========
       CONFIG_STM32F7_TIM2        TIM2
       CONFIG_STM32F7_TIM3        TIM3
       CONFIG_STM32F7_TIM4        TIM4
       CONFIG_STM32F7_TIM5        TIM5
       CONFIG_STM32F7_TIM6        TIM6
       CONFIG_STM32F7_TIM7        TIM7
       CONFIG_STM32F7_TIM12       TIM12
       CONFIG_STM32F7_TIM13       TIM13
       CONFIG_STM32F7_TIM14       TIM14
       CONFIG_STM32F7_LPTIM1      LPTIM1
       CONFIG_STM32F7_RTC         RTC
       CONFIG_STM32F7_BKP         BKP Registers
       CONFIG_STM32F7_WWDG        WWDG
       CONFIG_STM32F7_IWDG        IWDG
       CONFIG_STM32F7_SPI2        SPI2
       CONFIG_STM32F7_I2S2        I2S2
       CONFIG_STM32F7_SPI3        SPI3
       CONFIG_STM32F7_I2S3        I2S3
       CONFIG_STM32F7_SPDIFRX     SPDIFRX
       CONFIG_STM32F7_USART2      USART2
       CONFIG_STM32F7_USART3      USART3
       CONFIG_STM32F7_UART4       UART4
       CONFIG_STM32F7_UART5       UART5
       CONFIG_STM32F7_I2C1        I2C1
       CONFIG_STM32F7_I2C2        I2C2
       CONFIG_STM32F7_I2C3        I2C3
       CONFIG_STM32F7_I2C4        I2C4
       CONFIG_STM32F7_CAN1        CAN1
       CONFIG_STM32F7_CAN2        CAN2
       CONFIG_STM32F7_HDMICEC     HDMI-CEC
       CONFIG_STM32F7_PWR         PWR
       CONFIG_STM32F7_DAC         DAC
       CONFIG_STM32F7_UART7       UART7
       CONFIG_STM32F7_UART8       UART8
       =========================  ==========

       =========================  ==========
       APB2                       Peripheral
       =========================  ==========
       CONFIG_STM32F7_TIM1        TIM1
       CONFIG_STM32F7_TIM8        TIM8
       CONFIG_STM32F7_USART1      USART1
       CONFIG_STM32F7_USART6      USART6
       CONFIG_STM32F7_ADC         ADC1 - ADC2 - ADC3
       CONFIG_STM32F7_SDMMC1      SDMMC1
       CONFIG_STM32F7_SPI1        SPI1
       CONFIG_STM32F7_SPI4        SPI4
       CONFIG_STM32F7_SYSCFG      SYSCFG
       CONFIG_STM32F7_EXTI        EXTI
       CONFIG_STM32F7_TIM9        TIM9
       CONFIG_STM32F7_TIM10       TIM10
       CONFIG_STM32F7_TIM11       TIM11
       CONFIG_STM32F7_SPI5        SPI5
       CONFIG_STM32F7_SPI6        SPI6
       CONFIG_STM32F7_SAI1        SAI1
       CONFIG_STM32F7_SAI2        SAI2
       CONFIG_STM32F7_LTDC        LCD-TFT
       =========================  ==========

       =========================  ==========
       AHB1                       Peripheral
       =========================  ==========
       CONFIG_STM32F7_CRC         CRC
       CONFIG_STM32F7_BKPSRAM     BKPSRAM
       CONFIG_STM32F7_DMA1        DMA1
       CONFIG_STM32F7_DMA2        DMA2
       CONFIG_STM32F7_ETHMAC      Ethernet MAC
       CONFIG_STM32F7_DMA2D       Chrom-ART (DMA2D)
       CONFIG_STM32F7_OTGHS       USB OTG HS
       =========================  ==========

       =========================  ==========
       AHB2                       Peripheral
       =========================  ==========
       CONFIG_STM32F7_OTGFS       USB OTG FS
       CONFIG_STM32F7_DCMI        DCMI
       CONFIG_STM32F7_CRYP        CRYP
       CONFIG_STM32F7_HASH        HASH
       CONFIG_STM32F7_RNG         RNG
       =========================  ==========

       =========================  ==========
       AHB3                       Peripheral
       =========================  ==========
       CONFIG_STM32F7_FMC         FMC control registers
       CONFIG_STM32F7_QUADSPI     QuadSPI Control
       =========================  ==========

Porting STM32 F4 Drivers
========================

The STM32F746 is very similar to the STM32 F429 and many of the drivers
in the stm32/ directory could be ported here:  ADC, BBSRAM, CAN, DAC,
DMA2D, FLASH, I2C, IWDG, LSE, LSI, LTDC, OTGFS, OTGHS, PM, Quadrature
Encoder, RNG, RTCC, SDMMC (was SDIO), Timer/counters, and WWDG.

Many of these drivers would be ported very simply; many ports would just
be a matter of copying files and some seach-and-replacement.  Like:

1. Compare the two register definitions files; make sure that the STM32
   F4 peripheral is identical (or nearly identical) to the F7
   peripheral.  If so then,

2. Copy the register definition file from the stm32/chip directory to
   the stm32f7/chip directory, making name changes as appropriate and
   updating the driver for any minor register differences.

3. Copy the corresponding C  file (and possibly a matching .h file) from
   the stm32/ directory to the stm32f7/ directory again with naming
   changes and changes for any register differences.

4. Update the Make.defs file to include the new C file in the build.

For other files, particularly those that use DMA, the port will be
significantly more complex.  That is because the STM32F7 has a D-Cache
and, as a result, we need to exercise much more care to maintain cache
coherency.  There is a Wiki page discussing the issues of porting
drivers from the stm32/ to the stm32f7/ directories here:
https://cwiki.apache.org/confluence/display/NUTTX/Porting+Drivers+to+the+STM32+F7

Memory
------

CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case)::

       CONFIG_RAM_SIZE=0x00010000 (64Kb)

CONFIG_RAM_START - The start address of installed SRAM (SRAM1)::

       CONFIG_RAM_START=0x20010000
       CONFIG_RAM_SIZE=245760

This configurations use only SRAM1 for data storage.  The heap includes
the remainder of SRAM1.  If CONFIG_MM_REGIONS=2, then SRAM2 will be
included in the heap.

DTCM SRAM is never included in the heap because it cannot be used for
DMA.  A DTCM allocator is available, however, so that DTCM can be
managed with dtcm_malloc(), dtcm_free(), etc.

In order to use FMC SRAM, the following additional things need to be
present in the NuttX configuration file:

CONFIG_STM32F7_FMC_SRAM - Indicates that SRAM is available via the
FMC (as opposed to an LCD or FLASH).

CONFIG_HEAP2_BASE - The base address of the SRAM in the FMC address space (hex)

CONFIG_HEAP2_SIZE - The size of the SRAM in the FMC address space (decimal)

CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
stack. If defined, this symbol is the size of the interrupt
stack in bytes.  If not defined, the user task stacks will be
used during interrupt handling.

CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

Clock
-----

CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
configuration features.::

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation of delay loops

TIMER
-----

Timer devices may be used for different purposes.  One special purpose is
to generate modulated outputs for such things as motor control.  If CONFIG_STM32F7_TIMn
is defined (as above) then the following may also be defined to indicate that
the timer is intended to be used for pulsed output modulation, ADC conversion,
or DAC conversion. Note that ADC/DAC require two definition:  Not only do you have
to assign the timer (n) for used by the ADC or DAC, but then you also have to
configure which ADC or DAC (m) it is assigned to.::

    CONFIG_STM32F7_TIMn_PWM   Reserve timer n for use by PWM, n=1,..,14
    CONFIG_STM32F7_TIMn_ADC   Reserve timer n for use by ADC, n=1,..,14
    CONFIG_STM32F7_TIMn_ADCm  Reserve timer n to trigger ADCm, n=1,..,14, m=1,..,3
    CONFIG_STM32F7_TIMn_DAC   Reserve timer n for use by DAC, n=1,..,14
    CONFIG_STM32F7_TIMn_DACm  Reserve timer n to trigger DACm, n=1,..,14, m=1,..,2

For each timer that is enabled for PWM usage, we need the following additional
configuration settings::

    CONFIG_STM32F7_TIMx_CHANNEL - Specifies the timer output channel {1,..,4}

NOTE: The STM32 timers are each capable of generating different signals on
each of the four channels with different duty cycles.  That capability is
not supported by this driver:  Only one output channel per timer.

JTAG
----

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

CONFIG_CAN - Enables CAN support (one or both of CONFIG_STM32F7_CAN1 or
CONFIG_STM32F7_CAN2 must also be defined)

CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default Standard 11-bit IDs.

CONFIG_CAN_FIFOSIZE - The size of the circular buffer of CAN messages.  Default: 8

CONFIG_CAN_NPENDINGRTR - The size of the list of pending RTR requests.  Default: 4

CONFIG_CAN_LOOPBACK - A CAN driver may or may not support a loopback
mode for testing. The STM32 CAN driver does support loopback mode.

CONFIG_STM32F7_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32F7_CAN1 is defined.

CONFIG_STM32F7_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32F7_CAN2 is defined.

CONFIG_STM32_CAN_TSEG1 - The number of CAN time quanta in segment 1.  Default: 6

CONFIG_STM32_CAN_TSEG2 - the number of CAN time quanta in segment 2.  Default: 7

CONFIG_STM32_CAN_REGDEBUG - If CONFIG_DEBUG_FEATURES is set, this will generate an
dump of all CAN registers.

SPI
---

CONFIG_STM32F7_SPI_INTERRUPTS - Select to enable interrupt driven SPI
support. Non-interrupt-driven, poll-waiting is recommended if the
interrupt rate would be to high in the interrupt driven case.

CONFIG_STM32F7_SPIx_DMA - Use DMA to improve SPIx transfer performance.
Cannot be used with CONFIG_STM32F7_SPI_INTERRUPT.

DMA
---

CONFIG_SDIO_DMA - Support DMA data transfers.  Requires CONFIG_STM32F7_SDIO and CONFIG_STM32F7_DMA2.

CONFIG_STM32_SDIO_DMAPRIO - Select SDIO DMA interrupt priority. Default:  Medium

CONFIG_STM32_SDIO_WIDTH_D1_ONLY - Select 1-bit transfer mode.  Default: 4-bit transfer mode.

USB
---

STM32 USB OTG FS Host Driver Support

Pre-requisites::

   CONFIG_USBDEV          - Enable USB device support
   CONFIG_USBHOST         - Enable USB host support
   CONFIG_STM32F7_OTGFS     - Enable the STM32 USB OTG FS block
   CONFIG_STM32F7_SYSCFG    - Needed
   CONFIG_SCHED_WORKQUEUE - Worker thread support is required

Options::

   CONFIG_STM32F7_OTGFS_RXFIFO_SIZE - Size of the RX FIFO in 32-bit words.
     Default 128 (512 bytes)
   CONFIG_STM32F7_OTGFS_NPTXFIFO_SIZE - Size of the non-periodic Tx FIFO
     in 32-bit words.  Default 96 (384 bytes)
   CONFIG_STM32F7_OTGFS_PTXFIFO_SIZE - Size of the periodic Tx FIFO in 32-bit
     words.  Default 96 (384 bytes)
   CONFIG_STM32F7_OTGFS_DESCSIZE - Maximum size of a descriptor.  Default: 128
   CONFIG_STM32F7_OTGFS_SOFINTR - Enable SOF interrupts.  Why would you ever
     want to do that?
   CONFIG_STM32F7_USBHOST_REGDEBUG - Enable very low-level register access
     debug.  Depends on CONFIG_DEBUG_FEATURES.
   CONFIG_STM32F7_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
     packets. Depends on CONFIG_DEBUG_FEATURES.

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

SPI Test
========

The builtin SPI test facility can be enabled with the following settings::

    +CONFIG_STM32F7_SPI=y
    +CONFIG_STM32F7_SPI1=y
    +CONFIG_STM32F7_SPI2=y
    +CONFIG_STM32F7_SPI3=y

    +# CONFIG_STM32F7_SPI_INTERRUPTS is not set
    +# CONFIG_STM32F7_SPI1_DMA is not set
    +# CONFIG_STM32F7_SPI2_DMA is not set
    +# CONFIG_STM32F7_SPI3_DMA is not set
     # CONFIG_STM32F7_CUSTOM_CLOCKCONFIG is not set

    +CONFIG_NUCLEO_SPI_TEST=y
    +CONFIG_NUCLEO_SPI_TEST_MESSAGE="Hello World"
    +CONFIG_NUCLEO_SPI1_TEST=y
    +CONFIG_NUCLEO_SPI1_TEST_FREQ=1000000
    +CONFIG_NUCLEO_SPI1_TEST_BITS=8
    +CONFIG_NUCLEO_SPI1_TEST_MODE3=y

    +CONFIG_NUCLEO_SPI2_TEST=y
    +CONFIG_NUCLEO_SPI2_TEST_FREQ=12000000
    +CONFIG_NUCLEO_SPI2_TEST_BITS=8
    +CONFIG_NUCLEO_SPI2_TEST_MODE3=y

    +CONFIG_NUCLEO_SPI3_TEST=y
    +CONFIG_NUCLEO_SPI3_TEST_FREQ=40000000
    +CONFIG_NUCLEO_SPI3_TEST_BITS=8
    +CONFIG_NUCLEO_SPI3_TEST_MODE3=y

    +CONFIG_BOARDCTL=y
    +CONFIG_NSH_ARCHINIT=y

Development Environment
=======================

Either Linux or Cygwin on Windows can be used for the development environment.
The source has been built only using the GNU toolchain (see below).  Other
toolchains will likely cause problems.

All testing has been conducted using the GNU toolchain from ARM for Linux.
found here https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/4.9/4.9-2015-q3-update/+download/gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2

If you change the default toolchain, then you may also have to modify the
PATH environment variable to include the path to the toolchain binaries.

IDEs
====

NuttX is built using command-line make.  It can be used with an IDE, but some
effort will be required to create the project.

Makefile Build
--------------

Under Eclipse, it is pretty easy to set up an "empty makefile project" and
simply use the NuttX makefile to build the system.  That is almost for free
under Linux.  Under Windows, you will need to set up the "Cygwin GCC" empty
makefile project in order to work with Windows (Google for "Eclipse Cygwin" -
there is a lot of help on the internet).

Basic configuration & build steps
==================================

A GNU GCC-based toolchain is assumed.  The PATH environment variable should
be modified to point to the correct path to the Cortex-M7 GCC toolchain (if
different from the default in your PATH variable).

- Configures nuttx creating .config file in the nuttx directory.::

     $ tools/configure.sh nucleo-f746zg:nsh

- Refreshes the .config file with the latest available configurations.::

     $ make oldconfig

- Select the features you want in the build.::

     $ make menuconfig

- Builds NuttX with the features you selected.::

     $ make

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
