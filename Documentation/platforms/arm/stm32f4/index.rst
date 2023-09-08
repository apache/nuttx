==========
ST STM32F4
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
DMA2D       Yes
EXTI        Yes
FMC         Yes
QUADSPI     Yes
ADC         Yes
DAC         Yes
DCMI        No
LTDC        Yes
DSI         No
RNG         Yes
CRYP        Yes
HASH        ?
TIM         Yes
IWDG        Yes
WWDG        Yes
RTC         Yes
I2C         Yes
USART       Yes
SPI         Yes
I2S         ?
SAI         No
SDIO        ?
CAN         Yes
OTG_FS      Yes
OTG_HS      Yes
ETH         Yes
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

Clock
-----

- CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
  configuration features.::

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

- CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
  of delay loops

TIMER
-----

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

- CONFIG_STM32_JTAG_FULL_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)

- CONFIG_STM32_JTAG_NOJNTRST_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
  but without JNTRST.

- CONFIG_STM32_JTAG_SW_ENABLE - Set JTAG-DP disabled and SW-DP enabled

USART
-----

- CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=1,2,3) or UART
  m (m=4,5) for the console and ttys0 (default is the USART1).

- CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
  This specific the size of the receive buffer

- CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
  being sent.  This specific the size of the transmit buffer

- CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be

- CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.

- CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity

- CONFIG_U[S]ARTn_2STOP - Two stop bits

CAN
---

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

SPI
---

- CONFIG_STM32_SPI_INTERRUPTS - Select to enable interrupt driven SPI
  support. Non-interrupt-driven, poll-waiting is recommended if the
  interrupt rate would be to high in the interrupt driven case.

- CONFIG_STM32_SPIx_DMA - Use DMA to improve SPIx transfer performance.
  Cannot be used with CONFIG_STM32_SPI_INTERRUPT.

SDIO
----

- CONFIG_SDIO_DMA - Support DMA data transfers.  Requires CONFIG_STM32_SDIO and CONFIG_STM32_DMA2.

- CONFIG_STM32_SDIO_PRI - Select SDIO interrupt priority.  Default: 128

- CONFIG_STM32_SDIO_DMAPRIO - Select SDIO DMA interrupt priority. Default:  Medium

- CONFIG_STM32_SDIO_WIDTH_D1_ONLY - Select 1-bit transfer mode.  Default:
  4-bit transfer mode.

USB
---

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

LTDC hardware acceleration
--------------------------

The LTDC driver provides two 2 LTDC overlays and supports the following hardware
acceleration and features:

Configured at build time:

- background color

- default color (outside visible screen)

Configurable by nuttx framebuffer interface:

- cmap support (color table is shared by both LTDC overlays and DMA2D when enabled)

Configurable via the nuttx framebuffer interface (for each layer separately):

- chromakey

- transparency (const alpha and pixel alpha)

- blank

- color (if DMA2D is enabled and cmap is disabled)

- blit (if DMA2D is enabled)

- blend (if DMA2D is enabled and cmap is disabled)

LTDC overlays are similar to a non-destructive overlay. Both LTDC overlays will
be permanently blended in the order (background -> overlay 0 -> overlay 1) and
converted to a resulting video signal by the LTDC controller. That means each
operation with a LTDC overlay (Overlay 0 and Overlay 1) via nuttx framebuffer
interface will be visible immediately.
Think about continuous blending between both overlays.

DMA2D hardware acceleration
---------------------------

The DMA2D driver implements the following hardware acceleration:

Configurable via the nuttx framebuffer interface:

- cmap support (color table is shared by all DMA2D overlays and LTDC overlays)

Configurable via the nuttx framebuffer interface (for each layer separately):

- color (fill memory region with a specific ARGB8888 color immediately), if
  cmap is disabled
- blit (copy memory region to another memory region with pixel format
  conversion if necessary)
- blend (blend two memory regions and copy the result to a third memory region
  with pixel format conversion if necessary), if cmap is disabled

Blit and blend operation using a fixes memory size defined by the background
layer. DMA2D controller doesn't support scaling.

DMA2D overlays are similar to destructive overlays. They are invisible. They can
be used for image preprocessing. The memory region affected by the operations
(color, blit, blend) can be addressed by the area control command before. The
configured overlay transparency of DMA2D overlays will be used for subsequently
blend operation and is valid for the whole overlay.

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

   An alternative implementation only saves and restores FPU registers only
   on context switches.  This means: (1) floating point registers are not
   stored on each context switch and, hence, possibly better interrupt
   performance.  But, (2) since floating point registers are not saved,
   you cannot use floating point operations within interrupt handlers.

   This logic can be enabled by simply adding the following to your .config file::

     CONFIG_ARCH_FPU=y

Development Environment
=======================

Either Linux or Cygwin on Windows can be used for the development environment.
The source has been built only using the GNU toolchain (see below).  Other
toolchains will likely cause problems.

GNU Toolchain Options
=====================

Toolchain Configurations
------------------------

The NuttX make system has been modified to support the following different
toolchain options.

1. The NuttX buildroot Toolchain (see below), or

2. Any generic arm-none-eabi GNU toolchain.

All testing has been conducted using the NuttX Codesourcery toolchain.  To use
a different toolchain, you simply need to modify the configuration.  As an
example::

    CONFIG_ARM_TOOLCHAIN_GNU_EABI : Generic arm-none-eabi toolchain

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

Using Sourcery CodeBench from http://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench/overview
Download and install the latest version (as of this writing it was sourceryg++-2013.05-64-arm-none-eabi)

Import the  project from git.
File->import->Git-URI, then import a Exiting code as a Makefile progject
from the working directory the git clone was done to.

Select the Sourcery CodeBench for ARM EABI. N.B. You must do one command line
build, before the make will work in CodeBench.

Native Build
------------

Here are a few tips before you start that effort:

1) Select the toolchain that you will be using in your .config file

2) Start the NuttX build at least one time from the Cygwin command line
   before trying to create your project.  This is necessary to create
   certain auto-generated files and directories that will be needed.

3) Set up include paths:  You will need include/, arch/arm/src/stm32,
   arch/arm/src/common, arch/arm/src/armv7-m, and sched/.

4) All assembly files need to have the definition option -D __ASSEMBLY__
   on the command line.

Startup files will probably cause you some headaches.  The NuttX startup file
is arch/arm/src/stm32/stm32_vectors.S.  With RIDE, I have to build NuttX
one time from the Cygwin command line in order to obtain the pre-built
startup object needed by RIDE.

NuttX EABI "buildroot" Toolchain
================================

A GNU GCC-based toolchain is assumed.  The PATH environment variable should
be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
different from the default in your PATH variable).

If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
Bitbucket download site (https://bitbucket.org/nuttx/buildroot/downloads/).
This GNU toolchain builds and executes in the Linux or Cygwin environment.

NXFLAT Toolchain
================

If you are *not* using the NuttX buildroot toolchain and you want to use
the NXFLAT tools, then you will still have to build a portion of the buildroot
tools -- just the NXFLAT tools.  The buildroot with the NXFLAT tools can
be downloaded from the NuttX Bitbucket download site
(https://bitbucket.org/nuttx/nuttx/downloads/).

This GNU toolchain builds and executes in the Linux or Cygwin environment.

1. You must have already configured NuttX in <some-dir>/nuttx.

   tools/configure.sh lpcxpresso-lpc1768:<sub-dir>

2. Download the latest buildroot package into <some-dir>

3. unpack the buildroot tarball.  The resulting directory may
   have versioning information on it like buildroot-x.y.z.  If so,
   rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

4. cd <some-dir>/buildroot

5. cp boards/cortexm3-defconfig-nxflat .config

6. make oldconfig

7. make

8. Make sure that the PATH variable includes the path to the newly built
     NXFLAT binaries.

Protected Mode Build
====================

  The "protected" mode build uses the Cormtex-M4 MPU to separate the FLASH and
  SRAM into kernel-mode and user-mode regions.  The kernel mode regions are
  then protected from any errant or mischievous behavior from user-space
  applications.

  Common notes for all protected mode builds follow:

1. It is recommends to use a special make command; not just 'make' but make
   with the following two arguments::

       make pass1 pass2

   In the normal case (just 'make'), make will attempt to build both user-
   and kernel-mode blobs more or less interleaved.  That actual works!
   However, for me it is very confusing so I prefer the above make command:
   Make the user-space binaries first (pass1), then make the kernel-space
   binaries (pass2)

2. At the end of the build, there will be several files in the top-level
   NuttX build directory::

       PASS1:
         nuttx_user.elf    - The pass1 user-space ELF file
         nuttx_user.hex    - The pass1 Intel HEX format file (selected in defconfig)
         User.map          - Symbols in the user-space ELF file

       PASS2:
         nuttx             - The pass2 kernel-space ELF file
         nuttx.hex         - The pass2 Intel HEX file (selected in defconfig)
         System.map        - Symbols in the kernel-space ELF file

       The J-Link programmer will except files in .hex, .mot, .srec, and .bin
       formats.

3. Combining .hex files.  If you plan to use the .hex files with your
   debugger or FLASH utility, then you may need to combine the two hex
   files into a single .hex file.  Here is how you can do that.

   a. The 'tail' of the nuttx.hex file should look something like this
      (with my comments added)::

          $ tail nuttx.hex
          # 00, data records
          ...
          :10 9DC0 00 01000000000800006400020100001F0004
          :10 9DD0 00 3B005A0078009700B500D400F300110151
          :08 9DE0 00 30014E016D0100008D
          # 05, Start Linear Address Record
          :04 0000 05 0800 0419 D2
          # 01, End Of File record
          :00 0000 01 FF

        Use an editor such as vi to remove the 05 and 01 records.

   b. The 'head' of the nuttx_user.hex file should look something like
        this (again with my comments added)::

          $ head nuttx_user.hex
          # 04, Extended Linear Address Record
          :02 0000 04 0801 F1
          # 00, data records
          :10 8000 00 BD89 01084C800108C8110208D01102087E
          :10 8010 00 0010 00201C1000201C1000203C16002026
          :10 8020 00 4D80 01085D80010869800108ED83010829
          ...

        Nothing needs to be done here.  The nuttx_user.hex file should
        be fine.

   c. Combine the edited nuttx.hex and un-edited nuttx_user.hex
      file to produce a single combined hex file::

        $ cat nuttx.hex nuttx_user.hex >combined.hex

     Then use the combined.hex file with the to write the FLASH image. With
     GDB this would be::

       gdb> mon reset
       gdb> mon halt
       gdb> mon clrbp
       gdb> load combined.hex

     If you do this a lot, you will probably want to invest a little time
     to develop a tool to automate these steps.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
