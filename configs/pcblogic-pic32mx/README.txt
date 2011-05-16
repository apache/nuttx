configs/pic32mx README
=====================

This README file discusses the port of NuttX to the PIC32MX board from
PCB Logic Design Co.  This board features the MicroChip PIC32MX460F512L.
The board is a very simple -- little more than a carrier for the PIC32
MCU plus voltage regulation, debug interface, and an OTG connector.

Contents
========

  Toolchains
  PIC32MX Configuration Options
  Configurations

Toolchains
==========

  I am using the free, LITE version of the PIC32MX toolchain available
  for download from the microchip.com web site.  I am using the Windows
  version.  The MicroChip toolchain is the only toolchaing currently
  supported in these configurations, but it should be a simple matter to
  adapt to other toolchains by modifying the Make.defs file include in
  each configuration.

  Toolchain Options:

  CONFIG_PIC32MX_MICROCHIPW      - MicroChip full toolchain for Windows
  CONFIG_PIC32MX_MICROCHIPL      - MicroChip full toolchain for Linux
  CONFIG_PIC32MX_MICROCHIPW_LITE - MicroChip LITE toolchain for Windows
  CONFIG_PIC32MX_MICROCHIPL_LITE - MicroChip LITE toolchain for Linux

  Windows Native Toolchains
  
  NOTE:  There are several limitations to using a Windows based toolchain in a
  Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath' utility
     but you might easily find some new path problems.  If so, check out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic links
     are used in Nuttx (e.g., include/arch).  The make system works around these
     problems for the Windows tools by copying directories instead of linking them.
     But this can also cause some confusion for you:  For example, you may edit
     a file in a "linked" directory and find that your changes had not effect.
     That is because you are building the copy of the file in the "fake" symbolic
     directory.  If you use a Windows toolchain, you should get in the habit of
     making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows pathes which do not
     work with the Cygwin make.

     Support has been added for making dependencies with the windows-native toolchains.
     That support can be enabled by modifying your Make.defs file as follows:

    -  MKDEP                = $(TOPDIR)/tools/mknulldeps.sh
    +  MKDEP                = $(TOPDIR)/tools/mkdeps.sh --winpaths "$(TOPDIR)"

     If you have problems with the dependency build (for example, if you are not
     building on C:), then you may need to modify tools/mkdeps.sh

PIC32MX Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  General Architecture Settings:

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
     be set to:

       CONFIG_ARCH=mips

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_MIPS=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_MIPS32=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=pic32mx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_PIC32MX460F512L=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=pcblogic-pic32mx

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_PCBLOGICPIC32MX=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (CPU SRAM in this case):

       CONFIG_DRAM_SIZE=(32*1024) (32Kb)

       There is an additional 32Kb of SRAM in AHB SRAM banks 0 and 1.

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x10000000

    CONFIG_DRAM_END - Last address+1 of installed RAM

       CONFIG_DRAM_END=(CONFIG_DRAM_START+CONFIG_DRAM_SIZE)

    CONFIG_ARCH_IRQPRIO - The PIC32MXx supports interrupt prioritization

       CONFIG_ARCH_IRQPRIO=y

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
       stack in bytes.  If not defined, the user task stacks will be
       used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
       cause a 100 second delay during boot-up.  This 100 second delay
       serves no purpose other than it allows you to calibratre
       CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
       the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
       the delay actually is 100 seconds.

    Individual subsystems can be enabled:

       CONFIG_PIC32MX_WDT
       CONFIG_PIC32MX_RTCC
       CONFIG_PIC32MX_TIMER1
       CONFIG_PIC32MX_TIMER2
       CONFIG_PIC32MX_TIMER3
       CONFIG_PIC32MX_TIMER4
       CONFIG_PIC32MX_TIMER5
       CONFIG_PIC32MX_IC1
       CONFIG_PIC32MX_IC2
       CONFIG_PIC32MX_IC3
       CONFIG_PIC32MX_IC4
       CONFIG_PIC32MX_IC5
       CONFIG_PIC32MX_OC1
       CONFIG_PIC32MX_OC2
       CONFIG_PIC32MX_OC3
       CONFIG_PIC32MX_OC4
       CONFIG_PIC32MX_OC5
       CONFIG_PIC32MX_I2C1
       CONFIG_PIC32MX_I2C2
       CONFIG_PIC32MX_SPI1
       CONFIG_PIC32MX_SPI2
       CONFIG_PIC32MX_UART1
       CONFIG_PIC32MX_UART2
       CONFIG_PIC32MX_PMP
       CONFIG_PIC32MX_ADC
       CONFIG_PIC32MX_CVR
       CONFIG_PIC32MX_CM1
       CONFIG_PIC32MX_CM2
       CONFIG_PIC32MX_OSC
       CONFIG_PIC32MX_DDP
       CONFIG_PIC32MX_FLASH
       CONFIG_PIC32MX_BMX
       CONFIG_PIC32MX_DMA
       CONFIG_PIC32MX_CHE
       CONFIG_PIC32MX_USB
       CONFIG_PIC32MX_IOPORTA
       CONFIG_PIC32MX_IOPORTB
       CONFIG_PIC32MX_IOPORTC
       CONFIG_PIC32MX_IOPORTD
       CONFIG_PIC32MX_IOPORTE
       CONFIG_PIC32MX_IOPORTF
       CONFIG_PIC32MX_IOPORTG

  PIC32MXx specific device driver settings

    CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn for the
       console and ttys0 (default is the UART0).
    CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_UARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_UARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_UARTn_2STOP - Two stop bits

  PIC32MXx USB Device Configuration

  PIC32MXx USB Host Configuration (the PIC32MX does not support USB Host)

Configurations
^^^^^^^^^^^^^^

Each PIC32MX configuration is maintained in a sudirectory and can be
selected as follow:

    cd tools
    ./configure.sh pcblogic-pic32mx/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  ostest:
    This configuration directory, performs a simple OS test using
    apps/examples/ostest.
