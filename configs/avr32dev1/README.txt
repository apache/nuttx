README
^^^^^^

This is the README file for the NuttX port to the Atmel AVR32DEV1 board.

Contents
^^^^^^^^

 * Toolchains
 * Development Environment
 * GNU Toolchains
 * IDEs
 * AVR32 Bootloader
 * AVR32DEV1 Configuration Options
 * Configurations

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchains
^^^^^^^^^^^^^^

The build logic in these directories assume that you are using the GNU
toolchain with the Atmel patches.  The patch file, pre-patched tool sources,
and pre-built binaries are available from the Atmel website.

  CONFIG_AVR32_AVRTOOLSW=y  # Use the windows version
  CONFIG_AVR32_AVRTOOLSL=y  # Ue the Linux version

NOTE: The NuttX builtroot cannot be used to build the AVR32 toolchain.
This is because the Atmel patches that add support for the AVR32 are not
included in the NuttX buildroot.

IDEs
^^^^

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project.
  
  Makefile Build
  --------------
  Under Eclipse, it is pretty easy to set up an "empty makefile project" and
  simply use the NuttX makefile to build the system.  That is almost for free
  under Linux.  Under Windows, you will need to set up the "Cygwin GCC" empty
  makefile project in order to work with Windows (Google for "Eclipse Cygwin" -
  there is a lot of help on the internet).

  Native Build
  ------------
  Here are a few tips before you start that effort:

  1) Select the toolchain that you will be using in your .config file
  2) Start the NuttX build at least one time from the Cygwin command line
     before trying to create your project.  This is necessary to create
     certain auto-generated files and directories that will be needed.
  3) Set up include pathes:  You will need include/, arch/avr/src/at91uc3,
     arch/avr/src/common, arch/arm/src/avr, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/avr/src/avr3/up_nommuhead.S.

AVR32 Bootloader
^^^^^^^^^^^^^^^^

  The linker scripts (ld.script) assume that you are using the bootloader.
  The bootloader resides at 0x8000:0000 and so the ld.script files link
  the application to execute after the bootloader at 0x8000:2000. To link
  so that NuttX boots directly without using the bootloader, change the
  flash definition from:

    flash (rxai!w)  : ORIGIN = 0x80002000, LENGTH = 256K - 8K

  to:
    flash (rxai!w)  : ORIGIN = 0x80000000, LENGTH = 256K

AVR32DEV1 Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
	   be set to:

	   CONFIG_ARCH=avr

	CONFIG_ARCH_family - For use in C code:

	   CONFIG_ARCH_AVR=y

	CONFIG_ARCH_architecture - For use in C code:

	   CONFIG_ARCH_AVR32=y

	CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

	   CONFIG_ARCH_CHIP=at91uc3

	CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
	   chip:

	   CONFIG_ARCH_CHIP_AT91UC3B0256

	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=avr32dev1 (for the AV32DEV1 board)

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_AVR32DEV1

	CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
	   of delay loops

	CONFIG_ENDIAN_BIG - define if big endian (default is little
	   endian)

	CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

	   CONFIG_DRAM_SIZE=0x00010000 (64Kb)

	CONFIG_DRAM_START - The start address of installed DRAM

	   CONFIG_DRAM_START=0x20000000

	CONFIG_DRAM_END - Last address+1 of installed RAM

	   CONFIG_DRAM_END=(CONFIG_DRAM_START+CONFIG_DRAM_SIZE)

	CONFIG_ARCH_IRQPRIO - The AT91UC3B0256 supports interrupt prioritization

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
  
	CONFIG_AVR32_GPIOIRQ - GPIO interrupt support
	CONFIG_AVR32_GPIOIRQSETA - Set of GPIOs on PORTA that support interrupts
	CONFIG_AVR32_GPIOIRQSETB - Set of GPIOs on PORTB that support interrupts

	CONFIG_AVR32_USARTn - Enable support for USARTn
	CONFIG_AVR32_USARTn_RS232 - Configure USARTn as an RS232 interface.
	CONFIG_AVR32_USARTn_SPI - Configure USARTn as an SPI interface.
	CONFIG_AVR32_USARTn_RS485 - Configure USARTn as an RS485 interface.
	CONFIG_AVR32_USARTn_MAN - Configure USARTn as an Manchester interface.
	CONFIG_AVR32_USARTn_MODEM - Configure USARTn as an Modem interface.
	CONFIG_AVR32_USARTn_IRDA - Configure USARTn as an IRDA interface.
	CONFIG_AVR32_USARTn_ISO786 - Configure USARTn as an ISO786 interface.

  AT91UC3B0256 specific device driver settings

	CONFIG_USARTn_SERIAL_CONSOLE - selects the USARTn for the
	   console and ttys0 (default is the USART0).
	CONFIG_USARTn_RXBUFSIZE - Characters are buffered as received.
	   This specific the size of the receive buffer
	CONFIG_USARTn_TXBUFSIZE - Characters are buffered before
	   being sent.  This specific the size of the transmit buffer
	CONFIG_USARTn_BAUD - The configure BAUD of the USART.  Must be
	CONFIG_USARTn_BITS - The number of bits.  Must be either 7 or 8.
	CONFIG_USARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
	CONFIG_USARTn_2STOP - Two stop bits

Configurations
^^^^^^^^^^^^^^

Each Stellaris LM3S6965 Evaluation Kit configuration is maintained in a
sudirectory and can be selected as follow:

	cd tools
	./configure.sh avr32dev1/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

  ostest:
    This configuration directory, performs a simple OS test using
    examples/ostest.


