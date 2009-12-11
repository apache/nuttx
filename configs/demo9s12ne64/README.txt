README
^^^^^^

This README discusses issues unique to NuttX configurations for the
Freescale DEMO9S12NE64 development board.

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems.

NuttX buildroot Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the HC12 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no HC12 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/project/showfiles.php?group_id=189573).
  This GNU toolchain builds and executes in the Linux or Cygwin
  environments.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh demo9s12nec64/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/m68hc12-defconfig-3.4.6 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.


HCS12/DEMO9S12NEC64-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
	   be set to:

	   CONFIG_ARCH=hc

	CONFIG_ARCH_family - For use in C code:

	   CONFIG_ARCH_HC=y

	CONFIG_ARCH_architecture - For use in C code:

	   CONFIG_ARCH_HCS12=y

	CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

	   CONFIG_ARCH_CHIP=mc92s12nec64

	CONFIG_ARCH_CHIP_name - For use in C code

	   CONFIG_ARCH_CHIP_MCS92S12NEC64

	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=demo9s12nec64

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_DEMOS92S12NEC64 (for the Spectrum Digital C5471 EVM)

	CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
	   of delay loops

	CONFIG_ENDIAN_BIG - define if big endian (default is little
	   endian)

	CONFIG_DRAM_SIZE - Describes the installed RAM.

	CONFIG_DRAM_START - The start address of installed RAM

	CONFIG_DRAM_END - Should be (CONFIG_DRAM_START+CONFIG_DRAM_SIZE)

	CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
	   have LEDs

	CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
	   stack. If defined, this symbol is the size of the interrupt
	   stack in bytes.  If not defined, the user task stacks will be
	  used during interrupt handling.

	CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

	CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
	   cause a 100 second delay during boot-up.  This 100 second delay
	   serves no purpose other than it allows you to calibratre
	   CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
	   the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
	   the delay actually is 100 seconds.

  HCS12 specific chip initialization

  HCS12 specific device driver settings

	CONFIG_HCS12_SERIALMON - Indicates that the target systems uses
	  the Freescale serial bootloader.

	CONFIG_SCIO_SERIAL_CONSOLE - selects the SCIO for the
	   console and ttys0 (default is the UART0).

	CONFIG_SCIO_RXBUFSIZE - Characters are buffered as received.
	   This specific the size of the receive buffer

	CONFIG_SCIO_TXBUFSIZE - Characters are buffered before
	   being sent.  This specific the size of the transmit buffer

	CONFIG_SCIO_BAUD - The configure BAUD of the UART.

	CONFIG_SCIO_BITS - The number of bits.  Must be either 7 or 8.

	CONFIG_SCIO_PARTIY - 0=no parity, 1=odd parity, 2=even parity, 3=mark 1, 4=space 0

	CONFIG_SCIO_2STOP - Two stop bits

Configurations
^^^^^^^^^^^^^^

Each Freescale HCS12 configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh demo9s12nec64/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

ostest:
  This configuration directory, performs a simple OS test using
  examples/ostest.

