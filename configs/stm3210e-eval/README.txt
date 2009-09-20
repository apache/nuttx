README
^^^^^^

References:
^^^^^^^^^^

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment because the Raisonance R-Link emulate and some RIDE7 development tools
  were used and those tools works only under Windows.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The CodeSourcery GNU toolchain,
  2. The devkitARM GNU toolchain,
  3. Raisonance GNU toolchain, or
  4. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the NuttX buildroot toolchain.  However,
  the make system is setup to default to use the devkitARM toolchain.  To use
  the CodeSourcery, devkitARM or Raisonance GNU toolchain, you simply need to
  add one of the following configuration options to your .config (or defconfig)
  file:

    CONFIG_STM32_CODESOURCERY=y
    CONFIG_STM32_DEVKITARM=y
    CONFIG_STM32_RAISONANCE=y
    CONFIG_STM32_BUILDROOT=y	(default)

  NOTE: the CodeSourcery, devkitARM, and Raisonance toolchains are Windows native
  toolchains.  The NuttX buildroot toolchain is a Cygwin or Linux native toolchain.
  There are several limitations to using a Windows based toolchain in a Cygwin
  environment.  The three biggest are:

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

  NOTE 1: The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.

NuttX buildroot Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/project/showfiles.php?group_id=189573).
  This GNU toolchain builds and executes in the Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh stm3210e-eval/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-4.3.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

STM3210E-EVAL-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
	   be set to:

	   CONFIG_ARCH=arm

	CONFIG_ARCH_family - For use in C code:

	   CONFIG_ARCH_ARM=y

	CONFIG_ARCH_architecture - For use in C code:

	   CONFIG_ARCH_CORTEXM3=y

	CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

	   CONFIG_ARCH_CHIP=stm32

	CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
	   chip:

	   CONFIG_ARCH_CHIP_STM32F103Z

	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=stm3210e_eval (for the STM3210E-EVAL development board)

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_STM3210E_EVAL=y

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

	CONFIG_ARCH_IRQPRIO - The STM32F103Z supports interrupt prioritization

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

  STM32F103Z specific device driver settings

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

	CONFIG_SSI0_DISABLE - Select to disable support for SSI0
	CONFIG_SSI1_DISABLE - Select to disable support for SSI1
	CONFIG_SSI_POLLWAIT - Select to disable interrupt driven SSI support.
	  Poll-waiting is recommended if the interrupt rate would be to
	  high in the interrupt driven case.
	CONFIG_SSI_TXLIMIT - Write this many words to the Tx FIFO before
	  emptying the Rx FIFO.  If the SPI frequency is high and this
	  value is large, then larger values of this setting may cause
	  Rx FIFO overrun errors.  Default: half of the Tx FIFO size (4).

Configurations
^^^^^^^^^^^^^^

Each STM3210E-EVAL configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh stm3210e-eval/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables both the serial and telnetd NSH interfaces.

  ostest:
    This configuration directory, performs a simple OS test using
    examples/ostest.


