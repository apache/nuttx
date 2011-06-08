README
^^^^^

This is the README file for the port of NuttX to the Micropendous 3 board.
This board is develepmend by http://code.google.com/p/opendous/.  The
Micropendous 3 is based on an Atmel AT90USB646, 647, 1286 or 1287 MCU.
NuttX was ported using the AT90USB647 version.  As of this writing,
documentation for the Micropendous board is available here:
http://code.google.com/p/micropendous/wiki/Micropendous3

Contents
^^^^^^^^

  o Micropendous3 Features
  o Toolchains
  o Windows Native Toolchains
  o NuttX buildroot Toolchain
  o avr-libc
  o Micropendous3 Configuration Options
  o Configurations

Micropendous3 Features
^^^^^^^^^^^^^^^^^^^^^^

  o Based on the 64-pin USB AVR Microcontrollers: AT90USB646, AT90USB647,
    AT90USB1286, or AT90USB1287.
  o USB Full Speed (12Mbit/s)
  o USB Device Mode (Host mode supported with AT90USBxx7 devices)
  o 60kb (AT90USB64) or 120kb (AT90USB128) of available FLASH memory for
    your programs (4kb(AT90USB64)/8kb(AT90USB128) used by USB bootloader -
    stock Atmel or LUFA)
  o 4 kbytes SRAM and 2 kbytes of EEPROM (AT90USB64) or 8 kbytes SRAM and 4
    kbytes of EEPROM (AT90USB128)
  o external SRAM is possible
  o USB powered
  o 16MHz crystal
  o 48 General Purpose IO Pins (47 with external SRAM)
  o Vcc=VBUS jumper selects whether USB VBUS or an external supply is used
    to power the board
  o RESET and HWB buttons to enable firmware loading over USB (no external
    programmer required)
  o HWB can be used as a user button
  o USB-A Plug
  o JTAG header
  o Size LxWxH (including headers): 3.15" x 0.8" x 0.6" =~ 8cm x 2cm x 1.5cm
  o Completely OpenHardware Design

Toolchains
^^^^^^^^^^

Buildroot:

  There is a DIY buildroot version for the AVR boards here:
  http://sourceforge.net/projects/nuttx/files/buildroot/.  See the
  following section for details on building this toolchain.

  It is assumed in some places that buildroot toolchain is available
  at ../misc/buildroot/build_avr.  Edit the setenv.sh file if
  this is not the case.

  After configuring NuttX, make sure that CONFIG_AVR_BUILDROOT=y is set in your
  .config file.

WinAVR:

  For Cygwin development environment on Windows machines, you can use
  WinAVR: http://sourceforge.net/projects/winavr/files/

  It is assumed in some places that WinAVR is installed at C:/WinAVR.  Edit the
  setenv.sh file if this is not the case.

  After configuring NuttX, make sure that CONFIG_AVR_WINAVR=y is set in your
  .config file.

  WARNING:  There is an incompatible version of cygwin.dll in the WinAVR/bin
  directory!  Make sure that the path to the correct cygwin.dll file precedes
  the path to the WinAVR binaries!

Linux:

  For Linux, there are widely available avr-gcc packages.  On Ubuntu, use:
  sudo apt-get install gcc-avr gdb-avr avr-libc

  After configuring NuttX, make sure that CONFIG_AVR_LINUXGCC=y is set in your
  .config file.

Windows Native Toolchains
^^^^^^^^^^^^^^^^^^^^^^^^^

  The WinAVR toolchain is a Windows native toolchain. There are several
  limitations to using a Windows native toolchain in a Cygwin environment. 
  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath'
     utility but you might easily find some new path problems.  If so, check
     out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic
     links are used in Nuttx (e.g., include/arch).  The make system works
     around these  problems for the Windows tools by copying directories
     instead of linking them.  But this can also cause some confusion for
     you:  For example, you may edit a file in a "linked" directory and find
     that your changes had not effect. That is because you are building the
     copy of the file in the "fake" symbolic directory.  If you use a
     Windows toolchain, you should get in the habit of making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This
     is because the dependencies are generated using Windows pathes which do
     not work with the Cygwin make.

     Support has been added for making dependencies with the windows-native
     toolchains.  That support can be enabled by modifying your Make.defs
     file as follows:

    -  MKDEP = $(TOPDIR)/tools/mknulldeps.sh
    +  MKDEP = $(TOPDIR)/tools/mkdeps.sh --winpaths "$(TOPDIR)"

     If you have problems with the dependency build (for example, if you are
     not building on C:), then you may need to modify tools/mkdeps.sh

  An additional issue with the WinAVR toolchain, in particular, is that it
  contains an incompatible version of the Cygwin DLL in its bin/ directory.
  You must take care that the correct Cygwin DLL is used.

NuttX buildroot Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^

  If NuttX buildroot toolchain source tarball cne can be downloaded from the
  NuttX SourceForge download site (https://sourceforge.net/projects/nuttx/files/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh micropendous3/<sub-dir>

	 NOTE: you also must copy avr-libc header files into the NuttX include
	 directory with command perhaps like:

	 cp -a /cygdrive/c/WinAVR/include/avr include/.

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/avr-defconfig-4.5.2 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you
  are building a toolchain for Cygwin under Windows.

avr-libc
^^^^^^^^

Header Files

  In any case, header files from avr-libc are required:  http://www.nongnu.org/avr-libc/.
  A snapshot of avr-lib is included in the WinAVR installation. For Linux
  development platforms, avr-libc package is readily available (and would
  be installed in the apt-get command shown above).  But if you are using
  the NuttX buildroot configuration on Cygwin, then you will have to build
  get avr-libc from binaries.

Header File Installation

  The NuttX build will required that the AVR header files be available via
  the NuttX include directory.  This can be accomplished by either copying
  the avr-libc header files into the NuttX include directory:

  cp -a <avr-libc-path>/include/avr <nuttx-path>/include/.

  Or simply using a symbolic link:

  ln -s <avr-libc-path>/include/avr <nuttx-path>/include/.

Build Notes:

  It may not necessary to have a built version of avr-lib; only header files
  are required.  Bu if you choose to use the optimized libraru functions of
  the flowing point library, then you may have to build avr-lib from sources.
  Below are instructions for building avr-lib from fresh sources:

  1. Download the avr-libc package from: 

     http://savannah.nongnu.org/projects/avr-libc/

     I am using avr-lib-1.7.1.tar.bz2

  2. Upack the tarball and cd into the 
 
     tar jxf avr-lib-1.7.1.tar.bz2
     cd avr-lib-1.7.1

  3. Configure avr-lib.  Assuming that WinAVR is installed at the following
     location:

     export PATH=/cygdrive/c/WinAVR/bin:$PATH
     ./configure --build=`./config.guess` --host=avr

     This takes a *long* time.

  4. Make avr-lib.

     make

     This also takes a long time because it generates variants for nearly
     all AVR chips.

  5. Install avr-lib.

     make install

Micropendous3 Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
	   be set to:

	   CONFIG_ARCH=avr

	CONFIG_ARCH_family - For use in C code:

	   CONFIG_ARCH_AVR=y

	CONFIG_ARCH_architecture - For use in C code:

	   CONFIG_ARCH_AT90USB=y

	CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

	   CONFIG_ARCH_CHIP=at90usb

	CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
	   chip.  This should be exactly one of

	   CONFIG_ARCH_CHIP_AT90USB646=y
	   CONFIG_ARCH_CHIP_AT90USB647=y
	   CONFIG_ARCH_CHIP_AT90USB1286=y
	   CONFIG_ARCH_CHIP_AT90USB1287=y

	   Depending on which Micropendous3 version you have.

	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=micropendous3

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_MICROPENOUS3=y

	CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
	   of delay loops

	CONFIG_ENDIAN_BIG - define if big endian (default is little
	   endian)

	CONFIG_DRAM_SIZE - Describes the installed DRAM.  One of:

	   CONFIG_DRAM_SIZE=(4*1024) - (4Kb)
	   CONFIG_DRAM_SIZE=(8*1024) - (8Kb)

	CONFIG_DRAM_START - The start address of installed DRAM

	   CONFIG_DRAM_START=0x10000000

	CONFIG_DRAM_END - Last address+1 of installed RAM

	   CONFIG_DRAM_END=(CONFIG_DRAM_START+CONFIG_DRAM_SIZE)

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

	  CONFIG_AVR_INT0=n
	  CONFIG_AVR_INT1=n
	  CONFIG_AVR_INT2=n
	  CONFIG_AVR_INT3=n
	  CONFIG_AVR_INT4=n
	  CONFIG_AVR_INT5=n
	  CONFIG_AVR_INT6=n
	  CONFIG_AVR_INT7=n
	  CONFIG_AVR_USBHOST=n
	  CONFIG_AVR_USBDEV=n
	  CONFIG_AVR_WDT=n
	  CONFIG_AVR_TIMER0=n
	  CONFIG_AVR_TIMER1=n
	  CONFIG_AVR_TIMER2=n
	  CONFIG_AVR_TIMER3=n
	  CONFIG_AVR_SPI=n
	  CONFIG_AVR_USART1=y
	  CONFIG_AVR_ANACOMP=n
	  CONFIG_AVR_ADC=n
	  CONFIG_AVR_TWI=n

  AT90USB specific device driver settings

	CONFIG_USARTn_SERIAL_CONSOLE - selects the USARTn for the
	   console and ttys0 (default is no serial console).
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

Each Micropendous3 configuration is maintained in a sudirectory and can
be selected as follow:

	cd tools
	./configure.sh micropendous3/<subdir>
	cd -
	. ./setenv.sh

NOTE: You must also copy avr-libc header files, perhaps like:

	 cp -a /cygdrive/c/WinAVR/include/avr include/.

Where <subdir> is one of the following:

  ostest:
    This configuration directory, performs a simple OS test using
    apps/examples/ostest.
