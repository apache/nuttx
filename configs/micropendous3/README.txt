README
^^^^^

This is the README file for the port of NuttX to the Micropendous 3 board.
This board is develepmend by http://code.google.com/p/opendous/.  The
Micropendous 3 is based on an Atmel AT90USB646, 647, 1286 or 1287 MCU.
NuttX was ported using the AT90USB647 version.  As of this writing,
documentation for the Micropendous board is available here:
http://code.google.com/p/micropendous/wiki/Micropendous3

Micropendous 3 Features
^^^^^^^^^^^^^^^^^^^^^^^

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

Contents
^^^^^^^^

  o Toolchains
  o Windows Native Toolchains
  o avr-libc

Toolchains
^^^^^^^^^^

Buildroot:

  There is a buildroot version for the AVR boards here:
  http://sourceforge.net/projects/nuttx/files/buildroot/.  However, that
  toolchain cannot be recommended at this time because it lacks certain
  important patches.

WinAVR:

  For Cygwin development environment on Windows machines, you can use
  WinAVR: http://sourceforge.net/projects/winavr/files/

  It is assumed in some places that WinAVR is installed at C:/WinAVR.

  After configuring NuttX, make sure that CONFIG_AVR_WINAVR=y is set in your
  .config file.

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

avr-libc
^^^^^^^^

Build Notes:

  In any case, avr-libc is required.  http://www.nongnu.org/avr-libc/.
  An snapshot of avr-lib is included in the WinAVR installation. For Linux
  development platforms, avr-libc package is readily available (and would
  be installed in the apt-get command shown above).

  Below are instructions for building avr-lib from fresh sources (I started
  this before I realized at tha avr-lib is included in the WinAVR install):

  1. Download the avr-libc package from: 

     http://savannah.nongnu.org/projects/avr-libc/

     I am using avr-lib-1.7.1.tar.bz2

  2. Upack the tarball and cd into the 
 
     tar jxf avr-lib-1.7.1.tar.bz2
     cd avr-lib-1.7.1

  3. Configure avr-lib.  Assuming that WinAVR is installed at

     export PATH=/cygdrive/c/WinAVR/bin:$PATH
     ./configure --build=`./config.guess` --host=avr

     This takes a *long* time.

  4. Make avr-lib.

     make

     This also takes a long time because it generates variants for nearly
     all AVR chips.

  5. Install avr-lib.

     make install

Include Path:

  After configuration, the Make.def file installed in the top-level NuttX
  directory will need to be modified to include the path to the where ever
  the include/avr directory was installed (no other avr-libc header files
  are needed).  For, for example, if WinAVR is installed at C:/WinAVR, the
  AVR header files will be at C:/WinAVR/avr/include/avr

  AVRLIBC_INCPATH=${cygpath -u "C:/WinAVR/avr/include/avr"}
