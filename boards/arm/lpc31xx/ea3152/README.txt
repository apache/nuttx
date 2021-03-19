README
^^^^^^

  This README file discusses the port of NuttX to the Embedded Artists
  EA3152 board.

Contents
^^^^^^^^

  o Development Environment
  o GNU Toolchain Options
  o IDEs
  o NuttX buildroot Toolchain
  o Boot Sequence
  o Image Format
  o Image Download to ISRAM
  o Using OpenOCD and GDB
  o ARM/EA3152-specific Configuration Options
  o Configurations

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The NuttX buildroot Toolchain (see below), or
  2. Any generic arm-none-eabi GNU toolchain.

  All testing has been conducted using the NuttX buildroot toolchain.  To use
  a different toolchain, you simply need to modify the configuration.  As an
  example:

    CONFIG_ARM_TOOLCHAIN_GNU_EABIL : Generic arm-none-eabi toolchain

  Generic arm-none-eabi GNU Toolchain
  -----------------------------------
  There are a number of toolchain projects providing support for ARMv4/v5
  class processors, including:

    GCC ARM Embedded
      https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

  Others exist for various Linux distributions, MacPorts, etc.  Any version
  based on GCC 4.6.3 or later should work.

IDEs
^^^^

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project

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
  3) Set up include paths:  You will need include/, arch/arm/src/lpc31xx,
     arch/arm/src/common, arch/arm/src/arm, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/lpc31xx/lpc31_vectors.S.  You may have to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by an IDE.

NuttX buildroot Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The PATH environment variable should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  Bitbucket download site (https://bitbucket.org/nuttx/buildroot/downloads/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured NuttX in <some-dir>/nuttx.

     tools/configure.sh ea3152:<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp boards/arm926t-defconfig-4.2.4 .config

  6. make oldconfig

  7. make

  8. Make sure that the PATH variable includes the path to the newly built
     binaries.

  See the file boards/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

Boot Sequence
^^^^^^^^^^^^^
  LPC315x has on chip bootrom which loads properly formatted images from multiple
  sources into SRAM.  These sources include including SPI Flash, NOR Flash, UART,
  USB, SD Card, and NAND Flash.

  In all configurations, NuttX is loaded directly into ISRAM.  NuttX is linked
  to execute from ISRAM, regardless of the boot source.

Image Format
^^^^^^^^^^^^

  In order to use the bootrom bootloader, a special header must be added to the
  beginning of the binary image that includes information about the binary (things
  like the entry point, the size, and CRC's to verify the image.

  NXP provides a Windows program to append such a header to the binary image.
  However, (1) that program won't run under Linux, and (2) when I try it under
  WinXP, Symantec immediately claims that the program is misbehaving and deletes
  it!

  To work around both of these issues, I have created a small program under
  boards/arm/lpc31xx/ea3152/tools to add the header.  This program can be built under
  either Linux or Cygwin (and probably other tool environments as well).  That
  tool can be built as follows:

  - cd boards/arm/lpc31xx/ea3152/tools
  - make

  Then, to build the NuttX binary ready to load with the bootloader, just
  following these steps:

  - tools/configure.sh ea3152:ostest # (using the ostest configuration for this example)
  - cd ..                            # Set up environment
  - make                             # Make NuttX.  This will produce nuttx.bin
  - mklpc.sh                         # Make the bootloader binary (nuttx.lpc)

  NOTES:

    1. You will need to set your PATH variable appropriately or use the full path
       to mklpc.sh in the final step.
    2. You can instruct Symantec to ignore the errors and it will stop quarantining
       the NXP program.
    3. The CRC32 logic in boards/arm/lpc31xx/ea3152/tools doesn't seem to work.  As a result,
       the CRC is currently disabled in the header:

       RCS file: /cvsroot/nuttx/nuttx/boards/arm/lpc31xx/ea3152/tools/lpchdr.c,v
       retrieving revision 1.2
       diff -r1.2 lpchdr.c
       264c264
       <   g_hdr.imagetype       = 0x0000000b;
       ---
       >   g_hdr.imagetype       = 0x0000000a;

Image Download to ISRAM
^^^^^^^^^^^^^^^^^^^^^^^

Assuming that you already have the FTDI driver installed*, then here is the
are the steps that I use for loading new code into the EA3152:

- Create the bootloader binary, nuttx.lpc, as described above.
- Connected the EA3152 using the FTDI USB port (not the lpc3152 USB port)
  This will power up the EA3152 and start the bootloader.
- Start a terminal emulator (such as TeraTerm) at 115200 8NI.
- Reset the EA3152 and you should see:
  LPC31xx READY FOR PLAIN IMAGE>
- Send the nuttx.lpc file and you should see:
  Download finished

That will load the NuttX binary into ISRAM and attempt to execute it.

*See the LPC315x documentation if you do not have the FTDI driver installed.

Using OpenOCD and GDB
^^^^^^^^^^^^^^^^^^^^^

  I have been using the Olimex ARM-USB-OCD JTAG debugger with the EA3152
  (http://www.olimex.com).  The OpenOCD configuration file is here:
  tools/armusbocb.cfg.  There is also a script on the tools directory that
  I used to start the OpenOCD daemon on my system called oocd.sh.  That
  script would probably require some modifications to work in another
  environment:

    - possibly the value of OPENOCD_PATH
    - If you are working under Linux you will need to change any
      occurrences of `cygpath -w blablabla` to just blablabla

  Then you should be able to start the OpenOCD daemon like:

    boards/arm/lpc31xx/ea3152/tools/oocd.sh $PWD

  Where it is assumed that you are executing oocd.sh from the top level
  directory where NuttX is installed.

  Once the OpenOCD daemon has been started, you can connect to it via
  GDB using the following GDB command:

   arm-nuttx-elf-gdb
   (gdb) target remote localhost:3333

  And you can load the NuttX ELF file:

   (gdb) symbol-file nuttx
   (gdb) load nuttx

ARM/EA3152-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_ARM926EJS=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lpc31xx

    CONFIG_ARCH_CHIP_name - For use in C code

       CONFIG_ARCH_CHIP_LPC3152

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=ea3152

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_EA3152

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - For most ARM9 architectures, this describes the
      size of installed DRAM.  For the LPC315X, it is used only to
      deterimine how to map the executable regions.  It is SDRAM size
      only if you are executing out of the external SDRAM; or it could
      be NOR FLASH size, external SRAM size, or internal SRAM size.

    CONFIG_RAM_START - The start address of installed DRAM (physical)

    CONFIG_RAM_VSTART - The startaddress of DRAM (virtual)

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
       stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    CONFIG_ARCH_BUTTONS -  Enable support for buttons. Unique to board architecture.

    CONFIG_ARCH_DMA - Support DMA initialization

    CONFIG_ARCH_LOWVECTORS - define if vectors reside at address 0x0000:00000
      Undefine if vectors reside at address 0xffff:0000

    CONFIG_ARCH_ROMPGTABLE - A pre-initialized, read-only page table is available.
      If defined, then board-specific logic must also define PGTABLE_BASE_PADDR,
      PGTABLE_BASE_VADDR, and all memory section mapping in a file named
      board_memorymap.h.

  Individual subsystems can be enabled:

    CONFIG_LPC31_MCI, CONFIG_LPC31_SPI, CONFIG_LPC31_UART

  External memory available on the board (see also CONFIG_MM_REGIONS)

    CONFIG_LPC31_EXTSRAM0 - Select if external SRAM0 is present
    CONFIG_LPC31_EXTSRAM0HEAP - Select if external SRAM0 should be
      configured as part of the NuttX heap.
    CONFIG_LPC31_EXTSRAM0SIZE - Size (in bytes) of the installed
      external SRAM0 memory
    CONFIG_LPC31_EXTSRAM1 - Select if external SRAM1 is present
    CONFIG_LPC31_EXTSRAM1HEAP - Select if external SRAM1 should be
      configured as part of the NuttX heap.
    CONFIG_LPC31_EXTSRAM1SIZE - Size (in bytes) of the installed
      external SRAM1 memory
    CONFIG_LPC31_EXTDRAM - Select if external SDRAM is present
    CONFIG_LPC31_EXTDRAMHEAP - Select if external SDRAM should be
      configured as part of the NuttX heap.
    CONFIG_LPC31_EXTDRAMSIZE - Size (in bytes) of the installed
      external SDRAM memory
    CONFIG_LPC31_EXTNAND - Select if external NAND is present
    CONFIG_LPC31_EXTNANDSIZE - Size (in bytes) of the installed
      external NAND memory

  LPC315X specific device driver settings

    CONFIG_UART_SERIAL_CONSOLE - selects the UART for the
      console and ttys0
    CONFIG_UART_RXBUFSIZE - Characters are buffered as received.
      This specific the size of the receive buffer
    CONFIG_UART_TXBUFSIZE - Characters are buffered before
      being sent.  This specific the size of the transmit buffer
    CONFIG_UART_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_UART_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_UART_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_UART_2STOP - Two stop bits

Configurations
^^^^^^^^^^^^^^

Each EA3152 configuration is maintained in a sub-directory and can be
selected as follow:

    tools/configure.sh ea3152:<subdir>

Where <subdir> is one of the following:

  ostest:
    This configuration directory, performs a simple OS test using
    examples/ostest.  By default, this project assumes that you are
    using the DFU bootloader.
