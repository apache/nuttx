README
^^^^^^

This is the README file for the port of NuttX to the Neuros OSD.

CONTENTS
^^^^^^^^
  - Dev vs. Production Neuros OSD v1.0 boards
  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX buildroot Toolchain
  - ARM/DM320-specific Configuration Options
  - Configurations
  - Configuration Options
  - Issues

Dev vs. Production Neuros OSD v1.0 boards
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  This port supports both the original Neuros OSD v1.0 Dev Board.
  This port has recently been extended to V1.0 Production board (and
  that is now the default configuration). References:

    http://www.neurostechnology.com/neuros-developer-community
    http://wiki.neurostechnology.com/index.php/OSD_1.0_Developer_Home
    http://wiki.neurostechnology.com/index.php/DM320_Platform_development

  There are some differences between the Dev Board and the currently
  available commercial v1.0 Boards, most notably in the amount of memory:
  8Mb FLASH and 32Mb RAM vs. 16Mb and 64Mb as in the production board.
  See the following for more information:

     http://wiki.neurostechnology.com/index.php/OSD_Developer_Board_v1

  NuttX operates on the ARM9EJS of this dual core processor.  The DSP
  is available and unused.

  STATUS: This port is code complete, verified, and included in the
  NuttX 0.2.1 release.

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
  3) Set up include paths:  You will need include/, arch/arm/src/dm320,
     arch/arm/src/common, arch/arm/src/arm, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/arm/up_head.S.  You may have to build the NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by the IDE.

NuttX buildroot Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The PATH environment variable should
  be modified to point to the correct path to the ARM926 GCC toolchain (if
  different from the default).

  If you have no ARM toolchain, one can be downloaded from the NuttX
  Bitbucket download site (https://bitbucket.org/nuttx/buildroot/downloads/).

  1. You must have already configured NuttX in <some-dir>nuttx.

     tools/configure.sh ntosd-dm320:<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack

  4. cd <some-dir>/buildroot

  5. cp boards/arm-defconfig .config OR
     cp boards/arm926t_defconfig-4.2.4 .config

  6. make oldconfig

  7. make

  8. Make sure that the PATH variable includes the path to the newly built
     binaries.

ARM/DM320-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_ARM926EJS=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=dm320

    CONFIG_ARCH_CHIP_name - For use in C code

       CONFIG_ARCH_CHIP_DM320

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=ntosd-dm320

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_NTOSD_DM320 (for the Spectrum Digital C5471 EVM)

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM.

    CONFIG_RAM_START - The start address of installed DRAM

    CONFIG_RAM_VSTART - The startaddress of DRAM (virtual)

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
       stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

  DM320 specific device driver settings

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

  DM320 USB Configuration

    CONFIG_DM320_GIO_USBATTACH
       GIO that detects USB attach/detach events
    CONFIG_DM320_GIO_USBDPPULLUP
       GIO
    CONFIG_DMA320_USBDEV_DMA
       Enable DM320-specific DMA support
    CONFIG_DM320_GIO_USBATTACH=6

Configurations
^^^^^^^^^^^^^^

Common Configuration Notes
--------------------------

  1. Each Neuros OSD configuration is maintained in a sub-directory and
     can be selected as follow:

       tools/configure.sh ntosd-dm320:<subdir>

     Where <subdir> is one of the configuration sub-directories described in
     the following paragraph.

  2. These configurations use the mconf-based configuration tool.  To
     change a configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

  3. By default, all configurations assume the ARM EABI toolchain under
     Linux.  This is easily reconfigured:

        CONFIG_HOST_LINUX=y
        CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL=y

Configuration Sub-Directories
-----------------------------

  nettest

    This alternative configuration directory may be used to
    enable networking using the OSDs DM9000A Ethernet interface.
    It uses examples/nettest to exercise the TCP/IP network.

  nsh

    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables both the serial and telnetd NSH interfaces.

  poll

    This configuration exercises the poll()/select() text at
    examples/poll

  udp

    This alternative configuration directory is similar to nettest
    except that it uses examples/udp to exercise UDP.

  webserver

    This configuration file demonstrates the tiny webserver
    at examples/webserver.

Configuration Options
^^^^^^^^^^^^^^^^^^^^^

In additional to the common configuration options listed in the
file boards/README.txt, there are other configuration options
specific to the DM320:

 CONFIG_ARCH - identifies the arch subdirectory and, hence, the
   processor architecture.
 CONFIG_ARCH_name - for use in C code.  This identifies the
   particular chip or SoC that the architecture is implemented
   in.
 CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory
 CONFIG_ARCH_CHIP_name - For use in C code
 CONFIG_ARCH_BOARD - identifies the boards/ subdirectory and, hence,
   the board that supports the particular chip or SoC.
 CONFIG_ENDIAN_BIG - define if big endian (default is little endian)
 CONFIG_ARCH_BOARD_name - for use in C code
 CONFIG_BOARD_LOOPSPERMSEC - for delay loops
 CONFIG_ARCH_LEDS - Use LEDs to show state.
 CONFIG_RAM_SIZE - Describes the internal DRAM.
 CONFIG_RAM_START - The start address of internal DRAM
 CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

DM320 specific device driver settings

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

DM320 USB Configuration

 CONFIG_DM320_GIO_USBATTACH
   GIO that detects USB attach/detach events
 CONFIG_DM320_GIO_USBDPPULLUP
   GIO connected to D+.  Support software connect/disconnect.
 CONFIG_DMA320_USBDEV_DMA
   Enable DM320-specific DMA support

Neuros OSD Configuration Options

 CONFIG_ARCH_NTOSD_DEVBOARD - Selects the old NTOSD development board.
   The default is the production OSD board which differs in
   several ways.

Issues
^^^^^^

  Title:       DEBUG ISSUES
  Description: config/ntos-dm320: It seems that when a lot of debug statements
               are added, the system no longer boots.  This is suspected to be
               a stack problem: Making the stack bigger or removing arrays on
               the stack seems to fix the problem (might also be the
               bootloader overwriting memory)
  Status:      Open
  Priority:    Medium

  Title:       USB DEVICE DRIVER UNTESTED
  Description: A USB device controller driver was added but has never been tested.
  Status:      Open
  Priority:    Medium

  Title:       FRAMEBUFFER DRIVER UNTESTED
  Description: A framebuffer "driver" was added, however, it remains untested.
  Status:      Open
  Priority:    Medium

  Title:       VIDEO ENCODER DRIVER
  Description: In order to use the framebuffer "driver" additional video encoder
               logic is required to setup composite video output or to interface
               with an LCD.
  Status:      Open
  Priority:    Medium (high if you need to use the framebuffer driver)
