README
^^^^^^

README file for the Microment Eagle100 NuttX port.

References:
^^^^^^^^^^

  Micromint: http://www.micromint.com/
  Luminary:  http://www.luminarymicro.com/

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment because the Luminary FLASH programming application was used for
  writing to FLASH and this application works only under Windows.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the multiple toolchain
  options including:

  1. The CodeSourcery GNU toolchain,
  2. The devkitARM GNU toolchain,
  3. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the NuttX buildroot toolchain.  However,
  the make system is setup to default to use the devkitARM toolchain.  To use
  the CodeSourcery or devkitARM, you simply need to add one of the following
  configuration options to your .config (or defconfig) file:

    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery under Windows
    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYL=y : CodeSourcery under Linux
    CONFIG_ARMV7M_TOOLCHAIN_DEVKITARM=y     : devkitARM under Windows
    CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y     : NuttX buildroot under Linux or Cygwin (default)

  If you are not using CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT, then you may also have to modify
  the PATH in the setenv.h file if your make cannot find the tools.

  NOTE: the CodeSourcery (for Windows) and devkitARM are Windows native toolchains.
  The CodeSourcey (for Linux) and NuttX buildroot toolchains are Cygwin and/or Linux
  native toolchains. There are several limitations to using a Windows based
  toolchain in a Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath' utility
     but you might easily find some new path problems.  If so, check out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic links
     are used in Nuttx (e.g., include/arch).  The make system works around these
     problems for the Windows tools by copying directories instead of linking them.
     But this can also cause some confusion for you:  For example, you may edit
     a file in a "linked" directory and find that your changes had no effect.
     That is because you are building the copy of the file in the "fake" symbolic
     directory.  If you use a Windows toolchain, you should get in the habit of
     making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows pathes which do not
     work with the Cygwin make.

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

  NOTE 1: The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.

NuttX EABI "buildroot" Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh eagle100/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-eabi-defconfig-4.6.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  details PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

  NOTE:  Unfortunately, the 4.6.3 EABI toolchain is not compatible with the
  the NXFLAT tools.  See the top-level TODO file (under "Binary loaders") for
  more information about this problem. If you plan to use NXFLAT, please do not
  use the GCC 4.6.3 EABI toochain; instead use the GCC 4.3.3 OABI toolchain.
  See instructions below.

NuttX OABI "buildroot" Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  The older, OABI buildroot toolchain is also available.  To use the OABI
  toolchain:

    When building the buildroot toolchain, either (1) modify the
    cortexm3-eabi-defconfig-4.6.3 configuration to use EABI (using
    'make menuconfig'), or (2) use an exising OABI configuration such
    as cortexm3-defconfig-4.3.3

NXFLAT Toolchain
^^^^^^^^^^^^^^^^

  If you are *not* using the NuttX buildroot toolchain and you want to use
  the NXFLAT tools, then you will still have to build a portion of the buildroot
  tools -- just the NXFLAT tools.  The buildroot with the NXFLAT tools can
  be downloaded from the NuttX SourceForge download site
  (https://sourceforge.net/projects/nuttx/files/).

  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh lpcxpresso-lpc1768/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-nxflat .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly builtNXFLAT binaries.

Ethernet-Bootloader
^^^^^^^^^^^^^^^^^^^

  Here are some notes about using the Luminary Ethernet boot-loader built
  into the Eagle-100 board.

  Built-In Application:

  - The board has no fixed IP address but uses DHCP to get an address.
    I used a D-link router; I can use a web browser to surf to the D-link
    web page to get the address assigned by

  - Then you can use this IP address in your browser to surf to the Eagle-100
    board.  It presents several interesting pages -- the most important is
    the page called "Firmware Update".  That page includes instructions on
    how to download code to the Eagle-100.

  - After you burn the first program, you lose this application.  Then you
    will probably be better off connected directly to the Eagle-100 board
    or through a switch (The router caused problems for me during downloads).

  Using the Ethernet Bootloader:

  - You will need the "LM Flash Programmer application".  You can get that
    program from the Luminary web site.  There is a link on the LM3S6918 page.

  - Is there any documentation for using the bootloader?  Yes and No:  There
    is an application note covering the bootloader on the Luminary site, but
    it is not very informative.  The Eagle100 User's Manual has the best
    information.

  - Are there any special things I have to do in my code, other than setting
    the origin to 0x0000:2000 (APP_START_ADDRESS)?  No.  The bootloader assumes
    that you have a vector table at that address .  The bootloader does the
    following each time it boots (after you have downloaded the first valid
    application):

    o The bootloader sets the vector table register to the APP_START_ADDRESS,
    o It sets the stack pointer to the address at APP_START_ADDRESS, and then
    o Jumps to the address at APP_START_ADDRESS+4.

  - You can force the bootloader to skip starting the application and stay
    in the update mode.  You will need to do this in order to download a new
    application.  You force the update mode by holding the user button on the
    Eagle-100 board while resetting the board.  The user button is GPIOA, pin 6
    (call FORCED_UPDATE_PIN in the bootloader code).

  - Note 1:  I had to remove my D-Link router from the configuration in order
    to use the LM Flash Programmer (the Bootloader issues BOOTP requests to
    communicate with the LM Flash Programmer, my router was responding to
    these BOOTP requests and hosing the download).  It is safer to connect
    via a switch or via an Ethernet switch.

  - Note 2:  You don't need the router's DHCPD server in the download
    configuration; the Luminary Flash Programmer has the capability of
    temporarily assigning the IP address to the Eagle-100 via BOOTP.

Eagle100-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lm

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LM3S6918

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=eagle100 (for the MicroMint Eagle-100 development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_EAGLE100

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=0x00010000 (64Kb)

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x20000000

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

  There are configurations for disabling support for interrupts GPIO ports.
  GPIOH and GPIOJ must be disabled because they do not exist on the LM3S6918.
  Additional interrupt support can be disabled if desired to reduce memory
  footprint.

    CONFIG_TIVA_DISABLE_GPIOA_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOB_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOC_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOD_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOE_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOF_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOG_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOH_IRQS=y
    CONFIG_TIVA_DISABLE_GPIOJ_IRQS=y

  LM3S6918 specific device driver settings

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

    CONFIG_TIVA_ETHERNET - This must be set (along with CONFIG_NET)
      to build the Stellaris Ethernet driver
    CONFIG_TIVA_ETHLEDS - Enable to use Ethernet LEDs on the board.
    CONFIG_TIVA_BOARDMAC - If the board-specific logic can provide
      a MAC address (via tiva_ethernetmac()), then this should be selected.
    CONFIG_TIVA_ETHHDUPLEX - Set to force half duplex operation
    CONFIG_TIVA_ETHNOAUTOCRC - Set to suppress auto-CRC generation
    CONFIG_TIVA_ETHNOPAD - Set to suppress Tx padding
    CONFIG_TIVA_MULTICAST - Set to enable multicast frames
    CONFIG_TIVA_PROMISCUOUS - Set to enable promiscuous mode
    CONFIG_TIVA_BADCRC - Set to enable bad CRC rejection.
    CONFIG_TIVA_DUMPPACKET - Dump each packet received/sent to the console.

Configurations
^^^^^^^^^^^^^^

Common Configuration Notes
--------------------------

  1. Each Eagle-100 configuration is maintained in a sub-directory and
     can be selected as follow:

       cd tools
       ./configure.sh eagle100/<subdir>
       cd -
       . ./setenv.sh

     Where <subdir> is one of the configuration sub-directories described in
     the following paragraph.

  2. These configurations use the mconf-based configuration tool.  To
     change a configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        and misc/tools/

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

Configuration Sub-Directories
-----------------------------

  nettest:
    This configuration directory may be used to enable networking using the
    LM3S6918's Ethernet controller. It uses examples/nettest to exercise the
    TCP/IP network.

    NOTES:

    1. This configuration is set to use Cygwin under Windows and the
       CodeSourcery toolchain.  That, however, is easily reconfigurable:

         CONFIG_HOST_WINDOWS=y
         CONFIG_WINDOWS_CYGWIN=y
         CONFIG_ARM7M_TOOLCHAIN_CODESOURCERYW=y

  httpd:
    This builds the uIP web server example using the examples/uip application
    (for execution from FLASH).

    NOTES:

    1. This configuration is set to use Cygwin under Windows and the
       CodeSourcery toolchain.  That, however, is easily reconfigurable:

         CONFIG_HOST_WINDOWS=y
         CONFIG_WINDOWS_CYGWIN=y
         CONFIG_ARM7M_TOOLCHAIN_CODESOURCERYW=y

       This example can only be built using the buildroot toolchain
       with NXFLAT support

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables only the serial NSH interfaces (the telnet
    interface should also be functional, but is not enabled in this
    configuration).

    NOTES:

    1. This configuration is set to use Cygwin under Windows and the
       devkitARM toolchain.  That, however, is easily reconfigurable:

       CONFIG_HOST_WINDOWS=y
       CONFIG_WINDOWS_CYGWIN=y
       CONFIG_ARM7M_TOOLCHAIN_DEVKITARM=y

  nxflat:
    This builds the NXFLAT example at apps/examples/nxfalt.

    NOTES:

    1. This example can only be built using the NuttX buildroot
       toolchain with the NXFLAT tools.

    2. This configuration is set to use Cygwin under Windows and the
       devkitARM toolchain.  That, however, is easily reconfigurable:

       CONFIG_HOST_WINDOWS=y
       CONFIG_WINDOWS_CYGWIN=y
       CONFIG_ARM7M_TOOLCHAIN_DEVKITARM=y

  thttpd:
    This builds the THTTPD web server example using the THTTPD and
    the apps/examples/thttpd application.

    NOTES:

    1. This configuration is set to use Linux and the buildroot toolchain.
       That, however, is easily reconfigurable:

         CONFIG_HOST_LINUX=y
         CONFIG_ARM7M_TOOLCHAIN_BUILDROOT=y

       This example can only be built using the buildroot toolchain
       with NXFLAT support

  By default, all of these examples are built to be used with the Luminary
  Ethernet Bootloader (you can change the ld.script file in any of these
  sub-directories to change that configuration).


