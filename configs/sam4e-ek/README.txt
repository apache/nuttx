README
======

This README discusses issues unique to NuttX configurations for the Atmel
SAM4E-EK development.  This board features the SAM4E16 MCU running at 96
or 120MHz.

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - Atmel Studio 6.1
  - Loading Code with J-Link
  - Writing to FLASH using SAM-BA
  - LEDs
  - Serial Console
  - Networking Support
  - AT25 Serial FLASH
  - USB Full-Speed Device
  - HSMCI
  - Touchscreen
  - ILI9325-Based LCD
  - SAM4E-EK-specific Configuration Options
  - Configurations

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchain Options
=====================

  The NuttX make system can be configured to support the various different
  toolchain options.  All testing has been conducted using the NuttX buildroot
  toolchain.  To use alternative toolchain, you simply need to add change of
  the following configuration options to your .config (or defconfig) file:

    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_ARMV7M_TOOLCHAIN_ATOLLIC=y        : Atollic toolchain for Windos
    CONFIG_ARMV7M_TOOLCHAIN_DEVKITARM=y      : devkitARM under Windows
    CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)
    CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL=y      : Generic GCC ARM EABI toolchain for Linux
    CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y      : Generic GCC ARM EABI toolchain for Windows

  You may also have to modify the PATH in the setenv.h file if your
  make cannot find the tools.

  NOTE about Windows native toolchains
  ------------------------------------

  There are basically three kinds of GCC toolchains that can be used:

    1. A Linux native toolchain in a Linux environment,
    2. The buildroot Cygwin tool chain built in the Cygwin environment,
    3. A Windows native toolchain.

  There are several limitations to using a Windows based toolchain (#3) in a
  Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath'
     utility but you might easily find some new path problems.  If so, check
     out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic
     links are used in Nuttx (e.g., include/arch).  The make system works
     around these problems for the Windows tools by copying directories
     instead of linking them. But this can also cause some confusion for
     you:  For example, you may edit a file in a "linked" directory and find
     that your changes had no effect.  That is because you are building the
     copy of the file in the "fake" symbolic directory.  If you use a
     Windows toolchain, you should get in the habit of making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This
     is because the dependencies are generated using Windows paths which do
     not work with the Cygwin make.

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

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

  Native Build
  ------------
  Here are a few tips before you start that effort:

  1) Select the toolchain that you will be using in your .config file
  2) Start the NuttX build at least one time from the Cygwin command line
     before trying to create your project.  This is necessary to create
     certain auto-generated files and directories that will be needed.
  3) Set up include pathes:  You will need include/, arch/arm/src/sam34,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/sam34/sam_vectors.S.  You may need to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by an IDE.

NuttX EABI "buildroot" Toolchain
================================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh sam4e-ek/<sub-dir>

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
================================

  The older, OABI buildroot toolchain is also available.  To use the OABI
  toolchain:

  1. When building the buildroot toolchain, either (1) modify the cortexm3-eabi-defconfig-4.6.3
     configuration to use EABI (using 'make menuconfig'), or (2) use an exising OABI
     configuration such as cortexm3-defconfig-4.3.3

  2. Modify the Make.defs file to use the OABI conventions:

    +CROSSDEV = arm-nuttx-elf-
    +ARCHCPUFLAGS = -mtune=cortex-m3 -march=armv7-m -mfloat-abi=soft
    +NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-gotoff.ld -no-check-sections
    -CROSSDEV = arm-nuttx-eabi-
    -ARCHCPUFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
    -NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-pcrel.ld -no-check-sections

NXFLAT Toolchain
================

  If you are *not* using the NuttX buildroot toolchain and you want to use
  the NXFLAT tools, then you will still have to build a portion of the buildroot
  tools -- just the NXFLAT tools.  The buildroot with the NXFLAT tools can
  be downloaded from the NuttX SourceForge download site
  (https://sourceforge.net/projects/nuttx/files/).

  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh sam4e-ek/<sub-dir>

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

Atmel Studio 6.1
================

  You can use Atmel Studio 6.1 to load and debug code.

  - To load code into FLASH:

    Tools menus:  Tools -> Device Programming.

    Configure the debugger and chip and you are in business.

  - Debugging the NuttX Object File:

    1) Rename object file from nutt to nuttx.elf.  That is an extension that
       will be recognized by the file menu.

    2) Select the project name, the full path to the NuttX object (called
       just nuttx with no extension), and chip.  Take the time to resolve
       all of the source file linkages or else you will not have source
       level debug!

       File menu: File -> Open -> Open object file for debugging
       - Select nuttx.elf object file
       - Select AT91SAM4E16
       - Select files for symbols as desired
       - Select debugger

    3) Debug menu: Debug -> Start debugging and break
       - This will reload the nuttx.elf file into FLASH

    STATUS: At this point, Atmel Studio 6.1 claims that my object files are
    not readable.  A little more needs to be done to wring out this procedure.

Loading Code into SRAM with J-Link
==================================

  Loading code with the Segger tools and GDB
  ------------------------------------------

    1) Change directories into the directory where you built NuttX.
    2) Start the GDB server and wait until it is ready to accept GDB
       connections.
    3) Then run GDB like this:

         $ arm-none-eabi-gdb
         (gdb) target remote localhost:2331
         (gdb) mon reset
         (gdb) load nuttx
         (gdb) ... start debugging ...

  Loading code using J-Link Commander
  ----------------------------------

    J-Link> r
    J-Link> loadbin <file> <address>
    J-Link> setpc <address of __start>
    J-Link> ... start debugging ...

  STATUS:  As of this writing, I have not been successful writing to FLASH
  using the GDB server; the write succeeds with no complaints, but the contents
  of the FLASH memory remain unchanged.  This may be because of issues with
  GPNVM1 settings and flash lock bits?  In any event, the GDB server works
  great for debugging after writing the program to FLASH using SAM-BA.

Writing to FLASH using SAM-BA
=============================

  Assumed starting configuration:

    1. You have installed the J-Link USB driver

  Using SAM-BA to write to FLASH:

    1. Start the SAM-BA application, selecting (1) the SAM-ICE/J-Link
       port, and (2) board = at91sam4e16-ek.
    2. The SAM-BA menu should appear.
    3. Select the FLASH tab and enable FLASH access
    4. "Send" the file to flash
    5. Enable "Boot from Flash (GPNVM1)
    6. Reset the board.

  STATUS: Works great!

LEDs
====

  The SAM4E-EK board has three, user-controllable LEDs labelled D2 (blue),
  D3 (amber), and D4 (green) on the board.  Usage of these LEDs is defined
  in include/board.h and src/up_leds.c. They are encoded as follows:

    SYMBOL              Meaning                 D3*     D2      D4
    ------------------- ----------------------- ------- ------- -------
    LED_STARTED         NuttX has been started  OFF     OFF     OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF     OFF     ON
    LED_IRQSENABLED     Interrupts enabled      OFF     ON      OFF
    LED_STACKCREATED    Idle stack created      OFF     ON      ON
    LED_INIRQ           In an interrupt**       N/C     FLASH   N/C
    LED_SIGNAL          In a signal handler***  N/C     N/C     FLASH
    LED_ASSERTION       An assertion failed     FLASH   N/C     N/C
    LED_PANIC           The system has crashed  FLASH   N/C     N/C

  * If D2 and D4 are statically on, then NuttX probably failed to boot
    and these LEDs will give you some indication of where the failure was
 ** The normal state is D3=OFF, D4=ON and D2 faintly glowing.  This faint
    glow is because of timer interrupts that result in the LED being
    illuminated on a small proportion of the time.
*** D4 may also flicker normally if signals are processed.

Serial Console
==============

  By default, all of these configurations use UART0 for the NuttX serial
  console.  UART0 corresponds to the DB-9 connector J17 labelled "DBGU".
  This is a male connector and will require a female-to-female, NUL modem
  cable to connect to a PC.

  An alternate is USART1 which connects to the other DB-9 connector labelled
  "USART1".  USART1 is not enabled by default unless specifically noted
  otherwise in the configuration description.  A NUL modem cable must be
  used with the port as well.

  NOTE:  To avoid any electrical conflict, the RS232 and RS485 transceiver
  are isolated from the receiving line PA21.

  - Chose RS485 channel: Close 1-2 pins on JP11 and set PA23 to high level
  - Chose RS232 channel: Close 2-3 pins on JP11 and set PA23 to low level

  By default serial console is configured for 115000, 8-bit, 1 stop bit, and
  no parity.

Networking Support
==================

  Networking support via the can be added to NSH by selecting the following
  configuration options.

  Selecting the EMAC peripheral
  -----------------------------

  System Type -> SAM34 Peripheral Support
    CONFIG_SAM34_EMAC=y                 : Enable the EMAC peripheral

  System Type -> EMAC device driver options
    CONFIG_SAM34_EMAC_NRXBUFFERS=16     : Set aside some RS and TX buffers
    CONFIG_SAM34_EMAC_NTXBUFFERS=4
    CONFIG_SAM34_EMAC_PHYADDR=1         : KSZ8051 PHY is at address 1
    CONFIG_SAM34_EMAC_AUTONEG=y         : Use autonegotiation
    CONFIG_SAM34_EMAC_MII=y             : Only the MII interface is supported
    CONFIG_SAM34_EMAC_PHYSR=30          : Address of PHY status register on KSZ8051
    CONFIG_SAM34_EMAC_PHYSR_ALTCONFIG=y : Needed for KSZ8051
    CONFIG_SAM34_EMAC_PHYSR_ALTMODE=0x7 : "    " " " "     "
    CONFIG_SAM34_EMAC_PHYSR_10HD=0x1    : "    " " " "     "
    CONFIG_SAM34_EMAC_PHYSR_100HD=0x2   : "    " " " "     "
    CONFIG_SAM34_EMAC_PHYSR_10FD=0x5    : "    " " " "     "
    CONFIG_SAM34_EMAC_PHYSR_100FD=0x6   : "    " " " "     "

  PHY selection.  Later in the configuration steps, you will need to select
  the KSZ8051 PHY for EMAC (See below)

  Networking Support
    CONFIG_NET=y                        : Enable Neworking
    CONFIG_NET_SOCKOPTS=y               : Enable socket operations
    CONFIG_NET_BUFSIZE=562              : Maximum packet size (MTD) 1518 is more standard
    CONFIG_NET_RECEIVE_WINDOW=536       : Should be the same as CONFIG_NET_BUFSIZE
    CONFIG_NET_TCP=y                    : Enable TCP/IP networking
    CONFIG_NET_TCPBACKLOG=y             : Support TCP/IP backlog
    CONFIG_NET_TCP_READAHEAD_BUFSIZE=536  Read-ahead buffer size
    CONFIG_NET_UDP=y                    : Enable UDP networking
    CONFIG_NET_BROADCAST=y              : Needed for DNS name resolution
    CONFIG_NET_ICMP=y                   : Enable ICMP networking
    CONFIG_NET_ICMP_PING=y              : Needed for NSH ping command
                                        : Defaults should be okay for other options
  Device drivers -> Network Device/PHY Support
    CONFIG_NETDEVICES=y                 : Enabled PHY selection
    CONFIG_ETH0_PHY_KSZ8051=y           : Select the KSZ8051 PHY (for EMAC)

  Application Configuration -> Network Utilities
    CONFIG_NETUTILS_DNSCLIENT=y            : Enable host address resolution
    CONFIG_NETUTILS_TELNETD=y           : Enable the Telnet daemon
    CONFIG_NETUTILS_TFTPC=y             : Enable TFTP data file transfers for get and put commands
    CONFIG_NETUTILS_UIPLIB=y            : Network library support is needed
    CONFIG_NETUTILS_WEBCLIENT=y         : Needed for wget support
                                        : Defaults should be okay for other options
  Application Configuration -> NSH Library
    CONFIG_NSH_TELNET=y                 : Enable NSH session via Telnet
    CONFIG_NSH_IPADDR=0x0a000002        : Select a fixed IP address
    CONFIG_NSH_DRIPADDR=0x0a000001      : IP address of gateway/host PC
    CONFIG_NSH_NETMASK=0xffffff00       : Netmask
    CONFIG_NSH_NOMAC=y                  : Need to make up a bogus MAC address
                                        : Defaults should be okay for other options

  You can also enable enable the DHCPC client for networks that use
  dynamically assigned address:

  Application Configuration -> Network Utilities
    CONFIG_NETUTILS_DHCPC=y             : Enables the DHCP client

  Networking Support
    CONFIG_NET_UDP=y                    : Depends on broadcast UDP

  Application Configuration -> NSH Library
    CONFIG_NET_BROADCAST=y
    CONFIG_NSH_DHCPC=y                  : Tells NSH to use DHCPC, not
                                        : the fixed addresses

  Using the network with NSH
  --------------------------

  So what can you do with this networking support?  First you see that
  NSH has several new network related commands:

    ifconfig, ifdown, ifup:  Commands to help manage your network
    get and put:             TFTP file transfers
    wget:                    HTML file transfers
    ping:                    Check for access to peers on the network
    Telnet console:          You can access the NSH remotely via telnet.

  You can also enable other add on features like full FTP or a Web
  Server or XML RPC and others.  There are also other features that
  you can enable like DHCP client (or server) or network name
  resolution.

  By default, the IP address of the SAM4E-EK will be 10.0.0.2 and
  it will assume that your host is the gateway and has the IP address
  10.0.0.1.

    nsh> ifconfig
    eth0    HWaddr 00:e0:de:ad:be:ef at UP
            IPaddr:10.0.0.2 DRaddr:10.0.0.1 Mask:255.255.255.0

  You can use ping to test for connectivity to the host (Careful,
  Window firewalls usually block ping-related ICMP traffic).  On the
  target side, you can:

    nsh> ping 10.0.0.1
    PING 10.0.0.1 56 bytes of data
    56 bytes from 10.0.0.1: icmp_seq=1 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=2 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=3 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=4 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=5 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=6 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=7 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=8 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=9 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=10 time=0 ms
    10 packets transmitted, 10 received, 0% packet loss, time 10100 ms

  NOTE: In this configuration is is normal to have packet loss > 0%
  the first time you ping due to the default handling of the ARP
  table.

  On the host side, you should also be able to ping the SAM4E-EK:

    $ ping 10.0.0.2

  You can also log into the NSH from the host PC like this:

    $ telnet 10.0.0.2
    Trying 10.0.0.2...
    Connected to 10.0.0.2.
    Escape character is '^]'.
    sh_telnetmain: Session [3] Started

    NuttShell (NSH) NuttX-6.31
    nsh> help
    help usage:  help [-v] [<cmd>]

      [           echo        ifconfig    mkdir       mw          sleep
      ?           exec        ifdown      mkfatfs     ping        test
      cat         exit        ifup        mkfifo      ps          umount
      cp          free        kill        mkrd        put         usleep
      cmp         get         losetup     mh          rm          wget
      dd          help        ls          mount       rmdir       xd
      df          hexdump     mb          mv          sh

    Builtin Apps:
    nsh>

  NOTE:  If you enable this feature, you experience a delay on booting.
  That is because the start-up logic waits for the network connection
  to be established before starting NuttX.  In a real application, you
  would probably want to do the network bringup on a separate thread
  so that access to the NSH prompt is not delayed.

  This delay will be especially long if the board is not connected to
  a network because additional time will be required to fail with timeout
  errors.

AT25 Serial FLASH
=================

  Connections
  -----------

  Both the SAM4E-EK include an Atmel AT25DF321A, 32-megabit, 2.7-volt
  SPI serial flash.  The SPI
  connection is as follows:

    ------ ------- ---------------
    SAM4E  AT25    SAM4E
    GPIO   PIN     FUNCTION
    ------ ------- ---------------
    PA13   SI      MOSI
    PA12   SO      MIS0
    PA14   SCK     SPCK
    PA5    /CS     NPCS3 (pulled high externally)
    ------ ------- ---------------

  Configuration
  -------------

  Support for the serial FLASH can be enabled in these configurations.  These
  are the relevant configuration settings.  These settings (1) Enable SPI0,
  (2) Enable DMAC0 to support DMA transfers on SPI for best performance,
  (3) Enable the AT25 Serial FLASH, and (3) Set up NuttX to configure the
  file system on the AT25 FLASH:

    System Type -> ATSAM3/4 Peripheral Support
      CONFIG_SAM34_SPI0=y                   : Enable SPI0
      CONFIG_SAM34_DMAC0=y                  : Enable DMA controller 0

    System Type -> SPI device driver options
      CONFIG_SAM34_SPI_DMA=y                : Use DMA for SPI transfers
      CONFIG_SAM34_SPI_DMATHRESHOLD=4       : Don't DMA for small transfers

    Device Drivers -> SPI Driver Support
      CONFIG_SPI=y                          : Enable SPI support
      CONFIG_SPI_EXCHANGE=y                 : Support the exchange method

    Device Drivers -> Memory Technology Device (MTD) Support
      CONFIG_MTD=y                          : Enable MTD support
      CONFIG_MTD_AT25=y                     : Enable the AT25 driver
      CONFIG_AT25_SPIMODE=0                 : Use SPI mode 0
      CONFIG_AT25_SPIFREQUENCY=20000000     : Use SPI frequency 12MHz

    The AT25 is capable of operation at 20MHz.  However, if you experience
    any issues with the AT25, then lower this frequency may give more
    predictable performance.

    File Systems -> FAT
      CONFIG_FS_FAT=y                       : Enable and configure FAT
      CONFIG_FAT_LCNAMES=y                  : Upper/lower case names
      CONFIG_FAT_LFN=y                      : Long file name support (See NOTE)
      CONFIG_FAT_MAXFNAME=32                : Limit filename sizes to 32 bytes

    NOTE: Use care if you plan to use FAT long file name feature in a product;
    There are issues with certain Microsoft patents on the long file name
    technology.

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

    Board Selection
      CONFIG_SAM4EEK_AT25_AUTOMOUNT=y       : Mounts AT25 for NSH
      CONFIG_SAM4EEK_AT25_FTL=y             : Create block driver for FAT

  You can then format the AT25 FLASH for a FAT file system and mount the
  file system at /mnt/at25 using these NSH commands:

    nsh> mkfatfs /dev/mtdblock0
    nsh> mount -t vfat /dev/mtdblock0 /mnt/at25

  Then you an use the FLASH as a normal FAT file system:

    nsh> echo "This is a test" >/mnt/at25/atest.txt
    nsh> ls -l /mnt/at25
    /mnt/at25:
     -rw-rw-rw-      16 atest.txt
    nsh> cat /mnt/at25/atest.txt
    This is a test

USB Full-Speed Device
=====================

  Basic USB Full-Speed Device Configuration
  -----------------------------------------

  Support the USB full-speed device (UDP) driver can be enabled with these
  NuttX configuration settings.

    Device Drivers -> USB Device Driver Support
      CONFIG_USBDEV=y                       : Enable USB device support
      CONFIG_USBDEV_DUALSPEED=n             : Device does not support High-Speed
      CONFIG_USBDEV_DMA=n                   : Device does not use DMA

    System Type -> ATSAM3/4 Peripheral Support
      CONFIG_SAM34_UDP=y                    : Enable UDP Full Speed USB device

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

  Mass Storage Class
  ------------------

  The Mass Storage Class (MSC) class driver can be selected for use with
  UDP.  Note: The following assumes that the internal AT25 Serial FLASH
  is configured to support a FAT file system through an FTL layer as
  described about under "AT25 Serial FLASH".

    Device Drivers -> USB Device Driver Support
      CONFIG_USBMSC=y                       : Enable the USB MSC class driver
      CONFIG_USBMSC_EPBULKOUT=1             : Use EP1 for the BULK OUT endpoint
      CONFIG_USBMSC_EPBULKIN=2              : Use EP2 for the BULK IN endpoint
      CONFIG_USBMSC_BULKINREQLEN=64         : (Defaults for full speed)
      CONFIG_USBMSC_BULKOUTREQLEN=64        :
                                            : Defaults for other settings as well?
    Board Selection
      CONFIG_SAM4EEK_AT25_BLOCKDEVICE=y     : Export AT25 serial FLASH device
      CONFIG_SAM4EEK_HSMCI_BLOCKDEVICE=n    : Don't export HSMCI SD card

  Note: If properly configured, you could export the HSMCI SD card instead
  of the internal AT25 Serial FLASH.

  The following setting enables an add-on that can can be used to control
  the USB MSC device.  It will add two new NSH commands:

    a. msconn will connect the USB serial device and export the AT25
       to the host, and
    b. msdis which will disconnect the USB serial device.

    Application Configuration -> System Add-Ons:
      CONFIG_SYSTEM_USBMSC=y                : Enable the USBMSC add-on
      CONFIG_SYSTEM_USBMSC_NLUNS=1          : One LUN
      CONFIG_SYSTEM_USBMSC_DEVMINOR1=0      : Minor device zero
      CONFIG_SYSTEM_USBMSC_DEVPATH1="/dev/mtdblock0"
                                            : Use a single, LUN:  The AT25
                                            : block driver.
    NOTES:

    a. To prevent file system corruption, make sure that the AT25 is un-
       mounted *before* exporting the mass storage device to the host:

         nsh> umount /mnt/at25
         nsh> mscon

       The AT25 can be re-mounted after the mass storage class is disconnected:

        nsh> msdis
        nsh> mount -t vfat /dev/mtdblock0 /mnt/at25

    b. If you change the value CONFIG_SYSTEM_USBMSC_DEVPATH1, then you
       can export other file systems:

        "/dev/mmcsd0" would export the HSMCI SD slot (not currently available,
        see the "HSMCI" section).

        "/dev/ram0" could even be used to export a RAM disk.  But you would
         first have to use mkrd to create the RAM disk and mkfatfs to put
         a FAT file system on it.

  STATUS:

  2014-3-25:  Marginally functional. Very slow to come up.  USB analyzer
              shows several resets before the host decides that it is
              happy with the device.  There are no obvious errors in the
              USB data capture.  Testing is insufficient.  This needs to
              be revisited.

              Last tested at 96MHz with the CMCC disabled.

  CDC/ACM Serial Device Class
  ---------------------------

  This will select the CDC/ACM serial device.  Defaults for the other
  options should be okay.

    Device Drivers -> USB Device Driver Support
      CONFIG_CDCACM=y                       : Enable the CDC/ACM device
      CONFIG_CDCACM_EPINTIN=1               : Select endpoint numbers
      CONFIG_CDCACM_EPBULKOUT=2
      CONFIG_CDCACM_EPBULKIN=3

  The following setting enables an example that can can be used to control
  the CDC/ACM device.  It will add two new NSH commands:

    a. sercon will connect the USB serial device (creating /dev/ttyACM0), and
    b. serdis which will disconnect the USB serial device (destroying
        /dev/ttyACM0).

    Application Configuration -> Examples:
      CONFIG_SYSTEM_CDCACM=y                : Enable an CDC/ACM example
      CONFIG_SYSTEM_CDCACM_DEVMINOR=0       : Use /dev/ttyUSB0

  NOTES:

  1. You cannot have both the CDC/ACM and the MSC class drivers enabled
     simultaneously in the way described here.  If you want to use both, then
     you will need to consider a USB "composite" devices that support supports
     both interfaces.  There are no instructures here for setting up the USB
     composite device, but there are other examples in the NuttX board support
     directories that can be used for reference.

  2. Linux supports the CDC/ACM driver out of the box.  Windows, on the other
     than requires that you first install a serial driver (a .inf file).  There
     are example .inf files for NuttX in the nuttx/configs/spark directories.

  3. There is hand-shaking to pace incoming serial data.  As a result, you may
     experience data loss due to RX overrun errors.  The overrun errors occur
     when more data is received than can be buffered in memory on the target.

     At present, the only workaround is to increase the amount of buffering
     in the target.  That allow the target to accept short bursts of larger
     volumes of data (but would still fail on sustained, high speed incoming
     data.  The following configuration options can be changed to increase
     the buffering.

     1. RX buffer size.  All incoming data is buffered by the serial driver
        until it can be read by the application.  The default size of this
        RX buffer is only 256 but can be increased as you see fit:

         CONFIG_CDCACM_RXBUFSIZE=256        : Default RX buffer size is only 256 bytes

     2. Upstream from the RX buffers are USB read request buffers.  Each
        buffer is the maximum size of one USB packet (64 byte) and that cannot
        really be changed.  But if you want to increase this upstream buffering
        capability, you can increase the number of available read requests.
        The default is four, providing an additional buffering capability of
        of 4*64=256 bytes.

        Each read request receives data from USB, copies the data into the
        serial RX buffer, and then is available to receive more data.  This
        recycling of read requests stalls as soon as the serial RX buffer is
        full.  Data loss occurs when there are no available read requests to
        accept the next packet from the host.  So increasing the number of
        read requests can also help to minimize RX overrun:

          CONFIG_CDCACM_NRDREQS=4           : Default is only 4 read requests

  STATUS:

  2013-2-23:  Checks out OK.  See discussion of the usbnsh configuration
              below.

  Debugging USB Device
  --------------------

  There is normal console debug output available that can be enabled with
  CONFIG_DEBUG + CONFIG_DEBUG_USB.  However, USB device operation is very
  time critical and enabling this debug output WILL interfere with the
  operation of the UDP.  USB device tracing is a less invasive way to get
  debug information:  If tracing is enabled, the USB device will save
  encoded trace output in in-memory buffer; if the USB monitor is also
  enabled, that trace buffer will be periodically emptied and dumped to the
  system logging device (the serial console in this configuration):

    Device Drivers -> "USB Device Driver Support:
      CONFIG_USBDEV_TRACE=y                   : Enable USB trace feature
      CONFIG_USBDEV_TRACE_NRECORDS=256        : Buffer 256 records in memory
      CONFIG_USBDEV_TRACE_STRINGS=y           : (optional)

    If you get data loss in the trace buffer, then you may want to increase the
    CONFIG_USBDEV_TRACE_NRECORDS.  I have used buffers up to 4096 records to
    avoid data loss.

    Application Configuration -> NSH LIbrary:
      CONFIG_NSH_USBDEV_TRACE=n               : No builtin tracing from NSH
      CONFIG_NSH_ARCHINIT=y                   : Automatically start the USB monitor

    Application Configuration -> System NSH Add-Ons:
      CONFIG_SYSTEM_USBMONITOR=y              : Enable the USB monitor daemon
      CONFIG_SYSTEM_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
      CONFIG_SYSTEM_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
      CONFIG_SYSTEM_USBMONITOR_INTERVAL=1     : Dump trace data every second
      CONFIG_SYSTEM_USBMONITOR_TRACEINIT=y    : Enable TRACE output
      CONFIG_SYSTEM_USBMONITOR_TRACECLASS=y
      CONFIG_SYSTEM_USBMONITOR_TRACETRANSFERS=y
      CONFIG_SYSTEM_USBMONITOR_TRACECONTROLLER=y
      CONFIG_SYSTEM_USBMONITOR_TRACEINTERRUPTS=y

  NOTE: If USB debug output is also enabled, both outputs will appear on the
  serial console.  However, the debug output will be asynchronous with the
  trace output and, hence, difficult to interpret.

HSMCI
=====

  Enabling HSMCI support. The SAM3U-KE provides a an SD memory card slot.
  Support for the SD slot can be enabled with the following settings:

    System Type->ATSAM3/4 Peripheral Support
      CONFIG_SAM34_HSMCI=y                    : Enable HSMCI support
      CONFIG_SAM34_DMAC0=y                    : DMAC support is needed by HSMCI

    System Type
      CONFIG_SAM34_GPIO_IRQ=y                 : PIO interrupts needed
      CONFIG_SAM34_GPIOA_IRQ=y                : Card detect pin is on PIOA

    Device Drivers -> MMC/SD Driver Support
      CONFIG_MMCSD=y                          : Enable MMC/SD support
      CONFIG_MMCSD_NSLOTS=1                   : One slot per driver instance
      CONFIG_MMCSD_HAVECARDDETECT=y           : Supports card-detect PIOs
      CONFIG_MMCSD_SDIO=y                     : SDIO-based MMC/SD support
      CONFIG_MMCSD_MULTIBLOCK_DISABLE=y       : Probably works but is untested

      CONFIG_SDIO_DMA=y                       : Use SDIO DMA
      CONFIG_SDIO_BLOCKSETUP=y                : Needs to know block sizes

    Library Routines
      CONFIG_SCHED_WORKQUEUE=y                : Driver needs work queue support
                                              : Defaults for other settings okay

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                   : NSH board-initialization
      CONFIG_NSH_MMCSDSLOTNO=0                : Only one slot, slot 0

  After an SD card is successfully initialized, the block device /dev/mmcsd0
  will be available.  To mount the SD card, use the following NSH command:

    nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard

  The SD card contents will then be available under /mnt/sdcard.

  NOTES:

  1. DMA is not currently functional and without DMA, there may not be
     reliable data transfers at high speeds due to data overrun problems.
     The current HSMCI driver supports DMA via the DMAC.  However, the data
     sheet only discusses PDC-based HSMCI DMA (although there is a DMA
     channel interface definition for HSMCI).

     Bottom line:  Untested and probably not usable on the SAM4E-EK in its
     current form.

Touchscreen
===========

  The NSH configuration can be used to verify the ADS7843E touchscreen on
  the SAM4E-EK LCD.  With these modifications, you can include the touchscreen
  test program at apps/examples/touchscreen as an NSH built-in application.
  You can enable the touchscreen and test by modifying the default
  configuration in the following ways:

    Device Drivers
      CONFIG_SPI=y                          : Enable SPI support
      CONFIG_SPI_EXCHANGE=y                 : The exchange() method is supported
      CONFIG_SPI_OWNBUS=y                   : Smaller code if this is the only SPI device

      CONFIG_INPUT=y                        : Enable support for input devices
      CONFIG_INPUT_ADS7843E=y               : Enable support for the XPT2046
      CONFIG_ADS7843E_SPIDEV=2              : Use SPI CS 2 for communication
      CONFIG_ADS7843E_SPIMODE=0             : Use SPI mode 0
      CONFIG_ADS7843E_FREQUENCY=1000000     : SPI BAUD 1MHz
      CONFIG_ADS7843E_SWAPXY=y              : If landscape orientation
      CONFIG_ADS7843E_THRESHX=51            : These will probably need to be tuned
      CONFIG_ADS7843E_THRESHY=39

    System Type -> Peripherals:
      CONFIG_SAM34_SPI0=y                   : Enable support for SPI

    System Type:
      CONFIG_SAM34_GPIO_IRQ=y               : GPIO interrupt support
      CONFIG_SAM34_GPIOA_IRQ=y              : Enable GPIO interrupts from port A

    RTOS Features:
      CONFIG_DISABLE_SIGNALS=n              : Signals are required

    Library Support:
      CONFIG_SCHED_WORKQUEUE=y              : Work queue support required

    Application Configuration:
      CONFIG_EXAMPLES_TOUCHSCREEN=y         : Enable the touchscreen built-in test

    Defaults should be okay for related touchscreen settings.  Touchscreen
    debug output on UART0 can be enabled with:

    Build Setup:
      CONFIG_DEBUG=y                    : Enable debug features
      CONFIG_DEBUG_VERBOSE=y            : Enable verbose debug output
      CONFIG_DEBUG_INPUT=y              : Enable debug output from input devices

  STATUS
    2014-3-27: As of this writing, the touchscreen is untested.

ILI9325-Based LCD
=================

  The SAM4E-EK carries a TFT transmissive LCD module with touch panel,
  FTM280C34D. Its integrated driver IC is ILI9325. The LCD display area is
  2.8 inches diagonally measured, with a native resolution of 240 x 320
  dots.

  No driver has been developed for the SAM4E-EK LCD as of this writing.
  Some technical information follows might be useful to anyone who is
  inspired to develop that driver:

  Connectivity
  ------------

    The SAM4E16 communicates with the LCD through PIOC where an 8-bit
    parallel "8080-like" protocol data bus has to be implemented in
    software.

    ---- ----- --------- --------------------------------
    PIN  PIO   SIGNAL    NOTES
    ---- ----- --------- --------------------------------
      1                  VDD
      2  PC7   DB17
      3  PC6   DB16
      4  PC5   DB15
      5  PC4   DB14
      6  PC3   DB13
      7  PC2   DB12
      8  PC1   DB11
      9  PC0   DB10
     10        DB9       Pulled low
     11        DB8       Pulled low
     12        DB7       Pulled low
     13        DB6       Pulled low
     14        DB5       Pulled low
     15        DB4       Pulled low
     16        DB3       Pulled low
     17        DB2       Pulled low
     18        DB1       Pulled low
     19        DB0       Pulled low
    ---- ----- --------- --------------------------------
     20                  VDD
     21  PC11  RD
     22  PC8   WR
     23  PC19  RS
     24  PD18  CS        Via J8, pulled high.  Connects to NRST.
     25        RESET     Connects to NSRST
     26        IM0       Pulled high
     27        IM1       Grounded
     28        GND
    ---- ----- --------- --------------------------------
     29 [PC13] LED-A     Backlight controls:  PC13 enables
     30 [PC13] LEDK1       AAT3155 charge pump that drives
     31 [PC13] LEDK2       the backlight LEDs
     32 [PC13] LEDK3
     33 [PC13] LEDK4
     34 [PC13] LEDK1
    ---- ----- --------- --------------------------------
     35        Y+        These go to the ADS7843
     36        Y-          touchscreen controller.
     37        X+
     38        X-
     39        NC
    ---- ----- --------- --------------------------------

  Backlight
  ---------

    LCD backlight is made of 4 white chip LEDs in parallel, driven by an
    AAT3155 charge pump, MN4. The AAT3155 is controlled by the SAM3U4E
    through a single line Simple Serial Control (S2Cwire) interface, which
    permits to enable, disable, and set the LED drive current (LED
    brightness control) from a 32-level logarithmic scale. Four resistors
    R93/R94/R95/R96 are implemented for optional current limitation.

  Resources
  ---------

    If you want to implement LCD support, here are some references that may
    help you:

    1. Atmel Sample Code (ASF).  There is no example for the SAM4E-EK, but
       there is for the SAM4S-EK.  The LCD and its processor connectivity
       appear to be equivalent to the SAM4E-EK so this sample code should be
       a good place to begin.  NOTE that the clock frequencies may be
       different and pin usage may be different.  So it may be necessary to
       adjust the SAM configuration to use this example.

    2. There is an example of an LCD driver for the SAM3U at
       configs/sam4u-ek/src/up_lcd.c.  That LCD driver is for an LCD with a
       different LCD controller but should provide the NuttX SAM framework
       for an LCD driver.

    3. There are other LCD drivers for different MCUs that do support the
       ILI9325 LCD.  Look at configs/shenzhou/src/up_ili93xx.c,
       configs/stm3220g-eval/src/up_lcd.c, and
       configs/stm3240g-eval/src/up_lcd.c.  I believe that the Shenzhou
       driver is the most recent.

  STATUS:
    2014-3-27:  Not implemented.

SAM4E-EK-specific Configuration Options
=======================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP="sam34"

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_SAM34
       CONFIG_ARCH_CHIP_SAM3U
       CONFIG_ARCH_CHIP_ATSAM3U4

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=sam4e-ek (for the SAM4E-EK development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_SAM4EEK=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=0x00020000 (128Kb)

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

  Individual subsystems can be enabled:

    CONFIG_SAM34_SPI0          - Serial Peripheral Interface 0 (SPI0)
    CONFIG_SAM34_SPI1          - Serial Peripheral Interface 1 (SPI1)
    CONFIG_SAM34_SSC           - Synchronous Serial Controller (SSC)
    CONFIG_SAM34_TC0           - Timer/Counter 0 (TC0)
    CONFIG_SAM34_TC1           - Timer/Counter 1 (TC1)
    CONFIG_SAM34_TC2           - Timer/Counter 2 (TC2)
    CONFIG_SAM34_TC3           - Timer/Counter 3 (TC3)
    CONFIG_SAM34_TC4           - Timer/Counter 4 (TC4)
    CONFIG_SAM34_TC5           - Timer/Counter 5 (TC5)
    CONFIG_SAM34_TC6           - Timer/Counter 6 (TC6)
    CONFIG_SAM34_TC7           - Timer/Counter 7 (TC6)
    CONFIG_SAM34_TC8           - Timer/Counter 6 (TC8)
    CONFIG_SAM34_PWM           - Pulse Width Modulation (PWM) Controller
    CONFIG_SAM34_TWIM0         - Two-wire Master Interface 0 (TWIM0)
    CONFIG_SAM34_TWIS0         - Two-wire Slave Interface 0 (TWIS0)
    CONFIG_SAM34_TWIM1B        - Two-wire Master Interface 1 (TWIM1)
    CONFIG_SAM34_TWIS1         - Two-wire Slave Interface 1 (TWIS1)
    CONFIG_SAM34_UART0         - UART 0
    CONFIG_SAM34_UART1         - UART 1
    CONFIG_SAM34_USART0        - USART 0
    CONFIG_SAM34_USART1        - USART 1
    CONFIG_SAM34_USART2        - USART 2
    CONFIG_SAM34_USART3        - USART 3
    CONFIG_SAM34_AFEC0         - Analog Front End 0
    CONFIG_SAM34_AFEC1         - Analog Front End 1
    CONFIG_SAM34_DACC          - Digital-to-Analog Converter
    CONFIG_SAM34_ACC           - Analog Comparator
    CONFIG_SAM34_EMAC          - Ethernet MAC
    CONFIG_SAM34_CAN0          - CAN 0
    CONFIG_SAM34_CAN1          - CAN 1
    CONFIG_SAM34_SMC           - Static Memory Controller
    CONFIG_SAM34_NAND          - NAND support
    CONFIG_SAM34_PDCA          - Peripheral DMA controller
    CONFIG_SAM34_DMAC0         - DMA controller
    CONFIG_SAM34_UDP           - USB 2.0 Full-Speed device
    CONFIG_SAM34_CHIPID        - Chip ID
    CONFIG_SAM34_RTC           - Real Time Clock
    CONFIG_SAM34_RTT           - Real Time Timer
    CONFIG_SAM34_WDT           - Watchdog Timer
    CONFIG_SAM34_EIC           - Interrupt controller
    CONFIG_SAM34_HSMCI         - High Speed Multimedia Card Interface

  Some subsystems can be configured to operate in different ways. The drivers
  need to know how to configure the subsystem.

    CONFIG_SAM34_GPIOA_IRQ
    CONFIG_SAM34_GPIOB_IRQ
    CONFIG_SAM34_GPIOC_IRQ
    CONFIG_SAM34_GPIOD_IRQ
    CONFIG_SAM34_GPIOE_IRQ
    CONFIG_SAM34_GPIOF_IRQ
    CONFIG_SAM34_GPIOG_IRQ
    CONFIG_SAM34_GPIOH_IRQ
    CONFIG_SAM34_GPIOJ_IRQ
    CONFIG_SAM34_GPIOK_IRQ
    CONFIG_SAM34_GPIOL_IRQ
    CONFIG_SAM34_GPIOM_IRQ
    CONFIG_SAM34_GPION_IRQ
    CONFIG_SAM34_GPIOP_IRQ
    CONFIG_SAM34_GPIOQ_IRQ

    CONFIG_USART0_ISUART
    CONFIG_USART1_ISUART
    CONFIG_USART2_ISUART
    CONFIG_USART3_ISUART

  SAM3U specific device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=0,1,2,3) or UART
           m (m=4,5) for the console and ttys0 (default is the USART1).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

  LCD Options.  Other than the standard LCD configuration options
  (see configs/README.txt), the SAM4E-EK driver also supports:

    CONFIG_LCD_PORTRAIT - Present the display in the standard 240x320
       "Portrait" orientation.  Default:  The display is rotated to
       support a 320x240 "Landscape" orientation.

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each SAM4E-EK configuration is maintained in a sub-directory and
  can be selected as follow:

    cd tools
    ./configure.sh sam4e-ek/<subdir>
    cd -
    . ./setenv.sh

  Before sourcing the setenv.sh file above, you should examine it and perform
  edits as necessary so that BUILDROOT_BIN is the correct path to the directory
  than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       and misc/tools/

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on UART0 (J3).

  3. All of these configurations are set up to build under Linux using the
     EABI buildroot toolchain (unless stated otherwise in the description of
     the configuration).  That build selection can easily be reconfigured
     using 'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_LINUX=y                 : Linux or other pure POSIX invironment

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot toolchain
       CONFIG_ARMV7M_OABI_TOOLCHAIN=n      : EABI (Not OABI

     If you want to use the Atmel GCC toolchain, for example, here are the
     steps to do so:

     Build Setup:
       CONFIG_HOST_WINDOWS=y   : Windows
       CONFIG_HOST_CYGWIN=y    : Using Cygwin or other POSIX environment

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : General GCC EABI toolchain under windows

     Library Routines ->
       CONFIG_CXX_NEWLONG=n                : size_t is an unsigned int, not long

     This re-configuration should be done before making NuttX or else the
     subsequent 'make' will fail.  If you have already attempted building
     NuttX then you will have to 1) 'make distclean' to remove the old
     configuration, 2) 'cd tools; ./configure.sh sam4e-ek/ksnh' to start
     with a fresh configuration, and 3) perform the configuration changes
     above.

     Also, make sure that your PATH variable has the new path to your
     Atmel tools.  Try 'which arm-none-eabi-gcc' to make sure that you
     are selecting the right tool.  setenv.sh is available for you to
     use to set or PATH variable.  The path in the that file may not,
     however, be correct for your installation.

     See also the "NOTE about Windows native toolchains" in the section call
     "GNU Toolchain Options" above.

  Configuration sub-directories
  -----------------------------

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables both the serial and telnetd NSH interfaces.

    NOTES:

    1. This configuration runs with a CPU clock of 120MHz and with the
       the CMCC enabled.  If you disable these, then you must also
       re-calibrate the delay loop.

    2. Default stack sizes are large and should really be tuned to reduce
       the RAM footprint:

         CONFIG_ARCH_INTERRUPTSTACK=2048
         CONFIG_IDLETHREAD_STACKSIZE=1024
         CONFIG_USERMAIN_STACKSIZE=2048
         CONFIG_PTHREAD_STACK_DEFAULT=2048
         ... and others ...

    3. NSH built-in applications are supported.

       Binary Formats:
         CONFIG_BUILTIN=y                    : Enable support for built-in programs

       Applicaton Configuration:
         CONFIG_NSH_BUILTIN_APPS=y           : Enable starting apps from NSH command line

    4. This configuration has the network enabled by default.  This can be
       easily disabled or reconfigured (See see the network related
       configuration settings above in the section entitled "Networking").

       NOTE: In boot-up sequence is very simple in this example; all
       initialization is done sequential (vs. in parallel) and so you will
       not see the NSH prompt until all initialization is complete.  The
       network bring-up in particular will add some delay before the NSH
       prompt appears.  In a real application, you would probably want to
       do the network bringup on a separate thread so that access to the
       NSH prompt is not delayed.

       This delay will be especially long if the board is not connected to
       a network because additional time will be required to fail with
       timeout errors.

       STATUS:
       2014-3-13: The basic NSH serial console is working.  Network support
                  has been verified.

    5. This configuration supports a network with fixed IP address.  You
       may have to change these settings for your network:

       CONFIG_NSH_IPADDR=0x0a000002        : IP address: 10.0.0.2
       CONFIG_NSH_DRIPADDR=0x0a000001      : Gateway:    10.0.0.1
       CONFIG_NSH_NETMASK=0xffffff00       : Netmask:    255.255.255.0

       You can also enable enable the DHCPC client for networks that use
       dynamically assigned address:

       CONFIG_NETUTILS_DHCPC=y             : Enables the DHCP client
       CONFIG_NET_UDP=y                    : Depends on broadcast UDP
       CONFIG_NET_BROADCAST=y
       CONFIG_NSH_DHCPC=y                  : Tells NSH to use DHCPC, not
                                           : the fixed addresses

    6. This configuration has the DMA-based SPI0 and AT25 Serial FLASH
       support enabled by default.  This can be easily disabled or
       reconfigured (See see the configuration settings and usage notes
       above in the section entitled "AT25 Serial FLASH").

       To mount the AT25 Serial FLASH as a FAT file system:

         nsh>mount -t vfat /dev/mtdblock0 /mnt/at25

       STATUS:
       2014-3-14: The DMA-based SPI appears to be functional and can be used
                  to support a FAT file system on the AT25 Serial FLASH.

    7. USB device support is not enabled in this configuration by default.
       To add USB device support to this configuration, see the instructions
       above under "USB Full-Speed Device."

       STATUS:
       2014-3-21: USB support is partially functional.  Additional test and
                  integration is required. See STATUS in the "USB Full-Speed
                  Device" for further information
       2014-3-22: USB seems to work properly (there are not obvious errors
                  in a USB bus capture.  However, as of this data the AT25
                  does not mount on either the Linux or Windows host.  This
                  needs to be retested.

    8. This configuration can be used to verify the touchscreen on on the
       SAM4E-EK LCD.  See the instructions above in the paragraph entitled
       "Touchscreen".

       STATUS:
         2014-3-21:  The touchscreen has not yet been tested.

    9. Enabling HSMCI support. The SAM3U-KE provides a an SD memory card
       slot.  Support for the SD slot can be enabled following the
       instructions provided above in the paragraph entitled "HSMCI."

       STATUS:
         2014-3-24:  DMA is not currently functional and without DMA, there
                     may not be reliable data transfers at high speeds due
                     to data overrun problems.  The current HSMCI driver
                     supports DMA via the DMAC.  However, the data sheet
                     only discusses PDC-based HSMCI DMA (although there is
                     a DMA channel interface definition for HSMCI).  So
                     this is effort is dead-in-the-water for now.

  usbnsh:

    This is another NSH example.  If differs from the 'nsh' configuration
    in that this configurations uses a USB serial device for console I/O.

    STATUS:
      2014-3-23: This configuration appears to be fully functional.

    NOTES:

    1. See the NOTES in the description of the nsh configuration.  Those
       notes all apply here as well.  Some additional notes unique to
       the USB console version follow:

    2. The configuration differences between this configuration and the
       nsh configuration is:

       a. USB device support is enabled as described in the paragraph
          entitled "USB Full-Speed Device",

       b. The CDC/ACM serial class is enabled as described in the paragraph
          "CDC/ACM Serial Device Class".

       c. The serial console is disabled:

          RTOS Features:
            CONFIG_DEV_CONSOLE=n           : No console at boot time

          Driver Support -> USB Device Driver Support
            CONFIG_UART0_SERIAL_CONSOLE=n  : UART0 is not the console
            CONFIG_NO_SERIAL_CONSOLE=y     : There is no serial console

          Driver Support -> USB Device Driver Support
            CONFIG_CDCACM_CONSOLE=y        : USB CDC/ACM console

       d. Support for debug output on UART0 is provided as described in the
          next note.

    3. If you send large amounts of data to the target, you may see data
       loss due to RX overrun errors.  See the NOTES in the section entitled
       "CDC/ACM Serial Device Class" for an explanation and some possible
       work-arounds.

    3. This configuration does have UART0 output enabled and set up as
       the system logging device:

           File Systems -> Advanced SYSLOG Features
             CONFIG_SYSLOG=y               : Enable output to syslog, not console
             CONFIG_SYSLOG_CHAR=y          : Use a character device for system logging
             CONFIG_SYSLOG_DEVPATH="/dev/ttyS0" : UART0 will be /dev/ttyS0

       However, there is nothing to generate SYLOG output in the default
       configuration so nothing should appear on UART0 unless you enable
       some debug output or enable the USB monitor.

       NOTE:  Using the SYSLOG to get debug output has limitations.  Among
       those are that you cannot get debug output from interrupt handlers.
       So, in particularly, debug output is not a useful way to debug the
       USB device controller driver.  Instead, use the USB monitor with
       USB debug off and USB trace on (see below).

    4. Enabling USB monitor SYSLOG output.  See the paragraph entitle
       "Debugging USB Device" for a summary of the configuration settings
       needed to enable the USB monitor and get USB debug data out UART0.

    5. By default, this configuration uses the CDC/ACM serial device to
       provide the USB console.  This works out-of-the-box for Linux.
       Windows, on the other hand, will require a CDC/ACM device driver
       (.inf file).  There is a sample .inf file in the nuttx/configs/spark
       directories.

    5. Using the Prolifics PL2303 Emulation

       You could also use the non-standard PL2303 serial device instead of
       the standard CDC/ACM serial device by changing:

        CONFIG_CDCACM=n                    : Disable the CDC/ACM serial device class
        CONFIG_CDCACM_CONSOLE=n            : The CDC/ACM serial device is NOT the console
        CONFIG_PL2303=y                    : The Prolifics PL2303 emulation is enabled
        CONFIG_PL2303_CONSOLE=y            : The PL2303 serial device is the console
