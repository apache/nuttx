README
======

This README file discusses the port of NuttX to the Atmel SAM V71 Xplained
Ultra Evaluation Kit (SAMV71-XULT).  This board features the ATSAMV71Q21 Cortex-M7
microcontroller.

Additional support of provided for the (optional) maXTouch Xplained Pro LCD.

Contents
========

  - Board Features
  - Status/Open Issues
  - Serial Console
  - SD card
  - Automounter
  - LEDs and Buttons
  - AT24MAC402 Serial EEPROM
  - Networking
  - Audio Interface
  - maXTouch Xplained Pro
  - Debugging
  - Configurations

Board Features
==============

  - ATSAMV71Q21 microcontroller: Cortex-M7, 300MHz, 2MiB FLASH, 384KiB SRAM,
    I/D-caches
  - One mechanical reset button
  - One power switch button
  - Two mechanical user pushbuttons
  - Two yellow user LEDs
  - Supercap backup
  - 12.0 MHz crystal
  - 32.768 kHz crystal
  - 2 MB SDRAM
  - 2 MB QSPI Flash
  - IEEE 802.3az 10Base-T/100Base-TX Ethernet RMII PHY
  - AT24MAC402 256KByte EEPROM with EUI-48 address
  - WM8904 stereo audio codec
  - ATA6561 CAN Transceiver
  - SD Card connector with SDIO support
  - Camera interface connector
  - MediaLB connector
  - Two Xplained Pro extension headers
  - One Xplained Pro LCD header
  - Coresight 20 connector for 4-bit ETM
  - Arduino due compatible shield connectors
  - External debugger connector
  - USB interface, device and host mode
  - Embedded Debugger with Data Gateway Interface and Virtual COM port (CDC)
  - External power input (5-14V) or USB powered

See the Atmel website for further information about this board:

  - http://www.atmel.com/tools/atsamv71-xult.aspx

tatus/Open Issues
=================

I would characterize the general port as very mature and stable.  However,
there are a number of issues, caveats, and unfinished drivers as detailed in
the following paragraphs.

The BASIC nsh configuration is fully function (as described below under
"Configurations").  There is also a graphics configuration (mxtxplnd), a
a configuration for network testing (netnsh), and a graphics demo (nxwm).
There are still open issues that need to be resolved.  General problems are
listed below.  But see the STATUS section associated with each configuration
for additional issues specific to a particular configuration.

  1. HSCMI. CONFIG_MMCSD_MULTIBLOCK_DISABLE=y is set to disable multi-block
     transfers only because I have not yet had a chance to verify this.  The
     is very low priority to me but might be important to you if you are need
     very high performance SD card accesses.

  2. HSMCI TX DMA is currently disabled for the SAMV7.  There is some
     issue with the TX DMA setup (HSMCI TX DMA the same driver works with
     the SAMA5D4 which has a different DMA subsystem).  This is a bug that
     needs to be resolved.

     DMA is enabled by these settings in the file arch/arm/src/samvy/sam_hsmci.c:

     #undef  HSCMI_NORXDMA              /* Define to disable RX DMA */
     #define HSCMI_NOTXDMA            1 /* Define to disable TX DMA */

  3. There may also be some issues with removing and re-inserting SD cards
     (of course with appropriate mounting and unmounting).  I all not sure
     of this and need to do more testing to characterize if the issue.

  4. There is a port of the SAMA5D4-EK Ethernet driver to the SAMV71-XULT.
     This driver appears to be 100% functional with the following caveats:

     - There is a compiler optimization issue.  At -O2, there is odd
       behavior on pings and ARP messages.  But the behavior is OK with
       optimization set to -O2.  This may or may not be a compiler
       optimization issue (it could also be a timing issue or a need
       for some additional volatile qualifiers).

     - I- and D-Caches are enabled but the D-Cache must be enabled in
       write-through mode.  This is to work around issues with the RX and TX
       descriptors with are 8-bytes in size.  But the D-Cache cache line size
       is 32-bytes.  That means that you cannot reload, clean or invalidate a
       descriptor without also effecting three neighboring descriptors.
       Setting write through mode eliminates the need for cleaning the D-Cache.
       If only reloading and invalidating are done, then there is no problem.

  5. The USBHS device controller driver (DCD) is complete but non-functional.
     At this point, work has stopped because I am stuck.  The problem is that
     bus events are not occurring:  Nothing is detected by the USBHS when the
     host is connected; no activity is seen on the bus by a USB analyzer when
     the host is connected.  Possibilities: (1) the pull-ups on DM and DP are
     not working.  This would prevent the host from detecting the presence of
     the device.  The DETACH bit is, however, being correctly cleared and the
     USB is in the suspend sate.  Or (2) some issue with clocking or
     configuration of the UTMI.  I see nothing wrong in this case.   I have
     done extensive comparison of the Atmel sample code and study of the data
     sheet, but I have not found the key to
     solving this.

  6. The full port for audio support is code complete:  WM8904 driver,
     SSC/I2C driver, and CS2100-CP driver.  But this code is untested.  The
     WM8904 interface was taken directly from the SAMA5D4-EK and may well
     need modification due to differences with the physical WM8904
     interface.

Serial Console
==============

The SAMV71-XULT has no on-board RS-232 drivers so it will be necessary to
use either the VCOM or an external RS-232 driver.  Here are some options.

  - Arduino Serial Shield:  One option is to use an Arduino-compatible
    serial shield.  This will use the RXD and TXD signals available at pins
    0 an 1, respectively, of the Arduino "Digital Low" connector.  On the
    SAMV71-XULT board, this corresponds to UART3:

    ------ ------ ------- ------- --------
    Pin on SAMV71 Arduino Arduino SAMV71
    J503   PIO    Name    Pin     Function
    ------ ------ ------- ------- --------
      1    PD28   RX0     0       URXD3
      2    PD30   TX0     1       UTXD3
    ------ ------ ------- ------- --------

    In this configuration, an external RS232 driver can also be used
    instead of the shield.  Simply connext as follows:

    --------- -----------
    Arduino   RS-232
    Pin Label Connection
    --------- -----------
    D0 (RXD)  RX
    D1 (TXD)  TX
    GND       GND
    5VO       Vcc
    --------- -----------

  - Arduino Communications.  Additional UART/USART connections are available
    on the Arduino Communications connection J505:

    ------ ------ ------- ------- --------
    Pin on SAMV71 Arduino Arduino SAMV71
    J503   PIO    Name    Pin     Function
    ------ ------ ------- ------- --------
      3    PD18   RX1     0       URXD4
      4    PD19   TX1     0       UTXD4
      5    PD15   RX2     0       RXD2
      6    PD16   TX2     0       TXD2
      7    PB0    RX3     0       RXD0
      8    PB1    TX3     1       TXD0
    ------ ------ ------- ------- --------

  - SAMV7-XULT EXTn connectors.  USART pins are also available the EXTn
    connectors.  The following are labelled in the User Guide for USART
    functionality:

    ---- -------- ------ --------
    EXT1 EXTI1    SAMV71 SAMV71
    Pin  Name     PIO    Function
    ---- -------- ------ --------
    13   USART_RX PB00   RXD0
    14   USART_TX PB01   TXD0

    ---- -------- ------ --------
    EXT2 EXTI2    SAMV71 SAMV71
    Pin  Name     PIO    Function
    ---- -------- ------ --------
    13   USART_RX PA21   RXD1
    14   USART_TX PB04   TXD1

  - VCOM.  The Virtual Com Port gateway is available on USART1:

    ------ --------
    SAMV71 SAMV71
    PIO    Function
    ------ --------
    PB04   TXD1
    PA21   RXD1
    ------ --------

Any of these options can be selected as the serial console by:

  1. Enabling the UART/USART peripheral in the
     "System Type -> Peripheral Selection" menu, then
  2. Configuring the peripheral in the "Drivers -> Serial Configuration"
     menu.

SD Card
=======

Card Slot
---------
The SAM V71 Xplained Ultra has one standard SD card connector which is
connected to the High Speed Multimedia Card Interface (HSMCI) of the SAM
V71. SD card connector:

  ------ ----------------- ---------------------
  SAMV71 SAMV71            Shared functionality
  Pin    Function
  ------ ----------------- ---------------------
  PA30   MCDA0 (DAT0)
  PA31   MCDA1 (DAT1)
  PA26   MCDA2 (DAT2)
  PA27   MCDA3 (DAT3)      Camera
  PA25   MCCK (CLK)        Shield
  PA28   MCCDA (CMD)
  PD18   Card Detect (C/D) Shield
  ------ ----------------- ---------------------

Configuration Settings
----------------------
Enabling HSMCI support. The SAMV7-XULT provides a one, full-size SD memory card slots.  The full size SD card slot connects via HSMCI0.  Support for the SD slots can be enabled with the following settings:

  System Type->SAMV7 Peripheral Selection
    CONFIG_SAMV7_HSMCI0=y                 : To enable HSMCI0 support
    CONFIG_SAMV7_XDMAC=y                  : XDMAC is needed by HSMCI0/1

  System Type
    CONFIG_SAMV7_PIO_IRQ=y                : PIO interrupts needed
    CONFIG_SAMV7_PIOD_IRQ=y               : Card detect pin is on PD18

  Device Drivers -> MMC/SD Driver Support
    CONFIG_MMCSD=y                        : Enable MMC/SD support
    CONFIG_MMSCD_NSLOTS=1                 : One slot per driver instance
    CONFIG_MMCSD_MULTIBLOCK_DISABLE=y     : (REVISIT)
    CONFIG_MMCSD_HAVECARDDETECT=y         : Supports card-detect PIOs
    CONFIG_MMCSD_MMCSUPPORT=n             : Interferes with some SD cards
    CONFIG_MMCSD_SPI=n                    : No SPI-based MMC/SD support
    CONFIG_MMCSD_SDIO=y                   : SDIO-based MMC/SD support
    CONFIG_SDIO_DMA=y                     : Use SDIO DMA
    CONFIG_SDIO_BLOCKSETUP=y              : Needs to know block sizes

  RTOS Features -> Work Queue Support
    CONFIG_SCHED_WORKQUEUE=y              : Driver needs work queue support

  Application Configuration -> NSH Library
    CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization, OR
    CONFIG_BOARD_INITIALIZE=y

Using the SD card
-----------------

1) After booting, the HSCMI device will appear as /dev/mmcsd0.

2) If you try mounting an SD card with nothing in the slot, the mount will
   fail:

     nsh> mount -t vfat /dev/mmcsd0 /mnt/sd0
     nsh: mount: mount failed: 19

   NSH can be configured to provide errors as strings instead of
   numbers.  But in this case, only the error number is reported.  The
   error numbers can be found in nuttx/include/errno.h:

     #define ENODEV              19
     #define ENODEV_STR          "No such device"

   So the mount command is saying that there is no device or, more
   correctly, that there is no card in the SD card slot.

3) Inserted the SD card.  Then the mount should succeed.

    nsh> mount -t vfat /dev/mmcsd0 /mnt/sd0
    nsh> ls /mnt/sd1
    /mnt/sd1:
     atest.txt
    nsh> cat /mnt/sd1/atest.txt
    This is a test

   NOTE:  See the next section entitled "Auto-Mounter" for another way
   to mount your SD card.

4) Before removing the card, you must umount the file system.  This is
   equivalent to "ejecting" or "safely removing" the card on Windows:  It
   flushes any cached data to an SD card and makes the SD card unavailable
   to the applications.

     nsh> umount -t /mnt/sd0

   It is now safe to remove the card.  NuttX provides into callbacks
   that can be used by an application to automatically unmount the
   volume when it is removed.  But those callbacks are not used in
   these configurations.

Auto-Mounter
============

  NuttX implements an auto-mounter than can make working with SD cards
  easier.  With the auto-mounter, the file system will be automatically
  mounted when the SD card is inserted into the HSMCI slot and automatically
  unmounted when the SD card is removed.

  Here is a sample configuration for the auto-mounter:

    File System Configuration
      CONFIG_FS_AUTOMOUNTER=y

    Board-Specific Options
      CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT=y
      CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_FSTYPE="vfat"
      CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_BLKDEV="/dev/mmcsd0"
      CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_MOUNTPOINT="/mnt/sdcard"
      CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_DDELAY=1000
      CONFIG_SAMV71XULT_HSMCI0_AUTOMOUNT_UDELAY=2000

  WARNING:  SD cards should never be removed without first unmounting
  them.  This is to avoid data and possible corruption of the file
  system.  Certainly this is the case if you are writing to the SD card
  at the time of the removal.  If you use the SD card for read-only access,
  however, then I cannot think of any reason why removing the card without
  mounting would be harmful.

LEDs and Buttons
================

LEDs
----
There are two yellow LED available on the SAM V71 Xplained Ultra board that
can be turned on and off.  The LEDs can be activated by driving the
connected I/O line to GND.

  ------ ----------- ---------------------
  SAMV71 Function    Shared functionality
  PIO
  ------ ----------- ---------------------
  PA23   Yellow LED0 EDBG GPIO
  PC09   Yellow LED1 LCD, and Shield
  ------ ----------- ---------------------

These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/sam_autoleds.c. The LEDs are used to encode
OS-related events as follows:

  -------------------  -----------------------  -------- --------
  SYMBOL                Meaning                     LED state
                                                  LED0     LED1
  -------------------  -----------------------  -------- --------
  LED_STARTED          NuttX has been started     OFF      OFF
  LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
  LED_IRQSENABLED      Interrupts enabled         OFF      OFF
  LED_STACKCREATED     Idle stack created         ON       OFF
  LED_INIRQ            In an interrupt              No change
  LED_SIGNAL           In a signal handler          No change
  LED_ASSERTION        An assertion failed          No change
  LED_PANIC            The system has crashed     N/C      Blinking
  LED_IDLE             MCU is is sleep mode         Not used
  -------------------  -----------------------  -------- --------

Thus if LED0 is statically on, NuttX has successfully booted and is,
apparently, running normally.  If LED1 is flashing at approximately
2Hz, then a fatal error has been detected and the system has halted.

NOTE: That LED0 is not used after completion of booting and may
be used by other board-specific logic.

Buttons
-------
SAM V71 Xplained Ultra contains three mechanical buttons. One button is the
RESET button connected to the SAM V71 reset line and the others are generic
user configurable buttons. When a button is pressed it will drive the I/O
line to GND.

  ------ ----------- ---------------------
  SAMV71 Function    Shared functionality
  PIO
  ------ ----------- ---------------------
  RESET  RESET       Trace, Shield, and EDBG
  PA09   SW0         EDBG GPIO and Camera
  PB12   SW1         EDBG SWD and Chip Erase
  ------ ----------- ---------------------

NOTES:

  - There are no pull-up resistors connected to the generic user buttons so
    it is necessary to enable the internal pull-up in the SAM V71 to use the
    button.
  - PB12 is set up as a system flash ERASE pin when the firmware boots. To
    use the SW1, PB12 has to be configured as a normal regular I/O pin in
    the MATRIX module. For more information see the SAM V71 datasheet.

AT24MAC402 Serial EEPROM
========================

Ethernet MAC Address
--------------------
The SAM V71 Xplained Ultra features one external AT24MAC402 serial EEPROM
with a EIA-48 MAC address connected to the SAM V71 through I2C. This device
contains a MAC address for use with the Ethernet interface.

Connectivity:

  ------ -------- -------- ------------------------------------------
  SAMV71 SAMV71   I2C      Shared
  Pin    Function Function Functionality
  ------ -------- -------- ------------------------------------------
  PA03   TWID0    SDA      EXT1, EXT2, EDBG I2C, LCD, Camera, Audio,
                           MediaLB, and Shield
  PA04   TWICK0   SCL      EXT1, EXT2, EDBG I2C, LCD, Camera, Audio,
                           MediaLB, and Shield
  ------ -------- -------- ------------------------------------------

I2C address:

  The 7-bit addresses of the AT24 part are 0b1010AAA for the normal 2Kbit
  memory and 0b1011aaa for the "extended memory" where aaa is the state of
  the A0, A1, and A3 pins on the part.  On the SAMV71-XULT board, these
  are all pulled high so the full, 7-bit address is 0x5f.

Configuration
-------------

  System Type -> SAMV7 Peripheral Support
    CONFIG_SAMV7_TWIHS0=y                : Used to access the EEPROM
    CONFIG_SAMV7_TWIHS0_FREQUENCY=100000

  Device drivers -> Memory Technology Devices
    CONFIG_MTD_AT24XX=y                  : Enable the AT24 device driver
    CONFIG_AT24XX_SIZE=2                 : Normal EEPROM is 2Kbit (256b)
    CONFIG_AT24XX_ADDR=0x57              : Normal EEPROM address */
    CONFIG_AT24XX_EXTENDED=y             : Supports an extended memory region
    CONFIG_AT24XX_EXTSIZE=160            : Extended address up to 0x9f

MTD Configuration Data
----------------------
The AT24 EEPROM can also be used to storage of up to 256 bytes of
configuration data:

  Device drivers -> Memory Technology Devices

The configuration data device will appear at /dev/config.

Networking
==========

KSZ8061RNBVA Connections
------------------------

  ------ --------- --------- --------------------------
  SAMV71 SAMV71    Ethernet  Shared functionality
  Pin    Function  Function
  ------ --------- --------- --------------------------
  PD00   GTXCK     REF_CLK   Shield
  PD01   GTXEN     TXEN
  PD02   GTX0      TXD0
  PD03   GTX1      TXD1
  PD04   GRXDV     CRS_DV    Trace
  PD05   GRX0      RXD0      Trace
  PD06   GRX1      RXD1      Trace
  PD07   GRXER     RXER      Trace
  PD08   GMDC      MDC       Trace
  PD09   GMDIO     MDIO
  PA19   GPIO      INTERRUPT EXT1, Shield
  PA29   GPIO      SIGDET
  PC10   GPIO      RESET
  ------ --------- --------- --------------------------

Selecting the GMAC peripheral
-----------------------------

  System Type -> SAMV7 Peripheral Support
    CONFIG_SAMV7_EMAC0=y                 : Enable the GMAC peripheral (aka, EMAC0)
    CONFIG_SAMV7_TWIHS0=y                : We will get the MAC address from the AT24 EEPROM
    CONFIG_SAMV7_TWIHS0_FREQUENCY=100000

  System Type -> EMAC device driver options
    CONFIG_SAMV7_EMAC0_NRXBUFFERS=16     : Set aside some RS and TX buffers
    CONFIG_SAMV7_EMAC0_NTXBUFFERS=8
    CONFIG_SAMV7_EMAC0_RMII=y            : The RMII interfaces is used on the board
    CONFIG_SAMV7_EMAC0_AUTONEG=y         : Use autonegotiation
    CONFIG_SAMV7_EMAC0_PHYADDR=1         : KSZ8061 PHY is at address 1
    CONFIG_SAMV7_EMAC0_PHYSR=30          : Address of PHY status register on KSZ8061
    CONFIG_SAMV7_EMAC0_PHYSR_ALTCONFIG=y : Needed for KSZ8061
    CONFIG_SAMV7_EMAC0_PHYSR_ALTMODE=0x7 : "    " " " "     "
    CONFIG_SAMV7_EMAC0_PHYSR_10HD=0x1    : "    " " " "     "
    CONFIG_SAMV7_EMAC0_PHYSR_100HD=0x2   : "    " " " "     "
    CONFIG_SAMV7_EMAC0_PHYSR_10FD=0x5    : "    " " " "     "
    CONFIG_SAMV7_EMAC0_PHYSR_100FD=0x6   : "    " " " "     "

  PHY selection.  Later in the configuration steps, you will need to select
  the KSZ8061 PHY for EMAC (See below)

  Networking Support
    CONFIG_NET=y                         : Enable Neworking
    CONFIG_NET_NOINTS=y                  : Use the work queue, not interrupts for processing
    CONFIG_NET_SOCKOPTS=y                : Enable socket operations
    CONFIG_NET_ETH_MTU=562               : Maximum packet size (MTU) 1518 is more standard
    CONFIG_NET_ETH_TCP_RECVWNDO=562      : Should be the same as CONFIG_NET_ETH_MTU
    CONFIG_NET_ARP=y                     : ARP support should be enabled
    CONFIG_NET_ARP_SEND=y                : Use ARP to get peer address before sending
    CONFIG_NET_TCP=y                     : Enable TCP/IP networking
    CONFIG_NET_TCPBACKLOG=y              : Support TCP/IP backlog
    CONFIG_NET_TCP_READAHEAD=y           : Enable TCP read-ahead buffering
    CONFIG_NET_TCP_WRITE_BUFFERS=y       : Enable TCP write buffering
    CONFIG_NET_UDP=y                     : Enable UDP networking
    CONFIG_NET_BROADCAST=y               : Support UDP broadcase packets
    CONFIG_NET_ICMP=y                    : Enable ICMP networking
    CONFIG_NET_ICMP_PING=y               : Needed for NSH ping command
                                         : Defaults should be okay for other options
  Device drivers -> Network Device/PHY Support
    CONFIG_NETDEVICES=y                  : Enabled PHY selection
    CONFIG_ETH0_PHY_KSZ8061=y            : Select the KSZ8061 PHY used with EMAC0

  Device drivers -> Memory Technology Devices
    CONFIG_MTD_AT24XX=y                  : Enable the AT24 device driver
    CONFIG_AT24XX_SIZE=2                 : Normal EEPROM is 2Kbit (256b)
    CONFIG_AT24XX_ADDR=0x57              : Normal EEPROM address */
    CONFIG_AT24XX_EXTENDED=y             : Supports an extended memory region
    CONFIG_AT24XX_EXTSIZE=160            : Extended address up to 0x9f

  RTOS Features ->Work Queue Support
    CONFIG_SCHED_WORKQUEUE=y             : Work queue support is needed
    CONFIG_SCHED_HPWORK=y
    CONFIG_SCHED_HPWORKSTACKSIZE=2048    : Might need to be increased

  Application Configuration -> Network Utilities
    CONFIG_NETUTILS_DNSCLIENT=y          : Enable host address resolution
    CONFIG_NETUTILS_TELNETD=y            : Enable the Telnet daemon
    CONFIG_NETUTILS_TFTPC=y              : Enable TFTP data file transfers for get and put commands
    CONFIG_NETUTILS_NETLIB=y             : Network library support is needed
    CONFIG_NETUTILS_WEBCLIENT=y          : Needed for wget support
                                         : Defaults should be okay for other options
  Application Configuration -> NSH Library
    CONFIG_NSH_TELNET=y                  : Enable NSH session via Telnet
    CONFIG_NSH_IPADDR=0x0a000002         : Select an IP address
    CONFIG_NSH_DRIPADDR=0x0a000001       : IP address of gateway/host PC
    CONFIG_NSH_NETMASK=0xffffff00        : Netmask
    CONFIG_NSH_NOMAC=n                   : We will get the IP address from EEPROM
                                         : Defaults should be okay for other options

Cache-Related Issues
--------------------

I- and D-Caches can be enabled but the D-Cache must be enabled in write-
through mode.  This is to work around issues with the RX and TX descriptors
with are 8-bytes in size.  But the D-Cache cache line size is 32-bytes.
That means that you cannot reload, clean or invalidate a descriptor without
also effecting three neighboring descriptors. Setting write through mode
eliminates the need for cleaning the D-Cache.  If only reloading and
invalidating are done, then there is no problem.

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

By default, the IP address of the SAMA4D4-EK will be 10.0.0.2 and
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

On the host side, you should also be able to ping the SAMA4D4-EK:

  $ ping 10.0.0.2

You can also log into the NSH from the host PC like this:

  $ telnet 10.0.0.2
  Trying 10.0.0.2...
  Connected to 10.0.0.2.
  Escape character is '^]'.
  sh_telnetmain: Session [3] Started

  NuttShell (NSH) NuttX-7.9
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
a network.  On the order of a minute!  You will probably think that
NuttX has crashed!  And then, when it finally does come up, the
network will not be available.

Network Initialization Thread
-----------------------------
There is a configuration option enabled by CONFIG_NSH_NETINIT_THREAD
that will do the NSH network bring-up asynchronously in parallel on
a separate thread.  This eliminates the (visible) networking delay
altogether.  This networking initialization feature by itself has
some limitations:

  - If no network is connected, the network bring-up will fail and
    the network initialization thread will simply exit.  There are no
    retries and no mechanism to know if the network initialization was
    successful.

  - Furthermore, there is no support for detecting loss of the network
    connection and recovery of networking when the connection is restored.

Both of these shortcomings can be eliminated by enabling the network
monitor:

Network Monitor
---------------
By default the network initialization thread will bring-up the network
then exit, freeing all of the resources that it required.  This is a
good behavior for systems with limited memory.

If the CONFIG_NSH_NETINIT_MONITOR option is selected, however, then the
network initialization thread will persist forever; it will monitor the
network status.  In the event that the network goes down (for example, if
a cable is removed), then the thread will monitor the link status and
attempt to bring the network back up.  In this case the resources
required for network initialization are never released.

Pre-requisites:

  - CONFIG_NSH_NETINIT_THREAD as described above.

  - CONFIG_NETDEV_PHY_IOCTL. Enable PHY IOCTL commands in the Ethernet
    device driver. Special IOCTL commands must be provided by the Ethernet
    driver to support certain PHY operations that will be needed for link
    management. There operations are not complex and are implemented for
    the Atmel SAMV7 family.

  - CONFIG_ARCH_PHY_INTERRUPT. This is not a user selectable option.
    Rather, it is set when you select a board that supports PHY interrupts.
    In most architectures, the PHY interrupt is not associated with the
    Ethernet driver at all. Rather, the PHY interrupt is provided via some
    board-specific GPIO and the board-specific logic must provide support
    for that GPIO interrupt. To do this, the board logic must do two things:
    (1) It must provide the function arch_phy_irq() as described and
    prototyped in the nuttx/include/nuttx/arch.h, and (2) it must select
    CONFIG_ARCH_PHY_INTERRUPT in the board configuration file to advertise
    that it supports arch_phy_irq().  This logic can be found at
    nuttx/configs/sama5d4-ek/src/sam_ethernet.c.

  - And a few other things: UDP support is required (CONFIG_NET_UDP) and
    signals must not be disabled (CONFIG_DISABLE_SIGNALS).

Given those prerequisites, the network monitor can be selected with these
additional settings.

  Networking Support -> Networking Device Support
    CONFIG_NETDEV_PHY_IOCTL=y             : Enable PHY ioctl support

  Application Configuration -> NSH Library -> Networking Configuration
    CONFIG_NSH_NETINIT_THREAD             : Enable the network initialization thread
    CONFIG_NSH_NETINIT_MONITOR=y          : Enable the network monitor
    CONFIG_NSH_NETINIT_RETRYMSEC=2000     : Configure the network monitor as you like
    CONFIG_NSH_NETINIT_SIGNO=18

Audio Interface
===============

WM8904 Audio Codec
------------------

  SAMV71 Interface        WM8904 Interface
  ---- ------------ ------- ----------------------------------
  PIO  Usage        Pin     Function
  ---- ------------ ------- ----------------------------------
  PA3  TWD0         SDA     I2C control interface, data line
  PA4  TWCK0        SCLK    I2C control interface, clock line
  PA10 RD           ADCDAT  Digital audio output (microphone)
  PB18 PCK2         MCLK    Master clock
  PB0  TF           LRCLK   Left/right data alignment clock
  PB1  TK           BCLK    Bit clock, for synchronization
  PD11 GPIO         IRQ     Audio interrupt
  PD24 RF           LRCLK   Left/right data alignment clock
  PD26 TD           DACDAT  Digital audio input (headphone)
  ---- ------------ ------- ----------------------------------

CP2100-CP Fractional-N Clock Multiplier
--------------------------------------

  SAMV71 Interface          CP2100-CP Interface
  ---- ------------ ------- ----------------------------------
  PIO  Usage        Pin     Function
  ---- ------------ ------- ----------------------------------
  PA3  TWD0         SDA     I2C control interface, data line
  PA4  TWCK0        SCLK    I2C control interface, clock line
  PD21 TIOA11       CLK_IN  PLL input
   -    -           XTI/XTO 12.0MHz crystal
  PA22 RK           CLK_OUT PLL output
   -    -           AUX_OUT N/C
  ---- ------------ ------- ----------------------------------

maXTouch Xplained Pro
=====================

Testing has also been performed using the maXTouch Xplained Pro LCD
(ATMXT-XPRO).

  **************************************************************************
  *  WARNING:                                                              *
  *   The maXTouch chip was not configured on all of the maXTouch Xplained *
  *   Pro boards that I have used (which is two).  The maXTouch is         *
  *   completely non-functional with no configuration in its NV memory!    *
  *                                                                        *
  *   My understanding is that this configuration can be set on Linux      *
  *   using the mxp-app program which is available on GitHub.  There is an *
  *   (awkward) way to do this with NuttX too. In order to set the         *
  *   maXTouch configuration with Nuttx you need to do these things:       *
  *                                                                        *
  *   - Copy the function atmxt_config() from the file                     *
  *     configs/samv71-xult/src/atmxt_config.c into the file               *
  *     drivers/input/mxt.c                                                *
  *   - Add a call to atmxt_config() in drivers/input/mxt.c in the         *
  *     function mxt_register() just before the touchscreen device is      *
  *     registered (i.e, the call to register_driver()).                   *
  *   - Run the code one time.  Your maXTouch is configured and should     *
  *     now work.                                                          *
  *   - Don't forget to remove atmxt_config() from drivers/input/mxt.c and *
  *     restore driver as it was.                                          *
  *                                                                        *
  **************************************************************************


maXTouch Xplained Pro Standard Extension Header
-----------------------------------------------
The LCD could be connected either via EXT1 or EXT2 using the 2x10 20-pin
cable and the maXTouch Xplained Pro standard extension header. Access would
then be performed in SPI mode.

NOTE: There is currently no support for use of the LCD in SPI mode.  See
the next paragraph where the LCD/EXT4 connection is discussion.

NOTE the 3 switch mode selector on the back of the maXtouch.  All switches
should be in the ON position to select 4-wire SPI mode.

  ---- -------- ---- ----------- ---- ----------- ------------------------------------------
                        SAMV71-XULT               maxTouch Xplained Pro
  PIN  FUNCTION EXT1 FUNC        EXT2 FUNC        Description
  ---- -------- ---- ----------- ---- ----------- ------------------------------------------
   1   ID        -    -           -    -          Communication line to ID chip
   2   GND       -    -           -    -          Ground
   3   N/C      PC31  -          PD30  -
   4   N/C      PA19  -          PC13  -
   5   GPIO     PB3  GPIO        PA6  GPIO        Command/Data Select
   6   N/C      PB2   -          PD11  -
   7   PWM      PA0  PWMC0_PWMH0 PC19 PWMC0_PMWH2 Backlight control
   8   N/C      PC30  -          PD26  -
   9   GPIO/IRQ PD28 GPIO        PA2  GPIO        IRQ from maXTouch controller
   10  GPIO     PA5  GPIO        PA24 GPIO        RESET signal for maXTouch and LCD controller
   11  I2C SDA  PA3  TWID0       PA3  TWID0       I2C Data line for maXTouch controller
   12  I2C SCL  PA4  TWICK0      PA4  TWICK0      I2C Clock line for maXTouch controller
   13  N/C      PB0   -          PA21  -
   14  N/C      PB1   -          PB4   -
   15  CS       PD25 GPIO        PD27 GPIO        CS line for LCD controller
   16  SPI MOSI PD21 SPI0_MOSI   PD21 SPI0_MOSI   SPI Data to LCD controller
   17  SPI MISO PD20 SPI0_MISO   PD20 SPI0_MISO   SPI Data from LCD controller
   18  SPI SCK  PD22 SPI0_SPCK   PD22 SPI0_SPCK   SPI Clock line
   19  GND       -    -           -      -        Ground
   20  VCC       -    -           -      -        Target supply voltage
  ---- -------- ---- ----------- ---- ----------- ------------------------------------------

NOTE: Use of EXT1 conflicts with the Arduino RXD pin (PD28).  You cannot
put the maXTouch Xplained in EXT1 and also use the Arduino RXD/TXD pins
as your serial console.

maXTouch Xplained Pro Xplained Pro LCD Connector
------------------------------------------------
It is also possible to connect the LCD via the flat cable to the EXT4 LCD
connector.  In this case, you would use the SMC/EBI to communicate with the
LCD.

NOTE: (1) Only the parallel interface is supported by the SAMV71-XULT and (2)
the 3 switch mode selector on the back of the maXtouch.  These switches should
be in the OFF-ON-OFF positions to select 16-bit color mode.

  ----------------- ------------- -----------------------------------------------------------
         LCD            SAMV71    Description
  Pin  Function     Pin  Function
  ---- ------------ ---- -------- -----------------------------------------------------------
   1   ID            -    -       Communication line to ID chip on extension board
   2   GND           -   GND      Ground
   3   D0           PC0  D0       Data line
   4   D1           PC1  D1       Data line
   5   D2           PC2  D2       Data line
   6   D3           PC3  D3       Data line
   7   GND           -   GND      Ground
   8   D4           PC4  D4       Data line
   9   D5           PC5  D5       Data line
  10   D6           PC6  D6       Data line
  11   D7           PC7  D7       Data line
  12   GND           -   GND      Ground
  13   D8           PE0  D8       Data line
  14   D9           PE1  D9       Data line
  15   D10          PE2  D10      Data line
  16   D11          PE3  D11      Data line
  17   GND           -   GND      Ground
  18   D12          PE4  D12      Data line
  19   D13          PE5  D13      Data line
  20   D14          PA15 D14      Data line
  21   D15          PA16 D15      Data line
  22   GND           -   GND      Ground
  23   D16           -    -       Data line
  24   D17           -    -       Data line
  25   N/C           -    -
  26   N/C           -    -
  27   GND           -   GND      Ground
  28   N/C           -    -
  29   N/C           -    -
  30   N/C           -    -
  31   N/C           -    -
  32   GND           -   GND      Ground
  33   PCLK/        PC30 GPIO     SMC: Pixel clock Display RAM select.
       CMD_DATA_SEL               SPI: One address line of the MCU for displays where it
                                       is possible to select either the register or the
                                       data interface
  34   VSYNC/CS     PD19 NCS3     SMC: Vertical synchronization.
                                  SPI: Chip select
  35   HSYNC/WE     PC8  NWE      SMC: Horizontal synchronization
                                  SPI: Write enable signal
  36   DATA ENABLE/ PC11 NRD      SMC: Data enable signal
       RE                         SPI: Read enable signal
  37   SPI SCK       -    -       SPI: Clock for SPI
  38   SPI MOSI      -    -       SPI: Master out slave in line of SPI
  39   SPI MISO      -    -       SPI: Master in slave out line of SPI
  40   SPI SS        -    -       SPI: Slave select for SPI
  41   N/C           -    -
  42   TWI SDA      PA3  TWD0     I2C data line (maXTouch®)
  43   TWI SCL      PA4  TWCK0    I2C clock line (maXTouch)
  44   IRQ1         PD28 WKUP5    maXTouch interrupt line
  45   N/C          PA2  WKUP2
  46   PWM          PC9  TIOB7    Backlight control
  47   RESET        PC13 GPIO     Reset for both display and maxTouch
  48   VCC           -    -       3.3V power supply for extension board
  49   VCC           -    -       3.3V power supply for extension board
  50   GND           -    -       Ground
  ---- ------------ ---- -------- -----------------------------------------------------------

NOTE: Use of LCD/EXT4 conflicts with the Arduino RXD pin (PD28).  You cannot
put the maXTouch Xplained in LCD/EXT4 and also use the Arduino RXD/TXD pins
as your serial console.

Connecting the flat cable.  I was embarrassed to say that I did not know how
the connectors worked.  Let me share this so that, perhaps, I can save you
the same embarrassment:

- The maXTouch Xplained Pro has an Omron XF2M-5015-1A connector.  There is a
  black bar at back (toward the baord).  Raise that bar and insert the cable
  with the contacts away from the board.  Lower that bar to lock the cable
  in place.

- The SAMV71-Xult has a TE Connectivity 5-1734839-0 FPC connector that works
  differently.  On each size of the connector are two small white tabs.  Pull
  these out and away from the board.  Insert the ribbon with the contacts
  toward the board.  Lock the cable in place by pushing the tabs back in
  place.

MXT Configuration Options
-------------------------

  System Type -> SAMV7 Peripheral Support
    CONFIG_SAMV7_TWIHS0=y                : Needed by the MaXTouch controller
    CONFIG_SAMV7_TWIHS0_FREQUENCY=100000

  Board Selection ->
    CONFIG_SAMV71XULT_MXTXPLND=y          : MaXTouch Xplained is connected
    CONFIG_SAMV71XULT_MXTXPLND_EXT1=y     : Connected on EXT1, or
    CONFIG_SAMV71XULT_MXTXPLND_EXT2=y     : Connected on EXT2, or
    CONFIG_SAMV71XULT_MXTXPLND_LCD=y      : Connected on LCD
    CONFIG_SAMV71XULT_MXT_DEVMINOR=0      : Register as /dev/input0
    CONFIG_SAMV71XULT_MXT_I2CFREQUENCY=400000

  NOTE: When selecting EXT1 or EXT2, be conscious of possible pin conflicts.
  EXT1, for example, will conflict with the use of the Arduino TXD and RXD
  pins for the serial console

  Device Drivers -> Input Devices
    CONFIG_INPUT=y                        : Enable support for human input devices
    CONFIG_INPUT_MXT=y                    : Enable support for the maXTouch controller

  The following enables a small built-in application that can be used to
  test the touchscreen:

  Application Configuration -> Examples -> Touchscreen example
    CONFIG_EXAMPLES_TOUCHSCREEN=y          : Enables the example
    CONFIG_EXAMPLES_TOUCHSCREEN_ARCHINIT=y : Have board-specific intialization
    CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH="/dev/input0"
    CONFIG_EXAMPLES_TOUCHSCREEN_MINOR=0

ILI9488 Configuration Options
-----------------------------

Currently only the parallel mode is supported.  This means that the LCD can
only be used in connected in the LCD (EXT4) connection.

  System Type -> SAMV7 Peripheral Support
    CONFIG_SAMV7_SMC=y                    : Needed by the ILI9466 driver controller
    CONFIG_SAMV7_XDMAC=y                  : Needed by the ILI9466 driver
    CONFIG_SAMV7_TWIHS0_FREQUENCY=100000

  Board Selection ->
    CONFIG_SAMV71XULT_MXTXPLND=y          : MaXTouch Xplained is connected
    CONFIG_SAMV71XULT_MXTXPLND_LCD=y      : Must be connected on LCD

  NOTE: When selecting EXT1 or EXT2, be conscious of possible pin conflicts.
  EXT1, for example, will conflict with the use of the Arduino TXD and RXD
  pins for the serial console

  Device Drivers -> LCD drivers
    CONFIG_LCD=y                          : Enable support for LCDs

  Graphics
    CONFIG_NX=y                           : Enable Graphics supported
    CONFIG_NX_LCDDRIVER=y                 : Enable LCD driver support
    CONFIG_NX_DISABLE_*BPP=y              : When * is {1,2,4,8,24, and 32}
    CONFIG_NXFONTS_CHARBITS=7
    CONFIG_NXFONT_SANS23X27=y             : One font must be enabled

  There are several graphics examples that can be enabled under apps/examples.
  nxlines is one of these and can be enabled as follows.  See
  apps/examples/README.txt for information about configuring other graphics
  examples.

  The following enables a small built-in application that can be used to
  test the touchscreen:

  Application Configuration -> Examples -> NX lines example
    CONFIG_EXAMPLES_NXLINES=y              : Enables the nxlines example
    CONFIG_EXAMPLES_NXLINES_VPLANE=0
    CONFIG_EXAMPLES_NXLINES_DEVNO=0

Debugging
=========

The on-board EDBG appears to work only with Atmel Studio.  You can however,
simply connect a SAM-ICE or J-Link to the JTAG/SWD connector on the board
and that works great.  The only tricky thing is getting the correct
orientation of the JTAG connection.

I have been using Atmel Studio to write code to flash then I use the Segger
J-Link GDB server to debug.  I have been using the 'Device Programming' I
available under the Atmel Studio 'Tool' menu.  I have to disconnect the
SAM-ICE while programming with the EDBG.  I am sure that you could come up
with a GDB server-only solution if you wanted.

I run GDB like this from the directory containing the NuttX ELF file:

  arm-none-eabi-gdb
  (gdb) target remote localhost:2331
  (gdb) mon reset
  (gdb) file nuttx
  (gdb) ... start debugging ...

Configurations
==============

Information Common to All Configurations
----------------------------------------
Each SAMV71-XULT configuration is maintained in a sub-directory and
can be selected as follow:

  cd tools
  ./configure.sh samv71-xult/<subdir>
  cd -
  . ./setenv.sh

Before sourcing the setenv.sh file above, you should examine it and perform
edits as necessary so that TOOLCHAIN_BIN is the correct path to the directory
than holds your toolchain binaries.

And then build NuttX by simply typing the following.  At the conclusion of
the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

  make oldconfig
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
     output on UART3 (i.e., for the Arduino serial shield).

  3. All of these configurations are set up to build under Windows using the
     "GNU Tools for ARM Embedded Processors" that is maintained by ARM
     (unless stated otherwise in the description of the configuration).

       https://launchpad.net/gcc-arm-embedded

     As of this writing (2015-03-11), full support is difficult to find
     for the Cortex-M&, but is supported by at least this realeasse of
     the ARM GNU tools:

       https://launchpadlibrarian.net/192228215/release.txt

     That toolchain selection can easily be reconfigured using
     'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Window environment
       CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : GNU ARM EABI toolchain

     NOTE: As of this writing, there are issues with using this tool at
     the -Os level of optimization.  This has not been proven to be a
     compiler issue (as least not one that might not be fixed with a
     well placed volatile qualifier).  However, in any event, it is
     recommend that you use not more that -O2 optimization.

Configuration sub-directories
-----------------------------

  mxtxplnd:

    Configures the NuttShell (nsh) located at examples/nsh.  There are three
    very similar NSH configurations:

      - nsh.  This configuration is focused on low level, command-line
        driver testing.  It has no network.
      - netnsh.  This configuration is focused on network testing and
        has only limited command support.
      - mxtxplnd.  This configuration is identical to the nsh configuration
        but assumes that you have a maXTouch Xplained Pro LCD attached
        and includes extra tests for the touchscreen and LCD.

    NOTES:

    1. See the notes associated with the nsh configuration below.  Only
       differences from that configuration will be addressed here.

    2. Basic touchscreen/LCD configuration settings are discussed above in
       the paragraph entitled, "maXTouch Xplained Pro".

    3. Unlike the nsh configuration, this configuration has the serial console
       setup to USART0 which is available on EXT1:

         ----------- --- ------- -----
         Connector   PIO Arduino SAMV7
         ----------- --- ------- -----
         EXT1 pin 13 PB0 RX3     RXD0
         EXT1 pin 14 PB1 TX3     TXD0
         ----------- --- ------- -----

       and also on the Arduino Communications connector (J505):

         ----------- --- ------- -----
         Connector   PIO Arduino SAMV7
         ----------- --- ------- -----
         J505 pin 7  PB0 RX3     RXD0
         J505 pin 8  PB1 TX3     TXD0
         ----------- --- ------- -----

       Use of either the EXT1 or the LCD/EXT4 connectors conflict with the
       Arduino RXD pin (UART3, PD28).  You cannot put the maXTouch Xplained
       in EXT1 or LCD/EXT4 and also use the Arduino RXD/TXD pins as your
       serial console.

       The LCD (EXT4) is configured by default because only the parallel LCD
       interface is currently supported and that is only available on that
       connector.

       If you plan to use EXT2 for some reason, you may re-configure the
       serial console to use UART3, the standard Arduino RXD/TXD.  You
       would also, of course, have to disable the LCD.

       NOTE that the USART0 pins PB0 and PB1 conflict with SSC TF and TK
       pins as connected to the WM8904 audio CODEC.  So, unless yet a
       different U[S]ART option is selected, Audio cannot be used with
       this configuration.

    4. SDRAM is NOT enabled in this configuration.

    5. Support for the ILI8488 LCD is enabled.  Only the parallel mode is
       supported at present.  As a consequence, the maXTouch Xplained Pro
       must be connected at the LCD (EXT4) connector.  This mode requires:

         CONFIG_SAMV71XULT_MXTXPLND_LCD=y : Must be connect in LCD (EXT4)
         CONFIG_SAMV7_SMC=y               : SMC/EBI support
         CONFIG_SAMV7_XDMAC=y             : XDMAC support

    6. The appx/examples/nxlines is enabled as a built-in application.
       This is a test that displays some simple graphis and can be
       executed from the NSH command line like:

         nsh> nxlines

    7. When the maXTouch Xplained is connected (in any position), a new I2C
       address appears at address 0x4a:

        nsh> i2c dev 3 77
             0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
        00:          -- -- -- -- -- -- -- -- -- -- -- -- --
        10: -- -- -- -- -- -- -- -- -- -- 1a -- -- -- -- --
        20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
        30: -- -- -- -- -- -- -- 37 -- -- -- -- -- -- -- --
        40: -- -- -- -- -- -- -- -- -- -- 4a -- -- -- 4e --
        50: -- -- -- -- -- -- -- 57 -- -- -- -- -- -- -- 5f
        60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        70: -- -- -- -- -- -- -- --

       This is the I2C address of the maXTouch touchscreen controller.

       (0x1a is the address of the WM8904 Audio CODEC, 0x28 is the
        address of TWI interface to the EDBG, 0x4e is the address of the
        CP2100CP programmable PLL, and 0x57 and 0x5f are the addresses of
        the AT2 EEPROM. I am not sure what the other address, 0x37, is).

    8. Support for the touchscreen test is enabled (see apps/examples/touchscreen),
       however, the maXTouch is not yet working (see STATUS below).

    STATUS:
      2015-04-05:  Partial support for the maXTouch Xplained Pro LCD is in
        place.  The ILI9488-based LCD is working well with a SMC DMA-based
        interface.  Very nice performance.
      2015-05-12:  After some difficulties, the maXTouch touchscreen
        controller is now fully functional as well.

  netnsh:

    Configures the NuttShell (nsh) located at examples/nsh.  There are three
    very similar NSH configurations:

      - nsh.  This configuration is focused on low level, command-line
        driver testing.  It has no network.
      - netnsh.  This configuration is focused on network testing and
        has only limited command support.
      - mxtxplnd.  This configuration is identical to the nsh configuration
        but assumes that you have a maXTouch Xplained Pro LCD attached
        and includes extra tests for the touchscreen and LCD.

    NOTES:

    1. The serial console is configured by default for use with and Arduino
       serial shield (UART3).  You will need to reconfigure if you will
       to use a different U[S]ART.

    2. Default stack sizes are large and should really be tuned to reduce
       the RAM footprint:

         CONFIG_SCHED_HPWORKSTACKSIZE=2048
         CONFIG_IDLETHREAD_STACKSIZE=1024
         CONFIG_USERMAIN_STACKSIZE=2048
         CONFIG_PTHREAD_STACK_MIN=256
         CONFIG_PTHREAD_STACK_DEFAULT=2048
         CONFIG_POSIX_SPAWN_PROXY_STACKSIZE=1024
         CONFIG_TASK_SPAWN_DEFAULT_STACKSIZE=2048
         CONFIG_BUILTIN_PROXY_STACKSIZE=1024
         CONFIG_NSH_TELNETD_DAEMONSTACKSIZE=2048
         CONFIG_NSH_TELNETD_CLIENTSTACKSIZE=2048

    3. NSH built-in applications are supported.  There are, however, not
       enabled built-in applications.

       Binary Formats:
         CONFIG_BUILTIN=y           : Enable support for built-in programs

       Application Configuration:
         CONFIG_NSH_BUILTIN_APPS=y  : Enable starting apps from NSH command line

    4. The network initialization thread is NOT enabled in this configuration.
       As a result, networking initialization is performed serially with
       NSH bring-up.  The time from reset to the NSH prompt will be determined
       primarily by this network initialization time.  And can be especially
       long, perhaps minutes, if the network cable is not connected!

       If fast boot times are required, you need to perform asynchronous
       network initialization as described under "Network Initialization Thread"

    5. SDRAM is NOT enabled in this configuration.

    6. TWI/I2C

       TWIHS0 is enabled in this configuration.  The SAM V71 Xplained Ultra
       supports two devices on the one on-board I2C device on the TWIHS0 bus:
       (1) The AT24MAC402 serial EEPROM described above and (2) the Wolfson
       WM8904 audio CODEC.  This device contains a MAC address for use with
       the Ethernet interface.

       Relevant configuration settings:

         CONFIG_SAMV7_TWIHS0=y
         CONFIG_SAMV7_TWIHS0_FREQUENCY=100000

         CONFIG_I2C=y
         CONFIG_I2C_TRANSFER=y

    7. TWIHS0 is used to support 256 byte non-volatile storage.  This EEPROM
       holds the assigned MAC address which is necessary for networking. The
       EEPROM is also available for storage of configuration data using the
       MTD configuration as described above under the heading, "MTD
       Configuration Data".

    8. Support for HSMCI is built-in by default. The SAMV71-XULT provides
       one full-size SD memory card slot.  Refer to the section entitled
       "SD card" for configuration-related information.

       See "Open Issues" above for issues related to HSMCI.

       The auto-mounter is not enabled.  See the section above entitled
       "Auto-Mounter".

    9. Performance-related Configuration settings:

       CONFIG_ARMV7M_ICACHE=y                : Instruction cache is enabled
       CONFIG_ARMV7M_DCACHE=y                : Data cache is enabled
       CONFIG_ARMV7M_DCACHE_WRITETHROUGH=y   : Write through mode
       CONFIG_ARCH_FPU=y                     : H/W floating point support is enabled
       CONFIG_ARCH_DPFPU=y                   : 64-bit H/W floating point support is enabled

       # CONFIG_ARMV7M_ITCM is not set       : Support not yet in place
       # CONFIG_ARMV7M_DTCM is not set       : Support not yet in place

       I- and D-Caches are enabled but the D-Cache must be enabled in write-
       through mode.  This is to work around issues with the RX and TX
       descriptors with are 8-bytes in size.  But the D-Cache cache line
       size is 32-bytes.  That means that you cannot reload, clean or
       invalidate a descriptor without also effecting three neighboring
       descriptors. Setting write through mode eliminates the need for
       cleaning the D-Cache.  If only reloading and invalidating are done,
       then there is no problem.

       Stack sizes are also large to simplify the bring-up and should be
       tuned for better memory usages.

    STATUS:
    2015-03-29:  I- and D-caches are currently enabled, but as noted
      above, the D-Cache must be enabled in write-through mode.  Also -Os
      optimization is not being used (-O2).  If the cache is enabled in
      Write-Back mode or if higher levels of optimization are enabled, then
      there are failures when trying to ping the target from a host.

  nsh:

    Configures the NuttShell (nsh) located at examples/nsh.  There are three
    very similar NSH configurations:

      - nsh.  This configuration is focused on low level, command-line
        driver testing.  It has no network.
      - netnsh.  This configuration is focused on network testing and
        has only limited command support.
      - mxtxplnd.  This configuration is identical to the nsh configuration
        but assumes that you have a maXTouch Xplained Pro LCD attached
        and includes extra tests for the touchscreen and LCD.

    NOTES:

    1. The serial console is configured by default for use with and Arduino
       serial shield (UART3).  You will need to reconfigure if you will
       to use a different U[S]ART.

    2. Default stack sizes are large and should really be tuned to reduce
       the RAM footprint:

         CONFIG_ARCH_INTERRUPTSTACK=2048
         CONFIG_IDLETHREAD_STACKSIZE=1024
         CONFIG_USERMAIN_STACKSIZE=2048
         CONFIG_PTHREAD_STACK_DEFAULT=2048
         ... and others ...

    3. NSH built-in applications are supported.

       Binary Formats:
         CONFIG_BUILTIN=y           : Enable support for built-in programs

       Application Configuration:
         CONFIG_NSH_BUILTIN_APPS=y  : Enable starting apps from NSH command line

    4. SDRAM is enabled in this configuration.  Here are the relevant
       configuration settings:

       System Type
         CONFIG_SAMV7_SDRAMC=y
         CONFIG_SAMV7_SDRAMSIZE=2097152

       SDRAM is not added to the heap in this configuration.  To do that
       you would need to set CONFIG_SAMV7_SDRAMHEAP=y and CONFIG_MM_REGIONS=2.
       Instead, the SDRAM is set up so that is can be used with a destructive
       RAM test enabled with this option:

       Application Configuration:
         CONFIG_SYSTEM_RAMTEST=y

       The RAM test can be executed as follows:

         nsh> ramtest -w 70000000 2097152

         NuttShell (NSH) NuttX-7.8
         nsh> ramtest -w 70000000 2097152
         RAMTest: Marching ones: 70000000 2097152
         RAMTest: Marching zeroes: 70000000 2097152
         RAMTest: Pattern test: 70000000 2097152 55555555 aaaaaaaa
         RAMTest: Pattern test: 70000000 2097152 66666666 99999999
         RAMTest: Pattern test: 70000000 2097152 33333333 cccccccc
         RAMTest: Address-in-address test: 70000000 2097152
         nsh>

    5. The button test at apps/examples/buttons is included in the
       configuration.  This configuration illustrates (1) use of the buttons
       on the evaluation board, and (2) the use of PIO interrupts.  Example
       usage:

       NuttShell (NSH) NuttX-7.8
       nsh> help
       help usage:  help [-v] [<cmd>]
       ...
       Builtin Apps:
         buttons
       nsh> buttons 3
       maxbuttons: 3
       Attached handler at 4078f7 to button 0 [SW0], oldhandler:0
       Attached handler at 4078e9 to button 1 [SW1], oldhandler:0
       IRQ:125 Button 1:SW1 SET:00:
         SW1 released
       IRQ:125 Button 1:SW1 SET:02:
         SW1 depressed
       IRQ:125 Button 1:SW1 SET:00:
         SW1 released
       IRQ:90 Button 0:SW0 SET:01:
         SW0 depressed
       IRQ:90 Button 0:SW0 SET:00:
         SW0 released
       IRQ:125 Button 1:SW1 SET:02:
         SW1 depressed
       nsh>

    6. TWI/I2C

       TWIHS0 is enabled in this configuration.  The SAM V71 Xplained Ultra
       supports two devices on the one on-board I2C device on the TWIHS0 bus:
       (1) The AT24MAC402 serial EEPROM described above and (2) the Wolfson
       WM8904 audio CODEC.  This device contains a MAC address for use with
       the Ethernet interface.

       In this configuration, the I2C tool at apps/system/i2ctool is
       enabled.  This tools supports interactive access to I2C devices on
       the enabled TWIHS bus.  Relevant configuration settings:

         CONFIG_SAMV7_TWIHS0=y
         CONFIG_SAMV7_TWIHS0_FREQUENCY=100000

         CONFIG_I2C=y
         CONFIG_I2C_TRANSFER=y

         CONFIG_SYSTEM_I2CTOOL=y
         CONFIG_I2CTOOL_MINBUS=0
         CONFIG_I2CTOOL_MAXBUS=0
         CONFIG_I2CTOOL_MINADDR=0x03
         CONFIG_I2CTOOL_MAXADDR=0x77
         CONFIG_I2CTOOL_MAXREGADDR=0xff
         CONFIG_I2CTOOL_DEFFREQ=400000

       Example usage:

         nsh> i2c
         Usage: i2c <cmd> [arguments]
         Where <cmd> is one of:

           Show help     : ?
           List busses   : bus
           List devices  : dev [OPTIONS] <first> <last>
           Read register : get [OPTIONS] [<repititions>]
           Show help     : help
           Write register: set [OPTIONS] <value> [<repititions>]
           Verify access : verf [OPTIONS] [<value>] [<repititions>]

         Where common "sticky" OPTIONS include:
           [-a addr] is the I2C device address (hex).  Default: 03 Current: 03
           [-b bus] is the I2C bus number (decimal).  Default: 0 Current: 0
           [-r regaddr] is the I2C device register address (hex).  Default: 00 Current: 00
           [-w width] is the data width (8 or 16 decimal).  Default: 8 Current: 8
           [-s|n], send/don't send start between command and data.  Default: -n Current: -n
           [-i|j], Auto increment|don't increment regaddr on repititions.  Default: NO Current: NO
           [-f freq] I2C frequency.  Default: 400000 Current: 400000

         NOTES:
         o An environment variable like $PATH may be used for any argument.
         o Arguments are "sticky".  For example, once the I2C address is
           specified, that address will be re-used until it is changed.

         WARNING:
         o The I2C dev command may have bad side effects on your I2C devices.
           Use only at your own risk.
         nsh> i2c bus
          BUS   EXISTS?
         Bus 0: YES
         nsh> i2c dev 3 77
              0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
         00:          -- -- -- -- -- -- -- -- -- -- -- -- --
         10: -- -- -- -- -- -- -- -- -- -- 1a -- -- -- -- --
         20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
         30: -- -- -- -- -- -- -- 37 -- -- -- -- -- -- -- --
         40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- 4e --
         50: -- -- -- -- -- -- -- 57 -- -- -- -- -- -- -- 5f
         60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         70: -- -- -- -- -- -- -- --
         nsh>

       Where 0x1a is the address of the WM8904 Audio CODEC, 0x28 is the
       address of TWI interface to the EDBG, 0x4e is the address of the
       CP2100CP programmable PLL, and 0x57 and 0x5f are the addresses of
       the AT2 EEPROM (I am not sure what the other address, 0x37, is
       as this writing).

    7. TWIHS0 is also used to support 256 byte non-volatile storage for
       configuration data using the MTD configuration as described above
       under the heading, "MTD Configuration Data".

    8. Support for HSMCI is built-in by default. The SAMV71-XULT provides
       one full-size SD memory card slot.  Refer to the section entitled
       "SD card" for configuration-related information.

       See "Open Issues" above for issues related to HSMCI.

       The auto-mounter is not enabled.  See the section above entitled
       "Auto-Mounter".

    9. Performance-related Configuration settings:

       CONFIG_ARMV7M_ICACHE=y                : Instruction cache is enabled
       CONFIG_ARMV7M_DCACHE=y                : Data cache is enabled
       CONFIG_ARMV7M_DCACHE_WRITETHROUGH=n   : Write back mode
       CONFIG_ARCH_FPU=y                     : H/W floating point support is enabled
       CONFIG_ARCH_DPFPU=y                   : 64-bit H/W floating point support is enabled

       # CONFIG_ARMV7M_ITCM is not set       : Support not yet in place
       # CONFIG_ARMV7M_DTCM is not set       : Support not yet in place

       Stack sizes are also large to simplify the bring-up and should be
       tuned for better memory usages.

    STATUS:
    2015-03-28: HSMCI TX DMA is disabled.  There are some issues with the TX
      DMA that need to be corrected.

  nxwm:

    This is a special configuration setup for the NxWM window manager
    UnitTest.  It provides an interactive windowing experience with the
    maXTouch Xplained Pro LCD.

    NOTES:

    1. The NxWM window manager is a tiny window manager tailored for use
       with smaller LCDs.  It supports a task, a start window, and
       multiple application windows with toolbars.  However, to make the
       best use of the visible LCD space, only one application window is
       visible at at time.

       The NxWM window manager can be found here:

         nuttx-git/NxWidgets/nxwm

       The NxWM unit test can be found at:

         nuttx-git/NxWidgets/UnitTests/nxwm

       Documentation for installing the NxWM unit test can be found here:

         nuttx-git/NxWidgets/UnitTests/README.txt

    2. Here is the quick summary of the build steps.  These steps assume
       that you have the entire NuttX GIT in some directory ~/nuttx-git.
       You may have these components installed elsewhere.  In that case, you
       will need to adjust all of the paths in the following accordingly:

       a. Install the nxwm configuration

          $ cd ~/nuttx-git/nuttx/tools
          $ ./configure.sh samv71-xult/nxwm

       b. Make the build context (only)

          $ cd ..
          $ . ./setenv.sh
          $ make context
          ...

          NOTE: the use of the setenv.sh file is optional.  All that it will
          do is to adjust your PATH variable so that the build system can find
          your tools.  If you use it, you will most likely need to modify the
          script so that it has the correct path to your tool binaries
          directory.

       c. Install the nxwm unit test

          $ cd ~/nuttx-git/NxWidgets
          $ tools/install.sh ~/nuttx-git/apps nxwm
          Creating symbolic link
           - To ~/nuttx-git/NxWidgets/UnitTests/nxwm
           - At ~/nuttx-git/apps/external

       d. Build the NxWidgets library

          $ cd ~/nuttx-git/NxWidgets/libnxwidgets
          $ make TOPDIR=~/nuttx-git/nuttx
          ...

       e. Build the NxWM library

          $ cd ~/nuttx-git/NxWidgets/nxwm
          $ make TOPDIR=~/nuttx-git/nuttx
          ...

       f. Built NuttX with the installed unit test as the application

          $ cd ~/nuttx-git/nuttx
          $ make

    3. Reading from the LCD is not currently functional.  The following
       settings are in the configuration that tell the system that this
       is a read-only LCD:

         CONFIG_LCD_NOGETRUN=y
         CONFIG_NX_WRITEONLY=y

    4. Small Icons are selected and can be very difficult to touch.  You
       might want to enable larger icons with:

         CONFIG_NXWM_LARGE_ICONS=y

    STATUS:
    2015-05-13:
      - The demo functions and produces displays but is not yet very stable.

      - I have two maXTouch  Xplained Pro displays.  One works well, the
        other has some issues which I suspect are due to the ribbon cable
        connector with fits too snugly on one side.

        Here are the symptoms of the LCD that does not work.  I attribute
        these problems with problems in the parallel interface due to a
        bad connection:

        - The color is wrong; to reddish.  This suggests some issue with color
          format or pixel width
        - Images are positioned correctly on the display, but all half the
          horizontal width that they should be, again suggesting some problem
          with the pixel with.
        - Some images are simply truncated to half the correct size (such as
          the touch circles in the calibration screen).
        - Other images are horizontally compressed (such as the initial NX
          logo on the background).

      - As mentioned above, reading fromthe LCD is not currently functional.
        There are some special settings work work around this but the
        bottom line is that transparent operations cannot yet be supported.

      - I am seeing some small artifacts with the font used in the HEX
        calculator display.

      - Line spacing in the NxTerm window is too much.  This is probably
        a font-related issue too.
