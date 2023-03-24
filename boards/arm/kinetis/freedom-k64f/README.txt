README.txt
==========

This is the README file for the port of NuttX to the Freescale Freedom-K64F
development board.

Contents
========

  o Freedom K64F Features
  o Serial Console
  o LEDs and Buttons
  o Networking Support
  o SD Card Support
  o USB Device Controller Support
  o Development Environment
  o GNU Toolchain Options
  o Freedom K64F Configuration Options
  o Configurations
  o Status

Kinetis Freedom K64F Features:
=============================

  The features of the FRDM-K64F hardware are as follows:

  - MK64FN1M0VLL12 MCU (120 MHz, 1 MB flash memory, 256 KB RAM, low-power,
    crystal-less USB, and 100 LQFP)
  - Dual role USB interface with micro-B USB connector
  - RGB LED
  - FXOS8700CQ - accelerometer and magnetometer
  - Two user push buttons
  - Flexible power supply option - OpenSDAv2 USB, K64 USB, and external
    source
  - Easy access to MCU input/output through Arduino R3TM compatible I/O
    connectors
  - Programmable OpenSDAv2 debug circuit supporting the CMSIS-DAP Interface
    software that provides:
    o Mass storage device (MSD) flash programming interface
    o CMSIS-DAP debug interface over a driver-less USB HID connection
       providing run-control debugging and compatibility with IDE tools
    o Virtual serial port interface
    o Open-source CMSIS-DAP software project: github.com/mbedmicro/CMSIS-DAP.
  - Ethernet
  - SDHC
  - Add-on RF module: nRF24L01+ Nordic 2.4GHz Radio
  - Add-on Bluetooth module: JY-MCU BT board V1.05 BT

OpenSDAv2
=========

  The FRDM-K64F platform features OpenSDAv2, the Freescale open-source
  hardware embedded serial and debug adapter running an open-source
  bootloader. This circuit offers several options for serial communication,
  flash programming, and run-control debugging. OpenSDAv2 is an mbed
  HDK-compatible debug interface preloaded with the open-source CMSIS-DAP
  Interface firmware (mbed interface) for rapid prototyping and product
  development.

  To use set raw binary output for nuttx.bin

Serial Console
==============

  USB VCOM Console
  ----------------
  The primary serial port interface signals are PTB16 UART0_RX and PTB17
  UART0_TX. These signals are connected to the OpenSDAv2 VCOM circuit.

  Serial Shield Console
  ---------------------
  An alternative serial port might use a standard serial shield mounted
  on the Freedom Board.  In this case, Arduino pin D1 provides UART TX and
  pin D0 provides UART RX.

  The I/O headers on the FRDM-K64F board are arranged to enable
  compatibility with Arduino shield. The outer rows of pins (even numbered
  pins) on the headers, share the same mechanical spacing and placement with
  the I/O headers on the Arduino Revision 3 (R3) standard.

  The Arduino D0 and D1 pins then correspond to pins 2 and 4 on the J1 I/O
  connector:

    Arduino Pin              FRDM-K64F J1 Connector
    ------------------------ -----------------------
    UART TX, Arduino D1 pin  Pin 4, PTC17, UART3_TX
    UART RX, Arduino D0 pin  Pin 2, PTC16, UART3_RX

  Default Serial Console
  ----------------------
  By default, these configuration are setup to use the Serial Console on
  UART3.  That, however, is easily reconfigured.

LEDs and Buttons
================

  RGB LED
  -------
  An RGB LED is connected through GPIO as shown below:

    LED    K64
    ------ -------------------------------------------------------
    RED    PTB22/SPI2_SOUT/FB_AD29/CMP2_OUT
    BLUE   PTB21/SPI2_SCK/FB_AD30/CMP1_OUT
    GREEN  PTE26/ENET_1588_CLKIN/UART4_CTS_b/RTC_CLKOUT/USB0_CLKIN

  If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board the
  Freedom KL25Z.  Usage of these LEDs is defined in include/board.h and
  src/k64_leds.c.  The following definitions describe how NuttX controls the LEDs:

    SYMBOL                Meaning                 LED state
                                                  RED   GREEN  BLUE
    -------------------  -----------------------  -----------------
    LED_STARTED          NuttX has been started    OFF  OFF  OFF
    LED_HEAPALLOCATE     Heap has been allocated   OFF  OFF  ON
    LED_IRQSENABLED      Interrupts enabled        OFF  OFF  ON
    LED_STACKCREATED     Idle stack created        OFF  ON   OFF
    LED_INIRQ            In an interrupt          (no change)
    LED_SIGNAL           In a signal handler      (no change)
    LED_ASSERTION        An assertion failed      (no change)
    LED_PANIC            The system has crashed    FLASH OFF OFF
    LED_IDLE             K64 is in sleep mode     (Optional, not used)

  Buttons
  -------
  Two push buttons, SW2 and SW3, are available on FRDM-K64F board, where
  SW2 is connected to PTC6 and SW3 is connected to PTA4. Besides the
  general purpose input/output functions, SW2 and SW3 can be low-power
  wake up signal. Also, only SW3 can be a non-maskable interrupt.

  Switch    GPIO Function
  --------- ---------------------------------------------------------------
  SW2       PTC6/SPI0_SOUT/PD0_EXTRG/I2S0_RX_BCLK/FB_AD9/I2S0_MCLK/LLWU_P10
  SW3       PTA4/FTM0_CH1/NMI_b/LLWU_P3

Networking Support
==================

  Ethernet MAC/KSZ8081 PHY
  ------------------------
  ------------ ----------------- --------------------------------------------
  KSZ8081      Board Signal(s)   K64F Pin
  Pin Signal                     Function                       pinmux Name
  --- -------- ----------------- --------------------------------------------
   1  VDD_1V2  VDDPLL_1.2V       ---                            ---
   2  VDDA_3V3 VDDA_ENET         ---                            ---
   3  RXM      ENET1_RX-         ---                            ---
   4  RXP      ENET1_RX+         ---                            ---
   5  TXM      ENET1_TX-         ---                            ---
   6  TXP      ENET1_TX+         ---                            ---
   7  X0       RMII_XTAL0        ---                            ---
   8  XI       RMII_XTAL1        ---                            ---
   9  REXT     ---               ---, Apparently not connected  ---
  10  MDIO     RMII0_MDIO        PTB0/RMII0_MDIO                PIN_RMII0_MDIO
  11  MDC      RMII0_MDC         PTB1/RMII0_MDC                 PIN_RMII0_MDC
  12  RXD1     RMII0_RXD_1       PTA12/RMII0_RXD1               PIN_RMII0_RXD1
  13  RXD0     RMII0_RXD_0       PTA13/RMII0_RXD0               PIN_RMII0_RXD0
  14  VDDIO    VDDIO_ENET        ---                            ---
  15  CRS_DIV                    PTA14/RMII0_CRS_DV             PIN_RMII0_CRS_DV
  16  REF_CLK  RMII_RXCLK        PTA18/EXTAL0, PHY clock input  ---
  17  RXER     RMII0_RXER        PTA5/RMII0_RXER                PIN_RMII0_RXER
  18  INTRP    RMII0_INT_B,      J14 Pin 2, Apparently not      ---
               PHY_INT_1         available unless jumpered
  19  TXEN     RMII0_TXEN        PTA15/RMII0_TXEN               PIN_RMII0_TXEN
  20  TXD0     RMII0_TXD_0       PTA16/RMII0_TXD0               PIN_RMII0_TXD0
  21  TXD1     RMII0_TXD_1       PTA17/RMII0_TXD1               PIN_RMII0_TXD1
  22  GND1     ---               ---                            ---
  24  nRST     PHY_RST_B         ---                            ---
  25  GND2     ---               ---                            ---
  --- -------- ----------------- --------------------------------------------

  No external pullup is available on MDIO signal when MK64FN1M0VLL12 MCU is
  requests status of the Ethernet link connection. Internal pullup is required
  when port configuration for MDIO signal is enabled:

    CONFIG_KINETIS_ENET_MDIOPULLUP=y

  Networking support via the can be added to NSH by selecting the following
  configuration options.

  Selecting the EMAC peripheral
  -----------------------------

  System Type -> Kinetis Peripheral Support
    CONFIG_KINETIS_ENET=y               : Enable the EThernet MAC peripheral

  System Type -> Ethernet Configuration
    CONFIG_KINETIS_ENETNETHIFS=1
    CONFIG_KINETIS_ENETNRXBUFFERS=6
    CONFIG_KINETIS_ENETNTXBUFFERS=2
    CONFIG_KINETIS_ENET_MDIOPULLUP=y

  Networking Support
    CONFIG_NET=y                        : Enable Neworking
    CONFIG_NET_ETHERNET=y               : Support Ethernet data link
    CONFIG_NET_SOCKOPTS=y               : Enable socket operations
    CONFIG_NET_ETH_PKTSIZE=590          : Maximum packet size 1518 is more standard
    CONFIG_NET_ARP=y                    : Enable ARP
    CONFIG_NET_ARPTAB_SIZE=16           : ARP table size
    CONFIG_NET_ARP_IPIN=y               : Enable ARP address harvesting
    CONFIG_NET_ARP_SEND=y               : Send ARP request before sending data
    CONFIG_NET_TCP=y                    : Enable TCP/IP networking
    CONFIG_NET_TCP_WRITE_BUFFERS=y      : Support TCP write-buffering
    CONFIG_NET_TCPBACKLOG=y             : Support TCP/IP backlog
    CONFIG_NET_MAX_LISTENPORTS=20       :
    CONFIG_NET_UDP=y                    : Enable UDP networking
    CONFIG_NET_BROADCAST=y              : Needed for DNS name resolution
    CONFIG_NET_ICMP=y                   : Enable ICMP networking
    CONFIG_NET_ICMP_SOCKET=y            : Needed for NSH ping command
                                        : Defaults should be okay for other options
f Application Configuration -> Network Utilities
    CONFIG_NETDB_DNSCLIENT=y               : Enable host address resolution
    CONFIG_NETUTILS_TELNETD=y           : Enable the Telnet daemon
    CONFIG_NETUTILS_TFTPC=y             : Enable TFTP data file transfers for get and put commands
    CONFIG_NETUTILS_NETLIB=y            : Network library support is needed
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

  By default, the IP address of the FRDM-K64F will be 10.0.0.2 and
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

  On the host side, you should also be able to ping the FRDM-K64F:

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
      df          hexdump     mb          mv          source

    Builtin Apps:
    nsh>

  NOTE:  If you enable this networking as described above, you will
  experience a delay on booting NSH.  That is because the start-up logic
  waits for the network connection to be established before starting
  NuttX.  In a real application, you would probably want to do the
  network bringup on a separate thread so that access to the NSH prompt
  is not delayed.

  This delay will be especially long if the board is not connected to
  a network.  On the order of minutes!  You will probably think that
  NuttX has crashed!  And then, when it finally does come up after
  numerous timeouts and retries, the network will not be available --
  even if the network cable is plugged in later.

  The long delays can be eliminated by using a separate the network
  initialization thread discussed below.  Recovering after the network
  becomes available requires the network monitor feature, also discussed
  below.

  Network Initialization Thread
  -----------------------------
  There is a configuration option enabled by CONFIG_NSH_NETINIT_THREAD
  that will do the NSH network bring-up asynchronously in parallel on
  a separate thread.  This eliminates the (visible) networking delay
  altogether.  This current implementation, however, has some limitations:

    - If no network is connected, the network bring-up will fail and
      the network initialization thread will simply exit.  There are no
      retries and no mechanism to know if the network initialization was
      successful (it could perform a network Ioctl to see if the link is
      up and it now, keep trying, but it does not do that now).

    - Furthermore, there is currently no support for detecting loss of
      network connection and recovery of the connection (similarly, this
      thread could poll periodically for network status, but does not).

  Both of these shortcomings could be eliminated by enabling the network
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

    - The K64F EMAC block does not support PHY interrupts.  The KSZ8081
      PHY interrupt line is brought to a jumper block and it should be
      possible to connect that some some interrupt port pin.  You would
      need to provide some custom logic in the Freedcom K64F
      configuration to set up that PHY interrupt.

    - In addition to the PHY interrupt, the Network Monitor also requires the
      following setting:

        CONFIG_NETDEV_PHY_IOCTL. Enable PHY IOCTL commands in the Ethernet
        device driver. Special IOCTL commands must be provided by the Ethernet
        driver to support certain PHY operations that will be needed for link
        management. There operations are not complex and are implemented for
        the Atmel SAMA5 family.

        CONFIG_ARCH_PHY_INTERRUPT. This is not a user selectable option.
        Rather, it is set when you select a board that supports PHY
        interrupts.  For the K64F, like most other architectures, the PHY
        interrupt must be provided via some board-specific GPIO.  In any
        event, the board-specific logic must provide support for the PHY
        interrupt. To do this, the board logic must do two things: (1) It
        must provide the function arch_phy_irq() as described and prototyped
        in the nuttx/include/nuttx/arch.h, and (2) it must select
        CONFIG_ARCH_PHY_INTERRUPT in the board configuration file to
        advertise that it supports arch_phy_irq().

        One other thing: UDP support is required (CONFIG_NET_UDP).

  Given those prerequisites, the network monitor can be selected with these
  additional settings.

    System Type -> Kinetis Ethernet Configuration
      CONFIG_ARCH_PHY_INTERRUPT=y           : (auto-selected)
      CONFIG_NETDEV_PHY_IOCTL=y             : (auto-selected)

    Application Configuration -> NSH Library -> Networking Configuration
      CONFIG_NSH_NETINIT_THREAD             : Enable the network initialization thread
      CONFIG_NSH_NETINIT_MONITOR=y          : Enable the network monitor
      CONFIG_NSH_NETINIT_RETRYMSEC=2000     : Configure the network monitor as you like

SD Card Support
===============

  Card Slot
  ---------
  A micro Secure Digital (SD) card slot is available on the FRDM-K64F connected to
  the SD Host Controller (SDHC) signals of the MCU. This slot will accept micro
  format SD memory cards. The SD card detect pin (PTE6) is an open switch that
  shorts with VDD when card is inserted.

    ------------ ------------- --------
    SD Card Slot Board Signal  K64F Pin
    ------------ ------------- --------
    DAT0         SDHC0_D0      PTE1
    DAT1         SDHC0_D1      PTE0
    DAT2         SDHC0_D2      PTE5
    CD/DAT3      SDHC0_D3      PTE4
    CMD          SDHC0_CMD     PTE3
    CLK          SDHC0_DCLK    PTE2
    SWITCH       D_CARD_DETECT PTE6
    ------------ ------------- --------

  There is no Write Protect pin available to the K64F.

  Configuration Settings
  ----------------------
  Enabling SDHC support. The Freedom K64F provides one microSD memory card
  slot.  Support for the SD slots can be enabled with the following
  settings:

    System Type->Kinetic Peripheral Selection
      CONFIG_KINETIS_SDHC=y                 : To enable SDHC0 support
      CONFIG_KINETIS_SDHC_DMA=y             : Use SDIO DMA

    System Type
      CONFIG_KINETIS_GPIOIRQ=y              : GPIO interrupts needed
      CONFIG_KINETIS_PORTEINTS=y            : Card detect pin is on PTE6

    Device Drivers -> MMC/SD Driver Support
      CONFIG_MMCSD=y                        : Enable MMC/SD support
      CONFIG_MMSCD_NSLOTS=1                 : One slot per driver instance
      CONFIG_MMCSD_MULTIBLOCK_LIMIT=1       : (REVISIT)
      CONFIG_MMCSD_HAVE_CARDDETECT=y        : Supports card-detect PIOs
      CONFIG_MMCSD_MMCSUPPORT=n             : Interferes with some SD cards
      CONFIG_MMCSD_SPI=n                    : No SPI-based MMC/SD support
      CONFIG_MMCSD_SDIO=y                   : SDIO-based MMC/SD support
      CONFIG_SDIO_BLOCKSETUP=y              : Needs to know block sizes

    RTOS Features -> Work Queue Support
      CONFIG_SCHED_WORKQUEUE=y              : Driver needs work queue support
      CONFIG_SCHED_HPWORK=y

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization, and
      CONFIG_BOARDCTL=y                 : Or
      CONFIG_BOARD_LATE_INITIALIZE=y

  Using the SD card
  -----------------

  1. After booting, the SDHC device will appear as /dev/mmcsd0.
  2. If you try mounting an SD card with nothing in the slot, the mount will
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

  3. Insert the SD card.  Then the mount should succeed.

      nsh> mount -t vfat /dev/mmcsd0 /mnt/sd0
      nsh> ls /mnt/sd1
      /mnt/sd1:
       atest.txt
      nsh> cat /mnt/sd1/atest.txt
      This is a test

     NOTE:  See the next section entitled "Auto-Mounter" for another way
     to mount your SD card.

  4. Before removing the card, you must umount the file system.  This is
     equivalent to "ejecting" or "safely removing" the card on Windows:  It
     flushes any cached data to an SD card and makes the SD card unavailable
     to the applications.

       nsh> umount -t /mnt/sd0

     It is now safe to remove the card.  NuttX provides into callbacks
     that can be used by an application to automatically unmount the
     volume when it is removed.  But those callbacks are not used in
     these configurations.

  Auto-Mounter
  ------------
  NuttX implements an auto-mounter than can make working with SD cards
  easier.  With the auto-mounter, the file system will be automatically
  mounted when the SD card is inserted into the SDHC slot and automatically
  unmounted when the SD card is removed.

  Here is a sample configuration for the auto-mounter:

    File System Configuration
      CONFIG_FS_AUTOMOUNTER=y

    Board-Specific Options
      CONFIG_FRDMK64F_SDHC_AUTOMOUNT=y
      CONFIG_FRDMK64F_SDHC_AUTOMOUNT_FSTYPE="vfat"
      CONFIG_FRDMK64F_SDHC_AUTOMOUNT_BLKDEV="/dev/mmcsd0"
      CONFIG_FRDMK64F_SDHC_AUTOMOUNT_MOUNTPOINT="/mnt/sdcard"
      CONFIG_FRDMK64F_SDHC_AUTOMOUNT_DDELAY=1000
      CONFIG_FRDMK64F_SDHC_AUTOMOUNT_UDELAY=2000

  WARNING:  SD cards should never be removed without first unmounting
  them.  This is to avoid data and possible corruption of the file
  system.  Certainly this is the case if you are writing to the SD card
  at the time of the removal.  If you use the SD card for read-only access,
  however, then I cannot think of any reason why removing the card without
  mounting would be harmful.

USB Device Controller Support
==============================

  USB Device Controller Support
  -----------------------------
  The USBHS device controller driver is enabled with he following
  configurationsettings:

    Device Drivers -> USB Device Driver Support
      CONFIG_USBDEV=y                           : Enable USB device support
    For full-speed/low-power mode:
      CONFIG_USBDEV_DUALSPEED=n                 : Disable High speed support
    For high-speed/normal mode:
      CONFIG_USBDEV_DUALSPEED=y                 : Enable High speed support
      CONFIG_USBDEV_DMA=y                       : Enable DMA methods
      CONFIG_USBDEV_MAXPOWER=100                : Maximum power consumption
      CONFIG_USBDEV_SELFPOWERED=y               : Self-powered device

    System Type -> Kinetis Peripheral Selection
      CONFIG_KINETIS_USBOTG=y

  CDC/ACM Device Class
  --------------------
  In order to be usable, you must all enabled some class driver(s) for the
  USBHS device controller.  Here, for example, is how to configure the CDC/ACM
  serial device class:

    Device Drivers -> USB Device Driver Support
      CONFIG_CDCACM=y                           : USB Modem (CDC ACM) support
      CONFIG_CDCACM_EP0MAXPACKET=64             : Endpoint 0 packet size
      CONFIG_CDCACM_EPINTIN=1                   : Interrupt IN endpoint number
      CONFIG_CDCACM_EPINTIN_FSSIZE=64           : Full speed packet size
      CONFIG_CDCACM_EPINTIN_HSSIZE=64           : High speed packet size
      CONFIG_CDCACM_EPBULKOUT=3                 : Bulk OUT endpoint number
      CONFIG_CDCACM_EPBULKOUT_FSSIZE=64         : Full speed packet size
      CONFIG_CDCACM_EPBULKOUT_HSSIZE=512        : High speed packet size
      CONFIG_CDCACM_EPBULKIN=2                  : Bulk IN endpoint number
      CONFIG_CDCACM_EPBULKIN_FSSIZE=64          : Full speed packet size
      CONFIG_CDCACM_EPBULKIN_HSSIZE=512         : High speed packet size
      CONFIG_CDCACM_NWRREQS=4                   : Number of write requests
      CONFIG_CDCACM_NRDREQS=8                   : Number of read requests
      CONFIG_CDCACM_BULKIN_REQLEN=96            : Size of write request buffer (for full speed)
      CONFIG_CDCACM_BULKIN_REQLEN=768           : Size of write request buffer (for high speed)
      CONFIG_CDCACM_RXBUFSIZE=257               : Serial read buffer size
      CONFIG_CDCACM_TXBUFSIZE=193               : Serial transmit buffer size (for full speed)
      CONFIG_CDCACM_TXBUFSIZE=769               : Serial transmit buffer size (for high speed)
      CONFIG_CDCACM_VENDORID=0x0525             : Vendor ID
      CONFIG_CDCACM_PRODUCTID=0xa4a7            : Product ID
      CONFIG_CDCACM_VENDORSTR="NuttX"           : Vendor string
      CONFIG_CDCACM_PRODUCTSTR="CDC/ACM Serial" : Product string

    Device Drivers -> Serial Driver Support
      CONFIG_SERIAL_REMOVABLE=y                 : Support for removable serial device

  The CDC/ACM application provides commands to connect and disconnect the
  CDC/ACM serial device:

    CONFIG_SYSTEM_CDCACM=y                     : Enable connect/disconnect support
    CONFIG_SYSTEM_CDCACM_DEVMINOR=0            : Use device /dev/ttyACM0
    CONFIG_CDCACM_RXBUFSIZE=???                : A large RX may be needed

  If you include this CDC/ACM application, then you can connect the CDC/ACM
  serial device to the host by entering the command 'sercon' and you detach
  the serial device with the command 'serdis'.  If you do no use this
  application, they you will have to write logic in your board initialization
  code to initialize and attach the USB device.

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchain Options
=====================

  The NuttX make system supports several GNU-based toolchains under Linux,
  Cygwin under Windows, and Windows native.  To select a toolchain:

  1. Use 'make menuconfig' and select the toolchain that you are using
     under the System Type menu.
  2. The default toolchain is the NuttX buildroot under Linux or Cygwin:

    CONFIG_ARM_TOOLCHAIN_BUILDROOT=y

  You may also have to modify the PATH environment variable if your make cannot
  find the tools.

Freedom K64F Configuration Options
==================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM4=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=kinetis

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_MK64FN1M0VLL12

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD="freedom-k64f" (for the Freedom K64F development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_FREEDOM_K64F=y

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

  Individual subsystems can be enabled:

    CONFIG_KINETIS_TRACE    -- Enable trace clocking on power up.
    CONFIG_KINETIS_FLEXBUS  -- Enable flexbus clocking on power up.
    CONFIG_KINETIS_UART0    -- Support UART0
    CONFIG_KINETIS_UART1    -- Support UART1
    CONFIG_KINETIS_UART2    -- Support UART2
    CONFIG_KINETIS_UART3    -- Support UART3
    CONFIG_KINETIS_UART4    -- Support UART4
    CONFIG_KINETIS_UART5    -- Support UART5
    CONFIG_KINETIS_ENET     -- Support Ethernet (K5x, K6x, and K7x only)
    CONFIG_KINETIS_RNGB     -- Support the random number generator(K6x only)
    CONFIG_KINETIS_FLEXCAN0 -- Support FlexCAN0
    CONFIG_KINETIS_FLEXCAN1 -- Support FlexCAN1
    CONFIG_KINETIS_SPI0     -- Support SPI0
    CONFIG_KINETIS_SPI1     -- Support SPI1
    CONFIG_KINETIS_SPI2     -- Support SPI2
    CONFIG_KINETIS_I2C0     -- Support I2C0
    CONFIG_KINETIS_I2C1     -- Support I2C1
    CONFIG_KINETIS_I2S      -- Support I2S
    CONFIG_KINETIS_DAC0     -- Support DAC0
    CONFIG_KINETIS_DAC1     -- Support DAC1
    CONFIG_KINETIS_ADC0     -- Support ADC0
    CONFIG_KINETIS_ADC1     -- Support ADC1
    CONFIG_KINETIS_CMP      -- Support CMP
    CONFIG_KINETIS_VREF     -- Support VREF
    CONFIG_KINETIS_SDHC     -- Support SD host controller
    CONFIG_KINETIS_FTM0     -- Support FlexTimer 0
    CONFIG_KINETIS_FTM1     -- Support FlexTimer 1
    CONFIG_KINETIS_FTM2     -- Support FlexTimer 2
    CONFIG_KINETIS_LPTMR0   -- Support the low power timer 0
    CONFIG_KINETIS_RTC      -- Support RTC
    CONFIG_KINETIS_SLCD     -- Support the segment LCD (K3x, K4x, and K5x only)
    CONFIG_KINETIS_EWM      -- Support the external watchdog
    CONFIG_KINETIS_CMT      -- Support Carrier Modulator Transmitter
    CONFIG_KINETIS_USBOTG   -- Support USB OTG (see also CONFIG_USBHOST and CONFIG_USBDEV)
    CONFIG_KINETIS_USBDCD   -- Support the USB Device Charger Detection module
    CONFIG_KINETIS_LLWU     -- Support the Low Leakage Wake-Up Unit
    CONFIG_KINETIS_TSI      -- Support the touch screeen interface
    CONFIG_KINETIS_FTFL     -- Support FLASH
    CONFIG_KINETIS_DMA      -- Support DMA
    CONFIG_KINETIS_CRC      -- Support CRC
    CONFIG_KINETIS_PDB      -- Support the Programmable Delay Block
    CONFIG_KINETIS_PIT      -- Support Programmable Interval Timers
    CONFIG_ARM_MPU          -- Support the MPU

  Kinetis interrupt priorities (Default is the mid priority).  These should
  not be set because they can cause unhandled, nested interrupts.  All
  interrupts need to be at the default priority in the current design.

    CONFIG_KINETIS_UART0PRIO
    CONFIG_KINETIS_UART1PRIO
    CONFIG_KINETIS_UART2PRIO
    CONFIG_KINETIS_UART3PRIO
    CONFIG_KINETIS_UART4PRIO
    CONFIG_KINETIS_UART5PRIO

    CONFIG_KINETIS_EMACTMR_PRIO
    CONFIG_KINETIS_EMACTX_PRIO
    CONFIG_KINETIS_EMACRX_PRIO
    CONFIG_KINETIS_EMACMISC_PRIO

    CONFIG_KINETIS_SDHC_PRIO

  PIN Interrupt Support

    CONFIG_KINETIS_GPIOIRQ   -- Enable pin interrupt support.  Also needs
      one or more of the following:
    CONFIG_KINETIS_PORTAINTS -- Support 32 Port A interrupts
    CONFIG_KINETIS_PORTBINTS -- Support 32 Port B interrupts
    CONFIG_KINETIS_PORTCINTS -- Support 32 Port C interrupts
    CONFIG_KINETIS_PORTDINTS -- Support 32 Port D interrupts
    CONFIG_KINETIS_PORTEINTS -- Support 32 Port E interrupts

  Kinetis K64 specific device driver settings

    CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn (n=0..5) for the
      console and ttys0 (default is the UART0).
    CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_UARTn_BAUD - The configure BAUD of the UART.
    CONFIG_UARTn_BITS - The number of bits.  Must be either 8 or 8.
    CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity

  Kenetis ethernet controller settings

    CONFIG_ENET_NRXBUFFERS - Number of RX buffers.  The size of one
        buffer is determined by CONFIG_NET_ETH_PKTSIZE.  Default: 6
    CONFIG_ENET_NTXBUFFERS - Number of TX buffers.  The size of one
        buffer is determined by CONFIG_NET_ETH_PKTSIZE.  Default: 2
    CONFIG_ENET_USEMII - Use MII mode.  Default: RMII mode.
    CONFIG_ENET_PHYADDR - PHY address

Configurations
==============

Each Freedom K64F configuration is maintained in a sub-directory and
can be selected as follow:

    tools/configure.sh freedom-k64f:<subdir>

Where <subdir> is one of the following:

  netnsh:
  ------
    This configuration is identical to the nsh configuration described
    below except that networking support is enabled.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default platform/toolchain:

       CONFIG_HOST_WINDOWS=y               : Cygwin under Windows
       CONFIG_WINDOWS_CYGWIN=y
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y  : ARM/mbed toolcahin (arm-none-elf-gcc)
       CONFIG_INTELHEX_BINARY=y            : Output formats: Intel hex binary

    3. The Serial Console is provided on UART3 with the correct pin
       configuration for use with an Arduino Serial Shield.

    4. SDHC support is not enabled in this configuration.  Refer to the
       configuration settings listed above under "SD Card Support".

    5. Support for NSH built-in applications is enabled, but no built-in
       applications have been configured in.

    6. No external pullup is available on MDIO signal when MK64FN1M0VLL12 MCU
       is requests status of the Ethernet link connection. Internal pullup is
       required when port configuration for MDIO signal is enabled:

       CONFIG_KINETIS_ENET_MDIOPULLUP=y

    7. Configured to use a fixed IPv4 address:

       CONFIG_NSH_IPADDR=0x0a000002
       CONFIG_NSH_DRIPADDR=0x0a000001
       CONFIG_NSH_NETMASK=0xffffff00

       And a bogus MAC address:

       CONFIG_NSH_NOMAC=y
       CONFIG_NSH_SWMAC=y
       CONFIG_NETINIT_MACADDR=0x00e0deadbeef

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh using a
    serial console on UART3.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default platform/toolchain:

         CONFIG_HOST_WINDOWS=y               : Cygwin under Windows
         CONFIG_WINDOWS_CYGWIN=y
         CONFIG_ARM_TOOLCHAIN_GNU_EABI=y  : ARM/mbed toolcahin (arm-none-elf-gcc)
         CONFIG_INTELHEX_BINARY=y            : Output formats: Intel hex binary

    3. The Serial Console is provided on UART0 with the correct pin
       configuration for use with the OpenSDAv2 VCOM.  This can be switched
       to use a RS-232 shield on UART3 by reconfiguring the serial console.

         -CONFIG_KINETIS_UART0=y
         +CONFIG_KINETIS_UART3=y
         -CONFIG_UART0_SERIALDRIVER=y
         +CONFIG_UART3_SERIALDRIVER=y
         -CONFIG_UART0_SERIAL_CONSOLE=y
         +CONFIG_UART3_SERIAL_CONSOLE=y
         -CONFIG_UART0_RXBUFSIZE=256
         +CONFIG_UART3_RXBUFSIZE=256
         -CONFIG_UART0_TXBUFSIZE=256
         +CONFIG_UART3_TXBUFSIZE=256
         -CONFIG_UART0_BAUD=115200
         +CONFIG_UART3_BAUD=115200
         -CONFIG_UART0_BITS=8
         +CONFIG_UART3_BITS=8
         -CONFIG_UART0_PARITY=0
         +CONFIG_UART3_PARITY=0
         -CONFIG_UART0_2STOP=0
         +CONFIG_UART3_2STOP=0

       NOTE: On my Windows 10 / Cygwin64 system, the OpenSDAv2 VCOM is not
       recognized.  I probably need to install a driver?

       There is a serial USB driver on the mbed web site.  However, this
       driver would not install on Windows 10 for me.  I understand that
       it installs OK on Windows 7.

    4. Support for NSH built-in applications is enabled, but no built-in
       applications have been configured in.

    5. An SDHC driver is enabled in this configuration but does not yet work.
       The basic problem seems to be that it does not sense the presence of
       the SD card on PTE6.  No interrupts are generated when the SD card is
       inserted or removed.  You might want to disable SDHC and MMC/SD if
       you are using this configuration.  Refer to the configuration
       settings listed above under "SD Card Support".

Status
======

  2016-07-11:  Received hardware today and the board came up on the very
    first try.  That does not happen often.  At this point, the very basic
    NSH configuration is working and LEDs are working.

    The only odd behavior that I see is that pressing SW3 causes an NMI
    interrupt (followed by a crash):

      kinetis_nmi: PANIC!!! NMI received

    I don't yet understand why this is.

  2016-07-12:  Added support for the KSZ8081 PHY and added the netnsh
    configuration.  The network is basically functional.  More testing is
    needed, but I have not seen any obvious network failures.

    In testing, I notice a strange thing.  If I run at full optimization the
    code runs (albeit with bugs-to-be-solved).  But with no optimization or
    even at -O1, the system fails to boot.  This seems to be related to the
    watchdog timer.

  2016-07-13:  Add SD automounter logic; broke out SDHC logic into a separate
    file.  The nsh configuration now has SDHC enabled be default.  Does not
    yet work.  The basic problem seems to be that it does not sense the
    presence of the SD card on PTE6.  No interrupts are generated when the
    SD card is inserted or removed.  You might want to disable SDHC and
    MMC/SD if you are using this configuration.

    The nsh configuration now builds successfully with USB device enabled.
    USB device, however, has not yet been tested.  I have not yet looked
    into 48MHz clocking requirements.

  2017-02-10:  These have been numerous SDHC fixes submitted by Marc Rechte'.
    These may or may not have fixed the SDHC issues mentioned about.  You
    would have to retest to verify the SDHC functionality.
