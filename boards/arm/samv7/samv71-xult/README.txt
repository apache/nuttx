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
  - S25FL116K QuadSPI FLASH
  - Program FLASH Access
  - Networking
  - USBHS Device Controller Driver
  - Audio Interface
  - maXTouch Xplained Pro
  - MCAN1 Loopback Test
  - SPI Slave
  - Click Shield
  - Tickless OS
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

Status/Open Issues
==================

I would characterize the general port as very mature and stable.  However,
there are a number of issues, caveats, and unfinished drivers as detailed in
the following paragraphs.

The BASIC nsh configuration is fully function (as described below under
"Configurations").  There is also a graphics configuration (mxtxplnd), a
a configuration for network testing (netnsh), a graphics demo (nxwm), and
a sample protected mode build (knsh).  There are still open issues that need
to be resolved.  General problems are listed below.  But see the STATUS
section associated with each configuration for additional issues specific
to a particular configuration.

  1. HSCMI. CONFIG_MMCSD_MULTIBLOCK_LIMIT=1 is set to disable multi-block
     transfers only because I have not yet had a chance to verify this.  The
     is very low priority to me but might be important to you if you are need
     very high performance SD card accesses.

  2. HSMCI TX DMA is currently disabled for the SAMV7.  There is some
     issue with the TX DMA setup.  This is a bug that needs to be resolved.

     DMA is enabled by these settings in the file arch/arm/src/samv7/sam_hsmci.c:

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

       Update: I have switch toolchains and no longer see this issue.  This
       is probably not a driver-related issue.

     - I- and D-Caches are enabled but the D-Cache must be enabled in
       write-through mode.  This is to work around issues with the RX and TX
       descriptors with are 8-bytes in size.  But the D-Cache cache line size
       is 32-bytes.  That means that you cannot reload, clean or invalidate a
       descriptor without also effecting three neighboring descriptors.
       Setting write through mode eliminates the need for cleaning the D-Cache.
       If only reloading and invalidating are done, then there is no problem.

  5. The USBHS device controller driver (DCD) is also fully functional.  It
     has only be tested with the CDC/ACM driver as described below. Like
     the Ethernet driver:

     - This driver does not work reliably with the write back D-Cache.  The
       write-through D-Cache must be enabled.

     - As of this writing (2015-08-22), the USBHS only works in full speed
       mode (aka, USBHS Low-Power mode).  When configured in normal mode,
       SETUP packets are no longer received or responded to; the firmware
       only detects bus reset events.  This is probably some issue with
       480MHZ high speed clock setup, but I have not yet found the issue.

  6. The full port for audio support is code complete:  WM8904 driver,
     SSC/I2C driver, and CS2100-CP driver.  But this code is untested.  The
     WM8904 interface was taken directly from the SAMA5D4-EK and may well
     need modification due to differences with the physical WM8904
     interface.

  7. An MCAN driver as added and verified on 2015-08-08 using the loopback
     test at apps/examples/can.  Like the Ethernet driver, the MCAN driver
     does not work if the D-Cache is configured in write-back mode; write-
     through mode is required.

  8. An SPI slave driver as added on 2015-08-09 but has not been verified
     as of this writing. See discussion in include/nuttx/spi/slave.h and
     in the section entitle "SPI Slave" below.

  9. A QSPI FLASH driver was added and verified on 2015-11-10.  This driver
     operated in the memory mapped Serial Memory Mode (SMM).  See the
     "S25FL116K QuadSPI FLASH" section below for further information.

 10. On-chip FLASH support as added and verified on 2015-11-13.  See the
     "Program FLASH Access" section below for further information.

 11. The knsh "protected mode" configuration was added on 2015-11-18.  The
     configuration has not been tested as of this writing.

Serial Console
==============

The SAMV71-XULT has no on-board RS-232 drivers so it will be necessary to
use either the VCOM or an external RS-232 driver.  Here are some options.

  - VCOM.  The Virtual Com Port gateway is available on USART1 and it is the
    default console.  Besides PB04 is by default connected to debug pin TDI,
    both JTAG port and EDBG can only be used in SWD mode in this board.

    ------ --------
    SAMV71 SAMV71
    PIO    Function
    ------ --------
    PB04   TXD1
    PA21   RXD1
    ------ --------

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
    instead of the shield.  Simply connect as follows:

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
    CONFIG_SAMV7_GPIO_IRQ=y               : PIO interrupts needed
    CONFIG_SAMV7_GPIOD_IRQ=y              : Card detect pin is on PD18

  Device Drivers -> MMC/SD Driver Support
    CONFIG_MMCSD=y                        : Enable MMC/SD support
    CONFIG_MMSCD_NSLOTS=1                 : One slot per driver instance
    CONFIG_MMCSD_MULTIBLOCK_LIMIT=1     : (REVISIT)
    CONFIG_MMCSD_HAVE_CARDDETECT=y         : Supports card-detect PIOs
    CONFIG_MMCSD_MMCSUPPORT=n             : Interferes with some SD cards
    CONFIG_MMCSD_SPI=n                    : No SPI-based MMC/SD support
    CONFIG_MMCSD_SDIO=y                   : SDIO-based MMC/SD support
    CONFIG_SDIO_DMA=y                     : Use SDIO DMA
    CONFIG_SDIO_BLOCKSETUP=y              : Needs to know block sizes

  RTOS Features -> Work Queue Support
    CONFIG_SCHED_WORKQUEUE=y              : Driver needs work queue support

  Application Configuration -> NSH Library
    CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization, OR
    CONFIG_BOARD_LATE_INITIALIZE=y

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
      CONFIG_SAMV7_HSMCI0_AUTOMOUNT=y
      CONFIG_SAMV7_HSMCI0_AUTOMOUNT_FSTYPE="vfat"
      CONFIG_SAMV7_HSMCI0_AUTOMOUNT_BLKDEV="/dev/mmcsd0"
      CONFIG_SAMV7_HSMCI0_AUTOMOUNT_MOUNTPOINT="/mnt/sdcard"
      CONFIG_SAMV7_HSMCI0_AUTOMOUNT_DDELAY=1000
      CONFIG_SAMV7_HSMCI0_AUTOMOUNT_UDELAY=2000

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

S25FL116K QuadSPI FLASH
====================

A QSPI FLASH driver was added and verified on 2015-11-07.  This driver
operated in the memory mapped Serial Memory Mode (SMM).  These
configuration options were enabled to test QSPI:

  CONFIG_SAMV7_QSPI=y
  CONFIG_SAMV7_QSPI_DLYBCT=0
  CONFIG_SAMV7_QSPI_DLYBS=0
  CONFIG_SAMV7_QSPI_DMA=y
  CONFIG_SAMV7_QSPI_DMATHRESHOLD=8

The MPU must be enabled to use QSPI:

  CONFIG_ARCH_USE_MPU=y
  CONFIG_ARM_MPU=y
  CONFIG_ARM_MPU_NREGIONS=16

And there options enable the driver for the on-board S25FL116K QuadSPI
FLASH:

  CONFIG_MTD_S25FL1=y
  CONFIG_S25FL1_QSPIMODE=0
  CONFIG_S25FL1_QSPI_FREQUENCY=108000000

SmartFS
-------
The SmartFS file system is selected with the following settings.

  CONFIG_FS_SMARTFS=y
  CONFIG_SMARTFS_ERASEDSTATE=0xff
  CONFIG_SMARTFS_MAXNAMLEN=16

  CONFIG_MTD_SMART=y
  CONFIG_MTD_SMART_SECTOR_SIZE=512
  CONFIG_MTD_SMART_WEAR_LEVEL=y

Upon boot, the on-board S25FL116k flash device will appears as:

  /dev/smart0

Before SmartFS can be used, it must be formatted. So this command
must be used one time the first time that the system boots:

  nsh> mksmartfs /dev/smart0

Then it can be mounted using the following NSH command:

  nsh> mount -t smartfs /dev/smart0 /mnt/qspi

The first time that you boot the system, there will be a long delay
before the nsh> prompt.  That long delay is SmartFS scanning the
large FLASH part.  Likewise, the when you format the SmartFS, you
also expect a significant delay.

A better application design would perform SmartFS initialization
asynchronously on a separate thread to avoid the delay at the user
interface.

NXFFS
-----
The NXFFS file system is selected with the following settings.

  CONFIG_FS_NXFFS=y
  CONFIG_NXFFS_ERASEDSTATE=0xff
  CONFIG_NXFFS_MAXNAMLEN=255
  CONFIG_NXFFS_PACKTHRESHOLD=32
  CONFIG_NXFFS_PREALLOCATED=y
  CONFIG_NXFFS_TAILTHRESHOLD=8192

The NXFFS file system is automatically mounted by logic src/sam_bringup.c when the system boots:

  nsh> mount
    /mnt/s25fl1 type nxffs
  nsh> echo "This is a test" >/mnt/s25fl1/atest.txt
  nsh> ls /mnt/s25fl1
  /mnt/s25fl1:
   atest.txt
  nsh> cat /mnt/s25fl1/atest.txt
  This is a test

Character Driver
----------------
If neither SmartFS nor NXFFS are defined, then the S25FL1 driver will be
wrapped as a character driver and available as /dev/mtd0.

Program FLASH Access
====================
An on-chip FLASH driver was added and verified on 2015-11-13.  These
configuration options were enabled to test the on-chip FLASH support:

  CONFIG_MTD_PROGMEM=y
  CONFIG_ARCH_RAMFUNCS=y
  CONFIG_SAMV7_PROGMEM=y
  CONFIG_SAMV7_PROGMEM_NSECTORS=8

D-Cache must be configured in write-through mode:

  CONFIG_ARMV7M_DCACHE_WRITETHROUGH=y

The total FLASH on the SAMV71 is organized as 128KB/sector x 16 sectors
= 2MB.  The sectors are all uniform (except for sector zero which will
never be used by the driver).

The configuration sets aside 8 sectors, or 8 * 128KB = 1MB of the FLASH
for programmable memory (CONFIG_SAMV7_PROGMEM_NSECTORS=8).  The exact
number of sectors set aside is optional.

NOTE: Ideally, one should also modify the linker script and reduce the size
of the available FLASH the amount set aside for program usage to avoid
difficult run-time problems.  That would be 1MB in this configuration.  I
did not do that only because I know that my test program is small.

When the system boots, you can see the FLASH driver:

  NuttShell (NSH) NuttX-7.12
  nsh> ls /dev
  /dev:
   config
   console
   mmcsd0
   mtd1
   mtdblock1
   null
   ttyS0

/dev/mtdblock1 is a block driver that can be used with any file system on
the FLASH; /dev/mtd1 is the corresponding character driver used by the
apps/examples/media test.

Each of the uniform sectors is divided up into 256 512B "pages".  This is
not really useful, however, because we can only erase a minimum of groups
of 16 pages or 8KB.  In the code, I you will see that I refer to these
groups of 16 pages as "clusters."  So the cluster is the smallest thing
that you can perform a read/write/modify operation on.

Using 8 sectors yields 16 *8 = 128 clusters (aka blocks).  You can see
this when the apps/examples/media test runs:

  nsh> media
  MTD Geometry:
    blocksize:    8192
    erasesize:    8192
    neraseblocks: 128
  Using:
    blocksize:    8192
    nblocks:      128
  Write/Verify: Block 0
  Write/Verify: Block 1
  Write/Verify: Block 2
  Write/Verify: Block 3
  ...
  Write/Verify: Block 127
  Re-read/Verify: Block 0
  Re-read/Verify: Block 1
  Re-read/Verify: Block 2
  Re-read/Verify: Block 3
  ...
  Re-read/Verify: Block 127
  nsh>

NOTE: The media test can be added to the NSH configuration with:

  CONFIG_EXAMPLES_MEDIA=y
  CONFIG_EXAMPLES_MEDIA_BLOCKSIZE=8192
  CONFIG_EXAMPLES_MEDIA_DEVPATH="/dev/mtd1"

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
    CONFIG_NET_SOCKOPTS=y                : Enable socket operations
    CONFIG_NET_ETH_PKTSIZE=562           : Maximum packet size 1518 is more standard
    CONFIG_NET_ARP=y                     : ARP support should be enabled
    CONFIG_NET_ARP_SEND=y                : Use ARP to get peer address before sending
    CONFIG_NET_TCP=y                     : Enable TCP/IP networking
    CONFIG_NET_TCPBACKLOG=y              : Support TCP/IP backlog
    CONFIG_NET_TCP_WRITE_BUFFERS=y       : Enable TCP write buffering
    CONFIG_NET_UDP=y                     : Enable UDP networking
    CONFIG_NET_BROADCAST=y               : Support UDP broadcast packets
    CONFIG_NET_ICMP=y                    : Enable ICMP networking
    CONFIG_NET_ICMP_SOCKET=y             : Needed for NSH ping command
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
    CONFIG_NETDB_DNSCLIENT=y             : Enable host address resolution
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

SAMV71 Versions
---------------

WARNING: The newer SAMV71 have 6 GMAC queues, not 3. All queues must be
configured for the GMAC to work correctly, even the queues that you are not
using (you can just configure these queues with a very small ring buffer.)

The older uses the Cortex-M7 core r0p1 and the newer r1p1 revisions.  The
SAMV71 revisions are called "rev A" (or sometimes "MRLA") and "rev B"
("MRLB"). There should be a small "A" or "B" on the chip package just below
the reference and you can also differentiate them at runtime with the
VERSION field in the CHIPID CIDR register.

Cache-Related Issues
--------------------

I- and D-Caches can be enabled but the D-Cache must be enabled in write-
through mode.  This is to work around issues with the RX and TX descriptors
which are 8-bytes in size.  But the D-Cache cache line size is 32-bytes.
That means that you cannot reload, clean or invalidate a descriptor without
also affecting three neighboring descriptors.  Setting write through mode
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

By default, the IP address of the SAMV71-XULT will be 10.0.0.2 and
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

On the host side, you should also be able to ping the SAMV71-XULT:

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
    df          hexdump     mb          mv          source

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
    nuttx/boards/arm/samv7/samv71-xult/src/sam_ethernet.c.

  - One other thing: UDP support is required.

Given those prerequisites, the network monitor can be selected with these
additional settings.

  Networking Support -> Networking Device Support
    CONFIG_NETDEV_PHY_IOCTL=y             : Enable PHY ioctl support

  Application Configuration -> NSH Library -> Networking Configuration
    CONFIG_NSH_NETINIT_THREAD             : Enable the network initialization thread
    CONFIG_NSH_NETINIT_MONITOR=y          : Enable the network monitor
    CONFIG_NSH_NETINIT_RETRYMSEC=2000     : Configure the network monitor as you like
    CONFIG_NSH_NETINIT_SIGNO=18

USBHS Device Controller Driver
==============================
The USBHS device controller driver is enabled with he following configuration
settings:

  Device Drivers -> USB Device Driver Support
    CONFIG_USBDEV=y                           : Enable USB device support
  For full-speed/low-power mode:
    CONFIG_USBDEV_DUALSPEED=n                 : Disable High speed support
  For high-speed/normal mode:
    CONFIG_USBDEV_DUALSPEED=y                 : Enable High speed support
    CONFIG_USBDEV_DMA=y                       : Enable DMA methods
    CONFIG_USBDEV_MAXPOWER=100                : Maximum power consumption
    CONFIG_USBDEV_SELFPOWERED=y               : Self-powered device

  System Type -> SAMV7 Peripheral Selection
    CONFIG_SAMV7_USBDEVHS=y

  System Type -> SAMV7 USB High Sppeed Device Controller (DCD options
  For full-speed/low-power mode:
    CONFIG_SAMV7_USBDEVHS_LOWPOWER=y          : Select low power mode
  For high-speed/normal mode:
    CONFIG_SAMV7_USBDEVHS_LOWPOWER=n          : Don't select low power mode
    CONFIG_SAMV7_USBHS_NDTDS=32               : Number of DMA transfer descriptors
    CONFIG_SAMV7_USBHS_PREALLOCATE=y          : Pre-allocate descriptors

As noted above, this driver will not work correctly if the write back
data cache is enabled. You must have:

    CONFIG_ARMV7M_DCACHE_WRITETHROUGH=y

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
  *   maXTouch configuration with NuttX you need to do these things:       *
  *                                                                        *
  *   - Copy the function atmxt_config() from the file                     *
  *     boards/arm/samv7/samv71-xult/src/atmxt_config.c into the file               *
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
  black bar at back (toward the board).  Raise that bar and insert the cable
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

MCAN1 Loopback Test
===================

  MCAN1
  -----
  SAM V71 Xplained Ultra has two MCAN modules that performs communication according
  to ISO11898-1 (Bosch CAN specification 2.0 part A,B) and Bosch CAN FD
  specification V1.0.  MCAN1 is connected to an on-board ATA6561 CAN physical-layer
  transceiver.

    ------- -------- -------- -------------
    SAM V71 FUNCTION ATA6561  SHARED
    PIN              FUNCTION FUNCTIONALITY
    ------- -------- -------- -------------
    PC14    CANTX1   TXD      Shield
    PC12    CANRX1   RXD      Shield
    ------- -------- -------- -------------

  Enabling MCAN1
  --------------
  These modifications may be applied to the samv71-xult/nsh configuration in order
  to enable MCAN1:

    Device Drivers -> CAN Driver support
       CONFIG_CAN=y                            # Enable the upper-half CAN driver
       CONFIG_CAN_FIFOSIZE=8
       CONFIG_CAN_NPENDINGRTR=4

    System Type -> SAMV7 Peripheral Selections
       CONFIG_SAMV7_MCAN1=y                    # Enable MCAN1 as the lower-half

    System Type -> MCAN device driver options
       CONFIG_SAMV7_MCAN_CLKSRC_MAIN=y         # Use the MAIN clock as the source
       CONFIG_SAMV7_MCAN_CLKSRC_PRESCALER=1

    System Type ->MCAN device driver options -> MCAN1 device driver options
       CONFIG_SAMV7_MCAN1_ISO11899_1=y         # Loopback test only support ISO11899-1
       CONFIG_SAMV7_MCAN1_LOOPBACK=y           # Needed for loopback test
       CONFIG_SAMV7_MCAN1_BITRATE=500000       # Not critical for loopback test
       CONFIG_SAMV7_MCAN1_PROPSEG=2            # Bit timing setup
       CONFIG_SAMV7_MCAN1_PHASESEG1=11         # " " "    " "   "
       CONFIG_SAMV7_MCAN1_PHASESEG2=11         # " " "    " "   "
       CONFIG_SAMV7_MCAN1_FSJW=4               # " " "    " "   "
       CONFIG_SAMV7_MCAN1_FBITRATE=2000000     # CAN_FD BTW mode is not used
       CONFIG_SAMV7_MCAN1_FPROPSEG=2           # "    " " " "  " "" " " "  "
       CONFIG_SAMV7_MCAN1_FPHASESEG1=4         # "    " " " "  " "" " " "  "
       CONFIG_SAMV7_MCAN1_FPHASESEG2=4         # "    " " " "  " "" " " "  "
       CONFIG_SAMV7_MCAN1_FFSJW=2              # "    " " " "  " "" " " "  "
       CONFIG_SAMV7_MCAN1_NSTDFILTERS=0        # Filters are not used in the loopback test
       CONFIG_SAMV7_MCAN1_NEXTFILTERS=0        # "     " " " " " "  " "" " " "      " "  "
       CONFIG_SAMV7_MCAN1_RXFIFO0_32BYTES=y    # Each RX FIFO0 element is 32 bytes
       CONFIG_SAMV7_MCAN1_RXFIFO0_SIZE=8       # There are 8 queue elements
       CONFIG_SAMV7_MCAN1_RXFIFO0_32BYTES=y    # Each RX FIFO1 element is 32 bytes
       CONFIG_SAMV7_MCAN1_RXFIFO0_SIZE=8       # There are 8 queue elements
       CONFIG_SAMV7_MCAN1_RXBUFFER_32BYTES=y   # Each RX BUFFER is 32 bytes
       CONFIG_SAMV7_MCAN1_TXBUFFER_32BYTES=y   # Each TX BUFFER is 32 bytes
       CONFIG_SAMV7_MCAN1_TXFIFOQ_SIZE=8       # There are 8 queue elements
       CONFIG_SAMV7_MCAN1_TXEVENTFIFO_SIZE=0   # The event FIFO is not used

    Enabling the CAN Loopback Test
    ------------------------------
    Application Configuration -> Examples -> CAN Example
      CONFIG_EXAMPLES_CAN=y                    # Enables the CAN test

    Enabling CAN Debug Output
    -------------------------
    Build Setup -> Debug Options
      CONFIG_DEBUG_FEATURES=y                  # Enables general debug features
      CONFIG_DEBUG_INFO=y                      # Enables verbose output
      CONFIG_DEBUG_CAN_INFO=y                  # Enables debug output from CAN

      CONFIG_STACK_COLORATION=y                # Monitor stack usage
      CONFIG_DEBUG_SYMBOLS=y                   # Needed only for use with a debugger
      CONFIG_DEBUG_NOOPT=y                     # Disables optimization

    System Type -> MCAN device driver options
     CONFIG_SAMV7_MCAN_REGDEBUG=y              # Super low level register debug output

SPI Slave
=========

  An interrupt driven SPI slave driver as added on 2015-08-09 but has not
  been verified as of this writing. See discussion in include/nuttx/spi/slave.h
  and below.

  I do not yet have a design that supports SPI slave DMA.  And, under
  certain, very limited conditions, I think it can be done.  Those
  certain conditions are:

  a) The master does not tie the chip select to ground.  The master must
     raise chip select at the end of the transfer.  Then I do not need to
     know the length of the transfer; I can cancel the DMA when the chip
     is de-selected.

  b) The protocol includes a dummy read after sending the command.  This
     is very common in SPI device and should not be an issue if it is
     specified.   This dummy read time provides time to set up the DMA.
     So the protocol would be:

     i)   Master drops the chip select.
     ii)  Master sends the command which will indicate whether the master
          is reading, writing, or exchanging data.  The master discards
          the garbage return value.
     iii) Slave is interrupted when the command word is received.  The
          SPI device then decodes the command word and setups up the
          subsequent DMA.
     iv)  Master sends a dummy word and discards the return value.
          During the bit times to shift the dummy word, the slave has time
          to set up the DMA.
     v)   Master then reads or writes (or exchanges) the data  If the DMA
          is in place, the transfer should continue normally.
     vi)  At the end of the data transfer the master raises the chip
     select.

   c) There are limitations in the word time, i.e., the time between the
      interrupt for each word shifted in from the master.

  The controller driver will get events after the receipt of each word in
  ii), iv), and v).  The time between each word will be:

    word-time = nbits * bit time + inter-word-gap

  So for an 8 bit interface at 20MHz, the words will be received from the
  master a 8 * 50nsec = 400 nsec + inter-word-gap.  That is the time
  during which the dummy word would be shifted and during which we
  receive the interrupt for the command word, interpret the command word,
  and to set up the DMA for the remaining word transfer.  I don't think
  that is possible, at least not at 20 MHz.

  That is far too fast even for the interrupt driven solution that I have
  in place now.  It could not work at 20MHz.  If we suppose that interrupt
  processing is around 1 usec, then an 8 bit interface could not have bit
  times more than 125 nsec or 8 KHz.  Interrupt handling should be faster
  than 1 usec, but not a lot faster.  I have not benchmarked it.  NuttX
  also supports special, zero latency interrupts that could bring the
  interrupt time down even more.

  Note that we would also have a little more processing time if you used
  16-bit SPI word size.

  Note also that the interrupt driven approach would have this same basic
  performance limitation with the additional disadvantage that:

  a) The driver will receive two interrupts per word exchanged:

     i)  One interrupt will be received when the word is shifted in from
         the master (at the end of 8-bit times).  This is a data received
         interrupt.

     ii) And another interrupt when the next words moved to the shift-out
         register, freeing up the transmit holding register.  This is the
         data sent interrupt.

     The ii) event should be very soon after the i) event.

     Without DMA, the only way to reduce the interrupt rate would be to add
     interrupt-level polling to detect the when transmit holding register
     is available.  That is not really a good idea.

  b) It will hog all of the CPU for the duration of the transfer).

Click Shield
============

  In the mrf24j40-starhub configuration, a click shield from
  MikroElectronika was used along with a Click "Bee" module.  The click
  shield supports two click shields and the following tables describe the
  relationship between the pins on each click shield, the Arduino
  connector and the SAMV71 pins.

  --------- ---------------------- -------- --------- ------------------ ----------
  mikroBUS1 Arduino                SAMV71   mikroBUS2 Arduino            SAMV71
  --------- ---------------------- -------- --------- ------------------ ----------
  AN        HD1 A0  AN0      Pin 1 AD0 PD26 AN        HD1 A1 AN1  Pin 2  AD1 PC31
  RST       HD1 A3           Pin 4 AD3 PA19 RST       HD1 A2      Pin 3  AD2 PD30
  CS        HD4 D10 SPI-SS   Pin 8 D10 PD25 CS        HD4 D9      Pin 9  D9  PC9
  SCK       HD4 D13 SPI-SCK  Pin 5 D13 PD22 SCK       Same
  MISO      HD4 D12 SPI-MISO Pin 6 D12 PD20 MISO      Same
  MOSI      HD4 D11 SPI-MOSI Pin 7 D11 PD21 MOSI      Same
  3.3V      N/A                             3.3V      N/A
  GND       N/A                             GND       N/A
  PWM       HD3 D6 PWMA      Pin 2 D6  PC19 PWM       HD3 D5 PWMB Pin 5  D5 PD11
  INT       HD3 D2 INT0      Pin 6 D2  PA5  INT       HD3 D3 INT1 Pin 5  D3 PA6
  RX        HD3 D0 HDR-RX*   Pin 8 D0  PD28 RX        Same
  TX        HD3 D1 HDR-TX*   Pin 7 D1  PD30 TX        Same
  SCL       HD1 A5 I2C-SCL   Pin 5 AD5 PC30 SDA       Same
  SDA       HD1 A4 I2C-SDA   Pin 6 AD4 PC13 SCL       Same
  5V        N/A                             5V        N/A
  GND       N/A                             GND       N/A
  --------- ---------------------- -------- --------- ------------------ ----------

  * Depends upon setting of SW1, UART vs PROG.

  --- ----- ------------------------------ ---------------------------------
  PIN PORT  SHIELD FUNCTION                SAMV71PIN CONFIGURATION
  --- ----- ------------------------------ ---------------------------------
  AD0 PD26  microBUS2 Analog TD            PD26 *** Not an AFE pin ***
  AD1 PC31  microBUS2 Analog               PC31 AFE1_AD6   GPIO_AFE1_AD6
  AD2 PD30  microBUS2 GPIO reset output    PD30
  AD3 PA19  microBUS1 GPIO reset output    PA19
  AD4 PC13  (both) I2C-SDA                 PC13 *** Does not support I2C SDA ***
  AD5 PC30  (both) I2C-SCL                 PC30 *** Does not support I2C SCL ***
  AD6 PA17  *** Not used ***
  AD7 PC12  *** Not used ***
  D0  PD28  (both) HDR_RX                  PD28 URXD3      GPIO_UART3_RXD
  D1  PD30  (both) HDR_TX                  PD30 UTXD3      GPIO_UART3_TXD_1
  D2  PA0   microBUS1 GPIO interrupt input PA0
  D3  PA6   microBUS2 GPIO interrupt input PA6
  D4  PD27  *** Not used ***
  D5  PD11  microBUS2 PWMB                 PD11 PWMC0_H0
  D6  PC19  microBUS1 PWMA                 PC19 PWMC0_H2
  D7  PA2   *** Not used ***
  D8  PA5   *** Not used ***
  D9  PC9   microBUS2 CS GPIO output       PC9
  D10 PD25  microBUS1 CS GPIO output       PD25 SPI0_NPCS1
  D11 PD21  (both) SPI-MOSI                PD21 SPI0_MOSI  GPIO_SPI0_MOSI
  D12 PD20  (both) SPI-MISO                PD20 SPI0_MISO  GPIO_SPI0_MISO
  D13 PD22  (both) SPI-SCK                 PD22 SPI0_SPCK  GPIO_SPI0_SPCK

Tickless OS
===========

  Background
  ----------
  By default, a NuttX configuration uses a periodic timer interrupt that
  drives all system timing. The timer is provided by architecture-specific
  code that calls into NuttX at a rate controlled by CONFIG_USEC_PER_TICK.
  The default value of CONFIG_USEC_PER_TICK is 10000 microseconds which
  corresponds to a timer interrupt rate of 100 Hz.

  An option is to configure NuttX to operation in a "tickless" mode. Some
  limitations of default system timer are, in increasing order of
  importance:

  - Overhead: Although the CPU usage of the system timer interrupt at 100Hz
    is really very low, it is still mostly wasted processing time. One most
    timer interrupts, there is really nothing that needs be done other than
    incrementing the counter.
  - Resolution: Resolution of all system timing is also determined by
    CONFIG_USEC_PER_TICK. So nothing that be time with resolution finer than
    10 milliseconds be default. To increase this resolution,
    CONFIG_USEC_PER_TICK an be reduced. However, then the system timer
    interrupts use more of the CPU bandwidth processing useless interrupts.
  - Power Usage: But the biggest issue is power usage. When the system is
    IDLE, it enters a light, low-power mode (for ARMs, this mode is entered
    with the wfi or wfe instructions for example). But each interrupt
    awakens the system from this low power mode. Therefore, higher rates
    of interrupts cause greater power consumption.

  The so-called Tickless OS provides one solution to issue. The basic
  concept here is that the periodic, timer interrupt is eliminated and
  replaced with a one-shot, interval timer. It becomes event driven
  instead of polled: The default system timer is a polled design. On
  each interrupt, the NuttX logic checks if it needs to do anything
  and, if so, it does it.

  Using an interval timer, one can anticipate when the next interesting
  OS event will occur, program the interval time and wait for it to fire.
  When the interval time fires, then the scheduled activity is performed.

  Configuration
  -------------
  The following configuration options will enable support for the Tickless
  OS for the SAMV7 platforms using TC0 channels 0-1 (other timers or
  timer channels could be used making the obvious substitutions):

    RTOS Features -> Clocks and Timers
      CONFIG_SCHED_TICKLESS=y          : Configures the RTOS in tickless mode
      CONFIG_SCHED_TICKLESS_ALARM=n    : (option not implemented)
      CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP=y

    System Type -> SAMV7 Peripheral Support
      CONFIG_SAMV7_TC0=y               : Enable TC0 (TC channels 0-3

    System Type -> Timer/counter Configuration
      CONFIG_SAMV7_ONESHOT=y           : Enables one-shot timer wrapper
      CONFIG_SAMV7_FREERUN=y           : Enabled free-running timer wrapper
      CONFIG_SAMV7_TICKLESS_ONESHOT=0  : Selects TC0 channel 0 for the one-shot
      CONFIG_SAMV7_TICKLESS_FREERUN=1  : Selects TC0 channel 1 for the free-
                                       : running timer

  The resolution of the clock is provided by the CONFIG_USEC_PER_TICK
  setting in the configuration file.

  NOTE: In most cases, the slow clock will be used as the timer/counter
  input.  The 32.768KHz crystal is selected by the definition
  BOARD_HAVE_SLOWXTAL in the boards/arm/samv7/samv71-xult/board.h file.

  The slow clock has a resolution of about 30.518 microseconds.  Ideally,
  the value of CONFIG_USEC_PER_TICK should be the exact clock resolution.
  Otherwise there will be cumulative timing inaccuracies.  But a choice
  choice of:

    CONFIG_USEC_PER_TICK=31

  will have an error of 0.6%  and will have inaccuracies that will
  effect the time due to long term error build-up.

  Using the slow clock clock input, the Tickless support is functional,
  however, there are inaccuracies  in delays.  For example,

    nsh> sleep 10

  results in a delay of maybe 5.4 seconds.  But the timing accuracy is
  correct if all competing uses of the interval timer are disabled (mostly
  from the high priority work queue).  Therefore, I conclude that this
  inaccuracy is due to the inaccuracies in the representation of the clock
  rate.  30.518 usec cannot be represented accurately.   Each timing
  calculation results in a small error.  When the interval timer is very
  busy, long delays will be divided into many small pieces and each small
  piece has a large error in the calculation.  The cumulative error is the
  cause of the problem.

  Solution:  The samv71-xult/src/sam_boot.c file has additional logic
  to enable the programmable clock PCK6 as a clock source for the
  timer/counters if the Tickless mode is selected.  The ideal frequency
  would be:

    frequency = 1,000,000 / CONFIG_USEC_PER_TICK

  The main crystal is selected as the frequency source.  The maximum
  prescaler value is 256 so the minimum frequency is 46,875 Hz which
  corresponds to a period of 21.3 microseconds.  A value of
  CONFIG_USEC_PER_TICK=20, or 50KHz, would give an exact solution with
  a divider of 240.

  SAMV7 Timer Usage
  -----------------
  This current implementation uses two timers:  A one-shot timer to
  provide the timed events and a free running timer to provide the current
  time.  Since timers are a limited resource, that could be an issue on
  some systems.

  We could do the job with a single timer if we were to keep the single
  timer in a free-running at all times.  The SAMV7 timer/counters have
  16-bit counters with the capability to generate a compare interrupt when
  the timer matches a compare value but also to continue counting without
  stopping (giving another, different interrupt when the timer rolls over
  from 0xffff to zero).  So we could potentially just set the compare at
  the number of ticks you want PLUS the current value of timer.  Then you
  could have both with a single timer:  An interval timer and a free-
  running counter with the same timer!  In this case, you would want to
  to set CONFIG_SCHED_TICKLESS_ALARM in the NuttX configuration.

  Patches are welcome!

Debugging
=========

  The on-board EDBG appears to work only with Atmel Studio.  You can however,
  simply connect a SAM-ICE or J-Link to the JTAG/SWD connector on the board
  and that works great.  The only tricky thing is getting the correct
  orientation of the JTAG connection.

  I have been using Atmel Studio to write code to flash then I use the Segger
  J-Link GDB server to debug.  I have been using the 'Device Programming'
  available under the Atmel Studio 'Tool' menu.  I have to disconnect the
  SAM-ICE while programming with the EDBG.

  You can also load code into flash directory with J-Link:

    arm-none-eabi-gdb
    (gdb) target remote localhost:2331
    (gdb) mon reset
    (gdb) mon halt
    (gdb) load nuttx

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

   tools/configure.sh [OPTIONS] samv71-xult:<subdir>

Where typical options are -l to configure to build on Linux or -c to
configure for Cygwin under Linux.  'tools/configure.sh -h' will show
you all of the options.

Before building, make sure the PATH environment variable include the
correct path to the directory than holds your toolchain binaries.

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
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on UART3 (i.e., for the Arduino serial shield).

  3. All of these configurations are set up to build under Windows using the
     "GNU Tools for ARM Embedded Processors" that is maintained by ARM
     (unless stated otherwise in the description of the configuration).

       https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

     That toolchain selection can easily be reconfigured using
     'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Window environment
       CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : GNU ARM EABI toolchain

Configuration sub-directories
-----------------------------

  knsh:

    This is identical to the nsh configuration below except that NuttX
    is built as a protected mode, monolithic module and the user applications
    are built separately.  There are four very similar NSH configurations:

      - knsh.  This is a somewhat simplified version of the nsh configuration
        that builds using the protected build mode (CONFIG_BUILD_PROTECTED=y).
      - nsh.  This configuration is focused on low level, command-line
        driver testing.  It has no network.
      - netnsh.  This configuration is focused on network testing and
        has only limited command support.
      - mxtxplnd.  This configuration is identical to the nsh configuration
        but assumes that you have a maXTouch Xplained Pro LCD attached
        and includes extra tests for the touchscreen and LCD.

    It is recommends to use a special make command; not just 'make' but make
    with the following two arguments:

        make pass1 pass2

    In the normal case (just 'make'), make will attempt to build both user-
    and kernel-mode blobs more or less interleaved.  This actual works!
    However, for me it is very confusing so I prefer the above make command:
    Make the user-space binaries first (pass1), then make the kernel-space
    binaries (pass2)

    NOTES:

    1. At the end of the build, there will be several files in the top-level
       NuttX build directory:

       PASS1:
         nuttx_user.elf    - The pass1 user-space ELF file
         nuttx_user.hex    - The pass1 Intel HEX format file (selected in defconfig)
         User.map          - Symbols in the user-space ELF file

       PASS2:
         nuttx             - The pass2 kernel-space ELF file
         nuttx.hex         - The pass2 Intel HEX file (selected in defconfig)
         System.map        - Symbols in the kernel-space ELF file

       The J-Link programmer will except files in .hex, .mot, .srec, and .bin
       formats.

    2. Combining .hex files.  If you plan to use the .hex files with your
       debugger or FLASH utility, then you may need to combine the two hex
       files into a single .hex file.  Here is how you can do that.

       a. The 'tail' of the nuttx.hex file should look something like this
          (with my comments added):

            $ tail nuttx.hex
            # 00, data records
            ...
            :10 9DC0 00 01000000000800006400020100001F0004
            :10 9DD0 00 3B005A0078009700B500D400F300110151
            :08 9DE0 00 30014E016D0100008D
            # 05, Start Linear Address Record
            :04 0000 05 0800 0419 D2
            # 01, End Of File record
            :00 0000 01 FF

          Use an editor such as vi to remove the 05 and 01 records.

       b. The 'head' of the nuttx_user.hex file should look something like
          this (again with my comments added):

            $ head nuttx_user.hex
            # 04, Extended Linear Address Record
            :02 0000 04 0801 F1
            # 00, data records
            :10 8000 00 BD89 01084C800108C8110208D01102087E
            :10 8010 00 0010 00201C1000201C1000203C16002026
            :10 8020 00 4D80 01085D80010869800108ED83010829
            ...

          Nothing needs to be done here.  The nuttx_user.hex file should
          be fine.

       c. Combine the edited nuttx.hex and un-edited nuttx_user.hex
          file to produce a single combined hex file:

          $ cat nuttx.hex nuttx_user.hex >combined.hex

       Then use the combined.hex file with the to write the FLASH image.
       If you do this a lot, you will probably want to invest a little time
       to develop a tool to automate these steps.

  module:

    A simple stripped down configuration that was used for testing NuttX
    OS modules.  There are five
    very similar NSH configurations:

      - knsh.  This is a somewhat simplified version of the nsh configuration
        that builds using the protected build mode (CONFIG_BUILD_PROTECTED=y).
      - nsh.  This configuration is focused on low level, command-line
        driver testing.  It has no network.
      - netnsh.  This configuration is focused on network testing and
        has only limited command support.
      - module.  A simple stripped down configuration that was used for testing
        NuttXOS modules.
      - mxtxplnd.  This configuration is identical to the nsh configuration
        but assumes that you have a maXTouch Xplained Pro LCD attached
        and includes extra tests for the touchscreen and LCD.

    NOTES:

    1. Kernel Modules / Shared Libraries

       I intend to use this configuration for testing NuttX kernel modules
       in the FLAT build with the following configuration additions to the
       configuration file:

         CONFIG_BOARDCTL_OS_SYMTAB=y
         CONFIG_EXAMPLES_MODULE=y
         CONFIG_EXAMPLES_MODULE_BINDIR="/mnt/sdcard"
         CONFIG_FS_ROMFS=y
         CONFIG_LIBC_ARCH_ELF=y
         CONFIG_MODULE=y
         CONFIG_LIBC_MODLIB=y
         CONFIG_MODLIB_ALIGN_LOG2=2
         CONFIG_MODLIB_BUFFERINCR=32
         CONFIG_MODLIB_BUFFERSIZE=128

       Add the following for testing shared libraries in the FLAT
       build:

         CONFIG_LIBC_DLFCN=y
         CONFIG_EXAMPLES_SOTEST=y
         CONFIG_EXAMPLES_SOTEST_BINDIR="/mnt/sdcard"

    STATUS:
    2017-01-30: Does not yet run correctly.

  mrf24j40-starhub

    This configuration implement a hub node in a 6LoWPAN start network.
    It is intended for the us the mrf24j40-starpoint configuration with
    the clicker2-stm32 configurations.  Essentially, the SAMV71-XULT
    plays the roll of the hub in the configuration and the clicker2-stm32
    boards are the endpoints in the start.

    NOTES:
    1. The serial console is configured by default for use with and Arduino
       serial shield (UART3).  You will need to reconfigure if you will
       to use a different U[S]ART.

    2. This configuration derives from the netnsh configuration, but adds
       support for IPv6, 6LoWPAN, and the MRF24J40 IEEE 802.15.4 radio.

    3. This configuration uses the Mikroe BEE MRF24j40 click boards and
       connects to the SAMV71-XULT using a click shield as described above.

    4. You must must have also have at least two clicker2-stm32 boards each
       with an  MRF24J40 BEE click board in order to run these tests.

    5. The network initialization thread is NOT enabled.  As a result, the
       startup will hang if the Ethernet cable is not plugged in.  For more
       information, see the paragraphs above entitled "Network Initialization
       Thread" and "Network Monitor".

    6. This configuration supports logging of debug output to a circular
       buffer in RAM.  This feature is discussed fully in this Wiki page:
       https://cwiki.apache.org/confluence/display/NUTTX/SYSLOG . Relevant
       configuration settings are summarized below:

       Device Drivers:
         CONFIG_RAMLOG=y             : Enable the RAM-based logging feature.
         CONFIG_RAMLOG_SYSLOG=y      : This enables the RAM-based logger as the
                                     system logger.
         CONFIG_RAMLOG_NONBLOCKING=y : Needs to be non-blocking for dmesg
         CONFIG_RAMLOG_BUFSIZE=8192  : Buffer size is 8KiB

       NOTE: This RAMLOG feature is really only of value if debug output
       is enabled.  But, by default, no debug output is disabled in this
       configuration.  Therefore, there is no logic that will add anything
       to the RAM buffer.  This feature is configured and in place only
       to support any future debugging needs that you may have.

       If you don't plan on using the debug features, then by all means
       disable this feature and save 8KiB of RAM!

       NOTE: There is an issue with capturing data in the RAMLOG:  If
       the system crashes, all of the crash dump information will go into
       the RAMLOG and you will be unable to access it!  You can tell that
       the system has crashed because (a) it will be unresponsive and (b)
       the LD2 will be blinking at about 2Hz.

       You can also reconfigure to use stdout for debug output be disabling
       all of the CONFIG_RAMLOG* settings listed above and enabling the
       following in the .config file:

         CONFIG_SYSLOG_CONSOLE=y

    7. Telnet:  The clicker2-stm32 star point configuration supports the
       Telnet daemon, but not the Telnet client; the star hub configuration
       supports the Telnet client, but not the Telnet daemon.  Therefore,
       the star hub can Telnet to any point in the star, the star endpoints
       cannot initiate telnet sessions.

    8. TCP and UDP Tests:  The same TCP and UDP tests as described for
       the clicker2-stm32 mrf24j40-starpoint configuration are supported on
       the star endpoints, but NOT on the star hub.  Therefore, all network
       testing is between endpoints with the hub acting, well, only like a
       hub.

       The nsh> dmesg command can be use at any time on any node to see
       any debug output that you have selected.

       Telenet sessions may be initiated only from the hub to a star
       endpoint:

         C: nsh> telnet <server-ip> <-- Runs the Telnet client

       Where <server-ip> is the IP address of either the E1 or I2 endpoints.

    STATUS:
      2017-07-02:  Configurations added.  Not yet tested.

      2017-07-03:  Initial testing, appears to be working, but endpoints
        fail to associate; sniffer shows that nothing sent from the star
        hub.  I am thinking that there is something wrong with the
        GPIO interrupt configuration so that no MRF24J40 interrupt are
        being received.

      2017-08-15:  I think the GPIO interrupts are fixed but there still
        seems to be some issue with the SPI communications.

      2017-08-16:  I believe that there is something interfering with the
        MRF24J40 on the SPI0.  There are other things on the bus.  The
        MRF24J40 requires sole use of the SPI bus because it holds MISO
        low when not selected.

        I successfully brought the same logic up on the SAME70-Xplained.
        The SPI signals look clean on the board and the MRF24J40 seems
        fully functional.

      2017-08-26:  There was only a single buffer for reassemblying larger
        packets.  This could be a problem issue for the hub configuration
        which really needs the capability concurrently reassemble multiple
        incoming streams.  The design was extended to support multiple
        reassembly buffers but additional testing is needed.

  mxtxplnd:

    Configures the NuttShell (nsh) located at examples/nsh.  There are five
    very similar NSH configurations:

      - knsh.  This is a somewhat simplified version of the nsh configuration
        that builds using the protected build mode (CONFIG_BUILD_PROTECTED=y).
      - nsh.  This configuration is focused on low level, command-line
        driver testing.  It has no network.
      - netnsh.  This configuration is focused on network testing and
        has only limited command support.
      - module.  A simple stripped down configuration that was used for testing
        NuttXOS modules.
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

    Configures the NuttShell (nsh) located at examples/nsh.  There are five
    very similar NSH configurations:

      - knsh.  This is a somewhat simplified version of the nsh configuration
        that builds using the protected build mode (CONFIG_BUILD_PROTECTED=y).
      - nsh.  This configuration is focused on low level, command-line
        driver testing.  It has no network.
      - netnsh.  This configuration is focused on network testing and
        has only limited command support.
      - module.  A simple stripped down configuration that was used for testing
        NuttXOS modules.
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
         CONFIG_INIT_STACKSIZE=2048
         CONFIG_PTHREAD_STACK_MIN=256
         CONFIG_PTHREAD_STACK_DEFAULT=2048
         CONFIG_POSIX_SPAWN_PROXY_STACKSIZE=1024
         CONFIG_TASK_SPAWN_DEFAULT_STACKSIZE=2048
         CONFIG_NSH_TELNETD_DAEMONSTACKSIZE=2048
         CONFIG_NSH_TELNETD_CLIENTSTACKSIZE=2048

    3. NSH built-in applications are supported.  There are, however, not
       enabled built-in applications.

       Binary Formats:
         CONFIG_BUILTIN=y           : Enable support for built-in programs

       Application Configuration:
         CONFIG_NSH_BUILTIN_APPS=y  : Enable starting apps from NSH command line

    4. The network initialization thread and the NSH network montior are
       enabled in this configuration. As a result, networking initialization
       is performed asynchronously with NSH bring-up.  For more information,
       see the paragraphs above entitled "Network Initialization Thread" and
       "Network Monitor".

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

    Configures the NuttShell (nsh) located at examples/nsh.  There are five
    very similar NSH configurations:

      - knsh.  This is a somewhat simplified version of the nsh configuration
        that builds using the protected build mode (CONFIG_BUILD_PROTECTED=y).
      - nsh.  This configuration is focused on low level, command-line
        driver testing.  It has no network.
      - netnsh.  This configuration is focused on network testing and
        has only limited command support.
      - module.  A simple stripped down configuration that was used for testing
        NuttXOS modules.
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
         CONFIG_INIT_STACKSIZE=2048
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

    5. TWI/I2C

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
           List buses    : bus
           List devices  : dev [OPTIONS] <first> <last>
           Read register : get [OPTIONS] [<repetitions>]
           Show help     : help
           Write register: set [OPTIONS] <value> [<repetitions>]
           Verify access : verf [OPTIONS] [<value>] [<repetitions>]

         Where common "sticky" OPTIONS include:
           [-a addr] is the I2C device address (hex).  Default: 03 Current: 03
           [-b bus] is the I2C bus number (decimal).  Default: 0 Current: 0
           [-r regaddr] is the I2C device register address (hex).  Default: 00 Current: 00
           [-w width] is the data width (8 or 16 decimal).  Default: 8 Current: 8
           [-s|n], send/don't send start between command and data.  Default: -n Current: -n
           [-i|j], Auto increment|don't increment regaddr on repetitions.  Default: NO Current: NO
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

    6. TWIHS0 is also used to support 256 byte non-volatile storage for
       configuration data using the MTD configuration as described above
       under the heading, "MTD Configuration Data".

    7. Support for HSMCI is built-in by default. The SAMV71-XULT provides
       one full-size SD memory card slot.  Refer to the section entitled
       "SD card" for configuration-related information.

       See "Open Issues" above for issues related to HSMCI.

       The auto-mounter is not enabled.  See the section above entitled
       "Auto-Mounter".

    8. Performance-related Configuration settings:

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

         apps/graphics/NxWidgets/nxwm

       The NxWM unit test can be found at:

         apps/graphics/NxWidgets/UnitTests/nxwm

    2. Reading from the LCD is not currently functional.  The following
       settings are in the configuration that tell the system that this
       is a read-only LCD:

         CONFIG_LCD_NOGETRUN=y
         CONFIG_NX_WRITEONLY=y

    3. Small Icons are selected and can be very difficult to touch.  You
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

      - As mentioned above, reading from the LCD is not currently functional.
        There are some special settings work work around this but the
        bottom line is that transparent operations cannot yet be supported.

      - I am seeing some small artifacts with the font used in the HEX
        calculator display.

      - Line spacing in the NxTerm window is too much.  This is probably
        a font-related issue too.

  vnc:

    This is a special version of an NSH configuration.  It has networking
    and graphics enabled.  It is configured to use the VNC server to provide
    a remote desktop for use with VNC client on a PC.  It includes the
    graphics text at apps/examples/nximage.

    NOTES:

    1. Network configuration:  IP address 10.0.0.2.  The is easily changed
       via 'make menuconfig'.  The VNC server address is 10.0.0.2:5900.

    2. The default (local) framebuffer configuration is 320x240 with 8-bit
       RGB color.

    3. There are complicated interactions between VNC and the network
       configuration.  The CONFIG_VNCSERVER_UPDATE_BUFSIZE determines the
       size of update messages.  That is 1024 bytes in that configuration
       (the full message with the header will be a little larger).  The
       CONFIG_NET_ETH_PKTSIZE is set to 590 so that a full update will
       require several packets.

       Write buffering also effects network performance.  This will break
       up the large updates into small (196 byte) groups.  When we run out
       of read-ahead buffers, then partial updates may be sent causing a
       loss of synchronization.

    4. Hint: If you are debugging using the RealVNC clint, turn off all
       mouse/keyboard inputs in the options/input menu.  That will make
       things a little clearer.

    5. To select 16-bits per pixel RGB15 5:6:5

         CONFIG_NX_DISABLE_8BPP=y
         # CONFIG_NX_DISABLE_16BPP is not set

         # CONFIG_VNCSERVER_COLORFMT_RGB8 is not set
         CONFIG_VNCSERVER_COLORFMT_RGB16=y

         CONFIG_EXAMPLES_NXIMAGE_BPP=16

       To re-select 8-bits per pixel RGB8 3:3:2

         # CONFIG_NX_DISABLE_8BPP is not set
         CONFIG_NX_DISABLE_16BPP=y

         CONFIG_VNCSERVER_COLORFMT_RGB8=y
         # CONFIG_VNCSERVER_COLORFMT_RGB16 is not set

         # CONFIG_EXAMPLES_NXIMAGE_GREYSCALE is not set
         CONFIG_EXAMPLES_NXIMAGE_BPP=8

    STATUS:
      2016-04-21:  I have gotten the apps/examples/nximage to work with
        lots issues with 16-bit RGB and verbose GRAPHICS and UPDATER debug
        ON.  There are reliability problems and it hangs at the end of the
        test.

      2016-04-22:  The default configuration now uses RGB8 which needs a lot
        less SRAM for the local frame buffer and does not degrade the color
        quality in the remote display (since it is also 8 BPP).  At 8
        BPP, the remote display is correct even with both GRAPHICS and
        UPDATER debug OFF -- and there is no hang!

     2106-04-23:  The NxImage test was selected because it is a very simple
        graphics test.  Continued testing, however, requires a more complex
        configuration.  Hence, the vnxwm configuration was created.

        A memory clobber error was fixed and this probably corrects some of
        the reliability problems noted on 2016-04-21.

  vnxwm:

    This is a special configuration setup for the NxWM window manager
    UnitTest.  It provides an interactive windowing experience via a remote
    VNC client window running on your PC.  The SAMV71-XULT is connected to
    the PC via Ethernet.

    NOTES:

    1. The NxWM window manager is a tiny window manager tailored for use
       with smaller LCDs.  It supports a task, a start window, and
       multiple application windows with toolbars.  However, to make the
       best use of the visible LCD space, only one application window is
       visible at at time.

       The NxWM window manager can be found here:

         apps/graphics/NxWidgets/nxwm

       The NxWM unit test can be found at:

         apps/graphics/NxWidgets/UnitTests/nxwm

    2. Network configuration:  IP address 10.0.0.2.  The is easily changed
       via 'make menuconfig'.  The VNC server address is 10.0.0.2:5900.

    3. The default (local) framebuffer configuration is 320x240 with 8-bit
       RGB color.

       I had some problems at 16-bits per pixle (see STATUS below).  To
       select 16-bits per pixel RGB15 5:6:5

         CONFIG_NX_DISABLE_8BPP=y
         # CONFIG_NX_DISABLE_16BPP is not set

         # CONFIG_VNCSERVER_COLORFMT_RGB8 is not set
         CONFIG_VNCSERVER_COLORFMT_RGB16=y

         CONFIG_EXAMPLES_NXIMAGE_BPP=16

       To re-select 8-bits per pixel RGB8 3:3:2

         # CONFIG_NX_DISABLE_8BPP is not set
         CONFIG_NX_DISABLE_16BPP=y

         CONFIG_VNCSERVER_COLORFMT_RGB8=y
         # CONFIG_VNCSERVER_COLORFMT_RGB16 is not set

         # CONFIG_EXAMPLES_NXIMAGE_GREYSCALE is not set

    2. There are complicated interactions between VNC and the network
       configuration.  The CONFIG_VNCSERVER_UPDATE_BUFSIZE determines the
       size of update messages.  That is 1024 bytes in that configuration
       (the full message with the header will be a little larger).  The
       CONFIG_NET_ETH_PKTSIZE is set to 590 so that a full update will
       require several packets.

       Write buffering also effects network performance.  This will break
       up the large updates into small (196 byte) groups.  When we run out
       of read-ahead buffers, then partial updates may be sent causing a
       loss of synchronization.

    STATUS:
       2106-04-23:  Configuration created.  See status up to this data in
         the vnc configuration.  That probably all applies here as well.

         Only some initial testing has been performed:  The configuration
         is partially functional.  Menus do appear and mouse input is
         probably working correctly.

         But there are a lot of instabilities.  I see assertions of
         various kinds and the RealVNC client often crashes as well.
         Some of the assertions I see are:

           while (sem_wait(&session->queuesem) < 0)
           ...
           rect = (FAR struct vnc_fbupdate_s *)sq_remfirst(&session->updqueue);
           DEBUGASSERT(rect != NULL);

         I would think that could mean only that the semaphore counting is
         out of sync with the number of updates in the queue.

         But also the assertion at devif/devif_iobsend.c line: 102 which
         probably means some kind of memory corruption.

       2017-01-30: knsh configuration does not yet run correctly.

  mcuboot-loader:
    This configuration exercises the port of MCUboot loader to NuttX.

    In this configuration both primary, secondary and scratch partitions are
    mapped into the internal flash.
    Relevant configuration settings:

      CONFIG_BOARD_LATE_INITIALIZE=y

      CONFIG_BOOT_MCUBOOT=y
      CONFIG_MCUBOOT_BOOTLOADER=y

      CONFIG_SAMV7_FORMAT_MCUBOOT=y
      CONFIG_INIT_ENTRYPOINT="mcuboot_loader_main"

      Flash bootloader using embedded debugger:
      openocd -f interface/cmsis-dap.cfg \
              -c 'transport select swd' \
	      -c 'set CHIPNAME atsamv71q21' \
	      -f target/atsamv.cfg \
	      -c 'reset_config srst_only' \
	      -c init -c targets \
	      -c 'reset halt' \
	      -c 'program nuttx.bin 0x400000' \
	      -c 'reset halt' \
	      -c 'atsamv gpnvm set 1' \
	      -c 'reset run' -c shutdown

  mcuboot-nsh:
    This configuration exercises the MCUboot compatible application slot
    example. The application is NuttX nsh with some special commands.

    Generate signed binaries for MCUboot compatible application:

      ./apps/boot/mcuboot/mcuboot/scripts/imgtool.py sign \
        --key apps/boot/mcuboot/mcuboot/root-rsa-2048.pem --align 8 \
        --version 1.0.0 --header-size 0x200 --pad-header --slot-size 0xe0000 \
        nuttx/nuttx.bin mcuboot_nuttx.app.nsh.confirmed-v1.bin

      ./apps/boot/mcuboot/mcuboot/scripts/imgtool.py sign \
        --key apps/boot/mcuboot/mcuboot/root-rsa-2048.pem --align 8 \
        --version 2.0.0 --header-size 0x200 --pad-header --slot-size 0xe0000 \
        nuttx/nuttx.bin mcuboot_nuttx.app.nsh.confirmed-v2.bin

      Flash application version 1.0.0 at MCUboot Slot-0:

      openocd -f interface/cmsis-dap.cfg \
              -c 'transport select swd' \
	      -c 'set CHIPNAME atsamv71q21' \
	      -f target/atsamv.cfg \
	      -c 'reset_config srst_only' \
	      -c init -c targets \
	      -c 'reset halt' \
	      -c 'program mcuboot_nuttx.app.nsh.confirmed-v1.bin 0x420000' \
	      -c 'reset halt' \
	      -c 'atsamv gpnvm set 1' \
	      -c 'reset run' -c shutdown

      Flash version 2.0.0 at MCUboot Slot-1:

      openocd -f interface/cmsis-dap.cfg \
              -c 'transport select swd' \
	      -c 'set CHIPNAME atsamv71q21' \
	      -f target/atsamv.cfg \
	      -c 'reset_config srst_only' \
	      -c init -c targets \
	      -c 'reset halt' \
	      -c 'program mcuboot_nuttx.app.nsh.confirmed-v2.bin 0x500000' \
	      -c 'reset halt' \
	      -c 'atsamv gpnvm set 1' \
	      -c 'reset run' -c shutdown

    Relevant configuration settings:

      CONFIG_BOOT_MCUBOOT=y
      CONFIG_MCUBOOT_SLOT_CONFIRM_EXAMPLE=y

      CONFIG_SAMV7_FORMAT_MCUBOOT=y
      CONFIG_INIT_ENTRYPOINT="nsh_main"
