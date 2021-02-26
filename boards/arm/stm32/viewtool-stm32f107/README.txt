README
======

  This README discusses issues unique to NuttX configurations for the
  ViewTool STM32F103/F107 V1.2 board.  This board may be fitted with either

    - STM32F107VCT6, or
    - STM32F103VCT6

  The board is very modular with connectors for a variety of peripherals.
  Features on the base board include:

    - User and Wake-Up Keys
    - LEDs

  See http://www.viewtool.com/ for further information.

Contents
========

  o User and Wake-Up keys
  o LEDs
  o Serial Console
    - Console Configuration
    - J5 - USART1
    - PL-2013 USB-to-Serial Interface
    - RS-232 Module
  o USB Interface
  o microSD Card Interface
  o ViewTool DP83848 Ethernet Module
  o Freescale MPL115A barometer sensor
  o LCD/Touchscreen Interface
  o FT80x Integration
  o MAX3421E Integration
  o Toolchains
    - NOTE about Windows native toolchains
  o Configurations
    - Information Common to All Configurations
    - Configuration Sub-directories

User and Wake-Up keys
=====================

  All pulled high and will be sensed low when depressed.

    SW2 PC11  Needs J42 closed
    SW3 PC12  Needs J43 closed
    SW4 PA0   Needs J44 closed

LEDs
====

  There are four LEDs on the ViewTool STM32F103/F107 board that can be controlled
  by software:  LED1 through LED4.  All pulled high and can be illuminated by
  driving the output to low

    LED1 PA6
    LED2 PA7
    LED3 PB12
    LED4 PB13

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/stm32_leds.c. The LEDs are used to encode OS-related
  events as follows:

    SYMBOL            Meaning                      LED state
                                               LED1 LED2 LED3 LED4
    ----------------- -----------------------  ---- ---- ---- ----
    LED_STARTED       NuttX has been started   ON   OFF  OFF  OFF
    LED_HEAPALLOCATE  Heap has been allocated  OFF  ON   OFF  OFF
    LED_IRQSENABLED   Interrupts enabled       ON   ON   OFF  OFF
    LED_STACKCREATED  Idle stack created       OFF  OFF  ON   OFF
    LED_INIRQ         In an interrupt          N/C  N/C  N/C  Soft glow
    LED_SIGNAL        In a signal handler      N/C  N/C  N/C  Soft glow
    LED_ASSERTION     An assertion failed      N/C  N/C  N/C  Soft glow
    LED_PANIC         The system has crashed   N/C  N/C  N/C  2Hz Flashing
    LED_IDLE          MCU is is sleep mode         Not used

  After booting, LED1-3 are not longer used by the system and can be used for
  other purposes by the application (Of course, all LEDs are available to the
  application if CONFIG_ARCH_LEDS is not defined.

Serial Console
==============

  Console Configuration
  ---------------------
  The NuttX console is configured by default on USART1 at 115200 BAUD 8N1
  (8-bits, not parity, one stop bit).  These setting can, of course, easily
  be changed by reconfiguring NuttX.

  J5 - USART1
  -----------
  The boards come with a PL-2303 based USB-to-serial board.  Also available
  as an option is an RS-232 board.  Both have the same pin out on a 6-pin
  connector that mates with the upper row of J5.

    PIN MODULE BOARD J5
    --- ------ ---------------------------
     1   5V    1  POWER Power jumper
     2   GND   3  GND   Ground
     3   TXD   5  RXD1  PA10    USART1_RXD
     4   RXD   7  TXD1  PA9     USART1_TXD
     5   RTS?  9  CTS?  PA12    USART1_RTS
     6   CTS?  11 RTS?  PA11    USART1_CTS

  PL-2013 USB-to-Serial Interface
  -------------------------------

    J37 - CON4.  Jumper Settings:
      1 <-> 3 : Connects PA9 to the RXD1 output pin
      2 <-> 4 : Connects PA10 to the TXD1 input pin

    J35 - CON2.  Jumper Setting:
      Open.  the PL2303 adapter receives its power from the USB host.

  RS-232 Module
  -------------

    J37 - CON4.  Jumper Settings:
      1 <-> 3 : Connects PA9 to the RXD1 output pin
      2 <-> 4 : Connects PA10 to the TXD1 input pin

    J35 - CON2.  Jumper Setting:
      1 <-> 2 : Proves 3.3V to the RS-232 module.

USB Interface
=============

  USB Connector
  -------------

  The Viewtool base board has a USB Mini-B connector.  Only USB device can
  be supported with this connector.

  ------------------------- ------------------------------------
         USB Connector
         J10 mini-USB       GPIO CONFIGURATION(s)
  --- --------- ----------- ------------------------------------
  Pin Signal
  --- --------- ----------- ------------------------------------
   1  USB_VBUS  VDD_USB     (No sensing available)
   2  OTG_DM    PA11        GPIO_OTGFS_DM (F107) GPIO_USB_DM (F103)
   3  OTG_DP    PA12        GPIO_OTGFS_DP (F107) GPIO_USB_DP (F103)
   4  OTG_ID    PA10        GPIO_OTGFS_ID (F107)
   5  Shield    N/A         N/A
   6  Shield    N/A         N/A
   7  Shield    N/A         N/A
   8  Shield    N/A         N/A
   9  Shield    N/A         N/A
                PE11 USB_EN   GPIO controlled soft pull-up (if J51 closed)

   NOTES:
   1. GPIO_OTGFS_VBUS (F107) should not be configured.  No VBUS sensing
   2. GPIO_OTGFS_SOF (F107) is not used
   3. The OTG FS module has is own, internal soft pull-up logic.  J51 should
      be open so that PE11 activity does effect USB.


  STM32F103 Configuration
  -----------------------

    System Type -> STM32 Peripheral Support
      CONFIG_STM32_USB=y                 : Enable USB FS device

    Device Drivers
      CONFIG_USBDEV                      : USB device support

    STATUS:  All of the code is in place, but no testing has been performed.

  STM32F107 Configuration
  -----------------------

    System Type -> STM32 Peripheral Support
      CONFIG_STM32_OTGFS=y               : Enable OTG FS

    Device Drivers
      CONFIG_USBDEV                      : USB device support

    STATUS:  All of the code is in place, but USB is not yet functional.

  CDC/ACM Configuration
  ---------------------

  This will select the CDC/ACM serial device.  Defaults for the other
  options should be okay.

    Device Drivers -> USB Device Driver Support
      CONFIG_CDCACM=y                     : Enable the CDC/ACM device

  The following setting enables an example that can can be used to control
  the CDC/ACM device.  It will add two new NSH commands:

    a. sercon will connect the USB serial device (creating /dev/ttyACM0), and
    b. serdis which will disconnect the USB serial device (destroying
        /dev/ttyACM0).

    Application Configuration -> Examples:
      CONFIG_SYSTEM_CDCACM=y              : Enable an CDC/ACM example

  USB MSC Configuration
  ---------------------
  [WARNING: This configuration has not yet been verified]

  The Mass Storage Class (MSC) class driver can be selected in order to
  export the microSD card to the host computer.  MSC support is selected:

    Device Drivers -> USB Device Driver Support
      CONFIG_USBMSC=y                       : Enable the USB MSC class driver
      CONFIG_USBMSC_EPBULKOUT=1             : Use EP1 for the BULK OUT endpoint
      CONFIG_USBMSC_EPBULKIN=2              : Use EP2 for the BULK IN endpoint

  The following setting enables an add-on that can can be used to control
  the USB MSC device.  It will add two new NSH commands:

    a. msconn will connect the USB serial device and export the microSD
       card to the host, and
    b. msdis which will disconnect the USB serial device.

    Application Configuration -> System Add-Ons:
      CONFIG_SYSTEM_USBMSC=y                : Enable the USBMSC add-on
      CONFIG_SYSTEM_USBMSC_NLUNS=1          : One LUN
      CONFIG_SYSTEM_USBMSC_DEVMINOR1=0      : Minor device zero
      CONFIG_SYSTEM_USBMSC_DEVPATH1="/dev/mmcsd0"
                                            : Use a single, LUN:  The microSD
                                            : block driver.

    NOTES:

    a. To prevent file system corruption, make sure that the microSD is un-
       mounted *before* exporting the mass storage device to the host:

         nsh> umount /mnt/sdcard
         nsh> mscon

       The microSD can be re-mounted after the mass storage class is disconnected:

        nsh> msdis
        nsh> mount -t vfat /dev/mtdblock0 /mnt/at25

microSD Card Interface
======================

  microSD Connector
  -----------------

    ----------------------------- ------------------------- --------------------------------
           Connector J17            GPIO CONFIGURATION(s)
    PIN SIGNAL        LEGEND          (no remapping)                 DP83848C Board
    --- ------------- ----------- ------------------------- --------------------------------
    1   VDD 3.3       N/A         N/A                       3.3
    2   GND           N/A         N/A                       GND
    3   PC8           SDIO_D0     GPIO_SDIO_D0              D0
    4   PD2           SDIO_CMD    GPIO_SDIO_CMD             CMD
    5   PC12          SDIO_CLK    GPIO_SDIO_CK              CLK
    6   PC11          SDIO_D3     GPIO_SDIO_D3              D3
    7   PC10          SDIO_D2     GPIO_SDIO_D2              D2
    8   PC9           SDIO_D1     GPIO_SDIO_D1              D1
    9   PA8           CD          Board-specific GPIO input CD
    --- ------------- ----------- ------------------------- --------------------------------

    NOTES:
    1. The STM32F107 does not support the SDIO/memory card interface.  So the SD card
       cannot be used with the STM32F107 (unless the pin-out just happens to match up
       with an SPI-based card interface???)

  Configuration (STM32F103 only)
  ------------------------------
  [WARNING: This configuration has not yet been verified]

  Enabling SDIO-based MMC/SD support:

    System Type->STM32 Peripheral Support
      CONFIG_STM32_SDIO=y                   : Enable SDIO support
      CONFIG_STM32_DMA2=y                   : DMA2 is needed by the driver

    Device Drivers -> MMC/SD Driver Support
      CONFIG_MMCSD=y                        : Enable MMC/SD support
      CONFIG_MMSCD_NSLOTS=1                 : One slot per driver instance
      CONFIG_MMCSD_HAVE_CARDDETECT=y         : Supports card-detect PIOs
      CONFIG_MMCSD_MMCSUPPORT=n             : Interferes with some SD cards
      CONFIG_MMCSD_SPI=n                    : No SPI-based MMC/SD support
      CONFIG_MMCSD_SDIO=y                   : SDIO-based MMC/SD support
      CONFIG_SDIO_DMA=y                     : Use SDIO DMA
      CONFIG_SDIO_BLOCKSETUP=y              : Needs to know block sizes

    Library Routines
      CONFIG_SCHED_WORKQUEUE=y              : Driver needs work queue support

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

    Using the SD card
    -----------------

    1) After booting, an SDIO device will appear as /dev/mmcsd0

    2) If you try mounting an SD card with nothing in the slot, the
       mount will fail:

         nsh> mount -t vfat /dev/mmcsd1 /mnt/sd1
         nsh: mount: mount failed: 19

    STATUS:  All of the code is in place, but no testing has been performed.

ViewTool DP83848 Ethernet Module
================================

  Ethernet Connector
  ------------------

    ----------------------------- ------------------------ --------------------------------
           Connector J2            GPIO CONFIGURATION(s)
    PIN SIGNAL        LEGEND         (no remapping)                 DP83848C Board
    --- ------------- ----------- ------------------------ --------------------------------
    1   PA0           MII_CRS     N/A                      N/C
    2   PB11/SDA2     COM_TX_EN   GPIO_ETH_RMII_TX_EN      TX_EN
    3   PA3/LED_G2    MII_COL     N/A                      N/C
    4   PB12/NSS2     COM_TXD0    GPIO_ETH_RMII_TXD0       TXD0
    5   PA1           MII_RX_CLK  GPIO_ETH_RMII_REF_CLK    OSCIN
    6   PB13/SCK2     COM_TXD1    GPIO_ETH_RMII_TXD1       TXD1
    7   PB1/CD_RESET  MII_RXD3    N/A                      N/C
    8   PC4/LCDTP     COM_RXD0    GPIO_ETH_RMII_RXD0       RXD0
    9   PB0/BL_PWM    MII_RXD2    N/A                      N/C
    10  PC5           COM_RXD1    GPIO_ETH_RMII_RXD1       RXD1
    11  PB8/CAN1_RX   MII_TXD3    N/A                      N/C
    12  PC1/LED_R1    COM_MDC     GPIO_ETH_MDC             MDC
    13  PC2/LED_R2    MII_TXD2    N/A                      N/C
    14  PA2/LED_G1    COM_MDIO    GPIO_ETH_MDIO            MDIO
    15  PC3/ONEW      MII_TX_CLK  N/A                      N/C
    16  PB10/SCL2     RX_ER       N/A                      N/C
    17  PD2           GPIO1       N/A                      N/C
    18  PA7/MOSI1     COM_RX_DV   GPIO_ETH_RMII_CRS_DV     CRS_DIV
    19  PD3           GPIO2       N/A                      N/C
    20  PB5           COM_PPS_OUT N/A                      N/C
    21  VDD 3.3       VDD_3.3     N/A                      3.3V
    22  VDD 3.3       VDD_3.3     N/A                      3.3V
    23  GND           GND         N/A                      GND
    24  GND           GND         N/A                      GND
    --- ------------- ----------- ------------------------ --------------------------------

    NOTES:
    1. RMII interface is used
    2. There is a 50MHz clock on board the DP83848.  No MCO clock need be provided.

  Configuration
  -------------

    System Type -> STM32 Peripheral Support
      CONFIG_STM32_ETHMAC=y                  : Enable Ethernet driver

    System Type -> Ethernet MAC Configuration
      CONFIG_STM32_RMII=y                    : Configuration RM-II DP83848C PHY
      CONFIG_STM32_AUTONEG=y
      CONFIG_STM32_PHYADDR=1
      CONFIG_STM32_PHYSR=16
      CONFIG_STM32_PHYSR_SPEED=0x0002
      CONFIG_STM32_PHYSR_100MBPS=0x0000
      CONFIG_STM32_PHYSR_MODE=0x0004
      CONFIG_STM32_PHYSR_FULLDUPLEX=0x0004
      CONFIG_STM32_RMII_EXTCLK=y

    Device Drivers -> Networking Devices
      CONFIG_NETDEVICES=y                    : More PHY stuff
      CONFIG_ETH0_PHY_DP83848C=y

    Networking (required)
      CONFIG_NET=y                           : Enabled networking support
      CONFIG_NSH_NOMAC=y

    Networking (recommended/typical)
      CONFIG_NSOCKET_DESCRIPTORS=10          : Socket-related
      CONFIG_NET_SOCKOPTS=y

      CONFIG_NET_ETH_PKTSIZE=650             : Maximum packet size

      CONFIG_NET_TCP=y                       : TCP support
      CONFIG_NET_NTCP_READAHEAD_BUFFERS=8

      CONFIG_NET_UDP=y                       : UDP support
      CONFIG_NET_UDP_CONNS=8

      CONFIG_NET_ICMP=y                      : ICMP support
      CONFIG_NET_ICMP_SOCKET=y

      CONFIG_NSH_DRIPADDR=0x0a000001         : Network identity
      CONFIG_NSH_IPADDR=0x0a000002
      CONFIG_NSH_NETMASK=0xffffff00

    Network Utilities (basic)
      CONFIG_NETUTILS_TFTPC=y                : Needed by NSH unless to disable TFTP commands
      CONFIG_NETUTILS_DHCPC=y                : Fun stuff
      CONFIG_NETUTILS_TELNETD=y              : Support for a Telnet NSH console
      CONFIG_NSH_TELNET=y

      (also FTP, TFTP, WGET, NFS, etc. if you also have a mass storage
      device).

Freescale MPL115A barometer sensor
==================================

  This board support package includes hooks that can be used to enable
  testing of a Freescale MPL115A barometer sensor connected via SPI3 with
  chip select on PB6,

  Here are the configuration settings that would have to be included to
  enabled support for the barometer:

    System Type -> Peripherals
      CONFIG_STM32_SPI3=y

    Drivers -> SPI
      CONFIG_SPI=y
      CONFIG_SPI_EXCHANGE=y

    Drivers -> Sensors
      CONFIG_SENSORS=y
      CONFIG_SENSORS_MPL115A=y
      CONFIG_NSH_ARCHINIT=y


  Note: this driver uses SPI3 then since PB3 pin is also use to JTAG TDO you
  need to disable JTAG support to get this driver working:

    System Type
      CONFIG_STM32_JTAG_DISABLE=y

LCD/Touchscreen Interface
=========================

  An LCD may be connected via J11.  Only the STM32F103 supports the FSMC signals
  needed to drive the LCD.

  The LCD features an (1) HY32D module with built-in SSD1289 LCD controller, and (a)
  a XPT2046 touch screen controller.

  LCD Connector
  -------------

    ----------------------------- ------------------------ --------------------------------
           Connector J11           GPIO CONFIGURATION(s)
    PIN SIGNAL        LEGEND          (F103 only)                   LCD Module
    --- ------------- ----------- ------------------------ --------------------------------
    1   VDD_5         NC          N/A                      5V      ---
    2   GND           GND         N/A                      GND     ---
    3   PD14          DATA0       GPIO_NPS_D0              D0      HY32D
    4   PD15          DATA1       GPIO_NPS_D1              D1      HY32D
    5   PD0           DATA2       GPIO_NPS_D2              D2      HY32D
    6   PD1           DATA3       GPIO_NPS_D3              D3      HY32D
    7   PE7           DATA4       GPIO_NPS_D4              D4      HY32D
    8   PE8           DATA5       GPIO_NPS_D5              D5      HY32D
    9   PE9           DATA6       GPIO_NPS_D6              D6      HY32D
    10  PE10          DATA7       GPIO_NPS_D7              D7      HY32D
    11  PE11          DATA8       GPIO_NPS_D8              D8      HY32D
    12  PE12          DATA9       GPIO_NPS_D9              D9      HY32D
    13  PE13          DATA10      GPIO_NPS_D10             D10     HY32D
    14  PE14          DATA11      GPIO_NPS_D11             D11     HY32D
    15  PE15          DATA12      GPIO_NPS_D12             D12     HY32D
    16  PD8           DATA13      GPIO_NPS_D13             D13     HY32D
    17  PD9           DATA14      GPIO_NPS_D14             D14     HY32D
    18  PD10          DATA15      GPIO_NPS_D15             D15     HY32D
    19  (3)           LCD_CS      GPIO_NPS_NE1             CS      HY32D
    20  PD11          LCD_RS      GPIO_NPS_A16             RS      HY32D
    21  PD5           LCD_R/W     GPIO_NPS_NWE             WR      HY32D
    22  PD4           LCD_RD      GPIO_NPS_NOE             RD      HY32D
    23  PB1           LCD_RESET   (GPIO)                   RESET   HY32D
    24  N/C           NC          N/A                      TE      (unused?)
    25  VDD_3.3       BL_VCC      N/A                      BLVDD   CA6219 (Drives LCD backlight)
    26  GND           BL_GND      N/A                      BLGND   CA6219
    27  PB0           BL_PWM      GPIO_TIM3_CH3OUT(2)      BL_CNT  CA6219
    28  PC5           LCDTP_IRQ   (GPIO)                   TP_IRQ  XPT2046
    29  PC4           LCDTP_CS    (GPIO)                   TP_CS   XPT2046
    30  PB13          LCDTP_CLK   GPIO_SPI2_SCK            TP_SCK  XPT2046
    31  PB15          LCDTP_DIN   GPIO_SPI2_MOSI           TP_SI   XPT2046
    32  PB14          LCDTP_DOUT  GPIO_SPI2_MISO           TP_SO   XPT2046
    33  VDD_3.3       VDD_3.3     N/A                      3.3V    ---
    34  GND           GND         N/A                      GND     ---
    --- ------------- ----------- ------------------------ --------------------------------

    NOTES:
    1) Only the F103 version of the board supports the FSMC
    2) No remap
    3) LCD_CS is controlled by J13 JUMPER4 (under the LCD unfortunately):

       1->2 : PD7 (GPIO_NPS_NE1) enables the multiplexor  : 1E\ enable input (active LOW)
       3->4 : PD13 provides 1A0 input (1A1 is grounded).  : 1A0 address input
              So will chip enable to either LCD_CS or
              Flash_CS.
       5->6 : 1Y0 output to LCD_CS                        : 1Y0 address output
       7->8 : 1Y1 output to Flash_CE                      : 1Y1 address output

       Truth Table:
       1E\ 1A0 1A1 1Y0 1Y1
       --- --- --- --- ---
       HI  N/A N/A HI  HI
       LO  LO  LO  LO  HI
       LO  HI  LO  HI  LO

FT80x Integration
=================

  I have used the ViewTool F107 for initial testing of the three displays
  based on FTDI/BridgeTek FT80x GUIs:

  Haoyu 5"
  --------
  I purchased a Haoyu 5" FT800 display on eBay.  Pin out and board
  connectivity is as follows:

  2x5 Connector J2 using SPI1:
  PIN  NAME   VIEWTOOL    STM32      PIN  NAME   VIEWTOOL   STM32
   1   5V     J18 Pin  2              2   GND    J8 Pin  8
   3   SCK    J8  Pin 11  PA5/SCK1    4   MISO   J8 Pin  9  PA6/MISO1
   5   MOSI   J8  Pin 10  PA7/MOSI1   6   CS     J8 Pin 12  PA4/NSS1
   7   INT    J18 Pin  8  PA1         8   PD     J18 Pin 6  PC5
   9   AUDIO-L                       10   GND

  2x5 Connector J2 using SPI2:
  PIN  NAME   VIEWTOOL    STM32      PIN  NAME   VIEWTOOL   STM32
   1   5V     J18 Pin  2              2   GND    J8  Pin 2
   3   SCK    J8  Pin  5  PB13/SCK2   4   MISO   J8  Pin 3  PB14/MISO2
   5   MOSI   J8  Pin  4  PB15/MOSI2  6   CS     J8  Pin 6  PB12/NSS2
   7   INT    J18 Pin  8  PA1         8   PD     J18 Pin 6  PC5
   9   AUDIO-L                       10   GND    J18 Pin 4

  The Haoyu display has no audio amplifier on board;  Output is raw PWM
  audio.

  GPIO0 and MODE are pulled low meaning that SPI is the default interface
  with slave address bit 0 = 0.  GPIO1 is not connected.

  This display should have:

    CONFIG_LCD_FT800=y
    CONFIG_LCD_FT80X_SPI=y
    CONFIG_LCD_FT80X_WQVGA=y
    CONFIG_LCD_FT80X_AUDIO_NOSHUTDOWN=y
    CONFIG_EXAMPLES_FT80X_DEVPATH="/dev/ft800"

  MikroElektronkia ConnectEVE FT800
  ---------------------------------

  2x5 Connector CN2 using SPI1:
  ---- ------ ----------- ---------- ---- ------ ---------- ----------
  PIN  NAME   VIEWTOOL    STM32      PIN  NAME   VIEWTOOL   STM32
  ---- ------ ----------- ---------- ---- ------ ---------- ----------
   1   PD#    J18 Pin 6   PC5         2   INT#   J18 Pin  8 PA1
   3   CS#    J8  Pin 12  PA4/NSS1    4   SCK    8   Pin 11 PA5/SCK1
   5   MISO   J8  Pin  9  PA6/MISO1   6   MOSI   J8  Pin 10 PA7/MOSI1
   7   N/C                            8   N/C
   9   3.3V   J8  Pin 7              10   GND    J8  Pin  8

  2x5 Connector CN2 using SPI2:
  ---- ------ ----------- ---------- ---- ------ ---------- ----------
  PIN  NAME   VIEWTOOL    STM32      PIN  NAME   VIEWTOOL   STM32
  ---- ------ ----------- ---------- ---- ------ ---------- ----------
   1   PD#    J18 Pin 6   PC5         2   INT#   J18 Pin  8 PA1
   3   CS#    J8  Pin 6   PB12/NSS2   4   SCK    J8  Pin  5 PB13/SCK2
   5   MISO   J8  Pin 3   PB14/MISO2  6   MOSI   J8  Pin  4 PB15/MOSI2
   7   N/C                            8   N/C
   9   3.3V   J8  Pin 1              10   GND    J8  Pin  2

  1x10 Connector CN3 using SPI1:
  ---- ------ ----------- -----------
  PIN  NAME   VIEWTOOL    STM32
  ---- ------ ----------- -----------
   1   CS#    J8  Pin 12  PA4/NSS1
   2   SCK    J8  Pin 11  PA5/SCK1
   3   MISO   J8  Pin  9  PA6/MISO1
   4   MOSI   J8  Pin 10  PA7/MOSI1
   5   INT#   J18 Pin  8  PA1
   6   PD#    J18 Pin  6  PC5
   7   AUDIO+
   8   AUDIO-
   9   3.3V   J8  Pin 7
  10   GND    J8  Pin 8

  1x10 Connector CN3 using SPI2:
  ---- ------ ----------- -----------
  PIN  NAME   VIEWTOOL    STM32
  ---- ------ ----------- -----------
   1   CS#    J8  Pin  6  PB12/NSS2
   2   SCK    J8  Pin  5  PB13/SCK2
   3   MISO   J8  Pin  3  PB14/MISO2
   4   MOSI   J8  Pin  4  PB15/MOSI2
   5   INT#   J18 Pin  8  PA1
   6   PD#    J18 Pin  6  PC5
   7   AUDIO+
   8   AUDIO-
   9   3.3V   J8  Pin 1
  10   GND    J8  Pin 2

  Configurations using FT80x should not enable Ethernet, CAN2 or LED
  support.  The LCD connector, J28 pin 9,  and the upper row of J18 are
  also assumed to be unused:

  J8 upper row (SPI2) conflicts:

    Pin  2 PB14 also used by LCD
    Pin  4 PB15 also used by LCD
    Pin  5 PB13 also used by Ethernet, CAN2, LCD and LED4
    Pin  6 PB12 also used by Ethernet, CAN2, J28 pin 9, and LED3

  J8 lower row (SPI1) conflicts:

    Pin  9 PA6 also used by J8 pin 9 and LED1
    Pin 10 PA7 also used Ethernet
    Pin 11 PA5 also used by J8 pin 7
    Pin 12 PA4 also used by J8 pin 5 (J8 pin 5 not used)

  J18 upper row is not used in this configuration.  Cannot be used with
  SPI1.  Not used with SPI2 because SPI2 has the same conflicts as the
  lower row so why bother?

    Pin  5 PA4 also used by SPI1/NSS1
    Pin  7 PA5 also used by SPI1/SCK1
    Pin  9 PA6 also used by SPI1/MOSI1 and LED1

  J18 lower row conflicts:

    Pin  6 PC5 also used by Ethernet and the LCD interface
    Pin  8 PA1 also used by Ethernet
    Pin 10 PA0 also used by Ethernet and Wake-up button (not used)

  Remapped SPI1 pins are not supported, but that would permit these options:

    PA15/NSS1 also used by LCD
    PB3/SCK1  also used by USART1 and JTAG
    PB4/MISO1 also used by JTAG
    PB5/MOSI1 also used by USART1, Ethernet, and J28 pin 10

  There is a LM4864 audio amplifier on board so audio outputs are ready for
  use with a small 1W 8Ohm speaker.    GPIO0 should be configured as an
  output because it is used to control the shutdown pin of the LM4864 audio
  output.

  GPIO0 is not connected.

  This display should have:

    CONFIG_LCD_FT800=y
    CONFIG_LCD_FT80X_SPI=y
    CONFIG_LCD_FT80X_WQVGA=y
    CONFIG_LCD_FT80X_AUDIO_GPIOSHUTDOWN=y
    CONFIG_LCD_FT80X_AUDIO_GPIO=0
    CONFIG_EXAMPLES_FT80X_DEVPATH="/dev/ft800"

  Reverdi RVT43ULFNWC01
  ---------------------

  I used this FT801 board with a 20 pin breakout module.

  2x10 Connector CN2 using SPI1:
  ---- --------- ----------- ----------- ---- --------- ----------- -----------
  PIN  NAME      VIEWTOOL    STM32       PIN  NAME      VIEWTOOL    STM32
  ---- --------- ----------- ----------- ---- --------- ----------- -----------
    1  VDD       J8  Pin  7 *             2  GND        J8  Pin  8
    3  SPI_CLK   J8  Pin 11  PA5/SCK1     4  MISO       J8  Pin  9  PA6/MISO1
    5  MOSI/IO1  J8  Pin 10  PA7/MOSI1    6  CS         J8  Pin 12  PA4/NSS1
    7  INT       J18 Pin  8  PA1          8  PD         J18 Pin  6  PC5
    9  NC        N/C                     10  AUDIO OUT  N/C
   11  GPIO0/IO2 N/C                     12  GPIO0/IO3  N/C
   13  GPIO2     N/C                     14  GPIO3      N/C
   15  NC        N/C                     16  NC         N/C
   17  BLVDD     N/C **                  18  BLVDD      N/C **
   19  BLGND     N/C **                  20  BLGND      N/C **

  2x10 Connector CN2 using SPI2:
  ---- --------- ----------- ----------- ---- --------- ----------- -----------
  PIN  NAME      VIEWTOOL    STM32       PIN  NAME      VIEWTOOL    STM32
  ---- --------- ----------- ---------- ---- --------- ----------- ------------
    1  VDD       J8  Pin  1 *             2  GND        J8  Pin  2
    3  SPI_CLK   J8  Pin  5  PB13/SCK2    4  MISO       J8  Pin  3  PB14/MISO2
    5  MOSI/IO1  J8  Pin  4  PB15/MOSI2   6  CS         J8  Pin  6  PB12/NSS2
    7  INT       J18 Pin  8  PA1          8  PD         J18 Pin  6  PC5
    9  NC        N/C                     10  AUDIO OUT  N/C
   11  GPIO0/IO2 N/C                     12  GPIO0/IO3  N/C
   13  GPIO2     N/C                     14  GPIO3      N/C
   15  NC        N/C                     16  NC         N/C
   17  BLVDD     N/C **                  18  BLVDD      N/C **
   19  BLGND     N/C **                  20  BLGND      N/C **

  *  0.0-4.0V
  ** May be connected to VDD, 0.0-7.0V

     I did not see a backlight without BLVDD or BLGND connected.  Possibly
     this depends on the 3.3V current provided by the board?  Obvious
     connections would be J18 pins 2 and 4.

  This display should have:

    CONFIG_LCD_FT801=y
    CONFIG_LCD_FT80X_SPI=y
    CONFIG_LCD_FT80X_WQVGA=y
    CONFIG_LCD_FT80X_AUDIO_NOSHUTDOWN=y
    CONFIG_EXAMPLES_FT80X_DEVPATH="/dev/ft801"

MAX3421E Integration
====================

  Board Connections
  -----------------

  USBHostShield-v13 (See schematic).

  DuinoFun UHS mini v2.0.  No schematics available.  This is how the pins
  are labeled:

     INT                                                 MAX_RST
      o     o     o     o     o     o     o     o     o     o     o     o
      o     o     o     o     o
    V_BUS  INT   GPX MAX_RST  SS

      o     o     o     o     o     o     o     o     o     o     o     o
      SS   CLK*  MISO  MOSI*                         VCC         GND**

  *  NOTE:  There is a error in the silkscreen:  The pin labeled CLK is
     actually MOSI; the pin labeled MOSI is the clock
  ** Not labeled

  Using SPI1 on J8 pins 7-12, discretes on J18

    ------ ----------- ----------- ------------------ ----------------------
    NAME   VIEWTOOL    STM32       USBHostShield-v13  DuinoFun UHS mini v2.0
    ------ ----------- ----------- ------------------ ----------------------
    CS#    J8  Pin 12  PA4/NSS1    D10                SS
    SCK    J8  Pin 11  PA5/SCK1    D13                CLK (label MOSI)
    MISO   J8  Pin  9  PA6/MISO1   D12                MISO
    MOSI   J8  Pin 10  PA7/MOSI1   D11                MOSI (label CLK)
    INT#   J18 Pin 10  PA0         D9                 INT
    RST#   J18 Pin  8  PA1         D7                 MAX_RST
    GPX    J18 Pin  6  PC5         D8                 GPX (not used)
    VBUS   J18 Pin  2  5V          VIN                V_BUS
    3.3V   J8  Pin  7              N/C                VCC
    GND    J8  Pin  8              GND                GND (no label)

  Using SPI2 on J8 pins 1-6, discretes on J18

    ------ ----------- ----------- ------------------ ----------------------
    NAME   VIEWTOOL    STM32       USBHostShield-v13 DuinoFun UHS mini v2.0
    ------ ----------- ----------- ------------------ ----------------------
    CS#    J8  Pin  6  PB12/NSS2   D10                SS
    SCK    J8  Pin  5  PB13/SCK2   D13                CLK (label MOSI)
    MISO   J8  Pin  3  PB14/MISO2  D12                MISO
    MOSI   J8  Pin  4  PB15/MOSI2  D11                MOSI (label CLK)
    INT#   J18 Pin 10  PA0         D9                 INT
    RST#   J18 Pin  8  PA1         D7                 MAX_RST
    GPX    J18 Pin  6  PC5         D8                 GPX (not used)
    VBUS   J18 Pin  2  5V          VIN                V_BUS
    3.3V   J8  Pin  1              N/C                VCC
    GND    J8  Pin  2              GND                GND (no label)

  5V VBUS power is also needed.  This might be directly connected to the USB
  host connector (as assumed here), or switched via additional logic.  Then
  GPX pin might also be necessary if VBUS detect is used with self-powered
  devices.

  Configuration Options
  ---------------------
  These options have to be added to the basic NSH configuration in order to
  support the MAX3421E:

    CONFIG_EXPERIMENTAL=y         # EXPERIMENTAL required for now (might change)
    CONFIG_NSH_ARCHINIT=y         # Board level initialization required
    CONFIG_STM32_SPI1=y           # SPI for the MAX3421E (could use SPI2)
    CONFIG_USBHOST=y              # General USB host support
    CONFIG_USBHOST_ISOC_DISABLE=y # Does not support Isochronous endpoints
    CONFIG_USBHOST_MAX3421E=y     # MAX3421E support
    CONFIG_USBHOST_MSC=y          # USB MSC class

  Using SPI1:

    CONFIG_VIEWTOOL_MAX3421E_SPI1=y
    CONFIG_VIEWTOOL_MAX3421E_FREQUENCY=20000000
    CONFIG_VIEWTOOL_MAX3421E_RST=y
    # CONFIG_VIEWTOOL_MAX3421E_PWR is not set
    CONFIG_VIEWTOOL_MAX3421E_CONNMON_STACKSIZE=2048
    CONFIG_VIEWTOOL_MAX3421E_CONNMON_PRIORITY=100

  Settings not listed above can be left at their default values.

Toolchains
==========

  NOTE about Windows native toolchains
  ------------------------------------

  There are several limitations to using a Windows based toolchain in a
  Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath'
     utility but you might easily find some new path problems.  If so, check
     out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic
     links are used in NuttX (e.g., include/arch).  The make system works
     around these problems for the Windows tools by copying directories
     instead of linking them.  But this can also cause some confusion for
     you:  For example, you may edit a file in a "linked" directory and find
     that your changes had no effect.  That is because you are building the
     copy of the file in the "fake" symbolic directory.  If you use a\
     Windows toolchain, you should get in the habit of making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each SAM3U-EK configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh viewtool-stm32f107:<subdir>

  Before starting the build, make sure that your PATH environment variable
  includes the correct path to your toolchain.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

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
     output on USART1.

  3. Unless otherwise stated, the configurations are setup for
     Cygwin under Windows:

     Build Setup:
       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under windows

  4. All of these configurations use the ARM EABI GCC toolchain for Windows
     (unless stated otherwise in the description of the configuration).  That
     toolchain selection can easily be reconfigured using 'make menuconfig'.
     Here are the relevant current settings:

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y     : GNU EABI toolchain for Windows

     See also the "NOTE about Windows native toolchains" in the section call
     "GNU Toolchain Options" above.

  4. These configurations all assume that the STM32F107VCT6 is mounted on
     board.  This is configurable; you can select the STM32F103VCT6 as an
     alternative.

  5. These configurations all assume that you are loading code using
     something like the ST-Link v2 JTAG.  None of these configurations are
     setup to use the DFU bootloader but should be easily reconfigured to
     use that bootloader if so desired.

  Configuration Sub-directories
  -----------------------------

  f80x:

    This configuration was added in order to verify the FTDI/Bridgetick
    Ft80x driver using apps/examples/ft80x with apps/graphics/ft80x.  It
    is very similar to the NSH configuration with support for the FTDI
    FT80x LCD enabled on SPI1.

    This configuration is properly setup for the MikroElektronika
    ConnectEVE LCD.  To use the Reverdi FT801 LCD, the following changes
    would be required to the configuration:

      -CONFIG_LCD_FT800=y
      +CONFIG_LCD_FT801=y

      -CONFIG_LCD_FT80X_AUDIO_GPIOSHUTDOWN=y
      -CONFIG_LCD_FT80X_AUDIO_GPIO=0
      +CONFIG_LCD_FT80X_AUDIO_NOSHUTDOWN=y

      -CONFIG_EXAMPLES_FT80X_DEVPATH="/dev/ft800"
      +CONFIG_EXAMPLES_FT80X_DEVPATH="/dev/ft801"

    STATUS:
    2018-03-09:  The ConnectEVE display is basically working.  There are
      some specific issues with some of the demos in apps/examples/ft80x
      that still need to be addressed.  I have the Riverdi display FT801
      display in hand as well, but have not tested with the display yet.

      I have seen issues also where the board does not recover after a
      reset.  It required a full power cycle to get functionality back.
      This is not too surprising since there is no reset signal to the
      FT80x (there is power down/up).  It might be necessary to perform
      a software reset of the FT80x during initialization.

    1028-03-10:  Most of issues have been worked out in the FT80x demos
      and the driver appears 100% functional.

  netnsh:

    This configuration directory provide the NuttShell (NSH) with
    networking support.

    NOTES:
    1. This configuration will work only on the version the viewtool
       board with the STM32F107VCT6 installed.  If you have a board
       with the STM32F103VCT6 installed, please use the nsh configuration
       described below.

    2. There is no PHY on the base viewtool stm32f107 board.  You must
       also have the "ViewTool DP83848 Ethernet Module" installed on J2
       in order to support networking.

    3. Since networking is enabled, you will see some boot-up delays when
       the network connection is established.  These delays can be quite
       large if no network is attached (A production design to bring up the
       network asynchronously to avoid these start up delays).

    4. This configuration uses the default USART1 serial console.  That
       is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as
       the console device.

    5. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain (such as the CodeSourcery
       toolchain).  Both the build environment and the toolchain
       selection can easily be changed by reconfiguring:

       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under Windows
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y     : GNU EABI toolchain for Windows

    6. USB support is disabled by default.  See the section above entitled,
       "USB Interface"

  nsh:

    This configuration directory provide the basic NuttShell (NSH).

    NOTES:
    1. This configuration will work with either the version of the board
       with STM32F107VCT6 or STM32F103VCT6 installed.  The default
       configuration is for the STM32F107VCT6.  To use this configuration
       with a STM32F103VCT6, it would have to be modified as follows:

      System Type -> STM32 Configuration Options
         CONFIG_ARCH_CHIP_STM32F103VC=y
         CONFIG_ARCH_CHIP_STM32F107VC=n

    2. This configuration uses the default USART1 serial console.  That
       is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as
       the console device.

    3. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain (such as the CodeSourcery
       toolchain).  Both the build environment and the toolchain
       selection can easily be changed by reconfiguring:

       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under Windows
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y     : GNU EABI toolchain for Windows

    4. USB support is disabled by default.  See the section above entitled,
       "USB Interface"

    3. This configured can be re-configured to use either the Viewtool LCD
       module. NOTE:  The LCD module can only be used on the STM32F103 version
       of the board.  The LCD requires FSMC support.

          System Type -> STM32 Chip Selection:
            CONFIG_ARCH_CHIP_STM32F103VC=y      : Select STM32F103VCT6

          System Type -> Peripherals:
            CONFIG_STM32_FSMC=y                   : Enable FSMC LCD interface

          Device Drivers -> LCD Driver Support
            CONFIG_LCD=y                          : Enable LCD support
            CONFIG_NX_LCDDRIVER=y                 : LCD graphics device
            CONFIG_LCD_MAXCONTRAST=1
            CONFIG_LCD_MAXPOWER=255
            CONFIG_LCD_LANDSCAPE=y                : Landscape orientation
            CONFIG_LCD_SSD1289=y                  : Select the SSD1289
            CONFIG_SSD1289_PROFILE1=y

          Graphics Support
            CONFIG_NX=y

          Graphics Support -> Supported Pixel Depths
            CONFIG_NX_DISABLE_1BPP=y              : Only 16BPP supported
            CONFIG_NX_DISABLE_2BPP=y
            CONFIG_NX_DISABLE_4BPP=y
            CONFIG_NX_DISABLE_8BPP=y
            CONFIG_NX_DISABLE_24BPP=y
            CONFIG_NX_DISABLE_32BPP=y

          Graphics Support -> Font Selections
            CONFIG_NXFONTS_CHARBITS=7
            CONFIG_NXFONT_SANS22X29B=y
            CONFIG_NXFONT_SANS23X27=y

          Application Configuration -> Examples
            CONFIG_EXAMPLES_NXLINES=y
            CONFIG_EXAMPLES_NXLINES_BGCOLOR=0x0320
            CONFIG_EXAMPLES_NXLINES_LINEWIDTH=16
            CONFIG_EXAMPLES_NXLINES_LINECOLOR=0xffe0
            CONFIG_EXAMPLES_NXLINES_BORDERWIDTH=4
            CONFIG_EXAMPLES_NXLINES_BORDERCOLOR=0xffe0
            CONFIG_EXAMPLES_NXLINES_CIRCLECOLOR=0xf7bb
            CONFIG_EXAMPLES_NXLINES_BPP=16

       STATUS: Not working; reads 0x8999 as device ID.  This may perhaps
               be due to incorrect jumper settings

    6. This configuration has been used for verifying the touchscreen on
       on the Viewtool LCD module.  NOTE:  The LCD module can really only
       be used on the STM32F103 version of the board.  The LCD requires
       FSMC support (the touchscreen, however, does not but the touchscreen
       is not very meaningful with no LCD).

          System Type -> STM32 Chip Selection:
           CONFIG_ARCH_CHIP_STM32F103VC=y    : Select STM32F103VCT6

       With the following modifications, you can include the touchscreen
       test program at apps/examples/touchscreen as an NSH built-in
       application.  You can enable the touchscreen and test by modifying
       the default configuration in the following ways:

          Device Drivers
            CONFIG_SPI=y                       : Enable SPI support
            CONFIG_SPI_EXCHANGE=y              : The exchange() method is supported

            CONFIG_INPUT=y                     : Enable support for input devices
            CONFIG_INPUT_ADS7843E=y            : Enable support for the XPT2046
            CONFIG_ADS7843E_SPIDEV=2           : Use SPI2 for communication
            CONFIG_ADS7843E_SPIMODE=0          : Use SPI mode 0
            CONFIG_ADS7843E_FREQUENCY=1000000  : SPI BAUD 1MHz
            CONFIG_ADS7843E_SWAPXY=y           : If landscape orientation
            CONFIG_ADS7843E_THRESHX=51         : These will probably need to be tuned
            CONFIG_ADS7843E_THRESHY=39

          System Type -> Peripherals:
            CONFIG_STM32_SPI2=y                : Enable support for SPI2

          Library Support:
            CONFIG_SCHED_WORKQUEUE=y           : Work queue support required

          Application Configuration:
            CONFIG_EXAMPLES_TOUCHSCREEN=y      : Enable the touchscreen built-int test

          Defaults should be okay for related touchscreen settings.  Touchscreen
          debug output on USART1 can be enabled with:

          Build Setup:
            CONFIG_DEBUG_FEATURES=y            : Enable debug features
            CONFIG_DEBUG_INFO=y                : Enable verbose debug output
            CONFIG_DEBUG_INPUT=y               : Enable debug output from input devices

       STATUS: Working

  highpri:

    This configuration was used to verify the NuttX high priority, nested
    interrupt feature.  This is a board-specific test and probably not
    of much interest now other than for reference.

    This configuration targets the viewtool board with the STM32F103VCT6

  tcpblaster:

    The tcpblaster example derives from the nettest example and basically
    duplicates that application when the nettest PERFORMANCE option is selected.
    tcpblaster has a little better reporting of performance stats, however.

    This configuration derives directly from the netnsh configuration and most
    of the notes there should apply equally here.

    General usage instructions:

    1. On the host:
       a. cd to apps/examples/tcpblaster
       b. Run the host tcpserver[.exe] program that was built in that directory
    2. On the target:
       a. Run the tcpclient built in application.
    3. When you get tire of watch the numbers scroll by, just kill the tcpserver
       on the host.
