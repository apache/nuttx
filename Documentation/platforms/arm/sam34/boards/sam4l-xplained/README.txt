README
^^^^^^

This README discusses issues unique to NuttX configurations for the
Atmel SAM4L Xplained Pro development board.  This board features the
ATSAM4LC4C MCU.

The SAM4L Xplained Pro Starter Kit is bundled with four modules:

1) I/O1   - An MMC/SD card slot, PWM LED control, ADC light sensor, UART
            loopback, TWI AT30TSE758 Temperature sensor.
2) OLED1  - An OLED plus 3 additional switches and 3 additional LEDs
3) SLCD1  - A segment LCD that connects directly to the "EXT5 SEGMENT LCD"
           connector
4) PROTO1 - A prototyping board with logic on board (other than power-related
            logic).

Contents
^^^^^^^^

  - Modules
  - LEDs
  - Serial Consoles
  - SAM4L Xplained Pro-specific Configuration Options
  - Configurations

Modules
^^^^^^^
  The SAM4L Xplained Pro Starter Kit is bundled with four modules:

  I/O1
  ----
    The primary function of this module is to provide SD card support, but
    the full list of modules features include:

    - microSD card connector (SPI interface)
    - PWM (LED control)
    - ADC (light sensor)
    - UART loopback
    - TWI AT30TSE758 Temperature sensor with EEPROM

    SPI is available on two of the SAM4L Xplained connectors, EXT1 and EXT2.
    They mate with the I/O1 connector as indicated in this table.

    I/O1 Connector
    --------------
    I/O1              EXT1                 EXT2                 Other use of either pin
    ----------------- -------------------- -------------------- ------------------------------------
    1  ID             1                    1
    2  GND            2       GND          2
    3  LIGHT_SENSOR   3  PA04 ADCIFE/AD0   3  PA07 ADCIFE/AD2
    4  LP_OUT         4  PA05 ADCIFE/AD1   4  PB02 ADCIFE/AD3
    5  GPIO1          5  PB12 GPIO         5  PC08 GPIO         PB12 and PC8 on EXT5
    6  GPIO2          6  PC02 GPIO         6  PB10 GPIO         PB10 on EXT5
    7  LED            7  PC00 TC/1/A0      7  PC04 TC/1/A2
    8  LP_IN          8  PC01 TC/1/B0      8  PC05 TC/1/B2      PC05 on EXT5
    9  TEMP_ALERT     9  PC25 EIC/EXTINT2  9  PC06 EIC/EXTINT8  PC25 on EXT5
    10 microSD_DETECT 10 PB13 SPI/NPCS1    10 PC09 GPIO         PB13 on EXT5
    11 TWI SDA        11 PA23 TWIMS/0/TWD  11 PB14 TWIMS/3/TWD  PB14 on EXT3&4, PA23 and PB14 on EXT5
    12 TWI SCL        12 PA24 TWIMS/0/TWCK 12 PB15 TWIMS/3/TWCK PB15 on EXT3&4, PA24 and PB15 on EXT5
    13 UART RX        13 PB00 USART/0/RXD  13 PC26 USART/1/RXD  PB00 on EXT4, PC26 on EXT3&5
    14 UART TX        14 PB01 USART/0/TXD  14 PC27 USART/1/TXD  PB01 on EXT4, PC27 on EXT3&5
    15 microSD_SS     15 PC03 SPI/NPCS0    15 PB11 SPI/NPCS2    PB11 on EXT5
    16 SPI_MOSI       16 PA22 SPI/MOSI     16 PA22 SPI/MOSI     PA22 on EXT5
    17 SPI_MISO       17 PA21 SPI/MISO     17 PA21 SPI/MISO     PA21 on EXT5
    18 SPI_SCK        18 PC30 SPI/SCK      18 PC30 SPI/SCK      PC30 on EXT5
    19 GND            19      GND             GND
    20 VCC            20      VCC             VCC

    The mapping between the I/O1 pins and the SD connector are shown in the
    following table.

    SD Card Connection
    ------------------
    I/O1 SD   PIN Description
    ---- ---- --- -------------------------------------------------
         D2   1   Data line 2 (not used)
    15   D3   2   Data line 3. Active low chip select, pulled high
    16   CMD  3   Command line, connected to SPI_MOSI.
    20   VDD  4
    18   CLK  5   Clock line, connected to SPI_SCK.
    2/19 GND  6
    17   D0   7   Data line 0, connected to SPI_MISO.
         D1   8   Data line 1 (not used)
    10   SW_A 9   Card detect
    2/19 SW_B 10  GND

    Card Detect
    -----------
    When a microSD card is put into the connector SW_A and SW_B are short-
    circuited. SW_A is connected to the microSD_DETECT signal. To use this
    as a card indicator remember to enable internal pullup in the target
    device.

    GPIOs
    -----
    So all that is required to connect the SD is configure the SPI

    PIN EXT1           EXT2            Description
    --- -------------- --------------- -------------------------------------
    15  PC03 SPI/NPCS0 PB11 SPI/NPCS2  Active low chip select OUTPUT, pulled
                                       high on board.
    10  PB13 SPI/NPCS1 10 PC09 GPIO    Active low card detect INPUT, must
                                       use internal pull-up.

    Configuration Options:
    ----------------------
      CONFIG_SAM4L_XPLAINED_IOMODULE=y      : Informs the system that the
                                              I/O1 module is installed
      CONFIG_SAM4L_XPLAINED_IOMODULE_EXT1=y : The module is installed in EXT1
      CONFIG_SAM4L_XPLAINED_IOMODULE_EXT2=y : The mdoule is installed in EXT2

    See the set-up in the discussion of the nsh configuration below for other
    required configuration options.

    NOTE: As of this writing, only the SD card slot is supported in the I/O1
    module.

  OLED1
  -----
    This module provides an OLED plus 3 additional switches and 3 additional
    LEDs.

    OLED1 Connector
    --------------
    OLED1             EXT1                 EXT2                 Other use of either pin
    ----------------- -------------------- -------------------- ------------------------------------
    1  ID             1                    1
    2  GND            2       GND          2
    3  BUTTON2        3  PA04 ADCIFE/AD0   3  PA07 ADCIFE/AD2
    4  BUTTON3        4  PA05 ADCIFE/AD1   4  PB02 ADCIFE/AD3
    5  DATA_CMD_SEL   5  PB12 GPIO         5  PC08 GPIO         PB12 and PC8 on EXT5
    6  LED3           6  PC02 GPIO         6  PB10 GPIO         PB10 on EXT5
    7  LED1           7  PC00 TC/1/A0      7  PC04 TC/1/A2
    8  LED2           8  PC01 TC/1/B0      8  PC05 TC/1/B2      PC05 on EXT5
    9  BUTTON1        9  PC25 EIC/EXTINT2  9  PC06 EIC/EXTINT8  PC25 on EXT5
    10 DISPLAY_RESET  10 PB13 SPI/NPCS1    10 PC09 GPIO         PB13 on EXT5
    11 N/C            11 PA23 TWIMS/0/TWD  11 PB14 TWIMS/3/TWD  PB14 on EXT3&4, PA23 and PB14 on EXT5
    12 N/C            12 PA24 TWIMS/0/TWCK 12 PB15 TWIMS/3/TWCK PB15 on EXT3&4, PA24 and PB15 on EXT5
    13 N/C            13 PB00 USART/0/RXD  13 PC26 USART/1/RXD  PB00 on EXT4, PC26 on EXT3&5
    14 N/C            14 PB01 USART/0/TXD  14 PC27 USART/1/TXD  PB01 on EXT4, PC27 on EXT3&5
    15 DISPLAY_SS     15 PC03 SPI/NPCS0    15 PB11 SPI/NPCS2    PB11 on EXT5
    16 SPI_MOSI       16 PA22 SPI/MOSI     16 PA22 SPI/MOSI     PA22 on EXT5
    17 N/C            17 PA21 SPI/MISO     17 PA21 SPI/MISO     PA21 on EXT5
    18 SPI_SCK        18 PC30 SPI/SCK      18 PC30 SPI/SCK      PC30 on EXT5
    19 GND            19      GND             GND
    20 VCC            20      VCC             VCC

    Configuration Options:
    ----------------------
      CONFIG_SAM4L_XPLAINED_OLED1MODULE=y      : Informs the system that the
                                                 I/O1 module is installed
      CONFIG_SAM4L_XPLAINED_OLED1MODULE_EXT1=y : The module is installed in EXT1
      CONFIG_SAM4L_XPLAINED_OLED1MODULE_EXT2=y : The mdoule is installed in EXT2

    See the set-up in the discussion of the nsh configuration below for other
    required configuration options.

  SLCD1
  -----
    This module provides a A segment LCD that connects directly to the "EXT5
    SEGMENT LCD" connector

    Configuration Options:
    ----------------------
      CONFIG_SAM4L_XPLAINED_SLCD1MODULE=y   : Informs the system that the
                                              I/O1 module is installed

    See the set-up in the discussion of the nsh configuration below for other
    required configuration options.

  PROTO1
  ------
  A prototyping board with logic on board (other than power-related logic).
  There is no built-in support for the PROTO1 module.

LEDs
^^^^
  There are three LEDs on board the SAM4L Xplained Pro board:  The EDBG
  controls two of the LEDs, a power LED and a status LED.  There is only
  one user controllable LED, a yellow LED labeled LED0 near the SAM4L USB
  connector.

  This LED is controlled by PC07 and LED0 can be activated by driving the
  PC07 to GND.

  When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
  control LED0 as follows:

    SYMBOL              Meaning                 LED0
    ------------------- ----------------------- ------
    LED_STARTED         NuttX has been started  OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF
    LED_IRQSENABLED     Interrupts enabled      OFF
    LED_STACKCREATED    Idle stack created      ON
    LED_INIRQ           In an interrupt         N/C
    LED_SIGNAL          In a signal handler     N/C
    LED_ASSERTION       An assertion failed     N/C
    LED_PANIC           The system has crashed  FLASH

  Thus is LED0 is statically on, NuttX has successfully  booted and is,
  apparently, running normmally.  If LED0 is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

Serial Consoles
^^^^^^^^^^^^^^^

  USART0
  ------

  USART0 is available on connectors EXT1 and EXT4

    EXT1  EXT4  GPIO  Function
    ----  ---- ------ -----------
     13    13   PB00  USART0_RXD
     14    14   PB01  USART0_TXD
     19    19         GND
     20    20         VCC

  If you have a TTL to RS-232 converter then this is the most convenient
  serial console to use.  It is the default in all of these configurations.
  An option is to use the virtual COM port.

  Virtual COM Port
  ----------------

  The SAM4L Xplained Pro contains an Embedded Debugger (EDBG) that can be
  used to program and debug the ATSAM4LC4C using Serial Wire Debug (SWD).
  The Embedded debugger also include a Virtual Com port interface over
  USART1.  Virtual COM port connections:

    PC26 USART1 RXD
    PC27 USART1 TXD

SAM4L Xplained Pro-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM4=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP="sam34"

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_SAM34
       CONFIG_ARCH_CHIP_SAM4L
       CONFIG_ARCH_CHIP_ATSAM4LC4C

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=sam4l-xplained (for the SAM4L Xplained Pro development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_SAM4L_XPLAINED=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=0x00008000 (32Kb)

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

    CPU
    ---
    CONFIG_SAM34_OCD

    HSB
    ---
    CONFIG_SAM34_APBA
    CONFIG_SAM34_AESA

    PBA
    ---
    CONFIG_SAM34_IISC
    CONFIG_SAM34_SPI0
    CONFIG_SAM34_TC0
    CONFIG_SAM34_TC1
    CONFIG_SAM34_TWIM0
    CONFIG_SAM34_TWIS0
    CONFIG_SAM34_TWIM1
    CONFIG_SAM34_TWIS1
    CONFIG_SAM34_USART0
    CONFIG_SAM34_USART1
    CONFIG_SAM34_USART2
    CONFIG_SAM34_USART3
    CONFIG_SAM34_ADC12B
    CONFIG_SAM34_DACC
    CONFIG_SAM34_ACC
    CONFIG_SAM34_GLOC
    CONFIG_SAM34_ABDACB
    CONFIG_SAM34_TRNG
    CONFIG_SAM34_PARC
    CONFIG_SAM34_CATB
    CONFIG_SAM34_TWIM2
    CONFIG_SAM34_TWIM3
    CONFIG_SAM34_LCDCA

    PBB
    ---
    CONFIG_SAM34_HRAMC1
    CONFIG_SAM34_HMATRIX
    CONFIG_SAM34_PDCA
    CONFIG_SAM34_CRCCU
    CONFIG_SAM34_USBC
    CONFIG_SAM34_PEVC

    PBC
    ---
    CONFIG_SAM34_CHIPID
    CONFIG_SAM34_FREQM

    PBD
    ---
    CONFIG_SAM34_AST
    CONFIG_SAM34_WDT
    CONFIG_SAM34_EIC
    CONFIG_SAM34_PICOUART

  Some subsystems can be configured to operate in different ways. The drivers
  need to know how to configure the subsystem.

    CONFIG_SAM34_GPIOA_IRQ
    CONFIG_SAM34_GPIOB_IRQ
    CONFIG_SAM34_GPIOC_IRQ
    CONFIG_USART0_SERIALDRIVER
    CONFIG_USART1_SERIALDRIVER
    CONFIG_USART2_SERIALDRIVER
    CONFIG_USART3_SERIALDRIVER

  ST91SAM4L specific device driver settings

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

Configurations
^^^^^^^^^^^^^^

  Each SAM4L Xplained Pro configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh sam4l-xplained:<subdir>

  Before building, make sure the PATH environment variable includes the
  correct path to the directory than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTE:  These configurations use the mconf-based configuration tool.  To
  change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output of on USART0 which is available on EXT1 or EXT4 (see the
     section "Serial Consoles" above).  The virtual COM port could
     be used, instead, by reconfiguring to use USART1 instead of
     USART0:

       System Type -> AT91SAM3/4 Peripheral Support
         CONFIG_SAM_USART0=y
         CONFIG_SAM_USART1=n

       Device Drivers -> Serial Driver Support -> Serial Console
         CONFIG_USART0_SERIAL_CONSOLE=y

       Device Drivers -> Serial Driver Support -> USART0 Configuration
         CONFIG_USART0_2STOP=0
         CONFIG_USART0_BAUD=115200
         CONFIG_USART0_BITS=8
         CONFIG_USART0_PARITY=0
         CONFIG_USART0_RXBUFSIZE=256
         CONFIG_USART0_TXBUFSIZE=256

  3. Unless otherwise stated, the configurations are setup for
     Linux (or any other POSIX environment like Cygwin under Windows):

     Build Setup:
       CONFIG_HOST_LINUX=y   : Linux or other POSIX environment

  4. These configurations use the older, OABI, buildroot toolchain.  But
     that is easily reconfigured:

     System Type -> Toolchain:
       CONFIG_ARM_TOOLCHAIN_BUILDROOT=y : Buildroot toolchain
       CONFIG_ARM_TOOLCHAIN_BUILDROOT_OABI=y      : Older, OABI toolchain

     If you want to use the Atmel GCC toolchain, here are the steps to
     do so:

     Build Setup:
       CONFIG_HOST_WINDOWS=y   : Windows
       CONFIG_HOST_CYGWIN=y    : Using Cygwin or other POSIX environment

     System Type -> Toolchain:
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y  : General GCC EABI toolchain under windows

     This re-configuration should be done before making NuttX or else the
     subsequent 'make' will fail.  If you have already attempted building
     NuttX then you will have to 1) 'make distclean' to remove the old
     configuration, 2) 'tools/configure.sh sam3u-ek/ksnh' to start
     with a fresh configuration, and 3) perform the configuration changes
     above.

     Also, make sure that your PATH variable has the new path to your
     Atmel tools.  Try 'which arm-none-eabi-gcc' to make sure that you
     are selecting the right tool.

     See also the "NOTE about Windows native toolchains" in the section call
     "GNU Toolchain Options" above.

Configuration sub-directories
-----------------------------

  nsh:
    This configuration directory will built the NuttShell.  See NOTES above
    and below:

    NOTES:

    1. NOTE: If you get a compilation error like:

         libxx_new.cxx:74:40: error: 'operator new' takes type 'size_t'
                              ('unsigned int') as first parameter [-fper

       Sometimes NuttX and your toolchain will disagree on the underlying
       type of size_t; sometimes it is an 'unsigned int' and sometimes it is
       an 'unsigned long int'.  If this error occurs, then you may need to
       toggle the value of CONFIG_ARCH_SIZET_LONG.

    2. If the I/O1 module is connected to the SAM4L Xplained Pro, then
       support for the SD card slot can be enabled by making the following
       changes to the configuration:

       File Systems:
         CONFIG_FS_FAT=y                   : Enable the FAT file system
         CONFIG_FAT_LCNAMES=y              : Enable upper/lower case 8.3 file names (Optional, see below)
         CONFIG_FAT_LFN=y                  : Enable long file named (Optional, see below)
         CONFIG_FAT_MAXFNAME=32            : Maximum supported file name length

         There are issues related to patents that Microsoft holds on FAT long
         file name technologies.  See the top level NOTICE file for further
         details.

       System Type -> Peripherals:
         CONFIG_SAM34_SPI0=y                : Enable the SAM4L SPI peripheral

       Device Drivers
         CONFIG_SPI=y                      : Enable SPI support
         CONFIG_SPI_EXCHANGE=y             : The exchange() method is supported

         CONFIG_MMCSD=y                    : Enable MMC/SD support
         CONFIG_MMCSD_NSLOTS=1             : Only one MMC/SD card slot
         CONFIG_MMCSD_MULTIBLOCK_LIMIT=0   : Should not need to disable multi-block transfers
         CONFIG_MMCSD_HAVE_CARDDETECT=y    : I/O1 module as a card detect GPIO
         CONFIG_MMCSD_SPI=y                : Use the SPI interface to the MMC/SD card
         CONFIG_MMCSD_SPICLOCK=20000000    : This is a guess for the optimal MMC/SD frequency
         CONFIG_MMCSD_SPIMODE=0            : Mode 0 is required

       Board Selection -> Common Board Options
         CONFIG_NSH_MMCSDSLOTNO=0          : Only one MMC/SD slot, slot 0
         CONFIG_NSH_MMCSDSPIPORTNO=0       : Use CS=0 if the I/O1 is in EXT1, OR
         CONFIG_NSH_MMCSDSPIPORTNO=2       : Use CS=2 if the I/O1 is in EXT2

       Board Selection -> SAM4L Xplained Pro Modules
         CONFIG_SAM4L_XPLAINED_IOMODULE=y      : I/O1 module is connected
         CONFIG_SAM4L_XPLAINED_IOMODULE_EXT1=y : In EXT1, or EXT2
         CONFIG_SAM4L_XPLAINED_IOMODULE_EXT2=y

       Application Configuration -> NSH Library
         CONFIG_NSH_ARCHINIT=y             : Board has architecture-specific initialization

       NOTE: If you enable the I/O1 this configuration with USART0 as the
       console and with the I/O1 module in EXT1, you *must* remove UART
       jumper.  Otherwise, you have lookpack on USART0 and NSH will *not*
       behave very well (since its outgoing prompts also appear as incoming
       commands).

       STATUS:  As of 2013-6-18, this configuration appears completely
       functional.  Testing, however, has been very light.  Example:

         NuttShell (NSH) NuttX-6.28
         nsh> mount -t vfat /dev/mmcsd0 /mnt/stuff
         nsh> ls /mnt/stuff
         /mnt/stuff:
         nsh> echo "This is a test" >/mnt/stuff/atest.txt
         nsh> ls /mnt/stuff
         /mnt/stuff:
          atest.txt
         nsh> cat /mnt/stuff/atest.txt
         This is a test
         nsh>

    3. If the OLED1 module is connected to the SAM4L Xplained Pro, then
       support for the OLED display can be enabled by making the following
       changes to the configuration:

       System Type -> Peripherals:
         CONFIG_SAM34_SPI0=y                 : Enable the SAM4L SPI peripheral

       Device Drivers -> SPI
         CONFIG_SPI=y                       : Enable SPI support
         CONFIG_SPI_EXCHANGE=y              : The exchange() method is supported
         CONFIG_SPI_CMDDATA=y               : CMD/DATA support is required

       Device Drivers -> LCDs
         CONFIG_LCD=y                       : Enable LCD support
         CONFIG_LCD_MAXCONTRAST=255         : Maximum contrast value
         CONFIG_LCD_LANDSCAPE=y             : Landscape orientation (see below*)
         CONFIG_LCD_UG2832HSWEG04=y         : Enable support for the OLED
         CONFIG_LCD_SSD1306_SPIMODE=0       : SPI Mode 0
         CONFIG_LCD_SSD1306_SPIMODE=3500000 : Pick an SPI frequency

       Board Selection -> SAM4L Xplained Pro Modules
         CONFIG_SAM4L_XPLAINED_OLED1MODULE=y      : OLED1 module is connected
         CONFIG_SAM4L_XPLAINED_OLED1MODULE_EXT1=y : In EXT1, or EXT2
         CONFIG_SAM4L_XPLAINED_OLED1MODULE_EXT2=y

       The NX graphics subsystem also needs to be configured:

         CONFIG_NX=y                        : Enable graphics support
         CONFIG_NX_LCDDRIVER=y              : Using an LCD driver
         CONFIG_NX_NPLANES=1                : With a single color plane
         CONFIG_NX_WRITEONLY=n              : You can read from the LCD (see below**)
         CONFIG_NX_DISABLE_2BPP=y           : Disable all resolutions except 1BPP
         CONFIG_NX_DISABLE_4BPP=y
         CONFIG_NX_DISABLE_8BPP=y
         CONFIG_NX_DISABLE_16BPP=y
         CONFIG_NX_DISABLE_24BPP=y
         CONFIG_NX_DISABLE_32BPP=y
         CONFIG_NX_PACKEDMSFIRST=y          : LSB packed first (shouldn't matter)
         CONFIG_NXSTART_EXTERNINIT=y        : We have board_graphics_setup()
         CONFIG_NXTK_BORDERWIDTH=2          : Use a small border
         CONFIG_NXTK_DEFAULT_BORDERCOLORS=y : Default border colors
         CONFIG_NXFONTS_CHARBITS=7          : 7-bit fonts
         CONFIG_NXFONT_SANS17X23B=y         : Pick a font (any that will fit)

        * This orientation will put the buttons "above" the LCD.  The
          reverse landscape configuration (CONFIG_LCD_RLANDSCAPE) will
          "flip" the display so that the buttons are "below" the LCD.

       ** The hardware is write only, but the driver maintains a frame buffer
          to support read and read-write-modiry operations on the LCD.
          Reading from the frame buffer is, however, untested.

       Then, in order to use the OLED, you will need to build some kind of
       graphics application or use one of the NuttX graphics examples.
       Here, for example, is the setup for the graphic "Hello, World!"
       example:

         CONFIG_EXAMPLES_NXHELLO=y                : Enables the example
         CONFIG_EXAMPLES_NXHELLO_DEFAULT_COLORS=y : Use default colors (see below *)
         CONFIG_EXAMPLES_NXHELLO_DEFAULT_FONT=y   : Use the default font
         CONFIG_EXAMPLES_NXHELLO_BPP=1            : One bit per pixel
         CONFIG_EXAMPLES_NXHELLO_EXTERNINIT=y     : Special initialization is required.

        * The OLED is monochrome so the only "colors" are black and white.
          The default "colors" will give you while text on a black background.
          You can override the faults it you want black text on a while background.

       NOTE:  One issue that I have seen with the NXHello example when
       running as an NSH command is that it only works the first time.
       So, after you run the 'nxhello' command one time, you will have to
       reset the board before you run it again.

       This is clearly some issue with initializing, un-initializing, and
       then re-initializing. If you want to fix this, patches are quite
       welcome.

    4. If the LCD1 module is connected to the SAM4L Xplained Pro, then
       support for the SLCDt can be enabled by making the following
       changes to the configuration:

       System Type -> AT91SAM3/4 Peripheral Support
         CONFIG_SAM34_LCDCA=y

       System Type -> AT91SAM3/4 Clock Configuration
         CONFIG_SAM34_OSC32K=y

       Board Selection -> Board-Specific Options -> SAM4L Xplained Pro Modules
         CONFIG_SAM4L_XPLAINED_SLCD1MODULE=y

       Device Drivers
         CONFIG_LCD=y
         CONFIG_LCD_MAXCONTRAST=63

       Library Routines -> Non-standard Library Support
         CONFIG_LIBC_SLCDCODEC=y

       The SLCD example can be enabled to verify the SLCD:

       Application Configuration -> Examples
         CONFIG_EXAMPLES_SLCD=y
         CONFIG_EXAMPLES_SLCD_DEVNAME="/dev/slcd0"
         CONFIG_EXAMPLES_SLCD_BUFSIZE=64

       Application Configuration -> NSH Library
         CONFIG_NSH_ARCHINIT=y

       NOTE:  In order to use the segment LCD you *must* open the VLCD_A and
       VLCD_BC jumpers or the SLD will not be powered!
