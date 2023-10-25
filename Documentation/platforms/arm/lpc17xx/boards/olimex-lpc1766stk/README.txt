README
^^^^^^

README for NuttX port to the Olimex LPC1766-STK development board

Contents
^^^^^^^^

  Olimex LPC1766-STK development board
  LEDs
  Serial Console
  Using OpenOCD and GDB with an FT2232 JTAG emulator
  Olimex LPC1766-STK Configuration Options
  USB Host Configuration
  Configurations

Olimex LPC1766-STK development board
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  GPIO Usage:
  -----------

  GPIO                             PIN  SIGNAL NAME
  -------------------------------- ---- --------------
  P0[0]/RD1/TXD3/SDA1               46  RD1
  P0[1]/TD1/RXD3/SCL1               47  TD1
  P0[2]/TXD0/AD0[7]                 98  TXD0
  P0[3]/RXD0/AD0[6]                 99  RXD0
  P0[4]/I2SRX_CLK/RD2/CAP2[0]       81  LED2/ACC IRQ
  P0[5]/I2SRX_WS/TD2/CAP2[1]        80  CENTER
  P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     79  SSEL1
  P0[7]/I2STX_CLK/SCK1/MAT2[1]      78  SCK1
  P0[8]/I2STX_WS/MISO1/MAT2[2]      77  MISO1
  P0[9]/I2STX_SDA/MOSI1/MAT2[3]     76  MOSI1
  P0[10]/TXD2/SDA2/MAT3[0]          48  SDA2
  P0[11]/RXD2/SCL2/MAT3[1]          49  SCL2
  P0[15]/TXD1/SCK0/SCK              62  TXD1
  P0[16]/RXD1/SSEL0/SSEL            63  RXD1
  P0[17]/CTS1/MISO0/MISO            61  CTS1
  P0[18]/DCD1/MOSI0/MOSI            60  DCD1
  P0[19]/DSR1/SDA1                  59  DSR1
  P0[20]/DTR1/SCL1                  58  DTR1
  P0[21]/RI1/RD1                    57  MMC PWR
  P0[22]/RTS1/TD1                   56  RTS1
  P0[23]/AD0[0]/I2SRX_CLK/CAP3[0]    9  BUT1
  P0[24]/AD0[1]/I2SRX_WS/CAP3[1]     8  TEMP
  P0[25]/AD0[2]/I2SRX_SDA/TXD3       7  MIC IN
  P0[26]/AD0[3]/AOUT/RXD3            6  AOUT
  P0[27]/SDA0/USB_SDA               25  USB_SDA
  P0[28]/SCL0/USB_SCL               24  USB_SCL
  P0[29]/USB_D+                     29  USB_D+
  P0[30]/USB_D-                     30  USB_D-
  P1[0]/ENET_TXD0                   95  E_TXD0
  P1[1]/ENET_TXD1                   94  E_TXD1
  P1[4]/ENET_TX_EN                  93  E_TX_EN
  P1[8]/ENET_CRS                    92  E_CRS
  P1[9]/ENET_RXD0                   91  E_RXD0
  P1[10]/ENET_RXD1                  90  E_RXD1
  P1[14]/ENET_RX_ER                 89  E_RX_ER
  P1[15]/ENET_REF_CLK               88  E_REF_CLK
  P1[16]/ENET_MDC                   87  E_MDC
  P1[17]/ENET_MDIO                  86  E_MDIO
  P1[18]/USB_UP_LED/PWM1[1]/CAP1[0] 32  USB_UP_LED
  P1[19]/MC0A/#USB_PPWR/CAP1[1]     33  #USB_PPWR
  P1[20]/MCFB0/PWM1[2]/SCK0         34  SCK0
  P1[21]/MCABORT/PWM1[3]/SSEL0      35  SSEL0
  P1[22]/MC0B/USB_PWRD/MAT1[0]      36  USBH_PWRD
  P1[23]/MCFB1/PWM1[4]/MISO0        37  MISO0
  P1[24]/MCFB2/PWM1[5]/MOSI0        38  MOSI0
  P1[25]/MC1A/MAT1[1]               39  LED1
  P1[26]/MC1B/PWM1[6]/CAP0[0]       40  CS_UEXT
  P1[27]/CLKOUT/#USB_OVRCR/CAP0[1]  43  #USB_OVRCR
  P1[28]/MC2A/PCAP1[0]/MAT0[0]      44  P1.28
  P1[29]/MC2B/PCAP1[1]/MAT0[1]      45  P1.29
  P1[30]/VBUS/AD0[4]                21  VBUS
  P1[31]/SCK1/AD0[5]                20  AIN5
  P2[0]/PWM1[1]/TXD1                75  UP
  P2[1]/PWM1[2]/RXD1                74  DOWN
  P2[2]/PWM1[3]/CTS1/TRACEDATA[3]   73  TRACE_D3
  P2[3]/PWM1[4]/DCD1/TRACEDATA[2]   70  TRACE_D2
  P2[4]/PWM1[5]/DSR1/TRACEDATA[1]   69  TRACE_D1
  P2[5]/PWM1[6]/DTR1/TRACEDATA[0]   68  TRACE_D0
  P2[6]/PCAP1[0]/RI1/TRACECLK       67  TRACE_CLK
  P2[7]/RD2/RTS1                    66  LEFT
  P2[8]/TD2/TXD2                    65  RIGHT
  P2[9]/USB_CONNECT/RXD2            64  USBD_CONNECT
  P2[10]/#EINT0/NMI                 53  ISP_E4
  P2[11]/#EINT1/I2STX_CLK           52  #EINT1
  P2[12]/#EINT2/I2STX_WS            51  WAKE-UP
  P2[13]/#EINT3/I2STX_SDA           50  BUT2
  P3[25]/MAT0[0]/PWM1[2]            27  LCD_RST
  P3[26]/STCLK/MAT0[1]/PWM1[3]      26  LCD_BL

  Serial Console
  --------------

  The LPC1766-STK board has two serial connectors.  One, RS232_0, connects to
  the LPC1766 UART0.  This is the DB-9 connector next to the power connector.
  The other RS232_1, connect to the LPC1766 UART1.  This is he DB-9 connector
  next to the Ethernet connector.

  Simple UART1 is the more flexible UART and since the needs for a serial
  console are minimal, the more minimal UART0/RS232_0 is used for the NuttX
  system console.  Of course, this can be changed by editing the NuttX
  configuration file as discussed below.

  The serial console is configured as follows (57600 8N1):

    BAUD: 57600
    Number of Bits: 8
    Parity: None
    Stop bits: 1

  You will need to connect a monitor program (Hyperterminal, Tera Term,
  minicom, whatever) to UART0/RS232_0 and configure the serial port as
  shown above.

  NOTE: These configurations have problems at 115200 baud.

  LCD
  ---

  The LPC1766-STK has a Nokia 6100 132x132 LCD and either a Phillips PCF8833
  or an Epson S1D15G10 LCD controller.  The NuttX configuration may have to
  be adjusted depending on which controller is used with the LCD.  The
  "LPC1766-STK development board Users Manual" states tha the board features
  a "LCD NOKIA 6610 128x128 x12bit color TFT with Epson LCD controller."
  But, referring to a different Olimex board, "Nokia 6100 LCD Display
  Driver," Revision 1, James P. Lynch ("Nokia 6100 LCD Display Driver.pdf")
  says:

  "The major irritant in using this display is identifying the graphics
   controller; there are two possibilities (Epson S1D15G00 or Philips
   PCF8833). The LCD display sold by the German Web Shop Jelu has a Leadis
   LDS176 controller but it is 100% compatible with the Philips PCF8833).
   So how do you tell which controller you have? Some message boards have
   suggested that the LCD display be disassembled and the controller chip
   measured with a digital caliper well that's getting a bit extreme.

  "Here's what I know. The Olimex boards have both display controllers
   possible; if the LCD has a GE-12 sticker on it, it's a Philips PCF8833.
   If it has a GE-8 sticker, it's an Epson controller. The older Sparkfun
   6100 displays were Epson, their web site indicates that the newer ones
   are an Epson clone. Sparkfun software examples sometimes refer to the
   Philips controller so the whole issue has become a bit murky. The
   trading companies in Honk Kong have no idea what is inside the displays
   they are selling. A Nokia 6100 display that I purchased from Hong Kong
   a couple of weeks ago had the Philips controller."

  The LCD connects to the LPC1766 via SPI and two GPIOs.  The two GPIOs are
  noted above:

    P1.21 is the SPI chip select, and
    P3.25 is the LCD reset
    P3.26 is PWM1 output used to control the backlight intensity.

  MISO0 and MOSI0 are join via a 1K ohm resistor so the LCD appears to be
  write only.

  STATUS:  The LCD driver was never properly integrated.  It was awkward
  to use because it relied on a 9-bit SPI interface (the 9th bit being
  the command/data bit which is normally a discrete input).  All support
  for the Nokia 6100 was removed on May 19, 2018.  That obsoleted
  driver can be viewed in the
  nuttx/drivers/lcd and boards/arm/lpc17xx_40xx/olimex-lpc1766stk
  directories of the Obsoleted repository.

  The obsoleted driver attempted to created the 9th bit on-they-flay in the
  data by expanding the 8-bit data to 16-bits with the 9th bit managed.  I
  no longer believe that is the correct technical approach.  I now believe
  that the best solution would be to provide custom management of the 9th
  data bit inside of the low-level MCU driver, the LPC17 SPI driver in thisi
  case, via a configuration option on the low-level driver.

LEDs
^^^^

  If CONFIG_ARCH_LEDS is defined, then support for the LPC1766-STK LEDs will be
  included in the build.  See:

  - boards/arm/lpc17xx_40xx/olimex-lpc1766stk/include/board.h - Defines LED
           constants, types and prototypes the LED interface functions.

  - boards/arm/lpc17xx_40xx/olimex-lpc1766stk/src/lpc1766stk.h - GPIO settings
           for the LEDs.

  - boards/arm/lpc17xx_40xx/olimex-lpc1766stk/src/up_leds.c - LED control logic.

  The LPC1766-STK has two LEDs.  If CONFIG_ARCH_LEDS is defined, these LEDs will
  be controlled as follows for NuttX debug functionality (where NC means "No Change").
  Basically,

  LED1:
  - OFF means that the OS is still initializing. Initialization is very fast so
    if you see this at all, it probably means that the system is hanging up
    somewhere in the initialization phases.
  - ON means that the OS completed initialization.
  - Glowing means that the LPC17 is running in a reduced power mode: LED1 is
    turned off when the processor enters sleep mode and back on when it wakesup
    up.

  LED2:
  - ON/OFF toggles means that various events are happening.
  - GLowing: LED2 is turned on and off on every interrupt so even timer interrupts
    should cause LED2 to glow faintly in the normal case.
  - Flashing. If the LED2 is flashing at about 2Hz, that means that a crash
    has occurred.  If CONFIG_ARCH_STACKDUMP=y, you will get some diagnostic
    information on the console to help debug what happened.

  NOTE:  LED2 is controlled by a jumper labeled: ACC_IRQ/LED2.  That jump must be
  in the LED2 position in order to support LED2.

  LED1    LED2      Meaning
  ------- --------  --------------------------------------------------------------------
   OFF    OFF      Still initializing and there is no interrupt activity.
                    Initialization is very fast so if you see this, it probably means
                    that the system is hung up somewhere in the initialization phases.
   OFF     Glowing  Still initializing (see above) but taking interrupts.
   OFF     ON       This would mean that (1) initialization did not complete but the
                    software is hung, perhaps in an infinite loop, somewhere inside
                    of an interrupt handler.
   OFF     Flashing Ooops!  We crashed before finishing initialization (or, perhaps
                    after initialization, during an interrupt while the LPC17xx/LPC40xx was
                    sleeping -- see below).

   ON      OFF      The system has completed initialization, but is apparently not taking
                    any interrupts.
   ON      Glowing  The OS successfully initialized and is taking interrupts (but, for
                    some reason, is never entering a reduced power mode -- perhaps the
                    CPU is very busy?).
   ON      ON       This would mean that (1) the OS complete initialization, but (2)
                    the software is hung, perhaps in an infinite loop, somewhere inside
                    of a signal or interrupt handler.
   Glowing Glowing  This is also a normal healthy state: The OS successfully initialized,
                    is running in reduced power mode, but taking interrupts.  The glow
                    is very faint and you may have to dim the lights to see that LEDs are
                    active at all!  See note below.
   ON      Flashing Ooops!  We crashed sometime after initialization.

  NOTE: In glowing/glowing case, you get some good subjective information about the
  behavior of your system by looking at the level of the LED glow (or better, by
  connecting O-Scope and calculating the actual duty):

  1. The intensity of the glow is determined by the duty of LED on/off toggle --
     as the ON period becomes larger with respect the OFF period, the LED will
     glow more brightly.
  2. LED2 is turned ON when entering an interrupt and turned OFF when returning from
     the interrupt.  A brighter LED2 means that the system is spending more time in
     interrupt handling.
  3. LED1 is turned OFF just before the processor goes to sleep.  The processor
     sleeps until awakened by an interrupt.  LED1 is turned back ON after the
     processor is re-awakened -- actually after returning from the interrupt that
     cause the processor to re-awaken (LED1 will be off during the execution of
     that interrupt).  So a brighter LED1 means that the processor is spending
     less time sleeping.

  When my LPC1766 sits IDLE -- doing absolutely nothing but processing timer interrupts --
  I see the following:

  1. LED1 glows dimly due to the timer interrupts.
  2. But LED2 is even more dim!  The LED ON time excludes the time processing the
     interrupt that re-awakens the processing.  So this tells me that the LPC1766 is
     spending more time processing timer interrupts than doing any other kind of
     processing.  That, of course, makes sense if the system is truly idle and only
     processing timer interrupts.

Serial Console
^^^^^^^^^^^^^^

  By default, all of these configurations use UART0 for the NuttX serial
  console.  UART0 corresponds to the DB-9 connector labelled "RS232_0".  This
  is a female connector and will require a normal male-to-female RS232 cable
  to connect to a PC.

  An alternate is UART1 which connects to the other DB-9 connector labeled
  "RS232_1".  UART1 is not enabled by default unless specifically noted
  otherwise in the configuration description.  A normal serial cable must be
  used with the port as well.

  By default serial console is configured for 57600 baud, 8-bit, 1 stop bit,
  and no parity.  Higher rates will probably require minor modification of
  the UART initialization logic to use the fractional dividers.

Using OpenOCD and GDB with an FT2232 JTAG emulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  Downloading OpenOCD

    You can get information about OpenOCD here: http://openocd.berlios.de/web/
    and you can download it from here. http://sourceforge.net/projects/openocd/files/.
    To get the latest OpenOCD with more mature lpc17xx, you have to download
    from the GIT archive.

      git clone git://openocd.git.sourceforge.net/gitroot/openocd/openocd

    At present, there is only the older, frozen 0.4.0 version.  These, of course,
    may have changed since I wrote this.

  Building OpenOCD under Cygwin:

    You can build OpenOCD for Windows using the Cygwin tools.  Below are a
    few notes that worked as of November 7, 2010.  Things may have changed
    by the time you read this, but perhaps the following will be helpful to
    you:

    1. Install Cygwin (http://www.cygwin.com/).  My recommendation is to install
       everything.  There are many tools you will need and it is best just to
       waste a little disk space and have everything you need.  Everything will
       require a couple of gigbytes of disk space.

    2. Create a directory /home/OpenOCD.

    3. Get the FT2232 driver from http://www.ftdichip.com/Drivers/D2XX.htm and
       extract it into /home/OpenOCD/ftd2xx

       $ pwd
       /home/OpenOCD
       $ ls
       CDM20802 WHQL Certified.zip
       $ mkdir ftd2xx
       $ cd ftd2xx
       $ unzip ..CDM20802\ WHQL\ Certified.zip
       Archive:  CDM20802 WHQL Certified.zip
       ...

    3. Get the latest OpenOCD source

       $ pwd
       /home/OpenOCD
       $ git clone git://openocd.git.sourceforge.net/gitroot/openocd/openocd

       You will then have the source code in /home/OpenOCD/openocd

    4. Build OpenOCD for the FT22322 interface

       $ pwd
       /home/OpenOCD/openocd
       $ ./bootstrap

       Jim is a tiny version of the Tcl scripting language.  It is needed
       by more recent versions of OpenOCD.  Build libjim.a using the following
       instructions:

       $ git submodule init
       $ git submodule update
       $ cd jimtcl
       $ ./configure --with-jim-ext=nvp
       $ make
       $ make install

       Configure OpenOCD:

       $ ./configure --enable-maintainer-mode --disable-werror --disable-shared \
                    --enable-ft2232_ftd2xx --with-ftd2xx-win32-zipdir=/home/OpenOCD/ftd2xx \
                    LDFLAGS="-L/home/OpenOCD/openocd/jimtcl"

        Then build OpenOCD and its HTML documentation:

        $ make
        $ make html

        The result of the first make will be the "openocd.exe" will be
        created in the folder /home/openocd/src.  The following command
        will install OpenOCD to a standard location (/usr/local/bin)
        using using this command:

        $ make install

  Helper Scripts.

    I have been using the Olimex ARM-USB-OCD JTAG debugger with the
    LPC1766-STK (http://www.olimex.com).  OpenOCD requires a configuration
    file.  I keep the one I used last here:

      boards/arm/lpc17xx_40xx/olimex-lpc1766stk/tools/olimex.cfg

    However, the "correct" configuration script to use with OpenOCD may
    change as the features of OpenOCD evolve.  So you should at least
    compare that olimex.cfg file with configuration files in
    /usr/local/share/openocd/scripts/target (or /home/OpenOCD/openocd/tcl/target).
    As of this writing, there is no script for the lpc1766, but the
    lpc1768 configuration can be used after changing the flash size to
    256Kb.  That is, change:

      flash bank $_FLASHNAME lpc2000 0x0 0x80000 0 0 $_TARGETNAME ...

    To:

      flash bank $_FLASHNAME lpc2000 0x0 0x40000 0 0 $_TARGETNAME ...

    There is also a script on the tools/ directory that I use to start
    the OpenOCD daemon on my system called oocd.sh.  That script will
    probably require some modifications to work in another environment:

    - Possibly the value of OPENOCD_PATH and TARGET_PATH
    - It assumes that the correct script to use is the one at
      boards/arm/lpc17xx_40xx/olimex-lpc1766stk/tools/olimex.cfg

  Starting OpenOCD

    Then you should be able to start the OpenOCD daemon like:

      boards/arm/lpc17xx_40xx/olimex-lpc1766stk/tools/oocd.sh $PWD

    If you add the path to oocd.sh to your PATH environment variable,
    the command simplifies to just:

      oocd.sh $PWD

    Where it is assumed that you are executing oocd.sh from the top-level
    directory where NuttX is installed.  $PWD will be the path to the
    top-level NuttX directory.

  Connecting GDB

    Once the OpenOCD daemon has been started, you can connect to it via
    GDB using the following GDB command:

      arm-nuttx-elf-gdb
      (gdb) target remote localhost:3333

    NOTE:  The name of your GDB program may differ.  For example, with the
    ARM EABI toolchain, the ARM GDB would be called arm-none-eabi-gdb.

    After starting GDB, you can load the NuttX ELF file:

      (gdb) symbol-file nuttx
      (gdb) load nuttx

    NOTES:
    1. Loading the symbol-file is only useful if you have built NuttX to
       include debug symbols (by setting CONFIG_DEBUG_SYMBOLS=y in the
       .config file).
    2. I usually have to reset, halt, and 'load nuttx' a second time.  For
       some reason, the first time apparently does not fully program the
       FLASH.
    3. The MCU must be halted prior to loading code using 'mon reset'
       as described below.

    OpenOCD will support several special 'monitor' commands.  These
    GDB commands will send comments to the OpenOCD monitor.  Here
    are a couple that you will need to use:

     (gdb) monitor reset
     (gdb) monitor halt

    NOTES:
    1. The MCU must be halted using 'mon halt' prior to loading code.
    2. Reset will restart the processor after loading code.
    3. The 'monitor' command can be abbreviated as just 'mon'.

Olimex LPC1766-STK Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lpc17xx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LPC1766=y

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=olimex-lpc1766stk (for the Olimex LPC1766-STK)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_LPC1766STK=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (CPU SRAM in this case):

       CONFIG_RAM_SIZE=(32*1024) (32Kb)

       There is an additional 32Kb of SRAM in AHB SRAM banks 0 and 1.

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x10000000

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
        stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    Individual subsystems can be enabled:

      CONFIG_LPC17_40_MAINOSC=y
      CONFIG_LPC17_40_PLL0=y
      CONFIG_LPC17_40_PLL1=n
      CONFIG_LPC17_40_ETHERNET=n
      CONFIG_LPC17_40_USBHOST=n
      CONFIG_LPC17_40_USBOTG=n
      CONFIG_LPC17_40_USBDEV=n
      CONFIG_LPC17_40_UART0=y
      CONFIG_LPC17_40_UART1=n
      CONFIG_LPC17_40_UART2=n
      CONFIG_LPC17_40_UART3=n
      CONFIG_LPC17_40_CAN1=n
      CONFIG_LPC17_40_CAN2=n
      CONFIG_LPC17_40_SPI=n
      CONFIG_LPC17_40_SSP0=n
      CONFIG_LPC17_40_SSP1=n
      CONFIG_LPC17_40_I2C0=n
      CONFIG_LPC17_40_I2C1=n
      CONFIG_LPC17_40_I2S=n
      CONFIG_LPC17_40_TMR0=n
      CONFIG_LPC17_40_TMR1=n
      CONFIG_LPC17_40_TMR2=n
      CONFIG_LPC17_40_TMR3=n
      CONFIG_LPC17_40_RIT=n
      CONFIG_LPC17_40_PWM0=n
      CONFIG_LPC17_40_MCPWM=n
      CONFIG_LPC17_40_QEI=n
      CONFIG_LPC17_40_RTC=n
      CONFIG_LPC17_40_WDT=n
      CONFIG_LPC17_40_ADC=n
      CONFIG_LPC17_40_DAC=n
      CONFIG_LPC17_40_GPDMA=n
      CONFIG_LPC17_40_FLASH=n

  LPC17xx/LPC40xx specific device driver settings

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

  LPC17xx/LPC40xx specific CAN device driver settings.  These settings all
  require CONFIG_CAN:

    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_LPC17_40_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC17_40_CAN1
      is defined.
    CONFIG_LPC17_40_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC17_40_CAN2
      is defined.
    CONFIG_LPC17_40_CAN1_DIVISOR - CAN1 is clocked at CCLK divided by this
      number. (the CCLK frequency is divided by this number to get the CAN
      clock). Options = {1,2,4,6}. Default: 4.
    CONFIG_LPC17_40_CAN2_DIVISOR - CAN2 is clocked at CCLK divided by this
      number.  (the CCLK frequency is divided by this number to get the CAN
      clock).  Options = {1,2,4,6}. Default: 4.
    CONFIG_LPC17_40_CAN_TSEG1 - The number of CAN time quanta in segment 1.
      Default: 6
    CONFIG_LPC17_40_CAN_TSEG2 = the number of CAN time quanta in segment 2.
      Default: 7

  LPC17xx/LPC40xx specific PHY/Ethernet device driver settings.  These setting
  also require CONFIG_NET and CONFIG_LPC17_40_ETHERNET.

    CONFIG_ETH0_PHY_KS8721 - Selects Micrel KS8721 PHY
    CONFIG_LPC17_40_PHY_AUTONEG - Enable auto-negotiation
    CONFIG_LPC17_40_PHY_SPEED100 - Select 100Mbit vs. 10Mbit speed.
    CONFIG_LPC17_40_PHY_FDUPLEX - Select full (vs. half) duplex

    CONFIG_LPC17_40_EMACRAM_SIZE - Size of EMAC RAM.  Default: 16Kb
    CONFIG_LPC17_40_ETH_NTXDESC - Configured number of Tx descriptors. Default: 18
    CONFIG_LPC17_40_ETH_NRXDESC - Configured number of Rx descriptors. Default: 18
    CONFIG_LPC17_40_ETH_WOL - Enable Wake-up on Lan (not fully implemented).
    CONFIG_NET_REGDEBUG - Enabled low level register debug.  Also needs
      CONFIG_DEBUG_FEATURES.
    CONFIG_NET_DUMPPACKET - Dump all received and transmitted packets.
      Also needs CONFIG_DEBUG_FEATURES.
    CONFIG_LPC17_40_ETH_HASH - Enable receipt of near-perfect match frames.
    CONFIG_LPC17_40_MULTICAST - Enable receipt of multicast (and unicast) frames.
      Automatically set if CONFIG_NET_MCASTGROUP is selected.

  LPC17xx/LPC40xx USB Device Configuration

    CONFIG_LPC17_40_USBDEV_FRAME_INTERRUPT
      Handle USB Start-Of-Frame events.
      Enable reading SOF from interrupt handler vs. simply reading on demand.
      Probably a bad idea... Unless there is some issue with sampling the SOF
      from hardware asynchronously.
    CONFIG_LPC17_40_USBDEV_EPFAST_INTERRUPT
      Enable high priority interrupts.  I have no idea why you might want to
      do that
    CONFIG_LPC17_40_USBDEV_NDMADESCRIPTORS
      Number of DMA descriptors to allocate in SRAM.
    CONFIG_LPC17_40_USBDEV_DMA
      Enable lpc17xx/lpc40xx-specific DMA support
    CONFIG_LPC17_40_USBDEV_NOVBUS
      Define if the hardware implementation does not support the VBUS signal
    CONFIG_LPC17_40_USBDEV_NOLED
      Define if the hardware  implementation does not support the LED output

  LPC17xx/LPC40xx USB Host Configuration
    CONFIG_LPC17_40_OHCIRAM_SIZE
      Total size of OHCI RAM (in AHB SRAM Bank 1)
    CONFIG_LP17_USBHOST_NEDS
      Number of endpoint descriptors
    CONFIG_LP17_USBHOST_NTDS
      Number of transfer descriptors
    CONFIG_LPC17_40_USBHOST_TDBUFFERS
      Number of transfer descriptor buffers
    CONFIG_LPC17_40_USBHOST_TDBUFSIZE
      Size of one transfer descriptor buffer
    CONFIG_LPC17_40_USBHOST_IOBUFSIZE
      Size of one end-user I/O buffer.  This can be zero if the
      application can guarantee that all end-user I/O buffers
      reside in AHB SRAM.

USB Host Configuration
^^^^^^^^^^^^^^^^^^^^^^

The NuttShell (NSH) configuration can be modified in order to support
USB host operations.  To make these modifications, do the following:

1. First configure to build the NSH configuration from the top-level
   NuttX directory:

     ./configure olimex-lpc1766stk/nsh

2. Modify the top-level .config file to enable USB host using:

     make menuconfig

   Make the following changes:

     System Type -> LPC17xx/LPC40xx Peripheral Support
       CONFIG_LPC17_40_USBHOST=y

     Device Drivers-> USB Host Driver Support
       CONFIG_USBHOST=y
       CONFIG_USBHOST_ISOC_DISABLE=y
       CONFIG_USBHOST_MSC=y

     Library Routines
       CONFIG_SCHED_WORKQUEUE=y

When this change is made, NSH should be extended to support USB flash
devices.  When a FLASH device is inserted, you should see a device
appear in the /dev (pseudo) directory.  The device name should be
like /dev/sda, /dev/sdb, etc.  The USB mass storage device, is present
it can be mounted from the NSH command line like:

   ls /dev
   mount -t vfat /dev/sda /mnt/flash

Files on the connect USB flash device should then be accessible under
the mountpoint /mnt/flash.

Configurations
^^^^^^^^^^^^^^

Common Configuration Notes
--------------------------

  1. Each Olimex LPC1766-STK configuration is maintained in a
     sub-directory and can be selected as follow:

       tools/configure.sh olimex-lpc1766stk:<subdir>

     Where <subdir> is one of the sub-directories identified in the following
     paragraphs.

     Use configure.bat instead of configure.sh if you are building in a
     Windows native environment.

  2. These configurations use the mconf-based configuration tool.  To
     change a configuration using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

Configuration Sub-Directories
-----------------------------

  ftpc:
    This is a simple FTP client shell used to exercise the capabilities
    of the FTPC library (apps/netutils/ftpc).  This example is configured
    to that it will only work as a "built-in" program that can be run from
    NSH when CONFIG_NSH_BUILTIN_APPS is defined.

    From NSH, the startup command sequence is then:

      nsh> mount -t vfat /dev/mmcsd0 /tmp # Mount the SD card at /tmp
      nsh> cd /tmp                        # cd into the /tmp directory
      nsh> ftpc xx.xx.xx.xx[:pp]          # Start the FTP client
      nfc> login <name> <password>        # Log into the FTP server
      nfc> help                           # See a list of FTP commands

    where xx.xx.xx.xx is the IP address of the FTP server and pp is an
    optional port number (default is the standard FTP port number 21).

    NOTES:

    1. Support for FAT long file names is built-in but can easily be
       removed if you are concerned about Microsoft patent issues (see the
       section "FAT Long File Names" in the top-level NOTICE file).

       CONFIG_FS_FAT=y
       CONFIG_FAT_LCNAMES=y <-- Long file name support
       CONFIG_FAT_LFN=y
       CONFIG_FAT_MAXFNAME=32
       CONFIG_FS_NXFFS=n
       CONFIG_FS_ROMFS=n

    2. This configuration targets Linux using a generic ARM EABI toolchain:

       CONFIG_LINUX=y
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y

       But that can easily be re-configured.

    2. You may also want to define the following in your configuration file.
       Otherwise, you will have not feedback about what is going on:

       CONFIG_DEBUG_FEATURES=y
       CONFIG_DEBUG_INFO=y
       CONFIG_DEBUG_FTPC=y

  hidkbd:
    This configuration directory supports a variant of an NSH configuration.
    It is set up to perform the HID keyboard test at apps/examples/hidkbd.

    NOTES:

    1. Default platform/toolchain: This is how the build is configured by
       be default.  These options can easily be re-confured, however.

       CONFIG_HOST_LINUX=y                     : Linux
       CONFIG_ARMV7M_TOOLCHAIN_EABIL=y         : Generic EABI toolchain

    STATUS:
      2018-10-07:  Not all keyboards will connect successfully. I have not
        looked into the details but it may be that those keyboards are not
        compatible with the driver (which only accepts "boot" keyboards).
        Also, when typing input into the HID keyboard, characters are often
        missing and sometimes duplicated.  This is like some issue with the
        read logic of drivers/usbhost_hidkbc.c.

  hidmouse:
    This configuration directory supports a variant of an NSH configuration.
    It is set up to perform the touchscreen test at apps/examples/touchscreen
    using a USB HIB mouse instead a touchsceen device.

    NOTES:

    1. Default platform/toolchain: This is how the build is configured by
       be default.  These options can easily be re-confured, however.

       CONFIG_HOST_WINDOWS=y                   : Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Cygwin environment on Windows
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y      : GNU EABI toolchain for Windows

    2. The mouse is really useless with no display and no cursor.  So this
       configuration is only suited for low-level testing.  It is also awkward
       to use.  Here are the steps:

       - Remove the USB HID mouse and reset the board.
       - When the NSH prompt comes up type 'tc'.  That will fail, but it
         will register the USB HID mouse class driver.
       - Now, insert the USB HID mouse.  The next time that you enter the
         'tc' command, the mouse device at /dev/mouse0 should be found.

  nettest:
    This configuration directory may be used to enable networking using the
    LPC17xx/LPC40xx's Ethernet controller. It uses apps/examples/nettest to exercise the
    TCP/IP network.

  nsh:
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables both the serial and telnet NSH interfaces.
    Support for the board's SPI-based MicroSD card is included.

    NOTE:  If you start the program with no SD card inserted, there will be
    a substantial delay. This is because there is no hardware support to sense
    whether or not an SD card is inserted.  As a result, the driver has to
    go through many retries and timeouts before it finally decides that there
    is not SD card in the slot.

    NOTES:

    1. Uses the older, OABI, buildroot toolchain.  But that is easily
       reconfigured:

       CONFIG_ARM_TOOLCHAIN_BUILDROOT=y : Buildroot toolchain
       CONFIG_ARM_TOOLCHAIN_BUILDROOT_OABI=y      : Older, OABI toolchain

    2. This configuration supports a network.  You may have to change
       these settings for your network:

       CONFIG_NSH_IPADDR=0x0a000002        : IP address: 10.0.0.2
       CONFIG_NSH_DRIPADDR=0x0a000001      : Gateway:    10.0.0.1
       CONFIG_NSH_NETMASK=0xffffff00       : Netmask:    255.255.255.0

    3. This configuration supports the SPI-based MMC/SD card slot.
       FAT file system support for FAT long file names is built-in but
       can easily be removed if you are concerned about Microsoft patent
       issues (see the section "FAT Long File Names" in the top-level
       NOTICE file).

       CONFIG_FAT_LFN=y                    : Enables long file name support

  slip-httpd:
    This configuration is identical to the thttpd configuration except that
    it uses the SLIP data link layer via a serial driver instead of the
    Ethernet data link layer.  The Ethernet driver is disabled; SLIP IP
    packets are exchanged on UART1; UART0 is still the serial console.

    1. Configure and build the slip-httpd configuration.
    2. Connect to a Linux box (assuming /dev/ttyS0)
    3. Reset on the target side and attach SLIP on the Linux side:

       $ modprobe slip
       $ slattach -L -p slip -s 57600 /dev/ttyS0 &

       This should create an interface with a name like sl0, or sl1, etc.
       Add -d to get debug output.  This will show the interface name.

       NOTE: The -L option is included to suppress use of hardware flow
       control.  This is necessary because I haven't figured out how to
       use the UART1 hardware flow control yet.

       NOTE: The Linux slip module hard-codes its MTU size to 296.  So you
       might as well set CONFIG_NET_ETH_PKTSIZE to 296 as well.

    4. After turning over the line to the SLIP driver, you must configure
       the network interface. Again, you do this using the standard
       ifconfig and route commands. Assume that we have connected to a
       host PC with address 192.168.0.101 from your target with address
       10.0.0.2. On the Linux PC you would execute the following as root:

       $ ifconfig sl0 10.0.0.1 pointopoint 10.0.0.2 up
       $ route add 10.0.0.2 dev sl0

       Assuming the SLIP is attached to device sl0.

    5. For monitoring/debugging traffic:

       $ tcpdump -n -nn -i sl0 -x -X -s 1500

    NOTE: Only UART1 supports the hardware handshake.  If hardware
    handshake is not available, then you might try the slattach option
    -L which is supposed to enable "3-wire operation."

    NOTE: This configurat only works with VERBOSE debug disabled.  For some
    reason, certain debug statements hang(?).

    NOTE: This example does not use UART1's hardware flow control.  UART1
    hardware flow control is partially implemented but does not behave as
    expected.  It needs a little more work.

  thttpd-binfs:
    This builds the THTTPD web server example using the THTTPD and
    the apps/examples/thttpd application.  This version uses the built-in
    binary format with the BINFS file system and the Union File System.
    Otherwise it is equivalent to thttpd-binfs.

    NOTES:

    1. Uses the ARM EABI toolchain under Windows.  But that is
       easily reconfigured:

       CONFIG_HOST_WINDOWS=y                   : Windows
       CONFIG_HOST_WINDOWS_CYGWIN=y            : under Cygwin
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y      : GNU EABI toolchain for Windows

  STATUS:
    2015-06-02.  This configuration was added in an attempt to replace
      thttpd-nxflat (see below).  I concurrently get out-of-memory errors
      during execution of CGI.  The 32KiB SRAM may be insufficient for
      this configuration; this might be fixed with some careful tuning
      of stack usage.

    2015-06-06: Modified to use the Union File System.  Untested.
      This configuration was ported to the lincoln60 which has an LPC1769
      and, hence, more SRAM.  Additional memory reduction steps were
      required to run on the LPC1769.  See nuttx/boards/lincoln60/README.txt
      for additional information.

  thttpd-nxflat:
    This builds the THTTPD web server example using the THTTPD and
    the apps/examples/thttpd application.  This version uses the NXFLAT
    binary format with the ROMFS file system, otherwise it is equivalent to
    thttpd-binfs.

    NOTES:

    1. Uses the newer, EABI, buildroot toolchain.  But that is easily
       reconfigured:

       CONFIG_HOST_LINUX=y                 : Linux
       CONFIG_ARM_TOOLCHAIN_BUILDROOT=y : Buildroot toolchain
       CONFIG_ARM_TOOLCHAIN_BUILDROOT_OABI=n      : Newer, EABI toolchain

  STATUS:
    2015-06-02.  Do to issues introduced by recent versions of GCC, NXFLAT
      is not often usable.

      See https://cwiki.apache.org/confluence/pages/viewpage.action?pageId=139630111

  usbserial:
    This configuration directory exercises the USB serial class
    driver at apps/examples/usbserial.  See apps/examples/README.txt for
    more information.

  usbmsc:
    This configuration directory exercises the USB mass storage
    class driver at apps/system/usbmsc.  See apps/examples/README.txt
    for more information.

  zmodem:
    This is an alternative NSH configuration that was used to test Zmodem
    file transfers.  It is similar to the standard NSH configuration but has
    the following differences:

    1. UART0 is still the NuttX serial console as with most of the other
       configurations here.  However, UART1 is also enabled for performing
       the Zmodem transfers.

         CONFIG_LPC17XX_40XX_UART1=y
         CONFIG_UART1_ISUART=y
         CONFIG_UART1_RXBUFSIZE=1024
         CONFIG_UART1_TXBUFSIZE=256
         CONFIG_UART1_BAUD=2400
         CONFIG_UART1_BITS=8
         CONFIG_UART1_PARITY=0
         CONFIG_UART1_2STOP=0

    2. Hardware Flow Control

       In principle, Zmodem transfers could be performed on the any serial
       device, including the console device.  However, only the LPC17xx/LPC40xx
       UART1 supports hardware flow control which is required for Zmodem
       transfers.  Also, this configuration permits debug output on the
       serial console while the transfer is in progress without interfering
       with the file transfer.

       In additional, a very low BAUD is selected to avoid other sources
       of data overrun.  This should be unnecessary if buffering and hardware
       flow control are set up correctly.

       However, in the LPC17xx/LPC40xx serial driver, hardware flow control only
       protects the hardware RX FIFO:  Data will not be lost in the hardware
       FIFO but can still be lost when it is taken from the FIFO.  We can
       still overflow the serial driver's RX buffer even with hardware flow
       control enabled! That is probably a bug.  But the workaround solution
       that I have used is to use lower data rates and a large serial driver
       RX buffer.

       Those measures should be unnecessary if buffering and hardware flow
       control are set up and working correctly.

    3. Buffering Notes:

       RX Buffer Size
       --------------
       The Zmodem protocol supports a message that informs the file sender
       of the maximum size of dat that you can buffer (ZRINIT).  However, my
       experience is that the Linux sz ignores this setting and always sends
       file data at the maximum size (1024) no matter what size of buffer you
       report.  That is unfortunate because that, combined with the
       possibilities of data overrun mean that you must use quite large
       buffering for Zmodem file receipt to be reliable (none of these issues
       effect sending of files).

       Buffer Recommendations
       ----------------------
       Based on the limitations of NuttX hardware flow control and of the
       Linux sz behavior, I have been testing with the following configuration
      (assuming UART1 is the Zmodem device):

       a) This setting determines that maximum size of a data packet frame:

          CONFIG_SYSTEM_ZMODEM_PKTBUFSIZE=1024

       b) Input Buffering.  If the input buffering is set to a full frame,
          then  data overflow is less likely.

          CONFIG_UART1_RXBUFSIZE=1024

       c) With a larger driver input buffer, the Zmodem receive I/O buffer
          can be smaller:

          CONFIG_SYSTEM_ZMODEM_RCVBUFSIZE=256

       d) Output buffering.  Overrun cannot occur on output (on the NuttX side)
          so there is no need to be so careful:

          CONFIG_SYSTEM_ZMODEM_SNDBUFSIZE=512
          CONFIG_UART1_TXBUFSIZE=256

    4. Support is included for the NuttX sz and rz commands.  In order to
       use these commands, you will need to mount the SD card so that you
       will have a file system to transfer files in and out of:

         nsh> mount -t vfat /dev/mmcds0 /mnt/sdcard

       NOTE:  You must use the mountpoint /mnt/sdcard because that is the
       Zmodem sandbox specified in the configuration:  All files received
       from the remote host will be stored at /mnt/sdcard because of:

         CONFIG_SYSTEM_ZMODEM_MOUNTPOINT="/mnt/sdcard"

       Hmmm.. I probably should set up an NSH script to just mount /dev/mmcsd0
       at /mnt/sdcard each time the board boots.

    4. Sending Files from the Target to the Linux Host PC

       This program has been verified against the rzsz programs running on a
       Linux PC.  To send a file to the PC, first make sure that the serial
       port is configured to work with the board:

         $ sudo stty -F /dev/ttyS0 2400     # Select 2400 BAUD
         $ sudo stty -F /dev/ttyS0 crtscts  # Enables CTS/RTS handshaking *
         $ sudo stty -F /dev/ttyS0 raw      # Puts the TTY in raw mode
         $ sudo stty -F /dev/ttyS0          # Show the TTY configuration

         * Only is hardware flow control is enabled.  It is *not* in this
           default configuration.

       Start rz on the Linux host:

         $ sudo rz </dev/ttyS0 >/dev/ttyS0

       You can add the rz -v option multiple times, each increases the level
       of debug output.

       NOTE: The NuttX Zmodem does sends rz\n when it starts in compliance with
       the Zmodem specification.  On Linux this, however, seems to start some
       other, incompatible version of rz.  You need to start rz manually to
       make sure that the correct version is selected.  You can tell when this
       evil rz/sz has inserted itself because you will see the '^' (0x5e)
       character replacing the standard Zmodem ZDLE character (0x19) in the
       binary data stream.

       If you don't have the rz command on your Linux box, the package to
       install rzsz (or possibly lrzsz).

       Then on the target:

         > sz -d /dev/ttyS1 <filename>

       Where filename is the full path to the file to send (i.e., it begins
       with the '/' character).

       /dev/ttyS1 is configured to support Hardware flow control in order to
       throttle therates of data transfer to fit within the allocated buffers.
       Other devices may be used but if they do not support hardware flow
       control, the transfers will fail

    5. Receiving Files on the Target from the Linux Host PC

       NOTE:  There are issues with using the Linux sz command with the NuttX
       rz command. See "STATUS" below.  It is recommended that you use the
       NuttX sz command on Linux as described in the next paragraph.

       To send a file to the target, first make sure that the serial port on
       the host is configured to work with the board:

         $ sudo stty -F /dev/ttyS0 2400     # Select 2400 BAUD
         $ sudo stty -F /dev/ttyS0 crtscts  # Enables CTS/RTS handshaking *
         $ sudo stty -F /dev/ttyS0 raw      # Puts the TTY in raw mode
         $ sudo stty -F /dev/ttyS0          # Show the TTY configuration

         * Only is hardware flow control is enabled.  It is *not* in this
           default configuration.

       Start rz on the on the target:

         nsh> rz -d /dev/ttyS1

       /dev/ttyS1 is configured to support Hardware flow control in order to
       throttle therates of data transfer to fit within the allocated buffers.
       Other devices may be used but if they do not support hardware flow
       control, the transfers will fail

       Then use the sz command on Linux to send the file to the target:

         $ sudo sz <filename> [-l nnnn] </dev/ttyS0 >/dev/ttyS0

       Where <filename> is the file that you want to send. If -l nnnn is not
       specified, then there will likely be packet buffer overflow errors.
       nnnn should be set to a value less than or equal to
       CONFIG_SYSTEM_ZMODEM_PKTBUFSIZE

       Where <filename> is the file that you want to send.

       The resulting file will be found where you have configured the Zmodem
       "sandbox" via CONFIG_SYSTEM_ZMODEM_MOUNTPOINT, in this case at
       /mnt/sdcard.

       You can add the az -v option multiple times, each increases the level
       of debug output.  If you want to capture the Linux rz output, then
       re-direct stderr to a log file by adding 2>az.log to the end of the
       rz command.

       If you don't have the az command on your Linux box, the package to
       install rzsz (or possibly lrzsz).

    STATUS
      2013-7-15:  Testing against the Linux rz/sz commands.

        I have been able to send large and small files with the target sz
        command. I have been able to receive small files, but there are
        problems receiving large files using the Linux sz command:  The
        Linux SZ does not obey the buffering limits and continues to send
        data while rz is writing the previously received data to the file
        and the serial driver's RX buffer is overrun by a few bytes while
        the write is in progress. As a result, when it reads the next
        buffer of data, a few bytes may be missing.  The symptom of this
        missing data is a CRC check failure.

        Either (1) we need a more courteous host application, or (2) we
        need to greatly improve the target side buffering capability!

        We might get better behavior if we use the NuttX rz/sz commands
        on the host side (see apps/system/zmodem/README.txt).

      2013-7-16:  More Testing against the Linux rz/sz commands.

        I have verified that with debug off and at lower serial
        BAUD (2400), the transfers of large files succeed without errors.  I
        do not consider this a "solution" to the problem.  I also found that
        the LPC17xx/LPC40xx hardware flow control causes strange hangs; Zmodem works
        much better with hardware flow control disabled.

        At this lower BAUD, RX buffer sizes could probably be reduced; Or
        perhaps the BAUD could be increased.  My thought, however, is that
        tuning in such an unhealthy situation is not the approach:  The
        best thing to do would be to use the matching NuttX sz on the Linux
        host side.

      2013-7-16. More Testing against the NuttX rz/sz on Both Ends.

        The NuttX sz/rz commands have been modified so that they can be
        built and executed under Linux.  In this case, there are no
        transfer problems at all in either direction and with large or
        small files.  This configuration could probably run at much higher
        serial speeds and with much smaller buffers (although that has not
        been verified as of this writing).

        CONCLUSION:  You really do need proper hardware flow control to
        use zmodem.  That is not currently implemented in the LPC17xx/LPC40xx
        family.
