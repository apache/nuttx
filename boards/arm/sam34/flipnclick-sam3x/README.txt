README
======

  This README discusses issues unique to NuttX configurations for the
  Mikroe Flip&Click SAM3X board.  This board is an Arduino-Due work-alike
  with four Mikroe Click bus interfaces.  Like the Arduino-Due, this board
  features the Atmel ATSAM3X8E MCU running at 84 MHz.

  Thanks to John Legg for contributing the Flip&Click SAM3X board!

Contents
========

  - STATUS
  - Buttons and LEDs
  - Serial Consoles
  - SPI
  - I2C
  - SSD1306 OLED
  - Loading Code
  - Flip&Click SAM3X-specific Configuration Options
  - Configurations

STATUS
======

  2018-01-07:  Created the configuration.  At present it does not work; I
    believe because of tool-related issues.  See discussion under "Loading
    Code" below.
  2018-01-24:  I ordered a JTAG connector and soldered that to the Flip'n'Click
    and I am now successfully able to load code.  The NSH configuration appears
    to be fully functional.
  2018-02-11:  Added the nxlines configuration to test the custom HiletGo
    OLED on a Click proto board.  This is the same logic from the Flip&Click
    PIC32MZ and the result is the same:  No complaints from the software, but
    nothing appears on the OLED.  There is, most likely, an error in my custom
    HiletGo Click.  Damn!

Buttons and LEDs
================

  Buttons
  -------
  There are no buttons on the Flip&Click SAM3X board.

  LEDs
  ----
  There are four LEDs on the top, blue side of the board.  Only
  one can be controlled by software:

    LED L - PB27 (PWM13)

  There are also four LEDs on the back, white side of the board:

    LED A - PC6
    LED B - PC5
    LED C - PC7
    LED D - PC8

  A high output value illuminates the LEDs.

  These LEDs are available to the application and are all available to the
  application unless CONFIG_ARCH_LEDS is defined.  In that case, the usage
  by the board port is defined in include/board.h and src/sam_autoleds.c.
  The LEDs are used to encode OS-related events as follows:

    SYMBOL           MEANING                        LED STATE
                                              L   A   B   C   D
    ---------------- ----------------------- --- --- --- --- ---
    LED_STARTED      NuttX has been started  OFF ON  OFF OFF OFF
    LED_HEAPALLOCATE Heap has been allocated OFF OFF ON  OFF OFF
    LED_IRQSENABLED  Interrupts enabled      OFF OFF OFF ON  OFF
    LED_STACKCREATED Idle stack created      OFF OFF OFF OFF ON
    LED_INIRQ        In an interrupt         GLO N/C N/C N/C N/C
    LED_SIGNAL       In a signal handler     GLO N/C N/C N/C N/C
    LED_ASSERTION    An assertion failed     GLO N/C N/C N/C N/C
    LED_PANIC        The system has crashed  2Hz N/C N/C N/C N/C
    LED_IDLE         MCU is is sleep mode    ---- Not used -----

  Thus if LED L is glowing faintly and all other LEDs are off (except LED D
  which was left on but is no longer controlled by NuttX and so may be in any
  state), NuttX has successfully booted and is, apparently, running normally
  and taking interrupts.  If any of LEDs A-D are statically set, then NuttX
  failed to boot and the LED indicates the initialization phase where the
  failure occurred.  If LED L is flashing at approximately 2Hz, then a fatal
  error has been detected and the system has halted.

  NOTE: After booting, LEDs A-D are no longer used by the system and may
  be controlled the application.

Serial Consoles
===============

  The SAM3X has a UART and 4 USARTS.  The Programming port uses a USB-to-
  serial chip connected to the first of the MCU (RX0 and TX0 on PA8 and PA9,
  respectively).  The output from that port is visible using the Arduino tool.

  [NOTE: My experience so far:  I get serial output on the virtual COM port
   via the UART, but I receive no serial input for keyboard data entered in
   the PC serial terminal.  I have not investigated this problem.  It may
   be something as simple as the Rx pin configuration.  Instead, I just
   switched to USART0.]

  Other convenient U[S]ARTs that may be used as the Serial console include:

  1) An Arduino Serial Shield.  The RX and TX pins are available on the
     Arduino connector D0 and D1 pins, respectively.  These are connected
     to USART0, RXD0 and TXD0 which are PA10 and PA11, respectively.

  2) Mikroe Click Serial Shield.  There are four Click bus connectors with
     serial ports available as follows:

     Click A:  USART0 RXD0 and TXD0 which are, again, PA10 and PA11.
     Click B:  USART1 RXD1 and TXD1 which are PA12 and PA13, respectively.
     Click C:  USART3 RXD3 and TXD3 which are PD5 and PD4, respectively.
     Click D:  USART3 RXD3 and TXD3 which are, again, PD5 and PD4.

  Other serial ports are probably available on the Arduino connector.  I
  will leave that as an exercise for the interested reader.

  The outputs from these pins is 3.3V.  You will need to connect RS232
  transceiver to get the signals to RS232 levels (or connect to the
  USB virtual COM port in the case of UART0).

  Any of UART and USART0-3 may be used as a serial console.  UART0 would
  be the preferred default console setting. However, due to the communication
  problems mentioned above, USART0 is used as the default serial console
  in all configurations.  But that is easily changed by modifying the
  configuration as described under "Configurations" below.

SPI
===

   SPI0 is available on the Arduino compatible SPI connector (but no SPI is
   available on pins D10-D13 of the main Arduino Shield connectors where
   you might expect then).  The SPI connector is configured as follows:

     Pin Board Signal SAM3X  Pin Board Signal SAM3X
     --- ------------ -----  --- ------------ -----
      1  SPI0_MISO    PA25    2  VCC-5V       N/A
      3  SPI0_SCK     PA27    4  SPI0_MOSI    PA26
      5  MRST         NRSTB   6  GND          N/A

   SPI0 is also available on each of the mikroBUS Click connectors (in
   addition to 5V and GND).  The connectivity differs only in the chip
   select pin:

     MikroBUS A:              MikroBUS B:
     Pin  Board Signal SAM3X  Pin  Board Signal SAM3X
     ---- ------------ -----  ---- ------------ -----
     CS   SPI0_CS0     PA28   CS   PA29         PA29
     SCK  SPI0_SCK     PA27   SCK  SPI0_SCK     PA27
     MISO SPI0_MISO    PA25   MISO SPI0_MISO    PA25
     MOSI SPI0_MOSI    PA26   MOSI SPI0_MOSI    PA26

     MikroBUS C:              MikroBUS D:
     Pin  Board Signal SAM3X  Pin  Board Signal SAM3X
     ---- ------------ -----  ---- ------------ -----
     CS   SPI0_CS2     PB21   CS   SPI0_CS3     PB23
     SCK  SPI0_SCK     PA27   SCK  SPI0_SCK     PA27
     MISO SPI0_MISO    PA25   MISO SPI0_MISO    PA25
     MOSI SPI0_MOSI    PA26   MOSI SPI0_MOSI    PA26

I2C
===

   I2C0 is available on pins D16-D17 of the Arduino Shield connectors where
   you would expect then.  The SPI connector is configured as follows:

     Pin Label J1 Board Signal SAM3X
     --- ----- -- ------------ -----
     D16 SCL1  8  I2C0_SCL     PA17
     D17 SDA1  7  I2C0_SDA     PA18

   I2C0 and I2C1 are also available on the mikroBUS Click connectors (in
   addition to 5V and GND).  The connectors A and B share I2C0 with the
   Arduino shield connector.  Connectors C and D both connect to I2C1:

     MikroBUS A:              MikroBUS B:
     Pin  Board Signal SAM3X  Pin  Board Signal SAM3X
     ---- ------------ -----  ---- ------------ -------
     SCL  I2C0_SCL     PA17   SCL  I2C0_SCL    PA17
     SDA  I2C0_SDA     PA1    SDA  I2C0_SDA    PA18

     MikroBUS C:              MikroBUS D:
     Pin  Board Signal SAM3X  Pin  Board Signal SAM3X
     ---- ------------ -----  ---- ------------ -------
     SCL  I2C1_SCL     PB13   SCL  I2C1_SCL     PB13
     SDA  I2C1_SDA     PB12   SDA  I2C1_SDA     PB12

SSD1306 OLED
============

  Hardware
  --------
  The HiletGo is a 128x64 OLED that can be driven either via SPI or I2C (SPI
  is the default and is what is used here).  I have mounted the OLED on a
  proto click board.  The OLED is connected as follows:

  OLED  ALIAS       DESCRIPTION   PROTO CLICK
  ----- ----------- ------------- -----------------
   GND              Ground        GND
   VCC              Power Supply  5V  (3-5V)
   D0   SCL,CLK,SCK Clock         SCK
   D1   SDA,MOSI    Data          MOSI,SDI
   RES  RST,RESET   Reset         RST (GPIO OUTPUT)
   DC   AO          Data/Command  INT (GPIO OUTPUT)
   CS               Chip Select   CS  (GPIO OUTPUT)

   NOTE that this is a write-only display (MOSI only)!

Loading Code
============

  [NOTE: This text was mostly copied from the Arduino Due README.txt.  I
   believe, however, that there have been significant changes to the
   tool environment such that Bossac may no longer be usable.  I don't
   know that for certain and perhaps someone with more knowledge of
   the tools than I could make this work.  See STATUS below for the
   current issues that I see.]

  Installing the Arduino USB Driver under Windows
  -----------------------------------------------

  1. Download the Windows version of the Arduino software, not the 1.0.x
     release but the latest (1.5.x or later) that supports the Arduino
     Due.  When the download finishes, unzip the downloaded file.

     In the current 1.8.x release, the Arduino Due support is not included
     in the base package but can be added by selecting the "Boards Manager"
     from the "Tools" menu.

  2. Connect the Flip&Click to your computer with a USB cable via the
     Programming port.

  3. The Windows driver installation should fail.

  4. Open the Device Manager

  5. Look for the listing named "Ports (COM & LPT)". You should see an open
     port named "Arduino Due Prog. Port".  Right click and select "Update
     driver".

  6. Select the "Browse my computer for Driver software" option.

  7. Right click on the "Arduino Due Prog. Port" and choose "Update Driver
     Software".

  8. Navigate to the folder with the Arduino IDE you downloaded and unzipped
     earlier. Locate and select the "Drivers" folder in the main Arduino
     folder (not the "FTDI USB Drivers" sub-directory).

  Loading NuttX to the Flip&Click Using Bossa
  -------------------------------------------

  Arduino uses BOSSA under the hood to load code and you can use BOSSA
  outside of Arduino.

  Where do you get it?

    Generic BOSSA installation files are available here:
    https://github.com/shumatech/BOSSA (formerly at
    http://sourceforge.net/projects/b-o-s-s-a/?source=dlp)

    Pre-built binaries are available: https://github.com/shumatech/BOSSA/releases

    The original Arduino DUE used a patched version of BOSSA available
    as source code here: https://github.com/shumatech/BOSSA/tree/arduino
    But that has most likely been incorporated into the main github
    repository.

    But, fortunately, since you already installed Arduino, you already have
    BOSSA installed.  In my installation, it is here:

    C:\Program Files (x86)\Arduino\arduino-1.5.2\hardware\tools\bossac.exe

  General Procedure

    1) Erase the FLASH and put the Flip&Click in bootloader mode
    2) Write the file to FLASH
    3) Configure to boot from FLASH
    4) Reset the Flip&Click

  Erase FLASH and Put the Flip&Click in Bootloader Mode

    This is accomplished by simply configuring the programming port in 1200
    baud and sending something on the programming port.  Here is some sample
    output from a Windows CMD.exe shell.  NOTE that my Arduino programming
    port shows up as COM7.  It may be different on your system.

    To enter boot mode, set the baud to 1200 and send anything to the
    programming port:

      C:\Program Files (x86)\Arduino\arduino-1.5.2\hardware\tools>mode com26:1200,n,8,1

      Status for device COM7:
      ------------------------
          Baud:            1200
          Parity:          None
          Data Bits:       8
          Stop Bits:       1
          Timeout:         ON
          XON/XOFF:        OFF
          CTS handshaking: OFF
          DSR handshaking: OFF
          DSR sensitivity: OFF
          DTR circuit:     ON
          RTS circuit:     ON

      C:\Program Files (x86)\Arduino\arduino-1.5.2\hardware\tools>bossac.exe --port=COM7 --usb-port=false -i
          Device       : ATSAM3X8
          Version      : v1.1 Dec 15 2010 19:25:04
          Address      : 0x80000
          Pages        : 2048
          Page Size    : 256 bytes
          Total Size   : 512KB
          Planes       : 2
          Lock Regions : 32
          Locked       : none
          Security     : false
          Boot Flash   : false

  Writing FLASH and Setting FLASH Boot Mode

    In a Cygwin BaSH shell:

      export PATH="/cygdrive/c/Program Files (x86)/Arduino/arduino-1.5.2/hardware/tools":$PATH

    Erasing, writing, and verifying FLASH with bossac:

      $ bossac.exe --port=COM7 --usb-port=false -e -w -v -b nuttx.bin -R
      Erase flash
      Write 86588 bytes to flash
      [==============================] 100% (339/339 pages)
      Verify 86588 bytes of flash
      [==============================] 100% (339/339 pages)
      Verify successful
      Set boot flash true
      CPU reset.

    Some things that can go wrong:

      $ bossac.exe --port=COM7 --usb-port=false -e -w -v -b nuttx.bin -R
      No device found on COM7

    This error means that there is code running on the Flip&Click already
    so the bootloader cannot connect. Press reset and try again

      $ bossac.exe --port=COM7 --usb-port=false -e -w -v -b nuttx.bin -R
      No device found on COM7

    Sill No connection because the board does not jump to bootloader after
    reset.  Set the baud to 1200 and send something then try again

      $ bossac.exe --port=COM7 --usb-port=false -e -w -v -b nuttx.bin -R
      Erase flash
      Write 86588 bytes to flash
      [==============================] 100% (339/339 pages)
      Verify 86588 bytes of flash
      [==============================] 100% (339/339 pages)
      Verify successful
      Set boot flash true
      CPU reset.

  Other useful bossac operations.

    a) Write code to FLASH don't change boot mode and don't reset.  This lets
       you examine the FLASH contents that you just loaded while the bootloader
       is still active.

       $ bossac.exe --port=COM7 --usb-port=false -e -w -v --boot=0 nuttx.bin
       Write 64628 bytes to flash
       [==============================] 100% (253/253 pages)
       Verify 64628 bytes of flash
       [==============================] 100% (253/253 pages)
       Verify successful

    b) Verify the FLASH contents (the bootloader must be running)

       $ bossac.exe --port=COM7 --usb-port=false -v nuttx.bin
       Verify 64628 bytes of flash
       [==============================] 100% (253/253 pages)
       Verify successful

    c) Read from FLASH to a file  (the bootloader must be running):

       $ bossac.exe --port=COM7 --usb-port=false --read=4096 nuttx.dump
       Read 4096 bytes from flash
       [==============================] 100% (16/16 pages)

    d) Change to boot from FLASH

       $ bossac.exe --port=COM7 --usb-port=false --boot=1
       Set boot flash true

  STATUS:
    At present this procedure does not work.  I do the following:

    a) Open TeraTerm, select COM7 at 1200 baud, type a few ENTERs, and
       close teraterm.

    b) Execute the following command which claims to have successfully
       written to FLASH.

       bossac.exe --info --debug --port COM7 --usb-port=0 --erase --write --verify -b nuttx.bin -R

       But the code does not boot.  There is no indication of life.

    c) Repeat a) then

       bossac.exe --info --debug --port COM7 --usb-port=0 --verify -b nuttx.bin

       And it says that the content of the FLASH is not good.

  Uploading NuttX to the Flip&Click Using JTAG
  --------------------------------------------

  The JTAG/SWD signals are brought out to a 10-pin header JTAG connector:

    PIN SIGNAL         JTAG STANDARD     NOTES
    --- -------------- ----------------- --------------------------------
     1  VCC-3.3V       VTref
     2  JTAG_TMS       SWDIO/TMS         SAM3X pin 31, Pulled up on board
     3  GND            GND
     4  JTAG_TCK       SWDCLK/TCK        SAM3X pin 28, Pulled up on board
     5  GND            GND
     6  JTAG_TDO       SWO/EXta/TRACECTL SAM3X pin 30, Pulled up on board
     7  N/C            Key
     8  JTAG_TDI       NC/EXTb/TDI       SAM3X pin 29, Pulled up on board
     9  GND            GNDDetect
    10  MRST           nReset

   NOTE:  The 10-pin JTAG connector is not populated on the Flip&Click
   SAM3X.  This is the part number for the SMD connector recommended by
   ARM.com:  Samtec FTSH-105-01-L-DV-K. For example:

   https://www.digikey.com/product-detail/en/samtec-inc/FTSH-105-01-L-DV-K/SAM8799-ND/1875039

   You should be able to use a 10- to 20-pin adapter to connect a SAM-ICE
   or J-Link debugger to the Flip&Click SAM3X.  I have this Olimex adapter:
   https://www.olimex.com/Products/ARM/JTAG/ARM-JTAG-20-10/ .  I have been
   loading code and debugging with no problems using JTAG.

Flip&Click SAM3X-specific Configuration Options
===============================================

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
    CONFIG_ARCH_CHIP_SAM3X
    CONFIG_ARCH_CHIP_ATSAM3X8E

  CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
  hence, the board that supports the particular chip or SoC.

    CONFIG_ARCH_BOARD=flipnclick-sam3x (for the Flip&Click SAM3X development board)

  CONFIG_ARCH_BOARD_name - For use in C code

    CONFIG_ARCH_BOARD_FLIPNCLICK_SAM3X=y

  CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
  of delay loops

  CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

    CONFIG_RAM_SIZE=65536 (64Kb)

  CONFIG_RAM_START - The start address of installed DRAM

    CONFIG_RAM_START=0x20000000

  CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
  have LEDs

  Individual subsystems can be enabled:

    CONFIG_SAM34_ADC12B      - 12-bit Analog To Digital Converter
    CONFIG_SAM34_CAN0        - CAN Controller 0
    CONFIG_SAM34_CAN1        - CAN Controller 1
    CONFIG_SAM34_DACC        - Digital To Analog Converter
    CONFIG_SAM34_DMAC0       - DMA Controller
    CONFIG_SAM34_EMAC        - Ethernet MAC
    CONFIG_SAM34_HSMCI       - High Speed Multimedia Card Interface
    CONFIG_SAM34_PWM         - Pulse Width Modulation
    CONFIG_SAM34_RTC         - Real Time Clock
    CONFIG_SAM34_RTT         - Real Time Timer
    CONFIG_SAM34_SDRAMC      - SDRAM Controller
    CONFIG_SAM34_SMC         - Static Memory Controller
    CONFIG_SAM34_SPI0        - Serial Peripheral Interface 0
    CONFIG_SAM34_SPI1        - Serial Peripheral Interface 1
    CONFIG_SAM34_SSC         - Synchronous Serial Controller
    CONFIG_SAM34_TC0         - Timer Counter 0
    CONFIG_SAM34_TC1         - Timer Counter 1
    CONFIG_SAM34_TC2         - Timer Counter 2
    CONFIG_SAM34_TC3         - Timer Counter 3
    CONFIG_SAM34_TC4         - Timer Counter 4
    CONFIG_SAM34_TC5         - Timer Counter 5
    CONFIG_SAM34_TC6         - Timer Counter 6
    CONFIG_SAM34_TC7         - Timer Counter 7
    CONFIG_SAM34_TC8         - Timer Counter 8
    CONFIG_SAM34_TRNG        - True Random Number Generator
    CONFIG_SAM34_TWIM/S0     - Two-Wire Interface 0 (master/slave)
    CONFIG_SAM34_TWIM/S1     - Two-Wire Interface 1 (master/slave)
    CONFIG_SAM34_UART0       - UART 0
    CONFIG_SAM34_UOTGHS      - USB OTG High Speed
    CONFIG_SAM34_USART0      - USART 0
    CONFIG_SAM34_USART1      - USART 1
    CONFIG_SAM34_USART2      - USART 2
    CONFIG_SAM34_USART3      - USART 3
    CONFIG_SAM34_WDT         - Watchdog Timer

  Some subsystems can be configured to operate in different ways. The drivers
  need to know how to configure the subsystem.

    CONFIG_SAM34_GPIOA_IRQ
    CONFIG_SAM34_GPIOB_IRQ
    CONFIG_SAM34_GPIOC_IRQ
    CONFIG_SAM34_GPIOD_IRQ
    CONFIG_SAM34_GPIOE_IRQ
    CONFIG_SAM34_GPIOF_IRQ

Configurations
==============

  Each Flip&Click SAM3X configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh [OPTIONS] flipnclick-sam3x:<subdir>

  Where typical options are -l to configure to build on Linux or -c to
  configure for Cygwin under Linux.  'tools/configure.sh -h' will show
  you all of the options.

  Before building, make sure the PATH environment variable includes the
  correct path to the directory than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply,
  nuttx.

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
     output on USART0 which is available either on the Arduion Shield
     connector or on mikroBUS A as described above in the section entitled
     "Serial Consoles".

  3. Unless otherwise stated, the configurations are setup for
     Cygwin under Windows:

     Build Setup:
       CONFIG_HOST_WINDOWS=y   : Microsoft Windows
       CONFIG_WINDIWS_CYGWIN=y : Cygwin under Windows

  3. All of these configurations are set up to build under Windows using the
     "GNU Tools for ARM Embedded Processors" that is maintained by ARM
     (unless stated otherwise in the description of the configuration).

       https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

     That toolchain selection can easily be reconfigured using
     'make menuconfig'.  Here are the relevant current settings:

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABI=y  : GNU ARM EABI toolchain for Windows

Configuration sub-directories
-----------------------------

  nsh:
    This configuration directory will build the NuttShell.  See NOTES above.

    NOTES:
    1. NSH built-in applications are supported.  However, there are
       no built-in applications built with the default configuration.

       Binary Formats:
         CONFIG_BUILTIN=y                    : Enable support for built-in programs

       Application Configuration:
         CONFIG_NSH_BUILTIN_APPS=y           : Enable starting apps from NSH command line

  nxlines

    This is an NSH configuration that supports the NX graphics example at
    apps/examples/nxlines as a built-in application.

    NOTES:

    1. This configuration derives from the nsh configuration.  All of the
       notes there apply here as well.

    2. The default configuration assumes there is the custom HiletGo OLED
       in the mikroBUS B slot (and a Mikroe RS-232 Click card in the
       mikroBUS A slot).  That is easily changed by reconfiguring, however.
       See the section entitled "HiletGo OLED" for information about this
       custom click card.

  STATUS:
    2018-02-11:  No complaints from the software, but nothing appears on the
      OLED.  There is, most likely, an error in my custom HiletGo Click.  Damn!
