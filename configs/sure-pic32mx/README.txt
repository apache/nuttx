configs/pic32mx README
=====================

This README file discusses the port of NuttX to the "Advanced USB Storage
Demo Board," Model DB-DP11215, from Sure Electronics
(http://www.sureelectronics.net/).  This board features the MicroChip
PIC32MX440F512H.  See also http://www.sureelectronics.net/goods.php?id=1168
for further information about the Sure DB-DP11215 board.

DB_DP11215 PIC32 Storage Demo Board

  - PIC32MX44F512H
  - SD card slot
  - RS-2323 Interface
  - USB (MINI-B)
  - 2x16 LCD display
  - Three tactile switches
  - Four user LEDs

Also available (but not yet supported).

DB-DP11212 PIC32 General Purpose Demo Board

  - PIC32MX44F512H
  - LM75A temperature sensor and temperature resistor (NTC-SMD thermistor)
  - SPI FLASH: AT25DF041A
  - USB (MINI-B)
  - 2x16 LCD display
  - 4 digit, 8 segment LED display
  - Three tactile switches
  - Four user LEDs

NOTE:  I see that Sure Electronics shows both of these boards at end-of-Life
(EOL).  So I assume that these boards will no longer be generally available.
This work should still be useful, however, for other PIC32MX4-based boards
(2012-5-27).

Contents
========

  PIC32MX440F512H Pin Out
  Toolchains
  Loading NuttX with PICkit2
  LCD1602
  PIC32MX Configuration Options
  Configurations

PIC32MX440F512H Pin Out
=======================

  DB_DP11215 PIC32 Storage Demo Board
  -----------------------------------
  PIC32MX440F512H 64-Pin QFN (USB) Pin Out as used on the DB_DP11215 PIC32 Storage
  Demo Board.

  LEFT SIDE, TOP-TO-BOTTOM (if pin 1 is in upper left)
  PIN  NAME                          SIGNAL         NOTES
  ---- ----------------------------- -------------- -------------------------------
    1  PMD5/RE5                      PMPD5          Display, JP1-12, DB4
    2  PMD6/RE6                      PMPD6          Display, JP1-13, DB6
    3  PMD7/RE7                      PMPD7          Display, JP1-14, DB7
    4  SCK2/PMA5/CN8/RG6             SCK            SD connector SCK, FLASH (U1) SCK*
    5  SDI2/PMA4/CN9/RG7             SDI            SD connector DO, FLASH (U1) SO*
    6  SDO2/PMA3/CN10/RG8            SDO            SD connector DI, FLASH (U1) SI*
    7  MCLR\                         PIC_MCLR       Pulled high, J7-1, ICSP
    8  SS2/PMA2/CN11/RG9             UTIL_CS        FLASH (U1) CS*
    9  Vss                                          Grounded
   10  Vdd                           +3.3V          ---
   11  AN5/C1IN+/Vbuson/CN7/RB5      Vbuson/AN5/RB5 To USB VBUS circuitry
   12  AN4/C1IN-/CN6/RB4             SW_OK          SW3, Pull high, low means SW3 closed
   13  AN3/C2IN+/CN5/RB3             SW_UP          SW1, Pull high, low means SW1 closed
   14  AN2/C2IN-/CN4/RB2             SW_Down        SW2, Pull high, low means SW2 closed
   15  PGEC1/AN1/Vref-/CVref-/CN3/   ADC_SENSE_SWITCHED_+VBUS To USB VBUS circuitry
       RB1
   16  PGED1/AN0/VREF+/CVREF+/PMA6/                     N/C            Not connected
       CN2/RB0

  *FLASH (U1, SOIC) not populated

  BOTTOM SIDE, LEFT-TO-RIGHT (if pin 1 is in upper left)
  PIN  NAME                          SIGNAL         NOTES
  ---- ----------------------------- -------------- -------------------------------
   17  PGEC2/AN6/OCFA/RB6            PIC_PGC2       J7-5, ICSP
   18  PGED2/AN7/RB7                 PIC_PGD2       J7-4, ICSP
   19  AVdd                          +3.3V          ---
   20  AVss                                         Grounded
   21  AN8/U2CTS/C1OUT/RB8           N/C            Not connected
   22  AN9/C2OUT/PMA7/RB9            N/C            Not connected
   23  TMS/AN10/CVREFOUT/PMA13/RB10  UTIL_WP        FLASH (U1) WP*
   24  TDO/AN11/PMA12/RB11           SD_CS          SD connector CS
   25  Vss                                          Grounded
   26  Vdd                           +3.3V          ---
   27  TCK/AN12/PMA11/RB12           SD_CD          SD connector CD
   28  TDI/AN13/PMA10/RB13           SD_WD          SD connector WD
   29  AN14/U2RTS/PMALH/PMA1/RB14    N/C            Not connected
   30  AN15/OCFB/PMALL/PMA0/CN12/    PMPA0          Display, JP1-4, RS
       RB15
   31  SDA2/U2RX/PMA9/CN17/RF4       RXD2_MCU       J5 DB9 via RS232 driver
   32  SCL2/U2TX/PMA8/CN18/RF5       TXD2_MCU       J5 DB9 via RS232 driver

  *FLASH (U1, SOIC) not populated

  RIGHT SIDE, TOP-TO-BOTTOM (if pin 1 is in upper left)
  PIN  NAME                          SIGNAL         NOTES
  ---- ----------------------------- -------------- -------------------------------
   48  SOSCO/T1CK/CN0/RC14           SOSCO          32.768KHz XTAL (Y1)
   47  SOSCI/CN1/RC13                SOSCI          32.768KHz XTAL (Y1)
   46  OC1/INT0/RD0                  PWM1           Used to control backlight level (K)
   45  IC4/PMCS1/PMA14/INT4/RD11     PMPCS1         Display, JP1-6, E
   44  SCL1/IC3/PMCS2/PMA15/INT3/    USB_OPT        USB PHY
       RD10
   43  U1CTS/SDA1/IC2/INT2/RD9       USB_OPTEN      USB PHY
   42  RTCC/IC1/INT1/RD8             N/C            Not connected
   41  Vss                                          Grounded
   40  OSC2/CLKO/RC15                OSC2           20MHz XTAL (Y2)
   39  OSC1/CLKI/RC12                OSC1           20MHz XTAL (Y2)
   38  Vdd                           +3.3V          ---
   37  D+/RG2                        APPS_D+        USB connectors via PHY
   36  D-/RG3                        APPS_D-        USB connectors via PHY
   35  Vusb                          +3.3V          ---
   34  Vbus                          VBUS_DEVICE_MODE Display, USB Mini-B, USB Type A, JP1-1, +5V
   33  USBID/RF3                     N/C            Not connected

  TOP SIDE, LEFT-TO-RIGHT (if pin 1 is in upper left)
  PIN  NAME                          SIGNAL         NOTES
  ---- ----------------------------- -------------- -------------------------------
   64  PMPD4/RE4                     PMPD4          Display, JP1-11, DB4
   63  PMPD3/RE3                     PMPD3          Display, JP1-10, DB3
   62  PMPD2/RE2                     PMPD2          Display, JP1-9, DB2
   61  PMPD1/RE1                     PMPD1          Display, JP1-8, DB1
   60  PMPD0/RE0                     PMPD0          Display, JP1-7, DB0
   59  RF1                           RF1            Low illuminates LED/R/ERR
   58  RF0                           RF0            Low illuminates LED/Y/flash
   57  ENVREG                        ENVREG         Pulled high
   56  Vcap/Vddcore                  VDDCORE        Capactors to ground
   55  CN16/RD7                      RD7            Low illuminates LED/Y/USB
   54  CN15/RD6                      RD6            Low illuminates LED/Y/SD
   53  PMRD/CN14/RD5                 PMPRD          Display, JP1-5, R/W
   52  OC5/IC5/PMWR/CN13/RD4         N/C            Not connected
   51  U1TX/OC4/RD3                  CP2102_RXD     J6-3, UART1 (also CP2102*)
   50  U1RX/OC3/RD2                  CP2102_TXD     J6-2, UART1 (also CP2102*)
   49  U1RTS/OC2/RD1                 PWM2           Used to control backlight level (Vo)

  *USB-to-UART bridge (U1, CP2102) not populated

  DB-DP11212 PIC32 General Purpose Demo Board
  -------------------------------------------
  PIC32MX440F512H 64-Pin QFN (USB) Pin Out as used on the DB-DP11212 PIC32 General
  Purpose Demo Board

  LEFT SIDE, TOP-TO-BOTTOM (if pin 1 is in upper left)
  PIN  NAME                          SIGNAL         NOTES
  ---- ----------------------------- -------------- -------------------------------
    1  PMD5/RE5                      PMPD5          Display, JP1-12, DB5
    2  PMD6/RE6                      PMPD6          Display, JP1-13, DB6
    3  PMD7/RE7                      PMPD7          Display, JP1-14, DB7
    4  SCK2/PMA5/CN8/RG6             SCK            FLASH (U4) SCK*
    5  SDI2/PMA4/CN9/RG7             SDI            FLASH (U4) SO*
    6  SDO2/PMA3/CN10/RG8            SDO            FLASH (U4) SI*
    7  MCLR\                         PIC_MCLR       Pulled high, J2-1, ICSP
    8  SS2/PMA2/CN11/RG9             N/C            Not connected
    9  Vss                                          Grounded
   10  Vdd                           +3.3V          ---
   11  Vbuson/AN5/CN7/RB5            RB5            LCD SEG5 (F), U5-10
   12  AN4/CN6/RB4                   RB4            LCD SEG4 (E), U5-1
   13  AN3/CN5/RB3                   RB3            LCD SEG3 (D), U5-2
   14  AN2/CN4/RB2                   RB2            LCD SEG2 (C), U5-4
   15  PGEC1/AN1/Vref-/CN3/RB1       RB1            LCD SEG1 (B), U5-7
   16  PGED1/AN0/VREF+/CVREF+/PMA6/  RB0            LCD SEG0 (A), U5-11
       CN2/RB0

  *FLASH (U4, SOIC) not populated

  BOTTOM SIDE, LEFT-TO-RIGHT (if pin 1 is in upper left)
  PIN  NAME                          SIGNAL         NOTES
  ---- ----------------------------- -------------- -------------------------------
   17  PGEC2/AN6/OCFA/RB6            PIC_PGC2       J2-5, ICSP
   18  PGED2/AN7/RB7                 PIC_PGD2       J2-4, ICSP
   19  AVdd                          +3.3V          ---
   20  AVss                                         Grounded
   21  AN8/U2CTS/RB8                 RB8            LCD SEG6 (G), U5-5
   22  AN9/PMA7/RB9                  RB9            LCD SEG7 (DP), U5-3
   23  TMS/AN10/PMA13/RB10           UTIL_WP        FLASH (U4) WP*
   24  TDO/AN11/PMA12/RB11           UTIL_CS        FLASH (U4) CS*
   25  Vss                                          Grounded
   26  Vdd                           +3.3V          ---
   27  TCK/AN12/PMA11/RB12           N/C            Not connected
   28  TDI/AN13/PMA10/RB13           N/C            Not connected
   29  AN14/U2RTS/PMA1/RB14          temp_AD        temp_AD
   30  AN15/PMA0/CN12/RB15           PMPA0          Display, JP1-4, RS
   31  SDA2/U2RX/PMA9/CN17/RF4       SDA            LM75/SO, U3-1, SDA
   32  SCL2/U2TX/PMA8/CN18/RF5       SCL            LM75/SO, U3-2, SCL

  *FLASH (U4, SOIC) not populated

  RIGHT SIDE, TOP-TO-BOTTOM (if pin 1 is in upper left)
  PIN  NAME                          SIGNAL         NOTES
  ---- ----------------------------- -------------- -------------------------------
   48  SOSCO/T1CK/CN0/RC14           SOSCO          32.768KHz XTAL (Y1)
   47  SOSCI/CN1/RC13                SOSCI          32.768KHz XTAL (Y1)
   46  OC1/INT0/RD0                  RD0            LCD DIG1, U5-12
   45  IC4/PMCS1/PMA14/RD11          PMCS1          Display, JP1-6, E
   44  SCL1/PMCS2/PMA15              RD10           LCD DIG2, U5-9
   43  SDA1/RD9                      RD9            LCD DIG3, U5-8
   42  RTCC/RD8                      RD8            LCD DIG4, U5-6
   41  Vss                                          Grounded
   40  OSC2/CLKO/RC15                OSC2           20MHz XTAL (Y2)
   39  OSC1/CLKI/RC12                OSC1           20MHz XTAL (Y2)
   38  Vdd                           +3.3V          ---
   37  D+                            MCU_D+         USB connectors via PHY
   36  D-                            MCU_D-         USB connectors via PHY
   35  Vusb                          +3.3V          ---
   34  Vbus                          +5V_DUSB       Display, USB Mini-B, USB Type A, JP1-1, +5V
   33  USBID/RF3                     N/C            Not connected

  TOP SIDE, LEFT-TO-RIGHT (if pin 1 is in upper left)
  PIN  NAME                          SIGNAL         NOTES
  ---- ----------------------------- -------------- -------------------------------
   64  PMPD4/RD4                     PMPD4          Display, JP1-11, DB4
   63  PMPD3/RD3                     PMPD3          Display, JP1-10, DB3
   62  PMPD2/RD2                     PMPD2          Display, JP1-9, DB2
   61  PMPD1/RD1                     PMPD1          Display, JP1-8, DB1
   60  PMPD0/RE0                     PMPD0          Display, JP1-7, DB0
   59  RF1                           Key3           SW3-1
   58  RF0                           Key2           SW2-1
   57  ENVREG                        ENVREG         Pulled high
   56  Vcap/Vddcore                  VDDCORE        Capacitors to ground
   55  CN16/RD7                      N/C            Not connected
   54  CN15/RD6                      Key5           SW5-1
   53  PMRD/CN14/RD5                 PMPRD          ---
   52  OC5/PMWR/CN13/RD4             PWM2           Used to control backlight level (Vo)
   51  U1TX/OC4/RD3                  N/C            Not connected
   50  U1RX/OC3/RD2                  N/C            Not connected
   49  OC2/RD1                       PWM1           Used to control backlight level (K)

Toolchains
==========

  MPLAB/C32
  ---------

  I am using the free, "Lite" version of the PIC32MX toolchain available
  for download from the microchip.com web site.  I am using the Windows
  version.  The MicroChip toolchain is the only toolchain currently
  supported in these configurations, but it should be a simple matter to
  adapt to other toolchains by modifying the Make.defs file include in
  each configuration.

  C32 Toolchain Options:

    CONFIG_MIPS32_TOOLCHAIN_MICROCHIPW      - MicroChip full toolchain for Windows
    CONFIG_MIPS32_TOOLCHAIN_MICROCHIPL      - MicroChip full toolchain for Linux
    CONFIG_MIPS32_TOOLCHAIN_MICROCHIPW_LITE - MicroChip "Lite" toolchain for Windows
    CONFIG_MIPS32_TOOLCHAIN_MICROCHIPL_LITE - MicroChip "Lite" toolchain for Linux
    CONFIG_MIPS32_TOOLCHAIN_PINGUINOL       - Pinquino toolchain for Linux
    CONFIG_MIPS32_TOOLCHAIN_PINGUINOW       - Pinquino toolchain for Windows
    CONFIG_MIPS32_TOOLCHAIN_MICROCHIPOPENL  - Microchip open toolchain for Linux
    CONFIG_MIPS32_TOOLCHAIN_GNU_ELF         - General mips-elf toolchain for Linux

  NOTE:  The "Lite" versions of the toolchain does not support C++.  Also
  certain optimization levels are not supported by the "Lite" toolchain.

  MicrochipOpen
  -------------

  An alternative, build-it-yourself toolchain is available here:
  http://sourceforge.net/projects/microchipopen/ .  These tools were
  last updated circa 2010.  NOTE:  C++ support still not available
  in this toolchain.

  Building MicrochipOpen (on Linux)

  1) Get the build script from this location:

      http://microchipopen.svn.sourceforge.net/viewvc/microchipopen/ccompiler4pic32/buildscripts/trunk/

  2) Build the code using the build script, for example:

      ./build.sh -b v105_freeze

     This will check out the selected branch and build the tools.

  3) Binaries will then be available in a subdirectory with a name something like
     pic32-v105-freeze-20120622/install-image/bin (depending on the current data
     and the branch that you selected.

     Note that the tools will have the prefix, mypic32- so, for example, the
     compiler will be called mypic32-gcc.

  Penguino mips-elf Toolchain
  ---------------------------

  Another option is the mips-elf toolchain used with the Penguino project.  This
  is a relatively current mips-elf GCC and should provide free C++ support as
  well. This toolchain can be downloded from the Penguino website:
  http://wiki.pinguino.cc/index.php/Main_Page#Download .

  See also configs/mirtoo/README.txt.  There is an experimental (untested)
  configuration for the Mirtoo platform in that directory.

  MPLAB/C32 vs MPLABX/X32
  -----------------------

  It appears that Microchip is phasing out the MPLAB/C32 toolchain and replacing
  it with MPLABX and XC32.  At present, the XC32 toolchain is *not* compatible
  with the NuttX build scripts.  Here are some of the issues that I see when trying
  to build with XC32:

  1) Make.def changes:  You have to change the tool prefix:

     CROSSDEV=xc32-

  2) debug.ld/release.ld:  The like expect some things that are not present in
     the current linker scripts (or are expected with different names).  Here
     are some partial fixes:

     Rename:  kseg0_progmem to kseg0_program_mem
     Rename:  kseg1_datamem to kseg1_data_mem

  Even then, there are more warnings from the linker and some undefined symbols
  for non-NuttX code that resides in the unused Microchip libraries.  You will
  have to solve at least this undefined symbol problem if you want to used the
  XC32 toolchain.

  Windows Native Toolchains
  -------------------------

  NOTE:  There are several limitations to using a Windows based toolchain in a
  Cygwin environment.  The three biggest are:

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

Loading NuttX with PICkit2
==========================

  NOTE:  You need a PICKit3 if you plan to use the MPLAB debugger!  The PICKit2
  can, however, still be used to load programs.  Instructions for the PICKit3
  are similar.

  Intel Hex Forma Files:
  ----------------------

    When NuttX is built it will produce two files in the top-level NuttX
    directory:

    1) nuttx - This is an ELF file, and
    2) nuttx.hex - This is an Intel Hex format file.  This is controlled by
       the setting CONFIG_INTELHEX_BINARY in the .config file.

    The PICkit tool wants an Intel Hex format file to burn into FLASH. However,
    there is a problem with the generated nutt.hex: The tool expects the nuttx.hex
    file to contain physical addresses.  But the nuttx.hex file generated from the
    top-level make will have address in the KSEG0 and KSEG1 regions.

  tools/pic32mx/mkpichex:
  ----------------------

    There is a simple tool in the NuttX tools/pic32mx directory that can be
    used to solve both issues with the nuttx.hex file.  But, first, you must
    build the tool:

      cd tools/pic32mx
      make

    Now you will have an excecutable file call mkpichex (or mkpichex.exe on
    Cygwin).  This program will take the nutt.hex file as an input, it will
    convert all of the KSEG0 and KSEG1 addresses to physical address, and
    it will write the modified file, replacing the original nuttx.hex.

    To use this file, you need to do the following things:

                       # Add the NuttX tools/pic32mx directory to your
                       # PATH variable
      make             # Build nuttx and nuttx.hex
      mkpichex $PWD    # Convert addresses in nuttx.hex.  $PWD is the path
                       # to the top-level build directory.  It is the only
                       # required input to mkpichex.

LCD1602
=======

  The on-board LCD is a 2x16 segment LCD and appears to be compatible with
  the LCD1602 and is like an LCD1602 LCD here.

  LCD pin mapping (see configs/pcblogic-pic32mx/README.txt)

    --------------------- ---------- ----------------------------------
    PIC32                  Sure JP1   Sure Signal Description
    PIN  SIGNAL NAME      PIN NAME(s)
    --------------------- ---------- ----------------------------------
     34  Vbus             1.  +5V    +5V VBUS device mode
                                      To GND via capacitor
                          2.  GND    GND
     49  RD1              3.  Vo     Transistor circuit driven by PWM2
     44  PMA0/AN15/RB15   4.  RS     PMA0, Selects registers
     53  PMRD/RD5         5.  RW     PMRD/PMWR, Selects read or write
     45  PMPCS1/RD11      6.  E      Starts data read/write
     60  PMD0/RE0         7.  DB0    PMD0
     61  PMD1/RE1         8.  DB1    PMD1
     62  PMD2/RE2         9.  DB2    PMD2
     63  PMD3/RE3         10. DB3    PMD3
     64  PMD4/RE4         11. DB4    PMD4
      1  PMD5/RE5         12. DB5    PMD5
      2  PMD6/RE6         13. DB6    PMD6
      3  PMD7/RE7         14. DB7    PMD7
                          15. A      +5V_DUSB
     46 INT0/RD0          16. K      Transistor circuit driven by PWM1
    --------------------- ---------- ----------------------------------

    Vbus power also requires Vbuson/AN5/RB5

PIC32MX Configuration Options
=============================

  General Architecture Settings:

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
     be set to:

       CONFIG_ARCH=mips

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_MIPS=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_MIPS32=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=pic32mx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_PIC32MX440F512H=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=sure-pic32mx

    CONFIG_ARCH_DBDP11215 Distinguishes the DB_DP11215 PIC32 Storage
      Demo Board

    CONFIG_ARCH_DBDP11212 Distingustes the DB-DP11212 PIC32 General
      Purpose Demo Board

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_SUREPIC32MX=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (CPU SRAM in this case):

       CONFIG_RAM_SIZE=(32*1024) (32Kb)

       There is an additional 32Kb of SRAM in AHB SRAM banks 0 and 1.

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0xa0000000

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
       stack in bytes.  If not defined, the user task stacks will be
       used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    PIC32MX Configuration

      CONFIG_PIC32MX_MVEC - Select muli- vs. single-vectored interrupts

    Individual subsystems can be enabled:

       CONFIG_PIC32MX_WDT            - Watchdog timer
       CONFIG_PIC32MX_T2             - Timer 2 (Timer 1 is the system time and always enabled)
       CONFIG_PIC32MX_T3             - Timer 3
       CONFIG_PIC32MX_T4             - Timer 4
       CONFIG_PIC32MX_T5             - Timer 5
       CONFIG_PIC32MX_IC1            - Input Capture 1
       CONFIG_PIC32MX_IC2            - Input Capture 2
       CONFIG_PIC32MX_IC3            - Input Capture 3
       CONFIG_PIC32MX_IC4            - Input Capture 4
       CONFIG_PIC32MX_IC5            - Input Capture 5
       CONFIG_PIC32MX_OC1            - Output Compare 1
       CONFIG_PIC32MX_OC2            - Output Compare 2
       CONFIG_PIC32MX_OC3            - Output Compare 3
       CONFIG_PIC32MX_OC4            - Output Compare 4
       CONFIG_PIC32MX_OC5            - Output Compare 5
       CONFIG_PIC32MX_I2C1           - I2C 1
       CONFIG_PIC32MX_I2C2           - I2C 2
       CONFIG_PIC32MX_SPI2           - SPI 2
       CONFIG_PIC32MX_UART1          - UART 1
       CONFIG_PIC32MX_UART2          - UART 2
       CONFIG_PIC32MX_ADC            - ADC 1
       CONFIG_PIC32MX_PMP            - Parallel Master Port
       CONFIG_PIC32MX_CM1            - Comparator 1
       CONFIG_PIC32MX_CM2            - Comparator 2
       CONFIG_PIC32MX_RTCC           - Real-Time Clock and Calendar
       CONFIG_PIC32MX_DMA            - DMA
       CONFIG_PIC32MX_FLASH          - FLASH
       CONFIG_PIC32MX_USBDEV         - USB device
       CONFIG_PIC32MX_USBHOST        - USB host

    PIC32MX Configuration Settings
    DEVCFG0:
      CONFIG_PIC32MX_DEBUGGER - Background Debugger Enable. Default 3 (disabled). The
        value 2 enables.
      CONFIG_PIC32MX_ICESEL - In-Circuit Emulator/Debugger Communication Channel Select
        Default 1 (PG2)
      CONFIG_PIC32MX_PROGFLASHWP  - Program FLASH write protect.  Default 0xff (disabled)
      CONFIG_PIC32MX_BOOTFLASHWP - Default 1 (disabled)
      CONFIG_PIC32MX_CODEWP - Default 1 (disabled)
    DEVCFG1: (All settings determined by selections in board.h)
    DEVCFG2: (All settings determined by selections in board.h)
    DEVCFG3:
      CONFIG_PIC32MX_USBIDO - USB USBID Selection.  Default 1 if USB enabled
        (USBID pin is controlled by the USB module), but 0 (GPIO) otherwise.
      CONFIG_PIC32MX_VBUSIO - USB VBUSON Selection (Default 1 if USB enabled
        (VBUSON pin is controlled by the USB module, but 0 (GPIO) otherwise.
      CONFIG_PIC32MX_WDENABLE - Enabled watchdog on power up.  Default 0 (watchdog
        can be enabled later by software).

    The priority of interrupts may be specified.  The value ranage of
    priority is 4-31. The default (16) will be used if these any of these
    are undefined.

       CONFIG_PIC32MX_CTPRIO         - Core Timer Interrupt
       CONFIG_PIC32MX_CS0PRIO        - Core Software Interrupt 0
       CONFIG_PIC32MX_CS1PRIO        - Core Software Interrupt 1
       CONFIG_PIC32MX_INT0PRIO       - External Interrupt 0
       CONFIG_PIC32MX_INT1PRIO       - External Interrupt 1
       CONFIG_PIC32MX_INT2PRIO       - External Interrupt 2
       CONFIG_PIC32MX_INT3PRIO       - External Interrupt 3
       CONFIG_PIC32MX_INT4PRIO       - External Interrupt 4
       CONFIG_PIC32MX_FSCMPRIO       - Fail-Safe Clock Monitor
       CONFIG_PIC32MX_T1PRIO         - Timer 1 (System timer) priority
       CONFIG_PIC32MX_T2PRIO         - Timer 2 priority
       CONFIG_PIC32MX_T3PRIO         - Timer 3 priority
       CONFIG_PIC32MX_T4PRIO         - Timer 4 priority
       CONFIG_PIC32MX_T5PRIO         - Timer 5 priority
       CONFIG_PIC32MX_IC1PRIO        - Input Capture 1
       CONFIG_PIC32MX_IC2PRIO        - Input Capture 2
       CONFIG_PIC32MX_IC3PRIO        - Input Capture 3
       CONFIG_PIC32MX_IC4PRIO        - Input Capture 4
       CONFIG_PIC32MX_IC5PRIO        - Input Capture 5
       CONFIG_PIC32MX_OC1PRIO        - Output Compare 1
       CONFIG_PIC32MX_OC2PRIO        - Output Compare 2
       CONFIG_PIC32MX_OC3PRIO        - Output Compare 3
       CONFIG_PIC32MX_OC4PRIO        - Output Compare 4
       CONFIG_PIC32MX_OC5PRIO        - Output Compare 5
       CONFIG_PIC32MX_I2C1PRIO       - I2C 1
       CONFIG_PIC32MX_I2C2PRIO       - I2C 2
       CONFIG_PIC32MX_SPI2PRIO       - SPI 2
       CONFIG_PIC32MX_UART1PRIO      - UART 1
       CONFIG_PIC32MX_UART2PRIO      - UART 2
       CONFIG_PIC32MX_CN             - Input Change Interrupt
       CONFIG_PIC32MX_ADCPRIO        - ADC1 Convert Done
       CONFIG_PIC32MX_PMPPRIO        - Parallel Master Port
       CONFIG_PIC32MX_CM1PRIO        - Comparator 1
       CONFIG_PIC32MX_CM2PRIO        - Comparator 2
       CONFIG_PIC32MX_FSCMPRIO       - Fail-Safe Clock Monitor
       CONFIG_PIC32MX_RTCCPRIO       - Real-Time Clock and Calendar
       CONFIG_PIC32MX_DMA0PRIO       - DMA Channel 0
       CONFIG_PIC32MX_DMA1PRIO       - DMA Channel 1
       CONFIG_PIC32MX_DMA2PRIO       - DMA Channel 2
       CONFIG_PIC32MX_DMA3PRIO       - DMA Channel 3
       CONFIG_PIC32MX_FCEPRIO        - Flash Control Event
       CONFIG_PIC32MX_USBPRIO        - USB

  PIC32MXx specific device driver settings.  NOTE:  For the Sure board,
  UART2 is brought out to the DB9 connector and serves as the serial
  console.

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

  PIC32MXx USB Device Configuration

  PIC32MXx USB Host Configuration (the PIC32MX does not support USB Host)

Configurations
==============

  Each PIC32MX configuration is maintained in a sub-directory and can be
  selected as follow:

    tools/configure.sh sure-pic32mx/<subdir>

  Where <subdir> is one of the following sub-directories.

  NOTE:  These configurations use the mconf-based configuration tool.  To
  change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

Configuration sub-directories
-----------------------------

Where <subdir> is one of the following:

  nsh:
  ====
    Description.
    ------------
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables only the serial NSH interface.

    Notes.
    -----
    1. By default, this configuration uses an older Microchip C32 toolchain
       for Windows (the newer ones seem to be incompatible) and builds under
       Cygwin (or probably MSYS).  That can easily be reconfigured, of course.

       Build Setup:
         CONFIG_HOST_WINDOWS=y    : Builds under Windows
         CONFIG_WINDOWS_CYGWIN=y  : Using Cygwin

       System Type:
         CONFIG_MIPS32_TOOLCHAIN_MICROCHIPW_LITE=y : Older C32 toolchain

    2. USB Configuations.

      Several USB device configurations can be enabled and included
      as NSH built-in built in functions.  All require the following
      basic setup in your .config to enable USB device support:

        Drivers:
          CONFIG_USBDEV=y           : Enable basic USB device support

       System Type -> PIC32MX Peripheral Support:
          CONFIG_PIC32MX_USBDEV=y   : Enable PIC32 USB device support

      system/cdcacm -  The system/cdcacm program can be included as an
      function by dding the following to the NuttX configuration file:

        Application Configuration->Examples:
          CONFIG_SYSTEM_CDCACM=y  : Select apps/system/cdcacm

      and defining the following in your .config file:

        Drivers->USB Device Driver Support
          CONFIG_CDCACM=y           : Enable the CDCACM device

      system/usbmsc - To enable the USB mass storage class (MSC)device,
      you would need to add the following to the NuttX configuration file.
      However, this device cannot work until support for the SD card is
      also incorporated.

        Drivers->USB Device Driver Support
          CONFIG_USBMSC=y           : Enables the USB MSC class

        Application Configuration->Examples:
          CONFIG_SYSTEM_USBMSC=y  : Enhables apps/system/usbmsc

    3. SD Card Support.

      Support for the on-board, SPI-based SD card is available but is
      not yet functional (at least at the time of this writing).  SD
      card support can be enabled for testing by simply enabling SPI2
      support in the configuration file:

       System Type -> PIC32MX Peripheral Support:
         CONFIG_PIC32MX_SPI2=y      : Enable SPI2

       Drivers:
         CONFIG_MMCSD=y             : MMC/SD support
         CONFIG_MMCSD_SPI=y         : SPI-based MMC/SD support

       File Systems:
         CONFIG_FS_FAT=y            : FAT file system
                                    : Other FAT options

    Debug output for testing the SD card can be enabled using:

       Build Setup:
         CONFIG_DEBUG_FEATURES=y    : Enable debug features
         CONFIG_DEBUG_INFO=y        : Enable verbose debug output
         CONFIG_DEBUG_FS=y          : Enable file system debug
         CONFIG_DEBUG_SPI=y         : Enable SPI debug

    4. To enable LCD1602 support:

       Device Drivers ->LCD Driver Support:
         CONFIG_LCD=y               : Enable LCD menus
         CONFIG_LCD_LCD1602=y       : Select LCD1602
         CONFIG_LCD_MAXCONTRAST=255 : (Or any large-ish value that you prefer)
         CONFIG_LCD_MAXPOWER=255    : (Or any large-ish value that you prefer)

       Library Routines:
         CONFIG_LIB_SLCDCODEC=y     : Enable the SLCD CODEC

       NOTE that is is not necessary to select the PMP peripheral; this LCD
       driver is a bit-bang driver that just happens to use the PMP pins as
       GPIOS.

       To enable apps/examples/slcd to test the LCD:

       Application Configuration -> NSH Library:
         CONFIG_NSH_ARCHINIT=y      : Needed to initialize the SLCD

       Application Configuration -> Examples:
         CONFIG_EXAMPLES_SLCD=y     : Enable apps/examples/slcd use /dev/lcd1602
         CONFIG_EXAMPLES_SLCD_DEVNAME="/dev/lcd1602"

       To enable LCD debug output:

       Build Setup -> Debug Options:
         CONFIG_DEBUG_FEATURES=y             : Enable debug features
         CONFIG_DEBUG_INFO=y     : Enable verbose debug output
         CONFIG_DEBUG_LCD=y         : Enable LCD debug output

       NOTES:
       2013-05-27: The LCD1602 has been verified on the DB-DP11212 using
         this configuration.  It has not been used with the usbnsh configuration
         or with the DB-11112 board.  It looks to me like the connection to the
         LCD1602 is identical on the DB-11112 and so I would expect that to work.

         At this point in time, testing of the SLCD is very limited because
         there is not much in apps/examples/slcd.  Basically  driver with a working
         test setup and ready to be tested and debugged.

  usbnsh:
  =======
    Description.
    ------------
    This is another NSH example.  If differs from the 'nsh' configuration
    above in that this configurations uses a USB serial device for console
    I/O.  This configuration was created to support the "DB-DP11212 PIC32
    General Purpose Demo Board" which has no easily accessible serial port.
    However, as of this writing, the configuration has set for the
    "DB_DP11215 PIC32 Storage Demo Board" and has only be testing on that
    board.

    Notes.
    -----
    1. By default, this configuration uses an older Microchip C32 toolchain
       for Windows (the newer ones seem to be incompatible) and builds under
       Cygwin (or probably MSYS).  That can easily be reconfigured, of course.

       Build Setup:
         CONFIG_HOST_WINDOWS=y      : Builds under Windows
         CONFIG_WINDOWS_CYGWIN=y    : Using Cygwin

       System Type:
         CONFIG_MIPS32_TOOLCHAIN_MICROCHIPW_LITE=y : Older C32 toolchain

    2. Comparison to nsh

      Below summarizes the key configuration differences between the 'nsh'
      and the 'upnsh' configurations:

        CONFIG_USBDEV=y               : NuttX USB device support is enabled
        CONFIG_PIC32MX_USBDEV=y       : The PIC32MX USB device driver is built
        CONFIG_DEV_CONSOLE=n          : /dev/console does not exist on power up
        CONFIG_UART1_SERIAL_CONSOLE=n : There is no serial console
        CONFIG_UART2_SERIAL_CONSOLE=n :
        CONFIG_CDCACM=y               : The CDC/ACM serial device class is enabled
        CONFIG_CDCACM_CONSOLE=y       : The CDC/ACM serial device is the console

    3. Using the Prolifics PL2303 Emulation

      You could also use the non-standard PL2303 serial device instead of
      the standard CDC/ACM serial device by changing:

        Drivers->USB Device Driver Support
          CONFIG_CDCACM=n             : Disable the CDC/ACM serial device class
          CONFIG_CDCACM_CONSOLE=n     : The CDC/ACM serial device is NOT the console
          CONFIG_PL2303=y             : The Prolifics PL2303 emulation is enabled
          CONFIG_PL2303_CONSOLE=y     : The PL2303 serial device is the console

      Why would you want to use a non-standard USB serial driver?  You might
      to use the PL2303 driver with a Windows host because it should
      automatically install the PL2303 driver (you might have to go through
      some effort to get Windows to recognize the CDC/ACM device).

    4. Since this configuration is current set for the "DB_DP11215 PIC32
       Storage Demo Board," UART2 is available and is configured to used as
       the SYSLOG device.  That means that all debug output will be directed
       out UART2.  Debug output is not enabled by default, however, so these
       settings do nothing until you enable debug ouput.

        Device Drivers -> System Logging Device Options:
          CONFIG_SYSLOG_CHAR=y
          CONFIG_SYSLOG_DEVPATH="/dev/ttyS0"

        System Type -> PIC32MX Peripheral Support:
          CONFIG_PIC32MX_UART2=y      : Enable UART2

        Device Drivers -> Serial Driver Support:
          CONFIG_UART2_2STOP=0        : UART2 configuration
          CONFIG_UART2_BAUD=115200
          CONFIG_UART2_BITS=8
          CONFIG_UART2_PARITY=0
          CONFIG_UART2_RXBUFSIZE=64
          CONFIG_UART2_TXBUFSIZE=64

       NOTE:  Using the SYSLOG to get debug output has limitations.  Among
       those are that you cannot get debug output from interrupt handlers.
       So, in particularly, debug output is not a useful way to debug the
       USB device controller driver.  Instead, use the USB monitor with
       USB debug off and USB trance on (see below).

    5. Enabling USB monitor SYSLOG output.  If tracing is enabled, the USB
       device will save encoded trace output in in-memory buffer; if the
       USB monitor is enabled, that trace buffer will be periodically
       emptied and dumped to the system logging device (UART2 in this
       configuration):

        Device Drivers -> "USB Device Driver Support:
          CONFIG_USBDEV_TRACE=y                   : Enable USB trace feature
          CONFIG_USBDEV_TRACE_NRECORDS=256        : Buffer 256 records in memory

        Application Configuration -> NSH LIbrary:
          CONFIG_NSH_USBDEV_TRACE=n               : No builtin tracing from NSH
          CONFIG_NSH_ARCHINIT=y                   : Automatically start the USB monitor

        Application Configuration -> System NSH Add-Ons:
          CONFIG_USBMONITOR=y              : Enable the USB monitor daemon
          CONFIG_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
          CONFIG_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
          CONFIG_USBMONITOR_INTERVAL=1     : Dump trace data every second
          CONFIG_USBMONITOR_TRACEINIT=y    : Enable TRACE output
          CONFIG_USBMONITOR_TRACECLASS=y
          CONFIG_USBMONITOR_TRACETRANSFERS=y
          CONFIG_USBMONITOR_TRACECONTROLLER=y
          CONFIG_USBMONITOR_TRACEINTERRUPTS=y

       NOTE: USB debug output also be enabled in this case.  Both will appear
       on the serial SYSLOG output.  However, the debug output will be
       asynchronous with the trace output and, hence, difficult to interpret.

    6. If you want to try this configuration on the DB-DP11212 PIC32 General
       Purpose Demo Board", here are the changes that you should make:

        Board Configuration:
           CONFIG_ARCH_DBDP11215=n    : Disable the DB-DP11215
           CONFIG_ARCH_DBDP11212=y    : Enable the DB-DP11212
           CONFIG_ARCH_LEDS=n         : The DB-DP11212 has no LEDs

        System Type -> PIC32MX Peripheral Support:
           CONFIG_PIC32MX_UART2=n     : Disable UART2

        The SYSLOG output on UART2 cannot by used.  You have two choices,
        first, you can simply disable the SYSLOG device.  Then 1) debug
        output will come the USB console, and 2) all debug output prior
        to connecting the USB console will be lost:

        The second options is to configure a RAM SYLOG device.  This is
        a circular buffer that accumulated debug output in memory.  The
        contents of the circular buffer can be dumped from the NSH command
        line using the 'dmesg' command.

        Device Drivers -> System Logging Device Options:
          CONFIG_RAMLOG=y             : Enable the RAM-based logging feature.
          CONFIG_RAMLOG_CONSOLE=n     : (there is no default console device)
          CONFIG_RAMLOG_SYSLOG=y      : This enables the RAM-based logger as the
                                        system logger.

        Logging is currently can be set up to use any amount of memory (here 8KB):

          CONFIG_RAMLOG_BUFSIZE=8192

        STATUS:
          2013-7-4:  This configuration was last verified.

     7. See the notes for the nsh configuration.  Most also apply to the usbnsh
        configuration as well.
