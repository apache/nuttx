README.txt
==========

This is the README file for the NuttX port to the 16z board. The 16z board
is based on the ZiLOG ZNEOZ16F2811AL20EG part.  See https://github.com/toyaga/16z
for further information.

Contents
========

  - GPIO Configuration
  - ZDS-II Compiler Versions
  - Serial Console
  - LEDs
  - RAM
  - Selecting Configurations
  - Configuration Sub-directories

GPIO Configuration
==================

  --------------------------- ------ --------------------------------------------
  GPIO                        SIGNAL On-Board Connections
  --------------------------- ------ --------------------------------------------
  PA0/T0IN/T0OUT/DMA0REQ      GP8    PS/2 / GPIO, Expansion slots
  PA1/T0OUT/DMA0ACK           GP9    PS/2 / GPIO, Expansion slots
  PA2/DE0/FAULTY              ~INTI  Power section, RF transceiver (1)
  PA3/CTS0/FAULT0             ~INTX  Expansion slots
  PA4/RXD0/CS1                RXD    MAX3232D RS-232
  PA5/TXD0/CS2                TXD    MAX3232D RS-232
  PA6/SCL/CS3                 SCL    RTC / UID, Expansion slots
  PA7/SDA/CS4                 SDA    RTC / UID, Expansion slots
  --------------------------- ------ --------------------------------------------
  PB0/ANA0/T0IN0              GP0    Expansion slots
  PB1/ANA1/T0IN1              GP1    Expansion slots
  PB2/ANA2/T0IN2              GP2    Expansion slots
  PB3/ANA3/OPOUT              GP3    Expansion slots
  PB4/ANA4                    GP4    Expansion slots
  PB5/ANA5                    GP5    Expansion slots
  PB6/ANA6/OPINP/CINN         GP6    Expansion slots
  PB7/ANA7/OPINN              GP7    Expansion slots
  --------------------------- ------ --------------------------------------------
  PC0/T1IN/T1OUT/DMA1REQ/CINN GP10   PS/2 / GPIO, Expansion slots
  PC1/T1OUT/DMA1ACK/COMPOUT   GP11   PS/2 / GPIO, Expansion slots
  PC2/SS/CS4                  ~EXP   Expansion slots
  PC3/SCK/DMA2REQ             SCK    FT800Q, Serial memory (1), RF Transceiver (1),
                                     Expansion slots, SD0, 1, and 2
  PC4/MOSI/DMA2ACK            MOSI   FT800Q, Serial memory (1), RF Transceiver (1),
                                     Expansion slots, SD0, 1, and 2
  PC5/MISO/CS5                MISO   FT800Q, Serial memory (1), RF Transceiver (1),
                                     Expansion slots, SD0, 1, and 2
  PC6/T2IN/T2OUT/PWMH0        ~CTS   MAX3232D RS-232
  PC7/T2OUT/PWML0             ~RTS   MAX3232D RS-232, Power section (?)
  --------------------------- ------ --------------------------------------------
  PD0/PWMH1/ADR20             A20    RAM, Expansion slots
  PD1/PWML1/ADR21             A21    RAM, Expansion slots
  PD2/PWMH2/ADR22             A22    RAM, Expansion slots
  PD3/DE1/ADR16               A16    RAM, Expansion slots
  PD4/RXD1/ADR18              A18    RAM, Expansion slots
  PD5/TXD1/ADR19              A19    RAM, Expansion slots
  PD6/CTS1/ADR17              A17    RAM, Expansion slots
  PD7/PWML2/ADR23             A23    Expansion slots
  --------------------------- ------ --------------------------------------------
  PE0/DATA0                   D0     RAM, Expansion slots
  PE1/DATA1                   D1     RAM, Expansion slots
  PE2/DATA2                   D2     RAM, Expansion slots
  PE3/DATA3                   D3     RAM, Expansion slots
  PE4/DATA4                   D4     RAM, Expansion slots
  PE5/DATA5                   D5     RAM, Expansion slots
  PE6/DATA6                   D6     RAM, Expansion slots
  PE7/DATA7                   D7     RAM, Expansion slots
  --------------------------- ------ --------------------------------------------
  PF0/ADR0                    A0     Expansion slots
  PF1/ADR1                    A1     RAM, Expansion slots
  PF2/ADR2                    A2     RAM, Expansion slots
  PF3/ADR3                    A3     RAM, Expansion slots
  PF4/ADR4                    A4     RAM, Expansion slots
  PF5/ADR5                    A5     RAM, Expansion slots
  PF6/ADR6                    A6     RAM, Expansion slots
  PF7/ADR7                    A7     RAM, Expansion slots
  --------------------------- ------ --------------------------------------------
  PG0/ADR0                    A8     RAM, Expansion slots
  PG1/ADR0                    A9     RAM, Expansion slots
  PG2/ADR0                    A10    RAM, Expansion slots
  PG3/ADR0                    A11    RAM, Expansion slots
  PG4/ADR0                    A12    RAM, Expansion slots
  PG5/ADR0                    A13    RAM, Expansion slots
  PG6/ADR0                    A14    RAM, Expansion slots
  PG7/ADR0                    A15    RAM, Expansion slots
  --------------------------- ------ --------------------------------------------
  PH0/ANA8/WR                 ~WR    RAM, Expansion slots
  PH1/ANA9/RD                 ~RD    RAM, Expansion slots
  PH2/ANA10/CS0               ~RF    LED3, RF transceiver, X2 (1)
  PH3/ANA11/CINP/WAIT         ~SXM   LED4, Chip select for the serial memory, U4 (1)
  --------------------------- ------ --------------------------------------------
  PJ0/DATA8                   ~SD1   LED5, Chip select for the SD card 1, X11.
  PJ1/DATA9                   ~DT1   Card detect for SD card 1
  PJ2/DATA10                  WP1    Write protect for SD card 1
  PJ3/DATA11                  EVE    EVE chip select
  PJ4/DATA12                  ~SD2   LED6, Chip select for the SD card 2, X10.
  PJ5/DATA13                  ~DT2   Card detect for SD card 2
  PJ6/DATA14                  WP2    Write protect for SD card 2
  PJ7/DATA15                  ~SD0   LED7, Chip select for the microSD 0, X12.
  --------------------------- ------ --------------------------------------------
  PK0/BHEN                    ~BHE   RAM, Expansion slots
  PK1/BLEN                    ~BLE   RAM, Expansion slots
  PK2/CS0                     ~0000  Bottom RAM bank, Expansion slots
  PK3/CS1                     ~8000  Top RAM bank, Expansion slots
  PK4/CS2                     ~F000  Expansion slots
  PK5/CS3                     ~FFC8  Expansion slots
  PK6/CS4                     ~FFD0  Expansion slots
  PK7/CS5                     ~FFD8  Expansion slots
  --------------------------- ------ --------------------------------------------

  Note 1:  Not populated on my board

ZDS-II Compiler Versions
========================

Version 5.0.1

  All testing has been performed with ZSD II verion 5.0.1 for the ZNEO.

Other Versions

  If you use any version of ZDS-II other than 5.0.1 or if you install ZDS-II
  at any location other than the default location, you will have to modify
  two files:  (1) configs/16z/*/setenv.sh and (2) configs/16z/*/Make.defs. 
  Simply edit these two files, changing 5.0.1 to whatever.

Serial Console
==============

The 16z supports a single UART, UART0, that will be used to support the
NuttX serial console.

LEDs
====

The 16z board has 7 LEDs, five of which are controllable via software:

  ----- ------ ------ ------------------------------------
  LED   Color  Signal Description
  ----- ------ ------ ------------------------------------
  LED1  Red     3V3   Indicates the presence of +3.3V
  LED2  Red     5V    Indicates the presence of +5V

  LED3  Blue    ~RF   Controlled via PH2.  Notes: 1, 2
  LED4  Green   ~SXM  Controlled via PH3.  Notes: 1, 3
  LED5  Green   ~SD1  Controlled via PJ0.  Notes: 1, 4
  LED6  Yellow  ~SD2  Controlled via PJ4.  Notes: 1, 5
  LED7  Yellow  ~SD0  Controlled via PJ7.  Notes: 1, 6
  ----- ------ ------ ------------------------------------

  Note 1:  Pulled high so a low output illuminates the LED.
  Note 2:  PH2/~RF is also used by the RF transceiver, X2.  That part is not
           populated on my board.
  Note 3:  ~SXM is the chip select for the serial memory, U4.  That part is
           not populated on my board.
  Note 4:  ~SD1 is the chip select for the SD card 1, X11.
  Note 5:  ~SD2 is the chip select for the SD card 2, X10.
  Note 6:  ~SD0 is the chip select for the microSD 0, X12.

In conclusion:  None of the LEDs are available to indicate software status
without potentially sacrificing board functionality.  If the RF transceiver
is not installed (CONFIG_16Z_RFTRANSCEIVER=n) and if LED support is
requested (CONFIG_ARCH_LEDS), then LED3 will be used to indicate status:  A
solid color means that the board has boot successfully; flashing at a rate
of approximately 2Hz indicates a software failure.

RAM
===

The 16z has two IS66WVE4M16BLL 64Mb (4M x 16b) "Pseudo" SRAM parts on board.
This provides a total of 16MiB of SRAM from program usage.

Selecting Configurations
========================

Variations on the basic 16z configuration are maintained in subdirectories.
To configure any specific configuration, do the following steps:

   cd <nuttx-top-directory>/tools
   ./configure.sh 16z/<sub-directory>
   cd <nuttx-top-directory>
   make

Where <sub-directory> is the specific board configuration that you wish to
build.  The following board-specific configurations are available.

Before entering the make command, make certain that the path to the ZNEO
compiler is in you PATH variable.  You make modify and use the setenv.sh
script to set that PATH if you like.  You can simply source setenv.sh
before making like:

  ...
  . ./setenv.sh
  make


Configuration Sub-directories
=============================

source/ and include/
--------------------

  These directories contain common logic for all 16z configurations.

nsh
---
  nsh:
    This configuration directory will built the NuttShell (NSH).  See
    the NSH user manual in the documents directory (or online at nuttx.org).
    See also the README.txt file in the nsh sub-directory for information
    about using ZDS-II.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this configuration assumes that you are using the
       Cygwin environment on Windows.  An option is to use the native
       CMD.exe window build as described in the top-level README.txt
       file.  To set up that configuration:

       -CONFIG_WINDOWS_CYGWIN=y
       +CONFIG_WINDOWS_NATIVE=y

       And after configuring, make sure that CONFIG_APPS_DIR uses
       the back slash character.  For example:

        CONFIG_APPS_DIR="..\apps"

      NOTES:

      a. If you need to change the toolchain path used in Make.defs, you
         will need to use the short 8.3 filenames to avoid spaces.  On my
         PC, C:\PROGRA~1\ is is C:\Program Files\ and C:\PROGRA~2\ is
         C:\Program Files (x86)\
      b. I have not tried to use this configuration with the native
         Windows build, but I would expect the same issues as is listed
         for the ostest configuration..

   STATUS:
      Currently, NSH failes nsh_consoleoutput().  Here is an example.
      This echo command causes the system to hang:

        nsh> echo abc

      Below is some annotated output from the debugger.  Here is the 30,000 ft view:

        - cmd_echo loops for each argv[i], 1 >=i > argc.

        - It calls:

            vtbl->output(vtbl, "%s ", argv[i])

          where the prototype for output is:

            int (*output)(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...);

        - vtbl->output maps to nsh_consoleoutput() in this case.

        - cmd_echo passes all of the arguments to output in registers.

        - nsh_consoleoutput expects all of the parameters on the stack.

        - nsh_console calls vfprintf() using bad values from the stack.

        - vfprintf crashes and never returns.

      Looks like a compiler bug to me.

      # int cmd_echo(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
      #
      # All input parameters are in registers
      #
      #   R1=00802DA0                              # vtbl
      #   R2=00000002                              # argc
      #   R3=00802D15                              # argv
      #                                            # argv[0]=00802DD7
      #                                            # argv[1]=00802DDC
      #                                            # 00802DD7="echo\0abc\0"
      #   SP=00802CDD

      0001024C 05F0 PUSHMHI <R8-R11>               # SP=00802CCD
      0001024E 4418 LD   R8,R1                     # R8=00802DA0  vtbl
      00010250 442A LD   R10,R2                    # R10=00000002 argc
      00010252 443B LD   R11,R3                    # R11=00802D15 argv
      00010254 3901 LD   R9,#%1                    # R9=00000001  arg index
      00010256 C00C JP   %10270

      00010270 A5A9 CP   R9,R10                    # Bottom of loop
      00010272 E1F2 JP   lt,%10258

      00010258 48840010 LD   R4,%10(R8)            # R4=00011156 adddress of output() method
      0001025C 4490 LD   R0,R9                     # R0=00000001 Index of argv[1]
      0001025E BC20 SLL  R0,#%2                    # R0=00000004 Offset to argv[1]
      00010260 A0B0 ADD  R0,R11                    # R0=00802D19 Address of argv[1]
      00010262 4481 LD   R1,R8                     # R1=00802DA0 vtbl address
      00010264 452200008ADB LD   R2,#%8ADB         # R2=00008ADB = "%s "
      0001026A 1203 LD   R3,(R0)                   # R3=00802DDC Value of argv[1]
      0001026C F214 CALL (R4)                      # Call vtbl->output(vtbl, "%s ", argv[i]);
                                                   # vtbl->output is nsh_consoleoutput

      # static int nsh_consoleoutput(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...)
      #
      # All parameters are in registers:
      #
      #   R1=00802DA0 vtbl address
      #   R2=00008ADB "%s "
      #   R3=00802DDC Value of argv[1]

      # First is a check if the output file is open
      #
      #  if (nsh_openifnotopen(pstate) != 0)
      #   {
      #     return ERROR;
      #   }

      00011156 0800 LINK #%0                       # SP=00802CC9, R14=00802CC9
      00011158 5C81 LD   R1,%8(FP)                 # R1=0000017F Should be value file FILE * for output
      0001115A DF96 CALL %11088                    # Call nsh_openifnotopen(), returns R0=00000000
      0001115C 9000 CP   R0,#%0
      0001115E E602 JP   z,%11164                  # Skip over error return

      00011160 30FF LD   R0,#-%1
      00011162 C007 JP   %11172

      # Then the failing call to vfprintf:
      #
      #   va_start(ap, fmt);
      #   ret = vfprintf(pstate->cn_outstream, fmt, ap);
      #   va_end(ap);
      #
      #   return ret;

      00011164 4D03 LEA  R3,%10(FP)                # R3=00802CD5 ap=GARBAGE
      00011166 5C80 LD   R0,%8(FP)                 # R0=0000017F Should be value of pstate
      00011168 48010033 LD   R1,%33(R0)            # R1=01000000 pstate->cn_outstream.  Looks suspicious
      0001116C 5CC2 LD   R2,%C(FP)                 # R2=00802DA0
      0001116E F10003FB CALL %11968                # Call vfprintf(01000000, 00802DA0, 00802CD5)
                                                   # All arguments are bad
                                                   # Does not survive call to vfprintf

ostest
------

    This builds the examples/ostest application for execution from FLASH.
    See the README.txt file in the ostest sub-directory for information
    about using ZDS-II.  See also apps/examples/README.txt for information
    about ostest.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this configuration assumes that you are using the
       Cygwin environment on Windows.  An option is to use the native
       CMD.exe window build as described in the top-level README.txt
       file.  To set up that configuration:

       -CONFIG_WINDOWS_CYGWIN=y
       +CONFIG_WINDOWS_NATIVE=y

       And after configuring, make sure that CONFIG_APPS_DIR uses
       the back slash character.  For example:

        CONFIG_APPS_DIR="..\apps"

      NOTES:

      a. If you need to change the toolchain path used in Make.defs, you
         will need to use the short 8.3 filenames to avoid spaces.  On my
         PC, C:\PROGRA~1\ is is C:\Program Files\ and C:\PROGRA~2\ is
         C:\Program Files (x86)\
      b. You can't use setenv.sh in the native Windows environment.  Try
         scripts/setenv.bat instead.
      c. At present, the native Windows build fails at the final link stages.
         The failure is due to problems in arch/z16/src/nuttx.linkcmd that
         is autogenerated by arch/z16/src/Makefile.  The basic problem
         is the spurious spaces and and carrirage returns are generated at
         the end of the lines after a line continuation (\ ^M).  If these
         trailing bad characters are manually eliminated, then the build
         will succeed on the next try.

Check out any README.txt files in these <sub-directory>s.
