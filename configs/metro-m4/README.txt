README
======

  This directory contains the port of NuttX to the Adafruit Metro M4.  The
  Metro M4 uses a Arduino form factor and and pinout.  It's powered with an
  ATSAMD51J19:

  o Cortex M4 core running at 120 MHz
  o Hardware DSP and floating point support
  o 512 KB flash, 192 KB RAM
  o 32-bit, 3.3V logic and power
  o Dual 1 MSPS DAC (A0 and A1)
  o Dual 1 MSPS ADC (8 analog pins)
  o 6 x hardware SERCOM (I2C, SPI or UART)
  o 16 x PWM outputs
  o Stereo I2S input/output with MCK pin
  o 10-bit Parallel capture controller (for camera/video in)
  o Built in crypto engines with AES (256 bit), true RNG, Pubkey controller
  o 64 QFN

Contents
========

  o STATUS
  o Unlocking FLASH
  o Serial Console
  o LEDs
  o Run from SRAM
  o Configurations

STATUS
======

  2018-07-26:  The basic port was merged into master.  It is still
    incomplete and untested.

  2018-07-29:  Code complete.  Clock configuration complete.  Unverified
    SERCOM USART, SPI, I2C, Port configuration, and DMA support have been
    added. I still have no hardware in hand to test.

  2018-07-20:  Brought in the USB driver from the SAML21.  It is the same
    USB IP with only small differences.  There a a few, small open issues
    still to be resolved.

  2018-08-01:  Hardware in hand.  Initial attempts to program the board
    using a Segger J-Link connected via SWD were unsuccessful because the
    Metro M4 comes with an application in FLASH and the FLASH locked.  See
    "Unlocking FLASH with J-Link Commander" below.  After unlocking the
    FLASH, I was able to successfully write the NuttX image.

    Unfortunately, the board seems to have become unusable after the first
    NuttX image was written to FLASH.  I am unable to connect the JTAG
    debugger.  The primary JTAG problem seems to be that it is now unable
    to halt the CPU.

    Future me:  This boot-up failure was do to bad clock initialization
    logic that caused infinite loops during clock configuration.  Unlocking
    and erasing the FLASH is innocuous, but the JTAG will apparently not
    work if the clocks are not in a good state.

    I would say that as a general practice, any changes to the clock
    configuration should be testing in SRAM first before risking the
    write to FLASH.

  2018-08-03:  Added a configuration option to run out of SRAM vs FLASH.
    This is a safer way to do the initial board bring-up since it does
    not modify the FLASH image nor does it require unlocking the FLASH
    pages.

  2018-08-31:  I finally have a new Metro M4 and have successfully
    debugged from SRAM (with FLASH unlocked and erased).  Several
    errors in clock configuration logic have been corrected and it now
    gets through clock configuration okay.  It now hangs in the low-level
    USART initialization.

    It hangs trying to enabled the SERCOM slow clock channel.  The clock
    sequence is:

      1. 32.678KHz crystal -> XOSC32K
         This is configured and says that XOSC32K is ready.
      2. XOSCK32 -> GCLK3.
         This is configured and it says that is is ready (GENEN=1).
      3. GCLK3 ->SERCOM slow clock channel.
         This hangs when I try to enable the peripheral clock.

  2018-09-01:  I found a workaround by substituting OSCULP32K for XOSC32
    as the source to GCLK3.   With that workaround, the port gets past all
    clock and USART  configuration.  A new configuration option was added,
    CONFIG_METRO_M4_32KHZXTAL.  By default this workaround is in place.
    But you can enable CONFIG_METRO_M4_32KHZXTAL if you want to further
    study the XOSC32K problem.

    With that workaround (and a bunch of other fixes), the basic NSH
    configuration appears fully function, indicating the the board bring-
    up is complete.

    There are additional drivers ported from SAML21 which has, in most cases,
    identical peripherals.  None of these drivers have been verified on the
    SAMD51, However.  These include:  DMAC, I2C, SPI, and USB.

Unlocking FLASH
===============

  Options
  -------
  The Adafruit Metro M4 comes with a very nice bootloader resident in FLASH.
  so we have two options:

  1. Learn to play well with others.  Make NuttX coexist and work in the
     memory partition available to it.  Or,
  2. Be greedy, unlock the FLASH and overwrite the bootloader.

  I chose to do the last one.  I used a Segger J-Link and here are the steps
  that I took.  You can probably do these things in Atmel Studio (?) but for
  other debug environments, you would have to come up with the solution.

  Unlocking FLASH with J-Link Commander
  -------------------------------------

  1. Start J-Link Commander:

      SEGGER J-Link Commander V6.32i (Compiled Jul 24 2018 15:20:49)
      DLL version V6.32i, compiled Jul 24 2018 15:19:55

      Connecting to J-Link via USB...O.K.
      Firmware: J-Link V9 compiled Apr 20 2018 16:47:26
      Hardware version: V9.30
      S/N: 269303123
      License(s): FlashBP, GDB
      OEM: SEGGER-EDU
      VTref=3.296V


      Type "connect" to establish a target connection, '?' for help
      J-Link>con
      Please specify device / core. <Default>: ATSAMD51P19
      Type '?' for selection dialog
      Device>ATSAMD51P19
      Please specify target interface:
        J) JTAG (Default)
        S) SWD
      TIF>S
      Specify target interface speed [kHz]. <Default>: 4000 kHz
      Speed>
      Device "ATSAMD51P19" selected.


      Connecting to target via SWD
      Found SW-DP with ID 0x2BA01477
      Scanning AP map to find all available APs
      ...etc. ...

  2. Look at The NVM "user page" memory at address 0x00804000:

       J-Link>mem8 804000, 10
       00804000 = 39 92 9A F6 80 FF EC AE FF FF FF FF FF FF FF FF

     The field NVM BOOT (also called BOOTPROT) is the field that locks the
     lower part of FLASH to support the boot loader.  This is bits 26-29
     of the NVM user page:

       J-Link>mem32 804000, 1
       00804000 = F69A9239

    In binary 11|11 01|10 1001 1010  1001 0010 0011 1001, so NVM Boot 1101.
    To unlock the FLASH memory reserved for the bootloader, we need to
    change this field to 111 so that:

      11|11 11|10 10|01 1010  1001 0010 0011 1001 = F7da9239, or
      00804000 = 39 92 9A FE 80 FF EC AE FF FF FF FF FF FF FF FF

    is read.

  3. Modify the NVM "user page"

    I did this using the instructions for the SAMD21 found at

      https://roamingthings.de/use-j-link-to-change-the-boot-loader-protection-of-a-sam-d21/

    We will need to create a small Motorola S-REC file to write new values
    into NVM.  See https://en.m.wikipedia.org/wiki/SREC_(file_format) for a
    description of the Motorola SREC format.

    I wrote a small program at configs/metro-m4-scripts/nvm.c that will
    generate this Motorola SREC file with the correct checksum.  The file at
    configs/metro-m4-scripts/nvm.c is the output of that program.

      J-Link>mem8 804000,10
      00804000 = 39 92 9A F6 80 FF EC AE FF FF FF FF FF FF FF FF
      J-Link>loadfile D:\Spuda\Documents\projects\nuttx\master\nuttx\configs\metro-m4\scripts\nvm.srec
      Downloading file [D:\Spuda\Documents\projects\nuttx\master\nuttx\configs\metro-m4\scripts\nvm.srec]...
      J-Link: Flash download: Bank 1 @ 0x00804000: 1 range affected (16 bytes)
      J-Link: Flash download: Total time needed: 0.089s (Prepare: 0.035s, Compare: 0.011s, Erase: 0.000s, Program: 0.019s, Verify: 0.011s, Restore: 0.011s)
      O.K.
      J-Link>mem8 804000,10
      00804000 = 39 92 9A FE 80 FF EC AE FF FF FF FF FF FF FF FF

    You will, of course, have to change the path as appropriate for your system.

  4. Erase FLASH (optional)

      J-Link>erase
      Erasing device (ATSAMD51P19)...
      J-Link: Flash download: Total time needed: 2.596s (Prepare: 0.031s, Compare: 0.000s, Erase: 2.553s, Program: 0.000s, Verify: 0.000s, Restore: 0.012s)
      J-Link: Flash download: Total time needed: 0.066s (Prepare: 0.038s, Compare: 0.000s, Erase: 0.016s, Program: 0.000s, Verify: 0.000s, Restore: 0.010s)
      Erasing done.
      J-Link>

Serial Console
==============

  An Arduino compatible serial Shield is assumed (or equivalently, and
  external RS-232 or serial-to-USB adapter connected on Arduino pins D0 and
  D1):

    ------ ----------------- -----------
    SHIELD SAMD5E5           FUNCTION
    ------ ----------------- -----------
    D0     PA23 SERCOM3 PAD2 RXD
    D1     PA22 SERCOM3 PAD0 TXD

LEDs
====

  The Adafruit Metro M4 has four LEDs, but only two are controllable by software:

    1. The red LED on the Arduino D13 pin, and
    2. A NeoPixel RGB LED.

  Currently, only the red LED is supported.

    ------ ----------------- -----------
    SHIELD SAMD5E5           FUNCTION
    ------ ----------------- -----------
    D13    PA16              GPIO output

Run from SRAM
=============

  I bricked my first Metro M4 board because there were problems in the
  bring-up logic.  These problems left the chip in a bad state that was
  repeated on each reset because the code was written into FLASH and I was
  unable to ever connect to it again via SWD.

  To make the bring-up less risky, I added a configuration option to build
  the code to execution entirely out of SRAM.  By default, the setting
  CONFIG_METRO_M4_RUNFROMFLASH=y is used and the code is built to run out of
  FLASH.  If CONFIG_METRO_M4_RUNFROMSRAM=y is selected instead, then the
  code is built to run out of SRAM.

  To use the code in this configuration, the program must be started a
  little differently:

    gdb> mon reset
    gdb> mon halt
    gdb> load nuttx             << Load NuttX into SRAM
    gdb> file nuttx             << Assuming debug symbols are enabled
    gdb> mon memu32 0x20000000  << Get the address of initial stack
    gdb> mon reg sp 0x200161c4  << Set the initial stack pointer using this address
    gdb> mon memu32 0x20000004  << Get the address of __start entry point
    gdb> mon reg pc 0x20000264  << Set the PC using this address (without bit 0 set)
    gdb> si                     << Step in just to make sure everything is okay
    gdb> [ set breakpoints ]
    gdb> c                      << Then continue until you hit a breakpoint

  Where 0x200161c4 and 0x20000264 are the values of the initial stack and
  the __start entry point that I read from SRAM

Configurations
==============

  Each Adafruit Metro M4 configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh [OPTIONS] metro-m4/<subdir>

  Do 'tools/configure.sh -h' for the list of options.  If you are building
  under Windows with Cygwin, you would need the -c option, for example.

  Before building, make sure that the PATH environmental variable includes the
  correct path to the directory than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of configurations listed in the following paragraph.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
     change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output of on SERCOM3 which is available on a Arduino Serial
     Shield (see the section "Serial Console" above).

  3. Unless otherwise stated, the configurations are setup build under
     Linux with a generic ARM EABI toolchain:

Configuration sub-directories
-----------------------------

  nsh:
    This configuration directory will built the NuttShell.  See NOTES ;for
    common configuration above and the following:

    NOTES:

    1. The CMCC (Cortex M Cache Controller) is enabled.

