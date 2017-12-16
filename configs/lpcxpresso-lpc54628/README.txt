README
======

  This directory contains the port to the NXP LPCXpress-LPC54628 board
  (OMI1309UL).  This board features:

    - LPC54628 Cortex-M4 microcontroller running at up to 220MHz
    - 272x480 color LCD with capacitive touch screen
    - On-board, high-speed USB, Link2 debug probe with CMSIS-DAP and SEGGER
      J-Link protocol options
    - UART and SPI port bridging from LPC546xx target to USB via the on-
      board debug probe
    - Support for external debug probe
    - 3 x user LEDs, plus Reset, ISP (3) and user buttons
    - Multiple Expansion options, including Arduino UNO and PMod
    - Built-in power consumption measurement for target LPC546xx MCU
    - 128Mb Micron MT25QL128 Quad-SPI flash
    - 16MB Micron MT48LC8M16A2B4 SDRAM
    - Knowles SPH0641LM4H digital microphone
    - Full size SD/MMC card slot
    - NXP MMA8652FCR1 accelerometer
    - Stereo audio codec with line in/out
    - High and full speed USB ports with micro A/B connector for host or
      device functionality
    - 10/100Mbps Ethernet (RJ45 connector)

STATUS
======

  2017-12-10:  The basic NSH configuration is functional at 220MHz with a
    Serial console, timer and LED support.  Added support for the external
    SDRAM and for the RAM test utility -- UNTESTED!
  2017-12-11:  Fixed an error in board LEDs.  SDRAM is partially functional
    but not reliable.  Added framework for future I2C and SPI flexcomm
    drivers (mostly empty files for now)
  2017-12-12:  The SDRAM is now functional passes the commplete RAM test.
    Included configurations and logic to add none, portions, or all of the
    external SDRAM to the system heap.  Brought in the LPC1788 LCD driver.
    The LPC1788 LCD registers are identical to the LPC54xx (other than a
    minor clock source setting).  That port required modifications only
    for differences in some SYSCON and pin-related settings.
  2017-12-13:  Created the fb configuration for testing the LCD.  Only
    minimal testing has been performed.  As of this writing, there is
    some framebuffer functionality.  There are recognizable but corrupted
    patterns on the LCD.  There are color formatting problems and some
    horizontal elongation.
  2017-12-14:  Corrected a misconception about how the video data lines
    were configured.  Now the LCD appears to be fully functional.
  2017-12-15:  Added an I2C driver.  This is the first step on the road
    to getting support for the capacitive touchscreen on the TFT panel.
    The I2C driver appears to be functional but is not yet well-tested.
  2017-12-16:  Added support for LPC54xx GPIO interrupts; added button
    support (with interrupts) to the NSH configuration.  The button
    test is partially functional but appears to miss a lot of button-
    related events.  More testing is needed.

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each LPCXpresso-LPC54628 configuration is maintained in a sub-directory
  and can be selected as follow:

    .tools/configure.sh [OPTIONS] xmc5400-relax/<subdir>

  See '.tools/configure.sh -h' for a list of all options.  The most typical
  are -l to select the Linux host or -c to select the Windows Cygwin host.

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
       output on USART0 (aka Flexcomm0).  USART0 connects to the serial
       bridge on LPC4322JET100 and should be available as a USB serial
       device on your host PC.

    3. All of these configurations are set up to build under Windows using
       the  "GNU Tools for ARM Embedded Processors" that is maintained by
       ARM (unless stated otherwise in the description of the configuration).

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

  fb:

    A simple NSH configuration used for some basic debug of LCD using the
    framebuffer character drivers.  This configuration provides the test
    programs:

      - apps/examples/pdcurses, and
      - apps/examples/fb

    as NSH built-in applications.

    NOTES:
    1. This configuration enables SDRAM to hold the LCD framebuffer and
       enables the LPC54xx LCD driver in order to support the LPCXpresso's
       TFT panel.  In this configuration, the framebuffer resides in the
       the lower half megabyte of SDRAM beginning at address 0xa0000000
       The remainder of the SDRAM from 0xa0080000 up to 0xa1000000 is added
       to the heap.

       The is wasteful of SDRAM:  Only 261,120 bytes actually used for the
       framebuffer.  This memory could be reclaimed by changing the DRAM
       CS0 offset value in the .config file.

    2. Some of the pdcurses test rely on some positional input device and so
       is not yet usable.  Others work fine with no user include:  charset,
       xmas, firework, worms, rain, for examples.

  nsh:

    Configures the NuttShell (nsh) application located at examples/nsh.
    This configuration was used to bring up the board support and, hence,
    is focused on low level, command-line driver testing.  It has no
    network.

    NOTES:

    1. NSH built-in applications are supported.

       Binary Formats:
         CONFIG_BUILTIN=y           : Enable support for built-in programs

       Application Configuration:
         CONFIG_NSH_BUILTIN_APPS=y  : Enable starting apps from NSH command line

    2. SDRAM support is enabled, but the SDRAM is *not* added to the system
       heap.  The apps/system/ramtest utility is include in the build as an
       NSH builtin function that can be used to verify the SDRAM.

         nsh> ramtest -h
         RAMTest: Missing required arguments

         Usage: <noname> [-w|h|b] <hex-address> <decimal-size>

         Where:
           <hex-address> starting address of the test.
           <decimal-size> number of memory locations (in bytes).
           -w Sets the width of a memory location to 32-bits.
           -h Sets the width of a memory location to 16-bits (default).
           -b Sets the width of a memory location to 8-bits.

       The  MTL48LC8M16A2B4-6A SDRAM is on CS0 which corresponds to address
       0xa0000000, the size of the memory is 128Mbits or 16Mb.  So the DRAM
       may be tested with this command:

         NuttShell (NSH) NuttX-7.23
         nsh> ramtest a0000000 16777216
         RAMTest: Marching ones: a0000000 16777216
         RAMTest: Marching zeroes: a0000000 16777216
         RAMTest: Pattern test: a0000000 16777216 55555555 aaaaaaaa
         RAMTest: Pattern test: a0000000 16777216 66666666 99999999
         RAMTest: Pattern test: a0000000 16777216 33333333 cccccccc
         RAMTest: Address-in-address test: a0000000 16777216
         nsh>

    3. I2C2 is enabled (will be used with the capacitive touchscreen).  In
       order to verify I2C functionality, the I2C tool at apps/system/i2ctool
       is enabled in this configuration.

         nsh> i2c bus
          BUS   EXISTS?
         Bus 0: NO
         Bus 1: NO
         Bus 2: YES
         Bus 3: NO
         Bus 4: NO
         Bus 5: NO
         Bus 6: NO
         Bus 7: NO
         Bus 8: NO
         Bus 9: NO
         nsh> i2c dev -b 2 3 77
              0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
         00:          -- -- -- -- -- -- -- -- -- -- -- -- --
         10: -- -- -- -- -- -- -- -- -- -- 1a -- -- 1d -- --
         20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         70: -- -- -- -- -- -- -- --

       I believe that the on-board Accelerometer, Audio Codec, and touch
       panel controller should have been detected (but perhaps the touch
       panel is not powered in this configuration since the LCD is not
       configured?)

         Codec I2C address:        0x1a
         Accel I2C address:        0x1d
         Touch panel I2C address:  0x38

    4. Support for the on-board USER button is included as well as the
       button test program at apps/examples/buttons.  This test is useful
       for verifying the functionality of GPIO interrupts.
