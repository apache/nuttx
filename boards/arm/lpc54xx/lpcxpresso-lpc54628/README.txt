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

CONTENTS
========

  - STATUS
  - Configurations

STATUS
======

  2017-12-10:  The basic NSH configuration is functional at 220MHz with a
    Serial console, timer and LED support.  Added support for the external
    SDRAM and for the RAM test utility.
  2017-12-11:  Fixed an error in board LEDs.  Added framework for future
    I2C and SPI flexcomm drivers.
  2017-12-12:  The SDRAM is now functional and passes the complete RAM
    test.  Added configuration options and logic to add none, portions, or
    all of the external SDRAM to the system heap.  Brought in the LPC1788
    LCD driver.  The LPC1788 LCD registers are identical to the LPC54xx
    (other than a minor clock source setting).
  2017-12-13:  Created the fb configuration for testing the LCD.
  2017-12-14:  Corrected a misconception about how the video data lines
    were configured.  The LCD now appears to be fully functional.
  2017-12-15:  Added an I2C driver.
  2017-12-16:  Added support for LPC54xx GPIO interrupts; added button
    support (with interrupts) to the NSH configuration.  The button
    test appears to functional functional.  There are noticeable delays
    in receiving the button events, especially when the button is
    released.  But if you do not press the buttons too quickly all events
    are processed.  This, I suspect, is a consequence of the strong glitch
    filtering that is enabled in the pin configuration.  Snappier
    response my be obtainable with filtering off.
  2017-12-17:  Added a driver for the FT5x06 capacitive, multi-touch
    controller.  Add support logic for the LPCXpresso-LPC54528 to
    initialize and the register the FT5x06 driver.  Unfortunately, the
    FT5x06 interrupt is on pin P4.0 but pin interrupts are only supported
    on P0.m and P1.m, m=0..31.
  2017-12-18:  Added an option to the FT5x06 driver to support a timer-
    based poll instead of interrupts.  This is very inefficient in that it
    will introduce delays in touchscreen response and will consume more CPU
    bandwidth.  The driver appears to be functional.  Added the NxWM
    configuration to do some integrated testing.  NxWM seems to be fully
    functional.  However, the action of the touchscreen could use some
    human factors improvements.  I imagine that this is a consequence of
    the polled solution.
  2017-12-19:  Brought in Alan Carvalho de Assis' LPC43xx SD/MMC driver from
    https://github.com/Smoothieware/smoothie-nuttx/tree/master/nuttx/arch/arm/src/lpc43xx
    and adapted it for use by the LPC54xx.  Unverified as of this writing.
  2017-12-21:  Some things are working with he SDMMC drivers but read DMAs
    are non-functional and, hence not usable.
  2017-12-23:  SDMMC is still non-functional.  The first DMA read of 512 bytes
    fails with a CRC error.  Similar result if clock is reduced, if 1-bit bus
    is used, if DMA is disabled., if DEBUG output is disabled.
  2017-12-24:  Added basic DMA support; brought in the WWDT driver from the
    LPC43 which has the same peripheral.  Neither tested; almost certainly
    non-functional without some additional investment.
  2017-12-25:  Added an RTC driver.  It appears to be functional but has not
    been well tested.
  2017-12-26:  Added an RNG driver.  The RNG is actually controlled by a ROM
    function.  This driver seems to work fine when single stepping.  However,
    if I collect samples indefinitely, I get a reserved interrupt.  The symptom
    before the crash is that local variables are getting corrupted after the
    call into ROM. Increasing the stack size does not seem to help.  Perhaps
    to use the ROM at high frequencies it may be necessary to modify the ROM
    access timing in some way???
  2017-12-30:  Completed implementation of an Ethernet driver.  Untested as
    of this writing.  Also added the netnsh configuration will, eventually,
    be used to test the Ethernet driver.
  2018-01-01:  There Ethernet driver appears to be fully functional although
    more testing is certainly needed.
  2018-01-14:  The basic SPI driver is code complete but still untested.  It
    is "basic" in the sense that it supports only polled mode (no DMA).
  2018-01-18:  Added the lvgl configuration.  See notes under "Configuration
    Sub-directories" for additional status.
  2018-10-22:  Dave Marples recently fixed the LPC43 version of the USB
    device controller driver.  That driver is a clone from the LPC54 USB
    DCD.  I have backported Dave's changes to the LPC54 DCD.  Unfortunately,
    it did not fix the problem.  Then I discovered this errata for the LPC54:

      For the 4-bit mode to work successfully, four otherwise unused upper
      data bits (SD_D[4] to SD_D[7]) must be functionally assigned to GPIO
      pins with pull-up resistor. These pins do not need to be physically
      connected on the hardware.

    With that change (and a lot of other fidgeting), there is some
    improvement.  I am able to mount and read the SD card .. at least most
    of the time.  I still get CRC errors when writing and I have not
    successfully written to the SD card.  It is closer but more TLC is
    needed.
  2018-10-24:  Dave Marples now has the LPC43 SD/MMC working reliably.  I
    have ported all of Dave's change to the LPC54 but have done no further
    testing as of this writing.  The feature is still marked EXPERIMENTAL.
  2019-05-08:  I brought in the USB0 OHCI USB host driver from LPC17.  Since
    OHCI is well standardized, this should work out of the box provided that
    the peripheral is properly configured, initialized, and clocked.  The
    clock setup logic is missing as of this writing (the driver is not yet
    even included in the build and completely unverified).
  2019-00-22:  Although everyone with the older Rev C boards were booting
    happily, it was reported that people with Rev F boards could not boot.
    A patch updating the clock configuration for those boards was verified
    (on both Rev C and Rev F) and pushed upstream.

  There is still no support for the Accelerometer, SPIFI, or USB.  There is
  a complete but not entirely functional SD card driver and and tested SPI
  driver.  There is also a partial port of the USB0 OHCI host driver if
  anyone is ambitious enough to finish that off.  There are no on-board
  devices to support SPI testing.

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each LPCXpresso-LPC54628 configuration is maintained in a sub-directory
  and can be selected as follow:

    .tools/configure.sh [OPTIONS] lpcxpresso-lpc54628:<subdir>

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
         CONFIG_ARM_TOOLCHAIN_GNU_EABI=y  : GNU ARM EABI toolchain

  Configuration Sub-directories
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

    3. I2C2 is enabled (will be used with the capacitive touchscreen).  In
       order to verify I2C functionality, the I2C tool at apps/system/i2ctool
       is enabled in this configuration.

         nsh> i2c dev -b 2 3 77
              0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
         00:          -- -- -- -- -- -- -- -- -- -- -- -- --
         10: -- -- -- -- -- -- -- -- -- -- 1a -- -- 1d -- --
         20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         30: -- -- -- -- -- -- -- -- 38 -- -- -- -- -- -- --
         40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         70: -- -- -- -- -- -- -- --

         Codec I2C address:        0x1a
         Accel I2C address:        0x1d
         Touch panel I2C address:  0x38

    4. The touchscreen test program at apps/examples/touchscreen is also
       included in this configuration.

         nsh> tc 5
         tc_main: nsamples: 2
         tc_main: Initializing external touchscreen device
         tc_main: Opening /dev/input0
         Sample     :
            npoints : 1
         Point 1    :
                 id : 0
              flags : 1a
                  x : 230
                  y : 84
                  h : 0
                  w : 0
           pressure : 0
         etc.

      NOTE that the touchscreen controlled must run in a polled mode!  The
      FT5x06 interrupt GPIO is on P4.0 and, as far as I know, GPIO
      interrupts are not supported on P4.  So polled mode only for this
      puppy.

  lvgl
  ----
    This is a demonstration of the LittlevGL graphics library running on
    the NuttX frame buffer driver (as in the fb configuration).  You can
    find LittlevGL here:

      https://littlevgl.com/
      https://github.com/littlevgl

    This configuration uses the LittlevGL demonstration at apps/examples/lvgldemo.

    NOTES:

    1. The LittlevGL demonstration is quit large, due mostly to some large
       graphic images.  So memory is tight in the LPC54628's 512Kb FLASH.  In
       fact, if you disable optimization, the demo will not fit into FLASH
       memory (at least not with debug output also enabled).

       A longer term solution might load the large images into the abundant
       SDRAM at runtime instead of linking it statically in FLASH.

    STATUS:

      2018-01-18:  The demo is basically function but has some issues:

        a) The font is too big on the "Write" screen.  They don't fit in on
           the keyboard.
        b) The "List" display is filled with a big box that says "Click a
           button to copy its text to Text area."  There are no buttons and
           nothing to click on (maybe they are behind the big box?).  This
           may also be a font size issue.
        c) The "Chart" display looks okay.

  netnsh:
  ------
    This is a special version of the NuttShell (nsh) configuration that is
    tailored for network testing.  This version derives from nsh
    configuration so many of the notes there apply here except as noted
    below.

    NOTES:

    1. Networking is enabled.  The LPCXpressio-LPC54628 has an SMC _LAN8720 PHY
       and RJ45 network connector.  Support is enabled for IPv4, IPv6, TCP/IP,
       UDP, ICMP, ICMPv6, and ARP.

       The default IP addresses are 10.0.0.2 (IPv4) and fc00::2 (IPv6).  You
       should reconfigure these as appropriate for your test network.

    2. SD card and I2C support are not enabled.  The I2C tool application is
       not enabled

    3. SDRAM support is enabled and the SDRAM is added to the system heap.
       The RAM test application is not enabled.

    4. This configuration does not include support for asynchronous network
       initialization.  As a consequence, NSH must bring up the network
       before you get the NSH prompt.  If the network cable is unplugged,
       this can mean a significant delay before you see the prompt.

    5. In this configuration, the network the network and the Telnet
       daemon are available by the time that the NSH prompt is presented.

       Telnet defaults to IPv6 so to use it from the host PS, you would have
       to do:

         $ telnet fc00::42

  nsh:

    Configures the NuttShell (nsh) application located at examples/nsh.
    This configuration was used to bring up the board support and, hence,
    is focused on low level, command-line driver testing.  It has no
    network and no graphics capability.

    NOTES:

    1. NSH built-in applications are supported.

       Binary Formats:
         CONFIG_BUILTIN=y           : Enable support for built-in programs

       Application Configuration:
         CONFIG_NSH_BUILTIN_APPS=y  : Enable starting apps from NSH command line

    2. SDRAM support is enabled, but the SDRAM is *not* added to the system
       heap.  The apps/system/ramtest utility is include in the build as an
       NSH built-in function that can be used to verify the SDRAM.

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

       I believe that the on-board Accelerometer, Audio CODEC, and touch
       panel controller should have been detected (but perhaps the touch
       panel is not powered in this configuration since the LCD is not
       configured?)

         Codec I2C address:        0x1a
         Accel I2C address:        0x1d
         Touch panel I2C address:  0x38

    4. Support for the on-board USER button is included as well as the
       button test program at apps/examples/buttons.  This test is useful
       for verifying the functionality of GPIO interrupts.

         NuttShell (NSH) NuttX-7.23
         nsh> buttons
         buttons_main: Starting the button_daemon
         buttons_main: button_daemon started
         button_daemon: Running
         button_daemon: Opening /dev/buttons
         button_daemon: Supported BUTTONs 0x01
         nsh> Sample = 1
         Sample = 0
         Sample = 1
         Sample = 0
         Sample = 1
         Sample = 0
         Sample = 1
         etc.

       There are noticeable delays in receiving the button events,
       especially when the button is released.  But if you do not press the
       buttons too quickly all events are processed.  This, I suspect, is a
       consequence of the strong glitch filtering that is enabled in the pin
       configuration.  Snappier response my be obtainable with filtering off
       if desired.

    5. This configuration has been used for testing the SDMMC driver with
       these configuration additions:

         CONFIG_EXPERIMENTAL=y

         CONFIG_LPC54_SDMMC=y
         CONFIG_LPC54_SDMMC_PWRCTRL=y
         CONFIG_LPC54_SDMMC_DMA=y

         CONFIG_SCHED_WORKQUEUE=y
         CONFIG_SCHED_HPWORK=y
         CONFIG_SCHED_HPWORKPRIORITY=224
         CONFIG_SCHED_HPWORKSTACKSIZE=2048

         CONFIG_MMCSD=y
         CONFIG_MMCSD_NSLOTS=1
         CONFIG_MMCSD_MULTIBLOCK_LIMIT=1
         CONFIG_MMCSD_HAVE_CARDDETECT=y
         CONFIG_MMCSD_HAVE_WRITEPROTECT=y
         CONFIG_ARCH_HAVE_SDIO=y
         CONFIG_SDIO_DMA=y
         CONFIG_MMCSD_SDIO=y

         CONFIG_NSH_MMCSDSLOTNO=0

    6. The RTC is enabled in this configuration.

         NuttShell (NSH) NuttX-7.23
         nsh> date
         Jan 01 00:00:06 1970
         nsh> date -s "DEC 25 08:00:00 2017"
         nsh> date
         Dec 25 08:00:01 2017

       After reset:

         NuttShell (NSH) NuttX-7.23
         nsh> date
         Dec 25 08:00:05 2017

  nxwm:

    This is a special configuration setup for the NxWM window manager
    UnitTest.  This builds on top of the features that were unit tested in
    by the fb configuration.

    The NxWM window manager can be found here:

      apps/graphics/NxWidgets/nxwm

    The NxWM unit test can be found at:

      apps/graphics/NxWidgets/UnitTests/nxwm

  pwfb:

    This configuration uses the test at apps/examples/pwfb to verify the
    operation of the per-window framebuffer.  That example shows three
    windows containing text moving around, crossing each other from
    "above" and from "below".  The example application is NOT updating the
    windows any anyway!  The application is only changing the window
    position.  The windows are being updated from the per-winidow
    frame buffers automatically.

    This example is reminiscent of Pong:  Each window travels in straight
    line until it hits an edge, then it bounces off.  The window is also
    raised when it hits the edge (gets "focus").  This tests all
    combinations of overlap.

      2019-03-19:  Everything works fine!

  pwlines:

    This configuration uses the test at apps/examples/pwline.  It is another
    verification of the operation of the per-window frame buffers.  This
    examples is very similar to the pwfb example used in pwfb configuration
    except that instead of text, each window has an (trivial) animated
    graphic (based on the rotating line of apps/examples/nslines).

      2019-03-20:  Everything works fine!

  twm4nx1 and twmnx2:

    These configuration exercises the port of TWM to NuttX.  A description of
    that port is available at apps/graphics/twm4nx/README.txt.  The two
    configurations are identical, differing on in the "theme" of the UI.
    twm4nx1 uses framed windows in dark, bright primary colors reminiscent of
    Windows98. twm4nx2 uses border-less windows in pastel shades for a more
    contemporary look.

    NOTES:
    1. This version uses the on-board display with the touchscreen for
       positional input (instead of a mouse).  Keyboard input is currently
       disabled only because (1) there is no Twm4Nx application that needs
       it, and (2) I will first need to create a USB host driver to support
       a USB keyboard.

    STATUS:

    Refer to apps/graphics/twm4nx/README.txt for an overall status.  Here
    are just some issues/topics unique to the LPCXpresso-LPC54628 and/or
    this configuration.

    1. There is a responsive-ness issue the the FT5x06 touchscreen controller.
       The pin selected by the board designers will not support interrupts.
       Therefore, a fall-back polled mode is use.  This polled mode has
       significant inherent delays that effect the user experience when
       touching buttons or grabbing and moving objects on the desktop.

    2. The NxTerm application is available as the "NuttShell" entry in the
       Main Menu.  When pressed, this will bring up an NSH session in a
       Twm4Nx window.  There is a performance issue, however, due to another
       issue with the polled mode in the ft5x06 driver.  When that driver
       runs in polled mode, it samples the touch data at a high rate.  Each
       sample is sufficient to wake up the Twm4Nx poll() with POLLIN data
       availability.  But when Twm4Nx tries to read the data, it falls under
       the FT5x06.c there threshold and no data is returned.

       The actual delay various dynamically from 50 to 200 millisecond
       intervals.

       A possible solution might be to beef up the POLLIN notification logic
       in FT5x06.c to avoid these false poll() wake-ups.  That, however, is
       non-trivial since it would have to support the polled as well as the
       non-polled mode.

    3. Color artifacts:  In the CLASSIC configuration, the background of the
       central NX image is a slightly different hue of blue.  For the
       CONTEMPORARY configuration, the toolbar buttons are supposed to be
       borderless.  There is however, a fine border around each toolbar
       widget with ruins the feel that the theme was trying for.

    4. Revisiting this configuration after a few months, I found that the
       configuration generated a hard fault.  This turned out to be an issue
       with the toolchain and dropping the optimization level eliminated the
       problem; -O2 was sufficient for me.  If you see odd behavior like
       this, you might want to do the same.
