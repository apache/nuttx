README.txt
==========

  This README file discusses the port of NuttX to the WaveShare Open1788 board:
  See http://wvshare.com/product/Open1788-Standard.htm. This board features the
  NXP LPC1788 MCU

CONTENTS
========

  o LEDs
  o Buttons
  o Serial Console
  o Using OpenOCD with the Olimex ARM-USB-OCD
  o Loading Code with the ISP Board
  o Configuration

LEDs
====

  The Open1788 base board has four user LEDs

    LED1 : Connected to P1[14]
    LED2 : Connected to P0[16]
    LED3 : Connected to P1[13]
    LED4 : Connected to P4[27]

  If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
  any way using the definitions provided in the board.h header file.

  If CONFIG_ARCH_LEDs is defined, then NuttX will control the 3 LEDs on the
  WaveShare Open1788K.  The following definitions describe how NuttX controls
  the LEDs:
                               LED1 LED2 LED3 LED4
    LED_STARTED                OFF  OFF  OFF  OFF
    LED_HEAPALLOCATE           ON   OFF  OFF  OFF
    LED_IRQSENABLED            OFF   ON  OFF  OFF
    LED_STACKCREATED           ON    ON  OFF  OFF
    LED_INIRQ                  LED3 glows, on while in interrupt
    LED_SIGNAL                 LED3 glows, on while in signal handler
    LED_ASSERTION              LED3 glows, on while in assertion
    LED_PANIC                  LED3 Flashes at 2Hz
    LED_IDLE                   LED glows: ON while active; OFF while sleeping

Buttons
=======

  The Open1788K supports several buttons:

    USER1           : Connected to P4[26]
    USER2           : Connected to P2[22]
    USER3           : Connected to P0[10]

  And a Joystick

    JOY_A           : Connected to P2[25]
    JOY_B           : Connected to P2[26]
    JOY_C           : Connected to P2[23]
    JOY_D           : Connected to P2[19]
    JOY_CTR         : Connected to P0[14]

  These can be accessed using the definitions and interfaces defined in the
  board.h header file.

Serial Console
==============

  By Default, UART0 is used as the serial console in all configurations.  This
  may be connected to your computer via an external RS-232 driver or via the
  WaveShare USB ISP/VCOM module.

  As an option, UART1 can also be used for the serial console.  You might want,
  to do this, for example, if you use UART0 for the ISP function and you want
  to use a different UART for console output.  UART1 can be configured as the
  serial console by changing the configuration as follows:

    System Type:
      CONFIG_LPC17_40_UART0=n          : Disable UART0 if it is no longer used
      CONFIG_LPC17_40_UART1=y          : Enable UART1

    Drivers:
      CONFIG_UART1_SERIAL_CONSOLE=y : Setup up the UART1 configuration
      CONFIG_UART1_RXBUFSIZE=256
      CONFIG_UART1_TXBUFSIZE=256
      CONFIG_UART1_BAUD=115200
      CONFIG_UART1_BITS=8
      CONFIG_UART1_PARITY=0
      CONFIG_UART1_2STOP=0

  In this configuration using UART1, it is necessary to disable LED support
  on the board.  That is because UART1 RXD is set for pin p0.16, but so is
  LED2.  If you do not disable LED support then no incoming serial data will
  be received.

    Common Board Options
      CONFIG_ARCH_LEDS=n             : Disable LED support

  You should also remove the LED2 jumper so that the RXD input does not
  attempt to drive LED2 as well (However, this does not seem to interfere with
  data receipt).

  NOTE:  If you intend to use LEDs with UART1, then you might want to
  redesign some of the LED logic in the src/ subdirectory so that it does not
  attempt to use LED2.

Using OpenOCD with the Olimex ARM-USB-OCD
=========================================

  Building OpenOCD under Cygwin:

    Refer to boards/olimex-lpc1766stk/README.txt

  Installing OpenOCD in Ubuntu Linux:

    sudo apt-get install openocd

  Helper Scripts.

    I have been using the Olimex ARM-USB-OCD debugger.  OpenOCD
    requires a configuration file.  I keep the one I used last here:

      boards/arm/lpc17xx_40xx/open1788/tools/open1788.cfg

    However, the "correct" configuration script to use with OpenOCD may
    change as the features of OpenOCD evolve.  So you should at least
    compare that open1788.cfg file with configuration files in
    /usr/share/openocd/scripts.  As of this writing, the configuration
    files of interest were:

      /usr/local/share/openocd/scripts/interface/openocd-usb.cfg
        This is the configuration file for the Olimex ARM-USB-OCD
        debugger.  Select a different file if you are using some
        other debugger supported by OpenOCD.

      /usr/local/share/openocd/scripts/board/?
        I don't see a board configuration file for the WaveShare
        Open1788 board.

      /usr/local/share/openocd/scripts/target/lpc1788.cfg
        This is the configuration file for the LPC1788 target.
        It just sets up a few parameters then sources lpc17xx.cfg

      /usr/local/share/openocd/scripts/target/lpc17xx.cfg
        This is the generic LPC configuration for the LPC17xx/LPC40xx
        family.  It is included by lpc1788.cfg.

    NOTE:  These files could also be located under /usr/share in some
    installations.  They could be most anywhwere if you are using a
    windows version of OpenOCD.

      boards/arm/lpc17xx_40xx/open1788/tools/open1788.cfg
        This is simply openocd-usb.cfg, lpc1788.cfg, and lpc17xx.cfg
        concatenated into one file for convenience.  Don't use it
        unless you have to.

    There is also a script on the tools/ directory that I use to start
    the OpenOCD daemon on my system called oocd.sh.  That script will
    probably require some modifications to work in another environment:

    - Possibly the value of OPENOCD_PATH and TARGET_PATH
    - It assumes that the correct script to use is the one at
      boards/arm/lpc17xx_40xx/open1788/tools/open1788.cfg

  Starting OpenOCD

    Then you should be able to start the OpenOCD daemon as follows.  This
    assumes that you have already CD'ed to the NuttX build directory and
    that you have set the full path to the onfigs/open1788/tools in your
    PATH environment variable:

      oocd.sh $PWD

    or, if the PATH variable is not so configured:

      boards/arm/lpc17xx_40xx/open1788/tools/oocd.sh $PWD

  Connecting GDB

    Once the OpenOCD daemon has been started, you can connect to it via
    GDB using the following GDB command:

      arm-nuttx-elf-gdb
      (gdb) target remote localhost:3333

    NOTE:  The name of your GDB program may differ.  For example, with the
    CodeSourcery toolchain, the ARM GDB would be called arm-none-eabi-gdb.

    OpenOCD will support several special 'monitor' sub-commands.  You can
    use the 'monitor' (or simply 'mon') command to invoke these sub-
    commands. These GDB commands will send comments to the OpenOCD monitor.
    Here are a couple that you will need to use:

     (gdb) monitor reset
     (gdb) monitor halt

    NOTES:

    1. The MCU must be halted using 'monitor halt' prior to loading code.

    2. 'monitor reset' will restart the processor after loading code.

    3. The 'monitor' command can be abbreviated as just 'mon'.

    After starting GDB, you can load the NuttX ELF file like this:

      (gdb) mon halt
      (gdb) load nuttx

    NOTES:

    1. NuttX should have been built so that it has debugging symbols
       (by setting CONFIG_DEBUG_SYMBOLS=y in the .config file).

    2. The MCU must be halted prior to loading code.

    3. I find that there are often undetected write failures when using
       the Olimex ARM-USB-OCD debugber and that if you start the program
       with a bad FLASH failure, it will lock up OpenOCD.  I usually
       oad nuttx twice, restarting OpenOCD in between in order to assure
       good FLASH contents:

      (gdb) mon halt
      (gdb) load nuttx
      (gdb) mon reset

      Exit GDB, kill the OpenOCD server, recycle power on the board,
      restart the OpenOCD server and GDB, then:

      (gdb) mon halt
      (gdb) load nuttx
      (gdb) mon reset

      Other debuggers may not have these issues and such drastic steps may
      not be necessary.

Loading Code with the ISP Board
===============================

  Use can also load code onto the board using the WaveShare and the UART0
  ISP/VCOM board.  I use the FlashMagic program for Windows available here:
  http://www.flashmagictool.com/ . It is so easy to use that no further
  explanation should be necessary:  Just select the LPC1788, the ISP COM
  port, and the NuttX .hex file and program it.

CONFIGURATION
=============

Information Common to All Configurations
----------------------------------------

  1. These configurations use the mconf-based configuration tool.  To
     change this configuration using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository
        README.txt.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

  2. Most (but not all) configurations use the "GNU Tools for ARM
     Embedded Processors" that is maintained by ARM:

       https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

     unless otherwise stated.

     That toolchain selection can easily be reconfigured using
     'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Window environment
       CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABI=y  : GNU ARM EABI toolchain

  3. By Default, UART0 is used as the serial console in all configurations.
     This may be connected to your computer via an external RS-232 driver or
     via the WaveShare USB ISP/VCOM module.  See the section above entitled
     "Serial Console" for other options.

Configuration Directories
-------------------------

  fb
  --

    A simple configuration used for some basic (non-graphic) debug of the
    framebuffer character drivers using apps/examples/fb.  This
    configuration enables SDRAM to hold the LCD framebuffer and enables
    the LPC178x LCD driver in order to support the WaveShare 4.3 inch TFT
    panel.

    NOTES:

    1. In this configuration, the SDRAM is not added to heap but is
       dedicated to supporting an LCD frame buffer at address 0xa0010000.

  knsh
  ----
    This is identical to the nsh configuration below except that NuttX
    is built as a kernel-mode, monolithic module and the user applications
    are built separately.  Is is recommended to use a special make command;
    not just 'make' but make with the following two arguments:

        make pass1 pass2

    In the normal case (just 'make'), make will attempt to build both user-
    and kernel-mode blobs more or less interleaved.  This actual works!
    However, for me it is very confusing so I prefer the above make command:
    Make the user-space binaries first (pass1), then make the kernel-space
    binaries (pass2)

    1. Uses the older, OABI, buildroot toolchain.  But that is easily
       reconfigured:

       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot toolchain
       CONFIG_ARMV7M_OABI_TOOLCHAIN=y      : Older, OABI toolchain

    2. This configuration has DMA-based SD card support enabled by
       default.  That support can be disabled as follow:

       CONFIG_LPC17_40_GPDMA=n                : No DMA
       CONFIG_ARCH_DMA=n
       CONFIG_LPC17_40_SDCARD=n               : No SD card driver
       CONFIG_SDIO_DMA=n                   : No SD card DMA
       CONFIG_MMCSD=n                      : No MMC/SD driver support
       CONFIG_FS_FAT=n                     : No FAT file system support

    3. At the end of the build, there will be several files in the top-level
       NuttX build directory:

       PASS1:
         nuttx_user.elf    - The pass1 user-space ELF file
         nuttx_user.hex    - The pass1 Intel HEX format file (selected in defconfig)
         User.map          - Symbols in the user-space ELF file

       PASS2:
         nuttx             - The pass2 kernel-space ELF file
         nuttx.hex         - The pass2 Intel HEX file (selected in defconfig)
         System.map        - Symbols in the kernel-space ELF file

       Loading these .elf files with OpenOCD is tricky.  It appears to me
       that when nuttx_user.elf is loaded, it destroys the nuttx image
       in FLASH.  But loading the nuttx ELF does not harm the nuttx_user.elf
       in FLASH.  Conclusion:  Always load nuttx_user.elf before nuttx.

       Just to complicate matters, it is sometimes the case that you need
       load objects twice to account for write failures.  I have not yet
       found a simple foolproof way to reliably get the code into FLASH.

    4. Combining .hex files.  If you plan to use the .hex files with your
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

  knxterm
  -------

    This is another protected mode build.  This configuration was used to
    verify that NxTerms can be used in protected mode.  See the knsh
    configuration notes for instructions on building and loading this
    configuration.

    This is a very minimal configuration intended only for low level testing.
    To be useful, the font sizes, windows sizes, and line widths would need
    to be optimized to make a friendlier display.

  nsh
  ---
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables only the serial NSH interface.

    NOTES:

    1. Uses the older, OABI, buildroot toolchain.  But that is easily
       reconfigured:

       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot toolchain
       CONFIG_ARMV7M_OABI_TOOLCHAIN=y      : Older, OABI toolchain

    2. This NSH has support for built-in applications enabled, however,
       no built-in configurations are built in the defulat configuration.

    3. This configuration has DMA-based SD card support enabled by
       default.  That support can be disabled as follow:

       CONFIG_LPC17_40_GPDMA=n                : No DMA
       CONFIG_ARCH_DMA=n
       CONFIG_LPC17_40_SDCARD=n               : No SD card driver
       CONFIG_SDIO_DMA=n                   : No SD card DMA
       CONFIG_MMCSD=n                      : No MMC/SD driver support
       CONFIG_FS_FAT=n                     : No FAT file system support

    4. This configuration has been used for verifying SDRAM by modifying
       the configuration in the following ways:

       CONFIG_LPC17_40_EMC=y                  : Enable the EMC
       CONFIG_LPC17_40_EXTDRAM=y              : Configure external DRAM
       CONFIG_LPC17_40_EXTDRAMSIZE=67108864   : DRAM size 2x256/8 = 64MB
       CONFIG_SYSTEM_RAMTEST=y             : Enable the RAM test built-in

       In this configuration, the SDRAM is not added to heap and so is
       not excessible to the applications.  So the RAM test can be
       freely executed against the SRAM memory beginning at address
       0xa000:0000 (CS0).

    6. This configuration has been used for verifying the touchscreen on
       on the 4.3" LCD module.

       a) As of this writing, this touchscreen is still not functional.
          Rommel Marcelo has tracked this problem down to noise on the
          PENIRQ interrupt.  There are so many false interrupts that
          the NuttX interrupt-driven touchscreen driver cannot be used.
          Other compatible LCDs, however, may not have this issue.

       b) You can enable the touchscreen by modifying the configuration
          in the following ways:

          Drivers:
            CONFIG_INPUT=y                    : Enable support for input devices
            CONFIG_INPUT_ADS7843E=y           : Enable support for the XPT2048
            CONFIG_ADS7843E_SPIDEV=1          : Use SSP1 for communication
            CONFIG_SPI=y                      : Enable SPI support
            CONFIG_SPI_EXCHANGE=n             : exchange() method is not supported

          System Type:
            CONFIG_LPC17_40_GPIOIRQ=y            : GPIO interrupt support
            CONFIG_LPC17_40_SSP1=y               : Enable support for SSP1

          Library Support:
            CONFIG_SCHED_WORKQUEUE=y          : Work queue support required

          Application Configuration:
            CONFIG_EXAMPLES_TOUCHSCREEN=y     : Enable the touchscreen built-int test

          Defaults should be okay for related touchscreen settings.  Touchscreen
          debug output can be enabled with:

          Build Setup:
            CONFIG_DEBUG_FEATURES=y           : Enable debug features
            CONFIG_DEBUG_INFO=y               : Enable verbose debug output
            CONFIG_DEBUG_INPUT=y              : Enable debug output from input devices

       c) You will also have to disable SD card support to use this test.  The
          SD card detect (CD) signal is on P0[13].  This signal is shared.  It
          is also used for MOSI1 and USB_UP_LED.  The CD pin may be disconnected.
          There is a jumper on board that enables the CD pin.  OR, you can simply
          remove the SD module so that it does not drive the CD pin.

          Drivers:
            CONFIG_MMCSD=n                    : No MMC/SD driver support

          System Type:
            CONFIG_LPC17_40_GPDMA=n              : No DMA
            CONFIG_LPC17_40_SDCARD=n             : No SD card driver
            CONFIG_SDIO_DMA=n                 : No SD card DMA
            CONFIG_ARCH_DMA=n

          File Systems:
            CONFIG_FS_FAT=n                   : No FAT file system support

          For touchscreen debug output:

          Build Setup:
            CONFIG_DEBUG_FEATURES=y
            CONFIG_DEBUG_INFO=y
            CONFIG_DEBUG_INPUT=y

  nxlines
  -------
    Configures the graphics example located at examples/nsh.  This
    configuration enables SDRAM to hold the LCD framebuffer and enables
    the LPC178x LCD driver in order to support the WaveShare 4.3 inch TFT
    panel.

    NOTES:

    1. Uses the default toolchain for the selected platform.  But that is
       easily reconfigured.

    2. In this configuration, the SDRAM is not added to heap but is
       dedicated to supporting an LCD frame buffer at address 0xa0010000.

  pdcurses
  -------

    A simple NSH configuration used for some basic (non-graphic) debug of
    the pdcurses library on top of a framebuffer character drivers.  This
    configuration provides the test programs from:

      - apps/examples/pdcurses,
      - apps/examples/fb, and
      - apps/examples/djoystick

    as NSH built-in applications.  A discrete joystick is provided to
    control pdcurses examples using the joystick buttons on the Open1788
    board.

    This configuration enables SDRAM to hold the LCD framebuffer and
    enables the LPC178x LCD driver in order to support the WaveShare 4.3
    inch TFT panel.  In this configuration, the SDRAM is not added to heap
    but is dedicated to supporting an LCD frame buffer at address
    0xa0010000.

    STATUS:
    2017-11-20:  Basic graphics functionality appears to be functional, but
       is not fully tested.

       Only keyboard and mouse input are supported by pdcurses.  NuttX
       supports only USB HID keyboard and mouse.  It would require a hub to
       use them simultaneously.  In a handheld device with an ncurses-style
       UI, I don't think that a mouse (or even a touchscreen) makes sense.

       For a handheld device, I think input would be via GPIO keypad, rather
       that a full keyboard, and I doubt that you would do any significant
       text data entry.  I think that up-down-left-right arrows keys and an
       enter key is basically all you need for most interaction.

       In NuttX naming that is called a discrete joystick djoystick.  There
       is a well defined djoystick interface in include/nuttx/input/djoystick.h.
       Note that there are discrete joystick buttons on the Open1788 board so
       would be a natural interface in this case.

       A discrete joystick driver was added to pdcurses configuration.  It
       has been verified that the pdcurses demos that require menu
       interactions work well with the discrete joystick.

  pwfb:
  -----

    This configuration uses the test at apps/examples/pwfb to verify the
    operation of the per-window framebuffers.  That example shows three
    windows containing text moving around, crossing each other from
    "above" and from "below".  The example application is NOT updating the
    windows any anyway!  The application is only changing the window
    position.  The windows are being updated from the per-winidow
    framebuffers automatically.

    This example is reminiscent of Pong:  Each window travels in straight
    line until it hits an edge, then it bounces off.  The window is also
    raised when it hits the edge (gets "focus").  This tests all
    combinations of overap.

    STATUS:
      2019-03-16:  The test does not succeed.  There are still numerou
        problems to be overcome.
      2019-03-18:  Every works fine!  Hmm... except the colors some off,
        everything is too blue????
