README
======

This directory holds the port to the NXP RDDRONE-BMS772 board with S32K144
MCU.

A NuttX compatible smart battery application for RDDRONE-BMS772 is also
available.  It contains additional drivers and example software to use
most features of the battery management system.

This application is currently published in a separate repository, but
(parts) may eventually be upstreamed to Apache NuttX:
https://github.com/NXPHoverGames/RDDRONE-BMS772


Contents
========

  o Status
  o Serial Console
  o LEDs
  o Thread-Aware Debugging with Eclipse
  o Configurations

Status
======

  2020-11-02:  Configuration first published in separate GitHub repository.
    Tested with the aforementioned BMS application.

  2022-05-25:  Basic board configuration prepared for upstreaming to Apache NuttX.

Serial Console
==============

  By default, the serial console will be provided on the DCD-LZ UART
  (available on the 7-pin DCD-LZ debug connector J19):

    DCD-LZ UART RX  PTC6  (LPUART1_RX)
    DCD-LZ UART TX  PTC7  (LPUART1_TX)

LEDs and Buttons
================

  LEDs
  ----
  The RDDRONE-BMS772 has one RGB LED:

    RedLED    PTD16  (FTM0 CH1)
    GreenLED  PTB13  (FTM0 CH1)
    BlueLED   PTD15  (FTM0 CH0)

  An output of '0' illuminates the LED.

  If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
  any way.  The following definitions are used to access individual RGB
  components (see rddrone-bms772.h):

    GPIO_LED_R
    GPIO_LED_G
    GPIO_LED_B

  The RGB components could, alternatively, be controlled through PWM using
  the common RGB LED driver.

  If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board
  the RDDRONE-BMS772.  The following definitions describe how NuttX controls
  the LEDs:

    ==========================================+========+========+=========
                                                 RED     GREEN     BLUE
    ==========================================+========+========+=========

    LED_STARTED      NuttX has been started      OFF      OFF      OFF
    LED_HEAPALLOCATE Heap has been allocated     OFF      OFF      ON
    LED_IRQSENABLED  Interrupts enabled          OFF      OFF      ON
    LED_STACKCREATED Idle stack created          OFF      ON       OFF
    LED_INIRQ        In an interrupt                   (no change)
    LED_SIGNAL       In a signal handler               (no change)
    LED_ASSERTION    An assertion failed               (no change)
    LED_PANIC        The system has crashed      FLASH    OFF      OFF
    LED_IDLE         S32K144 in sleep mode             (no change)
    ==========================================+========+========+=========

Thread-Aware Debugging with Eclipse
===================================

  Thread-aware debugging is possible with openocd-nuttx
  ( https://github.com/sony/openocd-nuttx ) and was tested together with the
  Eclipse-based S32 Design Studio for Arm:
  https://www.nxp.com/design/software/development-software/s32-design-studio-ide/s32-design-studio-for-arm:S32DS-ARM

  NOTE: This method was last tested with NuttX 8.2 and S32DS for Arm 2018.R1.
  It may not work anymore with recent releases of NuttX and/or S32DS.

  1. NuttX should be build with debug symbols enabled.

  2. Build OpenOCD as described here (using the same parameters as well):
     https://micro.ros.org/docs/tutorials/old/debugging/

  3. A s32k144.cfg file is available in the scripts/ folder.  Start OpenOCD
     with the following command (adapt the path info):
     /usr/local/bin/openocd -f /usr/share/openocd/scripts/interface/jlink.cfg \
     -f boards/s32k1xx/rddrone-bms772/scripts/s32k144.cfg -c init -c "reset halt"

  4. Setup a GDB debug session in Eclipse.  The resulting debug window shows
     the NuttX threads.  The full stack details can be viewed.

Configurations
==============

  Common Information
  ------------------
  Each RDDRONE-BMS772 configuration is maintained in a sub-directory and can be
  selected as follows:

    tools/configure.sh rddrone-bms772:<subdir>

  Where <subdir> is one of the sub-directories listed in the next paragraph.

    NOTES (common for all configurations):

    1. This configuration uses the mconf-based configuration tool.  To change
       this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt.
          Also see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Unless otherwise stated, the serial console used is LPUART1 at
       115,200 8N1.  This corresponds to the OpenSDA VCOM port.

  Configuration Sub-directories
  -----------------------------

    nsh:
    ---
      Configures the NuttShell (nsh) located at apps/examples/nsh.  Support
      for builtin applications is enabled, but in the base configuration the
      only application selected is the "Hello, World!" example.
