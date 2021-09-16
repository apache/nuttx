README
======

This directory holds the port to the NXP UCANS32K146 boards.  There exist a
few different revisions/variants of this board.  All variants with the
S32K146 microcontroller are supported.

Contents
========

  o Status
  o Serial Console
  o LEDs and Buttons
  o Thread-Aware Debugging with Eclipse
  o Configurations

Status
======

  2020-01-23:  Configuration created (copy-paste from S32K146EVB).
    Tested: Serial console, I2C, SPI.

  2020-06-15:  Added FlexCAN driver with SocketCAN support to the S32K1XX
    arch, which has been tested with the UCANS32K146 board as well.

  2020-06-16:  Added Emulated EEPROM driver and initialization.

Serial Console
==============

  By default, the serial console will be provided on the DCD-LZ UART
  (available on the 7-pin DCD-LZ debug connector P6):

    OpenSDA UART RX  PTC6  (LPUART1_RX)
    OpenSDA UART TX  PTC7  (LPUART1_TX)

  USB drivers for the PEmicro CDC Serial Port are available here:
  http://www.pemicro.com/opensda/

LEDs and Buttons
================

  LEDs
  ----
  The UCANS32K146 has one RGB LED:

    RedLED    PTD15  (FTM0 CH0)
    GreenLED  PTD16  (FTM0 CH1)
    BlueLED   PTD0   (FTM0 CH2)

  An output of '0' illuminates the LED.

  If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
  any way.  The following definitions are used to access individual RGB
  components (see ucans32k146.h):

    GPIO_LED_R
    GPIO_LED_G
    GPIO_LED_B

  The RGB components could, alternatively, be controlled through PWM using
  the common RGB LED driver.

  If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board
  the UCANS32K146.  The following definitions describe how NuttX controls the
  LEDs:

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
    LED_IDLE         S32K146 in sleep mode             (no change)
    ==========================================+========+========+=========

  Buttons
  -------
  The UCANS32K146 supports one button:

    SW3  PTC14

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

  3. A s32k146.cfg file is available in the scripts/ folder.  Start OpenOCD
     with the following command (adapt the path info):
     /usr/local/bin/openocd -f /usr/share/openocd/scripts/interface/jlink.cfg \
     -f boards/s32k1xx/ucans32k146/scripts/s32k146.cfg -c init -c "reset halt"

  4. Setup a GDB debug session in Eclipse.  The resulting debug window shows
     the NuttX threads.  The full stack details can be viewed.

Configurations
==============

  Common Information
  ------------------
  Each UCANS32K146 configuration is maintained in a sub-directory and can be
  selected as follows:

    tools/configure.sh ucans32k146:<subdir>

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

    can:
    ---
      Besides the NuttShell this configuration also enables (Socket)CAN
      support, as well as I2C and SPI support.  It includes the SLCAN and
      can-utils applications for monitoring and debugging CAN applications.
