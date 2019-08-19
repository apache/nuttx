README
======

This directory hold the port to the NXP S32K118EVB-Q064 Development board.

Contents
========

  o Status
  o Serial Console
  o LEDs and Buttons
  o Configurations

Status
======

  2019-08-14:  Configuration created but entirely untested.  Support for the
    S32K1XX family is incomplete.  This configuration is intended, initially,
    to support development of the architecture support.  This is VERY much
    a work in progress and you should not use this configuration unless you
    are interested in assisting with the bring-up.
  2019-08-17:  The port is code complete.  It compiles with no errors or
    warnings but is untested.  Still waiting for hardware.

Serial Console
==============

  By default, the serial console will be provided on the OpenSDA VCOM port:

    OpenSDA UART TX  PTB1(LPUART0_TX)
    OpenSDA UART RX  PTB0(LPUART0_RX)

LEDs and Buttons
================

  LEDs
  ----
  The S32K118EVB has one RGB LED:

    RedLED   PTD16 (FTM0 CH1)
    GreenLED PTD15 (FTM0 CH0)
    BlueLED  PTE8  (FTM0 CH6)

  If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
  any way.  The following definitions are used to access individual RGB
  components.

  The RGB components could, alternatively be controlled through PWM using
  the common RGB LED driver.

  If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board
  the s32k118evb.  The following definitions describe how NuttX controls the
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
    LED_IDLE         S32K118EVN in sleep mode          (no change)
    ==========================================+========+========+=========

  Buttons
  -------
  The S32K118EVB supports two buttons:

    SW2  PTD3
    SW3  PTD5

Configurations
==============

  Common Information
  ------------------
  Each S32K118EVB configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh s32k118evb:<subdir>

  Where <subdir> is one of the sub-directories listed in the next paragraph

    NOTES (common for all configurations):

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Unless otherwise stated, the serial console used is LPUART0 at
       115,200 8N1.  This corresponds to the OpenSDA VCOM port.

  Configuration Sub-directories
  -------------------------

    nsh:
    ---
      Configures the NuttShell (nsh) located at apps/examples/nsh.  Support
      for builtin applications is enabled, but in the base configuration no
      builtin applications are selected.
