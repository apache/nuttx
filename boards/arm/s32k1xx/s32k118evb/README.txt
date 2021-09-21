README
======

This directory holds the port to the NXP S32K118EVB-Q064 development board.

Contents
========

  o Status
  o Serial Console
  o LEDs and Buttons
  o OpenSDA Notes
  o Configurations

Status
======

  2019-08-14:  Configuration created but entirely untested.

  2019-08-17:  The port is code complete.  It compiles with no errors or
    warnings but is untested.  Still waiting for hardware.

  2019-08-20:  The very first image that I wrote to FLASH seems to have
    "bricked" the board.  The board is sensitive to (1) resetting into a bad
    state and (2) incorrect flash configurations.  It is difficult to
    impossible to recover from these start-up errors.

  2019-08-22:  My S32K118EVB is still borked, but after some additional
    changes, Fabio Balzano has verified that the NSH is functional on that
    board.

  2019-10-19: The aforementioned fixes for the FLASH issues were converted
    into a set of FLASH configuration options, with a proven default state.

  2020-06-15:  Added FlexCAN driver with SocketCAN support to the S32K1XX
    arch.  Might work on the S32K118EVB as well, but remains untested.

  2020-06-16:  Added Emulated EEPROM driver and initialization.

  TODO:  Need to calibrate the delay loop.  The current value of
  CONFIG_BOARD_LOOPSPERMSEC is a bogus value retained from a copy-paste
  (see apps/examples/calib_udelay).

Serial Console
==============

  By default, the serial console will be provided on the OpenSDA VCOM port:

    OpenSDA UART RX  PTB0  (LPUART0_RX)
    OpenSDA UART TX  PTB1  (LPUART0_TX)

  USB drivers for the PEmicro CDC Serial Port are available here:
  http://www.pemicro.com/opensda/

LEDs and Buttons
================

  LEDs
  ----
  The S32K118EVB has one RGB LED:

    RedLED    PTD16  (FTM0 CH1)
    GreenLED  PTD15  (FTM0 CH0)
    BlueLED   PTE8   (FTM0 CH6)

  An output of '1' illuminates the LED.

  If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
  any way.  The following definitions are used to access individual RGB
  components (see s32k118evb.h):

    GPIO_LED_R
    GPIO_LED_G
    GPIO_LED_B

  The RGB components could, alternatively, be controlled through PWM using
  the common RGB LED driver.

  If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board
  the S32K118EVB.  The following definitions describe how NuttX controls the
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
    LED_IDLE         S32K118 in sleep mode             (no change)
    ==========================================+========+========+=========

  Buttons
  -------
  The S32K118EVB supports two buttons:

    SW2  PTD3
    SW3  PTD5

OpenSDA Notes
=============

  - USB drivers for the PEmicro CDC Serial Port are available here:
    http://www.pemicro.com/opensda/

  - The drag'n'drog interface expects files in .srec format.

  - Using Segger J-Link:  Easy... but remember to use the SWD connector J14
    near the touch electrodes and not the OpenSDA connector near the OpenSDA
    USB connector J7.

Configurations
==============

  Common Information
  ------------------
  Each S32K118EVB configuration is maintained in a sub-directory and can be
  selected as follows:

    tools/configure.sh s32k118evb:<subdir>

  Where <subdir> is one of the sub-directories listed in the next paragraph.

    NOTES (common for all configurations):

    1. This configuration uses the mconf-based configuration tool.  To change
       this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt.
          Also see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Unless otherwise stated, the serial console used is LPUART0 at
       115,200 8N1.  This corresponds to the OpenSDA VCOM port.

  Configuration Sub-directories
  -----------------------------

    nsh:
    ---
      Configures the NuttShell (nsh) located at apps/examples/nsh.  Support
      for builtin applications is enabled, but in the base configuration no
      builtin applications are selected.

      NOTE:  This is a very minimal NSH configuration in order to reduce
      memory usage.  Most of the NSH niceties are not available.
