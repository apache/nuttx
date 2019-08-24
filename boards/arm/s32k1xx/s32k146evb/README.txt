README
======

This directory hold the port to the NXP S32K146EVB-Q144 Development board.

Contents
========

  o Status
  o Serial Console
  o LEDs and Buttons
  o OpenSDA Notes
  o Configurations

Status
======

  2019-08-18:  Configuration created but entirely untested.

  2019-08-20:  For initial testing, I ran out of SRAM to avoid the
    brickage problems I had with the S32K118EVB (i.e., with
    CONFIG_BOOT_RUNFROMISRAM=y). In this mode, the NSH configuration
    appears worked correctly.

  2019-18-21:  Writing a relocated version of that same functional binary
    into FLASH, however, did not work and, in fact, bricked my S32K146EVB.
    That is because the first version of the FLASH image that I used
    clobbered the FLASH Configuration bytes at address 0x0400 (I
    didn't even know about these!).  I have since modified the linker script
    to skip this are in FLASH.  There is some fragmentary discussion
    for recovery from this condition at: https://community.nxp.com/thread/505593 .
    But none of those options are working for me.

    Give the success running of of SRAM and the success of the same fixes
    on the S32K118, I believe that the NSH configuration should now run out
    of FLASH.  Unfortunately, I cannot demonstrate that.

  TODO:  Need calibrate the delay loop.  The current value of
  CONFIG_BOARD_LOOPSPERMSEC is a bogus value retained from a copy-paste
  (see apps/examples/calib_udelay).

Serial Console
==============

  By default, the serial console will be provided on the OpenSDA VCOM port:

    OpenSDA UART TX  PTC7 (LPUART1_TX)
    OpenSDA UART RX  PTC6 (LPUART1_RX)

  USB drivers for the PEMIcro CDC Serial port are available here:
  http://www.pemicro.com/opensda/

LEDs and Buttons
================

  LEDs
  ----
  The S32K146EVB has one RGB LED:

    RedLED   PTD15 (FTM0 CH0)
    GreenLED PTD16 (FTM0 CH1)
    BlueLED  PTD0  (FTM0 CH2)

  An output of '1' illuminates the LED.

  If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
  any way.  The following definitions are used to access individual RGB
  components.

  The RGB components could, alternatively be controlled through PWM using
  the common RGB LED driver.

  If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board
  the s32k146evb.  The following definitions describe how NuttX controls the
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
    LED_IDLE         S32K146EVN in sleep mode          (no change)
    ==========================================+========+========+=========

  Buttons
  -------
  The S32K146EVB supports two buttons:

    SW2  PTC12
    SW3  PTC13

OpenSDA Notes
=============

  - USB drivers for the PEMIcro CDC Serial port are available here:
    http://www.pemicro.com/opensda/

  - The drag'n'drog interface expects files in .srec format.

  - Using Segger J-Link:  Easy... but remember to use the SWD J14 connector
    in the center of the board and not the OpenSDA connector closer to the
    OpenSDA USB connector J7.

Configurations
==============

  Common Information
  ------------------
  Each S32K146EVB configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh s32k146evb:<subdir>

  Where <subdir> is one of the sub-directories listed in the next paragraph

    NOTES (common for all configurations):

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Unless otherwise stated, the serial console used is LPUART1 at
       115,200 8N1.  This corresponds to the OpenSDA VCOM port.

  Configuration Sub-directories
  -----------------------------

    nsh:
    ---
      Configures the NuttShell (nsh) located at apps/examples/nsh.  Support
      for builtin applications is enabled, but in the base configuration but
      the builtin applications selected is the "Hello, World!" example.
