README
^^^^^^

  This README provides some information about the port of NuttX to the TI
  Hercules TMS570LS04x/03x LaunchPad Evaluation Kit (LAUNCHXL-TMS57004)
  featuring the Hercules TMS570LS0432PZ chip.

Contents
^^^^^^^^

  LEDs and Buttons
  Serial Console

LEDs and Buttons
================

  LEDs
  ----
  The launchpad has a four LEDs two power LEDs labeled D1 (red) that
  connects to the TMS570's NERROR pin and D7 (blue) that indicates the
  XDS200 POWER_EN signal, and two white, user LEDs labeled D12 that
  connects to the NHET08 pin and D11 that connects to GIOA2.

  NHET08 is one of 32 N2HET pins than can be available to the user if
  not used by N2HET.  This implementation, however, uses only the single
  LED driven by GIOA2.  That LED is tied to ground and illuminated
  with a high level output value.

  This LED is not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/tms570_autoleds.c. The LED is used to encode
  OS-related events as follows:

  ------------------- ----------------------- ------
  SYMBOL              Meaning                 LED
  ------------------- ----------------------- ------
  LED_STARTED         NuttX has been started  OFF
  LED_HEAPALLOCATE    Heap has been allocated OFF
  LED_IRQSENABLED     Interrupts enabled      OFF
  LED_STACKCREATED    Idle stack created      ON
  LED_INIRQ           In an interrupt         N/C
  LED_SIGNAL          In a signal handler     N/C
  LED_ASSERTION       An assertion failed     N/C
  LED_PANIC           The system has crashed  FLASH

  Thus if the LED is statically on, NuttX has successfully  booted and is,
  apparently, running normally.  If the LED is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

  Buttons
  -------
  The launchpad has three mechanical buttons. Two of these are reset buttons:
  One button is labeled PORRST performs a power-on reset and one labeled RST
  performs an MCU reset.  Only one button is available for general software
  usage.  That button is labeled GIOA7 and is, obviously, sensed on GIOA7.

  GIOA7 is tied to ground, but will be pulled high if the GIOA7 button is
  depressed.

Serial Console
^^^^^^^^^^^^^^

  This TMS570 has a single SCI.  The SCI_RX and TX pins are connected to
  the FTDI chip which provides a virtual COM port for the launchpad.

