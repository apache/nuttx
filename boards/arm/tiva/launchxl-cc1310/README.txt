README
======

  This directory holds NuttX board support for the TI LaunchXL-CC1310.  This
  board features the CC1310F128 part with 128Kb of FLASH and 20Kb of SRAM.

Contents
========

  o Status
  o Serial Console
  o LEDs and Buttons
  o Using J-Link

Status
======

  2019-01-21:  Fragmentary board support in place.  The initial intent
    of this board support is simply to assist in the CC13x0 architecture
    development.  Serious board development will occur later. At present,
    the CC13x0 does not even compile error-free:  Compilation of cc14x0_rom.c
    fails because the DDI0 OSC header file has not yet been ported.

Serial Console
==============

  The on-board XDS110 Debugger provide a USB virtual serial console using
  UART0 (PA0/U0RX and PA1/U0TX).

  A J-Link debugger is used (see below), then the RXD/TXD jumper pins can
  be used to support a serial console through appropriate TTL level adapter
  (RS-232 or USB serial).

LEDs and Buttons
================

LEDs
----

  The LaunchXL-cc1310 and two LEDs controlled by software: DIO7_GLED (CR1)
  and DIO6_RLED (CR2).  A high output value illuminates an LED.

    DIO7_GLED  CR1  High output illuminuates
    DIO6_RLED  CR2  High output illuminuates

  If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
  any way.  The definitions provided in the board.h header file can be used
  to access individual LEDs.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/cc1312_autoleds.c. The LEDs are used to encode
  OS-related events as follows:

    SYMBOL              Meaning                  LED state
                                                GLED   RLED
    ------------------ ------------------------ ------ ------
    LED_STARTED        NuttX has been started   OFF    OFF
    LED_HEAPALLOCATE   Heap has been allocated  OFF    ON
    LED_IRQSENABLED    Interrupts enabled       OFF    ON
    LED_STACKCREATED   Idle stack created       ON     OFF
    LED_INIRQ          In an interrupt          N/C    GLOW
    LED_SIGNAL         In a signal handler      N/C    GLOW
    LED_ASSERTION      An assertion failed      N/C    GLOW
    LED_PANIC          The system has crashed   OFF    Blinking
    LED_IDLE           MCU is is sleep mode      Not used

  Thus iF GLED is statically on, NuttX has successfully booted and is,
  apparently, running normally.  A soft glow of the RLED means that the
  board is taking interrupts.   If GLED is off and GLED is flashing at
  approximately 2Hz, then a fatal error has been detected and the system
  has halted.

Buttons
-------

  The LaunchXL-CC1310 has two push-puttons:

  DIO13_BTN1  SW1  Low input sensed when depressed
  DIO14_BTN2  SW2  Low input sensed when depressed

Using J-Link
============

  Reference https://wiki.segger.com/CC1310_LaunchPad:

  When shipped, the TI CC1310 LaunchPad evaluation board is configured to be
  used with the on-board debug probe.  In order to use it with J-Link, the
  on-board debug probe needs to be isolated to make sure that it does not
  drive the debug signals.  This can be done by removing some jumpers next
  to the XDS110 Out / CC1310 In connector [RXD, TXD, RST, TMS, TCK, TDO, TDI,
  SWO].  After isolating the on-board probe, the CC130F128 device can be
  debugged using J-Link.  The J-Link needs to be connected to the board
  using the micro JTAG connector marked "In".

  I use the Olimex ARM-JTAG-20-10 to interface with the board:
  https://www.olimex.com/Products/ARM/JTAG/ARM-JTAG-20-10/

  NOTE:  When connecting the J-Link GDB server, the interface must be set to
  JTAG, not SWD as you might expect.

  The RXD/TXD pins. PA0/U0RX and PA1/U0TX, can then support a Serial console
  using the appropriate TTL adapter (TTL to RS-232 or TTL to USB serial).
