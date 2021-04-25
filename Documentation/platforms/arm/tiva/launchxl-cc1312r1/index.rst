====================
TI LaunchXL-CC1312R1
====================

SimpleLink Sub-1 GHz CC1312R Wireless Microcontroller (MCU) LaunchPad Development Kit

`LaunchXL-CC1312R1 <https://www.ti.com/tool/LAUNCHXL-CC1312R1>`_


Serial Console
==============

The on-board XDS110 Debugger provide a USB virtual serial console using
UART0 (DIO2_RXD and DIO3_TXD).

A J-Link debugger is used (see below), then the RXD/TXD jumper pins can
be used to support a serial console through these same pins via an
appropriate TTL level adapter (RS-232 or USB serial).

LEDs and Buttons
================

LEDs
----

The LaunchXL-cc1312R1 has two LEDs controlled by software:  DIO7_GLED (CR1)
and DIO6_RLED (CR2).  A high output value illuminates an LED.

=========  ==== ========================
DIO7_GLED  CR1  High output illuminates
DIO6_RLED  CR2  High output illuminates
=========  ==== ========================

If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
any way.  The definitions provided in the board.h header file can be used
to access individual LEDs.

These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/cc1312_autoleds.c. The LEDs are used to encode
OS-related events as follows:

================== ======================== ====== ======
SYMBOL              Meaning                 G-LED   R-LED
================== ======================== ====== ======
LED_STARTED        NuttX has been started   OFF    OFF
LED_HEAPALLOCATE   Heap has been allocated  OFF    ON
LED_IRQSENABLED    Interrupts enabled       OFF    ON
LED_STACKCREATED   Idle stack created       ON     OFF
LED_INIRQ          In an interrupt          N/C    GLOW
LED_SIGNAL         In a signal handler      N/C    GLOW
LED_ASSERTION      An assertion failed      N/C    GLOW
LED_PANIC          The system has crashed   OFF    Blinking
LED_IDLE           MCU is is sleep mode     N/A    N/A
================== ======================== ====== ======

Thus if GLED is statically on, NuttX has successfully booted and is,
apparently, running normally.  A soft glow of the RLED means that the
board is taking interrupts.   If GLED is off and GLED is flashing at
approximately 2Hz, then a fatal error has been detected and the system
has halted.

Buttons
-------

The LaunchXL-CC1312R1 has two push-puttons:

=========== ==== ===============================
DIO13_BTN1  SW1  Low input sensed when depressed
DIO14_BTN2  SW2  Low input sensed when depressed
=========== ==== ===============================

Version 1 or 2?
===============

Two versions of the CC1312R1 are supported selected by CONFIG_ARCH_CHIP_CC13XX_V1
or CONFIG_ARCH_CHIP_CC13XX_V2.  It is not clear how to identify the chip version
from markings on it.

What you can do is enable CONFIG_DEBUG_ASSERTIONS.  The firmware can
determine which version you have by looking at register contents.  The
firmware will assert if you select the wrong version.  If that occurs,
switch to the other version and the assertion should go away.

Running from SRAM
=================

The LaunchXL-CC1312R1 port supports execution from RAM.  Execution from
SRAM as a "safe" way to bring up a new board port without concern about
borking the board because of a bad FLASH image.

if CONFIG_BOOT_RUNFROMFLASH=y is set in the configuration, then the code
will build to run from FLASH.  Otherwise (presumably CONFIG_BOOT_RUNFROMSRAM=y)
the code will build to run from SRAM.  This is determined by the Make.defs
file in the scripts/ sub-directory.  Based on those configuration
settings, either scripts/flash.ld or sram.ld will be selected as the
linker script file to be used.

Using J-Link
============

Reference https://wiki.segger.com/CC1310_LaunchPad (for the CC1310 but also
applies to the CC1312R1):

When shipped, the TI CC1312R1 LaunchPad evaluation board is configured to be
used with the on-board debug probe.  In order to use it with J-Link, the
on-board debug probe needs to be isolated to make sure that it does not
drive the debug signals.  This can be done by removing some jumpers next
to the XDS110 Out / CC1310 In connector [RXD, TXD, RST, TMS, TCK, TDO, TDI,
SWO].  After isolating the on-board probe, the CC1312R1 device can be
debugged using J-Link.  The J-Link needs to be connected to the board
using the micro JTAG connector marked "Target In".

NOTE:  When connecting the J-Link GDB server, the interface must be set to
JTAG, not SWD as you might expect.

The RXD/TXD pins, DIO2_RXD and DIO3_TXD, can then support a Serial console
using the appropriate TTL adapter (TTL to RS-232 or TTL to USB serial).

One odd behavior that is after a reset from the J-Link, the SP and PC
registers are not automatically set and have to be manually set as shown below:

.. code-block:: gdb

  (gdb) target remote localhost:2331
  (gdb) mon reset
  (gdb) mon halt
  (gdb) file nuttx
  (gdb) mon memu32 0
  Reading from address 0x00000000 (Data = 0x20001950)
  (gdb) mon memu32 4
  Reading from address 0x00000004 (Data = 0x00000139)
  (gdb) mon reg sp 0x20001950
  Writing register (SP = 0x20001950)
  (gdb) mon reg pc 0x00000139
  Writing register (PC = 0x00000139)
  (gdb) n
  232       cc13xx_trim_device();
