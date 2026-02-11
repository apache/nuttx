===============
Freescale M9S12
===============

.. tags:: vendor:freescale

Freescale M68HCS12
------------------

**MC9S12NE64**. Support for the MC9S12NE64 MCU and two boards are
included:

-  The Freescale DEMO9S12NE64 Evaluation Board, and
-  The Future Electronics Group NE64 /PoE Badge board.

Both use a GNU arm-nuttx-elf toolchain\* under Linux or Cygwin. The
NuttX `buildroot <https://bitbucket.org/nuttx/buildroot/downloads/>`__
provides a properly patched GCC 3.4.4 toolchain that is highly optimized
for the m9s12x family.

Features
========

* 16-bit HCS12 core

  * HCS12 CPU
  * Upward compatible with M68HC11 instruction set
  * Interrupt stacking and programmer's model identical to M68HC11
  * Instruction queue
  * Enhanced indexed addressing
  * Memory map and interface (MMC)
  * Interrupt control (INT)
  * Background debug mode (BDM)
  * Enhanced debug12 module, including breakpoints and change-of-flow trace
    buffer (DBG)
  * Multiplexed expansion bus interface (MEBI) - available only in 112-pin
    package version

* Wakeup interrupt inputs

  * Up to 21 port bits available for wakeup interrupt function with digital
    filtering

* Memory

  * 64K bytes of FLASH EEPROM
  * 8K bytes of RAM

* Analog-to-digital converter (ATD)

  * One 8-channel module with 10-bit resolution
  * External conversion trigger capability

* Timer module (TIM)

  * 4-channel timer
  * Each channel configurable as either input capture or output compare
  * Simple PWM mode
  * Modulo reset of timer counter
  * 16-bit pulse accumulator
  * External event counting
  * Gated time accumulation

* Serial interfaces

  * Two asynchronous serial communications interface (SCI)
  * One synchronous serial peripheral interface (SPI)
  * One inter-IC bus (IIC)

* Ethernet Media access controller (EMAC)

  * IEEE 802.3 compliant
  * Medium-independent interface (MII)
  * Full-duplex and half-duplex modes
  * Flow control using pause frames
  * MII management function
  * Address recognition
  * Frames with broadcast address are always accepted or always rejected
  * Exact match for single 48-bit individual (unicast) address
  * Hash (64-bit hash) check of group (multicast) addresses
  * Promiscuous mode

* Ethertype filter
* Loopback mode
* Two receive and one transmit Ethernet buffer interfaces

* Ethernet 10/100 Mbps transceiver (EPHY)

  * IEEE 802.3 compliant
  * Digital adaptive equalization
  * Half-duplex and full-duplex
  * Auto-negotiation next page ability
  * Baseline wander (BLW) correction
  * 125-MHz clock generator and timing recovery
  * Integrated wave-shaping circuitry
  * Loopback modes

* CRG (clock and reset generator module)

  * Windowed COP watchdog
  * Real-time interrupt
  * Clock monitor
  * Pierce oscillator
  * Phase-locked loop clock frequency multiplier
  * Limp home mode in absence of external clock
  * 25-MHz crystal oscillator reference clock

* Operating frequency

  * 50 MHz equivalent to 25 MHz bus speed for single chip
  * 32 MHz equivalent to 16 MHz bus speed in expanded bus modes

* Internal 2.5-V regulator

  * Supports an input voltage range from 3.3 V ± 5%
  * Low-power mode capability
  * Includes low-voltage reset (LVR) circuitry

* 80-pin TQFP-EP or 112-pin LQFP package

  * Up to 70 I/O pins with 3.3 V input and drive capability (112-pin package)
  * Up to two dedicated 3.3 V input only lines (IRQ, XIRQ)

* Development support

  * Single-wire background debug mode (BDM)
  * On-chip hardware breakpoints
  * Enhanced DBG debug features

FreeScale HCS12 Serial Monitor
==============================

The NuttX HCS12 port is configured to use the Freescale HCS serial monitor. This
monitor supports primitive debug commands that allow FLASH/EEPROM programming
and debugging through an RS-232 serial interface. The serial monior is 2Kb in
size and resides in FLASH at addresses ``0xf800`` to ``0xffff``. The monitor
does not use any RAM other than the stack itself.

.. note::

   The serial monitor is described in detail in Freescale Application Note
   AN2458.pdf.

**COP:**

The serial monitor uses the COP for the cold reset function and should not be
used by the application without some precautions (see AN2458).

**Clocking:**

The serial monitor sets the operating frequency to 24 MHz. This is not altered
by the NuttX start-up; doing so would interfere with the operation of the serial
monitor.

**Memory Configuration:**

Register space is located at 0x0000-0x03ff.

FLASH memory is any address greater than 0x4000. All paged addresses are assumed
to be FLASH memory. Application code should exclude the 0xf780-0xff7f memory.

RAM ends at 0x3FFF and builds down to the limit of the device's available RAM.
The serial monitor's stack pointer is set to the end of RAM+1 (0x4000).

EEPROM (if the target device has any) is limited to the available space between
the registers and the RAM (0x0400 to start of RAM).

External devices attached to the multiplexed external bus interface are not
supported.

**Interrupts:**

The serial monitor redirects interrupt vectors to an unprotected portion of
FLASH just before the protected monitor program (0xf780-0xf7fe). The
monitor will automatically redirect vector programming operations to these
user vectors. The user code should therefore keep the normal (non-monitor)
vector locations (0xff80-0xfffe).

Serial Communication
====================

The serial monitor uses RS-232 serial communications through ``SCI0`` at 115200
baud. The monitor must have exclusive use of this interface. Access to the
serial port is available through a monitor jump table.

Soft Registers
==============

The mc68hcs12 compilation is prone to errors like the following:

.. code:: console

   CC:  lib_b16sin.c
   lib_b16sin.c: In function `b16sin':
   lib_b16sin.c:110: error: unable to find a register to spill in class `S_REGS'
   lib_b16sin.c:110: error: this is the insn:
   (insn:HI 41 46 44 8 (parallel [
               (set (subreg:SI (reg:DI 58 [ rad ]) 4)
                   (reg/v:SI 54 [ rad ]))
               (clobber (scratch:HI))
           ]) 20 {movsi_internal} (insn_list 46 (nil))
       (expr_list:REG_UNUSED (scratch:HI)
           (expr_list:REG_NO_CONFLICT (reg/v:SI 54 [ rad ])
               (nil))))
   lib_b16sin.c:110: confused by earlier errors, bailing out

There are several ways that this error could be fixed:

1. Increase the number of soft registers (i.e., "fake" registers defined at
   fixed memory locations). This can be done by adding something like
   ``-msoft-reg-count=4`` to the ``CFLAGS``. This approach was not taken
   because:

   * This slows hcs12 performance

   * All of these soft registers would have to be saved and restored on every
     interrupt and context switch.

2. Lowering the optimization level by dropping ``-Os`` to ``-O2`` or, more
   likely, by removing ``-fomit-frame-pointer``. Also not desirable because 99%
   of the files that do not have this problem also increase in size. Special
   case compilation with reduced optimization levels just for the files that
   need it could be done, but this would complicate the make system.

3. Restructuring files to reduce the complexity. If you add local variables to
   hold intermediate computational results, this error can be eliminated. This
   is the approach taken in NuttX. It has disadvantages only in that (1) it
   takes some effort and good guessing to eliminate the problem, and (2) the
   problem is not really eliminated -- it can and will re-occur when files are
   changed or new files are added.

4. Many files are built that are needed by DEM09S12NE64. Another very simple
   option if those problem files are needed is to just remove the offending
   files from the Make.defs file so that they no longer cause a problem.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
