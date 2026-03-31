==========
sam4cmp-db
==========

README for the NuttX port to the SAM4CMP-DB board.

The `SAM4CMP-DB <http://www.atmel.com/tools/SAM4CMP-DB.aspx>`_ board is used
to exercise NuttX SMP support for dual Cortex-M4 systems.

Settings
========

1. Both CPUs run at 92.160 MHz using PLLB.
2. The serial console is available via the on-board USB-to-UART interface at
   115200 8N1.
3. Interrupt handlers such as the timer and UART run on CPU0.
4. Both CPUs share internal SRAM0 (128 KB).
5. SRAM1 is used to boot CPU1.
6. Cache controllers are disabled because the device has no snooping support.

Status
======

The SMP feature works on the board, but it is not yet stable.

1. ``nsh> sleep 1 &`` works without crashing.
2. ``nsh> smp`` sometimes works, but assertions may occur.
3. ``nsh> ostest`` causes deadlocks during the test.
