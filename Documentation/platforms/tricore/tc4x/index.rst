==========
AURIX TC4X
==========

NuttX support for the Infineon AURIX TC4X (TriCore TC1.8P) family.

The TC4X port covers:

- SCU clock tree bring-up (PLL, CCU dividers, Safety/CPU ENDINIT
  unlock).
- IR/SRC interrupt routing (SRPN equals the NuttX IRQ number).
- System Timer (STM) periodic tick (64-bit ABS).
- ASCLIN-based UART (NSH console).
- GPIO pinmuxing through the AURIX layer.

Boards select a particular TC4X variant through ``CONFIG_TC4X_CHIP_*``.

Memory map (TC4D7 CPU0 view)
============================

================= ============ =================================
Region            Base         Size / notes
================= ============ =================================
PFLASH (uncached) 0x80000000   4 MiB (program flash, CPU0 bank)
PFLASH (cached)   0xA0000000   4 MiB (alias of PFLASH)
DSPR0             0x70000000   240 KiB (CPU0 data scratch-pad)
PSPR0             0x70100000   64 KiB (CPU0 program scratch-pad)
DLMU0             0x90000000   512 KiB (distributed LMU, CPU0)
LMU0              0x90400000   5 MiB (local memory unit)
================= ============ =================================

NuttX places the boot image in PFLASH starting at 0x80000000 (entry
``__start``).  CPU0's data, BSS and CSA pool live in DSPR0; the
interrupt stack lives in PSPR0; the heap lives in LMU0.

Interrupt Router mapping
========================

The TC4X Interrupt Router exposes SRC registers at
``0xF4432000 + 4*SRN`` and per-CPU LASR registers at
``0xF4430000 + 0x0C20 + tos*0x34``.  ``SRPN`` (Service Request Priority
Number) equals the NuttX IRQ number; no remapping is performed.

Peripherals used in the upstream port:

================ ==== =========================================
Peripheral       SRN  Default NuttX IRQ (Kconfig)
================ ==== =========================================
ASCLIN0 RX       172  ``CONFIG_TRICORE_UART0_RXIRQ`` = 172
ASCLIN0 TX       173  ``CONFIG_TRICORE_UART0_TXIRQ`` = 173
ASCLIN0 ER       174  ``CONFIG_TRICORE_UART0_ERIRQ`` = 174
ASCLIN1 RX       175  ``CONFIG_TRICORE_UART1_RXIRQ`` = 175
ASCLIN1 TX       176  ``CONFIG_TRICORE_UART1_TXIRQ`` = 176
ASCLIN1 ER       177  ``CONFIG_TRICORE_UART1_ERIRQ`` = 177
STM0 SR0/SR1     8/9  (per-CPU; CPU0 only in this port)
================ ==== =========================================

Clock tree
==========

The TC4X CCU clock tree is configured by
``arch/tricore/src/tc4x/tc4x_clockconfig.c``.  The implementation
brings up the external oscillator, RAMP, SYSPLL and PERPLL, then
programs SYSCCUCON0/1 and PERCCUCON0/1 to produce the target divider
ratios documented in ``arch/tricore/src/tc4x/hardware/tc4x_clock.h``.

ENDINIT unlock
==============

TC4X folds the ENDINIT password sequence into the clock configuration
itself rather than using a separate WDT password helper.  A single
direct-SFR routine in ``tc4x_clockconfig.c`` performs the required
Safety ENDINIT and CPU ENDINIT unlock/lock dance around every SCU
register write.

Supported boards
================

.. toctree::
   :maxdepth: 1
   :glob:

   */*
