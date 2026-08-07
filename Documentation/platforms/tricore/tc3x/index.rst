==========
AURIX TC3X
==========

NuttX support for the Infineon AURIX TC3X (TriCore TC1.6.2P) family.

The TC3X port covers:

- SCU clock tree bring-up (PLL, CCU dividers, ENDINIT password rotation).
- IR/SRC interrupt routing and ICR-driven priority arbitration.
- System Timer (STM) periodic tick.
- ASCLIN-based UART (NSH console).
- GPIO pinmuxing through the AURIX layer.

Boards select a particular TC3X variant through ``CONFIG_TC3X_CHIP_*``.

Memory map (TC375 CPU0 view)
============================

================= ============ =================================
Region            Base         Size / notes
================= ============ =================================
PFLASH (uncached) 0x80000000   4 MiB (program flash)
PFLASH (cached)   0xA0000000   4 MiB (alias of PFLASH)
DSPR0             0x70000000   240 KiB (CPU0 data scratch-pad)
PSPR0             0x70100000   64 KiB (CPU0 program scratch-pad)
LMU0              0x90000000   96 KiB (local memory unit)
================= ============ =================================

NuttX places the boot image in PFLASH starting at 0x80000000 (entry
``__start``).  CPU0's data, BSS and CSA pool live in DSPR0; the
interrupt stack lives in PSPR0; the heap lives in LMU0.

Interrupt Router mapping
========================

The Interrupt Router (IR) on TC3X exposes SRC registers at
``0xF0038000 + 4*SRN`` and CPU latched-active-service registers at
``0xF0037000 + LASR_offset``.

Peripherals used in the upstream port:

================ ======= =========================================
Peripheral       SRN     Default NuttX IRQ (Kconfig)
================ ======= =========================================
ASCLIN0 TX       20      ``CONFIG_TRICORE_UART0_TXIRQ`` = 20
ASCLIN0 RX       21      ``CONFIG_TRICORE_UART0_RXIRQ`` = 21
ASCLIN0 ER       22      ``CONFIG_TRICORE_UART0_ERIRQ`` = 22
ASCLIN1 TX       23      ``CONFIG_TRICORE_UART1_TXIRQ`` = 23
ASCLIN1 RX       24      ``CONFIG_TRICORE_UART1_RXIRQ`` = 24
ASCLIN1 ER       25      ``CONFIG_TRICORE_UART1_ERIRQ`` = 25
STM0 SR0/SR1     192/193 (per-CPU; CPU0 only in this port)
================ ======= =========================================

ENDINIT password rotation
=========================

TC3X SCU registers are write-protected by a CPU-wide ENDINIT and a
separate Safety ENDINIT.  Each unlock writes the inverted password to
the matching WDT...CON0 register.  The port uses direct SFR access for
the rotation; no iLLD password helper is linked in.  See
``arch/tricore/src/tc3x/tc3x_endinit.c`` and the corresponding
TC3X System Control Unit chapter in the AURIX TC3xx User Manual.

Supported boards
================

.. toctree::
   :maxdepth: 1
   :glob:

   */*
