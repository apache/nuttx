=====
RH850
=====

.. tags:: arch:rh850, vendor:renesas

The RH850 port runs NuttX on the Renesas RH850 family with its G4KH CPU core
(V850E3v5 instruction set, RH850 ABI).  Sources are in ``arch/rh850``.

arch/rh850/src/common/ Directory
================================

Logic common to all RH850 chips: reset entry and vector table, exception frame
save and restore, the TRAP 0 system call used for context switches, EIINT
interrupt entry with interrupt nesting, CPU exception reporting, stacks, signals
and the idle loop.

Chip-Specific Directories
=========================

``u2c``
  The RH850/U2C (R7F702600) chip: clock and standby control, the INTC1/INTC2
  interrupt controllers, the OSTM0 system timer and the RLIN3 UART.

.. toctree::
   :maxdepth: 1
   :glob:

   */*
