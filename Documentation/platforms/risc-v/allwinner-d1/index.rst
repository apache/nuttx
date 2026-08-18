============
Allwinner D1
============

The Allwinner D1 is a 64-bit RISC-V SoC based on a single T-Head C906 core.
It implements the RV64IMAFDC ISA and provides an Sv39 MMU, a platform-level
interrupt controller (PLIC), general-purpose timers, UARTs and other
peripherals.  Boards typically pair the SoC with external DDR3 memory.

NuttX runs in supervisor mode under OpenSBI.  The initial port uses a FLAT
build and does not consume a device tree.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
