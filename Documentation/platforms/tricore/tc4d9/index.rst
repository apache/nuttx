=============
TriCore/TC4DA
=============

**TriCore/TC4DA** An TriCore flat address port was ported in NuttX-12.0. It
consists of the following features:

- Runs in Supervisor Mode.
- IRQs are managed by Interrupt Router (INT), IR Service Request Control Registers (SRC).
- Used System timer (STM) for systick.

This kernel with ostest have been tested with

-  Infineon's AURIX™ TC4DA Evaluation Board: TRIBOARD_TC4X9_COM

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
