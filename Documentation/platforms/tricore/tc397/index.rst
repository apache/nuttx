=============
TriCore/TC397
=============

**TriCore/TC397** An TriCore flat address port was ported in NuttX-12.0. It
consists of the following feautres:

- Runs in Supervisor Mode.
- IRQs are managed by Interrupt Router (INT), IR Service Request Control Registers (SRC).
- Used System timer (STM) for systick.

This kernel with ostest have been tested with

-  Infineon's AURIX™ TC397 Evaluation Board: KIT_A2G_TC397_5V_TFT

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
