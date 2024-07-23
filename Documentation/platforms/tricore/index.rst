=======
TriCore
=======

All TriCore source reside in lower-level common, chip-specific, and architecture-specific
directories.

arch/tricore/src/common/ Directory
==================================

This directory holds source files common to all TriCore architectures.

Architecture-Specific Directories
=================================

Architecture-specific directories hold common source files shared for by
implementations of specific TriCore architectures.

``TriCore``
  This directory holds logic appropriate for any instantiation of the 32-bit
  TriCore architecture.

Chip-Specific directories
=========================

For SoC chips, in particular, on-chip devices and differing interrupt
structures may require special, chip-specific definitions in these chip-
specific directories.

The core Chip implementation is based on Infineon Low Level Drivers (iLLDs).
The unified API is more friendly to developers familiar with Infineon SDK/HAL.
We can get more code examples on Infineon's official Github: `AURIX_code_examples <https://github.com/Infineon/AURIX_code_examples>`__

``TC3xx``
  This is the implementation of NuttX on the Infineon’s AURIX™- TC3xx microcontroller family.

.. toctree::
   :maxdepth: 1
   :glob:

   */*
