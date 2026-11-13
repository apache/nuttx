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

The chip support programs the AURIX registers directly and does not need
the Infineon Low Level Drivers (iLLDs).  The register definitions of each
chip family are in ``arch/tricore/src/<chip>/hardware``, and the drivers
common to TC3xx and TC4xx, such as the ASCLIN UART, are in
``arch/tricore/src/aurix``.

``TC3xx/TC4xx``
  This is the implementation of NuttX on the Infineon’s AURIX™- TC3xx/TC4xx microcontroller family.

.. toctree::
   :maxdepth: 1
   :glob:

   */*
