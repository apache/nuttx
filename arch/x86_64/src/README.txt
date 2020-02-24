arch/x86/src/README.txt
^^^^^^^^^^^^^^^^^^^^^^^

This directory holds x86_64-specific source files.  All x86 source reside in
lower-level common, chip-specific, and architecture-specific directories.

common/ Directory
^^^^^^^^^^^^^^^^^

This directory holds source files common to all x86_64 architectures.

Architecture-Specific Directories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Architecture-specific directories hold common source files shared for by
implementations of specific x86_64 architectures.

intel64
  This directory holds logic appropriate for any instantiation of the 64-bit
  intel64 architecture.

Chip-Specific directories
^^^^^^^^^^^^^^^^^^^^^^^^^

The same x86 architecture may be realized in different chip implementations.
For SoC chips, in particular, on-chip devices and differing interrupt
structures may require special, chip-specific definitions in these chip-
specific directories.

broadwell
  This is the implementation of NuttX on the Intel Broadwell processors.
