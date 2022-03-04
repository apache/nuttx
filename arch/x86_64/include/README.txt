arch/x86_64/include/README.txt
^^^^^^^^^^^^^^^^^^^^^^^^^^^

This directory holds x86_64-specific header files.  The top-level header files in
arch/x86_64/include simply include corresponding header files from lower lower-
level chip-specific and architecture-specific directories.

Architecture-Specific Directories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Architecture-specific directories hold common header files for specific x86_64
architectures.

intel64
  This directory holds definitions appropriate for any instantiation of the
  Intel architecture in 64bit long mode.

Chip-Specific directories
^^^^^^^^^^^^^^^^^^^^^^^^^

The same x86 architecture may be realized in different chip implementations.
For SoC chips, in particular, on-chip devices and differing interrupt
structures may require special, chip-specific definitions in these chip-
specific directories.

broadwell
  This is the implementation of NuttX on the Intel Broadwell processors.

