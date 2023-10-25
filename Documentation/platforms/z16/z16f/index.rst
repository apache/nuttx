===============
Zilog ZNEO Z16F
===============

**Zilog z16f2800100zcog development kit**. This port use the Zilog
z16f2800100zcog development kit and the Zilog ZDS-II Windows command
line tools. The development environment is either Windows native or
Cygwin under Windows.

**STATUS:** The initial release of support for the z16f was made
available in NuttX version 0.3.7. A working NuttShell (NSH)
configuration as added in NuttX-6.33 (although a patch is required to
work around an issue with a ZDS-II 5.0.1 tool problem). An ESPI
driver was added in NuttX-7.2. Refer to the NuttX board
`README <https://github.com/apache/nuttx/blob/master/boards/z16/z16f/z16f2800100zcog/README.txt>`__
file for further information.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
