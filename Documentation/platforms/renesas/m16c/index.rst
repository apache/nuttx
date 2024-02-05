============
Renesas M16C
============

**Renesas M16C/26 Microcontroller**. This port uses the Renesas SKP16C26
Starter kit and the GNU M32C toolchain. The development environment is
either Linux or Cygwin under WinXP.

**STATUS:** Initial source files released in nuttx-0.4.2. At this point,
the port has not been integrated; the target cannot be built because the
GNU ``m16c-nuttx-elf-ld`` link fails with the following message:

Where the reference line is:

No workaround is known at this time. This is a show stopper for M16C.
Refer to the NuttX board
`README <https://github.com/apache/nuttx/blob/master/boards/renesas/m16c/skp16c26/README.txt>`__
file for further information.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
