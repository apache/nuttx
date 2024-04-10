========
QEMU x86
========

**QEMU/Bifferboard i486**. This port uses the
`QEMU <http://wiki.qemu.org/Main_Page>`__ i486 and the native Linux,
Cygwin, MinGW the GCC toolchain under Linux or Cygwin.

**STATUS:** The basic port was code-complete in NuttX-5.19 and verified
in NuttX-6.0. The port was verified using the OS and NuttShell (NSH)
examples under QEMU. The port is reported to be functional on the
`Bifferboard <http://bifferos.bizhat.com>`__ as well. In NuttX 7.1,
Lizhuoyi contributed additional keyboard and VGA drivers. This is a
great, stable starting point for anyone interested in fleshing out the
x86 port! Refer to the NuttX
`README <https://github.com/apache/nuttx/blob/master/Documentation/platforms/x86/qemu/boards/qemu-i486/README.txt>`__
file for further information.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
