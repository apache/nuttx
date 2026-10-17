=================================
``pffault`` User-Space Fault Test
=================================

``pffault`` makes one access to a kernel address from a user task. In a
protected or kernel build, the access must fault, the kernel must terminate
only this task, and the shell must keep running. ``pffault`` prints the
access before it makes it. If the access returns, ``pffault`` prints that,
and the build did not stop it. For a test with a control target and
resource checks, see :doc:`../sandbox/index`.

Usage::

  pffault [r|w] [<address>]

``w`` writes the address; any other first argument reads it. To give an
address, give the first argument too. The default address is
``0x3fc98000``, the base of the kernel DRAM of the ESP32-S3. Give an address
for other chips.

Configuration
=============

- ``CONFIG_EXAMPLES_PFFAULT`` -- Enable the test.
- ``CONFIG_EXAMPLES_PFFAULT_PROGNAME``, ``CONFIG_EXAMPLES_PFFAULT_PRIORITY``
  and ``CONFIG_EXAMPLES_PFFAULT_STACKSIZE`` -- Program name, priority and
  stack size.
