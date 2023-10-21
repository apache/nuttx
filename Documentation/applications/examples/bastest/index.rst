``bastest`` Bas BASIC Interpreter
=================================

This directory contains a small program that will mount a ROMFS file system
containing the BASIC test files extracted from the Bas ``2.4`` release. See
``examples/bastest/README.md`` for licensing and usage information.

- ``CONFIG_EXAMPLES_BASTEST_DEVMINOR`` – The minor device number of the ROMFS
  block driver. For example, the ``N`` in ``/dev/ramN``. Used for registering the
  RAM block driver that will hold the ROMFS file system containing the BASIC
  files to be tested. Default: ``0``.

- ``CONFIG_EXAMPLES_BASTEST_DEVPATH`` – The path to the ROMFS block driver device.
  This must match ``EXAMPLES_BASTEST_DEVMINOR``. Used for registering the RAM
  block driver that will hold the ROMFS file system containing the BASIC files
  to be tested. Default: ``/dev/ram0``.
