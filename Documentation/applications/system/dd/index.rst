==========================
``dd`` system 'dd' command
==========================

Overview
========

The ``dd`` command copies data from an input file or device to an output
file or device, optionally performing conversions along the way. It is
commonly used for tasks such as backing up and restoring raw disk
partitions, creating disk images, and converting data formats.

The name ``dd`` stands for "convert and copy" (historically "disk dump").
It reads from ``stdin`` by default and writes to ``stdout`` by default,
making it useful in pipelines and for low-level I/O operations.

Configuration
=============

Enable the command with ``CONFIG_SYSTEM_DD``. This option is enabled by
default unless ``CONFIG_DEFAULT_SMALL`` is set.

The following configuration options are available:

``CONFIG_SYSTEM_DD_PROGNAME``
  Program name for the ``dd`` command. Default: ``dd``.

``CONFIG_SYSTEM_DD_PRIORITY``
  Task priority for the ``dd`` command. Default: 100.

``CONFIG_SYSTEM_DD_STACKSIZE``
  Stack size for the ``dd`` command. Default: ``DEFAULT_TASK_STACKSIZE``.

``CONFIG_SYSTEM_DD_STATS``
  Enable transfer statistics output. Default: yes.

Usage
=====

.. code-block:: console

   dd [if=<infile>] [of=<outfile>] [bs=<sectsize>] [count=<sectors>] [skip=<sectors>] [seek=<sectors>] [verify] [conv=<nocreat,notrunc>]

Options
=======

``if=<infile>``
  Input file. If not specified, ``stdin`` is used.

``of=<outfile>``
  Output file. If not specified, ``stdout`` is used.

``bs=<sectsize>``
  Block size in bytes. Default: 512 bytes.

``count=<sectors>``
  Number of blocks to copy. Default: copy until end of input.

``skip=<sectors>``
  Skip ``<sectors>`` blocks at the start of the input.

``seek=<sectors>``
  Skip ``<sectors>`` blocks at the start of the output.

``verify``
  Verify that the output matches the input after copying. Requires both
  ``if`` and ``of`` to be specified.

``conv=<nocreat,notrunc>``
  Conversion options:

  - ``nocreat``: Do not create the output file if it does not exist.
  - ``notrunc``: Do not truncate the output file before writing.

  Multiple conversion options can be separated by commas.

``--help``
  Display usage information and exit.

Examples
========

Copy from stdin to stdout:

.. code-block:: console

   nsh> dd if=/dev/zero of=/tmp/zero.bin bs=1024 count=10
   10+0 records in
   10+0 records out

Create a disk image:

.. code-block:: console

   nsh> dd if=/dev/sda of=/tmp/disk.img bs=512
   1024+0 records in
   1024+0 records out

Restore a disk image:

.. code-block:: console

   nsh> dd if=/tmp/disk.img of=/dev/sda bs=512
   1024+0 records in
   1024+0 records out

Copy with verification:

.. code-block:: console

   nsh> dd if=/dev/sda of=/tmp/sda_backup.img bs=512 verify
   1024+0 records in
   1024+0 records out

Skip blocks in input:

.. code-block:: console

   nsh> dd if=/dev/sda of=/tmp/partition.img bs=512 skip=63 count=1024
   1024+0 records in
   1024+0 records out

Display usage information:

.. code-block:: console

   nsh> dd --help
   usage:
     dd [if=<infile>] [of=<outfile>] [bs=<sectsize>] [count=<sectors>] [skip=<sectors>] [seek=<sectors>] [verify] [conv=<nocreat,notrunc>]

Notes
=====

- The default block size is 512 bytes, which is the standard sector size
  for most storage devices.
- When ``count`` is not specified, ``dd`` copies until the end of the
  input is reached.
- The ``verify`` option requires both ``if`` and ``of`` to be specified.
  It reads back the output and compares it to the input.
- Transfer statistics are displayed when ``CONFIG_SYSTEM_DD_STATS`` is
  enabled. The output shows the number of records read/written and the
  transfer rate.
- The ``conv`` options can be combined by separating them with commas.
  For example, ``conv=nocreat,notrunc``.
- ``dd`` is often used in pipelines with other commands. For example,
  ``dd if=/dev/urandom bs=1 count=10 | hexdump``.
