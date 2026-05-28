============================
``lzf`` LZF compression tool
============================

Overview
========

The ``lzf`` command compresses and decompresses files using the LZF
algorithm by Marc Alexander Lehmann.  LZF is a very lightweight
compression format optimized for speed; it trades a modest compression
ratio for fast decompression that is nearly as fast as a memory copy.

The tool reads input files (or ``stdin`` when no files are given),
writes the result to a derived output file (or ``stdout``), and
optionally removes the original on success.

File format
-----------

An LZF stream consists of zero or more blocks.  Each block starts
with the two-byte magic ``ZV`` followed by a type byte:

- **Type 0** (5-byte header): uncompressed literal — 2-byte length,
  then raw data.
- **Type 1** (7-byte header): compressed — 2-byte compressed length,
  2-byte uncompressed length, then LZF-compressed data.

A ``\x00`` byte signals end-of-stream.

Configuration
=============

Enable the command with ``CONFIG_SYSTEM_LZF`` (tristate).  This
option depends on ``CONFIG_LIBC_LZF``, which provides the core
compression and decompression library.

Additional configuration symbols:

- ``CONFIG_SYSTEM_LZF_BLOG`` — Log2 of the block size used for
  compression and decompression (default ``10``, range 9–12).  The
  actual block size is ``(1 << BLOG) - 1`` bytes.  Larger values may
  yield slightly better compression at the cost of more static memory
  (approximately ``2 * (1 << BLOG)`` bytes of ``.bss``).
- ``CONFIG_SYSTEM_LZF_PROGNAME`` — Program name registered with NSH
  (default ``"lzf"``).
- ``CONFIG_SYSTEM_LZF_PRIORITY`` — Task priority (default ``100``).
- ``CONFIG_SYSTEM_LZF_STACKSIZE`` — Task stack size (default
  ``DEFAULT_TASK_STACKSIZE``).

Usage
=====

.. code-block:: console

   lzf [-cdfvh] [-b <size>] [file ...]
   lzf -h

When no ``file`` arguments are given, ``lzf`` reads from ``stdin``
and writes to ``stdout``.  The ``-f`` flag is required in this mode
if the data direction would otherwise be a terminal (to prevent
accidental interactive use).

When files are given, the output file name is derived automatically:

- **Compress** (default or ``-c``): the first ``.`` in the filename is
  replaced with ``_``, and a ``.lzf`` suffix is appended.  For
  example, ``data.bin`` becomes ``data_bin.lzf``.
- **Decompress** (``-d``): the ``.lzf`` suffix is stripped and the
  first ``_`` is replaced back with ``.``.  For example,
  ``data_bin.lzf`` becomes ``data.bin``.

On success the original file is removed (replaced).

Options
=======

.. list-table::
   :header-rows: 1

   * - Option
     - Description
   * - ``-c``
     - Compress (default mode).
   * - ``-d``
     - Decompress.
   * - ``-f``
     - Force overwrite of the output file.  Also suppresses the
       terminal-safety check when reading from ``stdin`` or writing
       to ``stdout``.
   * - ``-h``
     - Show usage information and exit.
   * - ``-v``
     - Verbose mode.  Prints the compression ratio and the output
       filename for each processed file on ``stderr``.
   * - ``-b <size>``
     - Set the block size in bytes.  Must be between 1 and
       ``(1 << CONFIG_SYSTEM_LZF_BLOG) - 1``.  Values outside this
       range are silently clamped to the compiled-in default.

Environment
-----------

When ``CONFIG_DISABLE_ENVIRON`` is not set, the ``LZF_BLOCKSIZE``
environment variable can override the default block size.  The
command-line ``-b`` option takes precedence over the environment
variable.

Examples
========

Compress a file (replaces ``sensor.log`` with
``sensor_log.lzf``):

.. code-block:: console

   nsh> lzf sensor.log

Decompress a file (replaces ``sensor_log.lzf`` with
``sensor.log``):

.. code-block:: console

   nsh> lzf -d sensor_log.lzf

Compress with verbose output showing the ratio:

.. code-block:: console

   nsh> lzf -v largefile.dat

Compress from ``stdin`` to ``stdout`` (pipeline):

.. code-block:: console

   nsh> cat data.bin | lzf -f > data.bin.lzf

Decompress from ``stdin`` to ``stdout``:

.. code-block:: console

   nsh> cat data.bin.lzf | lzf -df > data.bin

Set a smaller block size for low-memory systems:

.. code-block:: console

   nsh> lzf -b 512 sensor.log

Notes
=====

- The default mode is compress; ``-c`` is optional but may be used for
  clarity.
- ``-c`` and ``-d`` are mutually exclusive.  If both are specified the
  last one on the command line wins.
- When run in FLAT or PROTECTED build modes (``CONFIG_SYSTEM_LZF`` set
  to ``y``), the tool serializes concurrent invocations via a mutex
  because the compression hash table and I/O buffers are statically
  allocated.  In KERNEL build mode (``CONFIG_SYSTEM_LZF`` set to ``m``)
  each instance has its own address space and no serialization is
  needed.
- The LZF library is provided under a BSD-2-Clause license.
- The output file is created with mode ``0600`` (owner read/write only).
- If the output file already exists and ``-f`` is not given, the
  command fails with an error.
