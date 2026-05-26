===================
``tee`` Tee Command
===================

Overview
========

The ``tee`` command copies standard input to standard output and to each
specified file.  It is useful in scripts or command pipelines when output
needs to be displayed and saved at the same time.

When no files are specified, ``tee`` copies standard input to standard
output only.

Configuration
=============

Enable the application with ``CONFIG_SYSTEM_TEE``.

The task priority and stack size are controlled by:

- ``CONFIG_SYSTEM_TEE_PRIORITY``
- ``CONFIG_SYSTEM_TEE_STACKSIZE``

Usage
=====

.. code-block:: console

   tee [-a] [file ...]
   tee -h

Options
=======

.. list-table::
   :header-rows: 1

   * - Option
     - Description
   * - ``-a``
     - Append to each output file instead of truncating it.
   * - ``-h``
     - Show command usage and exit.

Arguments
=========

``file``
  Optional output file.  Any number of files may be specified.  Output is
  written to standard output in addition to each named file.

Examples
========

Copy input to standard output and save it in ``output.txt``:

.. code-block:: console

   nsh> echo "hello" | tee output.txt
   hello

Append input to an existing log file while also displaying it:

.. code-block:: console

   nsh> echo "next line" | tee -a log.txt
   next line

Copy input to more than one file:

.. code-block:: console

   nsh> echo "sample" | tee first.txt second.txt
   sample

Notes
=====

The command opens each named file for writing.  Without ``-a``, existing
files are truncated; with ``-a``, output is appended.  Files that do not
already exist are created with mode ``0644``.

If opening an output file, reading from standard input, or writing to an
output destination fails, the command exits with failure status.
