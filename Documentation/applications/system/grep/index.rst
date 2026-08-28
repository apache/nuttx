==============================
``grep`` system 'grep' command
==============================

Overview
========

The ``grep`` command searches for lines matching a regular expression in
text files (or standard input). It uses the NuttX libc regular expression
library (``regcomp`` / ``regexec``), so it supports real regular
expressions including anchors (``^``, ``$``) and character classes
(``[...]``), not just fixed substring matching.

By default ``grep`` reads from ``stdin`` when no file is given, which
makes it useful in NSH pipelines. When more than one file is searched (or
when recursing into directories), the matching file name is prefixed to
each result line.

Configuration
=============

Enable the command with ``CONFIG_SYSTEM_GREP``. This option depends on
``CONFIG_LIBC_REGEX`` and is disabled by default.

The following configuration options are available:

``CONFIG_SYSTEM_GREP_PROGNAME``
  Program name for the ``grep`` command. Default: ``grep``.

``CONFIG_SYSTEM_GREP_PRIORITY``
  Task priority for the ``grep`` command. Default: 100.

``CONFIG_SYSTEM_GREP_STACKSIZE``
  Stack size for the ``grep`` command. Default:
  ``DEFAULT_TASK_STACKSIZE``.

Usage
=====

.. code-block:: console

   grep [-invrHh] <pattern> [<file>...]

When no ``<file>`` is given, ``grep`` reads from ``stdin``.

Options
=======

``-i``
  Case-insensitive match.

``-n``
  Prefix each matching line with its 1-based line number.

``-v``
  Invert the match: select lines that do **not** match.

``-r``
  Recurse into subdirectories. When set, the file name is always prefixed
  to each result.

``-H``
  Always print the file name prefix, even with a single input file.

``-h``
  Never print the file name prefix.

Exit status
===========

``0``
  At least one line matched.

``1``
  No line matched.

``2``
  An error occurred (invalid option, unreadable file, bad regex, ...).

Examples
========

Search ``stdin`` for lines containing ``error``:

.. code-block:: console

   nsh> dmesg | grep error

Case-insensitive search with line numbers:

.. code-block:: console

   nsh> grep -in error /tmp/log.txt

Invert the match (print non-matching lines):

.. code-block:: console

   nsh> grep -v error /tmp/log.txt

Regular expression anchor -- match lines starting with ``ERR``:

.. code-block:: console

   nsh> grep "^ERR" /tmp/log.txt

Recurse a directory, always printing the file name:

.. code-block:: console

   nsh> grep -r -H TODO /data/src

Pipe usage -- filter the output of another NSH command:

.. code-block:: console

   nsh> ps | grep work

Notes
=====

- ``grep`` is built on the libc regex library; ``CONFIG_SYSTEM_GREP``
  depends on ``CONFIG_LIBC_REGEX``, so both must be enabled to satisfy the
  ``regcomp`` / ``regexec`` dependency.
- The ``-r`` option implies multi-file mode, so the file name is always
  prefixed regardless of ``-H`` / ``-h``.
- Long lines are truncated at ``GREP_LINE_MAX`` (512 bytes); matching is
  performed on the (newline-stripped) content of each line.
