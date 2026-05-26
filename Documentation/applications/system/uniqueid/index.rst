===============================
``uniqueid`` "uniqueid" command
===============================

Overview
========

The ``uniqueid`` command prints the board unique ID returned by the
``BOARDIOC_UNIQUEID`` ``boardctl()`` command.  It is useful for checking the
identifier that board-specific logic exposes through ``board_uniqueid()`` and
for formatting selected bytes of that identifier from NSH scripts.

Configuration
=============

Enable the command with ``CONFIG_SYSTEM_UNIQUEID``.  This option depends on
``CONFIG_BOARDCTL_UNIQUEID`` because the board must provide the
``BOARDIOC_UNIQUEID`` implementation.  The number of bytes printed by default
is controlled by ``CONFIG_BOARDCTL_UNIQUEID_SIZE``.

Usage
=====

.. code-block:: console

   uniqueid [-b <bytes>] [-d <delimiter>] [-f <format>] [-p <prefix>]

When ``-b`` is omitted, ``uniqueid`` prints all
``CONFIG_BOARDCTL_UNIQUEID_SIZE`` bytes.

Options
=======

``-b <bytes>``
  Select the bytes to print.  Byte positions are one-based and may be provided
  as a comma-separated list, ranges, or a mix of both.  Ranges may be ascending
  or descending, for example ``1,2,4-8`` or ``8-1``.

``-d <delimiter>``
  Print ``<delimiter>`` between selected bytes.

``-f <format>``
  Format each byte with the supplied ``printf`` conversion suffix.  The default
  is ``02x``.  The command accepts integer and character conversions ending in
  ``d``, ``i``, ``o``, ``u``, ``x``, ``X``, or ``c``.

``-p <prefix>``
  Print ``<prefix>`` before the first byte.

Examples
========

Print the full unique ID as hexadecimal bytes without separators:

.. code-block:: console

   nsh> uniqueid
   00112233445566778899aabbccddeeff

Print the ID as colon-separated hexadecimal bytes:

.. code-block:: console

   nsh> uniqueid -d :
   00:11:22:33:44:55:66:77:88:99:aa:bb:cc:dd:ee:ff

Print only the first four bytes with a prefix:

.. code-block:: console

   nsh> uniqueid -b 1-4 -d : -p uid:
   uid:00:11:22:33

Notes
=====

The actual unique ID value and its stability across resets or boards are
provided by the board-specific ``board_uniqueid()`` implementation.  If the
board does not implement ``BOARDIOC_UNIQUEID``, the command reports an error
from ``boardctl()``.
