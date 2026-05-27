=============================================
``hex2bin`` Intel HEX to binary conversion
=============================================

Overview
========

The ``hex2bin`` command reads an Intel HEX file and writes the
corresponding binary data to an output file.  Intel HEX is a common
text format for representing compiled program data; ``hex2bin`` converts
that format into raw binary suitable for flashing or further processing.

The command operates on two file arguments: the source HEX file and the
destination binary file.

Configuration
=============

Enable the command with ``CONFIG_SYSTEM_HEX2BIN``.  The NSH built-in
variant additionally requires ``CONFIG_SYSTEM_HEX2BIN_BUILTIN``.

The task priority and stack size are controlled by:

- ``CONFIG_SYSTEM_HEX2BIN_PRIORITY``
- ``CONFIG_SYSTEM_HEX2BIN_STACKSIZE``

The default values for the ``-s``, ``-e``, and ``-w`` options can be
pre-set through:

- ``CONFIG_SYSTEM_HEX2BIN_BASEADDR`` — default start address
- ``CONFIG_SYSTEM_HEX2BIN_ENDPADDR`` — default end address
- ``CONFIG_SYSTEM_HEX2BIN_SWAP`` — default byte-swap mode

The ``-h`` usage message can be suppressed to save flash by disabling
``CONFIG_SYSTEM_HEX2BIN_USAGE``.

Usage
=====

.. code-block:: console

   hex2bin [-s <addr>] [-e <addr>] [-w <swap>] <hexfile> <binfile>
   hex2bin -h

Options
=======

.. list-table::
   :header-rows: 1

   * - Option
     - Description
   * - ``-s <start address>``
     - Set the start address of the binary output in hexadecimal.
       This value is used to calculate offsets into the output stream
       and for error checking.  Default: ``0x00000000`` (or the value
       of ``CONFIG_SYSTEM_HEX2BIN_BASEADDR``).
   * - ``-e <end address>``
     - Set the maximum address (plus one) of the binary output in
       hexadecimal.  Used for range-checking only.  A value of zero
       disables range checking.  Default: ``0x00000000`` (or the value
       of ``CONFIG_SYSTEM_HEX2BIN_ENDPADDR``).
   * - ``-w <swap code>``
     - Control byte ordering of the output.  ``0``: no swap, ``1``:
       swap bytes in 16-bit values, ``2``: swap bytes in 32-bit
       values.  Default: ``0`` (or the value of
       ``CONFIG_SYSTEM_HEX2BIN_SWAP``).
   * - ``-h``
     - Show command usage and exit.  Only available when
       ``CONFIG_SYSTEM_HEX2BIN_USAGE`` is enabled.

Arguments
=========

``hexfile``
  The input file containing Intel HEX records.

``binfile``
  The output file to be created with the converted binary data.

Examples
========

Convert a HEX file to binary with default settings:

.. code-block:: console

   nsh> hex2bin firmware.hex firmware.bin

Convert with a custom start address and 16-bit byte swap:

.. code-block:: console

   nsh> hex2bin -s 0x08000000 -w 1 firmware.hex firmware.bin

Show usage information:

.. code-block:: console

   nsh> hex2bin -h

Notes
=====

- The command requires both ``<hexfile>`` and ``<binfile>`` arguments;
  omitting either produces an error.
- When the ``-e`` end address is non-zero and the HEX data exceeds that
  range, the conversion fails with an error.
- The ``hex2bin`` library function (``CONFIG_LIBC_HEX2BIN``) can also
  be used directly from C code without the NSH command interface.
