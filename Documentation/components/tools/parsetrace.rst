parsetrace.py
=============

`parsetrace.py` is a trace log parsing tool for the NuttX RTOS. It supports converting binary or text trace logs into a human-readable systrace format, and can resolve symbols and type information from ELF files. The tool also supports real-time trace data parsing via serial port.

Features
--------
- Supports parsing both binary and text trace log formats.
- Integrates with ELF files to resolve symbols and type information for improved log readability.
- Supports real-time trace data parsing from a serial device.
- Outputs systrace-compatible format for performance analysis and debugging.

Dependencies
------------
- Python 3
- pyelftools
- cxxfilt
- pydantic
- parse
- pycstruct
- colorlog
- serial

Install dependencies:

.. code-block:: bash

   pip install pyelftools cxxfilt pydantic parse pycstruct colorlog serial

Usage
-----

.. code-block:: bash

   python3 tools/parsetrace.py -t <trace_file> -e <elf_file> [-o <output_file>] [-v]

Arguments:

- ``-t, --trace``: Path to the original trace file (supports binary or text format)
- ``-e, --elf``: Path to the NuttX ELF file (for symbol resolution)
- ``-o, --output``: Output file path, default is ``trace.systrace``
- ``-v, --verbose``: Enable verbose output
- ``-d, --device``: Serial device name (for real-time trace parsing)
- ``-b, --baudrate``: Serial baud rate, default is 115200

Examples
--------

Parse a text trace log and output as systrace format:

.. code-block:: bash

   python3 tools/parsetrace.py -t trace.log -e nuttx.elf -o trace.systrace

Parse a binary trace log:

.. code-block:: bash

   python3 tools/parsetrace.py -t trace.bin -e nuttx.elf

Parse trace data from a serial device in real time:

.. code-block:: bash

   python3 tools/parsetrace.py -d /dev/ttyUSB0 -e nuttx.elf

Main Classes and Functions
--------------------------

- ``SymbolTables``: Handles ELF symbol and type information parsing.
- ``Trace``: Parses text trace logs.
- ``ParseBinaryLogTool``: Parses binary trace logs.
- ``TraceDecoder``: Parses real-time trace data from serial port.

For more details, refer to the source code in ``tools/parsetrace.py``.
