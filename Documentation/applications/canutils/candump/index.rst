==================================
``candump`` SocketCAN candump tool
==================================

``candump`` is a SocketCAN traffic monitor ported from the Linux CAN utilities.
It listens on one or more CAN interfaces and prints received CAN or CAN FD
frames to the console. It can also timestamp frames, apply receive filters,
monitor dropped frames, decode error frames in a human-readable format, and
write captured traffic to a log file.

Configuration
=============

Enable the application with ``CONFIG_CANUTILS_CANDUMP``. The option depends on
the SocketCAN network stack and canutils support:

- ``CONFIG_NET_CAN``
- ``CONFIG_CANUTILS_LIBCANUTILS``
- ``CONFIG_ENABLE_ALL_SIGNALS``

You can also adjust the task stack size with
``CONFIG_CANUTILS_CANDUMP_STACKSIZE``.

Usage
=====

The command is available as ``candump`` in NSH when built as a builtin
application.

.. code-block:: console

	nsh> candump [options] <CAN interface>+

Each interface argument may optionally include a comma-separated filter list in
the form ``<ifname>[,filter]*``.

Common options include:

- ``-t <type>``: print timestamps using absolute, delta, zero-based, or
  absolute-with-date modes
- ``-l``: log received CAN frames to a file
- ``-L``: print log-file format to standard output
- ``-n <count>``: stop after receiving ``count`` frames
- ``-r <size>``: set the socket receive buffer size
- ``-d``: monitor dropped CAN frames
- ``-e``: decode CAN error frames in a human-readable format
- ``-x``: print extra message information such as RX/TX, BRS, and ESI
- ``-T <msecs>``: terminate after a timeout with no received frames

Filter syntax
=============

Each CAN interface can be followed by zero or more filters:

- ``<can_id>:<can_mask>`` matches frames where
  ``received_can_id & mask == can_id & mask``
- ``<can_id>~<can_mask>`` matches frames where
  ``received_can_id & mask != can_id & mask``
- ``#<error_mask>`` sets the error frame filter
- ``j`` or ``J`` joins the specified filters with logical AND semantics

CAN IDs, masks, and data values are specified in hexadecimal. Without any
explicit filter, ``candump`` uses the default filter ``0:0`` and receives all
data frames. Use interface name ``any`` to receive from all CAN interfaces.

Examples
========

Listen on a single CAN interface:

.. code-block:: console

	nsh> candump can0

Listen on multiple interfaces with filters and absolute timestamps:

.. code-block:: console

	nsh> candump -t a can0,123:7FF,400:700,#000000FF can2,400~7F0 can3

Log all CAN traffic and error frames from every available CAN interface:

.. code-block:: console

	nsh> candump -l any,0:0,#FFFFFFFF

Capture only error frames:

.. code-block:: console

	nsh> candump -l any,0~0,#FFFFFFFF

This command is useful when debugging SocketCAN drivers, verifying traffic on a
bus, or collecting logs for later analysis.
