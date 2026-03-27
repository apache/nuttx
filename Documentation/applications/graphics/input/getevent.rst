================================
``getevent`` Input Event Monitor
================================

``getevent`` is a command-line input event monitor. It captures and displays
real-time events from mouse, touchscreen, and keyboard devices via the input
subsystem.

It is useful for driver bring-up, input debugging, and system integration
testing.

Supported Event Types
=====================

- **Mouse** -- button state, x/y coordinates, wheel (when
  ``CONFIG_INPUT_MOUSE_WHEEL`` is enabled)
- **Touchscreen** -- multi-touch points with coordinates, flags, timestamps,
  and optional pressure/size detail
- **Keyboard** -- key code and event type

Configuration
=============

Enable ``getevent`` in the NuttX configuration::

  CONFIG_GRAPHICS_INPUT_GETEVENT=y

Optional settings:

.. list-table::
   :header-rows: 1

   * - Option
     - Default
     - Description
   * - ``CONFIG_GRAPHICS_INPUT_GETEVENT_STACKSIZE``
     - ``DEFAULT_TASK_STACKSIZE``
     - Stack size for the getevent task
   * - ``CONFIG_GRAPHICS_INPUT_GETEVENT_PRIORITY``
     - ``100``
     - Task priority
   * - ``CONFIG_GRAPHICS_INPUT_GETEVENT_DETAIL_INFO``
     - ``n``
     - Show additional touch detail (pressure, width, height)

Usage
=====

Auto-detect all input devices::

  nsh> getevent

Monitor specific devices::

  nsh> getevent -m /dev/mouse0
  nsh> getevent -t /dev/input0
  nsh> getevent -k /dev/kbd0

Combine multiple devices::

  nsh> getevent -m /dev/mouse0 -t /dev/input0 -k /dev/kbd0

Show help::

  nsh> getevent -h

Press ``Ctrl+C`` to stop monitoring.

How It Works
============

When started without arguments, ``getevent`` scans ``/dev`` for character
devices matching known input device name patterns (``mouse*``, ``input*``,
``kbd*``). Each detected device is opened in non-blocking mode.

The main event loop uses ``poll()`` with a 500 ms timeout to wait for data on
all open file descriptors. When an event is available, the corresponding read
callback decodes and prints the event structure via ``syslog``.

``SIGINT`` (``Ctrl+C``) is handled to allow graceful shutdown with proper
resource cleanup.

Example Output
==============

Mouse event::

  [getevent]: mouse event: /dev/mouse0
  [getevent]:    buttons : 01
  [getevent]:          x : 120
  [getevent]:          y : 340

Touch event::

  [getevent]: touch event: /dev/input0
  [getevent]:    npoints : 1
  [getevent]: Point      : 0
  [getevent]:      flags : 03
  [getevent]:          x : 200
  [getevent]:          y : 450
  [getevent]:  timestamp : 123456789

Keyboard event::

  [getevent]: keyboard event: /dev/kbd0
  [getevent]:          type : 1
  [getevent]:          code : 28
