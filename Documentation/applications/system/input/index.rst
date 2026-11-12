====================
``input`` input tool
====================

``input`` is a command-line tool that injects touch, button and mouse wheel
events through the uinput devices. It is useful for driving graphics
applications from the shell, for automated UI tests, and for input driver
bring-up together with :doc:`/applications/graphics/input/getevent`.

The tool is built on the input generator library in
``apps/graphics/input/generator`` and writes to these devices:

- ``/dev/utouch`` -- touch events (``tap``, ``swipe``, ``drag``)
- ``/dev/ubutton`` -- button events (``button``)
- ``/dev/umouse`` -- mouse wheel events (``wheel``, ``scroll``)

Configuration
=============

Enable the tool in the NuttX configuration. It depends on the uinput
devices and the input generator library::

  CONFIG_UINPUT_TOUCH=y
  CONFIG_UINPUT_BUTTONS=y
  CONFIG_UINPUT_MOUSE=y
  CONFIG_GRAPHICS_INPUT_GENERATOR=y
  CONFIG_GRAPHICS_INPUT_TOOL=y

The ``wheel`` and ``scroll`` commands also require::

  CONFIG_INPUT_MOUSE_WHEEL=y

Optional settings:

.. list-table::
   :header-rows: 1

   * - Option
     - Default
     - Description
   * - ``CONFIG_GRAPHICS_INPUT_TOOL_STACKSIZE``
     - ``4096``
     - Stack size for the input task
   * - ``CONFIG_GRAPHICS_INPUT_TOOL_PRIORITY``
     - ``110``
     - Task priority
   * - ``CONFIG_GRAPHICS_INPUT_TOOL_SCROLL_THRESHOLD``
     - ``10``
     - Scroll activation threshold used by ``scroll`` (range 0 to 32766).
       It should match the threshold of the application that receives the
       events. Requires ``CONFIG_INPUT_MOUSE_WHEEL``.

Usage
=====

::

  input <command> [<arg>...]

Coordinates and wheel values are signed 16-bit integers. Durations are in
milliseconds. An unknown command or an invalid argument prints the usage
text and the tool exits with a failure status.

``help``
  Print the list of commands.

``sleep <ms>``
  Wait for the given time.

``tap <x> <y>``
  Press at ``(x, y)`` for 50 ms and release.

``swipe <x1> <y1> <x2> <y2> [duration]``
  Move a touch point from ``(x1, y1)`` to ``(x2, y2)`` in ``duration``
  milliseconds (default 300), then release.

``drag <x1> <y1> <x2> <y2> [duration]``, ``draganddrop ...``
  Like ``swipe``, but press for 50 ms before moving and hold for 5 ms
  before releasing. The default ``duration`` is 500 ms.

``button <buttonvalue> [duration]``
  Press the buttons in the mask ``buttonvalue`` (decimal or ``0x``
  hexadecimal) and release them. Without ``duration`` the buttons are held
  for 100 ms; with ``duration`` they are held for that long (long press).

``wheel <value>``
  Send a single mouse wheel report with ``value``.

``scroll <value>``, ``scroll --axis SCROLL,<value>``
  Send a paced sequence of mouse wheel reports for applications that only
  start scrolling once a threshold is exceeded:

  1. One activation report of the scroll threshold plus one, in the
     direction of ``value``. The threshold is
     ``CONFIG_GRAPHICS_INPUT_TOOL_SCROLL_THRESHOLD``.
  2. ``value`` split into steps of ``max(1, |value| / 10)``, with the last
     step taking the remainder, sent 30 ms apart.

  The activation report is sent in addition to ``value``. ``scroll 0``
  sends nothing. The ``--axis`` form follows the Android ``input scroll``
  syntax.

Examples
========

::

  nsh> input tap 100 200
  nsh> input swipe 10 10 200 200 300
  nsh> input button 0x01
  nsh> input button 0x01 1000
  nsh> input wheel 5
  nsh> input scroll 100
  nsh> input scroll --axis SCROLL,-50

Wheel reports of ``input scroll 100`` with the default threshold, as
printed by ``getevent -m /dev/umouse``::

  [getevent]:      wheel : 11
  [getevent]:      wheel : 10
  ... (ten reports of 10 in total)
