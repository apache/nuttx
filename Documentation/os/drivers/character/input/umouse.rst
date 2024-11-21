======================
Virtual Mouse (uMouse)
======================

The uMouse driver provides a virtual mouse device for software-injected
mouse reports without requiring physical mouse hardware.  Reports are
delivered through the existing mouse upper-half driver.

When enabled, the driver registers a character device at ``/dev/umouse``.
Writing a complete mouse report to this device is enough to generate a
mouse event for every opened consumer.

Configuration
=============

- ``CONFIG_INPUT=y`` enables the input subsystem.
- ``CONFIG_UINPUT_MOUSE=y`` enables the virtual mouse device driver.

``CONFIG_UINPUT_MOUSE`` automatically selects the ``INPUT_MOUSE`` and
``INPUT_UINPUT`` options.  When enabled, the device is registered
automatically during system start-up, so no manual initialization step
is required.

``CONFIG_UINPUT_MOUSE_BUFNUMBER`` sets the number of mouse reports that
each opened consumer can buffer.  The default value is ``8``.

Interface
=========

Each non-empty ``write()`` to ``/dev/umouse`` must provide exactly one
``struct mouse_report_s``; other non-zero write sizes are rejected.  The
report carries the button state and the ``x``, ``y`` and ``wheel``
fields (see ``include/nuttx/input/mouse.h``).

Consumers open the device and use ``read()`` and ``poll()`` to obtain
the reported events.  Each ``open()`` creates an independent report
queue of ``CONFIG_UINPUT_MOUSE_BUFNUMBER`` reports.  When a queue
is full, the oldest report is discarded to make room for the new one.

Optional RPMsg
==============

With ``CONFIG_UINPUT_RPMSG=y`` and ``RPMSG`` support enabled, the mouse
reports are forwarded to another core using the existing uinput RPMsg
infrastructure, so reports written on one core can reach consumers on
another.