======================================
``cdcacm`` USB CDC/ACM Device Commands
======================================

This very simple add-on allows the USB CDC/ACM serial device can be dynamically
connected and disconnected from a host. This add-on can only be used as an NSH
built-in command. If built-in, then two new NSH commands will be supported:

1. ``sercon`` – Connect the CDC/ACM serial device
2. ``serdis`` – Disconnect the CDC/ACM serial device

Configuration prerequisites (not complete):

- ``CONFIG_USBDEV=y`` – USB device support must be enabled
- ``CONFIG_CDCACM=y`` – The CDC/ACM driver must be built

Configuration options specific to this add-on:

- ``CONFIG_SYSTEM_CDCACM_DEVMINOR`` – The minor number of the CDC/ACM device,
  i.e., the ``x`` in ``/dev/ttyACMx``.

If ``CONFIG_USBDEV_TRACE`` is enabled (or ``CONFIG_DEBUG_FEATURES`` and
``CONFIG_DEBUG_USB``, or ``CONFIG_USBDEV_TRACE``), then the add-on code will also
initialize the USB trace output. The amount of trace output can be controlled
using:

- ``CONFIG_SYSTEM_CDCACM_TRACEINIT`` – Show initialization events.
- ``CONFIG_SYSTEM_CDCACM_TRACECLASS`` – Show class driver events.
- ``CONFIG_SYSTEM_CDCACM_TRACETRANSFERS`` – Show data transfer events.
- ``CONFIG_SYSTEM_CDCACM_TRACECONTROLLER`` – Show controller events.
- ``CONFIG_SYSTEM_CDCACM_TRACEINTERRUPTS`` – Show interrupt-related events.

**Note**: This add-on is only enables or disable USB CDC/ACM via the NSH
``sercon`` and ``serdis`` command. It will enable and disable tracing per the
settings before enabling and after disabling the CDC/ACM device. It will not,
however, monitor buffered trace data in the interim. If ``CONFIG_USBDEV_TRACE`` is
defined (and the debug options are not), other application logic will need to
monitor the buffered trace data.
