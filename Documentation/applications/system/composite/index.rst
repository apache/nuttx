===========================================
``composite`` USB Composite Device Commands
===========================================

This logic adds a NSH command to control a USB composite device. The only
supported devices in the composite are CDC/ACM serial and a USB mass storage
device. Which devices are enclosed in a composite device is configured with an
array of configuration-structs, handed over to the function
``composite_initialize()``.

Required overall configuration:

Enable the USB Support of your Hardware / Processor e.g. ``SAMV7_USBDEVHS=y``

- ``CONFIG_USBDEV=y`` – USB device support.
- ``CONFIG_USBDEV_COMPOSITE=y`` – USB composite device support.
- ``CONFIG_COMPOSITE_IAD=y`` – Interface associate descriptor needed.
- ``CONFIG_CDCACM=y`` – USB CDC/ACM serial device support.
- ``CONFIG_CDCACM_COMPOSITE=y`` – USB CDC/ACM serial composite device support.

The interface-, string-descriptor- and endpoint-numbers are configured via the
configuration-structs as noted above. The CDC/ACM serial device needs three
endpoints; one interrupt-driven and two bulk endpoints.

- ``CONFIG_USBMSC=y`` – USB mass storage device support.
- ``CONFIG_USBMSC_COMPOSITE=y`` – USB mass storage composite device support.

Like the configuration for the CDC/ACM, the descriptor- and endpoint-numbers are
configured via the configuration struct.

Depending on the configuration struct you need to configure different vendor-
and product-IDs. Each ``VID``/``PID`` is unique to a device and thus to a dedicated
configuration.

Linux tries to detect the device types and installs default drivers if the
``VID``/``PID`` pair is unknown.

Windows insists on a known and installed configuration. With an Atmel hardware
and Atmel-Studio or the Atmel-USB-drivers installed, you can test your
configuration with Atmel Example Vendor- and Product-IDs.

If you have a USBMSC and a CDC/ACM configured in your combo, then you can try to
use

- ``VID = 0x03EB`` (ATMEL)
- ``PID = 0x2424`` (ASF Example with MSC and CDC)

If for example you try to test a configuration with up to seven CDCs, then

- ``VID = 0x03EB`` (ATMEL)
- ``PID = 0x2426`` (ASF Example with up to seven CDCs)

This add-on can be built as two NSH "built-in" commands:

- ``CONFIG_NSH_BUILTIN_APPS`` – if this option is selected: ``conn`` will connect
  the USB composite device; ``disconn`` will disconnect the USB composite device.

Configuration options unique to this add-on:

- ``CONFIG_SYSTEM_COMPOSITE_DEBUGMM`` – Enables some debug tests to check for
  memory usage and memory leaks.

If ``CONFIG_USBDEV_TRACE`` is enabled (or ``CONFIG_DEBUG_FEATURES`` and
``CONFIG_DEBUG_USB``), then the add-on code will also manage the USB trace output.
The amount of trace output can be controlled using:

- ``CONFIG_SYSTEM_COMPOSITE_TRACEINIT`` – Show initialization events.
- ``CONFIG_SYSTEM_COMPOSITE_TRACECLASS`` – Show class driver events.
- ``CONFIG_SYSTEM_COMPOSITE_TRACETRANSFERS`` – Show data transfer events.
- ``CONFIG_SYSTEM_COMPOSITE_TRACECONTROLLER`` – Show controller events.
- ``CONFIG_SYSTEM_COMPOSITE_TRACEINTERRUPTS`` – Show interrupt-related events.
