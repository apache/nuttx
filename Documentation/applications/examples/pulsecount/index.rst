====================================
``pulsecount`` Pulse Count Example
====================================

This example starts a finite pulse train on a pulsecount device and
waits for completion when the device is opened in blocking mode.

Required configuration:

- ``CONFIG_PULSECOUNT`` – Enables pulsecount support.
- ``CONFIG_NSH_BUILTIN_APPS`` – Builds the example as an NSH built-in.

Specific configuration options:

- ``CONFIG_EXAMPLES_PULSECOUNT_DEVPATH`` – The default pulsecount device.
  Default: ``/dev/pulsecount0``.
- ``CONFIG_EXAMPLES_PULSECOUNT_HIGH_NS`` – The pulse high time. Default:
  ``5000000`` ns.
- ``CONFIG_EXAMPLES_PULSECOUNT_LOW_NS`` – The pulse low time. Default:
  ``5000000`` ns.
- ``CONFIG_EXAMPLES_PULSECOUNT_COUNT`` – The finite pulse count.
