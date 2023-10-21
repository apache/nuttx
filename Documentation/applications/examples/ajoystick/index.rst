``ajoystick`` Analog Joystick
=============================

This is a simple test of the analog joystick driver. See details about this
driver in ``nuttx/include/nuttx/input/ajoystick.h``.

Configuration Pre-requisites:

- ``CONFIG_AJOYSTICK`` – The analog joystick driver.

Example Configuration:
- ``CONFIG_EXAMPLES_AJOYSTICK`` – Enabled the analog joystick example.
- ``CONFIG_EXAMPLES_AJOYSTICK_DEVNAME`` – Joystick device name. Default: ``/dev/adjoy0``.
- ``CONFIG_EXAMPLES_AJOYSTICK_SIGNO`` – Signal used to signal the test
application. Default: ``32``.

