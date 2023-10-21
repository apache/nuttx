``djoystick`` Discrete Joystick
===============================

This is a simple test of the discrete joystick driver. See details about this
driver in ``nuttx/include/nuttx/input/djoystick.h``.

Configuration Pre-requisites:

- ``CONFIG_INPUT_DJOYSTICK`` – The discrete joystick driver.

Example Configuration:

- ``CONFIG_EXAMPLES_DJOYSTICK`` – Enabled the discrete joystick example.
- ``CONFIG_EXAMPLES_DJOYSTICK_DEVNAME`` – Joystick device name. Default
  ``/dev/djoy0``.
- ``CONFIG_EXAMPLES_DJOYSTICK_SIGNO`` – Signal used to signal the test
  application. Default ``32``.
