==================================
``touchscreen`` Touchscreen Events
==================================

This configuration implements a simple touchscreen test at
``apps/examples/touchscreen``. This test will create an empty X11 window and will
print the touchscreen output as it is received from the simulated touchscreen
driver.

- ``CONFIG_NSH_BUILTIN_APPS`` – Build the touchscreen test as an NSH built-in
  function. Default: Built as a standalone program.
- ``CONFIG_EXAMPLES_TOUCHSCREEN_MINOR`` – The minor device number. Minor ``N``
  corresponds to touchscreen device ``/dev/inputN``. Note this value must with
  ``CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH``. Default ``0``.
- ``CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH`` – The path to the touchscreen device.
  This must be consistent with ``CONFIG_EXAMPLES_TOUCHSCREEN_MINOR``. Default:
  ``/dev/input0``.
- ``CONFIG_EXAMPLES_TOUCHSCREEN_NSAMPLES`` – This number of samples is collected
  and the program terminates. Default: Samples are collected indefinitely.
- ``CONFIG_EXAMPLES_TOUCHSCREEN_MOUSE`` – The touchscreen test can also be
  configured to work with a mouse driver by setting this option.

The following additional configurations must be set in the NuttX configuration
file:

- ``CONFIG_INPUT=y`` (plus any touchscreen-specific settings)

The following must also be defined in your apps configuration file:

- ``CONFIG_EXAMPLES_TOUCHSREEN=y``

This example code will call ``boardctl()`` to setup the touchscreen driver for
texting. The implementation of ``boardctl()`` will require that board- specific
logic  provide the following interfaces that will be called by the ``boardctl()``
in order to initialize the touchscreen hardware:

.. code-block:: C

  int board_tsc_setup(int minor);
