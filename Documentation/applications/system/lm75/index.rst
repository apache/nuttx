=========================
``lm75`` LM75 Temperature
=========================

The ``lm75`` command reads temperature data from an LM75 (or compatible)
I2C temperature sensor and prints the result to standard output. The
sensor must be enabled via ``CONFIG_LM75_I2C``.

Configuration
=============

Enable the command with ``CONFIG_SYSTEM_LM75``. This option depends on
``CONFIG_LM75_I2C``.

The following configuration options are available:

``CONFIG_SYSTEM_LM75_DEVNAME``
  Sensor device path. Default: ``/dev/temp``.

``CONFIG_SYSTEM_LM75_FAHRENHEIT``
  Display temperature in degrees Fahrenheit. This is the default.

``CONFIG_SYSTEM_LM75_CELSIUS``
  Display temperature in degrees Celsius.

``CONFIG_SYSTEM_LM75_STACKSIZE``
  Stack size in bytes. Default: 1024.

``CONFIG_SYSTEM_LM75_PRIORITY``
  Command task priority. Default: 100.

Usage
=====

.. code-block:: text

   temp [OPTIONS]

Options
=======

``-n <count>``
  Number of temperature samples to collect. Each sample is read with a
  500 ms interval. Default: 1.

``-h``
  Show help message and exit.

Examples
========

Read a single temperature sample:

.. code-block:: text

   nsh> temp
   72.50 degrees Fahrenheit

Read five consecutive samples:

.. code-block:: text

   nsh> temp -n 5
   72.50 degrees Fahrenheit
   72.54 degrees Fahrenheit
   72.60 degrees Fahrenheit
   72.58 degrees Fahrenheit
   72.52 degrees Fahrenheit

If ``CONFIG_SYSTEM_LM75_CELSIUS`` is selected instead of Fahrenheit,
the output displays degrees Celsius:

.. code-block:: text

   nsh> temp
   22.50 degrees Celsius
