===============================
``sensortest`` sensor test tool
===============================

Reads data from a sensor driver via the uORB topic interface and prints the
output to the console. Useful for verifying that a sensor driver is working
correctly and producing valid data.

Configuration
=============

Enable the command with ``CONFIG_SYSTEM_SENSORTEST``. This option requires
``CONFIG_SENSORS`` and ``CONFIG_ENABLE_ALL_SIGNALS``.

The following configuration options are available:

``CONFIG_SYSTEM_SENSORTEST_PROGNAME``
  Program name for the ``sensortest`` command. Default: ``sensortest``.

``CONFIG_SYSTEM_SENSORTEST_PRIORITY``
  Task priority. Default: ``100``.

``CONFIG_SYSTEM_SENSORTEST_STACKSIZE``
  Stack size. Default: ``DEFAULT_TASK_STACKSIZE``.

Usage
=====

.. code-block:: console

   nsh> sensortest [-i <interval>] [-b <latency>] [-n <count>] <sensor_name>

Options
=======

``-i <interval>``
  Data output period in microseconds. Controls how often the sensor is
  polled for new data. Optional. Default: ``1000000`` (1 second).

``-b <latency>``
  Maximum report latency in microseconds. When set to a non-zero value,
  the sensor may batch multiple readings and deliver them at once.
  Optional. Default: ``0`` (no batching).

``-n <count>``
  Number of data samples to read before exiting. When set to ``0``,
  the command runs continuously until interrupted. Optional.
  Default: ``0`` (unlimited).

``-h``
  Print help message and exit.

Commands
========

``<sensor_name>``
  The sensor node name without the ``sensor_`` prefix and ``/dev/uorb/``
  path. For example, use ``accel`` to read from ``/dev/uorb/sensor_accel``.

Supported Sensors
=================

The following sensor types are supported:

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Name
     - Data Type
     - Description
   * - ``accel``
     - vec3 (x, y, z)
     - Accelerometer
   * - ``baro``
     - valf2
     - Barometer (pressure, temperature)
   * - ``cap``
     - cap
     - Capacitance
   * - ``co2``
     - valf
     - CO2 concentration
   * - ``dust``
     - valf
     - Dust particle concentration
   * - ``ecg``
     - ecg
     - Electrocardiogram
   * - ``force``
     - force
     - Force sensor
   * - ``gnss``
     - gnss
     - GNSS position
   * - ``gnss_satellite``
     - gnss_satellite
     - GNSS satellite info
   * - ``gyro``
     - vec3 (x, y, z)
     - Gyroscope
   * - ``hall``
     - valb
     - Hall effect sensor
   * - ``hbeat``
     - valf
     - Heartbeat
   * - ``hcho``
     - valf
     - Formaldehyde concentration
   * - ``hrate``
     - valf
     - Heart rate
   * - ``humi``
     - valf
     - Humidity
   * - ``impd``
     - valf2
     - Impedance
   * - ``ir``
     - valf
     - Infrared
   * - ``light``
     - valf
     - Ambient light
   * - ``mag``
     - vec3 (x, y, z)
     - Magnetometer
   * - ``noise``
     - valf
     - Noise level
   * - ``ots``
     - vali2
     - Optical tracking speed
   * - ``ph``
     - valf
     - pH value
   * - ``pm10``
     - valf
     - PM10 particulate matter
   * - ``pm1p0``
     - valf
     - PM1.0 particulate matter
   * - ``pm25``
     - valf
     - PM2.5 particulate matter
   * - ``ppgd``
     - ppgd
     - PPG (photoplethysmography) data
   * - ``ppgq``
     - ppgq
     - PPG quality
   * - ``prox``
     - valf
     - Proximity
   * - ``rgb``
     - valf3
     - RGB color sensor
   * - ``velocity``
     - velocity
     - Velocity sensor
   * - ``temp``
     - valf
     - Temperature
   * - ``tvoc``
     - valf
     - Total volatile organic compounds
   * - ``uv``
     - valf
     - Ultraviolet

Examples
========

Read accelerometer data at the default 1-second interval:

.. code-block:: console

   nsh> sensortest accel
   accel: timestamp:1234567890 x:0.12 y:-0.03 z:9.81, temperature:25.50
   accel: timestamp:1235567890 x:0.11 y:-0.04 z:9.80, temperature:25.51
   ^C

Read gyroscope data at 100ms intervals, 10 samples:

.. code-block:: console

   nsh> sensortest -i 100000 -n 10 gyro
   gyro: timestamp:1234567890 x:0.01 y:0.02 z:0.00, temperature:26.00
   ...

Read temperature sensor with batching enabled:

.. code-block:: console

   nsh> sensortest -b 5000000 temp
   temp: timestamp:1234567890 value:25.50
   ...

Notes
=====

- The command reads from ``/dev/uorb/sensor_<name>`` using the uORB
  subscriber interface. Ensure the corresponding sensor driver is
  enabled and the uORB topic is published.
- The command runs continuously until interrupted with ``Ctrl-C``
  (``SIGINT``) unless a sample count is specified with ``-n``.
- The ``-b`` (latency) option controls batching behavior. When set to
  a non-zero value, the sensor driver may buffer multiple readings and
  deliver them together, reducing the number of wake-ups.
- The timestamp field in the output is provided by the sensor driver
  and uses the system monotonic clock.
