ADXL367 Accelerometer
=====================

The ADXL367 driver provides I2C accelerometer support through the sensor
framework (uORB). It reports three-axis acceleration at a fixed range of
+/-2 g and die temperature, with output data rates from 12.5 to 400 Hz.

Enable ``CONFIG_SENSORS_ADXL367`` and register an instance with
``adxl367_register()`` from ``<nuttx/sensors/adxl367.h>``. The function accepts
a device number, an I2C interface, and the sensor address (``0x1d`` or ``0x53``).
Samples are read on demand through ``/dev/uorb/sensor_accelN``, where ``N`` is
the device number.
