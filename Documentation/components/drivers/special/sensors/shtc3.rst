=============================
Sensirion SHTC3 Sensor Driver
=============================

This driver provides support for the Sensirion SHTC3 temperature and humidity sensor
using the uORB sensor framework.

It provides two uORB topics:
- ``SENSOR_TYPE_AMBIENT_TEMPERATURE``
- ``SENSOR_TYPE_RELATIVE_HUMIDITY``

Configuration
-------------

To enable the driver, select ``CONFIG_SENSORS_SHTC3=y`` in your board configuration.
You can configure the I2C frequency via ``CONFIG_SHTC3_I2C_FREQUENCY`` (default is 400kHz).
