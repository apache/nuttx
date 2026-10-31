=======
LPS22HB
=======

STMicroelectronics LPS22HB pressure sensor on I2C,
:doc:`uORB </os/drivers/special/sensors/sensors_uorb>` interface.

Configuration
=============

- ``CONFIG_SENSORS_LPS22HB``
- ``CONFIG_LPS22HB_I2C_FREQUENCY``

Usage
=====

.. code-block:: c

   #include <nuttx/sensors/lps22hb.h>

   int lps22hb_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                             uint8_t addr);

Registers ``sensor_baro<devno>`` (hPa, °C).
