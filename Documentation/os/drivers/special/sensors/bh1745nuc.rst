=========
BH1745NUC
=========

Rohm BH1745NUC color sensor on I2C,
:doc:`uORB </os/drivers/special/sensors/sensors_uorb>` interface.

Configuration
=============

- ``CONFIG_SENSORS_BH1745NUC``
- ``CONFIG_BH1745NUC_I2C_FREQUENCY``

Usage
=====

.. code-block:: c

   #include <nuttx/sensors/bh1745nuc.h>

   int bh1745nuc_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                               uint8_t addr);

Registers ``sensor_rgb<devno>`` with the raw 16-bit RGB counts.
