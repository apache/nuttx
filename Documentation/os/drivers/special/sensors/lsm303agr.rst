=========
LSM303AGR
=========

The LSM303AGR is a 3-axis accelerometer and 3-axis magnetometer by
STMicroelectronics. This driver supports it over I2C through the
:doc:`uorb </os/drivers/special/sensors/sensors_uorb>` interface. The
character driver ``CONFIG_SENSORS_LSM303AGR`` is a separate, legacy driver.

Application Programming Interface
=================================

.. code-block:: c

   #include <nuttx/sensors/lsm303agr.h>

   int lsm303agr_register_uorb(FAR struct i2c_master_s *i2c, int devno);

The function registers ``/dev/uorb/sensor_accel<n>`` and
``/dev/uorb/sensor_mag<n>``, ``n`` being ``devno``. The I2C addresses of the
LSM303AGR are fixed (0x19 and 0x1e). Both sensors are polled by one kernel
thread.

The accelerometer runs in high-resolution (12-bit) mode. Its data rate is
the slowest of 1, 10, 25, 50, 100, 200, 400 and 1344 Hz that is at least the
requested rate. The ``temperature`` field carries the chip temperature
sensor.

The magnetometer runs in continuous high-resolution mode with temperature
compensation, at 10, 20, 50 or 100 Hz selected the same way. Its
``temperature`` field is zero.

The magnetometer is similar to the :doc:`lis2mdl`, but not compatible: it
has no temperature output, for example. Common code of the two drivers
could be shared in the future.

``SNIOC_SETFULLSCALE``
----------------------

Sets the accelerometer full scale in g: 2 (default), 4, 8 or 16. The
magnetometer has a fixed full scale of +/-50 gauss.

``SNIOC_WHO_AM_I``
------------------

Reads the ``WHO_AM_I`` register: ``0x33`` for the accelerometer and
``0x40`` for the magnetometer.
