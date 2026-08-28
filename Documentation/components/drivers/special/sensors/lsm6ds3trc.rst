===========
LSM6DS3TR-C
===========

The LSM6DS3TR-C is a 6-axis IMU with a 3-axis gyroscope and 3-axis
accelerometer by STMicroelectronics, register-compatible with the
LSM6DSL/LSM6DSO32 family. It has both I2C and SPI interfaces, although
this driver only supports I2C.

This driver uses the :doc:`uorb
</components/drivers/special/sensors/sensors_uorb>` interface.

.. warning::
   The LSM6DS3TR-C is a feature-packed sensor, and this driver does not
   implement many of its features, such as the FIFO, self-test,
   calibration offsets, tap/wakeup detection, or the low-power and
   ultra-low-power accelerometer modes.

Application Programming Interface
=================================

.. code-block:: c

   #include <nuttx/sensors/lsm6ds3trc.h>

The LSM6DS3TR-C registration function allows the driver to be registered as
a uORB driver. Registering this driver will cause two uORB topics to be
registered under ``/dev/uorb/``: ``sensor_accel<n>`` and ``sensor_gyro<n>``,
where ``n`` is the value of ``devno``.

Unlike sensors that need one interrupt pin per sub-sensor, the LSM6DS3TR-C
can OR both the accelerometer's and the gyroscope's data-ready flags onto a
single INT pin (each has its own enable bit in that pin's ``INT<n>_CTRL``
register). Because of this, ``lsm6ds3trc_config_s`` only takes one
``int_pin``/``attach`` pair, shared by both topics: one interrupt handler
times the data-ready event, and one HPWORK worker bursts accelerometer,
gyroscope and temperature in a single I2C transaction, pushing whichever
topic(s) are currently subscribed.

The driver can be registered either in polling mode or interrupt-driven
mode. The polling mode will create a kernel thread per topic to poll the
sensor periodically according to each topic's own set interval. Polling
mode is used by leaving ``attach`` ``NULL`` in the ``config`` parameter.

.. warning::
   To use interrupt-driven mode, ``CONFIG_SCHED_HPWORK`` must be enabled.

The following snippet shows how to register the driver in polling mode.
The value of ``int_pin`` can be safely ignored for this mode.

.. code-block:: c

   /* Example for a generic MCU */

   struct lsm6ds3trc_config_s lsm6ds3trc_config =
   {
     .int_pin = LSM6DS3TRC_INT1,
     .attach = NULL,
   };

   ret = lsm6ds3trc_register(mcu_i2cbus_initialize(0), 0x6a, 0,
                             &lsm6ds3trc_config);
   if (ret < 0)
     {
       syslog(LOG_ERR, "Couldn't register LSM6DS3TR-C at 0x6a: %d\n", ret);
     }

The following snippet shows how to register the driver in interrupt-driven
mode. Only one interrupt pin needs to be attached, regardless of whether
one or both topics end up subscribed.

.. code-block:: c

   /* Example for a generic MCU */

   /* This function attaches (or detaches, if handler is NULL) the shared
    * data-ready interrupt handler.
    */

   static int board_lsm6ds3trc_attach(xcpt_t handler, FAR void *arg)
   {
     int ret;

     mcu_gpioirqdisable(GPIO_LSM6DS3TRC_INT);

     ret = mcu_gpio_irq(GPIO_LSM6DS3TRC_INT, handler, arg);
     if (ret < 0)
       {
         return ret;
       }

     mcu_gpioirqenable(GPIO_LSM6DS3TRC_INT);
     return OK;
   }

   /* Registration of the driver */

   struct lsm6ds3trc_config_s lsm6ds3trc_config =
   {
     .int_pin = LSM6DS3TRC_INT1, /* Both DRDY_XL and DRDY_G route here */
     .attach = board_lsm6ds3trc_attach,
   };

   ret = lsm6ds3trc_register(mcu_i2cbus_initialize(0), 0x6a, 0,
                             &lsm6ds3trc_config);
   if (ret < 0)
     {
       syslog(LOG_ERR, "Couldn't register LSM6DS3TR-C at 0x6a: %d\n", ret);
     }

To debug this device, you can include the ``uorb_listener`` application in
your build with debugging enabled. Running it will show the sensor
measurements.

The interface for setting the measurement interval operates individually
on the gyroscope and accelerometer. That is to say that they can have
different sampling rates.

.. code-block:: c

   unsigned freq = 52;
   err = orb_set_frequency(accel, freq);
   if (err)
     {
       fprintf(stderr, "Wasn't able to set frequency to %uHz: %d\n", freq, err);
       return EXIT_FAILURE;
     }

The temperature measurement included in the data for both the
accelerometer and gyroscope is pulled from the same on-board temperature
sensor, read in the same burst transaction as whichever sub-sensor's
sample triggered it -- it does not have an independent output data rate
of its own in this driver.

This sensor also has an additional command for gaining access to extra
functionality.

``SNIOC_WHO_AM_I``
------------------

This command reads the ``WHO_AM_I`` register of the LSM6DS3TR-C. This
should always return ``0x6a``. The argument is a pointer to an 8-bit
unsigned integer. This command has the same result when called on either
the accelerometer or gyroscope topic.

.. code-block:: c

   uint8_t id;
   err = orb_ioctl(accel, SNIOC_WHO_AM_I, (unsigned long)&id);

``SNIOC_SETFULLSCALE``
----------------------

This command allows the user to set the full scale range of either the
accelerometer or the gyroscope.

When called on the accelerometer, the argument should be the desired FSR
in units of 'g'. The available options are 2, 4, 8 and 16g.

When called on the gyroscope, the argument should be the desired FSR in
units of degrees per second. The available options are 125, 245, 500,
1000 and 2000 dps.

Note that by default, the accelerometer has a full scale range of +/-4g
and the gyroscope has a full scale range of +/-245dps.

.. code-block:: c

   err = orb_ioctl(accel, SNIOC_SETFULLSCALE, 16);
   err = orb_ioctl(gyro, SNIOC_SETFULLSCALE, 500);

To check the FSR, you can get the sensor info and check the ``max_range``
field. This value is in m/s^2 for the accelerometer and rad/s for the
gyroscope, so it must be converted to units of g or degrees per second in
order to directly compare it against what was set.

.. code-block:: c

   struct sensor_device_info_s info;
   err = orb_ioctl(accel, SNIOC_GET_INFO, (unsigned long)&info);
   if (err < 0)
     {
       fprintf(stderr, "Could not get sensor information: %d", errno);
       return EXIT_FAILURE;
     }

   printf("Sensor: %s\n", info.name);
   printf("Manufacturer: %s\n", info.vendor);
   printf("Max range: %.2f m/s^2\n", info.max_range);
