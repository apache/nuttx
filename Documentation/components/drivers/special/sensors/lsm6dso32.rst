=========
LSM6DSO32
=========

The LSM6DSO32 is a high-performance IMU with a 3-axis gyroscope and 3-axis
accelerometer by STMicroelectronics. It has both I2C and SPI interfaces,
although this driver only supports I2C.

This driver uses the :doc:`uorb
</components/drivers/special/sensors/sensors_uorb>` interface. It supports the
self-test capability for both the accelerometer and gyroscope.

.. warning::
   The LSM6DSO32 is a feature-packed sensor, and this driver does not implement
   many of its features, such as tap, wakeup, acting as a master to other
   sensors, etc.

Application Programming Interface
=================================

.. code-block:: c

   #include <nuttx/sensors/lsm6dso32.h>

The LSM6DSO32 registration function allows the driver to be registered as a uORB
driver. Registering this driver will cause two uORB topics to be registered
under ``/dev/uorb/``: ``sensor_accel<n>`` and ``sensor_gyro<n>``, where ``n`` is
the value of ``devno``.

The driver can be registered either in polling mode or interrupt-driven mode.
The polling mode will create a kernel thread to poll the sensor periodically
according to the set interval. Polling mode is register by leaving the
``attach`` functions ``NULL`` in the ``config`` parameter.

.. warning::
   To use interrupt-driven mode, ``CONFIG_SCHED_HPWORK`` must be enabled.

The following snippet shows how to register the driver in polling mode. The
values of ``gy_int`` and ``xl_int`` can be safely ignored for this mode.

.. code-block:: c

   /* Example for an RP2040 MCU */

    struct lsm6dso32_config_s lsm6dso32_config = {
      .gy_int = 0,
      .xl_int = 0,
      .gy_attach = NULL,
      .xl_attach = NULL,
    };
  
    ret = lsm6dso32_register(rp2040_i2cbus_initialize(0), 0x6b, 0,
                             &lsm6dso32_config);
    if (ret < 0)
      {
        syslog(LOG_ERR, "Couldn't register LSM6DSO32 at 0x6b: %d\n", ret);
      }

The following snippet shows how to register the driver in interrupt-driven mode.
Here, you must specify which interrupt pin is the DRDY signal for the gyroscope
interrupt handler, and which interrupt pin is the DRDY signal for the
accelerometer interrupt handler.

.. code-block:: c

   /* Example for an RP2040 MCU */

   /* This function registers the gyroscope interrupt handler and immediately
    * enables it */

   static int board_lsm6dso32_gy_attach(xcpt_t handler, FAR void *arg)
   {
     int err;
     err = rp2040_gpio_irq_attach(GPIO_GYRO_INT, RP2040_GPIO_INTR_EDGE_HIGH,
                                  handler, arg);
     if (err < 0)
       {
         return err;
       }
   
     rp2040_gpio_enable_irq(GPIO_GYRO_INT);
     return err;
   }
   
   /* This function registers the accelerometer interrupt handler and
    * immediately enables it */

   static int board_lsm6dso32_xl_attach(xcpt_t handler, FAR void *arg)
   {
     int err;
     err = rp2040_gpio_irq_attach(GPIO_XL_INT, RP2040_GPIO_INTR_EDGE_HIGH,
                                  handler, arg);
     if (err < 0)
       {
         return err;
       }
   
     rp2040_gpio_enable_irq(GPIO_XL_INT);
     return err;
   }

   /* Registration of the driver */

   struct lsm6dso32_config_s lsm6dso32_config = {
    .gy_int = LSM6DSO32_INT1, /* Gyroscope uses INT1 pin */
    .xl_int = LSM6DSO32_INT2, /* Accelerometer uses INT2 pin */
    .gy_attach = board_lsm6dso32_gy_attach;
    .xl_attach = board_lsm6dso32_xl_attach;
   };
  
   ret = lsm6dso32_register(rp2040_i2cbus_initialize(0), 0x6b, 0,
                            &lsm6dso32_config);
   if (ret < 0)
     {
       syslog(LOG_ERR, "Couldn't register LSM6DSO32 at 0x6b: %d\n", ret);
     }

To debug this device, you can include the ``uorb_listener`` application in your
build with debugging enabled. Running it will show the sensor measurements.

The selftest feature of this device driver makes self-testing available for both
the accelerometer and the gyroscope, based off the information in AN5473 by
STMicroelectronics. It thus only performs the positive self-test. The sensor
under test depends which topic the self-test was called on: the gyroscope or the
accelerometer. The self test for both sensors takes no arguments.

.. warning::
   The self-test feature must be performed while the sensor is stationary.

.. code-block:: c

   err = orb_ioctl(gyro, SNIOC_SELFTEST, 0);
   if (err < 0)
     {
       fprintf(stderr, "Gyroscope self-test failed: %d\n", errno);
     }

The ``SNIOC_SET_CALIBVALUE`` command for this device also varies depending on
which sensor topic it was called on.

For the accelerometer, the argument is an array of 3 floats, representing the X,
Y and Z offsets to be subtracted from measurements in meters per second squared,
in that order.

For the gyroscope, the argument is an array of 3 floats, representing the X,
Y and Z offsets to be subtracted from measurements in radians per second, in
that order.

.. code-block:: c

   /* Accelerometer offset example */

   float offsets[3] = {0.0f, 0.0f, 9.81f};
   err = orb_ioctl(accel, SNIOC_SET_CALIBVALUE, (unsigned long)(offsets));

The interface for setting the measurement interval operates individually on the
gyroscope and accelerometer. That is to say that they can have different
sampling rates.

.. warning::
   This driver does not implement the low-power mode sampling for the
   accelerometer at 1.6Hz, only 12.5Hz and above.

The temperature measurement including in the data for both the accelerometer and
gyroscope is pulled from the same on-board temperature sensor. The output data
rate of this temperature sensor is always 52Hz. This only changes if the
accelerometer is in low or ultra-low power mode, in which case the temperature
ODR matches that of the accelerometer. However, this driver currently does not
implement those power modes.

.. code-block:: c

   unsigned freq = 50;
   err = orb_set_frequency(accel, freq);
   if (err)
     {
       fprintf(stderr, "Wasn't able to set frequency to %uHz: %d\n", freq, err);
       return EXIT_FAILURE;
     }

This sensor also has additional commands for gaining access to extra
functionality.

``SNIOC_WHO_AM_I``
------------------

This command reads the ``WHOAMI`` register of the LSM6DSO32. This should always
return `0x6c`. The argument is a pointer to an 8-bit unsigned integer. This
command has the same result when called on either the accelerometer or gyroscope
topic.

.. code-block:: c

   uint8_t id;
   err = orb_ioctl(accel, SNIOC_WHO_AM_I, (unsigned long)&id);

``SNIOC_SETFULLSCALE``
----------------------

This command allows the user to set the full scale range of either the
accelerometer on the gyroscope.

When called on the accelerometer, the argument should be the desired FSR in
units of 'g'. The available options are 4, 8, 16 and 32g.

When called on the gyroscope, the argument should be the desired FSR in units of
degrees per second. The available options are 125, 250, 500, 1000 and 2000 dps.

Note that by default, the accelerometer has a full scale range of +/-4g and the
gyroscope has a full scale range of +/-125dps.

.. code-block:: c

   err = orb_ioctl(accel, SNIOC_SETFULLSCALE, 16);
   err = orb_ioctl(gyro, SNIOC_SETFULLSCALE, 150);

To check the FSR, you can get the sensor info and check the ``max_range`` field.
This value is in m/s^2 for the accelerometer and rad/s for the gyroscope, so it
must be converted to units of g or degree per second in order to directly
compare it against what was set.

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
