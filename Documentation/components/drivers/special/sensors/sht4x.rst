=====
SHT4X
=====

Contributed by Matteo Golin.

The SHT4x is a family of temperature and humidity sensors created by Sensirion
which operates over I2C. They include a small heating element.

The driver provided allows interfacing with the sensor over I2C. It has been
tested against the SHT41. This driver uses the :doc:`uorb
</components/drivers/special/sensors/sensors_uorb>` interface.

Application Programming Interface
=================================

The header file for the SHT4X driver interface can be included using:

.. code-block:: c

   #include <nuttx/sensors/sht4x.h>

The SHT4x registration function allows the driver to be registered as a UORB
driver.

The SHT4x measures both ambient temperature and humidity, so registering this
driver will cause two new UORB topics to appear: ``sensor_humi<n>`` and
``sensor_temp<n>``.

.. code-block:: c

   int err;
   err = sht4x_register(i2c_master, 0, 0x44);
   if (err < 0)
   {
     syslog(LOG_ERR, "Couldn't register SHT4X driver at 0x44: %d\n", err);
   }

To debug this device, you can include the ``uorb_listener`` in your build with
debugging enabled. Running it will show the sensor measurements.

This sensor also offers some addition control commands for using the onboard
heater and checking the serial number. These control commands can be used on
either topic (humidity or temperature), since they control the device as a
whole.

``SNIOC_RESET``
----------------

This will perform the SHT4X's soft reset command.

.. code-block:: c

  err = orb_ioctl(sensor, SNIOC_RESET);
  if (err) {
    fprintf(stderr, "SNIOC_RESET: %s\n", strerror(errno));
  } else {
    puts("RESET success!");
  }

``SNIOC_WHO_AM_I``
------------------

This command reads the serial number of the SHT4X sensor. The serial number is
returned in the argument to the command, which must be a `uint32_t` pointer.

.. code-block:: c

  uint32_t serialno = 0;
  err = orb_ioctl(sensor, SNIOC_WHO_AM_I, &serialno);

``SNIOC_HEAT``
--------------

This command will instruct the SHT4X to turn on its heater unit for the
specified time.

The argument to this command must be of type `enum sht4x_heater_e`, which will
indicate the duration the heater is on and the power used.

Heating commands are not allowed more than once per second to avoid damaging the
sensor. If a command is issued before this one second cool-down period is over,
`EAGAIN` is returned.

.. code-block:: c

  err = orb_ioctl(sensor, SNIOC_HEAT, SHT4X_HEATER_200MW_1);

``SNIOC_CONFIGURE``
-------------------

This command allows the caller to configure the precision of the SHT4X sensor
used by subsequent measurement commands. By default, the sensor starts at high
precision.

The argument to this command is one of the values in `enum sht4x_precision_e`.

.. code-block:: c

  err = orb_ioctl(sensor, SNIOC_CONFIGURE, SHT4X_PREC_LOW);
