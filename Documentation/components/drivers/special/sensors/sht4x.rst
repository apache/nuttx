SHT4X
=====

Contributed by Matteo Golin.

The SHT4x is a family of temperature and humidity sensors created by Sensirion
which operates over I2C. They include a small heating element.

The driver provided allows interfacing with the sensor over I2C. It has been
tested against the SHT41.

Application Programming Interface
=================================

The header file for the SHT4X driver interface can be included using:

.. code-block:: c

   # include <nuttx/sensors/sht4x.h>

The SHT4x registration function allows the driver to be registered as a POSIX
character driver.

The standard POSIX `read()` operation will return the temperature and humidity
measurements in plain-text, which is useful when debugging/testing the driver
using `cat` from the shell.

The `write()` operation is not implemented for this sensor.

Specific operations the sensor offers can be performed via the POSIX `ioctl`
operation. The supported commands are:

 * :c:macro:`SNIOC_RESET`
 * :c:macro:`SNIOC_WHO_AM_I`
 * :c:macro:`SNIOC_READ_RAW_DATA`
 * :c:macro:`SNIOC_MEASURE`
 * :c:macro:`SNIOC_READ_CONVERT_DATA`
 * :c:macro:`SNIOC_HEAT`
 * :c:macro:`SNIOC_CONFIGURE`

.. c:macro:: SNIOC_RESET

   This will perform the SHT4X's soft reset command.

.. code-block:: c

  err = ioctl(sensor, SNIOC_RESET);
  if (err) {
    fprintf(stderr, "SNIOC_RESET: %s\n", strerror(errno));
  } else {
    puts("RESET success!");
  }

.. c:macro:: SNIOC_WHO_AM_I

This command reads the serial number of the SHT4X sensor. The serial number is
returned in the argument to the command, which must be a `uint32_t` pointer.

.. code-block:: c

  uint32_t serialno = 0;
  err = ioctl(sensor, SNIOC_WHO_AM_I, &serialno);

.. c:macro:: SNIOC_READ_RAW_DATA

This command allows the caller to read the raw data returned from the sensor,
without the driver performing any calculation to convert it into familiar units
(i.e. degrees Celsius for temperature).

The argument to this command must be a pointer to a `struct sht4x_raw_data_s`
structure. The raw data will be returned here.

.. code-block:: c

  struct sht4x_raw_data_s raw;
  err = ioctl(sensor, SNIOC_READ_RAW_DATA, &raw);

.. c:macro:: SNIOC_MEASURE

This command will measure temperature and humidity, and return it in familiar
units to the user. Temperature will be in degrees (Fahrenheit or Celsius depends
on the Kconfig options selected during compilation) and humidity will be %RH.

The argument to this command must be a pointer to a `struct sht4x_conv_data_s`.
This is where the converted data will be returned.

.. code-block:: c

  struct sht4x_conv_data_s data;
  err = ioctl(sensor, SNIOC_MEASURE, &data);

.. c:macro:: SNIOC_READ_CONVERT_DATA

Same as `SNIOC_MEASURE`.

.. c:macro:: SNIOC_HEAT

This command will instruct the SHT4X to turn on its heater unit for the
specified time. Afterwards, a measurement of temperature and humidity is taken,
and the converted data is returned to the caller.

The argument to this command must be a pointer to a `struct sht4x_conv_data_s`.
This is where the converted data will be returned. The `temperature` field of
the struct must contain a value from the `enum sht4x_heater_e`, which will
indicate the duration the heater is on and the power used.

Heating commands are not allowed more than once per second to avoid damaging the
sensor. If a command is issued before this one second cool-down period is over,
`EAGAIN` is returned.

.. code-block:: c

  struct sht4x_conv_data_s data;
  data.temp = SHT4X_HEATER_200MW_1;
  err = ioctl(sensor, SNIOC_HEAT, &data);

.. c:macro:: SNIOC_CONFIGURE

This command allows the caller to configure the precision of the SHT4X sensor
used by subsequent measurement commands. By default, the sensor starts at high
precision.

The argument to this command is one of the values in `enum sht4x_precision_e`.

.. code-block:: c

  err = ioctl(sensor, SNIOC_CONFIGURE, SHT4X_PREC_LOW);
