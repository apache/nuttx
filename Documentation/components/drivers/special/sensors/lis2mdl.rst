=======
LIS2MDL
=======

Contributed by Matteo Golin.

The LIS2MDL is a low power, high-performance 3-axis magnetometer by ST
Microelectronics. It has I2C and SPI interfaces, although this driver currently
only supports I2C.

The driver uses the :doc:`uorb
</components/drivers/special/sensors/sensors_uorb>` interface. It supports the
self-test capability

Application Programming Interface
=================================

.. code-block:: c

   #include <nuttx/sensors/lis2mdl.h>

The LIS2MDL registration function allows the driver to be registered as a UORB
driver. Registering this driver will cause the ``/dev/uorb/sensor_mag<n>`` topic
to appear, where ``n`` is the value of ``devno``.

The driver can be registered either in polling mode or in interrupt-driven mode.
The polling mode will create a kernel thread to poll the sensor periodically
according to the set interval. Polling mode is registered with ``NULL`` instead
of a real function pointer to the ``attach`` parameter, like so:

.. code-block:: c

   int err;

   /* Creates /dev/uorb/mag0 in polled mode */

   err = lis2mdl_register(i2c_master, 0, 0x1e, NULL);
   if (err < 0)
   {
     syslog(LOG_ERR, "Could not register LIS2MDL driver at 0x1E: %d\n", err);
   }

To register in interrupt-driven mode, a function must be provided by the
calling code to register the LIS2MDL interrupt handler properly. This function
should take the interrupt handler and ``arg`` reference as arguments, and return
0 on success or a negated error code on failure. This function must also enable
the interrupt after it is registered successfully.

.. warning:: 
   To use interrupt-driven mode, ``CONFIG_SCHED_HPWORK`` must be enabled.

The example below shows the example process for an RP2040 based board, but it
will be similar on other architectures:

.. code-block:: c

   /* IRQ attach function example for RP2040 board */

   static int board_lis2mdl_attach(xcpt_t handler, FAR void *arg)
   {
     int err;
     err = rp2040_gpio_irq_attach(GPIO_MAG_INT, RP2040_GPIO_INTR_EDGE_HIGH,
                                  handler, arg);
     if (err < 0)
       {
         return err;
       }

     rp2040_gpio_enable_irq(GPIO_MAG_INT);
     return err;
   }

   /* Later, in the board bringup code ... */

   int err;
   err = lis2mdl_register(i2c_master, 0, 0x1e, board_lis2mdl_attach);
   if (err < 0)
   {
     syslog(LOG_ERR, "Couldn't register LIS2MDL driver: %d\n", err);
   }

To debug this device, you can include the ``uorb_listener`` in your build with
debugging enabled. Running it will show the sensor measurements.

.. warning:: 
   By default, when the sensor is deactivated via the UORB interface, it is put
   into low power mode and set to idle. When it is reactivated, it is put in
   high resolution mode and set to continuous measurement. If you want to
   measure continuously in low power mode, you will need to use the
   ``SNIOC_SET_POWER_MODE`` command explained below.

The ``set_calibvalue`` interface to this sensor takes an array of three `float`
types, representing hard-iron offsets in micro Teslas. This offset is set on the
sensor and is subtracted from measurements to compensate for environmental
effects.

Some additional control commands for the LIS2MDL are listed below.

``SNIOC_WHO_AM_I``
------------------

This command reads the ``WHOAMI`` register of the LIS2MDL. This should always
return ``0x40``. The argument is a pointer to an 8-bit unsigned integer.

.. code-block:: c

   uint8_t id; /* Should always contain 0x40 */
   err = orb_ioctl(sensor, SNIOC_WHO_AM_I, &id);

``SNIOC_SET_POWER_MODE``
------------------------

This command selects the power mode of the LIS2MDL sensor. An argument of
``true`` puts the sensor in low power mode, and ``false`` puts the sensor into
high resolution mode.

.. code-block:: c

   /* Puts LIS2MDL into low power mode */
   err = orb_ioctl(sensor, SNIOC_WHO_AM_I, true);

``SNIOC_RESET``
----------------

Performs a soft reset of the LIS2MDL, which resets the user registers. This
command takes no arguments. Once this command is issue, 5 microseconds must pass
before the sensor is operational again.

.. code-block:: c

   err = orb_ioctl(sensor, SNIOC_RESET, NULL);

``SNIOC_SENSOR_OFF``
--------------------

Performs a reboot of the LIS2MDL's memory contents. This command takes no
arguments. After the command is issued, 20ms must pass before the sensor is
operational again.

.. code-block:: c

   err = orb_ioctl(sensor, SNIOC_SENSOR_OFF, NULL);

``SNIOC_SET_TEMP_OFFSET``
--------------------------

Enables or disables temperature compensation on the magnetometer. A arg of
``true`` enables compensation, ``false`` disables it. By default this is
enabled.

.. code-block:: c

   err = orb_ioctl(sensor, SNIOC_SET_TEMP_OFFSET, true);

``SNIOC_LPF``
-------------

Enables or disables the magnetometer low pass filter. A arg of ``true`` enables
the filter, ``false`` disables it. By default this is disabled.

.. code-block:: c

   err = orb_ioctl(sensor, SNIOC_LPF, true);
