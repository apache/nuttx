======
TC74Ax
======

Driver for TC74Ax thermal sensor by Microchip. There are multiple variants
of this sensor named TC74A0, TC74A1, TC74A2 etc. up to TC74A7. This driver
is suitable for all of them.

.. Listed at least some of the chips by full name to make them visible
.. for search engines

Configuration
=============

As with other sensors, the driver is enabled
in :menuselection:`Device Drivers --> Sensor device support`.
It can be further configured as follows:

I\ :sup:`2`\ C frequency
------------------------

I\ :sup:`2`\ C frequency used when communicating with the device.
Must be in range of 10-100kHz.

Power Management
----------------

The sensor is continuously doing temperature measurements since
power-on. It also features a standby mode with reduced power consumption.
The drivers has multiple options for doing power management:

Standby unless opened
^^^^^^^^^^^^^^^^^^^^^

The driver will switch the sensor to standby mode during initialization.
It is then kept in standby mode until an application opens corresponding
device file.

When all applications close the device file, the sensor is put back
to standby mode.

Manual control with ``ioctl``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Sensor's operating mode is controlled manually using
``SNIOC_SET_OPERATIONAL_MODE`` ``ioctl``. The parameter must
be one of ``TC74AX_OPERATION_MODE_OPERATING`` or
``TC74AX_OPERATION_MODE_STANDBY``. Example:

.. code-block:: c

   ioctl(sensor_fd, SNIOC_SET_OPERATIONAL_MODE, TC74AX_OPERATION_MODE_STANDBY);

Note that attempt to read temperature from sensor in standby mode
is considered an error and the driver will return ``EIO``.

This selection unlocks an additional option to switch the sensor
to standby mode during boot. If not selected, the sensor will be
kept in running state.

No power management
^^^^^^^^^^^^^^^^^^^

The driver provides no means for power management and the sensor
is always kept in running state.

Multimaster mode
^^^^^^^^^^^^^^^^

With this mode, it is assumed that the sensor is used by multiple
I\ :sup:`2`\ C masters. Most notably, this relates to sensor's
power management. The driver will obey its configuration for power
management as given by previous configuration options but then it
will make no assumptions on sensor's current state. Instead, it
will determine it in runtime every time the temperature is read.

Note that if no power management is configured and the driver
finds that the sensor is in standby mode (presumably configured
that way by another master), it will not be able to switch
it to running state and will return ``EIO`` instead.

See more details an additional information in Kconfig entry
for this configuration item.

API and usage
=============

Registration
------------

Include header file for the driver:


.. code-block:: c

   #include <nuttx/sensors/tc74ax.h>

Register the device:

.. code-block:: c

   tc74ax_register("/dev/tc74ax", i2c, address);

Parameter ``i2c`` is a pointer to ``struct i2c_master_s``. Address provided
in ``address`` variable must be a positive number in range of 72-79. This is
a base address of 72 with the chip variant (0-7) added to it.

First parameter is a path in ``/dev`` and can be chosen to suit board's needs.

Temperature reading
-------------------

The driver is not an
:doc:`UORB </components/drivers/special/sensors/sensors_uorb>` driver,
the application needs to read and interpret its values directly:

.. code-block:: c

   int8_t buffer;

   fd = open("/dev/tc74ax", O_RDONLY);
   res = read(fd, &buffer, 1);

Value stored in ``buffer`` is a temperature in degrees Celsius.
(See the datasheet for temperature range.)

The read is synchronous. Whenever ``read`` function is called,
the driver submits I\ :sup:`2`\ C request to read current temperature.

In case of errors, the driver mostly returns error code
from the underlying I\ :sup:`2`\ C driver. Other than that, most
error conditions cause return of ``EIO``.

Reading duration and timeouts
-----------------------------

The duration of read always depends on I\ :sup:`2`\ C bus being
available and on time needed for the transmission. Other than that, there
are additional factors that may increase the read duration.

Wakeup from standby mode
^^^^^^^^^^^^^^^^^^^^^^^^

The sensor needs some time to do the first measurement after being
switched from standby to running mode. The read will wait for that
to happen.

Multimaster contention
^^^^^^^^^^^^^^^^^^^^^^

In multimaster mode, the driver always reads the power state
of the sensor before reading the temperature. The read therefore
takes more time because 4 I\ :sup:`2`\ C messages need
to be transmitted instead of one.

Additionally - if the read detects that the sensor is in standby
mode, the read duration increases by the time needed to wake it up
and to do the first measurement.

Note that if the contention is severe enough - that is, if the other
master keeps switching the sensor to standby mode - the read may
never complete successfully. There is a timeout imposed on total
duration of reading attempts and ``EIO`` is returned if this timeout
is exceeded.

I/O error recovery
------------------

Whenever there is an I/O error on the I\ :sup:`2`\ C bus, recovery
procedure may be needed. If configured for non-multimaster mode,
this depends on power management mode:

  * with no power management, no recovery is needed. Further reads
    will simply try again
  * with ``ioctl``-based power management, the user needs to switch
    the sensor to standby mode and then back to running mode
  * with power management based on opening the device file,
    all users need to close the file handle

If configured for multimaster mode, no error recovery is needed.
