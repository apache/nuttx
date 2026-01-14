=======
L86-XXX
=======

.. tags:: experimental

This driver provides support for the L86-XXX family of GNSS modules by
Quectel via the :doc:`uorb </components/drivers/special/sensors/sensors_uorb>` interface. 
Functionality for this driver was tested using the Quectel L86-M33.

.. warning::
   This driver only contains preliminary support for a handful of proprietary
   'PMTK' commands There is no support for the entire suite of commands yet.
   This driver also does not use the standard uORD GNSS upper half driver and
   should eventually be modified such that it does. CONSIDER THIS DRIVER EXPERIMENTAL.

Application Programming Interface
=================================

To register the device for use, you will need to enable the standard upper half
serial drivers (``CONFIG_STANDARD_SERIAL``), since the L86-XXX driver requires
the path to the UART interface the module is connected to. You will also need to 
ensure that the baud rate of the UART interface is set to 9600, which is the default 
baud rate of the L86-XXX series of GNSS modules. 

The driver supports changing the default baud rate and update rate of the GNSS
module. As a result, you will also need to enable serial TERMIOS support
(``CONFIG_SERIAL_TERMIOS``). The baud rate and update rate of the GNSS module
can be configured using the ``L86_XXX_BAUD`` and ``L86_XXX_FIX_INT`` options
respectively. Note that a faster update rate will require a higher baud rate to
support it and the supported baud rates for the L86-XXX series of GNSS modules
are: 

* 4800
* 9600
* 14400
* 19200
* 38400
* 57600
* 115200

The baud rate and update rates of the module are changed at registration time.

.. code-block:: c

   #if defined(CONFIG_SENSORS_L86_XXX)
      #include <nuttx/sensors/l86xxx.h>
      
      /* Register L86xxx device on USART2 */

      ret = l86xxx_register("/dev/ttyS2", 0);
      if (ret < 0) {
         syslog(LOG_ERR, "Failed to register L86-M33: %d\n", ret);
      }
   #endif

Once the driver is registered, it starts a thread that continuously reads raw
output from the specified UART device and parses the output according to `NMEA
<https://en.wikipedia.org/wiki/NMEA_0183>`_ standards using the `minmea
<https://github.com/kosma/minmea>`_ library included in NuttX. The driver
populates the ``sensor_gnss`` struct and pushes it to the appropriate event once
all NMEA messages in its sequence have been read.


**uORB commands**
-----------------

The driver implements the ``orb_activate``, ``orb_set_interval`` and,
``orb_ioctl`` operations to interact with the device. The latter is used to send
proprietary 'PMTK' commands which are documented further below.

**Activate**

There are 4 modes that the L86-XXX GNSS modules can be in:

* Full On Mode
* Standby Mode
* Backup Mode
* Periodic Mode
* AlwaysLocateTM Mode

Calling ``orb_activate`` with ``enable`` set to false will enter the
module into "Standby Mode". In "Standby Mode", the module doesn't output any
NMEA messages but the internal core and I/O power domain are still active.

The module can be re-enabled by calling ``orb_activate`` with ``enable`` set to
true, which will hot start the module OR by sending any 'PMTK' command.

**Set interval**

The L86-XXX GNSS modules support interval rates from 1Hz to 10Hz (100ms -
10000ms). When using ``orb_set_interval``, be aware that increasing the interval
of the module may also require and increase in baud rate. An example of how this
is performed can be found in source code of this driver in the register
function.

Any interval rate outside of the supported range will result in a failed call to this function.

**Control**

The ``orb_ioctl`` interface allows one to send proprietary 'PMTK' commands to the L86-XXX GNSS module. It effectively works
as a wrapper for the command framework outlined by Quectel. The return value of calls to ``orb_ioctl`` follow this pattern:

* ``EINVAL`` - Invalid packet
* ``ENOSYS`` - Unsupported packet type
* ``EIO`` - Valid packet, but action failed
* ``0`` - Valid packet, action succeeded
* Other - Command failed during writing

The supported commands are their arguments are listed below.

``SNIOC_HOT_START``
-------------------

Used to "Hot start" the GNSS module. Normally hot start means the GNSS module was powered down for less
than 3 hours (RTC must be alive) and its ephemeris is still valid. As there is no need for downloading 
ephemeris, it is the fastest startup method.

.. code-block:: c

   orb_ioctl(sensor, SNIOC_HOT_START);

``SNIOC_WARM_START``
--------------------

Used to "Warm start" the GNSS module. Warm start means the GNSS module has approximate information of time,
position and coarse data on satellite positions, but it needs to download ephemeris until it can get a fix.

.. code-block:: c

   orb_ioctl(sensor, SNIOC_WARM_START);

``SNIOC_COLD_START``
--------------------

Used to "Cold start" the GNSS module. Using this message will force the GNSS module to be restarted without
any prior location information, including time, position, almanacs and ephemeris data.

.. code-block:: c

   orb_ioctl(sensor, SNIOC_COLD_START);

``SNIOC_FULL_COLD_START``
-------------------------

Used to "Full cold start" the GNSS module. This is effectively the same as a cold restart, but additionally
clears system and user configurations. In other words, this resets the GNSS module to its factory settings.
When full-cold started, the GNSS module has no information on its last location.

.. code-block:: c

   orb_ioctl(sensor, SNIOC_FULL_COLD_START);

``SNIOC_SET_INTERVAL``
----------------------

Used to modify the position fix interval of the GNSS module. The argument is an
integer between 100 and 10000, default value is 1000.

.. code-block:: c

   orb_ioctl(sensor, SNIOC_SET_INTERVAL, 1000);

``SNIOC_SET_BAUD``
------------------
.. note::

   This feature requires termios support to be enabled (``CONFIG_SERIAL_TERMIOS``)

Used to modify the baud rate of the GNSS module. The argument is an integer representing a supported baud rate, default value is 9600.
Upon sending this command, the baud rate of the UART interface used to communicate with the module is also modified.
Supported baud rates for the L86-XXX series of GNSS modules are:

* 4800
* 9600
* 14400
* 19200
* 38400
* 57600
* 115200

.. code-block:: c

   orb_ioctl(sensor, SNIOC_SET_BAUD, 9600);

``SNIOC_SET_OPERATIONAL_MODE``
------------------------------

Used to set the navigation mode of the GNSS module. The argument is an ``L86XXX_OPERATIONAL_MODE`` enum:

* NORMAL - For general purpose
* FITNESS - For instances in which low-speed movements (<5 m/s>) will affect calculations
* AVIATION - For high-dynamic purposes that the large-acceleration movement will have more effect on the position calculation
* BALLOON - For high-altitude balloon purposes that vertical movement will have more effect on the position calculation
* STANDBY - Used to enter standby mode for power saving

Default mode is NORMAL

.. code-block:: c

   orb_ioctl(sensor, SNIOC_SET_OPERATIONAL_MODE, NORMAL);
