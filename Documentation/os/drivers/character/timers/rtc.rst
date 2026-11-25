===========
RTC Drivers
===========

NuttX supports a low-level, two-part RealTime Clock (RTC) driver.

#. An "upper half", generic driver that provides the common RTC
   interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the RTC functionality.

Files supporting the RTC driver can be found in the following
locations:

-  **Interface Definition**. The header file for the NuttX RTC
   driver reside at ``include/nuttx/timers/rtc.h``. This header
   file includes both the application level interface to the RTC
   driver as well as the interface between the "upper half" and
   "lower half" drivers. The RTC driver uses a standard character
   driver framework.
-  **"Upper Half" Driver**. The generic, "upper half" RTC driver
   resides at ``drivers/timers/rtc.c``.
-  **"Lower Half" Drivers**. Platform-specific RTC drivers reside
   in ``arch/<architecture>/src/<hardware>``
   directory for the specific processor ``<architecture>`` and for
   the specific ``<chip>`` RTC peripheral devices.

External RTC Drivers
====================

When ``CONFIG_RTC_EXTERNAL`` is selected, the system clock is provided by
an RTC chip on an external bus instead of an on-chip peripheral.  Such
drivers live in ``drivers/timers/`` and implement the ``up_rtc_*()``
interfaces directly.  Since the bus is not yet available when
``clock_initialize()`` runs, board logic must bind the driver to its bus
and then call ``clock_synchronize()`` to load the system time from it.
If the architecture does not provide ``up_rtc_initialize()`` when no
on-chip RTC driver is enabled, the board must provide it (it can simply
return ``OK``).

PCF85063
--------

The NXP PCF85063A/PCF85063TP is an I2C real-time clock with a 32.768 kHz
oscillator, battery backup and a single byte of free RAM.

- **Driver**: ``drivers/timers/pcf85063.c``
- **Header**: ``include/nuttx/timers/pcf85063.h``
- **Configuration**: ``CONFIG_RTC``, ``CONFIG_RTC_DATETIME``,
  ``CONFIG_RTC_EXTERNAL`` and ``CONFIG_RTC_PCF85063``.  The I2C address
  (``CONFIG_PCF85063_I2C_ADDRESS``, default ``0x51``) and bus frequency
  (``CONFIG_PCF85063_I2C_FREQUENCY``, default 400 kHz) can be changed.

Behavior:

- The chip is switched to 24-hour mode during initialization (a 12-hour
  hours register is converted) and its oscillator is restarted if it was
  stopped.  The load capacitance selection is left as found.
- The chip stores two digits of year, and its leap year logic is only
  valid for 2000-2099, so only that range is supported.  Setting a date
  outside it fails with ``-EINVAL``.
- While the oscillator stop (OS) flag is set the time is not valid, and
  ``up_rtc_getdatetime()`` fails with ``-ENODATA``.  Setting the time
  clears the flag.
- The alarm and countdown timer of the PCF85063A are not supported.

Example board initialization:

.. code-block:: c

   #include <nuttx/clock.h>
   #include <nuttx/timers/pcf85063.h>

   struct i2c_master_s *i2c;
   int ret;

   i2c = board_i2c_initialize(0);   /* Board/arch specific */
   if (i2c != NULL)
     {
       ret = pcf85063_rtc_initialize(i2c);
       if (ret >= 0)
         {
           clock_synchronize(NULL);
         }
     }
