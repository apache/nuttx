======================
Watchdog Timer Drivers
======================

NuttX supports a low-level, two-part watchdog timer driver.

#. An "upper half", generic driver that provides the common
   watchdog timer interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the watchdog timer
   functionality.

Files supporting the watchdog timer driver can be found in the
following locations:

-  **Interface Definition**. The header file for the NuttX
   watchdog timer driver reside at
   ``include/nuttx/timers/watchdog.h``. This header file includes
   both the application level interface to the watchdog timer
   driver as well as the interface between the "upper half" and
   "lower half" drivers. The watchdog timer driver uses a standard
   character driver framework.
-  **"Upper Half" Driver**. The generic, "upper half" watchdog
   timer driver resides at ``drivers/timers/watchdog.c``.
-  **"Lower Half" Drivers**. Platform-specific watchdog timer
   drivers reside in
   ``arch/``\ *<architecture>*\ ``/src/``\ *<hardware>* directory
   for the specific processor *<architecture>* and for the
   specific *<chip>* watchdog timer peripheral devices.
