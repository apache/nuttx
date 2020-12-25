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
   in ``arch/``\ *<architecture>*\ ``/src/``\ *<hardware>*
   directory for the specific processor *<architecture>* and for
   the specific *<chip>* RTC peripheral devices.
