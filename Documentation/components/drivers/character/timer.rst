Timer Drivers
=============

Files supporting the timer driver can be found in the following
locations:

-  **Interface Definition**. The header file for the NuttX timer
   driver reside at ``include/nuttx/timers/timer.h``. This header
   file includes both the application level interface to the timer
   driver as well as the interface between the "upper half" and
   "lower half" drivers. The timer driver uses a standard
   character driver framework.
-  **"Upper Half" Driver**. The generic, "upper half" timer driver
   resides at ``drivers/timers/timer.c``.
-  **"Lower Half" Drivers**. Platform-specific timer drivers
   reside in ``arch/``\ *<architecture>*\ ``/src/``\ *<hardware>*
   directory for the specific processor *<architecture>* and for
   the specific *<chip>* timer peripheral devices.
