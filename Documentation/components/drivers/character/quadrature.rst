==========================
Quadrature Encoder Drivers
==========================

NuttX supports a low-level, two-part Quadrature Encoder driver.

#. An "upper half", generic driver that provides the common
   Quadrature Encoder interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the Quadrature Encoder
   functionality.

Files supporting the Quadrature Encoder can be found in the
following locations:

-  **Interface Definition**. The header file for the NuttX
   Quadrature Encoder driver reside at
   ``include/nuttx/sensors/qencoder.h``. This header file includes
   both the application level interface to the Quadrature Encoder
   driver as well as the interface between the "upper half" and
   "lower half" drivers. The Quadrature Encoder module uses a
   standard character driver framework.
-  **"Upper Half" Driver**. The generic, "upper half" Quadrature
   Encoder driver resides at ``drivers/sensors/qencoder.c``.
-  **"Lower Half" Drivers**. Platform-specific Quadrature Encoder
   drivers reside in
   ``arch/``\ *<architecture>*\ ``/src/``\ *<hardware>* directory
   for the specific processor *<architecture>* and for the
   specific *<chip>* Quadrature Encoder peripheral devices.

