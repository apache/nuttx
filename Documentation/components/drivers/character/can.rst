===========
CAN Drivers
===========

NuttX supports only a very low-level CAN driver. This driver
supports only the data exchange and does not include any
high-level CAN protocol. The NuttX CAN driver is split into two
parts:

#. An "upper half", generic driver that provides the common CAN
   interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the CAN functionality.

Files supporting CAN can be found in the following locations:

-  **Interface Definition**. The header file for the NuttX CAN
   driver resides at ``include/nuttx/can/can.h``. This header file
   includes both the application level interface to the CAN driver
   as well as the interface between the "upper half" and "lower
   half" drivers. The CAN module uses a standard character driver
   framework.
-  **"Upper Half" Driver**. The generic, "upper half" CAN driver
   resides at ``drivers/can.c``.
-  **"Lower Half" Drivers**. Platform-specific CAN drivers reside
   in ``arch/<architecture>/src/<hardware>``
   directory for the specific processor ``<architecture>`` and for
   the specific ``<chip>`` CAN peripheral devices.

**Usage Note**: When reading from the CAN driver multiple messages
may be returned, depending on (1) the size the returned can
messages, and (2) the size of the buffer provided to receive CAN
messages. *Never assume that a single message will be returned*...
if you do this, *you will lose CAN data* under conditions where
your read buffer can hold more than one small message. Below is an
example about how you should think of the CAN read operation:
