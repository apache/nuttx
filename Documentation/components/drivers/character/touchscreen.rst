==========================
Touchscreen Device Drivers
==========================

NuttX supports a two-part touchscreen driver architecture.

#. An "upper half", generic driver that provides the common
   touchscreen interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level touchscreen controls to implement the touchscreen
   functionality.

Files supporting the touchscreen controller (TSC) driver can be
found in the following locations:

-  **Interface Definition**. The header files for NuttX
   touchscreen drivers reside in the
   ``include/nuttx/include/input`` directory. The interface
   between the touchscreen controller "upper half" and "lower
   half" drivers are *not* common, but vary from
   controller-to-controller. Because of this, each touchscreen
   driver has its own unique header file that describes the "upper
   half"/"lower half" interface in that directory. The application
   level interface to each touchscreen driver, on the other hand,
   *is* the same for each touchscreen driver and is described
   ``include/nuttx/include/input/touchscreen.h``. The touchscreen
   driver uses a standard character driver framework but read
   operations return specially formatted data.
-  **"Upper Half" Driver**. The controller-specific, "upper half"
   touchscreen drivers reside in the directory ``drivers/input``.
-  **"Lower Half" Drivers**. Platform-specific touchscreen drivers
   reside in either: (1) The
   ``arch/<architecture>/src/<hardware>`` directory
   for the processor architectures that have build in touchscreen
   controllers or (2) the
   ``boards/<arch>/<chip>/<board>/src/``
   directory for boards that use an external touchscreen
   controller chip.

Application Programming Interface
=================================

The first thing to be done in order to use the touchscreen driver from an
application is to include the correct header filer. It contains the 
Application Programming Interface to the driver. To do so, include

.. code-block:: c

  #include <nuttx/input/touchscreen.h>

Touchscreen driver is registered as a POSIX character device file into 
``/dev`` namespace. It is necessary to open the device to get a file descriptor
for further operations. This can be done with standard POSIX ``open()`` call.

The driver is accessed through ``read``, ``write``, ``poll`` and ``ioctl``
interface, Following ``ioctl`` commands are available:

 * :c:macro:`TSIOC_GRAB`

.. c:macro:: TSIOC_GRAB

This command let the current handle has the device grabbed. When a handle grabs
a device it becomes sole recipient for all touchscreen events coming from the
device. An argument is an ``int32_t`` variable to enable or disable the grab.


