====================
FOC Driver Interface
====================

Field Oriented Control (FOC) is a common technique to control
either synchronous or asynchronous alternating current machines.
The main goal of FOC is to control direct current (Id) and
quadrature current (Iq) in powered device.

The device on the kernel side is responsible for the following:

#. update PWM duty cycles
#. return ADC current samples
#. synchronize user-space with PWM events

The NuttX FOC driver is split into two parts:

#. An "upper half", generic driver that provides the common FOC
   interface to application level code,
#. A "lower half", platform-specific driver that implements
   the low-level logic to implement the FOC functionality

Files supporting FOC can be found in the following locations:

-  ``include/nuttx/motor/foc/foc.h``.
   "Upper-half" FOC interface available for the user-space.
-  ``include/nuttx/motor/foc/foc_lower.h``.
   "Lower-half" FOC interface.
-  ``drivers/motor/foc/foc_dev.c``.
   The generic "upper half" FOC driver.

The majority of the functionality available to the application
is implemented in driver ioctl calls. Supported ioctl commands:

- ``MTRIOC_START`` - Start the FOC device, arg: none.
- ``MTRIOC_STOP`` - Stop the FOC device, arg: none.
- ``MTRIOC_GET_STATE`` - Get the FOC device state,
  arg: ``struct foc_state_s`` pointer.
  This is a blocking operation that is used to synchronize the user space
  application with ADC samples.
- ``MTRIOC_CLEAR_FAULT`` - Clear the FOC device fault state,
  arg: none.
- ``MTRIOC_SET_PARAMS`` - Set the FOC device operation parameters,
  arg: ``struct foc_params_s`` pointer.
- ``MTRIOC_SET_CONFIG`` - Set the FOC device configuration,
  arg: ``struct foc_cfg_s`` pointer.
- ``MTRIOC_GET_INFO`` -  Get the FOC device info,
  arg: ``struct foc_info_s`` pointer.
