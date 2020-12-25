===========
PWM Drivers
===========

For the purposes of this driver, a PWM device is any device that
generates periodic output pulses of controlled frequency and pulse
width. Such a device might be used, for example, to perform
pulse-width modulated output or frequency/pulse-count modulated
output (such as might be needed to control a stepper motor).

The NuttX PWM driver is split into two parts:

#. An "upper half", generic driver that provides the common PWM
   interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the PWM functionality.

Files supporting PWM can be found in the following locations:

-  **Interface Definition**. The header file for the NuttX PWM
   driver reside at ``include/nuttx/timers/pwm.h``. This header
   file includes both the application level interface to the PWM
   driver as well as the interface between the "upper half" and
   "lower half" drivers. The PWM module uses a standard character
   driver framework. However, since the PWM driver is a device
   control interface and not a data transfer interface, the
   majority of the functionality available to the application is
   implemented in driver ioctl calls.
-  **"Upper Half" Driver**. The generic, "upper half" PWM driver
   resides at ``drivers/timers/pwm.c``.
-  **"Lower Half" Drivers**. Platform-specific PWM drivers reside
   in ``arch/``\ *<architecture>*\ ``/src/``\ *<hardware>*
   directory for the specific processor *<architecture>* and for
   the specific *<chip>* PWM peripheral devices.
