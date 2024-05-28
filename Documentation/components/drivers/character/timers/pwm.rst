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
   in ``arch/<architecture>/src/<hardware>``
   directory for the specific processor ``<architecture>`` and for
   the specific ``<chip>`` PWM peripheral devices.

Application Level Interface
===========================

The first necessary thing to be done in order to use the PWM driver in an
application is to include the header file for the NuttX timer driver. It contains
the Application Level Interface to the PWM driver. To do so, include:

.. code-block:: c

  #include <nuttx/timers/pwm.h>

PWM driver is registered as a POSIX character device file into ``/dev``
namespace. It is necessary to open the device to get a file descriptor for
further operations.

The PWM is accessed only through ``ioctl`` interface, functions ``read``
and ``write`` does not have any affect. Following ``ioctl`` commands are
available:

 * :c:macro:`PWMIOC_SETCHARACTERISTICS`
 * :c:macro:`PWMIOC_GETCHARACTERISTICS`
 * :c:macro:`PWMIOC_START`
 * :c:macro:`PWMIOC_STOP`

.. c:macro:: PWMIOC_SETCHARACTERISTICS

The ``PWMIOC_SETCHARACTERISTICS`` command sets PWM characteristics such as
frequency, duty cycle, dead times and so on. These characteristics are
set through ``pwm_info_s`` structure.

.. c:struct:: pwm_info_s
.. code-block:: c

   struct pwm_info_s
   {
      /* Frequency of the pulse train */
      uint32_t           frequency;
   #ifdef CONFIG_PWM_MULTICHAN
      /* Per-channel output state */
      struct pwm_chan_s  channels[CONFIG_PWM_NCHANNELS];
   #else
      /* Duty of the pulse train, "1"-to-"0" duration.
       * Maximum: 65535/65536 (0x0000ffff)
       * Minimum:     1/65536 (0x00000001)
       */
      ub16_t             duty;
   #ifdef CONFIG_PWM_DEADTIME
      /* Dead time value for main output */
      ub16_t             dead_time_a;
      /* Dead time value for complementary output */
      ub16_t             dead_time_b;
   #endif
   #ifdef CONFIG_PWM_PULSECOUNT
      /* The number of pulse to generate.  0 means to
       * generate an indefinite number of pulses
       */
      uint32_t           count;
   #endif
      /* Channel polarity */
      uint8_t            cpol;
      /* Disabled channel polarity */
      uint8_t            dcpol;
   #endif /* CONFIG_PWM_MULTICHAN */
      /* User provided argument to be used in the lower half */
      FAR void           *arg;
   };

Structure ``pwm_chan_s`` holds the representation of one PWM channel
if multiple channels are used ( ``CONFIG_PWM_MULTICHAN`` is set).

.. c:struct:: pwm_chan_s
.. code-block:: c

   struct pwm_chan_s
   {
      /* Duty of the pulse train, "1"-to-"0" duration.
       * Maximum: 65535/65536 (0x0000ffff)
       * Minimum:     1/65536 (0x00000001)
       */
      ub16_t duty;
   #ifdef CONFIG_PWM_OVERWRITE
      /* Channel overwrite */
      bool ch_outp_ovrwr;
      /* Channel overwrite value */
      bool ch_outp_ovrwr_val;
   #endif
   #ifdef CONFIG_PWM_DEADTIME
      /* Dead time value for main output */
      ub16_t dead_time_a;
      /* Dead time value for complementary output */
      ub16_t dead_time_b;
   #endif
      /* Channel polarity */
      uint8_t cpol;
      /* Disabled channel polarity */
      uint8_t dcpol;
      /* Channel number */
      int8_t channel;
   };

Apart from duty cycle and frequency, the ``ioctl`` command allows to
set many other PWM characteristics. These functionalities might not be
supported by all PWM controllers and user should always refer to target
documentation in this case.

If ``CONFIG_PWM_OVERWRITE`` is set and ``ch_outp_ovrwr`` is true, it is
possible to overwrite channel output with value set in ``ch_outp_ovrwr_val``.
Configuration option ``CONFIG_PWM_DEADTIME`` and fields ``dead_time_a``
and ``dead_time_b`` provides an option to set dead time between complementary
outputs. This instructs the driver to automatically insert output activation
delay for complementary PWM outputs and is useful for H-bridge motor control
for example.

User may also set default channel polarity ``cpol`` and disabled channel
polarity ``dcpol``. If set to zero, default controller values (or values
determined in the configuration) are used. Following defines can be used
to set the polarities:

.. code-block:: c

   /* Not defined, the default output state is arch dependant */
   #define PWM_CPOL_NDEF             0
   /* Logical zero */
   #define PWM_CPOL_LOW              1
   /* Logical one */
   #define PWM_CPOL_HIGH             2

   /* Not defined, the default output state is arch dependant */
   #define PWM_DCPOL_NDEF            0
    /* Logical zero */
   #define PWM_DCPOL_LOW             1
    /* Logical one  */
   #define PWM_DCPOL_HIGH            2

.. c:macro:: PWMIOC_GETCHARACTERISTICS

Command ``PWMIOC_GETCHARACTERISTICS`` operates the same way as
``PWMIOC_SETCHARACTERISTICS`` but it obtains currently set values
instead of setting them.

.. c:macro:: PWMIOC_START

The ``PWMIOC_START`` command starts the pulsed output. Characteristics
of PWM channels should be set before this operation.

.. c:macro:: PWMIOC_STOPS

The ``PWMIOC_STOPS`` command stops the pulsed output.

Application Example
~~~~~~~~~~~~~~~~~~~

An example application can be found in ``nuttx-apps`` repository under the
path ``examples/pwm``.

.. code-block :: bash

   nsh> pwm

Configuration
=============

This section describes common PWM configuration in ``Kconfig``. The reader
should refer to target documentation for target specific configuration.

PWM is enabled by ``CONFIG_PWM`` configuration option. Option
``CONFIG_PWM_MULTICHAN`` selects support for multiple channels for one PWM
instance. If multiple channels are used, configuration option
``CONFIG_PWM_NCHANNELS`` defines the maximum number of channels per instance.
Each timer/controller may support fewer output channels than this value.

Generation of pin overwrite is enabled by ``CONFIG_PWM_OVERWRITE`` option.
This supports generation of a pin overwrite with 0 or 1 without the need to
wait for an end of cycle.

The ``CONFIG_PWM_DEADTIME`` option brings the possibility to introduce
dead time values between complementary PWM outputs.
