===================
Pulsecount Drivers
===================

The pulsecount driver generates a finite pulse train and reports completion
after the requested number of pulses has been produced.  It is intended for
hardware that can generate pulse output with a fixed repetition count, often
using PWM/timer peripherals internally.

This interface is separate from the PWM driver.  PWM describes a continuous
periodic output using frequency and duty cycle.  Pulsecount describes a finite
waveform using explicit high time, low time, and pulse count.

Driver Model
============

The NuttX pulsecount driver is split into two parts:

#. An upper-half character driver that provides the common application
   interface.
#. A lower-half platform driver that programs the hardware timer/PWM
   peripheral and calls ``pulsecount_expired()`` when the finite pulse train
   completes.

Files supporting pulsecount can be found in the following locations:

- ``include/nuttx/timers/pulsecount.h`` - public interface and lower-half
  callbacks.
- ``drivers/timers/pulsecount.c`` - generic upper-half driver.
- ``arch/<architecture>/src/<chip>/*pulsecount*.c`` - platform lower-half
  drivers.

Application Interface
=====================

Applications use the pulsecount driver through a character device such as
``/dev/pulsecount0``.  Include the pulsecount header:

.. code-block:: c

   #include <nuttx/timers/pulsecount.h>

The driver is controlled through ``ioctl`` commands:

- ``PULSECOUNTIOC_SETCHARACTERISTICS``
- ``PULSECOUNTIOC_GETCHARACTERISTICS``
- ``PULSECOUNTIOC_START``
- ``PULSECOUNTIOC_STOP``

Pulse Characteristics
=====================

``PULSECOUNTIOC_SETCHARACTERISTICS`` takes a pointer to
``struct pulsecount_info_s``:

.. code-block:: c

   struct pulsecount_info_s
   {
     uint32_t high_ns;  /* Pulse high time in nanoseconds */
     uint32_t low_ns;   /* Pulse low time in nanoseconds */
     uint32_t count;    /* Number of pulses to generate */
   };

``high_ns`` and ``low_ns`` must both be non-zero.  Their sum is the pulse
period.  ``count`` is the number of complete high/low pulses to generate.

This API previously followed the PWM-style ``frequency`` plus ``duty``
model.  It now uses explicit high/low nanosecond timing because that is more
user-friendly for finite pulse trains: applications can describe the waveform
directly and do not need to convert timing requirements into fixed-point duty
cycle values.

The lower-half driver may quantize the requested timings to the nearest values
that the hardware timer can represent.  Very long periods or very short
pulses can be rejected if they are outside the timer's clock and counter
range.

Starting And Stopping
=====================

After setting the pulse characteristics, start the pulse train with
``PULSECOUNTIOC_START``.  By default, this call blocks until the requested
pulse count completes.  Open the device with ``O_NONBLOCK`` to start the
pulse train and return immediately.

``PULSECOUNTIOC_STOP`` stops pulse generation before the count completes.
TODO: support cancelling a blocking ``PULSECOUNTIOC_START``.

Example
=======

The ``apps/examples/pulsecount`` example starts a finite pulse train from
NSH.  It can be built with ``CONFIG_EXAMPLES_PULSECOUNT`` and configured with
``CONFIG_EXAMPLES_PULSECOUNT_HIGH_NS``,
``CONFIG_EXAMPLES_PULSECOUNT_LOW_NS``, and
``CONFIG_EXAMPLES_PULSECOUNT_COUNT``.
