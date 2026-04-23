===========
OCU Drivers
===========

The OCU (Output Compare Unit) driver provides a standard interface for
hardware output compare functionality.  An OCU channel compares a free-running
counter against a programmable threshold and performs configurable actions
(pin toggle, notification callback) on match.

This interface is distinct from PWM, Timer, and Pulsecount drivers:

- **PWM** outputs a continuous periodic waveform (frequency + duty cycle).
- **Timer** fires after a relative time interval (countdown).
- **Pulsecount** generates a finite pulse train (high_ns + low_ns × count)
  and stops.  All pulses share the same parameters; no per-pulse variation or
  static pin level control is supported.
- **OCU** triggers at an absolute counter position and can dynamically update
  the next trigger point from within the compare match ISR.  It also provides
  direct pin level control (``SETPINSTATE``) independent of compare match
  events.

OCU is commonly used for precise time-based control such as engine ignition
timing, camshaft angle control, and motor commutation where the trigger point
must be programmed as an absolute position on a free-running counter.

Driver Model
============

The NuttX OCU driver is split into two parts:

#. An upper-half character driver that provides the common application
   interface via ioctl commands.
#. A lower-half platform driver that programs the hardware timer/compare
   peripheral.  The lower-half holds a callback pointer and calls it directly
   from the compare match ISR (timer-style ``setcallback`` pattern).

Files supporting OCU can be found in the following locations:

- ``include/nuttx/timers/ocu.h`` - public interface, types, ioctl commands,
  and lower-half ops structure.
- ``drivers/timers/ocu.c`` - generic upper-half driver.
- ``arch/<architecture>/src/<chip>/*ocu*.c`` - platform lower-half drivers.

Application Interface
=====================

Applications use the OCU driver through a character device such as
``/dev/ocu0``.  Include the OCU header:

.. code-block:: c

   #include <nuttx/timers/ocu.h>

The driver is controlled through ``ioctl`` commands:

- ``OCUIOC_START`` - Start the channel, enabling compare match actions.
- ``OCUIOC_STOP`` - Stop compare match actions (counter keeps running).
- ``OCUIOC_GETSTATUS`` - Get channel status (flags, counter, threshold).
- ``OCUIOC_GETCOUNTER`` - Read current counter value in ticks.
- ``OCUIOC_SETABSTHRESHOLD`` - Set absolute compare threshold.
- ``OCUIOC_SETRELTHRESHOLD`` - Set threshold relative to current counter.
- ``OCUIOC_SETADVANCETHRESHOLD`` - Advance threshold by step from current threshold.
- ``OCUIOC_SETPINSTATE`` - Set output pin level (only when stopped).
- ``OCUIOC_SETPINACTION`` - Set automatic pin action on compare match.
- ``OCUIOC_NOTIFICATION`` - Enable/disable signal notification to user space.

Threshold Setting
=================

Three modes are available for setting the compare threshold, corresponding to
AUTOSAR OCU specification SRS_Ocu_00010:

**Absolute** (``OCUIOC_SETABSTHRESHOLD``):

.. code-block:: c

   uint32_t current_counter;
   ioctl(fd, OCUIOC_GETCOUNTER, &current_counter);

   struct ocu_abs_threshold_s t = {
       .reference = current_counter,  /* Base for interval check */
       .absolute  = 5000,             /* Trigger at tick 5000 */
   };
   ioctl(fd, OCUIOC_SETABSTHRESHOLD, &t);
   /* t.result indicates if match is in current reference interval */

**Relative to counter** (``OCUIOC_SETRELTHRESHOLD``):

.. code-block:: c

   struct ocu_rel_threshold_s t = { .relative = 1000 };
   ioctl(fd, OCUIOC_SETRELTHRESHOLD, &t);
   /* new_threshold = current_counter + 1000 */

**Advance from current threshold** (``OCUIOC_SETADVANCETHRESHOLD``):

.. code-block:: c

   struct ocu_advance_threshold_s t = { .step = 500 };
   ioctl(fd, OCUIOC_SETADVANCETHRESHOLD, &t);
   /* new_threshold = current_threshold + 500 */

The last mode is useful for evenly-spaced triggers that span counter rollover.

Pin Control
===========

OCU channels may have an associated output pin.  The pin can be controlled in
two ways:

- ``OCUIOC_SETPINSTATE`` sets the pin level immediately (LOW or HIGH).  This
  is only allowed when the channel is stopped.
- ``OCUIOC_SETPINACTION`` configures the automatic action on compare match:
  ``OCU_PINACTION_DISABLE``, ``OCU_PINACTION_SET_LOW``,
  ``OCU_PINACTION_SET_HIGH``, or ``OCU_PINACTION_TOGGLE``.

If the hardware does not have an output pin, these calls return ``-ENOTSUP``.

Notification Mechanisms
=======================

Two notification paths exist for compare match events, following the same
design pattern as the NuttX Timer driver (``TIMER_SETCALLBACK`` macro vs
``TCIOC_NOTIFICATION`` ioctl):

Kernel-Internal Callback (ISR Context)
--------------------------------------

For lowest latency, kernel drivers can register a callback directly with the
lower-half using the ``OCU_SETCALLBACK()`` macro.  The callback runs in
interrupt context and can atomically update the next threshold:

.. code-block:: c

   #include <nuttx/timers/ocu.h>

   static bool my_handler(FAR uint32_t *next_threshold, FAR void *arg)
   {
       *next_threshold = (*next_threshold + STEP) % MAX;
       return true;  /* Re-arm with new threshold */
   }

   /* Called from kernel code, NOT from user-space ioctl */
   OCU_SETCALLBACK(lower, my_handler, priv);
   OCU_NOTIFICATION(lower, true);   /* Enable the hardware compare match IRQ */

.. important::

   ``OCU_SETCALLBACK()`` and ``OCU_NOTIFICATION()`` are **separate**
   operations and must be used together on the kernel-internal path:

   - ``OCU_SETCALLBACK()`` only *stores* the callback pointer in the
     lower-half.  It does **not** enable any hardware interrupt.
   - ``OCU_NOTIFICATION(lower, true)`` enables the hardware compare-match
     notification (e.g. ``Ocu_EnableNotification()`` on Infineon AURIX);
     ``OCU_NOTIFICATION(lower, false)`` disables it.

   A kernel driver that registers a callback via ``OCU_SETCALLBACK()`` but
   forgets ``OCU_NOTIFICATION(lower, true)`` will **never** receive compare
   match callbacks, because the hardware interrupt stays disabled.  To tear
   down, disable first, then clear the callback::

      OCU_NOTIFICATION(lower, false);
      OCU_SETCALLBACK(lower, NULL, NULL);

.. note::

   ``OCU_SETCALLBACK()`` is a macro that calls ``lower->ops->setcallback()``
   when the lower-half provides it (the call is skipped safely if the op is
   ``NULL``).  It is only available to kernel-internal drivers (same pattern
   as ``TIMER_SETCALLBACK``).  There is **no** ioctl for registering function
   pointers from user space.  Neither ``start()`` nor ``stop()`` touches the
   notification state; enabling/disabling the hardware interrupt is done
   exclusively through ``OCU_NOTIFICATION()``.

User-Space Signal Notification
------------------------------

User-space applications receive compare match events via signal notification,
configured through ``OCUIOC_NOTIFICATION``:

.. code-block:: c

   struct ocu_notify_s n = {
       .enable = true,
       .pid    = getpid(),
       .event  = { .sigev_signo = SIGUSR1 },
   };
   ioctl(fd, OCUIOC_NOTIFICATION, &n);

When enabled, the ``OCUIOC_NOTIFICATION`` handler performs two steps: it
registers an internal notifier callback (``ocu_notifier``) with the
lower-half via ``setcallback()``, then enables the hardware compare-match
notification via the ``notification()`` op.  On compare match, the hardware
interrupt invokes ``ocu_notifier``, which sends a signal to the registered
process.  Disabling reverses the order (disable hardware notification, then
clear the callback).  The upper-half does **not** re-arm the threshold
automatically — user space must explicitly set the next threshold via ioctl
after receiving the signal.

To disable notification:

.. code-block:: c

   struct ocu_notify_s n = { .enable = false };
   ioctl(fd, OCUIOC_NOTIFICATION, &n);

.. note::

   The two paths are mutually exclusive on the same channel.  If a kernel
   driver registers a callback via ``OCU_SETCALLBACK()``, it overrides any
   user-space notification previously set up through ``OCUIOC_NOTIFICATION``.

Lifecycle
=========

The OCU driver supports multiple opens via reference counting:

- First ``open()`` calls ``ops->setup()`` to initialize hardware.
- Last ``close()`` calls ``ops->stop()`` (if running) then
  ``ops->shutdown()`` to release hardware.

The channel state machine:

.. code-block:: text

   [CLOSED] --open()--> [STOPPED] --start()--> [RUNNING]
                            ^                       |
                            |------stop()-----------|
   [STOPPED] --close(last fd)--> [CLOSED]

Configuration
=============

- ``CONFIG_OCU`` - Enable the OCU driver framework.

Lower-Half Interface
====================

Platform drivers implement ``struct ocu_ops_s`` and register with:

.. code-block:: c

   int ocu_register(FAR const char *devpath,
                    FAR struct ocu_lowerhalf_s *lower);

Required ops: ``setup``, ``shutdown``, ``start``, ``stop``, ``getstatus``,
``getcounter``, ``setabsthreshold``, ``setrelthreshold``, ``setcallback``.

Optional ops (NULL if unsupported): ``notification``, ``setadvancethreshold``,
``setpinstate``, ``setpinaction``, ``ioctl``.

The ``notification`` op enables/disables the hardware compare-match interrupt
and is invoked by the upper-half from the ``OCUIOC_NOTIFICATION`` ioctl (and
on ``close()``), paired with ``setcallback``.  If a lower-half leaves it
``NULL``, ``OCU_NOTIFICATION()`` returns ``-ENOTSUP`` and hardware
compare-match notification is unavailable.
