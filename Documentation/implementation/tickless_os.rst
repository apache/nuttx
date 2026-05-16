.. _tickless:

===========
Tickless OS
===========
 
Default System Timer
====================

By default, a NuttX configuration includes a periodic timer interrupt
that drives all system timing.
The timer is provided by architecture-specific code that calls into NuttX
at a rate controlled by ``CONFIG_USEC_PER_TICK``.
The default value of ``CONFIG_USEC_PER_TICK`` is 10000 microseconds which
corresponds to a timer interrupt rate of 100Hz.

On each timer interrupt, NuttX does these things:

* Increments a counter. This counter is the system time and has a resolution
  of ``CONFIG_USEC_PER_TICK`` microseconds.
* Checks if it is time to perform time-slice operations on tasks that
  have select round-robin scheduling.
* Checks for expiration of timed events.

What is wrong with this default system timer? Nothing really.
It is reliable and uses only a small fraction of the CPU band width.
But we can do better. Some limitations of default system timer are,
in increasing order of importance:

1. **Overhead:** Although the CPU usage of the system timer interrupt
   at 100Hz is really very low, it is still mostly wasted processing time.
   On most timer interrupts, there is really nothing that needs be done
   other than incrementing the counter.
2. **Resolution:** Resolution of all system timing is also determined
   by ``CONFIG_USEC_PER_TICK``. So nothing can be timed with resolution
   finer than 10 milliseconds by default. If you want to increase this
   resolution, then you can reduce ``CONFIG_USEC_PER_TICK``.
   However, then the system timer interrupts use more of the CPU bandwidth
   processing useless interrupts.
3. **Power Usage:** But the biggest issue is power usage.
   When the system is IDLE, it enters a light, low-power mode (for ARMs,
   this mode is entered with the ``wfi`` instruction for example).
   But each interrupt awakens the system from this low power mode.
   Therefore, higher rates of interrupts cause greater power consumption.


Tickless OS Advantages
======================

The so-called **Tickless OS** provides one solution to issue.
The basic concept here is that the periodic, timer interrupt is eliminated
and replaced with a one-shot, interval timer.
It becomes event driven instead of polled: The default system timer
is a polled design. On each interrupt, the NuttX logic checks if it needs
to do anything and, if so, it does it.

Using an interval timer, you can anticipate when the next interesting
OS event will occur, program the interval time and wait for it to fire.
When the interval time fires, then the scheduled activity is performed.


Platform Support
================

In order to use the Tickless OS, you must provide special support
from the platform-specific code. Just as with the default system timer,
the platform-specific code must provide the timer resources to support
the OS behavior.

Currently these timer resources are only provided on a few platforms.
You can see that implementation for the simulation in
``nuttx/arch/sim/src/up_tickless.c``. There is another implementation 
for the Atmel SAMA5 at ``nuttx/arch/arm/src/sama5/sam_tickless.c``.
This Wiki will explain how you can provide the Tickless OS support
for your platform.


Configuration Options
=====================

* ``CONFIG_ARCH_HAVE_TICKLESS``: If your platform provides support
  for the Tickless OS, then you should set this option in the ``Kconfig``
  file for your board. Here is what the selection looks in the
  ``arch/Kconfig`` file for the simulated platform::

    config ARCH_SIM
      bool "Simulation"
      select ARCH_HAVE_TICKLESS
      ---help---
              Linux/Cywgin user-mode simulation.

* ``CONFIG_SCHED_TICKLESS``: If ``CONFIG_ARCH_HAVE_TICKLESS`` is selected,
  then you will be able to use this option to enable the Tickless OS features
  in NuttX.
* ``CONFIG_SCHED_TICKLESS_ALARM``: The tickless option can be supported either
  via a simple interval timer (plus elapsed time) or via an alarm.
  The interval timer allows programming events to occur after an interval.
  With the alarm, you can set a time in the future and get an event when
  that alarm goes off. This option selects the use of an alarm.
  The advantage of an alarm is that it avoids some small timing errors;
  the advantage of the use of the interval timer is that the hardware
  requirement may be less.
* ``CONFIG_USEC_PER_TICK``: This is not a new option, but changes
  its relevance when the Tickless OS is selected.

In the default configuration where system time is provided by a periodic timer
interrupt, the default system timer is configure the timer for 100Hz or
``CONFIG_USEC_PER_TICK=10000``. If ``CONFIG_SCHED_TICKLESS`` is selected,
then there are no system timer interrupt. In this case,
``CONFIG_USEC_PER_TICK`` does not control any timer rates.
Rather, it only determines the resolution of time reported by
``clock_systimer()`` and the resolution of times that can be set for certain
delays including watchdog timers and delayed work.

In this case there is still a trade-off:
It is better to have the ``CONFIG_USEC_PER_TICK`` as low as possible
for higher timing resolution. However, the the time is currently held
in ``unsigned int``. On some systems, this may be ``16-bit`` in width
but on most contemporary systems it will be ``32-bit``.
In either case, smaller values of ``CONFIG_USEC_PER_TICK`` will reduce
the range of values that delays that can be represented.
So the trade-off is between range and resolution (you could also modify
the code to use a 64-bit value if you really want both).

.. note:: Recently NuttX switched to ``int64_t`` for time stamps.

The default, 100 microseconds, will provide for a range of delays
up to 120 hours.

This value should never be less than the underlying resolution of the timer.
Error may ensue.


System Interfaces
=================

Implementation Notes
--------------------

Notice that with the ``CONFIG_SCHED_TICKLESS`` option, an implementation
might require two hardware timers:
(1) An interval timer to satisfy the requirements for ``up_timer_start()``
and ``up_timer_cancel()``, and
(2) a counter to handle the requirement of ``up_timer_gettime()``.

Since timers are a limited resource, the use of two timers could be an issue
on some systems. The job could be done with a single timer if, for example,
the single timer were kept in a free-running at all times.
Some timer/counters have the capability to generate a compare interrupt when
the timer matches a comparison value but also to continue counting without
stopping. If your hardware supports such counters, one might used the
``CONFIG_SCHED_TICKLESS_ALARM`` option and be able to simply set the
comparison count at the value of the free running timer PLUS the desired
delay. Then you could have both with a single timer:
An alarm and a free-running counter with the same timer!

Imported Intefaces
------------------

The interfaces that must be provided by the platform specified code
are defined in ``include/nuttx/arch.h`` and summarized below.

``up_timer_intialize()``
^^^^^^^^^^^^^^^^^^^^^^^^

.. code:: c

   void up_timer_initialize(void);

* **Description:** Initializes all platform-specific timer facilities.
  This function is called early in the initialization sequence by
  ``up_intialize()``. On return, the current up-time should be available
  from ``up_timer_gettime()`` and the interval timer is ready for use
  (but not actively timing).
* **Input Parameters:** None.
* **Returned Value:** None.
* **Assumptions:** Called early in the initialization sequence before
  any special concurrency protections are required.

``up_timer_gettime()``
^^^^^^^^^^^^^^^^^^^^^^

.. code:: c

   int up_timer_gettime(FAR struct timespec *ts);

* **Description:** Return the elapsed time since power-up (or, more correctly,
  since ``up_timer_initialize()`` was called). This function is functionally
  equivalent to ``clock_gettime()`` for the clock ID ``CLOCK_MONOTONIC``.
  This function provides the basis for reporting the current time and also
  is used to eliminate error build-up from small errors
  in interval time calculations.
* **Input Parameters:** ``*ts`` - Provides the location in which
  to return the up-time.
* **Returned Value:** Zero (``OK``) is returned on success;
  a negated ``errno`` value is returned on any failure.
* **Assumptions:** Called from the the normal tasking context. The implementation must provide whatever mutual exclusion is necessary for correct operation. This can include disabling interrupts in order to assure atomic register operations.

``up_alarm_cancel()``
^^^^^^^^^^^^^^^^^^^^^

.. code:: c

   int up_alarm_cancel(FAR struct timespec *ts);

* **Description:** Cancel the alarm and return the time of cancellation
  of the alarm. These two steps need to be as nearly atomic as possible.
  ``sched_timer_expiration()`` will not be called unless the alarm
  is restarted with ``up_alarm_start()``. If, as a race condition,
  the alarm has already expired when this function is called,
  then time returned is the current time.
* **Input Parameters:** ``*ts`` - Location to return the expiration time.
  The current time should be returned if the timer is not active.
  ``ts`` may be ``NULL`` in which case the time is not returned.
* **Returned Value:** Zero (``OK``) is returned on success;
  a negated ``errno`` value is returned on any failure.
* **Assumptions:** May be called from interrupt level handling or from the
  normal tasking level. Interrupts may need to be disabled internally
  to assure non-reentrancy.

.. note:: This function is only required when ``CONFIG_SCHED_TICKLESS_ALARM``
          is defined.

``up_alarm_start()``
^^^^^^^^^^^^^^^^^^^^

.. code:: c

   int up_alarm_start(FAR const struct timespec *ts);

* **Description:** Start the alarm. ``sched_timer_expiration()`` will be
  called at alarm expires (unless ``up_alarm_cancel()`` is called to stop
  the alarm.
* **Input Parameters:** ``*ts`` - The time in the future at the alarm
  is expected to occur. When the alarm occurs the timer logic will call
  ``ched_timer_expiration()`` is called.
* **Returned Value:** Zero (``OK``) is returned on success;
  a negated ``errno`` value is returned on any failure.
* **Assumptions:** May be called from interrupt level handling
  or from the normal tasking level. Interrupts may need to be disabled
  internally to assure non-reentrancy.

.. note:: This function is only required when ``CONFIG_SCHED_TICKLESS_ALARM``
          is defined.

``up_timer_cancel()``
^^^^^^^^^^^^^^^^^^^^^

.. code:: c

   int up_timer_cancel(FAR struct timespec *ts);

* **Description:** Cancel the interval timer and return the time remaining
  on the timer. These two steps need to be as nearly atomic as possible.
  ``sched_timer_expiration()`` will not be called unless the timer
  is restarted with ``up_timer_start()``. If, as a race condition,
  the timer has already expired when this function is called,
  then that pending interrupt must be cleared so that
  ``up_timer_start()`` and the remaining time of zero should be returned.
* **Input Parameters:** ``*ts`` - Location to return the remaining time.
  Zero should be returned if the timer is not active. ``ts`` may be ``NULL``
  in which case the time remainging is not returned
* **Returned Value:** Zero (``OK``) is returned on success;
  a negated ``errno`` value is returned on any failure.
* **Assumptions:** May be called from interrupt level handling
  or from the normal tasking level. Interrupts may need to be disabled
  internally to assure non-reentrancy.

.. note:: This function is only required when ``CONFIG_SCHED_TICKLESS_ALARM``
          is not defined.

``up_timer_start()``
^^^^^^^^^^^^^^^^^^^^

.. code:: c

   int up_timer_start(FAR const struct timespec *ts);

* **Description:** Start the interval timer. ``sched_timer_expiration()``
  will be called at the completion of the timeout (unless
  ``up_timer_cancel()`` is called to stop the timing.
* **Input Parameters:** ``*ts`` - Provides the time interval until
  ``sched_timer_expiration()`` is called.
* **Returned Value:** Zero (``OK``) is returned on success;
  a negated ``errno`` value is returned on any failure.
* **Assumptions:** May be called from interrupt level handling or from the
  normal tasking level. Interrupts may need to be disabled internally
  to assure non-reentrancy.

.. note:: This function is only required when ``CONFIG_SCHED_TICKLESS_ALARM``
          is not defined.


Exported Interfaces
-------------------

In addition, the following interface is provided by the RTOS for use
by the platform specific code:

``sched_alarm_expiration()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code:: c

   void sched_timer_expiration(FAR const timespec *tc);

* **Description:** if ``CONFIG_SCHED_TICKLESS`` is defined, then this
  function is provided by the RTOS base code and called from
  platform-specific code when the alaram used to implement
  the tick-less OS expires.
* **Input Parameters:** ``*ts`` - The time when the alarm expired.
* **Returned Value:** None.
* **Assumptions:** Base code implementation assumes that this function
  is called from interrupt handling logic with interrupts disabled.

.. note:: This function is only provided when ``CONFIG_SCHED_TICKLESS_ALARM``
          is defined.

``sched_timer_expiration()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code:: c

   void sched_timer_expiration(void);

* **Description:** if ``CONFIG_SCHED_TICKLESS`` is defined, then this
  function is provided by the RTOS base code and called from
  platform-specific code when the interval timer used to implement
  the tick-less OS expires.
* **Input Parameters:** None.
* **Returned Value:** None.
* **Assumptions:** Base code implementation assumes that this function
  is called from interrupt handling logic with interrupts disabled.

.. note:: This function is only provided when ``CONFIG_SCHED_TICKLESS_ALARM``
          is not defined.


Issues
======

Let me describe an experience to illustrate some of the problems
that you may run into:

I implemented the tickless mode on an MCU with ``16-bit`` timers.
Most of the clock sources were too fast for use with the 16-bit timer,
so I tried using a 32.768 kHz clock source.
This resulted in a time resolution of about 30.518 microseconds.

That time, 30.518 microseconds, can not be represented accurately with
``CONFIG_USEC_PER_TICK`` so I used the value 31 which is in error
by about 0.6%.

What I found on a busy system is that the delay I get when executing
this NSH command was actually about 5.5 seconds::

  nsh> sleep 10

This was very puzzling to me and cost me a lot of debug time.
Finally, I disabled all competing usage of the interval timer by disabling
features (the high priority work queue, by the way, is the most significant
user of the interval timer).
After disabling all competing usage, the NSH command sleep 10 resulted
in a delay of about 10.3 seconds. Still not accurate but not so bad.

I concluded that the gross inaccuracy in the first case was due to
the inaccuracies in the representation of the clock rate.
30.518 usec cannot be represented accurately.
Each timing calculation results in a small error.
When the interval timer is very busy, long delays will be divided
into many small pieces and each small piece has a large error
in the calculation. The cumulative error is the cause of the problem.

.. caution:: The moral of this story: Do not use timer sources
             that cannot be represented by ``CONFIG_USEC_PER_TICK``.
