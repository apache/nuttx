=======================================
Oneshot Timers and CPU Load Measurement
=======================================
 
Issues with CPU Load Measurement
================================

There is been support for CPU load measurement for a long time.
Enabled with:

.. code-block:: sh

  CONFIG_SCHED_CPULOAD=y
  CONFIG_SCHED_CPULOAD_TIMECONSTANT=2

The main problem with the existing logic is that it ran synchronously
with the system timer. That turns out to be useless for measurement of the
performance of some tasks.

Many tasks operate synchronously with the timer interrupt, i.e., timed event
is the stimulus for task execution.
Those tasks are always sampled at essentially the same point in the execution
profile leading to nonsense results.

In order to get believable results in all cases you would need to:

* Sample at times are completely random with respect to program behavior, AND
* Sample at a high rate or for a very long time.


External Clock
==============

In order to work around these things, an option to use an external clock
was added:

.. code-block:: sh

  CONFIG_SCHED_CPULOAD_EXTCLK=y
  CONFIG_SCHED_CPULOAD_TICKSPERSEC=77

This should work well if ``CONFIG_SCHED_CPULOAD_TICKSPERSEC`` is prime
and less than ``CONFIG_USEC_PER_TICK``.
However, you were completely on your own one how to implement this
external clock. But no longer.


Using a Oneshot Timer to Drive CPU Load Measurement
===================================================

Recently, a generic, platform-independent one-shot lower half interface
was developed. That interface is described in
``include/nuttx/timers/oneshot.h``. There are implementations of the one shot
timer lower half available for STM32, STM32L4, SAM4CM, SAMA5D3/4,
and SAMV71/SAME70.

And now there is also an OS component that can be initialized and used to
drive the CPU load sampling from a precision, high rate oneshot timer.
The implementation of that feature resides at
``sched/sched/sched_cpuload_oneshot.c`` and is enabled with:

.. code-block:: sh

  CONFIG_ONESHOT=y
  CONFIG_CPULOAD_ONESHOT=y

Perhaps with additional configuration options that are likely required
to enable board-specific oneshot timer lower-half support.
The oneshot timer must still be configured by board specific logic which
must call:

.. code-block:: c

  void sched_oneshot_extclk(FAR struct oneshot_lowerhalf_s *lower);

To start the CPU load measurement.

``sched_oneshot_extclk()`` is prototyped in ``include/nuttx/clock.h``.
There is some example setup code in the NuttX simulation code at
``boards/sim/sim/sim/src/sim_bringup.c``.


Entropy
=======

Another recent addition to NuttX came from David Alessio.
David contributed support for ``/dev/urandom`` with a built-in XorShift128
pseduo-random number generator (PRNG).

We have detached the XorShift128 implementation from the ``/dev/urandom``
implementation and moved it to ``/libc/misc`` so that is it available
for other purposes.

In particular, there is an option to use the XorShift128 PRNG to add entropy
to the CPU load measurement of the oneshot timer.
That feature is enabled with:

.. code-block:: sh

  CONFIG_CPULOAD_ONESHOT_ENTROPY=n

for ``n=1..30`` and disabled with ``CONFIG_CPULOAD_ONESHOT_ENTROPY=0``.
This value represents the number of bits of entropy that will be added
to the oneshot interval delays.

The oneshot timer will be set to the following interval each time
the oneshot timer is restarted is ``CONFIG_CPULOAD_ONESHOT_ENTROPY``:

.. code-block:: sh 

  CPULOAD_ONESHOT_NOMINAL \
  - (CPULOAD_ONESHOT_ENTROPY / 2) \
  + error \
  + nrand(CPULOAD_ONESHOT_ENTROPY)

Where:

* ``CPULOAD_ONESHOT_NOMINAL`` is ``CONFIG_SCHED_CPULOAD_TICKSPERSEC``
  in units of microseconds.
* ``CPULOAD_ONESHOT_ENTROPY`` is ``(1 << CONFIG_CPULOAD_ONESHOT_ENTROPY)``.
* ``error`` is an error value that is retained from interval to interval
  so that although individual intervals are randomized,
  the average will still be equal to ``CONFIG_SCHED_CPULOAD_TICKSPERSEC``.

If ``CONFIG_CPULOAD_ONESHOT_ENTROPY=0``, then the interval delay will
always be equal to ``CPULOAD_ONESHOT_NOMINAL``.
