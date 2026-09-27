================
Time and Timers
================

The system clock, the timers built on it, and what happens when a delay is
shorter than a clock tick.  The code lives in ``sched/clock/``,
``sched/timer/``, ``sched/wdog/`` and ``sched/hrtimer/``.

Three things shape everything on this page.

The first is whether the system runs on a periodic tick or
:doc:`tickless <tickless_os>`.  By default a periodic timer interrupt drives
system time; tickless replaces it with an interval timer programmed for the
next OS event, so the system can stay asleep in between.  It is not a free
choice: ``CONFIG_SCHED_TICKLESS`` depends on ``CONFIG_ARCH_HAVE_TICKLESS``,
and a tickless port has additional interfaces to implement, declared in
``include/nuttx/arch.h``.

The second is ``CONFIG_USEC_PER_TICK``, the length of one tick -- and it
matters in *both* configurations, which is the part that catches people out.
Without tickless it is the interval at which the hardware interrupts the OS,
10 ms by default.  With tickless there are no such interrupts and it controls
no timer rate at all, but it still sets the resolution of the time that
``clock_systime_ticks()`` reports, and of the delays you can ask for from
watchdog timers and delayed work.  Its default simply drops to 100 µs.

That makes the tick a trade-off rather than a dial to turn down.  The count is
held in an ``unsigned int`` -- 32 bits on most targets, 16 on some -- so a
smaller tick buys resolution at the cost of the longest delay that can be
represented: the 100 µs default reaches about 120 hours.  It should also never
be set below the resolution of the underlying timer.

The third is that neither of those bounds you when you need timing finer than
a tick, and this section holds two different answers.  :doc:`Short delays
<short_time_delays>` covers the counter-intuitive things that happen when a
requested delay is near or below one tick -- a discussion that applies under
tickless too, only in different terminology.  The high-resolution timer in
``sched/hrtimer/`` (``CONFIG_HRTIMER``) is the other answer, offering
nanosecond-level precision; it is described under
:doc:`System Time and Clock <time_clock>`.  Its callbacks run in interrupt
context, which is the price.

.. toctree::
   :maxdepth: 1

   time_clock.rst
   tickless_os.rst
   short_time_delays.rst
   oneshot_timers_and_cpu_load.rst
   sleep.rst
