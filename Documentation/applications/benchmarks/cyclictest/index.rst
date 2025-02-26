================================
``Cyclictest`` benchmark utility
================================

Cyclictest is a simple program used to measure the real-time capabilities
of a RTOS. Originally, this program comes from the Linux ``rt-tests``.
However, NuttX features its own cyclictest utility which is heavily inspired
by the original program but does not use some advanced features, while adding
features that are NuttX related.

The creation of the new cyclictest arose from the fact that as of February
2025, POSIX time functions (such as ``clock_gettime`` and ``clock_nanosleep``)
depend on the systemtick (if the system is not compiled in the Tickless mode)
which makes small delays practically impossible. However, if we utilize
a hardware device timer, small periodic delays can be achieved with some ``ioctl``
calls.

The documentation needs to be revisited to see how cyclictest performs
when NuttX is compiled in tickless mode.

Replacement for ``clock_gettime`` and ``clock_nanosleep`` in NuttX
------------------------------------------------------------------

Configuring such device timer is simple: firstly, the timer's timeout is set using
the ``TCIOC_SETTIMEOUT`` ``ioctl`` call. Then the ``TCIOC_NOTIFICATION`` ``ioctl`` call
is performed. Afterwards, the timer can be polled using the ``poll`` function
which returns when the timer timeouts.

The thread latency wakeup can be measured using this timer by calling
``TCIOC_GETSTATUS`` ``ioctl`` call after the ``poll`` function has returned.
The ``ioctl`` call fills the ``timer_status_s`` struct which contains two important
fields: ``uint32_t timeleft`` and ``uint32_t timeout``. The latency of the thread can
then be calculated  as ``timeout - timeleft``.

Usage of this program
---------------------

Despite some differences, the NuttX port stays as faithful as possible to the original
program, keeping the most important command-line parameters the same.
The user can choose one of two "waiting methods":

- ``clock_nanosleep`` (``W_NANOSLEEP``),
- polling the device (``W_DEVTIMER``). 

The user can also choose one of two "measuring methods":

- ``clock_gettime`` (``M_GETTIME``),
- utilizing the device timer (``M_TIMER_API``).
  
It is possible to combine the waiting and measuring methods. As of February 2025,
using ``W_DEVTIMER`` and ``M_TIMER_API`` produces the best results.
However, it requires a timer device to be registered by your BSP (such as ``/dev/timer1``).
Be also advised that when ``W_DEVTIMER`` is used, only one thread can poll the timer.

Following command-line parameters can be supplied:

- ``-c --clock [CLOCK]``: 0 selects ``CLOCK_REALTIME``, 1 selects ``CLOCK_MONOTONIC`` (default)
- ``-d --distance [US]``: The distance of thread intervals. Default is 500 us.
- ``-D --duration [TIME]``: Set the test duration in seconds. Default is 0 (endless).
- ``-e --help``: Displays help and exits.
- ``-h --histogram [US]``: Output the histogram data to stdout. US is the maximum value to be printed.
- ``-H --histofall``: Same as ``-h`` except that an additional histogram column is displayed at the right that contains summary data of all thread histograms. If cyclictest runs a single thread only, the ``-H`` option is equivalent to ``-h``.
- ``-i --interval [US]``: The thread interval. Default is 1000 us.
- ``-l --loops [N]``: The number of measurement loops. Default is 0 (endless).
- ``-m --measurement [METHODS]``: Sets the time measurement method. 0 selects ``clock_gettime``, 1 uses the NuttX timer API. Be advised that if 1 is selected, you need to specify a timer device (e.g. ``/dev/timer0``) in ``-T``.
- ``-n --nanosleep [METHOD]``: Sets the waiting method: 0 selects ``clock_nanosleep``, 1 waits for the POLLIN flag on a timer device. Default is 0. Choosing 1 works only with one thread, the ``-t`` value is therefore set to 1. If METHOD 1 is selected, you need to specify a timer device (e.g. ``/dev/timer0``) in ``-T``.
- ``-p --prio``: Sets the priority of the first thread.
- ``-t --threads [N]``: The number of test threads to be created. Default is 1.
- ``-T --timer-device [DEV]``: The measuring timer device. Must be specified when ``-m=1`` or ``-n=1``.
- ``-y --policy [NAME]``: Set the scheduler policy, where NAME is fifo, rr, batch, idle, normal, other.

Example usage
-------------
``cyclictest -p 150 -T /dev/timer1 -m 1 -n 1 -h 20 -D 100 -i 50``

Since ``W_DEVTIMER`` is used, only one thread runs every 50 us.
The measurement method is the device timer itself, specified in ``-T``.
The test runs for 100 seconds. The priority is boosted to 150, so the
measurement is not affected by other tasks or communication.

Output of the command (tested on Microchip ATSAMV71Q21B @ 300 MHz):

.. code-block:: text

  # Histogram
  000000 000000
  000001 000000
  000002 000000
  000003 000000
  000004 000000
  000005 000000
  000006 000000
  000007 000000
  000008 000000
  000009 000000
  000010 603045
  000011 1395782
  000012 000804
  000013 000153
  000014 000034
  000015 000083
  000016 000030
  000017 000000
  000018 000000
  000019 000000
  # Total: 001999931
  # Min Latencies: 00010
  # Avg Latencies: 00010
  # Max Latencies: 00016
  # Histogram Overflows: 00000
