=====
Sleep
=====

NuttX provides three different types of sleep interfaces. 

Common Sleep Interfaces
=======================

Scheduled Sleep Interfaces (tick-based)
---------------------------------------

Suspend the calling thread for a specified amount of time until the time expires
or the thread is explicitly woken up through scheduler operations.

.. c:function:: void nxsched_ticksleep(unsigned int ticks)
    
    Suspends the calling thread from execution for the specified number of system ticks.

    :param ticks: The number of system ticks to sleep.

.. c:function:: void nxsched_usleep(useconds_t usec)

    Suspends the calling thread from execution for the specified number of microseconds.

    :param usec: The number of microseconds to sleep.

.. c:function:: void nxsched_msleep(unsigned int msec)

    Suspends the calling thread from execution for the specified number of milliseconds.

    :param msec: The number of milliseconds to sleep.

.. c:function:: void nxsched_sleep(unsigned int sec)

    Suspends the calling thread from execution for the specified number of seconds.

    :param sec: The number of seconds to sleep.

.. c:function:: int nxsched_nanosleep(const struct timespec *rqtp, struct timespec *rmtp)

    Suspends the calling thread from execution for the specified rqtp argument. This
    function will return the remaining time via updating rmtp if the sleep is interrupted
    by a signal.

    :param rqtp: The amount of time to be suspended from execution.
    :param rmtp: If the rmtp argument is non-NULL, the timespec structure referenced
     by it is updated to contain the amount of time remaining.
    
    :return: 0 (OK), or negated errno if unsuccessful.

.. c:function:: void nxsched_wakeup(struct tcb_s *tcb)

    This function is used to wake up a thread that is currently in sleeping state
    before its timeout expires.

    :param tcb: Pointer to the TCB of the task to be awakened.

Signal-based Sleep Interfaces (timespec-based)
----------------------------------------------

Suspend the calling thread for a specified amount of time until the
time expires or a signal is delivered to the calling thread. 

    .. note::
        Implementations are dependent on the signal framework and based on standard
        timespec conversion.

.. c:function:: void nxsig_usleep(useconds_t usec)
    
    Suspends the calling thread from execution for the specified number of microseconds.

    :param usec: The number of microseconds to sleep.

.. c:function:: void nxsig_sleep(unsigned int sec)

    Suspends the calling thread from execution for the specified number of seconds.

    :param sec: The number of seconds to sleep.

.. c:function:: int nxsig_nanosleep(const struct timespec *rqtp, struct timespec *rmtp)

    Suspends the calling thread from execution for the specified rqtp argument. This
    function will return the remaining time via updating rmtp if the sleep is interrupted
    by a signal.

    :param rqtp: The amount of time to be suspended from execution.
    :param rmtp: If the rmtp argument is non-NULL, the timespec structure referenced
     by it is updated to contain the amount of time remaining.
    
    :return: 0 (OK), or negated errno if unsuccessful.

Busy Sleep Interfaces
---------------------

Spin in a loop for the requested duration and never yield the CPU. The delay accuracy depends on
``CONFIG_BOARD_LOOPSPERMSEC``.

.. c:function:: void up_mdelay(unsigned int milliseconds)

    Delay inline for the requested number of milliseconds.

    :param milliseconds: The number of milliseconds to delay.

.. c:function:: void up_udelay(useconds_t microseconds)

    Delay inline for the requested number of microseconds.

    :param microseconds: The number of microseconds to delay.

.. c:function:: void up_ndelay(unsigned long nanoseconds)

    Delay inline for the requested number of nanoseconds.

    :param nanoseconds: The number of nanoseconds to delay.
