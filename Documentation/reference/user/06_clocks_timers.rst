=================
Clocks and Timers
=================

- :c:func:`clock_settime`
- :c:func:`clock_gettime`
- :c:func:`clock_getres`
- :c:func:`mktime`
- :c:func:`gmtime`
- :c:func:`localtime`
- :c:func:`asctime`
- :c:func:`ctime`
- :c:func:`gmtime_r`
- :c:func:`localtime_r`
- :c:func:`asctime_r`
- :c:func:`ctime_r`
- :c:func:`timer_create`
- :c:func:`timer_delete`
- :c:func:`timer_settime`
- :c:func:`timer_gettime`
- :c:func:`timer_getoverrun`
- :c:func:`gettimeofday`
- :c:func:`gethrtime`

.. c:function:: int clock_settime(clockid_t clockid, const struct timespec *tp)

  :return: If successful, returns zero (``OK``). Otherwise,
    a non-zero error number will be returned to indicate the error.

.. c:function:: int clock_gettime(clockid_t clockid, struct timespec *tp)

  :return: If successful, returns zero (``OK``). Otherwise,
    a non-zero error number will be returned to indicate the error.

.. c:function:: int clock_getres(clockid_t clockid, struct timespec *res)

  :return: If successful, returns zero (``OK``). Otherwise,
    a non-zero error number will be returned to indicate the error.

.. c:function:: time_t mktime(struct tm *tp);

  :return: If successful, returns zero (``OK``). Otherwise,
    a non-zero error number will be returned to indicate the error.

.. c:function:: FAR struct tm *gmtime(FAR const time_t *timep);

  Represents GMT date/time in a type ``struct tm``. This
  function is not re-entrant.

  :param timep: Represents GMT calendar time. This is an absolute time
    value representing the number of seconds elapsed since 00:00:00 on
    January 1, 1970, Coordinated Universal Time (UTC).

  :return: If successful, the function will return the pointer to a statically defined
    instance of ``struct tm``. Otherwise, a NULL will be returned to
    indicate the error:

.. c:function:: FAR struct tm *localtime(FAR const time_t *timep)

  Represents local date/time in a type ``struct tm``.
  This function is not re-entrant.

  :param timep: Represents GMT calendar time. This is an absolute time
    value representing the number of seconds elapsed since 00:00:00 on
    January 1, 1970, Coordinated Universal Time (UTC).

  :return: If successful, the function will return the pointer to a statically defined
    instance of ``struct tm``. Otherwise, a NULL will be returned to
    indicate the error:

.. c:function:: FAR char *asctime(FAR const struct tm *tp);

  Converts the time provided in a
  ``struct tm`` to a string representation. ``asctime()`` is not
  re-entrant.

  :param tp: Pointer to the time to be converted.
  :return: If successful, the function will
    return a pointer to a statically defined string holding the converted
    time. Otherwise, a NULL will be returned to indicate the error.

.. c:function:: FAR char *ctime(FAR const time_t *timep)

  Converts the time provided in seconds since
  the epoch to a string representation. ``ctime()`` is not re-entrant.

  :param timep: The current time represented as seconds since the epoch.
  :return: If successful, the function will return
    the pointer to the converted string. Otherwise, a NULL will be returned
    to indicate the error.

.. c:function:: struct tm *gmtime_r(const time_t *timep, struct tm *result);

  Represents GMT date/time in a type ``struct tm``. This
  function is re-entrant.

  :param timep: Represents GMT calendar time. This is an absolute time
    value representing the number of seconds elapsed since 00:00:00 on
    January 1, 1970, Coordinated Universal Time (UTC).
  :param result: A user-provided buffer to receive the converted time
    structure.
  :return: If successful, the ``gmtime_r()`` function will
    return the pointer, ``result``, provided by the caller. Otherwise, a
    NULL will be returned to indicate the error:

.. c:function:: FAR struct tm *localtime_r(FAR const time_t *timep, FAR struct tm *result)

  Represents local date/time in a type ``struct tm``.
  This function is re-entrant.

  :param timep: Represents GMT calendar time. This is an absolute time
    value representing the number of seconds elapsed since 00:00:00 on
    January 1, 1970, Coordinated Universal Time (UTC).
  :param result: A user-provided buffer to receive the converted time
    structure.
  :return: If successful, the
    ``localtime_r()`` function will return the pointer, ``result``, provided
    by the caller. Otherwise, a NULL will be returned to indicate the error:

.. c:function:: FAR char *asctime_r(FAR const struct tm *tp, FAR char *buf)

  Converts the time provided in a
  ``struct tm`` to a string representation. ``asctime-r()`` is re-entrant.

  :param tp: Pointer to the time to be converted.
  :param buf: The user provider buffer. of size >= 26 characters, to
    receive the converted time.
  :return: If successful, the ``asctime_r()`` function will
    return the pointer, ``buf``, provided by the caller. Otherwise, a NULL
    will be returned to indicate the error.

.. c:function:: FAR char *ctime_r(FAR const time_t *timep, FAR char *buf)

  Converts the time provided in seconds
  since the epoch to a string representation. ``ctime()`` is re-entrant.

  :param timep: The current time represented as seconds since the epoch.
  :param buf: The user provider buffer. of size >= 26 characters, to
    receive the converted time.
  :return: If successful, the ``ctime_r()`` function will
    return the pointer, ``buf``, provided by the caller. Otherwise, a NULL
    will be returned to indicate the error.

.. c:function:: int timer_create(clockid_t clockid, struct sigevent *evp, timer_t *timerid);

  Creates per-thread
  timer using the specified clock, ``clock_id``, as the timing base. The
  ``timer_create()`` function returns, in the location referenced by
  ``timerid``, a timer ID of type timer_t used to identify the timer in
  timer requests. This timer ID is unique until the timer is deleted. The
  particular clock, ``clock_id``, is defined in ``<time.h>``. The timer
  whose ID is returned will be in a disarmed state upon return from
  ``timer_create()``.

  The ``evp`` argument, if non-NULL, points to a ``sigevent`` structure.
  This structure is allocated by the called and defines the asynchronous
  notification to occur. If the ``evp`` argument is NULL, the effect is as
  if the ``evp`` argument pointed to a ``sigevent`` structure with the
  ``sigev_notify`` member having the value ``SIGEV_SIGNAL``, the
  ``sigev_signo`` having a default signal number, and the ``sigev_value``
  member having the value of the timer ID.

  Each implementation defines a set of clocks that can be used as timing
  bases for per-thread timers. All implementations will support a
  ``clock_id`` of ``CLOCK_REALTIME``.

  :param clockid: Specifies the clock to use as the timing base. Must be
    ``CLOCK_REALTIME``.
  :param ``evp``: Refers to a user allocated sigevent structure that defines
    the asynchronous notification. evp may be NULL (see above).
  :param ``timerid``: The pre-thread timer created by the call to
    timer_create().
  :return: If the call succeeds, ``timer_create()`` will return
    0 (``OK``) and update the location referenced by ``timerid`` to a
    ``timer_t``, which can be passed to the other per-thread timer calls. If
    an error occurs, the function will return a value of -1 (``ERROR``) and
    set ``errno`` to indicate the error.

    -  ``EAGAIN``. The system lacks sufficient signal queuing resources to
       honor the request.
    -  ``EAGAIN``. The calling process has already created all of the timers
       it is allowed by this implementation.
    -  ``EINVAL``. The specified clock ID is not defined.
    -  ``ENOTSUP``. The implementation does not support the creation of a
       timer attached to the CPU-time clock that is specified by clock_id
       and associated with a thread different thread invoking
       timer_create().

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the full POSIX implementation include:

  -  Only ``CLOCK_REALTIME`` is supported for the ``clockid`` argument.

.. c:function:: int timer_delete(timer_t timerid);

  Deletes the specified
  timer, ``timerid``, previously created by the ``timer_create()``
  function. If the timer is armed when ``timer_delete()`` is called, the
  timer will be automatically disarmed before removal. The disposition of
  pending signals for the deleted timer is unspecified.

  :param timerid: The pre-thread timer, previously created by the call to
    timer_create(), to be deleted.
  :return: If successful, the ``timer_delete()`` function will
    return zero (``OK``). Otherwise, the function will return a value of -1
    (``ERROR``) and set ``errno`` to indicate the error:

    -  ``EINVAL``. The timer specified timerid is not valid.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int timer_settime(timer_t timerid, int flags, const struct itimerspec *value, \
                         struct itimerspec *ovalue);

  Sets the time until
  the next expiration of the timer specified by ``timerid`` from the
  ``it_value`` member of the value argument and arm the timer if the
  ``it_value`` member of value is non-zero. If the specified timer was
  already armed when ``timer_settime()`` is called, this call will reset
  the time until next expiration to the value specified. If the
  ``it_value`` member of value is zero, the timer will be disarmed. The
  effect of disarming or resetting a timer with pending expiration
  notifications is unspecified.

  If the flag ``TIMER_ABSTIME`` is not set in the argument flags,
  ``timer_settime()`` will behave as if the time until next expiration is
  set to be equal to the interval specified by the ``it_value`` member of
  value. That is, the timer will expire in ``it_value`` nanoseconds from
  when the call is made. If the flag ``TIMER_ABSTIME`` is set in the
  argument flags, ``timer_settime()`` will behave as if the time until
  next expiration is set to be equal to the difference between the
  absolute time specified by the ``it_value`` member of value and the
  current value of the clock associated with ``timerid``. That is, the
  timer will expire when the clock reaches the value specified by the
  ``it_value`` member of value. If the specified time has already passed,
  the function will succeed and the expiration notification will be made.

  The reload value of the timer will be set to the value specified by the
  ``it_interval`` member of value. When a timer is armed with a non-zero
  ``it_interval``, a periodic (or repetitive) timer is specified.

  Time values that are between two consecutive non-negative integer
  multiples of the resolution of the specified timer will be rounded up to
  the larger multiple of the resolution. Quantization error will not cause
  the timer to expire earlier than the rounded time value.

  If the argument ``ovalue`` is not NULL, the t\ ``imer_settime()``
  function will store, in the location referenced by ``ovalue``, a value
  representing the previous amount of time before the timer would have
  expired, or zero if the timer was disarmed, together with the previous
  timer reload value. Timers will not expire before their scheduled time.

  **NOTE:**\ At present, the ``ovalue`` argument is ignored.

  :param timerid: The pre-thread timer, previously created by the call to
    timer_create(), to be be set.
  :param flags: Specify characteristics of the timer (see above)
  :param value: Specifies the timer value to set
  :param ovalue: A location in which to return the time remaining from the
    previous timer setting (ignored).

  :return: If the timer_gettime() succeeds, a value of 0
    (``OK``) will be returned. If an error occurs, the value -1 (``ERROR``)
    will be returned, and ```errno`` <#ErrnoAccess>`__ set to indicate the
    error.

    -  ``EINVAL``. The timerid argument does not correspond to an ID
       returned by timer_create() but not yet deleted by timer_delete().
    -  ``EINVAL``. A value structure specified a nanosecond value less than
       zero or greater than or equal to 1000 million, and the it_value
       member of that structure did not specify zero seconds and
       nanoseconds.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the full POSIX implementation include:

  -  The ``ovalue`` argument is ignored.

.. c:function:: int timer_gettime(timer_t timerid, struct itimerspec *value);

  Stores the amount
  of time until the specified timer, ``timerid``, expires and the reload
  value of the timer into the space pointed to by the ``value`` argument.
  The ``it_value`` member of this structure will contain the amount of
  time before the timer expires, or zero if the timer is disarmed. This
  value is returned as the interval until timer expiration, even if the
  timer was armed with absolute time. The ``it_interval`` member of
  ``value`` will contain the reload value last set by ``timer_settime()``.

  Due to the asynchronous operation of this function, the time reported by
  this function could be significantly more than that actual time
  remaining on the timer at any time.

  :param timerid: Specifies pre-thread timer, previously created by the
    call to ``timer_create()``, whose remaining count will be returned.
  :return: If successful, the ``timer_gettime()`` function will
    return zero (``OK``). Otherwise, an non-zero error number will be
    returned to indicate the error:

    -  ``EINVAL``. The ``timerid`` argument does not correspond to an ID
       returned by ``timer_create()`` but not yet deleted by
       ``timer_delete()``.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int timer_getoverrun(timer_t timerid);

  Only a single signal will be queued to the process for
  a given timer at any point in time. When a timer for which a signal is
  still pending expires, no signal will be queued, and a timer overrun
  will occur. When a timer expiration signal is delivered to or accepted
  by a process, if the implementation supports the *Realtime Signals
  Extension*, the ``timer_getoverrun()`` function will return the timer
  expiration overrun count for the specified timer. The overrun count
  returned contains the number of extra timer expirations that occurred
  between the time the signal was generated (queued) and when it was
  delivered or accepted, up to but not including an implementation-defined
  maximum of ``DELAYTIMER_MAX``. If the number of such extra expirations
  is greater than or equal to ``DELAYTIMER_MAX``, then the overrun count
  will be set to ``DELAYTIMER_MAX``. The value returned by
  ``timer_getoverrun()`` will apply to the most recent expiration signal
  delivery or acceptance for the timer. If no expiration signal has been
  delivered for the timer, or if the *Realtime Signals Extension* is not
  supported, the return value of ``timer_getoverrun()`` is unspecified.

  **NOTE:** This interface is not currently implemented in NuttX.

  :param timerid: Specifies pre-thread timer, previously created by the
    call to ``timer_create()``, whose overrun count will be returned.

  :return: If the ``timer_getoverrun()`` function succeeds, it
    will return the timer expiration overrun count as explained above.
    ``timer_getoverrun()`` will fail if:

    -  ``EINVAL``. The ``timerid`` argument does not correspond to an ID
       returned by ``timer_create()`` but not yet deleted by
       ``timer_delete()``.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the full POSIX implementation include:

  -  This interface is not currently implemented by NuttX.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int gettimeofday(struct timeval *tp, void *tzp);

  This implementation of ``gettimeofday()`` is simply a
  thin wrapper around :c:func:`clock_gettime`. It simply
  calls ``clock_gettime()`` using the ``CLOCK_REALTIME`` timer and
  converts the result to the required ``struct timeval``.

  :param tp: The current time will be returned to this user provided
    location.
  :param tzp: A reference to the timezone -- *IGNORED*.

  :return: See :c:func:`clock_gettime`.

.. c:function:: hrtime_t gethrtime(void);

  This implementation of ``gethrtime()`` is simply a
  thin wrapper around :c:func:`clock_gettime`. It simply
  calls ``clock_gettime()`` using the ``CLOCK_REALTIME`` or ``CLOCK_MONOTONIC``,
  and converts the result to the required hrtime_t.

  :return: current system time in ns
