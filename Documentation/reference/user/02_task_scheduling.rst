==========================
Task Scheduling Interfaces
==========================

By default, NuttX performs strict priority scheduling: Tasks of higher
priority have exclusive access to the CPU until they become blocked. At
that time, the CPU is available to tasks of lower priority. Tasks of
equal priority are scheduled FIFO.

Optionally, a NuttX task or thread can be configured with round-robin or
*sporadic* scheduler. The round-robin is similar to priority scheduling
*except* that tasks with equal priority and share CPU time via
*time-slicing*. The time-slice interval is a constant determined by the
configuration setting ``CONFIG_RR_INTERVAL`` to a positive, non-zero
value. Sporadic scheduling scheduling is more complex, varying the
priority of a thread over a *replenishment* period. Support for sporadic
scheduling is enabled by the configuration option
``CONFIG_SCHED_SPORADIC``.

The OS interfaces described in the following paragraphs provide a POSIX-
compliant interface to the NuttX scheduler:

  - :c:func:`sched_setparam`
  - :c:func:`sched_getparam`
  - :c:func:`sched_setscheduler`
  - :c:func:`sched_getscheduler`
  - :c:func:`sched_yield`
  - :c:func:`sched_get_priority_max`
  - :c:func:`sched_get_priority_min`
  - :c:func:`sched_get_rr_interval`

Functions
=========

.. c:function:: int sched_setparam(pid_t pid, FAR const struct sched_param *param)

  This function sets the priority of the task specified
  by pid input parameter.

  :param pid: The task ID of the task. If ``pid`` is zero, the priority of
     the calling task is set.
  :param param: A structure whose member ``sched_priority`` is the integer
     priority. The range of valid priority numbers is from
     ``SCHED_PRIORITY_MIN`` through ``SCHED_PRIORITY_MAX``.

  :return: On success, sched_setparam() returns 0 (``OK``). On
    error, -1 (``ERROR``) is returned, and ```errno`` <#ErrnoAccess>`__ is
    set appropriately.

    -  ``EINVAL``. The parameter ``param`` is invalid or does not make sense
       for the current scheduling policy.
    -  ``EPERM``. The calling task does not have appropriate privileges.
    -  ``ESRCH``. The task whose ID is ``pid`` could not be found.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the full POSIX implementation include:

    -  The range of priority values for the POSIX call is 0 to 255. The
       priority 0 is the lowest priority and 255 is the highest priority.

  .. note:: Setting a task's priority to the same value has the similar effect
    to ``sched_yield()``: The task will be moved to after all other tasks
    with the same priority.

.. c:function:: int sched_getparam(pid_t pid, FAR struct sched_param *param)

  This function gets the scheduling priority of the task
  specified by pid.

  :param pid: The task ID of the task. If pid is zero, the priority of the
     calling task is returned.
  :param param: A structure whose member ``sched_priority`` is the integer
     priority. The task's priority is copied to the ``sched_priority``
     element of this structure.

  :return: 0 (``OK``) if successful, otherwise -1 (``ERROR``).

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sched_setscheduler (pid_t pid, int policy, const struct sched_param *param)

  ``sched_setscheduler()`` sets both the scheduling
  policy and the priority for the task identified by ``pid``. If ``pid``
  equals zero, the scheduler of the calling thread will be set. The
  parameter ``param`` holds the priority of the thread under the new
  policy.

  :param pid: The task ID of the task. If ``pid`` is zero, the priority of
     the calling task is set.
  :param policy: Scheduling policy requested (either ``SCHED_FIFO`` or
     ``SCHED_RR``).
  :param param: A structure whose member ``sched_priority`` is the integer
     priority. The range of valid priority numbers is from
     ``SCHED_PRIORITY_MIN`` through ``SCHED_PRIORITY_MAX``.

  :return: On success, ``sched_setscheduler()`` returns ``OK``
    (zero). On error, ``ERROR`` (-1) is returned, and
    ``errno`` is set appropriately:

    -  ``EINVAL``: The scheduling ``policy`` is not one of the recognized
       policies.
    -  ``ESRCH``: The task whose ID is ``pid`` could not be found.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sched_getscheduler (pid_t pid)

  ``sched_getscheduler()`` returns the scheduling policy
  currently applied to the task identified by ``pid``. If ``pid`` equals
  zero, the policy of the calling process will be retrieved.

  :param pid: The task ID of the task to query. If ``pid`` is zero, the
     calling task is queried.

  :return: On success, ``sched_getscheduler()`` returns the policy for the task
    (either ``SCHED_FIFO`` or ``SCHED_RR``). On error, ``ERROR`` (-1) is
    returned, and ``errno`` is set appropriately:

    -  ``ESRCH``: The task whose ID is pid could not be found.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sched_yield(void)

  This function forces the calling task to give up the
  CPU (only to other tasks at the same priority).

  :return: 0 (``OK``) or -1 (``ERROR``)

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sched_get_priority_max (int policy)

  This function returns the value of the highest possible
  task priority for a specified scheduling policy.

  :param policy: Scheduling policy requested.

  :return: The maximum priority value or -1 (``ERROR``).

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sched_get_priority_min (int policy)

  This function returns the value of the lowest possible
  task priority for a specified scheduling policy.

  :param policy: Scheduling policy requested.

  :return: The minimum priority value or -1 (``ERROR``)

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sched_get_rr_interval (pid_t pid, struct timespec *interval)

  ``sched_rr_get_interval()`` writes the timeslice
  interval for task identified by ``pid`` into the timespec structure
  pointed to by ``interval``. If pid is zero, the timeslice for the
  calling process is written into 'interval. The identified process should
  be running under the SCHED_RR scheduling policy.'

  :param pid: The task ID of the task. If pid is zero, the priority of the
     calling task is returned.
  :param interval: A structure used to return the time slice.

  :return: On success, sched_rr_get_interval() returns OK (0).
    On error, ERROR (-1) is returned, and ``errno`` is
    set to:

    -  ``EFAULT``: Cannot copy to interval
    -  ``EINVAL``: Invalid pid.
    -  ``ENOSYS``: The system call is not yet implemented.
    -  ``ESRCH``: The process whose ID is pid could not be found.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.
