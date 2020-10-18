=======================
Task Control Interfaces
=======================

.. warning:: This section name is duplicate with the first, how should it be named?

- **Scheduler locking interfaces**. These *non-standard* interfaces are
  used to enable and disable pre-emption and to test is pre-emption is
  currently enabled.

    - :c:func:`sched_lock`
    - :c:func:`sched_unlock`
    - :c:func:`sched_lockcount`

- **Task synchronization interfaces** are used to wait for termination of child tasks.

    - :c:func:`waitpid`
    - :c:func:`waitid`
    - :c:func:`wait`

- **Task Exit Hooks** may be used to
  register callback functions that are executed when a *task group*
  terminates. A task group is the functional analog of a process: It is
  a group that consists of the main task thread and of all of the
  pthreads created by the main task thread or any of the other pthreads
  within the task group. Members of a task group share certain
  resources such as environment variables, file descriptors, ``FILE``
  streams, sockets, pthread keys and open message queues.

    - :c:func:`atexit`
    - :c:func:`on_exit`

  .. note::
     Behavior of features related to task group's depend of
     NuttX configuration settings. See the discussion of "Parent and
     Child Tasks," below. See also the\ `NuttX
     Tasking <https://cwiki.apache.org/confluence/display/NUTTX/NuttX+Tasking>`__\ page
     and the\ `Tasks vs. Threads
     FAQ <https://cwiki.apache.org/confluence/display/NUTTX/Tasks+vs.+Threads+FAQ>`__\ for
     additional information on tasks and threads in NuttX.

  A *task group* terminates when the last thread within the group
  exits.

Parent and Child Tasks
======================

The task synchronization interfaces
historically depend upon parent and child relationships between tasks.
But default, NuttX does not use any parent/child knowledge. However,
there are three important configuration options that can change that.

  -  ``CONFIG_SCHED_HAVE_PARENT``: If this setting is defined, then it
     instructs NuttX to remember the task ID of the parent task when each
     new child task is created. This support enables some additional
     features (such as ``SIGCHLD``) and modifies the behavior of other
     interfaces. For example, it makes ``waitpid()`` more standards
     complete by restricting the waited-for tasks to the children of the
     caller.

  -  ``CONFIG_SCHED_CHILD_STATUS``: If this option is selected, then
     the exit status of the child task will be retained after the child
     task exits. This option should be selected if you require knowledge
     of a child process's exit status. Without this setting, ``wait()``,
     ``waitpid()`` or ``waitid()`` may fail. For example, if you do:

       #. Start child task
       #. Wait for exit status (using :c:func:`wait`, :c:func:`waitpid` or
          :c:func:`waitid`).

     This may fail because the child task may run to completion before the
     wait begins. There is a non-standard work-around in this case: The
     above sequence will work if you disable pre-emption using
     :c:func:`sched_lock` prior to starting the child task, then re-enable
     pre-emption with :c:func:`sched_unlock` after the wait completes. This
     works because the child task is not permitted to run until the wait
     is in place.

     The standard solution would be to enable
     ``CONFIG_SCHED_CHILD_STATUS``. In this case the exit status of the
     child task is retained after the child exits and the wait will
     successful obtain the child task's exit status whether it is called
     before the child task exits or not.

  -  ``CONFIG_PREALLOC_CHILDSTATUS``. To prevent runaway child status
     allocations and to improve allocation performance, child task exit
     status structures are pre-allocated when the system boots. This
     setting determines the number of child status structures that will be
     pre-allocated. If this setting is not defined or if it is defined to
     be zero then a value of 2\*\ ``MAX_TASKS`` is used.

     Note that there cannot be more that ``CONFIG_MAX_TASKS`` tasks in
     total. However, the number of child status structures may need to be
     significantly larger because this number includes the maximum number
     of tasks that are running PLUS the number of tasks that have exit'ed
     without having their exit status reaped (via :c:func:`wait`,
     :c:func:`waitpid` or :c:func:`waitid`).

     Obviously, if tasks spawn children indefinitely and never have the
     exit status reaped, then you may have a memory leak! (See **Warning**
     below)

.. warning:: If you enable the ``CONFIG_SCHED_CHILD_STATUS`` feature,
  then your application must either (1) take responsibility for reaping
  the child status with ``wait()``, ``waitpid()`` or ``waitid()``, or (2)
  suppress retention of child status. If you do not reap the child status,
  then you have a memory leak and your system will eventually fail.

Retention of child status can be suppressed on the parent using logic
like:

.. code-block:: c

  struct sigaction sa;

  sa.sa_handler = SIG_IGN;
  sa.sa_flags = SA_NOCLDWAIT;
  int ret = sigaction(SIGCHLD, &sa, NULL);

Functions
=========

.. c:function:: int sched_lock(void)

  Disables context switching by Disabling
  addition of new tasks to the ready-to-run task list. The task that calls
  this function will be the only task that is allowed to run until it
  either calls sched_unlock (the appropriate number of times) or until it
  blocks itself.

  :return: OK or ERROR.

  **POSIX Compatibility:** This is a NON-POSIX interface. VxWorks provides
  the comparable interface:

  .. code-block:: c

    STATUS taskLock(void);

.. c:function:: int sched_unlock(void)

  Decrements the preemption lock count.
  Typically this is paired with sched_lock() and concludes a critical
  section of code. Preemption will not be unlocked until sched_unlock()
  has been called as many times as sched_lock(). When the lockCount is
  decremented to zero, any tasks that were eligible to preempt the current
  task will execute.

  :return: OK or ERROR.

  **POSIX Compatibility:** This is a NON-POSIX interface. VxWorks provides
  the comparable interface:

  .. code-block:: c

    STATUS taskUnlock(void);

.. c:function:: int32_t sched_lockcount(void)

  Returns the current value of the
  lockCount. If zero, preemption is enabled; if non-zero, this value
  indicates the number of times that sched_lock() has been called on this
  thread of execution.

  :return: The current value of the lockCount.

  **POSIX Compatibility:** None.

.. c:function:: ipid_t waitpid(pid_t pid, int *stat_loc, int options)

  .. note::
     The following discussion is a general description of the
     ``waitpid()`` interface. However, as of this writing, the
     implementation of ``waitpid()`` is incomplete (but usable). If
     ``CONFIG_SCHED_HAVE_PARENT`` is defined, ``waitpid()`` will be a
     little more compliant to specifications. Without
     ``CONFIG_SCHED_HAVE_PARENT``, ``waitpid()`` simply supports waiting
     for any task to complete execution. With
     ``CONFIG_SCHED_HAVE_PARENT``, ``waitpid()`` will use ``SIGCHLD`` and
     can, therefore, wait for any child of the parent to complete. The
     implementation is incomplete in either case, however: NuttX does not
     support any concept of process groups. Nor does NuttX retain the
     status of exited tasks so if ``waitpid()`` is called after a task has
     exited, then no status will be available. The options argument is
     currently ignored.

  The ``waitpid()`` functions will obtain status information pertaining to
  one of the caller's child processes. The ``waitpid()`` function will
  suspend execution of the calling thread until status information for one
  of the terminated child processes of the calling process is available,
  or until delivery of a signal whose action is either to execute a
  signal-catching function or to terminate the process. If more than one
  thread is suspended in ``waitpid()`` awaiting termination of the same
  process, exactly one thread will return the process status at the time
  of the target process termination. If status information is available
  prior to the call to ``waitpid()``, return will be immediate.

  **NOTE**: Because ``waitpid()`` is not fully POSIX compliant, it must be
  specifically enabled by setting ``CONFIG_SCHED_WAITPID`` in the NuttX
  configuration file.

  :param pid: The task ID of the thread to wait for
  :param stat_loc: The location to return the exit status
  :param options: ignored

  The ``pid`` argument specifies a set of child processes for which status
  is requested. The ``waitpid()`` function will only return the status of
  a child process from this set:

  -  If ``pid`` is equal to ``(pid_t)-1``), status is requested for any
     child process. In this respect, ``waitpid()`` is then equivalent to
     ``wait()``.
  -  If ``pid`` is greater than 0, it specifies the process ID of a single
     child process for which status is requested.
  -  If ``pid`` is 0, status is requested for any child process whose
     process group ID is equal to that of the calling process.
  -  If ``pid`` is less than ``(pid_t)-1``), status is requested for any
     child process whose process group ID is equal to the absolute value
     of pid.

  The ``options`` argument is constructed from the bitwise-inclusive OR of
  zero or more of the following flags, defined in the ``<sys/wait.h>``
  header:

  -  ``WCONTINUED``. The ``waitpid()`` function will report the status of
     any continued child process specified by pid whose status has not
     been reported since it continued from a job control stop.
  -  ``WNOHANG``. The ``waitpid()`` function will not suspend execution of
     the calling thread if status is not immediately available for one of
     the child processes specified by ``pid``.
  -  ``WUNTRACED``. The status of any child processes specified by ``pid``
     that are stopped, and whose status has not yet been reported since
     they stopped, will also be reported to the requesting process.

  If the calling process has ``SA_NOCLDWAIT`` set or has ``SIGCHLD`` set
  to ``SIG_IGN``, and the process has no unwaited-for children that were
  transformed into zombie processes, the calling thread will block until
  all of the children of the process containing the calling thread
  terminate, and ``waitpid()`` will fail and set ``errno`` to ``ECHILD``.

  If ``waitpid()`` returns because the status of a child process is
  available, these functions will return a value equal to the process ID
  of the child process. In this case, if the value of the argument
  stat_loc is not a null pointer, information will be stored in the
  location pointed to by ``stat_loc``. The value stored at the location
  pointed to by ``stat_loc`` will be 0 if and only if the status returned
  is from a terminated child process that terminated by one of the
  following means:

  #. The process returned 0 from ``main()``.
  #. The process called ``_exit()`` or ``exit()`` with a status argument
     of 0.
  #. The process was terminated because the last thread in the process
     terminated.

  Regardless of its value, this information may be interpreted using the
  following macros, which are defined in ``<sys/wait.h>`` and evaluate to
  integral expressions; the ``stat_val`` argument is the integer value
  pointed to by ``stat_loc``.

  -  ``WIFEXITED(stat_val)``. Evaluates to a non-zero value if status was
     returned for a child process that terminated normally.
  -  ``WEXITSTATUS(stat_val)``. If the value of ``WIFEXITED(stat_val)`` is
     non-zero, this macro evaluates to the low-order 8 bits of the status
     argument that the child process passed to ``_exit()`` or ``exit()``,
     or the value the child process returned from ``main()``.
  -  ``WIFSIGNALED(stat_val)``. Evaluates to a non-zero value if status
     was returned for a child process that terminated due to the receipt
     of a signal that was not caught (see >signal.h<).
  -  ``WTERMSIG(stat_val)``. If the value of ``WIFSIGNALED(stat_val)`` is
     non-zero, this macro evaluates to the number of the signal that
     caused the termination of the child process.
  -  ``WIFSTOPPED(stat_val)``. Evaluates to a non-zero value if status was
     returned for a child process that is currently stopped.
  -  ``WSTOPSIG(stat_val)``. If the value of ``WIFSTOPPED(stat_val)`` is
     non-zero, this macro evaluates to the number of the signal that
     caused the child process to stop.
  -  ``WIFCONTINUED(stat_val)``. Evaluates to a non-zero value if status
     was returned for a child process that has continued from a job
     control stop.

  :return:
    If ``waitpid()`` returns because the status of a child process is
    available, it will return a value equal to the process ID of the child
    process for which status is reported.

    If ``waitpid()`` returns due to the delivery of a signal to the calling
    process, -1 will be returned and ``errno`` set to ``EINTR``.

    If ``waitpid()`` was invoked with WNOHANG set in options, it has at
    least one child process specified by pid for which status is not
    available, and status is not available for any process specified by pid,
    0 is returned.

    Otherwise, ``(pid_t)-1errno`` set to indicate the error:

    -  ``ECHILD``. The process specified by ``pid`` does not exist or is not
       a child of the calling process, or the process group specified by
       ``pid`` does not exist or does not have any member process that is a
       child of the calling process.
    -  ``EINTR``. The function was interrupted by a signal. The value of the
       location pointed to by ``stat_loc`` is undefined.
    -  ``EINVAL``. The ``options`` argument is not valid.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name, but the implementation is incomplete (as detailed above).

.. c:function:: int waitid(idtype_t idtype, id_t id, FAR siginfo_t *info, int options)

  .. note::

     The following discussion is a general description of the ``waitid()``
     interface. However, as of this writing, the implementation of
     ``waitid()`` is incomplete (but usable). If
     ``CONFIG_SCHED_HAVE_PARENT`` is defined, ``waitid()`` will be a
     little more compliant to specifications. ``waitpid()`` simply
     supports waiting a specific child task (``P_PID`` or for any child
     task ``P_ALL`` to complete execution. ``SIGCHLD`` is used. The
     implementation is incomplete in either case, however: NuttX does not
     support any concept of process groups. Nor does NuttX retain the
     status of exited tasks so if ``waitpid()`` is called after a task has
     exited, then no status will be available. The options argument is
     currently ignored.

  The ``waitid()`` function suspends the calling thread until one child of
  the process containing the calling thread changes state. It records the
  current state of a child in the structure pointed to by ``info``. If a
  child process changed state prior to the call to ``waitid()``,
  ``waitid()`` returns immediately. If more than one thread is suspended
  in ``wait()`` or ``waitpid()`` waiting termination of the same process,
  exactly one thread will return the process status at the time of the
  target process termination

  The ``idtype`` and ``id`` arguments are used to specify which children
  ``waitid()`` will wait for.

  -  If ``idtype`` is P_PID, ``waitid()`` will wait for the child with a
     process ID equal to (pid_t)``id``.
  -  If ``idtype`` is P_PGID, ``waitid()`` will wait for any child with a
     process group ID equal to (pid_t)``id``.
  -  If ``idtype`` is P_ALL, ``waitid()`` will wait for any children and
     ``id`` is ignored.

  The ``options`` argument is used to specify which state changes
  ``waitid()`` will will wait for. It is formed by OR-ing together one or
  more of the following flags:

  -  ``WEXITED``: Wait for processes that have exited.
  -  ``WSTOPPED``: Status will be returned for any child that has stopped
     upon receipt of a signal.
  -  ``WCONTINUES``: Status will be returned for any child that was
     stopped and has been continued.
  -  ``WNOHANG``: Return immediately if there are no children to wait for.
  -  ``WNOWAIT``: Keep the process whose status is returned in ``info`` in
     a waitable state. This will not affect the state of the process; the
     process may be waited for again after this call completes.

  The ``info`` argument must point to a ``siginfo_t`` structure. If
  ``waitid()`` returns because a child process was found that satisfied
  the conditions indicated by the arguments ``idtype`` and options, then
  the structure pointed to by ``info`` will be filled in by the system
  with the status of the process. The ``si_signo`` member will always be
  equal to ``SIGCHLD``.

  :return: If ``waitid()`` returns due to the change of state
    of one of its children, 0 is returned. Otherwise, -1 is returned and
    ``errno`` is set to indicate the error.

  The ``waitid()`` function will fail if:

  -  ``ECHILD``:
  -  ``EINTR``:
  -  ``EINVAL``: An invalid value was specified for ``options``, or
     ``idtype`` and ``id`` specify an invalid set of processes.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name, but the implementation is incomplete (as detailed in the
  description above).

.. c:function:: pid_t wait(FAR int *stat_loc)

  .. note::
     The following discussion is a general description of the :c:func:`wait`
     interface. However, as of this writing, the implementation of
     :c:func:`wait` is incomplete (but usable). :c:func:`wait` is based
     on :c:func:`waitpid` (see description for further information).

  The ``wait()`` function will suspend execution of the calling thread
  until status information for one of its terminated child processes is
  available, or until delivery of a signal whose action is either to
  execute a signal-catching function or to terminate the process. If more
  than one thread is suspended in ``wait()`` awaiting termination of the
  same process, exactly one thread will return the process status at the
  time of the target process termination. If status information is
  available prior to the call to\ ``wait()``, return will be immediate.

  The ``waitpid()`` function will behave identically to ``wait()``, if its
  ``pid`` argument is (pid_t)-1 and the options argument is 0. Otherwise,
  its behavior will be modified by the values of the ``pid`` and
  ``options`` arguments.

  :param stat_loc: The location to return the exit status
  :return: See the values returned by :c:func:`waitpid`

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name, but the implementation is incomplete (as detailed in the
  description ```waitpaid()`` <#waitpid>`__).

.. c:function:: int atexit(void (*func)(void))

  Registers a function to be called at program exit. The
  ``atexit()`` function registers the given function to be called at
  normal process termination, whether via ``exit()`` or via return from
  the program's ``main()``.

  .. note:: ``CONFIG_SCHED_ATEXIT`` must be defined to enable this function.

  :param func: A pointer to the function to be called when the task exits.
  :return: On success, ``atexit()`` returns OK (0). On error,
    ERROR (-1) is returned, and ```errno`` <#ErrnoAccess>`__ is set to
    indicate the cause of the failure.

  **POSIX Compatibility:** Comparable to the ISO C interface of the same
  name. Limitations in the current implementation:

    #. Only a single ``atexit`` function can be registered unless
       ``CONFIG_SCHED_ATEXIT_MAX`` defines a larger number.
    #. ``atexit()`` functions are not inherited when a new task is created.

.. c:function:: int on_exit(CODE void (*func)(int, FAR void *), FAR void *arg)

  Registers a function to be called at program exit. The
  ``on_exit()`` function registers the given function to be called at
  normal process termination, whether via ``exit()`` or via return from
  the program's ``main()``. The function is passed the status argument
  given to the last call to ``exit()`` and the ``arg`` argument from
  ``on_exit()``.

  .. note: ``CONFIG_SCHED_ONEXIT`` must be defined to enable this
    function

  :param func: A pointer to the function to be called when the task exits.
  :param arg: An argument that will be provided to the ``on_exit()``
     function when the task exits.

  :return: On success, ``on_exit()`` returns OK (0). On error,
    ERROR (-1) is returned, and ```errno`` <#ErrnoAccess>`__ is set to
    indicate the cause of the failure.

  **POSIX Compatibility:** This function comes from SunOS 4, but is also
  present in libc4, libc5 and glibc. It no longer occurs in Solaris (SunOS
  5). Avoid this function, and use the standard ``atexit()`` instead.

    #. Only a single ``on_exit`` function can be registered unless
       ``CONFIG_SCHED_ONEXIT_MAX`` defines a larger number.
    #. ``on_exit()`` functions are not inherited when a new task is created.
