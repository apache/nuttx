Signal Interfaces
*****************

**Tasks and Signals**. NuttX provides signal interfaces for tasks and
pthreads. Signals are used to alter the flow control of tasks by
communicating asynchronous events within or between task contexts. Any
task or interrupt handler can post (or send) a signal to a particular
task using its task ID. The task being signaled will execute
task-specified signal handler function the next time that the task has
priority. The signal handler is a user-supplied function that is bound
to a specific signal and performs whatever actions are necessary
whenever the signal is received.

By default, here are no predefined actions for any signal. The default
action for all signals (i.e., when no signal handler has been supplied
by the user) is to ignore the signal. In this sense, all NuttX are *real
time* signals by default. If the configuration option
``CONFIG_SIG_DEFAULT=y`` is included, some signals will perform their
default actions dependent upon addition configuration settings as
summarized in the following table:

=======  ====================   =========================
Signal 	 Action 	              Additional Configuration
=======  ====================   =========================
SIGUSR1  Abnormal Termination 	CONFIG_SIG_SIGUSR1_ACTION
SIGUSR2  Abnormal Termination 	CONFIG_SIG_SIGUSR2_ACTION
SIGALRM  Abnormal Termination 	CONFIG_SIG_SIGALRM_ACTION
SIGPOLL  Abnormal Termination 	CONFIG_SIG_SIGPOLL_ACTION
SIGSTOP  Suspend task           CONFIG_SIG_SIGSTOP_ACTION
SIGSTP   Suspend task           CONFIG_SIG_SIGSTOP_ACTION
SIGCONT  Resume task            CONFIG_SIG_SIGSTOP_ACTION
SIGINT   Abnormal Termination 	CONFIG_SIG_SIGKILL_ACTION
SIGKILL  Abnormal Termination 	CONFIG_SIG_SIGKILL_ACTION
=======  ====================   =========================

Tasks may also suspend themselves and wait until a signal is received.

**Tasks Groups**. NuttX supports both tasks and pthreads. The primary
difference between tasks and pthreads is the tasks are much more
independent. Tasks can create pthreads and those pthreads will share the
resources of the task. The main task and its children pthreads together
are referred as a *task group*. A task group is used in NuttX to emulate
a POSIX *process*.

.. note::
  Behavior of features related to task group\ s depend of NuttX
  configuration settings. See also the\ `NuttX
  Tasking <https://cwiki.apache.org/confluence/display/NUTTX/NuttX+Tasking>`__\ page
  and the\ `Tasks vs. Threads
  FAQ <https://cwiki.apache.org/confluence/display/NUTTX/Tasks+vs.+Threads+FAQ>`__\ for
  additional information on tasks and threads in NuttX.

**Signaling Multi-threaded Task Groups**. The behavior of signals in the
multi-thread task group is complex. NuttX emulates a process model with
task groups and follows the POSIX rules for signaling behavior. Normally
when you signal the task group you would signal using the task ID of the
main task that created the group (in practice, a different task should
not know the IDs of the internal threads created within the task group);
that ID is remembered by the task group (even if the main task thread
exits).

Here are some of the things that should happen when you signal a
multi-threaded task group:

-  If a task group receives a signal then one and only one indeterminate
   thread in the task group which is not blocking the signal will
   receive the signal.
-  If a task group receives a signal and more than one thread is waiting
   on that signal, then one and only one indeterminate thread out of
   that waiting group will receive the signal.

You can mask out that signal using ''sigprocmask()'' (or
''pthread_sigmask()''). That signal will then be effectively disabled
and will never be received in those threads that have the signal masked.
On creation of a new thread, the new thread will inherit the signal mask
of the parent thread that created it. So you if block signal signals on
one thread then create new threads, those signals will also be blocked
in the new threads as well.

You can control which thread receives the signal by controlling the
signal mask. You can, for example, create a single thread whose sole
purpose it is to catch a particular signal and respond to it: Simply
block the signal in the main task; then the signal will be blocked in
all of the pthreads in the group too. In the one "signal processing"
pthread, enable the blocked signal. This thread will then be only thread
that will receive the signal.

**Signal Interfaces**. The following signal handling interfaces are
provided by NuttX:

- :c:func:`sigemptyset`
- :c:func:`sigfillset`
- :c:func:`sigaddset`
- :c:func:`sigdelset`
- :c:func:`sigismember`
- :c:func:`sigaction`
- :c:func:`sigignore`
- :c:func:`sigset`
- :c:func:`sigprocmask`
- :c:func:`sighold`
- :c:func:`sigrelse`
- :c:func:`sigpending`
- :c:func:`sigsuspend`
- :c:func:`sigpause`
- :c:func:`sigwaitinfo`
- :c:func:`sigtimedwait`
- :c:func:`sigqueue`
- :c:func:`kill`
- :c:func:`pause`

.. c:function:: int sigemptyset(sigset_t *set)

  Initializes the signal set specified by
  set such that all signals are excluded.

  :param set: Signal set to initialize.

  :return: 0 (``OK``), or -1 (``ERROR``) if the signal set cannot be
    initialized.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sigfillset(sigset_t *set);

  Initializes the signal set specified by
  set such that all signals are included.

  :param set: Signal set to initialize

  :return: 0 (``OK``), or -1 (``ERROR``) if the signal set cannot be
    initialized.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sigaddset(sigset_t *set, int signo);

  Adds the signal specified by signo to the
  signal set specified by set.

  :param set: Signal set to add signal to
  :param signo: Signal to add

  :return: 0 (``OK``), or -1 (``ERROR``) if the signal number is invalid.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sigdelset(sigset_t *set, int signo);

  Deletes the signal specified by signo
  from the signal set specified by set.

  :param set: Signal set to delete the signal from
  :param signo: Signal to delete

  :return: 0 (``OK``), or -1 (``ERROR``) if the signal number is invalid.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int  sigismember(const sigset_t *set, int signo);

  Tests whether the signal specified by
  signo is a member of the set specified by set.

  :param set: Signal set to test
  :param signo: Signal to test for

  :return:
    -  1 (TRUE), if the specified signal is a member of the set,
    -  0 (OK or FALSE), if it is not, or
    -  -1 (``ERROR``) if the signal number is invalid.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sigaction(int signo, const struct sigaction *act, \
                     struct sigaction *oact);

  Allows the calling task to examine and/or
  specify the action to be associated with a specific signal.

  The structure sigaction, used to describe an action to be taken, is
  defined to include the following members:

    -  ``sa_u.sa_handler``. A pointer to a signal-catching function.
    -  ``sa_u.sa_sigaction``. An alternative form for the signal catching
       function.
    -  ``sa_mask``. Additional set of signals to be blocked during execution
       of the signal-catching function.
    -  ``sa_flags``: Special flags to affect behavior of a signal.

  If the argument act is not NULL, it points to a structure specifying the
  action to be associated with the specified signal. If the argument oact
  is not NULL, the action previously associated with the signal is stored
  in the location pointed to by the argument oact. If the argument act is
  NULL, signal handling is unchanged by this function call; thus, the call
  can be used to inquire about the current handling of a given signal.

  When a signal is caught by a signal-catching function installed by the
  sigaction() function, a new signal mask is calculated and installed for
  the duration of the signal-catching function. This mask is formed by
  taking the union of the current signal mask and the value of the sa_mask
  for the signal being delivered, and then including the signal being
  delivered. If and when the signal handler returns, the original signal
  mask is restored.

  Signal catching functions execute in the same address environment as the
  task that called sigaction() to install the signal-catching function.

  Once an action is installed for a specific signal, it remains installed
  until another action is explicitly requested by another call to
  sigaction().

  :param sig: Signal of interest
  :param act: Location of new handler
  :param oact: Location to store old handler

  :return: 0 (``OK``), or -1 (``ERROR``) if the signal number is invalid.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the POSIX implementation include:

    -  There are no default actions so the special value ``SIG_DFL`` is
       treated like ``SIG_IGN``.
    -  All ``sa_flags`` in struct sigaction of act input are ignored (all
       treated like ``SA_SIGINFO``). The one exception is if
       ``CONFIG_SCHED_CHILD_STATUS`` is defined; then ``SA_NOCLDWAIT`` is
       supported but only for ``SIGCHLD``.

.. c:function:: int sigignore(int signo);

  Sets the disposition of ``signo`` to ``SIG_IGN``.

  :param signo: The signal number to act on

  :return:
    -  Zero is returned upon successful completion, otherwise -1 (``ERROR``)
       is returned with the errno set appropriately. The ``errno`` value of
       ``EINVAL``, for example, would indicate that ``signo`` argument is
       not a valid signal number.

.. c:function:: void (*sigset(int signo, void (*disp)(int)))(int);

  Modifies signal dispositions.
  The ``signo`` argument specifies the signal. The ``disp`` argument
  specifies the signal's disposition, which may be ``SIG_DFL``,
  ``SIG_IGN``, or the address of a signal handler. If ``disp`` is the
  address of a signal handler, the system will add ``signo`` to the
  calling process's signal mask before executing the signal handler; when
  the signal handler returns, the system will restore the calling
  process's signal mask to its state prior to the delivery of the signal.
  ``signo`` will be removed from the calling process's signal mask.

  NOTE: The value ``SIG_HOLD`` for ``disp`` is not currently supported.

  :param signo: The signal number to operate on
  :param disp: The new disposition of the signal

  :return:
    -  Upon successful completion, ``sigset()`` will the previous
       disposition of the signal. Otherwise, ``SIG_ERR`` will be returned
       and ``errno`` set to indicate the error.

.. c:function:: int sigprocmask(int how, const sigset_t *set, sigset_t *oset);

  Allows the calling task to examine and/or
  change its signal mask. If the set is not NULL, then it points to a set
  of signals to be used to change the currently blocked set. The value of
  how indicates the manner in which the set is changed.

  If there are any pending unblocked signals after the call to
  sigprocmask(), those signals will be delivered before sigprocmask()
  returns.

  If sigprocmask() fails, the signal mask of the task is not changed.

  :param how: How the signal mask will be changed.
    - ``SIG_BLOCK`` The resulting set is the union of the current set and the signal set pointed to by the ``set`` input parameter.
    - ``SIG_UNBLOCK`` The resulting set is the intersection of the current set and the complement of the signal set pointed to by the ``set`` input parameter.
    - ``SIG_SETMASK`` The resulting set is the signal set pointed to by the ``set`` input parameter.

  :param set: Location of the new signal mask
  :param oset: Location to store the old signal mask

  :return: 0 (``OK``), or -1 (``ERROR``) if how is invalid.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sighold(int signo);

  Adds ``signo`` to the calling process's signal mask

  :param signo: Identifies the signal to be blocked.

  :return:
    Zero is returned upon successful completion, otherwise -1 (``ERROR``)
    is returned with the errno set appropriately. The ``errno`` value of
    ``EINVAL``, for example, would indicate that ``signo`` argument is
    not a valid signal number.

.. c:function:: int sigrelse(int signo);

  Removes ``signo`` from the calling process's signal mask

  :param signo: Identifies the signal to be unblocked.

  :return:
    Zero is returned upon successful completion, otherwise -1 (``ERROR``)
    is returned with the errno set appropriately. The ``errno`` value of
    ``EINVAL``, for example, would indicate that ``signo`` argument is
    not a valid signal number.

.. c:function:: int sigpending(sigset_t *set);

  Stores the returns the set of signals
  that are blocked for delivery and that are pending for the calling task
  in the space pointed to by set.

  If the task receiving a signal has the signal blocked via its
  sigprocmask, the signal will pend until it is unmasked. Only one pending
  signal (for a given signo) is retained by the system. This is consistent
  with POSIX which states: "If a subsequent occurrence of a pending signal
  is generated, it is implementation defined as to whether the signal is
  delivered more than once."

  :param set: The location to return the pending signal set.

  :return: 0 (``OK``) or -1 (``ERROR``)

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sigsuspend(const sigset_t *set);

  Replaces the signal mask with
  the set of signals pointed to by the argument set and then suspends the
  task until delivery of a signal to the task.

  If the effect of the set argument is to unblock a pending signal, then
  no wait is performed.

  The original signal mask is restored when sigsuspend() returns.

  Waiting for an empty signal set stops a task without freeing any
  resources (a very bad idea).

  :param set: The value of the signal **mask** to use while suspended.

  :return: -1 (``ERROR``) always

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the POSIX specification include:

  -  POSIX does not indicate that the original signal mask is restored.
  -  POSIX states that sigsuspend() "suspends the task until delivery of a
     signal whose action is either to execute a signal-catching function
     or to terminate the task." Only delivery of the signal is required in
     the present implementation (even if the signal is ignored).

.. c:function:: int sigpause(int signo);

  Removes ``signo`` from the calling process's signal mask and suspend the calling process
  until a signal is received. The ``sigpause()``) function will restore
  the process's signal mask to its original state before returning.

  :param set: Identifies the signal to be unblocked while waiting.

  :return:
    ``sigpause`` always returns -1 (``ERROR``). On a successful wait for
    a signal, the ``errno`` will be set to ``EINTR``.

.. c:function:: int sigwaitinfo(const sigset_t *set, struct siginfo *info);

Equivalent to sigtimedwait() with a NULL timeout parameter. (see below).

  :param set: The set of pending signals to wait for.
  :param info: The returned signal values

  :return:
    Signal number that cause the wait to be terminated, otherwise -1
    (``ERROR``) is returned.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sigtimedwait(const sigset_t *set, struct siginfo *info, \
                        const struct timespec *timeout);

  Selects the pending signal set specified
  by the argument set. If multiple signals are pending in set, it will
  remove and return the lowest numbered one. If no signals in set are
  pending at the time of the call, the calling task will be suspended
  until one of the signals in set becomes pending OR until the task
  interrupted by an unblocked signal OR until the time interval specified
  by timeout (if any), has expired. If timeout is NULL, then the timeout
  interval is forever.

  If the info argument is non-NULL, the selected signal number is stored
  in the si_signo member and the cause of the signal is store in the
  si_code member. The content of si_value is only meaningful if the signal
  was generated by sigqueue(). The following values for si_code are
  defined in signal.h:

  -  ``SI_USER``. Signal sent from kill, raise, or abort
  -  ``SI_QUEUE``. Signal sent from sigqueue
  -  ``SI_TIMER``. Signal is result of timer expiration
  -  ``SI_ASYNCIO``. Signal is the result of asynchronous IO completion
  -  ``SI_MESGQ``. Signal generated by arrival of a message on an empty
     message queue.

  :param set: The set of pending signals to wait for.
  :param info: The returned signal values
  :param timeout: The amount of time to wait

  :return: Signal number that cause the wait to be terminated, otherwise -1
    (``ERROR``) is returned.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the POSIX interface include:

  -  Values for si_codes differ
  -  No mechanism to return cause of ERROR. (It can be inferred from
     si_code in a non-standard way).
  -  POSIX states that "If no signal is pending at the time of the call,
     the calling task will be suspended until one or more signals in set
     become pending or until it is interrupted by an unblocked, *caught*
     signal." The present implementation does not require that the
     unblocked signal be caught; the task will be resumed even if the
     unblocked signal is ignored.

.. c:function:: int sigqueue (int tid, int signo, union sigval value);

  Sends the signal specified by signo with
  the signal parameter value to the task specified by tid.

  If the receiving task has the signal blocked via its sigprocmask, the
  signal will pend until it is unmasked. Only one pending signal (for a
  given signo) is retained by the system. This is consistent with POSIX
  which states: "If a subsequent occurrence of a pending signal is
  generated, it is implementation defined as to whether the signal is
  delivered more than once."

  :param tid: ID of the task to receive signal
  :param signo: Signal number
  :param value: Value to pass to task with signal

  :return:
    On success (at least one signal was sent), zero (``OK``) is returned.
    On error, -1 (``ERROR``) is returned, and
    ```errno`` <#ErrnoAccess>`__ is set appropriately.

    -  ``EGAIN``. The limit of signals which may be queued has been
       reached.
    -  ``EINVAL``. signo was invalid.
    -  ``EPERM``. The task does not have permission to send the signal to
       the receiving process.
    -  ``ESRCH``. No process has a PID matching pid.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the POSIX interface include:

  -  Default action is to ignore signals.
  -  Signals are processed one at a time in order
  -  POSIX states that, "If signo is zero (the null signal), error
     checking will be performed but no signal is actually sent." There is
     no null signal in the present implementation; a zero signal will be
     sent.

.. c:function:: int kill(pid_t pid, int sig);

  The kill() system call can be used to send any signal
  to any task.

  If the receiving task has the signal blocked via its sigprocmask, the
  signal will pend until it is unmasked. Only one pending signal (for a
  given signo) is retained by the system. This is consistent with POSIX
  which states: "If a subsequent occurrence of a pending signal is
  generated, it is implementation defined as to whether the signal is
  delivered more than once."

  :param pid: The id of the task to receive the signal. The POSIX
    ``kill()`` specification encodes process group information as zero
    and negative pid values. Only positive, non-zero values of pid are
    supported by this implementation. ID of the task to receive signal
  :param signo: The signal number to send. If signo is zero, no signal is
    sent, but all error checking is performed.

  :return: OK or ERROR

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the POSIX interface include:

  -  Default action is to ignore signals.
  -  Signals are processed one at a time in order
  -  Sending of signals to 'process groups' is not supported in NuttX.

.. c:function:: int pause(void);

  Suspends the calling thread until delivery of a non-blocked signal.

  :return: Since ``pause()`` suspends thread execution
    indefinitely unless interrupted a signal, there is no successful
    completion return value. A value of -1 (``ERROR`` will always be
    returned and errno set to indicate the error (``EINTR``).

  **POSIX Compatibility:** In the POSIX description of this function is
  the ``pause()`` function will suspend the calling thread until delivery
  of a signal whose action is either to execute a signal-catching function
  or to terminate the process. This implementation only waits for any
  non-blocked signal to be received.
