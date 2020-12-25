=============================
Counting Semaphore Interfaces
=============================

**Semaphores**. Semaphores are the basis for synchronization and mutual
exclusion in NuttX. NuttX supports POSIX semaphores.

Semaphores are the preferred mechanism for gaining exclusive access to a
resource. sched_lock() and sched_unlock() can also be used for this
purpose. However, sched_lock() and sched_unlock() have other undesirable
side-effects in the operation of the system: sched_lock() also prevents
higher-priority tasks from running that do not depend upon the
semaphore-managed resource and, as a result, can adversely affect system
response times.

Priority Inversion. Proper use of semaphores avoids the issues of
``sched_lock()``. However, consider the following example:

  #. Some low-priority task, *Task C*, acquires a semaphore in order to
     get exclusive access to a protected resource.
  #. *Task C* is suspended to allow some high-priority task,
  #. *Task A* attempts to acquire the semaphore held by *Task C* and gets
     blocked until *Task C* relinquishes the semaphore.
  #. *Task C* is allowed to execute again, but gets suspended by some
     medium-priority *Task B*.

At this point, the high-priority *Task A* cannot execute until *Task B*
(and possibly other medium-priority tasks) completes and until *Task C*
relinquishes the semaphore. In effect, the high-priority task, *Task A*
behaves as though it were lower in priority than the low-priority task,
*Task C*! This phenomenon is called *priority inversion*.

Some operating systems avoid priority inversion by *automatically*
increasing the priority of the low-priority *Task C* (the operable
buzz-word for this behavior is *priority inheritance*). NuttX supports
this behavior, but only if ``CONFIG_PRIORITY_INHERITANCE`` is defined in
your OS configuration file. If ``CONFIG_PRIORITY_INHERITANCE`` is not
defined, then it is left to the designer to provide implementations that
will not suffer from priority inversion. The designer may, as examples:

  -  Implement all tasks that need the semaphore-managed resources at the
     same priority level,
  -  Boost the priority of the low-priority task before the semaphore is
     acquired, or
  -  Use sched_lock() in the low-priority task.

Priority Inheritance. As mentioned, NuttX does support *priority
inheritance* provided that ``CONFIG_PRIORITY_INHERITANCE`` is defined in
your OS configuration file. However, the implementation and
configuration of the priority inheritance feature is sufficiently
complex that more needs to be said. How can a feature that can be
described by a single, simple sentence require such a complex
implementation:

  -  ``CONFIG_SEM_PREALLOCHOLDERS``. First of all, in NuttX priority
     inheritance is implement on POSIX counting semaphores. The reason for
     this is that these semaphores are the most primitive waiting
     mechanism in NuttX; Most other waiting facilities are based on
     semaphores. So if priority inheritance is implemented for POSIX
     counting semaphores, then most NuttX waiting mechanisms will have
     this capability.

     Complexity arises because counting semaphores can have numerous
     holders of semaphore counts. Therefore, in order to implement
     priority inheritance across all holders, then internal data
     structures must be allocated to manage the various holders associated
     with a semaphore. The setting ``CONFIG_SEM_PREALLOCHOLDERS`` defines
     the maximum number of different threads (minus one per semaphore
     instance) that can take counts on a semaphore with priority
     inheritance support. This setting defines the size of a single pool
     of pre-allocated structures. It may be set to zero if priority
     inheritance is disabled OR if you are only using semaphores as
     mutexes (only one holder) OR if no more than two threads participate
     using a counting semaphore.

     The cost associated with setting ``CONFIG_SEM_PREALLOCHOLDERS`` is
     slightly increased code size and around 6-12 bytes times the value of
     ``CONFIG_SEM_PREALLOCHOLDERS``.

  -  ``CONFIG_SEM_NNESTPRIO``: In addition, there may be multiple
     threads of various priorities that need to wait for a count from the
     semaphore. These, the lower priority thread holding the semaphore may
     have to be boosted numerous times and, to make things more complex,
     will have to keep track of all of the boost priorities values in
     order to correctly restore the priorities after a count has been
     handed out to the higher priority thread. The
     ``CONFIG_SEM_NNESTPRIO`` defines the size of an array, one array per
     active thread. This setting is the maximum number of higher priority
     threads (minus 1) than can be waiting for another thread to release a
     count on a semaphore. This value may be set to zero if no more than
     one thread is expected to wait for a semaphore.

     The cost associated with setting ``CONFIG_SEM_NNESTPRIO`` is slightly
     increased code size and (``CONFIG_SEM_PREALLOCHOLDERS`` + 1) times
     the maximum number of active threads.

  -  **Increased Susceptibility to Bad Thread Behavior**. These various
     structures tie the semaphore implementation more tightly to the
     behavior of the implementation. For examples, if a thread executes
     while holding counts on a semaphore, or if a thread exits without
     call ``sem_destroy()`` then. Or what if the thread with the boosted
     priority re-prioritizes itself? The NuttX implement of priority
     inheritance attempts to handle all of these types of corner cases,
     but it is very likely that some are missed. The worst case result is
     that memory could by stranded within the priority inheritance logic.

Locking versus Signaling Semaphores. Semaphores (and mutexes) may be
used for many different purposes. One typical use is for mutual
exclusion and locking of resources: In this usage, the thread that needs
exclusive access to a resources takes the semaphore to get access to the
resource. The same thread subsequently releases the semaphore count when
it no longer needs exclusive access. Priority inheritance is intended
just for this usage case.

In a different usage case, a semaphore may to be used to signal an
event: One thread A waits on a semaphore for an event to occur. When the
event occurs, another thread B will post the semaphore waking the
waiting thread A. This is a completely different usage model; notice
that in the mutual exclusion case, the same thread takes and posts the
semaphore. In the signaling case, one thread takes the semaphore and a
different thread posts the semaphore. Priority inheritance should
*never* be used in this signaling case. Subtle, strange behaviors may
result.

When priority inheritance is enabled with
``CONFIG_PRIORITY_INHERITANCE``, the default *protocol* for the
semaphore will be to use priority inheritance. For signaling semaphores,
priority inheritance must be explicitly disabled by calling
```sem_setprotocol`` <#semsetprotocol>`__ with ``SEM_PRIO_NONE``. For
the case of pthread mutexes,
```pthread_mutexattr_setprotocol`` <#pthreadmutexattrsetprotocol>`__
with ``PTHREAD_PRIO_NONE``.

This is discussed in much more detail on this `Wiki
page <https://cwiki.apache.org/confluence/display/NUTTX/Signaling+Semaphores+and+Priority+Inheritance>`__.

**POSIX semaphore interfaces:**

- :c:func:`sem_init`
- :c:func:`sem_destroy`
- :c:func:`sem_open`
- :c:func:`sem_close`
- :c:func:`sem_unlink`
- :c:func:`sem_wait`
- :c:func:`sem_timedwait`
- :c:func:`sem_trywait`
- :c:func:`sem_post`
- :c:func:`sem_getvalue`
- :c:func:`sem_getprotocol`
- :c:func:`sem_setprotocol`

.. c:function:: int sem_init(sem_t *sem, int pshared, unsigned int value)

  Initializes the UN-NAMED semaphore sem.
  Following a successful call to sem_init(), the semaphore may be used in
  subsequent calls to sem_wait(), sem_post(), and sem_trywait(). The
  semaphore remains usable until it is destroyed.

  Only ``sem`` itself may be used for performing synchronization. The
  result of referring to copies of ``sem`` in calls to ``sem_wait()``,
  ``sem_trywait()``, ``sem_post()``, and ``sem_destroy()``, is not
  defined.

  :param sem: Semaphore to be initialized
  :param pshared: Process sharing (not used)
  :param value: Semaphore initialization value

  :return: 0 (``OK``), or -1 (``ERROR``) if unsuccessful.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the full POSIX implementation include:

    - pshared is not used.

.. c:function:: int sem_destroy(sem_t *sem)

  Used to destroy the un-named semaphore
  indicated by ``sem``. Only a semaphore that was created using
  ``sem_init()`` may be destroyed using ``sem_destroy()``. The effect of
  calling ``sem_destroy()`` with a named semaphore is undefined. The
  effect of subsequent use of the semaphore ``sem`` is undefined until
  ``sem`` is re-initialized by another call to ``sem_init()``.

  The effect of destroying a semaphore upon which other tasks are
  currently blocked is undefined.

  :param sem: Semaphore to be destroyed.
  :return: 0 (``OK``), or -1 (``ERROR``) if unsuccessful.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: sem_t *sem_open(const char *name, int oflag, ...)

  Establishes a connection between named
  semaphores and a task. Following a call to sem_open() with the semaphore
  name, the task may reference the semaphore associated with name using
  the address returned by this call. The semaphore may be used in
  subsequent calls to sem_wait(), sem_trywait(), and sem_post(). The
  semaphore remains usable until the semaphore is closed by a successful
  call to sem_close().

  If a task makes multiple calls to sem_open() with the same name, then
  the same semaphore address is returned (provided there have been no
  calls to sem_unlink()).

  **Input Parameters:**

  :param name: Semaphore name
  :param oflag: Semaphore creation options. This may one of the following
     bit settings:

     -  ``oflag`` = 0: Connect to the semaphore only if it already exists.
     -  ``oflag`` = O_CREAT: Connect to the semaphore if it exists,
        otherwise create the semaphore.
     -  ``oflag`` = O_CREAT with O_EXCL (O_CREAT|O_EXCL): Create a new
        semaphore unless one of this name already exists.

  :param ``...``: **Optional parameters**. NOTE: When the O_CREAT flag is specified,
     POSIX requires that a third and fourth parameter be supplied:

     -  ``mode``. The mode parameter is of type mode_t. This parameter is
        required but not used in the present implementation.
     -  ``value``. The value parameter is type unsigned int. The semaphore
        is created with an initial value of ``value``. Valid initial
        values for semaphores must be less than or equal to
        ``SEM_VALUE_MAX`` (defined in ``include/limits.h``).

  :return: A pointer to sem_t or ``SEM_FAILED`` if unsuccessful.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the full POSIX implementation include:

    -  Treatment of links/connections is highly simplified. It is just a
       counting semaphore.

.. c:function:: int sem_close(sem_t *sem)

  This function is called to indicate that the calling
  task is finished with the specified named semaphore, sem. The
  sem_close() deallocates any system resources allocated by the system for
  this named semaphore.

  If the semaphore has not been removed with a call to sem_unlink(), then
  sem_close() has no effect on the named semaphore. However, when the
  named semaphore has been fully unlinked, the semaphore will vanish when
  the last task closes it.

  Care must be taken to avoid risking the deletion of a semaphore that
  another calling task has already locked.

  :param sem: Semaphore descriptor
  :return: 0 (``OK``), or -1 (``ERROR``) if unsuccessful.

  **Assumptions/Limitations:**

    -  Care must be taken to avoid deletion of a semaphore that another task
       has already locked.
    -  sem_close() must not be called with an un-named semaphore.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sem_unlink(const char *name)

  This function will remove the semaphore named by the
  input name parameter. If one or more tasks have the semaphore named by
  name open when sem_unlink() is called, destruction of the semaphore will
  be postponed until all references have been destroyed by calls to
  sem_close().

  :param name: Semaphore name
  :return: 0 (``OK``), or -1 (``ERROR``) if unsuccessful.

  **Assumptions/Limitations:**

    -  Care must be taken to avoid deletion of a semaphore that another task
       has already locked.
    -  sem_unlink() must not be called with an un-named semaphore.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Differences from the full POSIX implementation include:

    -  Treatment of links/connections is highly simplified. It is just a
       counting semaphore.
    -  Calls to sem_open() to re-create or re-connect to the semaphore may
       refer to the same semaphore; POSIX specifies that a new semaphore
       with the same name should be created after sem_unlink() is called.

.. c:function:: int sem_wait(sem_t *sem)

  This function attempts to lock the semaphore referenced
  by sem. If the semaphore as already locked by another task, the calling
  task will not return until it either successfully acquires the lock or
  the call is interrupted by a signal.

  :param sem: Semaphore descriptor.
  :return: 0 (``OK``), or -1 (``ERROR``) is unsuccessful

  If ``sem_wait`` returns -1 (``ERROR``) then the cause of the failure
  will be indicated by the thread-specific ```errno`` <#ErrnoAccess>`__.
  The following lists the possible values for
  ```errno`` <#ErrnoAccess>`__:

    -  ``EINVAL``: Indicates that the ``sem`` input parameter is not valid.
    -  ``EINTR``: Indicates that the wait was interrupt by a signal received
       by this task. In this case, the semaphore has not be acquired.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sem_timedwait(sem_t *sem, const struct timespec *abstime)

  This function will lock the semaphore referenced by sem
  as in the ``sem_wait()`` function. However, if the semaphore cannot be
  locked without waiting for another process or thread to unlock the
  semaphore by performing a ``sem_post()`` function, this wait will be
  terminated when the specified timeout expires.

  The timeout will expire when the absolute time specified by ``abstime``
  passes, as measured by the clock on which timeouts are based (that is,
  when the value of that clock equals or exceeds abstime), or if the
  absolute time specified by abstime has already been passed at the time
  of the call. This function attempts to lock the semaphore referenced by
  ``sem``. If the semaphore is already locked by another task, the calling
  task will not return until it either successfully acquires the lock or
  the call is interrupted by a signal.

  **Input Parameters:**

  :param sem: Semaphore descriptor.
  :param abstime: The absolute time to wait until a timeout is declared.
  :return: 0 (``OK``), or -1 (``ERROR``) is unsuccessful

  If ``sem_timedwait`` returns -1 (``ERROR``) then the cause of the
  failure will be indicated by the thread-specific
  ```errno`` <#ErrnoAccess>`__. The following lists the possible values
  for ```errno`` <#ErrnoAccess>`__:

  ``EINVAL``: Indicates that the ``sem`` input parameter is not valid or
  the thread would have blocked, and the abstime parameter specified a
  nanoseconds field value less than zero or greater than or equal to 1000
  million.

  ``ETIMEDOUT``: The semaphore could not be locked before the specified
  timeout expired.

  ``EDEADLK``: A deadlock condition was detected.

  ``EINTR``: Indicates that the wait was interrupt by a signal received by
  this task. In this case, the semaphore has not be acquired.

  **POSIX Compatibility:** Derived from IEEE Std 1003.1d-1999.

.. c:function:: int sem_trywait(sem_t *sem)

  This function locks the specified semaphore only if the
  semaphore is currently not locked. In any event, the call returns
  without blocking.

  :param sem: The semaphore descriptor
  :return: 0 (``OK``) or -1 (``ERROR``) if unsuccessful

  If ``sem_trywait`` returns -1 (``ERROR``) then the cause of the failure
  will be indicated by the thread-specific ```errno`` <#ErrnoAccess>`__.
  The following lists the possible values for
  ```errno`` <#ErrnoAccess>`__:

  -  ``EINVAL``: Indicates that the ``sem`` input parameter is not valid.
  -  ``EAGAIN``: Indicates that the semaphore was not acquired.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sem_post(sem_t *sem)

  When a task has finished with a semaphore, it will call
  sem_post(). This function unlocks the semaphore referenced by ``sem`` by
  performing the semaphore unlock operation.

  If the semaphore value resulting from this operation is positive, then
  no tasks were blocked waiting for the semaphore to become unlocked; The
  semaphore value is simply incremented.

  If the value of the semaphore resulting from this operation is zero,
  then on of the tasks blocked waiting for the semaphore will be allowed
  to return successfully from its call to ``sem_wait()``.

  .. note:: ``sem_post()`` may be called from an interrupt handler.

  :param sem: Semaphore descriptor
  :return: 0 (``OK``) or -1 (``ERROR``) if unsuccessful.

  **Assumptions/Limitations:**. When called from an interrupt handler, it
  will appear as though the interrupt task is the one that is performing
  the unlock.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sem_getvalue(sem_t *sem, int *sval)

  This function updates the location referenced by sval
  argument to have the value of the semaphore referenced by sem without
  effecting the state of the semaphore. The updated value represents the
  actual semaphore value that occurred at some unspecified time during the
  call, but may not reflect the actual value of the semaphore when it is
  returned to the calling task.

  If sem is locked, the value return by sem_getvalue() will either be zero
  or a negative number whose absolute value represents the number of tasks
  waiting for the semaphore.

  :param sem: Semaphore descriptor
  :param sval: Buffer by which the value is returned

  :return: 0 (``OK``) or -1 (``ERROR``) if unsuccessful.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int sem_getprotocol(FAR const pthread_mutexattr_t *attr, FAR int *protocol)

  Return the value of the semaphore protocol attribute.

  :param attr: A pointer to the semaphore to be queried
  :param protocol: The user provided location in which to store the
     protocol value. May be one of ``SEM_PRIO_NONE``, or
     ``SEM_PRIO_INHERIT``, ``SEM_PRIO_PROTECT``.
  :return: If successful, the ``sem_getprotocol()`` function will return zero
    (``OK``). Otherwise, an -1 (``ERROR``) will be returned and the
    ``errno`` value will be set to indicate the nature of the error.

  **POSIX Compatibility:** Non-standard NuttX interface. Should not be
  used in portable code. Analogous to
  ``pthread_muxtexattr_getprotocol()``.

.. c:function:: int sem_setprotocol(FAR pthread_mutexattr_t *attr, int protocol)

  Set semaphore protocol attribute. See the paragraph
  `Locking versus Signaling Semaphores <#lockingvssignaling>`__ for some
  important information about the use of this interface.

  :param attr: A pointer to the semaphore to be modified
  :param protocol: The new protocol to use. One of ``SEM_PRIO_NONE``, or
     ``SEM_PRIO_INHERIT``, ``SEM_PRIO_PROTECT``. ``SEM_PRIO_INHERIT`` is
     supported only if ``CONFIG_PRIORITY_INHERITANCE`` is defined;
     ``SEM_PRIO_PROTECT`` is not currently supported in any configuration.
  :return: If successful, the ``sem_setprotocol()`` function will return zero
    (``OK``). Otherwise, an -1 (``ERROR``) will be returned and the
    ``errno`` value will be set to indicate the nature of the error.

  **POSIX Compatibility:** Non-standard NuttX interface. Should not be
  used in portable code. Analogous to
  ``pthread_muxtexattr_setprotocol()``.
