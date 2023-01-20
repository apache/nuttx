==================
Pthread Interfaces
==================

NuttX does not support *processes* in the way that, say, Linux does.
NuttX only supports simple threads or tasks running within the same
address space. However, NuttX does support the concept of a *task
group*. A task group is the functional analog of a process: It is a
group that consists of the main task thread and of all of the pthreads
created by the main thread or any of the other pthreads within the task
group. Members of a task group share certain resources such as
environment variables, file descriptors, ``FILE`` streams, sockets,
pthread keys and open message queues.

.. note:: Behavior of features related to task groups depend of NuttX
  configuration settings. See also the\ `NuttX
  Tasking <https://cwiki.apache.org/confluence/display/NUTTX/NuttX+Tasking>`__\ page
  and the\ `Tasks vs. Threads
  FAQ <https://cwiki.apache.org/confluence/display/NUTTX/Tasks+vs.+Threads+FAQ>`__\ for
  additional information on tasks and threads in NuttX.

The following pthread interfaces are supported in some form by NuttX:

**pthread control interfaces**. Interfaces that allow you to create and
manage pthreads.

 - :c:func:`pthread_attr_init`
 - :c:func:`pthread_attr_destroy`
 - :c:func:`pthread_attr_setschedpolicy`
 - :c:func:`pthread_attr_getschedpolicy`
 - :c:func:`pthread_attr_setschedparam`
 - :c:func:`pthread_attr_getschedparam`
 - :c:func:`pthread_attr_setinheritsched`
 - :c:func:`pthread_attr_getinheritsched`
 - :c:func:`pthread_attr_setstacksize`
 - :c:func:`pthread_attr_getstacksize`
 - :c:func:`pthread_create`
 - :c:func:`pthread_detach`
 - :c:func:`pthread_exit`
 - :c:func:`pthread_cancel`
 - :c:func:`pthread_setcancelstate`
 - :c:func:`pthread_setcanceltype`
 - :c:func:`pthread_testcancel`
 - :c:func:`pthread_cleanup_pop`
 - :c:func:`pthread_cleanup_push`
 - :c:func:`pthread_join`
 - :c:func:`pthread_yield`
 - :c:func:`pthread_self`
 - :c:func:`pthread_getschedparam`
 - :c:func:`pthread_setschedparam`

**Thread Specific Data**. These interfaces can be used to create pthread
*keys* and then to access thread-specific data using these keys. Each
*task group* has its own set of pthread keys. NOTES: (1) pthread keys
create in one *task group* are not accessible in other task groups. (2)
The main task thread does not have thread-specific data.

 - :c:func:`pthread_key_create`
 - :c:func:`pthread_setspecific`
 - :c:func:`pthread_getspecific`
 - :c:func:`pthread_key_delete`

**pthread Mutexes**.

 - :c:func:`pthread_mutexattr_init`
 - :c:func:`pthread_mutexattr_destroy`
 - :c:func:`pthread_mutexattr_getpshared`
 - :c:func:`pthread_mutexattr_setpshared`
 - :c:func:`pthread_mutexattr_gettype`
 - :c:func:`pthread_mutexattr_settype`
 - :c:func:`pthread_mutexattr_getprotocol`
 - :c:func:`pthread_mutexattr_setprotocol`
 - :c:func:`pthread_mutex_init`
 - :c:func:`pthread_mutex_destroy`
 - :c:func:`pthread_mutex_lock`
 - :c:func:`pthread_mutex_timedlock`
 - :c:func:`pthread_mutex_trylock`
 - :c:func:`pthread_mutex_unlock`

**Condition Variables**.

 - :c:func:`pthread_condattr_init`
 - :c:func:`pthread_condattr_destroy`
 - :c:func:`pthread_cond_init`
 - :c:func:`pthread_cond_destroy`
 - :c:func:`pthread_cond_broadcast`
 - :c:func:`pthread_cond_signal`
 - :c:func:`pthread_cond_wait`
 - :c:func:`pthread_cond_timedwait`

**Barriers**.

 - :c:func:`pthread_barrierattr_init`
 - :c:func:`pthread_barrierattr_destroy`
 - :c:func:`pthread_barrierattr_setpshared`
 - :c:func:`pthread_barrierattr_getpshared`
 - :c:func:`pthread_barrier_init`
 - :c:func:`pthread_barrier_destroy`
 - :c:func:`pthread_barrier_wait`

**Initialization**.

 - :c:func:`pthread_once`

**Signals**.

 - :c:func:`pthread_kill`
 - :c:func:`pthread_sigmask`

No support for the following pthread interfaces is provided by NuttX:

  -  ``pthread_attr_getguardsize``. get and set the thread guardsize
     attribute.
  -  ``pthread_attr_getscope``. get and set the contentionscope attribute.
  -  ``pthread_attr_setguardsize``. get and set the thread guardsize
     attribute.
  -  ``pthread_attr_setscope``. get and set the contentionscope attribute.
  -  ``pthread_condattr_getpshared``. get the process-shared condition
     variable attribute.
  -  ``pthread_condattr_setpshared``. set the process-shared condition
     variable attribute.
  -  ``pthread_getconcurrency``. get and set the level of concurrency.
  -  ``pthread_getcpuclockid``. access a thread CPU-time clock.
  -  ``pthread_mutex_getprioceiling``. get and set the priority ceiling of
     a mutex.
  -  ``pthread_mutex_setprioceiling``. get and set the priority ceiling of
     a mutex.
  -  ``pthread_mutexattr_getprioceiling``. get and set the prioceiling
     attribute of the mutex attributes object.
  -  ``pthread_mutexattr_setprioceiling``. get and set the prioceiling
     attribute of the mutex attributes object.
  -  ``pthread_setconcurrency``. get and set the level of concurrency.

.. c:function:: int pthread_attr_init(pthread_attr_t *attr);

  Initializes a thread attributes object (attr) with
  default values for all of the individual attributes used by the
  implementation.

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_attr_init()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_attr_destroy(FAR pthread_attr_t *attr);

  An attributes object can be deleted when it is no
  longer needed.

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_attr_destroy()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_attr_setschedpolicy()`` function will
  return zero (``OK``). Otherwise, an error number will be returned to
  indicate the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_attr_getschedpolicy(FAR const pthread_attr_t *attr, FAR int *policy);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_attr_getschedpolicy()`` function will
  return zero (``OK``). Otherwise, an error number will be returned to
  indicate the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_attr_setschedparam(pthread_attr_t *attr, \
                                      const struct sched_param *param);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_attr_getschedpolicy()`` function will
  return zero (``OK``). Otherwise, an error number will be returned to
  indicate the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_attr_getschedparam(pthread_attr_t *attr, \
                                      struct sched_param *param);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_attr_getschedparam()`` function will return
  zero (``OK``). Otherwise, an error number will be returned to indicate
  the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_attr_setinheritsched(pthread_attr_t *attr, \
                                        int inheritsched);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_attr_setinheritsched()`` function will
  return zero (``OK``). Otherwise, an error number will be returned to
  indicate the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_attr_getinheritsched(const pthread_attr_t *attr, \
                                         int *inheritsched);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_attr_getinheritsched()`` function will
  return zero (``OK``). Otherwise, an error number will be returned to
  indicate the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_attr_setstacksize(pthread_attr_t *attr, long stacksize);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_attr_setstacksize()`` function will return
  zero (``OK``). Otherwise, an error number will be returned to indicate
  the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_attr_getstacksize(FAR const pthread_attr_t *attr, FAR size_t *stackaddr);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_attr_getstacksize()`` function will return
  zero (``OK``). Otherwise, an error number will be returned to indicate
  the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_create(pthread_t *thread, pthread_attr_t *attr, \
                          pthread_startroutine_t startRoutine, \
                          pthread_addr_t arg);

  To create a thread object and runnable thread, a
  routine must be specified as the new thread's start routine. An argument
  may be passed to this routine, as an untyped address; an untyped address
  may also be returned as the routine's value. An attributes object may be
  used to specify details about the kind of thread being created.

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_create()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_detach(pthread_t thread);

  A thread object may be "detached" to specify that the
  return value and completion status will not be requested.

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_detach()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: void pthread_exit(pthread_addr_t pvValue);

  A thread may terminate it's own execution.

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_exit()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_cancel(pthread_t thread);

  The ``pthread_cancel()`` function will request that thread be canceled.
  The target thread's cancellability state, enabled, or disabled,
  determines when the cancellation takes effect: When the cancellation is
  acted on, thread will be terminated. When cancellability is disabled,
  all cancellations are held pending in the target thread until the thread
  re-enables cancellability.

  The target thread's cancellability state determines how the cancellation
  is acted on: Either asynchronously or deferred. Asynchronous
  cancellations will be acted upon immediately (when enabled),
  interrupting the thread with its processing in an arbitrary state.

  When cancellability is deferred, all cancellations are held pending in
  the target thread until the thread changes the cancellability type or a
  `Cancellation
  Point <https://cwiki.apache.org/confluence/display/NUTTX/Cancellation+Points>`__
  function such as ```pthread_testcancel()`` <#pthreadtestcancel>`__ is
  entered.

  :param thread: Identifies the thread to be canceled.

  :return:
    If successful, the ``pthread_cancel()`` function will return zero
    (``OK``). Otherwise, an error number will be returned to indicate the
    error:

    -  ``ESRCH``. No thread could be found corresponding to that specified
       by the given thread ID.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. Except:

  -  The thread-specific data destructor functions will not be called for
     the thread. These destructors are not currently supported.

.. c:function:: int pthread_setcancelstate(int state, int *oldstate);

  The ``pthread_setcancelstate()`` function atomically sets both the
  calling thread's cancellability state to the indicated state and returns
  the previous cancellability state at the location referenced by
  oldstate. Legal values for state are PTHREAD_CANCEL_ENABLE and
  PTHREAD_CANCEL_DISABLE.

  Any pending thread cancellation may occur at the time that the
  cancellation state is set to PTHREAD_CANCEL_ENABLE.

  **Input Parameters:**

  :param state: New cancellation state. One of PTHREAD_CANCEL_ENABLE or
     PTHREAD_CANCEL_DISABLE.
  :param oldstate: Location to return the previous cancellation state.

  :return:
    If successful, the ``pthread_setcancelstate()`` function will return
    zero (``OK``). Otherwise, an error number will be returned to indicate
    the error:

    -  ``ESRCH``. No thread could be found corresponding to that specified
       by the given thread ID.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_setcanceltype(int type, FAR int *oldtype);

  The ``pthread_setcanceltype()`` function atomically
  both sets the calling thread's cancellability type to the indicated type
  and returns the previous cancellability type at the location referenced
  by ``oldtype``. Legal values for type are ``PTHREAD_CANCEL_DEFERRED``
  and ``PTHREAD_CANCEL_ASYNCHRONOUS``.

  The cancellability state and type of any newly created threads are
  ``PTHREAD_CANCEL_ENABLE`` and ``PTHREAD_CANCEL_DEFERRED respectively``.

  **Input Parameters:**

  :param type: New cancellation state. One of ``PTHREAD_CANCEL_DEFERRED``
     or ``PTHREAD_CANCEL_ASYNCHRONOUS``.
  :param oldtype: Location to return the previous cancellation type.

  :return:
    If successful, the ``pthread_setcancelstate()`` function will return
    zero (``OK``). Otherwise, an error number will be returned to indicate
    the error.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: void pthread_testcancel(void);

  The ``pthread_testcancel()`` function creates a `Cancellation
  Point <https://cwiki.apache.org/confluence/display/NUTTX/Cancellation+Points>`__
  in the calling thread. The ``pthread_testcancel()`` function has no
  effect if cancellability is disabled.

  **Input Parameters:** None

  **Returned Value:** None

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: void pthread_cleanup_pop(int execute);

  The ``pthread_cleanup_pop()`` function will remove the routine at the
  top of the calling thread's cancellation cleanup stack and optionally
  invoke it (if ``execute`` is non-zero).

  **Input Parameters:**

  -  ``execute``. Execute the popped cleanup function immediately.

  **Returned Value:**

  If successful, the ``pthread_setcancelstate()`` function will return
  zero (``OK``). Otherwise, an error number will be returned to indicate
  the error:

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: void pthread_cleanup_push(CODE void (*routine)(FAR void *), FAR void *arg);

  The ``pthread_cleanup_push()`` function will push the specified
  cancellation cleanup handler routine onto the calling thread's
  cancellation cleanup stack.

  The cancellation cleanup handler will be popped from the cancellation
  cleanup stack and invoked with the argument arg when:

  -  The thread exits (that is, calls ``pthread_exit()``).
  -  The thread acts upon a cancellation request.
  -  The thread calls ``pthread_cleanup_pop()`` with a non-zero execute
     argument.

  **Input Parameters:**

  -  ``routine``. The cleanup routine to be pushed on the cleanup stack.
  -  ``arg``. An argument that will accompany the callback.

  **Returned Value:**

  If successful, the ``pthread_setcancelstate()`` function will return
  zero (``OK``). Otherwise, an error number will be returned to indicate
  the error.

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_join(pthread_t thread, pthread_addr_t *ppvValue);

  A thread can await termination of another thread and
  retrieve the return value of the thread.

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_join()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: void pthread_yield(void);

  A thread may tell the scheduler that its processor can
  be made available.

  **Input Parameters:**

  -  None

  **Returned Value:**

  -  None. The ``pthread_yield()`` function always succeeds.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** This call is nonstandard, but present on
  several other systems. Use the POSIX
  ```sched_yield()`` <#sched_yield>`__ instead.

.. c:function:: pthread_t pthread_self(void);

  A thread may obtain a copy of its own thread handle.

  **Input Parameters:**

  -  None

  **Returned Value:**

  If successful, the ``pthread_self()`` function will return copy of
  caller's thread handle. Otherwise, in exceptional circumstances, the
  negated error code ``-ESRCH`` can be returned if the system cannot
  deduce the identity of the calling thread.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. The ``-ESRCH`` return value is non-standard; POSIX says
  ``pthread_self()`` must always succeed. NuttX implements
  ``pthread_self()`` as a macro only, not as a function as required by
  POSIX.

.. c:function:: int pthread_getschedparam(pthread_t thread, int *policy, \
                                 struct sched_param *param);

  The ``pthread_getschedparam()`` functions will get the
  scheduling policy and parameters of threads. For ``SCHED_FIFO`` and
  ``SCHED_RR``, the only required member of the ``sched_param`` structure
  is the priority ``sched_priority``.

  The ``pthread_getschedparam()`` function will retrieve the scheduling
  policy and scheduling parameters for the thread whose thread ID is given
  by ``thread`` and will store those values in ``policy`` and ``param``,
  respectively. The priority value returned from
  ``pthread_getschedparam()`` will be the value specified by the most
  recent ``pthread_setschedparam()``, ``pthread_setschedprio()``, or
  ``pthread_create()`` call affecting the target thread. It will not
  reflect any temporary adjustments to its priority (such as might result
  of any priority inheritance, for example).

  The policy parameter may have the value ``SCHED_FIFO``, ``SCHED_RR``, or
  ``SCHED_SPORADIC``. ``SCHED_RR`` requires the configuration setting
  ``CONFIG_RR_INTERVAL > 0``; ``SCHED_SPORADIC`` requires the
  configuration setting ``CONFIG_SCHED_SPORADIC=y``. (``SCHED_OTHER`` and
  non-standard scheduler policies, in particular, are not supported). The
  ``SCHED_FIFO`` and ``SCHED_RR`` policies will have a single scheduling
  parameter:

  -  ``sched_priority`` The thread priority.

  The ``SCHED_SPORADIC`` policy has four additional scheduling parameters:

  -  ``sched_ss_low_priority`` Low scheduling priority for sporadic
     server.
  -  ``sched_ss_repl_period`` Replenishment period for sporadic server.
  -  ``sched_ss_init_budget`` Initial budget for sporadic server.
  -  ``sched_ss_max_repl`` Maximum pending replenishments for sporadic
     server.

  **Input Parameters:**

  -  ``thread``. The ID of thread whose scheduling parameters will be
     queried.
  -  ``policy``. The location to store the thread's scheduling policy.
  -  ``param``. The location to store the thread's priority.

  **Returned Value:** 0 (``OK``) if successful. Otherwise, the error code
  ``ESRCH`` if the value specified by ``thread`` does not refer to an
  existing thread.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_setschedparam(pthread_t thread, int policy, \
                                 const struct sched_param *param);

  The ``pthread_setschedparam()`` functions will set the
  scheduling policy and parameters of threads. For ``SCHED_FIFO`` and
  ``SCHED_RR``, the only required member of the ``sched_param`` structure
  is the priority ``sched_priority``.

  The ``pthread_setschedparam()`` function will set the scheduling policy
  and associated scheduling parameters for the thread whose thread ID is
  given by ``thread`` to the policy and associated parameters provided in
  ``policy`` and ``param``, respectively.

  The policy parameter may have the value ``SCHED_FIFO`` or ``SCHED_RR``.
  (``SCHED_OTHER`` and ``SCHED_SPORADIC``, in particular, are not
  supported). The ``SCHED_FIFO`` and ``SCHED_RR`` policies will have a
  single scheduling parameter, ``sched_priority``.

  If the ``pthread_setschedparam()`` function fails, the scheduling
  parameters will not be changed for the target thread.

  **Input Parameters:**

  -  ``thread``. The ID of thread whose scheduling parameters will be
     modified.
  -  ``policy``. The new scheduling policy of the thread. Either
     ``SCHED_FIFO`` or ``SCHED_RR``. ``SCHED_OTHER`` and
     ``SCHED_SPORADIC`` are not supported.
  -  ``param``. The location to store the thread's priority.

  **Returned Value:**

  If successful, the ``pthread_setschedparam()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``EINVAL``. The value specified by ``policy`` or one of the
     scheduling parameters associated with the scheduling policy
     ``policy`` is invalid.
  -  ``ENOTSUP``. An attempt was made to set the policy or scheduling
     parameters to an unsupported value (``SCHED_OTHER`` and
     ``SCHED_SPORADIC`` in particular are not supported)
  -  ``EPERM``. The caller does not have the appropriate permission to set
     either the scheduling parameters or the scheduling policy of the
     specified thread. Or, the implementation does not allow the
     application to modify one of the parameters to the value specified.
  -  ``ESRCH``. The value specified by thread does not refer to a existing
     thread.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_key_create(pthread_key_t *key, void (*destructor)(void*))

  This function creates a thread-specific data key visible to all threads
  in the system. Although the same key value may be used by different
  threads, the values bound to the key by ``pthread_setspecific()`` are
  maintained on a per-thread basis and persist for the life of the calling
  thread.

  Upon key creation, the value ``NULL`` will be associated with the new
  key in all active threads. Upon thread creation, the value ``NULL`` will
  be associated with all defined keys in the new thread.

  **Input Parameters:**

  -  ``key`` is a pointer to the key to create.
  -  ``destructor`` is an optional destructor function that may be
     associated with each key that is invoked when a thread exits.
     However, this argument is ignored in the current implementation.

  **Returned Value:**

  If successful, the ``pthread_key_create()`` function will store the
  newly created key value at ``*key`` and return zero (``OK``). Otherwise,
  an error number will be returned to indicate the error:

  -  ``EAGAIN``. The system lacked sufficient resources to create another
     thread-specific data key, or the system-imposed limit on the total
     number of keys per task {``PTHREAD_KEYS_MAX``} has been exceeded.
  -  ``ENOMEM`` Insufficient memory exists to create the key.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

  -  The present implementation ignores the ``destructor`` argument.

.. c:function:: int pthread_setspecific(pthread_key_t key, void *value)

  The ``pthread_setspecific()`` function associates a thread-specific
  value with a key obtained via a previous call to
  ``pthread_key_create()``. Different threads may bind different values to
  the same key. These values are typically pointers to blocks of
  dynamically allocated memory that have been reserved for use by the
  calling thread.

  The effect of calling ``pthread_setspecific()`` with a key value not
  obtained from ``pthread_key_create()`` or after a key has been deleted
  with ``pthread_key_delete()`` is undefined.

  **Input Parameters:**

  -  ``key``. The data key to set the binding for.
  -  ``value``. The value to bind to the key.

  **Returned Value:**

  If successful, ``pthread_setspecific()`` will return zero (``OK``).
  Otherwise, an error number will be returned:

  -  ``ENOMEM``. Insufficient memory exists to associate the value with
     the key.
  -  ``EINVAL``. The key value is invalid.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

  -  ``pthread_setspecific()`` may be called from a thread-specific data
     destructor function.

.. c:function:: void *pthread_getspecific(pthread_key_t key)

  The ``pthread_getspecific()`` function returns the value currently bound
  to the specified key on behalf of the calling thread.

  The effect of calling ``pthread_getspecific()`` with a key value not
  obtained from ``pthread_key_create()`` or after a key has been deleted
  with ``pthread_key_delete()`` is undefined.

  **Input Parameters:**

  -  ``key``. The data key to get the binding for.

  **Returned Value:**

  The function ``pthread_getspecific()`` returns the thread-specific data
  associated with the given key. If no thread specific data is associated
  with the key, then the value ``NULL`` is returned.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

  -  ``pthread_getspecific()`` may be called from a thread-specific data
     destructor function.

.. c:function:: int pthread_key_delete(pthread_key_t key)

  This POSIX function deletes a thread-specific data key previously
  returned by ``pthread_key_create()``. No cleanup actions are done for
  data structures related to the deleted key or associated thread-specific
  data in any threads. It is undefined behavior to use ``key`` after it
  has been deleted.

  **Input Parameters:**

  -  ``key``. The key to delete

  **Returned Value:**

  If successful, the ``pthread_key_delete()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``EINVAL``. The parameter ``key`` is invalid.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_mutexattr_init(pthread_mutexattr_t *attr);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_mutexattr_init()`` function will return
  zero (``OK``). Otherwise, an error number will be returned to indicate
  the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_mutexattr_destroy(pthread_mutexattr_t *attr);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_mutexattr_destroy()`` function will return
  zero (``OK``). Otherwise, an error number will be returned to indicate
  the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_mutexattr_getpshared(pthread_mutexattr_t *attr, \
                                        int *pshared);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_mutexattr_getpshared()`` function will
  return zero (``OK``). Otherwise, an error number will be returned to
  indicate the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_mutexattr_setpshared(pthread_mutexattr_t *attr, \
                                       int pshared);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_mutexattr_setpshared()`` function will
  return zero (``OK``). Otherwise, an error number will be returned to
  indicate the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_mutexattr_gettype(FAR const pthread_mutexattr_t *attr, FAR int *type);

  **Input Parameters:**

  -  ``attr``. The mutex attributes to query
  -  ``type``. Location to return the mutex type. See
     ```pthread_mutexattr_settype()`` <#pthreadmutexattrsettype>`__ for a
     description of possible mutex types that may be returned.

  **Returned Value:**

  If successful, the ``pthread_mutexattr_settype()`` function will return
  zero (``OK``). Otherwise, an error number will be returned to indicate
  the error:

  -  ``EINVAL``. Parameters ``attr`` and/or ``attr`` are invalid.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_mutexattr_settype(pthread_mutexattr_t *attr, int type);

  Set the mutex type in the mutex attributes.

  **Input Parameters:**

  -  ``attr``. The mutex attributes in which to set the mutex type.
  -  ``type``. The mutex type value to set. The following values are
     supported:

     -  ``PTHREAD_MUTEX_NORMAL``. This type of mutex does not detect
        deadlock. A thread attempting to re-lock this mutex without first
        unlocking it will deadlock. Attempting to unlock a mutex locked by
        a different thread results in undefined behavior. Attempting to
        unlock an unlocked mutex results in undefined behavior.
     -  ``PTHREAD_MUTEX_ERRORCHECK``. This type of mutex provides error
        checking. A thread attempting to re-lock this mutex without first
        unlocking it will return with an error. A thread attempting to
        unlock a mutex which another thread has locked will return with an
        error. A thread attempting to unlock an unlocked mutex will return
        with an error.
     -  ``PTHREAD_MUTEX_RECURSIVE``. A thread attempting to re-lock this
        mutex without first unlocking it will succeed in locking the
        mutex. The re-locking deadlock which can occur with mutexes of
        type PTHREAD_MUTEX_NORMAL cannot occur with this type of mutex.
        Multiple locks of this mutex require the same number of unlocks to
        release the mutex before another thread can acquire the mutex. A
        thread attempting to unlock a mutex which another thread has
        locked will return with an error. A thread attempting to unlock an
        unlocked mutex will return with an error.
     -  ``PTHREAD_MUTEX_DEFAULT``. The default mutex type
        (PTHREAD_MUTEX_NORMAL).

     In NuttX, ``PTHREAD_MUTEX_NORMAL`` is not implemented. Rather, the
     behavior described for ``PTHREAD_MUTEX_ERRORCHECK`` is the *normal*
     behavior.

  **Returned Value:**

  If successful, the ``pthread_mutexattr_settype()`` function will return
  zero (``OK``). Otherwise, an error number will be returned to indicate
  the error:

  -  ``EINVAL``. Parameters ``attr`` and/or ``attr`` are invalid.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_mutexattr_getprotocol(FAR const pthread_mutexattr_t *attr, \
                                         FAR int *protocol);

  Return the value of the mutex protocol attribute..

  **Input Parameters:**

  -  ``attr``. A pointer to the mutex attributes to be queried
  -  ``protocol``. The user provided location in which to store the
     protocol value. May be one of ``PTHREAD_PRIO_NONE``, or
     ``PTHREAD_PRIO_INHERIT``, ``PTHREAD_PRIO_PROTECT``.

  **Returned Value:**

  If successful, the ``pthread_mutexattr_getprotocol()`` function will
  return zero (``OK``). Otherwise, an error number will be returned to
  indicate the error.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_mutexattr_setprotocol(FAR pthread_mutexattr_t *attr, \
                                         int protocol);

  Set mutex protocol attribute. See the paragraph
  `Locking versus Signaling Semaphores <#lockingvssignaling>`__ for some
  important information about the use of this interface.

  **Input Parameters:**

  -  ``attr``. A pointer to the mutex attributes to be modified
  -  ``protocol``. The new protocol to use. One of ``PTHREAD_PRIO_NONE``,
     or ``PTHREAD_PRIO_INHERIT``, ``PTHREAD_PRIO_PROTECT``.
     ``PTHREAD_PRIO_INHERIT`` is supported only if
     ``CONFIG_PRIORITY_INHERITANCE`` is defined; ``PTHREAD_PRIO_PROTECT``
     is not currently supported in any configuration.

  **Returned Value:**

  If successful, the ``pthread_mutexattr_setprotocol()`` function will
  return zero (``OK``). Otherwise, an error number will be returned to
  indicate the error.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_mutex_init(pthread_mutex_t *mutex, \
                              pthread_mutexattr_t *attr);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_mutex_init()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_mutex_destroy(pthread_mutex_t *mutex);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_mutex_destroy()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_mutex_lock(pthread_mutex_t *mutex);

  The mutex object referenced by mutex is locked by
  calling ``pthread_mutex_lock()``. If the mutex is already locked, the
  calling thread blocks until the mutex becomes available. This operation
  returns with the mutex object referenced by mutex in the locked state
  with the calling thread as its owner.

  If the mutex type is ``PTHREAD_MUTEX_NORMAL``, deadlock detection is not
  provided. Attempting to re-lock the mutex causes deadlock. If a thread
  attempts to unlock a mutex that it has not locked or a mutex which is
  unlocked, undefined behavior results.

  In NuttX, ``PTHREAD_MUTEX_NORMAL`` is not implemented. Rather, the
  behavior described for ``PTHREAD_MUTEX_ERRORCHECK`` is the *normal*
  behavior.

  If the mutex type is ``PTHREAD_MUTEX_ERRORCHECK``, then error checking
  is provided. If a thread attempts to re-lock a mutex that it has already
  locked, an error will be returned. If a thread attempts to unlock a
  mutex that it has not locked or a mutex which is unlocked, an error will
  be returned.

  If the mutex type is ``PTHREAD_MUTEX_RECURSIVE``, then the mutex
  maintains the concept of a lock count. When a thread successfully
  acquires a mutex for the first time, the lock count is set to one. Every
  time a thread re-locks this mutex, the lock count is incremented by one.
  Each time the thread unlocks the mutex, the lock count is decremented by
  one. When the lock count reaches zero, the mutex becomes available for
  other threads to acquire. If a thread attempts to unlock a mutex that it
  has not locked or a mutex which is unlocked, an error will be returned.

  If a signal is delivered to a thread waiting for a mutex, upon return
  from the signal handler the thread resumes waiting for the mutex as if
  it was not interrupted.

  **Input Parameters:**

  -  ``mutex``. A reference to the mutex to be locked.

  **Returned Value:**

  If successful, the ``pthread_mutex_lock()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  Note that this function will never return the error EINTR.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_mutex_timedlock(pthread_mutex_t *mutex, const struct timespec *abs_timeout);

  The ``pthread_mutex_timedlock()`` function will lock
  the mutex object referenced by ``mutex``. If the mutex is already
  locked, the calling thread will block until the mutex becomes available
  as in the ```pthread_mutex_lock()`` <#pthreadmutexlock>`__ function. If
  the mutex cannot be locked without waiting for another thread to unlock
  the mutex, this wait will be terminated when the specified
  ``abs_timeout`` expires.

  The timeout will expire when the absolute time specified by
  ``abs_timeout`` passes, as measured by the clock on which timeouts are
  based (that is, when the value of that clock equals or exceeds
  ``abs_timeout``), or if the absolute time specified by ``abs_timeout``
  has already been passed at the time of the call.

  **Input Parameters:**

  -  ``mutex``. A reference to the mutex to be locked.
  -  ``abs_timeout``. Maximum wait time (with ``NULL`` meaning to wait
     forever).

  **Returned Value:**

  If successful, the ``pthread_mutex_trylock()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error. Note that the errno ``EINTR`` is never returned by
  ``pthread_mutex_timedlock()``. The returned errno is ETIMEDOUT if the
  mutex could not be locked before the specified timeout expired

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name. This implementation does not return ``EAGAIN`` when the mutex
  could not be acquired because the maximum number of recursive locks for
  mutex has been exceeded.

.. c:function:: int pthread_mutex_trylock(pthread_mutex_t *mutex);

  The function ``pthread_mutex_trylock()`` is identical
  to ```pthread_mutex_lock()`` <#pthreadmutexlock>`__ except that if the
  mutex object referenced by mutex is currently locked (by any thread,
  including the current thread), the call returns immediately with the
  ``errno`` ``EBUSY``.

  If a signal is delivered to a thread waiting for a mutex, upon return
  from the signal handler the thread resumes waiting for the mutex as if
  it was not interrupted.

  **Input Parameters:**

  -  ``mutex``. A reference to the mutex to be locked.

  **Returned Value:**

  If successful, the ``pthread_mutex_trylock()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  Note that this function will never return the error EINTR.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_mutex_unlock(pthread_mutex_t *mutex);

  The ``pthread_mutex_unlock()`` function releases the mutex object
  referenced by mutex. The manner in which a mutex is released is
  dependent upon the mutex's type attribute. If there are threads blocked
  on the mutex object referenced by mutex when ``pthread_mutex_unlock()``
  is called, resulting in the mutex becoming available, the scheduling
  policy is used to determine which thread will acquire the mutex. (In the
  case of ``PTHREAD_MUTEX_RECURSIVE`` mutexes, the mutex becomes available
  when the count reaches zero and the calling thread no longer has any
  locks on this mutex).

  If a signal is delivered to a thread waiting for a mutex, upon return
  from the signal handler the thread resumes waiting for the mutex as if
  it was not interrupted.

  **Input Parameters:**

  -  ``mutex``.

  **Returned Value:**

  If successful, the ``pthread_mutex_unlock()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  Note that this function will never return the error EINTR.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_condattr_init(pthread_condattr_t *attr);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_condattr_init()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_condattr_destroy(pthread_condattr_t *attr);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_condattr_destroy()`` function will return
  zero (``OK``). Otherwise, an error number will be returned to indicate
  the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_cond_init(pthread_cond_t *cond, pthread_condattr_t *attr);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_cond_init()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_cond_destroy(pthread_cond_t *cond);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_cond_destroy()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_cond_broadcast(pthread_cond_t *cond);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_cond_broadcast()`` function will return
  zero (``OK``). Otherwise, an error number will be returned to indicate
  the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_cond_signal(pthread_cond_t *dond);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_cond_signal()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_cond_wait()`` function will return zero
  (``OK``). Otherwise, an error number will be returned to indicate the
  error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *mutex, \
                                  const struct timespec *abstime);

  **Input Parameters:**

  -  ``To be provided``.

  **Returned Value:**

  If successful, the ``pthread_cond_timedwait()`` function will return
  zero (``OK``). Otherwise, an error number will be returned to indicate
  the error:

  -  ``To be provided``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_barrierattr_init(FAR pthread_barrierattr_t *attr);

  The ``pthread_barrierattr_init()`` function will
  initialize a barrier attribute object ``attr`` with the default value
  for all of the attributes defined by the implementation.

  **Input Parameters:**

  -  ``attr``. Barrier attributes to be initialized.

  **Returned Value:** 0 (``OK``) on success or ``EINVAL`` if ``attr`` is
  invalid.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_barrierattr_destroy(FAR pthread_barrierattr_t *attr);

  The ``pthread_barrierattr_destroy()`` function will
  destroy a barrier attributes object. A destroyed attributes object can
  be reinitialized using ``pthread_barrierattr_init()``; the results of
  otherwise referencing the object after it has been destroyed are
  undefined.

  **Input Parameters:**

  -  ``attr``. Barrier attributes to be destroyed.

  **Returned Value:** 0 (``OK``) on success or ``EINVAL`` if attr is
  invalid.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_barrierattr_setpshared(FAR pthread_barrierattr_t *attr, int pshared);

  The process-shared attribute is set to
  ``PTHREAD_PROCESS_SHARED`` to permit a barrier to be operated upon by
  any thread that has access to the memory where the barrier is allocated.
  If the process-shared attribute is ``PTHREAD_PROCESS_PRIVATE``, the
  barrier can only be operated upon by threads created within the same
  process as the thread that initialized the barrier. If threads of
  different processes attempt to operate on such a barrier, the behavior
  is undefined. The default value of the attribute is
  ``PTHREAD_PROCESS_PRIVATE``.

  **Input Parameters:**

  -  ``attr``. Barrier attributes to be modified.
  -  ``pshared``. The new value of the pshared attribute.

  **Returned Value:** 0 (``OK``) on success or ``EINVAL`` if either
  ``attr`` is invalid or ``pshared`` is not one of
  ``PTHREAD_PROCESS_SHARED`` or ``PTHREAD_PROCESS_PRIVATE``.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_barrierattr_getpshared(FAR const pthread_barrierattr_t *attr, FAR int *pshared);

  The ``pthread_barrierattr_getpshared()`` function will
  obtain the value of the process-shared attribute from the attributes
  object referenced by ``attr``.

  **Input Parameters:**

  -  ``attr``. Barrier attributes to be queried.
  -  ``pshared``. The location to stored the current value of the pshared
     attribute.

  **Returned Value:** 0 (``OK``) on success or ``EINVAL`` if either
  ``attr`` or ``pshared`` is invalid.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_barrier_init(FAR pthread_barrier_t *barrier, \
                                FAR const pthread_barrierattr_t *attr, unsigned int count);

  The ``pthread_barrier_init()`` function allocates any
  resources required to use the barrier referenced by ``barrier`` and
  initialized the barrier with the attributes referenced by ``attr``. If
  ``attr`` is NULL, the default barrier attributes will be used. The
  results are undefined if ``pthread_barrier_init()`` is called when any
  thread is blocked on the barrier. The results are undefined if a barrier
  is used without first being initialized. The results are undefined if
  ``pthread_barrier_init()`` is called specifying an already initialized
  barrier.

  **Input Parameters:**

  -  ``barrier``. The barrier to be initialized.
  -  ``attr``. Barrier attributes to be used in the initialization.
  -  ``count``. The count to be associated with the barrier. The count
     argument specifies the number of threads that must call
     ``pthread_barrier_wait()`` before any of them successfully return
     from the call. The value specified by count must be greater than
     zero.

  **Returned Value:** 0 (``OK``) on success or one of the following error
  numbers:

  -  ``EAGAIN``. The system lacks the necessary resources to initialize
     another barrier.
  -  ``EINVAL``. The ``barrier`` reference is invalid, or the values
     specified by ``attr`` are invalid, or the value specified by
     ``count`` is equal to zero.
  -  ``ENOMEM``. Insufficient memory exists to initialize the barrier.
  -  ``EBUSY``. The implementation has detected an attempt to reinitialize
     a barrier while it is in use.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_barrier_destroy(FAR pthread_barrier_t *barrier);

  The ``pthread_barrier_destroy()`` function destroys the
  barrier referenced by ``barrie`` and releases any resources used by the
  barrier. The effect of subsequent use of the barrier is undefined until
  the barrier is reinitialized by another call to
  ``pthread_barrier_init()``. The results are undefined if
  ``pthread_barrier_destroy()`` is called when any thread is blocked on
  the barrier, or if this function is called with an uninitialized
  barrier.

  **Input Parameters:**

  -  ``barrier``. The barrier to be destroyed.

  **Returned Value:** 0 (``OK``) on success or one of the following error
  numbers:

  -  ``EBUSY``. The implementation has detected an attempt to destroy a
     barrier while it is in use.
  -  ``EINVAL``. The value specified by ``barrier`` is invalid.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_barrier_wait(FAR pthread_barrier_t *barrier);

  The ``pthread_barrier_wait()`` function synchronizes
  participating threads at the barrier referenced by ``barrier``. The
  calling thread is blocked until the required number of threads have
  called ``pthread_barrier_wait()`` specifying the same ``barrier``. When
  the required number of threads have called ``pthread_barrier_wait()``
  specifying the ``barrier``, the constant
  ``PTHREAD_BARRIER_SERIAL_THREAD`` will be returned to one unspecified
  thread and zero will be returned to each of the remaining threads. At
  this point, the barrier will be reset to the state it had as a result of
  the most recent ``pthread_barrier_init()`` function that referenced it.

  The constant ``PTHREAD_BARRIER_SERIAL_THREAD`` is defined in
  ``pthread.h`` and its value must be distinct from any other value
  returned by ``pthread_barrier_wait()``.

  The results are undefined if this function is called with an
  uninitialized barrier.

  If a signal is delivered to a thread blocked on a barrier, upon return
  from the signal handler the thread will resume waiting at the barrier if
  the barrier wait has not completed. Otherwise, the thread will continue
  as normal from the completed barrier wait. Until the thread in the
  signal handler returns from it, it is unspecified whether other threads
  may proceed past the barrier once they have all reached it.

  A thread that has blocked on a barrier will not prevent any unblocked
  thread that is eligible to use the same processing resources from
  eventually making forward progress in its execution. Eligibility for
  processing resources will be determined by the scheduling policy.

  **Input Parameters:**

  -  ``barrier``. The barrier on which to wait.

  **Returned Value:** 0 (``OK``) on success or ``EINVAL`` if the barrier
  is not valid.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_once(FAR pthread_once_t *once_control, CODE void (*init_routine)(void));

  The first call to ``pthread_once()`` by any thread with
  a given ``once_control``, will call the ``init_routine()`` with no
  arguments. Subsequent calls to ``pthread_once()`` with the same
  ``once_control`` will have no effect. On return from ``pthread_once()``,
  ``init_routine()`` will have completed.

  **Input Parameters:**

  -  ``once_control``. Determines if ``init_routine()`` should be called.
     ``once_control`` should be declared and initialized as follows:

     ``PTHREAD_ONCE_INIT`` is defined in ``pthread.h``.
  -  ``init_routine``. The initialization routine that will be called
     once.

  **Returned Value:** 0 (``OK``) on success or ``EINVAL`` if either
  once_control or init_routine are invalid.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_kill(pthread_t thread, int signo)

  The ``pthread_kill()`` system call can be used to send
  any signal to a thread. See ``kill()`` for further information as this
  is just a simple wrapper around the ``kill()`` function.

  **Input Parameters:**

  -  ``thread``. The id of the thread to receive the signal. Only
     positive, non-zero values of ``tthread``\ t are supported.
  -  ``signo``. The signal number to send. If ``signo`` is zero, no signal
     is sent, but all error checking is performed.

  **Returned Value:**

  On success, the signal was sent and zero is returned. On error one of
  the following error numbers is returned.

  -  ``EINVAL``. An invalid signal was specified.
  -  ``EPERM``. The thread does not have permission to send the signal to
     the target thread.
  -  ``ESRCH``. No thread could be found corresponding to that specified
     by the given thread ID.
  -  ``ENOSYS``. Do not support sending signals to process groups.

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.

.. c:function:: int pthread_sigmask(int how, FAR const sigset_t *set, FAR sigset_t *oset);

  This function is a simple wrapper around
  ``sigprocmask()``. See the ``sigprocmask()`` function description for
  further information.

  **Input Parameters:**

  -  ``how``. How the signal mask will be changed:

     -  ``SIG_BLOCK``: The resulting set is the union of the current set
        and the signal set pointed to by ``set``.
     -  ``SIG_UNBLOCK``: The resulting set is the intersection of the
        current set and the complement of the signal set pointed to by
        ``set``.
     -  ``SIG_SETMASK``: The resulting set is the signal set pointed to by
        ``set``.

  -  ``set``. Location of the new signal mask.
  -  ``oset``. Location to store the old signal mask.

  **Returned Value:**

  **Assumptions/Limitations:**

  **POSIX Compatibility:** Comparable to the POSIX interface of the same
  name.
