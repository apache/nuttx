.. _cancellation-points:

===================
Cancellation Points
===================

Cancellation Types
==================

In POSIX, there are two pthread cancellation types:
``PTHREAD_CANCEL_DEFERRED`` and ``PTHREAD_CANCEL_ASYNCHRONOUS``.

``PTHREAD_CANCEL_DEFERRED`` is the default and the normal kind of pthread
cancellation that should be used.
You cannot have the ``PTHREAD_CANCEL_DEFERRED`` cancellation type without
cancellation points and you will not have cancellation points unless you
enable them with ``CONFIG_CANCELLATION_POINTS=y``.

``PTHREAD_CANCEL_ASYNCHRONOUS`` is brutal; it simply kills the pthread
immediately without regard for what it is doing.
``PTHREAD_CANCEL_DEFERRED`` works differently; nothing happens immediately.
Instead, the cancellation will be deferred until the pthread is at
(or a better preposition would be within) a cancellation point.

Cancellation points add special instrumentation to a specific set
of interface functions. Those interface functions are listed here:
http://OpenGroup.org.


Enter and Leave Cancelation Point
=================================

In NuttX, each of these functions ``CONFIG_CANCELLATION_POINTS=y`` will enable
a special call to ``enter_cancellation_point()`` at the beginning
of the function and a matching call to ``leave_cancellation_point()`` before
the interface function returns.

These may be nested: One cancellation point function may call another which
may call another, etc. The ``select()`` function, for example, calls
``poll()`` which calls ``sem_wait()`` so the nesting will be three deep.

These two cancellation point hooks behave in a complementary way.
``enter_cancellation_point()`` increments the nesting level and
``leave_cancellation_point()`` decrements the nesting level.
No action is taken unless the nesting level is zero then, in that case,
if there is a pending cancellation then the thread exists normally
by simply calling ``pthread_exit()``.


pthread_cancel()
================

When ``pthread_cancel()`` is called, it will check if the deferred
cancellation mode is enabled for the target pthread. If so it will:

1. Mark the cancellation as pending.
2. If the thread is waiting on a semaphore or message queue (and is in
   a cancelable state), it will wake it just in the same was as if
   the thread received a single (except with ``ECANCELED``
   instead of ``EINTR``).

If, on the other hand, the asynchronous mode is enabled (and the thread is
in a cancelable state), ``pthread_cancel()`` will, instead,
kill the thread immediately.

.. note:: For pthreads, the cancelable state is controlled by the interface
          ``pthread_setcancelstate().`` This allows the pthread to disable or
          re-enable cancelability. If the thread is not cancelable when
          ``pthread_cancel()`` is called, it will never do more than just mark
          the cancellation as pending. When the cancelable state is
          re-enabled, then any pending cancellation will take effect.


select()
========

Let's walk through the case of ``select()`` to see how it works:

* When ``select()`` is called, it allocates some memory that it needs to
  handle the conversion to ``poll()``.
* It then calls ``poll()`` which sets up the poll with all of the relevant
  drivers and, eventually, calls ``sem_wait()``.

So this pthread will spend almost all of its time waiting in ``sem_wait()``
and that is the most likely state of the pthread when ``pthread_cancel()``
is called with deferred cancellation eanbled.

* When ``pthread_cancel()`` is called, it will set the pending cancellation
  state and wake up ``sem_wait()`` with the ``ECANCELED`` error.
* ``sem_wait()`` will see the pending cancellation, but will do nothing
  because the nesting level is decremented only to two. It will return to
  ``poll()`` with the ``ECANCELED`` error. Note that the semaphore is left
  in a healthy state.
* ``poll()`` will tear-down down the poll from all of the drivers and call
  ``leave_cancellation_point()`` before it exits. Again, it will see the
  pending cancellation but will do nothing because the nesting level
  decrements only to 1. Note that since ``poll()`` does the complete
  tear-down and all of the drivers are left in a healthy state.
* Finally, ``select()`` cleans up all of its memory allocations and calls
  ``leave_cancellation_point()``. This time, the nesting level decrements
  to zero and ``leave_cancellation_point()`` will call ``pthread_exit()``
  with everything in a proper state.

If asynchronous pthread cancellation were selected, then the behavior would
be very different. The first two steps would be the same but then:

* When ``pthread_cancel()`` is called, the thread would be immediately killed.
  The semaphore would be left in the locked state; the memory allocated by
  ``select()`` would be stranded; drivers would be left with stale pointers
  to stranded memory. Not a good state to leave the system!
* ``select()`` will not return to the caller in either case.


Application Interfaces
======================

The following pthread application interfaces are available to manage cancellation:

* ``pthread_cancel()`` cancels a thread.
* ``pthread_setcancelstate()`` can enable or disable a cancellation.
* ``pthread_setcanceltype()`` can be used to switched between asynchronous
  and deferred thread cancellation.
* ``pthread_testcancel()`` can be used to force a cancellation point.


Task Deletion
=============

NuttX also supports some non-standard interfaces for task deletion
(i.e., cancellation) of tasks as well. The cancellation point logic
is identical and the task application interfaces are very similar:

* ``task_delete()`` deletes (cancels) a task.
* ``task_setcancelstate()`` can enable or disable task cancellation.
* ``task_setcanceltype()`` can be used to switched between asynchronous
  and deferred task cancellation.
* ``task_testcancel()`` can be used to force a cancellation point.


Design Issues
=============

.. note:: The issue addressed in this paragraph is covered by
          `Issue 619 <https://github.com/apache/nuttx/issues/619>`_
          has been corrected in the code base through a series of changes
          culiminating in
          `PR 733 <https://github.com/apache/nuttx/pull/733>`_.
          This discussion still has value, however, as a warning
          for things that one needs to take into consideration
          when designing logic within the OS.

There is one logical problem in many parts of the system. The following
sequence of code appears in many places in the code base.
It will work not work with thread/task cancellation:

.. code-block:: c

  void some_lock(FAR sem_t *sem)
  {
    while ((ret = nxsem_wait(sem)) < 0)
      {
        DEBUGASSERT(ret = -EINTR || ret = -ECANCELED);
      }
  }

That logic will makes it impossible to cancel the thread.
Let me explain why:

* The fact that ``-ECANCELED`` is occurring means that some other task
  tried to cancel this one.
* The logic sequence is probably something like:

  * OS Function ``A`` is called by the application. Function ``A`` is
    a cancellation point. It calls ``enter_cancellation_point()`` on entry
    and ``leave_cancellation_point()`` on exit. If the application task is
    canceled, then the call to ``leave_cancellation_point()`` is where
    the task actually ends (by simply calling exit()).
  * OS Function ``A`` calls some internal Function ``B`` which has the above
    killer logic in it. It calls ``some_lock()`` and the task is suspended
    on waiting to get a count on the semaphore.

When the task is canceled, ``nxsem_wait()`` wakes up with the ``-ECANCELED``
error and returns that error to Function ``B``. But because Function ``B``
just loops and calls ``nxsem_wait()`` again, the task does not terminate;
it just goes back to waiting.

The end result is that Function ``B`` does not return to Function ``A`` so
``leave_cancellation_point()`` is never called and the task is not canceled.
The fix requires a little re-design. The locking function would
have to be like:

.. code-block:: c

  int some_lock(FAR sem_t *sem)
  {
    while ((ret = nxsem_wait(sem)) < 0)
      {
        DEBUGASSERT(ret = -EINTR || ret = -ECANCELED);
        if (ret != -EINTR)
          {
            break;
          }
      }

    return ret;
  }

It then returns a success/failure indication. Calling logic would also have
to be modified to detect the failure case and handle it appropriately.
In the case that the failure case, the calling logic (Function ``B`` in this
example) needs to clean up and return the failure upstream (to Function ``A``)
which must also clean up. Then, eventually, ``leave_critical_section()`` will
be called and the function will exit correctly.


Issue 619 Discussion
====================

.. note:: The
          `Issue 619 <https://github.com/apache/nuttx/issues/619>`_
          has been corrected in the code base through a series of changes
          culiminating in
          `PR 733 <https://github.com/apache/nuttx/pull/733>`_.

nxsem_wait_uninterruptible()
----------------------------

In NuttX, thread or tasks may be canceled using ``pthread_cancle()`` or
``task_delete()``. These can also be canceled via certain signals if default
signal actions are enabled.

When threads or tasks are canceled asynchronously, resources may be stranded.
For example, if memory is allocated, it will not be deallocated when the
task/thread is canceled in the FLAT build.  These kind on leaks can be handled
by the application with functions registered with ``pthread_cleanup()`` or
``on_exit()``, but others cannot.

The generic solution for NuttX is through the use of cancellation points.
Cancellation points are a mechanism that assures that all resources used
internal by OS interface calls are cleaned-up automatically at designated
cancellation points. An interface is normally a cancellation point if
it has any possibility of blocking.

In NuttX, when threads are blocked waiting of a semaphore when they are
canceled, ``nxsem_wait()`` will return ``-ECANCELED`` and the correct behavior
to let the error ripple back to the cancellation point where the thread/task
will be canceled after all resources have been cleaned up.

Many internal OS functions use ``nxsem_wait_uninterruptible()`` for waiting.
``nxsem_wait_uninterruptible()`` is defined in ``include/nuttx/semaphore.h``.
``nxsem_wait_uninterruptible()`` wraps a call to ``nxsem_wait()`` in
a loop like:

.. code-block:: c

  static inline int nxsem_wait_uninterruptible(FAR sem_t *sem)
  {
    int ret;

    do
      {
        /* Take the semaphore (perhaps waiting) */

        ret = nxsem_wait(sem);
      }
    while (ret == -EINTR || ret == -ECANCELED);

    return ret;
  }

The correct behavior is to continue waiting on ``-EINTR`` only.
If the task/thread is awakened by a signal just continue waiting.
The behavior for ``-ECANCELED`` is not correct. It should not loop.
Looping like this will prevent the cancellation from working.
Instead of returning through the cancellation point, the code will be stuck
in the above loop (in defferred cancellation mode) and cannot be cancelled.

The correct behavior is to return when ``-ECANCELED`` is encountered,
not to continue looping:

* GOOD: ``while (ret == -EINTR);``
* BAD:  ``while (ret == -EINTR || ret == -ECANCELED);``

The only complexity to this suggested change is that the callers of
``nxsem_wait_uninterruptible()`` must also check the return value for any
errors and make sure that that error is returned to the caller.
The error must propogate all the way back up to the cancellation point.

These are other related inline functions in ``include/nuttx/semaphore.h``
that behave in this same way. This issue applies to them as well.

.. important:: If cancellation points are enabled, the default cancellation
               mode should be deferred mode.

.. note:: There are three interfaces affected by this issue:
          ``nxsem_wait_uninterruptible()`` as discussed above, but also
          ``nxsem_timedwait_uninterruptible()`` and
          ``nxsem_tickwait_uninteruptible()`` which are closely related.

Mutual Exclusion Semaphores vs. Signaling Semaphores
----------------------------------------------------

Semaphore have two usages in NuttX:

* Mutual Exclusion Semaphores: Protect the shared data and force mutually
  exclusive use of semaphores.
* Signling Semaphores: Used to wait for events to be signaled asynchronously.

Should we return ``-ECANCEL/-EINTR`` for both? Or only for ithe second?
I think both should work the same: For both the semaphore wait should be
terminated and correct error should be returned.

Mutual Exclusion Semaphores
---------------------------

If the API requires returning ``EINTR`` if a signal is received, we should
return the error. This is required in many POSIX interfaces and don't see
how you could avoid that. However, the mutual exclusion semaphores are
the just there for the correctness in the design. There is seldom any real
competitor for the exclusion semaphore so, in general, they do not block and,
hence, would never generate any error.

And even if taking the mutual exclusion semaphore does cause the thread
to block, it should block for only a very brief moment of time and the
likelihood of a signal received or of the task being canceled is very small.

But if there is even a small possibility of that event happening,
it WILL happen in an embedded system (Murphy's law).

But if during the that brief moment while the thread is blocked,
a signal is received, I think that it is correct to return ``-EINTR``.
This is not so critical only because such an event it is rare.
But the specification requires it so we should do it.

When to Return ECANCELED
^^^^^^^^^^^^^^^^^^^^^^^^

The ``ECANCELED`` error is **ABSOLUTELY** critical to return under any
circumstance. That cancellation notification is received only once and
it if is ignored, then the thread will not cancel it will continue to run.
It really must return immediately with ``ECANCELED`` for the cancellation
to work quickly and reliably, where ever ``ECANCELED`` is detected.
Ignoring it is a hard bug in any case.

Signaling Semaphores
^^^^^^^^^^^^^^^^^^^^

The signaling semaphores are a little different story. They may block
for a very long time, for example, waiting for the receipt of data that
may never come. For example, a read from a serial port will hang indefinitely
if nothing is received on the serial port. So if a signal is received or
if the thread is canceled, then it is essential to terminate the semaphore
wait and return the error condition.
We should have no difference of opinion on that.

There is really no difference in the actions that must be taken if a signal
is received of if the task is canceled. Both mutual exclusion semaphores
and signaling semaphores should wake up and return the error condition.
The only real differences is that mutual exclusion semaphores either
do not block or, if they do, the do not block as long.

Mutual Exclusion Semaphores and Serialization
---------------------------------------------

There are situations were the task may block for a very long time,
even indefinitely, on a mutual exclusion semaphore. Consider the following
scenario:  A driver that reads data could have a sequence like this:

1. Get exclusive access to the driver by taking a semaphore.
2. Wait on a signaling semaphore for data (might be a long time).
3. Release the mutual exclusion semaphore.

Now suppose there are two applications, Task ``A`` and Task ``B``,
that open the driver and try to read from it. The first, Task ``A``, will get
the mutual exclusion semaphore at ``(1)`` and then will hang waiting
for data at (2). It may have to wait for a very long time to receive data.
When re-entered by the second, Task ``B``, it will hang at ``(1)``
also for very long time. Task ``B`` is effectively queued and cannot event
begin its wait on the signaling semphore until Task ``A`` receives its data
and releases the mutual exclusion semaphore. This is normal behavior
and exactly, how this kind of driver is supposed to work:
This use of mutual exclusion impelements serialization of I/O.

But now suppose that Task ``B`` receives a signal interrupt or a cancellation
event while waiting on the mutual exclusion semaphore. It MUST terminate the
wait and return ``-EINTR`` or ``-ECANCEL`` immediately. Task ``A`` may never
receive data and if that action is not taken, the signal interrup
or cancellation event is lost and the functionality has failed.

When to Return EINTR
^^^^^^^^^^^^^^^^^^^^

Returning ``EINTR`` really only applies to ``read()`` and ``write()``
(and ``open()``) functions as covered by Issue #669 . In other places,
``EINTR`` can be ignored. ``open()`` and ``close()`` should also
return ``EINTR``. But I don't think that is very meaningful for ``close()``
since most people ignore the return value from ``close()``.
But, on the other hand, ``ECANCELED`` can never be ignored under any
condition.

Detecting, Remembering, and Propagating Signal Interrupts and Cancellation Events
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Currently, the only way for an OS interface function to know if a signal
was received is to be awakened with an ``EINTR`` error. That is why
it is critical to always return the ``EINTR`` error and conform with
the POSIX requirements.

However, if a signal is received while the OS interface is NOT waiting,
then there is no way to know if the signal was received. I think that
is a limitation in the current design. There probably should be some latching
indication, perhaps in the TCB, that a signal interrupt has been received.

There is already a ``TCB_FLAG_CANCEL_PENDING`` in the TCB that will tell us
that the if the task has been canceled. That flag is tested in the
``leave_cancellation_point()`` function so I don't think that there is any
corresponding issue for thread cancellation. We just need to make sure that
all waits are aborted if ``ECANCELED`` is received and let the error
indication ripple all they back to the ``leave_cancellation_point()``
function. Then the function will exit cleanly, safely, and quickly.

Interestingly, the ``ECANCELED`` error will never be seen by the application.
It just triggers the return uwind sequence where all resources are recovered
and finally until ``leave_cancellation_point()`` is called. Then the thread
will exit before it returns to the applicaton.

You can see all of this working in the ``board/sim/sim/sim/configs/ostest``
configuration if you also enable::

  CONFIG_CANCELLATION_POINTS=y
  CONFIG_PTHREAD_CLEANUP=y

There is a ``cancel.c`` test within the OS test, but the more interesting test
is the ``pthread_cleanup.c`` test. You can see how all this works when the code
unwinds with the ``ECANCELED`` error and calls ``leave_cancellation_point()``.
Just single step through ``pthread_cond_wait()`` to see this.
``pthread_cond_wait()`` is a ``cancellation_point()`` and that is where
the thread exit will occur.

`PR#749 <https://github.com/apache/nuttx/pull/749>`_ adds those settings
to that ``sim:otest`` defconfig.
