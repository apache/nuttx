=====================
Mutual Exclusion lock
=====================

nxmutex
=======

Use `nxmutex` prefixed api to protect resources. In fact, nxmutex is implemented
based on nxsem. The difference between nxmutex and nxsem is that nxmutex supports
priority inheritance by default, nxsem do not support priority inheritance by default.

Typical Usage
-------------

Call nxmutex_init() for driver, when two tasks will use driver, their timing will be:

=================  ====================
taskA 	           taskB
=================  ====================
nxmutex_lock()     nxmutex_lock()
get lock running   wait for lock
nxmutex_unlock()   wait for lock
-                  get lock running
-                  nxmutex_unlock()
=================  ====================

Priority inheritance
====================

If `CONFIG_PRIORITY_INHERITANCE` is chosen, the priority of the task holding the mutex
may be changed.
This is an example:

  There are three tasks. Their priorities are high, medium, and low.
  We refer to them as `Htask` `Mtask` `Ltask`

  `Htask` and `Ltask` will hold the same mutex. `Mtask` does not hold mutex

if `CONFIG_PRIORITY_INHERITANCE` is not chosen, task running order
  #. `Ltask` hold a mutex first
  #.  Then `Htask` running, `Htask` can't hold the mutex,so wait
  #.  Then `Mtask` running, because `Mtask` priority higher than `Ltask`.
  #.  When `Mtask` finish, `Ltask` will start running.
  #.  When `Ltask` finish, `Htask` will start running.

From the above process, we can see that the medium-priority tasks run ahead of 
the high-priority tasks, which is unacceptable.

if `CONFIG_PRIORITY_INHERITANCE` is chosen, task running order
  #. `Ltask` hold a mutex first.
  #. Then `Htask` running, `Htask` can't hold the mutex, then boost the priority of `Ltask` 
to be the same as `Htask`.
  #. Because `Ltask` priority is higher than `Mtask`,so `Mtask` not running.
  #. When 'Ltask' finish, `Htask` will start running.
  #. When `Htask` finish, `Mtask` will start running.

Priority inheritance prevents medium-priority tasks from running ahead of
high-priority tasks

Api description
===============
.. c:function:: void nxmutex_init(FAR mutex_t *mutex)

    This function initialize the UNNAMED mutex
    :param mutex: mutex to be initialized.

    :return:
      Zero(OK) is returned on success.A negated errno value is returned on failure.

.. c:function:: void nxmutex_destroy(FAR mutex_t *mutex)

    This function destroy the UNNAMED mutex
    :param mutex: mutex to be destroyed.

    :return:
      Zero(OK) is returned on success.A negated errno value is returned on failure.

.. c:function:: void nxmutex_lock(FAR mutex_t *mutex)

    This function attempts to lock the mutex referenced by 'mutex'.  The
    mutex is implemented with a semaphore, so if the semaphore value is
    (<=) zero, then the calling task will not return until it successfully
    acquires the lock.

    :param mutex: mutex descriptor.

    :return:
      Zero(OK) is returned on success.A negated errno value is returned on failure.

.. c:function:: void nxmutex_trylock(FAR mutex_t *mutex)

    This function locks the mutex only if the mutex is currently not locked.
    If the mutex has been locked already, the call returns without blocking.

    :param mutex: mutex descriptor.

    :return:
      Zero(OK) is returned on success.A negated errno value is returned on failure.
      Possible returned errors:

      EINVAL - Invalid attempt to lock the mutex
      EAGAIN - The mutex is not available.

.. c:function:: void nxmutex_is_locked(FAR mutex_t *mutex)

    This function get the lock state the mutex referenced by 'mutex'.

    :param mutex: mutex descriptor.

    :return:
      if mutex is locked will return `ture`. if not will return `false`

.. c:function:: void nxmutex_unlock(FAR mutex_t *mutex)

    This function attempts to unlock the mutex referenced by 'mutex'.

    :param mutex: mutex descriptor.

    :return:
      Zero(OK) is returned on success.A negated errno value is returned on failure.

.. c:function:: void nxmutex_reset(FAR mutex_t *mutex)

    This function resets mutex states by 'mutex'.

    :param mutex: mutex descriptor.

    :return:
      Zero(OK) is returned on success.A negated errno value is returned on failure.
