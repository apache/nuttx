====================================
Spinlock Debug
====================================

Overview
--------

``CONFIG_SPINLOCK_DEBUG`` adds a set of debug tools that verify safe
spinlock use, in particular on SMP builds.  When enabled, every
``spin_lock`` / ``spin_unlock`` (and the recursive ``rspin_lock`` /
``rspin_unlock`` variants) records the holder and checks the locking
sequence of the running task.  A violation is reported with ``_alert()``
messages and triggers ``PANIC()`` so that the fault is caught at the point
of misuse instead of later as a hang or corruption.

With ``CONFIG_SPINLOCK_BACKTRACE`` additionally enabled, the holder's call
stack is recorded when a spinlock is locked and printed in the dump, which
makes it easier to identify the code path that holds a lock.

Configuration
-------------

.. code-block:: kconfig

    config SPINLOCK_DEBUG
        bool "Support debugging safe use of spinlock"
        depends on DEBUG_FEATURES

    config SPINLOCK_BACKTRACE
        bool "Enable spinlock backtrace"
        depends on SPINLOCK_DEBUG && LIBC_BACKTRACE_DEPTH > 0

* ``CONFIG_SPINLOCK_DEBUG`` depends on ``DEBUG_FEATURES`` and defaults to
  ``n``.  It is primarily useful on SMP configurations where lock
  ordering bugs are otherwise hard to reproduce.
* ``CONFIG_SPINLOCK_BACKTRACE`` additionally requires
  ``LIBC_BACKTRACE_DEPTH > 0``; ``spinlock_debug_t`` then carries a
  ``stack`` field.
* Both options build with or without ``CONFIG_SPINLOCK_INLINE``; the
  recursive ``rspinlock`` family is covered as well.

Detected misuse
---------------

The debug tools detect the following conditions while a task runs:

1. A task waits for more than one spinlock at the same time: it marks the
   first ``wait_spinlock`` and then enters a busy-wait on a second lock.
2. A task relocks a spinlock it already holds (the non-recursive
   ``spinlock_t`` is not reentrant).
3. A task acquires a lock different from the one it was marked as waiting
   for.
4. A task unlocks a spinlock it does not hold.
5. A task unlocks spinlocks in the wrong (non LIFO) order.
6. A task switches context while holding a spinlock.

For each violation the current ``hold_spinlock`` queue of the task is
dumped before ``PANIC()``.

Unlock ordering
---------------

On unlock, the holder is marked as released *before* the lock value is
written back.  Performing ``spinlock_mark_unlocked()`` first avoids an SMP
race window in which another thread observes the lock as free but still
sees the previous ``holder`` and is misreported as relocking the lock.

Backtrace
---------

When ``CONFIG_SPINLOCK_BACKTRACE`` is enabled, ``spinlock_mark_locked()``
records the caller stack via ``backtrace_record()`` into the ``stack``
field of ``spinlock_debug_t``; ``spinlock_mark_unlocked()`` removes the
record.  The dump printed on a violation formats the recorded stack, for
example::

    Spinlock(0x40210a40) backtrace: 0x1234abcd 0x1234cdc0 ...

Without backtrace, the dump prints a short locked notice, matching the
smaller ``spinlock_debug_t`` layout.

Notes
-----

* The checks run on the busy-wait (non-``trylock``) acquisition path,
  because a trylock cannot cause a circular wait.  Interrupt context is
  skipped.
* In SMP builds the critical-section lock (``g_cpu_irqlock``) is excluded
  from the multiple-wait check, since it is legitimately acquired from
  inside a wait, for example from the assert path.