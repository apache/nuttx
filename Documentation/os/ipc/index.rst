==========================
Inter-Thread Communication
==========================

How threads wait for each other and pass data.  The mechanisms live under
``sched/``: ``sched/mqueue/`` for message queues, ``sched/semaphore/`` for
semaphores and mutexes, ``sched/signal/`` for signals and ``sched/event/``
for events.

Each of them is in three pieces, and knowing which is which saves a search.
``sched/`` holds the internal machinery, named ``nxsem_``, ``nxmq_`` and so
on.  ``libs/libc/semaphore/`` and ``libs/libc/signal/`` hold the POSIX
surface that calls into it -- ``sem_wait()``, ``sigemptyset()``.  And
``fs/mqueue/``, ``fs/semaphore/`` and ``fs/event/`` hold only the *named*
variants, the ``open``, ``close`` and ``unlink`` of each: giving one of
these a name makes it a file system object, which is why that part lives
with the file system.

Which one to reach for:

.. list-table::
   :header-rows: 1
   :widths: 24 76

   * - Mechanism
     - Use it when
   * - Semaphore
     - Counting something: a resource with N instances, or one thread
       telling another that work is ready.
   * - Mutex
     - Protecting data.  Unlike a semaphore it has an owner: only the thread
       that locked it may unlock it, and never from an interrupt handler.
   * - Message queue
     - Passing data, not just a signal.  The queue decouples the sender from
       the receiver, up to the point where it fills: a sender that finds it
       full waits for room.
   * - Signal
     - Interrupting a thread that is doing something else, or reacting to an
       asynchronous event.
   * - Event
     - Waiting on a combination of conditions rather than a single one.
       An interrupt handler may deliver one, which a mutex cannot accept.
       The only one of the five you have to ask for: ``CONFIG_SCHED_EVENTS``
       is off by default, where message queues and signals are on and are
       removed by ``CONFIG_DISABLE_MQUEUE`` and
       ``CONFIG_DISABLE_ALL_SIGNALS``.

Priority inheritance is not one of the things that separates the two.
``CONFIG_PRIORITY_INHERITANCE`` covers semaphores as well as mutexes, and
when it is set both start out with inheritance enabled; a semaphore used for
signalling rather than locking is the case where you turn it back off, with
``sem_setprotocol(sem, SEM_PRIO_NONE)``.

.. toctree::
   :maxdepth: 1

   mutex.rst
   events.rst
   signal_handlers.rst
