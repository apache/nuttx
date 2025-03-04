=============================================
Signaling Semaphores and Priority Inheritance
=============================================

.. warning:: Migrated from 
    https://cwiki.apache.org/confluence/display/NUTTX/Signaling+Semaphores+and+Priority+Inheritance

Locking vs Signaling Semaphores
===============================

Locking Semaphores
------------------
POSIX counting semaphores have multiple uses. The typical usage is where 
the semaphore is used as lock on one or more resources. In this typical 
case, priority inheritance works perfectly: The holder of a semaphore 
count must be remembered so that its priority can be boosted if a higher 
priority task requires a count from the semaphore. It remains the 
holder until the same task calls ``sem_post()`` to release the count on 
the semaphore.

Mutual Exclusion Example
------------------------
This usage is very common for providing mutual exclusion. The semaphore 
is initialized to a value of one. The first task to take the semaphore 
has access; additional tasks that need access will then block until 
the first holder calls ``sem_post()`` to relinquish access:

+---------------------+--------------------+
|     **TASK A**      |     **TASK B**     |
+=====================+====================+
| `have access`       |                    |
+---------------------+--------------------+
| `priority boost`    | **sem_wait(sem);** |
+---------------------+--------------------+
| `priority restored` | `have access`      |
+---------------------+--------------------+
| **sem_post(sem);**  |                    |
+---------------------+--------------------+
| **sem_wait(sem);**  |                    |
+---------------------+--------------------+
|                     | `blocked`          |
+---------------------+--------------------+

The important thing to note is that ``sem_wait()`` and ``sem_post()`` both 
called on the same thread, TASK A. When ``sem_wait()`` succeeds, TASK 
A becomes the holder of the semaphore and, while it is the holder 
of the semaphore (1) other threads, such as TASK B, cannot access 
the protected resource and (2) the priority of TASK A may be modified 
by the priority inheritance logic. TASK A remains the holder until 
is calls ``sem_post()`` on the `same thread`. At that time, (1) its 
priority may be restored and (2) TASK B has access to the resource.

Signaling Semaphores
--------------------
But a very different usage model for semaphores is for signaling 
events. In this case, the semaphore count is initialized to 
zero and the receiving task calls ``sem_wait()`` to wait for the 
next event of interest to occur. When an event of interest is 
detected by another task (or even an interrupt handler), 
``sem_post()`` is called which increments the count to 1 and 
wakes up the receiving task.

Signaling Semaphores and Priority Inheritance details
=====================================================

Example
-------
For example, in the following TASK A waits on a semaphore 
for events and TASK B (or perhaps an interrupt handler) 
signals task A of the occurrence of the events by posting 
to that semaphore:

+--------------------------+--------------------+
|        **TASK A**        |     **TASK B**     |
+==========================+====================+
| **sem_init(sem, 0, 0);** |                    |
+--------------------------+--------------------+
| **sem_wait(sem);**       |                    |
+--------------------------+--------------------+
| `blocked`                |                    |
+--------------------------+--------------------+
|                          | **sem_post(sem);** |
+--------------------------+--------------------+
| `Awakens as holder`      |                    |
+--------------------------+--------------------+

Notice that unlike the mutual exclusion case above, 
``sem_wait()`` and ``sem_post()`` are called on `different` 
threads.

Usage in Drivers
----------------

This usage case is used often within drivers, for example, 
when the user calls the ``read()`` method and there is no data 
available. ``sem_wait()`` is called to wait for new data to be 
received; ``sem_post()`` is called when the new data arrives 
and the user task is re-awakened.

Priority Inheritance Fails
--------------------------

These two usage models, the locking modeling and the 
signaling model, are really very different and priority 
inheritance simply does not apply when the semaphore is 
used for signalling rather than locking. In this signaling 
case priority inheritance can interfere with the operation 
of the semaphore. The problem is that when TASK A is 
awakened it is a holder of the semaphore. Normally, a 
task is removed from the holder list when it finally 
releases the semaphore via ``sem_post()``.

In this case, TASK B calls ``sem_post(sem)`` but TASK B is 
not the holder of the semaphore. Since TASK A never 
calls ``sem_post(sem)`` it becomes a permanently a holder 
of the semaphore and may have its priority boosted at 
any time when any other task tries to acquire the 
semaphore.

Who's to Blame
--------------

In the POSIX case, priority inheritance is specified only 
in the pthread mutex layer. In NuttX, on the other hand, 
pthread mutexes are simply built on top of binary locking 
semaphores. Hence, in NuttX, priority inheritance is 
implemented in the semaphore layer.

In the case of a mutex this could be simply resolved since 
there is only one holder but for the case of counting 
semaphores, there may be many holders and if the holder 
is not the thread that calls ``sem_post()``, then it is not 
possible to know which thread/holder should be released.

Selecting the Semaphore Protocol
================================

``sem_setprotocol()``
---------------------

The fix is to call non-standard NuttX function 
``sem_setprotocol(SEM_PRIO_NONE)`` immediately after the 
``sem_init()``. The effect of this function call is to 
disable priority inheritance for that specific 
semaphore. There should then be no priority inheritance 
operations on this semaphore that is used for signalling.

.. code-block:: C

    sem_t sem
    // ...
    sem_init(&sem, 0, 0);
    sem_setprotocol(&sem, SEM_PRIO_NONE);

Here is the rule: If you have priority inheritance 
enabled and you use semaphores for signaling events, 
then you `must` call ``sem_setprotocol(SEM_PRIO_NONE)`` 
immediately after initializing the semaphore.


Why Another Non-Standard OS Interface?
--------------------------------------

The non-standard ``sem_setprotocol()`` is the `moral` 
`equivalent` of the POSIX ``pthread_mutexattr_setprotocol()`` 
and its naming reflects that relationship. In most 
implementations, priority inheritance is implemented 
only in the pthread mutex layer. In NuttX, on the 
other hand, pthread mutexes are simply built on top 
of binary locking semaphores. Hence, in NuttX, 
priority inheritance is implemented in the semaphore 
layer. This architecture then requires an interface 
like ``sem_setprotocol()`` in order to manage the protocol 
of the underlying semaphore.


``pthread_mutexattr_setprotocol()``
-----------------------------------

Since NuttX implements pthread mutexes on top of 
binary semaphores, the above recommendation also 
applies when pthread mutexes are used for inter-thread 
signaling. That is, a mutex that is used for 
signaling should be initialize like this (simplified, 
no error checking here):

.. code-block:: c

    pthread_mutexattr_t attr;
    pthread_mutex_t mutex;
    // ...
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_PRIO_NONE);
    pthread_mutex_init(&mutex, &attr);

Is this Always a Problem?
=========================

Ideally ``sem_setprotocol(SEM_PRIO_NONE)`` should be 
called for all signaling semaphores. But, no, 
often the use of a signaling semaphore with priority 
inversion is not a problem. It is not a problem 
if the signaling semaphore is always taken on 
the same thread. For example:

* If the driver is used by only a single task, or
* If the semaphore is only taken on the worker thread.

But this can be a serious problem if multiple tasks 
ever wait on the signaling semaphore. Drivers like 
the serial driver, for example, have many user 
threads that may call into the driver.