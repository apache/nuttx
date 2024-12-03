========================================
Signaling Events from Interrupt Handlers
========================================

.. warning:: Migrated from 
    https://cwiki.apache.org/confluence/display/NUTTX/Signaling+Events+from+Interrupt+Handlers

Best way to wake multiple threads from interrupt?
=================================================

    I want to make a character device driver that passes the same data to 
    all tasks that are reading it. It is not so important whether the data 
    is queued or if just latest sample is retrieved. Problem is just how to 
    wake up the waiting threads.

At the most primitive level, a thread can be waiting for a semaphore, a signal, 
or a message queue (not empty or not full). Then there are higher 
level wrappers around these like mutexes, semaphores, poll waits, 
etc. But under the hood those are the three fundamental wait 
mechanisms. Any could be used to accomplish what you want.

In NuttX, some additional effort was put into the design of the signalling 
side of each of the IPCs so that they could be easily used by interrupts 
handlers. This behavior is unique to NuttX; POSIX says nothing about 
interrupt handlers. As a result, we will be talking about primarily 
non-portable OS interfaces.

    So far I've considered the following options:

And you basically have gone through the list of wait mechanisms:

Message Queues
==============

  1) Open a message queue when the device is opened (a new queue for each 
  task) and keep them in a list. Post to a non-blocking endpoint of these 
  queues in the ISR. Read from a blocking endpoint in the device ``read()``. 
  I would need to generate names for the message queues, as there doesn't 
  seem to be anonymous message queues?

When you start a project. It is a good idea to decide upon a common IPC 
mechanism to base your design on. POSIX message queues are one good 
choice to do that: Assign each thread a message queue and the ``main()`` 
of each thread simply waits on the message queue. It is a good 
architecture and used frequently.

However, I would probably avoid creating a lot of message queues just 
to support the interrupt level signaling. There are other ways to do 
that that do not use so much memory. So, if you have message queues, 
use them. If not, keep it simple.

In this case, your waiting task will block on a call to ``mq_receive()`` 
until a message is received. It will then wake up and can process 
the message. In the interrupt handler, it will call ``mq_send()`` when 
an event of interest occurs which will, in turn, wake up the waiting 
task.

Advantages of the use of message queues in this case are that 1) you 
can pass quite a lot of data in the message, and 2) it integrates 
well in a message-based application architecture. A disadvantage 
is that there is a limitation on the number of messages that can be 
sent from an interrupt handler so it is possible to get data overrun 
conditions, that is, more interrupt events may be received than can 
be reported with the available messages.

This limitation is due to the fact that you cannot allocate memory 
dynamically from an interrupt handler. Instead, interrupt handlers 
are limited to the use of pre-allocated messages. The number of 
pre-allocated messages is given by ``CONFIG_PREALLOC_MQ_MSGS`` + 8. 
The ``CONFIG_PREALLOC_MQ_MSGS`` can be used either by normal tasking 
logic or by interrupt level logic. The extra eight are an emergency 
pool for interrupt handling logic only (that value is not currently 
configurable).

If the task logic consumes all of the ``CONFIG_PREALLOC_MQ_MSGS`` messages, it 
will fall back to dynamically allocating messages at some cost to 
performance and deterministic behavior.

If the interrupt level consumes all of the ``CONFIG_PREALLOC_MQ_MSGS`` 
messages, it will fall back and use the emergency pool of 8 
pre-allocated messages. If those are also exhausted, then the message 
will not be sent and an interrupt is effectively lost.

Semaphores
==========

  2) Allocate a semaphore per each device open and keep them in a list. 
  Post the semaphores when new data is available in a shared buffer. 
  Read the data inside ``sched_lock()``.

If you don't have an architecture that uses message queues, and all of 
these threads are waiting only for the interrupt event and nothing else, 
then signaling semaphores would work fine too. You are basically using 
semaphores as condition variables in this case so you do have to be careful.

NOTE: You do not need multiple semaphores. You can do this with a single 
semaphore. If the semaphore is used for this purpose then you initialize 
it to zero:

.. code-block:: c

    sem_init(&sem, 0, 0);
    sem_setprotocol(&sem, SEM_PRIO_NONE);

``sem_setprotocol()`` is a non-standard NuttX function that should be called 
immediately after the ``sem_init()``. The effect of this function call is to 
disable priority inheritance for that specific semaphore. There should 
then be no priority inheritance operations on this semaphore that is 
used for signaling. See :doc:`/guides/signaling_sem_priority_inheritance` 
for further information.

Since the semaphore is initialized to zero, each time that a thread joins 
the group of waiting threads, the count is decremented. So a simple loop 
like this would wake up all waiting threads:

.. code-block:: c

    int svalue;
    int ret;
    
    for (; ; )
    {
        ret = sem_getvalue(&sem, &svalue);
        if (svalue < 0)
        {
            sem_post(&sem);
        }
        else
        {
            break;
        }
    }

NOTE: This use of ``sem_getvalue()`` is not portable. In many environments, 
``sem_getvalue()`` will not return negative values if there are waiters on 
the semaphore.

The above code snippet is essentially what the NuttX 
``pthread_cond_broadcast()`` does (see `nuttx/sched/pthread_condbroadcast.c <https://github.com/apache/nuttx/blob/master/sched/pthread/pthread_condbroadcast.c>`_). 
In NuttX condition variables are really just wrappers around semaphores 
that give them a few new properties. You could even call 
``pthread_cond_broadcast()`` from an interrupt handler: See 
http://pubs.opengroup.org/onlinepubs/009695399/functions/pthread_cond_signal.html 
for usage information.

Neither of the above mechanisms are portable uses of these interfaces. 
However, there is no portable interface for communicating directly with 
interrupt handlers.

If you want to signal a single waiting thread, there are simpler things 
you an do. In the waiting task:

.. code-block:: c

    semt_t g_mysemaphore;
    volatile bool g_waiting;
    ...
    
    sem_init(&g_mysemaphore);
    sem_setprotocol(&g_mysemaphore, SEM_PRIO_NONE);
    ...
    
    flags = enter_critical_section();
    g_waiting = true;
    while (g_waiting)
    {
        ret = sem_wait(&g_mysemaphore);
        ... handler errors ...
    }
    
    leave_critical_section(flags);

In the above code snippet, interrupts are disabled to set and test 
``g_waiting``. Interrupts will, of course, be re-enabled automatically 
and atomically while the task is waiting for the interrupt event.

Then in the interrupt handler

.. code-block:: c 

    extern semt_t g_mysemaphore;
    extern volatile bool g_waiting;
    ...
    
    if (g_waiting)
    {
        g_waiting = false;
        sem_post(&g_mysemaphore);
    }

An integer type counter could also be used instead of a type bool to 
support multiple waitings. In that case, this is equivalent to the 
case above using ``sem_getvalue()`` but does not depend on non-portable 
properties of ``sem_getvalue()``.

NOTE: There is possibility of improper interactions between the 
semaphore when it is used for signaling and priority inheritance. 
In this case, you should disable priority inheritance on the 
signaling semaphore using ``sem_setprotocol(SEM_PRIO_NONE)``. See 
:doc:`/guides/signaling_sem_priority_inheritance` 
for further information.

Signals
=======

  3) Store the thread id's in a list when ``read()`` is called. Wake up the  
  threads using ``sigqueue()``. Read the data from a shared buffer 
  inside ``sched_lock()``.

Signals would work fine too. Signals have a side-effect that is sometimes 
helpful and sometimes a pain in the butt: They cause almost all kinds of 
waits (``read()``, ``sem_wait()``, etc.) to wake up and return an error with 
``errno=EINTR``.

That is sometimes helpful because you can wake up a ``recv()`` or a ``read()`` 
etc., detect the event that generated the signal, and do something 
about it. It is sometimes a pain because you have to remember to 
handle the ``EINTR`` return value even when you don't care about it.

The POSIX signal definition includes some support that would make this 
easier for you. This support is not currently implemented in NuttX. 
The ``kill()`` interface for example 
(http://pubs.opengroup.org/onlinepubs/009695399/functions/kill.html) 
supports this behavior:

"If pid is 0, sig will be sent to all processes (excluding an unspecified 
set of system processes) whose process group ID is equal to the process 
group ID of the sender, and for which the process has permission to send 
a signal.

"If pid is -1, sig will be sent to all processes (excluding an unspecified 
set of system processes) for which the process has permission to send that 
signal."

"If pid is negative, but not -1, sig will be sent to all processes (excluding 
an unspecified set of system processes) whose process group ID is equal to 
the absolute value of pid, and for which the process has permission to send 
a signal."

NuttX does not currently support process groups. But that might be a good 
RTOS extension. If you and others think that would be useful I could 
probably add the basics of such a feature in a day or so.

``poll()``
==========

  Is there some better way that I haven't discovered?

The obvious thing that you did not mention is ``poll()``. See 
http://pubs.opengroup.org/onlinepubs/009695399/functions/poll.html . 
Since you are writing a device driver, support for the ``poll()`` method 
in your driver seems to be the natural solution. See the ``drivers/`` 
directory for many examples, ``drivers/pipes/pipe_common.c`` for one. 
Each thread could simply wait on ``poll()``; when the event occurs the 
driver could then wake up the set of waiters. Under the hood, this 
is again just a set of ``sem_post``'s. But it is also a very standard 
mechanism.

In your case, the semantics of ``poll()`` might have to be bent just a 
little. You might have to bend the meaning of some of the event 
flags since they are all focused on data I/O events.

Another creative use of ``poll()`` for use in cases like this:

  That would be something great! PX4 project has that implemented somehow
  (in C++), so maybe - if license permits - it could be ported to NuttX in
  no time?
  
  https://pixhawk.ethz.ch/px4/dev/shared_object_communication

I don't know a lot about this, but it might be worth looking into 
if it matches your need.