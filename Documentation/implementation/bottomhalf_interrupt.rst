==============================
Bottom-Half Interrupt Handlers
==============================

RTOS Interrupts
===============

A well-design RTOS depends on the most minimal of interrupt level processing.
This is a very different concept that for bare metal programming:

* With bare metal programming most of the real-time work is usually performed
  in interrupt handlers. Interrupt handler execution may then extend in time
  considerably due to this interrupt level processing.

To compensate for this extended interrupt processing time, bare metal programmers
also need prioritized interrupts:

* If an interrupt request for a higher priority interrupt occurs during the
  extended processing of the lower priority interrupt, then that interrupt handler
  will itself be interrupted to service the higher priority interrupt requests.
  In this way bare metal interrupt handling is nested.

With an RTOS, the real-time strategy is very different:

* Interrupts must run very, very briefly so that they do not interfere with the
  RTOS real-time scheduling. Normally, the interrupt simply performs whatever
  minor housekeeping is necessary and then immediately defers processing by waking up
  some task via some Inter-Process Communication(IPC). The RTOS is then responsible for
  the real-time behavior, not the interrupt. And,
  
* since the interrupts must be very brief, there is little or no gain from nesting of interrupts.

Extending interrupt processing
==============================

But what if extended interrupt processing is required?
What if there is a significant amount of hardware-related operations that absolutely
must be performed as quickly as possible before we can turn processing over to
general, real-time tasking?

In NuttX, this is handled through a high priority trampoline called
the "High Priority Work Queue". It is a trampoline because it changes the interrupt
processing context for extended interrupt processing before notifying the normal
real-time task.

Processing on that ultra-high priority work thread then completes the extended
interrupt processing with interrupts enabled, but without interference from any
other real-time tasks.

At the completion of the extended processing, the high priority worker thread can
then continue processing via some IPC to a normal real-time task.

The portion of interrupt processing that is performed in the interrupt handler with
interrupts disabled is referred to as Top Half Interrupt processing; the portion of
interrupt processing that is performed on the high priority work queue with interrupts
enabled is referred to as Bottom Half Interrupt processing.

High Priority Work Queue
========================

NuttX supports a high priority work queue as well as a low priority work queue with
somewhat different properties.
The high priority work queue is dedicated to the support of Bottom Half Interrupt
processing.
Other uses of the high priority work queue may be inappropriate and may harm the
real-time performance of your system.

The high priority work queue must have these properties:

* **Highest Priority** The high priority work queue must be the highest priority
  task in your system. No other task should execute at a higher priority; No other
  task can be permitted to interfere with execution of the high priority work queue.

* **Zero Latency Context Switches** Provided that the priority of the high priority
  work queue is the highest in the system, then there will be no context switch
  overhead in getting from the Top Half Interrupt processing to the Bottom Half
  Interrupt processing other that the normal overhead of returning from an interrupt.
  Upon return from the interrupt, the system will immediately vector to high priority
  worker thread.

* **Brief Processing** Processing on the high priority work queue must still be brief.
  If there is high priority work in progress when the high priority worker is signaled,
  then that processing will be queued and delayed until it can be processed. That delay
  will add jitter to your real-time response. You must not generate a backlog of work
  for the high priority worker thread!

* **No Waiting** Work executing on the high priority work queue must not wait for
  resources or events on the high priority worker thread. Waiting on the high priority
  work queue blocks the queue and will, again, damage real-time performance.

Setting Up Bottom Half Interrupt Processing
===========================================

Bottom half interrupt processing is scheduled by top half interrupt processing by
simply calling the function ``work_queue()``:

.. code-block:: C

   int work_queue(int qid, FAR struct work_s *work, worker_t worker,
                  FAR void *arg, clock_t delay);

This same interface is the same for both high- and low-priority.
The qid argument distinguishes which work queue will be used. For bottom half
interrupt processing, ``qid`` must be set to ``HPWORK``.

The work argument is memory that will be used to actually queue the work.
It has no meaning to the caller; it is simply a memory allocation by the caller.
Otherwise, the work structure is completely managed by the work queue logic.
The caller should never modify the contents of the work queue structure directly.
If ``work_queue()`` is called before the previous work as been performed and removed
from the queue, then any pending work will be canceled and lost.
The ``work_available()`` function can be called to determine if the work represented
by the work structure is still in-use.

For the interrupt handling case at hand, the work structure must be pre-allocated
or statically allocated since dynamic allocations are not supported from the
interrupt handling context.

The ``worker`` is the name of the function that will perform the bottom half interrupt
work.
``arg`` is an arbitrary value that the user provides and will be given to the worker
function when it executes.
Normally ``arg`` produces some context in which the work will be performed.
The type of the worker function is given by:

.. code-block:: C

   typedef CODE void (*worker_t)(FAR void *arg);

Where ``arg`` has the same value as was passed to ``work_queue()``.

Processing or work can be delayed in time.
The ``work_queue()`` ``delay`` argument provides that time delay in units of system
clock ticks. However, when used to provide bottom half interrupt processing, the
delay should always be zero.
