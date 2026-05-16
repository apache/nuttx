.. _smp:

===============================
SMP (Symmetric MultiProcessing)
===============================

Definition
==========

According to Wikipedia:

  "Symmetric multiprocessing (SMP) involves a symmetric
  multiprocessor system hardware and software architecture where two or more
  identical processors connect to a single, shared main memory, have full access
  to all I/O devices, and are controlled by a single operating system instance
  that treats all processors equally, reserving none for special purposes.
  Most multiprocessor systems today use an SMP architecture.
  In the case of multi-core processors, the SMP architecture applies to
  the cores, treating them as separate processors.

  (..)

  SMP systems are tightly coupled multiprocessor systems with a pool
  of homogeneous processors running independently, each processor executing
  different programs and working on different data and with capability
  of sharing common resources (memory, I/O device, interrupt system and so on)
  and connected using a system bus or a crossbar."

  -- Source: https://en.wikipedia.org/wiki/Symmetric_multiprocessing.


Development Status
==================

SMP support is complete and stable in NuttX on several multi-core platforms.


Enabling SMP
============

SMP can be enabled on NuttX with the following configuration settings:

* ``CONFIG_SMP`` - Enables support for Symmetric Multi-Processing (SMP)
  on a multi-CPU platform.
* ``CONFIG_SMP_NCPUS`` - This value identifies the number of CPUs support
  by the processor that will be used for SMP.
* ``CONFIG_SMP_IDLETHREAD_STACKSIZE`` - Each CPU will have its own IDLE task.
  System initialization occurs on CPU0 and uses
  ``CONFIG_IDLETHREAD_STACKSIZE``.
  This setting provides the stack size for the IDLE task on CPUS 1
  through ``(CONFIG_SMP_NCPUS-1)``.

This section provides the origin design specification for the implemention.
As a result, you may find that the test uses future and conditional tenses
when describing the implementation of SMP on NuttX.

This design has been maintained and now reflects the current "as-built"
state of SMP in NuttX.


Design Requirements
===================

The basic design requirements are pretty simple:

1. Need to be able to bring up NuttX running on multiple CPUs.
2. Need data structures to manage multiple active tasks.
3. Need to be able to schedule tasks on other CPUs.
4. Need to be able to modify tasks running on other CPUs.
5. Need to be able to manage critical sections on all CPUs.
6. Need spinlocks to block on all CPUs in all cases: semaphore,
   signal, message queue, etc.
7. Need to understand how some non-standard NuttX operations things
   like disabling pre-emption work.


Data Structures
===============

Task Lists
----------

At the core of the NuttX design are data structures called
**Task Control Blocks** or just **TCB**.

These data structures contain everything-you-need-to-know about
a thread or task.
These TCBs are retained in lists within the RTOS.
The state of a thread or task is then determined by which list
the TCB resides in.

The Read-To-Run Task List
-------------------------

On such TCB list is of particular importance in the implementation of SMP.
That is the so-called ready-to-run list, ``g_readytorun``.
That list contains the TCB of every task or thread that is not blocked
in any way and so is, well, ready to run.

The ``g_readytorun`` is a prioritized list. The lowest priority task
is in the list is the one at the end of the list and that must always
by the IDLE task.

That is the only task/thread that is permitted to have priority 0.
The highest priority, read-to-run task is always at the head of
``g_readytorun`` and must be the currently executing task.

All other tasks after this is eligle to run, but not currently running.

The Assigned Task List
----------------------

In order to support SMP, the function of the ``g_readytorun`` list
must change. This ``g_readytorun`` should still exist but it should
now contain only:

1. Only tasks/threads that are eligible to run, but not currently running, AND
2. Tasks/threads that have not been assigned to a CPU.

For SMP support there should be an array of assigned tasks like:

.. code-block:: c

    volatile dq_queue_t g_assignedtasks[CONFIG_SMP_NCPUS];

Where ``CONFIG_SMP_NCPUS`` is the configured number of CPUs supported
by the processors. As its name suggests, on ``g_assignedtasks`` queue for
``CPU n`` would contain only tasks/threads that are assigned to CPU n.
Threads would be assigned a particular CPU by one of two mechanisms:

1. (Semi-)permanently through an RTOS interfaces such as
   ``pthread_attr_setaffinity()``, OR 
2. Temporarily through new scheduling logic.

Tasks/threads that are assigned to a CPU via an interface like
``pthread_attr_setaffinity()`` would never go into the ``g_readytorun`` list,
but would only go into the ``g_assignedtasks[n]`` list for the CPU n to which
the thread has been assigned.
Hence, the ``g_readytorun`` list would hold only unassigned tasks/threads.

An indication within the TCB would indicated whether or not a task/thread
is assigned to a CPU and, if so, which CPU it is assigned to.

Scheduling logic would temporarily assign a task or thread to a CPU.
The assignment is only temporary because state data in the TCB would indicate
that the task is unassigned when, hence, it could be returned
to the ``g_readytorun`` list later.

The assigned tasks lists lists would be prioritized.
The highest priority task, and the one currently executing on CPU n would be
the one at the head of ``g_assignedtasks[n]``.
Tasks after the active task are ready-to-run and assigned to this CPU.
The tail of this assigned task list, the lowest priority task,
is always the CPU's IDLE task.

The CPU n scheduling logic would execute whenever the currently running task
is removed from the head of ``g_assignedtasks[n]``.
The algorithm might be something like:

.. code-block:: c

  /* Is the assigned task list for the CPU empty? */
    
  if (g_assignedtasks[cpu].head == NULL)
    {
      /* No.. Is the task at the head of the assigned list for the CPU lower
       * in priority that the current (unassigned) task at the head of the
       * ready-to-run list?
       */
    
      FAR struct tcb_s *rtcb = (FAR struct tcb_s *)g_readytorun.head ;
      FAR struct tcb_s *atcb = (FAR struct tcb_s *)g_assignedtasks[cpu].head;
      if (atcb->sched_priority < rtcb->sched_priority)
        {
          /* Remove the TCB from the head of the g_readytorun list. */
    
          /* Add that TCB to the g_assignedtasks[cpu] list (it will go at the
           * head of the list).
           */
        }
    
      /* Now activate the task at the head of the g_assignedtasks[cpu] list on
       * the CPU.
       */
    
    }

The Current Task
----------------

There is a lot of logic in the RTOS now that obtains the TCB for the currently
excuting task by examining the head of the ``g_readytorun`` list.
You will see this assignment in many places, both in the core OS logic
in ``nuttx/sched`` but also in architecture-specific logic under
``nuttx/arch``:

.. code-block:: c

  FAR struct tcb_s *rtcb = this_task();

Where ``this_task()`` is a macro defined in ``nuttx/sched/sched.h``
and expands as follows:

.. code-block:: c

  #define current_task(cpu)  ((FAR struct tcb_s *)g_readytorun.head)
  #define this_cpu()         (0)
  #define this_task()        (current_task(this_cpu))

Of course, that would not work with the proposed changes.
We would need to then get the TCB of the currently executing task/thread
for CPU n from the head of ``g_assignedtasks[n]``.
I would propose a replacing the above assignment with a macro like
``current_task()`` where that macro might expand to:

.. code-block:: c

  #ifdef CONFIG_SMP
  #  define current_task(cpu)  ((FAR struct tcb_s *)g_assignedtasks[cpu].head)
  #  define this_cpu()         up_cpu_index()
  #else
  #  define current_task(cpu)  ((FAR struct tcb_s *)g_readytorun.head)
  #  define this_cpu()         (0)
  #endif
  #define this_task()          (current_task(this_cpu))

where ``up_cpu_index()`` is some new MCU specific interface that will
return an index associated with the currently active CPU.

.. note::

  This is a two step operations: Step 1. Get the CPU number and
  Step 2: Use the CPU number as an index into the
  ``g_assignedtasks[]`` array of lists. **This must be atomic!**
  The schedule should be locked to assure that the task
  is not suspended after fetching the CPU number then restarted
  on a different CPU to access the ``g_assignedtasks[]`` array
  of lists.

The IDLE Task
-------------

Without SMP, the ``g_readytorun`` list always ends with the TCB of IDLE task.

It is always guaranteed to be at the end of the list because the list
is prioritized and because the IDLE task has an impossibly low priority
that no other task/thread could have.

The IDLE task is necessary because it gives the CPU something to execute
when there is nothing else to be done.

But with SMP, there are multiple CPUs that need something to do when there
is nothing else to do. We are tentatively thinking that each CPU needs its own
IDLE thread whose TCB would reside at the end of each
``g_assignedtasks[cpu]`` list. But that does feel wasteful

I am not certain the mechanism as of this writing, but I assume that
the ``nx_start()`` initialization logic would need to create an IDLE task
for each CPU and assign each IDLE task to each CPU.

CPU Index
---------

In order to access arrays indexed by a CPU ID value, some method must be
generated to provide the CPU ID that the currently executing task
is running on. To provide this index value, an interface
``up_cpu_index()`` is proposed.

For ARM, the implementation of ``up_cpu_index()`` can be accomplished
by reading the CP15 Multiprocessor Affinity Register (MPDIR).
That register has a 2 bit field index provides exactly the index that
we need for the SMP implementation.

Looking at how Linux does this, Linux uses an interface called ``get_cpu()``
which is analogous to the proposed ``up_cpu_index()``.
``get_cpu()`` maps to ``smp_processor_id()`` and if debug options are
not enabled, this further maps to ``raw_smp_procesor_id()``.
For the case of ARM, this maps to ``(current_thread_info()->cpu)``
where ``current_thread_info()`` is a location at the far end of the
allocated stack:
``(current_stack_pointer & ~(THREAD_SIZE-1))`` and
``THREAD_SIZE`` is ``(PAGE_SIZE << THREAD_SIZE_ORDER)``.

So, to make that long story short, Linux solves the problem by putting
some magic information at the base of far end of each stack when
a context switch occurs (and when the CPU is also known).
That magic information can then just be recovered using the thread's
stack pointer at any time. This is part of the basic implementation
of Thread Local Storage (TLS) in Linux.

Something similar could be done with NuttX and would require:

1. Special aligned stack allocation,
2. Logic to write the CPU index into the stack when each thread
   is [re-]started.

This would also place an upper limit on the size of the stack:
If we are going to find the far end of the stack by simply ANDing out
the lower bits, then size of that mask would also determine
the maximum size of the stack.

However, I believe that using the information from the MPIDR register
is a better general solution. Counter-arguments are:

1. There may be some architectures that do not have such a simple mechanism.
2. TLS has value in any event.
3. The stack-based TLS is in user-accessible memory and could be used
   by applications in protected and kernel builds.


System Startup
==============

I assume that initially, only one CPU is active.
System initialization would then occur on that single thread.
At the completion of the initialization of the OS, just before beginning
normal multitasking, the additional CPUs would be started.

Each CPU would be provided the entry point to is IDLE task when started.
Perhaps the MCU interface would be something like:

.. code-block:: c

    int up_cpu_start(int cpu, main_t idletask);

The OS initialization logic would call this function repeatedly
until each CPU is started.


Scheduler Interactions
======================

In the general case, the scheduler should have full control over the current
state of all tasks. It must make that that if there are N CPUs that the top N
highest priority tasks are running.

Srict priority scheduling is the requirement, but perhaps the scheduling logic
could do some load balancing to distribute work as evenly as possible
over the CPUs. When a new task or thread becomes ready to run,
the scheduler must include some heuristics for assigning that task to a CPU
to achieve some optimal performance.

There are complications to how one CPU controls the tasks already running
on another CPU. To determine a task should run, you would need to be able to:

* Keep the task data structures stable while they are being analyzed.
* Find the lowest priority running task which could be on any CPU.
* If that priority is lower than the priority task, then replace it with
  the new task at the head of the ``g_assignedtasks[]`` list.
* If not, find the task with the next lowest priority and compare that one.
* Continue until until the new task is assigned to a CPU or until
  it is determined that all of the currently running tasks are higher priority
  than the new task. In that base, the new task should be added
  to the ``g_readytorun`` list.

To support this behavior, I think that the following new MCU interfaces
will be needed:

.. code-block:: c

    int up_cpu_pause(int cpu);

Which would stop execution on CPU0, saving the state of the currently running
task so that it may be resumed. And:

.. code-block:: c

    int up_cpu_resume(int cpu);

Restart the CPU with the task at the head of the ``g_assignedtasks[]`` list.

.. note::

  Please also note the the "Signal Handling" paragraph below.
  The same issue exists for dispatching signals to threads actively
  running on another CPU.


Interrupt Handling
==================

Per-CPU Interrupts
------------------

How will interrupts be taken? On one CPU or on multiple CPUs?

This may work different on different hardware platforms.
This design requires only that:

* If the processor supports interrupts on only one CPU, then interrupts
  cannot be nested; further interrupts must be disabled while that interrupt
  handler runs (see Nested Interrupts and High Priority,
  Zero Latency Interrupts.).
* If the process supports device interrupts on multiple CPUs, the interrupt
  handling on the CPUs is not concurrent: When interrupts are disabled
  on one CPU, they are disabled on all CPUs (unless, of course, if interrupts
  are needed for inter-CPU communication).

However, I do not know of any CPU architecture that supports disabling
interrupts on one CPU from another CPU.
Instead, critical sections will need to be supported via spinlocks
as described below.

If interrupts can be taken by multiple CPUs then any data structures used
for interrupt handling would also need to become and array indexed by the
CPU number. Most architectures current use a data structure defined like:

.. code-block:: c

    volatile uint32_t *g_current_regs;

Which would have to become an array like:

.. code-block:: c

    volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];


System Calls
============

System Calls are normally implemented via software interrupts.

The System Call software interrupt should run on the same CPU as does
the logic that generated the System Call or, alternatively,
the design must have some way of obtaining the index of the CPU
that generated the System Call.


Critical Sections
=================

A critical section is a set of statements that must be able to execute
exclusively. Higher level applications will, of course, use OS application
interfaces such as ``sem_wait()`` and ``sem_post()`` to manage critical
sections. But within the OS, for example, in the low level implementation
of ``sem_wait()`` and ``sem_post()``, more primitive, non-standard methods
must be used to implement critical sections.


.. _spinlocks:

Spinlocks
=========

A spinlock is a lock which causes a thread trying to acquire it to simply wait
in a loop (spin) while repeatedly checking if the lock is available.
The thread remains active but is not performing a useful task.

The use of such a lock is a kind of busy waiting and is used commonly
in SMP implementations to manage access to resources by multiple CPUs.

Spinlock Implementation
-----------------------

In a NuttX implementation, the spinlock would probably involve only:

* A memory location with one value, say ``SP_LOCKED``, meaning that the lock
  is taken and another value, ``SP_UNLOCKED``, meaning that the lock
  is available.
* An integer type memory location that contains the number of the CPU
  holding the lock.
* An integer type memory location hold the number of counts on the lock.
* And a loop performs a test-and-set operation: The memory location
  is read by a thread and set to true in one atomic operation.
  If the read value is false, then the thread holds the lock.
  Otherwise, it must loop trying repeatedly until the thread gets the lock.

The meaning of the lock is that CPU holding the lock has exclusive access
to a resource that is shared by multiple CPUs.
So there is never any reason for two threads on the same CPU to spin:
If the CPU already holds the lock, additional threads need simply
only increment the lock count.

* If the test-and-set fails in the logic that is spinning, but if the lock
  is held by the logic that that CPU is running on, then the spin logic
  should simply increment the count of locks (which needs to be atomic only
  for single processor).

Could this cause one CPU to hog too much resource time?
Perhaps, been calls to the test-and-set logic, the spinlock should call
``sched_yield()`` which would at least let other threads
of the same priority run.

* Releasing the lock should be matter of decrementing the lock count
  and if the lock count would decrement to zero, setting the lock value to
  ``SP_UNLOCKED``. This will, of course, allow another thread spinning
  on the lock in a different CPU to take the lock for that CPU.

The following new, internal OS interfaces are proposed:

.. code-block:: c

    void spin_lock(FAR spinlock_t *lock);
    void spin_unlock(FAR spinlock_t *lock);

Where the type ``spinlock_t`` is defined in MCU-specific header files.
These new spinlock interfaces would also use the MCU-specific interface:

.. code-block:: c

    spinlock_t up_testset(FAR spinlock_t *lock);

.. note::

  A thread may take the lock while running on one CPU, but then later
  be assigned to a different CPU, and then release the lock while
  running on that other CPU. Is there a problem in this?
  Yes, probably. One solution might be lock the thread
  to a CPU if it holds the lock?

There is also a risk is that the thread holding the lock will be pre-empted
by the OS scheduler while holding the lock. If this happens, other threads
on other CPUs will be left spinning (repeatedly trying to acquire the lock),
while the thread holding the lock is not making progress towards releasing it.
The result is an indefinite postponement until the thread holding the lock
can finish and release it.

Spinlock logic can be common. However, there must be a unique instance
of that common spinlock logic in each OS operation that requires mutually
exclusive access by a CPU.

Now, what will we do with these spinlocks? Is there really a need for them?
Yes, probably. We will need examine every place in the OS that uses disabling
of pre-emption or disabling of interrupts to prevent other tasks
(and interrupts) from executing.

All of those cases need to be reconsidered and, most likely, protected
with spinlocks.

Let's next examine all of the cases of how resources are managed in NuttX.

Spinlocks in Semaphores, Signals, and Message Queues
----------------------------------------------------

A critical section using ``irqsave()`` and ``irqrestore()`` is already used
in the implementation of these inter-process communications to enforce
a critical section.

One a single CPU system, disabling interrupts will prevent context switches
(by prevent the asynchronous events that could cause a context switch)
and also prevents conflicts with interrupt level processing.

I believe that simply replacing ``irqsave()`` and ``irqrestore()`` with
new proposed functions ``enter_critical_section()`` and
``leave_critical_section()``, as described below under Disabling Interrupts,
should be sufficient.

These proposed functions include a spinlock to assure that they do enforce
a critical section.

Spinlocks and Data Caches
-------------------------

If spinlocks are used in a system with a data cache, then there may be
a problem with cache coherency in some CPU architectures.

When one CPU modifies the spinlock, the changes may not be visible
to another CPU if it does not share the data cache.
That would cause failure in the spinlock logic.

Flushing the D-cache on writes and invalidating before a read
is not a good option. Spinlocks are normally 8-bits in size and cache
lines are typically 32-bytes so that would have side effects unless
the spinlocks were made to be the same size as one cache line.

The better option is to add compiler independent "ornamentation"
to the spinlock so that the spinlocks are all linked together
into a separate, non-cacheable memory regions.
Because of region alignment and minimum region mapping sizes
this could still be wasteful of memory.
This would work in systems that have both data cache and either an MPU
(such as Cortex-m7) or an MMU (such as Cortex-Ax).


Disabling Pre-emption
=====================

Pre-emption is disabled via the interface ``sched_lock()``.
``sched_lock()`` currently works by preventing context switches from the
currently executing tasks.

This prevents other tasks from running (without disabling interrupts)
and gives the currently executing task exclusive access to the (single)
CPU resources.
Thus, ``sched_lock()`` and its companion, ``sched_unlcok()``,
are used to implement some critical sections.

Currnetly, Pre-emption is disabled using a simple lockcount in the TCB.
When the scheduling is locked, the lockcount is incremented;
when the scheduler is unlocked, the lockcount is decremented.
If the lockcount for the task at the head of the ``g_readytorun``
list has a ``lockcount > 0``, then pre-emption is disabled.

No special protection is required since only the executing task
can modify its lockcount.

Certainly, disabling context switches on one CPU would still be possible
in an SMP model, but it may not be possible to give a task exclusive access
to the (multiple) CPU resources without stopping the other CPUs:
Even though pre-emption is disabled, other threads will still be executing
on the other CPUS.

The full dynamics of the behavior of the scheduler logic in this case
is not certain.
However, I think that this would be an acceptable behavior provided that:

* There is a global lock count ``g_cpu_lockset`` that includes a bit
  for each CPU: If the bit is ``1``, then the corresponding CPU has
  the scheduler locked; if ``0``, then the CPU does not have the scheduler
  locked.
* Scheduling logic would set the bit associated with the cpu in
  ``g_cpu_lockset`` when the TCB at the head of the
  ``g_assignedtasks[cpu]`` list transitions has ``lockount > 0``.
  This might happen when ``sched_lock()`` is called, or after
  a context switch that changes the TCB at the head of the
  ``g_assignedtasks[cpu]`` list.
* Similarly, the cpu bit in the global ``g_cpu_lockset`` would be cleared
  when the TCB at the head of the ``g_assignedtasks[cpu]`` list has
  ``lockount == 0``. This might happen when ``sched_unlock()`` is called,
  or after a context switch that changes the TCB at the head of the
  ``g_assignedtasks[cpu]`` list.
* Modification of the global ``g_cpu_lockset`` must be protected
  by a simplified spinlock, ``g_cpu_schedlock``. That spinlock would be
  taken when ``sched_lock()`` is called, and released when ``sched_unlock()``
  is called. This assures that the scheduler does enforce the critical
  section. NOTE: Because of this spinlock, there should never be more
  than one bit set in ``g_cpu_lockset`` attempts to set additional bits
  should be cause the CPU to block on the spinlock. However, additional
  bits could get set in ``g_cpu_lockset`` due to the context switches
  on the various CPUs.
* Each the time the head of a ``g_assignedtasks[]`` list changes
  and the scheduler modifies ``g_cpu_lockset``, it must also set
  ``g_cpu_schedlock`` depending on the new state of ``g_cpu_lockset``.
* Logic that currently uses the currently running tasks lockcount
  should instead use the global ``g_cpu_schedlock``.
  A value of ``SP_UNLOCKED`` would mean that no CPU has pre-emption disabled;
  ``SP_LOCKED`` would mean that at least one CPU has pre-emption disabled.

Disabling pre-emption is a non-standard feature but the general capability
is common to many RTOS. But since feature is non-standard and perhaps
not realizable in the SMP model, another option would be
to simply eliminate it.


Disabling Interrupts
====================

Closely related to disabling pre-emption is the practice of disabling
interrupts to get exclusive access to resources.

Disabling interrupts is not really so different from disabling
pre-emption in practice.
It effectively disables pre-emption by preventing any asynchronous
events that could cause a context switch and, of course, in addition
prevents interrupt level processing.

So disabling of interrupts is also used in places to implement critical
sections and, because the similarity in behavior to disabling only
pre-emption, suffers from the same issues in the SMP environment.

Currently, interrupts on the single CPU are enabled and disabled with:

.. code-block:: c

    irqstate_t irqsave(void);
    void irqrestore(irqstate_t flags);

Those functions disable interrupts on the single CPU.
In the SMP environment, they would need to disable interrupts in all CPUs.

.. note::

  The legacy ``irqsave()`` and ``irqrestore()`` have been replaced
  with new functions implemented in the OS,
  ``enter_critical_section()`` and ``leave_critical_section()``.

These might be implemented as follows (highly simplified):

.. code-block:: c

  spinlock_t g_spu_irqlock = SP_UNLOCKED;
  
  #ifdef CONFIG_SMP
  irqstate_t enter_critical_section(void)
  {
    irqstate_t flags = irqsave();
    
    spinlock(&g_cpu_irqlock);
    g_cpu_irqset |= (1 << cpu);
    
    return flags;
  }
  
  void leave_critical_section(irqstate_t flags)
  {
    g_cpu_irqset &= ~(1 << cpu);
    spinunlock(&g_cpu_irqlock);
    irqrestore(flags);
  }

There is an unhandled complexities in the above simplified logic.
Consider this scenario:

1. The thread calls ``enter_critical_section()``, disabling interrupts
   on all CPUs and taking the spinlock.
2. The thread then suspends, waiting for an event. This is actually
   a very standard behavior to suspend with interrupts disabled:
   The system handles this gracefully be simply re-enabling interrupts
   (if they were enabled by the next task to run).
3. Later, the event occurs, the task is again made ready-to-run,
   and the interrupts are again disabled.

But,

1. There must be additional logic to release the spinlock
   when the task is suspended.
2. There must be additional logic to re-acquire the spinlock
   when the task restarts.
3. Is there any way that the spinlock could already be locked when the task
   restarts? No, I don't think this is possible. If interrupts are disabled
   and the spinlock is locked, then there should be no context switches.

There would be additional complexities if ``enter_critical_section()`` were
called during interrupt handling.

Interrupts are disabled during interrupt level processing, however, interrupt
level logic will attempt to establish critical sections even when
it does not need to do this: It will call ``enter_critical_section()`` anyway
because it will use some common logic with non interrupt level code.

There are many situations in which use of spinlocks as shown
in the simplified example will result in deadlock conditions.

As a result of these complexities, the full implementation of
``enter_critical_section()`` and ``leave_critical_section()`` are considerably
more complex.
See the logic in the file ``sched/irq/irq_csection.c`` if you are
really interested in the details.


Pre-Emption Controls and Critical Sections
==========================================

The effect of disabling pre-emption is to prevent to tasks from running
while on task has disabled pre-emption; the effect of entering a critical
section, on the other hand, is to:

1. Enforce exclusive access to the logic when in the critical section.
2. Keep the system stable while certain operations are performed.
3. Disable competing interrupt level activity when possible.

In order to keep the system stable within in the critical section
it is necessary, the critical section will modify the behavior of the
pre-emption controls.
The basic result is this modification is that new tasks are not permitted
to be started or resumed if:

1. Pre-emption is disabled, OR
2. Some other CPU other than the current CPU is in a critical section.

The CPU that has entered the critical section must have the ability
to start and stop tasks. Attempts to start new tasks from other CPUs when
one CPU is within the critical section is will result in the newly started
task being postponed in a pending task list, ``g_pendingtasks``.

Such pending tasks will only be allowed to run when:

1. All CPUs have re-enabled pre-emption, AND
2. All CPUs have left the critical section.

.. note::

  It can be determined which CPU(s) have the critical section
  by examining ``g_cpu_irqset``.


Signal Handlers
===============

There will be some issues related to how signals are delivered,
at least in regard to how signal handlers are executed.

I am thinking of the case where a signal is sent by a thread running
on one CPU to a thread running on another CPU that has a signal handler
installed. This would probably have to work as follows:

1. Stop the CPU on which the task is running ``using up_cpu_pause()``,
2. Schedule the signal action as is done in the existing logic, then
3. Re-start the CPU with ``up_cpu_resume()`` to resume execution with
   the signal handler.

A special wrapper function for ``up_cpu_pause()`` is provided in the OS
to support this operation:

.. code-block:: c

  int sched_tcb_pause(FAR struct tcb_s *tcb);

This function checks if the task associated with tcb is running on another CPU
and, if so, conditionally calls ``up_cpu_pause()`` to pause execution
on that CPU. It returns the CPU index of the paused CPU (or a negated
``errno`` value if no CPU was paused). While the CPU is paused, operations
can be performed on the data structures associated with the task.
Then the non-negative CPU index can then be used with
``up_cpu_resume()`` to restart the paused CPU.

This same sequence would have to be followed for other functions
that might need to modify the behavior of a running task such as
``task_delete()`` or ``task_restart()``.


Thread Affinity
===============

By default, a thread may run on any CPU.
There are some semi-standard interfaces that can be used to restrict
the set of CPUs that a thread may run on.
This set of CPUs is referred to as the threads affinity mask.
Semi-standard meaning used on Linux and available in ``GLIBC`` when
``__GNU_SOURCE`` is defined.

There are interfaces to set and get the affinity mask for a task prototyped
in ``sched.h``.

``cpusetsize`` is fixed in NuttX and must be equal to ``sizeof(cpu_set_t)``:

.. code-block:: c

  #ifdef CONFIG_SMP
  int sched_setaffinity(pid_t pid, size_t cpusetsize,
                        FAR const cpu_set_t *mask);
  int sched_getaffinity(pid_t pid, size_t cpusetsize, FAR cpu_set_t *mask);
  #endif

There are similar interfaces for a ``pthread`` prototyped in ``phtread.h``:

.. code-block:: c

  #ifdef CONFIG_SMP
  int pthread_setaffinity_np(pthread_t thread, size_t cpusetsize,
                             FAR const cpu_set_t *cpuset);
  int pthread_getaffinity_np(pthread_t thread, size_t cpusetsize,
                             FAR cpu_set_t *cpuset);
  #endif

.. note::

  The ``_np`` in the naming is to remind you that
  this **interface is non-POSIX**!

By default, a child task or pthread inherits the affinity mask of its parent.
The thread affinity mask for a ``pthread``, however, can also be set before
the thread is started via ``pthread_create()``:

.. code-block:: c

  #ifdef CONFIG_SMP

  int pthread_attr_setaffinity_np(FAR pthread_attr_t *attr,
                                  size_t cpusetsize,
                                  FAR const cpu_set_t *cpuset);

  int pthread_attr_getaffinity_np(FAR const pthread_attr_t *attr,
                                  size_t cpusetsize, cpu_set_t *cpuset);

  #endif

In addition, macros are defined in the header file ``include/sched.h``
to abstract operations are CPU sets.
There are several such macros with names like ``CPU_ZERO()``, ``CPU_SET()``,
``CPU_CLR()``, etc.
