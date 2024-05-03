=================
Critical Sections
=================

Types and Effects of Critical Sections
======================================

A critical section is a short sequence of code where exclusive execution is
assured by globally disabling other activities while that code sequence executes.
When we discuss critical sections here we really refer to one of two mechanisms:

* **Critical Section proper** A critical section is established by calling
  ``enter_critical_section()``; the code sequence exits the critical section by
  calling ``leave_critical_section()``. For the single CPU case, this amounts to
  simply disabling interrupts but is more complex in the SMP case where spinlocks
  are also involved.

* **Disabling Pre-emption** This is a related mechanism that is lumped into this
  discussion because of the similarity of its effects on the system. When pre-emption
  is disabled (via ``sched_lock()``), interrupts remain enabled, but context switches
  may not occur; the current task is locked in place and cannot be suspended until
  the scheduler is unlocked (via ``sched_unlock()``).

The use of either mechanism will always harm real-time performance.
The effects of critical sections on real-time performance is discussed in
:doc:`/implementation/preemption_latency`.
The end result is that a certain amount of **jitter** is added to the real-time response.

Critical sections cannot be avoided within the OS and, as a consequence, a certain
amount of "jitter" in the response time is expected. The important thing is to monitor
the maximum time that critical sections are in place in order to manage that jitter so
that the variability in response time is within an acceptable range.

NOTE: This discussion applies to Normal interrupt processing. Most of this discussion
does not apply to :doc:`/guides/zerolatencyinterrupts`. Those interrupts are not masked
in the same fashion and none of the issues address in this page apply to those
interrupts.

Single CPU Critical Sections
============================

OS Interfaces
-------------

Before we talk about SMP Critical Sections let's first review the internal OS
interfaces available and what they do in the single CPU case:

* ``up_irq_save()`` (and its companion, ``up_irq_restore()``). These simple
  interfaces just enable and disable interrupts globally. This is the simplest
  way to establish a critical section in the single CPU case. It does have
  side-effects to real-time behavior as discussed elsewhere.

* ``up_irq_save()`` should never be called directly, however. Instead, the wrapper
  macros enter_critical_section() (and its companion ``leave_critical_section()``)
  or ``spin_lock_irqsave()`` (and ``spin_unlock_irqrestore()``) should be used.
  In the single CPU case, these macros are defined to be simply ``up_irq_save()``
  (or ``up_irq_save()``). Rather than being called directly, they should always
  be called indirectly through these macros so that the code will function in the
  SMP environment as well.

* Finally, there is ``sched_lock()`` (and ``sched_unlock()``) that disable (and
  enable) pre-emption. That is, ``sched_lock()`` will lock your kernel thread in
  place and prevent other tasks from running. Interrupts are still enabled, but
  other tasks cannot run.


Using sched_lock() for Critical Sections – **DON'T**
----------------------------------------------------

In the single CPU case, ``sched_lock()`` can do a pretty good job of establishing a
critical section too. After all, if no other tasks can run on the single CPU,
then that task has pretty much exclusive access to all resources (provided that
those resources are not shared with interrupt handlers). However, ``sched_lock()``
must never be used to establish a critical section because it does not work the
same way in the SMP case. In the SMP case, locking the scheduer does not provide
any kind of exclusive access to resources. Tasks running on other CPUs are still
free to do whatever they wish.

SMP Critical Sections
=====================

``up_irq_save()`` and ``up_irq_restore()``
------------------------------------------

As mentioned, ``up_irq_save()`` and ``up_irq_restore()`` should never be called
directly. That is because the behavior is different in multiple CPU systems. In
the multiple CPU case, these functions only enable (or disable) interrupts on the
local CPU. They have no effect on interrupts in the other CPUs and hence really
accomplish very little. Certainly they do not provide a critical section in any
sense.

``enter_critical_section()`` and ``leave_critical_section()``
-------------------------------------------------------------

**spinlocks**

In order to establish a critical section, we also need to employ spinlocks. Spins
locks are simply loops that execute in one processor. If processor A sets spinlock
x, then processor B would have to wait for the spinlock like:

.. code-block:: C

  while (test_and_set(x))
   {
   }

Where test and set is an atomic operation that sets the value of a memory location
but also returns its previous value. Here we are talking about atomic in terms of
memory bus operations: The testing and setting of the memory location must be atomic
with respect to other bus operations. Special hardware support of some kind is
necessary to implement ``test_and_set()`` logic.

When Task A released the lock x, Task B will successfully take the spinlock and
continue.

**Implementation**

Without going into the details of the implementation of ``enter_critical_section()``
suffice it to say that it (1) disables interrupts on the local CPU and (2) uses
spinlocks to assure exclusive access to a code sequence across all CPUs.

NOTE that a critical section is indeed created: While within the critical section,
the code does have exclusive access to the resource being protected. However the
behavior is really very different:

* In the single CPU case, disable interrupts stops all possible activity from any
  other task. The single CPU becomes single threaded and un-interruptible.
* In the SMP case, tasks continue to run on other CPUs. It is only when those other
  tasks attempt to enter a code sequence protected by the critical section that those
  tasks on other CPUs will be stopped. They will be stopped waiting on a spinlock.

``spin_lock_irqsave()`` and ``spin_unlock_irqrestore()``
--------------------------------------------------------

**Generic Interrupt Controller (GIC)**

ARM provides a special, optional sub-system called MPCore that provides
multi-core support. One MPCore component is the Generic Interrupt Controller
or GIC. The GIC supports 16 inter-processor interrupts and is a key component for
implementing SMP on those platforms. The are called Software Generated Interrupts
or SGIs.

One odd behavior of the GIC is that the SGIs cannot be disabled (at least not
using the standard ARM global interrupt disable logic). So disabling local
interrupts does not prevent these GIC interrupts.

This causes numerous complexities and significant overhead in establishing a
critical section.

**ARMv7-M NVIC**

The GIC is available in all recent ARM architectures. However, most embedded
ARM7-M multi-core CPUs just incorporate the inter-processor interrupts as a
normal interrupt that is mask-able via the NVIC (each CPU will have its own NVIC).

This means in those cases, the critical section logic can be greatly simplified.

**Implementation**

For the case of the GIC with no support for disabling interrupts,
``spin_lock_irqsave()`` and ``spin_unlock_irqstore()`` are equivalent to
``enter_critical_section()`` and ``leave_critical_section()``. In is only in the
case where inter-processor interrupts can be disabled that there is a difference.

In that case, ``spin_lock_irqsave()`` will disable local interrupts and take
a spinlock. This is really very simple and efficient implementation of a critical
section.

There are two important things to note, however:

* The logic within this critical section must never suspend! For example, if
  code were to call ``spin_lock_irqsave()`` then ``sleep()``, then the sleep
  would occur with the spinlock in the lock state and the whole system could
  be blocked. Rather, ``spin_lock_irqsave()`` can only be used with straight
  line code.

* This is a different critical section than the one established via
  ``enter_critical_section()``. Taking one critical section, does not prevent
  logic on another CPU from taking the other critical section and the result
  is that you make not have the protection that you think you have.

``sched_lock()`` and ``sched_unlock()``
---------------------------------------

Other than some details, the SMP ``sched_lock()`` works much like it does in
the single CPU case. Here are the caveats:

* As in the single CPU case, the case that calls ``sched_lock()`` is locked
  in place and cannot be suspected.

* However, tasks will continue to run on other CPUs so ``sched_lock()`` cannot
  be used as a critical section.

* Tasks on other CPUs are also locked in place. However, they may opt to suspend
  themselves at any time (say, via ``sleep()``). In that case, only the CPU's
  IDLE task will be permitted to run.

The Critical Section Monitor
============================

Internal OS Hooks
-----------------

**The Critical Section Monitor**

In order to measure the time that tasks hold critical sections, the OS supports
a Critical Section Monitor. This is internal instrumentation that records the
time that a task holds a critical section. It also records the amount of time
that interrupts are disabled globally. The Critical Section Monitor then retains
the maximum time that the critical section is in place, both per-task and globally.

The Critical Section Monitor is enabled with the following setting in the
configuration::

  CONFIG_SCHED_CRITMONITOR=y

**Perf Timers interface**

.. todo:: missing description for perf_xxx interface

**Per Thread and Global Critical Sections**

In NuttX critical sections are controlled on a per-task basis. For example,
consider the following code sequence:

.. code-block:: C

   irqstate_t flags = enter_critical_section();
   sleep(5);
   leave_critical_section(flags);

The task, say Task A, establishes the critical section with
``enter_critical_section()``, but when Task A is suspended by the ``sleep(5)``
statement, it relinquishes the critical section. The state of the system will
then be determined by the next task to be resumed, say Task B: Typically, the
next task will not be in a critical section and so the critical section is
broken while the task sleeps. That critical section will be re-established when
that Task A runs again after the sleep time expires.

However, if Task B that is resumed is also within a critical section, then the
critical section will be extended even longer! This is why the global time that
the critical section in place may be longer than any time that an individual
thread holds the critical section.

ProcFS
------

The OS reports these maximum times via the ProcFS file system, typically
mounted at ``/proc``:

* The ``/proc/<ID>/critmon`` pseudo-file reports the per-thread maximum value
  for thread ID = <ID>. There is one instance of this critmon file for each
  active task in the system.

* The ``/proc/critmon`` pseuo-file reports similar information for the global
  state of the CPU.

The form of the output from the ``/proc/<ID>/critmon`` file is::

  X.XXXXXXXXX,X.XXXXXXXXX

Where ``X.XXXXXXXXX`` is the time in seconds with nanosecond precision
(but not necessarily accuracy, accuracy is dependent on the timing clock
source). The first number is the maximum time that the held pre-emption
disabled; the second number number is the longest duration that the critical
section was held.

This file cat be read from NSH like:

.. code-block:: bash

   nsh> cat /proc/1/critmon
   0.000009610,0.000001165

The form of the output from the ``/proc/critmon`` file is similar::

  X,X.XXXXXXXXX,X.XXXXXXXXX

Where the first X is the CPU number and the following two numbers have the
same interpretation as for ``/proc/<ID>/critmon``. In the single CPU case,
there will be one line in the pseudo-file with ``X=0``; in the SMP case
there will be multiple lines, one for each CPU.

This file can also be read from NSH:

.. code-block:: bash

   nsh> cat /proc/critmon
   0,0.000009902,0.000023590

These statistics are cleared each time that the pseudo-file is read so that
the reported values are the maximum since the last time that the ProcFS pseudo
file was read.

``apps/system/critmon``
-----------------------

Also available is a application daemon at ``apps/system/critmon``. This daemon
periodically reads the ProcFS files described above and dumps the output to
stdout. This daemon is enabled with:

.. code-block:: bash

   nsh> critmon_start
   Csection Monitor: Started: 3
   Csection Monitor: Running: 3
   nsh>
   PRE-EMPTION CSECTION    PID   DESCRIPTION
   MAX DISABLE MAX TIME
   0.000100767 0.000005242  ---  CPU 0
   0.000000292 0.000023590     0 Idle Task
   0.000036696 0.000004078     1 init
   0.000000000 0.000014562     3 Csection Monitor
   ...

And can be stopped with:

.. code-block:: bash

   nsh> critmon_stop
   Csection Monitor: Stopping: 3
   Csection Monitor: Stopped: 3

IRQ Monitor and Worst Case Response Time
========================================

The IRQ Monitor is additional OS instrumentation. A full discusssion of the
IRQ Monitor is beyond the scope of this page. Suffice it to say:

* The IRQ Monitor is enabled with ``CONFIG_SCHED_IRQMONITOR=y``.

* The data collected by the IRQ Monitor is provided in ``/proc/irqs``.

* This data can also be viewed using the ``nsh> irqinfo`` command.

* This data includes the number of interrupts received for each IRQ and the
  time required to process the interrupt, from entry into the attached
  interrupt handler until exit from the interrupt handler.

From this information we can calculate the worst case response time from
interrupt request until a task runs that can process the the interrupt.
That worst cast response time, ``Tresp``, is given by:

* ``Tresp1 = Tcrit + Tintr + C1``

* ``Tresp2 = Tintr + Tpreempt + C2``

* ``Tresp = MAX(Tresp1, Tresp2)``

Where:

* ``C1`` and ``C2`` are unknown, irreducible constants that reflect such things as
  hardware interrupt latency and context switching time,

* ``Tcrit`` is the longest observed time within a critical section,

* ``Tintr`` is the time required for interrupt handler execution for the event
  of interest, and

* ``Tpreempt`` is the longest observed time with preemption disabled.

NOTES:

#. This calculation assumes that the task of interest is the highest priority task
   in the system. It does not consider the possibility of the responding task being
   delayed due to insufficient priority.

#. This calculation does not address the case where the interfering task has both
   preemption disabled and holds the critical section. Certainly Tresp1 is valid
   in this case, but Tresp2 is not. There might some additional, unmeasured delay
   after the interrupt and before the responding task can run depending on the order
   in which the critical section is released and preemption is re-enabled:

     * When the task leaves the critical section, the pending interrupt will execute
       immediately with or without preemption enabled.

     * If preemption is enabled first, then the will be no delay after the interrupt
       because preemption will be enabled when the interrupt returns.

     * If the task leaves critical section first, then there will be some small delay
       of unknown duration after the interrupts returns and before the responding
       task can run because preemption will be disabled when the interrupt returns.

#. This calculation does not address concurrent interrupts. All interrupts run at the
   same priority and if an interrupt request occurs while within an interrupt handler,
   then it must pend until completion of that interrupt. So perhaps the above formula
   for ``Tresp1`` should instead be the following? (This assumes that hardware arbitration
   is such that the interrupt of interest will be deferred by no more than one interrupt).
   Concurrent, nested interrupts might be better supported with prioritized.
   See more: :doc:`/guides/nestedinterrupts`.

     * ``Tresp1 = Tcrit + Tintrmax + Tintr + C1``

       Where:

       * ``Tintrmax`` is the longest interrupt processing time of all interrupt sources
         (excluding the interrupt for the event under consideration).

What can you do?
----------------

What can you do if the timing data indicates that you cannot meet your deadline?
You have these options:

#. Use these tools to find the exact function that holds the critical section or
   disables preemption too long. Then optimize that function so that it releases
   that resource sooner. Often critical sections are established over long sequences
   or code when they could be re-designed to use critical sections over shorter code
   sequences.

#. In some cases, use of critical sections or disabling of pre-emption could be replaced
   with a locking semaphore. The scope of the locking effect for the use of such locks
   is not global but is limited only to tasks that share the same resource. Critical
   sections should correctly be used only to protect resources that are shared between
   tasking level logic and interrupt level logic.

#. Switch to :doc:`/guides/zerolatencyinterrupts`. Those interrupts are not subject
   to most of the issues discussed in this page.

**NOTE**

There are a few places in the OS were preemption is disabled via ``sched_lock()`` in
order to establish a critical section. That is an incorrect use of ``sched_lock()``.
``sched_lock()`` simply prevents the currently executing task from being suspended.
For the case of the single CPU platform, that does effectively create a critical
section: Since no other task can run, the locking task does have exclusive access
to all resources that are not shared with interrupt level logic.

But in the multi-CPU SMP case that is not true. ``sched_lock()`` still keeps the
current task running on CPU from being suspended, but it does not support any
exclusivity in accesses because there will be other tasks running on other CPUs
that may access the same resources.
