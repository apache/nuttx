.. _context-switches:

================
Context Switches
================

Two Types of Context Switches
=============================

There are really two different kinds of context switches.
We refer to them as synchronous and asynchronous context switches (but there
might be better names):

* **Synchronous context switch** occurs when the system is interrupted and,
  because of on actions within the interrupt handler, a context switch is
  generated. In this case, the state of the interrupted task is saved when
  the interrupt handler is entered but a different task state is restored
  when the interrupt handler returns.

* **Synchronous context switch** in our terminology occurs when a task
  explicitly suspends itself by calling some OS interface that causes
  the task to block, such as ``usleep()``.


Synchronous Context Switches
============================

There are two ways to implement a synchronous context switch:
``up_savecontext()`` and ``up_fullcontextrestore()``.

You can implement the moral equivalent of ``setjmp()`` and ``longjmp()``
on steroids. Some architectures have a function called ``up_savecontext()``
that is the moral equivalent of ``setjmp()``, it saves the current state
of the task (and like ``setjmp()`` returns ``0`` or ``1`` to indicate
if the context is being restored.
Another function ``up_fullcontextrestore()`` is like ``longjmp()``,
it restores the context saved by either the interrupt handler during
a previous asynchronous context switch or by the ``up_savecontext()``.

The naming differences ``save`` vs ``fullrestore`` is because when
``up_savecontext()`` is called, it does not need to save all of registers.
Only a subset needs to be saved because the processor ABI provides that some
registers are volatile or caller-saved when ``up_savecontext()`` is called.

The are a couple of downsides to the this approach.
First, the ``up_savecontext()`` and ``up_fullcontextrestore()`` functions
are tricky to write. Second, they have limited usage.
They can be used only in the FLAT build mode where all tasks are running with
the same privileges. If you were to try to do ``up_fullcontextrestore()``
to get from an unprivileged task to a privileged task, you would get an
access violation exception of some sort.

System Calls
============

In order to the limitations of ``up_savecontext()`` and
``up_fullcontextrestore()``, you have to do something a little differently.
One way is to use a system call (a **software interrupt**, a **trap** in x86
or an **SVCALL** in ARM land). This generates a software interrupt and uses
the mechanization of the asynchronous context switch: The software interrupt
saves the context of the old task on entry (replacing the functionality of
``up_savecontext()``) and restores the new task context on return (replacing
the functionality of ``up_fullcontextrestore()``).
The tiny software interrupt handler just sets up the context switch.

The ARMv7-M does synchronous contest switches in this way. You can see how
this is done in the ARMv-7M the SVCALL software interrupt handler.

The advantages of this approach are:

1. It is trivially easy to implement. If interrupt level asynchronous context
   switches work, then so will these synchronous context switches.
2. You an switch between privileged and unprivileged tasks.
   That happens for free when the interrupt returns.

The downside is only that it causes significantly slower synchronous context
switching times. It adds the overhead of interrupt processing and interrupt
IRQ dispatching to each context switch.
I think this is still the correct way to go despite its worse performance.

Several modifications would be required to convert from the first to the
second type of synchronous context switches:

1. Creation of the software handlers for the context switch.
2. Converting all of the back-to-back calls to ``up_savecontext()`` and
   ``up_restorefullcontext()`` to a single call to a function that executes
   the software interrupt, usually something like ``up_switchcontext()``.
