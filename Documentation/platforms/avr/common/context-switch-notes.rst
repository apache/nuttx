===========================
Notes on AVR context switch
===========================

This document describes the ways and circumstances in which context
switches happen in AVR MCUs.

Used terms and context switch basics
====================================

Context creation
----------------

There are two ways context is created when a task is suspended.
Either the task is suspended in response to a hardware interrupt
(context created ``in-interrupt`` in the following text), or it is
suspended voluntarily, eg. by calling sleep(), read() etc.
(``in-task`` context.)

The resulting context is identical and interchangeable with two differences:

  - ``SREG`` - ``in-interrupt`` context has global interrupt enabled ("I"-flag) set
  - position in the program where the task resumes running (arbitrary point for ``in-interrupt`` vs. inside ``up_switch_context()`` for ``in-task``

Task resumption
---------------

Task can be resumed in two corresponding situations - context switch
in response to a hardware interrupt (``by-interrupt``) or in response
to other task relinquishing the CPU (``by-task``)

Context switch
==============

Two ways of context creation combined with two ways of task resumption
give 4 possibilities of context switch process, two of which are
not interesting:

1st combination
---------------

``in-task`` context resumed in ``by-task`` context switch. Context
to be resumed has "I" flag cleared and SREG is restored with that flag cleared.
The ``ret`` instruction is used to resume the task,"returning" to the point
where it gave the CPU up, which is inside ``up_switch_context()``.

This function is supposed to be executed with interrupt disabled ("This
function is called only from the NuttX scheduling logic. Interrupts
will always be disabled when this function is
called." https://nuttx.apache.org/docs/latest/reference/os/arch.html ) Caller
of the context switch method is therefore responsible
for re-enabling interrupts.

2nd
---

``in-interrupt`` resumed in ``by-interrupt`` context switch. The task
essentially (from its point of view) exits interrupt handler after
it entered it. Instruction ``reti`` is used to return from the handler,
setting "I"-flag in the process.

3rd
---

Third and fourth combinations are more interesting:

``in-task`` context resumed in ``by-interrupt``. The CPU enters
ISR and regular program flow requires returning from ISR and setting
"I"-flag by ``reti`` which does not happen. Task is resumed with
interrupts disabled. However, it is resumed inside ``up_switch_context()``
and caller of that function will set the "I"-flag at some point.
Task then runs with interrupts enabled, all is well.

4th
---

``in-interrupt`` resumed in ``by-task``, ie. in ``up_switch_context()``.
``reti`` is used to resume the task, setting "I"-flag in the process.
This would be incorrect for ``up_switch_context()`` - it is supposed
to run with interrupts disabled - but the task resumes running
from the point where it was interrupted, which is not inside
of ``up_switch_context()``. All is well.


AVRDx core considerations
=========================

Now, all of the above holds true for eg. ATmega chips which control
interrupt execution solely by the "I"-flag, allowing the code
to not care about where the context switch was triggered. Regardless
of that, the MCU will always end up in correct state even if the
context switch cause doesn't match the context being restored
(cases 3 and 4.)

This is not the case for AVR Dx family which behaves differently.
The interrupt controller does respect the "I"-flag in a sense where
it considers interrupts disabled when the flag is cleared. However,
it is possible that interrupts are not enabled when the flag is set.
That depends on a logical AND between "I"-flag and "interrupt handler
is not executing" internal state. (Refer to the documentation
for more precise explanation.)

What this means is that if eg. ``in-task`` context gets
resumed in ``by-interrupt`` condition (case 3 above), then ``ret``
instruction is used
to resume the task. As discussed above, the "I"-flag is not set this
way but that is not a problem, it is eventually set later. However,
the internal state "running the interrupt handler" is not cleared.
This means that the task keeps running with "global interrupts are
enabled" but is actually unable to be interrupted. The context switch
code needs to handle this.

Conversely, there is a similar problem with ``in-interrupt`` context
being resumed in ``by-task`` (case 4). Instruction ``reti`` is used
but there is no internal state to be cleared. Unlike the previous case,
no problem related to this was observed but it still looks like something
the code wants to avoid. It could trigger all sorts of undefined behaviour
in the chip otherwise.
