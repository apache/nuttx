=============================
Per-Thread Interrupt Controls
=============================

Using NuttX, you will find that the interrupts enabled/disabled state is not a
global property. You can not just turn interrupts off and on for all tasks.
Rather, enabling and disabling interrupts effects only while the single task
that is controlling the interrupts runs. Consider the following sequence:

.. code-block:: C

   irqstate_t flags;
 
   flags = irqsave();   /* Disable interrupts */
   sleep(5);            /* Sleep for 5 seconds */
   irqrestore(flags);   /* Re-enable interrupts */

What happens while the task sleeps? Does that mean that interrupts will be
disabled for five seconds? No, interrupts will (probably) be re-enabled while
the task is sleeping. How does this work?

It is really very simple. Each time a context switches occurs, a set of
registers are saved for the task that is being suspended. Then those registers
are restored from the previously saved registers for a next task that will run.
This is why we often describe a context switch as just setjmp/longjmp on steroids:
A context switch works just like setjmp (save a set of registers) and longjmp
(restore a set of registers), except that more registers are saved and restored.


For the the ARMv7-M, as an example, you can see the set of registers that are
stored in ``arch/arm/include/armv7-m/irq.h``

Among those registers are saved and restore are the register(s) that determine if
interrupts are enable or not. For the ARMv7-M family that is either the ``PRIMASK``
register or the ``BASEPRI`` registers. So if a task disables interrupts then suspends,
the current value of ``PRIMASK``/``BASEPRI`` register is saved and replaced with the
stored value of the ``PRIMASK``/``BASEPRI`` register for the next task that will run,
thus re-enabling interrupts while the rist task is suspended.

So interrupt enabled/disable is a per-thread property, not a global property.
If you have been working with bare metal systems for a long time, this might seem
foreign to you.

By the way, locking the scheduler via ``sched_lock()`` behaves in this same way
(but the mechanism is a little different).
