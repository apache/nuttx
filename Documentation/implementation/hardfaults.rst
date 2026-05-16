.. _hardfaults:

==========
Hardfaults
==========

ARMv7
=====

Cortex-M3 and Cortex-M4
-----------------------

The most popular CPUs in current MCU designs are the Cortex-M3 (ARMv7-M) and
the Cortex-M4 (ARMv7E-M). Handling of these two architectures is almost
identical in NuttX (unless hardware floating point is enabled).

SVCALL
------

NuttX uses the SVCALL software interrupt in order to perform certain steps
in the context switching for the Cortex-M3 and Cortext-M4.
This sequence of logic appears in several places:

* Create a short critical section by disabling exceptions,
* Perform some set-up,
* Initiate the software exception / SVCALL, and
* When the software exception processing returns, re-enable exceptions.

.. note:: There is a technical difference between interrupts and exceptions.
          In this section the term exception will be used. It is probably
          more accurate since interrupts are really exceptions that result
          from device interrupt lines. Furthermore, when we refer to
          exceptions in this section, we are referring specifically to
          ARMv7-M configurable exceptions.

Disabling Interrupts via the PRIMASK register
---------------------------------------------

The ARMv7-M architecture supports a register called the ``PRIMASK`` register.
The ``PRIMASK`` register contains a single valid bit. If that bit is set to
one, then exceptions are disabled. If that bit is zero, exceptions are enabled.
More correctly, when this bit is set to one, it prevents the activation of all
exceptions with configurable priority.

The original NuttX implementation used this ``PRIMASK`` register to enable
and disable exceptions. Things now get interesting in the sequence of logic
listed above because the ``PRIMASK`` bit also disables the SVCALL exception!
So, instead of taking the ``SVCALL`` exception vector, the Cortex-M3/4
generates a hardfault exception (see ARM.com's discussion of
`Activation Levels <https://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0337g/Chdbdfjf.html>`_).

These hardfaults are not really a problem. The design of the NuttX hardfault
handler expects these exceptions and does the right thing.
However, the occurrence of hardfaults may come as a surprise to many people
and especially to some debuggers.

Hardfaults and Debuggers
------------------------

These hardfaults only become a technical issue when dealing with a debugger.
What does the debugger do when the hardfault occurs?

We need to back off and think about some system philosophy here.
The deep philosophical question here is: Who is in charge of system integrity?
The debugger or the RTOS?

If you are running a primitive NoOS program (like the famous blinky test
program), then you are running a barebones system and you need
all of the help you can get. So having the debugger make decisions about what
is the proper behavior of the blinky program and what is not is a good
thing for you.

But if you are using an advanced RTOS, then the RTOS will want to take
responsibility of the health of your system and now there is the possibility
of inconsistencies between the decisions that the RTOS makes and the decisions
that your debugger makes this hardfault handling is a perfect example here.

In NuttX, the hardfaults are controlled by the RTOS, but some debuggers will
break when the hardfault occurs and make debugging impossible.

Some people might take issue with this. Breaking on hardfault can make
debugging easier, since the break happens in the throwing context and you have
a hope of obtaining a backtrace and debugging the throwing side callstack.
However, I would suggest that putting a break point on ``up_assert()`` would
accomplish the same thing without being so intrusive.

How did the debugger know that the hardfault occurred? It knew because of
settings in the "ARM's Debug Exception and Monitor Control Register" or
``DEMCR``. Proper settings of the ``DEMCR`` register will allow debugger
to get break exceptions when a hardfault occrs.
So one workaround is to just reconfigure the ``DEMCR`` register so that break
exceptions are no longer generated when hardfaults occur.

Here is an example of such logic for the LPC43xx MCU.
Decoupling the hardfault from the break exception in this way does not work
with all debuggers, however. Presumably because some debuggers re-enable break
exceptions on hardfaults.

Disabling Interrupts via the BASEPRI register
---------------------------------------------

The ARMv7-M architecture supports another way to disable exceptions using
a register called the ``BASEPRI`` register. ARMv7-M exceptions are prioritized.
Each exception can be assigned an 8-bit priority. If the ``BASEPRI`` register
is set to a non-zero priority value, then it will filter exceptions in this sense:

* Exceptions with priority lower than or equal to the ``BASEPRI`` register
  will be disabled.
* Exceptions with priority higher than the ``BASEPRI`` register will still
  be enabled.

Normally this interrupt prioritization is used to support nested interrupt
handling, but it can also be used for disabling of all exceptions
if configured properly.

.. note:: In the ARMv7-M, higher values correspond to lower priority.
          This can be really confusing!

NuttX supports a configuration option called ``CONFIG_ARMV7M_USEBASEPRI``.
If this option is selected, then the exception prioritization and control
logic will be configured to use the ``BASEPRI`` register instead of the
``PRIMASK`` register to disable exceptions.
This configuration includes the following changes in the behavior:

* Normal interrupts and exceptions are restricted to the range
  ``{ lowest priority ... (highest priority - 1) }``.
* The priority of the ``SVCALL`` exception is set to highest priority.
* When exceptions are enabled, the ``BASEPRI`` register is set to zero,
  enabling exceptions of all priorities.
* When exceptions are disabled, the ``BASEPRI`` register is set to
  ``(highest priority - 1)``, disabling all exceptions except for the
  ``SVCALL`` exception.

In this way, the ``SVCALL`` exception remains enabled when exceptions
are disabled and no hardfault occurs.

.. note:: The above is inaccurate on several counts. It was simplified
          to make the discussion sane. Not only do higher values correspond
          to lower priorities, but the increment between consecutive 8-bit
          priority values is probably not one. The supported maximum and
          minimum priority values (as well as the step in each priority value)
          may be different for each MCU. To handle this, these values are
          exported in NuttX for each ARMv7-M architecture by header files at
          ``nuttx/arch/arm/include/<chip>/chip.h``.
