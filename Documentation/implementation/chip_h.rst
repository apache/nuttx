.. _chip-h:

======
chip.h
======

The purpose of the two chip.h files in each arm chip
====================================================

If you wonder about the purpose of the two ``chip.h`` files in each arm chip.

.. code:: sh

   $ find arch/arm -name chip.h | grep stm32
   arch/arm/include/stm32f4/chip.h
   arch/arm/src/stm32f4/chip.h

The reason behind ``arch/arm/src/stm32f4/chip.h`` file was a bad idea
that happened a long time ago.

Right now, I believe that its only required when ``CONFIG_ARMV7M_CMNVECTOR``
is selected in the configuration. In that case, ``arch/arm/src/stm32f4/chip.h``
is included by ``arch/arm/src/armv7-m/up_vectors.c`` in order provide
the number of interrupt vectors. In stm32, ``arch/arm/src/stm32f4/chip.h``
provides the number of vectors indirectly by including the correct,
chip-specific vectors.h file.
This function is a little more obvious in ``arch/arm/srch/lpc43xx/chip.h``.

This ``arch/arm/src/xyz/chip.h`` is also a good way to export awkward
internal header files in a cleaner way. But that use is optional
and not required outside of the chip- and board-related directories.

For the ``arch/include/stm32/chip.h`` file, only set of definitions
is required there, the NVIC priorities:

* ``NVIC_SYSH_PRIORITY_MIN`` provides the lowest interrupt priority
  supported by the system. This is the highest numeric value;
  but the lowest interrupt priority.
* ``NVIC_SYSH_PRIORITY_DEFAULT`` provides the default priority of all
  interrupts. Since truly nested interrupts are not supported, this is
  the priority of all interrupts except for the SVCall interrupt.
* ``NVIC_SYSH_PRIORITY_MAX`` provides the maximum interrupt priority.
  For all ARMv7-M's, this will be the value zero.
* ``NVIC_SYSH_PRIORITY_STEP`` is the value need to increment from one
  priority level to next, lower priority level.

If ``CONFIG_ARMV7M_USEBASEPRI`` is selected, then interrupts will be disabled
by setting the ``BASEPRI`` register to ``NVIC_SYSH_DISABLE_PRIORITY`` so that
most interrupts will not have execution priority.
SVCall must have execution priority in all cases.

In the normal cases, interrupts are not nest-able and all interrupts run
at an execution priority between ``NVIC_SYSH_PRIORITY_MIN`` and
``NVIC_SYSH_PRIORITY_MAX`` (with ``NVIC_SYSH_PRIORITY_MAX`` reserved
for SVCall).

If, in addition, ``CONFIG_ARCH_HIPRI_INTERRUPT`` is defined, then special high
priority interrupts are supported. These are not "nested" in the normal sense
of the word. These high priority interrupts can interrupt normal interrupt
processing but execute outside of OS (although they can "get back
into the game" via a PendSV interrupt).

In the normal course of things, interrupts must occasionally be disabled
using the ``up_irq_save()`` inline function to prevent contention in use
of resources that may be shared between interrupt level and non-interrupt
level logic. Now the question arises, if we are using the ``BASEPRI``
to disable interrupts and have high priority interrupts enabled
(``CONFIG_ARCH_HIPRI_INTERRUPT=y``), do we disable all interrupts except
SVCall (we cannot disable SVCall interrupts)?
Or do we only disable the "normal" interrupts?

If we are using the ``BASEPRI`` register to disable interrupts, then
the answer is that we must disable ONLY the normal interrupts. That is
because we cannot disable SVCALL interrupts and we cannot permit SVCAll
interrupts running at a higher priority than the high priority interrupts.
Otherwise, they will introduce jitter in the high priority interrupt
response time.

Hence, if you need to disable the high priority interrupt, you will have to
disable the interrupt either at the peripheral that generates the interrupt
or at the interrupt controller (e.g., ``NVIC``). Disabling global interrupts
via the ``BASEPRI`` register cannot effect high priority interrupts.

* ``NVIC_SYSH_MAXNORMAL_PRIORITY`` is the maximum priority of "normal"
  interrupts. Interrupt are disabled at the level
  ``NVIC_SYSH_MAXNORMAL_PRIORITY``, disabling all "normal" interrupt
  but leaving the high priority and SVCALL interrupts enabled.
* ``NVIC_SYSH_HIGH_PRIORITY`` is the priority of the high priority interrupt.
* ``NVIC_SYSH_DISABLE_PRIORITY`` is the level at which interrupts will
  be disabled. This will be equal to ``NVIC_SYSH_MAXNORMAL_PRIORITY``.
* ``NVIC_SYSH_SVCALL_PRIORITY`` is the priority of the SVCall interrupt.
  This must be higher priority than ``NVIC_SYSH_DISABLE_PRIORITY`` so that
  it is never disabled, but lower in priority than
  ``NVIC_SYSH_DISABLE_PRIORITY`` so that SVCall handling cannot interfere
  with high priority interrupt handling.

These definitions only have to be here because NVIC implementations may differ
in the number of priority levels.

Now, by convention, the ``arch/xyz/include/chip/abc/chip.h`` file is also used
to provide all of the definitions that discriminate the features of the
different members of the chip family.
It is not technically necessary that those chip feature definitions exist
in this header file, but it is a good convention to follow so that
it is easier for people who have to work across the differ architectures.

There is another really big difference between the two chip header files:
They also different in scope:

1. Code in the ``arch/arm/src/xyz`` and in ``boards/arm/xyz/abc/src``
   directories can both include the ``arch/arm/src/xyz/chip.h`` header file,
   but nothing else can (only because nothing else is provided the header
   file path in the build system). So that ``chip.h`` file is intended to be
   used only in low-level board/chip logic.
2. The ``chip.h`` header file at ``arch/arm/include/xyz/chip.h``,
   on the other hand, will be linked at include/arch/chip and, hence,
   can be included anywhere in the system, even from applications.
   It can be included like::

     #include <arch/chip/chip.h>

So the ``arch/arm/include/xyz/chip.h`` header file is a place where you would
want to put chip-related stuff that can be made visible to application code.
Chip capabilities is a good example of the kind of knowledge that
applications might care about.
