=============================================
AVR DA/DB family software interrupt emulation
=============================================

AVR DA/DB family chips do not have any mechanism to trigger a software-based
interrupt. This can be worked around using interrupts from hardware - using
a software action that triggers an interrupt (almost) immediately. In NuttX
on AVR DA/DB chips, direct manipulation with an I/O pin is used.

This feature is intended to be used by drivers, mostly those which implement
part of their functionality using (hardware) interrupt handlers as described
in :doc:`/guides/concurrency/zerolatencyinterrupts`. For example, such an interrupt may
be designed to do something quickly and repeatedly until some job is done.
If, at that point, the user space needs to be notified by posting a semaphore,
the interrupt handler cannot do that as it can't use any OS interface.

Instead, the interrupt handler can trigger a software interrupt and handler
for that interrupt - implemented as a standard NuttX interrupt handler - will
have access to any interface it needs.

Since there may be more than one user of the feature at any given time,
the implementation has a concept of software interrupt vector. Any user
can request one at runtime (generally during initialization) and have
its own handler executed.

Implementation
==============

A pin chosen by user (or board default configuration) in Kconfig is configured
as an output and also to trigger on level interrupt (ie. when the bit us pulled
low.)

Under normal operation, the pin is driven high. When some user wants to trigger
a software interrupt, it simply writes to the corresponding I/O register to pull
the pin low. That triggers the level interrupt, interrupt handler runs
and executes associated handler.

The handler also takes care of driving the pin high again to stop triggering
the interrupt.

  .. note:: The selected I/O pin behaves as output I/O pin. The board design
    needs to take this into account and nothing must be connected to the pin
    (unless the design wants to externally monitor software interrupts.)
    Also, the pin must not be overridden by a peripheral.

Configuration
=============

The configuration option that enables this feature is located at
:menuselection:`System Type --> Enable software interrupt emulation`.
Enabling it unlocks further options:

:menuselection:`Use GPIO multiplexer` - available
if :menuselection:`GPIO ISR multiplexer` is enabled. This option allows you
to use software interrupts via the selected pin while also using interrupts
from the other pins for other purposes. More information
in :doc:`gpio_intr_mux`.

:menuselection:`Software interrupt emulation I/O port`
and :menuselection:`Software interrupt emulation I/O pin`
options select the pin which will be used to trigger the software interrupt
handler. Enter numbers - port number always corresponds to its alphabetical
order (ie. A is port 0, B is port 1, E is port 4 etc.) Pin numbers are indexed
from zero.

(Default value of -1 is there to prevent the user from accidentally enabling
this option without setting it properly, which could cause damage to the device
if the pin is connected to something that is not a high-impedance node.)

:menuselection:`Software interrupt vector limit` - limits the number of users
of this feature. Most notably, this affects size of the variable that holds
list of software interrupt vectors that are currently raised.

Any value in range of 1-32 can be given but the variable size increases
only with multiples of 8 (from ``uint8_t`` to ``uint32_t``)

Usage
=====

Initialization
--------------

Software interrupt emulation needs to be initialized by calling
``avrdx_swint_initialize``. This happens automatically
in ``avrdx_up_initialize`` if the feature is enabled.

Any user of the software interrupts must first obtain a interrupt vector.
This is done using ``avrdx_swint_allocate``:

.. code-block:: c

   #include <nuttx/irq.h>
   #include "avrdx_swint.h"

   avrdx_swint_size_t vector;

   vector = avrdx_swint_allocate(handler, NULL);

Both ``handler`` and the second parameter correspond to parameters
of ``irq_attach`` function.

The returned value ``vector`` should be treated as an opaque identifier.

This identifier is then used in a call to ``avrdx_swint_raise``:

.. code-block:: c

   avrdx_swint_raise(vector);

Function ``handler`` from the call above will be called
(in an interrupt context.)
