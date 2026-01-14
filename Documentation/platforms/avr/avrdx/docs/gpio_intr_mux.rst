==========================
GPIO interrupt multiplexer
==========================

AVR DA/DB family chips have single interrupt vector for all changes
on an I/O port. This poses problem when multiple drivers want to claim
the same interrupt (might happen for example with button and discrete
joystick drivers using pins on the same port.)

The I/O multiplexer solves it by providing interface similar
to irq_attach. However, it allows registration of multiple handlers
for the same interrupt vector with additional information recording
which pins should be serviced by each handler.


Configuration
=============

The GPIO interrupt multiplexer is enabled in :menuselection:`System Type --> GPIO ISR Multiplexer`.
All basic initialization is done automatically.

Usage
=====

Taking button input driver as an example, this driver requires that the board
provides ``board_button_irq`` method which enables and disables servicing
interrupts for given button. Other than ID of the button to be configured,
interrupt handler (function pointer) and opaque argument (void pointer)
parameters are provided.

Instead of calling ``irq_attach`` directly, this function may call
``avrdx_irq_attach_gpio_mux``. Handler and argument are among parameters
of this call but added to that are port index, pins and event that should
trigger the interrupt.

The multiplexer then attaches itself as the interrupt handler for the port.
It only runs handlers for pin(s) which triggered the interrupt.

It is possible to have 4 different handlers per port (currently hardcoded.)
Handler is considered to be different even if it is the same function but
with different opaque argument.

Each handler can have multiple pins assigned to it (provided as a bitmask)
but it is not possible to register one pin to multiple handlers.
