==================================
Single Button Multi Actions Driver
==================================

**Single Button (aka SButton)** is an kind of keyboard that uses
only a single physical button (Switch) in the board. This kind of
button is used with simple interfaces like those used on 3D Printers
or other devices where all the user needs is to move to the next
option and confirm the selection.

It could be done detecting if the button was pressed for a short
period of time (i.e. less than 500ms) or long pressed. If it is a
short press the driver will return **TAB** and if it is a long
press the driver will return **ENTER**. Using it is possible to
navigate on those kind of menu.

**How does it work?**. The driver uses a simple config data (this
config data is equivalent to the platform data on Linux kernel) to
map the pin from MCU will be used to register and detect the
interrupt from this pin physically connected to the button.

It uses a kind of "polymorphism" in C to allow the driver to get
access to the functions responsible to attach and enable the
interrupt and to get the status of the pin.
See ``include/nuttx/input/sbutton.h``
and ``boards/arm/stm32/common/src/stm32_sbutton.c`` to understand
better how it works. But basically the board file (config data)
creates a struct when the first field (variable) is the config
struct used the but SButton driver (``drivers/input/sbutton.c``).

Every time the user presses or releases the key button an interrupt
is generated. The ISR of this interrupt inside sbutton
(``sbutton_interrupt()``) calls a workqueue to process it (because
we cannot spend time inside the ISR processing data, it could
degradate the performance of the RTOS). All that workqueue
(``sbutton_worker()``) needs to do it measure the elapsed time
(ticks) from the moment the key was pressed until the moment it
was released to decide if it is a "KEY_1" (**TAB**) or a "KEY_2"
(**ENTER**). Then is call the ``keyboard_event()`` from the
keyboard upper to send this key stroke to the user application.

