====================
Sim Input Interfaces
====================

Overview
========

The Sim can emulate buttons, keyboard, touchscreen-mouse based, mouse with 3 buttons and analog-mouse based joystick.

These drivers are particularly useful for graphical applications emulared over X11.

Buttons
=======

This driver emulates a button using the user Mouse button. You can enable it using
``CONFIG_SIM_BUTTONS``.

Joystick
========

This driver simulates an Analog Joystick using the user mouse.

Keyboard
========

There are two types of keyboards: is native one used by default on NuttX SIM, since it is used to access the NuttShell interface and the ``CONFIG_SIM_KEYBOARD`` used with graphical applications.

Mouse
=====

The driver emulates a Mouse with 3 buttons support. It can be enabled using the
symbol: ``CONFIG_SIM_MOUSE``. This driver is useful with used with more advanced interfaces like Microwindow/Nano-X that require at least left and right click buttons events.

Touchscreen
===========

The difference between this driver and the Mouse is because this only generates a pen-down event when the user click in the interface. It is useful to use with LVGL graphic interface.

