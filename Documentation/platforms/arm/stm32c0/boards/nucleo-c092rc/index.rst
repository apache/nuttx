================
ST Nucleo C092RC
================

.. tags:: chip:stm32, chip:stm32c0, chip:stm32c092

The Nucleo C092RC is a member of the Nucleo-64 board family.

CANFD not supported yet.

Buttons
=======

B1 USER: the user button is connected to the I/O PC13 of the STM32
microcontroller.

Serial Console
==============

At default USART2 connected to "Virtual COM Port" is used as serial console.

Configurations
==============

nsh
---

Configures the NuttShell (nsh) located at apps/examples/nsh.  The
Configuration enables the serial interfaces on USART2.  Support for
builtin applications is disabled.

jumbo
-----

This configuration enables many Apache NuttX features.  This is
mostly to help provide additional code coverage in CI, but also
allows for a users to see a wide range of features that are
supported by the OS.

Enabled features:

- NSH

- ADC with DMA enabled using A0 and A1 pins

- button with software debouncing enabled (no RC filter on the board)

can
---

Configuration with FDCAN enabled as character driver and
``examples/can`` included as builtin application.

cansock
-------

Configuration with FDCAN enabled as SocketCAN interface with ``candump``
and ``cansend`` utils included as builtin applications.
