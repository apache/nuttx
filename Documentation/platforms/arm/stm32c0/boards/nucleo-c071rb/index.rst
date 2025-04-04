================
ST Nucleo C071RB
================

The Nucleo C071RB is a member of the Nucleo-64 board family.

Buttons
=======

B1 USER: the user button is connected to the I/O PC13 of the STM32
microcontroller.

Serial Console
==============

At default USART2 connected to "Virtual COM Port" is used as serial console.

Configurations
==============

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
