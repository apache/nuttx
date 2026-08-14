================
ST Nucleo U083RC
================

.. tags:: chip:stm32, chip:stm32u0, chip:stm32u083

The Nucleo U083RC is a member of the Nucleo-64 board family.

USB not supported yet.

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

- NSH with builtin applications

- OS test (ostest)

- ADC with DMA enabled using A0-A3 pins

- PWM on TIM1 CH1 (PA8, D7)

- Quadrature encoder on TIM3 (PA6/PA7, D12/D11)

- Timer driver on TIM6 (/dev/timer0)

- IWDG and WWDG watchdogs

- button with software debouncing enabled (no RC filter on the board)
