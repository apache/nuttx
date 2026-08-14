================
ST STM32U083C-DK
================

.. tags:: chip:stm32, chip:stm32u0, chip:stm32u083

The STM32U083C-DK is a Discovery kit based on the STM32U083MC MCU.

USB, LCD and touchkey (TSC) not supported yet.

LEDs
====

The board has three user LEDs:

- LD3 (green) connected to the I/O PC13
- LD4 (blue) connected to the I/O PA5
- LD5 (red) connected to the I/O PB2

There are no GPIO user buttons on this board.  The touchkey is connected
to the TSC.

Joystick
========

The 4-direction joystick with selection is connected to a resistor ladder
on PC2 (ADC1 IN2).  It is supported by the discrete joystick driver
(``/dev/djoy0``) which decodes the ADC conversion result into joystick
positions.  Joystick events are detected by polling the ADC from the low
priority work queue.

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

- joystick on PC2 (ADC1 IN2) with the discrete joystick driver and ADC
  with DMA enabled

- PWM on TIM1 CH1 (PA8, arduino D5)

- IWDG and WWDG watchdogs
