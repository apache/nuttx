================
ST Nucleo F334R8
================

The Nucleo F334R8 is a member of the Nucleo-64 board family.

Configurations
==============

nsh:
----

Configures the NuttShell (nsh) located at apps/examples/nsh.

adc:
----

Configures the ADC example located at apps/examples/adc.

highpri:
--------

Configures the high priority interrupts example (ADC + PWM)

spwm1 and spwm2:
----------------

Configures the sinusoidal PWM (SPWM) example which presents a simple use case
of the STM32 PWM lower-half driver without generic upper-half PWM logic.

There are two variants of this example, where functionality is achieved with
different periperals:

- spwm1 uses HRTIM to generate PWM and change waveform samples
- spwm2 uses TIM1 to generate PWM and TIM6 to change waveform samples

At the moment, the waveform parameters are hardcoded, but it should be easy to
modify this example and make it more functional.
