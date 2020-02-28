Nucleo-64 Boards
================

The Nucleo-F334R8 is a member of the Nucleo-64 board family.  The Nucleo-64
is a standard board for use with several STM32 parts in the LQFP64 package.
Variants include

  Order code    Targeted STM32
  ------------- --------------
  NUCLEO-F030R8 STM32F030R8T6
  NUCLEO-F070RB STM32F070RBT6
  NUCLEO-F072RB STM32F072RBT6
  NUCLEO-F091RC STM32F091RCT6
  NUCLEO-F103RB STM32F103RBT6
  NUCLEO-F302R8 STM32F302R8T6
  NUCLEO-F303RE STM32F303RET6
  NUCLEO-F334R8 STM32F334R8T6
  NUCLEO-F401RE STM32F401RET6
  NUCLEO-F410RB STM32F410RBT6
  NUCLEO-F411RE STM32F411RET6
  NUCLEO-F446RE STM32F446RET6
  NUCLEO-L053R8 STM32L053R8T6
  NUCLEO-L073RZ STM32L073RZT6
  NUCLEO-L152RE STM32L152RET6
  NUCLEO-L452RE STM32L452RET6
  NUCLEO-L476RG STM32L476RGT6

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
