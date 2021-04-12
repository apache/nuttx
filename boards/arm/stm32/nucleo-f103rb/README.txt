Nucleo-64 Boards
================

The Nucleo-F103RB is a member of the Nucleo-64 board family.  The Nucleo-64
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

  ihm07m1_b16:
  ------------

    These examples are dedicated for the X-NUCLEO-IHM07M1 expansion board
    based on L6230 DMOS driver for three-phase brushless DC motors.

    X-NUCLEO-IHM07M1 must be configured to work with FOC and 3-shunt
    resistors. See ST documentation for details.

    Pin configuration for the X-NUCLEO-IHM07M1 (TIM1 configuration):

    Board Function   Chip Function      Chip Pin Number
    -------------   ----------------   -----------------
    Phase U high     TIM1_CH1           PA8
    Phase U enable   GPIO_PC10          PC10
    Phase V high     TIM1_CH2           PA9
    Phase V enable   GPIO_PC11          PC11
    Phase W high     TIM1_CH3           PA10
    Phase W enable   GPIO_PC12          PC12
    DIAG/EN          GPIO_PA11          PA11
    Current U        ADC1_IN0           PA0
    Current V        ADC1_IN11          PC1
    Current W        ADC1_IN10          PC0
    Temperature      ADC1_IN12          PC2
    VBUS             ADC1_IN1           PA1
    BEMF1            (NU)               PC3
    BEMF2            (NU)               PB0
    BEMF3            (NU)               PA7
    LED              GPIO_PB2           PB2
    +3V3 (CN7_16)
    GND (CN7_20)
    GPIO_BEMF        (NU)               PC9
    ENCO_A/HALL_H1   TIM2_CH1           PA15
    ENCO_B/HALL_H2   TIM2_CH2           PB3
    ENCO_Z/HALL_H3   TIM2_CH3           PB10
    GPIO1            (NU)               PB13
    GPIO2            (NU)               PB5
    GPIO3            (NU)               PA5
    CPOUT            (NU)               PA12
    BKIN1            (NU)               PB14
    POT              ADC1_IN9           PB1
    CURR_REF         (NU)               PB4
    DAC              DAC1_CH1           PA4
    DEBUG0           GPIO               PB8
    DEBUG1           GPIO               PB9
    DEBUG2           GPIO               PC6
    DEBUG3           GPIO               PC5
    DEBUG4           GPIO               PC8

    Current shunt resistance              = 0.33
    Current sense gain                    = -1.53 (inverted current)
    Vbus sense gain = 9.31k/(9.31k+169k)  = 0.0522124390107
    Vbus min                              = 8V
    Vbus max                              = 48V
    Iout max                              = 1.4A RMS

    IPHASE_RATIO = 1/(R_shunt*gain) = -1.98
    VBUS_RATIO   = 1/VBUS_gain      = 19.152

    For now only 3-shunt resistors configuration is supported.
