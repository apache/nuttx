================
ST Nucleo F302R8
================

The Nucleo F302R8 is a member of the Nucleo-64 board family.

Configurations
==============

ihm07m1_f32 and ihm07m1_b16:
----------------------------

These examples are dedicated for the X-NUCLEO-IHM07M1 expansion board
based on L6230 DMOS driver for three-phase brushless DC motors.

X-NUCLEO-IHM07M1 must be configured to work with FOC and 3-shunt
resistors. See ST documentation for details.

Pin configuration for the X-NUCLEO-IHM07M1 (TIM1 configuration):

    ==============   ================   =================
    Board Function   Chip Function      Chip Pin Number
    ==============   ================   =================
    Phase U high     TIM1_CH1           PA8
    Phase U enable   GPIO_PC10          PC10
    Phase V high     TIM1_CH2           PA9
    Phase V enable   GPIO_PC11          PC11
    Phase W high     TIM1_CH3           PA10
    Phase W enable   GPIO_PC12          PC12
    DIAG/EN          GPIO_PA11          PA11
    Current U        ADC1_IN1           PA0
    Current V        ADC1_IN7           PC1
    Current W        ADC1_IN6           PC0
    Temperature      ADC1_IN8           PC2
    VBUS             ADC1_IN2           PA1
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
    POT              ADC1_IN12          PB1
    CURR_REF         (NU)               PB4
    DAC              DAC1_CH1           PA4
    DEBUG0           GPIO               PB8
    DEBUG1           GPIO               PB9
    DEBUG2           GPIO               PC6
    DEBUG3           GPIO               PC5
    DEBUG4           GPIO               PC8
    ==============   ================   =================

    Current shunt resistance              = 0.33
    Current sense gain                    = -1.53 (inverted current)
    Vbus sense gain = 9.31k/(9.31k+169k)  = 0.0522124390107
    Vbus min                              = 8V
    Vbus max                              = 48V
    Iout max                              = 1.4A RMS

    IPHASE_RATIO = 1/(R_shunt*gain) = -1.98
    VBUS_RATIO   = 1/VBUS_gain      = 19.152

    For now only 3-shunt resistors configuration is supported.
