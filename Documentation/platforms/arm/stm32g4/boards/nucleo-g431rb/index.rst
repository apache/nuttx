================
ST Nucleo G431RB
================

The Nucleo G431RB is a member of the Nucleo-64 board family.

Configurations
==============

ihm16m1_f32 and ihm16m1_b16:
----------------------------

These examples are dedicated for the X-NUCLEO-IHM16M1 expansion board
based on STSPIN830 driver for three-phase brushless DC motors.

X-NUCLEO-IHM16M1 must be configured to work with FOC and 3-shunt
resistors. See ST documentation for details.

Pin configuration for the X-NUCLEO-IHM16M1 (TIM1 configuration):

    ==============  ================   =================
    Board Function  Chip Function      Chip Pin Number
    ==============  ================   =================
    Phase U high    TIM1_CH1           PA8
    Phase U enable  GPIO_PB13          PB13
    Phase V high    TIM1_CH2           PA9
    Phase V enable  GPIO_PB14          PB14
    Phase W high    TIM1_CH3           PA10
    Phase W enable  GPIO_PB15          PB15
    EN_FAULT        GPIO_PB12          PB12
    Current U       GPIO_ADC1_IN2      PA1
    Current V       GPIO_ADC1_IN12     PB1
    Current W       GPIO_ADC1_IN15     PB0
    Temperature     ?                  PC4
    VBUS            GPIO_ADC1_IN1      PA0
    BEMF1           NU                  
    BEMF2           NU                  
    BEMF3           (NU)                
    LED                                  
    +3V3 (CN7_16)                       
    GND (CN7_20)                        
    GPIO_BEMF       (NU)                
    ENCO_A/HALL_H1                      
    ENCO_B/HALL_H2                      
    ENCO_Z/HALL_H3                      
    GPIO1           (NU)                
    GPIO2           (NU)                
    GPIO3           (NU)                
    CPOUT           (NU)                
    BKIN1           (NU)                
    POT             GPIO_ADC1_IN8      PC2
    CURR_REF        (NU)                
    DAC             (NU)                
    ==============  ================   =================

    Current shunt resistance              = 0.33
    Current sense gain                    = -1.53 (inverted current)
    Vbus sense gain = 9.31k/(9.31k+169k)  = 0.0522124390107
    Vbus min                              = 7V
    Vbus max                              = 45V
    Iout max                              = 1.5A RMS

    IPHASE_RATIO = 1/(R_shunt*gain) = -1.98
    VBUS_RATIO   = 1/VBUS_gain      = 16

    For now only 3-shunt resistors configuration is supported.
