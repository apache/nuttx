README
======

The B-G431B-ESC Discovery kit board is based on the STM32G431CB microcontroller,
the L6387 driver and STL180N6F7 power MOSFETs.

UART/USART PINS
---------------

USART2 is accessible through J3 pads and ST LINK Virtual Console:
  USART2_TX - PB3
  USART2_RX - PB4

Configuration Sub-directories
-------------------------

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables the serial interfaces on USART2.

  foc_f32 and foc_b16:
  ---------------------
    FOC examples based on hardware on board.

    Pin configuration:

    Board Function   Chip Function      Chip Pin Number
    --------------   --------------     ---------------
    Phase U high     TIM1_CH1           PA8
    Phase U low      TIM1_CH1N          PC13
    Phase V high     TIM1_CH2           PA9
    Phase V low      TIM1_CH2N          PA12
    Phase W high     TIM1_CH3           PA10
    Phase W low      TIM1_CH3N          PB15
    Current U +      OPAMP1_VINP        PA1
    Current U -      OPAMP1_VINM        PA3
    Current V +      OPAMP2_VINP        PA7
    Current V -      OPAMP2_VINM        PA5
    Current W +      OPAMP3_VINP        PB0
    Current W -      OPAMP3_VINM        PB2
    Temperature                         PB14
    VBUS             ADC1_IN1           PA0
    POT              ADC1_IN11          PB12
    LED              GPIO_PC6           PC6
    ENCO_A/HALL_H1   TIM4_CH1           PB6
    ENCO_B/HALL_H2   TIM4_CH2           PB7
    ENCO_Z/HALL_H3   TIM4_CH3           PB8
    BUTTON           GPIO_PC10          PC10
    PWM                                 PA15
    CAN_RX           FDCAN1_RX          PA11
    CAN_TX           FDCAN1_TX          PB9
    CAN_TERM                            PC14
    GPIO_BEMF                           PB5
    BEMF1            ADC2_IN17          PA4
    BEMF2            ADC2_IN5           PC4
    BEMF3            ADC2_IN14          PB11

    Current shunt resistance          = 0.003
    PGA gain                          = 16
    Current sense gain                = -9.14 (inverted current)
    Vbus sense gain = 18k/(18k+169k)  = 0.0962
    Vbus min                          = ?
    Vbus max                          = 25V
    Iout max                          = 40A peak
    BEMF sense gain = 2.2k/(10k+2.2k) = 0.18

    IPHASE_RATIO = 1/(R_shunt*gain) = -36.47
    VBUS_RATIO   = 1/VBUS_gain      = 10.4
