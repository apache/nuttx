===========
ODrive V3.6
===========

ODrive V3.6 is an open-source dual-motor FOC controller based on
the STMicro STM32F405RG and TI DRV8301 gate drivers.

See https://odriverobotics.com/shop/odrive-v36 for further information.

For now we support only ODrive V3.6 56V.

Pin configuration
=================

=========  ==============  =========  =======================
Board Pin  Chip Function   Chip Pin   Notes
=========  ==============  =========  =======================
GPIO_1                     PA0
GPIO_2                     PA1
GPIO_3     USART2_TX       PA2        Serial TX
GPIO_4     USART2_RX       PA3        Serial RX
M1_TEMP    ADC1_IN4        PA4
AUX_TEMP   ADC1_IN5        PA5
VBUS_S     ADC1_IN6        PA6
M1_AL      TIM8 CH1N       PA7
M0_AH      TIM1 CH1        PA8
M0_BH      TIM1 CH2        PA9
M0_CH      TIM1 CH3        PA10
USB_DM     USB DM          PA11
USB_DP     USB DP          PA12
SWDIO                      PA13
SWCLK                      PA14
GPIO_7                     PA15
M0_SO1     ADC2_IN10       PC0        M0 current 1
M0_SO2     ADC2_IN11       PC1        M0 current 2
M1_SO2     ADC3_IN12       PC2        M1 current 2
M1_SO1     ADC3_IN13       PC3        M1 current 1
GPIO_5                     PC4
M0_TEMP    ADC1_IN15       PC5
M1_AH      TIM8 CH1        PC6
M1_BH      TIM8 CH2        PC7
M1_CH      TIM8 CH3        PC8
M0_ENC_Z                   PC9
SPI_SCK    SPI3_SCK        PC10       DRV8301 M0/M1
SPI_MISO   SPI3_MISO       PC11       DRV8301 M0/M1
SPI_MOSI   SPI3_MOSI       PC12       DRV8301 M0/M1
M0_NCS     SPI CS          PC13       DRV8301 M0 CS
M1_NCS     SPI CS          PC14       DRV8301 M1 CS
M1_ENC_Z                   PC15
M1_BL      TIM8 CH2N       PB0
M1_CL      TIM8 CH3N       PB1
GPIO_6                     PB2
GPIO_8                     PB3
M0_ENC_A   TIM3_CH1IN_2    PB4
M0_ENC_B   TIM3_CH2IN_2    PB5
M1_ENC_A   TIM4_CH1IN_1    PB6
M1_ENC_B   TIM4_CH2IN_1    PB7
CAN_R      CAN_R           PB8
CAN_D      CAN_D           PB9
AUX_L                      PB10
AUX_H                      PB11
EN_GATE    OUT             PB12       M0/M1 DRV8301
M0_AL      TIM1 CH1N       PB13
M0_BL      TIM1 CH2N       PB14
M0_CL      TIM1 CH3N       PB15
N_FAULT                    PD2        M0/M1 DRV8301 N_FAULT
=========  ==============  =========  =======================

Board hardware configuration
============================

=========================== ==================
Current shunt resistance    0.0005
Current sense gain          10/20/40/80
Vbus min                    12V
Vbus max                    24V or 56V
Iout max                    40A (no cooling for MOSFETs)
IPHASE_RATIO                1/(R_shunt*gain)
VBUS_RATIO = 1/VBUS_gain    11 or 19
=========================== ==================

Configurations
==============

nsh
---

Configures the NuttShell (nsh) located at apps/examples/nsh. The
Configuration enables the serial interfaces on USART2. Support for
builtin applications is enabled, but in the base configuration no
builtin applications are selected.

usbnsh
------

This is another NSH example. If differs from other 'nsh' configurations
in that this configurations uses a USB serial device for console I/O.
