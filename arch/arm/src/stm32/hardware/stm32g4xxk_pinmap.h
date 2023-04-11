/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32g4xxk_pinmap.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXK_PINMAP_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXK_PINMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Alternate Pin Functions.  All members of the STM32G4xxxx family share the
 * same pin multiplexing (although they differ in the pins physically
 * available).
 *
 * Alternative pin selections are provided with a numeric suffix like _1, _2,
 * etc.  Drivers, however, will use the pin selection without the numeric
 * suffix.  Additional definitions are required in the board.h file.  For
 * example, if CAN1_RX connects via PA11 on some board, then the following
 * definitions should appear in the board.h header file for that board:
 *
 * #define GPIO_CAN1_RX GPIO_CAN1_RX_1
 *
 * The driver will then automatically configure PA11 as the CAN1 RX pin.
 */

/* WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!!
 * Additional effort is required to select specific GPIO options such as
 * frequency, open-drain/push-pull, and pull-up/down!  Just the basics are
 * defined for most pins in this file.
 */

/* ADC - Analog Digital Converter *******************************************/

#define GPIO_ADC1_IN1_0                (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN0)
#define GPIO_ADC1_IN2_0                (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_ADC1_IN3_0                (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN2)
#define GPIO_ADC1_IN4_0                (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_ADC1_IN10_0               (GPIO_ANALOG | GPIO_PORTF | GPIO_PIN0)
#define GPIO_ADC1_IN15_0               (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0)

#define GPIO_ADC2_IN1_0                (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN0)
#define GPIO_ADC2_IN2_0                (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_ADC2_IN3_0                (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN6)
#define GPIO_ADC2_IN4_0                (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN7)
#define GPIO_ADC2_IN10_0               (GPIO_ANALOG | GPIO_PORTF | GPIO_PIN1)
#define GPIO_ADC2_IN13_0               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN5)
#define GPIO_ADC2_IN17_0               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN4)

/* COMP - Comparator ********************************************************/

/* Comparator Outputs */

#define GPIO_COMP1_OUT_1               (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN0)
#define GPIO_COMP1_OUT_2               (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_COMP1_OUT_3               (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_COMP1_OUT_4               (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN8)

#define GPIO_COMP2_OUT_1               (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN2)
#define GPIO_COMP2_OUT_2               (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN7)
#define GPIO_COMP2_OUT_3               (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN12)

#define GPIO_COMP3_OUT_1               (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN7)

#define GPIO_COMP4_OUT_2               (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN6)

/* Comparator Inputs non inverting */

#define GPIO_COMP1_INP_1               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_COMP1_INP_2               (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN1)

#define GPIO_COMP2_INP_1               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_COMP2_INP_2               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN7)

/* Comparator Inputs non inverting */

#define GPIO_COMP1_INM_1               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN4)
#define GPIO_COMP1_INM_2               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN0)

#define GPIO_COMP2_INM_1               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN2)
#define GPIO_COMP2_INM_2               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN5)

/* DAC **********************************************************************/

#define GPIO_DAC1_OUT1_0               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN4)
#define GPIO_DAC1_OUT2_0               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN5)

/* Clocks outputs ***********************************************************/

/* MCU clock output */

#define GPIO_MCO_1                     (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN8)
#define GPIO_MCO_2                     (GPIO_ALT | GPIO_AF0 | GPIO_PORTG | GPIO_PIN10)

/* Event outputs ************************************************************/

#define GPIO_PA0_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN0)
#define GPIO_PA1_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN1)
#define GPIO_PA2_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN2)
#define GPIO_PA3_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN3)
#define GPIO_PA4_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN4)
#define GPIO_PA5_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN5)
#define GPIO_PA6_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_PA7_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN7)
#define GPIO_PA8_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN8)
#define GPIO_PA9_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN9)
#define GPIO_PA10_EVENTOUT_0           (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_PA11_EVENTOUT_0           (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_PA12_EVENTOUT_0           (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN12)
#define GPIO_PA13_EVENTOUT_0           (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN13)
#define GPIO_PA14_EVENTOUT_0           (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN14)
#define GPIO_PA15_EVENTOUT_0           (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN15)

#define GPIO_PB0_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN0)
#define GPIO_PB3_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN3)
#define GPIO_PB4_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_PB5_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_PB6_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN6)
#define GPIO_PB7_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN7)
#define GPIO_PB8_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN8)

#define GPIO_PF0_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTF | GPIO_PIN0)
#define GPIO_PF1_EVENTOUT_0            (GPIO_ALT | GPIO_AF15 | GPIO_PORTF | GPIO_PIN1)

#define GPIO_PG10_EVENTOUT_0           (GPIO_ALT | GPIO_AF15 | GPIO_PORTG | GPIO_PIN10)

/* FDCAN ********************************************************************/

#define GPIO_FDCAN1_RX_1               (GPIO_ALT | GPIO_AF9 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN11)
#define GPIO_FDCAN1_RX_2               (GPIO_ALT | GPIO_AF9 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN8)
#define GPIO_FDCAN1_TX_1               (GPIO_ALT | GPIO_AF9 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN12)

#define GPIO_FDCAN2_RX_1               (GPIO_ALT | GPIO_AF9 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN5)
#define GPIO_FDCAN2_TX_1               (GPIO_ALT | GPIO_AF9 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN6)

/* I2C **********************************************************************/

#define GPIO_I2C1_SCL_1                (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN13)
#define GPIO_I2C1_SCL_2                (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN15)
#define GPIO_I2C1_SCL_3                (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTB | GPIO_PIN8)
#define GPIO_I2C1_SDA_1                (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN14)
#define GPIO_I2C1_SDA_2                (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTB | GPIO_PIN7)
#define GPIO_I2C1_SMBA_0               (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN5)

#define GPIO_I2C2_SCL_1                (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN9)
#define GPIO_I2C2_SDA_1                (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN8)
#define GPIO_I2C2_SDA_2                (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTF | GPIO_PIN0)
#define GPIO_I2C2_SMBA_1               (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN10)

#define GPIO_I2C3_SCL_0                (GPIO_ALT | GPIO_AF2 | GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN8)
#define GPIO_I2C3_SDA_1                (GPIO_ALT | GPIO_AF8 | GPIO_OPENDRAIN | GPIO_PORTB | GPIO_PIN5)
#define GPIO_I2C3_SMBA_1               (GPIO_ALT | GPIO_AF2 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN9)

/* I2S **********************************************************************/

#define GPIO_I2S_CKIN_0                (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN12)

#define GPIO_I2S2_CK_0                 (GPIO_ALT | GPIO_AF5 | GPIO_PORTF | GPIO_PIN1)
#define GPIO_I2S2_MCK_0                (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN8)
#define GPIO_I2S2_SD_0                 (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_I2S2_WS_0                 (GPIO_ALT | GPIO_AF5 | GPIO_PORTF | GPIO_PIN0)

#define GPIO_I2S3_CK_0                 (GPIO_ALT | GPIO_AF6 | GPIO_PORTB | GPIO_PIN3)
#define GPIO_I2S3_MCK_0                (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN9)
#define GPIO_I2S3_SD_0                 (GPIO_ALT | GPIO_AF6 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_I2S3_WS_1                 (GPIO_ALT | GPIO_AF6 | GPIO_PORTA | GPIO_PIN4)
#define GPIO_I2S3_WS_2                 (GPIO_ALT | GPIO_AF6 | GPIO_PORTA | GPIO_PIN15)

/* IR - Infrared with TIM16 channel 1 and TIM17 channel 1 *******************/

#define GPIO_IR_OUT_0                  (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN13)

/* LPTIM - Low Power Timer **************************************************/

#define GPIO_LPTIM1_ETR_0              (GPIO_ALT | GPIO_AF11 | GPIO_PORTB | GPIO_PIN6)
#define GPIO_LPTIM1_IN1_0              (GPIO_ALT | GPIO_AF11 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_LPTIM1_IN2_0              (GPIO_ALT | GPIO_AF11 | GPIO_PORTB | GPIO_PIN7)
#define GPIO_LPTIM1_OUT_1              (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN14)

/* LPUART - Low-Power Universal Asynchronous Receiver Transmitter ***********/

#define GPIO_LPUART1_CTS_0             (GPIO_ALT | GPIO_AF12 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_LPUART1_RX_0              (GPIO_ALT | GPIO_AF12 | GPIO_PORTA | GPIO_PIN3)
#define GPIO_LPUART1_TX_0              (GPIO_ALT | GPIO_AF12 | GPIO_PORTA | GPIO_PIN2)

/* JTAG *********************************************************************/

#define GPIO_JTCK_0                    (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN14)
#define GPIO_JTDI_0                    (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN15)
#define GPIO_JTDO_0                    (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN3)
#define GPIO_JTMS_0                    (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN13)
#define GPIO_NJTRST_0                  (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_SWCLK_0                   (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN14)
#define GPIO_SWDIO_0                   (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN13)

/* OPAMP ********************************************************************/

#define GPIO_OPAMP1_VINM_0             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OPAMP1_VINP_1             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_OPAMP1_VINP_2             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OPAMP1_VINP_3             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN7)
#define GPIO_OPAMP1_VOUT_0             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN2)

#define GPIO_OPAMP2_VINM_0             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN5)
#define GPIO_OPAMP2_VINP_1             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN7)
#define GPIO_OPAMP2_VINP_2             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0)
#define GPIO_OPAMP2_VOUT_0             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN6)

#define GPIO_OPAMP3_VINP_1             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_OPAMP3_VINP_2             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0)

#define GPIO_OPAMP6_VINM_1             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)

/* QUADSPI ******************************************************************/

#define GPIO_QUADSPI1_BK1_IO1_0        (GPIO_ALT | GPIO_AF10 | GPIO_PORTB | GPIO_PIN0)
#define GPIO_QUADSPI1_BK1_IO2_0        (GPIO_ALT | GPIO_AF10 | GPIO_PORTA | GPIO_PIN7)
#define GPIO_QUADSPI1_BK1_IO3_0        (GPIO_ALT | GPIO_AF10 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_QUADSPI1_BK1_NCS_1        (GPIO_ALT | GPIO_AF10 | GPIO_PORTA | GPIO_PIN2)
#define GPIO_QUADSPI1_CLK_1            (GPIO_ALT | GPIO_AF10 | GPIO_PORTA | GPIO_PIN3)

/* RTC **********************************************************************/

#define GPIO_RTC_REFIN_0               (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN1)

/* SAI - Serial Audio Interface *********************************************/

#define GPIO_SAI1_CK1_1                (GPIO_ALT | GPIO_AF3 | GPIO_PORTA | GPIO_PIN3)
#define GPIO_SAI1_CK1_2                (GPIO_ALT | GPIO_AF3 | GPIO_PORTB | GPIO_PIN8)
#define GPIO_SAI1_CK2_0                (GPIO_ALT | GPIO_AF12 | GPIO_PORTA | GPIO_PIN8)
#define GPIO_SAI1_D1_0                 (GPIO_ALT | GPIO_AF12 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_SAI1_FS_A_1               (GPIO_ALT | GPIO_AF14 | GPIO_PORTA | GPIO_PIN9)
#define GPIO_SAI1_FS_B_1               (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN4)
#define GPIO_SAI1_FS_B_2               (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN14)
#define GPIO_SAI1_FS_B_3               (GPIO_ALT | GPIO_AF14 | GPIO_PORTB | GPIO_PIN6)
#define GPIO_SAI1_MCLK_A_1             (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN3)
#define GPIO_SAI1_MCLK_A_2             (GPIO_ALT | GPIO_AF14 | GPIO_PORTB | GPIO_PIN8)
#define GPIO_SAI1_MCLK_B_0             (GPIO_ALT | GPIO_AF14 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_SAI1_SCK_A_1              (GPIO_ALT | GPIO_AF14 | GPIO_PORTA | GPIO_PIN8)
#define GPIO_SAI1_SCK_B_0              (GPIO_ALT | GPIO_AF14 | GPIO_PORTB | GPIO_PIN3)
#define GPIO_SAI1_SD_A_0               (GPIO_ALT | GPIO_AF14 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_SAI1_SD_B_1               (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN13)
#define GPIO_SAI1_SD_B_2               (GPIO_ALT | GPIO_AF12 | GPIO_PORTB | GPIO_PIN5)

/* SPI - Serial Peripheral Interface ****************************************/

#define GPIO_SPI1_MISO_1               (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_SPI1_MISO_2               (GPIO_ALT | GPIO_AF5 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_SPI1_MOSI_1               (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN7)
#define GPIO_SPI1_MOSI_2               (GPIO_ALT | GPIO_AF5 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_SPI1_NSS_1                (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN4)
#define GPIO_SPI1_NSS_2                (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN15)
#define GPIO_SPI1_SCK_1                (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN5)
#define GPIO_SPI1_SCK_2                (GPIO_ALT | GPIO_AF5 | GPIO_PORTB | GPIO_PIN3)

#define GPIO_SPI2_MISO_0               (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_SPI2_MOSI_0               (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_SPI2_NSS_0                (GPIO_ALT | GPIO_AF5 | GPIO_PORTF | GPIO_PIN0)
#define GPIO_SPI2_SCK_0                (GPIO_ALT | GPIO_AF5 | GPIO_PORTF | GPIO_PIN1)

#define GPIO_SPI3_MISO_0               (GPIO_ALT | GPIO_AF6 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_SPI3_MOSI_0               (GPIO_ALT | GPIO_AF6 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_SPI3_NSS_1                (GPIO_ALT | GPIO_AF6 | GPIO_PORTA | GPIO_PIN4)
#define GPIO_SPI3_NSS_2                (GPIO_ALT | GPIO_AF6 | GPIO_PORTA | GPIO_PIN15)
#define GPIO_SPI3_SCK_0                (GPIO_ALT | GPIO_AF6 | GPIO_PORTB | GPIO_PIN3)

/* TIM - Timers *************************************************************/

#define GPIO_TIM1_BKIN2_0              (GPIO_ALT | GPIO_AF12 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM1_BKIN_1               (GPIO_ALT | GPIO_AF6 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_TIM1_BKIN_2               (GPIO_ALT | GPIO_AF6 | GPIO_PORTA | GPIO_PIN14)
#define GPIO_TIM1_BKIN_3               (GPIO_ALT | GPIO_AF9 | GPIO_PORTA | GPIO_PIN15)
#define GPIO_TIM1_BKIN_4               (GPIO_ALT | GPIO_AF12 | GPIO_PORTB | GPIO_PIN8)
#define GPIO_TIM1_CH1IN_1              (GPIO_ALT | GPIO_AF6 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN8)
#define GPIO_TIM1_CH1NIN_1             (GPIO_ALT | GPIO_AF6 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM1_CH1NIN_2             (GPIO_ALT | GPIO_AF6 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM1_CH1NOUT_1            (GPIO_ALT | GPIO_AF6 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM1_CH1NOUT_2            (GPIO_ALT | GPIO_AF6 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM1_CH1OUT_1             (GPIO_ALT | GPIO_AF6 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN8)
#define GPIO_TIM1_CH2IN_1              (GPIO_ALT | GPIO_AF6 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN9)
#define GPIO_TIM1_CH2NIN_1             (GPIO_ALT | GPIO_AF6 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM1_CH2NIN_2             (GPIO_ALT | GPIO_AF6 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN0)
#define GPIO_TIM1_CH2NOUT_1            (GPIO_ALT | GPIO_AF6 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM1_CH2NOUT_2            (GPIO_ALT | GPIO_AF6 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN0)
#define GPIO_TIM1_CH2OUT_1             (GPIO_ALT | GPIO_AF6 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN9)
#define GPIO_TIM1_CH3IN_1              (GPIO_ALT | GPIO_AF6 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN10)
#define GPIO_TIM1_CH3NIN_1             (GPIO_ALT | GPIO_AF6 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN1)
#define GPIO_TIM1_CH3NOUT_1            (GPIO_ALT | GPIO_AF6 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN1)
#define GPIO_TIM1_CH3OUT_1             (GPIO_ALT | GPIO_AF6 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN10)
#define GPIO_TIM1_CH4IN_1              (GPIO_ALT | GPIO_AF11 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM1_CH4OUT_1             (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM1_ETR_1                (GPIO_ALT | GPIO_AF11 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)

#define GPIO_TIM2_CH1IN_1              (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN0)
#define GPIO_TIM2_CH1IN_2              (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN5)
#define GPIO_TIM2_CH1IN_3              (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN15)
#define GPIO_TIM2_CH1OUT_1             (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN0)
#define GPIO_TIM2_CH1OUT_2             (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN5)
#define GPIO_TIM2_CH1OUT_3             (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN15)
#define GPIO_TIM2_CH2IN_1              (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN1)
#define GPIO_TIM2_CH2IN_2              (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN3)
#define GPIO_TIM2_CH2OUT_1             (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN1)
#define GPIO_TIM2_CH2OUT_2             (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN3)
#define GPIO_TIM2_CH3IN_1              (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN2)
#define GPIO_TIM2_CH3IN_2              (GPIO_ALT | GPIO_AF10 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN9)
#define GPIO_TIM2_CH3OUT_1             (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN2)
#define GPIO_TIM2_CH3OUT_2             (GPIO_ALT | GPIO_AF10 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN9)
#define GPIO_TIM2_CH4IN_1              (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN3)
#define GPIO_TIM2_CH4IN_2              (GPIO_ALT | GPIO_AF10 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN10)
#define GPIO_TIM2_CH4OUT_1             (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN3)
#define GPIO_TIM2_CH4OUT_2             (GPIO_ALT | GPIO_AF10 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN10)
#define GPIO_TIM2_ETR_1                (GPIO_ALT | GPIO_AF14 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN0)
#define GPIO_TIM2_ETR_2                (GPIO_ALT | GPIO_AF2 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN5)
#define GPIO_TIM2_ETR_3                (GPIO_ALT | GPIO_AF14 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN15)

#define GPIO_TIM3_CH1IN_1              (GPIO_ALT | GPIO_AF2 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN6)
#define GPIO_TIM3_CH1IN_2              (GPIO_ALT | GPIO_AF2 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM3_CH1OUT_1             (GPIO_ALT | GPIO_AF2 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN6)
#define GPIO_TIM3_CH1OUT_2             (GPIO_ALT | GPIO_AF2 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM3_CH2IN_1              (GPIO_ALT | GPIO_AF2 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN4)
#define GPIO_TIM3_CH2IN_2              (GPIO_ALT | GPIO_AF2 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM3_CH2IN_3              (GPIO_ALT | GPIO_AF2 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN5)
#define GPIO_TIM3_CH2OUT_1             (GPIO_ALT | GPIO_AF2 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN4)
#define GPIO_TIM3_CH2OUT_2             (GPIO_ALT | GPIO_AF2 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM3_CH2OUT_3             (GPIO_ALT | GPIO_AF2 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN5)
#define GPIO_TIM3_CH3IN_1              (GPIO_ALT | GPIO_AF2 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN0)
#define GPIO_TIM3_CH3OUT_1             (GPIO_ALT | GPIO_AF2 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN0)
#define GPIO_TIM3_CH4IN_2              (GPIO_ALT | GPIO_AF10 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM3_CH4OUT_2             (GPIO_ALT | GPIO_AF10 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM3_ETR_1                (GPIO_ALT | GPIO_AF10 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN3)

#define GPIO_TIM4_CH1IN_1              (GPIO_ALT | GPIO_AF10 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM4_CH1IN_2              (GPIO_ALT | GPIO_AF2 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM4_CH1OUT_1             (GPIO_ALT | GPIO_AF10 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM4_CH1OUT_2             (GPIO_ALT | GPIO_AF2 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM4_CH2IN_1              (GPIO_ALT | GPIO_AF10 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM4_CH2IN_2              (GPIO_ALT | GPIO_AF2 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM4_CH2OUT_1             (GPIO_ALT | GPIO_AF10 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM4_CH2OUT_2             (GPIO_ALT | GPIO_AF2 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM4_CH3IN_1              (GPIO_ALT | GPIO_AF10 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN13)
#define GPIO_TIM4_CH3IN_2              (GPIO_ALT | GPIO_AF2 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN8)
#define GPIO_TIM4_CH3OUT_1             (GPIO_ALT | GPIO_AF10 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN13)
#define GPIO_TIM4_CH3OUT_2             (GPIO_ALT | GPIO_AF2 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN8)
#define GPIO_TIM4_ETR_1                (GPIO_ALT | GPIO_AF10 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN8)
#define GPIO_TIM4_ETR_2                (GPIO_ALT | GPIO_AF2 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN3)

#define GPIO_TIM8_BKIN2_0              (GPIO_ALT | GPIO_AF10 | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM8_BKIN_1               (GPIO_ALT | GPIO_AF9 | GPIO_PORTA | GPIO_PIN0)
#define GPIO_TIM8_BKIN_2               (GPIO_ALT | GPIO_AF4 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_TIM8_BKIN_3               (GPIO_ALT | GPIO_AF11 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_TIM8_BKIN_4               (GPIO_ALT | GPIO_AF5 | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM8_CH1IN_1              (GPIO_ALT | GPIO_AF2 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN15)
#define GPIO_TIM8_CH1IN_2              (GPIO_ALT | GPIO_AF5 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM8_CH1NIN_1             (GPIO_ALT | GPIO_AF4 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM8_CH1NIN_2             (GPIO_ALT | GPIO_AF4 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN3)
#define GPIO_TIM8_CH1NOUT_1            (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM8_CH1NOUT_2            (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN3)
#define GPIO_TIM8_CH1OUT_1             (GPIO_ALT | GPIO_AF2 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN15)
#define GPIO_TIM8_CH1OUT_2             (GPIO_ALT | GPIO_AF5 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM8_CH2IN_1              (GPIO_ALT | GPIO_AF5 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN14)
#define GPIO_TIM8_CH2IN_2              (GPIO_ALT | GPIO_AF10 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN8)
#define GPIO_TIM8_CH2NIN_1             (GPIO_ALT | GPIO_AF4 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN0)
#define GPIO_TIM8_CH2NIN_2             (GPIO_ALT | GPIO_AF4 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM8_CH2NOUT_1            (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN0)
#define GPIO_TIM8_CH2NOUT_2            (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM8_CH2OUT_1             (GPIO_ALT | GPIO_AF5 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN14)
#define GPIO_TIM8_CH2OUT_2             (GPIO_ALT | GPIO_AF10 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN8)
#define GPIO_TIM8_CH3NIN_2             (GPIO_ALT | GPIO_AF3 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN5)
#define GPIO_TIM8_CH3NOUT_2            (GPIO_ALT | GPIO_AF3 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN5)
#define GPIO_TIM8_ETR_1                (GPIO_ALT | GPIO_AF10 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN0)
#define GPIO_TIM8_ETR_2                (GPIO_ALT | GPIO_AF6 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN6)

#define GPIO_TIM15_BKIN_1              (GPIO_ALT | GPIO_AF9 | GPIO_PORTA | GPIO_PIN9)
#define GPIO_TIM15_CH1IN_1             (GPIO_ALT | GPIO_AF9 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN2)
#define GPIO_TIM15_CH1NIN_1            (GPIO_ALT | GPIO_AF9 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN1)
#define GPIO_TIM15_CH1NOUT_1           (GPIO_ALT | GPIO_AF9 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN1)
#define GPIO_TIM15_CH1OUT_1            (GPIO_ALT | GPIO_AF9 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN2)
#define GPIO_TIM15_CH2IN_1             (GPIO_ALT | GPIO_AF9 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN3)
#define GPIO_TIM15_CH2OUT_1            (GPIO_ALT | GPIO_AF9 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN3)

#define GPIO_TIM16_BKIN_1              (GPIO_ALT | GPIO_AF1 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_TIM16_CH1IN_1             (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN6)
#define GPIO_TIM16_CH1IN_2             (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM16_CH1IN_3             (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM16_CH1IN_4             (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN8)
#define GPIO_TIM16_CH1NIN_1            (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN13)
#define GPIO_TIM16_CH1NIN_2            (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM16_CH1NOUT_1           (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN13)
#define GPIO_TIM16_CH1NOUT_2           (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM16_CH1OUT_1            (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN6)
#define GPIO_TIM16_CH1OUT_2            (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM16_CH1OUT_3            (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM16_CH1OUT_4            (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN8)

#define GPIO_TIM17_BKIN_1              (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_TIM17_BKIN_2              (GPIO_ALT | GPIO_AF10 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM17_CH1IN_1             (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM17_CH1IN_2             (GPIO_ALT | GPIO_AF10 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN5)
#define GPIO_TIM17_CH1NIN_1            (GPIO_ALT | GPIO_AF1 | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM17_CH1NOUT_1           (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM17_CH1OUT_1            (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM17_CH1OUT_2            (GPIO_ALT | GPIO_AF10 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN5)

/* UARTs/USARTs *************************************************************/

#define GPIO_USART1_CK_0               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN8)
#define GPIO_USART1_CTS_0              (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_USART1_DE_0               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN12)
#define GPIO_USART1_NSS_0              (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_USART1_RTS_0              (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN12)
#define GPIO_USART1_RX_1               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_USART1_RX_2               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN7)
#define GPIO_USART1_TX_1               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN9)
#define GPIO_USART1_TX_2               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN6)

#define GPIO_USART2_CK_1               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN4)
#define GPIO_USART2_CK_2               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_USART2_CTS_0              (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN0)
#define GPIO_USART2_DE_0               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN1)
#define GPIO_USART2_NSS_0              (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN0)
#define GPIO_USART2_RTS_0              (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN1)
#define GPIO_USART2_RX_1               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN3)
#define GPIO_USART2_RX_2               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN15)
#define GPIO_USART2_RX_3               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_USART2_TX_1               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN2)
#define GPIO_USART2_TX_2               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN14)
#define GPIO_USART2_TX_3               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN3)

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXK_PINMAP_H */
