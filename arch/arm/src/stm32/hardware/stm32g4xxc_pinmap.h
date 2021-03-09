/****************************************************************************************************
 * arch/arm/src/stm32/hardware/stm32g4xxc_pinmap.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXC_PINMAP_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXC_PINMAP_H

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

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

/* WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!!
 * Additional effort is required to select specific GPIO options such as
 * frequency, open-drain/push-pull, and pull-up/down!  Just the basics are
 * defined for most pins in this file.
 */

/* ADC - Analog Digital Converter *******************************************************************/

#define GPIO_ADC1_IN1                  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN0)
#define GPIO_ADC1_IN2                  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_ADC1_IN3                  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN2)
#define GPIO_ADC1_IN4                  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_ADC1_IN5                  (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN14)
#define GPIO_ADC1_IN10                 (GPIO_ANALOG | GPIO_PORTF | GPIO_PIN0)
#define GPIO_ADC1_IN11                 (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN12)
#define GPIO_ADC1_IN12                 (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN1)
#define GPIO_ADC1_IN14                 (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN11)
#define GPIO_ADC1_IN15                 (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0)

#define GPIO_ADC2_IN1                  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN0)
#define GPIO_ADC2_IN2                  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_ADC2_IN3                  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN6)
#define GPIO_ADC2_IN4                  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN7)
#define GPIO_ADC2_IN5                  (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN4)
#define GPIO_ADC2_IN10                 (GPIO_ANALOG | GPIO_PORTF | GPIO_PIN1)
#define GPIO_ADC2_IN12                 (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN2)
#define GPIO_ADC2_IN13                 (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN5)
#define GPIO_ADC2_IN14                 (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN11)
#define GPIO_ADC2_IN15                 (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN15)
#define GPIO_ADC2_IN17                 (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN4)

#define GPIO_ADC3_IN1                  (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN1)
#define GPIO_ADC3_IN5                  (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN13)
#define GPIO_ADC3_IN12                 (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0)

#define GPIO_ADC4_IN3                  (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN12)
#define GPIO_ADC4_IN4                  (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN14)
#define GPIO_ADC4_IN5                  (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN15)

#define GPIO_ADC5_IN1                  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN8)
#define GPIO_ADC5_IN2                  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN9)

/* COMP - Comparator ********************************************************************************/

#define GPIO_COMP1_OUT_1               (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN0)
#define GPIO_COMP1_OUT_2               (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_COMP1_OUT_3               (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_COMP1_OUT_4               (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN8)

#define GPIO_COMP2_OUT_1               (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN2)
#define GPIO_COMP2_OUT_2               (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN7)
#define GPIO_COMP2_OUT_3               (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN12)
#define GPIO_COMP2_OUT_4               (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN9)

#define GPIO_COMP3_OUT_1               (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN7)
#define GPIO_COMP3_OUT_2               (GPIO_ALT | GPIO_AF3 | GPIO_PORTB | GPIO_PIN15)

#define GPIO_COMP4_OUT_1               (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN1)
#define GPIO_COMP4_OUT_2               (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN6)
#define GPIO_COMP4_OUT_3               (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN14)

#define GPIO_COMP5_OUT                 (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN9)

#define GPIO_COMP6_OUT_1               (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_COMP6_OUT_2               (GPIO_ALT | GPIO_AF7 | GPIO_PORTC | GPIO_PIN6)

#define GPIO_COMP7_OUT                 (GPIO_ALT | GPIO_AF8 | GPIO_PORTA | GPIO_PIN8)

/* CRS **********************************************************************************************/

/* REVISIT: Clock Recovery System (CRS_SYNC signal exposed to pin(s)?)
 * Before using the following defines, make sure they are correct!
 */

#if 0
#  define GPIO_USB_CRS_SYNC            (GPIO_ALT | GPIO_AF3 | GPIO_PORTA | GPIO_PIN10)
#  define GPIO_UCPD1_CRS_SYNC          (GPIO_ALT | GPIO_AF3 | GPIO_PORTB | GPIO_PIN3)
#endif

/* DAC **********************************************************************************************/

#define GPIO_DAC1_OUT1                 (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN4)
#define GPIO_DAC1_OUT2                 (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN5)
#define GPIO_DAC2_OUT1                 (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN6)

/* Clocks outputs ***********************************************************************************/

/* MCU clock output */

#define GPIO_MCO_1                     (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN8)
#define GPIO_MCO_2                     (GPIO_ALT | GPIO_AF0 | GPIO_PORTG | GPIO_PIN10)

/* Event outputs ************************************************************************************/

#define GPIO_PA0_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN0)
#define GPIO_PA1_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN1)
#define GPIO_PA2_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN2)
#define GPIO_PA3_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN3)
#define GPIO_PA4_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN4)
#define GPIO_PA5_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN5)
#define GPIO_PA6_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_PA7_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN7)
#define GPIO_PA8_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN8)
#define GPIO_PA9_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN9)
#define GPIO_PA10_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_PA11_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_PA12_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN12)
#define GPIO_PA13_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN13)
#define GPIO_PA14_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN14)
#define GPIO_PA15_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTA | GPIO_PIN15)

#define GPIO_PB0_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN0)
#define GPIO_PB1_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN1)
#define GPIO_PB2_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN2)
#define GPIO_PB3_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN3)
#define GPIO_PB4_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_PB5_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_PB6_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN6)
#define GPIO_PB7_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN7)
#define GPIO_PB8_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN8)
#define GPIO_PB9_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN9)
#define GPIO_PB10_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN10)
#define GPIO_PB11_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN11)
#define GPIO_PB12_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN12)
#define GPIO_PB13_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN13)
#define GPIO_PB14_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN14)
#define GPIO_PB15_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTB | GPIO_PIN15)

#define GPIO_PC4_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTC | GPIO_PIN4)
#define GPIO_PC6_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTC | GPIO_PIN6)
#define GPIO_PC10_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTC | GPIO_PIN10)
#define GPIO_PC11_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTC | GPIO_PIN11)
#define GPIO_PC13_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTC | GPIO_PIN13)
#define GPIO_PC14_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTC | GPIO_PIN14)
#define GPIO_PC15_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTC | GPIO_PIN15)

#define GPIO_PF0_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTF | GPIO_PIN0)
#define GPIO_PF1_EVENTOUT              (GPIO_ALT | GPIO_AF15 | GPIO_PORTF | GPIO_PIN1)

#define GPIO_PG10_EVENTOUT             (GPIO_ALT | GPIO_AF15 | GPIO_PORTG | GPIO_PIN10)

/* FDCAN ********************************************************************************************/

#define GPIO_FDCAN1_RX_1               (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN11)
#define GPIO_FDCAN1_RX_2               (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN8)
#define GPIO_FDCAN1_TX_1               (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN12)
#define GPIO_FDCAN1_TX_2               (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN9)

#define GPIO_FDCAN2_RX_1               (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN5)
#define GPIO_FDCAN2_RX_2               (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN12)
#define GPIO_FDCAN2_TX_1               (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN6)
#define GPIO_FDCAN2_TX_2               (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN13)

#define GPIO_FDCAN3_RX_1               (GPIO_ALT | GPIO_AF11 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN8)
#define GPIO_FDCAN3_RX_2               (GPIO_ALT | GPIO_AF11 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN3)
#define GPIO_FDCAN3_TX_1               (GPIO_ALT | GPIO_AF11 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN15)
#define GPIO_FDCAN3_TX_2               (GPIO_ALT | GPIO_AF11 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN4)

/* HRTIM - High-Resolution Timer ********************************************************************/

#define GPIO_HRTIM1_CHA1               (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN8)
#define GPIO_HRTIM1_CHA2               (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN9)
#define GPIO_HRTIM1_CHB1               (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_HRTIM1_CHB2               (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_HRTIM1_CHC1               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN12)
#define GPIO_HRTIM1_CHC2               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN13)
#define GPIO_HRTIM1_CHD1               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN14)
#define GPIO_HRTIM1_CHD2               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN15)
#define GPIO_HRTIM1_CHF1               (GPIO_ALT | GPIO_AF13 | GPIO_PORTC | GPIO_PIN6)
#define GPIO_HRTIM1_EEV2               (GPIO_ALT | GPIO_AF3 | GPIO_PORTC | GPIO_PIN11)
#define GPIO_HRTIM1_EEV3               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN7)
#define GPIO_HRTIM1_EEV4               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN6)
#define GPIO_HRTIM1_EEV5               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN9)
#define GPIO_HRTIM1_EEV6               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_HRTIM1_EEV7               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_HRTIM1_EEV8               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN8)
#define GPIO_HRTIM1_EEV9               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN3)
#define GPIO_HRTIM1_EEV10              (GPIO_ALT | GPIO_AF3 | GPIO_PORTC | GPIO_PIN6)
#define GPIO_HRTIM1_FLT1               (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN12)
#define GPIO_HRTIM1_FLT2               (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN15)
#define GPIO_HRTIM1_FLT3               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN10)
#define GPIO_HRTIM1_FLT4               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN11)
#define GPIO_HRTIM1_FLT5               (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN0)
#define GPIO_HRTIM1_FLT6               (GPIO_ALT | GPIO_AF13 | GPIO_PORTC | GPIO_PIN10)
#define GPIO_HRTIM1_SCIN_1             (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN2)
#define GPIO_HRTIM1_SCIN_2             (GPIO_ALT | GPIO_AF12 | GPIO_PORTB | GPIO_PIN6)
#define GPIO_HRTIM1_SCOUT_1            (GPIO_ALT | GPIO_AF13 | GPIO_PORTB | GPIO_PIN1)
#define GPIO_HRTIM1_SCOUT_2            (GPIO_ALT | GPIO_AF12 | GPIO_PORTB | GPIO_PIN3)

/* I2C **********************************************************************************************/

#define GPIO_I2C1_SCL_1                (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN13)
#define GPIO_I2C1_SCL_2                (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN15)
#define GPIO_I2C1_SCL_3                (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTB | GPIO_PIN8)
#define GPIO_I2C1_SDA_1                (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN14)
#define GPIO_I2C1_SDA_2                (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTB | GPIO_PIN7)
#define GPIO_I2C1_SDA_3                (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTB | GPIO_PIN9)
#define GPIO_I2C1_SMBA                 (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN5)

#define GPIO_I2C2_SCL_1                (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN9)
#define GPIO_I2C2_SCL_2                (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTC | GPIO_PIN4)
#define GPIO_I2C2_SDA_1                (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN8)
#define GPIO_I2C2_SDA_2                (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTF | GPIO_PIN0)
#define GPIO_I2C2_SMBA_1               (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN10)
#define GPIO_I2C2_SMBA_2               (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN12)

#define GPIO_I2C3_SCL                  (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN8)
#define GPIO_I2C3_SDA_1                (GPIO_ALT | GPIO_AF8 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTB | GPIO_PIN5)
#define GPIO_I2C3_SDA_2                (GPIO_ALT | GPIO_AF8 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTC | GPIO_PIN11)
#define GPIO_I2C3_SMBA_1               (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN9)
#define GPIO_I2C3_SMBA_2               (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN2)

#define GPIO_I2C4_SCL_1                (GPIO_ALT | GPIO_AF3 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN13)
#define GPIO_I2C4_SCL_2                (GPIO_ALT | GPIO_AF8 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTC | GPIO_PIN6)
#define GPIO_I2C4_SDA                  (GPIO_ALT | GPIO_AF3 | GPIO_SPEED_50MHz | GPIO_OPENDRAIN | GPIO_PORTB | GPIO_PIN7)
#define GPIO_I2C4_SMBA                 (GPIO_ALT | GPIO_AF3 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN14)

/* I2S **********************************************************************************************/

#define GPIO_I2S_CKIN                  (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN12)

#define GPIO_I2S2_CK_1                 (GPIO_ALT | GPIO_AF5 | GPIO_PORTB | GPIO_PIN13)
#define GPIO_I2S2_CK_2                 (GPIO_ALT | GPIO_AF5 | GPIO_PORTF | GPIO_PIN1)
#define GPIO_I2S2_MCK_1                (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN8)
#define GPIO_I2S2_MCK_2                (GPIO_ALT | GPIO_AF6 | GPIO_PORTC | GPIO_PIN6)
#define GPIO_I2S2_SD_1                 (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_I2S2_SD_2                 (GPIO_ALT | GPIO_AF5 | GPIO_PORTB | GPIO_PIN15)
#define GPIO_I2S2_WS_1                 (GPIO_ALT | GPIO_AF5 | GPIO_PORTB | GPIO_PIN12)
#define GPIO_I2S2_WS_2                 (GPIO_ALT | GPIO_AF5 | GPIO_PORTF | GPIO_PIN0)

#define GPIO_I2S3_CK_1                 (GPIO_ALT | GPIO_AF6 | GPIO_PORTB | GPIO_PIN3)
#define GPIO_I2S3_CK_2                 (GPIO_ALT | GPIO_AF6 | GPIO_PORTC | GPIO_PIN10)
#define GPIO_I2S3_MCK                  (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN9)
#define GPIO_I2S3_SD                   (GPIO_ALT | GPIO_AF6 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_I2S3_WS_1                 (GPIO_ALT | GPIO_AF6 | GPIO_PORTA | GPIO_PIN4)
#define GPIO_I2S3_WS_2                 (GPIO_ALT | GPIO_AF6 | GPIO_PORTA | GPIO_PIN15)

/* IR - Infrared with TIM16 channel 1 and TIM17 channel 1 *******************************************/

#define GPIO_IR_OUT_1                  (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN13)
#define GPIO_IR_OUT_2                  (GPIO_ALT | GPIO_AF6 | GPIO_PORTB | GPIO_PIN9)

/* LPTIM - Low Power Timer **************************************************************************/

#define GPIO_LPTIM1_ETR                (GPIO_ALT | GPIO_AF11 | GPIO_PORTB | GPIO_PIN6)
#define GPIO_LPTIM1_IN1                (GPIO_ALT | GPIO_AF11 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_LPTIM1_IN2                (GPIO_ALT | GPIO_AF11 | GPIO_PORTB | GPIO_PIN7)
#define GPIO_LPTIM1_OUT_1              (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN14)
#define GPIO_LPTIM1_OUT_2              (GPIO_ALT | GPIO_AF1 | GPIO_PORTB | GPIO_PIN2)

/* LPUART - Low-Power Universal Asynchronous Receiver Transmitter ***********************************/

#define GPIO_LPUART1_CTS_1             (GPIO_ALT | GPIO_AF12 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_LPUART1_CTS_2             (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN13)
#define GPIO_LPUART1_DE_1              (GPIO_ALT | GPIO_AF12 | GPIO_PORTB | GPIO_PIN1)
#define GPIO_LPUART1_DE_2              (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN12)
#define GPIO_LPUART1_RTS_1             (GPIO_ALT | GPIO_AF12 | GPIO_PORTB | GPIO_PIN1)
#define GPIO_LPUART1_RTS_2             (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN12)
#define GPIO_LPUART1_RX_1              (GPIO_ALT | GPIO_AF12 | GPIO_PORTA | GPIO_PIN3)
#define GPIO_LPUART1_RX_2              (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN10)
#define GPIO_LPUART1_TX_1              (GPIO_ALT | GPIO_AF12 | GPIO_PORTA | GPIO_PIN2)
#define GPIO_LPUART1_TX_2              (GPIO_ALT | GPIO_AF8 | GPIO_PORTB | GPIO_PIN11)

/* JTAG *********************************************************************************************/

#define GPIO_JTCK                      (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN14)
#define GPIO_JTDI                      (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN15)
#define GPIO_JTDO                      (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN3)
#define GPIO_JTMS                      (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN13)
#define GPIO_NJTRST                    (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_SWCLK                     (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN14)
#define GPIO_SWDIO                     (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN13)

/* OPAMP ********************************************************************************************/

#define GPIO_OPAMP1_VINM               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OPAMP1_VINM0              (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OPAMP1_VINM1              (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OPAMP1_VINM_SEC           (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OPAMP1_VINP_1             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_OPAMP1_VINP_2             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OPAMP1_VINP_3             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN7)
#define GPIO_OPAMP1_VINP_SEC_1         (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_OPAMP1_VINP_SEC_2         (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OPAMP1_VINP_SEC_3         (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN7)
#define GPIO_OPAMP1_VOUT               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN2)

#define GPIO_OPAMP2_VINM               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN5)
#define GPIO_OPAMP2_VINM0              (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN5)
#define GPIO_OPAMP2_VINM1              (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN5)
#define GPIO_OPAMP2_VINM_SEC           (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN5)
#define GPIO_OPAMP2_VINP_1             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN7)
#define GPIO_OPAMP2_VINP_2             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0)
#define GPIO_OPAMP2_VINP_3             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN14)
#define GPIO_OPAMP2_VINP_SEC_1         (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN7)
#define GPIO_OPAMP2_VINP_SEC_2         (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0)
#define GPIO_OPAMP2_VINP_SEC_3         (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN14)
#define GPIO_OPAMP2_VOUT               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN6)

#define GPIO_OPAMP3_VINM0_1            (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN2)
#define GPIO_OPAMP3_VINM0_2            (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN10)
#define GPIO_OPAMP3_VINM1_1            (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN2)
#define GPIO_OPAMP3_VINM1_2            (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN10)
#define GPIO_OPAMP3_VINM_1             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN2)
#define GPIO_OPAMP3_VINM_2             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN10)
#define GPIO_OPAMP3_VINM_SEC_1         (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN2)
#define GPIO_OPAMP3_VINM_SEC_2         (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN10)
#define GPIO_OPAMP3_VINP_1             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_OPAMP3_VINP_2             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0)
#define GPIO_OPAMP3_VINP_3             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN13)
#define GPIO_OPAMP3_VINP_SEC_1         (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_OPAMP3_VINP_SEC_2         (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0)
#define GPIO_OPAMP3_VINP_SEC_3         (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN13)
#define GPIO_OPAMP3_VOUT               (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN1)

#define GPIO_OPAMP4_VINM               (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN10)
#define GPIO_OPAMP4_VINM0              (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN10)
#define GPIO_OPAMP4_VINM1              (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN10)
#define GPIO_OPAMP4_VINM_SEC           (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN10)
#define GPIO_OPAMP4_VINP_1             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN11)
#define GPIO_OPAMP4_VINP_2             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN13)
#define GPIO_OPAMP4_VINP_SEC_1         (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN11)
#define GPIO_OPAMP4_VINP_SEC_2         (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN13)
#define GPIO_OPAMP4_VOUT               (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN12)

#define GPIO_OPAMP5_VINM0_1            (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OPAMP5_VINM0_2            (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN15)
#define GPIO_OPAMP5_VINM1_1            (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OPAMP5_VINM1_2            (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN15)
#define GPIO_OPAMP5_VINM_1             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OPAMP5_VINM_2             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN15)
#define GPIO_OPAMP5_VINM_SEC_1         (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OPAMP5_VINM_SEC_2         (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN15)
#define GPIO_OPAMP5_VINP               (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN14)
#define GPIO_OPAMP5_VINP_SEC           (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN14)
#define GPIO_OPAMP5_VOUT               (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN8)

#define GPIO_OPAMP6_VINM0_1            (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_OPAMP6_VINM0_2            (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN1)
#define GPIO_OPAMP6_VINM1_1            (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_OPAMP6_VINM1_2            (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN1)
#define GPIO_OPAMP6_VINM_1             (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_OPAMP6_VINM_2             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN1)
#define GPIO_OPAMP6_VINM_SEC_1         (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_OPAMP6_VINM_SEC_2         (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN1)
#define GPIO_OPAMP6_VINP_1             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN12)
#define GPIO_OPAMP6_VINP_2             (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN13)
#define GPIO_OPAMP6_VINP_SEC_1         (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN12)
#define GPIO_OPAMP6_VINP_SEC_2         (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN13)
#define GPIO_OPAMP6_VOUT               (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN11)

/* QUADSPI ******************************************************************************************/

#define GPIO_QUADSPI1_BK1_IO0          (GPIO_ALT | GPIO_AF10 | GPIO_PORTB | GPIO_PIN1)
#define GPIO_QUADSPI1_BK1_IO1          (GPIO_ALT | GPIO_AF10 | GPIO_PORTB | GPIO_PIN0)
#define GPIO_QUADSPI1_BK1_IO2          (GPIO_ALT | GPIO_AF10 | GPIO_PORTA | GPIO_PIN7)
#define GPIO_QUADSPI1_BK1_IO3          (GPIO_ALT | GPIO_AF10 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_QUADSPI1_BK1_NCS_1        (GPIO_ALT | GPIO_AF10 | GPIO_PORTA | GPIO_PIN2)
#define GPIO_QUADSPI1_BK1_NCS_2        (GPIO_ALT | GPIO_AF10 | GPIO_PORTB | GPIO_PIN11)
#define GPIO_QUADSPI1_BK2_IO1          (GPIO_ALT | GPIO_AF10 | GPIO_PORTB | GPIO_PIN2)
#define GPIO_QUADSPI1_BK2_IO3          (GPIO_ALT | GPIO_AF10 | GPIO_PORTC | GPIO_PIN4)
#define GPIO_QUADSPI1_CLK_1            (GPIO_ALT | GPIO_AF10 | GPIO_PORTA | GPIO_PIN3)
#define GPIO_QUADSPI1_CLK_2            (GPIO_ALT | GPIO_AF10 | GPIO_PORTB | GPIO_PIN10)

/* RTC **********************************************************************************************/

#define GPIO_RTC_OUT2                  (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN2)
#define GPIO_RTC_REFIN_1               (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN1)
#define GPIO_RTC_REFIN_2               (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN15)

#if 0

/* REVISIT: How do you actually enable OUT1, TAMP1, TAMP2, TS? The datasheet
 * (DS12288 Rev 2, page 54) shows OUT1, TS, and TAMP1 as "additional
 * functions" for PC13 on all STM32G474(C-M-Q-R-V)xxx P/Ns, but the alternate
 * function table (on page 72) makes no mention of these. Meanwhile, page 56
 * shows TAMP2 as an "additional function" but the same conundrum applies (for
 * now). Granted, these are in the "additional function" column, not the
 * "alternate function" column.
 */

#  define GPIO_RTC_OUT1                (GPIO_PORTC | GPIO_PIN13)
#  define GPIO_RTC_TS                  (GPIO_PORTC | GPIO_PIN13)
#  define GPIO_RTC_TAMP1               (GPIO_PORTC | GPIO_PIN13)
#  define GPIO_RTC_TAMP2               (GPIO_PORTA | GPIO_PIN0)

#endif

/* SAI - Serial Audio Interface *********************************************************************/

#define GPIO_SAI1_CK1_1                (GPIO_ALT | GPIO_AF3 | GPIO_PORTA | GPIO_PIN3)
#define GPIO_SAI1_CK1_2                (GPIO_ALT | GPIO_AF3 | GPIO_PORTB | GPIO_PIN8)
#define GPIO_SAI1_CK2                  (GPIO_ALT | GPIO_AF12 | GPIO_PORTA | GPIO_PIN8)
#define GPIO_SAI1_D1                   (GPIO_ALT | GPIO_AF12 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_SAI1_D2                   (GPIO_ALT | GPIO_AF3 | GPIO_PORTB | GPIO_PIN9)
#define GPIO_SAI1_FS_A_1               (GPIO_ALT | GPIO_AF14 | GPIO_PORTA | GPIO_PIN9)
#define GPIO_SAI1_FS_A_2               (GPIO_ALT | GPIO_AF14 | GPIO_PORTB | GPIO_PIN9)
#define GPIO_SAI1_FS_B_1               (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN4)
#define GPIO_SAI1_FS_B_2               (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN14)
#define GPIO_SAI1_FS_B_3               (GPIO_ALT | GPIO_AF14 | GPIO_PORTB | GPIO_PIN6)
#define GPIO_SAI1_MCLK_A_1             (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN3)
#define GPIO_SAI1_MCLK_A_2             (GPIO_ALT | GPIO_AF14 | GPIO_PORTB | GPIO_PIN8)
#define GPIO_SAI1_MCLK_B               (GPIO_ALT | GPIO_AF14 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_SAI1_SCK_A_1              (GPIO_ALT | GPIO_AF14 | GPIO_PORTA | GPIO_PIN8)
#define GPIO_SAI1_SCK_A_2              (GPIO_ALT | GPIO_AF14 | GPIO_PORTB | GPIO_PIN10)
#define GPIO_SAI1_SCK_B                (GPIO_ALT | GPIO_AF14 | GPIO_PORTB | GPIO_PIN3)
#define GPIO_SAI1_SD_A                 (GPIO_ALT | GPIO_AF14 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_SAI1_SD_B_1               (GPIO_ALT | GPIO_AF13 | GPIO_PORTA | GPIO_PIN13)
#define GPIO_SAI1_SD_B_2               (GPIO_ALT | GPIO_AF12 | GPIO_PORTB | GPIO_PIN5)

/* SPI - Serial Peripheral Interface ****************************************************************/

#define GPIO_SPI1_MISO_1               (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN6)
#define GPIO_SPI1_MISO_2               (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN4)
#define GPIO_SPI1_MOSI_1               (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN7)
#define GPIO_SPI1_MOSI_2               (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN5)
#define GPIO_SPI1_NSS_1                (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN4)
#define GPIO_SPI1_NSS_2                (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN15)
#define GPIO_SPI1_SCK_1                (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN5)
#define GPIO_SPI1_SCK_2                (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN3)

#define GPIO_SPI2_MISO_1               (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN10)
#define GPIO_SPI2_MISO_2               (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN14)
#define GPIO_SPI2_MOSI_1               (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN11)
#define GPIO_SPI2_MOSI_2               (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN15)
#define GPIO_SPI2_NSS_1                (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN12)
#define GPIO_SPI2_NSS_2                (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTF | GPIO_PIN0)
#define GPIO_SPI2_SCK_1                (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN13)
#define GPIO_SPI2_SCK_2                (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PORTF | GPIO_PIN1)

#define GPIO_SPI3_MISO_1               (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN4)
#define GPIO_SPI3_MISO_2               (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PORTC | GPIO_PIN11)
#define GPIO_SPI3_MOSI                 (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN5)
#define GPIO_SPI3_NSS_1                (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN4)
#define GPIO_SPI3_NSS_2                (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN15)
#define GPIO_SPI3_SCK_1                (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN3)
#define GPIO_SPI3_SCK_2                (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PORTC | GPIO_PIN10)

/* TIM - Timers *************************************************************************************/

#define GPIO_TIM1_BKIN2                (GPIO_ALT | GPIO_AF12 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM1_BKIN_1               (GPIO_ALT | GPIO_AF6 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_TIM1_BKIN_2               (GPIO_ALT | GPIO_AF6 | GPIO_PORTA | GPIO_PIN14)
#define GPIO_TIM1_BKIN_3               (GPIO_ALT | GPIO_AF9 | GPIO_PORTA | GPIO_PIN15)
#define GPIO_TIM1_BKIN_4               (GPIO_ALT | GPIO_AF12 | GPIO_PORTB | GPIO_PIN8)
#define GPIO_TIM1_BKIN_5               (GPIO_ALT | GPIO_AF12 | GPIO_PORTB | GPIO_PIN10)
#define GPIO_TIM1_BKIN_6               (GPIO_ALT | GPIO_AF6 | GPIO_PORTB | GPIO_PIN12)
#define GPIO_TIM1_BKIN_7               (GPIO_ALT | GPIO_AF2 | GPIO_PORTC | GPIO_PIN13)
#define GPIO_TIM1_CH1IN                (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN8)
#define GPIO_TIM1_CH1NIN_1             (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM1_CH1NIN_2             (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM1_CH1NIN_3             (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN13)
#define GPIO_TIM1_CH1NIN_4             (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN13)
#define GPIO_TIM1_CH1NOUT_1            (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM1_CH1NOUT_2            (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM1_CH1NOUT_3            (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN13)
#define GPIO_TIM1_CH1NOUT_4            (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN13)
#define GPIO_TIM1_CH1OUT               (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN8)
#define GPIO_TIM1_CH2IN                (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN9)
#define GPIO_TIM1_CH2NIN_1             (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM1_CH2NIN_2             (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN0)
#define GPIO_TIM1_CH2NIN_3             (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN14)
#define GPIO_TIM1_CH2NOUT_1            (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM1_CH2NOUT_2            (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN0)
#define GPIO_TIM1_CH2NOUT_3            (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN14)
#define GPIO_TIM1_CH2OUT               (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN9)
#define GPIO_TIM1_CH3IN                (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN10)
#define GPIO_TIM1_CH3NIN_1             (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN1)
#define GPIO_TIM1_CH3NIN_2             (GPIO_ALT | GPIO_AF12 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN9)
#define GPIO_TIM1_CH3NIN_3             (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN15)
#define GPIO_TIM1_CH3NIN_4             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN12)
#define GPIO_TIM1_CH3NOUT_1            (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN1)
#define GPIO_TIM1_CH3NOUT_2            (GPIO_ALT | GPIO_AF12 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN9)
#define GPIO_TIM1_CH3NOUT_3            (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN15)
#define GPIO_TIM1_CH3NOUT_4            (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTE | GPIO_PIN12)
#define GPIO_TIM1_CH3OUT               (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN10)
#define GPIO_TIM1_CH4IN                (GPIO_ALT | GPIO_AF11 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM1_CH4NIN               (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN5)
#define GPIO_TIM1_CH4OUT               (GPIO_ALT | GPIO_AF11 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM1_ETR                  (GPIO_ALT | GPIO_AF11 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM1_ETR_1                (GPIO_ALT | GPIO_AF11 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM1_ETR_2                (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN4)

#define GPIO_TIM2_CH1IN_1              (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN0)
#define GPIO_TIM2_CH1IN_2              (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN5)
#define GPIO_TIM2_CH1IN_3              (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN15)
#define GPIO_TIM2_CH1OUT_1             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN0)
#define GPIO_TIM2_CH1OUT_2             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN5)
#define GPIO_TIM2_CH1OUT_3             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN15)
#define GPIO_TIM2_CH2IN_1              (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN1)
#define GPIO_TIM2_CH2IN_2              (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN3)
#define GPIO_TIM2_CH2OUT_1             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN1)
#define GPIO_TIM2_CH2OUT_2             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN3)
#define GPIO_TIM2_CH3IN_1              (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN2)
#define GPIO_TIM2_CH3IN_2              (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN9)
#define GPIO_TIM2_CH3IN_3              (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN10)
#define GPIO_TIM2_CH3OUT_1             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN2)
#define GPIO_TIM2_CH3OUT_2             (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN9)
#define GPIO_TIM2_CH3OUT_3             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN10)
#define GPIO_TIM2_CH4IN_1              (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN3)
#define GPIO_TIM2_CH4IN_2              (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN10)
#define GPIO_TIM2_CH4IN_3              (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN11)
#define GPIO_TIM2_CH4OUT_1             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN3)
#define GPIO_TIM2_CH4OUT_2             (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN10)
#define GPIO_TIM2_CH4OUT_3             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN11)
#define GPIO_TIM2_ETR_1                (GPIO_ALT | GPIO_AF14 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN0)
#define GPIO_TIM2_ETR_2                (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN5)
#define GPIO_TIM2_ETR_3                (GPIO_ALT | GPIO_AF14 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN15)

#define GPIO_TIM3_CH1IN_1              (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN6)
#define GPIO_TIM3_CH1IN_2              (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM3_CH1IN_3              (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN6)
#define GPIO_TIM3_CH1OUT_1             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN6)
#define GPIO_TIM3_CH1OUT_2             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM3_CH1OUT_3             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN6)
#define GPIO_TIM3_CH2IN_1              (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN4)
#define GPIO_TIM3_CH2IN_2              (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM3_CH2IN_3              (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN5)
#define GPIO_TIM3_CH2OUT_1             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN4)
#define GPIO_TIM3_CH2OUT_2             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM3_CH2OUT_3             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN5)
#define GPIO_TIM3_CH3IN                (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN0)
#define GPIO_TIM3_CH3OUT               (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN0)
#define GPIO_TIM3_CH4IN_1              (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN1)
#define GPIO_TIM3_CH4IN_2              (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM3_CH4OUT_1             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN1)
#define GPIO_TIM3_CH4OUT_2             (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM3_ETR                  (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN3)

#define GPIO_TIM4_CH1IN_1              (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM4_CH1IN_2              (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM4_CH1OUT_1             (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN11)
#define GPIO_TIM4_CH1OUT_2             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM4_CH2IN_1              (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM4_CH2IN_2              (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM4_CH2OUT_1             (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM4_CH2OUT_2             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM4_CH3IN_1              (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN13)
#define GPIO_TIM4_CH3IN_2              (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN8)
#define GPIO_TIM4_CH3OUT_1             (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN13)
#define GPIO_TIM4_CH3OUT_2             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN8)
#define GPIO_TIM4_CH4IN                (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN9)
#define GPIO_TIM4_CH4OUT               (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN9)
#define GPIO_TIM4_ETR_1                (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN8)
#define GPIO_TIM4_ETR_2                (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN3)

#define GPIO_TIM5_CH1IN_1              (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN0)
#define GPIO_TIM5_CH1IN_2              (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN2)
#define GPIO_TIM5_CH1OUT_1             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN0)
#define GPIO_TIM5_CH1OUT_2             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN2)
#define GPIO_TIM5_CH2IN                (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN1)
#define GPIO_TIM5_CH2OUT               (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN1)
#define GPIO_TIM5_CH3IN                (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN2)
#define GPIO_TIM5_CH3OUT               (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN2)
#define GPIO_TIM5_CH4IN                (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN3)
#define GPIO_TIM5_CH4OUT               (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN3)
#define GPIO_TIM5_ETR                  (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN12)

#define GPIO_TIM8_BKIN2                (GPIO_ALT | GPIO_AF10 | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM8_BKIN_1               (GPIO_ALT | GPIO_AF9 | GPIO_PORTA | GPIO_PIN0)
#define GPIO_TIM8_BKIN_2               (GPIO_ALT | GPIO_AF4 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_TIM8_BKIN_3               (GPIO_ALT | GPIO_AF11 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_TIM8_BKIN_4               (GPIO_ALT | GPIO_AF5 | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM8_CH1IN_1              (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN15)
#define GPIO_TIM8_CH1IN_2              (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM8_CH1IN_3              (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN6)
#define GPIO_TIM8_CH1NIN_1             (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM8_CH1NIN_2             (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN3)
#define GPIO_TIM8_CH1NIN_3             (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN10)
#define GPIO_TIM8_CH1NOUT_1            (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM8_CH1NOUT_2            (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN3)
#define GPIO_TIM8_CH1NOUT_3            (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN10)
#define GPIO_TIM8_CH1OUT_1             (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN15)
#define GPIO_TIM8_CH1OUT_2             (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM8_CH1OUT_3             (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN6)
#define GPIO_TIM8_CH2IN_1              (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN14)
#define GPIO_TIM8_CH2IN_2              (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN8)
#define GPIO_TIM8_CH2NIN_1             (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN0)
#define GPIO_TIM8_CH2NIN_2             (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM8_CH2NIN_3             (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN11)
#define GPIO_TIM8_CH2NOUT_1            (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN0)
#define GPIO_TIM8_CH2NOUT_2            (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM8_CH2NOUT_3            (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN11)
#define GPIO_TIM8_CH2OUT_1             (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN14)
#define GPIO_TIM8_CH2OUT_2             (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN8)
#define GPIO_TIM8_CH3IN                (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN9)
#define GPIO_TIM8_CH3NIN_1             (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN1)
#define GPIO_TIM8_CH3NIN_2             (GPIO_ALT | GPIO_AF3 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN5)
#define GPIO_TIM8_CH3NOUT_1            (GPIO_ALT | GPIO_AF4 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN1)
#define GPIO_TIM8_CH3NOUT_2            (GPIO_ALT | GPIO_AF3 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN5)
#define GPIO_TIM8_CH3OUT               (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN9)
#define GPIO_TIM8_CH4NIN               (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN13)
#define GPIO_TIM8_CH4NOUT              (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN13)
#define GPIO_TIM8_ETR_1                (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN0)
#define GPIO_TIM8_ETR_2                (GPIO_ALT | GPIO_AF6 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN6)

#define GPIO_TIM15_BKIN                (GPIO_ALT | GPIO_AF9 | GPIO_PORTA | GPIO_PIN9)
#define GPIO_TIM15_CH1IN_1             (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN2)
#define GPIO_TIM15_CH1IN_2             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN14)
#define GPIO_TIM15_CH1NIN_1            (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN1)
#define GPIO_TIM15_CH1NIN_2            (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN15)
#define GPIO_TIM15_CH1NOUT_1           (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN1)
#define GPIO_TIM15_CH1NOUT_2           (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN15)
#define GPIO_TIM15_CH1OUT_1            (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN2)
#define GPIO_TIM15_CH1OUT_2            (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN14)
#define GPIO_TIM15_CH2IN_1             (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN3)
#define GPIO_TIM15_CH2IN_2             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN15)
#define GPIO_TIM15_CH2OUT_1            (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN3)
#define GPIO_TIM15_CH2OUT_2            (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN15)

#define GPIO_TIM16_BKIN                (GPIO_ALT | GPIO_AF1 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_TIM16_CH1IN_1             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN6)
#define GPIO_TIM16_CH1IN_2             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM16_CH1IN_3             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM16_CH1IN_4             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN8)
#define GPIO_TIM16_CH1NIN_1            (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN13)
#define GPIO_TIM16_CH1NIN_2            (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM16_CH1NOUT_1           (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN13)
#define GPIO_TIM16_CH1NOUT_2           (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN6)
#define GPIO_TIM16_CH1OUT_1            (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN6)
#define GPIO_TIM16_CH1OUT_2            (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN12)
#define GPIO_TIM16_CH1OUT_3            (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM16_CH1OUT_4            (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN8)

#define GPIO_TIM17_BKIN_1              (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_TIM17_BKIN_2              (GPIO_ALT | GPIO_AF10 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_TIM17_CH1IN_1             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM17_CH1IN_2             (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN5)
#define GPIO_TIM17_CH1IN_3             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN9)
#define GPIO_TIM17_CH1NIN              (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM17_CH1NOUT             (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN7)
#define GPIO_TIM17_CH1OUT_1            (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN7)
#define GPIO_TIM17_CH1OUT_2            (GPIO_ALT | GPIO_AF10 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN5)
#define GPIO_TIM17_CH1OUT_3            (GPIO_ALT | GPIO_AF1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN9)

#define GPIO_TIM20_CH1IN               (GPIO_ALT | GPIO_AF3 | GPIO_SPEED_50MHz | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN2)
#define GPIO_TIM20_CH1OUT              (GPIO_ALT | GPIO_AF3 | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN2)

/* UARTs/USARTs *************************************************************************************/

#define GPIO_USART1_CK                 (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN8)
#define GPIO_USART1_CTS                (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_USART1_DE                 (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN12)
#define GPIO_USART1_NSS                (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_USART1_RTS                (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN12)
#define GPIO_USART1_RX_1               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_USART1_RX_2               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN7)
#define GPIO_USART1_TX_1               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN9)
#define GPIO_USART1_TX_2               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN6)
#define GPIO_USART1_TX_3               (GPIO_ALT | GPIO_AF7 | GPIO_PORTC | GPIO_PIN4)

#define GPIO_USART2_CK_1               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN4)
#define GPIO_USART2_CK_2               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_USART2_CTS                (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN0)
#define GPIO_USART2_DE                 (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN1)
#define GPIO_USART2_NSS                (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN0)
#define GPIO_USART2_RTS                (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN1)
#define GPIO_USART2_RX_1               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN3)
#define GPIO_USART2_RX_2               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN15)
#define GPIO_USART2_RX_3               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_USART2_TX_1               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN2)
#define GPIO_USART2_TX_2               (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN14)
#define GPIO_USART2_TX_3               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN3)

#define GPIO_USART3_CK                 (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN12)
#define GPIO_USART3_CTS_1              (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN13)
#define GPIO_USART3_CTS_2              (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN13)
#define GPIO_USART3_DE                 (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN14)
#define GPIO_USART3_NSS_1              (GPIO_ALT | GPIO_AF7 | GPIO_PORTA | GPIO_PIN13)
#define GPIO_USART3_NSS_2              (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN13)
#define GPIO_USART3_RTS                (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN14)
#define GPIO_USART3_RX_1               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN8)
#define GPIO_USART3_RX_2               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN11)
#define GPIO_USART3_RX_3               (GPIO_ALT | GPIO_AF7 | GPIO_PORTC | GPIO_PIN11)
#define GPIO_USART3_TX_1               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN9)
#define GPIO_USART3_TX_2               (GPIO_ALT | GPIO_AF7 | GPIO_PORTB | GPIO_PIN10)
#define GPIO_USART3_TX_3               (GPIO_ALT | GPIO_AF7 | GPIO_PORTC | GPIO_PIN10)

/* USB Device Full Speed ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXC_PINMAP_H */
