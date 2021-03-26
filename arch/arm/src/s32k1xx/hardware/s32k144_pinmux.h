/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k144_pinmux.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K144_PINMUX_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K144_PINMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* In most cases, there are alternative configurations for various pins.
 * Those alternative pins are labeled with a suffix like _1, _2, etc. in
 * order to distinguish them.  Logic in the board.h file must select the
 * correct pin configuration for the board by defining a pin configuration
 *(with no suffix) that maps to the correct alternative.
 *
 * WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!!
 * Additional effort is required to select specific GPIO options such as
 * frequency, and pull-up/down!
 * Just the basics are defined for most pins in the initial version of
 * this file.
 */

/* ADC */

#define PIN_ADC0_SE0              (PIN_ANALOG | PIN_PORTA | PIN0)
#define PIN_ADC0_SE1              (PIN_ANALOG | PIN_PORTA | PIN1)
#define PIN_ADC0_SE2              (PIN_ANALOG | PIN_PORTA | PIN6)
#define PIN_ADC0_SE3              (PIN_ANALOG | PIN_PORTA | PIN7)
#define PIN_ADC0_SE4              (PIN_ANALOG | PIN_PORTB | PIN0)
#define PIN_ADC0_SE5              (PIN_ANALOG | PIN_PORTB | PIN1)
#define PIN_ADC0_SE6              (PIN_ANALOG | PIN_PORTB | PIN2)
#define PIN_ADC0_SE7              (PIN_ANALOG | PIN_PORTB | PIN3)
#define PIN_ADC0_SE8_1            (PIN_ANALOG | PIN_PORTB | PIN13)
#define PIN_ADC0_SE8_2            (PIN_ANALOG | PIN_PORTC | PIN0)
#define PIN_ADC0_SE9_1            (PIN_ANALOG | PIN_PORTB | PIN14)
#define PIN_ADC0_SE9_2            (PIN_ANALOG | PIN_PORTC | PIN1)
#define PIN_ADC0_SE10             (PIN_ANALOG | PIN_PORTC | PIN2)
#define PIN_ADC0_SE11             (PIN_ANALOG | PIN_PORTC | PIN3)
#define PIN_ADC0_SE12             (PIN_ANALOG | PIN_PORTC | PIN14)
#define PIN_ADC0_SE13             (PIN_ANALOG | PIN_PORTC | PIN15)
#define PIN_ADC0_SE14             (PIN_ANALOG | PIN_PORTC | PIN16)
#define PIN_ADC0_SE15             (PIN_ANALOG | PIN_PORTC | PIN17)

#define PIN_ADC1_SE0              (PIN_ANALOG | PIN_PORTA | PIN2)
#define PIN_ADC1_SE1              (PIN_ANALOG | PIN_PORTA | PIN3)
#define PIN_ADC1_SE2              (PIN_ANALOG | PIN_PORTD | PIN2)
#define PIN_ADC1_SE3              (PIN_ANALOG | PIN_PORTD | PIN3)
#define PIN_ADC1_SE4              (PIN_ANALOG | PIN_PORTC | PIN6)
#define PIN_ADC1_SE5              (PIN_ANALOG | PIN_PORTC | PIN7)
#define PIN_ADC1_SE6              (PIN_ANALOG | PIN_PORTD | PIN4)
#define PIN_ADC1_SE7              (PIN_ANALOG | PIN_PORTB | PIN12)
#define PIN_ADC1_SE8              (PIN_ANALOG | PIN_PORTB | PIN13)
#define PIN_ADC1_SE9              (PIN_ANALOG | PIN_PORTB | PIN14)
#define PIN_ADC1_SE10             (PIN_ANALOG | PIN_PORTE | PIN2)
#define PIN_ADC1_SE11             (PIN_ANALOG | PIN_PORTE | PIN6)
#define PIN_ADC1_SE12             (PIN_ANALOG | PIN_PORTA | PIN15)
#define PIN_ADC1_SE13             (PIN_ANALOG | PIN_PORTA | PIN16)
#define PIN_ADC1_SE14_1           (PIN_ANALOG | PIN_PORTB | PIN0)
#define PIN_ADC1_SE14_2           (PIN_ANALOG | PIN_PORTB | PIN15)
#define PIN_ADC1_SE15_1           (PIN_ANALOG | PIN_PORTB | PIN1)
#define PIN_ADC1_SE15_2           (PIN_ANALOG | PIN_PORTB | PIN16)

/* CAN */

#define PIN_CAN0_RX_1             (PIN_ALT3   | PIN_PORTC | PIN2)
#define PIN_CAN0_RX_2             (PIN_ALT5   | PIN_PORTB | PIN0)
#define PIN_CAN0_RX_3             (PIN_ALT5   | PIN_PORTE | PIN4)
#define PIN_CAN0_TX_1             (PIN_ALT3   | PIN_PORTC | PIN3)
#define PIN_CAN0_TX_2             (PIN_ALT5   | PIN_PORTB | PIN1)
#define PIN_CAN0_TX_3             (PIN_ALT5   | PIN_PORTE | PIN5)

#define PIN_CAN1_RX_1             (PIN_ALT3   | PIN_PORTA | PIN12)
#define PIN_CAN1_RX_2             (PIN_ALT3   | PIN_PORTC | PIN6)
#define PIN_CAN1_TX_1             (PIN_ALT3   | PIN_PORTA | PIN13)
#define PIN_CAN1_TX_2             (PIN_ALT3   | PIN_PORTC | PIN7)

#define PIN_CAN2_RX_1             (PIN_ALT3   | PIN_PORTC | PIN16)
#define PIN_CAN2_RX_2             (PIN_ALT4   | PIN_PORTB | PIN12)
#define PIN_CAN2_TX_1             (PIN_ALT3   | PIN_PORTC | PIN17)
#define PIN_CAN2_TX_2             (PIN_ALT4   | PIN_PORTB | PIN13)

/* Output clock */

#define PIN_CLKOUT_1              (PIN_ALT2   | PIN_PORTE | PIN10)
#define PIN_CLKOUT_2              (PIN_ALT5   | PIN_PORTB | PIN5)
#define PIN_CLKOUT_3              (PIN_ALT7   | PIN_PORTD | PIN14)

/* Comparators */

#define PIN_CMP0_IN0              (PIN_ANALOG | PIN_PORTA | PIN0)
#define PIN_CMP0_IN1              (PIN_ANALOG | PIN_PORTA | PIN1)
#define PIN_CMP0_IN2              (PIN_ANALOG | PIN_PORTC | PIN4)
#define PIN_CMP0_IN3              (PIN_ANALOG | PIN_PORTE | PIN8)
#define PIN_CMP0_IN4              (PIN_ANALOG | PIN_PORTC | PIN3)
#define PIN_CMP0_IN5              (PIN_ANALOG | PIN_PORTC | PIN2)
#define PIN_CMP0_IN6              (PIN_ANALOG | PIN_PORTD | PIN7)
#define PIN_CMP0_IN7              (PIN_ANALOG | PIN_PORTD | PIN6)
#define PIN_CMP0_OUT_1            (PIN_ALT4   | PIN_PORTA | PIN4)
#define PIN_CMP0_OUT_2            (PIN_ALT7   | PIN_PORTE | PIN3)
#define PIN_CMP0_RRT_1            (PIN_ALT5   | PIN_PORTA | PIN11)
#define PIN_CMP0_RRT_2            (PIN_ALT5   | PIN_PORTD | PIN16)

/* External Watchdog Monitor (EWM) */

#define PIN_EWM_IN_1              (PIN_ALT4   | PIN_PORTA | PIN14)
#define PIN_EWM_IN_2              (PIN_ALT4   | PIN_PORTA | PIN3)
#define PIN_EWM_IN_3              (PIN_ALT5   | PIN_PORTC | PIN4)
#define PIN_EWM_IN_4              (PIN_ALT7   | PIN_PORTE | PIN5)
#define PIN_EWM_OUT_1             (PIN_ALT4   | PIN_PORTA | PIN17)
#define PIN_EWM_OUT_2             (PIN_ALT4   | PIN_PORTA | PIN2)
#define PIN_EWM_OUT_3             (PIN_ALT5   | PIN_PORTA | PIN4)
#define PIN_EWM_OUT_4             (PIN_ALT7   | PIN_PORTE | PIN4)

/* FlexTimer Module (FTM) */

#define PIN_FTM0_CH0_1            (PIN_ALT2   | PIN_PORTB | PIN12)
#define PIN_FTM0_CH0_2            (PIN_ALT2   | PIN_PORTC | PIN0)
#define PIN_FTM0_CH0_3            (PIN_ALT2   | PIN_PORTD | PIN15)
#define PIN_FTM0_CH1_1            (PIN_ALT2   | PIN_PORTB | PIN13)
#define PIN_FTM0_CH1_2            (PIN_ALT2   | PIN_PORTC | PIN1)
#define PIN_FTM0_CH1_3            (PIN_ALT2   | PIN_PORTD | PIN16)
#define PIN_FTM0_CH2_1            (PIN_ALT2   | PIN_PORTB | PIN14)
#define PIN_FTM0_CH2_2            (PIN_ALT2   | PIN_PORTC | PIN2)
#define PIN_FTM0_CH2_3            (PIN_ALT2   | PIN_PORTD | PIN0)
#define PIN_FTM0_CH3_1            (PIN_ALT2   | PIN_PORTB | PIN15)
#define PIN_FTM0_CH3_2            (PIN_ALT2   | PIN_PORTC | PIN3)
#define PIN_FTM0_CH3_3            (PIN_ALT2   | PIN_PORTD | PIN1)
#define PIN_FTM0_CH4_1            (PIN_ALT2   | PIN_PORTB | PIN16)
#define PIN_FTM0_CH4_2            (PIN_ALT2   | PIN_PORTB | PIN4)
#define PIN_FTM0_CH5_1            (PIN_ALT2   | PIN_PORTB | PIN17)
#define PIN_FTM0_CH5_2            (PIN_ALT2   | PIN_PORTB | PIN5)
#define PIN_FTM0_CH6_1            (PIN_ALT2   | PIN_PORTA | PIN17)
#define PIN_FTM0_CH6_2            (PIN_ALT2   | PIN_PORTE | PIN8)
#define PIN_FTM0_CH7_1            (PIN_ALT2   | PIN_PORTE | PIN7)
#define PIN_FTM0_CH7_2            (PIN_ALT2   | PIN_PORTE | PIN9)
#define PIN_FTM0_FLT0_1           (PIN_ALT2   | PIN_PORTA | PIN14)
#define PIN_FTM0_FLT0_2           (PIN_ALT2   | PIN_PORTE | PIN3)
#define PIN_FTM0_FLT1_1           (PIN_ALT2   | PIN_PORTA | PIN6)
#define PIN_FTM0_FLT1_2           (PIN_ALT2   | PIN_PORTE | PIN14)
#define PIN_FTM0_FLT2_1           (PIN_ALT2   | PIN_PORTA | PIN7)
#define PIN_FTM0_FLT2_2           (PIN_ALT2   | PIN_PORTD | PIN17)
#define PIN_FTM0_FLT3_1           (PIN_ALT2   | PIN_PORTD | PIN4)
#define PIN_FTM0_FLT3_2           (PIN_ALT2   | PIN_PORTE | PIN12)

#define PIN_FTM1_CH0_1            (PIN_ALT2   | PIN_PORTB | PIN2)
#define PIN_FTM1_CH0_2            (PIN_ALT2   | PIN_PORTC | PIN4)
#define PIN_FTM1_CH1_1            (PIN_ALT2   | PIN_PORTA | PIN1)
#define PIN_FTM1_CH1_2            (PIN_ALT2   | PIN_PORTB | PIN3)
#define PIN_FTM1_CH2_1            (PIN_ALT2   | PIN_PORTA | PIN15)
#define PIN_FTM1_CH2_2            (PIN_ALT2   | PIN_PORTC | PIN14)
#define PIN_FTM1_CH3_1            (PIN_ALT2   | PIN_PORTA | PIN16)
#define PIN_FTM1_CH3_2            (PIN_ALT2   | PIN_PORTC | PIN15)
#define PIN_FTM1_CH4_1            (PIN_ALT2   | PIN_PORTA | PIN10)
#define PIN_FTM1_CH4_2            (PIN_ALT6   | PIN_PORTD | PIN8)
#define PIN_FTM1_CH5_1            (PIN_ALT2   | PIN_PORTA | PIN11)
#define PIN_FTM1_CH5_2            (PIN_ALT6   | PIN_PORTD | PIN9)
#define PIN_FTM1_CH6_1            (PIN_ALT2   | PIN_PORTA | PIN12)
#define PIN_FTM1_CH6_2            (PIN_ALT6   | PIN_PORTC | PIN0)
#define PIN_FTM1_CH7_1            (PIN_ALT2   | PIN_PORTA | PIN13)
#define PIN_FTM1_CH7_2            (PIN_ALT6   | PIN_PORTC | PIN1)
#define PIN_FTM1_FLT0_1           (PIN_ALT3   | PIN_PORTC | PIN8)
#define PIN_FTM1_FLT0_2           (PIN_ALT6   | PIN_PORTA | PIN14)
#define PIN_FTM1_FLT1_1           (PIN_ALT3   | PIN_PORTC | PIN9)
#define PIN_FTM1_FLT1_2           (PIN_ALT6   | PIN_PORTE | PIN1)
#define PIN_FTM1_FLT2_1           (PIN_ALT2   | PIN_PORTC | PIN16)
#define PIN_FTM1_FLT2_2           (PIN_ALT6   | PIN_PORTE | PIN0)
#define PIN_FTM1_FLT3_1           (PIN_ALT2   | PIN_PORTC | PIN17)
#define PIN_FTM1_FLT3_2           (PIN_ALT6   | PIN_PORTA | PIN9)
#define PIN_FTM1_QD_PHA_1         (PIN_ALT4   | PIN_PORTB | PIN3)
#define PIN_FTM1_QD_PHA_2         (PIN_ALT5   | PIN_PORTA | PIN1)
#define PIN_FTM1_QD_PHA_3         (PIN_ALT6   | PIN_PORTC | PIN7)
#define PIN_FTM1_QD_PHB_1         (PIN_ALT4   | PIN_PORTB | PIN2)
#define PIN_FTM1_QD_PHB_2         (PIN_ALT6   | PIN_PORTC | PIN4)
#define PIN_FTM1_QD_PHB_3         (PIN_ALT6   | PIN_PORTC | PIN6)

#define PIN_FTM2_CH0_1            (PIN_ALT2   | PIN_PORTC | PIN5)
#define PIN_FTM2_CH0_2            (PIN_ALT2   | PIN_PORTD | PIN10)
#define PIN_FTM2_CH0_3            (PIN_ALT4   | PIN_PORTD | PIN0)
#define PIN_FTM2_CH1_1            (PIN_ALT2   | PIN_PORTA | PIN0)
#define PIN_FTM2_CH1_2            (PIN_ALT2   | PIN_PORTD | PIN11)
#define PIN_FTM2_CH1_3            (PIN_ALT4   | PIN_PORTD | PIN1)
#define PIN_FTM2_CH2_1            (PIN_ALT2   | PIN_PORTD | PIN12)
#define PIN_FTM2_CH2_2            (PIN_ALT4   | PIN_PORTE | PIN4)
#define PIN_FTM2_CH3_1            (PIN_ALT2   | PIN_PORTD | PIN5)
#define PIN_FTM2_CH3_2            (PIN_ALT4   | PIN_PORTE | PIN5)
#define PIN_FTM2_CH4_1            (PIN_ALT2   | PIN_PORTD | PIN13)
#define PIN_FTM2_CH4_2            (PIN_ALT4   | PIN_PORTE | PIN10)
#define PIN_FTM2_CH5_1            (PIN_ALT2   | PIN_PORTD | PIN14)
#define PIN_FTM2_CH5_2            (PIN_ALT4   | PIN_PORTE | PIN11)
#define PIN_FTM2_CH6_1            (PIN_ALT3   | PIN_PORTC | PIN12)
#define PIN_FTM2_CH6_2            (PIN_ALT4   | PIN_PORTE | PIN15)
#define PIN_FTM2_CH7_1            (PIN_ALT3   | PIN_PORTC | PIN13)
#define PIN_FTM2_CH7_2            (PIN_ALT4   | PIN_PORTE | PIN16)
#define PIN_FTM2_FLT0_1           (PIN_ALT4   | PIN_PORTE | PIN13)
#define PIN_FTM2_FLT0_2           (PIN_ALT4   | PIN_PORTE | PIN3)
#define PIN_FTM2_FLT1_1           (PIN_ALT4   | PIN_PORTD | PIN5)
#define PIN_FTM2_FLT1_2           (PIN_ALT4   | PIN_PORTE | PIN14)
#define PIN_FTM2_FLT2_1           (PIN_ALT4   | PIN_PORTD | PIN6)
#define PIN_FTM2_FLT2_2           (PIN_ALT4   | PIN_PORTD | PIN8)
#define PIN_FTM2_FLT3_1           (PIN_ALT4   | PIN_PORTD | PIN7)
#define PIN_FTM2_FLT3_2           (PIN_ALT4   | PIN_PORTD | PIN9)
#define PIN_FTM2_QD_PHA_1         (PIN_ALT3   | PIN_PORTD | PIN11)
#define PIN_FTM2_QD_PHA_2         (PIN_ALT3   | PIN_PORTE | PIN5)
#define PIN_FTM2_QD_PHA_3         (PIN_ALT5   | PIN_PORTA | PIN0)
#define PIN_FTM2_QD_PHA_4         (PIN_ALT6   | PIN_PORTA | PIN13)
#define PIN_FTM2_QD_PHB_1         (PIN_ALT3   | PIN_PORTD | PIN10)
#define PIN_FTM2_QD_PHB_2         (PIN_ALT3   | PIN_PORTE | PIN4)
#define PIN_FTM2_QD_PHB_3         (PIN_ALT6   | PIN_PORTA | PIN12)
#define PIN_FTM2_QD_PHB_4         (PIN_ALT6   | PIN_PORTC | PIN5)

#define PIN_FTM3_CH0_1            (PIN_ALT2   | PIN_PORTA | PIN2)
#define PIN_FTM3_CH0_2            (PIN_ALT2   | PIN_PORTB | PIN8)
#define PIN_FTM3_CH1_1            (PIN_ALT2   | PIN_PORTA | PIN3)
#define PIN_FTM3_CH1_2            (PIN_ALT2   | PIN_PORTB | PIN9)
#define PIN_FTM3_CH2_1            (PIN_ALT2   | PIN_PORTB | PIN10)
#define PIN_FTM3_CH2_2            (PIN_ALT4   | PIN_PORTC | PIN6)
#define PIN_FTM3_CH3_1            (PIN_ALT2   | PIN_PORTB | PIN11)
#define PIN_FTM3_CH3_2            (PIN_ALT4   | PIN_PORTC | PIN7)
#define PIN_FTM3_CH4_1            (PIN_ALT2   | PIN_PORTC | PIN10)
#define PIN_FTM3_CH4_2            (PIN_ALT2   | PIN_PORTD | PIN2)
#define PIN_FTM3_CH5_1            (PIN_ALT2   | PIN_PORTC | PIN11)
#define PIN_FTM3_CH5_2            (PIN_ALT2   | PIN_PORTD | PIN3)
#define PIN_FTM3_CH6_1            (PIN_ALT2   | PIN_PORTC | PIN12)
#define PIN_FTM3_CH6_2            (PIN_ALT4   | PIN_PORTE | PIN2)
#define PIN_FTM3_CH7_1            (PIN_ALT2   | PIN_PORTC | PIN13)
#define PIN_FTM3_CH7_2            (PIN_ALT4   | PIN_PORTE | PIN6)
#define PIN_FTM3_FLT0_1           (PIN_ALT3   | PIN_PORTA | PIN17)
#define PIN_FTM3_FLT0_2           (PIN_ALT3   | PIN_PORTE | PIN7)
#define PIN_FTM3_FLT1_1           (PIN_ALT3   | PIN_PORTA | PIN14)
#define PIN_FTM3_FLT1_2           (PIN_ALT3   | PIN_PORTB | PIN13)
#define PIN_FTM3_FLT2_1           (PIN_ALT3   | PIN_PORTB | PIN12)
#define PIN_FTM3_FLT2_2           (PIN_ALT5   | PIN_PORTA | PIN9)
#define PIN_FTM3_FLT3_1           (PIN_ALT3   | PIN_PORTD | PIN4)
#define PIN_FTM3_FLT3_2           (PIN_ALT5   | PIN_PORTA | PIN8)

/* FlexIO */

#define PIN_FXIO_D0_1             (PIN_ALT3   | PIN_PORTD | PIN9)
#define PIN_FXIO_D0_2             (PIN_ALT4   | PIN_PORTA | PIN10)
#define PIN_FXIO_D0_3             (PIN_ALT6   | PIN_PORTD | PIN0)
#define PIN_FXIO_D1_1             (PIN_ALT4   | PIN_PORTA | PIN11)
#define PIN_FXIO_D1_2             (PIN_ALT5   | PIN_PORTD | PIN8)
#define PIN_FXIO_D1_3             (PIN_ALT6   | PIN_PORTD | PIN1)
#define PIN_FXIO_D2_1             (PIN_ALT4   | PIN_PORTA | PIN0)
#define PIN_FXIO_D2_2             (PIN_ALT6   | PIN_PORTE | PIN15)
#define PIN_FXIO_D3_1             (PIN_ALT4   | PIN_PORTA | PIN1)
#define PIN_FXIO_D3_2             (PIN_ALT6   | PIN_PORTE | PIN16)
#define PIN_FXIO_D4_1             (PIN_ALT4   | PIN_PORTD | PIN2)
#define PIN_FXIO_D4_2             (PIN_ALT5   | PIN_PORTA | PIN2)
#define PIN_FXIO_D4_3             (PIN_ALT6   | PIN_PORTE | PIN10)
#define PIN_FXIO_D5_1             (PIN_ALT4   | PIN_PORTD | PIN3)
#define PIN_FXIO_D5_2             (PIN_ALT5   | PIN_PORTA | PIN3)
#define PIN_FXIO_D5_3             (PIN_ALT6   | PIN_PORTE | PIN11)
#define PIN_FXIO_D6_1             (PIN_ALT4   | PIN_PORTA | PIN8)
#define PIN_FXIO_D6_2             (PIN_ALT5   | PIN_PORTD | PIN2)
#define PIN_FXIO_D6_3             (PIN_ALT6   | PIN_PORTE | PIN4)
#define PIN_FXIO_D7_1             (PIN_ALT4   | PIN_PORTA | PIN9)
#define PIN_FXIO_D7_2             (PIN_ALT5   | PIN_PORTD | PIN3)
#define PIN_FXIO_D7_3             (PIN_ALT6   | PIN_PORTE | PIN5)

/* JTAG */

#define PIN_JTAG_TCLK             (PIN_ALT7   | PIN_PORTC | PIN4)
#define PIN_JTAG_TDI              (PIN_ALT7   | PIN_PORTC | PIN5)
#define PIN_JTAG_TDO              (PIN_ALT7   | PIN_PORTA | PIN10)
#define PIN_JTAG_TMS              (PIN_ALT7   | PIN_PORTA | PIN4)

/* LPI2C */

#define PIN_LPI2C0_HREQ_1         (PIN_ALT3   | PIN_PORTB | PIN11)
#define PIN_LPI2C0_HREQ_2         (PIN_ALT3   | PIN_PORTE | PIN1)
#define PIN_LPI2C0_SCL_1          (PIN_ALT2   | PIN_PORTB | PIN7)
#define PIN_LPI2C0_SCL_2          (PIN_ALT3   | PIN_PORTA | PIN3)
#define PIN_LPI2C0_SCLS_1         (PIN_ALT3   | PIN_PORTA | PIN0)
#define PIN_LPI2C0_SCLS_2         (PIN_ALT3   | PIN_PORTB | PIN9)
#define PIN_LPI2C0_SDA_1          (PIN_ALT2   | PIN_PORTB | PIN6)
#define PIN_LPI2C0_SDA_2          (PIN_ALT3   | PIN_PORTA | PIN2)
#define PIN_LPI2C0_SDAS_1         (PIN_ALT3   | PIN_PORTA | PIN1)
#define PIN_LPI2C0_SDAS_2         (PIN_ALT3   | PIN_PORTB | PIN10)

/* LPSPI */

#define PIN_LPSPI0_PCS0_1         (PIN_ALT3   | PIN_PORTB | PIN0)
#define PIN_LPSPI0_PCS0_2         (PIN_ALT4   | PIN_PORTB | PIN5)
#define PIN_LPSPI0_PCS1           (PIN_ALT3   | PIN_PORTB | PIN5)
#define PIN_LPSPI0_PCS2           (PIN_ALT2   | PIN_PORTE | PIN6)
#define PIN_LPSPI0_PCS3           (PIN_ALT3   | PIN_PORTA | PIN15)
#define PIN_LPSPI0_SCK_1          (PIN_ALT2   | PIN_PORTE | PIN0)
#define PIN_LPSPI0_SCK_2          (PIN_ALT3   | PIN_PORTB | PIN2)
#define PIN_LPSPI0_SCK_3          (PIN_ALT4   | PIN_PORTD | PIN15)
#define PIN_LPSPI0_SIN_1          (PIN_ALT2   | PIN_PORTE | PIN1)
#define PIN_LPSPI0_SIN_2          (PIN_ALT3   | PIN_PORTB | PIN3)
#define PIN_LPSPI0_SIN_3          (PIN_ALT4   | PIN_PORTD | PIN16)
#define PIN_LPSPI0_SOUT_1         (PIN_ALT2   | PIN_PORTE | PIN2)
#define PIN_LPSPI0_SOUT_2         (PIN_ALT3   | PIN_PORTB | PIN1)
#define PIN_LPSPI0_SOUT_3         (PIN_ALT3   | PIN_PORTB | PIN4)

#define PIN_LPSPI1_PCS0_1         (PIN_ALT3   | PIN_PORTD | PIN3)
#define PIN_LPSPI1_PCS0_2         (PIN_ALT5   | PIN_PORTE | PIN1)
#define PIN_LPSPI1_PCS1           (PIN_ALT3   | PIN_PORTA | PIN6)
#define PIN_LPSPI1_PCS2           (PIN_ALT3   | PIN_PORTA | PIN16)
#define PIN_LPSPI1_PCS3           (PIN_ALT3   | PIN_PORTB | PIN17)
#define PIN_LPSPI1_SCK_1          (PIN_ALT3   | PIN_PORTB | PIN14)
#define PIN_LPSPI1_SCK_2          (PIN_ALT3   | PIN_PORTD | PIN0)
#define PIN_LPSPI1_SIN_1          (PIN_ALT3   | PIN_PORTB | PIN15)
#define PIN_LPSPI1_SIN_2          (PIN_ALT3   | PIN_PORTD | PIN1)
#define PIN_LPSPI1_SOUT_1         (PIN_ALT3   | PIN_PORTB | PIN16)
#define PIN_LPSPI1_SOUT_2         (PIN_ALT3   | PIN_PORTD | PIN2)
#define PIN_LPSPI1_SOUT_3         (PIN_ALT5   | PIN_PORTE | PIN0)

#define PIN_LPSPI2_PCS0_1         (PIN_ALT2   | PIN_PORTE | PIN11)
#define PIN_LPSPI2_PCS0_2         (PIN_ALT3   | PIN_PORTA | PIN9)
#define PIN_LPSPI2_PCS0_3         (PIN_ALT3   | PIN_PORTC | PIN14)
#define PIN_LPSPI2_PCS1           (PIN_ALT3   | PIN_PORTE | PIN10)
#define PIN_LPSPI2_PCS2           (PIN_ALT3   | PIN_PORTE | PIN13)
#define PIN_LPSPI2_PCS3           (PIN_ALT4   | PIN_PORTA | PIN15)
#define PIN_LPSPI2_SCK_1          (PIN_ALT3   | PIN_PORTC | PIN15)
#define PIN_LPSPI2_SCK_2          (PIN_ALT3   | PIN_PORTE | PIN15)
#define PIN_LPSPI2_SIN_1          (PIN_ALT3   | PIN_PORTC | PIN0)
#define PIN_LPSPI2_SIN_2          (PIN_ALT3   | PIN_PORTE | PIN16)
#define PIN_LPSPI2_SOUT_1         (PIN_ALT3   | PIN_PORTA | PIN8)
#define PIN_LPSPI2_SOUT_2         (PIN_ALT3   | PIN_PORTC | PIN1)

/* LPTimer */

#define PIN_LPTMR0_ALT1           (PIN_ALT3   | PIN_PORTE | PIN11)
#define PIN_LPTMR0_ALT2           (PIN_ALT3   | PIN_PORTD | PIN5)
#define PIN_LPTMR0_ALT3_1         (PIN_ALT3   | PIN_PORTE | PIN2)
#define PIN_LPTMR0_ALT3_2         (PIN_ALT4   | PIN_PORTB | PIN0)

/* LPUARTS */

#define PIN_LPUART0_CTS_1         (PIN_ALT6   | PIN_PORTA | PIN0)
#define PIN_LPUART0_CTS_2         (PIN_ALT6   | PIN_PORTC | PIN8)
#define PIN_LPUART0_RTS_1         (PIN_ALT6   | PIN_PORTA | PIN1)
#define PIN_LPUART0_RTS_2         (PIN_ALT6   | PIN_PORTC | PIN9)
#define PIN_LPUART0_RX_1          (PIN_ALT2   | PIN_PORTB | PIN0)
#define PIN_LPUART0_RX_2          (PIN_ALT4   | PIN_PORTC | PIN2)
#define PIN_LPUART0_RX_3          (PIN_ALT6   | PIN_PORTA | PIN2)
#define PIN_LPUART0_TX_1          (PIN_ALT2   | PIN_PORTB | PIN1)
#define PIN_LPUART0_TX_2          (PIN_ALT4   | PIN_PORTC | PIN3)
#define PIN_LPUART0_TX_3          (PIN_ALT6   | PIN_PORTA | PIN3)

#define PIN_LPUART1_CTS_1         (PIN_ALT2   | PIN_PORTE | PIN15)
#define PIN_LPUART1_CTS_2         (PIN_ALT6   | PIN_PORTA | PIN6)
#define PIN_LPUART1_CTS_3         (PIN_ALT6   | PIN_PORTE | PIN2)
#define PIN_LPUART1_RTS_1         (PIN_ALT2   | PIN_PORTE | PIN16)
#define PIN_LPUART1_RTS_2         (PIN_ALT6   | PIN_PORTA | PIN7)
#define PIN_LPUART1_RTS_3         (PIN_ALT6   | PIN_PORTE | PIN6)
#define PIN_LPUART1_RX_1          (PIN_ALT2   | PIN_PORTC | PIN6)
#define PIN_LPUART1_RX_2          (PIN_ALT2   | PIN_PORTC | PIN8)
#define PIN_LPUART1_RX_3          (PIN_ALT3   | PIN_PORTD | PIN13)
#define PIN_LPUART1_TX_1          (PIN_ALT2   | PIN_PORTC | PIN7)
#define PIN_LPUART1_TX_2          (PIN_ALT2   | PIN_PORTC | PIN9)
#define PIN_LPUART1_TX_3          (PIN_ALT3   | PIN_PORTD | PIN14)

#define PIN_LPUART2_CTS_1         (PIN_ALT3   | PIN_PORTE | PIN9)
#define PIN_LPUART2_CTS_2         (PIN_ALT4   | PIN_PORTC | PIN12)
#define PIN_LPUART2_CTS_3         (PIN_ALT6   | PIN_PORTD | PIN11)
#define PIN_LPUART2_RTS_1         (PIN_ALT3   | PIN_PORTE | PIN3)
#define PIN_LPUART2_RTS_2         (PIN_ALT4   | PIN_PORTC | PIN13)
#define PIN_LPUART2_RTS_3         (PIN_ALT6   | PIN_PORTD | PIN12)
#define PIN_LPUART2_RX_1          (PIN_ALT2   | PIN_PORTA | PIN8)
#define PIN_LPUART2_RX_2          (PIN_ALT2   | PIN_PORTD | PIN6)
#define PIN_LPUART2_RX_3          (PIN_ALT3   | PIN_PORTD | PIN17)
#define PIN_LPUART2_TX_1          (PIN_ALT2   | PIN_PORTA | PIN9)
#define PIN_LPUART2_TX_2          (PIN_ALT2   | PIN_PORTD | PIN7)
#define PIN_LPUART2_TX_3          (PIN_ALT3   | PIN_PORTE | PIN12)

/* NMI */

#define PIN_NMI                   (PIN_ALT7   | PIN_PORTD | PIN3)

/* GPIO */

#define PIN_PTA0                  (PIN_ALT1   | PIN_PORTA | PIN0)
#define PIN_PTA1                  (PIN_ALT1   | PIN_PORTA | PIN1)
#define PIN_PTA2                  (PIN_ALT1   | PIN_PORTA | PIN2)
#define PIN_PTA3                  (PIN_ALT1   | PIN_PORTA | PIN3)
#define PIN_PTA4                  (PIN_ALT1   | PIN_PORTA | PIN4)
#define PIN_PTA5                  (PIN_ALT1   | PIN_PORTA | PIN5)
#define PIN_PTA6                  (PIN_ALT1   | PIN_PORTA | PIN6)
#define PIN_PTA7                  (PIN_ALT1   | PIN_PORTA | PIN7)
#define PIN_PTA8                  (PIN_ALT1   | PIN_PORTA | PIN8)
#define PIN_PTA9                  (PIN_ALT1   | PIN_PORTA | PIN9)
#define PIN_PTA10                 (PIN_ALT1   | PIN_PORTA | PIN10)
#define PIN_PTA11                 (PIN_ALT1   | PIN_PORTA | PIN11)
#define PIN_PTA12                 (PIN_ALT1   | PIN_PORTA | PIN12)
#define PIN_PTA13                 (PIN_ALT1   | PIN_PORTA | PIN13)
#define PIN_PTA14                 (PIN_ALT1   | PIN_PORTA | PIN14)
#define PIN_PTA15                 (PIN_ALT1   | PIN_PORTA | PIN15)
#define PIN_PTA16                 (PIN_ALT1   | PIN_PORTA | PIN16)
#define PIN_PTA17                 (PIN_ALT1   | PIN_PORTA | PIN17)

#define PIN_PTB0                  (PIN_ALT1   | PIN_PORTB | PIN0)
#define PIN_PTB1                  (PIN_ALT1   | PIN_PORTB | PIN1)
#define PIN_PTB2                  (PIN_ALT1   | PIN_PORTB | PIN2)
#define PIN_PTB3                  (PIN_ALT1   | PIN_PORTB | PIN3)
#define PIN_PTB4                  (PIN_ALT1   | PIN_PORTB | PIN4)
#define PIN_PTB5                  (PIN_ALT1   | PIN_PORTB | PIN5)
#define PIN_PTB6                  (PIN_ALT1   | PIN_PORTB | PIN6)
#define PIN_PTB7                  (PIN_ALT1   | PIN_PORTB | PIN7)
#define PIN_PTB8                  (PIN_ALT1   | PIN_PORTB | PIN8)
#define PIN_PTB9                  (PIN_ALT1   | PIN_PORTB | PIN9)
#define PIN_PTB10                 (PIN_ALT1   | PIN_PORTB | PIN10)
#define PIN_PTB11                 (PIN_ALT1   | PIN_PORTB | PIN11)
#define PIN_PTB12                 (PIN_ALT1   | PIN_PORTB | PIN12)
#define PIN_PTB13                 (PIN_ALT1   | PIN_PORTB | PIN13)
#define PIN_PTB14                 (PIN_ALT1   | PIN_PORTB | PIN14)
#define PIN_PTB15                 (PIN_ALT1   | PIN_PORTB | PIN15)
#define PIN_PTB16                 (PIN_ALT1   | PIN_PORTB | PIN16)
#define PIN_PTB17                 (PIN_ALT1   | PIN_PORTB | PIN17)

#define PIN_PTC0                  (PIN_ALT1   | PIN_PORTC | PIN0)
#define PIN_PTC1                  (PIN_ALT1   | PIN_PORTC | PIN1)
#define PIN_PTC2                  (PIN_ALT1   | PIN_PORTC | PIN2)
#define PIN_PTC3                  (PIN_ALT1   | PIN_PORTC | PIN3)
#define PIN_PTC4                  (PIN_ALT1   | PIN_PORTC | PIN4)
#define PIN_PTC5                  (PIN_ALT1   | PIN_PORTC | PIN5)
#define PIN_PTC6                  (PIN_ALT1   | PIN_PORTC | PIN6)
#define PIN_PTC7                  (PIN_ALT1   | PIN_PORTC | PIN7)
#define PIN_PTC8                  (PIN_ALT1   | PIN_PORTC | PIN8)
#define PIN_PTC9                  (PIN_ALT1   | PIN_PORTC | PIN9)
#define PIN_PTC10                 (PIN_ALT1   | PIN_PORTC | PIN10)
#define PIN_PTC11                 (PIN_ALT1   | PIN_PORTC | PIN11)
#define PIN_PTC12                 (PIN_ALT1   | PIN_PORTC | PIN12)
#define PIN_PTC13                 (PIN_ALT1   | PIN_PORTC | PIN13)
#define PIN_PTC14                 (PIN_ALT1   | PIN_PORTC | PIN14)
#define PIN_PTC15                 (PIN_ALT1   | PIN_PORTC | PIN15)
#define PIN_PTC16                 (PIN_ALT1   | PIN_PORTC | PIN16)
#define PIN_PTC17                 (PIN_ALT1   | PIN_PORTC | PIN17)

#define PIN_PTD0                  (PIN_ALT1   | PIN_PORTD | PIN0)
#define PIN_PTD1                  (PIN_ALT1   | PIN_PORTD | PIN1)
#define PIN_PTD2                  (PIN_ALT1   | PIN_PORTD | PIN2)
#define PIN_PTD3                  (PIN_ALT1   | PIN_PORTD | PIN3)
#define PIN_PTD4                  (PIN_ALT1   | PIN_PORTD | PIN4)
#define PIN_PTD5                  (PIN_ALT1   | PIN_PORTD | PIN5)
#define PIN_PTD6                  (PIN_ALT1   | PIN_PORTD | PIN6)
#define PIN_PTD7                  (PIN_ALT1   | PIN_PORTD | PIN7)
#define PIN_PTD8                  (PIN_ALT1   | PIN_PORTD | PIN8)
#define PIN_PTD9                  (PIN_ALT1   | PIN_PORTD | PIN9)
#define PIN_PTD10                 (PIN_ALT1   | PIN_PORTD | PIN10)
#define PIN_PTD11                 (PIN_ALT1   | PIN_PORTD | PIN11)
#define PIN_PTD12                 (PIN_ALT1   | PIN_PORTD | PIN12)
#define PIN_PTD13                 (PIN_ALT1   | PIN_PORTD | PIN13)
#define PIN_PTD14                 (PIN_ALT1   | PIN_PORTD | PIN14)
#define PIN_PTD15                 (PIN_ALT1   | PIN_PORTD | PIN15)
#define PIN_PTD16                 (PIN_ALT1   | PIN_PORTD | PIN16)
#define PIN_PTD17                 (PIN_ALT1   | PIN_PORTD | PIN17)

#define PIN_PTE0                  (PIN_ALT1   | PIN_PORTE | PIN0)
#define PIN_PTE1                  (PIN_ALT1   | PIN_PORTE | PIN1)
#define PIN_PTE2                  (PIN_ALT1   | PIN_PORTE | PIN2)
#define PIN_PTE3                  (PIN_ALT1   | PIN_PORTE | PIN3)
#define PIN_PTE4                  (PIN_ALT1   | PIN_PORTE | PIN4)
#define PIN_PTE5                  (PIN_ALT1   | PIN_PORTE | PIN5)
#define PIN_PTE6                  (PIN_ALT1   | PIN_PORTE | PIN6)
#define PIN_PTE7                  (PIN_ALT1   | PIN_PORTE | PIN7)
#define PIN_PTE8                  (PIN_ALT1   | PIN_PORTE | PIN8)
#define PIN_PTE9                  (PIN_ALT1   | PIN_PORTE | PIN9)
#define PIN_PTE10                 (PIN_ALT1   | PIN_PORTE | PIN10)
#define PIN_PTE11                 (PIN_ALT1   | PIN_PORTE | PIN11)
#define PIN_PTE12                 (PIN_ALT1   | PIN_PORTE | PIN12)
#define PIN_PTE13                 (PIN_ALT1   | PIN_PORTE | PIN13)
#define PIN_PTE14                 (PIN_ALT1   | PIN_PORTE | PIN14)
#define PIN_PTE15                 (PIN_ALT1   | PIN_PORTE | PIN15)
#define PIN_PTE16                 (PIN_ALT1   | PIN_PORTE | PIN16)

/* Reset */

#define PIN_RESET                 (PIN_ALT7   | PIN_PORTA | PIN5)

/* RTC */

#define PIN_RTC_CLKIN             (PIN_ALT4   | PIN_PORTA | PIN7)
#define PIN_RTC_CLKOUT_1          (PIN_ALT3   | PIN_PORTC | PIN4)
#define PIN_RTC_CLKOUT_2          (PIN_ALT3   | PIN_PORTC | PIN5)
#define PIN_RTC_CLKOUT_3          (PIN_ALT7   | PIN_PORTD | PIN13)

/* SWD */

#define PIN_TRACE_SWO             (PIN_ALT7   | PIN_PORTA | PIN10)
#define PIN_SWD_CLK               (PIN_ALT7   | PIN_PORTC | PIN4)
#define PIN_SWD_DIO               (PIN_ALT7   | PIN_PORTA | PIN4)

/* Test Clock Input (TCLK) */

#define PIN_TCLK0                 (PIN_ALT4   | PIN_PORTB | PIN1)
#define PIN_TCLK1_1               (PIN_ALT3   | PIN_PORTA | PIN5)
#define PIN_TCLK1_2               (PIN_ALT3   | PIN_PORTE | PIN0)
#define PIN_TCLK2                 (PIN_ALT2   | PIN_PORTE | PIN5)

/* Trigger Mux Control (TRGMUX) */

#define PIN_TRGMUX_IN0            (PIN_ALT6   | PIN_PORTB | PIN5)
#define PIN_TRGMUX_IN1            (PIN_ALT6   | PIN_PORTB | PIN4)
#define PIN_TRGMUX_IN2            (PIN_ALT6   | PIN_PORTB | PIN3)
#define PIN_TRGMUX_IN3            (PIN_ALT6   | PIN_PORTB | PIN2)
#define PIN_TRGMUX_IN4            (PIN_ALT6   | PIN_PORTD | PIN3)
#define PIN_TRGMUX_IN5            (PIN_ALT6   | PIN_PORTD | PIN2)
#define PIN_TRGMUX_IN6            (PIN_ALT6   | PIN_PORTE | PIN3)
#define PIN_TRGMUX_IN7            (PIN_ALT6   | PIN_PORTD | PIN5)
#define PIN_TRGMUX_IN8            (PIN_ALT6   | PIN_PORTC | PIN15)
#define PIN_TRGMUX_IN9            (PIN_ALT6   | PIN_PORTC | PIN14)
#define PIN_TRGMUX_IN10           (PIN_ALT6   | PIN_PORTC | PIN11)
#define PIN_TRGMUX_IN11           (PIN_ALT6   | PIN_PORTC | PIN10)
#define PIN_TRGMUX_OUT0           (PIN_ALT7   | PIN_PORTA | PIN1)
#define PIN_TRGMUX_OUT1           (PIN_ALT7   | PIN_PORTD | PIN0)
#define PIN_TRGMUX_OUT2           (PIN_ALT7   | PIN_PORTD | PIN1)
#define PIN_TRGMUX_OUT3           (PIN_ALT7   | PIN_PORTA | PIN0)
#define PIN_TRGMUX_OUT4           (PIN_ALT7   | PIN_PORTE | PIN10)
#define PIN_TRGMUX_OUT5           (PIN_ALT7   | PIN_PORTE | PIN11)
#define PIN_TRGMUX_OUT6           (PIN_ALT7   | PIN_PORTE | PIN15)
#define PIN_TRGMUX_OUT7           (PIN_ALT7   | PIN_PORTE | PIN16)

/* External Crystal */

#define PIN_EXTAL                 (PIN_ANALOG | PIN_PORTB | PIN7)
#define PIN_XTAL                  (PIN_ANALOG | PIN_PORTB | PIN6)

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K144_PINMUX_H */
