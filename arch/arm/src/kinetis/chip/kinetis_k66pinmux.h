/********************************************************************************************
 * arch/arm/src/kinetis/chip/kinetis_k66pinmux.h
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_CHP_KINETIS_K66PINMUX_H
#define __ARCH_ARM_SRC_KINETIS_CHP_KINETIS_K66PINMUX_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef KINETIS_K66

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* In most cases, there are alternative configurations for various pins. Those alternative
 * pins are labeled with a suffix like _1, _2, etc. in order to distinguish them.  Logic in
 * the board.h file must select the correct pin configuration for the board by defining a pin
 * configuration (with no suffix) that maps to the correct alternative.
 *
 * WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!!
 * Additional effort is required to select specific GPIO options such as frequency,
 * open-drain/push-pull, and pull-up/down!  Just the basics are defined for most
 * pins in the initial version of this file.
 */

/* ADC */

#define PIN_ADC0_SE4B             (PIN_ANALOG | PIN_PORTC | PIN2)
#define PIN_ADC0_SE5B             (PIN_ANALOG | PIN_PORTD | PIN1)
#define PIN_ADC0_SE6B             (PIN_ANALOG | PIN_PORTD | PIN5)
#define PIN_ADC0_SE7B             (PIN_ANALOG | PIN_PORTD | PIN6)
#define PIN_ADC0_SE8              (PIN_ANALOG | PIN_PORTB | PIN0)
#define PIN_ADC0_SE9              (PIN_ANALOG | PIN_PORTB | PIN1)
#define PIN_ADC0_SE10             (PIN_ANALOG | PIN_PORTA | PIN7)
#define PIN_ADC0_SE11             (PIN_ANALOG | PIN_PORTA | PIN8)
#define PIN_ADC0_SE12             (PIN_ANALOG | PIN_PORTB | PIN2)
#define PIN_ADC0_SE13             (PIN_ANALOG | PIN_PORTB | PIN3)
#define PIN_ADC0_SE14             (PIN_ANALOG | PIN_PORTC | PIN0)
#define PIN_ADC0_SE15             (PIN_ANALOG | PIN_PORTC | PIN1)
#define PIN_ADC0_SE17             (PIN_ANALOG | PIN_PORTE | PIN24)
#define PIN_ADC0_SE18             (PIN_ANALOG | PIN_PORTE | PIN25)

#define PIN_ADC1_SE4A             (PIN_ANALOG | PIN_PORTE | PIN0)
#define PIN_ADC1_SE4B             (PIN_ANALOG | PIN_PORTC | PIN8)
#define PIN_ADC1_SE5A             (PIN_ANALOG | PIN_PORTE | PIN1)
#define PIN_ADC1_SE5B             (PIN_ANALOG | PIN_PORTC | PIN9)
#define PIN_ADC1_SE6A             (PIN_ANALOG | PIN_PORTE | PIN2)
#define PIN_ADC1_SE6B             (PIN_ANALOG | PIN_PORTC | PIN10)
#define PIN_ADC1_SE7A             (PIN_ANALOG | PIN_PORTE | PIN3)
#define PIN_ADC1_SE7B             (PIN_ANALOG | PIN_PORTC | PIN11)
#define PIN_ADC1_SE8              (PIN_ANALOG | PIN_PORTB | PIN0)
#define PIN_ADC1_SE9              (PIN_ANALOG | PIN_PORTB | PIN1)
#define PIN_ADC1_SE10             (PIN_ANALOG | PIN_PORTB | PIN4)
#define PIN_ADC1_SE11             (PIN_ANALOG | PIN_PORTB | PIN5)
#define PIN_ADC1_SE12             (PIN_ANALOG | PIN_PORTB | PIN6)
#define PIN_ADC1_SE13             (PIN_ANALOG | PIN_PORTB | PIN7)
#define PIN_ADC1_SE14             (PIN_ANALOG | PIN_PORTB | PIN10)
#define PIN_ADC1_SE15             (PIN_ANALOG | PIN_PORTB | PIN11)
#define PIN_ADC1_SE17             (PIN_ANALOG | PIN_PORTA | PIN17)

/* CAN */

#define PIN_CAN0_RX_1             (PIN_ALT2   | PIN_PORTA | PIN13)
#define PIN_CAN0_RX_2             (PIN_ALT2   | PIN_PORTB | PIN19)
#define PIN_CAN0_TX_1             (PIN_ALT2   | PIN_PORTA | PIN12)
#define PIN_CAN0_TX_2             (PIN_ALT2   | PIN_PORTB | PIN18)

#define PIN_CAN1_TX_1             (PIN_ALT2   | PIN_PORTE | PIN24)
#define PIN_CAN1_TX_2             (PIN_ALT2   | PIN_PORTC | PIN17)
#define PIN_CAN1_RX_1             (PIN_ALT2   | PIN_PORTE | PIN25)
#define PIN_CAN1_RX_2             (PIN_ALT2   | PIN_PORTC | PIN16)

/* Output clock */

#define PIN_CLKOUT_1              (PIN_ALT5   | PIN_PORTA | PIN6)
#define PIN_CLKOUT_2              (PIN_ALT5   | PIN_PORTC | PIN3)

/* Comparators */

#define PIN_CMP0_IN0              (PIN_ANALOG | PIN_PORTC | PIN6)
#define PIN_CMP0_IN1              (PIN_ANALOG | PIN_PORTC | PIN7)
#define PIN_CMP0_IN2              (PIN_ANALOG | PIN_PORTC | PIN8)
#define PIN_CMP0_IN3              (PIN_ANALOG | PIN_PORTC | PIN9)
#define PIN_CMP0_OUT_1            (PIN_ALT6   | PIN_PORTB | PIN20)
#define PIN_CMP0_OUT_2            (PIN_ALT6   | PIN_PORTC | PIN5)

#define PIN_CMP1_IN0              (PIN_ANALOG | PIN_PORTC | PIN2)
#define PIN_CMP1_IN1              (PIN_ANALOG | PIN_PORTC | PIN3)
#define PIN_CMP1_OUT_1            (PIN_ALT6   | PIN_PORTB | PIN21)
#define PIN_CMP1_OUT_2            (PIN_ALT6   | PIN_PORTC | PIN4)

#define PIN_CMP2_IN0              (PIN_ANALOG | PIN_PORTA | PIN12)
#define PIN_CMP2_IN1              (PIN_ANALOG | PIN_PORTA | PIN13)
#define PIN_CMP2_OUT_1            (PIN_ALT5   | PIN_PORTA | PIN5)
#define PIN_CMP2_OUT_2            (PIN_ALT6   | PIN_PORTB | PIN22)

#define PIN_CMP3_IN1              (PIN_ANALOG | PIN_PORTA | PIN15)
#define PIN_CMP3_IN2              (PIN_ANALOG | PIN_PORTA | PIN16)
#define PIN_CMP3_IN4              (PIN_ANALOG | PIN_PORTA | PIN24)
#define PIN_CMP3_IN5              (PIN_ANALOG | PIN_PORTA | PIN25)
#define PIN_CMP3_OUT_1            (PIN_ALT6   | PIN_PORTB | PIN23)

/* Carrier Modulator Transmittor (CMT) */

#define PIN_CMT_IRO               (PIN_ALT2   | PIN_PORTD | PIN7)

/* Ethernet */

#define PIN_ENET_1588_CLKIN       (PIN_ALT2   | PIN_PORTE | PIN26)
#define PIN_ENET0_1588_TMR0_1     (PIN_ALT4   | PIN_PORTB | PIN2)
#define PIN_ENET0_1588_TMR0_2     (PIN_ALT4   | PIN_PORTC | PIN16)
#define PIN_ENET0_1588_TMR1_1     (PIN_ALT4   | PIN_PORTB | PIN3)
#define PIN_ENET0_1588_TMR1_2     (PIN_ALT4   | PIN_PORTC | PIN17)
#define PIN_ENET0_1588_TMR2_1     (PIN_ALT4   | PIN_PORTB | PIN4)
#define PIN_ENET0_1588_TMR2_2     (PIN_ALT4   | PIN_PORTC | PIN18)
#define PIN_ENET0_1588_TMR3_1     (PIN_ALT4   | PIN_PORTB | PIN5)
#define PIN_ENET0_1588_TMR3_2     (PIN_ALT4   | PIN_PORTC | PIN19)

/* External Watchdog Monitor (EWM) */

#define PIN_EWM_IN_1              (PIN_ALT6   | PIN_PORTB | PIN16)
#define PIN_EWM_IN_2              (PIN_ALT6   | PIN_PORTD | PIN4)
#define PIN_EWM_IN_3              (PIN_ALT6   | PIN_PORTE | PIN25)
#define PIN_EWM_OUT_1             (PIN_ALT6   | PIN_PORTB | PIN17)
#define PIN_EWM_OUT_2             (PIN_ALT6   | PIN_PORTD | PIN5)
#define PIN_EWM_OUT_3             (PIN_ALT6   | PIN_PORTE | PIN24)

/* FlexBus */

#define PIN_FB_A16                (PIN_ALT6   | PIN_PORTD | PIN8)
#define PIN_FB_A17                (PIN_ALT6   | PIN_PORTD | PIN9)
#define PIN_FB_A18                (PIN_ALT6   | PIN_PORTD | PIN10)
#define PIN_FB_A19                (PIN_ALT6   | PIN_PORTD | PIN11)
#define PIN_FB_A20                (PIN_ALT6   | PIN_PORTD | PIN12)
#define PIN_FB_A21                (PIN_ALT6   | PIN_PORTD | PIN13)
#define PIN_FB_A22                (PIN_ALT6   | PIN_PORTD | PIN14)
#define PIN_FB_A23                (PIN_ALT6   | PIN_PORTD | PIN15)
#define PIN_FB_A24                (PIN_ALT6   | PIN_PORTA | PIN29)
#define PIN_FB_A25                (PIN_ALT6   | PIN_PORTA | PIN28)
#define PIN_FB_A26                (PIN_ALT6   | PIN_PORTA | PIN27)
#define PIN_FB_A27                (PIN_ALT6   | PIN_PORTA | PIN26)
#define PIN_FB_A28                (PIN_ALT6   | PIN_PORTA | PIN25)
#define PIN_FB_A29                (PIN_ALT6   | PIN_PORTA | PIN24)
#define PIN_FB_AD0                (PIN_ALT5   | PIN_PORTD | PIN6)
#define PIN_FB_AD1                (PIN_ALT5   | PIN_PORTD | PIN5)
#define PIN_FB_AD2                (PIN_ALT5   | PIN_PORTD | PIN4)
#define PIN_FB_AD3                (PIN_ALT5   | PIN_PORTD | PIN3)
#define PIN_FB_AD4                (PIN_ALT5   | PIN_PORTD | PIN2)
#define PIN_FB_AD5                (PIN_ALT5   | PIN_PORTC | PIN10)
#define PIN_FB_AD6                (PIN_ALT5   | PIN_PORTC | PIN9)
#define PIN_FB_AD7                (PIN_ALT5   | PIN_PORTC | PIN8)
#define PIN_FB_AD8                (PIN_ALT5   | PIN_PORTC | PIN7)
#define PIN_FB_AD9                (PIN_ALT5   | PIN_PORTC | PIN6)
#define PIN_FB_AD10               (PIN_ALT5   | PIN_PORTC | PIN5)
#define PIN_FB_AD11               (PIN_ALT5   | PIN_PORTC | PIN4)
#define PIN_FB_AD12               (PIN_ALT5   | PIN_PORTC | PIN2)
#define PIN_FB_AD13               (PIN_ALT5   | PIN_PORTC | PIN1)
#define PIN_FB_AD14               (PIN_ALT5   | PIN_PORTC | PIN0)
#define PIN_FB_AD15               (PIN_ALT5   | PIN_PORTB | PIN18)
#define PIN_FB_AD16               (PIN_ALT5   | PIN_PORTB | PIN17)
#define PIN_FB_AD17               (PIN_ALT5   | PIN_PORTB | PIN16)
#define PIN_FB_AD18               (PIN_ALT5   | PIN_PORTB | PIN11)
#define PIN_FB_AD19               (PIN_ALT5   | PIN_PORTB | PIN10)
#define PIN_FB_AD20               (PIN_ALT5   | PIN_PORTB | PIN9)
#define PIN_FB_AD21               (PIN_ALT5   | PIN_PORTB | PIN8)
#define PIN_FB_AD22               (PIN_ALT5   | PIN_PORTB | PIN7)
#define PIN_FB_AD23               (PIN_ALT5   | PIN_PORTB | PIN6)
#define PIN_FB_AD24               (PIN_ALT5   | PIN_PORTC | PIN15)
#define PIN_FB_AD25               (PIN_ALT5   | PIN_PORTC | PIN14)
#define PIN_FB_AD26               (PIN_ALT5   | PIN_PORTC | PIN13)
#define PIN_FB_AD27               (PIN_ALT5   | PIN_PORTC | PIN12)
#define PIN_FB_AD28               (PIN_ALT5   | PIN_PORTB | PIN23)
#define PIN_FB_AD29               (PIN_ALT5   | PIN_PORTB | PIN22)
#define PIN_FB_AD30               (PIN_ALT5   | PIN_PORTB | PIN21)
#define PIN_FB_AD31               (PIN_ALT5   | PIN_PORTB | PIN20)
#define PIN_FB_ALE                (PIN_ALT5   | PIN_PORTD | PIN0)
#define PIN_FB_BE15_8_BLS23_16    (PIN_ALT5   | PIN_PORTC | PIN18)
#define PIN_FB_BE23_16_BLS15_8    (PIN_ALT5   | PIN_PORTC | PIN16)
#define PIN_FB_BE31_24_BLS7_0     (PIN_ALT5   | PIN_PORTC | PIN17)
#define PIN_FB_BE7_0_BLS31_24     (PIN_ALT5   | PIN_PORTC | PIN19)
#define PIN_FB_CS0                (PIN_ALT5   | PIN_PORTD | PIN1)
#define PIN_FB_CS1                (PIN_ALT5   | PIN_PORTD | PIN0)
#define PIN_FB_CS2                (PIN_ALT5   | PIN_PORTC | PIN18)
#define PIN_FB_CS3                (PIN_ALT5   | PIN_PORTC | PIN19)
#define PIN_FB_CS4                (PIN_ALT5   | PIN_PORTC | PIN17)
#define PIN_FB_CS5                (PIN_ALT5   | PIN_PORTC | PIN16)
#define PIN_FB_OE                 (PIN_ALT5   | PIN_PORTB | PIN19)
#define PIN_FB_RW                 (PIN_ALT5   | PIN_PORTC | PIN11)
#define PIN_FB_TA                 (PIN_ALT6   | PIN_PORTC | PIN19)
#define PIN_FB_TBST               (PIN_ALT5   | PIN_PORTC | PIN18)
#define PIN_FB_TS                 (PIN_ALT5   | PIN_PORTD | PIN0)
#define PIN_FB_TSIZ0              (PIN_ALT5   | PIN_PORTC | PIN17)
#define PIN_FB_TSIZ1              (PIN_ALT5   | PIN_PORTC | PIN16)

/* FlexTimer Module (FTM) */

#define PIN_FTM_CLKIN0_1          (PIN_ALT4   | PIN_PORTA | PIN18)
#define PIN_FTM_CLKIN0_2          (PIN_ALT4   | PIN_PORTB | PIN16)
#define PIN_FTM_CLKIN0_3          (PIN_ALT4   | PIN_PORTC | PIN12)
#define PIN_FTM_CLKIN1_1          (PIN_ALT4   | PIN_PORTA | PIN19)
#define PIN_FTM_CLKIN1_2          (PIN_ALT4   | PIN_PORTB | PIN17)
#define PIN_FTM_CLKIN1_3          (PIN_ALT4   | PIN_PORTC | PIN13)

#define PIN_FTM0_CH0_1            (PIN_ALT3   | PIN_PORTA | PIN3)
#define PIN_FTM0_CH0_2            (PIN_ALT4   | PIN_PORTC | PIN1)
#define PIN_FTM0_CH1_1            (PIN_ALT3   | PIN_PORTA | PIN4)
#define PIN_FTM0_CH1_2            (PIN_ALT4   | PIN_PORTC | PIN2)
#define PIN_FTM0_CH2_1            (PIN_ALT3   | PIN_PORTA | PIN5)
#define PIN_FTM0_CH2_2            (PIN_ALT4   | PIN_PORTC | PIN3)
#define PIN_FTM0_CH2_3            (PIN_ALT7   | PIN_PORTC | PIN5)
#define PIN_FTM0_CH3_1            (PIN_ALT3   | PIN_PORTA | PIN6)
#define PIN_FTM0_CH3_2            (PIN_ALT4   | PIN_PORTC | PIN4)
#define PIN_FTM0_CH4_1            (PIN_ALT3   | PIN_PORTA | PIN7)
#define PIN_FTM0_CH4_2            (PIN_ALT4   | PIN_PORTB | PIN12)
#define PIN_FTM0_CH4_3            (PIN_ALT4   | PIN_PORTD | PIN4)
#define PIN_FTM0_CH5_1            (PIN_ALT3   | PIN_PORTA | PIN0)
#define PIN_FTM0_CH5_2            (PIN_ALT4   | PIN_PORTB | PIN13)
#define PIN_FTM0_CH5_3            (PIN_ALT4   | PIN_PORTD | PIN5)
#define PIN_FTM0_CH6_1            (PIN_ALT3   | PIN_PORTA | PIN1)
#define PIN_FTM0_CH6_2            (PIN_ALT4   | PIN_PORTD | PIN6)
#define PIN_FTM0_CH7_1            (PIN_ALT3   | PIN_PORTA | PIN2)
#define PIN_FTM0_CH7_2            (PIN_ALT4   | PIN_PORTD | PIN7)
#define PIN_FTM0_FLT0_1           (PIN_ALT6   | PIN_PORTB | PIN3)
#define PIN_FTM0_FLT0_2           (PIN_ALT6   | PIN_PORTD | PIN6)
#define PIN_FTM0_FLT1_1           (PIN_ALT6   | PIN_PORTB | PIN10)
#define PIN_FTM0_FLT1_2           (PIN_ALT6   | PIN_PORTD | PIN7)
#define PIN_FTM0_FLT2_1           (PIN_ALT3   | PIN_PORTA | PIN18)
#define PIN_FTM0_FLT2_2           (PIN_ALT6   | PIN_PORTB | PIN11)
#define PIN_FTM0_FLT3             (PIN_ALT6   | PIN_PORTB | PIN2)

#define PIN_FTM1_CH0_1            (PIN_ALT3   | PIN_PORTA | PIN12)
#define PIN_FTM1_CH0_2            (PIN_ALT3   | PIN_PORTA | PIN8)
#define PIN_FTM1_CH0_3            (PIN_ALT3   | PIN_PORTB | PIN0)
#define PIN_FTM1_CH0_4            (PIN_ALT3   | PIN_PORTB | PIN12)
#define PIN_FTM1_CH1_1            (PIN_ALT3   | PIN_PORTA | PIN13)
#define PIN_FTM1_CH1_2            (PIN_ALT3   | PIN_PORTA | PIN9)
#define PIN_FTM1_CH1_3            (PIN_ALT3   | PIN_PORTB | PIN1)
#define PIN_FTM1_CH1_4            (PIN_ALT3   | PIN_PORTB | PIN13)
#define PIN_FTM1_FLT0_1           (PIN_ALT3   | PIN_PORTA | PIN19)
#define PIN_FTM1_FLT0_2           (PIN_ALT6   | PIN_PORTB | PIN4)
#define PIN_FTM1_QD_PHA_1         (PIN_ALT6   | PIN_PORTA | PIN8)
#define PIN_FTM1_QD_PHA_2         (PIN_ALT6   | PIN_PORTB | PIN0)
#define PIN_FTM1_QD_PHA_3         (PIN_ALT6   | PIN_PORTB | PIN12)
#define PIN_FTM1_QD_PHA_4         (PIN_ALT7   | PIN_PORTA | PIN12)
#define PIN_FTM1_QD_PHB_1         (PIN_ALT6   | PIN_PORTA | PIN9)
#define PIN_FTM1_QD_PHB_2         (PIN_ALT6   | PIN_PORTB | PIN1)
#define PIN_FTM1_QD_PHB_3         (PIN_ALT6   | PIN_PORTB | PIN13)
#define PIN_FTM1_QD_PHB_4         (PIN_ALT7   | PIN_PORTA | PIN13)

#define PIN_FTM2_CH0_1            (PIN_ALT3   | PIN_PORTA | PIN10)
#define PIN_FTM2_CH0_2            (PIN_ALT3   | PIN_PORTB | PIN18)
#define PIN_FTM2_CH1_1            (PIN_ALT3   | PIN_PORTA | PIN11)
#define PIN_FTM2_CH1_2            (PIN_ALT3   | PIN_PORTB | PIN19)
#define PIN_FTM2_FLT0_1           (PIN_ALT6   | PIN_PORTB | PIN5)
#define PIN_FTM2_FLT0_2           (PIN_ALT6   | PIN_PORTC | PIN9)
#define PIN_FTM2_QD_PHA_1         (PIN_ALT6   | PIN_PORTA | PIN10)
#define PIN_FTM2_QD_PHA_2         (PIN_ALT6   | PIN_PORTB | PIN18)
#define PIN_FTM2_QD_PHB_1         (PIN_ALT6   | PIN_PORTA | PIN11)
#define PIN_FTM2_QD_PHB_2         (PIN_ALT6   | PIN_PORTB | PIN19)

#define PIN_FTM3_CH0_1            (PIN_ALT4   | PIN_PORTD | PIN0)
#define PIN_FTM3_CH0_2            (PIN_ALT6   | PIN_PORTE | PIN5)
#define PIN_FTM3_CH1_1            (PIN_ALT4   | PIN_PORTD | PIN1)
#define PIN_FTM3_CH1_2            (PIN_ALT6   | PIN_PORTE | PIN6)
#define PIN_FTM3_CH2_1            (PIN_ALT4   | PIN_PORTD | PIN2)
#define PIN_FTM3_CH2_2            (PIN_ALT6   | PIN_PORTE | PIN7)
#define PIN_FTM3_CH3_1            (PIN_ALT4   | PIN_PORTD | PIN3)
#define PIN_FTM3_CH3_2            (PIN_ALT6   | PIN_PORTE | PIN8)
#define PIN_FTM3_CH4_1            (PIN_ALT3   | PIN_PORTC | PIN8)
#define PIN_FTM3_CH4_2            (PIN_ALT6   | PIN_PORTE | PIN9)
#define PIN_FTM3_CH5_1            (PIN_ALT3   | PIN_PORTC | PIN9)
#define PIN_FTM3_CH5_2            (PIN_ALT6   | PIN_PORTE | PIN10)
#define PIN_FTM3_CH6_1            (PIN_ALT3   | PIN_PORTC | PIN10)
#define PIN_FTM3_CH6_2            (PIN_ALT6   | PIN_PORTE | PIN11)
#define PIN_FTM3_CH7_1            (PIN_ALT3   | PIN_PORTC | PIN11)
#define PIN_FTM3_CH7_2            (PIN_ALT6   | PIN_PORTE | PIN12)
#define PIN_FTM3_FLT0_1           (PIN_ALT3   | PIN_PORTD | PIN12)
#define PIN_FTM3_FLT0_2           (PIN_ALT6   | PIN_PORTC | PIN12)

/* I2C */

#define PIN_I2C0_SCL_1            (PIN_ALT2_OPENDRAIN | PIN_PORTB | PIN0)
#define PIN_I2C0_SCL_2            (PIN_ALT2_OPENDRAIN | PIN_PORTB | PIN2)
#define PIN_I2C0_SCL_3            (PIN_ALT2_OPENDRAIN | PIN_PORTD | PIN8)
#define PIN_I2C0_SCL_4            (PIN_ALT5_OPENDRAIN | PIN_PORTE | PIN24)
#define PIN_I2C0_SCL_5            (PIN_ALT7_OPENDRAIN | PIN_PORTD | PIN2)
#define PIN_I2C0_SDA_1            (PIN_ALT2_OPENDRAIN | PIN_PORTB | PIN1)
#define PIN_I2C0_SDA_2            (PIN_ALT2_OPENDRAIN | PIN_PORTB | PIN3)
#define PIN_I2C0_SDA_3            (PIN_ALT2_OPENDRAIN | PIN_PORTD | PIN9)
#define PIN_I2C0_SDA_4            (PIN_ALT5_OPENDRAIN | PIN_PORTE | PIN25)
#define PIN_I2C0_SDA_5            (PIN_ALT7_OPENDRAIN | PIN_PORTD | PIN3)

#define PIN_I2C1_SCL_1            (PIN_ALT2_OPENDRAIN | PIN_PORTC | PIN10)
#define PIN_I2C1_SCL_2            (PIN_ALT6_OPENDRAIN | PIN_PORTE | PIN1)
#define PIN_I2C1_SDA_1            (PIN_ALT2_OPENDRAIN | PIN_PORTC | PIN11)
#define PIN_I2C1_SDA_2            (PIN_ALT6_OPENDRAIN | PIN_PORTE | PIN0)

#define PIN_I2C2_SCL_1            (PIN_ALT5_OPENDRAIN | PIN_PORTA | PIN12)
#define PIN_I2C2_SCL_2            (PIN_ALT5_OPENDRAIN | PIN_PORTA | PIN14)
#define PIN_I2C2_SDA_1            (PIN_ALT5_OPENDRAIN | PIN_PORTA | PIN11)
#define PIN_I2C2_SDA_2            (PIN_ALT5_OPENDRAIN | PIN_PORTA | PIN13)

#define PIN_I2C3_SCL_1            (PIN_ALT2_OPENDRAIN | PIN_PORTE | PIN11)
#define PIN_I2C3_SCL_2            (PIN_ALT4_OPENDRAIN | PIN_PORTA | PIN2)
#define PIN_I2C3_SDA_1            (PIN_ALT2_OPENDRAIN | PIN_PORTE | PIN10)
#define PIN_I2C3_SDA_2            (PIN_ALT4_OPENDRAIN | PIN_PORTA | PIN1)

/* I2S */

#define PIN_I2S0_MCLK_1           (PIN_ALT4   | PIN_PORTC | PIN8)
#define PIN_I2S0_MCLK_2           (PIN_ALT4   | PIN_PORTE | PIN6)
#define PIN_I2S0_MCLK_3           (PIN_ALT6   | PIN_PORTA | PIN17)
#define PIN_I2S0_MCLK_4           (PIN_ALT6   | PIN_PORTC | PIN6)
#define PIN_I2S0_RX_BCLK_1        (PIN_ALT4   | PIN_PORTC | PIN6)
#define PIN_I2S0_RX_BCLK_2        (PIN_ALT4   | PIN_PORTC | PIN9)
#define PIN_I2S0_RX_BCLK_3        (PIN_ALT4   | PIN_PORTE | PIN9)
#define PIN_I2S0_RX_BCLK_4        (PIN_ALT6   | PIN_PORTA | PIN14)
#define PIN_I2S0_RX_FS_1          (PIN_ALT4   | PIN_PORTC | PIN10)
#define PIN_I2S0_RX_FS_2          (PIN_ALT4   | PIN_PORTC | PIN7)
#define PIN_I2S0_RX_FS_3          (PIN_ALT4   | PIN_PORTE | PIN8)
#define PIN_I2S0_RX_FS_4          (PIN_ALT6   | PIN_PORTA | PIN16)
#define PIN_I2S0_RXD0_1           (PIN_ALT4   | PIN_PORTC | PIN5)
#define PIN_I2S0_RXD0_2           (PIN_ALT4   | PIN_PORTE | PIN7)
#define PIN_I2S0_RXD0_3           (PIN_ALT6   | PIN_PORTA | PIN15)
#define PIN_I2S0_RXD1_1           (PIN_ALT2   | PIN_PORTE | PIN8)
#define PIN_I2S0_RXD1_2           (PIN_ALT4   | PIN_PORTC | PIN11)
#define PIN_I2S0_RXD1_3           (PIN_ALT7   | PIN_PORTA | PIN16)
#define PIN_I2S0_TX_BCLK_1        (PIN_ALT4   | PIN_PORTB | PIN18)
#define PIN_I2S0_TX_BCLK_2        (PIN_ALT4   | PIN_PORTE | PIN12)
#define PIN_I2S0_TX_BCLK_3        (PIN_ALT6   | PIN_PORTA | PIN5)
#define PIN_I2S0_TX_BCLK_4        (PIN_ALT6   | PIN_PORTC | PIN3)
#define PIN_I2S0_TX_FS_1          (PIN_ALT4   | PIN_PORTB | PIN19)
#define PIN_I2S0_TX_FS_2          (PIN_ALT4   | PIN_PORTE | PIN11)
#define PIN_I2S0_TX_FS_3          (PIN_ALT6   | PIN_PORTA | PIN13)
#define PIN_I2S0_TX_FS_4          (PIN_ALT6   | PIN_PORTC | PIN2)
#define PIN_I2S0_TXD0_1           (PIN_ALT4   | PIN_PORTE | PIN10)
#define PIN_I2S0_TXD0_2           (PIN_ALT6   | PIN_PORTA | PIN12)
#define PIN_I2S0_TXD0_3           (PIN_ALT6   | PIN_PORTC | PIN1)
#define PIN_I2S0_TXD1_1           (PIN_ALT2   | PIN_PORTE | PIN9)
#define PIN_I2S0_TXD1_2           (PIN_ALT6   | PIN_PORTC | PIN0)
#define PIN_I2S0_TXD1_3           (PIN_ALT7   | PIN_PORTA | PIN14)

/* JTAG */

#define PIN_JTAG_TCLK             (PIN_ALT7   | PIN_PORTA | PIN0)
#define PIN_JTAG_TDI              (PIN_ALT7   | PIN_PORTA | PIN1)
#define PIN_JTAG_TDO              (PIN_ALT7   | PIN_PORTA | PIN2)
#define PIN_JTAG_TMS              (PIN_ALT7   | PIN_PORTA | PIN3)
#define PIN_JTAG_TRST             (PIN_ALT7   | PIN_PORTA | PIN5)

/* Low-leakage wakeup module (LLWU, actually GPIO configurations) */

#define PIN_LLWU_P0               (PIN_ALT1   | PIN_PORTE | PIN1)
#define PIN_LLWU_P1               (PIN_ALT1   | PIN_PORTE | PIN2)
#define PIN_LLWU_P2               (PIN_ALT1   | PIN_PORTE | PIN4)
#define PIN_LLWU_P3               (PIN_ALT1   | PIN_PORTA | PIN4)
#define PIN_LLWU_P4               (PIN_ALT1   | PIN_PORTA | PIN13)
#define PIN_LLWU_P5               (PIN_ALT1   | PIN_PORTB | PIN0)
#define PIN_LLWU_P6               (PIN_ALT1   | PIN_PORTC | PIN1)
#define PIN_LLWU_P7               (PIN_ALT1   | PIN_PORTC | PIN3)
#define PIN_LLWU_P8               (PIN_ALT1   | PIN_PORTC | PIN4)
#define PIN_LLWU_P9               (PIN_ALT1   | PIN_PORTC | PIN5)
#define PIN_LLWU_P10              (PIN_ALT1   | PIN_PORTC | PIN6)
#define PIN_LLWU_P11              (PIN_ALT1   | PIN_PORTC | PIN11)
#define PIN_LLWU_P12              (PIN_ALT1   | PIN_PORTD | PIN0)
#define PIN_LLWU_P13              (PIN_ALT1   | PIN_PORTD | PIN2)
#define PIN_LLWU_P14              (PIN_ALT1   | PIN_PORTD | PIN4)
#define PIN_LLWU_P15              (PIN_ALT1   | PIN_PORTD | PIN6)
#define PIN_LLWU_P16              (PIN_ALT1   | PIN_PORTE | PIN6)
#define PIN_LLWU_P17              (PIN_ALT1   | PIN_PORTE | PIN9)
#define PIN_LLWU_P18              (PIN_ALT1   | PIN_PORTE | PIN10)
#define PIN_LLWU_P21              (PIN_ALT1   | PIN_PORTE | PIN25)
#define PIN_LLWU_P22              (PIN_ALT1   | PIN_PORTA | PIN10)
#define PIN_LLWU_P23              (PIN_ALT1   | PIN_PORTA | PIN11)
#define PIN_LLWU_P24              (PIN_ALT1   | PIN_PORTD | PIN8)
#define PIN_LLWU_P25              (PIN_ALT1   | PIN_PORTD | PIN11)

/* Low-Power Timer (LPTMR) */

#define PIN_LPTMR0_ALT1           (PIN_ALT6   | PIN_PORTA | PIN19)
#define PIN_LPTMR0_ALT2           (PIN_ALT3   | PIN_PORTC | PIN5)

/* MII */

#define PIN_MII0_COL              (PIN_ALT4   | PIN_PORTA | PIN29)
#define PIN_MII0_CRS              (PIN_ALT4   | PIN_PORTA | PIN27)
#define PIN_MII0_MDC_1            (PIN_ALT4   | PIN_PORTB | PIN1)
#define PIN_MII0_MDC_2            (PIN_ALT5   | PIN_PORTA | PIN8)
#ifdef CONFIG_KINETIS_ENET_MDIOPULLUP
#  define PIN_MII0_MDIO_1         (PIN_ALT4_PULLUP | PIN_PORTB | PIN0)
#  define PIN_MII0_MDIO_2         (PIN_ALT5_PULLUP | PIN_PORTA | PIN7)
#else
#  define PIN_MII0_MDIO_1         (PIN_ALT4   | PIN_PORTB | PIN0)
#  define PIN_MII0_MDIO_2         (PIN_ALT5   | PIN_PORTA | PIN7)
#endif
#define PIN_MII0_RXCLK            (PIN_ALT4   | PIN_PORTA | PIN11)
#define PIN_MII0_RXD0             (PIN_ALT4   | PIN_PORTA | PIN13)
#define PIN_MII0_RXD1             (PIN_ALT4   | PIN_PORTA | PIN12)
#define PIN_MII0_RXD2             (PIN_ALT4   | PIN_PORTA | PIN10)
#define PIN_MII0_RXD3             (PIN_ALT4   | PIN_PORTA | PIN9)
#define PIN_MII0_RXDV             (PIN_ALT4   | PIN_PORTA | PIN14)
#ifdef CONFIG_KINETIS_ENET_NORXER
#  define PIN_MII0_RXER           (GPIO_PULLDOWN | PIN_PORTA | PIN5)
#else
#  define PIN_MII0_RXER           (PIN_ALT4   | PIN_PORTA | PIN5)
#endif
#define PIN_MII0_TXCLK            (PIN_ALT4   | PIN_PORTA | PIN25)
#define PIN_MII0_TXD0             (PIN_ALT4   | PIN_PORTA | PIN16)
#define PIN_MII0_TXD1             (PIN_ALT4   | PIN_PORTA | PIN17)
#define PIN_MII0_TXD2             (PIN_ALT4   | PIN_PORTA | PIN24)
#define PIN_MII0_TXD3             (PIN_ALT4   | PIN_PORTA | PIN26)
#define PIN_MII0_TXEN             (PIN_ALT4   | PIN_PORTA | PIN15)
#define PIN_MII0_TXER             (PIN_ALT4   | PIN_PORTA | PIN28)

/* NMI */

#define PIN_NMI                   (PIN_ALT7   | PIN_PORTA | PIN4)

/* Programmable Delay Block (PDB) */

#define PIN_PDB0_EXTRG_1          (PIN_ALT3   | PIN_PORTC | PIN0)
#define PIN_PDB0_EXTRG_2          (PIN_ALT3   | PIN_PORTC | PIN6)

/* RMII */

#define PIN_RMII0_CRS_DV          (PIN_ALT4   | PIN_PORTA | PIN14)
#define PIN_RMII0_MDC_1           (PIN_ALT4   | PIN_PORTB | PIN1)
#define PIN_RMII0_MDC_2           (PIN_ALT5   | PIN_PORTA | PIN8)
#ifdef CONFIG_KINETIS_ENET_MDIOPULLUP
#  define PIN_RMII0_MDIO_1        (PIN_ALT4_PULLUP | PIN_PORTB | PIN0)
#  define PIN_RMII0_MDIO_2        (PIN_ALT5_PULLUP | PIN_PORTA | PIN7)
#else
#  define PIN_RMII0_MDIO_1        (PIN_ALT4   | PIN_PORTB | PIN0)
#  define PIN_RMII0_MDIO_2        (PIN_ALT5   | PIN_PORTA | PIN7)
#endif
#define PIN_RMII0_RXD0            (PIN_ALT4   | PIN_PORTA | PIN13)
#define PIN_RMII0_RXD1            (PIN_ALT4   | PIN_PORTA | PIN12)
#ifdef CONFIG_KINETIS_ENET_NORXER
#  define PIN_RMII0_RXER          (GPIO_PULLDOWN | PIN_PORTA | PIN5)
#else
#  define PIN_RMII0_RXER          (PIN_ALT4   | PIN_PORTA | PIN5)
#endif
#define PIN_RMII0_TXD0            (PIN_ALT4   | PIN_PORTA | PIN16)
#define PIN_RMII0_TXD1            (PIN_ALT4   | PIN_PORTA | PIN17)
#define PIN_RMII0_TXEN            (PIN_ALT4   | PIN_PORTA | PIN15)

/* Real-Time Clock (RTC) */

#define PIN_RTC_CLKOUT_1          (PIN_ALT6   | PIN_PORTE | PIN26)
#define PIN_RTC_CLKOUT_2          (PIN_ALT7   | PIN_PORTE | PIN0)

/* Synchronous DRAM Controller Module (SDRAM) */

#define PIN_SDRAM_CAS_B           (PIN_ALT5   | PIN_PORTB | PIN0)
#define PIN_SDRAM_RAS_B           (PIN_ALT5   | PIN_PORTB | PIN1)
#define PIN_SDRAM_WE              (PIN_ALT5   | PIN_PORTB | PIN2)
#define PIN_SDRAM_CKE             (PIN_ALT5   | PIN_PORTD | PIN7)
#define PIN_SDRAM_CS0_B           (PIN_ALT5   | PIN_PORTB | PIN3)
#define PIN_SDRAM_CS1_B           (PIN_ALT5   | PIN_PORTB | PIN4)
#define PIN_SDRAM_D16             (PIN_ALT5   | PIN_PORTB | PIN17)
#define PIN_SDRAM_D17             (PIN_ALT5   | PIN_PORTB | PIN16)
#define PIN_SDRAM_D18             (PIN_ALT5   | PIN_PORTB | PIN11)
#define PIN_SDRAM_D19             (PIN_ALT5   | PIN_PORTB | PIN10)
#define PIN_SDRAM_D20             (PIN_ALT5   | PIN_PORTB | PIN9)
#define PIN_SDRAM_D21             (PIN_ALT5   | PIN_PORTB | PIN8)
#define PIN_SDRAM_D22             (PIN_ALT5   | PIN_PORTB | PIN7)
#define PIN_SDRAM_D23             (PIN_ALT5   | PIN_PORTB | PIN6)
#define PIN_SDRAM_D24             (PIN_ALT5   | PIN_PORTC | PIN15)
#define PIN_SDRAM_D25             (PIN_ALT5   | PIN_PORTC | PIN14)
#define PIN_SDRAM_D26             (PIN_ALT5   | PIN_PORTC | PIN13)
#define PIN_SDRAM_D27             (PIN_ALT5   | PIN_PORTC | PIN12)
#define PIN_SDRAM_D28             (PIN_ALT5   | PIN_PORTB | PIN23)
#define PIN_SDRAM_D29             (PIN_ALT5   | PIN_PORTB | PIN22)
#define PIN_SDRAM_D30             (PIN_ALT5   | PIN_PORTB | PIN21)
#define PIN_SDRAM_D31             (PIN_ALT5   | PIN_PORTB | PIN20)

#define PIN_SDRAM_DQM0            (PIN_ALT5   | PIN_PORTC | PIN19)
#define PIN_SDRAM_DQM1            (PIN_ALT5   | PIN_PORTC | PIN18)
#define PIN_SDRAM_DQM2            (PIN_ALT5   | PIN_PORTC | PIN16)
#define PIN_SDRAM_DQM3            (PIN_ALT5   | PIN_PORTC | PIN17)

#define PIN_SDRAM_A9              (PIN_ALT5   | PIN_PORTD | PIN5)
#define PIN_SDRAM_A10             (PIN_ALT5   | PIN_PORTD | PIN4)
#define PIN_SDRAM_A11             (PIN_ALT5   | PIN_PORTD | PIN3)
#define PIN_SDRAM_A12             (PIN_ALT5   | PIN_PORTD | PIN2)
#define PIN_SDRAM_A13             (PIN_ALT5   | PIN_PORTC | PIN10)
#define PIN_SDRAM_A14             (PIN_ALT5   | PIN_PORTC | PIN9)
#define PIN_SDRAM_A15             (PIN_ALT5   | PIN_PORTC | PIN8)
#define PIN_SDRAM_A16             (PIN_ALT5   | PIN_PORTC | PIN7)
#define PIN_SDRAM_A17             (PIN_ALT5   | PIN_PORTC | PIN6)
#define PIN_SDRAM_A18             (PIN_ALT5   | PIN_PORTC | PIN5)
#define PIN_SDRAM_A19             (PIN_ALT5   | PIN_PORTC | PIN4)
#define PIN_SDRAM_A20             (PIN_ALT5   | PIN_PORTC | PIN2)
#define PIN_SDRAM_A21             (PIN_ALT5   | PIN_PORTC | PIN1)
#define PIN_SDRAM_A22             (PIN_ALT5   | PIN_PORTC | PIN0)
#define PIN_SDRAM_A23             (PIN_ALT5   | PIN_PORTB | PIN18)

/* Secured digital host controller (SDHC) */

#define PIN_SDHC0_CLKIN           (PIN_ALT4   | PIN_PORTD | PIN11)
#define PIN_SDHC0_CMD             (PIN_ALT4   | PIN_PORTE | PIN3)
#define PIN_SDHC0_D0              (PIN_ALT4   | PIN_PORTE | PIN1)
#define PIN_SDHC0_D1              (PIN_ALT4   | PIN_PORTE | PIN0)
#define PIN_SDHC0_D2              (PIN_ALT4   | PIN_PORTE | PIN5)
#define PIN_SDHC0_D3              (PIN_ALT4   | PIN_PORTE | PIN4)
#define PIN_SDHC0_D4              (PIN_ALT4   | PIN_PORTD | PIN12)
#define PIN_SDHC0_D5              (PIN_ALT4   | PIN_PORTD | PIN13)
#define PIN_SDHC0_D6              (PIN_ALT4   | PIN_PORTD | PIN14)
#define PIN_SDHC0_D7              (PIN_ALT4   | PIN_PORTD | PIN15)
#define PIN_SDHC0_DCLK            (PIN_ALT4   | PIN_PORTE | PIN2)

/* SPI */

#define PIN_SPI0_PCS0_1           (PIN_ALT2   | PIN_PORTA | PIN14)
#define PIN_SPI0_PCS0_2           (PIN_ALT2   | PIN_PORTC | PIN4)
#define PIN_SPI0_PCS0_3           (PIN_ALT2   | PIN_PORTD | PIN0)
#define PIN_SPI0_PCS1_1           (PIN_ALT2   | PIN_PORTC | PIN3)
#define PIN_SPI0_PCS1_2           (PIN_ALT2   | PIN_PORTD | PIN4)
#define PIN_SPI0_PCS2_1           (PIN_ALT2   | PIN_PORTC | PIN2)
#define PIN_SPI0_PCS2_3           (PIN_ALT2   | PIN_PORTD | PIN5)
#define PIN_SPI0_PCS3_1           (PIN_ALT2   | PIN_PORTC | PIN1)
#define PIN_SPI0_PCS3_2           (PIN_ALT2   | PIN_PORTD | PIN6)
#define PIN_SPI0_PCS4             (PIN_ALT2   | PIN_PORTC | PIN0)
#define PIN_SPI0_PCS5             (PIN_ALT3   | PIN_PORTB | PIN23)
#define PIN_SPI0_SCK_1            (PIN_ALT2   | PIN_PORTA | PIN15)
#define PIN_SPI0_SCK_2            (PIN_ALT2   | PIN_PORTC | PIN5)
#define PIN_SPI0_SCK_3            (PIN_ALT2   | PIN_PORTD | PIN1)
#define PIN_SPI0_SIN_1            (PIN_ALT2   | PIN_PORTA | PIN17)
#define PIN_SPI0_SIN_2            (PIN_ALT2   | PIN_PORTC | PIN7)
#define PIN_SPI0_SIN_3            (PIN_ALT2   | PIN_PORTD | PIN3)
#define PIN_SPI0_SOUT_1           (PIN_ALT2   | PIN_PORTA | PIN16)
#define PIN_SPI0_SOUT_2           (PIN_ALT2   | PIN_PORTC | PIN6)
#define PIN_SPI0_SOUT_3           (PIN_ALT2   | PIN_PORTD | PIN2)

#define PIN_SPI1_PCS0_1           (PIN_ALT2   | PIN_PORTB | PIN10)
#define PIN_SPI1_PCS0_2           (PIN_ALT2   | PIN_PORTE | PIN4)
#define PIN_SPI1_PCS0_3           (PIN_ALT7   | PIN_PORTD | PIN4)
#define PIN_SPI1_PCS1_1           (PIN_ALT2   | PIN_PORTB | PIN9)
#define PIN_SPI1_PCS1_2           (PIN_ALT2   | PIN_PORTE | PIN0)
#define PIN_SPI1_PCS2             (PIN_ALT2   | PIN_PORTE | PIN5)
#define PIN_SPI1_PCS3             (PIN_ALT2   | PIN_PORTE | PIN6)
#define PIN_SPI1_SCK_1            (PIN_ALT2   | PIN_PORTB | PIN11)
#define PIN_SPI1_SCK_2            (PIN_ALT2   | PIN_PORTE | PIN2)
#define PIN_SPI1_SCK_3            (PIN_ALT7   | PIN_PORTD | PIN5)
#define PIN_SPI1_SIN_1            (PIN_ALT2   | PIN_PORTB | PIN17)
#define PIN_SPI1_SIN_2            (PIN_ALT2   | PIN_PORTE | PIN3)
#define PIN_SPI1_SIN_3            (PIN_ALT7   | PIN_PORTD | PIN7)
#define PIN_SPI1_SIN_4            (PIN_ALT7   | PIN_PORTE | PIN1)
#define PIN_SPI1_SOUT_1           (PIN_ALT2   | PIN_PORTB | PIN16)
#define PIN_SPI1_SOUT_2           (PIN_ALT2   | PIN_PORTE | PIN1)
#define PIN_SPI1_SOUT_3           (PIN_ALT7   | PIN_PORTD | PIN6)
#define PIN_SPI1_SOUT_4           (PIN_ALT7   | PIN_PORTE | PIN3)

#define PIN_SPI2_PCS0_1           (PIN_ALT2   | PIN_PORTB | PIN20)
#define PIN_SPI2_PCS0_2           (PIN_ALT2   | PIN_PORTD | PIN11)
#define PIN_SPI2_PCS1             (PIN_ALT2   | PIN_PORTD | PIN15)
#define PIN_SPI2_SCK_1            (PIN_ALT2   | PIN_PORTB | PIN21)
#define PIN_SPI2_SCK_2            (PIN_ALT2   | PIN_PORTD | PIN12)
#define PIN_SPI2_SIN_1            (PIN_ALT2   | PIN_PORTB | PIN23)
#define PIN_SPI2_SIN_2            (PIN_ALT2   | PIN_PORTD | PIN14)
#define PIN_SPI2_SOUT_1           (PIN_ALT2   | PIN_PORTB | PIN22)
#define PIN_SPI2_SOUT_2           (PIN_ALT2   | PIN_PORTD | PIN13)

/* SWD */

#define PIN_SWD_CLK               (PIN_ALT7   | PIN_PORTA | PIN0)
#define PIN_SWD_DIO               (PIN_ALT7   | PIN_PORTA | PIN3)

/* Timer/PWM Module (TPM) */

#define PIN_TPM1_CH0_1            (PIN_ALT6   | PIN_PORTA | PIN8)
#define PIN_TPM1_CH0_2            (PIN_ALT7   | PIN_PORTA | PIN12)
#define PIN_TPM1_CH0_3            (PIN_ALT6   | PIN_PORTB | PIN0)
#define PIN_TPM1_CH1_1            (PIN_ALT6   | PIN_PORTA | PIN9)
#define PIN_TPM1_CH1_2            (PIN_ALT7   | PIN_PORTA | PIN13)
#define PIN_TPM1_CH1_3            (PIN_ALT6   | PIN_PORTB | PIN1)
#define PIN_TPM2_CH0_1            (PIN_ALT6   | PIN_PORTA | PIN10)
#define PIN_TPM2_CH0_2            (PIN_ALT6   | PIN_PORTB | PIN18)
#define PIN_TPM2_CH1_1            (PIN_ALT6   | PIN_PORTA | PIN11)
#define PIN_TPM2_CH1_2            (PIN_ALT6   | PIN_PORTB | PIN19)

#define PIN_TPM_CLKIN0_1          (PIN_ALT7   | PIN_PORTA | PIN18)
#define PIN_TPM_CLKIN0_2          (PIN_ALT7   | PIN_PORTB | PIN16)
#define PIN_TPM_CLKIN0_3          (PIN_ALT7   | PIN_PORTC | PIN12)
#define PIN_TPM_CLKIN1_1          (PIN_ALT7   | PIN_PORTA | PIN17)
#define PIN_TPM_CLKIN1_2          (PIN_ALT7   | PIN_PORTB | PIN17)
#define PIN_TPM_CLKIN1_3          (PIN_ALT7   | PIN_PORTC | PIN13)

/* Touch Sensing Input (TSI) */

#define PIN_TSI0_CH0_1            (PIN_ANALOG   | PIN_PORTB | PIN0)
#define PIN_TSI0_CH1_1            (PIN_ANALOG   | PIN_PORTA | PIN0)
#define PIN_TSI0_CH2_1            (PIN_ANALOG   | PIN_PORTA | PIN1)
#define PIN_TSI0_CH3_1            (PIN_ANALOG   | PIN_PORTA | PIN2)
#define PIN_TSI0_CH4_1            (PIN_ANALOG   | PIN_PORTA | PIN3)
#define PIN_TSI0_CH5_1            (PIN_ANALOG   | PIN_PORTA | PIN4)
#define PIN_TSI0_CH6_1            (PIN_ANALOG   | PIN_PORTB | PIN1)
#define PIN_TSI0_CH7_1            (PIN_ANALOG   | PIN_PORTB | PIN2)
#define PIN_TSI0_CH8_1            (PIN_ANALOG   | PIN_PORTB | PIN3)
#define PIN_TSI0_CH9_1            (PIN_ANALOG   | PIN_PORTB | PIN16)
#define PIN_TSI0_CH10_1           (PIN_ANALOG   | PIN_PORTB | PIN17)
#define PIN_TSI0_CH11_1           (PIN_ANALOG   | PIN_PORTB | PIN18)
#define PIN_TSI0_CH12_1           (PIN_ANALOG   | PIN_PORTB | PIN19)
#define PIN_TSI0_CH13_1           (PIN_ANALOG   | PIN_PORTC | PIN0)
#define PIN_TSI0_CH14_1           (PIN_ANALOG   | PIN_PORTC | PIN1)
#define PIN_TSI0_CH15_1           (PIN_ANALOG   | PIN_PORTC | PIN2)

/* Trace */

#define PIN_TRACE_CLKOUT_1        (PIN_ALT5   | PIN_PORTE | PIN0)
#define PIN_TRACE_CLKOUT_2        (PIN_ALT7   | PIN_PORTA | PIN6)
#define PIN_TRACE_D0_1            (PIN_ALT5   | PIN_PORTE | PIN4)
#define PIN_TRACE_D0_2            (PIN_ALT7   | PIN_PORTA | PIN10)
#define PIN_TRACE_D1_1            (PIN_ALT5   | PIN_PORTE | PIN3)
#define PIN_TRACE_D1_2            (PIN_ALT7   | PIN_PORTA | PIN9)
#define PIN_TRACE_D2_1            (PIN_ALT5   | PIN_PORTE | PIN2)
#define PIN_TRACE_D2_2            (PIN_ALT7   | PIN_PORTA | PIN8)
#define PIN_TRACE_D3_1            (PIN_ALT5   | PIN_PORTE | PIN1)
#define PIN_TRACE_D3_2            (PIN_ALT7   | PIN_PORTA | PIN7)
#define PIN_TRACE_SWO             (PIN_ALT7   | PIN_PORTA | PIN2)

/* UARTs */

#define PIN_UART0_COL_1           (PIN_ALT2   | PIN_PORTA | PIN0)
#define PIN_UART0_COL_2           (PIN_ALT3   | PIN_PORTA | PIN16)
#define PIN_UART0_COL_3           (PIN_ALT3   | PIN_PORTB | PIN3)
#define PIN_UART0_COL_4           (PIN_ALT3   | PIN_PORTD | PIN5)
#define PIN_UART0_CTS_1           (PIN_ALT2   | PIN_PORTA | PIN0)
#define PIN_UART0_CTS_2           (PIN_ALT3   | PIN_PORTA | PIN16)
#define PIN_UART0_CTS_3           (PIN_ALT3   | PIN_PORTB | PIN3)
#define PIN_UART0_CTS_4           (PIN_ALT3   | PIN_PORTD | PIN5)
#define PIN_UART0_RTS_1           (PIN_ALT2   | PIN_PORTA | PIN3)
#define PIN_UART0_RTS_2           (PIN_ALT3   | PIN_PORTA | PIN17)
#define PIN_UART0_RTS_3           (PIN_ALT3   | PIN_PORTB | PIN2)
#define PIN_UART0_RTS_4           (PIN_ALT3   | PIN_PORTD | PIN4)
#define PIN_UART0_RX_1            (PIN_ALT2   | PIN_PORTA | PIN1)
#define PIN_UART0_RX_2            (PIN_ALT3   | PIN_PORTA | PIN15)
#define PIN_UART0_RX_3            (PIN_ALT3   | PIN_PORTB | PIN16)
#define PIN_UART0_RX_4            (PIN_ALT3   | PIN_PORTD | PIN6)
#define PIN_UART0_TX_1            (PIN_ALT2   | PIN_PORTA | PIN2)
#define PIN_UART0_TX_2            (PIN_ALT3   | PIN_PORTA | PIN14)
#define PIN_UART0_TX_3            (PIN_ALT3   | PIN_PORTB | PIN17)
#define PIN_UART0_TX_4            (PIN_ALT3   | PIN_PORTD | PIN7)

#define PIN_UART1_CTS_1           (PIN_ALT3   | PIN_PORTC | PIN2)
#define PIN_UART1_CTS_2           (PIN_ALT3   | PIN_PORTE | PIN2)
#define PIN_UART1_RTS_1           (PIN_ALT3   | PIN_PORTC | PIN1)
#define PIN_UART1_RTS_2           (PIN_ALT3   | PIN_PORTE | PIN3)
#define PIN_UART1_RX_1            (PIN_ALT3   | PIN_PORTC | PIN3)
#define PIN_UART1_RX_2            (PIN_ALT3   | PIN_PORTE | PIN1)
#define PIN_UART1_TX_1            (PIN_ALT3   | PIN_PORTC | PIN4)
#define PIN_UART1_TX_2            (PIN_ALT3   | PIN_PORTE | PIN0)

#define PIN_UART2_CTS             (PIN_ALT3   | PIN_PORTD | PIN1)
#define PIN_UART2_RTS             (PIN_ALT3   | PIN_PORTD | PIN0)
#define PIN_UART2_RX              (PIN_ALT3   | PIN_PORTD | PIN2)
#define PIN_UART2_TX              (PIN_ALT3   | PIN_PORTD | PIN3)

#define PIN_UART3_CTS_1           (PIN_ALT2   | PIN_PORTB | PIN13)
#define PIN_UART3_CTS_2           (PIN_ALT3   | PIN_PORTB | PIN9)
#define PIN_UART3_CTS_3           (PIN_ALT3   | PIN_PORTC | PIN19)
#define PIN_UART3_CTS_4           (PIN_ALT3   | PIN_PORTE | PIN6)
#define PIN_UART3_RTS_1           (PIN_ALT2   | PIN_PORTB | PIN12)
#define PIN_UART3_RTS_2           (PIN_ALT3   | PIN_PORTB | PIN8)
#define PIN_UART3_RTS_3           (PIN_ALT3   | PIN_PORTC | PIN18)
#define PIN_UART3_RTS_4           (PIN_ALT3   | PIN_PORTE | PIN7)
#define PIN_UART3_RX_1            (PIN_ALT3   | PIN_PORTB | PIN10)
#define PIN_UART3_RX_2            (PIN_ALT3   | PIN_PORTC | PIN16)
#define PIN_UART3_RX_3            (PIN_ALT3   | PIN_PORTE | PIN5)
#define PIN_UART3_TX_1            (PIN_ALT3   | PIN_PORTB | PIN11)
#define PIN_UART3_TX_2            (PIN_ALT3   | PIN_PORTC | PIN17)
#define PIN_UART3_TX_3            (PIN_ALT3   | PIN_PORTE | PIN4)

#define PIN_UART4_CTS_1           (PIN_ALT3   | PIN_PORTC | PIN13)
#define PIN_UART4_CTS_2           (PIN_ALT3   | PIN_PORTE | PIN26)
#define PIN_UART4_RTS_1           (PIN_ALT3   | PIN_PORTC | PIN12)
#define PIN_UART4_RTS_2           (PIN_ALT3   | PIN_PORTE | PIN27)
#define PIN_UART4_RX_1            (PIN_ALT3   | PIN_PORTC | PIN14)
#define PIN_UART4_RX_2            (PIN_ALT3   | PIN_PORTE | PIN25)
#define PIN_UART4_TX_1            (PIN_ALT3   | PIN_PORTC | PIN15)
#define PIN_UART4_TX_2            (PIN_ALT3   | PIN_PORTE | PIN24)

#define PIN_LPUART0_CTS_B_1       (PIN_ALT5   | PIN_PORTE | PIN10)
#define PIN_LPUART0_CTS_B_2       (PIN_ALT5   | PIN_PORTA | PIN0)
#define PIN_LPUART0_CTS_B_3       (PIN_ALT5   | PIN_PORTD | PIN11)
#define PIN_LPUART0_RTS_B_1       (PIN_ALT5   | PIN_PORTE | PIN11)
#define PIN_LPUART0_RTS_B_2       (PIN_ALT5   | PIN_PORTA | PIN3)
#define PIN_LPUART0_RTS_B_3       (PIN_ALT5   | PIN_PORTD | PIN10)
#define PIN_LPUART0_RX_1          (PIN_ALT5   | PIN_PORTE | PIN9)
#define PIN_LPUART0_RX_2          (PIN_ALT5   | PIN_PORTA | PIN1)
#define PIN_LPUART0_RX_3          (PIN_ALT5   | PIN_PORTD | PIN8)
#define PIN_LPUART0_TX_1          (PIN_ALT5   | PIN_PORTE | PIN8)
#define PIN_LPUART0_TX_2          (PIN_ALT5   | PIN_PORTA | PIN2)
#define PIN_LPUART0_TX_3          (PIN_ALT5   | PIN_PORTD | PIN9)

/* USB */

#define PIN_USB0_CLKIN_1          (PIN_ALT2   | PIN_PORTA | PIN5)
#define PIN_USB0_CLKIN_2          (PIN_ALT7   | PIN_PORTE | PIN26)
#define PIN_USB0_SOF_OUT_1        (PIN_ALT3   | PIN_PORTC | PIN7)
#define PIN_USB0_SOF_OUT_2        (PIN_ALT4   | PIN_PORTC | PIN0)
#define PIN_USB0_SOF_OUT_3        (PIN_ALT7   | PIN_PORTE | PIN6)

#define PIN_USB1_ID_1             (PIN_ALT7   | PIN_PORTE | PIN10)

/* External Crystal */

#define PIN_EXTAL0                (PIN_ANALOG | PIN_PORTA | PIN18)
#define PIN_XTAL0                 (PIN_ANALOG | PIN_PORTA | PIN19)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* KINETIS_K66 */
#endif /* __ARCH_ARM_SRC_KINETIS_CHP_KINETIS_K66PINMUX_H */
