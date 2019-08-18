/********************************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k116_pinmux.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K116_PINMUX_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K116_PINMUX_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

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
 * and pull-up/down!  Just the basics are defined for most pins in the initial version of
 * this file.
 */

/* ADC */

#define PIN_ADC0_SE0              (PIN_ANALOG | PIN_PORTA | PIN0)
#define PIN_ADC0_SE1              (PIN_ANALOG | PIN_PORTA | PIN1)
#define PIN_ADC0_SE3              (PIN_ANALOG | PIN_PORTA | PIN7)
#define PIN_ADC0_SE4              (PIN_ANALOG | PIN_PORTB | PIN0)
#define PIN_ADC0_SE5              (PIN_ANALOG | PIN_PORTB | PIN1)
#define PIN_ADC0_SE6              (PIN_ANALOG | PIN_PORTB | PIN2)
#define PIN_ADC0_SE7              (PIN_ANALOG | PIN_PORTB | PIN3)
#define PIN_ADC0_SE9              (PIN_ANALOG | PIN_PORTC | PIN1)
#define PIN_ADC0_SE10             (PIN_ANALOG | PIN_PORTC | PIN2)
#define PIN_ADC0_SE11             (PIN_ANALOG | PIN_PORTC | PIN3)
#define PIN_ADC0_SE12             (PIN_ANALOG | PIN_PORTC | PIN14)
#define PIN_ADC0_SE13             (PIN_ANALOG | PIN_PORTC | PIN15)
#define PIN_ADC0_SE14             (PIN_ANALOG | PIN_PORTC | PIN16)

/* CAN */

#define PIN_CAN0_RX_1             (PIN_ALT3   | PIN_PORTC | PIN2)
#define PIN_CAN0_RX_2             (PIN_ALT5   | PIN_PORTB | PIN0)
#define PIN_CAN0_RX_3             (PIN_ALT5   | PIN_PORTE | PIN4)
#define PIN_CAN0_TX_1             (PIN_ALT3   | PIN_PORTC | PIN3)
#define PIN_CAN0_TX_2             (PIN_ALT5   | PIN_PORTB | PIN1)
#define PIN_CAN0_TX_3             (PIN_ALT5   | PIN_PORTE | PIN5)

/* Output clock */

#define PIN_CLKOUT                (PIN_ALT5   | PIN_PORTB | PIN5)

/* Comparators */

#define PIN_CMP0_IN0              (PIN_ANALOG | PIN_PORTA | PIN0)
#define PIN_CMP0_IN1              (PIN_ANALOG | PIN_PORTA | PIN1)
#define PIN_CMP0_IN2              (PIN_ANALOG | PIN_PORTC | PIN4)
#define PIN_CMP0_IN3              (PIN_ANALOG | PIN_PORTE | PIN8)
#define PIN_CMP0_IN4              (PIN_ANALOG | PIN_PORTC | PIN3)
#define PIN_CMP0_IN5              (PIN_ANALOG | PIN_PORTC | PIN2)
#define PIN_CMP0_OUT              (PIN_ALT4   | PIN_PORTA | PIN4)
#define PIN_CMP0_RRT_1            (PIN_ALT5   | PIN_PORTA | PIN11)
#define PIN_CMP0_RRT_2            (PIN_ALT5   | PIN_PORTD | PIN16)

/* FlexTimer Module (FTM) */

#define PIN_FTM0_CH0              (PIN_ALT2   | PIN_PORTD | PIN15)
#define PIN_FTM0_CH1_1            (PIN_ALT2   | PIN_PORTB | PIN13)
#define PIN_FTM0_CH1_2            (PIN_ALT2   | PIN_PORTC | PIN1)
#define PIN_FTM0_CH1_3            (PIN_ALT2   | PIN_PORTD | PIN16)
#define PIN_FTM0_CH2_1            (PIN_ALT2   | PIN_PORTC | PIN2)
#define PIN_FTM0_CH2_2            (PIN_ALT2   | PIN_PORTD | PIN0)
#define PIN_FTM0_CH3_1            (PIN_ALT2   | PIN_PORTC | PIN3)
#define PIN_FTM0_CH3_2            (PIN_ALT2   | PIN_PORTD | PIN1)
#define PIN_FTM0_CH4              (PIN_ALT2   | PIN_PORTB | PIN4)
#define PIN_FTM0_CH5              (PIN_ALT2   | PIN_PORTB | PIN5)
#define PIN_FTM0_CH6              (PIN_ALT2   | PIN_PORTE | PIN8)
#define PIN_FTM0_CH7              (PIN_ALT2   | PIN_PORTE | PIN9)
#define PIN_FTM0_FLT2             (PIN_ALT2   | PIN_PORTA | PIN7)

#define PIN_FTM1_CH0_1            (PIN_ALT2   | PIN_PORTB | PIN2)
#define PIN_FTM1_CH0_2            (PIN_ALT2   | PIN_PORTC | PIN4)
#define PIN_FTM1_CH1_1            (PIN_ALT2   | PIN_PORTA | PIN1)
#define PIN_FTM1_CH1_2            (PIN_ALT2   | PIN_PORTB | PIN3)
#define PIN_FTM1_CH2              (PIN_ALT2   | PIN_PORTC | PIN14)
#define PIN_FTM1_CH3              (PIN_ALT2   | PIN_PORTC | PIN15)
#define PIN_FTM1_CH4              (PIN_ALT2   | PIN_PORTA | PIN10)
#define PIN_FTM1_CH5              (PIN_ALT2   | PIN_PORTA | PIN11)
#define PIN_FTM1_CH6              (PIN_ALT2   | PIN_PORTA | PIN12)
#define PIN_FTM1_CH7_1            (PIN_ALT2   | PIN_PORTA | PIN13)
#define PIN_FTM1_CH7_2            (PIN_ALT6   | PIN_PORTC | PIN1)
#define PIN_FTM1_FLT0             (PIN_ALT3   | PIN_PORTC | PIN8)
#define PIN_FTM1_FLT1             (PIN_ALT3   | PIN_PORTC | PIN9)
#define PIN_FTM1_FLT2             (PIN_ALT2   | PIN_PORTC | PIN16)
#define PIN_FTM1_QD_PHA_1         (PIN_ALT4   | PIN_PORTB | PIN3)
#define PIN_FTM1_QD_PHA_2         (PIN_ALT5   | PIN_PORTA | PIN1)
#define PIN_FTM1_QD_PHA_3         (PIN_ALT6   | PIN_PORTC | PIN7)
#define PIN_FTM1_QD_PHB_1         (PIN_ALT4   | PIN_PORTB | PIN2)
#define PIN_FTM1_QD_PHB_2         (PIN_ALT6   | PIN_PORTC | PIN4)
#define PIN_FTM1_QD_PHB_3         (PIN_ALT6   | PIN_PORTC | PIN6)

/* FlexIO */

#define PIN_FXIO_D0_1             (PIN_ALT4   | PIN_PORTA | PIN10)
#define PIN_FXIO_D0_2             (PIN_ALT6   | PIN_PORTD | PIN0)
#define PIN_FXIO_D1_1             (PIN_ALT4   | PIN_PORTA | PIN11)
#define PIN_FXIO_D1_2             (PIN_ALT6   | PIN_PORTD | PIN1)
#define PIN_FXIO_D2               (PIN_ALT4   | PIN_PORTA | PIN0)
#define PIN_FXIO_D3               (PIN_ALT4   | PIN_PORTA | PIN1)
#define PIN_FXIO_D4_1             (PIN_ALT4   | PIN_PORTD | PIN2)
#define PIN_FXIO_D4_2             (PIN_ALT5   | PIN_PORTA | PIN2)
#define PIN_FXIO_D5_1             (PIN_ALT4   | PIN_PORTD | PIN3)
#define PIN_FXIO_D5_2             (PIN_ALT5   | PIN_PORTA | PIN3)
#define PIN_FXIO_D6_1             (PIN_ALT5   | PIN_PORTD | PIN2)
#define PIN_FXIO_D6_2             (PIN_ALT6   | PIN_PORTE | PIN4)
#define PIN_FXIO_D7_1             (PIN_ALT5   | PIN_PORTD | PIN3)
#define PIN_FXIO_D7_2             (PIN_ALT6   | PIN_PORTE | PIN5)

/* JTAG */

#define PIN_JTAG_TCLK             (PIN_ALT7   | PIN_PORTC | PIN4)
#define PIN_JTAG_TDI              (PIN_ALT7   | PIN_PORTC | PIN5)
#define PIN_JTAG_TDO              (PIN_ALT7   | PIN_PORTA | PIN10)
#define PIN_JTAG_TMS              (PIN_ALT7   | PIN_PORTA | PIN4)

/* LPI2C */

#define PIN_LPI2C0_SCL_1          (PIN_ALT2   | PIN_PORTB | PIN7)
#define PIN_LPI2C0_SCL_2          (PIN_ALT3   | PIN_PORTA | PIN3)
#define PIN_LPI2C0_SCLS           (PIN_ALT3   | PIN_PORTA | PIN0)
#define PIN_LPI2C0_SDA_1          (PIN_ALT2   | PIN_PORTB | PIN6)
#define PIN_LPI2C0_SDA_2          (PIN_ALT3   | PIN_PORTA | PIN2)
#define PIN_LPI2C0_SDAS           (PIN_ALT3   | PIN_PORTA | PIN1)
#define PIN_LPSPI0_PCS0_1         (PIN_ALT3   | PIN_PORTB | PIN0)
#define PIN_LPSPI0_PCS0_2         (PIN_ALT4   | PIN_PORTB | PIN5)
#define PIN_LPSPI0_PCS1           (PIN_ALT3   | PIN_PORTB | PIN5)
#define PIN_LPSPI0_SCK_1          (PIN_ALT3   | PIN_PORTB | PIN2)
#define PIN_LPSPI0_SCK_2          (PIN_ALT4   | PIN_PORTD | PIN15)
#define PIN_LPSPI0_SIN_1          (PIN_ALT3   | PIN_PORTB | PIN3)
#define PIN_LPSPI0_SIN_2          (PIN_ALT4   | PIN_PORTD | PIN16)
#define PIN_LPSPI0_SOUT_1         (PIN_ALT3   | PIN_PORTB | PIN1)
#define PIN_LPSPI0_SOUT_2         (PIN_ALT3   | PIN_PORTB | PIN4)

/* LPTimer */

#define PIN_LPTMR0_ALT2           (PIN_ALT3   | PIN_PORTD | PIN5)
#define PIN_LPTMR0_ALT3           (PIN_ALT4   | PIN_PORTB | PIN0)

/* LPUARTs */

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

#define PIN_LPUART1_RTS           (PIN_ALT6   | PIN_PORTA | PIN7)
#define PIN_LPUART1_RX_1          (PIN_ALT2   | PIN_PORTC | PIN6)
#define PIN_LPUART1_RX_2          (PIN_ALT2   | PIN_PORTC | PIN8)
#define PIN_LPUART1_TX_1          (PIN_ALT2   | PIN_PORTC | PIN7)
#define PIN_LPUART1_TX_2          (PIN_ALT2   | PIN_PORTC | PIN9)

/* NMI */

#define PIN_NMI                   (PIN_ALT7   | PIN_PORTD | PIN3)

/* GPIO */

#define PIN_PTA0                  (PIN_ALT1   | PIN_PORTA | PIN0)
#define PIN_PTA1                  (PIN_ALT1   | PIN_PORTA | PIN1)
#define PIN_PTA2                  (PIN_ALT1   | PIN_PORTA | PIN2)
#define PIN_PTA3                  (PIN_ALT1   | PIN_PORTA | PIN3)
#define PIN_PTA4                  (PIN_ALT1   | PIN_PORTA | PIN4)
#define PIN_PTA5                  (PIN_ALT1   | PIN_PORTA | PIN5)
#define PIN_PTA7                  (PIN_ALT1   | PIN_PORTA | PIN7)
#define PIN_PTA10                 (PIN_ALT1   | PIN_PORTA | PIN10)
#define PIN_PTA11                 (PIN_ALT1   | PIN_PORTA | PIN11)
#define PIN_PTA12                 (PIN_ALT1   | PIN_PORTA | PIN12)
#define PIN_PTA13                 (PIN_ALT1   | PIN_PORTA | PIN13)

#define PIN_PTB0                  (PIN_ALT1   | PIN_PORTB | PIN0)
#define PIN_PTB1                  (PIN_ALT1   | PIN_PORTB | PIN1)
#define PIN_PTB2                  (PIN_ALT1   | PIN_PORTB | PIN2)
#define PIN_PTB3                  (PIN_ALT1   | PIN_PORTB | PIN3)
#define PIN_PTB4                  (PIN_ALT1   | PIN_PORTB | PIN4)
#define PIN_PTB5                  (PIN_ALT1   | PIN_PORTB | PIN5)
#define PIN_PTB6                  (PIN_ALT1   | PIN_PORTB | PIN6)
#define PIN_PTB7                  (PIN_ALT1   | PIN_PORTB | PIN7)
#define PIN_PTB13                 (PIN_ALT1   | PIN_PORTB | PIN13)

#define PIN_PTC1                  (PIN_ALT1   | PIN_PORTC | PIN1)
#define PIN_PTC2                  (PIN_ALT1   | PIN_PORTC | PIN2)
#define PIN_PTC3                  (PIN_ALT1   | PIN_PORTC | PIN3)
#define PIN_PTC4                  (PIN_ALT1   | PIN_PORTC | PIN4)
#define PIN_PTC5                  (PIN_ALT1   | PIN_PORTC | PIN5)
#define PIN_PTC6                  (PIN_ALT1   | PIN_PORTC | PIN6)
#define PIN_PTC7                  (PIN_ALT1   | PIN_PORTC | PIN7)
#define PIN_PTC8                  (PIN_ALT1   | PIN_PORTC | PIN8)
#define PIN_PTC9                  (PIN_ALT1   | PIN_PORTC | PIN9)
#define PIN_PTC14                 (PIN_ALT1   | PIN_PORTC | PIN14)
#define PIN_PTC15                 (PIN_ALT1   | PIN_PORTC | PIN15)
#define PIN_PTC16                 (PIN_ALT1   | PIN_PORTC | PIN16)

#define PIN_PTD0                  (PIN_ALT1   | PIN_PORTD | PIN0)
#define PIN_PTD1                  (PIN_ALT1   | PIN_PORTD | PIN1)
#define PIN_PTD2                  (PIN_ALT1   | PIN_PORTD | PIN2)
#define PIN_PTD3                  (PIN_ALT1   | PIN_PORTD | PIN3)
#define PIN_PTD5                  (PIN_ALT1   | PIN_PORTD | PIN5)
#define PIN_PTE4                  (PIN_ALT1   | PIN_PORTE | PIN4)
#define PIN_PTE5                  (PIN_ALT1   | PIN_PORTE | PIN5)
#define PIN_PTE8                  (PIN_ALT1   | PIN_PORTE | PIN8)
#define PIN_PTE9                  (PIN_ALT1   | PIN_PORTE | PIN9)
#define PIN_PTD15                 (PIN_ALT1   | PIN_PORTD | PIN15)
#define PIN_PTD16                 (PIN_ALT1   | PIN_PORTD | PIN16)

/* Reset */

#define PIN_RESET                 (PIN_ALT7   | PIN_PORTA | PIN5)

/* RTC */

#define PIN_RTC_CLKIN             (PIN_ALT4   | PIN_PORTA | PIN7)
#define PIN_RTC_CLKOUT_1          (PIN_ALT3   | PIN_PORTC | PIN4)
#define PIN_RTC_CLKOUT_2          (PIN_ALT3   | PIN_PORTC | PIN5)

/* SWD */

#define PIN_SWD_CLK               (PIN_ALT7   | PIN_PORTC | PIN4)
#define PIN_SWD_DIO               (PIN_ALT7   | PIN_PORTA | PIN4)

/* Test Clock Input (TCLK) */

#define PIN_TCLK0                 (PIN_ALT4   | PIN_PORTB | PIN1)
#define PIN_TCLK1                 (PIN_ALT3   | PIN_PORTA | PIN5)
#define PIN_TCLK2                 (PIN_ALT2   | PIN_PORTE | PIN5)

/* Trigger Mux Control (TRGMUX) */

#define PIN_TRGMUX_IN0            (PIN_ALT6   | PIN_PORTB | PIN5)
#define PIN_TRGMUX_IN1            (PIN_ALT6   | PIN_PORTB | PIN4)
#define PIN_TRGMUX_IN2            (PIN_ALT6   | PIN_PORTB | PIN3)
#define PIN_TRGMUX_IN3            (PIN_ALT6   | PIN_PORTB | PIN2)
#define PIN_TRGMUX_IN4            (PIN_ALT6   | PIN_PORTD | PIN3)
#define PIN_TRGMUX_IN5            (PIN_ALT6   | PIN_PORTD | PIN2)
#define PIN_TRGMUX_IN7            (PIN_ALT6   | PIN_PORTD | PIN5)
#define PIN_TRGMUX_IN8            (PIN_ALT6   | PIN_PORTC | PIN15)
#define PIN_TRGMUX_IN9            (PIN_ALT6   | PIN_PORTC | PIN14)
#define PIN_TRGMUX_OUT0           (PIN_ALT7   | PIN_PORTA | PIN1)
#define PIN_TRGMUX_OUT1           (PIN_ALT7   | PIN_PORTD | PIN0)
#define PIN_TRGMUX_OUT2           (PIN_ALT7   | PIN_PORTD | PIN1)
#define PIN_TRGMUX_OUT3           (PIN_ALT7   | PIN_PORTA | PIN0)

/* External Crystal */

#define PIN_EXTAL                 (PIN_ANALOG | PIN_PORTB | PIN7)
#define PIN_XTAL                  (PIN_ANALOG | PIN_PORTB | PIN6)

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K116_PINMUX_H */
