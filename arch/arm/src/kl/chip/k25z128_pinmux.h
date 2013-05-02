/********************************************************************************************
 * arch/arm/src/kl/k25z128_pinmux.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_KL25PINMUX_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_KL25PINMUX_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* Reference: Paragraph 10.3.1, p 227, of FreeScale document K40P144M100SF2RM
 *
 * In most cases, there are alternative configurations for various pins. Those alternative
 * pins are labeled with a suffix like _1, _2, etc. in order to distinguish them.  Logic in
 * the board.h file must select the correct pin configuration for the board by defining a pin
 * configuration (with no suffix) that maps to the correct alternative.
 */

#define PIN_TSI0_CH1             (PIN_ANALOG | PIN_PORTA | PIN0)
#define PIN_TPM0_CH5_1           (PIN_ALT3   | PIN_PORTA | PIN0)
#define PIN_SWD_CLK              (PIN_ALT7   | PIN_PORTA | PIN0)

#define PIN_TSI0_CH2             (PIN_ANALOG | PIN_PORTA | PIN1)
#define PIN_UART0_RX_1           (PIN_ALT2   | PIN_PORTA | PIN1)
#define PIN_TPM2_CH0_1           (PIN_ALT3   | PIN_PORTA | PIN1)

#define PIN_TSI0_CH3             (PIN_ANALOG | PIN_PORTA | PIN2)
#define PIN_UART0_TX_1           (PIN_ALT2   | PIN_PORTA | PIN2)
#define PIN_TPM2_CH1_1           (PIN_ALT3   | PIN_PORTA | PIN2)

#define PIN_TSI0_CH4             (PIN_ANALOG | PIN_PORTA | PIN3)
#define PIN_I2C1_SCL_1           (PIN_ALT2   | PIN_PORTA | PIN3)
#define PIN_TPM0_CH0_1           (PIN_ALT3   | PIN_PORTA | PIN3)
#define PIN_SWD_DIO              (PIN_ALT7   | PIN_PORTA | PIN3)

#define PIN_TSI0_CH5             (PIN_ANALOG | PIN_PORTA | PIN4)
#define PIN_I2C1_SDA_1           (PIN_ALT2   | PIN_PORTA | PIN4)
#define PIN_TPM0_CH1_1           (PIN_ALT3   | PIN_PORTA | PIN4)
#define PIN_NMI                  (PIN_ALT7   | PIN_PORTA | PIN4)

#define PIN_USB_CLKIN            (PIN_ALT2   | PIN_PORTA | PIN5)
#define PIN_TPM0_CH2_1             (PIN_ALT3   | PIN_PORTA | PIN5)

/* pins PTA6 up to PTA11 are not define at
 * 10.3.1 KL25 Signal Multiplexing and Pin Assignments
 */

#define PIN_TPM1_CH0_1           (PIN_ALT3   | PIN_PORTA | PIN12)

#define PIN_TPM1_CH1_1           (PIN_ALT3   | PIN_PORTA | PIN12)

#define PIN_SPI0_PCS0_1          (PIN_ALT2   | PIN_PORTA | PIN14)
#define PIN_UART0_TX_2           (PIN_ALT3   | PIN_PORTA | PIN14)

#define PIN_SPI0_SCK_1           (PIN_ALT2   | PIN_PORTA | PIN15)
#define PIN_UART0_RX_2           (PIN_ALT3   | PIN_PORTA | PIN15)

#define PIN_SPI0_MOSI_1          (PIN_ALT2   | PIN_PORTA | PIN16)
#define PIN_SPI0_MISO_1          (PIN_ALT5   | PIN_PORTA | PIN16)

#define PIN_SPI0_MISO_2          (PIN_ALT2   | PIN_PORTA | PIN17)
#define PIN_SPI0_MOSI_2          (PIN_ALT5   | PIN_PORTA | PIN17)

#define PIN_EXTAL0               (PIN_ANALOG | PIN_PORTA | PIN18)
#define PIN_UART1_RX_1           (PIN_ALT3   | PIN_PORTA | PIN18)
#define PIN_TPM_CLKIN0_1         (PIN_ALT4   | PIN_PORTA | PIN18)

#define PIN_XTAL0                (PIN_ANALOG | PIN_PORTA | PIN19)
#define PIN_UART1_TX_1           (PIN_ALT3   | PIN_PORTA | PIN19)
#define PIN_TPM_CLKIN1_1         (PIN_ALT4   | PIN_PORTA | PIN19)
#define PIN_LPTMR0_ALT1          (PIN_ALT6   | PIN_PORTA | PIN19)

/* pin PTA20 is RESET and pins PTA21 up to PTA31 are not define at
 * 10.3.1 KL25 Signal Multiplexing and Pin Assignments
 */

#define PIN_TSI0_CH0             (PIN_ANALOG | PIN_PORTB | PIN0)
#define PIN_ADC0_SE8             (PIN_ANALOG | PIN_PORTB | PIN0)
#define PIN_LLWU_P5              (PIN_ALT1   | PIN_PORTB | PIN0)
#define PIN_I2C0_SCL_1           (PIN_ALT2   | PIN_PORTB | PIN0)
#define PIN_TPM1_CH0_2           (PIN_ALT3   | PIN_PORTB | PIN0)

#define PIN_ADC0_SE9             (PIN_ANALOG | PIN_PORTB | PIN1)
#define PIN_TSI0_CH6             (PIN_ANALOG | PIN_PORTB | PIN1)
#define PIN_I2C0_SDA_1           (PIN_ALT2   | PIN_PORTB | PIN1)
#define PIN_TPM1_CH1_2           (PIN_ALT3   | PIN_PORTB | PIN1)

#define PIN_ADC0_SE12            (PIN_ANALOG | PIN_PORTB | PIN2)
#define PIN_TSI0_CH7             (PIN_ANALOG | PIN_PORTB | PIN2)
#define PIN_I2C0_SCL_2           (PIN_ALT2   | PIN_PORTB | PIN2)
#define PIN_TPM2_CH0_2           (PIN_ALT3   | PIN_PORTB | PIN2)

#define PIN_ADC0_SE13            (PIN_ANALOG | PIN_PORTB | PIN3)
#define PIN_TSI0_CH8             (PIN_ANALOG | PIN_PORTB | PIN3)
#define PIN_I2C0_SDA_2           (PIN_ALT2   | PIN_PORTB | PIN3)
#define PIN_TPM2_CH1_2           (PIN_ALT3   | PIN_PORTB | PIN3)

/* pins PTB4 up to PTB7 are not define at
 * 10.3.1 KL25 Signal Multiplexing and Pin Assignments
 */

#define PIN_EXTRG_IN_1           (PIN_ALT3   | PIN_PORTB | PIN8)

#define PIN_SPI1_PCS0_1          (PIN_ALT2   | PIN_PORTB | PIN10)

#define PIN_SPI1_SCK_1           (PIN_ALT2   | PIN_PORTB | PIN11)

#define PIN_TSI0_CH9             (PIN_ANALOG | PIN_PORTB | PIN16)
#define PIN_SPI1_MOSI_1          (PIN_ALT2   | PIN_PORTB | PIN16)
#define PIN_UART0_RX_3           (PIN_ALT3   | PIN_PORTB | PIN16)
#define PIN_TPM_CLKIN0_2         (PIN_ALT4   | PIN_PORTB | PIN16)
#define PIN_SPI1_MISO_1          (PIN_ALT5   | PIN_PORTB | PIN16)

#define PIN_TSI0_CH10            (PIN_ANALOG | PIN_PORTB | PIN17)
#define PIN_SPI1_MISO_2          (PIN_ALT2   | PIN_PORTB | PIN17)
#define PIN_UART0_TX_3           (PIN_ALT3   | PIN_PORTB | PIN17)
#define PIN_TPM_CLKIN1_2         (PIN_ALT4   | PIN_PORTB | PIN17)
#define PIN_SPI1_MOSI_2          (PIN_ALT7   | PIN_PORTB | PIN17)

#define PIN_TSI0_CH11            (PIN_ANALOG | PIN_PORTB | PIN18)
#define PIN_TPM2_CH0_3           (PIN_ALT3   | PIN_PORTB | PIN18)

#define PIN_TSI0_CH12            (PIN_ANALOG | PIN_PORTB | PIN19)
#define PIN_TPM2_CH1_3           (PIN_ALT3   | PIN_PORTB | PIN19)

/* pins PTB20 up to PTB31 are not define at
 * 10.3.1 KL25 Signal Multiplexing and Pin Assignments
 */

#define PIN_ADC0_SE14            (PIN_ANALOG | PIN_PORTC | PIN0)
#define PIN_TSI0_CH13            (PIN_ANALOG | PIN_PORTC | PIN0)
#define PIN_EXTRG_IN_2           (PIN_ALT3   | PIN_PORTC | PIN0)
#define PIN_CPM0_OUT             (PIN_ALT5   | PIN_PORTC | PIN0)

#define PIN_ADC0_SE15            (PIN_ANALOG | PIN_PORTC | PIN1)
#define PIN_TSI0_CH14            (PIN_ANALOG | PIN_PORTC | PIN1)
#define PIN_LLWU_P6              (PIN_ALT1   | PIN_PORTC | PIN1)
#define PIN_RTC_CLKIN            (PIN_ALT1   | PIN_PORTC | PIN1)
#define PIN_I2C1_SCL_2           (PIN_ALT2   | PIN_PORTC | PIN1)
#define PIN_TPM0_CH0_2           (PIN_ALT4   | PIN_PORTC | PIN1)

#define PIN_ADC0_SE11            (PIN_ANALOG | PIN_PORTC | PIN2)
#define PIN_TSI0_CH15            (PIN_ANALOG | PIN_PORTC | PIN2)
#define PIN_I2C1_SDA_2           (PIN_ALT2   | PIN_PORTC | PIN2)
#define PIN_TPM0_CH1_2           (PIN_ALT4   | PIN_PORTC | PIN2)

#define PIN_LLWU_P7              (PIN_ALT1   | PIN_PORTC | PIN3)
#define PIN_UART1_RX_2           (PIN_ALT3   | PIN_PORTC | PIN3)
#define PIN_TPM0_CH2_2           (PIN_ALT4   | PIN_PORTC | PIN3)
#define PIN_CLKOUT               (PIN_ALT5   | PIN_PORTC | PIN3)

#define PIN_LLWU_P8              (PIN_ALT1   | PIN_PORTC | PIN4)
#define PIN_SPI0_PCS0_2          (PIN_ALT2   | PIN_PORTC | PIN4)
#define PIN_UART1_TX_2           (PIN_ALT3   | PIN_PORTC | PIN4)
#define PIN_TPM0_CH3_1           (PIN_ALT4   | PIN_PORTC | PIN4)

#define PIN_LLWU_P9              (PIN_ALT1   | PIN_PORTC | PIN5)
#define PIN_SPI0_SCK_2           (PIN_ALT2   | PIN_PORTC | PIN5)
#define PIN_LPTMR0_ALT2          (PIN_ALT4   | PIN_PORTC | PIN5)
#define PIN_CMP0_OUT_1           (PIN_ALT6   | PIN_PORTC | PIN5)

#define PIN_CMP0_IN0             (PIN_ANALOG | PIN_PORTC | PIN6)
#define PIN_LLWU_P10             (PIN_ALT1   | PIN_PORTC | PIN6)
#define PIN_SPI0_MOSI_3          (PIN_ALT2   | PIN_PORTC | PIN6)
#define PIN_EXTRG_IN_3           (PIN_ALT3   | PIN_PORTC | PIN6)
#define PIN_SPI0_MISO_3          (PIN_ALT5   | PIN_PORTC | PIN6)

#define PIN_CMP0_IN1             (PIN_ANALOG | PIN_PORTC | PIN7)
#define PIN_SPI0_MISO_4          (PIN_ALT2   | PIN_PORTC | PIN7)
#define PIN_SPI0_MOSI_4          (PIN_ALT5   | PIN_PORTC | PIN7)

#define PIN_CMP0_IN2             (PIN_ANALOG | PIN_PORTC | PIN8)
#define PIN_I2C0_SCL_3           (PIN_ALT2   | PIN_PORTC | PIN8)
#define PIN_TPM0_CH4_1           (PIN_ALT3   | PIN_PORTC | PIN8)

#define PIN_CMP0_IN3             (PIN_ANALOG | PIN_PORTC | PIN9)
#define PIN_I2C0_SDA_3           (PIN_ALT2   | PIN_PORTC | PIN9)
#define PIN_TPM0_CH5_2           (PIN_ALT3   | PIN_PORTC | PIN9)

#define PIN_I2C1_SCL_3           (PIN_ALT2   | PIN_PORTC | PIN10)

#define PIN_I2C1_SDA_3           (PIN_ALT2   | PIN_PORTC | PIN11)

#define PIN_TPM_CLKIN0_3         (PIN_ALT4   | PIN_PORTC | PIN12)

#define PIN_TPM_CLKIN1_3         (PIN_ALT4   | PIN_PORTC | PIN13)

/* pins PTC18 up to PTC31 are not define at
 * 10.3.1 KL25 Signal Multiplexing and Pin Assignments
 */

#define PIN_SPI0_PCS0_3          (PIN_ALT2   | PIN_PORTD | PIN0)
#define PIN_TPM0_CH0_3           (PIN_ALT4   | PIN_PORTD | PIN0)

#define PIN_ADC0_SE5B            (PIN_ANALOG | PIN_PORTD | PIN1)
#define PIN_SPI0_SCK_3           (PIN_ALT2   | PIN_PORTD | PIN1)
#define PIN_TPM0_CH1_3           (PIN_ALT4   | PIN_PORTD | PIN1)

#define PIN_SPI0_MOSI_5          (PIN_ALT2   | PIN_PORTD | PIN2)
#define PIN_UART2_RX_1           (PIN_ALT3   | PIN_PORTD | PIN2)
#define PIN_TPM0_CH2_3           (PIN_ALT4   | PIN_PORTD | PIN2)
#define PIN_SPI0_MISO_5          (PIN_ALT5   | PIN_PORTD | PIN2)

#define PIN_SPI0_MISO_6          (PIN_ALT2   | PIN_PORTD | PIN3)
#define PIN_UART2_TX_1           (PIN_ALT3   | PIN_PORTD | PIN3)
#define PIN_TPM0_CH3_2           (PIN_ALT4   | PIN_PORTD | PIN3)
#define PIN_SPI0_MOSI_6          (PIN_ALT5   | PIN_PORTD | PIN3)

#define PIN_LLWU_P14             (PIN_ALT1   | PIN_PORTD | PIN4)
#define PIN_SPI0_PCS0_4          (PIN_ALT2   | PIN_PORTD | PIN4)
#define PIN_UART2_RX_2           (PIN_ALT3   | PIN_PORTD | PIN4)
#define PIN_TPM0_CH4_2           (PIN_ALT4   | PIN_PORTD | PIN4)

#define PIN_ADC0_SE6B            (PIN_ANALOG | PIN_PORTD | PIN5)
#define PIN_SPI1_SCK_2           (PIN_ALT2   | PIN_PORTD | PIN5)
#define PIN_UART2_TX_2           (PIN_ALT3   | PIN_PORTD | PIN5)
#define PIN_TPM0_CH5_3           (PIN_ALT4   | PIN_PORTD | PIN5)

#define PIN_ADC0_SE7B            (PIN_ANALOG | PIN_PORTD | PIN6)
#define PIN_LLWU_P15             (PIN_ALT1   | PIN_PORTD | PIN6)
#define PIN_SPI0_MOSI_7          (PIN_ALT2   | PIN_PORTD | PIN6)
#define PIN_UART0_RX_4           (PIN_ALT3   | PIN_PORTD | PIN6)
#define PIN_SPI0_MISO_7          (PIN_ALT5   | PIN_PORTD | PIN6)

#define PIN_SPI1_MISO_3          (PIN_ALT2   | PIN_PORTD | PIN7)
#define PIN_UART0_TX_4           (PIN_ALT3   | PIN_PORTD | PIN7)
#define PIN_SPI1_MOSI_3          (PIN_ALT5   | PIN_PORTD | PIN7)

/* pins PTD8 up to PTD31 are not define at
 * 10.3.1 KL25 Signal Multiplexing and Pin Assignments
 */

#define PIN_UART1_TX_3           (PIN_ALT3   | PIN_PORTE | PIN0)
#define PIN_RTC_CLKOUT           (PIN_ALT4   | PIN_PORTE | PIN0)
#define PIN_CMP0_OUT_2           (PIN_ALT5   | PIN_PORTE | PIN0)
#define PIN_I2C1_SDA_4           (PIN_ALT6   | PIN_PORTE | PIN0)

#define PIN_SPI1_MOSI_4          (PIN_ALT2   | PIN_PORTE | PIN1)
#define PIN_UART1_RX_3           (PIN_ALT3   | PIN_PORTE | PIN1)
#define PIN_SPI1_MISO_4          (PIN_ALT5   | PIN_PORTE | PIN1)
#define PIN_I2C1_SCL_4           (PIN_ALT6   | PIN_PORTE | PIN1)

#define PIN_SPI1_SCK_3           (PIN_ALT2   | PIN_PORTE | PIN2)

#define PIN_SPI1_MISO_5          (PIN_ALT2   | PIN_PORTE | PIN3)

#define PIN_SPI1_PCS0_2          (PIN_ALT2   | PIN_PORTE | PIN4)

#define PIN_ADC0_DP0             (PIN_ANALOG | PIN_PORTE | PIN20)
#define PIN_ADC0_SE0             (PIN_ANALOG | PIN_PORTE | PIN20)
#define PIN_TPM1_CH0_3           (PIN_ALT3   | PIN_PORTE | PIN20)
#define PIN_UART0_TX_5           (PIN_ALT4   | PIN_PORTE | PIN20)

#define PIN_ADC0_DM0             (PIN_ANALOG | PIN_PORTE | PIN21)
#define PIN_ADC0_SE4A            (PIN_ANALOG | PIN_PORTE | PIN21)
#define PIN_TPM1_CH1_3           (PIN_ALT3   | PIN_PORTE | PIN21)
#define PIN_UART0_RX_5           (PIN_ALT4   | PIN_PORTE | PIN21)

#define PIN_ADC0_DP3             (PIN_ANALOG | PIN_PORTE | PIN22)
#define PIN_ADC0_SE3             (PIN_ANALOG | PIN_PORTE | PIN22)
#define PIN_TPM2_CH0_4           (PIN_ALT3   | PIN_PORTE | PIN22)
#define PIN_UART2_TX_3           (PIN_ALT4   | PIN_PORTE | PIN22)

#define PIN_ADC0_DM3             (PIN_ANALOG | PIN_PORTE | PIN23)
#define PIN_ADC0_SE7A            (PIN_ANALOG | PIN_PORTE | PIN23)
#define PIN_TPM2_CH1_4           (PIN_ALT3   | PIN_PORTE | PIN23)
#define PIN_UART2_RX_3           (PIN_ALT4   | PIN_PORTE | PIN23)

#define PIN_TPM0_CH0_4           (PIN_ALT3   | PIN_PORTE | PIN24)
#define PIN_I2C0_SCL_4           (PIN_ALT5   | PIN_PORTE | PIN24)

#define PIN_TPM0_CH1_4           (PIN_ALT3   | PIN_PORTE | PIN25)
#define PIN_I2C0_SDA_4           (PIN_ALT5   | PIN_PORTE | PIN25)

#define PIN_CMP0_IN5             (PIN_ANALOG | PIN_PORTE | PIN29)
#define PIN_ADC0_SE4B            (PIN_ANALOG | PIN_PORTE | PIN29)
#define PIN_TPM0_CH2_4           (PIN_ALT3   | PIN_PORTE | PIN29)
#define PIN_TPM_CLKIN0_4         (PIN_ALT4   | PIN_PORTE | PIN29)

#define PIN_DAC0_OUT             (PIN_ANALOG | PIN_PORTE | PIN30)
#define PIN_ADC0_SE23            (PIN_ANALOG | PIN_PORTE | PIN30)
#define PIN_CMP0_IN4             (PIN_ANALOG | PIN_PORTE | PIN30)
#define PIN_TPM0_CH3_3           (PIN_ALT3   | PIN_PORTE | PIN30)
#define PIN_TPM_CLKIN1_4         (PIN_ALT4   | PIN_PORTE | PIN30)

#define PIN_TPM0_CH4_3           (PIN_ALT3   | PIN_PORTE | PIN31)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_K40PINMUX_H */
