/********************************************************************************************
 * arch/arm/src/kinetis/kinetis_k40pinmux.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_K20PINMUX_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_K20PINMUX_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* Reference: Paragraph 10.3.1, p 207, of FreeScale document K20P64M72SF1RM
 *
 * In most cases, there are alternative configurations for various pins. Those alternative
 * pins are labelled with a suffix like _1, _2, etc. in order to distinguish them.  Logic in
 * the board.h file must select the correct pin configuration for the board by defining a pin
 * configuration (with no suffix) that maps to the correct alternative.
 */

#if defined(CONFIG_ARCH_CHIP_MK20DX256VLH7)

#  define PIN_ADC0_SE4b       (PIN_ALT0 | PIN_PORTC | PIN2)
#  define PIN_ADC0_SE5b       (PIN_ALT0 | PIN_PORTD | PIN1)
#  define PIN_ADC0_SE6b       (PIN_ALT0 | PIN_PORTD | PIN5)
#  define PIN_ADC0_SE7b       (PIN_ALT0 | PIN_PORTD | PIN6)
#  define PIN_ADC0_SE8        (PIN_ALT0 | PIN_PORTB | PIN0)
#  define PIN_ADC0_SE9        (PIN_ALT0 | PIN_PORTB | PIN1)
#  define PIN_ADC0_SE12       (PIN_ALT0 | PIN_PORTB | PIN2)
#  define PIN_ADC0_SE13       (PIN_ALT0 | PIN_PORTB | PIN3)
#  define PIN_ADC0_SE14       (PIN_ALT0 | PIN_PORTC | PIN0)
#  define PIN_ADC0_SE15       (PIN_ALT0 | PIN_PORTC | PIN1)
#  define PIN_ADC1_SE4a       (PIN_ALT0 | PIN_PORTE | PIN0)
#  define PIN_ADC1_SE4b       (PIN_ALT0 | PIN_PORTC | PIN8)
#  define PIN_ADC1_SE5a       (PIN_ALT0 | PIN_PORTE | PIN1)
#  define PIN_ADC1_SE5b       (PIN_ALT0 | PIN_PORTC | PIN9)
#  define PIN_ADC1_SE6b       (PIN_ALT0 | PIN_PORTC | PIN10)
#  define PIN_ADC1_SE7b       (PIN_ALT0 | PIN_PORTC | PIN11)
#  define PIN_ADC1_SE8        (PIN_ALT0 | PIN_PORTB | PIN0)
#  define PIN_ADC1_SE9        (PIN_ALT0 | PIN_PORTB | PIN1)

#  define PIN_CAN0_RX_1       (PIN_ALT2 | PIN_PORTA | PIN13)
#  define PIN_CAN0_RX_2       (PIN_ALT2 | PIN_PORTB | PIN19)
#  define PIN_CAN0_TX_1       (PIN_ALT2 | PIN_PORTA | PIN12)
#  define PIN_CAN0_TX_2       (PIN_ALT2 | PIN_PORTB | PIN18)
#  define PIN_CLKOUT          (PIN_ALT5 | PIN_PORTC | PIN3)
#  define PIN_CMP0_IN0        (PIN_ALT0 | PIN_PORTC | PIN6)
#  define PIN_CMP0_IN1        (PIN_ALT0 | PIN_PORTC | PIN7)
#  define PIN_CMP0_IN2        (PIN_ALT0 | PIN_PORTC | PIN8)
#  define PIN_CMP0_IN3        (PIN_ALT0 | PIN_PORTC | PIN9)
#  define PIN_CMP0_OUT        (PIN_ALT6 | PIN_PORTC | PIN5)
#  define PIN_CMP1_IN0        (PIN_ALT0 | PIN_PORTC | PIN2)
#  define PIN_CMP1_IN1        (PIN_ALT0 | PIN_PORTC | PIN3)
#  define PIN_CMP1_OUT        (PIN_ALT6 | PIN_PORTC | PIN4)
#  define PIN_CMP2_IN0        (PIN_ALT0 | PIN_PORTA | PIN12)
#  define PIN_CMP2_IN1        (PIN_ALT0 | PIN_PORTA | PIN13)
#  define PIN_CMP2_OUT        (PIN_ALT4 | PIN_PORTA | PIN5)
#  define PIN_CMT_IRO         (PIN_ALT2 | PIN_PORTD | PIN7)
#  define PIN_EWM_IN_1        (PIN_ALT6 | PIN_PORTB | PIN16)
#  define PIN_EWM_IN_2        (PIN_ALT6 | PIN_PORTD | PIN4)
#  define PIN_EWM_OUT_b_1     (PIN_ALT6 | PIN_PORTB | PIN17)
#  define PIN_EWM_OUT_b_2     (PIN_ALT6 | PIN_PORTD | PIN5)

#  define PIN_FB_AD0          (PIN_ALT5 | PIN_PORTD | PIN6)
#  define PIN_FB_AD1          (PIN_ALT5 | PIN_PORTD | PIN5)
#  define PIN_FB_AD2          (PIN_ALT5 | PIN_PORTD | PIN4)
#  define PIN_FB_AD3          (PIN_ALT5 | PIN_PORTD | PIN3)
#  define PIN_FB_AD4          (PIN_ALT5 | PIN_PORTD | PIN2)
#  define PIN_FB_AD5          (PIN_ALT5 | PIN_PORTC | PIN10)
#  define PIN_FB_AD6          (PIN_ALT5 | PIN_PORTC | PIN9)
#  define PIN_FB_AD7          (PIN_ALT5 | PIN_PORTC | PIN8)
#  define PIN_FB_AD8          (PIN_ALT5 | PIN_PORTC | PIN7)
#  define PIN_FB_AD9          (PIN_ALT5 | PIN_PORTC | PIN6)
#  define PIN_FB_AD10         (PIN_ALT5 | PIN_PORTC | PIN5)
#  define PIN_FB_AD11         (PIN_ALT5 | PIN_PORTC | PIN4)
#  define PIN_FB_AD12         (PIN_ALT5 | PIN_PORTC | PIN2)
#  define PIN_FB_AD13         (PIN_ALT5 | PIN_PORTC | PIN1)
#  define PIN_FB_AD14         (PIN_ALT5 | PIN_PORTC | PIN0)
#  define PIN_FB_AD15         (PIN_ALT5 | PIN_PORTB | PIN18)
#  define PIN_FB_AD16         (PIN_ALT5 | PIN_PORTB | PIN17)
#  define PIN_FB_AD17         (PIN_ALT5 | PIN_PORTB | PIN16)
#  define PIN_FB_ALE          (PIN_ALT5 | PIN_PORTD | PIN0)
#  define PIN_FB_CS0_b        (PIN_ALT5 | PIN_PORTD | PIN1)
#  define PIN_FB_CS1_b        (PIN_ALT5 | PIN_PORTD | PIN0)
#  define PIN_FB_OE_b         (PIN_ALT5 | PIN_PORTB | PIN19)
#  define PIN_FB_RW_b         (PIN_ALT5 | PIN_PORTC | PIN11)
#  define PIN_FB_TS_b         (PIN_ALT5 | PIN_PORTD | PIN0)

#  define PIN_FTM_CLKIN0      (PIN_ALT4 | PIN_PORTA | PIN18)
#  define PIN_FTM_CLKIN1      (PIN_ALT4 | PIN_PORTA | PIN19)
#  define PIN_FTM0_CH0_1      (PIN_ALT3 | PIN_PORTA | PIN3)
#  define PIN_FTM0_CH0_2      (PIN_ALT4 | PIN_PORTC | PIN1)
#  define PIN_FTM0_CH1_1      (PIN_ALT2 | PIN_PORTA | PIN4)
#  define PIN_FTM0_CH1_2      (PIN_ALT4 | PIN_PORTC | PIN2)
#  define PIN_FTM0_CH2_1      (PIN_ALT2 | PIN_PORTA | PIN5)
#  define PIN_FTM0_CH2_2      (PIN_ALT4 | PIN_PORTC | PIN3)
#  define PIN_FTM0_CH3        (PIN_ALT4 | PIN_PORTC | PIN4)
#  define PIN_FTM0_CH4        (PIN_ALT4 | PIN_PORTD | PIN4)
#  define PIN_FTM0_CH5_1      (PIN_ALT3 | PIN_PORTA | PIN0)
#  define PIN_FTM0_CH5_2      (PIN_ALT4 | PIN_PORTD | PIN5)
#  define PIN_FTM0_CH6_1      (PIN_ALT3 | PIN_PORTA | PIN1)
#  define PIN_FTM0_CH6_2      (PIN_ALT4 | PIN_PORTD | PIN6)
#  define PIN_FTM0_CH7_1      (PIN_ALT3 | PIN_PORTA | PIN2)
#  define PIN_FTM0_CH7_2      (PIN_ALT4 | PIN_PORTD | PIN7)
#  define PIN_FTM0_FLT0_1     (PIN_ALT6 | PIN_PORTB | PIN3)
#  define PIN_FTM0_FLT0_2     (PIN_ALT6 | PIN_PORTD | PIN6)
#  define PIN_FTM0_FLT1       (PIN_ALT6 | PIN_PORTD | PIN7)
#  define PIN_FTM0_FLT2       (PIN_ALT3 | PIN_PORTA | PIN18)
#  define PIN_FTM0_FLT3       (PIN_ALT6 | PIN_PORTB | PIN2)
#  define PIN_FTM1_CH0_1      (PIN_ALT3 | PIN_PORTA | PIN12)
#  define PIN_FTM1_CH0_2      (PIN_ALT3 | PIN_PORTB | PIN0)
#  define PIN_FTM1_CH1_1      (PIN_ALT3 | PIN_PORTA | PIN13)
#  define PIN_FTM1_CH1_2      (PIN_ALT3 | PIN_PORTB | PIN1)
#  define PIN_FTM1_FLT0       (PIN_ALT3 | PIN_PORTA | PIN19)
#  define PIN_FTM1_QD_PHA_1   (PIN_ALT6 | PIN_PORTB | PIN0)
#  define PIN_FTM1_QD_PHA_2   (PIN_ALT7 | PIN_PORTA | PIN12)
#  define PIN_FTM1_QD_PHB_1   (PIN_ALT6 | PIN_PORTB | PIN1)
#  define PIN_FTM1_QD_PHB_2   (PIN_ALT7 | PIN_PORTA | PIN13)
#  define PIN_FTM2_CH0        (PIN_ALT3 | PIN_PORTB | PIN18)
#  define PIN_FTM2_CH1        (PIN_ALT3 | PIN_PORTB | PIN19)
#  define PIN_FTM2_FLT0       (PIN_ALT6 | PIN_PORTC | PIN9)
#  define PIN_FTM2_QD_PHA     (PIN_ALT6 | PIN_PORTB | PIN18)
#  define PIN_FTM2_QD_PHB     (PIN_ALT6 | PIN_PORTB | PIN19)

#  define PIN_I2C0_SCL_1      (PIN_ALT2 | PIN_PORTB | PIN0)
#  define PIN_I2C0_SCL_2      (PIN_ALT2 | PIN_PORTB | PIN2)
#  define PIN_I2C0_SDA_1      (PIN_ALT2 | PIN_PORTB | PIN1)
#  define PIN_I2C0_SDA_2      (PIN_ALT2 | PIN_PORTB | PIN3)
#  define PIN_I2C1_SCL_1      (PIN_ALT2 | PIN_PORTC | PIN10)
#  define PIN_I2C1_SCL_2      (PIN_ALT6 | PIN_PORTE | PIN1)
#  define PIN_I2C1_SDA_1      (PIN_ALT2 | PIN_PORTC | PIN11)
#  define PIN_I2C1_SDA_2      (PIN_ALT6 | PIN_PORTE | PIN0)

#  define PIN_I2S0_MCLK_1     (PIN_ALT4 | PIN_PORTC | PIN8)
#  define PIN_I2S0_MCLK_2     (PIN_ALT6 | PIN_PORTC | PIN6)
#  define PIN_I2S0_RX_BCLK_1  (PIN_ALT4 | PIN_PORTC | PIN6)
#  define PIN_I2S0_RX_BCLK_2  (PIN_ALT4 | PIN_PORTC | PIN9)
#  define PIN_I2S0_RX_FS_1    (PIN_ALT4 | PIN_PORTC | PIN10)
#  define PIN_I2S0_RX_FS_2    (PIN_ALT4 | PIN_PORTC | PIN7)
#  define PIN_I2S0_RXD0       (PIN_ALT4 | PIN_PORTC | PIN5)
#  define PIN_I2S0_RXD1       (PIN_ALT4 | PIN_PORTC | PIN11)
#  define PIN_I2S0_TX_BCLK_1  (PIN_ALT4 | PIN_PORTB | PIN18)
#  define PIN_I2S0_TX_BCLK_2  (PIN_ALT5 | PIN_PORTA | PIN5)
#  define PIN_I2S0_TX_BCLK_3  (PIN_ALT6 | PIN_PORTC | PIN3)
#  define PIN_I2S0_TX_FS_1    (PIN_ALT4 | PIN_PORTB | PIN19)
#  define PIN_I2S0_TX_FS_2    (PIN_ALT6 | PIN_PORTA | PIN13)
#  define PIN_I2S0_TX_FS_3    (PIN_ALT6 | PIN_PORTC | PIN2)
#  define PIN_I2S0_TXD0_1     (PIN_ALT6 | PIN_PORTA | PIN12)
#  define PIN_I2S0_TXD0_2     (PIN_ALT6 | PIN_PORTC | PIN1)
#  define PIN_I2S0_TXD1       (PIN_ALT6 | PIN_PORTC | PIN0)

#  define PIN_JTAG_TCLK       (PIN_ALT7 | PIN_PORTA | PIN0)
#  define PIN_JTAG_TDI        (PIN_ALT7 | PIN_PORTA | PIN1)
#  define PIN_JTAG_TDO        (PIN_ALT7 | PIN_PORTA | PIN2)
#  define PIN_JTAG_TMS        (PIN_ALT7 | PIN_PORTA | PIN3)
#  define PIN_JTAG_TRST_b     (PIN_ALT6 | PIN_PORTA | PIN5)

#  define PIN_LLWU_P0         (PIN_ALT1 | PIN_PORTE | PIN1)
#  define PIN_LLWU_P3         (PIN_ALT1 | PIN_PORTA | PIN4)
#  define PIN_LLWU_P4         (PIN_ALT1 | PIN_PORTA | PIN13)
#  define PIN_LLWU_P5         (PIN_ALT1 | PIN_PORTB | PIN0)
#  define PIN_LLWU_P6         (PIN_ALT1 | PIN_PORTC | PIN1)
#  define PIN_LLWU_P7         (PIN_ALT1 | PIN_PORTC | PIN3)
#  define PIN_LLWU_P8         (PIN_ALT1 | PIN_PORTC | PIN4)
#  define PIN_LLWU_P9         (PIN_ALT1 | PIN_PORTC | PIN5)
#  define PIN_LLWU_P10        (PIN_ALT1 | PIN_PORTC | PIN6)
#  define PIN_LLWU_P11        (PIN_ALT1 | PIN_PORTC | PIN11)
#  define PIN_LLWU_P12        (PIN_ALT1 | PIN_PORTD | PIN0)
#  define PIN_LLWU_P13        (PIN_ALT1 | PIN_PORTD | PIN2)
#  define PIN_LLWU_P14        (PIN_ALT1 | PIN_PORTD | PIN4)
#  define PIN_LLWU_P15        (PIN_ALT1 | PIN_PORTD | PIN6)

#  define PIN_LPTMR0_ALT1     (PIN_ALT6 | PIN_PORTA | PIN19)
#  define PIN_LPTMR0_ALT2     (PIN_ALT3 | PIN_PORTC | PIN5)

#  define PIN_NMI_b           (PIN_ALT6 | PIN_PORTA | PIN4)

#  define PIN_PDB0_EXTRG_1    (PIN_ALT3 | PIN_PORTC | PIN0)
#  define PIN_PDB0_EXTRG_2    (PIN_ALT3 | PIN_PORTC | PIN6)

#  define PIN_PTA0            (PIN_ALT1 | PIN_PORTA | PIN0)
#  define PIN_PTA1            (PIN_ALT1 | PIN_PORTA | PIN1)
#  define PIN_PTA2            (PIN_ALT1 | PIN_PORTA | PIN2)
#  define PIN_PTA3            (PIN_ALT1 | PIN_PORTA | PIN3)
#  define PIN_PTA4            (PIN_ALT1 | PIN_PORTA | PIN4)
#  define PIN_PTA5            (PIN_ALT0 | PIN_PORTA | PIN5)
#  define PIN_PTA12           (PIN_ALT1 | PIN_PORTA | PIN12)
#  define PIN_PTA13           (PIN_ALT1 | PIN_PORTA | PIN13)
#  define PIN_PTA18           (PIN_ALT1 | PIN_PORTA | PIN18)
#  define PIN_PTA19           (PIN_ALT1 | PIN_PORTA | PIN19)
#  define PIN_PTB0            (PIN_ALT1 | PIN_PORTB | PIN0)
#  define PIN_PTB1            (PIN_ALT1 | PIN_PORTB | PIN1)
#  define PIN_PTB2            (PIN_ALT1 | PIN_PORTB | PIN2)
#  define PIN_PTB3            (PIN_ALT1 | PIN_PORTB | PIN3)
#  define PIN_PTB16           (PIN_ALT1 | PIN_PORTB | PIN16)
#  define PIN_PTB17           (PIN_ALT1 | PIN_PORTB | PIN17)
#  define PIN_PTB18           (PIN_ALT1 | PIN_PORTB | PIN18)
#  define PIN_PTB19           (PIN_ALT1 | PIN_PORTB | PIN19)
#  define PIN_PTC0            (PIN_ALT1 | PIN_PORTC | PIN0)
#  define PIN_PTC1            (PIN_ALT1 | PIN_PORTC | PIN1)
#  define PIN_PTC2            (PIN_ALT1 | PIN_PORTC | PIN2)
#  define PIN_PTC3            (PIN_ALT1 | PIN_PORTC | PIN3)
#  define PIN_PTC4            (PIN_ALT1 | PIN_PORTC | PIN4)
#  define PIN_PTC5            (PIN_ALT1 | PIN_PORTC | PIN5)
#  define PIN_PTC6            (PIN_ALT1 | PIN_PORTC | PIN6)
#  define PIN_PTC7            (PIN_ALT1 | PIN_PORTC | PIN7)
#  define PIN_PTC8            (PIN_ALT1 | PIN_PORTC | PIN8)
#  define PIN_PTC9            (PIN_ALT1 | PIN_PORTC | PIN9)
#  define PIN_PTC10           (PIN_ALT1 | PIN_PORTC | PIN10)
#  define PIN_PTC11           (PIN_ALT1 | PIN_PORTC | PIN11)
#  define PIN_PTD0            (PIN_ALT1 | PIN_PORTD | PIN0)
#  define PIN_PTD1            (PIN_ALT1 | PIN_PORTD | PIN1)
#  define PIN_PTD2            (PIN_ALT1 | PIN_PORTD | PIN2)
#  define PIN_PTD3            (PIN_ALT1 | PIN_PORTD | PIN3)
#  define PIN_PTD4            (PIN_ALT1 | PIN_PORTD | PIN4)
#  define PIN_PTD5            (PIN_ALT1 | PIN_PORTD | PIN5)
#  define PIN_PTD6            (PIN_ALT1 | PIN_PORTD | PIN6)
#  define PIN_PTD7            (PIN_ALT1 | PIN_PORTD | PIN7)
#  define PIN_PTE0            (PIN_ALT1 | PIN_PORTE | PIN0)
#  define PIN_PTE1            (PIN_ALT1 | PIN_PORTE | PIN1)

#  define PIN_RTC_CLKOUT      (PIN_ALT7 | PIN_PORTE | PIN0)

#  define PIN_SPI0_PCS0_1     (PIN_ALT2 | PIN_PORTC | PIN4)
#  define PIN_SPI0_PCS0_2     (PIN_ALT2 | PIN_PORTD | PIN0)
#  define PIN_SPI0_PCS1_1     (PIN_ALT2 | PIN_PORTC | PIN3)
#  define PIN_SPI0_PCS1_2     (PIN_ALT2 | PIN_PORTD | PIN4)
#  define PIN_SPI0_PCS2_1     (PIN_ALT2 | PIN_PORTC | PIN2)
#  define PIN_SPI0_PCS2_2     (PIN_ALT2 | PIN_PORTD | PIN5)
#  define PIN_SPI0_PCS3_1     (PIN_ALT2 | PIN_PORTC | PIN1)
#  define PIN_SPI0_PCS3_2     (PIN_ALT2 | PIN_PORTD | PIN6)
#  define PIN_SPI0_PCS4       (PIN_ALT2 | PIN_PORTC | PIN0)
#  define PIN_SPI0_SCK_1      (PIN_ALT2 | PIN_PORTC | PIN5)
#  define PIN_SPI0_SCK_2      (PIN_ALT2 | PIN_PORTD | PIN1)
#  define PIN_SPI0_SIN_1      (PIN_ALT2 | PIN_PORTC | PIN7)
#  define PIN_SPI0_SIN_2      (PIN_ALT2 | PIN_PORTD | PIN3)
#  define PIN_SPI0_SOUT_1     (PIN_ALT2 | PIN_PORTC | PIN6)
#  define PIN_SPI0_SOUT_2     (PIN_ALT2 | PIN_PORTD | PIN2)
#  define PIN_SPI1_PCS1       (PIN_ALT2 | PIN_PORTE | PIN0)
#  define PIN_SPI1_SIN_1      (PIN_ALT2 | PIN_PORTB | PIN17)
#  define PIN_SPI1_SIN_2      (PIN_ALT7 | PIN_PORTE | PIN1)
#  define PIN_SPI1_SOUT_1     (PIN_ALT2 | PIN_PORTB | PIN16)
#  define PIN_SPI1_SOUT_2     (PIN_ALT2 | PIN_PORTE | PIN1)

#  define PIN_SWD_CLK         (PIN_ALT7 | PIN_PORTA | PIN0)
#  define PIN_SWD_DIO         (PIN_ALT7 | PIN_PORTA | PIN3)
#  define PIN_TRACE_SWO       (PIN_ALT7 | PIN_PORTA | PIN2)
#  define PIN_TSI0_CH0        (PIN_ALT0 | PIN_PORTB | PIN0)
#  define PIN_TSI0_CH1        (PIN_ALT0 | PIN_PORTA | PIN0)
#  define PIN_TSI0_CH2        (PIN_ALT0 | PIN_PORTA | PIN1)
#  define PIN_TSI0_CH3        (PIN_ALT0 | PIN_PORTA | PIN2)
#  define PIN_TSI0_CH4        (PIN_ALT0 | PIN_PORTA | PIN3)
#  define PIN_TSI0_CH5        (PIN_ALT0 | PIN_PORTA | PIN4)
#  define PIN_TSI0_CH6        (PIN_ALT0 | PIN_PORTB | PIN1)
#  define PIN_TSI0_CH7        (PIN_ALT0 | PIN_PORTB | PIN2)
#  define PIN_TSI0_CH8        (PIN_ALT0 | PIN_PORTB | PIN3)
#  define PIN_TSI0_CH9        (PIN_ALT0 | PIN_PORTB | PIN16)
#  define PIN_TSI0_CH10       (PIN_ALT0 | PIN_PORTB | PIN17)
#  define PIN_TSI0_CH11       (PIN_ALT0 | PIN_PORTB | PIN18)
#  define PIN_TSI0_CH12       (PIN_ALT0 | PIN_PORTB | PIN19)
#  define PIN_TSI0_CH13       (PIN_ALT0 | PIN_PORTC | PIN0)
#  define PIN_TSI0_CH14       (PIN_ALT0 | PIN_PORTC | PIN1)
#  define PIN_TSI0_CH15       (PIN_ALT0 | PIN_PORTC | PIN2)

#  define PIN_UART0_COL_b_1   (PIN_ALT2 | PIN_PORTA | PIN0)
#  define PIN_UART0_COL_b_2   (PIN_ALT3 | PIN_PORTB | PIN3)
#  define PIN_UART0_COL_b_3   (PIN_ALT3 | PIN_PORTD | PIN5)
#  define PIN_UART0_CTS_b_1   (PIN_ALT2 | PIN_PORTA | PIN0)
#  define PIN_UART0_CTS_b_2   (PIN_ALT3 | PIN_PORTB | PIN3)
#  define PIN_UART0_CTS_b_3   (PIN_ALT3 | PIN_PORTD | PIN5)
#  define PIN_UART0_RTS_b_1   (PIN_ALT2 | PIN_PORTA | PIN3)
#  define PIN_UART0_RTS_b_2   (PIN_ALT3 | PIN_PORTB | PIN2)
#  define PIN_UART0_RTS_b_3   (PIN_ALT3 | PIN_PORTD | PIN4)
#  define PIN_UART0_RX_1      (PIN_ALT2 | PIN_PORTA | PIN1)
#  define PIN_UART0_RX_2      (PIN_ALT3 | PIN_PORTB | PIN16)
#  define PIN_UART0_RX_3      (PIN_ALT3 | PIN_PORTD | PIN6)
#  define PIN_UART0_TX_1      (PIN_ALT2 | PIN_PORTA | PIN2)
#  define PIN_UART0_TX_2      (PIN_ALT3 | PIN_PORTB | PIN17)
#  define PIN_UART0_TX_3      (PIN_ALT3 | PIN_PORTD | PIN7)
#  define PIN_UART1_CTS_b     (PIN_ALT3 | PIN_PORTC | PIN2)
#  define PIN_UART1_RTS_b     (PIN_ALT3 | PIN_PORTC | PIN1)
#  define PIN_UART1_RX_1      (PIN_ALT3 | PIN_PORTC | PIN3)
#  define PIN_UART1_RX_2      (PIN_ALT3 | PIN_PORTE | PIN1)
#  define PIN_UART1_TX_1      (PIN_ALT3 | PIN_PORTC | PIN4)
#  define PIN_UART1_TX_2      (PIN_ALT3 | PIN_PORTE | PIN0)
#  define PIN_UART2_CTS_b     (PIN_ALT3 | PIN_PORTD | PIN1)
#  define PIN_UART2_RTS_b     (PIN_ALT3 | PIN_PORTD | PIN0)
#  define PIN_UART2_RX        (PIN_ALT3 | PIN_PORTD | PIN2)
#  define PIN_UART2_TX        (PIN_ALT3 | PIN_PORTD | PIN3)

#  define PIN_USB_CLKIN       (PIN_ALT1 | PIN_PORTA | PIN5)
#  define PIN_USB_SOF_OUT     (PIN_ALT3 | PIN_PORTC | PIN7)

#  define PIN_EXTAL0          (PIN_ALT0 | PIN_PORTA | PIN18)
#  define PIN_XTAL0           (PIN_ALT0 | PIN_PORTA | PIN19)

#else
  /* The pin muxing for other K20 parts is defined in other documents */

#  error "No pin multiplexing for this Kinetis K20 part"
#endif

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_K20PINMUX_H */
