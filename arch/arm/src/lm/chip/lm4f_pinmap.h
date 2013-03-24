/************************************************************************************
 * arch/arm/src/lm/chip/lm4f_pinmap.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LM_CHIP_LM4F_PINMAP_H
#define __ARCH_ARM_SRC_LM_CHIP_LM4F_PINMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Alternate Pin Functions.  All members of the LM4F family share the same pin
 * multiplexing (although they may differ in the pins physically available).
 *
 * Alternative pin selections are provided with a numeric suffix like _1, _2, etc.
 * Drivers, however, will use the pin selection without the numeric suffix.
 * Additional definitions are required in the board.h file.  For example, if
 * CAN1_RX connects vis PA11 on some board, then the following definitions should
 * appear inthe board.h header file for that board:
 *
 * #define GPIO_CAN1_RX GPIO_CAN1_RX_1
 *
 * The driver will then automatically configre PA11 as the CAN1 RX pin.
 */

/* WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!!
 * Additional effort is required to select specific GPIO options such as frequency,
 * open-drain/push-pull, and pull-up/down!  Just the basics are defined for most
 * pins in this file.
 */

#if defined(CONFIG_ARCH_CHIP_LM4F120)

#  define GPIO_ADC_IN0      (GPIO_FUNC_ANINPUT | GPIO_PORTE | GPIO_PIN_3)
#  define GPIO_ADC_IN1      (GPIO_FUNC_ANINPUT | GPIO_PORTE | GPIO_PIN_2)
#  define GPIO_ADC_IN2      (GPIO_FUNC_ANINPUT | GPIO_PORTE | GPIO_PIN_1)
#  define GPIO_ADC_IN3      (GPIO_FUNC_ANINPUT | GPIO_PORTE | GPIO_PIN_0)
#  define GPIO_ADC_IN4      (GPIO_FUNC_ANINPUT | GPIO_PORTD | GPIO_PIN_3)
#  define GPIO_ADC_IN5      (GPIO_FUNC_ANINPUT | GPIO_PORTD | GPIO_PIN_2)
#  define GPIO_ADC_IN6      (GPIO_FUNC_ANINPUT | GPIO_PORTD | GPIO_PIN_1)
#  define GPIO_ADC_IN7      (GPIO_FUNC_ANINPUT | GPIO_PORTD | GPIO_PIN_0)
#  define GPIO_ADC_IN8      (GPIO_FUNC_ANINPUT | GPIO_PORTE | GPIO_PIN_5)
#  define GPIO_ADC_IN9      (GPIO_FUNC_ANINPUT | GPIO_PORTE | GPIO_PIN_4)
#  define GPIO_ADC_IN10     (GPIO_FUNC_ANINPUT | GPIO_PORTB | GPIO_PIN_4)
#  define GPIO_ADC_IN11     (GPIO_FUNC_ANINPUT | GPIO_PORTB | GPIO_PIN_5)

#  define GPIO_CAN0_RX_1    (GPIO_FUNC_PFINPUT | GPIO_ALT_3 | GPIO_PORTF | GPIO_PIN_0)
#  define GPIO_CAN0_RX_2    (GPIO_FUNC_PFINPUT | GPIO_ALT_8 | GPIO_PORTB | GPIO_PIN_4)
#  define GPIO_CAN0_RX_3    (GPIO_FUNC_PFINPUT | GPIO_ALT_8 | GPIO_PORTE | GPIO_PIN_4)
#  define GPIO_CAN0_TX_1    (GPIO_FUNC_PFOUTPUT | GPIO_ALT_3 | GPIO_PORTF | GPIO_PIN_3)
#  define GPIO_CAN0_TX_2    (GPIO_FUNC_PFOUTPUT | GPIO_ALT_8 | GPIO_PORTB | GPIO_PIN_5)
#  define GPIO_CAN0_TX_3    (GPIO_FUNC_PFOUTPUT | GPIO_ALT_8 | GPIO_PORTE | GPIO_PIN_5)

#  define GPIO_CMP0_NIN     (GPIO_FUNC_ANINPUT | GPIO_PORTC | GPIO_PIN_7)
#  define GPIO_CMP0_OUT     (GPIO_FUNC_PFOUTPUT | GPIO_ALT_9 | GPIO_PORTF | GPIO_PIN_0)
#  define GPIO_CMP0_PIN     (GPIO_FUNC_ANINPUT | GPIO_PORTC | GPIO_PIN_6)
#  define GPIO_CMP1_NIN     (GPIO_FUNC_ANINPUT | GPIO_PORTC | GPIO_PIN_4)
#  define GPIO_CMP1_OUT     (GPIO_FUNC_PFOUTPUT | GPIO_ALT_9 | GPIO_PORTF | GPIO_PIN_1)
#  define GPIO_CMP1_PIN     (GPIO_FUNC_ANINPUT | GPIO_PORTC | GPIO_PIN_5)

#  define GPIO_CORE_TRCLK   (GPIO_FUNC_PFOUTPUT | GPIO_ALT_14 | GPIO_PORTF | GPIO_PIN_3)
#  define GPIO_CORE_TRD0    (GPIO_FUNC_PFOUTPUT | GPIO_ALT_14 | GPIO_PORTF | GPIO_PIN_2)
#  define GPIO_CORE_TRD1    (GPIO_FUNC_PFOUTPUT | GPIO_ALT_14 | GPIO_PORTF | GPIO_PIN_1)

#  define GPIO_I2C0_SCL     (GPIO_FUNC_PFODIO | GPIO_ALT_3 | GPIO_PORTB | GPIO_PIN_2)
#  define GPIO_I2C0_SDA     (GPIO_FUNC_PFODIO | GPIO_ALT_3 | GPIO_PORTB | GPIO_PIN_3)
#  define GPIO_I2C1_SCL     (GPIO_FUNC_PFODIO | GPIO_ALT_3 | GPIO_PORTA | GPIO_PIN_6)
#  define GPIO_I2C1_SDA     (GPIO_FUNC_PFODIO | GPIO_ALT_3 | GPIO_PORTA | GPIO_PIN_7)
#  define GPIO_I2C2_SCL     (GPIO_FUNC_PFODIO | GPIO_ALT_3 | GPIO_PORTE | GPIO_PIN_4)
#  define GPIO_I2C2_SDA     (GPIO_FUNC_PFODIO | GPIO_ALT_3 | GPIO_PORTE | GPIO_PIN_5)
#  define GPIO_I2C3_SCL     (GPIO_FUNC_PFODIO | GPIO_ALT_3 | GPIO_PORTD | GPIO_PIN_0)
#  define GPIO_I2C3_SDA     (GPIO_FUNC_PFODIO | GPIO_ALT_3 | GPIO_PORTD | GPIO_PIN_1)

#  define GPIO_JTAG_SWCLK   (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTC | GPIO_PIN_0)
#  define GPIO_JTAG_SWDIO   (GPIO_FUNC_PFIO | GPIO_ALT_1 | GPIO_PORTC | GPIO_PIN_1)
#  define GPIO_JTAG_SWO     (GPIO_FUNC_PFOUTPUT | GPIO_ALT_1 | GPIO_PORTC | GPIO_PIN_3)
#  define GPIO_JTAG_TCK     (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTC | GPIO_PIN_0)
#  define GPIO_JTAG_TDI     (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTC | GPIO_PIN_2)
#  define GPIO_JTAG_TDO     (GPIO_FUNC_PFOUTPUT | GPIO_ALT_1 | GPIO_PORTC | GPIO_PIN_3)
#  define GPIO_JTAG_TMS     (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTC | GPIO_PIN_1)

#  define GPIO_SSI0_CLK     (GPIO_FUNC_PFIO | GPIO_ALT_2 | GPIO_PORTA | GPIO_PIN_2)
#  define GPIO_SSI0_FSS     (GPIO_FUNC_PFIO | GPIO_ALT_2 | GPIO_PORTA | GPIO_PIN_3)
#  define GPIO_SSI0_RX      (GPIO_FUNC_PFINPUT | GPIO_ALT_2 | GPIO_PORTA | GPIO_PIN_4)
#  define GPIO_SSI0_TX      (GPIO_FUNC_PFOUTPUT | GPIO_ALT_2 | GPIO_PORTA | GPIO_PIN_5)
#  define GPIO_SSI1_CLK_1   (GPIO_FUNC_PFIO | GPIO_ALT_2 | GPIO_PORTD | GPIO_PIN_0)
#  define GPIO_SSI1_CLK_2   (GPIO_FUNC_PFIO | GPIO_ALT_2 | GPIO_PORTF | GPIO_PIN_2)
#  define GPIO_SSI1_FSS_1   (GPIO_FUNC_PFIO | GPIO_ALT_2 | GPIO_PORTD | GPIO_PIN_1)
#  define GPIO_SSI1_FSS_2   (GPIO_FUNC_PFIO | GPIO_ALT_2 | GPIO_PORTF | GPIO_PIN_3)
#  define GPIO_SSI1_RX_1    (GPIO_FUNC_PFINPUT | GPIO_ALT_2 | GPIO_PORTD | GPIO_PIN_2)
#  define GPIO_SSI1_RX_2    (GPIO_FUNC_PFINPUT | GPIO_ALT_2 | GPIO_PORTF | GPIO_PIN_0)
#  define GPIO_SSI1_TX_1    (GPIO_FUNC_PFOUTPUT | GPIO_ALT_2 | GPIO_PORTD | GPIO_PIN_3)
#  define GPIO_SSI1_TX_2    (GPIO_FUNC_PFOUTPUT | GPIO_ALT_2 | GPIO_PORTF | GPIO_PIN_1)
#  define GPIO_SSI2_CLK     (GPIO_FUNC_PFIO | GPIO_ALT_2 | GPIO_PORTB | GPIO_PIN_4)
#  define GPIO_SSI2_FSS     (GPIO_FUNC_PFIO | GPIO_ALT_2 | GPIO_PORTB | GPIO_PIN_5)
#  define GPIO_SSI2_RX      (GPIO_FUNC_PFINPUT | GPIO_ALT_2 | GPIO_PORTB | GPIO_PIN_6)
#  define GPIO_SSI2_TX      (GPIO_FUNC_PFOUTPUT | GPIO_ALT_2 | GPIO_PORTB | GPIO_PIN_7)
#  define GPIO_SSI3_CLK     (GPIO_FUNC_PFIO | GPIO_ALT_1 | GPIO_PORTD | GPIO_PIN_0)
#  define GPIO_SSI3_FSS     (GPIO_FUNC_PFIO | GPIO_ALT_1 | GPIO_PORTD | GPIO_PIN_1)
#  define GPIO_SSI3_RX      (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTD | GPIO_PIN_2)
#  define GPIO_SSI3_TX      (GPIO_FUNC_PFOUTPUT | GPIO_ALT_1 | GPIO_PORTD | GPIO_PIN_3)

#  define GPIO_SYSCON_NMI_1 (GPIO_FUNC_PFINPUT | GPIO_ALT_8 | GPIO_PORTD | GPIO_PIN_7)
#  define GPIO_SYSCON_NMI_2 (GPIO_FUNC_PFINPUT | GPIO_ALT_8 | GPIO_PORTF | GPIO_PIN_0)

#  define GPIO_TIM0_CCP0_1  (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTB | GPIO_PIN_6)
#  define GPIO_TIM0_CCP0_2  (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTF | GPIO_PIN_0)
#  define GPIO_TIM0_CCP1_1  (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTB | GPIO_PIN_7)
#  define GPIO_TIM0_CCP1_2  (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTF | GPIO_PIN_1)
#  define GPIO_TIM1_CCP0_1  (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTB | GPIO_PIN_4)
#  define GPIO_TIM1_CCP0_2  (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTF | GPIO_PIN_2)
#  define GPIO_TIM1_CCP1_1  (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTB | GPIO_PIN_5)
#  define GPIO_TIM1_CCP1_2  (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTF | GPIO_PIN_3)
#  define GPIO_TIM2_CCP0_1  (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTB | GPIO_PIN_0)
#  define GPIO_TIM2_CCP0_2  (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTF | GPIO_PIN_4)
#  define GPIO_TIM2_CCP1    (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTB | GPIO_PIN_1)
#  define GPIO_TIM3_CCP0    (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTB | GPIO_PIN_2)
#  define GPIO_TIM3_CCP1    (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTB | GPIO_PIN_3)
#  define GPIO_TIM4_CCP0    (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTC | GPIO_PIN_0)
#  define GPIO_TIM4_CCP1    (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTC | GPIO_PIN_1)
#  define GPIO_TIM5_CCP0    (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTC | GPIO_PIN_2)
#  define GPIO_TIM5_CCP1    (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTC | GPIO_PIN_3)

#  define GPIO_UART0_RX     (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTA | GPIO_PIN_0)
#  define GPIO_UART0_TX     (GPIO_FUNC_PFOUTPUT | GPIO_ALT_1 | GPIO_PORTA | GPIO_PIN_1)
#  define GPIO_UART1_CTS_1  (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTF | GPIO_PIN_1)
#  define GPIO_UART1_CTS_2  (GPIO_FUNC_PFINPUT | GPIO_ALT_8 | GPIO_PORTC | GPIO_PIN_5)
#  define GPIO_UART1_RTS_1  (GPIO_FUNC_PFOUTPUT | GPIO_ALT_1 | GPIO_PORTF | GPIO_PIN_0)
#  define GPIO_UART1_RTS_2  (GPIO_FUNC_PFOUTPUT | GPIO_ALT_8 | GPIO_PORTC | GPIO_PIN_4)
#  define GPIO_UART1_RX_1   (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTB | GPIO_PIN_0)
#  define GPIO_UART1_RX_2   (GPIO_FUNC_PFINPUT | GPIO_ALT_2 | GPIO_PORTC | GPIO_PIN_4)
#  define GPIO_UART1_TX_1   (GPIO_FUNC_PFOUTPUT | GPIO_ALT_1 | GPIO_PORTB | GPIO_PIN_1)
#  define GPIO_UART1_TX_2   (GPIO_FUNC_PFOUTPUT | GPIO_ALT_2 | GPIO_PORTC | GPIO_PIN_5)
#  define GPIO_UART2_RX     (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTD | GPIO_PIN_6)
#  define GPIO_UART2_TX     (GPIO_FUNC_PFOUTPUT | GPIO_ALT_1 | GPIO_PORTD | GPIO_PIN_7)
#  define GPIO_UART3_RX     (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTC | GPIO_PIN_6)
#  define GPIO_UART3_TX     (GPIO_FUNC_PFOUTPUT | GPIO_ALT_1 | GPIO_PORTC | GPIO_PIN_7)
#  define GPIO_UART4_RX     (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTC | GPIO_PIN_4)
#  define GPIO_UART4_TX     (GPIO_FUNC_PFOUTPUT | GPIO_ALT_1 | GPIO_PORTC | GPIO_PIN_5)
#  define GPIO_UART5_RX     (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTE | GPIO_PIN_4)
#  define GPIO_UART5_TX     (GPIO_FUNC_PFOUTPUT | GPIO_ALT_1 | GPIO_PORTE | GPIO_PIN_5)
#  define GPIO_UART6_RX     (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTD | GPIO_PIN_4)
#  define GPIO_UART6_TX     (GPIO_FUNC_PFOUTPUT | GPIO_ALT_1 | GPIO_PORTD | GPIO_PIN_5)
#  define GPIO_UART7_RX     (GPIO_FUNC_PFINPUT | GPIO_ALT_1 | GPIO_PORTE | GPIO_PIN_0)
#  define GPIO_UART7_TX     (GPIO_FUNC_PFOUTPUT | GPIO_ALT_1 | GPIO_PORTE | GPIO_PIN_1)

#  define GPIO_USB0_DM      (GPIO_FUNC_ANIO | GPIO_PORTD | GPIO_PIN_4)
#  define GPIO_USB0_DP      (GPIO_FUNC_ANIO | GPIO_PORTD | GPIO_PIN_5)

#  define GPIO_WTIM0_CCP0   (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTC | GPIO_PIN_4)
#  define GPIO_WTIM0_CCP1   (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTC | GPIO_PIN_5)
#  define GPIO_WTIM1_CCP0   (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTC | GPIO_PIN_6)
#  define GPIO_WTIM1_CCP1   (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTC | GPIO_PIN_7)
#  define GPIO_WTIM2_CCP0   (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTD | GPIO_PIN_0)
#  define GPIO_WTIM2_CCP1   (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTD | GPIO_PIN_1)
#  define GPIO_WTIM3_CCP0   (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTD | GPIO_PIN_2)
#  define GPIO_WTIM3_CCP1   (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTD | GPIO_PIN_3)
#  define GPIO_WTIM4_CCP0   (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTD | GPIO_PIN_4)
#  define GPIO_WTIM4_CCP1   (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTD | GPIO_PIN_5)
#  define GPIO_WTIM5_CCP0   (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTD | GPIO_PIN_6)
#  define GPIO_WTIM5_CCP1   (GPIO_FUNC_PFIO | GPIO_ALT_7 | GPIO_PORTD | GPIO_PIN_7)

#else
#  error "Unknown Stellaris chip"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LM_CHIP_LM4F_PINMAP_H */
