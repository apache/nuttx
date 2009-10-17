/************************************************************************************
 * arch/arm/src/stm32/stm32_internal.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_INTERNAL_H
#define __ARCH_ARM_SRC_STM32_STM32_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "up_internal.h"
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* NVIC priority levels */

#define NVIC_SYSH_PRIORITY_MIN     0xff /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */

/* Bit-encoded input to stm32_configgpio() *******************************************/

/* Encoding:
 * .... .... .... .... OFFS S... VPPP BBBB
 */

/* Output mode:
 *
 * .... .... .... .... O... .... VPPP BBBB
 */

#define GPIO_INPUT                    (1 << 15)                  /* Bit 15: 1=Input mode */
#define GPIO_OUTPUT                   (0)                        /*         0=Output or alternate function */
#define GPIO_ALT                      (0)

/* These bits set the primary function of the pin:
 * .... .... .... .... .FF. .... .... ....
 */

#define GPIO_CNF_SHIFT                13                         /* Bits 13-14: GPIO function */
#define GPIO_CNF_MASK                 (3 << GPIO_CNF_SHIFT)

#  define GPIO_CNF_ANALOGIN           (0 << GPIO_CNF_SHIFT)      /* Analog input */
#  define GPIO_CNF_INFLOAT            (1 << GPIO_CNF_SHIFT)      /* Input floating */
#  define GPIO_CNF_INPULLUP           (2 << GPIO_CNF_SHIFT)      /* Input pull-up */
#  define GPIO_CNF_INPULLDWN          (3 << GPIO_CNF_SHIFT)      /* Input pull-down */

#  define GPIO_CNF_OUTPP              (0 << GPIO_CNF_SHIFT)      /* Output push-pull */
#  define GPIO_CNF_OUTOD              (1 << GPIO_CNF_SHIFT)      /* Output open-drain */
#  define GPIO_CNF_AFPP               (2 << GPIO_CNF_SHIFT)      /* Alternate function push-pull */
#  define GPIO_CNF_AFOD               (3 << GPIO_CNF_SHIFT)      /* Alternate function open-drain */

/* Maximum frequency selection:
 * .... .... .... .... ...S S... .... ....
 */

#define GPIO_MODE_SHIFT               11                         /* Bits 11-12: GPIO frequency selection */
#define GPIO_MODE_MASK                (3 << GPIO_MODE_SHIFT)
#  define GPIO_MODE_INPUT             (0 << GPIO_MODE_SHIFT)     /* Input mode (reset state) */
#  define GPIO_MODE_10MHz             (1 << GPIO_MODE_SHIFT)     /* Output mode, max speed 10 MHz */
#  define GPIO_MODE_2MHz              (2 << GPIO_MODE_SHIFT)     /* Output mode, max speed 2 MHz */
#  define GPIO_MODE_50MHz             (3 << GPIO_MODE_SHIFT)     /* Output mode, max speed 50 MHz */

/* If the pin is an GPIO digital output, then this identifies the initial output value:
 * .... .... .... .... .... .... V... ....
 */

#define GPIO_OUTPUT_SET               (1 << 7)                   /* Bit 7: If output, inital value of output */
#define GPIO_OUTPUT_CLEAR             (0) 

/* This identifies the GPIO port:
 * .... .... .... .... .... .... .PPP ....
 */

#define GPIO_PORT_SHIFT               4                          /* Bit 4-6:  Port number */
#define GPIO_PORT_MASK                (7 << GPIO_PORT_SHIFT)
#define GPIO_PORTA                    (0 << GPIO_PORT_SHIFT)     /*   GPIOA */
#define GPIO_PORTB                    (1 << GPIO_PORT_SHIFT)     /*   GPIOB */
#define GPIO_PORTC                    (2 << GPIO_PORT_SHIFT)     /*   GPIOC */
#define GPIO_PORTD                    (3 << GPIO_PORT_SHIFT)     /*   GPIOD */
#define GPIO_PORTE                    (4 << GPIO_PORT_SHIFT)     /*   GPIOE */
#define GPIO_PORTF                    (5 << GPIO_PORT_SHIFT)     /*   GPIOF */
#define GPIO_PORTG                    (6 << GPIO_PORT_SHIFT)     /*   GPIOG */

/* This identifies the bit in the port:
 * .... .... .... .... .... .... .... BBBB
 */

#define GPIO_PIN_SHIFT                 0                           /* Bits 0-3: GPIO number: 0-15 */
#define GPIO_PIN_MASK                  (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN1                      (1 << GPIO_PIN_SHIFT)
#define GPIO_PIN2                      (2 << GPIO_PIN_SHIFT)
#define GPIO_PIN3                      (3 << GPIO_PIN_SHIFT)
#define GPIO_PIN4                      (4 << GPIO_PIN_SHIFT)
#define GPIO_PIN5                      (5 << GPIO_PIN_SHIFT)
#define GPIO_PIN6                      (6 << GPIO_PIN_SHIFT)
#define GPIO_PIN7                      (7 << GPIO_PIN_SHIFT)
#define GPIO_PIN8                      (8 << GPIO_PIN_SHIFT)
#define GPIO_PIN9                      (9 << GPIO_PIN_SHIFT)
#define GPIO_PIN10                     (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11                     (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12                     (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13                     (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14                     (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15                     (15 << GPIO_PIN_SHIFT)

/* Alternate Pin Functions: */

#if defined(CONFIG_STM32_TIM1_FULL_REMAP)
#  define GPIO_TIM1_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN7)
#  define GPIO_TIM1_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN9)
#  define GPIO_TIM1_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN9)
#  define GPIO_TIM1_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN11)
#  define GPIO_TIM1_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN11)
#  define GPIO_TIM1_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN13)
#  define GPIO_TIM1_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN13)
#  define GPIO_TIM1_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN14)
#  define GPIO_TIM1_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN14)
#  define GPIO_TIM1_BKIN    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN15)
#  define GPIO_TIM1_CH1N    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN8)
#  define GPIO_TIM1_CH2N    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN10)
#  define GPIO_TIM1_CH3N    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN12)
#elif defined(CONFIG_STM32_TIM1_PARTIAL_REMAP)
#  define GPIO_TIM1_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN12)
#  define GPIO_TIM1_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN8)
#  define GPIO_TIM1_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN8)
#  define GPIO_TIM1_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_TIM1_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_TIM1_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN10)
#  define GPIO_TIM1_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN10)
#  define GPIO_TIM1_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_TIM1_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_TIM1_BKIN    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN6)
#  define GPIO_TIM1_CH1N    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN7)
#  define GPIO_TIM1_CH2N    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM1_CH3N    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN1)
#else
#  define GPIO_TIM1_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN12)
#  define GPIO_TIM1_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN8)
#  define GPIO_TIM1_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN8)
#  define GPIO_TIM1_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_TIM1_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_TIM1_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN10)
#  define GPIO_TIM1_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN10)
#  define GPIO_TIM1_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_TIM1_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_TIM1_BKIN    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN12)
#  define GPIO_TIM1_CH1N    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN13)
#  define GPIO_TIM1_CH2N    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN14)
#  define GPIO_TIM1_CH3N    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN15)
#endif

#if defined(CONFIG_STM32_TIM2_FULL_REMAP)
#  define GPIO_TIM2_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_TIM2_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_TIM2_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_TIM2_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_TIM2_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#  define GPIO_TIM2_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN11)
#elif defined(CONFIG_STM32_TIM2_PARTIAL_REMAP_1)
#  define GPIO_TIM2_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_TIM2_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_TIM2_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM2_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM2_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#  define GPIO_TIM2_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN3)
#elif defined(CONFIG_STM32_TIM2_PARTIAL_REMAP_2)
#  define GPIO_TIM2_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_TIM2_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_TIM2_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_TIM2_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_TIM2_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#  define GPIO_TIM2_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN11)
#else
#  define GPIO_TIM2_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_TIM2_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_TIM2_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM2_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM2_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#  define GPIO_TIM2_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN3)
#endif

#if defined(CONFIG_STM32_TIM3_FULL_REMAP)
#  define GPIO_TIM3_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN6)
#  define GPIO_TIM3_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN6)
#  define GPIO_TIM3_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN7)
#  define GPIO_TIM3_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN7)
#  define GPIO_TIM3_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN8)
#  define GPIO_TIM3_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN8)
#  define GPIO_TIM3_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN9)
#  define GPIO_TIM3_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN9)
#elif defined(CONFIG_STM32_TIM3_PARTIAL_REMAP)
#  define GPIO_TIM3_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN4)
#  define GPIO_TIM3_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN4)
#  define GPIO_TIM3_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN5)
#  define GPIO_TIM3_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN5)
#  define GPIO_TIM3_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM3_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM3_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN1)
#  define GPIO_TIM3_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN1)
#else
#  define GPIO_TIM3_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN6)
#  define GPIO_TIM3_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN6)
#  define GPIO_TIM3_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN7)
#  define GPIO_TIM3_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN7)
#  define GPIO_TIM3_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM3_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM3_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN1)
#  define GPIO_TIM3_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN1)
#endif

#if defined(CONFIG_STM32_TIM4_REMAP)
#  define GPIO_TIM4_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN12)
#  define GPIO_TIM4_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN12)
#  define GPIO_TIM4_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN13)
#  define GPIO_TIM4_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN13)
#  define GPIO_TIM4_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN14)
#  define GPIO_TIM4_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN14)
#  define GPIO_TIM4_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN15)
#  define GPIO_TIM4_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN15)
#else
#  define GPIO_TIM4_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN6)
#  define GPIO_TIM4_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN6)
#  define GPIO_TIM4_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN7)
#  define GPIO_TIM4_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN7)
#  define GPIO_TIM4_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_TIM4_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_TIM4_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN9)
#  define GPIO_TIM4_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN9)
#endif

#define GPIO_TIM5_CH4IN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#define GPIO_TIM5_CH4OUT    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN3)

#if defined(CONFIG_STM32_USART1_REMAP)
#  define GPIO_USART1_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_USART1_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN10)
#else
#  define GPIO_USART1_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_USART1_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN10)
#endif

#if defined(CONFIG_STM32_USART2_REMAP)
#  define GPIO_USART2_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN3)
#  define GPIO_USART2_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN4)
#  define GPIO_USART2_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN5)
#  define GPIO_USART2_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN6)
#  define GPIO_USART2_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN7)
#else
#  define GPIO_USART2_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_USART2_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_USART2_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_USART2_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#  define GPIO_USART2_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN4)
#endif

#if defined(CONFIG_STM32_USART3_FULL_REMAP)
#  define GPIO_USART3_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN8)
#  define GPIO_USART3_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN9)
#  define GPIO_USART3_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN10)
#  define GPIO_USART3_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN11)
#  define GPIO_USART3_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN12)
#elif defined(CONFIG_STM32_USART3_PARTIAL_REMAP)
#  define GPIO_USART3_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN10)
#  define GPIO_USART3_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN11)
#  define GPIO_USART3_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN12)
#  define GPIO_USART3_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN13)
#  define GPIO_USART3_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN14)
#else
#  define GPIO_USART3_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_USART3_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#  define GPIO_USART3_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN12)
#  define GPIO_USART3_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN13)
#  define GPIO_USART3_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN14)
#endif

#if defined(CONFIG_STM32_SPI1_REMAP)
#  define GPIO_SPI1_NSS     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_SPI1_SCK     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_SPI1_MISO    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN4)
#  define GPIO_SPI1_MOSI    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN5)
#else
#  define GPIO_SPI1_NSS     (GPIO_INPUT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN4)
#  define GPIO_SPI1_SCK     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN5)
#  define GPIO_SPI1_MISO    (GPIO_INPUT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN6)
#  define GPIO_SPI1_MOSI    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN7)
#endif

#if defined(CONFIG_STM32_SPI3_REMAP)
#  define GPIO_SPI3_NSS     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN4)
#  define GPIO_SPI3_SCK     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN10)
#  define GPIO_SPI3_MISO    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN11)
#  define GPIO_SPI3_MOSI    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN12)
#else
#  define GPIO_SPI3_NSS     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_SPI3_SCK     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_SPI3_MISO    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN4)
#  define GPIO_SPI3_MOSI    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN5)
#endif

#if defined(CONFIG_STM32_I2C1_REMAP)
#  define GPIO_I2C1_SCL     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_I2C1_SDA     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN9)
#else
#  define GPIO_I2C1_SCL     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN6)
#  define GPIO_I2C1_SDA     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN7)
#endif

#if defined(CONFIG_STM32_CAN1_FULL_REMAP)
#  define GPIO_CAN1_TX      (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN0)
#  define GPIO_CAN1_RX      (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN1)
#elif defined(CONFIG_STM32_CAN1_PARTIAL_REMAP)
#  define GPIO_CAN1_TX      (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_CAN1_RX      (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN9)
#else
#  define GPIO_CAN1_TX      (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_CAN1_RX      (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN12)
#endif

#if defined(CONFIG_STM32_CAN2_REMAP)
#  define GPIO_CAN2_TX      (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN5)
#  define GPIO_CAN2_RX      (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN6)
#else
#  define GPIO_CAN2_TX      (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN12)
#  define GPIO_CAN2_RX      (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN13)
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, ubyte isr, void *arg);

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* This symbol references the Cortex-M3 vector table (as positioned by the the linker
 * script, ld.script or ld.script.dfu.  The standard location for the vector table is
 * at the beginning of FLASH at address 0x0800:0000.  If we are using the STMicro DFU
 * bootloader, then the vector table will be offset to a different location in FLASH
 * and we will need to set the NVIC vector location to this alternative location.
 */

extern uint32 stm32_vectors[];	/* See stm32_vectors.S */

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization.
 *
 ************************************************************************************/

EXTERN void stm32_lowsetup(void);

/************************************************************************************
 * Name: stm32_clockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 ************************************************************************************/

EXTERN void stm32_clockconfig(void);

/************************************************************************************
 * Name: stm32_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

EXTERN int stm32_configgpio(uint32 cfgset);

/************************************************************************************
 * Name: stm32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

EXTERN void stm32_gpiowrite(uint32 pinset, boolean value);

/************************************************************************************
 * Name: stm32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

EXTERN boolean stm32_gpioread(uint32 pinset);

/************************************************************************************
 * Function:  stm32_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG
EXTERN int stm32_dumpgpio(uint32 pinset, const char *msg);
#else
#  define stm32_dumpgpio(p,m)
#endif

/****************************************************************************
 * Name: stm32_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

EXTERN void weak_function stm32_dmainitialize(void);

/****************************************************************************
 * Name: stm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'chan' argument.
 *   DMA channels are shared on the STM32:  Devices sharing the same DMA
 *   channel cannot do DMA concurrently!  See the DMACHAN_* definitions in
 *   stm32_dma.h.
 *
 *   If the DMA channel is not available, then stm32_dmachannel() will wait
 *   until the holder of the channel relinquishes the channel by calling
 *   stm32_dmafree().  WARNING: If you have two devices sharing a DMA
 *   channel and the code never releases the channel, the stm32_dmachannel
 *   call for the other will hang forever in this function!  Don't let your
 *   design do that!
 *
 *   Hmm.. I suppose this interface could be extended to make a non-blocking
 *   version.  Feel free to do that if that is what you need.
 *
 * Returned Value:
 *   Provided that 'chan' is valid, this function ALWAYS returns a non-NULL,
 *   void* DMA channel handle.  (If 'chan' is invalid, the function will
 *   assert if debug is enabled or do something ignorant otherwise).
 *
 * Assumptions:
 *   - The caller does not hold he DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is no
 *     available.
 *
 ****************************************************************************/

EXTERN DMA_HANDLE stm32_dmachannel(int chan);

/****************************************************************************
 * Name: stm32_dmafree
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA channel
 *   in a call to stm32_dmachannel, then this function will re-assign the
 *   DMA channel to that thread and wake it up.  NOTE:  The 'handle' used
 *   in this argument must NEVER be used again until stm32_dmachannel() is
 *   called again to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

EXTERN void stm32_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

EXTERN void stm32_dmasetup(DMA_HANDLE handle, uint32 paddr, uint32 maddr,
                           size_t ntransfers, uint32 ccr);

/****************************************************************************
 * Name: stm32_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

EXTERN void stm32_dmastart(DMA_HANDLE handle, dma_callback_t callback,
                           void *arg, boolean half);

/************************************************************************************
 * Function: stm32_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.  If the STM32 chip
 *   supports multiple Ethernet controllers, then bould specific logic
 *   must implement up_netinitialize() and call this function to initialize
 *   the desiresed interfaces.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ************************************************************************************/

#if STM32_NTHERNET > 1
EXTERN int stm32_ethinitialize(int intf);
#endif

/************************************************************************************
 * Name:  stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi.h). All other methods (including up_spiinitialize())
 *   are provided by common STM32 logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *      The select() methods must call stm32_spitake() when the chip is selected
 *      and stm32_spigive() when the chip is deselected.  This assures mutually
 *      exclusive access to the SPI for the duration while a chip is selected.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

struct spi_dev_s;
enum spi_dev_e;
EXTERN void  stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, boolean selected);
EXTERN ubyte stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
EXTERN void  stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, boolean selected);
EXTERN ubyte stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
EXTERN void  stm32_spi3select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, boolean selected);
EXTERN ubyte stm32_spi3status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);

/************************************************************************************
 * Name: stm32_spitake() and stm32_spigive()
 *
 * Description:
 *   The stm32_spi1/2/3select() and stm32_spi1/2/3status() methods must call
 *   stm32_spitake() when the chip is selected and stm32_spigive() when the chip is
 *   deselected.  This assures mutually exclusive access to the SPI for the duration
 *   while a chip is selected.
 *
 ************************************************************************************/

EXTERN void stm32_spitake(FAR struct spi_dev_s *dev);
EXTERN void stm32_spigive(FAR struct spi_dev_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_INTERNAL_H */
