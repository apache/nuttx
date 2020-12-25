/****************************************************************************
 * boards/arm/stm32/nucleo-f334r8/include/board.h
 * include/arch/board/board.h
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

#ifndef __BOARDS_ARM_STM32_NUCLEO_F334R8_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_NUCLEO_F334R8_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#ifdef __KERNEL__
#  include "stm32.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - Internal 8 MHz RC Oscillator
 * LSI - 32 KHz RC
 * HSE - 8 MHz from MCO output of ST-LINK
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     8000000ul
#define STM32_LSI_FREQUENCY     32000            /* Between 30kHz and 60kHz */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768            /* X2 on board */

/* PLL source is HSE/1, PLL multipler is 9: PLL frequency is
 * 8MHz (XTAL) x 9 = 72MHz
 */

#define STM32_CFGR_PLLSRC       RCC_CFGR_PLLSRC
#define STM32_CFGR_PLLXTPRE     0
#define STM32_CFGR_PLLMUL       RCC_CFGR_PLLMUL_CLKx9
#define STM32_PLL_FREQUENCY     (9*STM32_BOARD_XTAL)

/* Use the PLL and set the SYSCLK source to be the PLL */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (72MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (72MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN        (STM32_PCLK2_FREQUENCY)

/* APB2 timers 1, 8, 15-17 and HRTIM1 will receive PCLK2. */

/* Timers driven from APB2 will be PCLK2 */

#define STM32_APB2_TIM1_CLKIN     (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN     (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM15_CLKIN    (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM16_CLKIN    (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM17_CLKIN    (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_THRTIM1_CLKIN  (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK/2 (36MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* APB1 timers 2-7 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM15_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM16_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM17_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM3_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM5_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM6_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM7_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_HRTIM1_FREQUENCY  STM32_HCLK_FREQUENCY

/* LED definitions **********************************************************/

/* The Nucleo F334R8 board has three LEDs.  Two of these are controlled by
 * logic on the board and are not available for software control:
 *
 * LD1 COM:  LD1 default status is red.  LD1 turns to green to indicate that
 *           communications are in progress between the PC and the
 *           ST-LINK/V2-1.
 * LD3 PWR:  red LED indicates that the board is powered.
 *
 * And one can be controlled by software:
 *
 * User LD2: green LED is a user LED connected to the I/O PA5 of the
 *           STM32F334R8.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED in
 * any way.  The following definition is used to access the LED.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1       0 /* User LD2 */
#define BOARD_NLEDS      1

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT   (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the Nucleo F334R8.  The following definitions describe how NuttX controls
 * the LED:
 *
 *   SYMBOL              Meaning                  LED1 state
 *   ------------------  -----------------------  ----------
 *   LED_STARTED         NuttX has been started   OFF
 *   LED_HEAPALLOCATE    Heap has been allocated  OFF
 *   LED_IRQSENABLED     Interrupts enabled       OFF
 *   LED_STACKCREATED    Idle stack created       ON
 *   LED_INIRQ           In an interrupt          No change
 *   LED_SIGNAL          In a signal handler      No change
 *   LED_ASSERTION       An assertion failed      No change
 *   LED_PANIC           The system has crashed   Blinking
 *   LED_IDLE            STM32 is is sleep mode   Not used
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        2
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Button definitions *******************************************************/

/* The Nucleo F334R8 supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:  user button connected to the I/O PC13 of the STM32F334R8.
 *   B2 RESET: push button connected to NRST is used to RESET the
 *             STM32F334R8.
 */

#define BUTTON_USER      0
#define NUM_BUTTONS      1

#define BUTTON_USER_BIT  (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* CAN */

#define GPIO_CAN1_RX GPIO_CAN_RX_2
#define GPIO_CAN1_TX GPIO_CAN_TX_2

/* I2C */

#define GPIO_I2C1_SCL GPIO_I2C1_SCL_3
#define GPIO_I2C1_SDA GPIO_I2C1_SDA_3

/* SPI */

#define GPIO_SPI1_MISO GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK GPIO_SPI1_SCK_1

/* TIM */

#define GPIO_TIM2_CH2OUT GPIO_TIM2_CH2OUT_2
#define GPIO_TIM2_CH3OUT GPIO_TIM2_CH3OUT_3

#define GPIO_TIM3_CH1OUT GPIO_TIM3_CH1OUT_2
#define GPIO_TIM3_CH2OUT GPIO_TIM3_CH2OUT_4

#define GPIO_TIM4_CH1OUT GPIO_TIM4_CH1OUT_2

/* USART */

/* By default the USART2 is connected to STLINK Virtual COM Port:
 * USART2_RX - PA3
 * USART2_TX - PA2
 */

#define GPIO_USART2_RX GPIO_USART2_RX_1 /* PA3 */
#define GPIO_USART2_TX GPIO_USART2_TX_1 /* PA2 */

#define GPIO_USART1_RX GPIO_USART1_RX_1 /* PA10 */
#define GPIO_USART1_TX GPIO_USART1_TX_1 /* PA9 */

/* COMP */

/* OPAMP */

#define OPAMP2_VMSEL OPAMP2_VMSEL_PC5
#define OPAMP2_VPSEL OPAMP2_VPSEL_PB14

/* Configuration specific to high priority interrupts example:
 *   - HRTIM Timer A trigger for ADC if DMA transfer and HRTIM
 *   - TIM1 CC1 trigger for ADC if DMA transfer and TIM1 PWM
 *   - ADC DMA transfer on DMA1_CH1
 */

#ifdef CONFIG_NUCLEOF334R8_HIGHPRI

#if defined(CONFIG_STM32_HRTIM1) && defined(CONFIG_STM32_ADC1_DMA)

/* HRTIM - ADC trigger */

#define HRTIM_TIMA_PRESCALER HRTIM_PRESCALER_128
#define HRTIM_TIMA_MODE      HRTIM_MODE_CONT
#define HRTIM_TIMA_UPDATE    0
#define HRTIM_TIMA_RESET     0

#define HRTIM_ADC_TRG1       HRTIM_ADCTRG13_APER

#endif /* CONFIG_STM32_HRTIM1 && CONFIG_STM32_ADC1_DMA*/
#endif /* CONFIG_NUCLEOF334R8_HIGHPRI */

#ifdef CONFIG_NUCLEOF334R8_SPWM
#  ifdef CONFIG_NUCLEOF334R8_SPWM_USE_TIM1

/* TIM1 PWM configuration ***************************************************/

#    define GPIO_TIM1_CH1OUT   GPIO_TIM1_CH1OUT_1 /* TIM1 CH1  - PA8 */
#    define GPIO_TIM1_CH1NOUT  GPIO_TIM1_CH1N_3   /* TIM1 CH1N - PA7 */
                                                  /* TIM1 CH2  - PA9 */
#    define GPIO_TIM1_CH2NOUT  GPIO_TIM1_CH2N_2   /* TIM1 CH2N - PB0 */
#    define GPIO_TIM1_CH3OUT   GPIO_TIM1_CH3OUT_1 /* TIM1 CH3  - PA10 */
#    define GPIO_TIM1_CH3NOUT  GPIO_TIM1_CH3N_2   /* TIM1 CH3N - PB1 */
#    define GPIO_TIM1_CH4OUT   GPIO_TIM1_CH4OUT_1 /* TIM1 CH4  - PA11 */
#  endif

#  ifdef CONFIG_NUCLEOF334R8_SPWM_USE_HRTIM1

/* HRTIM configuration ******************************************************/

#    define HRTIM_MASTER_PRESCALER HRTIM_PRESCALER_128
#    define HRTIM_MASTER_MODE      HRTIM_MODE_CONT

#    define HRTIM_TIMA_PRESCALER HRTIM_PRESCALER_128
#    define HRTIM_TIMA_MODE      (HRTIM_MODE_CONT | HRTIM_MODE_PRELOAD)
#    define HRTIM_TIMA_CH1_SET   HRTIM_OUT_SET_PER
#    define HRTIM_TIMA_CH1_RST   HRTIM_OUT_RST_CMP1
#    define HRTIM_TIMA_UPDATE    HRTIM_UPDATE_MSTU
#    define HRTIM_TIMA_RESET     0

#    define HRTIM_TIMB_PRESCALER HRTIM_PRESCALER_128
#    define HRTIM_TIMB_MODE      (HRTIM_MODE_CONT | HRTIM_MODE_PRELOAD)
#    define HRTIM_TIMB_CH1_SET   HRTIM_OUT_SET_PER
#    define HRTIM_TIMB_CH1_RST   HRTIM_OUT_RST_CMP1
#    define HRTIM_TIMB_UPDATE    HRTIM_UPDATE_MSTU
#    define HRTIM_TIMB_RESET     0

#    define HRTIM_TIMC_PRESCALER HRTIM_PRESCALER_128
#    define HRTIM_TIMC_MODE      (HRTIM_MODE_CONT | HRTIM_MODE_PRELOAD)
#    define HRTIM_TIMC_CH1_SET   HRTIM_OUT_SET_PER
#    define HRTIM_TIMC_CH1_RST   HRTIM_OUT_RST_CMP1
#    define HRTIM_TIMC_UPDATE    HRTIM_UPDATE_MSTU
#    define HRTIM_TIMC_RESET     0

#    define HRTIM_TIMD_PRESCALER HRTIM_PRESCALER_128
#    define HRTIM_TIMD_MODE      (HRTIM_MODE_CONT | HRTIM_MODE_PRELOAD)
#    define HRTIM_TIMD_CH1_SET   HRTIM_OUT_SET_PER
#    define HRTIM_TIMD_CH1_RST   HRTIM_OUT_RST_CMP1
#    define HRTIM_TIMD_UPDATE    HRTIM_UPDATE_MSTU
#    define HRTIM_TIMD_RESET     0

#    define HRTIM_TIME_PRESCALER HRTIM_PRESCALER_128
#    define HRTIM_TIME_MODE      (HRTIM_MODE_CONT | HRTIM_MODE_PRELOAD)
#    define HRTIM_TIME_CH1_SET   HRTIM_OUT_SET_PER
#    define HRTIM_TIME_CH1_RST   HRTIM_OUT_RST_CMP1
#    define HRTIM_TIME_UPDATE    HRTIM_UPDATE_MSTU
#    define HRTIM_TIME_RESET     0

#    define HRTIM_MASTER_IRQ     HRTIM_IRQ_MCMP1
#  endif

#endif /* CONFIG_NUCLEOF334R8_SPWM */

/* DMA channels *************************************************************/

/* ADC */

#define ADC1_DMA_CHAN DMACHAN_ADC1     /* DMA1_CH1 */

#endif /* __BOARDS_ARM_STM32_NUCLEO_F334R8_INCLUDE_BOARD_H */
