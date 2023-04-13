/****************************************************************************
 * boards/arm/stm32/olimexino-stm32/include/board.h
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

#ifndef __BOARDS_ARM_STM32_OLIMEXINO_STM32_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_OLIMEXINO_STM32_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif
#include "stm32_rcc.h"
#include "stm32_sdio.h"
#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - 8 MHz RC factory-trimmed
 * LSI - 40 KHz RC (30-60KHz, uncalibrated)
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     8000000ul
#define STM32_LSI_FREQUENCY     40000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* PLL source is HSE/1,
 * PLL multipler is 9:
 * PLL frequency is 8MHz (XTAL) x 9 = 72MHz
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
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (72MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY

/* APB2 timers 1 and 8 will receive PCLK2. */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK/2 (36MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* APB1 timers 2-7 will be twice PCLK1  */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)

/* USB divider -- Divide PLL clock by 1.5 */

#define STM32_CFGR_USBPRE       0

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM3_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM4_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM5_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM6_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM7_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY

/* Buttons ******************************************************************/

#define BUTTON_BOOT0_BIT  (0)
#define BUTTON_BOOT0_MASK (1<<BUTTON_BOOT0_BIT)

/* Leds *********************************************************************/

/* LED index values for use with board_userled() */

#define BOARD_LED1                0
#define BOARD_LED_GREEN           BOARD_LED1
#define BOARD_LED2                1
#define BOARD_LED_YELLOW          BOARD_LED2
#define BOARD_NLEDS               2

/* LED bits for use with board_userled_all() */

#define BOARD_LED_GREEN_BIT     (1 << BOARD_LED_GREEN)
#define BOARD_LED_YELLOW_BIT    (1 << BOARD_LED_YELLOW)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is as follows:
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                         Green  Yellow
 *   ------------------------  --------------------------  ------ ------
 */

#define LED_STARTED          0 /* NuttX has been started   OFF    OFF    */
#define LED_HEAPALLOCATE     1 /* Heap has been allocated  OFF    OFF    */
#define LED_IRQSENABLED      2 /* Interrupts enabled       OFF    OFF    */
#define LED_STACKCREATED     3 /* Idle stack created       ON     OFF    */
#define LED_INIRQ            4 /* In an interrupt          N/C    ON     */
#define LED_SIGNAL           5 /* In a signal handler      N/C    ON     */
#define LED_ASSERTION        6 /* An assertion failed      N/C    ON     */
#define LED_PANIC            7 /* The system has crashed   N/C  Blinking */
#define LED_IDLE             8 /* MCU is is sleep mode    OFF    N/C    */

/* Thus if the Green is statically on, NuttX has successfully booted and is,
 * apparently, running normally.  If the YellowLED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 *
 * NOTE: That the Yellow is not used after completion of booting and may
 * be used by other board-specific logic.
 */

/* ADC1 */

#define GPIO_ADC123_IN0      GPIO_ADC123_IN0_0  /* PA0 */
#define GPIO_ADC123_IN1      GPIO_ADC123_IN1_0  /* PA1 */
#define GPIO_ADC123_IN2      GPIO_ADC123_IN2_0  /* PA2 */
#define GPIO_ADC123_IN3      GPIO_ADC123_IN3_0  /* PA3 */
#define GPIO_ADC123_IN10     GPIO_ADC123_IN10_0 /* PC0 */
#define GPIO_ADC123_IN11     GPIO_ADC123_IN11_0 /* PC1 */
#define GPIO_ADC123_IN12     GPIO_ADC123_IN12_0 /* PC2 */
#define GPIO_ADC123_IN13     GPIO_ADC123_IN13_0 /* PC3 */

/* ADC1 */

#define GPIO_ADC12_IN4       GPIO_ADC12_IN4_0  /* PA4 */
#define GPIO_ADC12_IN5       GPIO_ADC12_IN5_0  /* PA5 */
#define GPIO_ADC12_IN6       GPIO_ADC12_IN6_0  /* PA6 */
#define GPIO_ADC12_IN7       GPIO_ADC12_IN7_0  /* PA7 */
#define GPIO_ADC12_IN8       GPIO_ADC12_IN8_0  /* PB0 */
#define GPIO_ADC12_IN9       GPIO_ADC12_IN9_0  /* PB1 */
#define GPIO_ADC12_IN14      GPIO_ADC12_IN14_0 /* PC4 */
#define GPIO_ADC12_IN15      GPIO_ADC12_IN15_0 /* PC5 */

/* TIM1 */

#define GPIO_TIM1_ETR        GPIO_TIM1_ETR_0                                        /* PA12 */
#define GPIO_TIM1_CH1IN      GPIO_TIM1_CH1IN_0                                      /* PA8 */
#define GPIO_TIM1_CH1OUT     GPIO_ADJUST_MODE(GPIO_TIM1_CH1OUT_0, GPIO_MODE_50MHz)  /* PA8 */
#define GPIO_TIM1_CH2IN      GPIO_TIM1_CH2IN_0                                      /* PA9 */
#define GPIO_TIM1_CH2OUT     GPIO_ADJUST_MODE(GPIO_TIM1_CH2OUT_0, GPIO_MODE_50MHz)  /* PA9 */
#define GPIO_TIM1_CH3IN      GPIO_TIM1_CH3IN_0                                      /* PA10 */
#define GPIO_TIM1_CH3OUT     GPIO_ADJUST_MODE(GPIO_TIM1_CH3OUT_0, GPIO_MODE_50MHz)  /* PA10 */
#define GPIO_TIM1_CH4IN      GPIO_TIM1_CH4IN_0                                      /* PA11 */
#define GPIO_TIM1_CH4OUT     GPIO_ADJUST_MODE(GPIO_TIM1_CH4OUT_0, GPIO_MODE_50MHz)  /* PA11 */
#define GPIO_TIM1_BKIN       GPIO_TIM1_BKIN_0                                       /* PA6 */
#define GPIO_TIM1_CH1NOUT    GPIO_ADJUST_MODE(GPIO_TIM1_CH1NOUT_0, GPIO_MODE_50MHz) /* PA7 */
#define GPIO_TIM1_CH2NOUT    GPIO_ADJUST_MODE(GPIO_TIM1_CH2NOUT_0, GPIO_MODE_50MHz) /* PB0 */
#define GPIO_TIM1_CH3NOUT    GPIO_ADJUST_MODE(GPIO_TIM1_CH3NOUT_0, GPIO_MODE_50MHz) /* PB1 */

/* TIM3 */

#define GPIO_TIM3_CH1IN      GPIO_TIM3_CH1IN_0                                     /* PB4 */
#define GPIO_TIM3_CH1OUT     GPIO_ADJUST_MODE(GPIO_TIM3_CH1OUT_0, GPIO_MODE_50MHz) /* PB4 */
#define GPIO_TIM3_CH2IN      GPIO_TIM3_CH2IN_0                                     /* PB5 */
#define GPIO_TIM3_CH2OUT     GPIO_ADJUST_MODE(GPIO_TIM3_CH2OUT_0, GPIO_MODE_50MHz) /* PB5 */
#define GPIO_TIM3_CH3IN      GPIO_TIM3_CH3IN_0                                     /* PB0 */
#define GPIO_TIM3_CH3OUT     GPIO_ADJUST_MODE(GPIO_TIM3_CH3OUT_0, GPIO_MODE_50MHz) /* PB0 */
#define GPIO_TIM3_CH4IN      GPIO_TIM3_CH4IN_0                                     /* PB1 */
#define GPIO_TIM3_CH4OUT     GPIO_ADJUST_MODE(GPIO_TIM3_CH4OUT_0, GPIO_MODE_50MHz) /* PB1 */

/* USART1 */

#define GPIO_USART1_TX       GPIO_ADJUST_MODE(GPIO_USART1_TX_0, GPIO_MODE_50MHz) /* PA9 */
#define GPIO_USART1_RX       GPIO_USART1_RX_0                                    /* PA10 */

/* USART2 */

#define GPIO_USART2_CTS      GPIO_USART2_CTS_0                                    /* PA0 */
#define GPIO_USART2_RTS      GPIO_ADJUST_MODE(GPIO_USART2_RTS_0, GPIO_MODE_50MHz) /* PA1 */
#define GPIO_USART2_TX       GPIO_ADJUST_MODE(GPIO_USART2_TX_0, GPIO_MODE_50MHz)  /* PA2 */
#define GPIO_USART2_RX       GPIO_USART2_RX_0                                     /* PA3 */
#define GPIO_USART2_CK       GPIO_ADJUST_MODE(GPIO_USART2_CK_0, GPIO_MODE_50MHz)  /* PA4 */

/* SPI1 */

#define GPIO_SPI1_NSS        GPIO_ADJUST_MODE(GPIO_SPI1_NSS_0, GPIO_MODE_50MHz)  /* PA4 */
#define GPIO_SPI1_SCK        GPIO_ADJUST_MODE(GPIO_SPI1_SCK_0, GPIO_MODE_50MHz)  /* PA5 */
#define GPIO_SPI1_MISO       GPIO_ADJUST_MODE(GPIO_SPI1_MISO_0, GPIO_MODE_50MHz) /* PA6 */
#define GPIO_SPI1_MOSI       GPIO_ADJUST_MODE(GPIO_SPI1_MOSI_0, GPIO_MODE_50MHz) /* PA7 */

/* SPI2 */

#define GPIO_SPI2_NSS        GPIO_ADJUST_MODE(GPIO_SPI2_NSS_0, GPIO_MODE_50MHz)  /* PB12 */
#define GPIO_SPI2_SCK        GPIO_ADJUST_MODE(GPIO_SPI2_SCK_0, GPIO_MODE_50MHz)  /* PB13 */
#define GPIO_SPI2_MISO       GPIO_ADJUST_MODE(GPIO_SPI2_MISO_0, GPIO_MODE_50MHz) /* PB14 */
#define GPIO_SPI2_MOSI       GPIO_ADJUST_MODE(GPIO_SPI2_MOSI_0, GPIO_MODE_50MHz) /* PB15 */

/* I2C2 */

#define GPIO_I2C2_SCL        GPIO_ADJUST_MODE(GPIO_I2C2_SCL_0, GPIO_MODE_50MHz)  /* PB10 */
#define GPIO_I2C2_SDA        GPIO_ADJUST_MODE(GPIO_I2C2_SDA_0, GPIO_MODE_50MHz)  /* PB11 */
#define GPIO_I2C2_SMBA       GPIO_ADJUST_MODE(GPIO_I2C2_SMBA_0, GPIO_MODE_50MHz) /* PB12 */

/* CAN1 */

#define GPIO_CAN1_RX         GPIO_CAN1_RX_0                                    /* PB8 */
#define GPIO_CAN1_TX         GPIO_ADJUST_MODE(GPIO_CAN1_TX_0, GPIO_MODE_50MHz) /* PB9 */

#endif /* __BOARDS_ARM_STM32_OLIMEXINO_STM32_INCLUDE_BOARD_H */
