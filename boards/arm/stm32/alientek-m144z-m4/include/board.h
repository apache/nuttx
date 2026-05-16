/****************************************************************************
 * boards/arm/stm32/alientek-m144z-m4/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM_STM32_ALIENTEK_M144Z_M4_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_ALIENTEK_M144Z_M4_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* ALIENTEK M144Z-M4 (STM32F407 minimum system board) features:
 *   - HSE : 8 MHz crystal (X1)
 *   - LSE : 32.768 kHz crystal (X2, for RTC)
 *
 * Canonical configuration:
 *   System Clock source     : PLL (HSE)
 *   SYSCLK(Hz)              : 168000000   (max for STM32F407)
 *   HCLK(Hz)                : 168000000
 *   AHB Prescaler           : 1
 *   APB1 Prescaler          : 4           (PCLK1 = 42 MHz, max 42 MHz)
 *   APB2 Prescaler          : 2           (PCLK2 = 84 MHz, max 84 MHz)
 *   HSE Frequency(Hz)       : 8000000
 *   PLLM                    : 4           (VCO input = HSE/PLLM = 2 MHz)
 *   PLLN                    : 168         (VCO out = 2 * 168 = 336 MHz)
 *   PLLP                    : 2           (SYSCLK = 336/2 = 168 MHz)
 *   PLLQ                    : 7           (48 MHz USB OTG FS / SDIO)
 *   Main regulator voltage  : Scale1 (required for >144 MHz)
 *   Flash Latency(WS)       : 5
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE (8 MHz crystal X1).
 *
 *   PLL_VCO  = (HSE / PLLM) * PLLN = (8 MHz / 4) * 168 = 336 MHz
 *   SYSCLK   = PLL_VCO / PLLP      = 336 / 2  = 168 MHz
 *   USB/SDIO = PLL_VCO / PLLQ      = 336 / 7  =  48 MHz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(4)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(168)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  168000000ul

/* AHB clock (HCLK) is SYSCLK (168MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2  */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same as APBx
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

/* LED definitions **********************************************************/

/* The ALIENTEK M144Z-M4 board provides two user LEDs:
 *   LED0 (red)   - PF9  (active LOW: write 0 to turn ON)
 *   LED1 (green) - PF10 (active LOW: write 0 to turn ON)
 *
 * Note: board_userled() takes a logical "on=true" argument; the board
 *       support code inverts the value before driving the pin.
 */

#define BOARD_LED0        0
#define BOARD_LED1        1
#define BOARD_NLEDS       2

/* For backward compatibility with NuttX board_autoled_* generic API. */

#define BOARD_LED_STATUS  BOARD_LED0

#define BOARD_LED0_BIT    (1 << BOARD_LED0)
#define BOARD_LED1_BIT    (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDS is defined, NuttX uses LED0 (red) as a status LED.
 * Encoding of board_autoled_on/off events (see stm32_autoleds.c):
 */

#define LED_STARTED       0  /* LED0 on  */
#define LED_HEAPALLOCATE  1  /* no change */
#define LED_IRQSENABLED   2  /* no change */
#define LED_STACKCREATED  3  /* no change */
#define LED_INIRQ         4  /* no change */
#define LED_SIGNAL        5  /* no change */
#define LED_ASSERTION     6  /* LED0 off */
#define LED_PANIC         7  /* LED0 blinking */

/* Button definitions *******************************************************/

/* The board provides 2 user buttons + RESET:
 *   KEY0   - PE4  (active LOW: pressed = read 0,
 *                  shares with BOOT0 via BAT54C for UART ISP entry)
 *   WK_UP  - PA0  (active HIGH: pressed = read 1)
 *   RESET  - NRST (handled by hardware)
 */

#define BUTTON_KEY0       0
#define BUTTON_WKUP       1
#define NUM_BUTTONS       2

#define BUTTON_KEY0_BIT   (1 << BUTTON_KEY0)
#define BUTTON_WKUP_BIT   (1 << BUTTON_WKUP)

/* Alternate function pin selections ****************************************/

/* USART1: routed to on-board CH340C USB-UART (Type-C USB1 connector).
 * This is the default console.
 */

#define GPIO_USART1_RX    GPIO_USART1_RX_1 /* PA10 */
#define GPIO_USART1_TX    GPIO_USART1_TX_1 /* PA9  */

/* USART2 / USART3: optional, exposed on JP1/JP2 headers (not on-board). */

#define GPIO_USART2_RX    GPIO_USART2_RX_1 /* PA3 */
#define GPIO_USART2_TX    GPIO_USART2_TX_1 /* PA2 */

#define GPIO_USART3_RX    GPIO_USART3_RX_1 /* PB11 */
#define GPIO_USART3_TX    GPIO_USART3_TX_1 /* PB10 */

/* I2C1: on-board 24C02 EEPROM (4.7k pull-ups already on board).
 * Future shared bus with MPU6050 / BME280 / SSD1306 over JP1/JP2.
 */

#define GPIO_I2C1_SCL     GPIO_I2C1_SCL_2  /* PB8 */
#define GPIO_I2C1_SDA     GPIO_I2C1_SDA_2  /* PB9 */

/* SPI1: on-board W25Q128 SPI flash (CS is GPIO PB14, see board source). */

#define GPIO_SPI1_SCK     GPIO_SPI1_SCK_2  /* PB3 */
#define GPIO_SPI1_MISO    GPIO_SPI1_MISO_2 /* PB4 */
#define GPIO_SPI1_MOSI    GPIO_SPI1_MOSI_2 /* PB5 */

/* SPI1 DMA stream selection (STM32F4 has two RX and two TX options for
 * SPI1; this BSP picks the lower-stream pair to leave 2/5 free for ADC
 * or future SPI peripherals).  These macros are read by
 * `arch/arm/src/stm32/stm32_spi.c` only when CONFIG_STM32_SPI1_DMA=y.
 *
 *   SPI1_RX: DMA2 Stream 0 / Channel 3
 *   SPI1_TX: DMA2 Stream 3 / Channel 3
 */

#define DMACHAN_SPI1_RX   DMAMAP_SPI1_RX_1
#define DMACHAN_SPI1_TX   DMAMAP_SPI1_TX_1

/* No Ethernet PHY on this board. CAN transceiver is also absent. */

#endif /* __BOARDS_ARM_STM32_ALIENTEK_M144Z_M4_INCLUDE_BOARD_H */
