/****************************************************************************
 * boards/arm/stm32/omnibusf4/include/board.h
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

#ifndef __BOARDS_ARM_STM32_OMNIBUSF4_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_OMNIBUSF4_INCLUDE_BOARD_H

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

/* The OMNIBUSF4 board uses a single 8MHz crystal.
 *
 * This is the canonical configuration:
 *   System Clock source    : PLL (HSE)
 *   SYSCLK(Hz)             : 168000000    Determined by PLL configuration
 *   HCLK(Hz)               : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler          : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler         : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler         : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)      : 8000000      (STM32_BOARD_XTAL)
 *   PLLM                   : 8            (STM32_PLLCFG_PLLM)
 *   PLLN                   : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                   : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                   : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator
 *   output voltage         : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)      : 5
 *   Prefetch Buffer        : OFF
 *   Instruction cache      : ON
 *   Data cache             : ON
 *   Require 48MHz for
 *   USB OTG FS,            : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (8,000,000 / 8) * 336
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 2 = 168,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(336)
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

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM3_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM4_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM5_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM6_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM7_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY

/* Pin configurations *******************************************************/

#define BOARD_NLEDS     2                      /* One literal LED, one beeper */
#define GPIO_LED1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |\
                         GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN5)
#define GPIO_BEEPER1    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |\
                         GPIO_OUTPUT_CLEAR | GPIO_PORTB|GPIO_PIN4)

/* USART1: */

#if 0
#define INVERTER_PIN_USART1     PC0            /* DYS F4 Pro, Omnibus F4 AIO 1st Gen only */
#endif
#define GPIO_USART1_RX  GPIO_USART1_RX_1       /* PA10 */
#define GPIO_USART1_TX  GPIO_USART1_TX_1       /* PA9  */

/* USART2:
 *
 * TODO: Do OMNIBUSF4 targets use USART2?
 */

/* USART3: */

#define GPIO_USART3_TX    GPIO_USART3_TX_1     /* PB10 */
#define GPIO_USART3_RX    GPIO_USART3_RX_1     /* PB11 */

/* USART4: */

/* USART6: */

#if 0
#define INVERTER_PIN_UART6      PC8            /* Omnibus F4 V3 and later, EXUAVF4PRO */
#endif
#define GPIO_USART6_RX    GPIO_USART6_RX_1     /* PC7 */
#define GPIO_USART6_TX    GPIO_USART6_TX_1     /* PC6 */

/* PWM - motor outputs, etc. are on these pins: */

#define GPIO_TIM3_CH3OUT  GPIO_TIM3_CH3OUT_1   /* S1_OUT  PB0 */
#define GPIO_TIM3_CH4OUT  GPIO_TIM3_CH4OUT_1   /* S2_OUT  PB1 */
#define GPIO_TIM2_CH4OUT  GPIO_TIM2_CH4OUT_1   /* S3_OUT  PA3 */
#define GPIO_TIM2_CH3OUT  GPIO_TIM3_CH3OUT_1   /* S4_OUT  PA2 */

/* SPI1 :
 *
 * MPU6000 6-axis motion sensor (accelerometer + gyroscope), or
 * MPU6500 6-Axis MEMS MotionTracking Device with DMP
 *
 * MPU6000 interrupts
 * #define USE_GYRO_EXTI
 * #define GYRO_1_EXTI_PIN         PC4
 * #define USE_MPU_DATA_READY_SIGNAL
 *
 * #define GYRO_1_ALIGN            CW270_DEG
 * #define ACC_1_ALIGN             CW270_DEG
 */

#define GPIO_SPI1_MISO    GPIO_SPI1_MISO_1  /* PA6 */
#define GPIO_SPI1_MOSI    GPIO_SPI1_MOSI_1  /* PA7 */
#define GPIO_SPI1_SCK     GPIO_SPI1_SCK_1   /* PA5 */
#if 0
#define GPIO_SPI1_NSS     GPIO_SPI1_NSS_2   /* PA4 */
#endif
#define DMACHAN_SPI1_RX   DMAMAP_SPI1_RX_1  /* 2:0:3 */
#define DMACHAN_SPI1_TX   DMAMAP_SPI1_TX_1  /* 2:3:3 */

/* SPI2 :
 *
 * Used for MMC/SD on OMNIBUSF4SD.
 */

#define GPIO_SPI2_MISO    GPIO_SPI2_MISO_1  /* PB14 */
#define GPIO_SPI2_MOSI    GPIO_SPI2_MOSI_1  /* PB15 */
#define GPIO_SPI2_NSS     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN12)
#define GPIO_SPI2_SCK     GPIO_SPI2_SCK_2   /* PB13 */
#define DMACHAN_SPI2_RX   DMAMAP_SPI2_RX    /* 1:3:0 */
#define DMACHAN_SPI2_TX   DMAMAP_SPI2_TX    /* 1:4:0 */

#define GPIO_MMCSD_NSS    GPIO_SPI2_NSS
#define GPIO_MMCSD_NCD    (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                           GPIO_PORTB | GPIO_PIN7) /* PB7 SD_DET */

/* SPI3 :
 *
 * OMNIBUSF4SD targets use PA15 for NSS; others use PB4
 * (? BF code says "PB3").
 * define GPIO_SPI3_NSS     GPIO_SPI3_NSS_2   PB4
 *
 * Barometer and/or MAX7456, depending on the target.
 * (OMNIBUSF4BASE targets appear to have a cyrf6936 device.)
 */

#define GPIO_SPI3_MISO    GPIO_SPI3_MISO_2  /* PC11 */
#define GPIO_SPI3_MOSI    GPIO_SPI3_MOSI_2  /* PC12 */
#define GPIO_SPI3_NSS     GPIO_SPI3_NSS_1   /* PA15 */ /* TODO: doesn't work like a chip select */
#define GPIO_SPI3_SCK     GPIO_SPI3_SCK_2   /* PC10 */

#if 0
/* I2C : */

#define GPIO_I2C1_SCL     GPIO_I2C1_SCL_1
#define GPIO_I2C1_SDA     GPIO_I2C1_SDA_2
#endif

#endif /* __BOARDS_ARM_STM32_OMNIBUSF4_INCLUDE_BOARD_H */
