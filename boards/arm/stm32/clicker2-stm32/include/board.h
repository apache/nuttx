/****************************************************************************
 * boards/arm/stm32/clicker2-stm32/include/board.h
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

#ifndef __BOARDS_ARM_STM32_CLICKER2_STM32_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_CLICKER2_STM32_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#ifdef __KERNEL__
#  include "stm32_rcc.h"
#  include "stm32_sdio.h"
#  include "stm32.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Clicker 2 for STM32 board features a 25Hz crystal and 32.768kHz RTC
 * crystal.
 *
 * This is the canonical configuration:
 *   System Clock source     : PLL (HSE)
 *   SYSCLK(Hz)              : 168000000    Determined by PLL configuration
 *   HCLK(Hz)                : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler           : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler          : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler          : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)       : 25000000     (STM32_BOARD_XTAL)
 *   PLLM                    : 25           (STM32_PLLCFG_PLLM)
 *   PLLN                    : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                    : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                    : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator
 *           output voltage  : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)       : 5
 *   Prefetch Buffer         : OFF
 *   Instruction cache       : ON
 *   Data cache              : ON
 *   Require 48MHz for
 *   USB OTG FS,             : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 25MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        25000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (25,000,000 / 25) * 336
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 2 = 168,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(25)
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

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(118+2)=400 KHz
 */

#define SDIO_INIT_CLKDIV        (118 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* LED definitions **********************************************************/

/* The Mikroe Clicker2 STM32 has two user controllable LEDs:
 *
 *   LD1 - PE12, Active high output illuminates
 *   LD2 - PE15, Active high output illuminates
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 2 LEDs on
 * board the Clicker2 for STM32.
 * The following definitions describe how NuttX controls the LEDs:
 *
 *   SYMBOL               Meaning                      LED state
 *                                                   LED1     LED2
 *   -------------------  -----------------------  -------- --------
 *   LED_STARTED          NuttX has been started     OFF      OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF      OFF
 *   LED_STACKCREATED     Idle stack created         ON       OFF
 *   LED_INIRQ            In an interrupt            N/C      ON
 *   LED_SIGNAL           In a signal handler          No change
 *   LED_ASSERTION        An assertion failed          No change
 *   LED_PANIC            The system has crashed     OFF      Blinking
 *   LED_IDLE             STM32 is is sleep mode       Not used
 */

#define LED_STARTED              0
#define LED_HEAPALLOCATE         0
#define LED_IRQSENABLED          0
#define LED_STACKCREATED         1
#define LED_INIRQ                2
#define LED_SIGNAL               3
#define LED_ASSERTION            3
#define LED_PANIC                4

/* Button definitions *******************************************************/

/* The Mikroe Clicker2 STM32 has two buttons available to software:
 *
 *   T2 - PE0, Low sensed when pressed
 *   T3 - PA10, Low sensed when pressed
 */

#define BUTTON_T2         0
#define BUTTON_T3         1
#define NUM_BUTTONS       2

#define BUTTON_T2_BIT    (1 << BUTTON_T2)
#define BUTTON_T3_BIT    (1 << BUTTON_T3)

/* Alternate function pin selections ****************************************/

/* U[S]ARTs
 *
 *   USART2 - mikroBUS1
 *   USART3 - mikroBUS2
 *
 * Assuming RS-232 connverted connected on mikroMB1/12
 */

#define GPIO_USART2_RX   GPIO_USART2_RX_2  /* PD6 */
#define GPIO_USART2_TX   GPIO_USART2_TX_2  /* PD5 */

#define GPIO_USART3_RX   GPIO_USART3_RX_3  /* PD9 */
#define GPIO_USART3_TX   GPIO_USART3_TX_3  /* PD8 */

/* SPI
 *
 *   SPI2 - mikroBUS2
 *   SPI3 - mikroBUS1
 */

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1  /* PC12 */
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1  /* PC11 */
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_2   /* PC10 */

#define GPIO_SPI3_MISO   GPIO_SPI3_MISO_2  /* PB15 */
#define GPIO_SPI3_MOSI   GPIO_SPI3_MOSI_2  /* PB14 */
#define GPIO_SPI3_SCK    GPIO_SPI3_SCK_2   /* PB13 */

/* I2C
 *
 *   I2C2 - mikroBUS2
 *   I2C3 - mikroBUS1
 */

#define GPIO_I2C2_SCL    GPIO_I2C2_SCL_1   /* PB10 */
#define GPIO_I2C2_SDA    GPIO_I2C2_SDA_1   /* PB11 */

#define GPIO_I2C3_SCL    GPIO_I2C3_SCL_1   /* PA8 */
#define GPIO_I2C3_SDA    GPIO_I2C3_SDA_1   /* PC9 */

/* Analog
 *
 *   mikroBUS1 ADC: PA2-MB1_AN
 *   mikroBUS1 ADC: PA3-MB2_AN
 */

/* PWM
 *
 *   mikroBUS1 ADC: PE9-MB1-PWM  (TIM1, channel 1)
 *   mikroBUS1 ADC: PD12-MB2-PWM (TIM4, channel 1)
 */

#define GPIO_TIM1_CH1OUT GPIO_TIM1_CH1OUT_2 /* PE9 */
#define GPIO_TIM4_CH1OUT GPIO_TIM4_CH1OUT_2 /* PD12 */

/* DMA Channel/Stream Selections ********************************************/

/* Stream selections are arbitrary for now but might become important in the
 * future if we set aside more DMA channels/streams.
 *
 * SDIO DMA
 *   DMAMAP_SDIO_1 = Channel 4, Stream 3
 *   DMAMAP_SDIO_2 = Channel 4, Stream 6
 */

#define DMAMAP_SDIO      DMAMAP_SDIO_1

#endif /* __BOARDS_ARM_STM32_CLICKER2_STM32_INCLUDE_BOARD_H */
