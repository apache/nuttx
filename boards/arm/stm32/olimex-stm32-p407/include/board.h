/****************************************************************************
 * boards/arm/stm32/olimex-stm32-p407/include/board.h
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

#ifndef __BOARDS_ARM_STM32_OLIMEX_STM32_P407_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_OLIMEX_STM32_P407_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC (30-60KHz, uncalibrated)
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
#define BOARD_TIM2_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM3_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM4_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM5_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM6_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM7_FREQUENCY    STM32_HCLK_FREQUENCY
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

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs
 * in any way. The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_LED4        3
#define BOARD_NLEDS       4

#define BOARD_LED_GREEN1  BOARD_LED1
#define BOARD_LED_YELLOW  BOARD_LED2
#define BOARD_LED_RED     BOARD_LED3
#define BOARD_LED_GREEN2  BOARD_LED4

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)
#define BOARD_LED4_BIT    (1 << BOARD_LED4)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 4 LEDs on
 * board the Olimex STM32-P407.  The following definitions describe how
 * NuttX controls the LEDs:
 */

#define LED_STARTED       0  /* LED1 */
#define LED_HEAPALLOCATE  1  /* LED2 */
#define LED_IRQSENABLED   2  /* LED1 + LED2 */
#define LED_STACKCREATED  3  /* LED3 */
#define LED_INIRQ         4  /* LED1 + LED3 */
#define LED_SIGNAL        5  /* LED2 + LED3 */
#define LED_ASSERTION     6  /* LED1 + LED2 + LED3 */
#define LED_PANIC         7  /* N/C  + N/C  + N/C + LED4 */

/* Button definitions *******************************************************/

/* The Olimex STM32-P407 supports seven buttons: */

#define BUTTON_TAMPER         0
#define BUTTON_WKUP           1

#ifdef CONFIG_INPUT_DJOYSTICK
#  define NUM_BUTTONS         2
#else
#  define JOYSTICK_RIGHT      2
#  define JOYSTICK_UP         3
#  define JOYSTICK_LEFT       4
#  define JOYSTICK_DOWN       5
#  define JOYSTICK_CENTER     6

#  define NUM_BUTTONS         7
#endif

#define BUTTON_TAMPER_BIT     (1 << BUTTON_TAMPER)
#define BUTTON_WKUP_BIT       (1 << BUTTON_WKUP)

#ifndef CONFIG_INPUT_DJOYSTICK
#  define JOYSTICK_RIGHT_BIT  (1 << JOYSTICK_RIGHT)
#  define JOYSTICK_UP_BIT     (1 << JOYSTICK_UP)
#  define JOYSTICK_LEFT_BIT   (1 << JOYSTICK_LEFT)
#  define JOYSTICK_DOWN_BIT   (1 << JOYSTICK_DOWN)
#  define JOYSTICK_CENTER_BIT (1 << JOYSTICK_CENTER)
#endif

/* Alternate function pin selections ****************************************/

/* USART3: */

#define GPIO_USART3_RX    GPIO_USART3_RX_3  /* PD9  */
#define GPIO_USART3_TX    GPIO_USART3_TX_3  /* PD8  */
#define GPIO_USART3_CTS   GPIO_USART3_CTS_2 /* PD11 */
#define GPIO_USART3_RTS   GPIO_USART3_RTS_2 /* PD12 */

/* UEXT USART3: This will redefine the above macros if enabled. */

#ifdef CONFIG_STM32_OLIMEXP407_UEXT_USART3
#  undef  GPIO_USART3_RX    GPIO_USART3_RX_3
#  undef  GPIO_USART3_TX    GPIO_USART3_TX_3
#  undef  GPIO_USART3_CTS   GPIO_USART3_CTS_2
#  undef  GPIO_USART3_RTS   GPIO_USART3_RTS_2

#  define GPIO_USART3_RX    GPIO_USART3_RX_2 /* PC11 */
#  define GPIO_USART3_TX    GPIO_USART3_TX_2 /* PC10 */
#endif

/* USART6: */

#define GPIO_USART6_RX    GPIO_USART6_RX_2  /* PG9  */
#define GPIO_USART6_TX    GPIO_USART6_TX_1  /* PC6  */

/* CAN: */

#define GPIO_CAN1_RX      GPIO_CAN1_RX_2    /* PB8 */
#define GPIO_CAN1_TX      GPIO_CAN1_TX_2    /* PB9 */

/* microSD Connector:
 *
 *   ----------------- ----------------- ------------------------
 *   SD/MMC CONNECTOR        BOARD        GPIO CONFIGURATION(s
 *   PIN SIGNAL             SIGNAL          (no remapping)
 *   --- ------------- ----------------- -------------------------
 *   1   DAT2/RES      SD_D2/USART3_TX/  PC10 GPIO_SDIO_D2
 *                     SPI3_SCK
 *   2   CD/DAT3/CS    SD_D3/USART3_RX/  PC11 GPIO_SDIO_D3
 *                     SPI3_MISO
 *   3   CMD/DI        SD_CMD            PD2  GPIO_SDIO_CMD
 *   4   VDD           N/A               N/A
 *   5   CLK/SCLK      SD_CLK/SPI3_MOSI  PC12 GPIO_SDIO_CK
 *   6   VSS           N/A               N/A
 *   7   DAT0/D0       SD_D0/DCMI_D2     PC8  GPIO_SDIO_D0
 *   8   DAT1/RES      SD_D1/DCMI_D3     PC9  GPIO_SDIO_D1
 *   --- ------------- ----------------- -------------------------
 *
 *   NOTES:
 *   1. DAT4, DAT4, DAT6, and DAT7 not connected.
 *   2. There are no alternative pin selections.
 *   3. There is no card detect (CD) GPIO input so we will not
 *      sense if there is a card in the SD slot or not.  This will
 *      make usage very awkward.
 */

/* Ethernet:
 *
 * - PA2  is ETH_MDIO
 * - PC1  is ETH_MDC
 * - PB5  is ETH_PPS_OUT     - NC (not connected)
 * - PA0  is ETH_MII_CRS     - NC
 * - PA3  is ETH_MII_COL     - NC
 * - PB10 is ETH_MII_RX_ER   - NC
 * - PB0  is ETH_MII_RXD2    - NC
 * - PH7  is ETH_MII_RXD3    - NC
 * - PC3  is ETH_MII_TX_CLK  - NC
 * - PC2  is ETH_MII_TXD2    - NC
 * - PB8  is ETH_MII_TXD3    - NC
 * - PA1  is ETH_MII_RX_CLK/ETH_RMII_REF_CLK
 * - PA7  is ETH_MII_RX_DV/ETH_RMII_CRS_DV
 * - PC4  is ETH_MII_RXD0/ETH_RMII_RXD0
 * - PC5  is ETH_MII_RXD1/ETH_RMII_RXD1
 * - PB11 is ETH_MII_TX_EN/ETH_RMII_TX_EN
 * - PG13 is ETH_MII_TXD0/ETH_RMII_TXD0
 * - PG14 is ETH_MII_TXD1/ETH_RMII_TXD1
 */

#define GPIO_ETH_PPS_OUT    GPIO_ETH_PPS_OUT_1
#define GPIO_ETH_MII_CRS    GPIO_ETH_MII_CRS_1
#define GPIO_ETH_MII_COL    GPIO_ETH_MII_COL_1
#define GPIO_ETH_MII_RX_ER  GPIO_ETH_MII_RX_ER_1
#define GPIO_ETH_MII_RXD2   GPIO_ETH_MII_RXD2_1
#define GPIO_ETH_MII_RXD3   GPIO_ETH_MII_RXD3_1
#define GPIO_ETH_MII_TXD3   GPIO_ETH_MII_TXD3_1
#define GPIO_ETH_MII_TX_EN  GPIO_ETH_MII_TX_EN_2
#define GPIO_ETH_MII_TXD0   GPIO_ETH_MII_TXD0_2
#define GPIO_ETH_MII_TXD1   GPIO_ETH_MII_TXD1_2
#define GPIO_ETH_RMII_TX_EN GPIO_ETH_RMII_TX_EN_1
#define GPIO_ETH_RMII_TXD0  GPIO_ETH_RMII_TXD0_2
#define GPIO_ETH_RMII_TXD1  GPIO_ETH_RMII_TXD1_2

/* DMA Channel/Stream Selections ********************************************/

/* Stream selections are arbitrary for now but might become important in
 * the future if we set aside more DMA channels/streams.
 *
 * SDIO DMA
 *   DMAMAP_SDIO_1      = Channel 4, Stream 3
 *   DMAMAP_SDIO_2      = Channel 4, Stream 6
 */

#define DMAMAP_SDIO       DMAMAP_SDIO_1

/* USART6
 *
 *   DMAMAP_USART6_RX_1 = Channel 5, Stream1
 *   DMAMAP_USART6_RX_2 = Channel 5, Stream2
 *   DMAMAP_USART6_TX_1 = Channel 5, Stream6
 *   DMAMAP_USART6_TX_2 = Channel 5, Stream7
 */

#define DMAMAP_USART6_RX  DMAMAP_USART6_RX_1
#define DMAMAP_USART6_TX  DMAMAP_USART6_TX_1

/* DHTxx pin configuration */

#define GPIO_DHTXX_PIN          (GPIO_PORTG|GPIO_PIN9)
#define GPIO_DHTXX_PIN_OUTPUT   (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_DHTXX_PIN)
#define GPIO_DHTXX_PIN_INPUT    (GPIO_INPUT|GPIO_FLOAT|GPIO_DHTXX_PIN)

#define BOARD_DHTXX_GPIO_INPUT   GPIO_DHTXX_PIN_INPUT
#define BOARD_DHTXX_GPIO_OUTPUT  GPIO_DHTXX_PIN_OUTPUT
#define BOARD_DHTXX_FRTIMER      1  /* Free-run timer 1 */

/* SPI3 - As present in the UEXT header */

#define GPIO_SPI3_MISO    GPIO_SPI3_MISO_2
#define GPIO_SPI3_MOSI    GPIO_SPI3_MOSI_2
#define GPIO_SPI3_SCK     GPIO_SPI3_SCK_2

#define DMACHAN_SPI3_RX   DMAMAP_SPI3_RX_1
#define DMACHAN_SPI3_TX   DMAMAP_SPI3_TX_1

/* I2S3 - CS4344 configuration uses I2S3 */

#define GPIO_I2S3_SD      GPIO_I2S3_SD_1
#define GPIO_I2S3_CK      GPIO_I2S3_CK_1
#define GPIO_I2S3_WS      GPIO_I2S3_WS_2

#define DMACHAN_I2S3_RX   DMAMAP_SPI3_RX_2
#define DMACHAN_I2S3_TX   DMAMAP_SPI3_TX_2

#endif /* __BOARDS_ARM_STM32_OLIMEX_STM32_P407_INCLUDE_BOARD_H */
