/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/include/board.h
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

#ifndef __BOARDS_ARM_STM32H7_NUCLEO_H743ZI_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32H7_NUCLEO_H743ZI_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Do not include STM32 H7 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Nucleo-144  board provides the following clock sources:
 *
 *   MCO: 8 MHz from MCO output of ST-LINK is used as input clock (default)
 *   X2:  32.768 KHz crystal for LSE
 *   X3:  HSE crystal oscillator (not provided)
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: 8 MHz from MCO output of ST-LINK
 *   LSE: 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul /* ST-LINK MCO */

#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

#define STM32_HSEBYP_ENABLE

/* I2C123 clock source - HSI */

#define STM32_RCC_D2CCIP2R_I2C123SRC RCC_D2CCIP2R_I2C123SEL_HSI

/* I2C4 clock source - HSI */

#define STM32_RCC_D3CCIPR_I2C4SRC    RCC_D3CCIPR_I2C4SEL_HSI

/* SPI123 clock source - PLL1Q */

#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL1

/* SPI45 clock source - APB (PCLK2?) */

#define STM32_RCC_D2CCIP1R_SPI45SRC  RCC_D2CCIP1R_SPI45SEL_APB

/* SPI6 clock source - APB (PCLK4) */

#define STM32_RCC_D3CCIPR_SPI6SRC    RCC_D3CCIPR_SPI6SEL_PCLK4

/* USB 1 and 2 clock source - HSI48 */

#define STM32_RCC_D2CCIP2R_USBSRC    RCC_D2CCIP2R_USBSEL_HSI48

/* ADC 1 2 3 clock source - pll2_pclk */

#define STM32_RCC_D3CCIPR_ADCSRC     RCC_D3CCIPR_ADCSEL_PLL2

/* FLASH wait states
 *
 *  ------------ ---------- -----------
 *  Vcore        MAX ACLK   WAIT STATES
 *  ------------ ---------- -----------
 *  1.15-1.26 V     70 MHz    0
 *  (VOS1 level)   140 MHz    1
 *                 210 MHz    2
 *  1.05-1.15 V     55 MHz    0
 *  (VOS2 level)   110 MHz    1
 *                 165 MHz    2
 *                 220 MHz    3
 *  0.95-1.05 V     45 MHz    0
 *  (VOS3 level)    90 MHz    1
 *                 135 MHz    2
 *                 180 MHz    3
 *                 225 MHz    4
 *  ------------ ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 4

/* SDMMC definitions ********************************************************/

/* Init 400kHz, PLL1Q/(2*250) */

#define STM32_SDMMC_INIT_CLKDIV     (250 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* Just set these to 25 MHz for now,
 * PLL1Q/(2*4), for default speed 12.5MB/s
 */

#define STM32_SDMMC_MMCXFR_CLKDIV   (4 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#define STM32_SDMMC_SDXFR_CLKDIV    (4 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

#define STM32_SDMMC_CLKCR_EDGE      STM32_SDMMC_CLKCR_NEGEDGE

/* Ethernet definitions *****************************************************/

#define GPIO_ETH_RMII_TXD0    (GPIO_ETH_RMII_TXD0_2 | GPIO_SPEED_100MHz)    /* PG13 */
#define GPIO_ETH_RMII_TXD1    (GPIO_ETH_RMII_TXD1_1 | GPIO_SPEED_100MHz)    /* PB13 */
#define GPIO_ETH_RMII_TX_EN   (GPIO_ETH_RMII_TX_EN_2 | GPIO_SPEED_100MHz)   /* PG11 */
#define GPIO_ETH_MDC          (GPIO_ETH_MDC_0 | GPIO_SPEED_100MHz)          /* PC1 */
#define GPIO_ETH_MDIO         (GPIO_ETH_MDIO_0 | GPIO_SPEED_100MHz)         /* PA2 */
#define GPIO_ETH_RMII_RXD0    (GPIO_ETH_RMII_RXD0_0 | GPIO_SPEED_100MHz)    /* PC4 */
#define GPIO_ETH_RMII_RXD1    (GPIO_ETH_RMII_RXD1_0 | GPIO_SPEED_100MHz)    /* PC5 */
#define GPIO_ETH_RMII_CRS_DV  (GPIO_ETH_RMII_CRS_DV_0 | GPIO_SPEED_100MHz)  /* PA7 */
#define GPIO_ETH_RMII_REF_CLK (GPIO_ETH_RMII_REF_CLK_0 | GPIO_SPEED_100MHz) /* PA1 */

/* LED definitions **********************************************************/

/* The Nucleo-144 board has numerous LEDs but only three, LD1 a Green LED,
 * LD2 a Blue LED and LD3 a Red LED, that can be controlled by software.
 * The following definitions assume the default Solder Bridges are installed.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_BLUE    BOARD_LED2
#define BOARD_LED_RED     BOARD_LED3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c.
 * The LEDs are used to encode OS-related events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        Red   Green Blue
 *   ----------------------  --------------------------  ------ ------ ---
 */

#define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  OFF    OFF   ON   */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    ON    ON   */
#define LED_INIRQ          4 /* In an interrupt          N/C    N/C   GLOW */
#define LED_SIGNAL         5 /* In a signal handler      N/C    GLOW  N/C  */
#define LED_ASSERTION      6 /* An assertion failed      GLOW   N/C   GLOW */
#define LED_PANIC          7 /* The system has crashed   Blink  OFF   N/C  */
#define LED_IDLE           8 /* MCU is is sleep mode     ON     OFF   OFF  */

/* Thus if the Green LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the Red LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Button definitions *******************************************************/

/* The NUCLEO board supports one button:  Pushbutton B1, labeled "User", is
 * connected to GPIO PI11.
 * A high value will be sensed when the button is depressed.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* ADC */

#define GPIO_ADC12_INP5   GPIO_ADC12_INP5_0                      /* PB1, channel 5 */
#define GPIO_ADC123_INP10 GPIO_ADC123_INP10_0                    /* PC0, channel 10 */
#define GPIO_ADC123_INP12 GPIO_ADC123_INP12_0                    /* PC2, channel 12 */
#define GPIO_ADC12_INP13  GPIO_ADC12_INP13_0                     /* PC3, channel 13 */
#define GPIO_ADC12_INP15  GPIO_ADC12_INP15_0                     /* PA3, channel 15 */
#define GPIO_ADC12_INP18  GPIO_ADC12_INP18_0                     /* PA4, channel 18 */
#define GPIO_ADC12_INP19  GPIO_ADC12_INP19_0                     /* PA5, channel 19 */
#define GPIO_ADC123_INP7  GPIO_ADC12_INP7_0                      /* PA7, channel 7 */
#define GPIO_ADC123_INP11 GPIO_ADC123_INP11_0                    /* PC1, channel 11 */
#define GPIO_ADC2_INP2    GPIO_ADC2_INP2_0                       /* PF13, channel 2 */
#define GPIO_ADC12_INP3   GPIO_ADC12_INP3_0                      /* PA6, channel 3  */
#define GPIO_ADC12_INP14  GPIO_ADC12_INP14_0                     /* PA2, channel 14 */
#define GPIO_ADC12_INP4   GPIO_ADC12_INP4_0                      /* PC4, channel 4  */
#define GPIO_ADC12_INP8   GPIO_ADC12_INP8_0                      /* PC5, channel 8  */

/* USART3 (Nucleo Virtual Console) */

#define GPIO_USART3_RX    (GPIO_USART3_RX_3 | GPIO_SPEED_100MHz) /* PD9 */
#define GPIO_USART3_TX    (GPIO_USART3_TX_3 | GPIO_SPEED_100MHz) /* PD8 */

#define DMAMAP_USART3_RX DMAMAP_DMA12_USART3RX_0
#define DMAMAP_USART3_TX DMAMAP_DMA12_USART3TX_1

/* USART6 (Arduino Serial Shield) */

#define GPIO_USART6_RX    (GPIO_USART6_RX_2 | GPIO_SPEED_100MHz) /* PG9 */
#define GPIO_USART6_TX    (GPIO_USART6_TX_2 | GPIO_SPEED_100MHz) /* PG14 */

/* I2C1 Use Nucleo I2C1 pins */

#define GPIO_I2C1_SCL     (GPIO_I2C1_SCL_2 | GPIO_SPEED_50MHz) /* PB8 - D15 */
#define GPIO_I2C1_SDA     (GPIO_I2C1_SDA_2 | GPIO_SPEED_50MHz) /* PB9 - D14 */

/* I2C2 Use Nucleo I2C2 pins */

#define GPIO_I2C2_SCL     (GPIO_I2C2_SCL_2  | GPIO_SPEED_50MHz) /* PF1 - D69 */
#define GPIO_I2C2_SDA     (GPIO_I2C2_SDA_2  | GPIO_SPEED_50MHz) /* PF0 - D68 */
#define GPIO_I2C2_SMBA    (GPIO_I2C2_SMBA_2 | GPIO_SPEED_50MHz) /* PF2 - D70 */

/* SPI3 */

#define GPIO_SPI3_MISO    (GPIO_SPI3_MISO_1 | GPIO_SPEED_50MHz) /* PB4 */
#define GPIO_SPI3_MOSI    (GPIO_SPI3_MOSI_4 | GPIO_SPEED_50MHz) /* PB5 */
#define GPIO_SPI3_SCK     (GPIO_SPI3_SCK_1 | GPIO_SPEED_50MHz)  /* PB3 */
#define GPIO_SPI3_NSS     (GPIO_SPI3_NSS_2 | GPIO_SPEED_50MHz)  /* PA4 */

/* TIM1 - Advanced Timer 16-bit (4 channels) */
#define GPIO_TIM1_CH1IN   (GPIO_TIM1_CH1IN_2)   /* PE9  */
#define GPIO_TIM1_CH2IN   (GPIO_TIM1_CH2IN_2)   /* PE11 */
#define GPIO_TIM1_CH3IN   (GPIO_TIM1_CH3IN_2)   /* PE13 */
#define GPIO_TIM1_CH4IN   (GPIO_TIM1_CH4IN_2)   /* PE14 */

#define GPIO_TIM1_CH1OUT  (GPIO_TIM1_CH1OUT_2)  /* PE9  - D6 */
#define GPIO_TIM1_CH1NOUT (GPIO_TIM1_CH1NOUT_3) /* PE8  - D42 */
#define GPIO_TIM1_CH2OUT  (GPIO_TIM1_CH2OUT_2)  /* PE11 - D5 */
#define GPIO_TIM1_CH2NOUT (GPIO_TIM1_CH2NOUT_3) /* PE10 - D40 */
#define GPIO_TIM1_CH3OUT  (GPIO_TIM1_CH3OUT_2)  /* PE13 - D3 */
#define GPIO_TIM1_CH3NOUT (GPIO_TIM1_CH3NOUT_3) /* PE12 - D39 */
#define GPIO_TIM1_CH4OUT  (GPIO_TIM1_CH4OUT_2)  /* PE14 - D38 */

/* TIM2 - General Purpose 32-bit Timer (4 channels) */
#define GPIO_TIM2_CH1IN   (GPIO_TIM2_CH1IN_2)   /* PA15 */
#define GPIO_TIM2_CH2IN   (GPIO_TIM2_CH2IN_1)   /* PB3 */
#define GPIO_TIM2_CH3IN   (GPIO_TIM2_CH3IN_1)   /* PB10 */
#define GPIO_TIM2_CH4IN   (GPIO_TIM2_CH4IN_1)   /* PB11 */

/* TIM3 - General Purpose 16-bit Timer (4 channels) */
#define GPIO_TIM3_CH1IN   (GPIO_TIM3_CH1IN_1)   /* PA6 */
#define GPIO_TIM3_CH2IN   (GPIO_TIM3_CH2IN_1)   /* PA7 */
#define GPIO_TIM3_CH3IN   (GPIO_TIM3_CH3IN_1)   /* PB0 */
#define GPIO_TIM3_CH4IN   (GPIO_TIM3_CH4IN_1)   /* PB1 */

/* TIM4 - General Purpose 16-bit Timer (4 channels) */
#define GPIO_TIM4_CH1IN   (GPIO_TIM4_CH1IN_2)   /* PD12 */
#define GPIO_TIM4_CH2IN   (GPIO_TIM4_CH2IN_2)   /* PD13 */
#define GPIO_TIM4_CH3IN   (GPIO_TIM4_CH3IN_2)   /* PD14 */
#define GPIO_TIM4_CH4IN   (GPIO_TIM4_CH4IN_2)   /* PD15 */

/* TIM5 - General Purpose 32-bit Timer (4 channels) */
#define GPIO_TIM5_CH1IN   (GPIO_TIM5_CH1IN_2)
#define GPIO_TIM5_CH2IN   (GPIO_TIM5_CH2IN_2)
#define GPIO_TIM5_CH3IN   (GPIO_TIM5_CH3IN_2)
#define GPIO_TIM5_CH4IN   (GPIO_TIM5_CH4IN_2)

/* TIM6 - Basic 16-bit Timer (0 channels) */

/* TIM7 - Basic 16-bit Timer (0 channels) */

/* TIM8 - Advanced 16-bit Timer (4 channels) */
#define GPIO_TIM8_CH1IN   (GPIO_TIM8_CH1IN_1)
#define GPIO_TIM8_CH2IN   (GPIO_TIM8_CH2IN_1)
#define GPIO_TIM8_CH3IN   (GPIO_TIM8_CH3IN_1)
#define GPIO_TIM8_CH4IN   (GPIO_TIM8_CH4IN_1)

/* TIM12 - General purpose 16-bit Timer (2 channels) */
#define GPIO_TIM12_CH1IN  (GPIO_TIM12_CH1IN_1)
#define GPIO_TIM12_CH2IN  (GPIO_TIM12_CH2IN_1)

/* TIM13 - General purpose 16-bit Timer (1 channels) */
#define GPIO_TIM13_CH1IN  (GPIO_TIM13_CH1IN_1)

/* TIM14 - General purpose 16-bit Timer (1 channels) */
#define GPIO_TIM14_CH1IN  (GPIO_TIM14_CH1IN_1)

/* TIM15 - General purpose 16-bit Timer (2 channels) */
#define GPIO_TIM15_CH1IN  (GPIO_TIM15_CH1IN_1)
#define GPIO_TIM15_CH2IN  (GPIO_TIM15_CH2IN_1)

/* TIM16 - General purpose 16-bit Timer (1 channels) */
#define GPIO_TIM16_CH1IN  (GPIO_TIM16_CH1IN_1)

/* TIM17 - General purpose 16-bit Timer (1 channels) */
#define GPIO_TIM17_CH1IN  (GPIO_TIM17_CH1IN_1)

/* OTGFS */

#define GPIO_OTGFS_DM  (GPIO_OTGFS_DM_0  | GPIO_SPEED_100MHz)
#define GPIO_OTGFS_DP  (GPIO_OTGFS_DP_0  | GPIO_SPEED_100MHz)
#define GPIO_OTGFS_ID  (GPIO_OTGFS_ID_0  | GPIO_SPEED_100MHz)

/* DMA **********************************************************************/

#define DMAMAP_SPI3_RX DMAMAP_DMA12_SPI3RX_0 /* DMA1 */
#define DMAMAP_SPI3_TX DMAMAP_DMA12_SPI3TX_0 /* DMA1 */

#define DMAMAP_USART6_RX DMAMAP_DMA12_USART6RX_1
#define DMAMAP_USART6_TX DMAMAP_DMA12_USART6TX_0

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32H7_NUCLEO_H743ZI_INCLUDE_BOARD_H */
