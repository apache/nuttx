/****************************************************************************
 * boards/arm/at32/at32f437-mini/include/board.h
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

#ifndef __BOARDS_ARM_AT32_AT32F437_MINI_INCLUDE_BOARD_H
#define __BOARDS_ARM_AT32_AT32F437_MINI_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/* Do not include at32-specific header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The AT32F437-MINI board features a single 8MHz crystal.
 * Space is provided for a 32kHz RTC backup crystal, but it is not stuffed.
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 288000000    Determined by PLL
 *                                                configuration
 *   HCLK(Hz)                      : 288000000    (AT32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (AT32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 2            (AT32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (AT32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 8000000      (AT32_BOARD_XTAL)
 *   PLLM                          : 1            (AT32_PLLCFG_PLLMS)
 *   PLLN                          : 144           (AT32_PLLCFG_PLLNS)
 *   PLLP                          : 4            (AT32_PLLCFG_PLLFR)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed
 *                                                SYSCLK
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO clock
 */

/* HSI - 48 MHz RC factory-trimmed
 * LSI - 40 KHz RC
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define AT32_BOARD_XTAL        8000000ul
#define AT32_HSI_FREQUENCY     48000000ul
#define AT32_LSI_FREQUENCY     40000
#define AT32_HSE_FREQUENCY     AT32_BOARD_XTAL
#define AT32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 *
 * FREQUENCY = HSE * AT32_PLLCFG_PLLN / (AT32_PLLCFG_PLLM * AT32_PLLCFG_PLLP)
 *
 */

#define AT32_PLLCFG_PLLM       CRM_PLL_CFG_PLL_MS(1)
#define AT32_PLLCFG_PLLN       CRM_PLL_CFG_PLL_NS(144)
#define AT32_PLLCFG_PLLP       CRM_PLL_CFG_PLL_FR_4

#define AT32_SYSCLK_FREQUENCY  288000000ul

/* AHB clock (HCLK) is SYSCLK (288MHz) */

#define AT32_HCLK_FREQUENCY    AT32_SYSCLK_FREQUENCY /* HCLK  = SYSCLK / 1 */

/* APB1 clock (PCLK1) is HCLK/2 (144MHz) */

#define AT32_PCLK1_FREQUENCY   (AT32_HCLK_FREQUENCY/2) /* PCLK1 = HCLK / 2 */

/* Timers driven from APB1 will be twice PCLK1 */

#define AT32_APB1_TIM2_CLKIN   (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM3_CLKIN   (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM4_CLKIN   (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM5_CLKIN   (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM6_CLKIN   (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM7_CLKIN   (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM12_CLKIN  (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM13_CLKIN  (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM14_CLKIN  (2*AT32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (144MHz) */

#define AT32_PCLK2_FREQUENCY   (AT32_HCLK_FREQUENCY/2) /* PCLK2 = HCLK / 2 */

/* Timers driven from APB2 will be twice PCLK2 */

#define AT32_APB2_TIM1_CLKIN   (2*AT32_PCLK2_FREQUENCY)
#define AT32_APB2_TIM8_CLKIN   (2*AT32_PCLK2_FREQUENCY)
#define AT32_APB2_TIM9_CLKIN   (2*AT32_PCLK2_FREQUENCY)
#define AT32_APB2_TIM10_CLKIN  (2*AT32_PCLK2_FREQUENCY)
#define AT32_APB2_TIM11_CLKIN  (2*AT32_PCLK2_FREQUENCY)
#define AT32_APB2_TIM20_CLKIN  (2*AT32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM2_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM3_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM4_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM5_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM6_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM7_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM8_FREQUENCY    (AT32_HCLK_FREQUENCY)

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDIOCLK=AT32_HCLK_FREQUENCY, SDIO_CK=SDIOCLK/(1438+2)=200 KHz
 */

#define SDIO_INIT_CLKDIV        (1438 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDIOCLK=AT32_HCLK_FREQUENCY, SDIO_CK=SDIOCLK/(10+2)=24 MHz
 * DMA OFF: SDIOCLK=AT32_HCLK_FREQUENCY, SDIO_CK=SDIOCLK/(22+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (10 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV    (22 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDIOCLK=AT32_HCLK_FREQUENCY, SDIO_CK=SDIOCLK/(10+2)=24 MHz
 * DMA OFF: SDIOCLK=AT32_HCLK_FREQUENCY, SDIO_CK=SDIOCLK/(22+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (10 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (22 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

#define GPIO_SDIO_CMD   GPIO_SDIO_CMD_1
#define GPIO_SDIO_CK    GPIO_SDIO_CK_1
#define GPIO_SDIO_D0    GPIO_SDIO_D0_3
#define GPIO_SDIO_D1    GPIO_SDIO_D1_1
#define GPIO_SDIO_D2    GPIO_SDIO_D2_1
#define GPIO_SDIO_D3    GPIO_SDIO_D3_1

/* LED definitions **********************************************************/

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs
 * in any way. The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_ORANGE  BOARD_LED2

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDS is defined, then NuttX will control the 2 LEDs on
 * board the AT32F437-MINI.  The following definitions describe how NuttX
 * controls the LEDs:
 *
 *   SYMBOL                Meaning                   LED state
 *                                                   LED1     LED2
 *   -------------------  -----------------------  -------- --------
 *   LED_STARTED          NuttX has been started     OFF      OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF      OFF
 *   LED_STACKCREATED     Idle stack created         ON       OFF
 *   LED_INIRQ            In an interrupt              No change
 *   LED_SIGNAL           In a signal handler          No change
 *   LED_ASSERTION        An assertion failed          No change
 *   LED_PANIC            The system has crashed     OFF      Blinking
 *   LED_IDLE             AT32 is is sleep mode        Not used
 */

#define LED_STARTED              0
#define LED_HEAPALLOCATE         0
#define LED_IRQSENABLED          0
#define LED_STACKCREATED         1
#define LED_INIRQ                2
#define LED_SIGNAL               2
#define LED_ASSERTION            2
#define LED_PANIC                3

/* USB
 * pll clock = AT32_HCLK_FREQUENCY(288MHz)
 * usb clock use pll
 * usb_clk = 288/6 = 48MHz
 */
#define USB_CONFIG_USBDIV       (CRM_MISC2_USBDIV_6P0)  

/* USART1 */

#  define GPIO_USART1_TX        GPIO_USART1_TX_1
#  define GPIO_USART1_RX        GPIO_USART1_RX_1

/* USART2 for RS485 */
#  define GPIO_USART2_TX        GPIO_USART2_TX_2
#  define GPIO_USART2_RX        GPIO_USART2_RX_2
#  define GPIO_USART2_RS485_DIR (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_DRV_STRONG |\
                                GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN4)

/* SPI1 */

#define GPIO_SPI1_MISO    GPIO_SPI1_MISO_2
#define GPIO_SPI1_MOSI    GPIO_SPI1_MOSI_2
#define GPIO_SPI1_SCK     GPIO_SPI1_SCK_2

/* CAN */

#define GPIO_CAN1_RX GPIO_CAN1_RX_3
#define GPIO_CAN1_TX GPIO_CAN1_TX_3

/* ETH */

#define GPIO_ETH_RMII_TX_EN GPIO_ETH_RMII_TX_EN_1    /* PB11 */
#define GPIO_ETH_RMII_TXD0  GPIO_ETH_RMII_TXD0_1     /* PB12 */
#define GPIO_ETH_RMII_TXD1  GPIO_ETH_RMII_TXD1_1     /* PB13 */

/* I2C */

#define GPIO_I2C3_SCL        GPIO_I2C3_SCL_2
#define GPIO_I2C3_SDA        GPIO_I2C3_SDA_2

/* PWM */

#define GPIO_TIM3_CH1OUT GPIO_TIM3_CH1OUT_4
#define GPIO_TIM20_CH1OUT GPIO_TIM20_CH1OUT_2

#endif /* __BOARDS_ARM_AT32_AT32F437-MINN_INCLUDE_BOARD_H */
