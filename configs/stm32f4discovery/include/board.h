/************************************************************************************
 * configs/stm32f4discovery/include/board.h
 *
 *   Copyright (C) 2012, 2014-2016, 2018 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIG_STM32F4DISCOVERY_INCLUDE_BOARD_H
#define __CONFIG_STM32F4DISCOVERY_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The STM32F4 Discovery board features a single 8MHz crystal.  Space is provided
 * for a 32kHz RTC backup crystal, but it is not stuffed.
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 168000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 8000000      (STM32_BOARD_XTAL)
 *   PLLM                          : 8            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
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
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

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

/* LED definitions ******************************************************************/
/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_LED4        3
#define BOARD_NLEDS       4

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_ORANGE  BOARD_LED2
#define BOARD_LED_RED     BOARD_LED3
#define BOARD_LED_BLUE    BOARD_LED4

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)
#define BOARD_LED4_BIT    (1 << BOARD_LED4)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 4 LEDs on board the
 * stm32f4discovery.  The following definitions describe how NuttX controls the LEDs:
 */

#define LED_STARTED       0  /* LED1 */
#define LED_HEAPALLOCATE  1  /* LED2 */
#define LED_IRQSENABLED   2  /* LED1 + LED2 */
#define LED_STACKCREATED  3  /* LED3 */
#define LED_INIRQ         4  /* LED1 + LED3 */
#define LED_SIGNAL        5  /* LED2 + LED3 */
#define LED_ASSERTION     6  /* LED1 + LED2 + LED3 */
#define LED_PANIC         7  /* N/C  + N/C  + N/C + LED4 */

/* Button definitions ***************************************************************/
/* The STM32F4 Discovery supports one button: */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ************************************************/
/* CAN */

#ifndef CONFIG_STM32_FSMC
#  define GPIO_CAN1_RX GPIO_CAN1_RX_3
#  define GPIO_CAN1_TX GPIO_CAN1_TX_3
#endif

#ifndef CONFIG_STM32_ETHMAC
#  define GPIO_CAN2_RX GPIO_CAN2_RX_1
#  define GPIO_CAN2_TX GPIO_CAN2_TX_1
#endif

/* USART2:
 *
 * The STM32F4 Discovery has no on-board serial devices, but the console is
 * brought out to PA2 (TX) and PA3 (RX) for connection to an external serial
 * device. (See the README.txt file for other options)
 *
 * These pins selections, however, conflict with pin usage on the STM32F4DIS-BB.
 */

#ifndef CONFIG_STM32F4DISBB
#  define GPIO_USART2_RX  GPIO_USART2_RX_1     /* PA3, P1 pin 13 */
#  define GPIO_USART2_TX  GPIO_USART2_TX_1     /* PA2, P1 pin 14 */
#  define GPIO_USART2_CTS GPIO_USART2_CTS_1    /* PA0, P1 pin 11 */
#  define GPIO_USART2_RTS GPIO_USART2_RTS_1    /* PA1, P1 pin 12 (conflict with USER button) */
#endif

/* USART3:
 *
 * Used in pseudoterm configuration and also with the BT860 HCI UART.
 * RTS/CTS Flow control support is needed by the HCI UART.
 *
 * There are conflicts with the STM32F4DIS-BB Ethernet in this configuration
 * when Ethernet is enabled:
 *
 *   PB-11 conflicts with Ethernet TXEN
 *   PB-13 conflicts with Ethernet TXD1
 *
 * UART3 TXD and RXD are available on CON4 PD8 and PD8 of the STM32F4DIS-BB,
 * respectively, but not CTS or RTS.  For now we assume that Ethernet is not
 * enabled if USART3 is used in a configuration with the STM32F4DIS-BB.
 */

#define GPIO_USART3_TX    GPIO_USART3_TX_1     /* PB10, P1 pin 34 (also MP45DT02 CLK_IN) */
#define GPIO_USART3_RX    GPIO_USART3_RX_1     /* PB11, P1 pin 35 */
#define GPIO_USART3_CTS   GPIO_USART3_CTS_1    /* PB13, P1 pin 37 */
#define GPIO_USART3_RTS   GPIO_USART3_RTS_1    /* PB14, P1 pin 38 */

/* USART6:
 *
 * The STM32F4DIS-BB base board provides RS-232 drivers and a DB9 connector
 * for USART6.  This is the preferred serial console for use with the STM32F4DIS-BB.
 *
 * NOTE: CTS and RTS are not brought out to the RS-232 connector on the baseboard.
 */

#define GPIO_USART6_RX    GPIO_USART6_RX_1     /* PC7 (also I2S3_MCK and P2 pin 48) */
#define GPIO_USART6_TX    GPIO_USART6_TX_1     /* PC6 (also P2 pin 47) */

/* PWM
 *
 * The STM32F4 Discovery has no real on-board PWM devices, but the board can be
 * configured to output a pulse train using TIM4 CH2 on PD13.
 */

#define GPIO_TIM4_CH2OUT  GPIO_TIM4_CH2OUT_2

/* RGB LED
 *
 * R = TIM1 CH1 on PE9 | G = TIM2 CH2 on PA1 | B = TIM3 CH3 on PB0
 */

#define GPIO_TIM1_CH1OUT  GPIO_TIM1_CH1OUT_2
#define GPIO_TIM2_CH2OUT  GPIO_TIM2_CH2OUT_1
#define GPIO_TIM3_CH3OUT  GPIO_TIM3_CH3OUT_1

/* SPI - There is a MEMS device on SPI1 using these pins: */

#define GPIO_SPI1_MISO    GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI    GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK     GPIO_SPI1_SCK_1

/* SPI DMA -- As used for I2S DMA transfer with the audio configuration */

#define DMACHAN_SPI1_RX   DMAMAP_SPI1_RX_1
#define DMACHAN_SPI1_TX   DMAMAP_SPI1_TX_1

/* SPI2 - Test MAX31855 on SPI2 PB10 = SCK, PB14 = MISO */

#define GPIO_SPI2_MISO    GPIO_SPI2_MISO_1
#define GPIO_SPI2_MOSI    GPIO_SPI2_MOSI_1
#define GPIO_SPI2_SCK     GPIO_SPI2_SCK_1

/* SPI3 DMA -- As used for I2S DMA transfer with the audio configuration */

#define GPIO_SPI3_MISO    GPIO_SPI3_MISO_1
#define GPIO_SPI3_MOSI    GPIO_SPI3_MOSI_1
#define GPIO_SPI3_SCK     GPIO_SPI3_SCK_1

#define DMACHAN_SPI3_RX   DMAMAP_SPI3_RX_1
#define DMACHAN_SPI3_TX   DMAMAP_SPI3_TX_1

/* I2S3 - CS43L22 configuration uses I2S3 */

#define GPIO_I2S3_SD      GPIO_I2S3_SD_2
#define GPIO_I2S3_CK      GPIO_I2S3_CK_2
#define GPIO_I2S3_WS      GPIO_I2S3_WS_1

#define DMACHAN_I2S3_RX   DMAMAP_SPI3_RX_2
#define DMACHAN_I2S3_TX   DMAMAP_SPI3_TX_2

/* I2C.  Only I2C1 is available on the stm32f4discovery.  I2C1_SCL and I2C1_SDA are
 * available on the following pins:
 *
 * - PB6  is I2C1_SCL
 * - PB9  is I2C1_SDA
 */

#define GPIO_I2C1_SCL     GPIO_I2C1_SCL_1
#define GPIO_I2C1_SDA     GPIO_I2C1_SDA_2

/* Timer Inputs/Outputs (see the README.txt file for options) */

#define GPIO_TIM2_CH1IN   GPIO_TIM2_CH1IN_2
#define GPIO_TIM2_CH2IN   GPIO_TIM2_CH2IN_1

#define GPIO_TIM8_CH1IN   GPIO_TIM8_CH1IN_1
#define GPIO_TIM8_CH2IN   GPIO_TIM8_CH2IN_1

/* Ethernet *************************************************************************/

#if defined(CONFIG_STM32F4DISBB) && defined(CONFIG_STM32_ETHMAC)
  /* RMII interface to the LAN8720 PHY */

#  ifndef CONFIG_STM32_RMII
#    error CONFIG_STM32_RMII must be defined
#  endif

  /* Clocking is provided by an external 25Mhz XTAL */

#  ifndef CONFIG_STM32_RMII_EXTCLK
#    error CONFIG_STM32_RMII_EXTCLK must be defined
#  endif

  /* Pin disambiguation */

#  define GPIO_ETH_RMII_TX_EN GPIO_ETH_RMII_TX_EN_1
#  define GPIO_ETH_RMII_TXD0  GPIO_ETH_RMII_TXD0_1
#  define GPIO_ETH_RMII_TXD1  GPIO_ETH_RMII_TXD1_1
#  define GPIO_ETH_PPS_OUT    GPIO_ETH_PPS_OUT_1

#endif

/* DMA Channl/Stream Selections *****************************************************/
/* Stream selections are arbitrary for now but might become important in the future
 * if we set aside more DMA channels/streams.
 *
 * SDIO DMA
 *   DMAMAP_SDIO_1 = Channel 4, Stream 3
 *   DMAMAP_SDIO_2 = Channel 4, Stream 6
 */

#define DMAMAP_SDIO DMAMAP_SDIO_1

#endif  /* __CONFIG_STM32F4DISCOVERY_INCLUDE_BOARD_H */
