/****************************************************************************
 * boards/arm/stm32/axoloti/include/board.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Jason T. Harris <sirmanlypowers@gmail.com>
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
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32_AXOLOTI_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_AXOLOTI_INCLUDE_BOARD_H

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

/****************************************************************************
 * Clocking
 * The Axoloti board has an external 8MHz crystal.
 * The SoC can run at 180MHz, but the required USB clock of 48MHz cannot be
 * configured at that system clock rate, so the core clock is 168MHz.
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

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK    /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY    /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4   /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2   /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/****************************************************************************
 * LED Definitions
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2
#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_RED     BOARD_LED2

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/****************************************************************************
 * Button Definitions
 * There are two buttons on the axoloti, one of them is GPIO connected. The other
 * is a reset button and is not under software control.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/****************************************************************************
 * UARTs
 * The MIDI in/out ports of the axoloti are connected on USART6.
 * It maybe convenient to run a serial port connected to the header pins,
 * so we can optionally use USART1 for that.
 */

/* USART1 - console on header pins */

#define GPIO_USART1_RX GPIO_USART1_RX_2 /* AF7, PB7 */
#define GPIO_USART1_TX GPIO_USART1_TX_2 /* AF7, PB6 */

/* USART6 - midi in/out */

#define GPIO_USART6_RX (GPIO_ALT|GPIO_AF8|GPIO_PORTG|GPIO_PIN9| \
                        GPIO_PULLUP|GPIO_SPEED_2MHz|GPIO_PUSHPULL)

#define GPIO_USART6_TX (GPIO_ALT|GPIO_AF8|GPIO_PORTG|GPIO_PIN14| \
                        GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OPENDRAIN)

/****************************************************************************
 * I2C Bus
 * Turn on the internal pullups since there are no external pullups.
 */

/* I2C1 - for external devices */

#define GPIO_I2C1_SCL (GPIO_ALT|GPIO_AF4|GPIO_PORTB|GPIO_PIN8| \
                       GPIO_SPEED_2MHz|GPIO_OPENDRAIN|GPIO_PULLUP)

#define GPIO_I2C1_SDA  (GPIO_ALT|GPIO_AF4|GPIO_PORTB|GPIO_PIN9| \
                       GPIO_SPEED_2MHz|GPIO_OPENDRAIN|GPIO_PULLUP)

/* I2C3 - for the ADAU1961 codec */

#define GPIO_I2C3_SCL (GPIO_ALT|GPIO_AF4|GPIO_PORTH|GPIO_PIN7| \
                       GPIO_SPEED_2MHz|GPIO_OPENDRAIN|GPIO_PULLUP)

#define GPIO_I2C3_SDA (GPIO_ALT|GPIO_AF4|GPIO_PORTH|GPIO_PIN8| \
                       GPIO_SPEED_2MHz|GPIO_OPENDRAIN|GPIO_PULLUP)

/****************************************************************************
 * SAI Bus
 * Used with the ADAU1961 CODEC
 * PE3_SAI1_SD_B (GPIO_SAI1_SD_B_1)
 * PE4_SAI1_FS_A (GPIO_SAI1_FS_A)
 * PE5_SAI1_SCK_A (GPIO_SAI1_SCK_A)
 * PE6_SAI1_SD_A (GPIO_SAI1_SD_A_2)
 * PA8_MCO1
 */

#define GPIO_SAI1_SD_B GPIO_SAI1_SD_B_1 /* AF6, PE3 */
#define GPIO_SAI1_SD_A GPIO_SAI1_SD_A_2 /* AF6, PE6 */

#define STM32_SAI1_FREQUENCY (48000 * 2 * 256)  /* TODO ?? */

/* DAC DMA to Codec
 * dma 2, stream 1, channel 0
 * memory to peripheral
 * 32 bits
 */
#define DMACHAN_SAI1_A DMAMAP_SAI1_A_1

/* ADC DMA from Codec
 * dma 2, stream 4, channel 1,
 * peripheral to memory
 * 32 bits
 */
#define DMACHAN_SAI1_B DMAMAP_SAI1_B_2

/****************************************************************************
 * SDIO
 * Used for the SD card interface.
 * d0 (AF12, PC8)
 * d1 (AF12, PC9)
 * d2 (AF12, PC10)
 * d3 (AF12, PC11)
 * clk (AF12, PC12)
 * cmd (AF12, PD2)
 * cd1 PD13
 */

/* SDIO dividers. Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(118+2)=400 KHz
 */

#define SDIO_INIT_CLKDIV (118 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* dma 2, stream 6, channel 4 */

#define DMAMAP_SDIO DMAMAP_SDIO_2

#endif /* __BOARDS_ARM_STM32_AXOLOTI_INCLUDE_BOARD_H */
