/************************************************************************************
 * configs/indium-f7/include/board.h
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *           Mark Olsson <post@markolsson.se>
 *           David Sidrane <david_s5@nscdg.com>
 *           Bob Feretich <bob.feretich@rafresearch.com>
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

#ifndef __CONFIG_INDIUM_F7_INCLUDE_BOARD_H
#define __CONFIG_INDIUM_F7_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#ifdef __KERNEL__
#include "stm32_rcc.h"
#ifdef CONFIG_STM32F7_SDMMC1
#  include "stm32_sdmmc.h"
#endif
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The Nucleo-144  board provides the following clock sources:
 *
 *   MCO: 8 MHz from MCO output of ST-LINK is used as input clock
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

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * If the HSI is selected then the PLL source is 16,000,000.
 * If the HSE is selected then the PLL source is 8,000,000.
 *
 * PLL_VCO = (PLL_source_frequency / PLLM) * PLLN
 * Subject to:
 *
 *     2 <= PLLM <= 63
 *   192 <= PLLN <= 432
 *   192 MHz <= PLL_VCO <= 432MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * Subject to
 *
 *   PLLP = {2, 4, 6, 8}
 *   SYSCLK <= 216 MHz
 *
 * USB OTG FS, SDMMC and RNG Clock = PLL_VCO / PLLQ
 * Subject to
 *   The USB OTG FS requires a 48 MHz clock to work correctly. The SDMMC
 *   and the random number generator need a frequency lower than or equal
 *   to 48 MHz to work correctly.
 *
 * 2 <= PLLQ <= 15
 *
 * Choose highest SYSCLK with USB OTG FS clock = 48 MHz
 *
 */

#if defined CONFIG_INDIUM_CLOCK_HSI 

/* 
 * pll_clock_source = HSI = 16 MHz
 * PLL_VCO = (16,000,000 / 8) * 216 = 432 MHz
 * SYSCLK  = 432 MHz / 2 = 216 MHz
 * USB OTG FS, SDMMC and RNG Clock = 432 MHz / 9 = 48 MHz
 */

#  define STM32_BOARD_USEHSI      1
#  define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#  define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(216)
#  define STM32_VCO_FREQUENCY     ((STM32_HSI_FREQUENCY / 8) * 216)

#else  

/* 
 * pll_clock_source = MCO = 8 MHz
 * PLL_VCO = (8,000,000 / 4) * 216 = 432 MHz
 * SYSCLK  = 432 MHz / 2 = 216 MHz
 * USB OTG FS, SDMMC and RNG Clock = 432 MHz / 9 = 48 MHz
 */

#  define STM32_BOARD_USEHSE      1
#  define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(4)
#  define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(216)
#  define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 4) * 216)

#endif

#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(9)

#define STM32_SYSCLK_FREQUENCY  (STM32_VCO_FREQUENCY / 2)
#define STM32_OTGFS_FREQUENCY   (STM32_VCO_FREQUENCY / 9)

/* Configure factors for  PLLSAI clock */

#define CONFIG_STM32F7_PLLSAI 1
#define STM32_RCC_PLLSAICFGR_PLLSAIN    RCC_PLLSAICFGR_PLLSAIN(192)
#define STM32_RCC_PLLSAICFGR_PLLSAIP    RCC_PLLSAICFGR_PLLSAIP(8)
#define STM32_RCC_PLLSAICFGR_PLLSAIQ    RCC_PLLSAICFGR_PLLSAIQ(4)
#define STM32_RCC_PLLSAICFGR_PLLSAIR    RCC_PLLSAICFGR_PLLSAIR(2)

/* Configure Dedicated Clock Configuration Register */

#define STM32_RCC_DCKCFGR1_PLLI2SDIVQ  RCC_DCKCFGR1_PLLI2SDIVQ(1)
#define STM32_RCC_DCKCFGR1_PLLSAIDIVQ  RCC_DCKCFGR1_PLLSAIDIVQ(1)
#define STM32_RCC_DCKCFGR1_PLLSAIDIVR  RCC_DCKCFGR1_PLLSAIDIVR(0)
#define STM32_RCC_DCKCFGR1_SAI1SRC     RCC_DCKCFGR1_SAI1SEL(0)
#define STM32_RCC_DCKCFGR1_SAI2SRC     RCC_DCKCFGR1_SAI2SEL(0)
#define STM32_RCC_DCKCFGR1_TIMPRESRC   0
#define STM32_RCC_DCKCFGR1_DFSDM1SRC   0
#define STM32_RCC_DCKCFGR1_ADFSDM1SRC  0

/* Configure factors for  PLLI2S clock */

#define CONFIG_STM32F7_PLLI2S 1
#define STM32_RCC_PLLI2SCFGR_PLLI2SN   RCC_PLLI2SCFGR_PLLI2SN(192)
#define STM32_RCC_PLLI2SCFGR_PLLI2SP   RCC_PLLI2SCFGR_PLLI2SP(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SQ   RCC_PLLI2SCFGR_PLLI2SQ(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SR   RCC_PLLI2SCFGR_PLLI2SR(2)

/* Configure Dedicated Clock Configuration Register 2 */

#define STM32_RCC_DCKCFGR2_USART1SRC  RCC_DCKCFGR2_USART1SEL_APB
#define STM32_RCC_DCKCFGR2_USART2SRC  RCC_DCKCFGR2_USART2SEL_APB
#define STM32_RCC_DCKCFGR2_UART4SRC   RCC_DCKCFGR2_UART4SEL_APB
#define STM32_RCC_DCKCFGR2_UART5SRC   RCC_DCKCFGR2_UART5SEL_APB
#define STM32_RCC_DCKCFGR2_USART6SRC  RCC_DCKCFGR2_USART6SEL_APB
#define STM32_RCC_DCKCFGR2_UART7SRC   RCC_DCKCFGR2_UART7SEL_APB
#define STM32_RCC_DCKCFGR2_UART8SRC   RCC_DCKCFGR2_UART8SEL_APB
#define STM32_RCC_DCKCFGR2_I2C1SRC    RCC_DCKCFGR2_I2C1SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C2SRC    RCC_DCKCFGR2_I2C2SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C3SRC    RCC_DCKCFGR2_I2C3SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C4SRC    RCC_DCKCFGR2_I2C4SEL_HSI
#define STM32_RCC_DCKCFGR2_LPTIM1SRC  RCC_DCKCFGR2_LPTIM1SEL_APB
#define STM32_RCC_DCKCFGR2_CECSRC     RCC_DCKCFGR2_CECSEL_HSI
#define STM32_RCC_DCKCFGR2_CK48MSRC   RCC_DCKCFGR2_CK48MSEL_PLL
#define STM32_RCC_DCKCFGR2_SDMMCSRC   RCC_DCKCFGR2_SDMMCSEL_48MHZ
#define STM32_RCC_DCKCFGR2_SDMMC2SRC  RCC_DCKCFGR2_SDMMC2SEL_48MHZ

/* Several prescalers allow the configuration of the two AHB buses, the
 * high-speed APB (APB2) and the low-speed APB (APB1) domains. The maximum
 * frequency of the two AHB buses is 216 MHz while the maximum frequency of
 * the high-speed APB domains is 108 MHz. The maximum allowed frequency of
 * the low-speed APB domain is 54 MHz.
 */

/* AHB clock (HCLK) is SYSCLK (216 MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (54 MHz) */

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

/* APB2 clock (PCLK2) is HCLK/2 (108MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* SDMMC dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(118+2)=400 KHz
 */

#define STM32_SDMMC_INIT_CLKDIV         (118 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(1+2)=16 MHz
 * DMA OFF: SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_MMCXFR_CLKDIV     (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_MMCXFR_CLKDIV     (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(1+2)=16 MHz
 * DMA OFF: SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_SDXFR_CLKDIV      (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_SDXFR_CLKDIV      (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

#if defined(CONFIG_STM32F7_SDMMC2)
#  define GPIO_SDMMC2_D0 GPIO_SDMMC2_D0_1
#  define GPIO_SDMMC2_D1 GPIO_SDMMC2_D1_1
#  define GPIO_SDMMC2_D2 GPIO_SDMMC2_D2_1
#  define GPIO_SDMMC2_D3 GPIO_SDMMC2_D3_1
#endif

/* DMA Channl/Stream Selections *****************************************************/
/* Stream selections for DMA1 */

#define DMAMAP_I2C1_RX STM32_DMA_MAP(DMA1,DMA_STREAM0,DMA_CHAN1)
#define DMAMAP_I2C2_RX STM32_DMA_MAP(DMA1,DMA_STREAM2,DMA_CHAN7)
#define DMAMAP_SPI2_RX STM32_DMA_MAP(DMA1,DMA_STREAM3,DMA_CHAN0)
#define DMAMAP_SPI2_TX STM32_DMA_MAP(DMA1,DMA_STREAM4,DMA_CHAN0)
#define DMAMAP_I2C1_TX STM32_DMA_MAP(DMA1,DMA_STREAM6,DMA_CHAN1)
#define DMAMAP_I2C2_TX STM32_DMA_MAP(DMA1,DMA_STREAM7,DMA_CHAN7)

/* Stream selections for DMA2 */

#define DMAMAP_SPI2_RX   STM32_DMA_MAP(DMA2,DMA_STREAM0,DMA_CHAN3)
#define DMAMAP_USART1_RX STM32_DMA_MAP(DMA2,DMA_STREAM2,DMA_CHAN4)
#define DMAMAP_SPI2_TX   STM32_DMA_MAP(DMA2,DMA_STREAM3,DMA_CHAN3)
#define DMAMAP_ADC1      STM32_DMA_MAP(DMA2,DMA_STREAM4,DMA_CHAN0)
#define DMAMAP_SDMMC1    STM32_DMA_MAP(DMA2,DMA_STREAM6,DMA_CHAN6)
#define DMAMAP_USART1_TX STM32_DMA_MAP(DMA2,DMA_STREAM7,DMA_CHAN4)

/* FLASH wait states
 *
 *  --------- ---------- -----------
 *  VDD       MAX SYSCLK WAIT STATES
 *  --------- ---------- -----------
 *  1.7-2.1 V   180 MHz    8
 *  2.1-2.4 V   216 MHz    9
 *  2.4-2.7 V   216 MHz    8
 *  2.7-3.6 V   216 MHz    7
 *  --------- ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 7

/* LED definitions ******************************************************************/
/* The Indium-F7 board has three software controllable LEDs; LD1 a Green LED, LD2 a Blue
 * LED and LD3 a Red LED.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any way.
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
 * include/board.h and src/stm32_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        Red   Green  Blue
 *   ----------------------  --------------------------  ----- ------ ---- */

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

/* Button definitions ***************************************************************/
/* The Indium-F7 supports one button:  Pushbutton B1, labeled "SW2", is
 * connected to GPIO PC15.  A high value will be sensed when the button is depressed.
 * If the button is held down for >3 seconds, the board should power-off.
 * For shorter periods, the button is interpreted as "selection" upon release.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ************************************************/

#if defined(CONFIG_INDIUM_CONSOLE_ARDUINO)
/* USART6:
 *
 * These configurations assume that you are using a standard Arduio RS-232 shield
 * with the serial interface with RX on pin D0 and TX on pin D1:
 *
 *   -------- ---------------
 *               STM32F7
 *   ARDUIONO FUNCTION  GPIO
 *   -- ----- --------- -----
 *   DO RX    USART6_RX PG9
 *   D1 TX    USART6_TX PG14
 *   -- ----- --------- -----
 */

 # define GPIO_USART6_RX GPIO_USART6_RX_2
 # define GPIO_USART6_TX GPIO_USART6_TX_2
#endif

/* USART3:
 * Use  USART3 and the USB virtual COM port
 */

#if defined(CONFIG_INDIUM_CONSOLE_VIRTUAL)
 # define GPIO_USART3_RX GPIO_USART3_RX_3
 # define GPIO_USART3_TX GPIO_USART3_TX_3
#endif

#if defined(CONFIG_INDIUM_CONSOLE_MORPHO_UART4)
/* UART4:
 *
 * This configuration assumes that you disabled Ethernet MII clocking
 * by removing SB13 to free PA1.
 *
 *   -------- ---------------
 *               STM32F7
 *   Pin      FUNCTION  GPIO
 *   -------  --------- -----
 *   CN11 30  UART4_RX  PA1
 *   CN11 28  UART4_TX  PA0
 *   -------  --------- -----
 */

 # define GPIO_UART4_RX GPIO_UART4_RX_1
 # define GPIO_UART4_TX GPIO_UART4_TX_1

#endif

/* DMA channels *************************************************************/
/* ADC */

#define ADC1_DMA_CHAN DMAMAP_ADC1_1
#define ADC2_DMA_CHAN DMAMAP_ADC2_1
#define ADC3_DMA_CHAN DMAMAP_ADC3_1

/* Standard Indium pin maps */

/* SPI
 *                    Indium   Nucleo
 *  PA6   SPI1_MISO    22      CN12-13
 *  PA7   SPI1_MOSI    23      CN12-15
 *  PA5   SPI1_SCK     21      CN12-11
 *
 *  PB14  SPI2_MISO    35      CN12-28
 *  PB15  SPI2_MOSI    36      CN12-26
 *  PB13  SPI2_SCK     34      CN12-30
 */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_3

/* I2C
 *                    Indium   Nucleo
 *  PB6   I2C1_SCL    58       CN10-13
 *  PB7   I2C1_SDA    59       CN11-21
 * 
 *  PB10  I2C2_SCL    28       CN11-51
 *  PB11  I2C2_SDA    29       CN12-18
 */

#define GPIO_I2C1_SCL GPIO_I2C1_SCL_1
#define GPIO_I2C1_SDA GPIO_I2C1_SDA_1

#define GPIO_I2C2_SCL GPIO_I2C2_SCL_1
#define GPIO_I2C2_SDA GPIO_I2C2_SDA_1

/* CAN
 *                    Indium   Nucleo
 *  PB8   CAN1_RX     61       CN12-3
 *  PB9   CAN1_TX     62       CN12-5
 */

#define GPIO_CAN1_RX  GPIO_CAN1_RX_2
#define GPIO_CAN1_TX  GPIO_CAN1_TX_2

/* USART
 *                    Indium   Nucleo
 *  PA12  USART1_RTS  45       CN12-12
 *  PA11  USART1_CTS  44       CN12-14
 *  PA10  USART1_RX   43       CN12-33
 *  PA9   USART1_TX   42       CN12-21
 *                  
 *  PA3   USART2_RX   17       CN12-37
 *  PA2   USART2_TX   16       CN12-35
 */

/* define GPIO_USART1_RTS  (GPIO_ALT|GPIO_AF7|GPIO_PORTA|GPIO_PIN12) is default. */
/* define GPIO_USART1_CTS  (GPIO_ALT|GPIO_AF7|GPIO_PORTA|GPIO_PIN11) is default. */

#define GPIO_USART1_RX     GPIO_USART1_RX_1
#define GPIO_USART1_TX     GPIO_USART1_TX_1

#define GPIO_USART2_RX     GPIO_USART2_RX_1
#define GPIO_USART2_TX     GPIO_USART2_TX_1

#endif  /* __CONFIG_INDIUM_F7_INCLUDE_BOARD_H */
