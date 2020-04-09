/****************************************************************************
 * boards/arm/stm32f4/nucleo-f429zi/include/board.h
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

#ifndef __BOARDS_ARM_STM32F4_NUCLEO_F429ZI_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F4_NUCLEO_F429ZI_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/* Do not include STM32 F4 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The STM32F4 Discovery board features a single 8MHz crystal. Space is
 * provided for a 32kHz RTC backup crystal, but it is not stuffed.
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 180000000    Determined by PLL config
 *   HCLK(Hz)                      : 180000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 8000000      (STM32_BOARD_XTAL)
 *   PLLM                          : 8            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator output voltage : Scale1 mode  Needed for highspeed SYSCLK
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
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY

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
#define BOARD_TIM2_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM3_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM4_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM5_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM6_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM7_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY

/* DMA Channel/Stream Selections ********************************************/

/* Stream selections are arbitrary for now but might become important in the
 * future if we set aside more DMA channels/streams.
 *
 * SDMMC DMA is on DMA2
 *
 * SDMMC1 DMA
 *   DMAMAP_SDMMC1_1 = Channel 4, Stream 3
 *   DMAMAP_SDMMC1_2 = Channel 4, Stream 6
 *
 * SDMMC2 DMA
 *   DMAMAP_SDMMC2_1 = Channel 11, Stream 0
 *   DMAMAP_SDMMC3_2 = Channel 11, Stream 5
 */

#define DMAMAP_SDMMC1  DMAMAP_SDMMC1_1
#define DMAMAP_SDMMC2  DMAMAP_SDMMC2_1

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

/* LED definitions **********************************************************/

/* The Nucleo-144 board has numerous LEDs but only three, LD1 a Green LED,
 * LD2 a Blue LED and LD3 a Red LED, that can be controlled by software.
 * The following definitions assume the default Solder Bridges are installed.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs
 * in any way.
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
 * include/board.h and src/stm32_leds.c. The LEDs are used to encode
 * OS-related events as follows:
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

/* The STM32F4 Discovery supports one button:  Pushbutton B1, labeled "User",
 * is connected to GPIO PI11.
 * A high value will be sensed when the button is depressed.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* TIM */

#define GPIO_TIM1_CH1OUT GPIO_TIM1_CH1OUT_1
#define GPIO_TIM2_CH1OUT GPIO_TIM2_CH1OUT_1
#define GPIO_TIM3_CH1OUT GPIO_TIM3_CH1OUT_1
#define GPIO_TIM4_CH1OUT GPIO_TIM4_CH1OUT_1

#if defined(CONFIG_NUCLEO_F429ZI_CONSOLE_ARDUINO)

/* USART6:
 *
 * These configurations assume that you are using a standard Arduio RS-232
 * shield with the serial interface with RX on pin D0 and TX on pin D1:
 *
 *   -------- ---------------
 *               STM32F4
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

#if defined(CONFIG_NUCLEO_F429ZI_CONSOLE_VIRTUAL)
 # define GPIO_USART3_RX GPIO_USART3_RX_3
 # define GPIO_USART3_TX GPIO_USART3_TX_3
#endif

/* DMA channels *************************************************************/

/* ADC */

#define ADC1_DMA_CHAN DMAMAP_ADC1_1
#define ADC2_DMA_CHAN DMAMAP_ADC2_1
#define ADC3_DMA_CHAN DMAMAP_ADC3_1

/* SPI
 *
 *
 *  PA6   SPI1_MISO CN12-13
 *  PA7   SPI1_MOSI CN12-15
 *  PA5   SPI1_SCK  CN12-11
 *
 *  PB14  SPI2_MISO CN12-28
 *  PB15  SPI2_MOSI CN12-26
 *  PB13  SPI2_SCK  CN12-30
 *
 *  PB4   SPI3_MISO CN12-27
 *  PB5   SPI3_MOSI CN12-29
 *  PB3   SPI3_SCK  CN12-31
 */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_3

#define GPIO_SPI3_MISO   GPIO_SPI3_MISO_1
#define GPIO_SPI3_MOSI   GPIO_SPI3_MOSI_2
#define GPIO_SPI3_SCK    GPIO_SPI3_SCK_1

/* I2C
 *
 *
 *  PB8   I2C1_SCL CN12-3
 *  PB9   I2C1_SDA CN12-5

 *  PB10   I2C2_SCL CN11-51
 *  PB11 I2C2_SDA CN12-18
 *
 *  PA8   I2C3_SCL CN12-23
 *  PC9   I2C3_SDA CN12-1
 *
 */

#define GPIO_I2C1_SCL GPIO_I2C1_SCL_2
#define GPIO_I2C1_SDA GPIO_I2C1_SDA_2

#define GPIO_I2C2_SCL GPIO_I2C2_SCL_1
#define GPIO_I2C2_SDA GPIO_I2C2_SDA_1

#define GPIO_I2C3_SCL GPIO_I2C3_SCL_1
#define GPIO_I2C3_SDA GPIO_I2C3_SDA_1

/* The STM32 F4 connects to a SMSC LAN8742A PHY using these pins:
 *
 *   STM32 F4 BOARD        LAN8742A
 *   GPIO     SIGNAL       PIN NAME
 *   -------- ------------ -------------
 *   PG11     RMII_TX_EN   TXEN
 *   PG13     RMII_TXD0    TXD0
 *   PB13     RMII_TXD1    TXD1
 *   PC4      RMII_RXD0    RXD0/MODE0
 *   PC5      RMII_RXD1    RXD1/MODE1
 *   PG2      RMII_RXER    RXER/PHYAD0 -- Not used
 *   PA7      RMII_CRS_DV  CRS_DV/MODE2
 *   PC1      RMII_MDC     MDC
 *   PA2      RMII_MDIO    MDIO
 *   N/A      NRST         nRST
 *   PA1      RMII_REF_CLK nINT/REFCLK0
 *   N/A      OSC_25M      XTAL1/CLKIN
 *
 * The PHY address is either 0 or 1, depending on the state of PG2 on reset.
 * PG2 is not controlled but appears to result in a PHY address of 0.
 */

#define GPIO_ETH_RMII_TX_EN   GPIO_ETH_RMII_TX_EN_2
#define GPIO_ETH_RMII_TXD0    GPIO_ETH_RMII_TXD0_2
#define GPIO_ETH_RMII_TXD1    GPIO_ETH_RMII_TXD1_1

#endif /* __BOARDS_ARM_STM32F4_NUCLEO_F429ZI_INCLUDE_BOARD_H */
