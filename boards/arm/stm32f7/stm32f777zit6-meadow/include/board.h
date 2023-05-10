/****************************************************************************
 * boards/arm/stm32f7/stm32f777zit6-meadow/include/board.h
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

#ifndef __BOARDS_ARM_STM32F777ZIT6_MEADOW_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F777ZIT6_MEADOW_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/* Do not include STM32F7 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The STM32F7 Discovery board provides the following clock sources:
 *
 *   X2:  25 MHz oscillator for STM32F777ZIT6 microcontroller
 *   and Ethernet PHY.
 *   X1:  32.768 KHz crystal for STM32F777ZIT6 embedded RTC
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: On-board crystal frequency is 25MHz
 *   LSE: 32.768 kHz
 */

#define STM32_BOARD_XTAL        25000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE = 25,000,000
 *
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
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
 */

#if defined(CONFIG_STM32F7_OTGFS)
/* USB OTG FS clock (= SDMMCCLK = RNGCLK) must be 48 MHz
 *
 * PLL_VCO = (25,000,000 / 25) * 384 = 384 MHz
 * SYSCLK  = 384 MHz / 2 = 192 MHz
 * USB OTG FS, SDMMC and RNG Clock = 384 MHz / 8 = 48MHz
 * DSI CLK = PLL_VCO / PLLR = 384 / 7 = 54,86 MHz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(25)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(384)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(8)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(7)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 25) * 384)
#define STM32_SYSCLK_FREQUENCY  (STM32_VCO_FREQUENCY / 2)
#define STM32_OTGFS_FREQUENCY   (STM32_VCO_FREQUENCY / 8)

#elif defined(CONFIG_STM32F7_SDMMC1) || defined(CONFIG_STM32F7_SDMMC2) || defined(CONFIG_STM32F7_RNG)
/* SDMMCCLK (= USB OTG FS clock = RNGCLK) should be <= 48MHz
 *
 * PLL_VCO = (25,000,000 / 25) * 432 = 432 MHz
 * SYSCLK  = 432 MHz / 2 = 216 MHz
 * USB OTG FS, SDMMC and RNG Clock = 432 MHz / 10 = 43.2 MHz
 * DSI CLK = PLL_VCO / PLLR = 432 / 8 = 54 MHz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(25)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(432)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(10)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(8)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 25) * 432)
#define STM32_SYSCLK_FREQUENCY  (STM32_VCO_FREQUENCY / 2)
#define STM32_OTGFS_FREQUENCY   (STM32_VCO_FREQUENCY / 10)

#else
/* No restrictions by OTGFS
 *
 * PLL_VCO = (25,000,000 / 25) * 432 = 432 MHz
 * SYSCLK  = 432 MHz / 2 = 216 MHz
 * USB OTG FS, SDMMC and RNG Clock = 432 MHz / 10 = 43.2 MHz
 * DSI CLK = PLL_VCO / PLLR = 432 / 8 = 54 MHz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(25)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(432)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(10)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(8)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 25) * 432)
#define STM32_SYSCLK_FREQUENCY  (STM32_VCO_FREQUENCY / 2)
#define STM32_OTGFS_FREQUENCY   (STM32_VCO_FREQUENCY / 10)
#endif

/* Configure factors for  PLLSAI clock */

#define STM32_RCC_PLLSAICFGR_PLLSAIN    RCC_PLLSAICFGR_PLLSAIN(192)
#define STM32_RCC_PLLSAICFGR_PLLSAIP    RCC_PLLSAICFGR_PLLSAIP(2)
#define STM32_RCC_PLLSAICFGR_PLLSAIQ    RCC_PLLSAICFGR_PLLSAIQ(2)
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
#define STM32_RCC_DCKCFGR2_CK48MSRC   RCC_DCKCFGR2_CK48MSEL_PLLSAI
#define STM32_RCC_DCKCFGR2_SDMMCSRC   RCC_DCKCFGR2_SDMMCSEL_48MHZ
#define STM32_RCC_DCKCFGR2_SDMMC2SRC  RCC_DCKCFGR2_SDMMC2SEL_48MHZ
#define STM32_RCC_DCKCFGR2_DSISRC     RCC_DCKCFGR2_DSISEL_PHY

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

#define BOARD_AHB_FREQUENCY     STM32_HCLK_FREQUENCY

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

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(118+2)=400 KHz
 */

#define STM32_SDMMC_INIT_CLKDIV      (118 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_MMCXFR_CLKDIV  (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_MMCXFR_CLKDIV  (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_SDXFR_CLKDIV   (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_SDXFR_CLKDIV   (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

/* LED definitions **********************************************************/

/* The STM32F777ZIT6-MEADOW board has numerous LEDs but only one, LD1
 * located near the reset button, that can be controlled by software
 * (LD2 is a power indicator, LD3-6
 * indicate USB status, LD7 is controlled by the ST-Link).
 *
 * LD1 is controlled by PI1 which is also the SPI2_SCK at the Arduino
 * interface.
 * One end of LD1 is grounded so a high output on PI1 will illuminate
 * the LED.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the
 * LEDs in any way.
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
 * include/board.h and src/stm32_leds.c. The LEDs are used
 * to encode OS-related events as follows:
 *
 *   SYMBOL              Meaning                 LD1
 *   ------------------- ----------------------- ------
 *   LED_STARTED         NuttX has been started  OFF
 *   LED_HEAPALLOCATE    Heap has been allocated OFF
 *   LED_IRQSENABLED     Interrupts enabled      OFF
 *   LED_STACKCREATED    Idle stack created      ON
 *   LED_INIRQ           In an interrupt         N/C
 *   LED_SIGNAL          In a signal handler     N/C
 *   LED_ASSERTION       An assertion failed     N/C
 *   LED_PANIC           The system has crashed  FLASH
 *
 * Thus is LD1 is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If LD1 is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
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

/* Button definitions *******************************************************/

/* The STM32F7 Discovery supports one button:
 *  Pushbutton B1, labelled "User", is connected to GPIO PA0.
 *  A high value will be sensed when the button is depressed.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* LED definitions **********************************************************/

/* LCD DISPLAY
 * (work in progress as of 2017 07 19)
 */

#define	BOARD_LTDC_WIDTH        800
#define	BOARD_LTDC_HEIGHT       472

#define	BOARD_LTDC_HSYNC        10
#define	BOARD_LTDC_HFP          10
#define	BOARD_LTDC_HBP          20
#define	BOARD_LTDC_VSYNC        2
#define	BOARD_LTDC_VFP          4
#define	BOARD_LTDC_VBP          2

#define	BOARD_LTDC_GCR_PCPOL    0
#define	BOARD_LTDC_GCR_DEPOL    0
#define	BOARD_LTDC_GCR_VSPOL    0
#define	BOARD_LTDC_GCR_HSPOL    0

/* DMA Channel/Stream Selections ********************************************/

/* SDMMC */

/* Stream selections are arbitrary for now but might become important in the
 * future if we set aside more DMA channels/streams.
 *
 * SDIO DMA
 *   DMAMAP_SDMMC1_1 = Channel 4, Stream 3
 *   DMAMAP_SDMMC1_2 = Channel 4, Stream 6
 *
 *   DMAMAP_SDMMC2_1 = Channel 11, Stream 0
 *   DMAMAP_SDMMC2_2 = Channel 11, Stream 5
 */

#define DMAMAP_SDMMC2  DMAMAP_SDMMC2_1

/* Alternate function pin selections ****************************************/

/* USART6:
 *
 * These configurations assume that you are using a standard Arduio
 * RS-232 shield with the serial interface with RX on pin D0 and TX
 * on pin D1:
 *
 *   -------- ---------------
 *               STM32F7
 *   ARDUINO  FUNCTION  GPIO
 *   -- ----- --------- -----
 *   DO RX    USART6_RX PC7
 *   D1 TX    USART6_TX PC6
 *   -- ----- --------- -----
 */

#define GPIO_USART6_RX (GPIO_USART6_RX_1|GPIO_SPEED_100MHz)
#define GPIO_USART6_TX (GPIO_USART6_TX_1|GPIO_SPEED_100MHz)

/* USART1:
 * USART1 is connected to the "Virtual Com Port" lines
 * of the ST-LINK controller.
 *
 *   -------- ---------------
 *               STM32F7
 *   SIGNAME  FUNCTION  GPIO
 *   -- ----- --------- -----
 *   VCP_RX   USART1_RX PA10
 *   VCP_TX   USART1_TX PA9
 *   -- ----- --------- -----
 */

#define GPIO_USART1_RX (GPIO_USART1_RX_3|GPIO_SPEED_100MHz)
#define GPIO_USART1_TX (GPIO_USART1_TX_3|GPIO_SPEED_100MHz)

/* The STM32 F7 connects to a SMSC LAN8742A PHY using these pins:
 *
 *   STM32 F7 BOARD        LAN8742A
 *   GPIO     SIGNAL       PIN NAME
 *   -------- ------------ -------------
 *   PG11     RMII_TX_EN   TXEN
 *   PG13     RMII_TXD0    TXD0
 *   PG14     RMII_TXD1    TXD1
 *   PC4      RMII_RXD0    RXD0/MODE0
 *   PC5      RMII_RXD1    RXD1/MODE1
 *   PD5      RMII_RXER    RXER/PHYAD0
 *   PA7      RMII_CRS_DV  CRS_DV/MODE2
 *   PC1      RMII_MDC     MDC
 *   PA2      RMII_MDIO    MDIO
 *   N/A      NRST         nRST
 *   PA1      RMII_REF_CLK nINT/REFCLK0
 *   N/A      OSC_25M      XTAL1/CLKIN
 *
 * The PHY address is 0, since RMII_RXER/PHYAD0 features a pull down.
 * After reset, RMII_RXER/PHYAD0 switches to the RXER function,
 * receive errors can be detected using GPIO pin PD5
 */

#define GPIO_ETH_MDC          (GPIO_ETH_MDC_0|GPIO_SPEED_100MHz)
#define GPIO_ETH_MDIO         (GPIO_ETH_MDIO_0|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_CRS_DV  (GPIO_ETH_RMII_CRS_DV_0|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_REF_CLK (GPIO_ETH_RMII_REF_CLK_0|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_RXD0    (GPIO_ETH_RMII_RXD0_0|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_RXD1    (GPIO_ETH_RMII_RXD1_0|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_TX_EN   (GPIO_ETH_RMII_TX_EN_2|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_TXD0    (GPIO_ETH_RMII_TXD0_2|GPIO_SPEED_100MHz)
#define GPIO_ETH_RMII_TXD1    (GPIO_ETH_RMII_TXD1_2|GPIO_SPEED_100MHz)

/* I2C Mapping
 * I2C #4 is connected to the LCD daughter board
 * and the WM8994 audio codec.
 *
 * I2C4_SCL - PD12
 * I2C4_SDA - PB7
 */
#define GPIO_I2C4_SCL        (GPIO_I2C4_SCL_1|GPIO_SPEED_50MHz)
#define GPIO_I2C4_SDA        (GPIO_I2C4_SDA_5|GPIO_SPEED_50MHz)

/* QSPI Mapping  */

#define GPIO_QSPI_CS  (GPIO_QUADSPI_BK1_NCS_2 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_IO0 (GPIO_QUADSPI_BK1_IO0_3 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_IO1 (GPIO_QUADSPI_BK1_IO1_3 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_IO2 (GPIO_QUADSPI_BK1_IO2_1 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_IO3 (GPIO_QUADSPI_BK1_IO3_2 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_SCK (GPIO_QUADSPI_CLK_1     | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)

/* SDMMC2 Pin mapping
 *
 * D0 - PG9
 * D1 - PG10
 * D2 - PB3
 * D3 - PB4
 */

#define GPIO_SDMMC2_CK  (GPIO_SDMMC2_CK_0|GPIO_SPEED_50MHz)
#define GPIO_SDMMC2_CMD (GPIO_SDMMC2_CMD_0|GPIO_SPEED_50MHz)
#define GPIO_SDMMC2_D0  (GPIO_SDMMC2_D0_2|GPIO_SPEED_50MHz)
#define GPIO_SDMMC2_D1  (GPIO_SDMMC2_D1_2|GPIO_SPEED_50MHz)
#define GPIO_SDMMC2_D2  (GPIO_SDMMC2_D2_1|GPIO_SPEED_50MHz)
#define GPIO_SDMMC2_D3  (GPIO_SDMMC2_D3_1|GPIO_SPEED_50MHz)

/* FMC - SDRAM */

#define GPIO_FMC_SDCKE1 (GPIO_FMC_SDCKE1_1|GPIO_SPEED_100MHz)
#define GPIO_FMC_SDNE1  (GPIO_FMC_SDNE1_1|GPIO_SPEED_100MHz)
#define GPIO_FMC_SDNWE  (GPIO_FMC_SDNWE_1|GPIO_SPEED_100MHz)

#endif  /* __BOARDS_ARM_STM32F777ZIT6_MEADOW_INCLUDE_BOARD_H */
