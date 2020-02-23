/****************************************************************************
 * boards/arm/stm32/stm3220g-eval/include/board.h
 *
 *   Copyright (C) 2012, 2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32_STM3220G_EVAL_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_STM3220G_EVAL_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32_rcc.h"
#include "stm32_sdio.h"
#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *************************************************************************/
/* Four clock sources are available on STM3220G-EVAL evaluation board for
 * STM32F207IGH6 and RTC embedded:
 *
 * X1, 25 MHz crystal for ethernet PHY with socket. It can be removed when clock is
 *     provided by MCO pin of the MCU
 * X2, 26 MHz crystal for USB OTG HS PHY
 * X3, 32 kHz crystal for embedded RTC
 * X4, 25 MHz crystal with socket for STM32F207IGH6 microcontroller (It can be removed
 *     from socket when internal RC clock is used.)
 *
 * This is the "standard" configuration as set up by arch/arm/src/stm32f40xx_rcc.c:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 120000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 120000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 25000000     (STM32_BOARD_XTAL)
 *   PLLM                          : 25           (STM32_PLLCFG_PLLM)
 *   PLLN                          : 240          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 5            (STM32_PLLCFG_PLLQ)
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
 *         = (25,000,000 / 25) * 240
 *         = 240,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 240,000,000 / 2 = 120,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 240,000,000 / 5 = 48,000,000
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(25)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(240)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(5)

#define STM32_SYSCLK_FREQUENCY  120000000ul

/* AHB clock (HCLK) is SYSCLK (120MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (30MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 (60Mhz)*/

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (60MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 (120Mhz)*/

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

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)= 16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)= 12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* Ethernet *************************************************************************/
/* We need to provide clocking to the MII PHY via MCO1 (PA8) */

#if defined(CONFIG_NET) && defined(CONFIG_STM32_ETHMAC)

#  if !defined(CONFIG_STM32_MII)
#    warning "CONFIG_STM32_MII required for Ethernet"
#  elif !defined(CONFIG_STM32_MII_MCO1)
#    warning "CONFIG_STM32_MII_MCO1 required for Ethernet MII"
#  else

  /* Output HSE clock (25MHz) on MCO1 pin (PA8) to clock the PHY */

#    define BOARD_CFGR_MC01_SOURCE  RCC_CFGR_MCO1_HSE
#    define BOARD_CFGR_MC01_DIVIDER RCC_CFGR_MCO1PRE_NONE

#  endif
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

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)
#define BOARD_LED4_BIT    (1 << BOARD_LED4)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 4 LEDs on board the
 * STM3220G-EVAL.  The following definitions describe how NuttX controls the LEDs:
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
/* The STM3220G-EVAL supports three buttons: */

#define BUTTON_WAKEUP      0
#define BUTTON_TAMPER      1
#define BUTTON_USER        2

#define NUM_BUTTONS        3

#define BUTTON_WAKEUP_BIT  (1 << BUTTON_WAKEUP)
#define BUTTON_TAMPER_BIT  (1 << BUTTON_TAMPER)
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ************************************************/

/* UART3:
 *
 * - PC11 is MicroSDCard_D3 & RS232/IrDA_RX (JP22 open)
 * - PC10 is MicroSDCard_D2 & RSS232/IrDA_TX
 */

#ifdef CONFIG_STM32_USART3
#  define GPIO_USART3_RX GPIO_USART3_RX_2
#  define GPIO_USART3_TX GPIO_USART3_TX_2
#endif

/* Ethernet:
 *
 * - PA2  is ETH_MDIO
 * - PC1  is ETH_MDC
 * - PB5  is ETH_PPS_OUT
 * - PH2  is ETH_MII_CRS
 * - PH3  is ETH_MII_COL
 * - PI10 is ETH_MII_RX_ER
 * - PH6  is ETH_MII_RXD2
 * - PH7  is ETH_MII_RXD3
 * - PC3  is ETH_MII_TX_CLK
 * - PC2  is ETH_MII_TXD2
 * - PB8  is ETH_MII_TXD3
 * - PA1  is ETH_MII_RX_CLK/ETH_RMII_REF_CLK
 * - PA7  is ETH_MII_RX_DV/ETH_RMII_CRS_DV
 * - PC4  is ETH_MII_RXD0/ETH_RMII_RXD0
 * - PC5  is ETH_MII_RXD1/ETH_RMII_RXD1
 * - PG11 is ETH_MII_TX_EN/ETH_RMII_TX_EN
 * - PG13 is ETH_MII_TXD0/ETH_RMII_TXD0
 * - PG14 is ETH_MII_TXD1/ETH_RMII_TXD1
 */

#define GPIO_ETH_PPS_OUT    GPIO_ETH_PPS_OUT_1
#define GPIO_ETH_MII_CRS    GPIO_ETH_MII_CRS_2
#define GPIO_ETH_MII_COL    GPIO_ETH_MII_COL_2
#define GPIO_ETH_MII_RX_ER  GPIO_ETH_MII_RX_ER_2
#define GPIO_ETH_MII_RXD2   GPIO_ETH_MII_RXD2_2
#define GPIO_ETH_MII_RXD3   GPIO_ETH_MII_RXD3_2
#define GPIO_ETH_MII_TXD3   GPIO_ETH_MII_TXD3_1
#define GPIO_ETH_MII_TX_EN  GPIO_ETH_MII_TX_EN_2
#define GPIO_ETH_MII_TXD0   GPIO_ETH_MII_TXD0_2
#define GPIO_ETH_MII_TXD1   GPIO_ETH_MII_TXD1_2
#define GPIO_ETH_RMII_TX_EN GPIO_ETH_RMII_TX_EN_2
#define GPIO_ETH_RMII_TXD0  GPIO_ETH_RMII_TXD0_2
#define GPIO_ETH_RMII_TXD1  GPIO_ETH_RMII_TXD1_2

/* PWM
 *
 * The STM3220G-Eval has no real on-board PWM devices, but the board can be
 * configured to output a pulse train using the following:
 *
 * If FSMC is not used:
 *   TIM4 CH2OUT: PD13 FSMC_A18 / MC_TIM4_CH2OUT
 *   Daughterboard Extension Connector, CN3, pin 32
 *   Motor Control Connector CN15, pin 33 -- not available unless you bridge SB14.
 *
 *   TIM1 CH1OUT: PE9 FSMC_D6
 *   Daughterboard Extension Connector, CN2, pin 24
 *
 *   TIM1_CH2OUT: PE11 FSMC_D8
 *   Daughterboard Extension Connector, CN2, pin 26
 *
 *   TIM1_CH3OUT: PE13 FSMC_D10
 *   Daughterboard Extension Connector, CN2, pin 28
 *
 *   TIM1_CH4OUT: PE14 FSMC_D11
 *   Daughterboard Extension Connector, CN2, pin 29
 *
 * If OTG FS is not used
 *
 *   TIM1_CH3OUT: PA10 OTG_FS_ID
 *   Daughterboard Extension Connector, CN3, pin 14
 *
 *   TIM1_CH4OUT: PA11 OTG_FS_DM
 *   Daughterboard Extension Connector, CN3, pin 11
 *
 * If DMCI is not used
 *
 *   TIM8 CH1OUT: PI5 DCMI_VSYNC & MC
 *   Daughterboard Extension Connector, CN4, pin 4
 *
 *   TIM8_CH2OUT: PI6 DCMI_D6 & MC
 *   Daughterboard Extension Connector, CN4, pin 3
 *
 *   TIM8_CH3OUT: PI7 DCMI_D7 & MC
 *   Daughterboard Extension Connector, CN4, pin 2
 *
 * If SDIO is not used
 *
 *   TIM8_CH3OUT: PC8 MicroSDCard_D0 & MC
 *   Daughterboard Extension Connector, CN3, pin 18
 *
 *   TIM8_CH4OUT: PC9 MicroSDCard_D1 & I2S_CKIN (Need JP16 open)
 *   Daughterboard Extension Connector, CN3, pin 17
 *
 * Others
 *
 *   TIM8 CH1OUT: PC6 I2S_MCK & Smartcard_IO (JP21 open)
 */

#if !defined(CONFIG_STM32_FSMC)
#  define GPIO_TIM4_CH2OUT GPIO_TIM4_CH2OUT_2
#  define GPIO_TIM1_CH1OUT GPIO_TIM1_CH1OUT_2
#  define GPIO_TIM1_CH2OUT GPIO_TIM1_CH2OUT_2
#  define GPIO_TIM1_CH3OUT GPIO_TIM1_CH3OUT_2
#  define GPIO_TIM1_CH4OUT GPIO_TIM1_CH4OUT_2
#elif !defined(CONFIG_STM32_OTGFS)
#  define GPIO_TIM1_CH3OUT GPIO_TIM1_CH3OUT_1
#  define GPIO_TIM1_CH4OUT GPIO_TIM1_CH4OUT_1
#endif

#if !defined(CONFIG_STM32_DCMI)
#  define GPIO_TIM8_CH1OUT GPIO_TIM8_CH1OUT_2
#  define GPIO_TIM8_CH2OUT GPIO_TIM8_CH2OUT_2
#  define GPIO_TIM8_CH3OUT GPIO_TIM8_CH3OUT_2
#else
#  define GPIO_TIM8_CH1OUT GPIO_TIM8_CH1OUT_1
#  if !defined(CONFIG_STM32_SDIO)
#  define GPIO_TIM8_CH3OUT GPIO_TIM8_CH3OUT_1
#  endif
#endif

#if !defined(CONFIG_STM32_SDIO)
#  define GPIO_TIM8_CH4OUT GPIO_TIM8_CH4OUT_1
#endif

/* CAN
 *
 * Connector 10 (CN10) is DB-9 male connector that can be used with CAN1 or CAN2.
 *
 *   JP10 connects CAN1_RX or CAN2_RX to the CAN transceiver
 *   JP3 connects CAN1_TX or CAN2_TX to the CAN transceiver
 *
 * CAN signals are then available on CN10 pins:
 *
 *   CN10 Pin 7 = CANH
 *   CN10 Pin 2 = CANL
 *
 * Mapping to STM32 GPIO pins:
 *
 *   PD0   = FSMC_D2 & CAN1_RX
 *   PD1   = FSMC_D3 & CAN1_TX
 *   PB13  = ULPI_D6 & CAN2_TX
 *   PB5   = ULPI_D7 & CAN2_RX
 */

#define GPIO_CAN1_RX        GPIO_CAN1_RX_3
#define GPIO_CAN1_TX        GPIO_CAN1_TX_3

#define GPIO_CAN2_RX        GPIO_CAN2_RX_2
#define GPIO_CAN2_TX        GPIO_CAN2_TX_1

/* I2C.  Only I2C1 is available on the STM3220G-EVAL.  I2C1_SCL and I2C1_SDA are
 * available on the following pins:
 *
 * - PB6  is I2C1_SCL
 * - PB9  is I2C1_SDA
 */

#define GPIO_I2C1_SCL       GPIO_I2C1_SCL_1
#define GPIO_I2C1_SDA       GPIO_I2C1_SDA_2

/* DMA Channel/Stream Selections *****************************************************/
/* Stream selections are arbitrary for now but might become important in the future
 * is we set aside more DMA channels/streams.
 *
 * SDIO DMA
 *   DMAMAP_SDIO_1 = Channel 4, Stream 3
 *   DMAMAP_SDIO_2 = Channel 4, Stream 6
 */

#define DMAMAP_SDIO DMAMAP_SDIO_1

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

/****************************************************************************
 * Name:  stm3220g_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the STM3210E-EVAL board.  Because
 *   of the various rotations, clearing the display in the normal way by writing a
 *   sequences of runs that covers the entire display can be very slow.  Here the
 *   display is cleared by simply setting all GRAM memory to the specified color.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm3220g_lcdclear(uint16_t color);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_STM3220G_EVAL_INCLUDE_BOARD_H */
