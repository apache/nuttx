/****************************************************************************
 * arch/arm/src/at32/at32f43xxx_rcc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "chip.h"
#include "at32_pwr.h"
#include "hardware/at32f43xxx_rcc.h"
#include "itm_syslog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.  Normally this is very fast, but I have seen at least one
 * board that required this long, long timeout for the HSE to be ready.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/* Same for HSI */

#define HSIRDY_TIMEOUT HSERDY_TIMEOUT

/* HSE divisor to yield ~1MHz RTC clock */

#define HSE_DIVISOR (AT32_HSE_FREQUENCY + 500000) / 1000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_reset
 *
 * Description:
 *   Reset the RCC clock configuration to the default reset state
 *
 ****************************************************************************/

static inline void rcc_reset(void)
{
  uint32_t regval;

  /* enable pwc clock */

  regval = getreg32(AT32_CRM_APB1EN);
  regval |= CRM_APB1EN_PWCEN;
  putreg32(regval, AT32_CRM_APB1EN);

  /* set ldo output 1.3V */

  regval  = getreg32(AT32_PWC_LDOOV);
  regval &= ~PWC_LDOOV_SEL_MASK;
  regval |= PWC_LDOOV_1V3;
  putreg32(regval, AT32_PWC_LDOOV);

  /* set flash div 3 */

  regval  = getreg32(AT32_FLASH_DIVR);
  regval &= ~FLASH_DIVR_FDIV_MASK;
  regval |= FLASH_DIVR_FDIV_3;
  putreg32(regval, AT32_FLASH_DIVR);

#if (AT32_SYSCLK_FREQUENCY <= 192000000ul)
  /* enable Flash non-zero wait area boost */

  regval  = getreg32(AT32_FLASH_PSR);
  regval |= FLASH_PSR_NZW_BST;
  putreg32(regval, AT32_FLASH_PSR);
#endif

  /* Flash continue read enable */

  regval  = getreg32(AT32_FLASH_CONTR);
  regval |= FLASH_CONTR_EN;
  putreg32(regval, AT32_FLASH_CONTR);

  /* reset the crm clock configuration to the default reset state */

  /* set hicken bit */

  regval = getreg32(AT32_CRM_CTRL);
  regval |= CRM_CTRL_HICKEN;
  putreg32(regval, AT32_CRM_CTRL);

  /* Wait High speed internal crystal stable */

  while ((getreg32(AT32_CRM_CTRL) & CRM_CTRL_HICKSTBL) != \
        CRM_CTRL_HICKSTBL);

  /* reset hexten, hextbyps, cfden and pllen bits */

  regval = getreg32(AT32_CRM_CTRL);
  regval &= ~CRM_CTRL_HEXTEN;
  regval &= ~CRM_CTRL_HEXTBYPS;
  regval &= ~CRM_CTRL_CFDEN;
  regval &= ~CRM_CTRL_PLLEN;
  putreg32(regval, AT32_CRM_CTRL);

  /* reset cfg register, include sclk switch, ahbdiv,
   * apb1div, apb2div, adcdiv, clkout bits
   */

  regval = 0;
  putreg32(regval, AT32_CRM_CFG);

  /* reset pllms pllns pllfr pllrcs bits */

  regval = 0x00033002ul;
  putreg32(regval, AT32_CRM_PLL_CFG);

  /* reset clkout[3], usbbufs, hickdiv, clkoutdiv */

  regval = 0;
  putreg32(regval, AT32_CRM_MISC1);

  /* disable all interrupts enable and clear pending bits  */

  regval = 0x009f0000ul;
  putreg32(regval, AT32_CRM_CLKINT);
}

/****************************************************************************
 * Name: rcc_enableahb1
 *
 * Description:
 *   Enable selected AHB1 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB1ENR register to enabled the
   * selected AHB1 peripherals.
   */

  regval = getreg32(AT32_CRM_AHBEN1);

  /* Enable GPIOA, GPIOB, .... GPIOH */

#if AT32_NGPIO_PORTS > 0
  regval |= (CRM_AHBEN1_GPIOAEN
#if AT32_NGPIO_PORTS > 1
             | CRM_AHBEN1_GPIOBEN
#endif
#if AT32_NGPIO_PORTS > 2
             | CRM_AHBEN1_GPIOCEN
#endif
#if AT32_NGPIO_PORTS > 3
             | CRM_AHBEN1_GPIODEN
#endif
#if AT32_NGPIO_PORTS > 4
             | CRM_AHBEN1_GPIOEEN
#endif
#if AT32_NGPIO_PORTS > 5
             | CRM_AHBEN1_GPIOFEN
#endif
#if AT32_NGPIO_PORTS > 6
             | CRM_AHBEN1_GPIOGEN
#endif
#if AT32_NGPIO_PORTS > 7
             | CRM_AHBEN1_GPIOHEN
#endif
             );
#endif

#ifdef CONFIG_AT32_CRC
  /* CRC clock enable */

  regval |= CRM_AHBEN1_CRCEN;
#endif

#ifdef CONFIG_AT32_EDMA
  /* EDMA clock enable */

  regval |= CRM_AHBEN1_EDMAEN;
#endif

#ifdef CONFIG_AT32_DMA1
  /* DMA 1 clock enable */

  regval |= CRM_AHBEN1_DMA1EN;
#endif

#ifdef CONFIG_AT32_DMA2
  /* DMA 2 clock enable */

  regval |= CRM_AHBEN1_DMA2EN;
#endif

#ifdef CONFIG_AT32_ETHMAC
  /* Ethernet MAC clocking */

  regval |= (CRM_AHBEN1_EMACEN | CRM_AHBEN1_EMACTXEN
            | CRM_AHBEN1_EMACRXEN);

#ifdef CONFIG_AT32_ETH_PTP
  /* Precision Time Protocol (PTP) */

  regval |= CRM_AHBEN1_EMACPTPEN;

#endif
#endif

#ifdef CONFIG_AT32_OTGFS2
  regval |= CRM_AHBEN1_OTGFS2EN;
#endif

  putreg32(regval, AT32_CRM_AHBEN1);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb2
 *
 * Description:
 *   Enable selected AHB2 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB2ENR register to enabled the
   * selected AHB2 peripherals.
   */

  regval = getreg32(AT32_CRM_AHBEN2);

#ifdef CONFIG_AT32_DVP
  /* Camera interface enable */

  regval |= CRM_AHBEN2_DVPEN;
#endif

#ifdef CONFIG_AT32_OTGFS
  /* USBOTG FS1 modules clock enable */

  regval |= CRM_AHBEN2_OTGFS1EN;
#endif

#ifdef CONFIG_AT32_SDIO
  /* SDIO1 clock enable */

  regval |= CRM_AHBEN2_SDIO1EN;
#endif

  putreg32(regval, AT32_CRM_AHBEN2);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb3
 *
 * Description:
 *   Enable selected AHB3 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb3(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB3ENR register to enabled the
   * selected AHB3 peripherals.
   */

  regval = getreg32(AT32_CRM_AHBEN3);

#ifdef CONFIG_AT32_XMC
  /* Flexible static memory controller module clock enable */

  regval |= CRM_AHBEN3_XMCEN;

#endif

#ifdef CONFIG_AT32_QSPI1
  /* QSPI1 clock enable */

  regval |= CRM_AHBEN3_QSPI1EN;
#endif

#ifdef CONFIG_AT32_QSPI2
  /* QSPI2 clock enable */

  regval |= CRM_AHBEN3_QSPI2EN;
#endif

#ifdef CONFIG_AT32_SDIO2
  /* SDIO2 clock enable */

  regval |= CRM_AHBEN3_SDIO2EN;
#endif

  putreg32(regval, AT32_CRM_AHBEN3);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb1
 *
 * Description:
 *   Enable selected APB1 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB1ENR register to enabled the
   * selected APB1 peripherals.
   */

  regval = getreg32(AT32_CRM_APB1EN);

#ifdef CONFIG_AT32_TRM2
  /* TRM2 clock enable */

  regval |= CRM_APB1EN_TMR2EN;
#endif

#ifdef CONFIG_AT32_TRM3
  /* TRM3 clock enable */

  regval |= CRM_APB1EN_TMR3EN;
#endif

#ifdef CONFIG_AT32_TRM4
  /* TRM4 clock enable */

  regval |= CRM_APB1EN_TMR4EN;
#endif

#ifdef CONFIG_AT32_TRM5
  /* TRM5 clock enable */

  regval |= CRM_APB1EN_TMR5EN;
#endif

#ifdef CONFIG_AT32_TRM6
  /* TRM6 clock enable */

  regval |= CRM_APB1EN_TMR6EN;
#endif

#ifdef CONFIG_AT32_TRM7
  /* TRM7 clock enable */

  regval |= CRM_APB1EN_TMR7EN;
#endif

#ifdef CONFIG_AT32_TRM12
  /* TRM12 clock enable */

  regval |= CRM_APB1EN_TMR12EN;
#endif

#ifdef CONFIG_AT32_TRM13
  /* TRM13 clock enable */

  regval |= CRM_APB1EN_TMR13EN;
#endif

#ifdef CONFIG_AT32_TRM14
  /* TRM14 clock enable */

  regval |= CRM_APB1EN_TMR14EN;
#endif

#ifdef CONFIG_AT32_WWDT
  /* Window watchdog clock enable */

  regval |= CRM_APB1EN_WWDTEN;
#endif

#ifdef CONFIG_AT32_SPI2
  /* SPI2 clock enable */

  regval |= CRM_APB1EN_SPI2EN;
#endif

#ifdef CONFIG_AT32_SPI3
  /* SPI3 clock enable */

  regval |= CRM_APB1EN_SPI3EN;
#endif

#ifdef CONFIG_AT32_USART2
  /* USART 2 clock enable */

  regval |= CRM_APB1EN_USART2EN;
#endif

#ifdef CONFIG_AT32_USART3
  /* USART3 clock enable */

  regval |= CRM_APB1EN_USART3EN;
#endif

#ifdef CONFIG_AT32_UART4
  /* UART4 clock enable */

  regval |= CRM_APB1EN_UART4EN;
#endif

#ifdef CONFIG_AT32_UART5
  /* UART5 clock enable */

  regval |= CRM_APB1EN_UART5EN;
#endif

#ifdef CONFIG_AT32_I2C1
  /* I2C1 clock enable */

  regval |= CRM_APB1EN_I2C1EN;
#endif

#ifdef CONFIG_AT32_I2C2
  /* I2C2 clock enable */

  regval |= CRM_APB1EN_I2C2EN;
#endif

#ifdef CONFIG_AT32_I2C3
  /* I2C3 clock enable */

  regval |= CRM_APB1EN_I2C3EN;
#endif

#ifdef CONFIG_AT32_CAN1
  /* CAN 1 clock enable */

  regval |= CRM_APB1EN_CAN1EN;
#endif

#ifdef CONFIG_AT32_CAN2
  /* CAN2 clock enable */

  regval |= CRM_APB1EN_CAN2EN ;
#endif

  /* Power interface clock enable.  The PWR block is always enabled so that
   * we can set the internal voltage regulator for maximum performance.
   */

  regval |= CRM_APB1EN_PWCEN;

#if defined (CONFIG_AT32_DAC)
  /* DAC interface clock enable */

  regval |= CRM_APB1EN_DACEN;
#endif

#ifdef CONFIG_AT32_UART7
  /* UART7 clock enable */

  regval |= CRM_APB1EN_UART7EN;
#endif

#ifdef CONFIG_AT32_UART8
  /* UART8 clock enable */

  regval |= CRM_APB1EN_UART8EN;
#endif

  putreg32(regval, AT32_CRM_APB1EN);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb2
 *
 * Description:
 *   Enable selected APB2 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB2ENR register to enabled the
   * selected APB2 peripherals.
   */

  regval = getreg32(AT32_CRM_APB2EN);

#ifdef CONFIG_AT32_TRM1
  /* TRM1 clock enable */

  regval |= CRM_APB2EN_TMR1EN;
#endif

#ifdef CONFIG_AT32_TRM8
  /* TRM8 clock enable */

  regval |= CRM_APB2EN_TMR8EN;
#endif

#ifdef CONFIG_AT32_USART1
  /* USART1 clock enable */

  regval |= CRM_APB2EN_USART1EN;
#endif

#ifdef CONFIG_AT32_USART6
  /* USART6 clock enable */

  regval |= CRM_APB2EN_USART6EN;
#endif

#ifdef CONFIG_AT32_ADC1
  /* ADC1 clock enable */

  regval |= CRM_APB2EN_ADC1EN;
#endif

#ifdef CONFIG_AT32_ADC2
  /* ADC2 clock enable */

  regval |= CRM_APB2EN_ADC2EN;
#endif

#ifdef CONFIG_AT32_ADC3
  /* ADC3 clock enable */

  regval |= CRM_APB2EN_ADC3EN;
#endif

#ifdef CONFIG_AT32_SPI1
  /* SPI1 clock enable */

  regval |= CRM_APB2EN_SPI1EN;
#endif

#ifdef CONFIG_AT32_SPI4
  /* SPI4 clock enable */

  regval |= CRM_APB2EN_SPI4EN;
#endif

#ifdef CONFIG_AT32_SYSCFG
  /* System configuration controller clock enable */

  regval |= CRM_APB2EN_SCFGEN;
#endif

#ifdef CONFIG_AT32_TRM9
  /* TRM9 clock enable */

  regval |= CRM_APB2EN_TMR9EN;
#endif

#ifdef CONFIG_AT32_TRM10
  /* TRM10 clock enable */

  regval |= CRM_APB2EN_TMR10EN;
#endif

#ifdef CONFIG_AT32_TRM11
  /* TRM11 clock enable */

  regval |= CRM_APB2EN_TMR11EN;
#endif

#ifdef CONFIG_AT32_TRM20
  /* TRM20 clock enable */

  regval |= CRM_APB2EN_TMR20EN;
#endif

#ifdef CONFIG_AT32_ACC
  /* ACC clock enable */

  regval |= CRM_APB2EN_ACCEN;
#endif

  putreg32(regval, AT32_CRM_APB2EN);   /* Enable peripherals */
}

/****************************************************************************
 * Name: at32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_AT32_CUSTOM_CLOCKCONFIG
static void at32_stdclockconfig(void)
{
  uint32_t regval;

  /* config external crystal as clock source */

  regval = getreg32(AT32_CRM_CTRL);
  regval &= ~CRM_CTRL_HICKEN;
  regval |= CRM_CTRL_HEXTEN;
  putreg32(regval, AT32_CRM_CTRL);

  /* Wait High speed external crystal stable */

  while ((getreg32(AT32_CRM_CTRL) & CRM_CTRL_HEXTSTBL) != \
        CRM_CTRL_HEXTSTBL);

  regval = getreg32(AT32_CRM_CFG);
  regval &= ~CRM_CFG_SCLKSEL_MASK;
  regval |= CRM_CFG_SEL_HEXT;
  putreg32(regval, AT32_CRM_CFG);

  /* config PLL */

  regval = getreg32(AT32_CRM_PLL_CFG);
  regval |= CRM_PLL_CFG_PLLRCS;
  regval &= ~CRM_PLL_CFG_PLL_MS_MASK;
  regval |= AT32_PLLCFG_PLLM;

  regval &= ~CRM_PLL_CFG_PLL_NS_MASK;
  regval |= AT32_PLLCFG_PLLN;

  regval &= ~CRM_PLL_CFG_PLL_FR_MASK;
  regval |= AT32_PLLCFG_PLLP;
  putreg32(regval, AT32_CRM_PLL_CFG);

  /* enable pll */

  regval = getreg32(AT32_CRM_CTRL);
  regval |= CRM_CTRL_PLLEN;
  putreg32(regval, AT32_CRM_CTRL);
  while ((getreg32(AT32_CRM_CTRL) & CRM_CTRL_PLLSTBL) != \
        CRM_CTRL_PLLSTBL);

  regval = getreg32(AT32_CRM_CFG);
  regval &= ~CRM_CFG_AHBDIV_MASK;
  regval |= CRM_CFG_AHBDIV_NONE;    /* ahb div 1 */
  regval &= ~CRM_CFG_APB1DIV_MASK;
  regval |= CRM_CFG_APB1DIV_2;      /* apb1 div 2 */
  regval &= ~CRM_CFG_APB2DIV_MASK;
  regval |= CRM_CFG_APB2DIV_2;      /* apb2 div 2 */
  putreg32(regval, AT32_CRM_CFG);

  /* entry step mode */

  regval = getreg32(AT32_CRM_MISC2);
  regval &= ~CRM_MISC2_AUTO_STEP_EN_MASK;
  regval |= CRM_MISC2_AUTO_STEP_EN_ENABLE;
  putreg32(regval, AT32_CRM_MISC2);

  /* config pll as system clock */

  regval = getreg32(AT32_CRM_CFG);
  regval &= ~CRM_CFG_SCLKSEL_MASK;
  regval |= CRM_CFG_SEL_PLL;
  putreg32(regval, AT32_CRM_CFG);

  while ((getreg32(AT32_CRM_CFG) & CRM_CFG_SCLKSTSL_MASK) != \
        CRM_CFG_STS_PLL);

  /* exit step mode */

  regval = getreg32(AT32_CRM_MISC2);
  regval &= ~CRM_MISC2_AUTO_STEP_EN_MASK;
  regval |= CRM_MISC2_AUTO_STEP_EN_DISABLE;
  putreg32(regval, AT32_CRM_MISC2);

#if defined(CONFIG_AT32_OTGFS) || defined(CONFIG_AT32_OTGFS2)

  /* set usbfs clock use pll */

  regval = getreg32(AT32_CRM_MISC1);
  regval &= ~CRM_MISC1_HICK_TO_USB;
  putreg32(regval, AT32_CRM_MISC1);

  /* usbfs clock div */

  regval = getreg32(AT32_CRM_MISC2);
  regval &= ~CRM_MISC2_USBDIV_MASK;
  regval |= USB_CONFIG_USBDIV;
  putreg32(regval, AT32_CRM_MISC2);

#endif
}
#endif

/****************************************************************************
 * Name: efm32_itm_syslog
 *
 * Description:
 *   Enable Serial wire output pin, configure debug clocking, and enable
 *   ITM syslog support.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_ITMSYSLOG
static inline void rcc_itm_syslog(void)
{
  uint32_t regval;

  regval = getreg32(AT32_DEBUG_CTRL);
  regval &= ~DEBUG_CTRL_SLEEP_DEBUG;
  putreg32(regval, AT32_DEBUG_CTRL);
}
#else
#  define rcc_itm_syslog()
#endif

/****************************************************************************
 * Name: rcc_enableperiphals
 ****************************************************************************/

static inline void rcc_enableperipherals(void)
{
  rcc_enableahb1();
  rcc_enableahb2();
  rcc_enableahb3();
  rcc_enableapb1();
  rcc_enableapb2();
  rcc_itm_syslog();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
