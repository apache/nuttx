/****************************************************************************
 * boards/arm/stm32h7/h747ai/src/stm32_clockconfig.c
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
#include <nuttx/board.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_pwr.h"
#include "hardware/stm32_axi.h"
#include "hardware/stm32_syscfg.h"
#include "hardware/stm32_flash.h"

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

#define HSE_DIVISOR (STM32_HSE_FREQUENCY + 500000) / 1000000

/* FLASH wait states */

#if !defined(BOARD_FLASH_WAITSTATES)
#  error BOARD_FLASH_WAITSTATES not defined
#elif BOARD_FLASH_WAITSTATES < 0 || BOARD_FLASH_WAITSTATES > 15
#  error BOARD_FLASH_WAITSTATES is out of range
#endif

/* Voltage output scale (default to Scale 1 mode) */

#ifndef STM32_PWR_VOS_SCALE
#  define STM32_PWR_VOS_SCALE PWR_D3CR_VOS_SCALE_1
#endif

#if !defined(BOARD_FLASH_PROGDELAY)
#  if (STM32_PWR_VOS_SCALE == PWR_D3CR_VOS_SCALE_1) || \
      (STM32_PWR_VOS_SCALE == PWR_D3CR_VOS_SCALE_0)
#    if STM32_SYSCLK_FREQUENCY <= 70000000 && BOARD_FLASH_WAITSTATES == 0
#      define BOARD_FLASH_PROGDELAY  0
#    elif STM32_SYSCLK_FREQUENCY <= 140000000 && BOARD_FLASH_WAITSTATES == 1
#      define BOARD_FLASH_PROGDELAY  10
#    elif STM32_SYSCLK_FREQUENCY <= 185000000 && BOARD_FLASH_WAITSTATES == 2
#      define BOARD_FLASH_PROGDELAY  1
#    elif STM32_SYSCLK_FREQUENCY <= 210000000 && BOARD_FLASH_WAITSTATES == 2
#      define BOARD_FLASH_PROGDELAY  2
#    elif STM32_SYSCLK_FREQUENCY <= 225000000 && BOARD_FLASH_WAITSTATES == 3
#      define BOARD_FLASH_PROGDELAY  2
#    else
#      define BOARD_FLASH_PROGDELAY  2
#    endif
#  else
#    define BOARD_FLASH_PROGDELAY    2
#  endif
#endif

/* PLL are only enabled if the P,Q or R outputs are enabled. */

#undef USE_PLL1
#if STM32_PLLCFG_PLL1CFG & (RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | \
                            RCC_PLLCFGR_DIVR1EN)
#  define USE_PLL1
#endif

#undef USE_PLL2
#if STM32_PLLCFG_PLL2CFG & (RCC_PLLCFGR_DIVP2EN | RCC_PLLCFGR_DIVQ2EN | \
                            RCC_PLLCFGR_DIVR2EN)
#  define USE_PLL2
#endif

#undef USE_PLL3
#if STM32_PLLCFG_PLL3CFG & (RCC_PLLCFGR_DIVP3EN | RCC_PLLCFGR_DIVQ3EN | \
                            RCC_PLLCFGR_DIVR3EN)
#  define USE_PLL3
#endif

#if defined(STM32_BOARD_USEHSI) && !defined(STM32_BOARD_HSIDIV)
#error When HSI is used, you have to define STM32_BOARD_HSIDIV in board/include/board.h
#endif

/* Over-drive is supported only for Voltage output scale 1 mode.
 * It is required when SYSCLK frequency is over 400 MHz or it can be forced
 * to a given state by adding define to the board.h configuration file:
 *
 *   #define STM32_VOS_OVERDRIVE 1 - force over-drive enabled,
 *   #define STM32_VOS_OVERDRIVE 0 - force over-drive disabled,
 *   #undef STM32_VOS_OVERDRIVE    - autoselect over-drive by logic below
 *
 * Boosting the core voltage can be a workaround solution to problems with
 * poor board signal integration for high-speed digital interfaces like ULPI.
 * Higher voltage means faster clock signal edges which may be sufficient to
 * synchronise the high-speed clock and data.
 */

#ifndef STM32_VOS_OVERDRIVE
#  if (STM32_PWR_VOS_SCALE == PWR_D3CR_VOS_SCALE_1) && \
      (STM32_SYSCLK_FREQUENCY > 400000000)
#    define STM32_VOS_OVERDRIVE 1
#  else
#    define STM32_VOS_OVERDRIVE 0
#  endif
#else
#  if (STM32_VOS_OVERDRIVE == 1) &&             \
      (STM32_PWR_VOS_SCALE != PWR_D3CR_VOS_SCALE_1)
#    error Over-drive can be selected only when VOS1 is configured
#  endif
#endif

static inline void rcc_enableahb4(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB4ENR register to enabled the
   * selected AHB4 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB4ENR);

#ifdef CONFIG_STM32H7_ADC3
  /* ADC3 clock enable */

  regval |= RCC_AHB4ENR_ADC3EN;
#endif

  /* Enable GPIO, GPIOB, ... GPIOK */

#if STM32H7_NGPIO > 0
  regval |= (RCC_AHB4ENR_GPIOAEN
#if STM32H7_NGPIO > 1
             | RCC_AHB4ENR_GPIOBEN
#endif
#if STM32H7_NGPIO > 2
             | RCC_AHB4ENR_GPIOCEN
#endif
#if STM32H7_NGPIO > 3
             | RCC_AHB4ENR_GPIODEN
#endif
#if STM32H7_NGPIO > 4
             | RCC_AHB4ENR_GPIOEEN
#endif
#if (STM32H7_NGPIO > 5) && (defined(CONFIG_STM32H7_HAVE_GPIOF))
             | RCC_AHB4ENR_GPIOFEN
#endif
#if (STM32H7_NGPIO > 6) && (defined(CONFIG_STM32H7_HAVE_GPIOG))
             | RCC_AHB4ENR_GPIOGEN
#endif
#if STM32H7_NGPIO > 7
             | RCC_AHB4ENR_GPIOHEN
#endif
#if STM32H7_NGPIO > 8
             | RCC_AHB4ENR_GPIOIEN
#endif
#if STM32H7_NGPIO > 9
             | RCC_AHB4ENR_GPIOJEN
#endif
#if STM32H7_NGPIO > 10
             | RCC_AHB4ENR_GPIOKEN
#endif
    );
#endif

#ifdef CONFIG_STM32H7_BDMA
  /* BDMA clock enable */

  regval |= RCC_AHB4ENR_BDMAEN;
#endif

#ifdef CONFIG_STM32H7_CRC
  /* CRC clock enable */

  regval |= RCC_AHB4ENR_CRCEN;
#endif

#ifdef CONFIG_STM32H7_BKPSRAM
  /* Backup SRAM clock enable */

  regval |= RCC_AHB4ENR_BKPSRAMEN;
#endif

  putreg32(regval, STM32_RCC_AHB4ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_board_clockconfig
 ****************************************************************************/

#if defined(CONFIG_STM32H7_CUSTOM_CLOCKCONFIG)
void stm32_board_clockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

#ifdef STM32_BOARD_USEHSI
  /* Enable Internal High-Speed Clock (HSI) */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSION;           /* Enable HSI */
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSI is ready (or until a timeout elapsed) */

  for (timeout = HSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSIRDY flag is the set in the CR */

      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSIRDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }

#else /* if STM32_BOARD_USEHSE */
  /* Enable External High-Speed Clock (HSE) */

  regval  = getreg32(STM32_RCC_CR);
#ifdef STM32_HSEBYP_ENABLE          /* May be defined in board.h header file */
  regval |= RCC_CR_HSEBYP;          /* Enable HSE clock bypass */
#else
  regval &= ~RCC_CR_HSEBYP;         /* Disable HSE clock bypass */
#endif
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSE is ready (or until a timeout elapsed) */

  for (timeout = HSERDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSERDY flag is the set in the CR */

      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSERDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }
#endif

#ifdef CONFIG_STM32H7_HSI48
  /* Enable HSI48 */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSI48ON;
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSI48 is ready */

  while ((getreg32(STM32_RCC_CR) & RCC_CR_HSI48RDY) == 0)
    {
    }
#endif

#ifdef CONFIG_STM32H7_CSI
  /* Enable CSI */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_CSION;
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the CSI is ready */

  while ((getreg32(STM32_RCC_CR) & RCC_CR_CSIRDY) == 0)
    {
    }
#endif

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout > 0)
    {
      /* Set the D1 domain Core prescaler and the HCLK source/divider */

      regval = getreg32(STM32_RCC_D1CFGR);
      regval &= ~(RCC_D1CFGR_HPRE_MASK | RCC_D1CFGR_D1CPRE_MASK);
      regval |= (STM32_RCC_D1CFGR_HPRE | STM32_RCC_D1CFGR_D1CPRE);
      putreg32(regval, STM32_RCC_D1CFGR);

      /* Set PCLK1 */

      regval = getreg32(STM32_RCC_D2CFGR);
      regval &= ~RCC_D2CFGR_D2PPRE2_MASK;
      regval |= STM32_RCC_D2CFGR_D2PPRE1;
      putreg32(regval, STM32_RCC_D2CFGR);

      /* Set PCLK2 */

      regval = getreg32(STM32_RCC_D2CFGR);
      regval &= ~RCC_D2CFGR_D2PPRE2_MASK;
      regval |= STM32_RCC_D2CFGR_D2PPRE2;
      putreg32(regval, STM32_RCC_D2CFGR);

      /* Set PCLK3 */

      regval = getreg32(STM32_RCC_D1CFGR);
      regval &= ~RCC_D1CFGR_D1PPRE_MASK;
      regval |= STM32_RCC_D1CFGR_D1PPRE;
      putreg32(regval, STM32_RCC_D1CFGR);

      /* Set PCLK4 */

      regval = getreg32(STM32_RCC_D3CFGR);
      regval &= ~RCC_D3CFGR_D3PPRE_MASK;
      regval |= STM32_RCC_D3CFGR_D3PPRE;
      putreg32(regval, STM32_RCC_D3CFGR);

#ifdef CONFIG_STM32H7_RTC_HSECLOCK
      /* Set the RTC clock divisor */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_RTCPRE_MASK;
      regval |= RCC_CFGR_RTCPRE(HSE_DIVISOR);
      putreg32(regval, STM32_RCC_CFGR);
#endif

      /* Configure PLL123 clock source and multipiers */

#ifdef STM32_BOARD_USEHSI
      regval = (RCC_PLLCKSELR_PLLSRC_HSI |
                STM32_PLLCFG_PLL1M |
                STM32_PLLCFG_PLL2M |
                STM32_PLLCFG_PLL3M);
#else /* if STM32_BOARD_USEHSE */
      regval = (RCC_PLLCKSELR_PLLSRC_HSE |
                STM32_PLLCFG_PLL1M |
                STM32_PLLCFG_PLL2M |
                STM32_PLLCFG_PLL3M);
#endif
      putreg32(regval, STM32_RCC_PLLCKSELR);

      /* Each PLL offers 3 outputs with post-dividers (PLLxP/PLLxQ/PLLxR) */

      /* Configure PLL1 dividers */

      regval = (STM32_PLLCFG_PLL1N |
                STM32_PLLCFG_PLL1P |
                STM32_PLLCFG_PLL1Q |
                STM32_PLLCFG_PLL1R);
      putreg32(regval, STM32_RCC_PLL1DIVR);

#ifdef STM32_PLLCFG_PLL1FRACN
      putreg32(STM32_PLLCFG_PLL1FRACN, STM32_RCC_PLL1FRACR);
#endif

      /* Configure PLL2 dividers */

      regval = (STM32_PLLCFG_PLL2N |
                STM32_PLLCFG_PLL2P |
                STM32_PLLCFG_PLL2Q |
                STM32_PLLCFG_PLL2R);
      putreg32(regval, STM32_RCC_PLL2DIVR);

#ifdef STM32_PLLCFG_PLL2FRACN
      putreg32(STM32_PLLCFG_PLL2FRACN, STM32_RCC_PLL2FRACR);
#endif

      /* Configure PLL3 dividers */

      regval = (STM32_PLLCFG_PLL3N |
                STM32_PLLCFG_PLL3P |
                STM32_PLLCFG_PLL3Q |
                STM32_PLLCFG_PLL3R);
      putreg32(regval, STM32_RCC_PLL3DIVR);

#ifdef STM32_PLLCFG_PLL3FRACN
      putreg32(STM32_PLLCFG_PLL3FRACN, STM32_RCC_PLL3FRACR);
#endif

      /* Configure PLLs */

      regval = (STM32_PLLCFG_PLL1CFG |
                STM32_PLLCFG_PLL2CFG |
                STM32_PLLCFG_PLL3CFG);
      putreg32(regval, STM32_RCC_PLLCFGR);

      regval = getreg32(STM32_RCC_CR);
#if defined(USE_PLL1)
      /* Enable the PLL1 */

      regval |= RCC_CR_PLL1ON;
#endif

#if defined(USE_PLL2)
      /* Enable the PLL2 */

      regval |= RCC_CR_PLL2ON;
#endif

#if defined(USE_PLL3)
      /* Enable the PLL3 */

      regval |= RCC_CR_PLL3ON;
#endif
      putreg32(regval, STM32_RCC_CR);

#if defined(USE_PLL1)
      /* Wait until the PLL1 is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL1RDY) == 0)
        {
        }
#endif

#if defined(USE_PLL2)
      /* Wait until the PLL2 is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL2RDY) == 0)
        {
        }
#endif

#if defined(USE_PLL3)
      /* Wait until the PLL3 is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL3RDY) == 0)
        {
        }
#endif

      /* We must write the lower byte of the PWR_CR3 register is written once
       * after POR and it shall be written before changing VOS level or
       * ck_sys clock frequency. No limitation applies to the upper bytes.
       *
       * Programming data corresponding to an invalid combination of
       * LDOEN and BYPASS bits will be ignored: data will not be written,
       * the written-once mechanism will lock the register and any further
       * write access will be ignored. The default supply configuration will
       * be kept and the ACTVOSRDY bit in PWR control status register 1
       * (PWR_CSR1) will go on indicating invalid voltage levels.
       *
       * N.B. The system shall be power cycled before writing a new value.
       */

#if defined(CONFIG_STM32H7_PWR_DIRECT_SMPS_SUPPLY)
      regval = getreg32(STM32_PWR_CR3);
      regval &= ~(STM32_PWR_CR3_BYPASS | STM32_PWR_CR3_LDOEN |
          STM32_PWR_CR3_SMPSEXTHP | STM32_PWR_CR3_SMPSLEVEL_MASK);
      regval |= STM32_PWR_CR3_SCUEN;
      putreg32(regval, STM32_PWR_CR3);
#else
      regval = getreg32(STM32_PWR_CR3);
      regval |= STM32_PWR_CR3_LDOEN | STM32_PWR_CR3_SCUEN;
      putreg32(regval, STM32_PWR_CR3);
#endif

      /* Set the voltage output scale */

      regval = getreg32(STM32_PWR_D3CR);
      regval &= ~STM32_PWR_D3CR_VOS_MASK;
      regval |= STM32_PWR_VOS_SCALE;
      putreg32(regval, STM32_PWR_D3CR);

      while ((getreg32(STM32_PWR_D3CR) & STM32_PWR_D3CR_VOSRDY) == 0)
        {
        }

#if STM32_PWR_VOS_SCALE == PWR_D3CR_VOS_SCALE_1
      /* See Reference manual Section 5.4.1, System supply startup */

      while ((getreg32(STM32_PWR_CSR1) & PWR_CSR1_ACTVOSRDY) == 0)
        {
        }
#endif

#if STM32_VOS_OVERDRIVE && (STM32_PWR_VOS_SCALE == PWR_D3CR_VOS_SCALE_1)
      /* Over-drive support for VOS1 */

      /* Enable System configuration controller clock to Enable ODEN */

      regval = getreg32(STM32_RCC_APB4ENR);
      regval |= RCC_APB4ENR_SYSCFGEN;
      putreg32(regval, STM32_RCC_APB4ENR);

      /* Enable Overdrive */

      regval = getreg32(STM32_SYSCFG_PWRCR);
      regval |= SYSCFG_PWRCR_ODEN;
      putreg32(regval, STM32_SYSCFG_PWRCR);

      while ((getreg32(STM32_PWR_D3CR) & STM32_PWR_D3CR_VOSRDY) == 0)
        {
        }
#endif

      /* Configure FLASH wait states */

      regval = FLASH_ACR_WRHIGHFREQ(BOARD_FLASH_PROGDELAY) |
               FLASH_ACR_LATENCY(BOARD_FLASH_WAITSTATES);

      putreg32(regval, STM32_FLASH_ACR);

      /* Select the PLL1P as system clock source */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_SW_MASK;
      regval |= RCC_CFGR_SW_PLL1;
      putreg32(regval, STM32_RCC_CFGR);

      /* Wait until the PLL source is used as the system clock source */

      while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) !=
             RCC_CFGR_SWS_PLL1)
        {
        }

      /* Configure SDMMC source clock */

#if defined(STM32_RCC_D1CCIPR_SDMMCSEL)
      regval = getreg32(STM32_RCC_D1CCIPR);
      regval &= ~RCC_D1CCIPR_SDMMC_MASK;
      regval |= STM32_RCC_D1CCIPR_SDMMCSEL;
      putreg32(regval, STM32_RCC_D1CCIPR);
#endif

      /* Configure USART234578 and I2C source clock */

      regval = getreg32(STM32_RCC_D2CCIP2R);
#if defined(STM32_RCC_D2CCIP2R_USART234578SRC)
      regval &= ~RCC_D2CCIP2R_USART234578SEL_MASK;
      regval |= STM32_RCC_D2CCIP2R_USART234578SRC;
#endif
#if defined(STM32_RCC_D2CCIP2R_I2C123SRC)
      regval &= ~RCC_D2CCIP2R_I2C123SEL_MASK;
      regval |= STM32_RCC_D2CCIP2R_I2C123SRC;
#endif
      putreg32(regval, STM32_RCC_D2CCIP2R);

#if defined(STM32_RCC_D3CCIPR_I2C4SRC)
      regval = getreg32(STM32_RCC_D3CCIPR);
      regval &= ~RCC_D3CCIPR_I2C4SEL_MASK;
      regval |= STM32_RCC_D3CCIPR_I2C4SRC;
      putreg32(regval, STM32_RCC_D3CCIPR);
#endif

      /* Configure SPI source clock */

#if defined(STM32_RCC_D2CCIP1R_SPI123SRC)
      regval = getreg32(STM32_RCC_D2CCIP1R);
      regval &= ~RCC_D2CCIP1R_SPI123SEL_MASK;
      regval |= STM32_RCC_D2CCIP1R_SPI123SRC;
      putreg32(regval, STM32_RCC_D2CCIP1R);
#endif

#if defined(STM32_RCC_D2CCIP1R_SPI45SRC)
      regval = getreg32(STM32_RCC_D2CCIP1R);
      regval &= ~RCC_D2CCIP1R_SPI45SEL_MASK;
      regval |= STM32_RCC_D2CCIP1R_SPI45SRC;
      putreg32(regval, STM32_RCC_D2CCIP1R);
#endif

#if defined(STM32_RCC_D3CCIPR_SPI6SRC)
      regval = getreg32(STM32_RCC_D3CCIPR);
      regval &= ~RCC_D3CCIPR_SPI6SEL_MASK;
      regval |= STM32_RCC_D3CCIPR_SPI6SRC;
      putreg32(regval, STM32_RCC_D3CCIPR);
#endif

      /* Configure USB source clock */

#if defined(STM32_RCC_D2CCIP2R_USBSRC)
      regval = getreg32(STM32_RCC_D2CCIP2R);
      regval &= ~RCC_D2CCIP2R_USBSEL_MASK;
      regval |= STM32_RCC_D2CCIP2R_USBSRC;
      putreg32(regval, STM32_RCC_D2CCIP2R);
#endif

      /* Configure ADC source clock */

#if defined(STM32_RCC_D3CCIPR_ADCSEL)
      regval = getreg32(STM32_RCC_D3CCIPR);
      regval &= ~RCC_D3CCIPR_ADCSEL_MASK;
      regval |= STM32_RCC_D3CCIPR_ADCSEL;
      putreg32(regval, STM32_RCC_D3CCIPR);
#endif

      /* Configure FDCAN source clock */

#if defined(STM32_RCC_D2CCIP1R_FDCANSEL)
      regval = getreg32(STM32_RCC_D2CCIP1R);
      regval &= ~RCC_D2CCIP1R_FDCANSEL_MASK;
      regval |= STM32_RCC_D2CCIP1R_FDCANSEL;
      putreg32(regval, STM32_RCC_D2CCIP1R);
#endif

#if defined(CONFIG_STM32H7_IWDG) || defined(CONFIG_STM32H7_RTC_LSICLOCK)
      /* Low speed internal clock source LSI */

      stm32_rcc_enablelsi();
#endif

#if defined(CONFIG_STM32H7_RTC_LSECLOCK)
      /* Low speed external clock source LSE
       *
       * TODO: There is another case where the LSE needs to
       * be enabled: if the MCO1 pin selects LSE as source.
       */

      stm32_rcc_enablelse();
#endif
    }
}
#endif
