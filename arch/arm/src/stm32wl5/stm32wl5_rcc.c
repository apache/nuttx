/****************************************************************************
 * arch/arm/src/stm32wl5/stm32wl5_rcc.c
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_internal.h"

#include "chip.h"
#include "stm32wl5_rcc.h"
#include "stm32wl5_flash.h"
#include "stm32wl5.h"
#include "stm32wl5_waste.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name
 *
 * Description
 *   The RTC needs to reset the Backup Domain to change RTCSEL and resetting
 *   the Backup Domain renders to disabling the LSE as consequence.   In
 *   order to avoid resetting the Backup Domain when we already configured
 *   LSE we will reset the Backup Domain early (here).
 *
 * Input Parameters
 *   None
 *
 * Returned Value
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_STM32WL5_PWR) && defined(CONFIG_STM32WL5_RTC)
static inline void stm32wl5_rcc_resetbkp(void)
{
  bool init_stat;

  /* Check if the RTC is already configured */

  init_stat = stm32wl5_rtc_is_initialized();
  if (!init_stat)
    {
      uint32_t bkregs[STM32WL5_RTC_BKCOUNT];
      int i;

      /* Backup backup-registers before RTC reset. */

      for (i = 0; i < STM32WL5_RTC_BKCOUNT; i++)
        {
          bkregs[i] = getreg32(STM32WL5_RTC_BKR(i));
        }

      /* Enable write access to the backup domain (RTC registers, RTC
       * backup data registers and backup SRAM).
       */

      (void)stm32wl5_pwr_enablebkp(true);

      /* We might be changing RTCSEL - to ensure such changes work, we must
       * reset the backup domain (having backed up the RTC_MAGIC token)
       */

      modifyreg32(STM32WL5_RCC_BDCR, 0, RCC_BDCR_BDRST);
      modifyreg32(STM32WL5_RCC_BDCR, RCC_BDCR_BDRST, 0);

      /* Restore backup-registers, except RTC related. */

      for (i = 0; i < STM32WL5_RTC_BKCOUNT; i++)
        {
          if (RTC_MAGIC_REG == STM32WL5_RTC_BKR(i))
            {
              continue;
            }

          putreg32(bkregs[i], STM32WL5_RTC_BKR(i));
        }

      (void)stm32wl5_pwr_enablebkp(false);
    }
}
#else
#  define rcc_resetbkp()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name
 *
 * Description
 *   Called to establish the clock settings based on the values in board.h.
 *   This function (by default) will reset most everything, enable the PLL,
 *   and enable peripheral clocking for all peripherals enabled in the NuttX
 *   configuration file.
 *
 *   If CONFIG_ARCH_BOARD_STM32WL5_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32wl5_board_clockconfig().
 *
 * Input Parameters
 *   None
 *
 * Returned Value
 *   None
 *
 ****************************************************************************/

void stm32wl5_clockconfig(void)
{
#if 0
  /* Make sure that we are starting in the reset state */

  rcc_reset();

  /* Reset backup domain if appropriate */

  rcc_resetbkp();
#endif
#if defined(CONFIG_ARCH_BOARD_STM32WL5_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  stm32wl5_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions in
   * board.h
   */

  stm32wl5_stdclockconfig();

#endif

  /* Enable peripheral clocking */

  stm32wl5_rcc_enableperipherals();
}

/****************************************************************************
 * Name: rcc_enableahb1
 *
 * Description:
 *   Enable selected AHB1 peripherals
 *
 ****************************************************************************/

static void stm32wl5_rcc_enableahb1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB1ENR register to enabled the
   * selected AHB1 peripherals.
   */

  regval = getreg32(STM32WL5_RCC_AHB1ENR);

#ifdef CONFIG_STM32WL5_DMA1
  /* DMA 1 clock enable */

  regval |= RCC_AHB1ENR_DMA1EN;
#endif

#ifdef CONFIG_STM32WL5_DMA2
  /* DMA 2 clock enable */

  regval |= RCC_AHB1ENR_DMA2EN;
#endif

#ifdef CONFIG_STM32WL5_CRC
  /* CRC clock enable */

  regval |= RCC_AHB1ENR_CRCEN;
#endif

  putreg32(regval, STM32WL5_RCC_AHB1ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb2
 *
 * Description:
 *   Enable selected AHB2 peripherals
 *
 ****************************************************************************/

static inline void stm32wl5_rcc_enableahb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB2ENR register to enable the
   * selected AHB2 peripherals.
   */

  regval = getreg32(STM32WL5_RCC_AHB2ENR);

  /* Enable GPIOA, GPIOB, .... GPIOH */

#if STM32WL5_NPORTS > 0
  regval |= (RCC_AHB2ENR_GPIOAEN
#if STM32WL5_NPORTS > 1
  | RCC_AHB2ENR_GPIOBEN
#endif
#if STM32WL5_NPORTS > 2
  | RCC_AHB2ENR_GPIOCEN
#endif
#if STM32WL5_NPORTS > 3
  | RCC_AHB2ENR_GPIOHEN
#endif
  );
#endif /* STM32WL5_NPORTS */

  putreg32(regval, STM32WL5_RCC_AHB2ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb3
 *
 * Description:
 *   Enable selected AHB3 peripherals
 *
 ****************************************************************************/

static inline void stm32wl5_rcc_enableahb3(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB3ENR register to enabled the
   * selected AHB3 peripherals.
   */

  regval = getreg32(STM32WL5_RCC_AHB3ENR);

#ifdef CONFIG_STM32WL5_AES
  /* Cryptographic modules clock enable */

  regval |= RCC_AHB2ENR_AESEN;
#endif

#ifdef CONFIG_STM32WL5_RNG
  /* Random number generator clock enable */

  regval |= RCC_AHB2ENR_RNGEN;
#endif

#ifdef CONFIG_STM32WL5_FLASHEN
  /* Flash memory interface clock enable */

  regval |= RCC_AHB3ENR_FLASHEN;
#endif

#ifdef CONFIG_STM32WL5_IPCC
  /* IPCC interface clock enable */

  regval |= RCC_AHB3ENR_IPCCEN;
#endif

  putreg32(regval, STM32WL5_RCC_AHB3ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb1
 *
 * Description:
 *   Enable selected APB1 peripherals
 *
 ****************************************************************************/

static inline void stm32wl5_rcc_enableapb1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB1ENR register to enabled the
   * selected APB1 peripherals.
   */

  regval = getreg32(STM32WL5_RCC_APB1ENR1);

#ifdef CONFIG_STM32WL5_TIM2
  /* TIM2 clock enable */

  regval |= RCC_APB1ENR1_TIM2EN;
#endif

#ifdef CONFIG_STM32WL5_SPI2
  /* SPI2 clock enable */

  regval |= RCC_APB1ENR1_SPI2EN;
#endif

#ifdef CONFIG_STM32WL5_USART2
  /* USART 2 clock enable */

  regval |= RCC_APB1ENR1_USART2EN;
#endif

#ifdef CONFIG_STM32WL5_I2C1
  /* I2C1 clock enable */

  regval |= RCC_APB1ENR1_I2C1EN;
#endif

#ifdef CONFIG_STM32WL5_I2C2
  /* I2C2 clock enable */

  regval |= RCC_APB1ENR1_I2C2EN;
#endif

#ifdef CONFIG_STM32WL5_I2C3
  /* I2C3 clock enable */

  regval |= RCC_APB1ENR1_I2C3EN;
#endif

#if defined (CONFIG_STM32WL5_DAC1)
  /* DAC interface clock enable */

  regval |= RCC_APB1ENR1_DAC1EN;
#endif

#ifdef CONFIG_STM32WL5_LPTIM1
  /* Low power timer 1 clock enable */

  regval |= RCC_APB1ENR1_LPTIM1EN;
#endif

  putreg32(regval, STM32WL5_RCC_APB1ENR1);   /* Enable peripherals */

  /* Second APB1 register */

  regval = getreg32(STM32WL5_RCC_APB1ENR2);

#ifdef CONFIG_STM32WL5_LPUART1
  /* Low power uart clock enable */

  regval |= RCC_APB1ENR2_LPUART1EN;
#endif

#ifdef CONFIG_STM32WL5_LPTIM2
  /* Low power timer 2 clock enable */

  regval |= RCC_APB1ENR2_LPTIM2EN;
#endif

#ifdef CONFIG_STM32WL5_LPTIM3
  /* Low power timer 3 clock enable */

  regval |= RCC_APB1ENR2_LPTIM3EN;
#endif

  putreg32(regval, STM32WL5_RCC_APB1ENR2);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb2
 *
 * Description:
 *   Enable selected APB2 peripherals
 *
 ****************************************************************************/

static inline void stm32wl5_rcc_enableapb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB2ENR register to enabled the
   * selected APB2 peripherals.
   */

  regval = getreg32(STM32WL5_RCC_APB2ENR);

#if defined(CONFIG_STM32WL5_ADC1)
  /* ADC clock enable */

  regval |= RCC_AHB2ENR_ADC1EN;
#endif

#ifdef CONFIG_STM32WL5_TIM1
  /* TIM1 clock enable */

  regval |= RCC_APB2ENR_TIM1EN;
#endif

#ifdef CONFIG_STM32WL5_SPI1
  /* SPI1 clock enable */

  regval |= RCC_APB2ENR_SPI1EN;
#endif

#ifdef CONFIG_STM32WL5_USART1
  /* USART1 clock enable */

  regval |= RCC_APB2ENR_USART1EN;
#endif

#ifdef CONFIG_STM32WL5_TIM16
  /* TIM16 clock enable */

  regval |= RCC_APB2ENR_TIM16EN;
#endif

#ifdef CONFIG_STM32WL5_TIM17
  /* TIM16 clock enable */

  regval |= RCC_APB2ENR_TIM17EN;
#endif

  putreg32(regval, STM32WL5_RCC_APB2ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableccip
 *
 * Description:
 *   Set peripherals independent clock configuration.
 *
 ****************************************************************************/

static inline void stm32wl5_rcc_enableccip(void)
{
  uint32_t regval;

  /* Certain peripherals have no clock selected even when their enable bit is
   * set. Set some defaults in the CCIPR register so those peripherals
   * will at least have a clock.
   */

  regval = getreg32(STM32WL5_RCC_CCIPR);

#if defined(STM32WL5_I2C_USE_HSI16)
#ifdef CONFIG_STM32WL5_I2C1
  /* Select HSI16 as I2C1 clock source. */

  regval |= RCC_CCIPR_I2C1SEL_HSI;
#endif
#ifdef CONFIG_STM32WL5_I2C2
  /* Select HSI16 as I2C2 clock source. */

  regval |= RCC_CCIPR_I2C2SEL_HSI;
#endif
#ifdef CONFIG_STM32WL5_I2C3
  /* Select HSI16 as I2C3 clock source. */

  regval |= RCC_CCIPR_I2C3SEL_HSI;
#endif
#endif /* STM32WL5_I2C_USE_HSI16 */

#if defined(STM32WL5_USE_CLK48)
  /* XXX sanity if sdmmc1 or usb or rng, then we need to set the clk48 source
   * and then we can also do away with STM32WL5_USE_CLK48, and give better
   * warning messages.
   */

  regval |= STM32WL5_CLK48_SEL;
#endif

#if defined(CONFIG_STM32WL5_ADC1)
  /* Select SYSCLK as ADC clock source */

  regval |= RCC_CCIPR_ADCSEL_SYSCLK;
#endif

#ifdef CONFIG_STM32WL5_DFSDM1
  /* Select SYSCLK as DFSDM clock source */

  /* RM0394 Rev 3, p. 525 is confused about DFSDM clock source.
   * ST has confirmed that at least in STM32WL551, the bit DFSDMSEL
   * in RCC_CCIPR is present and bit description was omitted from
   * RM0394 by accident.
   */

  regval |= RCC_CCIPR_DFSDMSEL_SYSCLK;
#endif

  putreg32(regval, STM32WL5_RCC_CCIPR);

  /* I2C4 alone has their clock selection in CCIPR2 register. */

#if defined(STM32WL5_I2C_USE_HSI16)
#ifdef CONFIG_STM32WL5_I2C4
  regval = getreg32(STM32WL5_RCC_CCIPR2);

  /* Select HSI16 as I2C4 clock source. */

  regval |= RCC_CCIPR_I2C4SEL_HSI;

  putreg32(regval, STM32WL5_RCC_CCIPR2);
#endif
#endif
}

/****************************************************************************
 * Name
 *
 * Description
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h.  This function is only available to support low-power modes of
 *   operation
 *   re-enable/re-start the PLL
 *
 *   This function performs a subset of the operations performed by
 *   stm32wl5_clockconfig()
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_STM32WL5_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32wl5_board_clockconfig().
 *
 * Input Parameters
 *   None
 *
 * Returned Value
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void stm32wl5_clockenable(void)
{
#if defined(CONFIG_ARCH_BOARD_STM32WL5_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  stm32wl5_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions in
   * board.h
   */

  stm32wl5_stdclockconfig();

#endif
}
#endif

/****************************************************************************
 * Name: stm32wl5_rcc_enableperipherals
 ****************************************************************************/

void stm32wl5_rcc_enableperipherals(void)
{
    stm32wl5_rcc_enableccip();
    stm32wl5_rcc_enableahb1();
    stm32wl5_rcc_enableahb2();
    stm32wl5_rcc_enableahb3();
    stm32wl5_rcc_enableapb1();
    stm32wl5_rcc_enableapb2();
}

/****************************************************************************
 * Name: stm32wl5_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32WL5_CUSTOM_CLOCKCONFIG
void stm32wl5_stdclockconfig(void)
{
  uint32_t regval;

#if defined(STM32WL5_BOARD_USEHSI) || defined(STM32WL5_I2C_USE_HSI16)
  /* Enable Internal High-Speed Clock (HSI) */

  regval  = getreg32(STM32WL5_RCC_CR);
  regval |= RCC_CR_HSION;           /* Enable HSI */
  putreg32(regval, STM32WL5_RCC_CR);

  /* Wait until the HSI is ready */

  for (; ; )
    {
      /* Check if the HSIRDY flag is the set in the CR */

      if ((getreg32(STM32WL5_RCC_CR) & RCC_CR_HSIRDY) != 0)
        {
          /* If so, then break-out */

          break;
        }
    }
#endif

#if defined(STM32WL5_BOARD_USEHSI)
  /* Already set above */

#elif defined(STM32WL5_BOARD_USEMSI)
  /* Enable Internal Multi-Speed Clock (MSI) */

  /* Wait until the MSI is either off or ready */

  for (; ; )
    {
      if ((regval = getreg32(STM32WL5_RCC_CR)),
          (regval & RCC_CR_MSIRDY) || ~(regval & RCC_CR_MSION))
        {
          /* If so, then break-out */

          break;
        }
    }

  /* setting MSIRANGE */

  regval  = getreg32(STM32WL5_RCC_CR);
  regval |= (STM32WL5_BOARD_MSIRANGE | RCC_CR_MSION);    /* Enable MSI and frequency */
  putreg32(regval, STM32WL5_RCC_CR);

  /* Wait until the MSI is ready */

  for (; ; )
    {
      /* Check if the MSIRDY flag is the set in the CR */

      if ((getreg32(STM32WL5_RCC_CR) & RCC_CR_MSIRDY) != 0)
        {
          /* If so, then break-out */

          break;
        }
    }

#elif defined(STM32WL5_BOARD_USEHSE)
  /* Enable External High-Speed Clock (HSE) */

#if defined(STM32WL5_BOARD_USETCXO)
  /* nucleo-wl55jc uses TCXO crystal, which needs to be first
   * powered up with PB0 pin - or more convinently by setting
   * HSEBYPPWR register. This has to be done before HSE is enabled
   */

  regval  = getreg32(STM32WL5_RCC_CR);
  regval |= RCC_CR_HSEBYPPWR;
  putreg32(regval, STM32WL5_RCC_CR);
#endif

  regval  = getreg32(STM32WL5_RCC_CR);
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32WL5_RCC_CR);

  /* Wait until the HSE is ready */

  for (; ; )
    {
      /* Check if the HSERDY flag is the set in the CR */

      if ((getreg32(STM32WL5_RCC_CR) & RCC_CR_HSERDY) != 0)
        {
          /* If so, then break-out */

          break;
        }
    }
#else

#  error stm32wl5_stdclockconfig(), must have one of STM32WL5_BOARD_USEHSI, STM32WL5_BOARD_USEMSI, STM32WL5_BOARD_USEHSE defined

#endif

  /* Select main regulator voltage range according to system clock
   * frequency.
   */

  /* Select correct main regulator range */

  regval = getreg32(STM32WL5_PWR_CR1);
  regval &= ~PWR_CR1_VOS_MASK;

  if (STM32WL5_SYSCLK_FREQUENCY <= 16000000)
    {
      /* set low power range for frequencies <= 16MHz */

      regval |= PWR_CR1_VOS_RANGE2;
    }
  else
    {
      /* set performance range for frequencies > 16MHz */

      regval |= PWR_CR1_VOS_RANGE1;
    }

  putreg32(regval, STM32WL5_PWR_CR1);

  /* Wait for voltage regulator to stabilize */

  while (getreg32(STM32WL5_PWR_SR2) & PWR_SR2_VOSF)
    {
    }

  /* Set the HCLK source/divider */

  regval  = getreg32(STM32WL5_RCC_CFGR);
  regval &= ~RCC_CFGR_HPRE_MASK;
  regval |= STM32WL5_RCC_CFGR_HPRE;
  putreg32(regval, STM32WL5_RCC_CFGR);

  /* Set the PCLK2 divider */

  regval  = getreg32(STM32WL5_RCC_CFGR);
  regval &= ~RCC_CFGR_PPRE2_MASK;
  regval |= STM32WL5_RCC_CFGR_PPRE2;
  putreg32(regval, STM32WL5_RCC_CFGR);

  /* Set the PCLK1 divider */

  regval  = getreg32(STM32WL5_RCC_CFGR);
  regval &= ~RCC_CFGR_PPRE1_MASK;
  regval |= STM32WL5_RCC_CFGR_PPRE1;
  putreg32(regval, STM32WL5_RCC_CFGR);

#ifdef CONFIG_STM32WL5_RTC_HSECLOCK
  /* Set the RTC clock divisor */

  regval  = getreg32(STM32WL5_RCC_CFGR);
  regval &= ~RCC_CFGR_RTCPRE_MASK;
  regval |= RCC_CFGR_RTCPRE(HSE_DIVISOR);
  putreg32(regval, STM32WL5_RCC_CFGR);
#endif

  /* Set the PLL source and main divider */

  regval  = getreg32(STM32WL5_RCC_PLLCFG);

  /* Configure Main PLL */

  /* Set the PLL dividers and multipliers to configure the main PLL */

  regval = (STM32WL5_PLLCFG_PLLM | STM32WL5_PLLCFG_PLLN |
            STM32WL5_PLLCFG_PLLP | STM32WL5_PLLCFG_PLLQ |
            STM32WL5_PLLCFG_PLLR);

#ifdef STM32WL5_PLLCFG_PLLP_ENABLED
  regval |= RCC_PLLCFG_PLLPEN;
#endif
#ifdef STM32WL5_PLLCFG_PLLQ_ENABLED
  regval |= RCC_PLLCFG_PLLQEN;
#endif
#ifdef STM32WL5_PLLCFG_PLLR_ENABLED
  regval |= RCC_PLLCFG_PLLREN;
#endif

  /* XXX The choice of clock source to PLL (all three) is independent
   * of the sys clock source choice, review the STM32WL5_BOARD_USEHSI
   * name; probably split it into two, one for PLL source and one
   * for sys clock source.
   */

#ifdef STM32WL5_BOARD_USEHSI
  regval |= RCC_PLLCFG_PLLSRC_HSI16;
#elif defined(STM32WL5_BOARD_USEMSI)
  regval |= RCC_PLLCFG_PLLSRC_MSI;
#else /* if STM32WL5_BOARD_USEHSE */
  regval |= RCC_PLLCFG_PLLSRC_HSE;
#endif

  putreg32(regval, STM32WL5_RCC_PLLCFG);

  /* Enable the main PLL */

  regval  = getreg32(STM32WL5_RCC_CR);
  regval |= RCC_CR_PLLON;
  putreg32(regval, STM32WL5_RCC_CR);

  /* Wait until the PLL is ready */

  while ((getreg32(STM32WL5_RCC_CR) & RCC_CR_PLLRDY) == 0)
    {
    }

  /* Configure flash wait states according to manual */

  if (STM32WL5_HCLK3_FREQUENCY <= 18000000 /* 18MHz */)
    {
      regval = FLASH_ACR_LATENCY_0;
    }
  else if (STM32WL5_HCLK3_FREQUENCY <= 36000000 /* 36MHz */)
    {
      regval = FLASH_ACR_LATENCY_1;
    }
  else /* STM32WL5_HCLK3_FREQUENCY <= 48MHz */
    {
      regval = FLASH_ACR_LATENCY_2;
    }

  putreg32(regval, STM32WL5_FLASH_ACR);

  /* Select the main PLL as system clock source */

  regval  = getreg32(STM32WL5_RCC_CFGR);
  regval &= ~RCC_CFGR_SW_MASK;
  regval |= RCC_CFGR_SW_PLL;
  putreg32(regval, STM32WL5_RCC_CFGR);

  /* Wait until the PLL source is used as the system clock source */

  while ((getreg32(STM32WL5_RCC_CFGR) & RCC_CFGR_SWS_MASK) !=
         RCC_CFGR_SWS_PLL)
    {
    }

#if defined(CONFIG_STM32WL5_IWDG) || defined(CONFIG_STM32WL5_RTC_LSICLOCK)
  /* Low speed internal clock source LSI */

  stm32wl5_rcc_enablelsi();
#endif
}
#endif
