/****************************************************************************
 * arch/arm/src/stm32h7/stm32h7x3xx_rcc.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

// TODO: #include "stm32_pwr.h"

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

  /* Enable the Internal High Speed clock (HSI) */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSION;
  putreg32(regval, STM32_RCC_CR);

  /* Reset CFGR register */

  putreg32(0x00000000, STM32_RCC_CFGR);

  /* Reset HSION, HSEON, CSSON and PLLON bits */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~(RCC_CR_HSEON | RCC_CR_HSI48ON | RCC_CR_CSION | RCC_CR_PLL1ON | RCC_CR_PLL2ON | RCC_CR_PLL3ON);
  putreg32(regval, STM32_RCC_CR);

  /* Reset PLLCFGR register to reset default */

  putreg32(RCC_PLLCFGR_RESET, STM32_RCC_PLLCFGR);

  /* Reset HSEBYP bit */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~RCC_CR_HSEBYP;
  putreg32(regval, STM32_RCC_CR);

  /* Disable all interrupts */

  putreg32(0x00000000, STM32_RCC_CIER);
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

  regval = getreg32(STM32_RCC_AHB1ENR);

  // TODO: ...

  putreg32(regval, STM32_RCC_AHB1ENR);   /* Enable peripherals */
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

  regval = getreg32(STM32_RCC_AHB2ENR);

  // TODO: ...

  putreg32(regval, STM32_RCC_AHB2ENR);   /* Enable peripherals */
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

  regval = getreg32(STM32_RCC_AHB3ENR);

  // TODO: ...

  putreg32(regval, STM32_RCC_AHB3ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb3
 *
 * Description:
 *   Enable selected AHB1 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb4(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB4ENR register to enabled the
   * selected AHB4 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB4ENR);

  // TODO: ...

  putreg32(regval, STM32_RCC_AHB4ENR);   /* Enable peripherals */
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

  /* Set the appropriate bits in the APB1L/HENR register to enabled the
   * selected APB1 peripherals.
   */

  regval = getreg32(STM32_RCC_APB1LENR);

  // TODO: ...

  putreg32(regval, STM32_RCC_APB1LENR);   /* Enable peripherals */

  regval = getreg32(STM32_RCC_APB1HENR);

  // TODO: ...

  putreg32(regval, STM32_RCC_APB1HENR);   /* Enable peripherals */
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

  regval = getreg32(STM32_RCC_APB2ENR);

  // TODO: ...

  putreg32(regval, STM32_RCC_APB2ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb3
 *
 * Description:
 *   Enable selected APB3 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb3(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB3ENR register to enabled the
   * selected APB3 peripherals.
   */

  regval = getreg32(STM32_RCC_APB3ENR);

  // TODO: ...

  putreg32(regval, STM32_RCC_APB3ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb4
 *
 * Description:
 *   Enable selected APB4 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb4(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB4ENR register to enabled the
   * selected APB4 peripherals.
   */

  regval = getreg32(STM32_RCC_APB4ENR);

  // TODO: ...

  putreg32(regval, STM32_RCC_APB4ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

#ifndef CONFIG_STM32H7_CUSTOM_CLOCKCONFIG
static void stm32_stdclockconfig(void)
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

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout > 0)
    {
      /* Select regulator voltage output Scale 1 mode to support system
       * frequencies up to 216 MHz.
       */

      regval  = getreg32(STM32_RCC_APB1ENR);
      regval |= RCC_APB1ENR_PWREN;
      putreg32(regval, STM32_RCC_APB1ENR);

      regval  = getreg32(STM32_PWR_CR1);
      regval &= ~PWR_CR1_VOS_MASK;
      regval |= PWR_CR1_VOS_SCALE_1;
      putreg32(regval, STM32_PWR_CR1);

      /* Set the HCLK source/divider */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_HPRE_MASK;
      regval |= STM32_RCC_CFGR_HPRE;
      putreg32(regval, STM32_RCC_CFGR);

      /* Set the PCLK2 divider */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_PPRE2_MASK;
      regval |= STM32_RCC_CFGR_PPRE2;
      putreg32(regval, STM32_RCC_CFGR);

      /* Set the PCLK1 divider */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_PPRE1_MASK;
      regval |= STM32_RCC_CFGR_PPRE1;
      putreg32(regval, STM32_RCC_CFGR);

#ifdef CONFIG_STM32H7_RTC_HSECLOCK
      /* Set the RTC clock divisor */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_RTCPRE_MASK;
      regval |= RCC_CFGR_RTCPRE(HSE_DIVISOR);
      putreg32(regval, STM32_RCC_CFGR);
#endif

      /* Set the PLL dividers and multipliers to configure the main PLL */

#ifdef STM32_BOARD_USEHSI
      regval = (STM32_PLLCFG_PLLM | STM32_PLLCFG_PLLN | STM32_PLLCFG_PLLP |
                RCC_PLLCFG_PLLSRC_HSI | STM32_PLLCFG_PLLQ);
#else /* if STM32_BOARD_USEHSE */
      regval = (STM32_PLLCFG_PLLM | STM32_PLLCFG_PLLN | STM32_PLLCFG_PLLP |
                RCC_PLLCFG_PLLSRC_HSE | STM32_PLLCFG_PLLQ);
#endif
      putreg32(regval, STM32_RCC_PLLCFG);

      /* Enable the main PLL */

      regval = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLON;
      putreg32(regval, STM32_RCC_CR);

      /* Wait until the PLL is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0)
        {
        }

      /* Enable the Over-drive to extend the clock frequency to 216 Mhz */

      regval  = getreg32(STM32_PWR_CR1);
      regval |= PWR_CR1_ODEN;
      putreg32(regval, STM32_PWR_CR1);
      while ((getreg32(STM32_PWR_CSR1) & PWR_CSR1_ODRDY) == 0)
        {
        }

      regval = getreg32(STM32_PWR_CR1);
      regval |= PWR_CR1_ODSWEN;
      putreg32(regval, STM32_PWR_CR1);
      while ((getreg32(STM32_PWR_CSR1) & PWR_CSR1_ODSWRDY) == 0)
        {
        }

      /* Configure FLASH wait states */

      regval = FLASH_ACR_LATENCY(BOARD_FLASH_WAITSTATES);

#ifdef CONFIG_STM32H7_FLASH_ART_ACCELERATOR
      /* The Flash memory interface accelerates code execution with a system of
       * instruction prefetch and cache lines on ITCM interface (ART
       * Accelerator™).
       */

      regval |= FLASH_ACR_ARTEN;
      regval |= FLASH_ACR_PRFTEN;
#endif

      putreg32(regval, STM32_FLASH_ACR);

      /* Select the main PLL as system clock source */

      regval  = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_SW_MASK;
      regval |= RCC_CFGR_SW_PLL;
      putreg32(regval, STM32_RCC_CFGR);

      /* Wait until the PLL source is used as the system clock source */

      while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL)
        {
        }

#if defined(CONFIG_STM32H7_LTDC) || defined(CONFIG_STM32H7_PLLSAI)

       /* Configure PLLSAI */

      regval = getreg32(STM32_RCC_PLLSAICFGR);
      regval &= ~(  RCC_PLLSAICFGR_PLLSAIN_MASK
                  | RCC_PLLSAICFGR_PLLSAIP_MASK
                  | RCC_PLLSAICFGR_PLLSAIQ_MASK
                  | RCC_PLLSAICFGR_PLLSAIR_MASK);
      regval |= (STM32_RCC_PLLSAICFGR_PLLSAIN
                 | STM32_RCC_PLLSAICFGR_PLLSAIP
                 | STM32_RCC_PLLSAICFGR_PLLSAIQ
                 | STM32_RCC_PLLSAICFGR_PLLSAIR);
      putreg32(regval, STM32_RCC_PLLSAICFGR);

      regval  = getreg32(STM32_RCC_DCKCFGR1);
      regval &= ~(RCC_DCKCFGR1_PLLI2SDIVQ_MASK
                  | RCC_DCKCFGR1_PLLSAIDIVQ_MASK
                  | RCC_DCKCFGR1_PLLSAIDIVR_MASK
                  | RCC_DCKCFGR1_SAI1SEL_MASK
                  | RCC_DCKCFGR1_SAI2SEL_MASK
                  | RCC_DCKCFGR1_TIMPRESEL);

      regval |= (STM32_RCC_DCKCFGR1_PLLI2SDIVQ
                 | STM32_RCC_DCKCFGR1_PLLSAIDIVQ
                 | STM32_RCC_DCKCFGR1_PLLSAIDIVR
                 | STM32_RCC_DCKCFGR1_SAI1SRC
                 | STM32_RCC_DCKCFGR1_SAI2SRC
                 | STM32_RCC_DCKCFGR1_TIMPRESRC);

      putreg32(regval, STM32_RCC_DCKCFGR1);


      /* Enable PLLSAI */

      regval = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLSAION;
      putreg32(regval, STM32_RCC_CR);

      /* Wait until the PLLSAI is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLSAIRDY) == 0)
        {
        }
#endif
#if defined(CONFIG_STM32H7_LTDC) || defined(CONFIG_STM32H7_PLLI2S)

      /* Configure PLLI2S */

      regval = getreg32(STM32_RCC_PLLI2SCFGR);
      regval &= ~(  RCC_PLLI2SCFGR_PLLI2SN_MASK
                  | RCC_PLLI2SCFGR_PLLI2SP_MASK
                  | RCC_PLLI2SCFGR_PLLI2SQ_MASK
                  | RCC_PLLI2SCFGR_PLLI2SR_MASK);
      regval |= (STM32_RCC_PLLI2SCFGR_PLLI2SN
                 | STM32_RCC_PLLI2SCFGR_PLLI2SP
                 | STM32_RCC_PLLI2SCFGR_PLLI2SQ
                 | STM32_RCC_PLLI2SCFGR_PLLI2SR);
      putreg32(regval, STM32_RCC_PLLI2SCFGR);

      regval  = getreg32(STM32_RCC_DCKCFGR2);
      regval &= ~(  RCC_DCKCFGR2_USART1SEL_MASK
                  | RCC_DCKCFGR2_USART2SEL_MASK
                  | RCC_DCKCFGR2_UART4SEL_MASK
                  | RCC_DCKCFGR2_UART5SEL_MASK
                  | RCC_DCKCFGR2_USART6SEL_MASK
                  | RCC_DCKCFGR2_UART7SEL_MASK
                  | RCC_DCKCFGR2_UART8SEL_MASK
                  | RCC_DCKCFGR2_I2C1SEL_MASK
                  | RCC_DCKCFGR2_I2C2SEL_MASK
                  | RCC_DCKCFGR2_I2C3SEL_MASK
                  | RCC_DCKCFGR2_I2C4SEL_MASK
                  | RCC_DCKCFGR2_LPTIM1SEL_MASK
                  | RCC_DCKCFGR2_CECSEL_MASK
                  | RCC_DCKCFGR2_CK48MSEL_MASK
                  | RCC_DCKCFGR2_SDMMCSEL_MASK);

      regval |= (  STM32_RCC_DCKCFGR2_USART1SRC
                 | STM32_RCC_DCKCFGR2_USART2SRC
                 | STM32_RCC_DCKCFGR2_UART4SRC
                 | STM32_RCC_DCKCFGR2_UART5SRC
                 | STM32_RCC_DCKCFGR2_USART6SRC
                 | STM32_RCC_DCKCFGR2_UART7SRC
                 | STM32_RCC_DCKCFGR2_UART8SRC
                 | STM32_RCC_DCKCFGR2_I2C1SRC
                 | STM32_RCC_DCKCFGR2_I2C2SRC
                 | STM32_RCC_DCKCFGR2_I2C3SRC
                 | STM32_RCC_DCKCFGR2_I2C4SRC
                 | STM32_RCC_DCKCFGR2_LPTIM1SRC
                 | STM32_RCC_DCKCFGR2_CECSRC
                 | STM32_RCC_DCKCFGR2_CK48MSRC
                 | STM32_RCC_DCKCFGR2_SDMMCSRC);

      putreg32(regval, STM32_RCC_DCKCFGR2);

      /* Enable PLLI2S */

      regval = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLI2SON;
      putreg32(regval, STM32_RCC_CR);

      /* Wait until the PLLI2S is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLI2SRDY) == 0)
        {
        }
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
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
