/****************************************************************************
 * arch/arm/src/stm32n6/stm32n6xx_rcc.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Board clock tree contract (board.h)
 *
 * Oscillators:
 *   STM32_HSI_FREQUENCY          64000000 (HSIDIV=1)
 *   STM32_BOARD_USEHSE           Enable the HSE oscillator
 *   STM32_HSE_FREQUENCY          HSE frequency in Hz
 *   STM32_HSE_BYPASS             HSE is an external clock, not a crystal
 *   STM32_HSE_DIV2               Route HSE/2 to the hse_div2 clock
 *
 * Voltage scaling:
 *   STM32_BOARD_SMPS_SUPPLY      VDDCORE from the internal SMPS step-down
 *                                converter (default: external supply)
 *   STM32_BOARD_VOS_HIGH         Select VOS high (CPU above 600 MHz)
 *   BOARD_VDDCORE_OVERDRIVE_GPIO GPIO driven before VOS high is selected
 *                                (board SMPS voltage select)
 *
 * PLLs (n = 1..4), enabled when STM32_PLLn_N is defined:
 *   STM32_PLLn_SRC               RCC_PLL1CFGR1_SEL_{HSI,MSI,HSE,I2S_CKIN}
 *   STM32_PLLn_M, STM32_PLLn_N   Reference divider and feedback divider
 *   STM32_PLLn_FRACN             Fractional part (default 0)
 *   STM32_PLLn_P1, STM32_PLLn_P2 Post dividers (default 1)
 *
 * Integrated clock dividers (n = 1..20), enabled when STM32_ICn_DIV is
 * defined:
 *   STM32_ICn_SRC                RCC_ICCFGR_SEL_PLL{1,2,3,4}
 *   STM32_ICn_DIV                Integer divider 1..256
 *
 * Clock switches and bus prescalers (defaults in parentheses):
 *   STM32_RCC_CFGR1_CPUSW        (RCC_CFGR1_CPUSW_IC1)
 *   STM32_RCC_CFGR1_SYSSW        (RCC_CFGR1_SYSSW_IC2_IC6_IC11)
 *   STM32_RCC_CFGR2_HPRE         (RCC_CFGR2_HPRE_SYSCLKd2)
 *   STM32_RCC_CFGR2_PPRE1/2/4/5  (RCC_CFGR2_PPREx_HCLK)
 *   STM32_RCC_CFGR2_TIMPRE       (RCC_CFGR2_TIMPRE_SYSCLK)
 *
 * Peripheral kernel clocks, applied when defined:
 *   STM32_RCC_CCIPRx_<PERIPH>SEL RCC_CCIPRx_<PERIPH>SEL_<source>
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/stm32n6/chip.h>
#include <arch/board/board.h>

#include <assert.h>

#include "stm32_rcc.h"
#include "stm32_pwr.h"
#include "stm32_gpio.h"
#include "hardware/stm32n6xxx_ramcfg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for a clock to become ready.  That is a
 * very long delay, but if the clock does not become ready we are hosed
 * anyway.  The LSE crystal may need up to a few seconds to start.
 */

#define CLKRDY_TIMEOUT  (100 * CONFIG_BOARD_LOOPSPERMSEC)
#define LSERDY_TIMEOUT  (3000 * CONFIG_BOARD_LOOPSPERMSEC)

/* Defaults for optional board.h settings */

#ifndef STM32_RCC_CFGR1_CPUSW
#  define STM32_RCC_CFGR1_CPUSW   RCC_CFGR1_CPUSW_IC1
#endif

#ifndef STM32_RCC_CFGR1_SYSSW
#  define STM32_RCC_CFGR1_SYSSW   RCC_CFGR1_SYSSW_IC2_IC6_IC11
#endif

#ifndef STM32_RCC_CFGR2_HPRE
#  define STM32_RCC_CFGR2_HPRE    RCC_CFGR2_HPRE_SYSCLKd2
#endif

#ifndef STM32_RCC_CFGR2_PPRE1
#  define STM32_RCC_CFGR2_PPRE1   RCC_CFGR2_PPRE1_HCLK
#endif

#ifndef STM32_RCC_CFGR2_PPRE2
#  define STM32_RCC_CFGR2_PPRE2   RCC_CFGR2_PPRE2_HCLK
#endif

#ifndef STM32_RCC_CFGR2_PPRE4
#  define STM32_RCC_CFGR2_PPRE4   RCC_CFGR2_PPRE4_HCLK
#endif

#ifndef STM32_RCC_CFGR2_PPRE5
#  define STM32_RCC_CFGR2_PPRE5   RCC_CFGR2_PPRE5_HCLK
#endif

#ifndef STM32_RCC_CFGR2_TIMPRE
#  define STM32_RCC_CFGR2_TIMPRE  RCC_CFGR2_TIMPRE_SYSCLK
#endif

#define RCC_CFGR1_SW_MASK         (RCC_CFGR1_CPUSW_MASK | RCC_CFGR1_SYSSW_MASK)
#define RCC_CFGR1_SWS_MASK        (RCC_CFGR1_CPUSWS_MASK | \
                                   RCC_CFGR1_SYSSWS_MASK)

/* The CPUSWS/SYSSWS status fields mirror the CPUSW/SYSSW request fields
 * four bits higher.
 */

#define RCC_CFGR1_SW2SWS(sw)      ((sw) << 4)

/* PLLn register block: CFGR1, CFGR2 and CFGR3 at 16-byte stride */

#define STM32_RCC_PLLCFGR1(n)     (STM32_RCC_PLL1CFGR1 + ((n) - 1) * 0x10)
#define STM32_RCC_PLLCFGR2(n)     (STM32_RCC_PLL1CFGR2 + ((n) - 1) * 0x10)
#define STM32_RCC_PLLCFGR3(n)     (STM32_RCC_PLL1CFGR3 + ((n) - 1) * 0x10)
#define RCC_CR_PLLON(n)           (RCC_CR_PLL1ON << ((n) - 1))
#define RCC_SR_PLLRDY(n)          (RCC_SR_PLL1RDY << ((n) - 1))

/* ICn configuration registers at 4-byte stride */

#define STM32_RCC_ICCFGR(n)       (STM32_RCC_IC1CFGR + ((n) - 1) * 4)
#define RCC_DIVENR_ICEN(n)        (1 << ((n) - 1))

#ifndef STM32_PLL1_FRACN
#  define STM32_PLL1_FRACN        0
#endif
#ifndef STM32_PLL2_FRACN
#  define STM32_PLL2_FRACN        0
#endif
#ifndef STM32_PLL3_FRACN
#  define STM32_PLL3_FRACN        0
#endif
#ifndef STM32_PLL4_FRACN
#  define STM32_PLL4_FRACN        0
#endif

#ifndef STM32_PLL1_P1
#  define STM32_PLL1_P1           1
#endif
#ifndef STM32_PLL1_P2
#  define STM32_PLL1_P2           1
#endif
#ifndef STM32_PLL2_P1
#  define STM32_PLL2_P1           1
#endif
#ifndef STM32_PLL2_P2
#  define STM32_PLL2_P2           1
#endif
#ifndef STM32_PLL3_P1
#  define STM32_PLL3_P1           1
#endif
#ifndef STM32_PLL3_P2
#  define STM32_PLL3_P2           1
#endif
#ifndef STM32_PLL4_P1
#  define STM32_PLL4_P1           1
#endif
#ifndef STM32_PLL4_P2
#  define STM32_PLL4_P2           1
#endif

/* Helper to program one PLL from its board.h macros */

#define RCC_CONFIG_PLL(n) \
  rcc_configpll(n, STM32_PLL##n##_SRC, STM32_PLL##n##_M, \
                STM32_PLL##n##_N, STM32_PLL##n##_FRACN, \
                STM32_PLL##n##_P1, STM32_PLL##n##_P2)

/* Helper to program one IC divider from its board.h macros */

#define RCC_CONFIG_IC(n) \
  do \
    { \
      rcc_configic(n, STM32_IC##n##_SRC, STM32_IC##n##_DIV); \
      divens |= RCC_DIVENR_ICEN(n); \
    } \
  while (0)

/* Helper to apply one kernel clock selection from board.h */

#define RCC_CONFIG_CCIP(reg, field) \
  modifyreg32(STM32_RCC_##reg, RCC_##reg##_##field##_MASK, \
              STM32_RCC_##reg##_##field)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_waitflag
 *
 * Description:
 *   Spin until all bits in mask of the register at addr are set (set ==
 *   true) or all are clear (set == false).  Returns false on timeout.
 *
 ****************************************************************************/

static bool rcc_waitflag(uint32_t addr, uint32_t mask, bool set)
{
  volatile int32_t timeout;

  for (timeout = CLKRDY_TIMEOUT; timeout > 0; timeout--)
    {
      uint32_t regval = getreg32(addr) & mask;

      if ((set && regval == mask) || (!set && regval == 0))
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: rcc_waitfield
 *
 * Description:
 *   Spin until the bits in mask of the register at addr equal value.
 *   Returns false on timeout.
 *
 ****************************************************************************/

static bool rcc_waitfield(uint32_t addr, uint32_t mask, uint32_t value)
{
  volatile int32_t timeout;

  for (timeout = CLKRDY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(addr) & mask) == value)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: rcc_enableahb1
 *
 * Description:
 *   Enable selected AHB1 peripherals.
 *
 ****************************************************************************/

static inline void rcc_enableahb1(void)
{
  uint32_t regval = 0;

#ifdef CONFIG_STM32_DMA1
  regval |= RCC_AHB1ENR_GPDMA1EN;
#endif

  if (regval != 0)
    {
      putreg32(regval, STM32_RCC_AHB1ENSR);
      putreg32(regval, STM32_RCC_AHB1LPENSR);
    }
}

/****************************************************************************
 * Name: rcc_enableahb3
 *
 * Description:
 *   Enable selected AHB3 peripherals.
 *
 ****************************************************************************/

static inline void rcc_enableahb3(void)
{
  uint32_t regval = 0;

#ifdef CONFIG_STM32_RNG
  regval |= RCC_AHB3ENR_RNGEN;
#endif

  if (regval != 0)
    {
      putreg32(regval, STM32_RCC_AHB3ENSR);
    }
}

/****************************************************************************
 * Name: rcc_enableahb5
 *
 * Description:
 *   Enable selected AHB5 peripherals.
 *
 ****************************************************************************/

static inline void rcc_enableahb5(void)
{
  uint32_t regval = 0;

#ifdef CONFIG_STM32_DMA2
  regval |= RCC_AHB5ENR_HPDMA1EN;
#endif

  if (regval != 0)
    {
      putreg32(regval, STM32_RCC_AHB5ENSR);
      putreg32(regval, STM32_RCC_AHB5LPENSR);
    }
}

/****************************************************************************
 * Name: rcc_enableahb4
 *
 * Description:
 *   Enable selected AHB4 peripherals - primarily GPIO clocks and PWR.
 *   On STM32N6, GPIO ports A-H and N-Q are on AHB4.
 *
 ****************************************************************************/

static inline void rcc_enableahb4(void)
{
  /* Use the write-1-to-set ENSR alias rather than a read-modify-write on
   * ENR so concurrent producers (e.g. RCC IRQ later) cannot lose bits.
   */

  putreg32(RCC_AHB4ENR_GPIOAEN
           | RCC_AHB4ENR_GPIOBEN
           | RCC_AHB4ENR_GPIOCEN
           | RCC_AHB4ENR_GPIODEN
           | RCC_AHB4ENR_GPIOEEN
           | RCC_AHB4ENR_GPIOFEN
           | RCC_AHB4ENR_GPIOGEN
           | RCC_AHB4ENR_GPIOHEN
           | RCC_AHB4ENR_GPIONEN
           | RCC_AHB4ENR_GPIOOEN
           | RCC_AHB4ENR_GPIOPEN
           | RCC_AHB4ENR_GPIOQEN
           | RCC_AHB4ENR_PWREN,
           STM32_RCC_AHB4ENSR);
}

/****************************************************************************
 * Name: rcc_enableapb1
 *
 * Description:
 *   Enable selected APB1 peripherals.
 *
 ****************************************************************************/

static inline void rcc_enableapb1(void)
{
  uint32_t regval = 0;

#ifdef CONFIG_STM32_WWDG
  regval |= RCC_APB1LENR_WWDGEN;
#endif

  if (regval != 0)
    {
      putreg32(regval, STM32_RCC_APB1LENSR);
    }
}

/****************************************************************************
 * Name: rcc_enableapb2
 *
 * Description:
 *   Enable selected APB2 peripherals.
 *
 ****************************************************************************/

static inline void rcc_enableapb2(void)
{
  uint32_t regval = 0;

#ifdef CONFIG_STM32_USART1
  regval |= RCC_APB2ENR_USART1EN;
#endif

  if (regval != 0)
    {
      putreg32(regval, STM32_RCC_APB2ENSR);
    }
}

/****************************************************************************
 * Name: rcc_enablehse
 *
 * Description:
 *   Configure and start the HSE oscillator (board option).
 *
 ****************************************************************************/

#ifdef STM32_BOARD_USEHSE
static void rcc_enablehse(void)
{
  uint32_t regval;

  regval = getreg32(STM32_RCC_HSECFGR);
  regval &= ~(RCC_HSECFGR_HSEBYP | RCC_HSECFGR_HSEEXT |
              RCC_HSECFGR_HSEDIV2SEL);
#ifdef STM32_HSE_BYPASS
  regval |= RCC_HSECFGR_HSEBYP;
#endif
#ifdef STM32_HSE_DIV2
  regval |= RCC_HSECFGR_HSEDIV2SEL;
#endif
  putreg32(regval, STM32_RCC_HSECFGR);

  putreg32(RCC_CR_HSEON, STM32_RCC_CSR);
  rcc_waitflag(STM32_RCC_SR, RCC_SR_HSERDY, true);
}
#endif

/****************************************************************************
 * Name: rcc_setvos
 *
 * Description:
 *   Select the VDDCORE voltage scaling range.  VOS high is required for
 *   CPU clocks above 600 MHz and requires the board SMPS to deliver the
 *   higher VDDCORE voltage, which the board may select through a GPIO.
 *   Called while running from HSI, so the frequency is low in both
 *   directions of the transition.
 *
 ****************************************************************************/

static void rcc_setvos(void)
{
  /* Select the VDDCORE supply configuration.  This must be written once
   * after power-on, before VOS or the system clock frequency change.
   */

#ifdef STM32_BOARD_SMPS_SUPPLY
  modifyreg32(STM32_PWR_CR1, 0, PWR_CR1_SDEN);
#else
  modifyreg32(STM32_PWR_CR1, PWR_CR1_SDEN, 0);
#endif

  rcc_waitflag(STM32_PWR_VOSCR, PWR_VOSCR_ACTVOSRDY, true);

#ifdef STM32_BOARD_VOS_HIGH
#  ifdef BOARD_VDDCORE_OVERDRIVE_GPIO
  volatile int32_t timeout;

  stm32_configgpio(BOARD_VDDCORE_OVERDRIVE_GPIO);

  /* Give the external regulator time to settle at the new voltage */

  for (timeout = CONFIG_BOARD_LOOPSPERMSEC; timeout > 0; timeout--)
    {
    }
#  endif

  modifyreg32(STM32_PWR_VOSCR, 0, PWR_VOSCR_VOS);
#else
  modifyreg32(STM32_PWR_VOSCR, PWR_VOSCR_VOS, 0);
#endif

  rcc_waitflag(STM32_PWR_VOSCR, PWR_VOSCR_VOSRDY, true);
  rcc_waitflag(STM32_PWR_VOSCR, PWR_VOSCR_ACTVOSRDY, true);
}

/****************************************************************************
 * Name: rcc_configpll
 *
 * Description:
 *   Program and start PLLn.  Follows the sequence of the ST HAL
 *   (RCC_PLL_Config): disable, configure in integer or fractional mode
 *   with spread spectrum disabled, enable the post divider, start.
 *
 ****************************************************************************/

static void rcc_configpll(unsigned int n, uint32_t src, uint32_t divm,
                          uint32_t divn, uint32_t fracn, uint32_t p1,
                          uint32_t p2)
{
  uint32_t regval;

  DEBUGASSERT(n >= 1 && n <= 4);

  /* Stop the PLL and wait until it is really off */

  putreg32(RCC_CR_PLLON(n), STM32_RCC_CCR);
  rcc_waitflag(STM32_RCC_SR, RCC_SR_PLLRDY(n), false);

  /* Disable spread spectrum modulation */

  modifyreg32(STM32_RCC_PLLCFGR3(n), 0, RCC_PLL1CFGR3_MODSSDIS);

  /* Clock source, reference and feedback dividers, no bypass */

  regval  = getreg32(STM32_RCC_PLLCFGR1(n));
  regval &= ~(RCC_PLL1CFGR1_BYP | RCC_PLL1CFGR1_SEL_MASK |
              RCC_PLL1CFGR1_DIVM_MASK | RCC_PLL1CFGR1_DIVN_MASK);
  regval |= src;
  regval |= divm << RCC_PLL1CFGR1_DIVM_SHIFT;
  regval |= divn << RCC_PLL1CFGR1_DIVN_SHIFT;
  putreg32(regval, STM32_RCC_PLLCFGR1(n));

  /* Post dividers */

  regval  = getreg32(STM32_RCC_PLLCFGR3(n));
  regval &= ~(RCC_PLL1CFGR3_PDIV1_MASK | RCC_PLL1CFGR3_PDIV2_MASK);
  regval |= p1 << RCC_PLL1CFGR3_PDIV1_SHIFT;
  regval |= p2 << RCC_PLL1CFGR3_PDIV2_SHIFT;
  putreg32(regval, STM32_RCC_PLLCFGR3(n));

  /* Fractional part.  MODDSEN is cleared before writing so an updated
   * value is latched when it is set again.
   */

  putreg32(fracn << RCC_PLL1CFGR2_DIVNFRAC_SHIFT, STM32_RCC_PLLCFGR2(n));
  modifyreg32(STM32_RCC_PLLCFGR3(n), RCC_PLL1CFGR3_MODDSEN, 0);

  if (fracn != 0)
    {
      modifyreg32(STM32_RCC_PLLCFGR3(n), 0,
                  RCC_PLL1CFGR3_MODDSEN | RCC_PLL1CFGR3_DACEN);
    }

  /* Keep the modulator in reset and enable the post divider output */

  modifyreg32(STM32_RCC_PLLCFGR3(n), 0,
              RCC_PLL1CFGR3_MODSSRST | RCC_PLL1CFGR3_PDIVEN);

  /* Start the PLL */

  putreg32(RCC_CR_PLLON(n), STM32_RCC_CSR);
  rcc_waitflag(STM32_RCC_SR, RCC_SR_PLLRDY(n), true);
}

/****************************************************************************
 * Name: rcc_configic
 *
 * Description:
 *   Program ICn: PLL source and integer divider (register holds div - 1).
 *
 ****************************************************************************/

static void rcc_configic(unsigned int n, uint32_t src, unsigned int div)
{
  DEBUGASSERT(n >= 1 && n <= 20 && div >= 1 && div <= 256);

  putreg32(src | ((div - 1) << RCC_ICCFGR_INT_SHIFT),
           STM32_RCC_ICCFGR(n));
}

/****************************************************************************
 * Name: rcc_enableccip
 *
 * Description:
 *   Apply the peripheral kernel clock selections defined by board.h.
 *
 ****************************************************************************/

static void rcc_enableccip(void)
{
#ifdef STM32_RCC_CCIPR1_ADF1SEL
  RCC_CONFIG_CCIP(CCIPR1, ADF1SEL);
#endif
#ifdef STM32_RCC_CCIPR1_ADC12SEL
  RCC_CONFIG_CCIP(CCIPR1, ADC12SEL);
#endif
#ifdef STM32_RCC_CCIPR1_ADCPRE
  RCC_CONFIG_CCIP(CCIPR1, ADCPRE);
#endif
#ifdef STM32_RCC_CCIPR1_DCMIPPSEL
  RCC_CONFIG_CCIP(CCIPR1, DCMIPPSEL);
#endif
#ifdef STM32_RCC_CCIPR2_ETH1PTPSEL
  RCC_CONFIG_CCIP(CCIPR2, ETH1PTPSEL);
#endif
#ifdef STM32_RCC_CCIPR2_ETH1PTPDIV
  RCC_CONFIG_CCIP(CCIPR2, ETH1PTPDIV);
#endif
#ifdef STM32_RCC_CCIPR2_ETH1CLKSEL
  RCC_CONFIG_CCIP(CCIPR2, ETH1CLKSEL);
#endif
#ifdef STM32_RCC_CCIPR2_ETH1SEL
  RCC_CONFIG_CCIP(CCIPR2, ETH1SEL);
#endif
#ifdef STM32_RCC_CCIPR3_FDCANSEL
  RCC_CONFIG_CCIP(CCIPR3, FDCANSEL);
#endif
#ifdef STM32_RCC_CCIPR3_FMCSEL
  RCC_CONFIG_CCIP(CCIPR3, FMCSEL);
#endif
#ifdef STM32_RCC_CCIPR4_I2C1SEL
  RCC_CONFIG_CCIP(CCIPR4, I2C1SEL);
#endif
#ifdef STM32_RCC_CCIPR4_I2C2SEL
  RCC_CONFIG_CCIP(CCIPR4, I2C2SEL);
#endif
#ifdef STM32_RCC_CCIPR4_I2C3SEL
  RCC_CONFIG_CCIP(CCIPR4, I2C3SEL);
#endif
#ifdef STM32_RCC_CCIPR4_I2C4SEL
  RCC_CONFIG_CCIP(CCIPR4, I2C4SEL);
#endif
#ifdef STM32_RCC_CCIPR4_I3C1SEL
  RCC_CONFIG_CCIP(CCIPR4, I3C1SEL);
#endif
#ifdef STM32_RCC_CCIPR4_I3C2SEL
  RCC_CONFIG_CCIP(CCIPR4, I3C2SEL);
#endif
#ifdef STM32_RCC_CCIPR4_LTDCSEL
  RCC_CONFIG_CCIP(CCIPR4, LTDCSEL);
#endif
#ifdef STM32_RCC_CCIPR5_MDF1SEL
  RCC_CONFIG_CCIP(CCIPR5, MDF1SEL);
#endif
#ifdef STM32_RCC_CCIPR6_XSPI1SEL
  RCC_CONFIG_CCIP(CCIPR6, XSPI1SEL);
#endif
#ifdef STM32_RCC_CCIPR6_XSPI2SEL
  RCC_CONFIG_CCIP(CCIPR6, XSPI2SEL);
#endif
#ifdef STM32_RCC_CCIPR6_XSPI3SEL
  RCC_CONFIG_CCIP(CCIPR6, XSPI3SEL);
#endif
#ifdef STM32_RCC_CCIPR6_OTGPHY1SEL
  RCC_CONFIG_CCIP(CCIPR6, OTGPHY1SEL);
#endif
#ifdef STM32_RCC_CCIPR6_OTGPHY2SEL
  RCC_CONFIG_CCIP(CCIPR6, OTGPHY2SEL);
#endif
#ifdef STM32_RCC_CCIPR7_PERSEL
  RCC_CONFIG_CCIP(CCIPR7, PERSEL);
#endif
#ifdef STM32_RCC_CCIPR7_PSSISEL
  RCC_CONFIG_CCIP(CCIPR7, PSSISEL);
#endif
#ifdef STM32_RCC_CCIPR7_SAI1SEL
  RCC_CONFIG_CCIP(CCIPR7, SAI1SEL);
#endif
#ifdef STM32_RCC_CCIPR7_SAI2SEL
  RCC_CONFIG_CCIP(CCIPR7, SAI2SEL);
#endif
#ifdef STM32_RCC_CCIPR8_SDMMC1SEL
  RCC_CONFIG_CCIP(CCIPR8, SDMMC1SEL);
#endif
#ifdef STM32_RCC_CCIPR8_SDMMC2SEL
  RCC_CONFIG_CCIP(CCIPR8, SDMMC2SEL);
#endif
#ifdef STM32_RCC_CCIPR9_SPDIFRX1SEL
  RCC_CONFIG_CCIP(CCIPR9, SPDIFRX1SEL);
#endif
#ifdef STM32_RCC_CCIPR9_SPI1SEL
  RCC_CONFIG_CCIP(CCIPR9, SPI1SEL);
#endif
#ifdef STM32_RCC_CCIPR9_SPI2SEL
  RCC_CONFIG_CCIP(CCIPR9, SPI2SEL);
#endif
#ifdef STM32_RCC_CCIPR9_SPI3SEL
  RCC_CONFIG_CCIP(CCIPR9, SPI3SEL);
#endif
#ifdef STM32_RCC_CCIPR9_SPI4SEL
  RCC_CONFIG_CCIP(CCIPR9, SPI4SEL);
#endif
#ifdef STM32_RCC_CCIPR9_SPI5SEL
  RCC_CONFIG_CCIP(CCIPR9, SPI5SEL);
#endif
#ifdef STM32_RCC_CCIPR9_SPI6SEL
  RCC_CONFIG_CCIP(CCIPR9, SPI6SEL);
#endif
#ifdef STM32_RCC_CCIPR12_LPTIM1SEL
  RCC_CONFIG_CCIP(CCIPR12, LPTIM1SEL);
#endif
#ifdef STM32_RCC_CCIPR12_LPTIM2SEL
  RCC_CONFIG_CCIP(CCIPR12, LPTIM2SEL);
#endif
#ifdef STM32_RCC_CCIPR12_LPTIM3SEL
  RCC_CONFIG_CCIP(CCIPR12, LPTIM3SEL);
#endif
#ifdef STM32_RCC_CCIPR12_LPTIM4SEL
  RCC_CONFIG_CCIP(CCIPR12, LPTIM4SEL);
#endif
#ifdef STM32_RCC_CCIPR12_LPTIM5SEL
  RCC_CONFIG_CCIP(CCIPR12, LPTIM5SEL);
#endif
#ifdef STM32_RCC_CCIPR13_USART1SEL
  RCC_CONFIG_CCIP(CCIPR13, USART1SEL);
#endif
#ifdef STM32_RCC_CCIPR13_USART2SEL
  RCC_CONFIG_CCIP(CCIPR13, USART2SEL);
#endif
#ifdef STM32_RCC_CCIPR13_USART3SEL
  RCC_CONFIG_CCIP(CCIPR13, USART3SEL);
#endif
#ifdef STM32_RCC_CCIPR13_UART4SEL
  RCC_CONFIG_CCIP(CCIPR13, UART4SEL);
#endif
#ifdef STM32_RCC_CCIPR13_UART5SEL
  RCC_CONFIG_CCIP(CCIPR13, UART5SEL);
#endif
#ifdef STM32_RCC_CCIPR13_USART6SEL
  RCC_CONFIG_CCIP(CCIPR13, USART6SEL);
#endif
#ifdef STM32_RCC_CCIPR13_UART7SEL
  RCC_CONFIG_CCIP(CCIPR13, UART7SEL);
#endif
#ifdef STM32_RCC_CCIPR13_UART8SEL
  RCC_CONFIG_CCIP(CCIPR13, UART8SEL);
#endif
#ifdef STM32_RCC_CCIPR14_UART9SEL
  RCC_CONFIG_CCIP(CCIPR14, UART9SEL);
#endif
#ifdef STM32_RCC_CCIPR14_USART10SEL
  RCC_CONFIG_CCIP(CCIPR14, USART10SEL);
#endif
#ifdef STM32_RCC_CCIPR14_LPUART1SEL
  RCC_CONFIG_CCIP(CCIPR14, LPUART1SEL);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rcc_enablelse
 *
 * Description:
 *   Enable the 32.768 kHz LSE oscillator.  The LSE lives in the backup
 *   domain, so write access is enabled for the duration of the call.
 *
 ****************************************************************************/

void stm32_rcc_enablelse(void)
{
  volatile int32_t timeout;
  bool writable;

  if ((getreg32(STM32_RCC_SR) & RCC_SR_LSERDY) != 0)
    {
      return;
    }

  writable = stm32_pwr_enablebkp(true);

#ifdef CONFIG_STM32_RTC_LSECLOCK_START_DRV_CAPABILITY
  modifyreg32(STM32_RCC_LSECFGR, RCC_LSECFGR_LSEDRV_MASK,
              CONFIG_STM32_RTC_LSECLOCK_START_DRV_CAPABILITY <<
              RCC_LSECFGR_LSEDRV_SHIFT);
#endif

  putreg32(RCC_CR_LSEON, STM32_RCC_CSR);
  for (timeout = LSERDY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_SR) & RCC_SR_LSERDY) != 0)
        {
          break;
        }
    }

  stm32_pwr_enablebkp(writable);
}

/****************************************************************************
 * Name: stm32_rcc_enablelsi
 *
 * Description:
 *   Enable the 32 kHz LSI oscillator.
 *
 ****************************************************************************/

void stm32_rcc_enablelsi(void)
{
  putreg32(RCC_CR_LSION, STM32_RCC_CSR);
  rcc_waitflag(STM32_RCC_SR, RCC_SR_LSIRDY, true);
}

/****************************************************************************
 * Name: stm32_rcc_enableperipherals
 ****************************************************************************/

void stm32_rcc_enableperipherals(void)
{
  int n;

  /* Enable all AXISRAM bank clocks.  The boot ROM only enables AXISRAM1/2
   * which is sufficient for code execution, but the NuttX heap extends
   * across AXISRAM1-6 and the CACHEAXI RAM.  Without these clocks,
   * mm_initialize writing the tail node at the heap end will cause an
   * IMPRECISERR bus fault.
   */

  putreg32(RCC_MEMENR_ALLAXISRAM | RCC_MEMENR_CACHEAXIRAMEN,
           STM32_RCC_MEMENSR);

  /* AXISRAM3-6 also come out of reset shut down in RAMCFG: they read as
   * zero and ignore writes until SRAMSD is cleared.
   */

  putreg32(RCC_AHB2ENR_RAMCFGEN, STM32_RCC_AHB2ENSR);
  for (n = 3; n <= 6; n++)
    {
      modifyreg32(STM32_RAMCFG_AXISRAM_CR(n), RAMCFG_CR_SRAMSD, 0);
    }

  rcc_enableahb1();
  rcc_enableahb3();
  rcc_enableahb4();
  rcc_enableahb5();
  rcc_enableapb1();
  rcc_enableapb2();
  rcc_enableccip();
}

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Establish the board clock tree described in board.h: oscillators,
 *   VDDCORE voltage scaling, PLL1-4, IC1-20 dividers, bus prescalers and
 *   the CPU/system clock switches.
 *
 *   The CPU and system clocks are moved to HSI first, since the boot ROM
 *   may already be using PLL1 with a different configuration.
 *
 ****************************************************************************/

void stm32_stdclockconfig(void)
{
  uint32_t regval;
  uint32_t divens = 0;

  /* GPIO and PWR clocks are needed below for voltage scaling and the
   * board supply-select GPIO.
   */

  rcc_enableahb4();

  /* Keep HSI running while the clock tree is reconfigured. */

  putreg32(RCC_CR_HSION, STM32_RCC_CSR);
  rcc_waitflag(STM32_RCC_SR, RCC_SR_HSIRDY, true);

  /* The flash boot ROM leaves CPUCLK at 400 MHz.  Move both CPU and
   * system clocks off PLL1 before touching it, then establish the board
   * clock tree instead of inheriting the ROM's configuration.
   */

  modifyreg32(STM32_RCC_CFGR1, RCC_CFGR1_SW_MASK, 0);
  rcc_waitflag(STM32_RCC_CFGR1, RCC_CFGR1_SWS_MASK, false);

#ifdef STM32_BOARD_USEHSE
  rcc_enablehse();
#endif

  /* Select the VDDCORE range before raising the frequency */

  rcc_setvos();

  /* PLLs */

#ifdef STM32_PLL1_N
  RCC_CONFIG_PLL(1);
#endif
#ifdef STM32_PLL2_N
  RCC_CONFIG_PLL(2);
#endif
#ifdef STM32_PLL3_N
  RCC_CONFIG_PLL(3);
#endif
#ifdef STM32_PLL4_N
  RCC_CONFIG_PLL(4);
#endif

  /* IC dividers */

#ifdef STM32_IC1_DIV
  RCC_CONFIG_IC(1);
#endif
#ifdef STM32_IC2_DIV
  RCC_CONFIG_IC(2);
#endif
#ifdef STM32_IC3_DIV
  RCC_CONFIG_IC(3);
#endif
#ifdef STM32_IC4_DIV
  RCC_CONFIG_IC(4);
#endif
#ifdef STM32_IC5_DIV
  RCC_CONFIG_IC(5);
#endif
#ifdef STM32_IC6_DIV
  RCC_CONFIG_IC(6);
#endif
#ifdef STM32_IC7_DIV
  RCC_CONFIG_IC(7);
#endif
#ifdef STM32_IC8_DIV
  RCC_CONFIG_IC(8);
#endif
#ifdef STM32_IC9_DIV
  RCC_CONFIG_IC(9);
#endif
#ifdef STM32_IC10_DIV
  RCC_CONFIG_IC(10);
#endif
#ifdef STM32_IC11_DIV
  RCC_CONFIG_IC(11);
#endif
#ifdef STM32_IC12_DIV
  RCC_CONFIG_IC(12);
#endif
#ifdef STM32_IC13_DIV
  RCC_CONFIG_IC(13);
#endif
#ifdef STM32_IC14_DIV
  RCC_CONFIG_IC(14);
#endif
#ifdef STM32_IC15_DIV
  RCC_CONFIG_IC(15);
#endif
#ifdef STM32_IC16_DIV
  RCC_CONFIG_IC(16);
#endif
#ifdef STM32_IC17_DIV
  RCC_CONFIG_IC(17);
#endif
#ifdef STM32_IC18_DIV
  RCC_CONFIG_IC(18);
#endif
#ifdef STM32_IC19_DIV
  RCC_CONFIG_IC(19);
#endif
#ifdef STM32_IC20_DIV
  RCC_CONFIG_IC(20);
#endif

  putreg32(divens, STM32_RCC_DIVENSR);

  /* Set the bus prescalers before switching CPU and system clocks to
   * the configured PLL outputs.
   */

  regval = STM32_RCC_CFGR2_HPRE | STM32_RCC_CFGR2_PPRE1 |
           STM32_RCC_CFGR2_PPRE2 | STM32_RCC_CFGR2_PPRE4 |
           STM32_RCC_CFGR2_PPRE5 | STM32_RCC_CFGR2_TIMPRE;
  putreg32(regval, STM32_RCC_CFGR2);

  /* Switch the CPU and system clocks */

  regval  = getreg32(STM32_RCC_CFGR1);
  regval &= ~RCC_CFGR1_SW_MASK;
  regval |= STM32_RCC_CFGR1_CPUSW | STM32_RCC_CFGR1_SYSSW;
  putreg32(regval, STM32_RCC_CFGR1);

  /* Some SRAM bank clocks drop out across the clock-domain switch on
   * STM32N6; re-arm them so the heap stays alive.
   */

  putreg32(RCC_MEMENR_ALLAXISRAM | RCC_MEMENR_CACHEAXIRAMEN,
           STM32_RCC_MEMENSR);

  rcc_waitfield(STM32_RCC_CFGR1, RCC_CFGR1_SWS_MASK,
                RCC_CFGR1_SW2SWS(STM32_RCC_CFGR1_CPUSW |
                                 STM32_RCC_CFGR1_SYSSW));
}
