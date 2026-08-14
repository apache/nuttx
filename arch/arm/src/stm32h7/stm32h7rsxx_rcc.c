/****************************************************************************
 * arch/arm/src/stm32h7/stm32h7rsxx_rcc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32_pwr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow the same timeout for the HSI as for the HSE */

#define HSIRDY_TIMEOUT HSERDY_TIMEOUT

/* FLASH wait states */

#if !defined(BOARD_FLASH_WAITSTATES)
#  error BOARD_FLASH_WAITSTATES not defined
#elif BOARD_FLASH_WAITSTATES < 0 || BOARD_FLASH_WAITSTATES > 15
#  error BOARD_FLASH_WAITSTATES is out of range
#endif

/* The PMU supply.  The H7R/S has both an LDO and an SMPS; which one feeds
 * the core is a board wiring decision, and the SMPS is only offered where
 * the chip selects STM32_HAVE_SMPS.
 */

#define STM32_PWR_CSR2_MASK       ~(PWR_CSR2_BYPASS      | \
                                    PWR_CSR2_LDOEN       | \
                                    PWR_CSR2_SDEN        | \
                                    PWR_CSR2_SMPSEXTHP   | \
                                    PWR_CSR2_SDHILEVEL)

#if defined(CONFIG_STM32_PWR_DIRECT_SMPS_SUPPLY)
#  define STM32_PWR_CSR2_SELECTION PWR_CSR2_SDEN
#elif defined(CONFIG_STM32_PWR_EXTERNAL_SOURCE_SUPPLY)
#  define STM32_PWR_CSR2_SELECTION PWR_CSR2_BYPASS
#else
#  define STM32_PWR_CSR2_SELECTION PWR_CSR2_LDOEN
#endif

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

  /* Switch to HSI before disabling the PLL. */

  modifyreg32(STM32_RCC_CR, 0, RCC_CR_HSION);
  while ((getreg32(STM32_RCC_CR) & RCC_CR_HSIRDY) == 0)
    {
    }

  modifyreg32(STM32_RCC_CFGR, RCC_CFGR_SW_MASK, RCC_CFGR_SW_HSI);
  while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) !=
         RCC_CFGR_SWS_HSI)
    {
    }

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~(RCC_CR_PLL1ON | RCC_CR_HSEON | RCC_CR_HSEBYP);
  putreg32(regval, STM32_RCC_CR);
}

/****************************************************************************
 * Name: rcc_enableahb5
 *
 * Description:
 *   Enable selected AHB5 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb5(void)
{
  uint32_t regval;

  regval = getreg32(STM32_RCC_AHB5ENR);
#ifdef CONFIG_STM32_FMC
  regval |= RCC_AHB5ENR_FMCEN;
#endif
#ifdef CONFIG_STM32_SDMMC1
  regval |= RCC_AHB5ENR_SDMMC1EN;
#endif
#ifdef CONFIG_STM32_DMA2D
  regval |= RCC_AHB5ENR_DMA2DEN;
#endif
  putreg32(regval, STM32_RCC_AHB5ENR);
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

  regval = getreg32(STM32_RCC_AHB1ENR);
#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2)
  regval |= RCC_AHB1ENR_ADC12EN;
#endif
#ifdef CONFIG_STM32_ETHMAC
  regval |= RCC_AHB1ENR_ETH1MACEN | RCC_AHB1ENR_ETH1TXEN |
            RCC_AHB1ENR_ETH1RXEN;
#endif
#ifdef CONFIG_STM32_OTGHS
  regval |= RCC_AHB1ENR_OTGHSEN | RCC_AHB1ENR_USBPHYCEN;
#endif
#ifdef CONFIG_STM32_OTGFS
  regval |= RCC_AHB1ENR_OTGFSEN;
#endif
  putreg32(regval, STM32_RCC_AHB1ENR);
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

  regval = getreg32(STM32_RCC_AHB2ENR);
#ifdef CONFIG_STM32_SDMMC2
  regval |= RCC_AHB2ENR_SDMMC2EN;
#endif
  putreg32(regval, STM32_RCC_AHB2ENR);
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

  regval = getreg32(STM32_RCC_AHB3ENR);
#ifdef CONFIG_STM32_RNG
  regval |= RCC_AHB3ENR_RNGEN;
#endif
#ifdef CONFIG_STM32_HASH
  regval |= RCC_AHB3ENR_HASHEN;
#endif
#ifdef CONFIG_STM32_CRYP
  regval |= RCC_AHB3ENR_CRYPEN;
#endif
  putreg32(regval, STM32_RCC_AHB3ENR);
}

/****************************************************************************
 * Name: rcc_enableahb4
 *
 * Description:
 *   Enable selected AHB4 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb4(void)
{
  uint32_t regval;

  regval = getreg32(STM32_RCC_AHB4ENR);
  regval |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN |
            RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIODEN |
            RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIOFEN |
            RCC_AHB4ENR_GPIOGEN | RCC_AHB4ENR_GPIOHEN |
            RCC_AHB4ENR_GPIOMEN | RCC_AHB4ENR_GPIONEN |
            RCC_AHB4ENR_GPIOOEN | RCC_AHB4ENR_GPIOPEN;
#ifdef CONFIG_STM32_CRC
  regval |= RCC_AHB4ENR_CRCEN;
#endif
#ifdef CONFIG_STM32_BKPSRAM
  regval |= RCC_AHB4ENR_BKPRAMEN;
#endif
  putreg32(regval, STM32_RCC_AHB4ENR);
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

  regval = getreg32(STM32_RCC_APB1ENR1);
#ifdef CONFIG_STM32_TIM2
  regval |= RCC_APB1ENR1_TIM2EN;
#endif
#ifdef CONFIG_STM32_TIM3
  regval |= RCC_APB1ENR1_TIM3EN;
#endif
#ifdef CONFIG_STM32_TIM4
  regval |= RCC_APB1ENR1_TIM4EN;
#endif
#ifdef CONFIG_STM32_TIM5
  regval |= RCC_APB1ENR1_TIM5EN;
#endif
#ifdef CONFIG_STM32_TIM6
  regval |= RCC_APB1ENR1_TIM6EN;
#endif
#ifdef CONFIG_STM32_TIM7
  regval |= RCC_APB1ENR1_TIM7EN;
#endif
#ifdef CONFIG_STM32_TIM12
  regval |= RCC_APB1ENR1_TIM12EN;
#endif
#ifdef CONFIG_STM32_TIM13
  regval |= RCC_APB1ENR1_TIM13EN;
#endif
#ifdef CONFIG_STM32_TIM14
  regval |= RCC_APB1ENR1_TIM14EN;
#endif
#ifdef CONFIG_STM32_LPTIM1
  regval |= RCC_APB1ENR1_LPTIM1EN;
#endif
#ifdef CONFIG_STM32_SPI2
  regval |= RCC_APB1ENR1_SPI2EN;
#endif
#ifdef CONFIG_STM32_SPI3
  regval |= RCC_APB1ENR1_SPI3EN;
#endif
#ifdef CONFIG_STM32_USART2
  regval |= RCC_APB1ENR1_USART2EN;
#endif
#ifdef CONFIG_STM32_USART3
  regval |= RCC_APB1ENR1_USART3EN;
#endif
#ifdef CONFIG_STM32_UART4
  regval |= RCC_APB1ENR1_UART4EN;
#endif
#ifdef CONFIG_STM32_UART5
  regval |= RCC_APB1ENR1_UART5EN;
#endif
#ifdef CONFIG_STM32_UART7
  regval |= RCC_APB1ENR1_UART7EN;
#endif
#ifdef CONFIG_STM32_UART8
  regval |= RCC_APB1ENR1_UART8EN;
#endif
#ifdef CONFIG_STM32_I2C1
  regval |= RCC_APB1ENR1_I2C1_I3C1EN;
#endif
#ifdef CONFIG_STM32_I2C2
  regval |= RCC_APB1ENR1_I2C2EN;
#endif
#ifdef CONFIG_STM32_I2C3
  regval |= RCC_APB1ENR1_I2C3EN;
#endif
  putreg32(regval, STM32_RCC_APB1ENR1);

  regval = getreg32(STM32_RCC_APB1ENR2);
#if defined(CONFIG_STM32_FDCAN1) || defined(CONFIG_STM32_FDCAN2)
  regval |= RCC_APB1ENR2_FDCANEN;
#endif
  putreg32(regval, STM32_RCC_APB1ENR2);
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

  regval = getreg32(STM32_RCC_APB2ENR);
#ifdef CONFIG_STM32_TIM1
  regval |= RCC_APB2ENR_TIM1EN;
#endif
#ifdef CONFIG_STM32_USART1
  regval |= RCC_APB2ENR_USART1EN;
#endif
#ifdef CONFIG_STM32_SPI1
  regval |= RCC_APB2ENR_SPI1EN;
#endif
#ifdef CONFIG_STM32_SPI4
  regval |= RCC_APB2ENR_SPI4EN;
#endif
#ifdef CONFIG_STM32_TIM15
  regval |= RCC_APB2ENR_TIM15EN;
#endif
#ifdef CONFIG_STM32_TIM16
  regval |= RCC_APB2ENR_TIM16EN;
#endif
#ifdef CONFIG_STM32_TIM17
  regval |= RCC_APB2ENR_TIM17EN;
#endif
#ifdef CONFIG_STM32_TIM9
  regval |= RCC_APB2ENR_TIM9EN;
#endif
#ifdef CONFIG_STM32_SPI5
  regval |= RCC_APB2ENR_SPI5EN;
#endif
#ifdef CONFIG_STM32_SAI1
  regval |= RCC_APB2ENR_SAI1EN;
#endif
#ifdef CONFIG_STM32_SAI2
  regval |= RCC_APB2ENR_SAI2EN;
#endif
  putreg32(regval, STM32_RCC_APB2ENR);
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

  regval = getreg32(STM32_RCC_APB4ENR);
  regval |= RCC_APB4ENR_SBSEN;
#ifdef CONFIG_STM32_SPI6
  regval |= RCC_APB4ENR_SPI6EN;
#endif
#ifdef CONFIG_STM32_LPTIM2
  regval |= RCC_APB4ENR_LPTIM2EN;
#endif
#ifdef CONFIG_STM32_LPTIM3
  regval |= RCC_APB4ENR_LPTIM3EN;
#endif
#ifdef CONFIG_STM32_LPTIM4
  regval |= RCC_APB4ENR_LPTIM4EN;
#endif
#ifdef CONFIG_STM32_LPTIM5
  regval |= RCC_APB4ENR_LPTIM5EN;
#endif
  putreg32(regval, STM32_RCC_APB4ENR);
}

/****************************************************************************
 * Name: rcc_enableapb5
 *
 * Description:
 *   Enable selected APB5 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb5(void)
{
  uint32_t regval;

  regval = getreg32(STM32_RCC_APB5ENR);
#ifdef CONFIG_STM32_LTDC
  regval |= RCC_APB5ENR_LTDCEN;
#endif
  putreg32(regval, STM32_RCC_APB5ENR);
}

/****************************************************************************
 * Name: rcc_enableperipherals
 *
 * Description:
 *   Enable all configured peripheral clocks
 *
 ****************************************************************************/

static inline void rcc_enableperipherals(void)
{
  rcc_enableahb5();
  rcc_enableahb1();
  rcc_enableahb2();
  rcc_enableahb3();
  rcc_enableahb4();
  rcc_enableapb1();
  rcc_enableapb2();
  rcc_enableapb4();
  rcc_enableapb5();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 *
 ****************************************************************************/

void stm32_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

  /* Select the PMU supply the board uses.  As on the other H7 parts the low
   * byte of this register is written once after POR and must be written
   * before the VOS level or ck_sys is changed.  An invalid combination of
   * BYPASS, LDOEN and SDEN is ignored and locks the register, after which
   * ACTVOSRDY goes on reporting an invalid voltage level and the part must
   * be power cycled.
   */

  regval  = getreg32(STM32_PWR_CSR2);
  regval &= STM32_PWR_CSR2_MASK;
  regval |= STM32_PWR_CSR2_SELECTION;
  putreg32(regval, STM32_PWR_CSR2);

  /* Wait for the supply to settle on a valid voltage level.  Requesting a
   * new VOS level before this reports ready never completes.
   */

  while ((getreg32(STM32_PWR_SR1) & PWR_SR1_ACTVOSRDY) == 0)
    {
    }

  /* The VOS high level and seven flash wait states are required to run
   * at 600 MHz.
   */

  modifyreg32(STM32_PWR_CSR4, 0, PWR_CSR4_VOS);
  while ((getreg32(STM32_PWR_CSR4) & PWR_CSR4_VOSRDY) == 0)
    {
    }

  modifyreg32(STM32_FLASH_ACR,
              FLASH_ACR_LATENCY_MASK | FLASH_ACR_WRHIGHFREQ_MASK,
              FLASH_ACR_LATENCY(BOARD_FLASH_WAITSTATES) |
              FLASH_ACR_WRHIGHFREQ(3));
  while ((getreg32(STM32_FLASH_ACR) & FLASH_ACR_LATENCY_MASK) !=
         FLASH_ACR_LATENCY(BOARD_FLASH_WAITSTATES))
    {
    }

#ifdef STM32_BOARD_USEHSI
  /* Enable Internal High-Speed Clock (HSI) */

  modifyreg32(STM32_RCC_CR, 0, RCC_CR_HSION);

  /* Wait until the HSI is ready (or until a timeout elapsed) */

  for (timeout = HSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSIRDY) != 0)
        {
          break;
        }
    }
#else
  /* Enable External High-Speed Clock (HSE) */

  modifyreg32(STM32_RCC_CR, 0,
#ifdef STM32_HSEBYP_ENABLE
              RCC_CR_HSEBYP |
#endif
              RCC_CR_HSEON);

  /* Wait until the HSE is ready (or until a timeout elapsed) */

  for (timeout = HSERDY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSERDY) != 0)
        {
          break;
        }
    }
#endif

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan; the system will run from the HSI.
   */

  if (timeout <= 0)
    {
      return;
    }

#ifdef CONFIG_STM32_HSI48
  /* Enable HSI48 */

  modifyreg32(STM32_RCC_CR, 0, RCC_CR_HSI48ON);
  while ((getreg32(STM32_RCC_CR) & RCC_CR_HSI48RDY) == 0)
    {
    }
#endif

#ifdef CONFIG_STM32_CSI
  /* Enable CSI */

  modifyreg32(STM32_RCC_CR, 0, RCC_CR_CSION);
  while ((getreg32(STM32_RCC_CR) & RCC_CR_CSIRDY) == 0)
    {
    }
#endif

  /* Disable the PLL before it is configured */

  modifyreg32(STM32_RCC_CR, RCC_CR_PLL1ON, 0);
  while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL1RDY) != 0)
    {
    }

  /* Configure PLL1 source, dividers and outputs */

  putreg32(STM32_PLLCFG_PLLSRC | STM32_PLLCFG_PLL1M,
           STM32_RCC_PLLCKSELR);
  putreg32(STM32_PLLCFG_PLL1CFG, STM32_RCC_PLLCFGR);
  putreg32(STM32_PLLCFG_PLL1N | STM32_PLLCFG_PLL1P |
           STM32_PLLCFG_PLL1Q | STM32_PLLCFG_PLL1R,
           STM32_RCC_PLL1DIVR1);

  modifyreg32(STM32_RCC_CR, 0, RCC_CR_PLL1ON);
  while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL1RDY) == 0)
    {
    }

  /* Configure the CPU, bus matrix and APB prescalers */

  putreg32(STM32_RCC_CDCFGR_CPRE, STM32_RCC_CDCFGR);
  putreg32(STM32_RCC_BMCFGR_HPRE, STM32_RCC_BMCFGR);
  putreg32(STM32_RCC_APBCFGR_PPRE1 | STM32_RCC_APBCFGR_PPRE2 |
           STM32_RCC_APBCFGR_PPRE4 | STM32_RCC_APBCFGR_PPRE5,
           STM32_RCC_APBCFGR);

  /* Select PLL1P as the system clock source */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_SW_MASK;
  regval |= RCC_CFGR_SW_PLL1;
  putreg32(regval, STM32_RCC_CFGR);
  while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) !=
         RCC_CFGR_SWS_PLL1)
    {
    }
}
