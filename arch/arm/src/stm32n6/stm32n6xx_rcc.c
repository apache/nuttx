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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/stm32n6/chip.h>
#include <arch/board/board.h>

#include <assert.h>

#include "stm32_rcc.h"
#include "stm32_pwr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.
 */

#define HSIRDY_TIMEOUT  (100 * CONFIG_BOARD_LOOPSPERMSEC)
#define PLL1RDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
   * Matches the ST LL pattern in stm32n6xx_ll_bus.h.
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
 * Name: rcc_enableapb2
 *
 * Description:
 *   Enable selected APB2 peripherals.
 *
 ****************************************************************************/

static inline void rcc_enableapb2(void)
{
  uint32_t regval = 0;

#ifdef CONFIG_STM32N6_USART1
  regval |= RCC_APB2ENR_USART1EN;
#endif

  if (regval != 0)
    {
      putreg32(regval, STM32_RCC_APB2ENSR);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rcc_enableperipherals
 ****************************************************************************/

void stm32_rcc_enableperipherals(void)
{
  /* Enable all AXISRAM bank clocks.  The boot ROM only enables AXISRAM1/2
   * which is sufficient for code execution, but the NuttX heap extends
   * across all SRAM banks (up to AXISRAM5 at 0x34400000+).  Without these
   * clocks, mm_initialize writing the tail node at the heap end will cause
   * an IMPRECISERR bus fault.
   */

  putreg32(RCC_MEMENR_ALLAXISRAM | RCC_MEMENR_CACHEAXIRAMEN,
           STM32_RCC_MEMENSR);

  rcc_enableahb4();
  rcc_enableapb2();
}

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Configure PLL1 from values supplied by board.h (M, N, IC1_DIV).  The
 *   clock tree is laid out in the board header; see board.h for the full
 *   diagram.
 *
 *   IMPORTANT: CFGR1 locks after the first write -- CPUSW and SYSSW must
 *   be written together in a single putreg32().  CFGR2 (bus prescalers)
 *   also locks after CFGR1 is written, so it must be set first.
 *
 ****************************************************************************/

void stm32_stdclockconfig(void)
{
  volatile int32_t timeout;
  uint32_t regval;

  /* If clocks are already configured (e.g. FSBL set up PLL1 and switched
   * CPUSW to IC1), skip PLL1/CFGR1 reconfiguration.  CFGR1 locks after
   * the first write - a second write crashes the system (SRAM goes
   * offline).
   */

  regval = getreg32(STM32_RCC_CFGR1);
  if ((regval & RCC_CFGR1_CPUSWS_MASK) == RCC_CFGR1_CPUSWS_IC1 &&
      (regval & RCC_CFGR1_SYSSWS_MASK) == RCC_CFGR1_SYSSWS_IC2_IC6_IC11)
    {
      return;
    }

  for (timeout = HSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_SR) & RCC_SR_HSIRDY) != 0)
        {
          break;
        }
    }

  putreg32(RCC_CR_PLL1ON, STM32_RCC_CCR);

  for (timeout = PLL1RDY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_SR) & RCC_SR_PLL1RDY) == 0)
        {
          break;
        }
    }

  regval = (RCC_PLL1CFGR1_SEL_HSI)
         | (STM32_PLL1_M << RCC_PLL1CFGR1_DIVM_SHIFT)
         | (STM32_PLL1_N << RCC_PLL1CFGR1_DIVN_SHIFT);
  putreg32(regval, STM32_RCC_PLL1CFGR1);

  regval = RCC_PLL1CFGR3_MODSSDIS
         | RCC_PLL1CFGR3_PDIVEN
         | (1 << RCC_PLL1CFGR3_PDIV1_SHIFT)
         | (1 << RCC_PLL1CFGR3_PDIV2_SHIFT);
  putreg32(regval, STM32_RCC_PLL1CFGR3);

  putreg32(RCC_CR_PLL1ON, STM32_RCC_CSR);

  for (timeout = PLL1RDY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_SR) & RCC_SR_PLL1RDY) != 0)
        {
          break;
        }
    }

  /* IC dividers: register field is (divider - 1).  IC1_DIV from board.h
   * picks the CPU rate; the other ICs are fixed multiples that feed
   * SYSCLK and the XSPI2 kernel clock.
   *
   *   IC1  = VCO / IC1_DIV       -> CPU
   *   IC2  = VCO / (IC1_DIV * 2) -> SYSCLK
   *   IC3  = VCO / (IC1_DIV * 4) -> XSPI2 kernel clock (reserved)
   *   IC6  = VCO / (IC1_DIV * 3) -> SYSCLK
   *   IC11 = VCO / (IC1_DIV * 2) -> SYSCLK
   */

  putreg32(RCC_ICCFGR_SEL_PLL1
         | ((STM32_PLL1_IC1_DIV - 1) << RCC_ICCFGR_INT_SHIFT),
           STM32_RCC_IC1CFGR);
  putreg32(RCC_ICCFGR_SEL_PLL1
         | ((STM32_PLL1_IC1_DIV * 2 - 1) << RCC_ICCFGR_INT_SHIFT),
           STM32_RCC_IC2CFGR);
  putreg32(RCC_ICCFGR_SEL_PLL1
         | ((STM32_PLL1_IC1_DIV * 4 - 1) << RCC_ICCFGR_INT_SHIFT),
           STM32_RCC_IC3CFGR);
  putreg32(RCC_ICCFGR_SEL_PLL1
         | ((STM32_PLL1_IC1_DIV * 3 - 1) << RCC_ICCFGR_INT_SHIFT),
           STM32_RCC_IC6CFGR);
  putreg32(RCC_ICCFGR_SEL_PLL1
         | ((STM32_PLL1_IC1_DIV * 2 - 1) << RCC_ICCFGR_INT_SHIFT),
           STM32_RCC_IC11CFGR);

  putreg32(RCC_DIVENR_IC1EN | RCC_DIVENR_IC2EN | RCC_DIVENR_IC3EN
         | RCC_DIVENR_IC6EN | RCC_DIVENR_IC11EN,
           STM32_RCC_DIVENSR);

  /* CFGR2 (bus prescalers) and CFGR1 (clock-source switch) both lock
   * after CFGR1 is written, so CFGR2 must be set first and CFGR1 must
   * be written exactly once with both CPUSW and SYSSW in place.
   */

  putreg32(RCC_CFGR2_HPRE_SYSCLKd2, STM32_RCC_CFGR2);

  regval = getreg32(STM32_RCC_CFGR1);
  regval &= ~(RCC_CFGR1_CPUSW_MASK | RCC_CFGR1_SYSSW_MASK);
  regval |= RCC_CFGR1_CPUSW_IC1 | RCC_CFGR1_SYSSW_IC2_IC6_IC11;
  putreg32(regval, STM32_RCC_CFGR1);

  /* Some SRAM bank clocks drop out across the clock-domain switch on
   * STM32N6; re-arm them so the heap stays alive.
   */

  putreg32(RCC_MEMENR_ALLAXISRAM | RCC_MEMENR_CACHEAXIRAMEN,
           STM32_RCC_MEMENSR);

  for (timeout = PLL1RDY_TIMEOUT; timeout > 0; timeout--)
    {
      if (((getreg32(STM32_RCC_CFGR1) & RCC_CFGR1_CPUSWS_MASK)
            == RCC_CFGR1_CPUSWS_IC1) &&
          ((getreg32(STM32_RCC_CFGR1) & RCC_CFGR1_SYSSWS_MASK)
            == RCC_CFGR1_SYSSWS_IC2_IC6_IC11))
        {
          break;
        }
    }
}
