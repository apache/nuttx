/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_rcu.c
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

#include "chip.h"
#include "gd32f4xx_gpio.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow 2 milliseconds for the IRC16M to become ready. */

#define IRC16M_STARTUP_TIMEOUT   (2 * CONFIG_BOARD_LOOPSPERMSEC)

/* Allow 10 milliseconds for the HXTAL to become ready. */

#define HXTAL_STARTUP_TIMEOUT   (10 * CONFIG_BOARD_LOOPSPERMSEC)

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include chip-specific clocking initialization logic */

#if defined(CONFIG_GD32F4_GD32F4XX)
#else
#  error "Unknown GD32 chip"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RCU_MODIFY         do {                                              \
                                volatile uint32_t i;                         \
                                uint32_t cfg_regval;                         \
                                cfg_regval = getreg32(GD32_RCU_CFG0);        \
                                cfg_regval |= RCU_CFG0_AHBPSC_CKSYS_DIV2;    \
                                putreg32(cfg_regval, GD32_RCU_CFG0);         \
                                for(i = 0; i < 50000; i++);                        \
                                cfg_regval = getreg32(GD32_RCU_CFG0);        \
                                cfg_regval |= RCU_CFG0_AHBPSC_CKSYS_DIV4;    \
                                putreg32(cfg_regval, GD32_RCU_CFG0);         \
                                for(i = 0; i < 50000; i++);                        \
                              }                                              \
                              while(0);

/****************************************************************************
 * Pravite Functions
 ****************************************************************************/

#ifdef GD32_BOARD_SYSCLK_IRC16MEN 
/****************************************************************************
 * Name: gd32_system_clock_irc16m
 *
 * Description:
 *   Select the IRC16M as system clock.
 *
 ****************************************************************************/

static void gd32_system_clock_irc16m(void)
{
  uint32_t regval;
  uint32_t timeout = 0;
  uint32_t stab_flag = 0;

  /* Wait until IRC16M is stable or the startup time is longer than
   * IRC16M_STARTUP_TIMEOUT
   */

  do
    {
      timeout++;
      stab_flag = (getreg32(GD32_RCU_CTL) & RCU_CTL_IRC16MSTB);
    }
  while ((0 == stab_flag) && (IRC16M_STARTUP_TIMEOUT != timeout));

  /* If fail */

  if (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_IRC16MSTB))
    {
      while (1)
        {
        }
    }

  /* Set the AHB = SYSCLK */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_AHBPSC_MASK;
  regval |= RCU_CFG0_AHBPSC_CKSYS_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB2 = AHB */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB2PSC_MASK;
  regval |= RCU_CFG0_APB2PSC_CKAHB_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB1 = AHB */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB1PSC_MASK;
  regval |= RCU_CFG0_APB1PSC_CKAHB_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* Select IRC16M as system clock */

  regval = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  regval |= RCU_CFG0_SCS_IRC16M;
  putreg32(regval, GD32_RCU_CFG0);

  /* Wait until IRC16M is selected as system clock */

  while (0 != (getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_IRC16M))
    {
    }
}

#elif defined (GD32_BOARD_SYSCLK_HXTAL)
/****************************************************************************
 * Name: gd32_system_clock_hxtal
 *
 * Description:
 *   Select the HXTAL as system clock.
 *
 ****************************************************************************/

static void gd32_system_clock_hxtal(void)
{
  uint32_t regval;
  uint32_t timeout = 0;
  uint32_t stab_flag = 0;

  /* Enable HXTAL */

#ifdef GD32_RCU_CTL_HXTALBPSEN

  /* Bypass HXTAL oscillator when using the external clock which drives the
   * OSCIN pin.
   * If use a crystal with HXTAL, do not define GD32_RCU_CTL_HXTALBPSEN.
   */

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_HXTALBPS;
  putreg32(regval, GD32_RCU_CTL);

#endif

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_HXTALEN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until HXTAL is stable or the startup time is longer than
   * HXTAL_STARTUP_TIMEOUT
   */

  do
    {
      timeout++;
      stab_flag = (getreg32(GD32_RCU_CTL) & RCU_CTL_HXTALSTB);
    }
  while ((0 == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

  /* If fail */

  if (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_HXTALSTB))
    {
      while (1)
        {
        }
    }

  /* Set the AHB = SYSCLK */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_AHBPSC_MASK;
  regval |= RCU_CFG0_AHBPSC_CKSYS_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB2 = AHB */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB2PSC_MASK;
  regval |= RCU_CFG0_APB2PSC_CKAHB_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB1 = AHB */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB1PSC_MASK;
  regval |= RCU_CFG0_APB1PSC_CKAHB_DIV1;
  putreg32(regval, GD32_RCU_CFG0);

  /* select HXTAL as system clock */

  regval = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  regval |= RCU_CFG0_SCS_HXTAL;
  putreg32(regval, GD32_RCU_CFG0);

  /* wait until HXTAL is selected as system clock */

  while (0 != (getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_HXTAL))
    {
    }
}

#elif defined (GD32_BOARD_SYSCLK_PLL_IRC16M)
/****************************************************************************
 * Name: gd32_system_clock_pll_irc16m
 *
 * Description:
 *   Configure the system clock by PLL which selects IRC16M as its clock
 *   source.
 *
 ****************************************************************************/

static void gd32_system_clock_pll_irc16m(void)
{
  uint32_t regval;
  uint32_t timeout = 0;
  uint32_t stab_flag = 0;

  /* Wait until IRC16M is stable or the startup time is longer than
   * IRC16M_STARTUP_TIMEOUT
   */

  do
    {
      timeout++;
      stab_flag = (getreg32(GD32_RCU_CTL) & RCU_CTL_IRC16MSTB);
    }
  while ((0 == stab_flag) && (IRC16M_STARTUP_TIMEOUT != timeout));

  /* If fail */

  if (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_IRC16MSTB))
    {
      while (1)
        {
        }
    }

  /* LDO output voltage high mode */

  regval  = getreg32(GD32_RCU_APB1EN);
  regval |= RCU_APB1EN_PMUEN;
  putreg32(regval, GD32_RCU_APB1EN);

  regval  = getreg32(GD32_PMU_CTL);
  regval &= ~PMU_CTL_LDOVS_MASK;
  regval |= PMU_LDOVS_HIGH;
  putreg32(regval, GD32_PMU_CTL);

  /* Set the AHB = SYSCLK */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_AHBPSC_MASK;
  regval |= GD32_RCU_CFG0_AHB_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB2 = AHB/2 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB2PSC_MASK;
  regval |= GD32_RCU_CFG0_APB2_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB1 = AHB/4 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB1PSC_MASK;
  regval |= GD32_RCU_CFG0_APB1_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Configure the main PLL, and set PSC, PLL_N, PLL_P, PLL_Q */

  regval  = getreg32(GD32_RCU_PLL);
  regval = (GD32_PLL_PLLPSC | GD32_PLL_PLLN | GD32_PLL_PLLP
            | GD32_PLL_PLLQ | RCU_PLL_PLLSEL_IRC16M);
  putreg32(regval, GD32_RCU_PLL);

  /* Enable the main PLL */

  regval  = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_PLLEN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until the PLL is ready */

  while ((getreg32(GD32_RCU_CTL) & RCU_CTL_PLLSTB) == 0)
    {
    }

  /* Enable the high-drive to extend the clock frequency to up to 200 Mhz */

  regval  = getreg32(GD32_PMU_CTL);
  regval |= PMU_CTL_HDEN;
  putreg32(regval, GD32_PMU_CTL);
  while ((getreg32(GD32_PMU_CS) & PMU_CS_HDRF) == 0)
    {
    }

  /* Select the high-drive mode */

  regval  = getreg32(GD32_PMU_CTL);
  regval |= PMU_CTL_HDS;
  putreg32(regval, GD32_PMU_CTL);
  while ((getreg32(GD32_PMU_CS) & PMU_CS_HDSRF) == 0)
    {
    }

  /* Select PLL as system clock */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  regval |= RCU_CFG0_SCS_PLLP;
  putreg32(regval, GD32_RCU_CFG0);

  /* Wait until PLL is selected as system clock */

  while ((getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_MASK)
          != RCU_CFG0_SCSS_PLLP)
    {
    }
}

#elif defined (GD32_BOARD_SYSCLK_PLL_HXTAL)
/****************************************************************************
 * Name: gd32_system_clock_pll_hxtal
 *
 * Description:
 *   Configure the system clock by PLL which selects HXTAL as its clock
 *   source.
 *
 ****************************************************************************/

static void gd32_system_clock_pll_hxtal(void)
{
  uint32_t regval;
  uint32_t timeout = 0;
  uint32_t stab_flag = 0;

  /* enable HXTAL */

#ifdef GD32_RCU_CTL_HXTALBPSEN

  /* Bypass HXTAL oscillator when using the external clock which drives the
   * OSCIN pin.
   * If use a crystal with HXTAL, do not define GD32_RCU_CTL_HXTALBPSEN.
   */

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_HXTALBPS;
  putreg32(regval, GD32_RCU_CTL);

#endif

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_HXTALEN;
  putreg32(regval, GD32_RCU_CTL);

  /* wait until HXTAL is stable or the startup time is longer than
   * HXTAL_STARTUP_TIMEOUT
   */

  do
    {
      timeout++;
      stab_flag = (getreg32(GD32_RCU_CTL) & RCU_CTL_HXTALSTB);
    }
  while ((0 == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

  /* if fail */

  if (0U == (getreg32(GD32_RCU_CTL) & RCU_CTL_HXTALSTB))
    {
      while (1)
        {
        }
    }

  /* LDO output voltage high mode */

  regval  = getreg32(GD32_RCU_APB1EN);
  regval |= RCU_APB1EN_PMUEN;
  putreg32(regval, GD32_RCU_APB1EN);

  regval  = getreg32(GD32_PMU_CTL);
  regval &= ~PMU_CTL_LDOVS_MASK;
  regval |= PMU_LDOVS_HIGH;
  putreg32(regval, GD32_PMU_CTL);

  /* Set the AHB = SYSCLK */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_AHBPSC_MASK;
  regval |= GD32_RCU_CFG0_AHB_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB2 = AHB/2 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB2PSC_MASK;
  regval |= GD32_RCU_CFG0_APB2_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Set the APB1 = AHB/4 */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_APB1PSC_MASK;
  regval |= GD32_RCU_CFG0_APB1_PSC;
  putreg32(regval, GD32_RCU_CFG0);

  /* Configure the main PLL, and set PSC, PLL_N, PLL_P, PLL_Q */

  regval  = getreg32(GD32_RCU_PLL);
  regval = (GD32_PLL_PLLPSC | GD32_PLL_PLLN | GD32_PLL_PLLP
            | GD32_PLL_PLLQ | RCU_PLL_PLLSEL_HXTAL);
  putreg32(regval, GD32_RCU_PLL);

  /* Enable the main PLL */

  regval  = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_PLLEN;
  putreg32(regval, GD32_RCU_CTL);

  /* Wait until the PLL is ready */

  while ((getreg32(GD32_RCU_CTL) & RCU_CTL_PLLSTB) == 0)
    {
    }

  /* Enable the high-drive to extend the clock frequency to up to 200 Mhz */

  regval  = getreg32(GD32_PMU_CTL);
  regval |= PMU_CTL_HDEN;
  putreg32(regval, GD32_PMU_CTL);
  while ((getreg32(GD32_PMU_CS) & PMU_CS_HDRF) == 0)
    {
    }

  /* Select the high-drive mode */

  regval  = getreg32(GD32_PMU_CTL);
  regval |= PMU_CTL_HDS;
  putreg32(regval, GD32_PMU_CTL);
  while ((getreg32(GD32_PMU_CS) & PMU_CS_HDSRF) == 0)
    {
    }

  /* Select PLL as system clock */

  regval  = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  regval |= RCU_CFG0_SCS_PLLP;
  putreg32(regval, GD32_RCU_CFG0);

  /* Wait until PLL is selected as system clock */

  while ((getreg32(GD32_RCU_CFG0) & RCU_CFG0_SCSS_MASK)
          != RCU_CFG0_SCSS_PLLP)
    {
    }
}
#endif

/****************************************************************************
 * Name: gd32_system_clock_config
 *
 * Description:
 *   Configure the system clock using the settings in board.h.
 *
 ****************************************************************************/

static void gd32_system_clock_config(void)
{
#ifdef GD32_BOARD_SYSCLK_IRC16MEN  

  /* Select IRC16M as SYSCLK based on board.h setting. */

  gd32_system_clock_irc16m();

#elif defined (GD32_BOARD_SYSCLK_HXTAL)

  /* Select HXTAL as SYSCLK based on board.h setting. */

  gd32_system_clock_hxtal();

#elif defined (GD32_BOARD_SYSCLK_PLL_IRC16M)

  /* Select PLL which source is IRC16M as SYSCLK based on board.h setting. */

  gd32_system_clock_pll_irc16m();

#elif defined (GD32_BOARD_SYSCLK_PLL_HXTAL)

  /* Select  PLL which source is HXTAL as SYSCLK based on board.h setting. */

  gd32_system_clock_pll_hxtal();

#else
  #error "Invalid system clock configuration."
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_clockconfig
 *
 * Description:
 *   Called to initialize the GD32F4.  This does whatever setup is needed to
 *   put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void gd32_clockconfig(void)
{
  uint32_t regval;

  /* Reset the RCU clock configuration to the default reset state */

  /* Set IRC16MEN bit */

  regval = getreg32(GD32_RCU_CTL);
  regval |= RCU_CTL_IRC16MEN;
  putreg32(regval, GD32_RCU_CTL);

  RCU_MODIFY

  regval = getreg32(GD32_RCU_CFG0);
  regval &= ~RCU_CFG0_SCS_MASK;
  putreg32(regval, GD32_RCU_CFG0);

  /* Reset HXTALEN, CKMEN and PLLEN bits */

  regval  = getreg32(GD32_RCU_CTL);
  regval &= ~(RCU_CTL_HXTALEN | RCU_CTL_CKMEN | RCU_CTL_PLLEN);
  putreg32(regval, GD32_RCU_CTL);

  /* Reset HXTALBPS bit */

  regval  = getreg32(GD32_RCU_CTL);
  regval &= ~RCU_CTL_HXTALBPS;
  putreg32(regval, GD32_RCU_CTL);

  /* Reset CFG0 register */

  putreg32(0x00000000, GD32_RCU_CFG0);

  /* wait until IRC16M is selected as system clock */

  regval  = getreg32(GD32_RCU_CFG0);
  while (0 != (regval & RCU_CFG0_SCSS_IRC16M))
    {
      regval  = getreg32(GD32_RCU_CFG0);
    }

  /* Reset PLL register */

  putreg32(0x24003010, GD32_RCU_PLL);

  /* Disable all interrupts */

  putreg32(0x00000000, GD32_RCU_INT);

#if defined(CONFIG_ARCH_BOARD_GD32F4_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  gd32_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions
   * in board.h
   */

  /* Configure the System clock source, PLL Multiplier and Divider factors,
   * AHB/APBx prescalers and Flash settings
   */

  gd32_system_clock_config();
#endif
}

/****************************************************************************
 * Name: gd32_rcu_ckout0_config
 *
 * Description:
 *   Configure the CK_OUT0 clock source and divider. CK_OUT0 is connected
 *   to PA8. PA8 should be configured in alternate function mode.
 *
 ****************************************************************************/

void gd32_rcu_ckout0_config(uint32_t src, uint32_t div)
{
  uint32_t regval;

  regval = getreg32(GD32_RCU_CFG0);
  regval &= ~(RCU_CFG0_CKOUT0SEL_MASK | RCU_CFG0_CKOUT0DIV_MASK);
  regval |= (src | div);
  putreg32(regval, GD32_RCU_CFG0);
}

/****************************************************************************
 * Name: gd32_rcu_ckout1_config
 *
 * Description:
 *   Configure the CK_OUT0 clock source and divider. CK_OUT0 is connected
 *   to PC9. PC9 should be configured in alternate function mode.
 ****************************************************************************/

void gd32_rcu_ckout1_config(uint32_t src, uint32_t div)
{
  uint32_t regval;

  regval = getreg32(GD32_RCU_CFG0);
  regval &= ~(RCU_CFG0_CKOUT1SEL_MASK | RCU_CFG0_CKOUT1DIV_MASK);
  regval |= (src | div);
  putreg32(regval, GD32_RCU_CFG0);
}

/****************************************************************************
 * Name: gd32_rcu_periph_clock_enable
 *
 * Description:
 *   Enable the peripherals clock.
 *
 ****************************************************************************/

void gd32_rcu_periph_clock_enable(uint32_t periph)
{
  uint32_t reg_off;
  uint32_t bit_pos;
  uint32_t regaddr;
  uint32_t regval;

  bit_pos = periph & 0x1f;
  reg_off = (periph >> RCU_PERI_REG_SHIFT);
  regaddr = GD32_RCU_BASE + reg_off;

  regval = getreg32(regaddr);
  regval |= (1 << bit_pos);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_rcu_periph_clock_disable
 *
 * Description:
 *   Enable the peripherals clock.
 *
 ****************************************************************************/

void gd32_rcu_periph_clock_disable(uint32_t periph)
{
  uint32_t reg_off;
  uint32_t bit_pos;
  uint32_t regaddr;
  uint32_t regval;

  bit_pos = periph & 0x1f;
  reg_off = (periph >> RCU_PERI_REG_SHIFT);
  regaddr = GD32_RCU_BASE + reg_off;

  regval = getreg32(regaddr);
  regval &= ~(1 << bit_pos);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_clock_enable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings
 *   in board.h. This function is only available to support low-power
 *   modes of operation:  When re-awakening from deep-sleep modes, it is
 *   necessary to re-enable/re-start the PLL
 *
 *   This functional performs a subset of the operations performed by
 *   gd32_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_GD32F4_CUSTOM_CLOCKCONFIG is defined, then clocking
 *   will be enabled by an externally provided, board-specific function
 *   called gd32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM

void gd32_clock_enable(void)
{
#if defined(CONFIG_ARCH_BOARD_GD32F4_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  gd32_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions
   * in board.h
   */

  /* Configure the System clock source, PLL Multiplier and Divider factors,
   * AHB/APBx prescalers and Flash settings
   */

  gd32_system_clock_config();

#endif
}
#endif
