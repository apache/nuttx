/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_clockconfig.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
 * Much of the logic within this file derives heavily from NXP sample code
 * for the S32K1xx MCUs.  That sample code has this licensing information:
 *
 *   Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 *   Copyright 2016-2018 NXP
 *   All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/power/pm.h>

#include "arm_internal.h"
#include "hardware/s32k1xx_scg.h"
#include "hardware/s32k1xx_smc.h"
#include "hardware/s32k1xx_sim.h"
#include "hardware/s32k1xx_pmc.h"
#include "s32k1xx_periphclocks.h"
#include "s32k1xx_clockconfig.h"
#include "s32k1xx_start.h"

#include <arch/board/board.h>  /* Include last.  May have dependencies */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Temporary system clock source configurations. */

#define TMP_SIRC_CLK    0
#define TMP_FIRC_CLK    1
#define TMP_SOSC_CLK    2
#define TMP_SPLL_CLK    3

#define TMP_SYS_DIV     0
#define TMP_BUS_DIV     1
#define TMP_SLOW_DIV    2

#define TMP_SYS_CLK_NO  4
#define TMP_SYS_DIV_NO  3

/* Supports arrays of  maximum clock frequencies of system clocks in all
 * power modes
 */

#define MODES_MAX_NO    7
#define SYS_CLK_MAX_NO  3
#define CORE_CLK_INDEX  0
#define BUS_CLK_INDEX   1
#define SLOW_CLK_INDEX  2

/* Time to wait for clocks to stabilize, ie., number of cycles when core
 * runs at maximum speed - 112 MHz.
 */

#define SIRC_STABILIZATION_TIMEOUT 100
#define FIRC_STABILIZATION_TIMEOUT 20
#define SOSC_STABILIZATION_TIMEOUT 3205000
#define SPLL_STABILIZATION_TIMEOUT 1000

/* System PLL reference clock after SCG_SPLLCFG[PREDIV] should be in the
 * range of SCG_SPLL_REF_MIN to SCG_SPLL_REF_MAX.
 */

#define SCG_SPLL_REF_MIN 8000000
#define SCG_SPLL_REF_MAX 32000000

/* Power management definitions */

#ifndef OK
#define OK 0
#endif

/****************************************************************************
 * Private Function Declarations
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int dowmin,
                         enum pm_state_e pmstate);
static int  up_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_tmp_sysclk[TMP_SYS_CLK_NO][TMP_SYS_DIV_NO] =
{
  {
    1,                                      /* SIRC SYS_CLK divider, range 1..16 */
    1,                                      /* SIRC BUS_CLK divider, range 1..16 */
    2                                       /* SIRC SLOW_CLK divider, range 1..16 */
  },
  {
    1,                                      /* FIRC SYS_CLK divider, range 1..16 */
    2,                                      /* FIRC BUS_CLK divider, range 1..16 */
    4                                       /* FIRC SLOW_CLK divider, range 1..16 */
  },
  {
    1,                                      /* SOSC SYS_CLK divider, range 1..16 */
    2,                                      /* SOSC BUS_CLK divider, range 1..16 */
    2                                       /* SOSC SLOW_CLK divider, range 1..16 */
  },
  {
    3,                                      /* SPLL SYS_CLK divider, range 1..16 */
    2,                                      /* SPLL BUS_CLK divider, range 1..16 */
    2                                       /* SPLL SLOW_CLK divider, range 1..16 */
  }
};

/* The maximum clock frequencies of system clocks in all power modes */

/* SYS_CLK      BUS_CLK      SLOW_CLK */

static const uint32_t g_vlpr_maxsysclks[MODES_MAX_NO][SYS_CLK_MAX_NO] =
{
  {         0ul,       0ul,         0ul },  /* Invalid entry */
  {   4000000ul, 4000000ul,   1000000ul },  /* Maximum frequencies when system clock is SOSC */
  {   4000000ul, 4000000ul,   1000000ul },  /* Maximum frequencies when system clock is SIRC */
  {   4000000ul, 4000000ul,   1000000ul },  /* Maximum frequencies when system clock is FIRC */
  {         0ul,       0ul,         0ul },  /* Invalid entry */
  {         0ul,       0ul,         0ul },  /* Invalid entry */
  {   4000000ul, 4000000ul,   1000000ul },  /* Maximum frequencies when system clock is SPLL */
};

/* SYS_CLK      BUS_CLK      SLOW_CLK */

static const uint32_t g_run_maxsysclks[MODES_MAX_NO][SYS_CLK_MAX_NO] =
{
  {         0ul,        0ul,         0ul }, /* Invalid entry */
  {  80000000ul, 48000000ul,  26670000ul }, /* Maximum frequencies when system clock is SOSC */
  {  80000000ul, 48000000ul,  26670000ul }, /* Maximum frequencies when system clock is SIRC */
  {  80000000ul, 48000000ul,  26670000ul }, /* Maximum frequencies when system clock is FIRC */
  {         0ul,        0ul,         0ul }, /* Invalid entry */
  {         0ul,        0ul,         0ul }, /* Invalid entry */
  {  80000000ul, 40000000ul,  26670000ul }, /* Maximum frequencies when system clock is SPLL */
};
#ifdef CONFIG_S32K1XX_HAVE_HSRUN

/* SYS_CLK      BUS_CLK      SLOW_CLK */

static const uint32_t g_hsrun_maxsysclks[MODES_MAX_NO][SYS_CLK_MAX_NO] =
{
  {         0ul,        0ul,         0ul },  /* Invalid entry */
  { 112000000ul, 56000000ul,  28000000ul },  /* Maximum frequencies when system clock is SOSC */
  { 112000000ul, 56000000ul,  28000000ul },  /* Maximum frequencies when system clock is SIRC */
  { 112000000ul, 56000000ul,  28000000ul },  /* Maximum frequencies when system clock is FIRC */
  {         0ul,        0ul,         0ul },  /* Invalid entry */
  {         0ul,        0ul,         0ul },  /* Invalid entry */
  { 112000000ul, 56000000ul,  28000000ul },  /* Maximum frequencies when system clock is SPLL */
};
#endif

#if 0 /* Not currently used */
static uint32_t g_rtc_clkin;                 /* RTC CLKIN clock */
#endif

#if 0 /* Not currently used */
static uint32_t g_tclkfreq[NUMBER_OF_TCLK_INPUTS];  /* TCLKx clocks */
#endif

#ifdef CONFIG_PM
static  struct pm_callback_s g_clock_pmcb =
{
  .notify       = up_pm_notify,
  .prepare      = up_pm_prepare,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_get_scgclk_source
 *
 * Description:
 *   Gets SCG current system clock source
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current system clock source.
 *
 ****************************************************************************/

static inline uint32_t s32k1xx_get_scgclk_source(void)
{
  return ((getreg32(S32K1XX_SCG_CSR) & SCG_CSR_SCS_MASK) >>
           SCG_CSR_SCS_SHIFT);
}

/****************************************************************************
 * Name: s32k1xx_get_soscfreq
 *
 * Description:
 *   Gets SCG System OSC clock frequency (SOSC).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The SOSC frequency.  Zero is returned if the SOSC is invalid.
 *
 ****************************************************************************/

static uint32_t s32k1xx_get_soscfreq(void)
{
  /* Check if the SOSC is valid */

  if ((getreg32(S32K1XX_SCG_SOSCCSR) & SCG_SOSCCSR_SOSCVLD) != 0)
    {
      return BOARD_XTAL_FREQUENCY;
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * Name: s32k1xx_get_sircfreq
 *
 * Description:
 *   Gets SCG Slow IRC clock frequency (SIRC).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The SIRC frequency.  Zero is returned if the SIRC is invalid.
 *
 ****************************************************************************/

static uint32_t s32k1xx_get_sircfreq(void)
{
  /* Check if the SIRC is valid */

  if ((getreg32(S32K1XX_SCG_SIRCCSR) & SCG_SIRCCSR_SIRCVLD) != 0)
    {
      /* Only high range is supported */

      if ((getreg32(S32K1XX_SCG_SIRCCFG) & SCG_SIRCCFG_RANGE) != 0)
        {
          return SCG_SIRC_HIGHRANGE_FREQUENCY;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: s32k1xx_get_fircfreq
 *
 * Description:
 *   Gets SCG Fast IRC clock frequency (FIRC).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The FIRC frequency.  Zero is returned if the FIRC is invalid.
 *
 ****************************************************************************/

static uint32_t s32k1xx_get_fircfreq(void)
{
  /* Check if the FIRC is valid */

  if ((getreg32(S32K1XX_SCG_FIRCCSR) & SCG_FIRCCSR_FIRCVLD) != 0)
    {
      return SCG_FIRC_FREQUENCY0;
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * Name: s32k1xx_get_spllfreq
 *
 * Description:
 *   Gets SCG System PLL clock frequency (SPLL).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The SPLL frequency.  Zero is returned if the SPLL is invalid.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_HAVE_SPLL
static uint32_t s32k1xx_get_spllfreq(void)
{
  uint32_t freq;
  uint32_t regval;
  uint32_t prediv;
  uint32_t mult;

  /* Check if the SPLL is valid */

  if ((getreg32(S32K1XX_SCG_SPLLCSR) & SCG_SPLLCSR_SPLLVLD) != 0)
    {
      /* Get System Oscillator frequency. */

      freq = s32k1xx_get_soscfreq();
      if (freq != 0)
        {
          regval = getreg32(S32K1XX_SCG_SPLLCFG);
          prediv = ((regval & SCG_SPLLCFG_PREDIV_MASK) >>
                     SCG_SPLLCFG_PREDIV_SHIFT) + 1;
          mult   = ((regval & SCG_SPLLCFG_MULT_MASK) >>
                     SCG_SPLLCFG_MULT_SHIFT) + 16;

          freq  /= prediv;
          freq  *= mult;
          freq >>= 1;  /* Divide VCO by 2. */
        }

      return freq;
    }
  else
    {
      return 0;
    }
}
#endif

/****************************************************************************
 * Name: s32k1xx_get_srcfreq
 *
 * Description:
 *   Return the clock source frequency.
 *
 * Input Parameters:
 *   src - Identities the clock source
 *
 * Returned Values:
 *   The requested clock source frequency.  Zero is returned on any error.
 *
 ****************************************************************************/

static uint32_t s32k1xx_get_srcfreq(enum scg_system_clock_src_e src)
{
  uint32_t srcfreq = 0;

  switch (src)
    {
      case SCG_SYSTEM_CLOCK_SRC_SYS_OSC:
        srcfreq = s32k1xx_get_soscfreq();
        break;

      case SCG_SYSTEM_CLOCK_SRC_SIRC:
        srcfreq = s32k1xx_get_sircfreq();
        break;

      case SCG_SYSTEM_CLOCK_SRC_FIRC:
        srcfreq = s32k1xx_get_fircfreq();
        break;

#ifdef CONFIG_S32K1XX_HAVE_SPLL
      case SCG_SYSTEM_CLOCK_SRC_SYS_PLL:
        srcfreq = s32k1xx_get_spllfreq();
        break;
#endif

      default:
        break;
    }

  return srcfreq;
}

/****************************************************************************
 * Name: s32k1xx_set_sysclk_cfg
 *
 * Description:
 *   This function sets the system configuration for the specified mode.
 *
 * Input Parameters:
 *   mode   -
 *   config -
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int
s32k1xx_set_sysclk_cfg(enum scg_system_clock_mode_e mode,
                             const struct scg_system_clock_config_s *config)
{
  uint32_t srcfreq      = 0;
  uint32_t sysfreq_mul  = (uint32_t)config->divcore;
  uint32_t busfreq_mul  = (uint32_t)config->divcore *
                          (uint32_t)config->divbus;
  uint32_t slowfreq_mul = (uint32_t)config->divcore *
                          (uint32_t)config->divslow;
  uint32_t regval;
  int ret               = OK;

  DEBUGASSERT(mode != SCG_SYSTEM_CLOCK_MODE_CURRENT);

  srcfreq = s32k1xx_get_srcfreq(config->src) >> 4;

  switch (mode)
    {
      case SCG_SYSTEM_CLOCK_MODE_RUN:    /* Run mode */

        /* Verify the frequencies of sys, bus and slow clocks. */

          if ((srcfreq > (sysfreq_mul *
                (g_run_maxsysclks[(uint32_t)config->src][CORE_CLK_INDEX] >>
                 4))) ||
              (srcfreq > (busfreq_mul *
                (g_run_maxsysclks[(uint32_t)config->src][BUS_CLK_INDEX] >>
                 4))) ||
              (srcfreq > (slowfreq_mul *
                (g_run_maxsysclks[(uint32_t)config->src][SLOW_CLK_INDEX] >>
                 4))))
            {
              /* Configuration for the next system clock source is not
               * valid.
               */

              ret = -EINVAL;
            }
          else
            {
              regval = (((uint32_t)config->src << SCG_RCCR_SCS_SHIFT) |
                        SCG_RCCR_DIVCORE(config->divcore) |
                        SCG_RCCR_DIVBUS(config->divbus)  |
                        SCG_RCCR_DIVSLOW(config->divslow));
              putreg32(regval, S32K1XX_SCG_RCCR);
            }
          break;

        case SCG_SYSTEM_CLOCK_MODE_VLPR:    /* Very Low Power Run mode */
          DEBUGASSERT(SCG_SYSTEM_CLOCK_SRC_SIRC    == config->src);

          /* Verify the frequencies of sys, bus and slow clocks. */

          if ((srcfreq > (sysfreq_mul *
              (g_vlpr_maxsysclks[(uint32_t)config->src][CORE_CLK_INDEX] >>
                4))) ||
              (srcfreq > (busfreq_mul *
              (g_vlpr_maxsysclks[(uint32_t)config->src][BUS_CLK_INDEX] >>
                4))) ||
              (srcfreq > (slowfreq_mul *
              (g_vlpr_maxsysclks[(uint32_t)config->src][SLOW_CLK_INDEX] >>
                4))))
            {
              /* Configuration for the next system clock source is not
               * valid.
               */

              ret = -EINVAL;
            }
          else
            {
              regval = (((uint32_t)config->src << SCG_VCCR_SCS_SHIFT) |
                        SCG_VCCR_DIVCORE(config->divcore) |
                        SCG_VCCR_DIVBUS(config->divbus)  |
                        SCG_VCCR_DIVSLOW(config->divslow));
              putreg32(regval, S32K1XX_SCG_VCCR);
            }
          break;

#ifdef CONFIG_S32K1XX_HAVE_HSRUN
        case SCG_SYSTEM_CLOCK_MODE_HSRUN:     /* High Speed Run mode. */
          DEBUGASSERT(SCG_SYSTEM_CLOCK_SRC_FIRC == config->src ||
                      SCG_SYSTEM_CLOCK_SRC_SYS_PLL == config->src);

          /* Verify the frequencies of sys, bus and slow clocks. */

          if ((srcfreq > (sysfreq_mul *
              (g_hsrun_maxsysclks[(uint32_t)config->src][CORE_CLK_INDEX] >>
                4))) ||
              (srcfreq > (busfreq_mul *
              (g_hsrun_maxsysclks[(uint32_t)config->src][BUS_CLK_INDEX] >>
                4))) ||
              (srcfreq > (slowfreq_mul *
              (g_hsrun_maxsysclks[(uint32_t)config->src][SLOW_CLK_INDEX] >>
                4))))
            {
              /* Configuration for the next system clock source is not
               * valid.
               */

              ret = -EINVAL;
            }
          else
            {
              regval = (((uint32_t)config->src << SCG_HCCR_SCS_SHIFT) |
                        SCG_HCCR_DIVCORE(config->divcore) |
                        SCG_HCCR_DIVBUS(config->divbus)  |
                        SCG_HCCR_DIVSLOW(config->divslow));
              putreg32(regval, S32K1XX_SCG_HCCR);
            }
          break;
#endif
        default:

          /* Invalid mode */

          DEBUGPANIC();
          break;
    }

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_transition_systemclock
 *
 * Description:
 *   Transition to a new system clock.
 *
 * Input Parameters:
 *   cfg - Describes the new system clock configuration
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int
s32k1xx_transition_systemclock(const struct scg_system_clock_config_s *cfg)
{
  enum scg_system_clock_mode_e run_mode;
  uint32_t timeout;
  int ret = OK;

  DEBUGASSERT(cfg != NULL && cfg->src != SCG_SYSTEM_CLOCK_SRC_NONE);

  /* Get and convert Run mode from SMC to SCG defines */

  run_mode = s32k1xx_get_runmode();

  /* Check the current mode */

  DEBUGASSERT(run_mode != SCG_SYSTEM_CLOCK_MODE_NONE);

  /* Update run mode configuration */

  ret = s32k1xx_set_sysclk_cfg(run_mode, cfg);
  if (ret == OK)
    {
      /* Wait for system clock to transition.
       *
       * e10777: The SCG_RCCR[SCS] and SCG_HCCR[SCS] may have a corrupted
       * status during the interval when the system clock is switching.
       * Workaround: The SCS field should be read twice by the software to
       * ensure the system clock switch has completed.
       */

#if 1 /* Errata E10777 */
      timeout = 10;
#else
      timeout = 1;
#endif

      do
        {
          timeout--;
        }
      while (s32k1xx_get_scgclk_source() != cfg->src && timeout > 0);

      if (timeout == 0)
        {
          ret = -ETIMEDOUT;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_firc_config
 *
 * Description:
 *   Configures FIRC module based on provided configuration.
 *
 * Input Parameters:
 *   firccfg - Describes the desired FORC configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int s32k1xx_firc_config(bool enable,
                               const struct scg_firc_config_s *firccfg)
{
  uint32_t regval;
  int32_t timeout;
  int ret = OK;

  DEBUGASSERT(firccfg != NULL);

  /* If clock is used by system, return error. */

  regval = getreg32(S32K1XX_SCG_FIRCCSR);
  if ((regval & SCG_FIRCCSR_FIRCSEL) != 0)
    {
      ret = -EBUSY;
    }

  /* Disable the FIRC */

  else
    {
      /* Clear LK bit field */

      regval &= ~SCG_FIRCCSR_LK;
      putreg32(regval, S32K1XX_SCG_FIRCCSR);

      /* Disable monitor, disable clock and clear error. */

      putreg32(SCG_FIRCCSR_FIRCERR, S32K1XX_SCG_FIRCCSR);
    }

  /* Configure FIRC. */

  if (enable && (ret == OK))
    {
      /* Now start to set up FIRC clock. */

      /* Step 1. Setup dividers. */

      regval = SCG_FIRCDIV_FIRCDIV1(firccfg->div1) |
               SCG_FIRCDIV_FIRCDIV2(firccfg->div2);
      putreg32(regval, S32K1XX_SCG_FIRCDIV);

      /* Step 2. Set FIRC configuration. */

      if (firccfg->range == 0)
        {
          regval = 0;
        }
      else
        {
          regval = SCG_FIRCCFG_48MHZ; /* REVISIT: Also zero */
        }

      putreg32(regval, S32K1XX_SCG_FIRCCFG);

      /* Step 3. Enable clock, config regulator and locking feature. */

      regval = SCG_FIRCCSR_FIRCEN;

      if (!firccfg->regulator)
        {
          regval |= SCG_FIRCCSR_FIRCREGOFF;
        }

      if (firccfg->locked)
        {
          regval |= SCG_FIRCCSR_LK;
        }

      putreg32(regval, S32K1XX_SCG_FIRCCSR);

      /* Wait for FIRC to initialize */

      for (timeout = FIRC_STABILIZATION_TIMEOUT;
           s32k1xx_get_fircfreq() == 0 && timeout > 0;
           timeout--)
        {
        }

      if (timeout <= 0)
        {
          ret = -ETIMEDOUT;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_firc_clocksource
 *
 * Description:
 *   Configure to the FIRC clock source.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int s32k1xx_firc_clocksource(void)
{
  struct scg_system_clock_config_s firccfg;
  int ret = OK;

  /* If the current system clock source is not FIRC:
   * 1. Enable FIRC (if it's not enabled)
   * 2. Switch to FIRC.
 */

  if (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_FIRC)
    {
      /* If FIRC is not on, then FIRC is configured with the default
       * configuration
       */

      if (s32k1xx_get_fircfreq() == 0)
        {
          ret = s32k1xx_firc_config(true, NULL);
        }

      /* FIRC is enabled, transition the system clock source to FIRC. */

      if (ret == OK)
        {
          firccfg.src     = SCG_SYSTEM_CLOCK_SRC_FIRC;
          firccfg.divcore = g_tmp_sysclk[TMP_FIRC_CLK][TMP_SYS_DIV];
          firccfg.divbus  = g_tmp_sysclk[TMP_FIRC_CLK][TMP_BUS_DIV];
          firccfg.divslow = g_tmp_sysclk[TMP_FIRC_CLK][TMP_SLOW_DIV];
          ret             = s32k1xx_transition_systemclock(&firccfg);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_sirc_config
 *
 * Description:
 *   Configures SIRC module based on provided configuration.
 *
 * Input Parameters:
 *   sirccfg - Describes the desired SIRC configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int s32k1xx_sirc_config(bool enable,
                               const struct scg_sirc_config_s *sirccfg)
{
  uint32_t regval;
  uint32_t timeout;
  int ret = OK;

  DEBUGASSERT(sirccfg != NULL);

  /* If clock is used by system, return error. */

  regval = getreg32(S32K1XX_SCG_SIRCCSR);
  if ((regval & SCG_SIRCCSR_SIRCSEL) != 0)
    {
      ret = -EBUSY;
    }

  /* Disable SIRC */

  else
    {
      /* Clear LK bit field */

      regval &= ~SCG_SIRCCSR_LK;
      putreg32(regval, S32K1XX_SCG_SIRCCSR);

      /* Disable monitor, disable clock and clear error. */

      putreg32(0, S32K1XX_SCG_SIRCCSR);
    }

  /* Configure SIRC. */

  if (enable  && (ret == OK))
    {
      /* Now start to set up SIRC clock. */

      /* Step 1. Setup dividers. */

      regval = SCG_SIRCDIV_SIRCDIV1(sirccfg->div1) |
               SCG_SIRCDIV_SIRCDIV2(sirccfg->div2);
      putreg32(regval, S32K1XX_SCG_SIRCDIV);

      /* Step 2. Set SIRC configuration: frequency range. */

      if (sirccfg->range == SCG_SIRC_RANGE_HIGH)
        {
          regval = SCG_SIRCCFG_HIGHRANGE;
        }
      else
        {
          regval = SCG_SIRCCFG_LOWRANGE;
        }

      putreg32(regval, S32K1XX_SCG_SIRCCFG);

      /* Step 3. Set SIRC control: enable clock, configure source in STOP
       * and VLP modes, configure lock feature.
       */

      regval = SCG_SIRCCSR_SIRCEN;

      if (sirccfg->stopmode)
        {
          regval |= SCG_SIRCCSR_SIRCSTEN;
        }

      if (sirccfg->lowpower)
        {
          regval |= SCG_SIRCCSR_SIRCLPEN;
        }

      if (sirccfg->locked)
        {
          regval |= SCG_SIRCCSR_LK;
        }

      putreg32(regval, S32K1XX_SCG_SIRCCSR);

      /* Wait for SIRC to initialize */

      for (timeout = SIRC_STABILIZATION_TIMEOUT;
           s32k1xx_get_sircfreq() == 0 && timeout > 0;
           timeout--)
        {
        }

      if (timeout <= 0)
        {
          ret = -ETIMEDOUT;
        }
    }

  return ret;
}

#if defined(CONFIG_VLPR_STANDBY) || defined(CONFIG_VLPR_SLEEP)

/****************************************************************************
 * Name: s32k1xx_sirc_clocksource
 *
 * Description:
 *   Configure to the SIRC clock source.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int s32k1xx_sirc_clocksource(void)
{
  struct scg_system_clock_config_s sirccfg;
  int ret = OK;

  /* If the current system clock source is not SIRC:
   * 1. Enable SIRC (if it's not enabled)
   * 2. Switch to SIRC.
   */

  if (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SIRC)
    {
      /* If SIRC is not on, then SIRC is configured with the default
       * configuration
       */

      if (s32k1xx_get_sircfreq() == 0)
        {
          ret = s32k1xx_sirc_config(true, NULL);
        }

      /* SIRC is enabled, transition the system clock source to SIRC. */

      if (ret == OK)
        {
          sirccfg.src     = SCG_SYSTEM_CLOCK_SRC_SIRC;
          sirccfg.divcore = g_tmp_sysclk[TMP_SIRC_CLK][TMP_SYS_DIV];
          sirccfg.divbus  = g_tmp_sysclk[TMP_SIRC_CLK][TMP_BUS_DIV];
          sirccfg.divslow = g_tmp_sysclk[TMP_SIRC_CLK][TMP_SLOW_DIV];
          ret             = s32k1xx_transition_systemclock(&sirccfg);
        }
    }

  return ret;
}

#endif

/****************************************************************************
 * Name: s32k1xx_sosc_config
 *
 * Description:
 *   CConfigures SOSC module based on provided configuration.
 *
 * Input Parameters:
 *   sosccfg - Describes the desired SOSC configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int s32k1xx_sosc_config(bool enable,
                               const struct scg_sosc_config_s *sosccfg)
{
  uint32_t regval;
  uint32_t timeout;
  int ret = OK;

  DEBUGASSERT(sosccfg != NULL);

  /* If clock is used by system, return error. */

  regval = getreg32(S32K1XX_SCG_SOSCCSR);
  if ((regval & SCG_SOSCCSR_SOSCSEL) != 0)
    {
      ret = -EBUSY;
    }

  /* Disable SOSC */

  else
    {
      /* Clear LK bit field */

      regval &= ~SCG_SOSCCSR_LK;
      putreg32(regval, S32K1XX_SCG_SOSCCSR);

      /* Disable monitor, disable clock and clear error. */

      putreg32(SCG_SOSCCSR_SOSCERR, S32K1XX_SCG_SOSCCSR);
    }

  /* Configure the SOSC */

  if (enable && (ret == OK))
    {
      /* Now start to set up OSC clock */

      /* Step 1. Setup dividers. */

      regval = SCG_SOSCDIV_SOSCDIV1(sosccfg->div1) |
               SCG_SOSCDIV_SOSCDIV2(sosccfg->div2);
      putreg32(regval, S32K1XX_SCG_SOSCDIV);

      /* Step 2. Set OSC configuration. */

      regval = SCG_SOSCCFG_RANGE(sosccfg->range);

      if (sosccfg->gain == SCG_SOSC_GAIN_HIGH)
        {
          regval |= SCG_SOSCCFG_HGO;
        }

      if (sosccfg->extref == SCG_SOSC_REF_OSC)
        {
          regval |= SCG_SOSCCFG_EREFS;
        }

      putreg32(regval, S32K1XX_SCG_SOSCCFG);

      /* Step 3. Enable clock, configure monitor, lock register. */

      regval = SCG_SOSCCSR_SOSCEN;

      if (sosccfg->locked)
        {
          regval |= SCG_SOSCCSR_LK;
        }

      switch (sosccfg->mode)
        {
          case SCG_SOSC_MONITOR_DISABLE:
            {
              putreg32(regval, S32K1XX_SCG_SOSCCSR);
            }
            break;

          case SCG_SOSC_MONITOR_INT:
            {
              regval |= SCG_SOSCCSR_SOSCCM;
              putreg32(regval, S32K1XX_SCG_SOSCCSR);
            }
            break;

          case SCG_SOSC_MONITOR_RESET:
            {
              regval |= SCG_SOSCCSR_SOSCCM | SCG_SOSCCSR_SOSCCMRE;
              putreg32(regval, S32K1XX_SCG_SOSCCSR);
            }
            break;

          default:

            /* Invalid monitor mode */

            DEBUGPANIC();
            break;
        }

      /* Wait for System OSC to initialize */

      for (timeout = SOSC_STABILIZATION_TIMEOUT;
           s32k1xx_get_soscfreq() == 0 && timeout > 0;
           timeout--)
        {
        }

      if (timeout <= 0)
        {
          ret = -ETIMEDOUT;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_spll_config
 *
 * Description:
 *    Configures SPLL module based on provided configuration.
 *
 * Input Parameters:
 *   spllccfg - Describes the desired SPLL configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_HAVE_SPLL
static int s32k1xx_spll_config(bool enable,
                               const struct scg_spll_config_s *spllcfg)
{
  uint32_t regval;
  uint32_t srcfreq;
  uint32_t timeout;
  int ret = OK;

  DEBUGASSERT(spllcfg != NULL);

  /* If clock is used by system, return error. */

  regval = getreg32(S32K1XX_SCG_SPLLCSR);
  if ((regval & SCG_SPLLCSR_SPLLSEL) != 0)
    {
      ret = -EBUSY;
    }

  /* Disable the SPLL. */

  else
    {
      /* Clear LK bit field */

      regval &= ~SCG_SPLLCSR_LK;
      putreg32(regval, S32K1XX_SCG_SPLLCSR);

      /* Disable monitor, disable clock and clear error. */

      putreg32(SCG_SPLLCSR_SPLLERR, S32K1XX_SCG_SPLLCSR);
    }

  /* Configure SPLL */

  if (enable && (ret == OK))
    {
      /* Get clock source frequency. */

      srcfreq = s32k1xx_get_soscfreq();
      DEBUGASSERT(srcfreq != 0);

      /* Pre-divider checking. */

      srcfreq /= spllcfg->prediv;
      DEBUGASSERT(srcfreq >= SCG_SPLL_REF_MIN &&
                  srcfreq <= SCG_SPLL_REF_MAX);

      /* Now start to set up PLL clock. */

      regval = SCG_SPLLDIV_SPLLDIV1(spllcfg->div1) |
               SCG_SPLLDIV_SPLLDIV2(spllcfg->div2);
      putreg32(regval, S32K1XX_SCG_SPLLDIV);

      /* Step 2. Set PLL configuration. */

      regval = SCG_SPLLCFG_PREDIV(spllcfg->prediv)  |
               SCG_SPLLCFG_MULT(spllcfg->mult);
      putreg32(regval, S32K1XX_SCG_SPLLCFG);

      /* Step 3.
       * Enable clock, configure monitor, lock register.
       */

      regval = SCG_SPLLCSR_SPLLEN;

      if (spllcfg->locked)
        {
          regval |= SCG_SPLLCSR_LK;
        }

      switch (spllcfg->mode)
        {
          case SCG_SPLL_MONITOR_DISABLE:
            {
              putreg32(regval, S32K1XX_SCG_SPLLCSR);
            }
            break;

          case SCG_SPLL_MONITOR_INT:
            {
              regval |= SCG_SPLLCSR_SPLLCM;
              putreg32(regval, S32K1XX_SCG_SPLLCSR);
            }
            break;

          case SCG_SPLL_MONITOR_RESET:
            {
              regval |= (SCG_SPLLCSR_SPLLCM | SCG_SPLLCSR_SPLLCMRE);
              putreg32(regval, S32K1XX_SCG_SPLLCSR);
            }
            break;

          default:

            /* Invalid monitor mode */

            DEBUGPANIC();
            break;
        }

      /* Wait for System PLL to initialize */

      for (timeout = SPLL_STABILIZATION_TIMEOUT;
           s32k1xx_get_spllfreq() == 0 && timeout > 0;
           timeout--)
        {
        }

      if (timeout <= 0)
        {
          ret = -ETIMEDOUT;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: s32k1xx_configure_scgmodules
 *
 * Description:
 *   Configures all modules from SCG (SIRC, FIRC, SOSC and SPLL)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int s32k1xx_configure_scgmodules(const struct scg_config_s *scgcfg)
{
  struct scg_system_clock_config_s sysclkcfg;
  const struct scg_system_clock_config_s *next;
  int ret = OK;

  /* Configure all clock sources that are different from the current system
   * clock source FIRC (SIRC, SOSC, SPLL).
   */

  ret = s32k1xx_sirc_config(scgcfg->sirc.initialize, &scgcfg->sirc);
  if (ret == OK)
    {
      ret = s32k1xx_sosc_config(scgcfg->sosc.initialize, &scgcfg->sosc);
#ifdef CONFIG_S32K1XX_HAVE_SPLL
      if (ret == OK)
        {
          ret = s32k1xx_spll_config(scgcfg->spll.initialize, &scgcfg->spll);
        }
#endif
    }

  /* Get the next system clock source */

  switch (s32k1xx_get_runmode())
    {
      case SCG_SYSTEM_CLOCK_MODE_RUN:
        {
          next = &scgcfg->clockmode.rccr;
        }
        break;

      case SCG_SYSTEM_CLOCK_MODE_VLPR:
        {
          next = &scgcfg->clockmode.vccr;
        }
        break;

#ifdef CONFIG_S32K1XX_HAVE_HSRUN
      case SCG_SYSTEM_CLOCK_MODE_HSRUN:
        {
          next = &scgcfg->clockmode.hccr;
        }
        break;
#endif

      default:
        DEBUGPANIC();
        next = NULL;
        break;
    }

  if (ret == OK)
    {
      /* The current system clock source is FIRC.  Verify whether the next
       * system clock source is FIRC.
       */

      if (next->src == SCG_SYSTEM_CLOCK_SRC_FIRC)
        {
          /* If they are the same, search for a temporary system clock source
          * (use one of the following sources: SPLL, SOSC, SIRC).  Assume
          * that a temporary clock is not found ret = -ENOENT.
          */

          ret = -ENOENT;

#ifdef CONFIG_S32K1XX_HAVE_SPLL
          /* SPLL is enabled */

          if (scgcfg->spll.initialize && (ret == -ENOENT))
            {
              sysclkcfg.src     = SCG_SYSTEM_CLOCK_SRC_SYS_PLL;
              sysclkcfg.divcore = g_tmp_sysclk[TMP_SPLL_CLK][TMP_SYS_DIV];
              sysclkcfg.divbus  = g_tmp_sysclk[TMP_SPLL_CLK][TMP_BUS_DIV];
              sysclkcfg.divslow = g_tmp_sysclk[TMP_SPLL_CLK][TMP_SLOW_DIV];
              ret               = s32k1xx_transition_systemclock(&sysclkcfg);
            }
#endif

          /* SOSC is enabled and SPLL configuration for system clock source
           * is not valid
           */

          if (scgcfg->sosc.initialize && (ret == -ENOENT))
            {
              sysclkcfg.src     = SCG_SYSTEM_CLOCK_SRC_SYS_OSC;
              sysclkcfg.divcore = g_tmp_sysclk[TMP_SOSC_CLK][TMP_SYS_DIV];
              sysclkcfg.divbus  = g_tmp_sysclk[TMP_SOSC_CLK][TMP_BUS_DIV];
              sysclkcfg.divslow = g_tmp_sysclk[TMP_SOSC_CLK][TMP_SLOW_DIV];
              ret               = s32k1xx_transition_systemclock(&sysclkcfg);
            }

            /* SIRC is enabled and SOSC configuration for system clock
             * source is not valid
             */

          if (scgcfg->sirc.initialize && (ret == -ENOENT))
            {
              sysclkcfg.src     = SCG_SYSTEM_CLOCK_SRC_SIRC;
              sysclkcfg.divcore = g_tmp_sysclk[TMP_SIRC_CLK][TMP_SYS_DIV];
              sysclkcfg.divbus  = g_tmp_sysclk[TMP_SIRC_CLK][TMP_BUS_DIV];
              sysclkcfg.divslow = g_tmp_sysclk[TMP_SIRC_CLK][TMP_SLOW_DIV];
              ret               = s32k1xx_transition_systemclock(&sysclkcfg);
            }

          /* Transitioned to a temporary system clock source. */

          if (ret == OK)
            {
              /* Configure the remaining clock source (FIRC). */

              ret = s32k1xx_firc_config(scgcfg->firc.initialize,
                                        &scgcfg->firc);
              if (ret == OK)
                {
                  /* Transition to the next system clock source. */

                  sysclkcfg.src     = next->src;
                  sysclkcfg.divcore = next->divcore;
                  sysclkcfg.divbus  = next->divbus;
                  sysclkcfg.divslow = next->divslow;
                  ret = s32k1xx_transition_systemclock(&sysclkcfg);
                }
            }
        }
      else
        {
          /* Transition to the next system clock source. */

          sysclkcfg.src     = next->src;
          sysclkcfg.divcore = next->divcore;
          sysclkcfg.divbus  = next->divbus;
          sysclkcfg.divslow = next->divslow;
          ret = s32k1xx_transition_systemclock(&sysclkcfg);

          if (ret == OK)
            {
              /* Configure the remaining clock source (FIRC) */

              ret = s32k1xx_firc_config(scgcfg->firc.initialize,
                                        &scgcfg->firc);
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_scg_config
 *
 * Description:
 *   Configure SCG clocking.
 *
 * Input Parameters:
 *   scgcfg - Describes the new SCG clock configuration
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int s32k1xx_scg_config(const struct scg_config_s *scgcfg)
{
  uint32_t regval;
  int ret = OK;

  DEBUGASSERT(scgcfg != NULL);

  /* Configure a temporary system clock source: FIRC if enabled */

  ret = s32k1xx_firc_clocksource();

  if (ret == OK)
    {
      /* Configure clock sources from SCG */

      ret = s32k1xx_configure_scgmodules(scgcfg);
    }

  if (ret == OK)
    {
      /* Configure RTC. */

#if 0 /* Not used */
      if (scgcfg->rtc.initialize)
        {
          /* RTC Clock settings. */

          g_rtc_clkin = scgcfg->rtc.clkin;
        }
#endif

      /* Configure SCG ClockOut. */

      if (scgcfg->clockout.initialize)
        {
          /* ClockOut settings. */

          regval  = getreg32(S32K1XX_SCG_CLKOUTCNFG);
          regval &= ~SCG_CLKOUTCNFG_CLKOUTSEL_MASK;
          regval |= SCG_CLKOUTCNFG_CLKOUTSEL(scgcfg->clockout.source);
          putreg32(regval, S32K1XX_SCG_CLKOUTCNFG);
        }

      /* Configure SCG clock modes. */

      if (scgcfg->clockmode.initialize)
        {
          /* Configure SCG clock modes */

          ret = s32k1xx_set_sysclk_cfg(SCG_SYSTEM_CLOCK_MODE_RUN,
                                       &scgcfg->clockmode.rccr);
          if (ret == OK)
            {
              ret = s32k1xx_set_sysclk_cfg(SCG_SYSTEM_CLOCK_MODE_VLPR,
                                           &scgcfg->clockmode.vccr);
            }

#ifdef CONFIG_S32K1XX_HAVE_HSRUN
          if (ret == OK)
            {
              ret = s32k1xx_set_sysclk_cfg(SCG_SYSTEM_CLOCK_MODE_HSRUN,
                                           &scgcfg->clockmode.hccr);
            }
#endif
        }
    }

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_sim_config
 *
 * Description:
 *   Configure PCC clocking.
 *
 * Input Parameters:
 *   simcfg - Describes the new SIM clock configuration
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void s32k1xx_sim_config(const struct sim_clock_config_s *simcfg)
{
  uint32_t regval;
#ifdef CONFIG_S32K1XX_HAVE_QSPI
  int i;
#endif

  DEBUGASSERT(simcfg != NULL);

  /* ClockOut settings. */

  if (simcfg->clockout.initialize)
    {
      regval  = getreg32(S32K1XX_SIM_CHIPCTL);
      regval &= ~(SIM_CHIPCTL_CLKOUTEN | SIM_CHIPCTL_CLKOUTDIV_MASK |
                  SIM_CHIPCTL_CLKOUTSEL_MASK);

      if (simcfg->clockout.enable)
        {
          regval |= SIM_CHIPCTL_CLKOUTEN;
        }

      regval |= SIM_CHIPCTL_CLKOUTSEL(simcfg->clockout.source);
      regval |= SIM_CHIPCTL_CLKOUTDIV(simcfg->clockout.divider);
      putreg32(regval, S32K1XX_SIM_CHIPCTL);
    }

  /* Low Power Clock settings from SIM. */

  if (simcfg->lpoclk.initialize)
    {
      regval  = getreg32(S32K1XX_SIM_LPOCLKS);
      regval &= ~(SIM_LPOCLKS_LPO1KCLKEN | SIM_LPOCLKS_LPO32KCLKEN |
                  SIM_LPOCLKS_LPOCLKSEL_MASK | SIM_LPOCLKS_RTCCLKSEL_MASK);

      if (simcfg->lpoclk.lpo1k)
        {
          regval |= SIM_LPOCLKS_LPO1KCLKEN;
        }

      if (simcfg->lpoclk.lpo32k)
        {
          regval |= SIM_LPOCLKS_LPO32KCLKEN;
        }

      regval |= SIM_LPOCLKS_LPOCLKSEL(simcfg->lpoclk.lpo_source);
      regval |= SIM_LPOCLKS_RTCCLKSEL(simcfg->lpoclk.rtc_source);

      putreg32(regval, S32K1XX_SIM_LPOCLKS);
    }

  /* Platform Gate Clock settings. */

  if (simcfg->platgate.initialize)
    {
      regval  = getreg32(S32K1XX_SIM_PLATCGC);
      regval &= ~(SIM_PLATCGC_CGCMSCM | SIM_PLATCGC_CGCMPU |
                  SIM_PLATCGC_CGCDMA | SIM_PLATCGC_CGCERM |
                  SIM_PLATCGC_CGCEIM);

      if (simcfg->platgate.mscm)
        {
          regval |= SIM_PLATCGC_CGCMSCM;
        }

      if (simcfg->platgate.mpu)
        {
          regval |= SIM_PLATCGC_CGCMPU;
        }

      if (simcfg->platgate.dma)
        {
          regval |= SIM_PLATCGC_CGCDMA;
        }

      if (simcfg->platgate.erm)
        {
          regval |= SIM_PLATCGC_CGCERM;
        }

      if (simcfg->platgate.eim)
        {
          regval |= SIM_PLATCGC_CGCEIM;
        }

      putreg32(regval, S32K1XX_SIM_PLATCGC);

#ifdef CONFIG_S32K1XX_HAVE_QSPI
      regval  = getreg32(S32K1XX_SIM_MISCTRL0);
      regval &= ~SIM_MISCTRL0_QSPI_CLK_SEL;

      if (simcfg->qspirefclk.refclk)
        {
          regval |= SIM_MISCTRL0_QSPI_CLK_SEL;
        }

      putreg32(regval, S32K1XX_SIM_MISCTRL0);
#endif
    }

#if 0 /* REVISIT:  Not currently used */
  /* TCLK Clock settings. */

  if (simcfg->tclk.initialize)
    {
      for (i = 0; i < NUMBER_OF_TCLK_INPUTS; i++)
        {
          g_tclkfreq[i] = simcfg->tclk.tclkfreq[i];
        }
    }
#endif

  /* Debug trace Clock settings. */

  if (simcfg->traceclk.initialize)
    {
      /* Disable divider. */

      putreg32(0, S32K1XX_SIM_CLKDIV4);

      /* Configure trace source. */

      regval  = getreg32(S32K1XX_SIM_CHIPCTL);
      regval &= ~SIM_CHIPCTL_TRACECLK_SEL;

      if (simcfg->traceclk.source)
        {
          regval |= SIM_CHIPCTL_TRACECLK_SEL;
        }

      putreg32(regval, S32K1XX_SIM_CHIPCTL);

      if (simcfg->traceclk.enable)
        {
          regval = SIM_CLKDIV4_TRACEDIVEN |
                   SIM_CLKDIV4_TRACEDIV(simcfg->traceclk.divider);

          if (simcfg->traceclk.fraction)
            {
              regval |= SIM_CLKDIV4_TRACEFRAC;
            }

          putreg32(regval, S32K1XX_SIM_CLKDIV4);
        }
    }
}

/****************************************************************************
 * Name: s32k1xx_pmc_config
 *
 * Description:
 *   Configure PMC clocking.
 *
 * Input Parameters:
 *   pmccfg - Describes the new PMC clock configuration
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void s32k1xx_pmc_config(const struct pmc_config_s *pmccfg)
{
  uint8_t regval;

  DEBUGASSERT(pmccfg != NULL);

  /* Low Power Clock settings from PMC. */

  if (pmccfg->lpoclk.initialize)
    {
      /* Enable/disable the low power oscillator. */

      regval = getreg8(S32K1XX_PMC_REGSC);

      if (pmccfg->lpoclk.enable)
        {
          regval &= ~PMC_REGSC_LPODIS;
        }
      else
        {
          regval |= PMC_REGSC_LPODIS;
        }

      /* Enable Biasing (needed for VLPR mode, no effect in RUN mode) */

      regval |= PMC_REGSC_BIASEN;

      putreg8(regval, S32K1XX_PMC_REGSC);

      /* Write trimming value. */

      putreg8(pmccfg->lpoclk.trim & PMC_LPOTRIM_MASK, S32K1XX_PMC_LPOTRIM);
    }
}

/****************************************************************************
 * Name: s32k1xx_allow_vlprmode
 *
 * Description:
 *   allow the very low power run mode.
 *
 * Input Parameters:
 *   allow - true if allowed, false otherwise.
 *
 * Returned Value:
 *   none.
 *
 ****************************************************************************/

void s32k1xx_allow_vlprmode(bool allow)
{
  uint32_t regval;

  /* get the SMC_PMPROT register */

  regval  =  getreg32(S32K1XX_SMC_PMPROT);

  /* mask the AVLP bit */

  regval &= ~SMC_PMPROT_AVLP;

  /* set the new bit */

  regval |= (allow << SMC_PMPROT_AVLP_SHIFT);

  /* set the registervalue */

  putreg32(regval, S32K1XX_SMC_PMPROT);
}

/****************************************************************************
 * Name: up_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is  called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   None - The driver already agreed to transition to the low power
 *   consumption state when when it returned OK to the prepare() call.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  int return_value;

  /* check if the transition is from the IDLE domain to the NORMAL domain */

  if (pm_querystate(PM_IDLE_DOMAIN) == PM_IDLE &&
    pmstate == PM_NORMAL)
    {
      /* return */

      return;
    }

  /* check what the new power state is */

  switch (pmstate)
    {
      /* if it needs to be set to RUN mode */

      case(PM_NORMAL):
        {
          /* Logic for PM_NORMAL goes here */

          /* change the microcontroller to RUN mode  */

          /* and wait until in RUN mode */

          return_value = (int)s32k1xx_set_runmode(SCG_SYSTEM_CLOCK_MODE_RUN);

          /* check for debug assertion */

          DEBUGASSERT(return_value != (int)SCG_SYSTEM_CLOCK_MODE_NONE);

          /* enable all clock sources again if needed  */

          /* these could be the FIRC, PPL, and SOSC */

          /* check if the FIRC was enabled and
           * it is not the system clock source
           */

          if (g_initial_clkconfig.scg.firc.initialize &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SIRC))
          {
            /* enable FIRC */

            return_value = s32k1xx_firc_config(true,
              &g_initial_clkconfig.scg.firc);
            DEBUGASSERT(!return_value);
          }

          /* check if the FIRC needs to be disabled and if it is enabled */

          else if ((!(g_initial_clkconfig.scg.firc.initialize)) &&
            (s32k1xx_get_srcfreq(SCG_SYSTEM_CLOCK_SRC_FIRC)))
          {
            /* disable FIRC */

            return_value = s32k1xx_firc_config(false,
              &g_initial_clkconfig.scg.firc);
            DEBUGASSERT(!return_value);
          }

          /* check if the SOSC was enabled and
           * it is not the system clock source
           */

          if (g_initial_clkconfig.scg.sosc.initialize &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SYS_OSC) &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SYS_PLL))
          {
            /* enable SOSC */

            return_value =
              s32k1xx_sosc_config(true, &g_initial_clkconfig.scg.sosc);
            DEBUGASSERT(!return_value);
          }

          /* check if the SOSC needs to be disabled and if it is enabled */

          else if ((!(g_initial_clkconfig.scg.sosc.initialize)) &&
            (s32k1xx_get_srcfreq(SCG_SYSTEM_CLOCK_SRC_SYS_OSC)))
          {
            /* disable SOSC */

            return_value =
              s32k1xx_sosc_config(false, &g_initial_clkconfig.scg.sosc);
            DEBUGASSERT(!return_value);
          }

          /* check if the SPLL was enabled and
           * it is not the system clock source
           */

          if (g_initial_clkconfig.scg.spll.initialize &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SYS_OSC) &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SYS_PLL))
          {
            /* enable SPLL */

            return_value = s32k1xx_spll_config(true,
              &g_initial_clkconfig.scg.spll);
            DEBUGASSERT(!return_value);
          }

          /* check if the SPLL needs to be disabled and if it is enabled */

          else if ((!(g_initial_clkconfig.scg.spll.initialize)) &&
            (s32k1xx_get_srcfreq(SCG_SYSTEM_CLOCK_SRC_SYS_PLL)))
          {
            /* disable SPLL */

            return_value = s32k1xx_spll_config(false,
              &g_initial_clkconfig.scg.spll);
            DEBUGASSERT(!return_value);
          }

          /* check if the RCCR clock source is enabled */

          if (s32k1xx_get_srcfreq(g_initial_clkconfig.scg.clockmode.rccr.src)
            != 0)
          {
            /* change the system clock back to the configured clock */

            /* and wait until clock changed */

            if (s32k1xx_transition_systemclock(
              &g_initial_clkconfig.scg.clockmode.rccr))
            {
              /* error */

              DEBUGPANIC();
            }
          }

          /* if it is 0 */

          else
          {
            /* error */

            DEBUGPANIC();
          }

          /* calculate the new clock ticks */

          up_timer_initialize();
        }
        break;

      case(PM_IDLE):
        {
          /* Logic for PM_IDLE goes here */
        }
        break;

      /* if it needs to be set to VLPR mode */

      case(PM_STANDBY):
        {
          /* Logic for PM_STANDBY goes here */

#ifdef CONFIG_RUN_STANDBY

          /* change the microcontroller to RUN mode */

          /* and wait until in RUN mode */

          return_value =
            (int)s32k1xx_set_runmode(SCG_SYSTEM_CLOCK_MODE_RUN);
          DEBUGASSERT(return_value != (int)SCG_SYSTEM_CLOCK_MODE_NONE);

          /* enable all clock sources again if needed  */

          /* these could be the FIRC, PPL, and SOSC */

          /* check if the FIRC was enabled and
           * it is not the system clock source
           */

          if (g_initial_clkconfig.scg.firc.initialize &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SIRC))
          {
            /* enable FIRC */

            return_value = s32k1xx_firc_config(true,
              &g_initial_clkconfig.scg.firc);
            DEBUGASSERT(!return_value);
          }

          /* check if the FIRC needs to be disabled and if it is enabled */

          else if ((!(g_initial_clkconfig.scg.firc.initialize)) &&
            (s32k1xx_get_srcfreq(SCG_SYSTEM_CLOCK_SRC_FIRC)))
          {
            /* disable FIRC */

            return_value = s32k1xx_firc_config(false,
              &g_initial_clkconfig.scg.firc);
            DEBUGASSERT(!return_value);
          }

          /* check if the SOSC was enabled and
           * it is not the system clock source
           */

          if (g_initial_clkconfig.scg.sosc.initialize &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SYS_OSC) &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SYS_PLL))
          {
            /* enable SOSC */

            return_value =
              s32k1xx_sosc_config(true, &g_initial_clkconfig.scg.sosc);
            DEBUGASSERT(!return_value);
          }

          /* check if the SOSC needs to be disabled and if it is enabled */

          else if ((!(g_initial_clkconfig.scg.sosc.initialize)) &&
            (s32k1xx_get_srcfreq(SCG_SYSTEM_CLOCK_SRC_SYS_OSC)))
          {
            /* disable SOSC */

            return_value =
              s32k1xx_sosc_config(false, &g_initial_clkconfig.scg.sosc);
            DEBUGASSERT(!return_value);
          }

          /* check if the SPLL was enabled and
           * it is not the system clock source
           */

          if (g_initial_clkconfig.scg.spll.initialize &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SYS_OSC) &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SYS_PLL))
          {
            /* enable SPLL */

            return_value = s32k1xx_spll_config(true,
              &g_initial_clkconfig.scg.spll);
            DEBUGASSERT(!return_value);
          }

          /* check if the SPLL needs to be disabled and if it is enabled */

          else if ((!(g_initial_clkconfig.scg.spll.initialize)) &&
            (s32k1xx_get_srcfreq(SCG_SYSTEM_CLOCK_SRC_SYS_PLL)))
          {
            /* disable SPLL */

            return_value = s32k1xx_spll_config(false,
              &g_initial_clkconfig.scg.spll);
            DEBUGASSERT(!return_value);
          }

          /* check if the RCCR clock source is enabled */

          if (s32k1xx_get_srcfreq(g_initial_clkconfig.scg.clockmode.rccr.src)
            != 0)
          {
            /* change the system clock back to the configured clock */

            /* and wait until clock changed */

            if (s32k1xx_transition_systemclock(
              &g_initial_clkconfig.scg.clockmode.rccr))
            {
              /* error */

              DEBUGPANIC();
            }
          }

          /* if it is 0 */

          else
          {
            /* error */

            DEBUGPANIC();
          }

#endif /* CONFIG_RUN_STANDBY */

#ifdef CONFIG_VLPR_STANDBY

          /* set the system clock to the SIRC 8MHz freq */

          /* this freq will change to the predefined vccr settings
           * when the mode change occures
           */

          /* and wait until system clock changed */

          return_value = s32k1xx_sirc_clocksource();
          DEBUGASSERT(!return_value);

          /* disable the other clock sources if not already disabled */

          /* these are the FIRC, PPL, and SOSC */

          /* check if the SPLL is enabled */

          if (s32k1xx_get_spllfreq() != 0)
          {
            /* disable SPLL */

            return_value = s32k1xx_spll_config(false,
              &g_initial_clkconfig.scg.spll);
            DEBUGASSERT(!return_value);
          }

          /* check if the SOSC is enabled */

          if (s32k1xx_get_soscfreq() != 0)
          {
            /* disable SOSC */

            return_value =
              s32k1xx_sosc_config(false, &g_initial_clkconfig.scg.sosc);
            DEBUGASSERT(!return_value);
          }

          /* check if the FIRC is enabled */

          if (s32k1xx_get_fircfreq() != 0)
          {
            /* disable FIRC */

            return_value = s32k1xx_firc_config(false,
              &g_initial_clkconfig.scg.firc);
            DEBUGASSERT(!return_value);
          }

  #ifdef CONFIG_ARCH_CHIP_S32K11X
            /* TODO make sure CMU is gated? (only for S32k11x) */

            #error Make sure CMU is gated
  #endif

          /* change the microcontroller to VLPR mode */

          /* and wait until it is in that runmode */

          return_value =
            (int)s32k1xx_set_runmode(SCG_SYSTEM_CLOCK_MODE_VLPR);
          DEBUGASSERT(return_value != (int)SCG_SYSTEM_CLOCK_MODE_NONE);

#endif /* CONFIG_VLPR_STANDBY */

          /* calculate the new clock ticks */

          up_timer_initialize();
        }
        break;

      case(PM_SLEEP):
        {
          /* Logic for PM_SLEEP goes here */

#ifdef CONFIG_RUN_SLEEP

          /* change the microcontroller to RUN mode */

          /* and wait until in RUN mode */

          return_value = (int)s32k1xx_set_runmode(SCG_SYSTEM_CLOCK_MODE_RUN);
          DEBUGASSERT(return_value != (int)SCG_SYSTEM_CLOCK_MODE_NONE);

          /* enable all clock sources again if needed */

          /* these could be the FIRC, PPL, and SOSC */

          /* check if the FIRC was enabled
           * and it is not the system clock source
           */

          if (g_initial_clkconfig.scg.firc.initialize &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SIRC))
          {
            /* enable FIRC */

            return_value = s32k1xx_firc_config(true,
              &g_initial_clkconfig.scg.firc);
            DEBUGASSERT(!return_value);
          }

          /* check if the FIRC needs to be disabled and if it is enabled */

          else if ((!(g_initial_clkconfig.scg.firc.initialize)) &&
            (s32k1xx_get_srcfreq(SCG_SYSTEM_CLOCK_SRC_FIRC)))
          {
            /* disable FIRC */

            return_value = s32k1xx_firc_config(false,
              &g_initial_clkconfig.scg.firc);
            DEBUGASSERT(!return_value);
          }

          /* check if the SOSC was enabled
           * and it is not the system clock source
           */

          if (g_initial_clkconfig.scg.sosc.initialize &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SYS_OSC) &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SYS_PLL))
          {
            /* enable SOSC */

            return_value =
              s32k1xx_sosc_config(true, &g_initial_clkconfig.scg.sosc);
            DEBUGASSERT(!return_value);
          }

          /* check if the SOSC needs to be disabled and if it is enabled */

          else if ((!(g_initial_clkconfig.scg.sosc.initialize)) &&
            (s32k1xx_get_srcfreq(SCG_SYSTEM_CLOCK_SRC_SYS_OSC)))
          {
            /* disable SOSC */

            return_value =
              s32k1xx_sosc_config(false, &g_initial_clkconfig.scg.sosc);
            DEBUGASSERT(!return_value);
          }

          /* check if the SPLL was enabled
           * and it is not the system clock source
           */

          if (g_initial_clkconfig.scg.spll.initialize &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SYS_OSC) &&
            (s32k1xx_get_scgclk_source() != SCG_SYSTEM_CLOCK_SRC_SYS_PLL))
          {
            /* enable SPLL */

            return_value = s32k1xx_spll_config(true,
              &g_initial_clkconfig.scg.spll);
            DEBUGASSERT(!return_value);
          }

          /* check if the SPLL needs to be disabled and if it is enabled */

          else if ((!(g_initial_clkconfig.scg.spll.initialize)) &&
            (s32k1xx_get_srcfreq(SCG_SYSTEM_CLOCK_SRC_SYS_PLL)))
          {
            /* disable SPLL */

            return_value = s32k1xx_spll_config(false,
              &g_initial_clkconfig.scg.spll);
            DEBUGASSERT(!return_value);
          }

          /* check if the RCCR clock source is enabled */

          if (s32k1xx_get_srcfreq(g_initial_clkconfig.scg.clockmode.rccr.src)
            != 0)
          {
            /* change the system clock back to the configured clock */

            /* and wait until clock changed */

            if (s32k1xx_transition_systemclock(
              &g_initial_clkconfig.scg.clockmode.rccr))
            {
              /* error */

              DEBUGPANIC();
            }
          }

          /* if it is 0 */

          else
          {
            /* error */

            DEBUGPANIC();
          }

#endif /* CONFIG_RUN_SLEEP */

#ifdef CONFIG_VLPR_SLEEP

          /* set the system clock to the SIRC 8MHz freq */

          /* this freq will change to the predefined vccr settings
           * when the mode change occures
           */

          /* and wait until system clock changed */

          return_value = s32k1xx_sirc_clocksource();
          DEBUGASSERT(!return_value);

          /* disable the other clock sources if not already disabled */

          /* these are the FIRC, PPL, and SOSC */

          /* check if the SPLL is enabled */

          if (s32k1xx_get_spllfreq() != 0)
          {
            /* disable SPLL */

            return_value =
            s32k1xx_spll_config(false, &g_initial_clkconfig.scg.spll);
            DEBUGASSERT(!return_value);
          }

          /* check if the SOSC is enabled */

          if (s32k1xx_get_soscfreq() != 0)
          {
            /* disable SOSC */

            return_value =
              s32k1xx_sosc_config(false, &g_initial_clkconfig.scg.sosc);
            DEBUGASSERT(!return_value);
          }

          /* check if the FIRC is enabled */

          if (s32k1xx_get_fircfreq() != 0)
          {
            /* disable FIRC */

            return_value =
            s32k1xx_firc_config(false, &g_initial_clkconfig.scg.firc);
            DEBUGASSERT(!return_value);
          }

  #ifdef CONFIG_ARCH_CHIP_S32K11X
            /* TODO make sure CMU is gated? (only for S32k11x) */

            #error Make sure CMU is gated
  #endif
          /* change the microcontroller to VLPR mode */

          /* and wait until it is in that runmode */

          return_value =
          (int)s32k1xx_set_runmode(SCG_SYSTEM_CLOCK_MODE_VLPR);
          DEBUGASSERT(return_value != (int)SCG_SYSTEM_CLOCK_MODE_NONE);

#endif /* CONFIG_VLPR_SLEEP */

          /* calculate the new clock ticks */

          up_timer_initialize();
        }
        break;

      default:

        /* Should not get here */

        break;
    }
}
#endif

/****************************************************************************
 * Name: up_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   Zero - (OK) means the event was successfully processed and that the
 *          driver is prepared for the PM state change.
 *
 *   Non-zero - means that the driver is not prepared to perform the tasks
 *              needed achieve this power setting and will cause the state
 *              change to be aborted. NOTE: The prepare() method will also
 *              be called when reverting from lower back to higher power
 *              consumption modes (say because another driver refused a
 *              lower power state change). Drivers are not permitted to
 *              return non-zero values when reverting back to higher power
 *              consumption modes!
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int up_pm_prepare(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  /* Logic to prepare for a reduced power state goes here. */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_get_runmode
 *
 * Description:
 *   Get the current running mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current running mode.
 *
 ****************************************************************************/

enum scg_system_clock_mode_e s32k1xx_get_runmode(void)
{
  enum scg_system_clock_mode_e mode;

  /* Get the current running mode */

  switch (getreg32(S32K1XX_SMC_PMSTAT) & SMC_PMSTAT_PMSTAT_MASK)
    {
      /* Run mode */

      case SMC_PMSTAT_PMSTAT_RUN:
        mode = SCG_SYSTEM_CLOCK_MODE_RUN;
        break;

      /* Very low power run mode */

      case SMC_PMSTAT_PMSTAT_VLPR:
        mode = SCG_SYSTEM_CLOCK_MODE_VLPR;
        break;

      /* High speed run mode */

      case SMC_PMSTAT_PMSTAT_HSRUN:
        mode = SCG_SYSTEM_CLOCK_MODE_HSRUN;
        break;

      /* This should never happen - core has to be in some run mode to
       * execute code
       */

      case SMC_PMSTAT_PMSTAT_VLPS:
      default:
        mode = SCG_SYSTEM_CLOCK_MODE_NONE;
        break;
    }

    return mode;
}

/****************************************************************************
 * Name: s32k1xx_set_runmode
 *
 * Description:
 *   Set the running mode.
 *
 * Input Parameters:
 *   next_run_mode - The next running mode.
 *
 * Returned Value:
 *   The current running mode.
 *
 ****************************************************************************/

enum scg_system_clock_mode_e s32k1xx_set_runmode(enum scg_system_clock_mode_e
  next_run_mode)
{
  enum scg_system_clock_mode_e mode;

  /* get the current run mode */

  mode = s32k1xx_get_runmode();
  uint32_t regval;

  /* check if the current runmode is not the same as the next runmode */

  if (mode != next_run_mode)
    {
      /* check what the next mode is */

      switch (next_run_mode)
      {
        /* in case of the RUN mode */

        /* it will use the clock configuration from S32K1XX_SCG_RCCR */

        case SCG_SYSTEM_CLOCK_MODE_RUN:

          /* check if in VLPR mode */

          if (mode == SCG_SYSTEM_CLOCK_MODE_VLPR)
          {
            /* get the SMC_PMCTRL register */

            regval = getreg32(S32K1XX_SMC_PMCTRL);

            /* mask the RUNM bits */

            regval &= ~SMC_PMCTRL_RUNM_MASK;

            /* change the mode to RUN mode */

            regval |= SMC_PMCTRL_RUNM_RUN;

            /* write the register */

            putreg32(regval, S32K1XX_SMC_PMCTRL);

            /* wait until it is in RUN mode */

            while (s32k1xx_get_runmode() != SCG_SYSTEM_CLOCK_MODE_RUN);
          }

        break;

        /* in case of the VLPR mode */

        /* it will use the clock configuration from S32K1XX_SCG_VCCR */

        case SCG_SYSTEM_CLOCK_MODE_VLPR:

          /* check if in RUN mode and VLPR mode is allowed */

          if ((mode == SCG_SYSTEM_CLOCK_MODE_RUN) &&
            (getreg32(S32K1XX_SMC_PMPROT) & SMC_PMPROT_AVLP))
          {
            /* get the SMC_PMCTRL register */

            regval = getreg32(S32K1XX_SMC_PMCTRL);

            /* mask the RUNM bits */

            regval &= ~SMC_PMCTRL_RUNM_MASK;

            /* change the mode to VLPR mode */

            regval |= SMC_PMCTRL_RUNM_VLPR;

            /* write the register */

            putreg32(regval, S32K1XX_SMC_PMCTRL);

            /* wait until it is in VLPR mode */

            while (s32k1xx_get_runmode() != SCG_SYSTEM_CLOCK_MODE_VLPR);
          }
        break;

        /* others are not implemented */

        default:
        break;
      }

      /* get the current run mode */

      mode = s32k1xx_get_runmode();
    }

  /* return the mode */

  return mode;
}

/****************************************************************************
 * Name: s32k1xx_clockconfig
 *
 * Description:
 *   Called to initialize the S32K1XX.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 * Input Parameters:
 *   clkcfg - Describes the new clock configuration
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int s32k1xx_clockconfig(const struct clock_configuration_s *clkcfg)
{
  int ret;

  DEBUGASSERT(clkcfg != NULL);

  /* Set SCG configuration */

  ret = s32k1xx_scg_config(&clkcfg->scg);
  if (ret >= 0)
    {
      /* Allow the VLPR mode */

      s32k1xx_allow_vlprmode(true);

      /* Set PCC configuration */

      s32k1xx_periphclocks(clkcfg->pcc.count, clkcfg->pcc.pclks);

      /* Set SIM configuration */

      s32k1xx_sim_config(&clkcfg->sim);

      /* Set PMC configuration */

      s32k1xx_pmc_config(&clkcfg->pmc);
    }

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_clock_pm_register
 *
 * Description:
 *   This function is called after OS and PM init in order to register to
 *   receive power management event callbacks.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/
#ifdef CONFIG_PM
void s32k1xx_clock_pm_register(void)
{
  /* Register to receive power management callbacks */

  pm_register(&g_clock_pmcb);
}
#endif

/****************************************************************************
 * Name: s32k1xx_get_coreclk
 *
 * Description:
 *   Return the current value of the CORE clock frequency.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   The current value of the CORE clock frequency.  Zero is returned on any
 *   failure.
 *
 ****************************************************************************/

uint32_t s32k1xx_get_coreclk(void)
{
  uint32_t coreclk = 0;
  uint32_t regval;
  uint32_t divider;
#ifdef CONFIG_S32K1XX_HAVE_SPLL
  uint32_t prediv;
  uint32_t mult;
#endif

  /* Get the core clock divider */

  regval  = getreg32(S32K1XX_SCG_CSR);
  divider = ((regval & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT) + 1;

  /* Handle according to the selection clock source */

  switch (regval & SCG_CSR_SCS_MASK)
    {
      case SCG_CSR_SCS_SOSC:  /* System OSC */

        coreclk = BOARD_XTAL_FREQUENCY;
        break;

      case SCG_CSR_SCS_SIRC:  /* Slow IRC */
        regval = getreg32(S32K1XX_SCG_SIRCCFG) & SCG_SIRCCFG_RANGE;
        if (regval == SCG_SIRCCFG_LOWRANGE)
          {
            /* Slow IRC low range clock (2 MHz) */

            return 0;
          }

        /* Slow IRC high range clock (8 MHz ) */

        coreclk = SCG_SIRC_HIGHRANGE_FREQUENCY;
        break;

      case SCG_CSR_SCS_FIRC:  /* Fast IRC */
        regval = getreg32(S32K1XX_SCG_FIRCCFG) & SCG_FIRCCFG_RANGE;
        if (regval != SCG_FIRCCFG_48MHZ)
          {
            return 0;
          }

        /* Fast IRC is trimmed to 48 MHz */

        coreclk = SCG_FIRC_FREQUENCY0;
        break;

#ifdef CONFIG_S32K1XX_HAVE_SPLL
      case SCG_CSR_SPLL_FIRC:  /* System PLL */

        /* Coreclock = Fxtal * mult / (2 * prediv) */

        regval  = getreg32(S32K1XX_SCG_SPLLCFG);
        prediv  = ((regval & SCG_SPLLCFG_PREDIV_MASK) >>
                    SCG_SPLLCFG_PREDIV_SHIFT) + 1;
        mult    = ((regval & SCG_SPLLCFG_MULT_MASK) >>
                    SCG_SPLLCFG_MULT_SHIFT) + 16;

        coreclk = ((BOARD_XTAL_FREQUENCY / 2) * mult) / prediv;
        break;
#endif

      default:
        return 0;
    }

  return coreclk / divider;
}

/****************************************************************************
 * Name: s32k1xx_get_sysclk
 *
 * Description:
 *   Return the current value of an SCG system clock frequency, these clocks
 *   are used for core, platform, external and bus clock domains..
 *
 * Input Parameters:
 *   type - Identifies the system clock of interest
 *
 * Returned Values:
 *   The current value of the system clock frequency.  Zero is returned on
 *   any failure.
 *
 ****************************************************************************/

uint32_t s32k1xx_get_sysclk(enum scg_system_clock_type_e type)
{
  enum scg_system_clock_src_e clksrc;
  uint32_t regval;
  uint32_t divider;
  uint32_t freq;

  /* Get the SCG current system clock source */

  clksrc  = (enum scg_system_clock_src_e)s32k1xx_get_scgclk_source();
  freq    = s32k1xx_get_srcfreq(clksrc);

  /* Adjust to count for the code divider */

  regval  = getreg32(S32K1XX_SCG_CSR);
  divider = ((regval & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT) + 1;
  freq   /= divider;

  /* Handle additional dividers for the clock type */

  switch (type)
    {
      case SCG_SYSTEM_CLOCK_CORE:
        break;

      case SCG_SYSTEM_CLOCK_BUS:
        divider = ((regval & SCG_CSR_DIVBUS_MASK) >>
                    SCG_CSR_DIVBUS_SHIFT) + 1;
        freq   /= divider;
        break;

      case SCG_SYSTEM_CLOCK_SLOW:
        divider = ((regval & SCG_CSR_DIVSLOW_MASK) >>
                    SCG_CSR_DIVSLOW_SHIFT) + 1;
        freq   /= divider;
        break;

      default:
        freq = 0;
        break;
    }

  return freq;
}

/****************************************************************************
 * Name: s32k1xx_get_asnchfreq
 *
 * Description:
 *   Gets SCG asynchronous clock frequency from a clock source.
 *
 * Input Parameters:
 *   clksrc - The requested clock source.
 *   type   - The requested clock type.
 *
 * Returned Value:
 *   The frequency of the requested asynchronous clock source.
 *
 ****************************************************************************/

uint32_t s32k1xx_get_asnchfreq(enum clock_names_e clksrc,
                               enum scg_async_clock_type_e type)
{
  uint32_t regval;
  uint32_t freq = 0;
  uint32_t div  = 0;

  switch (type)
    {
      case SCG_ASYNC_CLOCK_DIV1:
        {
          switch (clksrc)
            {
              case FIRC_CLK:
                {
                  freq   = s32k1xx_get_fircfreq();
                  regval = getreg32(S32K1XX_SCG_FIRCDIV);
                  div    = (regval & SCG_FIRCDIV_FIRCDIV1_MASK) >>
                            SCG_FIRCDIV_FIRCDIV1_SHIFT;
                }
                break;

              case SIRC_CLK:
                {
                  freq   = s32k1xx_get_sircfreq();
                  regval = getreg32(S32K1XX_SCG_SIRCDIV);
                  div    = (regval & SCG_SIRCDIV_SIRCDIV1_MASK) >>
                            SCG_SIRCDIV_SIRCDIV1_SHIFT;
                }
                break;

              case SOSC_CLK:
                {
                  freq   = s32k1xx_get_soscfreq();
                  regval = getreg32(S32K1XX_SCG_SOSCDIV);
                  div    = (regval & SCG_SOSCDIV_SOSCDIV1_MASK) >>
                            SCG_SOSCDIV_SOSCDIV1_SHIFT;
                }
                break;

#ifdef CONFIG_S32K1XX_HAVE_SPLL
              case SPLL_CLK:
                {
                  freq   = s32k1xx_get_spllfreq();
                  regval = getreg32(S32K1XX_SCG_SPLLDIV);
                  div    = (regval & SCG_SPLLDIV_SPLLDIV1_MASK) >>
                            SCG_SPLLDIV_SPLLDIV1_SHIFT;
                }
                break;
#endif

              default:
                {
                  /* Invalid clock source type */

                  freq = 0;
                  DEBUGPANIC();
                }
                break;
            }
        }
        break;

      case SCG_ASYNC_CLOCK_DIV2:
        {
          switch (clksrc)
            {
              case FIRC_CLK:
                {
                  freq   = s32k1xx_get_fircfreq();
                  regval = getreg32(S32K1XX_SCG_FIRCDIV);
                  div    = (regval & SCG_FIRCDIV_FIRCDIV2_MASK) >>
                            SCG_FIRCDIV_FIRCDIV2_SHIFT;
                }
                break;

              case SIRC_CLK:
                {
                  freq   = s32k1xx_get_sircfreq();
                  regval = getreg32(S32K1XX_SCG_SIRCDIV);
                  div    = (regval & SCG_SIRCDIV_SIRCDIV2_MASK) >>
                            SCG_SIRCDIV_SIRCDIV2_SHIFT;
                }
                break;
              case SOSC_CLK:
                {
                  freq   = s32k1xx_get_soscfreq();
                  regval = getreg32(S32K1XX_SCG_SOSCDIV);
                  div    = (regval & SCG_SOSCDIV_SOSCDIV2_MASK) >>
                            SCG_SOSCDIV_SOSCDIV2_SHIFT;
                }
                break;
#ifdef CONFIG_S32K1XX_HAVE_SPLL
              case SPLL_CLK:
                {
                  freq = s32k1xx_get_spllfreq();
                  regval = getreg32(S32K1XX_SCG_SPLLDIV);
                  div    = (regval & SCG_SPLLDIV_SPLLDIV2_MASK) >>
                            SCG_SPLLDIV_SPLLDIV2_SHIFT;
                }
                break;
#endif
              default:
                {
                  /* Invalid clock source type */

                  freq = 0;
                  DEBUGPANIC();
                }
                break;
            }
        }
        break;

      default:

          /* Invalid async clock source */

          freq = 0;
          DEBUGPANIC();
          break;
    }

  if (div != 0)
    {
      freq = (freq >> (div - 1));
    }
  else  /* Output disabled. */
    {
      freq = 0;
    }

  return freq;
}
