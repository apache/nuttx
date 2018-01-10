/****************************************************************************
 * arch/arm/src/lc823450/lc823450_dvfs2.c
 *
 *   Copyright (C) 2015-2017 Sony Corporation. All rights reserved.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
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

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <arch/board/board.h>
#include <string.h>

#include "up_arch.h"

#include "lc823450_clockconfig.h"
#include "lc823450_syscontrol.h"
#include "lc823450_intc.h"
#include "lc823450_sdc.h"
#include "lc823450_dvfs2.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FREQ_160 0
#define FREQ_080 1
#define FREQ_040 2

/****************************************************************************
 * Private Data
 ****************************************************************************/

typedef struct freq_entry
{
  uint16_t freq;
  uint16_t pll1;
  uint16_t mdiv;
  uint16_t hdiv;
} t_freq_entry;

static struct freq_entry _dvfs_act_tbl[3] =
{
  { 160, OSCCNT_MCSEL, OSCCNT_MAINDIV_1, 3},  /* PLL1 */
  {  80, OSCCNT_MCSEL, OSCCNT_MAINDIV_2, 1},  /* PLL1 */
  {  40, OSCCNT_MCSEL, OSCCNT_MAINDIV_4, 0},  /* PLL1 */
};

static struct freq_entry _dvfs_idl_tbl[3] =
{
  {  24,            0, OSCCNT_MAINDIV_1, 0},  /* XT1  */
  {  12,            0, OSCCNT_MAINDIV_2, 0},  /* XT1  */
  {   6,            0, OSCCNT_MAINDIV_4, 0},  /* XT1  */
};

static uint16_t _dvfs_cur_idx  = 0; /* current speed index */
static uint16_t _dvfs_cur_hdiv = 3;
static uint16_t _dvfs_cur_mdiv = OSCCNT_MAINDIV_1;

#if 0
static uint16_t _dvfs_init_timeout = (5 * 100); /* in ticks */
#endif

#if defined(CONFIG_SMP) && (CONFIG_SMP_NCPUS == 2)
static uint8_t  _dvfs_cpu_is_active[CONFIG_SMP_NCPUS];
#endif

static void lc823450_dvfs_set_div(int idx, int tbl);

/****************************************************************************
 * Public Data
 ****************************************************************************/

int8_t   g_dvfs_enabled  = 0;
uint16_t g_dvfs_cur_freq = 160;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_dvfs_update_lpm
 ****************************************************************************/

static void lc823450_dvfs_update_lpm(int freq)
{
  /* TODO */
}

#if defined(CONFIG_SMP) && (CONFIG_SMP_NCPUS == 2)
static int _dvfs_another_cpu_state(int me)
{
  if (0 == me)
    {
      return _dvfs_cpu_is_active[1];
    }
  else
    {
      return _dvfs_cpu_is_active[0];
    }
}
#endif

/****************************************************************************
 * Name: lc823450_dvfs_set_div
 * Set dividers in the OSC block
 * target tbl: 0=active, 1=idle
 ****************************************************************************/

static void lc823450_dvfs_set_div(int idx, int tbl)
{
  uint32_t target;
  uint32_t t_hdiv;
  uint32_t t_mdiv;

  if (0 == tbl)
    {
      target  = _dvfs_act_tbl[idx].freq;
      t_hdiv  = _dvfs_act_tbl[idx].hdiv;
      t_mdiv  = _dvfs_act_tbl[idx].mdiv;
    }
  else
    {
      target  = _dvfs_idl_tbl[idx].freq;
      t_hdiv  = _dvfs_idl_tbl[idx].hdiv;
      t_mdiv  = _dvfs_idl_tbl[idx].mdiv;
    }

  if (100 < target)
    {
      /* Set ROM wait cycle (CPU=1wait) */

      modifyreg32(MEMEN4, 0, MEMEN4_HWAIT);
    }

    /* adjust AHB */

  if (t_hdiv > _dvfs_cur_hdiv)
    {
      uint32_t pclkdiv = t_hdiv;
#ifdef CONFIG_LC823450_SDRAM
      pclkdiv += (t_hdiv << 16);
#endif
      putreg32(pclkdiv, PERICLKDIV);
    }

  uint32_t regval = getreg32(OSCCNT);

  /* NOTE: In LC823450, MCSEL is reflected first then MAINDIV */
  /* To avoid spec violation, 2-step clock change is needed */

  /* step 1 : change MAINDIV if needed */

  if (t_mdiv > _dvfs_cur_mdiv)
    {
      regval &= ~OSCCNT_MAINDIV_MASK;
      regval |= t_mdiv;

      /* change the MAINDIV first */

      putreg32(regval, OSCCNT);
    }

    /* step 2 : change MCSEL and MAINDIV */

  regval = getreg32(OSCCNT);
  regval &= ~(OSCCNT_MCSEL | OSCCNT_MAINDIV_MASK);

  if (0 == tbl)
    {
      regval |= _dvfs_act_tbl[idx].pll1;
    }
  else
    {
      regval |= _dvfs_idl_tbl[idx].pll1;
    }

  regval |= t_mdiv;

  /* set MCSEL and MAINDIV again */

  putreg32(regval, OSCCNT);

  /* update loops_per_msec for up_udelay(), up_mdelay() */

  if (0 == tbl)
    {
      lc823450_dvfs_update_lpm(_dvfs_act_tbl[idx].freq);
    }
  else
    {
      lc823450_dvfs_update_lpm(_dvfs_idl_tbl[idx].freq);
    }

  /* adjust AHB */

  if (t_hdiv < _dvfs_cur_hdiv)
    {
      uint32_t pclkdiv = t_hdiv;
#ifdef CONFIG_LC823450_SDRAM
      pclkdiv += (t_hdiv << 16);
#endif
      putreg32(pclkdiv, PERICLKDIV);
    }

  _dvfs_cur_idx   = idx;
  _dvfs_cur_hdiv  = t_hdiv;
  _dvfs_cur_mdiv  = t_mdiv;
  g_dvfs_cur_freq = target;

  if (100 > target)
    {
      /* Clear ROM wait cycle (CPU=0wait) */

      modifyreg32(MEMEN4, MEMEN4_HWAIT, 0);
    }

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_get_apb
 * Assumption: CPU=APB
 ****************************************************************************/

uint32_t lc823450_get_apb(void)
{
  return g_dvfs_cur_freq * 1000000;
}

/****************************************************************************
 * Name: lc823450_dvfs_tick_callback
 * This callback is called in the timer interupt
 ****************************************************************************/

void lc823450_dvfs_tick_callback(void)
{
#if 0
  if (_dvfs_init_timeout)
    {
      _dvfs_init_timeout--;

      if (0 == _dvfs_init_timeout)
        {
          g_dvfs_enabled = 1;
        }
    }
#endif
}

/****************************************************************************
 * Name: lc823450_dvfs_enter_idle
 ****************************************************************************/

void lc823450_dvfs_enter_idle(void)
{
  irqstate_t flags = spin_lock_irqsave();

  if (0 == g_dvfs_enabled)
    {
      goto exit_with_error;
    }

#if defined(CONFIG_SMP) && (CONFIG_SMP_NCPUS == 2)
  int me = up_cpu_index();

  /* Update my state first : 0 (idle) */

  _dvfs_cpu_is_active[me] = 0;

  /* check if another core is still active */

  if (_dvfs_another_cpu_state(me))
    {
      /* do not change to idle clock */

      goto exit_with_error;
    }

#endif

#ifdef CONFIG_DVFS_CHECK_SDC
  if (lc823450_sdc_locked())
    {
      goto exit_with_error;
    }
#endif

  /* NOTE: set idle freq : idx=same, change:tbl */

  lc823450_dvfs_set_div(_dvfs_cur_idx, 1);

exit_with_error:
  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: lc823450_dvfs_exit_idle
 * This API is called in up_ack_irq() (i.e. interrupt context)
 ****************************************************************************/

void lc823450_dvfs_exit_idle(int irq)
{
  irqstate_t flags = spin_lock_irqsave();

  if (0 == g_dvfs_enabled)
    {
      goto exit_with_error;
    }

#if defined(CONFIG_SMP) && (CONFIG_SMP_NCPUS == 2)
  int me = up_cpu_index();

  /* Update my state first: 1 (active) */

  _dvfs_cpu_is_active[me] = 1;

  /* Check if another core is already active */

  if (_dvfs_another_cpu_state(me))
    {
      /* do nothing */

      goto exit_with_error;
    }
#endif

  /* NOTE: set active freq : idx=same, change:tbl */

  lc823450_dvfs_set_div(_dvfs_cur_idx, 0);

exit_with_error:
  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: lc823450_dvfs_boost
 * boost the sytem clock to MAX (i.e. 160M)
 * timeout in msec
 ****************************************************************************/

int lc823450_dvfs_boost(int timeout)
{
  /* TODO */
  return 0;
}

/****************************************************************************
 * Name: lc823450_dvfs_set_freq
 * NOTE: should be called from dvfs command only
 ****************************************************************************/

int lc823450_dvfs_set_freq(int freq)
{
  int ret = 0;
  int idx;
  irqstate_t flags;

  if (0 == g_dvfs_enabled)
    {
      return -1;
    }

  flags = spin_lock_irqsave();

  switch (freq)
    {
      case 160:
        idx = FREQ_160;
        break;

      case 80:
        idx = FREQ_080;
        break;

      case 40:
        idx = FREQ_040;
        break;

      default:
        ret = -1;
        break;
    }

  if (0 == ret)
    {
      lc823450_dvfs_set_div(idx, 0);
    }

  spin_unlock_irqrestore(flags);
  return ret;
}
