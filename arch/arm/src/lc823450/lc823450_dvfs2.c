/****************************************************************************
 * arch/arm/src/lc823450/lc823450_dvfs2.c
 *
 *   Copyright 2015,2016,2017,2018 Sony Video & Sound Products Inc.
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
#include "lc823450_gpio.h"
#include "lc823450_timer.h"
#include "lc823450_dvfs2.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FREQ_160 0
#define FREQ_080 1
#define FREQ_040 2

#define FREQ_KP  2
#define FREQ_UP  1
#define FREQ_DN  0

#define UP_THRESHOLD 20
#define DN_THRESHOLD 60

#ifndef CONFIG_SMP_NCPUS
#  define CONFIG_SMP_NCPUS 1
#endif

#ifdef CONFIG_DVFS_CHANGE_VOLTAGE
#  define CORE12V_PIN (GPIO_PORT2 | GPIO_PIN1)
#endif

#ifndef CONFIG_RTC_HIRES
#  error "Should be enabled to get high-accuracy for CPU idle time"
#endif

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
static uint16_t _dvfs_core12v = 0;

static uint8_t  _dvfs_cpu_is_active[CONFIG_SMP_NCPUS];

static void lc823450_dvfs_set_div(int idx, int tbl);

static uint64_t g_idle_starttime[CONFIG_SMP_NCPUS];
static uint64_t g_idle_totaltime[CONFIG_SMP_NCPUS];
static uint64_t g_idle_totaltime0[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Data
 ****************************************************************************/

int8_t   g_dvfs_enabled  = 0;
int8_t   g_dvfs_auto     = 0;
uint16_t g_dvfs_cur_freq = 160;

uint32_t g_dvfs_freq_stat[3] =
  {
    0, 0, 0
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_get_current_time()
 ****************************************************************************/

static uint64_t _get_current_time64(void)
{
  struct timespec ts;

#ifdef CONFIG_CLOCK_MONOTONIC
  clock_gettime(CLOCK_MONOTONIC, &ts);
#else
  clock_gettime(CLOCK_REALTIME, &ts);
#endif
  return (uint64_t)ts.tv_sec * NSEC_PER_SEC + (uint64_t)ts.tv_nsec;
}

/****************************************************************************
 * Name: lc823450_dvfs_update_lpm
 ****************************************************************************/

static void lc823450_dvfs_update_lpm(int freq)
{
  /* TODO */
}

#if CONFIG_SMP_NCPUS == 2
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
 * Name: lc823450_dvfs_oneshot
 * Callback for 1 shot timer
 ****************************************************************************/

int lc823450_dvfs_oneshot(int irq, uint32_t *regs, FAR void *arg)
{
  /* voltage has reached at 1.2V */

  _dvfs_core12v = 1;

  lc823450_mtm_stop_oneshot();

  /* go to max freq */

  lc823450_dvfs_set_div(FREQ_160, 0);
  return 0;
}

/****************************************************************************
 * Name: lc832450_set_core_voltage
 * high=true (1.2V), high=false (1.0V)
 ****************************************************************************/

#ifdef CONFIG_DVFS_CHANGE_VOLTAGE
static void lc832450_set_core_voltage(bool high)
{
  if (false == high)
    {
      _dvfs_core12v = 0;
    }

  lc823450_gpio_write(CORE12V_PIN, (int)high);

  if (high)
    {
      /* start 3ms timer to change internal dividers */

      lc823450_mtm_start_oneshot(3);
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

#ifdef CONFIG_DVFS_CHANGE_VOLTAGE
  if (100 < target && !_dvfs_core12v)
    {
      /* Set high voltage (i.e. 1.2v) */

      lc832450_set_core_voltage(true);
      return;
    }
#endif

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


#ifdef CONFIG_DVFS_CHANGE_VOLTAGE
  /* NOTE: check the index instead of the target freq */

  if (_dvfs_core12v && _dvfs_cur_idx != FREQ_160)
    {
      /* set to lower voltage (i.e. 1.0v) */

      lc832450_set_core_voltage(false);
    }
#endif

  if (100 > target)
    {
      /* Clear ROM wait cycle (CPU=0wait) */

      modifyreg32(MEMEN4, MEMEN4_HWAIT, 0);
    }

}

/****************************************************************************
 * Name: lc823450_dvfs_change_idx
 * up: 1=clock up, 0=clock down
 * called in autonomous mode from interrupt context
 ****************************************************************************/

static void lc823450_dvfs_change_idx(int up)
{
  uint16_t idx = _dvfs_cur_idx;

  /* NOTE: clock index is in reverse order to the speed */

  /* clock up */

  if (up && (FREQ_160 < idx))
    {
      idx--;
    }

  /* clock down */

  if (!up && (FREQ_040 > idx))
    {
      idx++;
    }

#ifdef CONFIG_DVFS_EXCLUDE_40M
  if (idx == FREQ_040)
    {
      return;
    }
#endif

  if (idx != _dvfs_cur_idx)
    {
      lc823450_dvfs_set_div(idx, 0);
    }

}

/****************************************************************************
 * Name: lc823450_dvfs_do_auto
 ****************************************************************************/

static void lc823450_dvfs_do_auto(uint32_t idle[])
{
  uint32_t state[CONFIG_SMP_NCPUS];
  int i;

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      if (UP_THRESHOLD >= idle[i])
        {
          state[i] = FREQ_UP;
        }
      else if (DN_THRESHOLD <= idle[i])
        {
          state[i] = FREQ_DN;
        }
      else
        {
          state[i] = FREQ_KP;
        }
    }

#if CONFIG_SMP_NCPUS == 1
  if (FREQ_UP == state[0])
#elif CONFIG_SMP_NCPUS == 2
  if (FREQ_UP == state[0] || FREQ_UP == state[1])
#endif
    {
      lc823450_dvfs_change_idx(FREQ_UP);
    }

#if CONFIG_SMP_NCPUS == 1
  if (FREQ_DN == state[0])
#elif CONFIG_SMP_NCPUS == 2
  if (FREQ_DN == state[0] && FREQ_DN == state[1])
#endif
    {
      lc823450_dvfs_change_idx(FREQ_DN);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_dvfs_get_idletime
 ****************************************************************************/

void lc823450_dvfs_get_idletime(uint64_t idletime[])
{
  irqstate_t flags = spin_lock_irqsave();

  /* First, copy g_idle_totaltime to the caller */

  memcpy(idletime, g_idle_totaltime, sizeof(g_idle_totaltime));

#if CONFIG_SMP_NCPUS == 2
  int me = up_cpu_index();

  if (0 == _dvfs_another_cpu_state(me))
    {
      /* Another CPU is in idle, so consider this situation */

      int cpu = (me == 0) ? 1 : 0;
      idletime[cpu] += (_get_current_time64() - g_idle_starttime[cpu]);

      /* NOTE: g_idletotaltime[cpu] must not be updated */
    }
#endif

  spin_unlock_irqrestore(flags);
}

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
 * This callback is called in the timer interupt on CPU0
 ****************************************************************************/

void lc823450_dvfs_tick_callback(void)
{
  uint64_t tmp_idle_total[CONFIG_SMP_NCPUS];
  uint32_t idle[CONFIG_SMP_NCPUS];
  int i;

  if (g_dvfs_enabled && g_dvfs_auto)
    {
      lc823450_dvfs_get_idletime(tmp_idle_total);

      /* Calculate idle ratio for each CPU */

      for (i = 0; i < CONFIG_SMP_NCPUS; i++)
        {
          idle[i] = 100 * (tmp_idle_total[i] - g_idle_totaltime0[i])
            / NSEC_PER_TICK;
        }

      /* Update g_idle_totaltime0 */

      memcpy(g_idle_totaltime0, tmp_idle_total, sizeof(tmp_idle_total));

      /* Do autonomous mode */

      lc823450_dvfs_do_auto(idle);
    }

  /* Update freqency statistics */

  g_dvfs_freq_stat[_dvfs_cur_idx]++;
}

/****************************************************************************
 * Name: lc823450_dvfs_enter_idle
 ****************************************************************************/

void lc823450_dvfs_enter_idle(void)
{
  irqstate_t flags = spin_lock_irqsave();

  int me = up_cpu_index();

  /* Update my state first : 0 (idle) */

  _dvfs_cpu_is_active[me] = 0;

  /* Update my idle start time */

  g_idle_starttime[me] = _get_current_time64();

  if (0 == g_dvfs_enabled)
    {
      goto exit_with_error;
    }

#if CONFIG_SMP_NCPUS == 2
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

  int me = up_cpu_index();
  uint64_t d;
  uint64_t now;

  if (0 == g_dvfs_enabled)
    {
      goto exit_with_error;
    }

#if CONFIG_SMP_NCPUS == 2
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

  if (0 == _dvfs_cpu_is_active[me])
    {
      /* In case of idle to active transition */
      /* Accumulate idle total time on this CPU */

      now = _get_current_time64();
      d = now - g_idle_starttime[me];
      g_idle_totaltime[me] += d;
    }

  /* Finally update my state : 1 (active) */

  _dvfs_cpu_is_active[me] = 1;

  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: lc823450_dvfs_boost
 * boost the system clock to MAX (i.e. 160M)
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

#ifndef CONFIG_DVFS_EXCLUDE_40M
      case 40:
        idx = FREQ_040;
        break;
#endif

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
