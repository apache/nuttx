/****************************************************************************
 * arch/arm/src/ra8m1/ra_gpt.c
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

/* GPT as a generic /dev/timerN driver (upper half: drivers/timers/timer.c).
 *
 * Each channel runs in saw-wave PWM mode (GTCR.MD = 0), counting up.  GTCNT
 * counts 0..GTPR and the overflow (GTST.TCFPO, event GPTn_OVF) is the timer
 * expiry, so the period is (GTPR + 1) counts of the selected PCLKD divider.
 *
 * Notes from the RA8M1 User's Manual, section 21:
 *
 *  - MD, TPCS, GTUDDTYC.UD and GTCNT must be changed only while the counter
 *    is stopped.  GTCR.CST itself is synchronised to the count clock, so the
 *    counter may still tick (and raise an interrupt) after CST is cleared
 *    (21.10.4).
 *  - GTCNT must stay in 0 <= GTCNT <= GTPR (21.10.3).
 *  - There is no interrupt-enable bit for overflow in GTINTAD: the event is
 *    gated only by the ICU (IELSR routing + NVIC).
 *  - GTST flags are cleared by writing 0 to them (21.2.16).
 *  - The GPT clock is stopped after reset: clear the MSTPCRE bit first.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>

#include "arm_internal.h"
#include "ra_clockconfig.h"
#include "ra_icu.h"
#include "ra_gpt.h"
#include "hardware/ra8m1_gpt.h"
#include "hardware/ra8m1_memorymap.h"
#include "hardware/ra8m1_mstp.h"

#ifdef CONFIG_RA_GPT_TIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GPT counts PCLKD (manual 21.1) */

#define GPT_CLOCK_HZ      RA_PCLKD_FREQUENCY

/* Every GTST flag that is R/W*1 (TCFA-TCFPU, ADTR*F, PCF).  A clear writes
 * 0 to the target flag and 1 to the others (manual 21.2.16, note 1).
 */

#define GPT_GTST_W1_MASK  0x800f00ffu

/* Clock prescaler table: divider and its GTCR.TPCS[3:0] encoding */

struct ra_gpt_prescaler_s
{
  uint16_t divider;
  uint32_t tpcs;
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_gpt_s
{
  const struct timer_ops_s *ops;     /* Must be first (timer_lowerhalf_s) */
  uintptr_t   base;                  /* Channel register base */
  uint32_t    mstp;                  /* MSTPCRE bit for this channel */
  int         irq;                   /* GPTn_OVF IRQ (ICU slot) */
  uint8_t     bits;                  /* 32 (GPT32) or 16 (GPT16) */
  bool        initialized;           /* Clock enabled, IRQ attached */
  bool        started;               /* Counter running */
  tccb_t      callback;              /* Upper-half callback */
  void       *arg;                   /* Callback argument */
  uint32_t    timeout_us;            /* Current timeout */
  uint32_t    divider;               /* Selected PCLKD divider */
  uint32_t    tpcs;                  /* Selected GTCR.TPCS field */
  uint32_t    period;                /* GTPR value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  ra_gpt_interrupt(int irq, void *context, void *arg);
static int  ra_gpt_start(struct timer_lowerhalf_s *lower);
static int  ra_gpt_stop(struct timer_lowerhalf_s *lower);
static int  ra_gpt_getstatus(struct timer_lowerhalf_s *lower,
                             struct timer_status_s *status);
static int  ra_gpt_settimeout(struct timer_lowerhalf_s *lower,
                              uint32_t timeout);
static void ra_gpt_setcallback(struct timer_lowerhalf_s *lower,
                               tccb_t callback, void *arg);
static int  ra_gpt_maxtimeout(struct timer_lowerhalf_s *lower,
                              uint32_t *maxtimeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct timer_ops_s g_ra_gpt_ops =
{
  .start       = ra_gpt_start,
  .stop        = ra_gpt_stop,
  .getstatus   = ra_gpt_getstatus,
  .settimeout  = ra_gpt_settimeout,
  .setcallback = ra_gpt_setcallback,
  .maxtimeout  = ra_gpt_maxtimeout,
};

/* Finest divider first: the driver picks the first one whose range fits */

static const struct ra_gpt_prescaler_s g_prescalers[] =
{
  {    1, R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_1    },
  {    2, R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_2    },
  {    4, R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_4    },
  {    8, R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_8    },
  {   16, R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_16   },
  {   32, R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_32   },
  {   64, R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_64   },
  {  256, R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_256  },
  { 1024, R_GPT_GTCR_TPCS_PCLKGPTNPCLKC_1024 },
};

#define GPT_NPRESCALERS  (nitems(g_prescalers))

/* Per-channel state, only for the channels enabled in Kconfig.  Channels
 * 0-7 are GPT32, 8-13 are GPT16.  The module-stop bit is MSTPCRE.MSTPE
 * (31 - channel): GPT0-7 -> MSTPE31-24, GPT8-13 -> MSTPE23-18.
 */

#define RA_GPT_CHANNEL(n, w)                                    \
  {                                                             \
    .ops     = &g_ra_gpt_ops,                                   \
    .base    = R_GPT##n##_BASE,                                 \
    .mstp    = (1u << (31 - (n))),                              \
    .irq     = GPT##n##_COUNTER_OVERFLOW,                       \
    .bits    = (w),                                             \
  }

#ifdef CONFIG_RA_GPT0_GPT
static struct ra_gpt_s g_gpt0 = RA_GPT_CHANNEL(0, 32);
#endif
#ifdef CONFIG_RA_GPT1_GPT
static struct ra_gpt_s g_gpt1 = RA_GPT_CHANNEL(1, 32);
#endif
#ifdef CONFIG_RA_GPT2_GPT
static struct ra_gpt_s g_gpt2 = RA_GPT_CHANNEL(2, 32);
#endif
#ifdef CONFIG_RA_GPT3_GPT
static struct ra_gpt_s g_gpt3 = RA_GPT_CHANNEL(3, 32);
#endif
#ifdef CONFIG_RA_GPT4_GPT
static struct ra_gpt_s g_gpt4 = RA_GPT_CHANNEL(4, 32);
#endif
#ifdef CONFIG_RA_GPT5_GPT
static struct ra_gpt_s g_gpt5 = RA_GPT_CHANNEL(5, 32);
#endif
#ifdef CONFIG_RA_GPT6_GPT
static struct ra_gpt_s g_gpt6 = RA_GPT_CHANNEL(6, 32);
#endif
#ifdef CONFIG_RA_GPT7_GPT
static struct ra_gpt_s g_gpt7 = RA_GPT_CHANNEL(7, 32);
#endif
#ifdef CONFIG_RA_GPT8_GPT
static struct ra_gpt_s g_gpt8 = RA_GPT_CHANNEL(8, 16);
#endif
#ifdef CONFIG_RA_GPT9_GPT
static struct ra_gpt_s g_gpt9 = RA_GPT_CHANNEL(9, 16);
#endif
#ifdef CONFIG_RA_GPT10_GPT
static struct ra_gpt_s g_gpt10 = RA_GPT_CHANNEL(10, 16);
#endif
#ifdef CONFIG_RA_GPT11_GPT
static struct ra_gpt_s g_gpt11 = RA_GPT_CHANNEL(11, 16);
#endif
#ifdef CONFIG_RA_GPT12_GPT
static struct ra_gpt_s g_gpt12 = RA_GPT_CHANNEL(12, 16);
#endif
#ifdef CONFIG_RA_GPT13_GPT
static struct ra_gpt_s g_gpt13 = RA_GPT_CHANNEL(13, 16);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t gpt_getreg(struct ra_gpt_s *priv, uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void gpt_putreg(struct ra_gpt_s *priv, uint32_t offset,
                              uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: ra_gpt_lookup
 ****************************************************************************/

static struct ra_gpt_s *ra_gpt_lookup(int channel)
{
  static struct ra_gpt_s * const channels[RA_GPT16_LAST + 1] =
  {
#ifdef CONFIG_RA_GPT0_GPT
    [0]  = &g_gpt0,
#endif
#ifdef CONFIG_RA_GPT1_GPT
    [1]  = &g_gpt1,
#endif
#ifdef CONFIG_RA_GPT2_GPT
    [2]  = &g_gpt2,
#endif
#ifdef CONFIG_RA_GPT3_GPT
    [3]  = &g_gpt3,
#endif
#ifdef CONFIG_RA_GPT4_GPT
    [4]  = &g_gpt4,
#endif
#ifdef CONFIG_RA_GPT5_GPT
    [5]  = &g_gpt5,
#endif
#ifdef CONFIG_RA_GPT6_GPT
    [6]  = &g_gpt6,
#endif
#ifdef CONFIG_RA_GPT7_GPT
    [7]  = &g_gpt7,
#endif
#ifdef CONFIG_RA_GPT8_GPT
    [8]  = &g_gpt8,
#endif
#ifdef CONFIG_RA_GPT9_GPT
    [9]  = &g_gpt9,
#endif
#ifdef CONFIG_RA_GPT10_GPT
    [10] = &g_gpt10,
#endif
#ifdef CONFIG_RA_GPT11_GPT
    [11] = &g_gpt11,
#endif
#ifdef CONFIG_RA_GPT12_GPT
    [12] = &g_gpt12,
#endif
#ifdef CONFIG_RA_GPT13_GPT
    [13] = &g_gpt13,
#endif
  };

  if (channel < 0 || channel > RA_GPT16_LAST)
    {
      return NULL;
    }

  return channels[channel];
}

/****************************************************************************
 * Name: ra_gpt_maxcount
 *
 * Description:
 *   Largest GTPR value: 0xffffffff for GPT32, 0xffff for GPT16.
 ****************************************************************************/

static inline uint32_t ra_gpt_maxcount(struct ra_gpt_s *priv)
{
  return priv->bits == 32 ? 0xffffffffu : 0xffffu;
}

/****************************************************************************
 * Name: ra_gpt_calc
 *
 * Description:
 *   Pick the finest PCLKD divider whose counter range holds timeout_us and
 *   compute GTPR.  Saw-wave period is GTPR + 1 counts (manual 21.2.20).
 *
 ****************************************************************************/

static int ra_gpt_calc(struct ra_gpt_s *priv, uint32_t timeout_us)
{
  uint64_t ticks;
  size_t i;

  if (timeout_us == 0)
    {
      return -EINVAL;
    }

  for (i = 0; i < GPT_NPRESCALERS; i++)
    {
      ticks = ((uint64_t)timeout_us * GPT_CLOCK_HZ /
               g_prescalers[i].divider) / 1000000ull;

      if (ticks == 0)
        {
          ticks = 1;
        }

      if (ticks - 1 <= ra_gpt_maxcount(priv))
        {
          priv->divider    = g_prescalers[i].divider;
          priv->tpcs       = g_prescalers[i].tpcs;
          priv->period     = (uint32_t)(ticks - 1);
          priv->timeout_us = timeout_us;
          return OK;
        }
    }

  return -ERANGE;
}

/****************************************************************************
 * Name: ra_gpt_hwstop / ra_gpt_hwstart
 ****************************************************************************/

static void ra_gpt_hwstop(struct ra_gpt_s *priv)
{
  uint32_t regval;

  regval = gpt_getreg(priv, R_GPT_GTCR_OFFSET);
  gpt_putreg(priv, R_GPT_GTCR_OFFSET, regval & ~R_GPT_GTCR_CST);

  /* CST is synchronised to the count clock: wait until it reads back 0
   * before touching MD/TPCS/GTCNT.  TODO: bound this loop.
   */

  while ((gpt_getreg(priv, R_GPT_GTCR_OFFSET) & R_GPT_GTCR_CST) != 0);
}

static void ra_gpt_hwstart(struct ra_gpt_s *priv)
{
  uint32_t regval;

  regval = gpt_getreg(priv, R_GPT_GTCR_OFFSET);
  gpt_putreg(priv, R_GPT_GTCR_OFFSET, regval | R_GPT_GTCR_CST);
}

/****************************************************************************
 * Name: ra_gpt_configure
 *
 * Description:
 *   Program mode, clock, direction, period and counter (counter stopped).
 *   Follows Table 21.5 "periodic count operation in up-counting".
 *
 ****************************************************************************/

static void ra_gpt_configure(struct ra_gpt_s *priv)
{
  /* 1 + 3: saw-wave PWM mode (MD = 0) and prescaler */

  gpt_putreg(priv, R_GPT_GTCR_OFFSET,
             R_GPT_GTCR_MD_V000 | priv->tpcs);

  /* 2: count direction up: force UD=1 first (UDF=1), then release UDF */

  gpt_putreg(priv, R_GPT_GTUDDTYC_OFFSET,
             R_GPT_GTUDDTYC_UD | R_GPT_GTUDDTYC_UDF);
  gpt_putreg(priv, R_GPT_GTUDDTYC_OFFSET, R_GPT_GTUDDTYC_UD);

  /* Enable GTPBR -> GTPR buffering (manual 21.2.16): a later
   * same-prescaler ra_gpt_settimeout() can then reload the period
   * through GTPBR alone (see there) instead of stopping the counter.
   */

  gpt_putreg(priv, R_GPT_GTBER_OFFSET, R_GPT_GTBER_PR_V01);

  /* 4 + 5: cycle and initial counter (GTCNT must be <= GTPR).  GTPBR
   * must be kept in sync with GTPR here too: with GTBER.PR enabled
   * above, hardware auto-transfers whatever is in GTPBR into GTPR at
   * every trough, including the very first one after this start/
   * reconfigure -- an unwritten (power-on-reset garbage) GTPBR would
   * silently corrupt GTPR at the end of the first cycle, well before
   * any ra_gpt_settimeout() call ever touches GTPBR itself.
   */

  gpt_putreg(priv, R_GPT_GTPR_OFFSET, priv->period);
  gpt_putreg(priv, R_GPT_GTPBR_OFFSET, priv->period);
  gpt_putreg(priv, R_GPT_GTCNT_OFFSET, 0);

  /* Drop a stale overflow flag: write 0 to it, 1 to the other R/W*1 bits */

  gpt_putreg(priv, R_GPT_GTST_OFFSET, GPT_GTST_W1_MASK & ~R_GPT_GTST_TCFPO);
}

/****************************************************************************
 * Name: ra_gpt_interrupt
 *
 * Description:
 *   GPTn_OVF handler.  Same contract as the other timer lower halves:
 *   the callback returns true to keep the timer running (optionally
 *   updating the next interval) and false to stop it.
 *
 ****************************************************************************/

static int ra_gpt_interrupt(int irq, void *context, void *arg)
{
  struct ra_gpt_s *priv = arg;
  uint32_t next_us = 0;

  /* Acknowledge in the ICU (IELSR.IR) and in the GPT (GTST.TCFPO) */

  ra_clear_ir(irq);
  gpt_putreg(priv, R_GPT_GTST_OFFSET, GPT_GTST_W1_MASK & ~R_GPT_GTST_TCFPO);

  if (priv->callback != NULL)
    {
      if (priv->callback(&next_us, priv->arg))
        {
          if (next_us > 0 && next_us != priv->timeout_us)
            {
              uint32_t old_tpcs = priv->tpcs;

              if (ra_gpt_calc(priv, next_us) == OK)
                {
                  if (priv->tpcs == old_tpcs)
                    {
                      /* Same prescaler: reload through GTPBR instead
                       * of stopping the counter (see
                       * ra_gpt_settimeout()).
                       */

                      gpt_putreg(priv, R_GPT_GTPBR_OFFSET, priv->period);
                    }
                  else
                    {
                      ra_gpt_hwstop(priv);
                      ra_gpt_configure(priv);
                      ra_gpt_hwstart(priv);
                    }
                }
            }
        }
      else
        {
          ra_gpt_stop((struct timer_lowerhalf_s *)priv);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ra_gpt_start / ra_gpt_stop
 ****************************************************************************/

static int ra_gpt_start(struct timer_lowerhalf_s *lower)
{
  struct ra_gpt_s *priv = (struct ra_gpt_s *)lower;
  irqstate_t flags;

  if (priv->started)
    {
      return -EBUSY;
    }

  if (priv->timeout_us == 0)
    {
      return -EINVAL;             /* settimeout() first */
    }

  flags = enter_critical_section();

  ra_gpt_configure(priv);
  up_enable_irq(priv->irq);
  ra_gpt_hwstart(priv);
  priv->started = true;

  leave_critical_section(flags);
  return OK;
}

static int ra_gpt_stop(struct timer_lowerhalf_s *lower)
{
  struct ra_gpt_s *priv = (struct ra_gpt_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  ra_gpt_hwstop(priv);
  up_disable_irq(priv->irq);
  priv->started = false;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ra_gpt_getstatus
 ****************************************************************************/

static int ra_gpt_getstatus(struct timer_lowerhalf_s *lower,
                            struct timer_status_s *status)
{
  struct ra_gpt_s *priv = (struct ra_gpt_s *)lower;
  uint64_t left_ticks;
  irqstate_t flags;

  flags = enter_critical_section();

  status->flags = 0;
  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback != NULL)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  status->timeout = priv->timeout_us;

  if (priv->started)
    {
      left_ticks = gpt_getreg(priv, R_GPT_GTPR_OFFSET) -
                   gpt_getreg(priv, R_GPT_GTCNT_OFFSET);
      status->timeleft = (uint32_t)(left_ticks * priv->divider *
                                    1000000ull / GPT_CLOCK_HZ);
    }
  else
    {
      /* The hardware is only programmed by start(): report the full
       * timeout instead of the stale (reset or stopped) register values.
       */

      status->timeleft = priv->timeout_us;
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ra_gpt_settimeout
 ****************************************************************************/

static int ra_gpt_settimeout(struct timer_lowerhalf_s *lower,
                             uint32_t timeout)
{
  struct ra_gpt_s *priv = (struct ra_gpt_s *)lower;
  irqstate_t flags;
  uint32_t old_tpcs;
  int ret;

  flags = enter_critical_section();

  old_tpcs = priv->tpcs;
  ret = ra_gpt_calc(priv, timeout);
  if (ret == OK && priv->started)
    {
      if (priv->tpcs == old_tpcs)
        {
          /* Same prescaler: GTBER.PR (enabled in ra_gpt_configure())
           * buffers GTPR through GTPBR, so hardware swaps it in at the
           * next trough (manual 21.2.16/Table 21.5) -- the counter
           * never stops and no edge is lost.
           */

          gpt_putreg(priv, R_GPT_GTPBR_OFFSET, priv->period);
        }
      else
        {
          /* Prescaler changed: unlike GTPR, TPCS is not
           * double-buffered, so this still needs the counter stopped
           * (MD/TPCS/GTCNT may only change while it is, manual 21.10.4).
           */

          ra_gpt_hwstop(priv);
          ra_gpt_configure(priv);
          ra_gpt_hwstart(priv);
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: ra_gpt_setcallback
 ****************************************************************************/

static void ra_gpt_setcallback(struct timer_lowerhalf_s *lower,
                               tccb_t callback, void *arg)
{
  struct ra_gpt_s *priv = (struct ra_gpt_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  priv->callback = callback;
  priv->arg      = arg;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ra_gpt_maxtimeout
 *
 * Description:
 *   Longest timeout at the slowest divider, clamped to what a uint32_t
 *   microsecond count can hold.
 *
 ****************************************************************************/

static int ra_gpt_maxtimeout(struct timer_lowerhalf_s *lower,
                             uint32_t *maxtimeout)
{
  struct ra_gpt_s *priv = (struct ra_gpt_s *)lower;
  uint64_t max_us;

  max_us = ((uint64_t)ra_gpt_maxcount(priv) + 1) *
           g_prescalers[GPT_NPRESCALERS - 1].divider * 1000000ull /
           GPT_CLOCK_HZ;

  *maxtimeout = max_us > UINT32_MAX ? UINT32_MAX : (uint32_t)max_us;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_gpt_timer_initialize
 ****************************************************************************/

int ra_gpt_timer_initialize(const char *devpath, int channel)
{
  struct ra_gpt_s *priv = ra_gpt_lookup(channel);
  int ret;

  if (priv == NULL)
    {
      return -ENODEV;             /* Channel not enabled in Kconfig */
    }

  if (!priv->initialized)
    {
      /* Release module stop (GPT is stopped after reset, manual 21.10.1)
       * and dummy-read to make sure the write has landed.
       */

      modifyreg32(R_MSTP_MSTPCRE, priv->mstp, 0);
      getreg32(R_MSTP_MSTPCRE);

      /* Registers are write-enabled after reset (GTWP.WP = 0).  If a
       * bootloader locked them, unlock with:
       *   putreg32(R_GPT_GTWP_PRKEY_V0XA5, base + R_GPT_GTWP_OFFSET);
       */

      ret = irq_attach(priv->irq, ra_gpt_interrupt, priv);
      if (ret < 0)
        {
          return ret;
        }

      priv->initialized = true;
    }

  return timer_register(devpath, (struct timer_lowerhalf_s *)priv) != NULL ?
         OK : -EIO;
}

#endif /* CONFIG_RA_GPT_TIMER */
