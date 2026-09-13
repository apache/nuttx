/****************************************************************************
 * arch/arm/src/n32h7/n32_capture_lowerhalf.c
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
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <nuttx/irq.h>
#include <nuttx/timers/capture.h>
#include <arch/board/board.h>
#include "n32_capture.h"

#ifdef CONFIG_CAPTURE

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct n32_lowerhalf_s
{
  const struct cap_ops_s *ops;
  struct n32_cap_dev_s *cap;
  bool started;
  uint8_t resolution;
  uint8_t channel;
  uint32_t clock;
  uint8_t duty;
  uint32_t freq;
  uint32_t edges;
  uint32_t overflow;
  uint32_t tmp_duty;

#ifdef CONFIG_CAPTURE_NOTIFY
  capture_notify_t cb;          /* Edge notification callback */
  void    *priv;                /* Private data of the callback */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int n32_cap_handler(int irq, void *context, void *arg);
static int n32_start(struct cap_lowerhalf_s *lower);
static int n32_stop(struct cap_lowerhalf_s *lower);
static int n32_getduty(struct cap_lowerhalf_s *lower, uint8_t *duty);
static int n32_getfreq(struct cap_lowerhalf_s *lower, uint32_t *freq);
static int n32_getedges(struct cap_lowerhalf_s *lower, uint32_t *edges);
static int n32_ioctl(struct cap_lowerhalf_s *lower, int cmd,
                     unsigned long arg);
#ifdef CONFIG_CAPTURE_NOTIFY
static int n32_bind(struct cap_lowerhalf_s *lower, enum cap_type_e type,
                    capture_notify_t cb, void *priv);
static int n32_unbind(struct cap_lowerhalf_s *lower);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct cap_ops_s g_cap_ops =
{
  .start    = n32_start,
  .stop     = n32_stop,
  .getduty  = n32_getduty,
  .getfreq  = n32_getfreq,
  .getedges = n32_getedges,
  .ioctl    = n32_ioctl,
#ifdef CONFIG_CAPTURE_NOTIFY
  .bind     = n32_bind,
  .unbind   = n32_unbind,
#endif
};

#ifdef CONFIG_N32H7_ATIM1_CAP
static struct n32_lowerhalf_s g_cap1_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_ATIM1_CHANNEL,
  .clock       = CONFIG_N32H7_ATIM1_CLOCK,
};
#endif

#ifdef CONFIG_N32H7_ATIM2_CAP
static struct n32_lowerhalf_s g_cap2_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_ATIM2_CHANNEL,
  .clock       = CONFIG_N32H7_ATIM2_CLOCK,
};
#endif

#ifdef CONFIG_N32H7_GTIMA1_CAP
static struct n32_lowerhalf_s g_cap3_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_GTIMA1_CHANNEL,
  .clock       = CONFIG_N32H7_GTIMA1_CLOCK,
};
#endif

#ifdef CONFIG_N32H7_GTIMA2_CAP
static struct n32_lowerhalf_s g_cap4_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_GTIMA2_CHANNEL,
  .clock       = CONFIG_N32H7_GTIMA2_CLOCK,
};
#endif

#ifdef CONFIG_N32H7_GTIMA3_CAP
static struct n32_lowerhalf_s g_cap5_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_GTIMA3_CHANNEL,
  .clock       = CONFIG_N32H7_GTIMA3_CLOCK,
};
#endif

#ifdef CONFIG_N32H7_ATIM3_CAP
static struct n32_lowerhalf_s g_cap6_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_ATIM3_CHANNEL,
  .clock       = CONFIG_N32H7_ATIM3_CLOCK,
};
#endif

#ifdef CONFIG_N32H7_ATIM4_CAP
static struct n32_lowerhalf_s g_cap7_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_ATIM4_CHANNEL,
  .clock       = CONFIG_N32H7_ATIM4_CLOCK,
};
#endif

#ifdef CONFIG_N32H7_GTIMA4_CAP
static struct n32_lowerhalf_s g_cap8_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_GTIMA4_CHANNEL,
  .clock       = CONFIG_N32H7_GTIMA4_CLOCK,
};
#endif

#ifdef CONFIG_N32H7_GTIMA5_CAP
static struct n32_lowerhalf_s g_cap9_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_GTIMA5_CHANNEL,
  .clock       = CONFIG_N32H7_GTIMA5_CLOCK,
};
#endif

#ifdef CONFIG_N32H7_GTIMA6_CAP
static struct n32_lowerhalf_s g_cap10_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_GTIMA6_CHANNEL,
  .clock       = CONFIG_N32H7_GTIMA6_CLOCK,
};
#endif

#ifdef CONFIG_N32H7_GTIMA7_CAP
static struct n32_lowerhalf_s g_cap11_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_GTIMA7_CHANNEL,
  .clock       = CONFIG_N32H7_GTIMA7_CLOCK,
};
#endif

#ifdef CONFIG_N32H7_GTIMB1_CAP
static struct n32_lowerhalf_s g_cap12_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_GTIMB1_CHANNEL,
  .clock       = CONFIG_N32H7_GTIMB1_CLOCK,
};
#endif

#ifdef CONFIG_N32H7_GTIMB2_CAP
static struct n32_lowerhalf_s g_cap13_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_GTIMB2_CHANNEL,
  .clock       = CONFIG_N32H7_GTIMB2_CLOCK,
};
#endif

#ifdef CONFIG_N32H7_GTIMB3_CAP
static struct n32_lowerhalf_s g_cap14_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = 16,
  .channel     = CONFIG_N32H7_GTIMB3_CHANNEL,
  .clock       = CONFIG_N32H7_GTIMB3_CLOCK,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int n32_cap_handler(int irq, void *context, void *arg)
{
  struct n32_lowerhalf_s *lower = (struct n32_lowerhalf_s *)arg;
  uint8_t ch = 0x3 & lower->channel;
  uint32_t period = 0;
  int flags = 0;

  flags = (int)N32_CAP_GETFLAGS(lower->cap);
  N32_CAP_ACKFLAGS(lower->cap, flags);

  if (flags & (N32_CAP_FLAG_IRQ_CH_1 << (ch - 1)))
    {
      period = N32_CAP_GETCAPTURE(lower->cap, ch) +
               lower->overflow * (1 << lower->resolution) + 1;
      if (period != 0)
        {
          lower->duty = (100 * lower->tmp_duty) / period;
          lower->freq = lower->clock / period;
          lower->edges++;
          lower->overflow = 0;
          lower->tmp_duty = 0;
#ifdef CONFIG_CAPTURE_NOTIFY
          if (lower->cb != NULL)
            {
              lower->cb((struct cap_lowerhalf_s *)lower, lower->priv);
            }
#endif
        }
    }
  else if (flags & (N32_CAP_FLAG_IRQ_CH_2 << (2 - ch)))
    {
      lower->tmp_duty = N32_CAP_GETCAPTURE(lower->cap, 0x3 & (~ch))
                       + lower->overflow * (1 << lower->resolution) + 1;
    }
  else if (flags & N32_CAP_FLAG_IRQ_COUNTER)
    {
      lower->overflow++;
    }

  return OK;
}

static int n32_start(struct cap_lowerhalf_s *lower)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;
  int flags = 0;
  uint32_t maxtimeout = (1 << priv->resolution) - 1;

  if (priv->started)
    {
      return -EBUSY;
    }

  switch (priv->channel)
    {
      case 1:
        N32_CAP_SETSMC(priv->cap, (n32_cap_smc_cfg_t)(N32_ATIM_SMCTRL_RESET |
                      N32_ATIM_SMCTRL_TI1FP1 |
                      N32_ATIM_SMCTRL_MSMD));
        N32_CAP_SETCLOCK(priv->cap, priv->clock, maxtimeout);
        N32_CAP_SETCHANNEL(priv->cap, 1, (n32_cap_ch_cfg_t)
                        (N32_CAP_EDGE_RISING | N32_ATIM_CCMOD1_CC1IN_TI1));
        N32_CAP_SETCHANNEL(priv->cap, 2, (n32_cap_ch_cfg_t)
                        (N32_CAP_EDGE_FALLING | N32_ATIM_CCMOD1_CC1IN_TI2));
        flags = (int)N32_CAP_GETFLAGS(priv->cap);
        N32_CAP_ACKFLAGS(priv->cap, flags);
        N32_CAP_SETISR(priv->cap, n32_cap_handler, priv);
        N32_CAP_ENABLEINT(priv->cap, (n32_cap_flags_t)
                          (N32_CAP_FLAG_IRQ_CH_1 | N32_CAP_FLAG_IRQ_CH_2 |
                          N32_CAP_FLAG_IRQ_COUNTER), true);
        priv->started = true;
        break;
      case 2:
        N32_CAP_SETSMC(priv->cap, (n32_cap_smc_cfg_t)(N32_ATIM_SMCTRL_RESET |
                      N32_ATIM_SMCTRL_TI2FP2 |
                      N32_ATIM_SMCTRL_MSMD));
        N32_CAP_SETCLOCK(priv->cap, priv->clock, maxtimeout);
        N32_CAP_SETCHANNEL(priv->cap, 2, (n32_cap_ch_cfg_t)
                        (N32_CAP_EDGE_RISING | N32_ATIM_CCMOD1_CC1IN_TI1));
        N32_CAP_SETCHANNEL(priv->cap, 1, (n32_cap_ch_cfg_t)
                        (N32_CAP_EDGE_FALLING | N32_ATIM_CCMOD1_CC1IN_TI2));
        flags = (int)N32_CAP_GETFLAGS(priv->cap);
        N32_CAP_ACKFLAGS(priv->cap, flags);
        N32_CAP_SETISR(priv->cap, n32_cap_handler, priv);
        N32_CAP_ENABLEINT(priv->cap, (n32_cap_flags_t)
                          (N32_CAP_FLAG_IRQ_CH_2 | N32_CAP_FLAG_IRQ_CH_1 |
                          N32_CAP_FLAG_IRQ_COUNTER), true);
        priv->started = true;
        break;
      default:
        return -EINVAL;
    }

  /* The actual frequency is priv->clock (as set by SETCLOCK) */

  return OK;
}

static int n32_stop(struct cap_lowerhalf_s *lower)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;
  int flags = 0;

  if (!priv->started)
    {
      return -ENODEV;
    }

  N32_CAP_SETSMC(priv->cap, 0);
  N32_CAP_SETCLOCK(priv->cap, 0, 0);
  N32_CAP_SETCHANNEL(priv->cap, 1, N32_CAP_EDGE_DISABLED);
  N32_CAP_SETCHANNEL(priv->cap, 2, N32_CAP_EDGE_DISABLED);
  N32_CAP_ENABLEINT(priv->cap, N32_CAP_FLAG_IRQ_CH_1 |
                    N32_CAP_FLAG_IRQ_CH_2 |
                    N32_CAP_FLAG_IRQ_COUNTER, false);

  flags = (int)N32_CAP_GETFLAGS(priv->cap);
  N32_CAP_ACKFLAGS(priv->cap, flags);
  N32_CAP_SETISR(priv->cap, NULL, NULL);
  priv->started = false;
  priv->duty = 0;
  priv->freq = 0;
  priv->edges = 0;
  priv->overflow = 0;
  priv->tmp_duty = 0;

  return OK;
}

static int n32_getduty(struct cap_lowerhalf_s *lower, uint8_t *duty)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;
  irqstate_t flags = enter_critical_section();

  *duty = priv->duty;

  leave_critical_section(flags);
  return OK;
}

static int n32_getfreq(struct cap_lowerhalf_s *lower, uint32_t *freq)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;
  irqstate_t flags = enter_critical_section();

  *freq = priv->freq;

  leave_critical_section(flags);
  return OK;
}

static int n32_getedges(struct cap_lowerhalf_s *lower, uint32_t *edges)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;
  irqstate_t flags = enter_critical_section();

  *edges = priv->edges;

  leave_critical_section(flags);
  return OK;
}

static int n32_ioctl(struct cap_lowerhalf_s *lower, int cmd,
                     unsigned long arg)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;
  irqstate_t flags = enter_critical_section();

  switch (cmd)
    {
      case CAPIOC_CLR_CNT:
        priv->edges = 0;
        priv->overflow = 0;
        break;
      case CAPIOC_PULSES:
        {
          int *count = (int *)((uintptr_t)arg);

          DEBUGASSERT(count != NULL);
          *count = (int)priv->edges;
        }

        break;
      default:
        return -EINVAL;
    }

  leave_critical_section(flags);
  return OK;
}

#ifdef CONFIG_CAPTURE_NOTIFY
static int n32_bind(struct cap_lowerhalf_s *lower, enum cap_type_e type,
                    capture_notify_t cb, void *priv)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;
  irqstate_t flags;

  /* Only the rising edge of channel 0 is reported: it is the edge that
   * completes the measurement of a period.
   */

  if (type != CAP_TYPE_RISING)
    {
      return -ENOSYS;
    }

  flags     = enter_critical_section();
  priv->cb   = cb;
  priv->priv = priv;
  leave_critical_section(flags);

  return OK;
}

static int n32_unbind(struct cap_lowerhalf_s *lower)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;
  irqstate_t flags;

  flags     = enter_critical_section();
  priv->cb   = NULL;
  priv->priv = NULL;
  leave_critical_section(flags);

  return OK;
}
#endif /* CONFIG_CAPTURE_NOTIFY */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_cap_initialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level capture driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the N32 MCU and MCU family but is somewhere in
 *     the range of {1,..,5 8,...,14}.
 *
 * Returned Value:
 *   On success, a pointer to the N32 lower half capture driver returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct cap_lowerhalf_s *n32_cap_initialize(int timer)
{
  struct n32_lowerhalf_s *lower = NULL;

  switch (timer)
    {
  #ifdef CONFIG_N32H7_ATIM1_CAP
      case 1:
        lower = &g_cap1_lowerhalf;
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM2_CAP
      case 2:
        lower = &g_cap2_lowerhalf;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA1_CAP
      case 3:
        lower = &g_cap3_lowerhalf;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA2_CAP
      case 4:
        lower = &g_cap4_lowerhalf;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA3_CAP
      case 5:
        lower = &g_cap5_lowerhalf;
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM3_CAP
      case 6:
        lower = &g_cap6_lowerhalf;
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM4_CAP
      case 7:
        lower = &g_cap7_lowerhalf;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA4_CAP
      case 8:
        lower = &g_cap8_lowerhalf;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA5_CAP
      case 9:
        lower = &g_cap9_lowerhalf;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA6_CAP
      case 10:
        lower = &g_cap10_lowerhalf;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA7_CAP
      case 11:
        lower = &g_cap11_lowerhalf;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB1_CAP
      case 12:
        lower = &g_cap12_lowerhalf;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB2_CAP
      case 13:
        lower = &g_cap13_lowerhalf;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB3_CAP
      case 14:
        lower = &g_cap14_lowerhalf;
        break;
  #endif
      default:
        {
          lower = NULL;
          goto errout;
        }
    }

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->cap      = n32_cap_init(timer);

  if (lower->cap == NULL)
    {
      lower = NULL;
    }

errout:
  return (struct cap_lowerhalf_s *)lower;
}
#endif /* CONFIG_CAPTURE */
