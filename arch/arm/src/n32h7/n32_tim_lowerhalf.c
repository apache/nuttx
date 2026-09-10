/****************************************************************************
 * arch/arm/src/n32h7/n32_tim_lowerhalf.c
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
#include <nuttx/timers/timer.h>

#include <arch/board/board.h>

#include "n32_tim.h"

#if defined(CONFIG_TIMER) && \
    (defined(CONFIG_N32H7_ATIM1)  || defined(CONFIG_N32H7_ATIM2)  || \
     defined(CONFIG_N32H7_ATIM3)  || defined(CONFIG_N32H7_ATIM4)  || \
     defined(CONFIG_N32H7_GTIMA1) || defined(CONFIG_N32H7_GTIMA2) || \
     defined(CONFIG_N32H7_GTIMA3) || defined(CONFIG_N32H7_GTIMA4) || \
     defined(CONFIG_N32H7_GTIMA5) || defined(CONFIG_N32H7_GTIMA6) || \
     defined(CONFIG_N32H7_GTIMA7) || defined(CONFIG_N32H7_GTIMB1) || \
     defined(CONFIG_N32H7_GTIMB2) || defined(CONFIG_N32H7_GTIMB3) || \
     defined(CONFIG_N32H7_BTIM1)  || defined(CONFIG_N32H7_BTIM2)  || \
     defined(CONFIG_N32H7_BTIM3)  || defined(CONFIG_N32H7_BTIM4))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt source for timer upper half (update interrupt) */

#define N32_TIM_UIE    N32_ATIM_DINTEN_UIEN
#define N32_TIM_UIF    N32_ATIM_STS_UDITF

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.
 */

struct n32_lowerhalf_s
{
  const struct timer_ops_s *ops;        /* Lower half operations */
  struct n32_tim_dev_s    *tim;         /* N32 timer driver */
  tccb_t                   callback;    /* Current user interrupt callback */
  void                    *arg;         /* Argument passed to upper half callback */
  bool                     started;     /* True: Timer has been started */
  const uint8_t            resolution;  /* Number of bits in the timer (16 or 32 bits) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int n32_timer_handler(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int n32_start(struct timer_lowerhalf_s *lower);
static int n32_stop(struct timer_lowerhalf_s *lower);
static int n32_getstatus(struct timer_lowerhalf_s *lower,
                         struct timer_status_s *status);
static int n32_settimeout(struct timer_lowerhalf_s *lower,
                          uint32_t timeout);
static void n32_setcallback(struct timer_lowerhalf_s *lower,
                            tccb_t callback, void *arg);
static int n32_maxtimeout(struct timer_lowerhalf_s *lower,
                          uint32_t *maxtimeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = n32_start,
  .stop        = n32_stop,
  .getstatus   = n32_getstatus,
  .settimeout  = n32_settimeout,
  .setcallback = n32_setcallback,
  .ioctl       = NULL,
  .maxtimeout  = n32_maxtimeout,
  .tick_getstatus  = NULL,
  .tick_settimeout = NULL,
  .tick_maxtimeout = NULL,
};

#ifdef CONFIG_N32H7_ATIM1
static struct n32_lowerhalf_s g_tim1_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,  /* ATIM1 is 16-bit */
};
#endif

#ifdef CONFIG_N32H7_ATIM2
static struct n32_lowerhalf_s g_tim2_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,
};
#endif

#ifdef CONFIG_N32H7_GTIMA1
static struct n32_lowerhalf_s g_gtima1_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,
};
#endif

#ifdef CONFIG_N32H7_GTIMA2
static struct n32_lowerhalf_s g_gtima2_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,
};
#endif

#ifdef CONFIG_N32H7_GTIMA3
static struct n32_lowerhalf_s g_gtima3_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,
};
#endif

#ifdef CONFIG_N32H7_ATIM3
static struct n32_lowerhalf_s g_tim3_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,
};
#endif

#ifdef CONFIG_N32H7_ATIM4
static struct n32_lowerhalf_s g_tim4_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,
};
#endif

#ifdef CONFIG_N32H7_GTIMA4
static struct n32_lowerhalf_s g_gtima4_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,
};
#endif

#ifdef CONFIG_N32H7_GTIMA5
static struct n32_lowerhalf_s g_gtima5_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,
};
#endif

#ifdef CONFIG_N32H7_GTIMA6
static struct n32_lowerhalf_s g_gtima6_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,
};
#endif

#ifdef CONFIG_N32H7_GTIMA7
static struct n32_lowerhalf_s g_gtima7_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,
};
#endif

#ifdef CONFIG_N32H7_GTIMB1
static struct n32_lowerhalf_s g_gtimb1_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,
};
#endif

#ifdef CONFIG_N32H7_GTIMB2
static struct n32_lowerhalf_s g_gtimb2_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,
};
#endif

#ifdef CONFIG_N32H7_GTIMB3
static struct n32_lowerhalf_s g_gtimb3_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 16,
};
#endif

#ifdef CONFIG_N32H7_BTIM1
static struct n32_lowerhalf_s g_btim1_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 32,
};
#endif

#ifdef CONFIG_N32H7_BTIM2
static struct n32_lowerhalf_s g_btim2_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 32,
};
#endif

#ifdef CONFIG_N32H7_BTIM3
static struct n32_lowerhalf_s g_btim3_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 32,
};
#endif

#ifdef CONFIG_N32H7_BTIM4
static struct n32_lowerhalf_s g_btim4_lowerhalf =
{
  .ops         = &g_timer_ops,
  .resolution  = 32,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_timer_handler
 *
 * Description:
 *   Timer interrupt handler
 *
 ****************************************************************************/

static int n32_timer_handler(int irq, void *context, void *arg)
{
  struct n32_lowerhalf_s *lower = (struct n32_lowerhalf_s *)arg;
  uint32_t next_interval_us = 0;

  N32_TIM_ACKINT(lower->tim, N32_TIM_UIF);

  if (lower->callback(&next_interval_us, lower->arg))
    {
      if (next_interval_us > 0)
        {
          N32_TIM_SETPERIOD(lower->tim, next_interval_us);
        }
    }
  else
    {
      n32_stop((struct timer_lowerhalf_s *)lower);
    }

  return OK;
}

/****************************************************************************
 * Name: n32_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout.
 *
 ****************************************************************************/

static int n32_start(struct timer_lowerhalf_s *lower)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;

  if (!priv->started)
    {
      N32_TIM_SETMODE(priv->tim, N32_TIM_MODE_UP);

      if (priv->callback != NULL)
        {
          N32_TIM_SETISR(priv->tim, n32_timer_handler, priv,
                         N32_TIM_ISR_UPDATE);
          N32_TIM_ENABLEINT(priv->tim, N32_TIM_UIE);
        }

      priv->started = true;
      return OK;
    }

  return -EBUSY;
}

/****************************************************************************
 * Name: n32_stop
 *
 * Description:
 *   Stop the timer
 *
 ****************************************************************************/

static int n32_stop(struct timer_lowerhalf_s *lower)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;

  if (priv->started)
    {
      N32_TIM_SETMODE(priv->tim, N32_TIM_MODE_DISABLED);
      N32_TIM_DISABLEINT(priv->tim, N32_TIM_UIE);
      N32_TIM_SETISR(priv->tim, NULL, NULL, N32_TIM_ISR_UPDATE);
      priv->started = false;
      return OK;
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: n32_getstatus
 *
 * Description:
 *   Get the current timer status.
 *
 ****************************************************************************/

static int n32_getstatus(struct timer_lowerhalf_s *lower,
                         struct timer_status_s *status)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;

  if (status == NULL)
    {
      return -EINVAL;
    }

  memset(status, 0, sizeof(*status));

  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback != NULL)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  status->timeout  = N32_TIM_GETPERIOD(priv->tim);
  status->timeleft = status->timeout - N32_TIM_GETCOUNTER(priv->tim);

  return OK;
}

/****************************************************************************
 * Name: n32_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 ****************************************************************************/

static int n32_settimeout(struct timer_lowerhalf_s *lower,
                          uint32_t timeout)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;
  uint64_t maxtimeout;

  if (priv->started)
    {
      return -EPERM;
    }

  maxtimeout = (1ULL << priv->resolution) - 1;

  /* If timeout exceeds the maximum counter value, scale the clock down */

  if (timeout > maxtimeout)
    {
      uint64_t freq = (maxtimeout * 1000000ULL) / timeout;

      N32_TIM_SETCLOCK(priv->tim, (uint32_t)freq);
      N32_TIM_SETPERIOD(priv->tim, (uint32_t)maxtimeout);
    }
  else
    {
      N32_TIM_SETCLOCK(priv->tim, 1000000);
      N32_TIM_SETPERIOD(priv->tim, timeout);
    }

  return OK;
}

/****************************************************************************
 * Name: n32_setcallback
 *
 * Description:
 *   Call this user provided timeout callback.
 *
 ****************************************************************************/

static void n32_setcallback(struct timer_lowerhalf_s *lower,
                            tccb_t callback, void *arg)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Save the new callback */

  priv->callback = callback;
  priv->arg      = arg;

  if (callback != NULL && priv->started)
    {
      N32_TIM_SETISR(priv->tim, n32_timer_handler, priv, N32_TIM_ISR_UPDATE);
      N32_TIM_ENABLEINT(priv->tim, N32_TIM_UIE);
    }
  else
    {
      N32_TIM_DISABLEINT(priv->tim, N32_TIM_UIE);
      N32_TIM_SETISR(priv->tim, NULL, NULL, N32_TIM_ISR_UPDATE);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: n32_maxtimeout
 *
 * Description:
 *   Return the maximum supported timeout value in microseconds.
 *
 ****************************************************************************/

static int n32_maxtimeout(struct timer_lowerhalf_s *lower,
                          uint32_t *maxtimeout)
{
  struct n32_lowerhalf_s *priv = (struct n32_lowerhalf_s *)lower;

  if (maxtimeout == NULL)
    {
      return -EINVAL;
    }

  *maxtimeout = (1ULL << priv->resolution) - 1;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 ****************************************************************************/

int n32_timer_initialize(const char *devpath, int timer)
{
  struct n32_lowerhalf_s *lower;

  switch (timer)
    {
#ifdef CONFIG_N32H7_ATIM1
      case 1:
        lower = &g_tim1_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_ATIM2
      case 2:
        lower = &g_tim2_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA1
      case 3:
        lower = &g_gtima1_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA2
      case 4:
        lower = &g_gtima2_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA3
      case 5:
        lower = &g_gtima3_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_ATIM3
      case 6:
        lower = &g_tim3_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_ATIM4
      case 7:
        lower = &g_tim4_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA4
      case 8:
        lower = &g_gtima4_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA5
      case 9:
        lower = &g_gtima5_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA6
      case 10:
        lower = &g_gtima6_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA7
      case 11:
        lower = &g_gtima7_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_GTIMB1
      case 12:
        lower = &g_gtimb1_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_GTIMB2
      case 13:
        lower = &g_gtimb2_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_GTIMB3
      case 14:
        lower = &g_gtimb3_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_BTIM1
      case 15:
        lower = &g_btim1_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_BTIM2
      case 16:
        lower = &g_btim2_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_BTIM3
      case 17:
        lower = &g_btim3_lowerhalf;
        break;
#endif
#ifdef CONFIG_N32H7_BTIM4
      case 18:
        lower = &g_btim4_lowerhalf;
        break;
#endif
      default:
        return -ENODEV;
    }

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;
  lower->tim      = n32_tim_init(timer);

  if (lower->tim == NULL)
    {
      return -EINVAL;
    }

  /* Register the timer driver as /dev/timerX */

  void *drvr = timer_register(devpath,
                              (struct timer_lowerhalf_s *)lower);

  if (drvr == NULL)
    {
      return -EEXIST;
    }

  return OK;
}

#endif /* CONFIG_TIMER && ... */
