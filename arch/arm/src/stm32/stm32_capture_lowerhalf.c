/****************************************************************************
 * arch/arm/src/stm32/stm32_capture_lowerhalf.c
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

#include "stm32_capture.h"

#if defined(CONFIG_STM32_CAP)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_TIM1_RES   16
#if defined(CONFIG_STM32_STM32L15XX) || defined(CONFIG_STM32_STM32F10XX)
#  define STM32_TIM2_RES 16
#else
#  define STM32_TIM2_RES 32
#endif
#define STM32_TIM3_RES   16
#define STM32_TIM4_RES   16
#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)
#  define STM32_TIM5_RES 16
#else
#  define STM32_TIM5_RES 32
#endif
#define STM32_TIM8_RES   16
#define STM32_TIM9_RES   16
#define STM32_TIM10_RES  16
#define STM32_TIM11_RES  16
#define STM32_TIM12_RES  16
#define STM32_TIM13_RES  16
#define STM32_TIM14_RES  16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * cap_lowerhalf_s structure.
 */

struct stm32_lowerhalf_s
{
  const struct cap_ops_s *ops;       /* Lower half operations */
  struct stm32_cap_dev_s *cap;       /* stm32 capture driver */
  bool                   started;    /* True: Timer has been started */
  const uint8_t          resolution; /* Number of bits in the timer */
  uint8_t                channel;    /* pwm input channel */
  uint32_t               clock;      /* Timer clock frequence */
  uint8_t                duty;       /* Result pwm frequence */
  uint32_t               freq;       /* Result pwm frequence */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_cap_handler(int irq, void * context, void * arg);

/* "Lower half" driver methods **********************************************/

static int stm32_start(struct cap_lowerhalf_s *lower);
static int stm32_stop(struct cap_lowerhalf_s *lower);
static int stm32_getduty(struct cap_lowerhalf_s *lower, uint8_t *duty);
static int stm32_getfreq(struct cap_lowerhalf_s *lower, uint32_t *freq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct cap_ops_s g_cap_ops =
{
  .start       = stm32_start,
  .stop        = stm32_stop,
  .getduty     = stm32_getduty,
  .getfreq     = stm32_getfreq,
};

#ifdef CONFIG_STM32_TIM1_CAP
static struct stm32_lowerhalf_s g_cap1_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = STM32_TIM1_RES,
  .channel     = CONFIG_STM32_TIM1_CHANNEL,
  .clock       = CONFIG_STM32_TIM1_CLOCK,
};
#endif

#ifdef CONFIG_STM32_TIM2_CAP
static struct stm32_lowerhalf_s g_cap2_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = STM32_TIM2_RES,
  .channel     = CONFIG_STM32_TIM2_CHANNEL,
  .clock       = CONFIG_STM32_TIM2_CLOCK,
};
#endif

#ifdef CONFIG_STM32_TIM3_CAP
static struct stm32_lowerhalf_s g_cap3_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = STM32_TIM3_RES,
  .channel     = CONFIG_STM32_TIM3_CHANNEL,
  .clock       = CONFIG_STM32_TIM3_CLOCK,
};
#endif

#ifdef CONFIG_STM32_TIM4_CAP
static struct stm32_lowerhalf_s g_cap4_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = STM32_TIM4_RES,
  .channel     = CONFIG_STM32_TIM4_CHANNEL,
  .clock       = CONFIG_STM32_TIM4_CLOCK,
};
#endif

#ifdef CONFIG_STM32_TIM5_CAP
static struct stm32_lowerhalf_s g_cap5_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = STM32_TIM5_RES,
  .channel     = CONFIG_STM32_TIM5_CHANNEL,
  .clock       = CONFIG_STM32_TIM5_CLOCK,
};
#endif

#ifdef CONFIG_STM32_TIM8_CAP
static struct stm32_lowerhalf_s g_cap8_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = STM32_TIM8_RES,
  .channel     = CONFIG_STM32_TIM8_CHANNEL,
  .clock       = CONFIG_STM32_TIM8_CLOCK,
};
#endif

#ifdef CONFIG_STM32_TIM9_CAP
static struct stm32_lowerhalf_s g_cap9_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = STM32_TIM9_RES,
  .channel     = CONFIG_STM32_TIM9_CHANNEL,
  .clock       = CONFIG_STM32_TIM9_CLOCK,
};
#endif

#ifdef CONFIG_STM32_TIM10_CAP
static struct stm32_lowerhalf_s g_cap10_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = STM32_TIM10_RES,
  .channel     = CONFIG_STM32_TIM10_CHANNEL,
  .clock       = CONFIG_STM32_TIM10_CLOCK,
};
#endif

#ifdef CONFIG_STM32_TIM11_CAP
static struct stm32_lowerhalf_s g_cap11_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = STM32_TIM11_RES,
  .channel     = CONFIG_STM32_TIM11_CHANNEL,
  .clock       = CONFIG_STM32_TIM11_CLOCK,
};
#endif

#ifdef CONFIG_STM32_TIM12_CAP
static struct stm32_lowerhalf_s g_cap12_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = STM32_TIM12_RES,
  .channel     = CONFIG_STM32_TIM12_CHANNEL,
  .clock       = CONFIG_STM32_TIM12_CLOCK,
};
#endif

#ifdef CONFIG_STM32_TIM13_CAP
static struct stm32_lowerhalf_s g_cap13_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = STM32_TIM13_RES,
  .channel     = CONFIG_STM32_TIM13_CHANNEL,
  .clock       = CONFIG_STM32_TIM13_CLOCK,
};
#endif

#ifdef CONFIG_STM32_TIM14_CAP
static struct stm32_lowerhalf_s g_cap14_lowerhalf =
{
  .ops         = &g_cap_ops,
  .resolution  = STM32_TIM14_RES,
  .channel     = CONFIG_STM32_TIM14_CHANNEL,
  .clock       = CONFIG_STM32_TIM14_CLOCK,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_cap_handler
 *
 * Description:
 *   timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int stm32_cap_handler(int irq, void * context, void * arg)
{
  struct stm32_lowerhalf_s *lower = (struct stm32_lowerhalf_s *) arg;
  uint8_t ch = 0x3 & lower->channel;
  int period = 0;
  int flags = 0;

  flags = (int)STM32_CAP_GETFLAGS(lower->cap) ;

  STM32_CAP_ACKFLAGS(lower->cap, flags);

  period = STM32_CAP_GETCAPTURE(lower->cap, ch);

  if (period != 0)
    {
      lower->duty = (100 * STM32_CAP_GETCAPTURE(lower->cap, 0x3 & (~ch))) /
                    period;
    }
  else
    {
      lower->duty = 0;
    }

  lower->freq = lower->clock / period;

  return OK;
}

/****************************************************************************
 * Name: stm32_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_start(struct cap_lowerhalf_s *lower)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;
  int flags = 0;
  uint32_t maxtimeout = (1 << priv->resolution) - 1;

  if (priv->started)
    {
      /* Return EBUSY to indicate that the timer was already running */

      return -EBUSY;
    }

  switch (priv->channel)
    {
      case 1:
        STM32_CAP_SETSMC(priv->cap, STM32_CAP_SMS_RST |
                         STM32_CAP_TS_TI1FP1 |
                         STM32_CAP_MSM_MASK);

        STM32_CAP_SETCLOCK(priv->cap, priv->clock, maxtimeout);

        STM32_CAP_SETCHANNEL(priv->cap, 1,
                             STM32_CAP_EDGE_RISING |
                             STM32_CAP_MAPPED_TI1);
        STM32_CAP_SETCHANNEL(priv->cap, 2,
                             STM32_CAP_EDGE_FALLING |
                             STM32_CAP_MAPPED_TI2);

        flags = (int)STM32_CAP_GETFLAGS(priv->cap);
        STM32_CAP_ACKFLAGS(priv->cap, flags);

        STM32_CAP_SETISR(priv->cap, stm32_cap_handler, priv);
        STM32_CAP_ENABLEINT(priv->cap, STM32_CAP_FLAG_IRQ_CH_1, true);

        priv->started = true;
        break;

      case 2:
        STM32_CAP_SETSMC(priv->cap, STM32_CAP_SMS_RST |
                         STM32_CAP_TS_TI2FP2 |
                         STM32_CAP_MSM_MASK);

        STM32_CAP_SETCLOCK(priv->cap, priv->clock, maxtimeout);

        STM32_CAP_SETCHANNEL(priv->cap, 2,
                             STM32_CAP_EDGE_RISING |
                             STM32_CAP_MAPPED_TI1);
        STM32_CAP_SETCHANNEL(priv->cap, 1,
                             STM32_CAP_EDGE_FALLING |
                             STM32_CAP_MAPPED_TI2);

        flags = (int)STM32_CAP_GETFLAGS(priv->cap);
        STM32_CAP_ACKFLAGS(priv->cap, flags);

        STM32_CAP_SETISR(priv->cap, stm32_cap_handler, priv);
        STM32_CAP_ENABLEINT(priv->cap, STM32_CAP_FLAG_IRQ_CH_2, true);

        priv->started = true;
        break;

      default:
        return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_stop
 *
 * Description:
 *   Stop the capture
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_stop(struct cap_lowerhalf_s *lower)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;

  if (priv->started)
    {
      STM32_CAP_SETCHANNEL(priv->cap, STM32_CAP_FLAG_IRQ_COUNTER,
                           STM32_CAP_EDGE_DISABLED);
      switch (priv->channel)
        {
          case 1:
            STM32_CAP_ENABLEINT(priv->cap, STM32_CAP_FLAG_IRQ_CH_1, false);
            break;

          case 2:
            STM32_CAP_ENABLEINT(priv->cap, STM32_CAP_FLAG_IRQ_CH_2, false);
            break;

          default:
            return ERROR;
        }

      STM32_CAP_SETISR(priv->cap, NULL, NULL);
      priv->started = false;
      return OK;
    }

  /* Return ENODEV to indicate that the timer was not running */

  return -ENODEV;
}

/****************************************************************************
 * Name: stm32_getduty
 *
 * Description:
 *   get result duty
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   duty  - DutyCycle * 100.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_getduty(struct cap_lowerhalf_s *lower, uint8_t *duty)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;

  irqstate_t flags = enter_critical_section();

  *duty = priv->duty;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: stm32_getfreq
 *
 * Description:
 *   get result freq
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   freq  - Frequence in Hz .
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_getfreq(struct cap_lowerhalf_s *lower, uint32_t *freq)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;

  irqstate_t flags = enter_critical_section();

  *freq = priv->freq;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_cap_initialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level capture driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family but is somewhere in
 *     the range of {1,..,5 8,...,14}.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half capture driver returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct cap_lowerhalf_s *stm32_cap_initialize(int timer)
{
  struct stm32_lowerhalf_s *lower = NULL;

  switch (timer)
    {
#ifdef CONFIG_STM32_TIM1_CAP
      case 1:
        lower = &g_cap1_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32_TIM2_CAP
      case 2:
        lower = &g_cap2_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32_TIM3_CAP
      case 3:
        lower = &g_cap3_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32_TIM4_CAP
      case 4:
        lower = &g_cap4_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32_TIM5_CAP
      case 5:
        lower = &g_cap5_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32_TIM6_CAP
      case 6:
        lower = &g_cap6_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32_TIM9_CAP
      case 9:
        lower = &g_cap9_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32_TIM10_CAP
      case 10:
        lower = &g_cap10_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32_TIM11_CAP
      case 11:
        lower = &g_cap11_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32_TIM12_CAP
      case 12:
        lower = &g_cap12_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32_TIM13_CAP
      case 13:
        lower = &g_cap13_lowerhalf;
        break;
#endif
#ifdef CONFIG_STM32_TIM14_CAP
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
  lower->cap      = stm32_cap_init(timer);

  if (lower->cap == NULL)
    {
      lower = NULL;
    }

errout:
  return (struct cap_lowerhalf_s *)lower;
}

#endif /* CONFIG_STM32_CAP */
