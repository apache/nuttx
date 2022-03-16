/****************************************************************************
 * arch/risc-v/src/bl602/bl602_tim_lowerhalf.c
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
#include <nuttx/arch.h>

#include <sys/types.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>

#include <arch/board/board.h>
#include "riscv_internal.h"
#include "hardware/bl602_glb.h"
#include "hardware/bl602_timer.h"
#include "bl602_tim_lowerhalf.h"
#include "bl602_glb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER_MAX_VALUE (0xFFFFFFFF)
#define TIMER_CLK_DIV   (159)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bl602_tim_lowerhalf_s
{
  const struct timer_ops_s *ops; /* Lower half operations */

  tccb_t    callback; /* Current upper half interrupt callback */
  void     *arg;      /* Argument passed to upper half callback */
  bool      started;  /* True: Timer has been started */
  uint8_t   irq;      /* IRQ associated with this UART */
  uint8_t   tim;      /* timer tim 0,1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bl602_timer_handler(int irq, void *context, void *arg);

/* "Lower half" driver methods */

static int  bl602_tim_start(struct timer_lowerhalf_s *lower);
static int  bl602_tim_stop(struct timer_lowerhalf_s *lower);
static int  bl602_tim_getstatus(struct timer_lowerhalf_s *lower,
                                struct timer_status_s *status);
static int  bl602_tim_settimeout(struct timer_lowerhalf_s *lower,
                                 uint32_t timeout);
static void bl602_tim_setcallback(struct timer_lowerhalf_s *lower,
                                  tccb_t callback, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = bl602_tim_start,
  .stop        = bl602_tim_stop,
  .getstatus   = bl602_tim_getstatus,
  .settimeout  = bl602_tim_settimeout,
  .setcallback = bl602_tim_setcallback,
  .ioctl       = NULL,
};

#ifdef CONFIG_BL602_TIMER0
static struct bl602_tim_lowerhalf_s g_tim1_lowerhalf =
{
  .ops = &g_timer_ops,
  .irq = BL602_IRQ_TIMER_CH0,
  .tim = TIMER_CH0,
};
#endif

#ifdef CONFIG_BL602_TIMER1
static struct bl602_tim_lowerhalf_s g_tim2_lowerhalf =
{
  .ops = &g_timer_ops,
  .irq = BL602_IRQ_TIMER_CH1,
  .tim = TIMER_CH1,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_timer_handler
 *
 * Description:
 *   Timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int bl602_timer_handler(int irq, void *context, void *arg)
{
  struct bl602_tim_lowerhalf_s *priv = (struct bl602_tim_lowerhalf_s *)arg;
  uint32_t next_interval_us = 0;

  /* Clear Interrupt Bits */

  uint32_t int_id;
  uint32_t tmp_val;
  uint32_t tmp_addr;

  int_id   = getreg32(BL602_TIMER_TMSR2 + 4 * priv->tim);
  tmp_addr = BL602_TIMER_TICR2 + 4 * priv->tim;
  tmp_val  = getreg32(tmp_addr);

  /* Comparator 0 match interrupt */

  if ((int_id & TIMER_TMSR2_TMSR_0) != 0)
    {
      putreg32(tmp_val | TIMER_TMSR2_TMSR_0, tmp_addr);
      if (priv->callback(&next_interval_us, priv->arg))
        {
          if (next_interval_us > 0)
            {
              /* Set a value to the alarm */

              bl602_timer_disable(priv->tim);
              bl602_timer_setcompvalue(priv->tim, TIMER_COMP_ID_0,
                                       next_interval_us);
              bl602_timer_setpreloadvalue(priv->tim, 0);
              bl602_timer_enable(priv->tim);
            }
        }
      else
        {
          bl602_timer_disable(priv->tim);
          bl602_timer_setpreloadvalue(priv->tim, 0);
        }
    }

  /* Comparator 1 match interrupt */

  if ((int_id & TIMER_TMSR2_TMSR_1) != 0)
    {
      putreg32(tmp_val | TIMER_TICR2_TCLR_1, tmp_addr);
    }

  /* Comparator 2 match interrupt */

  if ((int_id & TIMER_TMSR2_TMSR_2) != 0)
    {
      putreg32(tmp_val | TIMER_TICR2_TCLR_2, tmp_addr);
    }

  return OK;
}

/****************************************************************************
 * Name: bl602_tim_start
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

static int bl602_tim_start(struct timer_lowerhalf_s *lower)
{
  struct bl602_tim_lowerhalf_s *priv = (struct bl602_tim_lowerhalf_s *)lower;

  if (!priv->started)
    {
      if (priv->callback == NULL)
        {
          return -EPERM;
        }

      bl602_timer_setpreloadvalue(priv->tim, 0);
      irq_attach(priv->irq, bl602_timer_handler, (void *)priv);
      up_enable_irq(priv->irq);
      bl602_timer_intmask(priv->tim, TIMER_INT_COMP_0, 0);
      bl602_timer_enable(priv->tim);
      priv->started = true;
      return OK;
    }

  /* Return EBUSY to indicate that the timer was already running */

  return -EBUSY;
}

/****************************************************************************
 * Name: bl602_tim_stop
 *
 * Description:
 *   Stop the timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bl602_tim_stop(struct timer_lowerhalf_s *lower)
{
  struct bl602_tim_lowerhalf_s *priv = (struct bl602_tim_lowerhalf_s *)lower;

  /* timer disable */

  if (priv->started)
    {
      bl602_timer_disable(priv->tim);
      priv->started = false;
      up_disable_irq(priv->irq);
      bl602_timer_intmask(priv->tim, TIMER_INT_COMP_0, 1);
      return OK;
    }

  /* Return ENODEV to indicate that the timer was not running */

  return -ENODEV;
}

/****************************************************************************
 * Name: bl602_tim_getstatus
 *
 * Description:
 *   get timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-
 *            half" driver state structure.
 *   status - The location to return the status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bl602_tim_getstatus(struct timer_lowerhalf_s *lower,
                               struct timer_status_s *status)
{
  struct bl602_tim_lowerhalf_s *priv = (struct bl602_tim_lowerhalf_s *)lower;
  uint32_t current_count;

  status->timeout = bl602_timer_getcompvalue(priv->tim, TIMER_COMP_ID_0);
  current_count   = bl602_timer_getcountervalue(priv->tim);
  if (current_count < status->timeout)
    {
      status->timeleft = status->timeout - current_count;
    }
  else
    {
      status->timeleft = 0;
    }

  return 0;
}

/****************************************************************************
 * Name: bl602_tim_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *"lower-half" driver state structure. timeout - The new timeout value in
 *microseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bl602_tim_settimeout(struct timer_lowerhalf_s *lower,
                                uint32_t timeout)
{
  struct bl602_tim_lowerhalf_s *priv = (struct bl602_tim_lowerhalf_s *)lower;

  bl602_timer_setcompvalue(priv->tim, TIMER_COMP_ID_0, timeout);

  return OK;
}

/****************************************************************************
 * Name: bl602_tim_setcallback
 *
 * Description:
 *   Call this user provided timeout handler.
 *
 * Input Parameters:
 *   lower    - A pointer the publicly visible representation of the
 *              "lower-half" driver state structure.
 *   callback - The new timer expiration function pointer.  If this function
 *              pointer is NULL, then the reset-on-expiration behavior is
 *              restored.
 *   arg      - Argument that will be provided in the callback
 *
 * Returned Value:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static void bl602_tim_setcallback(struct timer_lowerhalf_s *lower,
                                  tccb_t callback, void *arg)
{
  struct bl602_tim_lowerhalf_s *priv = (struct bl602_tim_lowerhalf_s *)lower;
  irqstate_t flags = enter_critical_section();

  /* Save the new callback */

  priv->callback = callback;
  priv->arg      = arg;

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *     form /dev/timer0
 *   timer - the timer's number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int bl602_timer_initialize(const char *devpath, int timer)
{
  struct bl602_tim_lowerhalf_s *lower;
  struct timer_cfg_s timstr;

  switch (timer)
    {
    case 0:
#ifdef CONFIG_BL602_TIMER0
      lower = &g_tim1_lowerhalf;
#endif
      break;
    case 1:
#ifdef CONFIG_BL602_TIMER1
      lower = &g_tim2_lowerhalf;
#endif
      break;
    default:
      return -ENODEV;
    }

  timstr.timer_ch = lower->tim;        /* Timer channel */
  timstr.clk_src  = TIMER_CLKSRC_FCLK; /* Timer clock source */
  timstr.pl_trig_src =
    TIMER_PRELOAD_TRIG_COMP0; /* Timer count register preload trigger source
                                 slelect */

  timstr.count_mode     = TIMER_COUNT_PRELOAD; /* Timer count mode */
  timstr.clock_division = TIMER_CLK_DIV;       /* Timer clock division value */
  timstr.match_val0     = TIMER_MAX_VALUE;     /* Timer match 0 value 0 */
  timstr.match_val1     = TIMER_MAX_VALUE;     /* Timer match 1 value 0 */
  timstr.match_val2     = TIMER_MAX_VALUE;     /* Timer match 2 value 0 */
  timstr.pre_load_val   = TIMER_MAX_VALUE;     /* Timer preload value */

  bl602_swrst_ahb_slave1(AHB_SLAVE1_TMR);

  bl602_timer_intmask(lower->tim, TIMER_INT_ALL, 1);

  /* timer disable */

  bl602_timer_disable(lower->tim);

  bl602_timer_init(&timstr);

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;

  /* Register the timer driver as /dev/timerX.  The returned value from
   * timer_register is a handle that could be used with timer_unregister().
   * REVISIT: The returned handle is discard here.
   */

  void *drvr = timer_register(devpath, (struct timer_lowerhalf_s *)lower);
  if (drvr == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the timer driver (such as if the
       * 'depath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      return -EEXIST;
    }

  return OK;
}
