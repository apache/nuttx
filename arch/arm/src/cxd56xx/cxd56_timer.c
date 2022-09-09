/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_timer.c
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

#include <inttypes.h>
#include <stdint.h>
#include <limits.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>
#include <arch/board/board.h>
#include <arch/chip/timer.h>

#include "arm_internal.h"
#include "cxd56_timer.h"
#include "hardware/cxd56_timer.h"
#include "cxd56_clock.h"

#ifdef CONFIG_TIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer divider definitions */

#if defined(CONFIG_CXD56_TIMER_DIVIDER_1)
#define TIMER_DIVIDER       (1)
#define TIMERCTRL_DIV       (TIMERCTRL_DIV_1)
#elif defined(CONFIG_CXD56_TIMER_DIVIDER_16)
#define TIMER_DIVIDER       (16)
#define TIMERCTRL_DIV       (TIMERCTRL_DIV_16)
#elif defined(CONFIG_CXD56_TIMER_DIVIDER_256)
#define TIMER_DIVIDER       (256)
#define TIMERCTRL_DIV       (TIMERCTRL_DIV_256)
#else
#define TIMER_DIVIDER       (16)
#define TIMERCTRL_DIV       (TIMERCTRL_DIV_16)
#endif

/* e.g.) When the timer divider is 16, timer's max clock is about 10MHz
 * (Divide max 160MHz resolution by 16) and the timer has 32bit counter.
 * Therefore, the max counter is the following value to avoid counter
 * wrap around. Timer's base clock is dynamically changed with cpu clock.
 */

#define CXD56_MAXTIMEOUT    (ULONG_MAX / 160 / TIMER_DIVIDER)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct cxd56_lowerhalf_s
{
  const struct timer_ops_s  *ops;  /* Lower half operations */

  /* Private data */

  uint32_t  base;     /* Base address of the timer */
  tccb_t    callback; /* Current user interrupt callback */
  void     *arg;      /* Argument passed to upper half callback */
  uint32_t  timeout;  /* The current timeout value (us) */
  uint32_t  clkticks; /* actual clock ticks for current interval */
  bool      started;  /* The timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt handling *******************************************************/

static int      cxd56_timer_interrupt(int irq, void *context,
                                      void *arg);

/* "Lower half" driver methods **********************************************/

static int      cxd56_start(struct timer_lowerhalf_s *lower);
static int      cxd56_stop(struct timer_lowerhalf_s *lower);
static int      cxd56_getstatus(struct timer_lowerhalf_s *lower,
                                struct timer_status_s *status);
static int      cxd56_settimeout(struct timer_lowerhalf_s *lower,
                                 uint32_t timeout);
static void     cxd56_setcallback(struct timer_lowerhalf_s *lower,
                                  tccb_t callback, void *arg);
static int      cxd56_ioctl(struct timer_lowerhalf_s *lower, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_tmrops =
{
  .start       = cxd56_start,
  .stop        = cxd56_stop,
  .getstatus   = cxd56_getstatus,
  .settimeout  = cxd56_settimeout,
  .setcallback = cxd56_setcallback,
  .ioctl       = cxd56_ioctl,
};

/* "Lower half" driver state */

static struct cxd56_lowerhalf_s g_tmrdevs[2];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_timer_interrupt
 *
 * Description:
 *   TC interrupt
 *
 * Input Parameters:
 *   Usual interrupt callback arguments.
 *
 * Returned Values:
 *   Always returns OK.
 *
 ****************************************************************************/

static int cxd56_timer_interrupt(int irq, void *context, void *arg)
{
  struct cxd56_lowerhalf_s *priv = (struct cxd56_lowerhalf_s *)arg;
  uint32_t timeout;
  uint32_t load;

  tmrinfo("Entry\n");
  DEBUGASSERT((irq >= CXD56_IRQ_TIMER0) && (irq <= CXD56_IRQ_TIMER1));

  /* Is there a registered callback?  If the callback has been
   * nullified, the timer will be stopped.
   */

  timeout = priv->timeout;
  if (priv->callback && priv->callback(&timeout, priv->arg))
    {
      if (timeout != priv->timeout)
        {
          /* Change period dynamically */

          priv->timeout = timeout;
          load =
            (((uint64_t)timeout * priv->clkticks) / TIMER_DIVIDER / 1000000);
          putreg32(load, priv->base + CXD56_TIMER_LOAD);
        }
    }
  else
    {
      /* No callback or the callback returned false.. stop the timer */

      cxd56_stop((struct timer_lowerhalf_s *)priv);
      tmrinfo("Stopped\n");
    }

  /* Clear the interrupts */

  putreg32(TIMER_INTERRUPT, priv->base + CXD56_TIMER_INTCLR);

  return OK;
}

/****************************************************************************
 * Name: cxd56_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_start(struct timer_lowerhalf_s *lower)
{
  struct cxd56_lowerhalf_s *priv = (struct cxd56_lowerhalf_s *)lower;

  tmrinfo("Entry: started %d\n", priv->started);

  /* Has the timer already been started? */

  if (!priv->started)
    {
      uint32_t ctrl = (TIMERCTRL_ENABLE | TIMERCTRL_DIV |
                       TIMERCTRL_SIZE_32BIT | TIMERCTRL_MODE_WRAP);

      if (priv->timeout)
        {
          ctrl |= (TIMERCTRL_PERIODIC | TIMERCTRL_INTENABLE);
        }
      else
        {
          ctrl |= (TIMERCTRL_FREERUN | TIMERCTRL_INTDISABLE);
        }

      /* Start the timer */

      putreg32(ctrl, priv->base + CXD56_TIMER_CONTROL);

      priv->started = true;
      return OK;
    }

  /* Return EBUSY to indicate that the timer was already running */

  return -EBUSY;
}

/****************************************************************************
 * Name: cxd56_stop
 *
 * Description:
 *   Stop the timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_stop(struct timer_lowerhalf_s *lower)
{
  struct cxd56_lowerhalf_s *priv = (struct cxd56_lowerhalf_s *)lower;

  tmrinfo("Entry: started %d\n", priv->started);

  /* Has the timer already been started? */

  if (priv->started)
    {
      /* Stop the timer */

      putreg32(0, priv->base + CXD56_TIMER_CONTROL);

      /* Clear interrupt just in case */

      putreg32(TIMER_INTERRUPT, priv->base + CXD56_TIMER_INTCLR);

      priv->started = false;
      return OK;
    }

  /* Return ENODEV to indicate that the timer was not running */

  return -ENODEV;
}

/****************************************************************************
 * Name: cxd56_getstatus
 *
 * Description:
 *   Get the current timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-
 *            half" driver state structure.
 *   status - The location to return the status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_getstatus(struct timer_lowerhalf_s *lower,
                           struct timer_status_s *status)
{
  struct cxd56_lowerhalf_s *priv = (struct cxd56_lowerhalf_s *)lower;
  uint64_t remaining;

  tmrinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = 0;
  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  /* Return the actual timeout in microseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the timer expires (in microseconds). */

  remaining = (uint64_t)getreg32(priv->base + CXD56_TIMER_VALUE);
  status->timeleft =
    (uint32_t)(remaining * 1000000ULL * TIMER_DIVIDER / priv->clkticks);

  tmrinfo("  flags    : %08" PRIx32 "\n", status->flags);
  tmrinfo("  timeout  : %" PRId32 "\n", status->timeout);
  tmrinfo("  timeleft : %" PRId32 "\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: cxd56_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower
 *             half" driver state structure.
 *   timeout - The new timeout value in microseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_settimeout(struct timer_lowerhalf_s *lower,
                            uint32_t timeout)
{
  struct cxd56_lowerhalf_s *priv = (struct cxd56_lowerhalf_s *)lower;
  uint32_t load;

  DEBUGASSERT(priv);

  if (priv->started)
    {
      return -EPERM;
    }

  tmrinfo("Entry: timeout=%" PRId32 "\n", timeout);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > CXD56_MAXTIMEOUT)
    {
      tmrerr("ERROR: Cannot represent timeout=%" PRIu32 " > %lu\n",
             timeout, CXD56_MAXTIMEOUT);
      return -ERANGE;
    }

  /* Intended timeout */

  priv->timeout = timeout;

  /* Actual clock ticks */

  priv->clkticks = cxd56_get_cpu_baseclk();

  load = (((uint64_t)timeout * priv->clkticks) / TIMER_DIVIDER / 1000000);
  putreg32(load, priv->base + CXD56_TIMER_LOAD);
  modifyreg32(priv->base + CXD56_TIMER_CONTROL, 0,
              TIMERCTRL_PERIODIC | TIMERCTRL_INTENABLE);

  tmrinfo("clkticks=%" PRId32 " timeout=%" PRId32 " load=%" PRId32 "\n",
          priv->clkticks, priv->timeout, load);

  return OK;
}

/****************************************************************************
 * Name: cxd56_setcallback
 *
 * Description:
 *   Call this user provided timeout callback.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the
 *                "lower-half" driver state structure.
 *   newhandler - The new timer expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Values:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static void cxd56_setcallback(struct timer_lowerhalf_s *lower,
                              tccb_t callback, void *arg)
{
  struct cxd56_lowerhalf_s *priv = (struct cxd56_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  DEBUGASSERT(priv);
  tmrinfo("Entry: callback=%p\n", callback);

  /* Save the new callback and argument */

  priv->callback = callback;
  priv->arg      = arg;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: cxd56_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *   cmd   - The ioctl command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_ioctl(struct timer_lowerhalf_s *lower, int cmd,
                       unsigned long arg)
{
  int ret = -ENOTTY;

  tmrinfo("Entry: cmd=%d arg=%ld\n", cmd, arg);

  /* Handle ioctl commands */

  switch (cmd)
    {
    /* cmd:         TCIOC_SETHANDLER
     * Description: Set interrupt callback function
     * Argument:    A pointer to struct timer_sethandler_s
     */

    case TCIOC_SETHANDLER:
      {
        struct timer_sethandler_s *param;

        /* Set user provided timeout callback function */

        param = (struct timer_sethandler_s *)((uintptr_t)arg);

        if (param != NULL)
          {
            cxd56_setcallback(lower, param->handler, param->arg);
            ret = OK;
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;
    default:
      break;
  }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_timer_initialize
 *
 * Description:
 *   Initialize the timer.  The timer is initialized and
 *   registers as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the timer.  This should be of the form
 *     /dev/timer0
 *   timer - the timer's number.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void cxd56_timer_initialize(const char *devpath, int timer)
{
  struct cxd56_lowerhalf_s *priv = &g_tmrdevs[timer];
  int irq;

  tmrinfo("Entry: devpath=%s\n", devpath);
  DEBUGASSERT((timer >= CXD56_TIMER0) && (timer <= CXD56_TIMER1));

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  switch (timer)
    {
      case CXD56_TIMER0:
        priv->base = CXD56_TIMER0_BASE;
        irq        = CXD56_IRQ_TIMER0;
        tmrinfo("Using: Timer 0");
        break;

      case CXD56_TIMER1:
        priv->base = CXD56_TIMER1_BASE;
        irq        = CXD56_IRQ_TIMER1;
        tmrinfo("Using: Timer 1");
        break;

      default:
        ASSERT(0);
    }

  priv->ops = &g_tmrops;

  irq_attach(irq, cxd56_timer_interrupt, priv);

  /* Enable NVIC interrupt. */

  up_enable_irq(irq);

  /* Register the timer driver as /dev/timerX */

  timer_register(devpath, (struct timer_lowerhalf_s *)priv);
}

#endif /* CONFIG_TIMER */
