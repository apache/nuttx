/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_timer.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "lpc43_timer.h"

#if defined(CONFIG_TIMER) && (defined(CONFIG_LPC43_TMR0) || \
    defined(CONFIG_LPC43_TMR1) || defined(CONFIG_LPC43_TMR2) || \
    defined(CONFIG_LPC43_TMR3) )

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_DEBUG_TIMER_INFO
#  undef CONFIG_LPC43_TMR_REGDEBUG
#endif

/* Clocking *****************************************************************/

/* TODO: Allow selection of any of the input clocks */

#define TMR_FCLK        (BOARD_FCLKOUT_FREQUENCY)
#define TMR_MAXTIMEOUT  ((1000000ULL * (1ULL + TMR_RVALUE_MASK)) / TMR_FCLK)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct lpc43_lowerhalf_s
{
  const struct timer_ops_s *ops; /* Lower half operations */

  /* Private data */

  uint32_t base;       /* Base address of the timer */
  tccb_t   callback;   /* Current user interrupt callback */
  void     *arg;       /* Argument passed to the callback function */
  uint32_t timeout;    /* The current timeout value (us) */
  uint32_t adjustment; /* time lost due to clock resolution truncation (us) */
  uint32_t clkticks;   /* actual clock ticks for current interval */
  bool     started;    /* The timer has been started */
  uint16_t tmrid;      /* Timer id */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_LPC43_TMR_REGDEBUG
static uint32_t lpc43_getreg(uint32_t addr);
static void     lpc43_putreg(uint32_t val, uint32_t addr);
#else
# define        lpc43_getreg(addr)     getreg32(addr)
# define        lpc43_putreg(val,addr) putreg32(val,addr)
#endif

/* Interrupt handling *******************************************************/

static int      lpc43_interrupt(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int      lpc43_start(struct timer_lowerhalf_s *lower);
static int      lpc43_stop(struct timer_lowerhalf_s *lower);
static int      lpc43_getstatus(struct timer_lowerhalf_s *lower,
                  struct timer_status_s *status);
static int      lpc43_settimeout(struct timer_lowerhalf_s *lower,
                  uint32_t timeout);
static void     lpc43_setcallback(struct timer_lowerhalf_s *lower,
                  tccb_t callback, void *arg);
static int      lpc43_ioctl(struct timer_lowerhalf_s *lower, int cmd,
                  unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_tmrops =
{
  .start       = lpc43_start,
  .stop        = lpc43_stop,
  .getstatus   = lpc43_getstatus,
  .settimeout  = lpc43_settimeout,
  .setcallback = lpc43_setcallback,
  .ioctl       = lpc43_ioctl,
};

/* "Lower half" driver state */

/* TODO - allocating all 6 now, even though we might not need them.
 *        May want to allocate the right number to not be wasteful.
 */

static struct lpc43_lowerhalf_s g_tmrdevs[4];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_getreg
 *
 * Description:
 *   Get the contents of a register
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_TMR_REGDEBUG
static uint32_t lpc43_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t count = 0;
  static uint32_t preval = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              tmrinfo("...\n");
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          tmrinfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  tmrinfo("%08lx->%08lx\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: lpc43_putreg
 *
 * Description:
 *   Set the contents of an LPC43 register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_TMR_REGDEBUG
static void lpc43_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  tmrinfo("%08lx<-%08lx\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

void tmr_clk_enable(uint16_t tmrid)
{
  uint32_t regval;

  /* Enable Timer 0 */

  if (tmrid == 0)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER0_CFG);
      regval |= CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER0_CFG);
    }

  /* Enable Timer 1 */

  if (tmrid == 1)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER1_CFG);
      regval |= CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER1_CFG);
    }

  /* Enable Timer 2 */

  if (tmrid == 2)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER2_CFG);
      regval |= CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER2_CFG);
    }

  /* Enable Timer 3 */

  if (tmrid == 3)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER3_CFG);
      regval |= CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER3_CFG);
    }
}

void tmr_clk_disable(uint16_t tmrid)
{
  uint32_t regval;

  /* Enable Timer 0 */

  if (tmrid == 0)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER0_CFG);
      regval &= ~CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER0_CFG);
    }

  /* Enable Timer 1 */

  if (tmrid == 1)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER1_CFG);
      regval &= ~CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER1_CFG);
    }

  /* Enable Timer 2 */

  if (tmrid == 2)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER2_CFG);
      regval &= ~CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER2_CFG);
    }

  /* Enable Timer 3 */

  if (tmrid == 3)
    {
      regval  = getreg32(LPC43_CCU1_M4_TIMER3_CFG);
      regval &= ~CCU_CLK_CFG_RUN;
      putreg32(regval, LPC43_CCU1_M4_TIMER3_CFG);
    }
}

/****************************************************************************
 * Name: lpc43_interrupt
 *
 * Description:
 *   TC interrupt
 *
 * Input Parameters:
 *   Usual interrupt callback arguments.
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

static int lpc43_interrupt(int irq, void *context, void *arg)
{
  uint8_t chan_int = 0x0f;
  struct lpc43_lowerhalf_s *priv = &g_tmrdevs[irq - LPC43M4_IRQ_TIMER0];

  tmrinfo("Entry\n");
  DEBUGASSERT((irq >= LPC43M4_IRQ_TIMER0) && (irq <= LPC43M4_IRQ_TIMER3));

  /* Check if the interrupt is really pending */

  if ((lpc43_getreg(priv->base + LPC43_TMR_IR_OFFSET) & chan_int) != 0)
    {
      uint32_t timeout;

      /* Is there a registered callback?  If the callback has been
       * nullified, the timer will be stopped.
       */

      if (priv->callback && priv->callback(&priv->timeout, priv->arg))
        {
          /* Calculate new ticks / dither adjustment */

          priv->clkticks = ((uint64_t)(priv->adjustment + priv->timeout)) *
            TMR_FCLK / 1000000;

          /* Set next interval interval. TODO: make sure the interval is not
           * so soon it will be missed!
           */

          lpc43_putreg(priv->clkticks, priv->base + LPC43_TMR_PR_OFFSET);

          /* Truncated timeout */

          timeout = (1000000ULL * priv->clkticks) / TMR_FCLK;

          /* Truncated time to be added to next interval (dither) */

          priv->adjustment = (priv->adjustment + priv->timeout) - timeout;
        }
      else
        {
          /* No callback or the callback returned false.. stop the timer */

          lpc43_stop((struct timer_lowerhalf_s *)priv);
          tmrinfo("Stopped\n");
        }

      /* Clear the interrupts */

      lpc43_putreg(chan_int, priv->base + LPC43_TMR_IR_OFFSET);
    }

  return OK;
}

/****************************************************************************
 * Name: lpc43_start
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

static int lpc43_start(struct timer_lowerhalf_s *lower)
{
  struct lpc43_lowerhalf_s *priv = (struct lpc43_lowerhalf_s *)lower;
  uint32_t presc_val;

  tmrinfo("Entry\n");
  DEBUGASSERT(priv);

  if (priv->started)
    {
      return -EINVAL;
    }

  /* Enable timer clock */

  tmr_clk_enable(priv->tmrid);

  /* Set it to Timer Mode */

  lpc43_putreg(0, priv->base + LPC43_TMR_CTCR_OFFSET);

  /* Disable the timer */

  lpc43_putreg(0, priv->base + LPC43_TMR_TCR_OFFSET);

  /* Set prescaler to increase TC each 1 us */

  presc_val = TMR_FCLK / 1000000;
  lpc43_putreg(presc_val - 1, priv->base + LPC43_TMR_PR_OFFSET);

  /* Set MR0 with a large enough initial value */

  lpc43_putreg(10000000, priv->base + LPC43_TMR_MR0_OFFSET);

  if (priv->callback)
    {
      /* Enable Match on MR0 generate interrupt and auto-restart */

      lpc43_putreg(3, priv->base + LPC43_TMR_MCR_OFFSET);
    }

  /* Enable the timer */

  lpc43_putreg(TMR_TCR_EN, priv->base + LPC43_TMR_TCR_OFFSET);

  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: lpc43_stop
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

static int lpc43_stop(struct timer_lowerhalf_s *lower)
{
  struct lpc43_lowerhalf_s *priv = (struct lpc43_lowerhalf_s *)lower;
  tmrinfo("Entry\n");
  DEBUGASSERT(priv);

  if (!priv->started)
    {
      return -EINVAL;
    }

  /* Disable timer */

  lpc43_putreg(0, priv->base + LPC43_TMR_TCR_OFFSET);

  /* Disable interrupt */

  lpc43_putreg(0, priv->base + LPC43_TMR_MCR_OFFSET);

  /* Disable timer clock */

  tmr_clk_disable(priv->tmrid);

  priv->started = false;

  return OK;
}

/****************************************************************************
 * Name: lpc43_getstatus
 *
 * Description:
 *   Get the current timer status
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

static int lpc43_getstatus(struct timer_lowerhalf_s *lower,
                           struct timer_status_s *status)
{
  struct lpc43_lowerhalf_s *priv = (struct lpc43_lowerhalf_s *)lower;
  uint32_t elapsed;

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

  /* Return the actual timeout is milliseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the timer expires (in microseconds) */

  /* TODO - check on the +1 in the time left calculation */

  elapsed = lpc43_getreg(priv->base + LPC43_TMR_TC_OFFSET);
  status->timeleft = ((uint64_t)priv->timeout * elapsed) /
    (priv->clkticks + 1);

  tmrinfo("  flags    : %08" PRIx32 "\n", status->flags);
  tmrinfo("  timeout  : %" PRId32 "\n", status->timeout);
  tmrinfo("  timeleft : %" PRId32 "\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: lpc43_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower
 *             half" driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc43_settimeout(struct timer_lowerhalf_s *lower,
                            uint32_t timeout)
{
  struct lpc43_lowerhalf_s *priv = (struct lpc43_lowerhalf_s *)lower;

  DEBUGASSERT(priv);

  if (priv->started)
    {
      return -EPERM;
    }

  tmrinfo("Entry: timeout=%" PRId32 "\n", timeout);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > TMR_MAXTIMEOUT)
    {
      tmrerr("ERROR: Cannot represent timeout=%" PRIu32 " > %llu\n",
             timeout, TMR_MAXTIMEOUT);
      return -ERANGE;
    }

  /* Intended timeout */

  priv->timeout = timeout;

  /* Actual clock ticks */

  priv->clkticks = (((uint64_t)timeout * TMR_FCLK) / 1000000);

  /* Truncated timeout */

  timeout = (1000000ULL * priv->clkticks) / TMR_FCLK;

  /* Truncated time to be added to next interval (dither) */

  priv->adjustment = priv->timeout - timeout;

  tmrinfo("fclk=%d clkticks=%" PRId32
          " timeout=%" PRId32 ", adjustment=%" PRId32 "\n",
          TMR_FCLK, priv->clkticks, priv->timeout, priv->adjustment);

  return OK;
}

/****************************************************************************
 * Name: lpc43_setcallback
 *
 * Description:
 *   Call this user provided timeout callback.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the
 *                "lower-half" driver state structure.
 *   newcallback - The new timer expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Value:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static void lpc43_setcallback(struct timer_lowerhalf_s *lower,
                              tccb_t callback, void *arg)
{
  struct lpc43_lowerhalf_s *priv = (struct lpc43_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  DEBUGASSERT(priv);
  tmrinfo("Entry: callback=%p\n", callback);

  /* Save the new callback and its argument */

  priv->callback = callback;
  priv->arg      = arg;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc43_ioctl
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
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc43_ioctl(struct timer_lowerhalf_s *lower, int cmd,
                       unsigned long arg)
{
  struct lpc43_lowerhalf_s *priv = (struct lpc43_lowerhalf_s *)lower;
  int ret = -ENOTTY;

  DEBUGASSERT(priv);
  tmrinfo("Entry: cmd=%d arg=%ld\n", cmd, arg);
  UNUSED(priv);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_tmrinitialize
 *
 * Description:
 *   Initialize the timer.  The timer is initialized and
 *   registers as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the timer.  This should be of the form
 *     /dev/tmr0
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc43_tmrinitialize(const char *devpath, int irq)
{
  struct lpc43_lowerhalf_s *priv = &g_tmrdevs[irq - LPC43M4_IRQ_TIMER0];

  tmrinfo("Entry: devpath=%s\n", devpath);
  DEBUGASSERT((irq >= LPC43M4_IRQ_TIMER0) && (irq <= LPC43M4_IRQ_TIMER3));

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  switch (irq)
    {
#if defined(CONFIG_LPC43_TMR0)
    case LPC43M4_IRQ_TIMER0:
      priv->base = LPC43_TIMER0_BASE;
      priv->tmrid = 0;
      tmrinfo("Using: Timer 0");
      break;
#endif

#if defined(CONFIG_LPC43_TMR1)
    case LPC43M4_IRQ_TIMER1:
      priv->base = LPC43_TIMER1_BASE;
      priv->tmrid = 1;
      tmrinfo("Using: Timer 1");
      break;
#endif

#if defined(CONFIG_LPC43_TMR2)
    case LPC43M4_IRQ_TIMER2:
      priv->base = LPC43_TIMER2_BASE;
      priv->tmrid = 2;
      tmrinfo("Using: Timer 2");
      break;
#endif

#if defined(CONFIG_LPC43_TMR3)
    case LPC43M4_IRQ_TIMER3:
      priv->base = LPC43_TIMER3_BASE;
      priv->tmrid = 3;
      tmrinfo("Using: Timer 3");
      break;
#endif

    default:
      DEBUGASSERT(0);
    }

  priv->ops = &g_tmrops;

  irq_attach(irq, lpc43_interrupt, NULL);

  /* Enable NVIC interrupt. */

  up_enable_irq(irq);

  /* Register the timer driver as /dev/timerX */

  timer_register(devpath, (struct timer_lowerhalf_s *)priv);
}

#endif /* CONFIG_TIMER && CONFIG_LPC43_TMRx */
