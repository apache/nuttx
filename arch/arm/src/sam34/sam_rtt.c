/****************************************************************************
 * arch/arm/src/sam34/sam_rtt.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Dioron
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

#include <sys/types.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/timers/timer.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "sam_rtt.h"
#include "sam_periphclks.h"

#if defined(CONFIG_TIMER) && (defined(CONFIG_SAM34_RTT))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Clocking *****************************************************************/

#if defined(CONFIG_RTC_HIRES) && defined (CONFIG_SAM34_RTC)
#  define RTT_PRES      (32768/CONFIG_RTC_FREQUENCY)
#else
/* TODO: Allow prescaler selection. */
#  define RTT_PRES      1
#endif

#define RTT_FCLK        (BOARD_SCLK_FREQUENCY/RTT_PRES)
#define RTT_MAXTIMEOUT  ((1000000ULL * (0x100000000ULL)) / RTT_FCLK)

/* Configuration ************************************************************/

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the timer
 * driver.  NOTE: that only lldbg types are used so that the output is
 * immediately available.
 */

#ifdef CONFIG_DEBUG_RTT
#  define rttdbg    lldbg
#  define rttvdbg   llvdbg
#else
#  define rttdbg(x...)
#  define rttvdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct sam34_lowerhalf_s
{
  FAR const struct timer_ops_s  *ops;  /* Lower half operations */

  /* Private data */

  tccb_t   handler;         /* Current user interrupt handler */
  uint32_t timeout;         /* The current timeout value (us) */
  uint32_t clkticks;        /* actual clock ticks for current interval */
  uint32_t val;             /* rtt value of current timeout */
  uint32_t adjustment;      /* time lost due to clock resolution truncation (us) */
  bool     started;         /* The timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#if defined(CONFIG_SAM34_RTT_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t sam34_getreg(uint32_t addr);
static void     sam34_putreg(uint32_t val, uint32_t addr);
#else
# define        sam34_getreg(addr)     getreg32(addr)
# define        sam34_putreg(val,addr) putreg32(val,addr)
#endif

/* Interrupt handling *******************************************************/

static int      sam34_interrupt(int irq, FAR void *context);

/* "Lower half" driver methods **********************************************/

static int      sam34_start(FAR struct timer_lowerhalf_s *lower);
static int      sam34_stop(FAR struct timer_lowerhalf_s *lower);
static int      sam34_getstatus(FAR struct timer_lowerhalf_s *lower,
                  FAR struct timer_status_s *status);
static int      sam34_settimeout(FAR struct timer_lowerhalf_s *lower,
                  uint32_t timeout);
static tccb_t   sam34_sethandler(FAR struct timer_lowerhalf_s *lower,
                  tccb_t handler);
static int      sam34_ioctl(FAR struct timer_lowerhalf_s *lower, int cmd,
                  unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* "Lower half" driver methods */

static const struct timer_ops_s g_tcops =
{
  .start      = sam34_start,
  .stop       = sam34_stop,
  .getstatus  = sam34_getstatus,
  .settimeout = sam34_settimeout,
  .sethandler = sam34_sethandler,
  .ioctl      = sam34_ioctl,
};

/* "Lower half" driver state */

static struct sam34_lowerhalf_s g_tcdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam34_readvr
 *
 * Description:
 *   Get the contents of the value register.
 *
 ****************************************************************************/
static inline uint32_t sam34_readvr(void)
{
  register uint32_t v;

  /* Async counter, read until two consecutive reads match */

  do
    {
      v = getreg32(SAM_RTT_VR);
    }
  while(v != getreg32(SAM_RTT_VR));

  return v;
}

/****************************************************************************
 * Name: sam34_getreg
 *
 * Description:
 *   Get the contents of a register
 *
 ****************************************************************************/

#if defined(CONFIG_SAM34_RTT_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t sam34_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t count = 0;
  static uint32_t preval = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same registe last time?  Are
   * we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
           if (count == 4)
             {
               lldbg("...\n");
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

           lldbg("[repeats %d more times]\n", count-3);
         }

       /* Save the new address, value, and count */

       prevaddr = addr;
       preval   = val;
       count    = 1;
    }

  /* Show the register value read */

  lldbg("%08lx->%08lx\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: sam34_putreg
 *
 * Description:
 *   Set the contents of an SAM34 register to a value
 *
 ****************************************************************************/

#if defined(CONFIG_SAM34_RTT_REGDEBUG) && defined(CONFIG_DEBUG)
static void sam34_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08lx<-%08lx\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: sam34_interrupt
 *
 * Description:
 *   TC interrupt
 *
 * Input Parameters:
 *   Usual interrupt handler arguments.
 *
 * Returned Values:
 *   Always returns OK.
 *
 ****************************************************************************/

static int sam34_interrupt(int irq, FAR void *context)
{
  FAR struct sam34_lowerhalf_s *priv = &g_tcdev;

  rttvdbg("Entry\n");
  DEBUGASSERT(irq == SAM_IRQ_RTT);

  /* Check if the interrupt is really pending */

  if ((sam34_getreg(SAM_RTT_SR) & RTT_SR_ALMS) != 0)
    {
      uint32_t timeout;
      uint32_t mrval;
      uint32_t vr;
      uint32_t lateticks;

      /* Is there a registered handler? */

      if (priv->handler && priv->handler(&priv->timeout))
        {
          /* Disable int before writing new alarm */

          mrval = sam34_getreg(SAM_RTT_MR);
          sam34_putreg(mrval & ~RTT_MR_ALMIEN, SAM_RTT_MR);

          /* Calculate new ticks / dither adjustment */

          vr = sam34_readvr();
          priv->clkticks =
            ((uint64_t)(priv->adjustment + priv->timeout)) * RTT_FCLK / 1000000;

          /* Subtract off how late we are, but only up to half an interval.
           * TODO calculate lost ticks?
           */

          lateticks = vr - priv->val;
          if (lateticks <= (priv->clkticks>>1))
            {
              priv->clkticks -= lateticks;
            }

          /* Set next interval interval. */

          priv->val = vr + priv->clkticks;
          sam34_putreg(priv->val-1, SAM_RTT_AR);

          /* Re-enable alarm */

          sam34_putreg(mrval, SAM_RTT_MR);

          /* Truncated timeout */

          timeout = (1000000ULL * priv->clkticks) / RTT_FCLK;

          /* Truncated time to be added to next interval (dither) */

          priv->adjustment = (priv->adjustment + priv->timeout) - timeout;
        }
      else /* stop */
        {
          sam34_stop((FAR struct timer_lowerhalf_s *)priv);
        }

      /* RTT_SR_ALMS is cleared by reading SAM_RTT_SR */
    }

  return OK;
}

/****************************************************************************
 * Name: sam34_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_start(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct sam34_lowerhalf_s *priv = (FAR struct sam34_lowerhalf_s *)lower;
  uint32_t mr;
  uint32_t vr;

  rttvdbg("Entry\n");
  DEBUGASSERT(priv);

  if (priv->started)
    {
      return -EINVAL;
    }

#if defined(CONFIG_RTC_HIRES) && defined (CONFIG_SAM34_RTC)
  /* RTT is started with the RTC and always on */

  mr = sam34_getreg(SAM_RTT_MR) & ~RTT_MR_ALMIEN;
  sam34_putreg(mr, SAM_RTT_MR);
  vr = sam34_readvr();

#else
  sam_rtt_enableclk();                   /* Enable peripheral clock */
  mr = RTT_MR_RTPRES(RTT_PRES);
  sam34_putreg(mr, SAM_RTT_MR);          /* Set prescaler, disable ints */
  vr = 0;                                /* we're going to reset the counter */
#endif

  priv->val = vr + priv->clkticks;       /* value at end of interval */
  sam34_putreg(priv->val-1, SAM_RTT_AR); /* Set interval */

  if (priv->handler)
    {
      /* Clear status and enable interrupt */

      sam34_getreg(SAM_RTT_SR);
      mr |= RTT_MR_ALMIEN;
      sam34_putreg(mr, SAM_RTT_MR);
    }

#if !(defined(CONFIG_RTC_HIRES) && defined (CONFIG_SAM34_RTC))
  sam34_putreg(mr | RTT_MR_RTTRST, SAM_RTT_MR); /* Start counter */
#endif /* !(CONFIG_RTC_HIRES && CONFIG_SAM34_RTC) */

  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: sam34_stop
 *
 * Description:
 *   Stop the timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_stop(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct sam34_lowerhalf_s *priv = (FAR struct sam34_lowerhalf_s *)lower;
  rttvdbg("Entry\n");
  DEBUGASSERT(priv);

  if(!priv->started)
    {
      return -EINVAL;
    }

#if !(defined(CONFIG_RTC_HIRES) && defined (CONFIG_SAM34_RTC))
#if defined(RTT_MR_RTTDIS)
  sam34_putreg(RTT_MR_RTTDIS, SAM_RTT_MR);    /* Disable RTT */
#endif
  sam_rtt_disableclk();                       /* Disable peripheral clock */
#endif

  priv->started = false;

  return OK;
}

/****************************************************************************
 * Name: sam34_getstatus
 *
 * Description:
 *   Get the current timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-half"
 *            driver state structure.
 *   status - The location to return the status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_getstatus(FAR struct timer_lowerhalf_s *lower,
                           FAR struct timer_status_s *status)
{
  FAR struct sam34_lowerhalf_s *priv = (FAR struct sam34_lowerhalf_s *)lower;

  rttvdbg("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = 0;
  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->handler)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  /* Return the actual timeout is milliseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the timer expires (in microseconds) */

  status->timeleft = 1000000ULL*(sam34_getreg(SAM_RTT_AR) - sam34_readvr())/RTT_FCLK;

  rttvdbg("  flags    : %08x\n", status->flags);
  rttvdbg("  timeout  : %d\n", status->timeout);
  rttvdbg("  timeleft : %d\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: sam34_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_settimeout(FAR struct timer_lowerhalf_s *lower,
                            uint32_t timeout)
{
  FAR struct sam34_lowerhalf_s *priv = (FAR struct sam34_lowerhalf_s *)lower;

  DEBUGASSERT(priv);
  rttvdbg("Entry: timeout=%d\n", timeout);

  if(priv->started) return -EPERM;

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > RTT_MAXTIMEOUT)
    {
      rttdbg("Cannot represent timeout=%lu > %lu\n",
            timeout, RTT_MAXTIMEOUT);
      return -ERANGE;
    }

  priv->timeout = timeout;                                     /* Intended timeout */
  priv->clkticks = (((uint64_t)timeout * RTT_FCLK) / 1000000); /* Actual clock ticks */
  timeout = (1000000ULL * priv->clkticks) / RTT_FCLK;          /* Truncated timeout */
  priv->adjustment = priv->timeout - timeout;                  /* Truncated time to be added to next interval (dither) */

  rttvdbg("fclk=%d clkticks=%d timout=%d, adjustment=%d\n",
         RTT_FCLK, priv->clkticks, priv->timeout, priv->adjustment);

  return OK;
}

/****************************************************************************
 * Name: sam34_sethandler
 *
 * Description:
 *   Call this user provided timeout handler.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the "lower-half"
 *                driver state structure.
 *   newhandler - The new timer expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Values:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static tccb_t sam34_sethandler(FAR struct timer_lowerhalf_s *lower,
                               tccb_t handler)
{
  FAR struct sam34_lowerhalf_s *priv = (FAR struct sam34_lowerhalf_s *)lower;
  irqstate_t flags;
  tccb_t oldhandler;

  flags = irqsave();

  DEBUGASSERT(priv);
  rttvdbg("Entry: handler=%p\n", handler);

  /* Get the old handler return value */

  oldhandler = priv->handler;

  /* Save the new handler */

   priv->handler = handler;

  irqrestore(flags);
  return oldhandler;
}

/****************************************************************************
 * Name: sam34_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *   cmd   - The ioctl command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_ioctl(FAR struct timer_lowerhalf_s *lower, int cmd,
                    unsigned long arg)
{
  FAR struct sam34_lowerhalf_s *priv = (FAR struct sam34_lowerhalf_s *)lower;
  int ret = -ENOTTY;

  DEBUGASSERT(priv);
  rttvdbg("Entry: cmd=%d arg=%ld\n", cmd, arg);
  UNUSED(priv);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_rttinitialize
 *
 * Description:
 *   Initialize the timer.  The timer is initialized and registers as
 *   'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the timer.  This should be of the form
 *     /dev/rtt0
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void sam_rttinitialize(FAR const char *devpath)
{
  FAR struct sam34_lowerhalf_s *priv = &g_tcdev;

  rttvdbg("Entry: devpath=%s\n", devpath);

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  priv->ops = &g_tcops;

  (void)irq_attach(SAM_IRQ_RTT, sam34_interrupt);

  /* Enable NVIC interrupt. */

  up_enable_irq(SAM_IRQ_RTT);

  /* Register the timer driver as /dev/timerX */

  (void)timer_register(devpath, (FAR struct timer_lowerhalf_s *)priv);
}

#endif /* CONFIG_TIMER && CONFIG_SAM34_TCx */
