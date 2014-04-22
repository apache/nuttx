/****************************************************************************
 * arch/arm/src/sam34/sam_wdt.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
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

#include <nuttx/watchdog.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "sam_wdt.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_SAM34_WDT)

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Clocking *****************************************************************/
/* The minimum frequency of the WWDG clock is:
 *
 * So the maximum delay (in milliseconds) is then:
 *
 *   1000 * (WDT_CR_WDV_MAX+1) / WDT_FCLK
 *
 * For example, if SCLK = 32768MHz, then the maximum delay is:
 *
 *   Fmin = 1281.74
 *   1000 * 64 / Fmin = 49.93 msec
 */

#define WDT_FCLK        (BOARD_SLCK_FREQUENCY / 128)
#define WDT_MAXTIMEOUT  ((1000 * (WDT_MR_WDV_MAX+1)) / WDT_FCLK)

/* Configuration ************************************************************/

#ifndef CONFIG_SAM34_WDT_DEFTIMOUT
#  define CONFIG_SAM34_WDT_DEFTIMOUT WDT_MAXTIMEOUT
#endif

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the watchdog
 * driver.  NOTE: that only lldbg types are used so that the output is
 * immediately available.
 */

#ifdef CONFIG_DEBUG_WATCHDOG
#  define wddbg    lldbg
#  define wdvdbg   llvdbg
#else
#  define wddbg(x...)
#  define wdvdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct sam34_lowerhalf_s
{
  FAR const struct watchdog_ops_s  *ops;  /* Lower half operations */
  xcpt_t   handler;  /* Current EWI interrupt handler */
  uint32_t timeout;  /* The actual timeout value */
  bool     started;  /* The timer has been started */
  uint16_t reload;   /* The 12-bit reload field reset value (WDV) */
  uint16_t window;   /* The 12-bit window field value (WDD) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#if defined(CONFIG_SAM34_WDT_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t sam34_getreg(uint32_t addr);
static void     sam34_putreg(uint32_t val, uint32_t addr);
#else
# define        sam34_getreg(addr)     getreg32(addr)
# define        sam34_putreg(val,addr) putreg32(val,addr)
#endif

/* Interrupt handling *******************************************************/

static int      sam34_interrupt(int irq, FAR void *context);

/* "Lower half" driver methods **********************************************/

static int      sam34_start(FAR struct watchdog_lowerhalf_s *lower);
static int      sam34_stop(FAR struct watchdog_lowerhalf_s *lower);
static int      sam34_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int      sam34_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                  FAR struct watchdog_status_s *status);
static int      sam34_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);
static xcpt_t   sam34_capture(FAR struct watchdog_lowerhalf_s *lower,
                  xcpt_t handler);
static int      sam34_ioctl(FAR struct watchdog_lowerhalf_s *lower, int cmd,
                  unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = sam34_start,
  .stop       = sam34_stop,
  .keepalive  = sam34_keepalive,
  .getstatus  = sam34_getstatus,
  .settimeout = sam34_settimeout,
  .capture    = sam34_capture,
  .ioctl      = sam34_ioctl,
};

/* "Lower half" driver state */

static struct sam34_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam34_getreg
 *
 * Description:
 *   Get the contents of a register
 *
 ****************************************************************************/

#if defined(CONFIG_SAM34_WDT_REGDEBUG) && defined(CONFIG_DEBUG)
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

  lldbg("%08x->%08x\n", addr, val);
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

#if defined(CONFIG_SAM34_WDT_REGDEBUG) && defined(CONFIG_DEBUG)
static void sam34_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: sam34_interrupt
 *
 * Description:
 *   WDT interrupt
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
  FAR struct sam34_lowerhalf_s *priv = &g_wdgdev;
  uint16_t regval;

  /* Check if the EWI interrupt is really pending */

  regval = sam34_getreg(SAM_WDT_SR);
  if ((regval & (WDT_SR_WDUNF | WDT_SR_WDERR)) != 0)
    {
      /* Is there a registered handler? */

      if (priv->handler)
        {
          /* Yes... NOTE:  This interrupt service routine (ISR) must reload
           * the WWDG counter to prevent the reset.  Otherwise, we will reset
           * upon return.
           */

          priv->handler(irq, context);
        }

      /* The EWI interrupt is cleared by the WDT_SR register. */
    }

  return OK;
}

/****************************************************************************
 * Name: sam34_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct sam34_lowerhalf_s *priv = (FAR struct sam34_lowerhalf_s *)lower;
  uint32_t mr_val = 0;

  wdvdbg("Entry\n");
  DEBUGASSERT(priv);

  /* The watchdog is always disabled after a reset. It is enabled by setting
   * the WDGA bit in the WWDG_CR register, then it cannot be disabled again
   * except by a reset.
   */

#if defined(CONFIG_SAM34_JTAG_FULL_ENABLE) || \
    defined(CONFIG_SAM34_JTAG_NOJNTRST_ENABLE) || \
    defined(CONFIG_SAM34_JTAG_SW_ENABLE)
  {
    mr_val |= (WDT_MR_WDDBGHLT|WDT_MR_WDIDLEHLT);
  }
#endif

 /* TODO: WDT_MR_WDFIEN if handler available? WDT_MR_WDRPROC? */

  mr_val |= (WDT_MR_WDD(priv->window) | WDT_MR_WDV(priv->reload) | WDT_MR_WDRSTEN);
  sam34_putreg(mr_val, SAM_WDT_MR);
  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: sam34_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  /* The watchdog is always disabled after a reset. It is enabled by clearing
   * the WDDIS bit in the WDT_CR register, then it cannot be disabled again
   * except by a reset.
   */

  wdvdbg("Entry\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: sam34_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 *   The application program must write in the WDT_CR register at regular
 *   intervals during normal operation to prevent an MCU reset. 
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct sam34_lowerhalf_s *priv = (FAR struct sam34_lowerhalf_s *)lower;

  wdvdbg("Entry\n");
  DEBUGASSERT(priv);
  
  sam34_putreg((WDT_CR_KEY | WDT_CR_WDRSTT), SAM_WDT_CR);
  return OK;
}

/****************************************************************************
 * Name: sam34_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-half"
 *            driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                           FAR struct watchdog_status_s *status)
{
  FAR struct sam34_lowerhalf_s *priv = (FAR struct sam34_lowerhalf_s *)lower;
  uint32_t elapsed;

  wdvdbg("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }
 
  if (priv->handler)
    {
      status->flags |= WDFLAGS_CAPTURE;
    }

  /* Return the actual timeout is milliseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the watchdog expires (in milliseconds) */
  /* REVISIT: not sure if you can read this... */

  elapsed = ((sam34_getreg(SAM_WDT_MR) & WDT_MR_WDV_MASK) >> WDT_MR_WDV_SHIFT);

  status->timeleft = (priv->timeout * elapsed) / (priv->reload + 1);

  wdvdbg("Status     : %08x\n", sam34_getreg(SAM_WDT_SR));
  wdvdbg("  flags    : %08x\n", status->flags);
  wdvdbg("  timeout  : %d\n", status->timeout);
  wdvdbg("  timeleft : %d\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: sam34_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   timeout - The new timeout value in millisecnds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  FAR struct sam34_lowerhalf_s *priv = (FAR struct sam34_lowerhalf_s *)lower;
  uint32_t reload;

  DEBUGASSERT(priv);
  wdvdbg("Entry: timeout=%d\n", timeout);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > WDT_MAXTIMEOUT)
    {
      wddbg("Cannot represent timeout=%d > %d\n",
            timeout, WDT_MAXTIMEOUT);
      return -ERANGE;
    }

   
    reload = ((timeout * WDT_FCLK) / 1000) - 1;

  /* Make sure that the final reload value is within range */

  if (reload > WDT_MR_WDV_MAX)
    {
      reload = WDT_MR_WDV_MAX;
    }

  /* Calculate and save the actual timeout value in milliseconds:
   *
   * timeout =  1000 * (reload + 1) / Fwdt
   */

  priv->timeout = 1000 * (reload + 1) / WDT_FCLK;

  /* Remember the selected values */

  priv->reload = reload;

  wdvdbg("fwdt=%d reload=%d timout=%d\n",
         WDT_FCLK, reload, priv->timeout);
  
  /* Don't commit to MR register until started! */

  return OK;
}

/****************************************************************************
 * Name: sam34_capture
 *
 * Description:
 *   Don't reset on watchdog timer timeout; instead, call this user provider
 *   timeout handler.  NOTE:  Providing handler==NULL will restore the reset
 *   behavior.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the "lower-half"
 *                driver state structure.
 *   newhandler - The new watchdog expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Values:
 *   The previous watchdog expiration function pointer or NULL is there was
 *   no previous function pointer, i.e., if the previous behavior was
 *   reset-on-expiration (NULL is also returned if an error occurs).
 *
 ****************************************************************************/

static xcpt_t sam34_capture(FAR struct watchdog_lowerhalf_s *lower,
                            xcpt_t handler)
{
#if 0 // TODO
  FAR struct sam34_lowerhalf_s *priv = (FAR struct sam34_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;
  uint16_t regval;

  DEBUGASSERT(priv);
  wdvdbg("Entry: handler=%p\n", handler);

  /* Get the old handler return value */
  flags = irqsave();
  oldhandler = priv->handler;

  /* Save the new handler */

   priv->handler = handler;

  /* Are we attaching or detaching the handler? */

  regval = sam34_getreg(SAM_WDT_CFR);
  if (handler)
    {
      /* Attaching... Enable the EWI interrupt */

      regval |= WWDG_CFR_EWI;
      sam34_putreg(regval, SAM_WDT_CFR);
 
      up_enable_irq(STM32_IRQ_WWDG);
    }
  else
    {
      /* Detaching... Disable the EWI interrupt */

      regval &= ~WWDG_CFR_EWI;
      sam34_putreg(regval, SAM_WDT_CFR);

      up_disable_irq(STM32_IRQ_WWDG);
    }

  irqrestore(flags);
  return oldhandler;
#endif
  ASSERT(0);
  return NULL;
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

static int sam34_ioctl(FAR struct watchdog_lowerhalf_s *lower, int cmd,
                    unsigned long arg)
{
  FAR struct sam34_lowerhalf_s *priv = (FAR struct sam34_lowerhalf_s *)lower;
  int ret = -ENOTTY;

  DEBUGASSERT(priv);
  wdvdbg("Entry: cmd=%d arg=%ld\n", cmd, arg);

  /* WDIOC_MINTIME: Set the minimum ping time.  If two keepalive ioctls
   * are received within this time, a reset event will be generated.
   * Argument: A 32-bit time value in milliseconds.
   */

  if (cmd == WDIOC_MINTIME)
    {
      uint32_t mintime = (uint32_t)arg;

      ret = -EINVAL;
      if (priv->started) 
        {
          ret = -ENOSYS; /* can't write the MR more than once! */
        }

      /* The minimum time should be strictly less than the total delay
       * which, in turn, will be less than or equal to WDT_CR_MAX
       */

      else if (mintime < priv->timeout)
        {
          uint32_t window = (((priv->timeout - mintime) * WDT_FCLK) / 1000) - 1;
          DEBUGASSERT(window < priv->reload);
          priv->window = window; 
          ret = OK;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_wdtinitialize
 *
 * Description:
 *   Initialize the WDT watchdog timer.  The watchdog timer is initialized and
 *   registers as 'devpath'.  
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_WDT_DISABLE_ON_RESET
void sam_wdtinitialize(FAR const char *devpath)
{
  FAR struct sam34_lowerhalf_s *priv = &g_wdgdev;

  wdvdbg("Entry: devpath=%s\n", devpath);

  /* NOTE we assume that clocking to the IWDG has already been provided by
   * the RCC initialization logic.
   */

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  priv->ops = &g_wdgops;

  /* Attach our EWI interrupt handler (But don't enable it yet) */

  (void)irq_attach(SAM_IRQ_WDT, sam34_interrupt);

  /* Select an arbitrary initial timeout value.  But don't start the watchdog
   * yet. NOTE: If the "Hardware watchdog" feature is enabled through the
   * device option bits, the watchdog is automatically enabled at power-on.
   */

  sam34_settimeout((FAR struct watchdog_lowerhalf_s *)priv,
                   CONFIG_WDT_TIMEOUT);

  /* Register the watchdog driver as /dev/watchdog0 */

  (void)watchdog_register(devpath, (FAR struct watchdog_lowerhalf_s *)priv);

}
#endif /* CONFIG_WDT_DISABLE_ON_RESET */

#endif /* CONFIG_WATCHDOG && CONFIG_SAM34_WDT */
