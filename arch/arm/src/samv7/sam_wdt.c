/****************************************************************************
 * arch/arm/src/samv7/sam_wdg.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "sam_wdt.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_SAMV7_WDT)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_DEBUG_WATCHDOG_INFO
#  undef CONFIG_SAMV7_WDT_REGDEBUG
#endif

/* The Watchdog Timer uses the Slow Clock divided by 128 to establish the
 * maximum Watchdog period to be 16 seconds (with a typical Slow Clock of
 * 32768 kHz).
 */

#ifndef BOARD_SCLK_FREQUENCY
#  define BOARD_SCLK_FREQUENCY 32768
#endif

#define WDT_FREQUENCY (BOARD_SCLK_FREQUENCY / 128)

/* At 32768Hz, the maximum timeout value will be:
 *
 *   4096 / WDT_FREQUENCY = 256 seconds or 16,000 milliseconds
 *
 * And the minimum (non-zero) timeout would be:
 *
 *   1 / WDT_FREQUENCY = 3.9 milliseconds
 */

#define WDT_MINTIMEOUT ((1000 + WDT_FREQUENCY - 1) / WDT_FREQUENCY)
#define WDT_MAXTIMEOUT ((4096 * 1000) / WDT_FREQUENCY)

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct sam_lowerhalf_s
{
  FAR const struct watchdog_ops_s  *ops;  /* Lower half operations */
#ifdef CONFIG_SAMV7_WDT_INTERRUPT
  xcpt_t   handler;  /* Current WDT interrupt handler */
#endif
  uint32_t timeout;  /* The actual timeout value (milliseconds) */
  uint16_t reload;   /* The 12-bit watchdog reload value */
  bool     started;  /* The timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#ifdef CONFIG_SAMV7_WDT_REGDEBUG
static uint32_t sam_getreg(uintptr_t regaddr);
static void     sam_putreg(uint32_t regval, uintptr_t regaddr);
#else
# define        sam_getreg(regaddr)        getreg32(regaddr)
# define        sam_putreg(regval,regaddr) putreg32(regval,regaddr)
#endif

/* Interrupt handling *******************************************************/

#ifdef CONFIG_SAMV7_WDT_INTERRUPT
static int      sam_interrupt(int irq, FAR void *context, FAR void *arg);
#endif

/* "Lower half" driver methods **********************************************/

static int      sam_start(FAR struct watchdog_lowerhalf_s *lower);
static int      sam_stop(FAR struct watchdog_lowerhalf_s *lower);
static int      sam_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int      sam_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                  FAR struct watchdog_status_s *status);
static int      sam_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);
static xcpt_t   sam_capture(FAR struct watchdog_lowerhalf_s *lower,
                  xcpt_t handler);
static int      sam_ioctl(FAR struct watchdog_lowerhalf_s *lower, int cmd,
                  unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = sam_start,
  .stop       = sam_stop,
  .keepalive  = sam_keepalive,
  .getstatus  = sam_getstatus,
  .settimeout = sam_settimeout,
  .capture    = sam_capture,
  .ioctl      = sam_ioctl,
};

/* "Lower half" driver state */

static struct sam_lowerhalf_s g_wdtdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_getreg
 *
 * Description:
 *   Get the contents of an SAMV7 register
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_WDT_REGDEBUG
static uint32_t sam_getreg(uintptr_t regaddr)
{
  static uint32_t prevaddr = 0;
  static uint32_t count    = 0;
  static uint32_t preval   = 0;

  /* Read the value from the register */

  uint32_t regval = getreg32(regaddr);

  /* Is this the same value that we read from the same register last time?  Are
   * we polling the register?  If so, suppress some of the output.
   */

  if (regaddr == prevaddr && regval == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              wdinfo("...\n");
            }

          return regval;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          wdinfo("[repeats %d more times]\n", count-3);
        }

      /* Save the new address, value, and count */

      prevaddr = regaddr;
      preval   = regval;
      count    = 1;
    }

  /* Show the register value read */

  wdinfo("%08x->%048\n", regaddr, regval);
  return regval;
}
#endif

/****************************************************************************
 * Name: sam_putreg
 *
 * Description:
 *   Set the contents of an SAMV7 register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_WDT_REGDEBUG
static void sam_putreg(uint32_t regval, uintptr_t regaddr)
{
  /* Show the register value being written */

  wdinfo("%08x<-%08x\n", regaddr, regval);

  /* Write the value */

  putreg32(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: sam_interrupt
 *
 * Description:
 *   WDT early warning interrupt
 *
 * Input Parameters:
 *   Usual interrupt handler arguments.
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_WDT_INTERRUPT
static int sam_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct sam_lowerhalf_s *priv = &g_wdtdev;

  /* Is there a registered handler? */

  if (priv->handler)
    {
      /* Yes... NOTE:  This interrupt service routine (ISR) must reload
       * the WDT counter to prevent the reset.  Otherwise, we will reset
       * upon return.
       */

      priv->handler(irq, context);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: sam_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct sam_lowerhalf_s *priv = (FAR struct sam_lowerhalf_s *)lower;

  /* The watchdog timer is enabled or disabled by writing to the MR register.
   *
   * NOTE: The Watchdog Mode Register (WDT_MR) can be written only once.  Only
   * a processor reset resets it.  Writing the WDT_MR register reloads the
   * timer with the newly programmed mode parameters.
   */

  wdinfo("Entry\n");
  return priv->started ? OK : -ENOSYS;
}

/****************************************************************************
 * Name: sam_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  /* The watchdog timer is enabled or disabled by writing to the MR register.
   *
   * NOTE: The Watchdog Mode Register (WDT_MR) can be written only once.  Only
   * a processor reset resets it.  Writing the WDT_MR register reloads the
   * timer with the newly programmed mode parameters.
   */

  wdinfo("Entry\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: sam_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the atchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  wdinfo("Entry\n");

  /* Write WDT_CR_WDRSTT to the WDT CR register (along with the KEY value)
   * will restart the watchdog timer.
   */

  sam_putreg(WDT_CR_WDRSTT | WDT_CR_KEY, SAM_WDT_CR);
  return OK;
}

/****************************************************************************
 * Name: sam_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   stawtus - The location to return the watchdog status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                         FAR struct watchdog_status_s *status)
{
  FAR struct sam_lowerhalf_s *priv = (FAR struct sam_lowerhalf_s *)lower;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

#ifdef CONFIG_SAMV7_WDT_INTERRUPT
  if (priv->handler)
    {
      status->flags |= WDFLAGS_CAPTURE;
    }
#endif

  /* Return the actual timeout is milliseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the watchdog expires (in milliseconds)
   *
   * REVISIT:  I think this that this information is available.
   */

  status->timeleft = 0;

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08x\n", status->flags);
  wdinfo("  timeout  : %d\n", status->timeout);
  wdinfo("  timeleft : %d\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: sam_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   timeout - The new timeout value in millisecnds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  FAR struct sam_lowerhalf_s *priv = (FAR struct sam_lowerhalf_s *)lower;
  uint32_t reload;
  uint32_t regval;

  DEBUGASSERT(priv);
  wdinfo("Entry: timeout=%d\n", timeout);

  /* Can this timeout be represented? */

  if (timeout < WDT_MINTIMEOUT || timeout >= WDT_MAXTIMEOUT)
    {
      wderr("ERROR: Cannot represent timeout: %d < %d > %d\n",
            WDT_MINTIMEOUT, timeout, WDT_MAXTIMEOUT);
      return -ERANGE;
    }

  /* Calculate the reload value to achiee this (approximate) timeout.
   *
   * Examples with WDT_FREQUENCY = 32768 / 128 = 256:
   *  timeout = 4     -> reload = 1
   *  timeout = 16000 -> reload = 4096
   */

  reload = (timeout * WDT_FREQUENCY + 500) / 1000;
  if (reload < 1)
    {
      reload = 1;
    }
  else if (reload > 4095)
    {
      reload = 4095;
    }

  /* Calculate and save the actual timeout value in milliseconds:
   *
   * timeout =  1000 * (reload + 1) / Fwwdg
   */

  priv->timeout = (1000 * reload + WDT_FREQUENCY/2) / WDT_FREQUENCY;

  /* Remember the selected values */

  priv->reload = reload;

  wdinfo("reload=%d timeout: %d->%d\n",
         reload, timeout, priv->timeout);

  /* Set the WDT_MR according to calculated value
   *
   * NOTE: The Watchdog Mode Register (WDT_MR) can be written only once.  Only
   * a processor reset resets it.  Writing the WDT_MR register reloads the
   * timer with the newly programmed mode parameters.
   *
   * NOTE: The WDD Value is the lower bound of a so called Forbidden Window
   * (see Datasheet for further Information).  To disable this Forbidden
   * Window we have to set the WDD Value greater than or equal to WDV
   * (according the Datasheet).
   *
   * When setting the WDD Value equal to WDV we have to wait at least one clock
   * pulse of the (very slow) watchdog clock source between two resets (or the
   * configuration and the first reset) of the watchdog.
   *
   * On fast systems this can lead to a direct hit of the WDD boundary and
   * thus to a reset of the system.  This is why we program the WDD Value to
   * WDT_MR_WDD_MAX to truly disable this Forbidden Window Feature.
   */

  regval = WDT_MR_WDV(reload) | WDT_MR_WDD(WDT_MR_WDD_MAX);

#ifdef CONFIG_SAMV7_WDT_INTERRUPT
  /* Generate an interrupt whent he watchdog timer expires */

  regval |= WDT_MR_WDFIEN;
#else
  /* Reset (everything) if the watchdog timer expires.
   *
   * REVISIT:  Set WDT_MR_WDRPROC so that only the processor is reset?
   */

  regval |= WDT_MR_WDRSTEN;
#endif

#ifdef CONFIG_SAMV7_WDT_DEBUGHALT
  /* Halt the watchdog in the debug state */

  regval |= WDT_MR_WDDBGHLT;
#endif

#ifdef CONFIG_SAMV7_WDT_IDLEHALT
  /* Halt the watchdog in the IDLE mode */

  regval |= WDT_MR_WDIDLEHLT;
#endif

  sam_putreg(regval, SAM_WDT_MR);

  /* NOTE:  We had to start the watchdog here (because we cannot re-write the
   * MR register).  So sam_start will not be able to do anything.
   */

  priv->started = true;

  wdinfo("Setup: CR: %08x MR: %08x SR: %08x\n",
         sam_getreg(SAM_WDT_CR), sam_getreg(SAM_WDT_MR),
         sam_getreg(SAM_WDT_SR));

  return OK;
}

/****************************************************************************
 * Name: sam_capture
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
 * Returned Value:
 *   The previous watchdog expiration function pointer or NULL is there was
 *   no previous function pointer, i.e., if the previous behavior was
 *   reset-on-expiration (NULL is also returned if an error occurs).
 *
 ****************************************************************************/

static xcpt_t sam_capture(FAR struct watchdog_lowerhalf_s *lower,
                            xcpt_t handler)
{
#ifndef CONFIG_SAMV7_WDT_INTERRUPT
  wderr("ERROR: Not configured for this mode\n");
  return NULL;
#else
  FAR struct sam_lowerhalf_s *priv = (FAR struct sam_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;

  DEBUGASSERT(priv);
  wdinfo("Entry: handler=%p\n", handler);

  /* Get the old handler return value */

  flags = enter_critical_section();
  oldhandler = priv->handler;

  /* Save the new handler */

   priv->handler = handler;

  /* Are we attaching or detaching the handler? */

  if (handler)
    {
      /* Attaching... Enable the WDT interrupt */

      up_enable_irq(SAM_IRQ_WDT);
    }
  else
    {
      /* Detaching... Disable the WDT interrupt */

      up_disable_irq(SAM_IRQ_WDT);
    }

  leave_critical_section(flags);
  return oldhandler;
#endif
}

/****************************************************************************
 * Name: sam_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *   cmd   - The ioctol command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_ioctl(FAR struct watchdog_lowerhalf_s *lower, int cmd,
                    unsigned long arg)
{
  wdinfo("cmd=%d arg=%ld\n", cmd, arg);

  /* No ioctls are supported */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_wdt_initialize
 *
 * Description:
 *   Initialize the WDT watchdog time.  The watchdog timer is initialized and
 *   registered as 'devpath.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int sam_wdt_initialize(void)
{
  FAR struct sam_lowerhalf_s *priv = &g_wdtdev;

  wdinfo("Entry: CR: %08x MR: %08x SR: %08x\n",
         sam_getreg(SAM_WDT_CR), sam_getreg(SAM_WDT_MR),
         sam_getreg(SAM_WDT_SR));

  /* Check if some previous logic was disabled the watchdog timer.  Since the
   * MR can be written only one time, we are out of business if that is the
   * case.
   */

  DEBUGASSERT((sam_getreg(SAM_WDT_MR) & WDT_MR_WDDIS) == 0);

  /* No clock setup is required.  The Watchdog Timer uses the Slow Clock
   * divided by 128 to establish the maximum Watchdog period to be 16 seconds
   * (with a typical Slow Clock of 32768 kHz).
   */

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  priv->ops = &g_wdgops;

#ifdef CONFIG_SAMV7_WDT_INTERRUPT
  /* Attach our WDT interrupt handler (But don't enable it yet) */

  irq_attach(SAM_IRQ_WDT, sam_interrupt, NULL);
#endif

  /* Register the watchdog driver as device-node configured via .config.
   * Normally /dev/watchdog0
   */

  watchdog_register(CONFIG_WATCHDOG_DEVPATH,
                    (FAR struct watchdog_lowerhalf_s *)priv);
  return OK;
}

#endif /* CONFIG_WATCHDOG && CONFIG_SAMV7_WDT */
