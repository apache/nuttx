/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_wwdt.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
#include "hardware/lpc54_wwdt.h"
#include "lpc54_power.h"
#include "lpc54_wdt.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_LPC54_WWDT)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Clocking *****************************************************************/

#define WWDT_FREQ              3000000  /* Watchdog clock is IRC 12MHz, but
                                         * it has fixed divider by 4 */
#define LPC54_MAX_WWDT_TC      0xFFFFFF /* 24-bit counter max value */
#define LPC54_MIN_WWDT_TC      0xFF     /* 8-bit counter min value */
#define LPC54_MAX_WWDT_WINDOW  0xFFFFFF /* 24-bit max value */
#define LPC54_MIN_WWDT_WINDOW  0x100    /* Minimum window value allowed */
#define WWDT_WARNINT_VALUE     0x3FF    /* 10-bit max value */
#define WWDT_MAXTIMEOUT        5592     /* Max timeout value in milliseconds */

/* Configuration ************************************************************/

#ifndef CONFIG_LPC54_WWDT_DEFTIMOUT
#  define CONFIG_LPC54_WWDT_DEFTIMOUT WWDT_MAXTIMEOUT
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct lpc54_lowerhalf_wwdt_s
{
  FAR const struct watchdog_ops_s  *ops;  /* Lower half operations */
  xcpt_t   handler;  /* Current watchdog interrupt handler */
  uint32_t timeout;  /* The actual timeout value */
  bool     started;  /* The timer has been started */
  uint32_t reload;   /* The 24-bit reload field reset value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void   lpc54_setwindow(uint32_t window);
static void   lpc54_setwarning(uint32_t warning);

/* Interrupt handling *******************************************************/

static int    lpc54_wwdt_interrupt(int irq, FAR void *context);

/* "Lower half" driver methods **********************************************/

static int    lpc54_start(FAR struct watchdog_lowerhalf_s *lower);
static int    lpc54_stop(FAR struct watchdog_lowerhalf_s *lower);
static int    lpc54_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int    lpc54_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                FAR struct watchdog_status_s *status);
static int    lpc54_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                uint32_t timeout);
static xcpt_t lpc54_capture(FAR struct watchdog_lowerhalf_s *lower,
                 xcpt_t handler);
static int    lpc54_ioctl(FAR struct watchdog_lowerhalf_s *lower, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = lpc54_start,
  .stop       = lpc54_stop,
  .keepalive  = lpc54_keepalive,
  .getstatus  = lpc54_getstatus,
  .settimeout = lpc54_settimeout,
  .capture    = lpc54_capture,
  .ioctl      = lpc54_ioctl,
};

/* "Lower half" driver state */

static struct lpc54_lowerhalf_wwdt_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_setwindow
 *
 * Description:
 *  The window register determines the highest timeout value allowed when a
 *  watchdog feed is performed. If a feed valid sequence completes prior to
 *  timeout value reaching the value in window, a watchdog event will occur.
 *
 *  window resets to the maximum possible timeout value, so windowing is not
 *  in effect.  Values of window below 0x100 will make it impossible to ever
 *  feed the watchdog successfully
 *
 ****************************************************************************/

static void lpc54_setwindow(uint32_t window)
{
  /* WWDT window minimum value limiting */

  if (window < 0x100)
    {
        window = 0x100;
    }

  putreg32(window, LPC54_WWDT_WINDOW);
}

/****************************************************************************
 * Name: lpc54_setwarning
 *
 * Description:
 *  The WDWARNINT register determines the watchdog timer counter value that
 *  will generate a watchdog interrupt. When the watchdog timer counter
 *  matches the value defined by WDWARNINT, an interrupt will be generated
 *  after the subsequent WDCLK.  A match of the watchdog timer counter to
 *  WDWARNINT occurs when the bottom 10 bits of the counter have the same
 *  value as the 10 bits of WARNINT, and the remaining upper bits of the
 *  counter are all 0. This gives a maximum time of 1,023 watchdog timer
 *  counts (4,096 watchdog clocks) for the interrupt to occur prior to a
 *  watchdog event. If WDWARNINT is set to 0, the interrupt will occur at
 *  the same time as the watchdog event.
 *
 ****************************************************************************/

static void lpc54_setwarning(uint32_t warning)
{
  /* WWDT warning maximum value limiting */

  if (warning > 0x3FF)
    {
        warning = 0x3FF;
    }

  putreg32(warning, LPC54_WWDT_WARNINT);
}

/****************************************************************************
 * Name: lpc54_wwdt_interrupt
 *
 * Description:
 *   WWDT warning interrupt
 *
 * Input Parameters:
 *   Usual interrupt handler arguments.
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

static int lpc54_wwdt_interrupt(int irq, FAR void *context)
{
  FAR struct lpc54_lowerhalf_wwdt_s *priv = &g_wdgdev;
  uint32_t regval;

  /* Check if the watchdog warning interrupt is really pending */

  regval = getreg32(LPC54_WWDT_MOD);
  if ((regval & WWDT_MOD_WDINT) != 0)
    {
      /* Is there a registered handler? */

      if (priv->handler)
        {
          /* Yes... NOTE:  This interrupt service routine (ISR) must reload
           * the WWDT counter to prevent the reset.  Otherwise, we will
           * reset upon return.
           */

          priv->handler(irq, context);
        }

      /* The watchdog interrupt flag is cleared by writing '1' to the WDINT
       * bit in the WDMOD register.
       */

      regval |= WWDT_MOD_WDINT;
      putreg32(regval, LPC54_WWDT_MOD);
    }

  return OK;
}

/****************************************************************************
 * Name: lpc54_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc54_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct lpc54_lowerhalf_wwdt_s *priv =
    (FAR struct lpc54_lowerhalf_wwdt_s *)lower;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* The watchdog is always disabled after a reset. It is enabled by setting
   * the WDEN bit in the WDMOD register, then it cannot be disabled again
   * except by a reset.
   *
   * Watchdog is enabled and will reset the chip
   */

  putreg32(WWDT_MOD_WDEN | WWDT_MOD_WDRESET , LPC54_WWDT_MOD);

  /* Feed the watchdog to enable it */

  putreg32(0xAA, LPC54_WWDT_FEED);
  putreg32(0x55, LPC54_WWDT_FEED);

  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: lpc54_stop
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

static int lpc54_stop(FAR struct watchdog_lowerhalf_s *lower)
{

  /* The watchdog is always disabled after a reset. It is enabled by setting
  * the WDEN bit in the WDMOD register, then it cannot be disabled again
  * except by a reset.
  */

  wdinfo("Entry\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: lpc54_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "feeding the dog".
 *
 *   The application program must write in the FEED register at regular
 *   intervals during normal operation to prevent an MCU reset. This operation
 *   must occur only when the counter value is lower than the window register
 *   value.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc54_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct lpc54_lowerhalf_wwdt_s *priv =
    (FAR struct lpc54_lowerhalf_wwdt_s *)lower;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Feed the watchdog */

  putreg32(0xAA, LPC54_WWDT_FEED);
  putreg32(0x55, LPC54_WWDT_FEED);

  return OK;
}

/****************************************************************************
 * Name: lpc54_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-half"
 *            driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc54_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                           FAR struct watchdog_status_s *status)
{
  FAR struct lpc54_lowerhalf_wwdt_s *priv =
    (FAR struct lpc54_lowerhalf_wwdt_s *)lower;
  uint32_t elapsed;
  uint32_t reload;

  wdinfo("Entry\n");
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


  reload  = getreg32(LPC54_WWDT_TC);
  elapsed = priv->reload - reload;
  status->timeleft = (priv->timeout * elapsed) / (priv->reload + 1);

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08x\n", status->flags);
  wdinfo("  timeout  : %d\n", status->timeout);
  wdinfo("  timeleft : %d\n", status->flags);
  return OK;
}

/****************************************************************************
 * Name: lpc54_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc54_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  FAR struct lpc54_lowerhalf_wwdt_s *priv =
    (FAR struct lpc54_lowerhalf_wwdt_s *)lower;
  uint32_t reload;
  uint32_t regval;

  DEBUGASSERT(priv);
  wdinfo("Entry: timeout=%d\n", timeout);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > WWDT_MAXTIMEOUT)
    {
      wderr("ERROR: Cannot represent timeout=%d > %d\n",
            timeout, WWDT_MAXTIMEOUT_MS);
      return -ERANGE;
    }

  /* Determine timeout value */

  reload = WWDT_FREQ/1000;
  reload = timeout * reload;

  /* Make sure that the final reload value is within range */

  if (reload > LPC54_MAX_WWDT_TC)
    {
      reload = LPC54_MAX_WWDT_TC;
    }

  /* Save the actual timeout value in milliseconds*/

  priv->timeout = timeout;

  /* Remember the selected values */

  priv->reload = reload;
  wdinfo("reload=%d timeout=%d\n", reload, priv->timeout);
  regval = reload;
  putreg32(regval, LPC54_WWDT_TC);

  /* Reset the t window value to the maximum value.. essentially disabling
   * the lower limit of the watchdog reset time.
   */

  lpc54_setwindow(LPC54_MAX_WWDT_WINDOW);

  /* Set the warning interrupt register value */

  lpc54_setwarning(WWDT_WARNINT_VALUE);
  return OK;
}

/****************************************************************************
 * Name: lpc54_capture
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

static xcpt_t lpc54_capture(FAR struct watchdog_lowerhalf_s *lower,
                            xcpt_t handler)
{
  FAR struct lpc54_lowerhalf_wwdt_s *priv =
    (FAR struct lpc54_lowerhalf_wwdt_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;
  uint16_t regval;

  DEBUGASSERT(priv);
  wdinfo("Entry: handler=%p\n", handler);

  /* Get the old handler return value */

  flags = enter_critical_section();
  oldhandler = priv->handler;

  /* Save the new handler */

   priv->handler = handler;

  /* Are we attaching or detaching the handler? */

  regval = getreg32(LPC54_WWDT_MOD);
  if (handler)
    {
      /* Attaching... Enable the watchdog interrupt */

      regval |= WWDT_MOD_WDINT;
      putreg32(regval, LPC54_WWDT_MOD);

      up_enable_irq(LPC54_IRQ_WDT);
    }
  else
    {
      /* Detaching... Disable the EWI interrupt */

      regval &= ~WWDT_MOD_WDINT;
      putreg32(regval, LPC54_WWDT_MOD);

      up_disable_irq(LPC54_IRQ_WDT);
    }

  leave_critical_section(flags);
  return oldhandler;
}

/****************************************************************************
 * Name: lpc54_ioctl
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
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lpc54_ioctl(FAR struct watchdog_lowerhalf_s *lower, int cmd,
                       unsigned long arg)
{
  FAR struct lpc54_lowerhalf_wwdt_s *priv =
    (FAR struct lpc54_lowerhalf_wwdt_s *)lower;
  int ret = -ENOTTY;

  DEBUGASSERT(priv);
  wdinfo("Entry: cmd=%d arg=%ld\n", cmd, arg);

  /* WDIOC_MINTIME: Set the minimum ping time.  If two keepalive ioctls
   * are received within this time, a reset event will be generated.
   * Argument: A 32-bit time value in milliseconds.
   */

  if (cmd == WDIOC_MINTIME)
    {
      uint32_t mintime = (uint32_t)arg;

      /* The minimum time should be strictly less than the total delay
       * which, in turn, will be less than or equal to LPC54_MAX_WWDT_TC
       */

      ret = -EINVAL;
      if (mintime < priv->timeout)
        {
          uint32_t window = mintime*WWDT_FREQ/1000;
          DEBUGASSERT(window < priv->reload);
          lpc54_setwindow( window );
          ret = OK;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_wwdt_initialize
 *
 * Description:
 *   Initialize the WWDT watchdog time.  The watchdog timer is initialized and
 *   registers as 'devpath.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc54_wwdt_initialize(FAR const char *devpath)
{
  FAR struct lpc54_lowerhalf_wwdt_s *priv = &g_wdgdev;

  wdinfo("Entry: devpath=%s\n", devpath);

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  priv->ops = &g_wdgops;

  /* Turn on and configure the Watchdog oscillator. Set the PDEN_WDT_OSC bit
   * in the PDRUNCG0 register and setup the Watchdog oscillator control
   * register, WDTOSCCTRL.
   */

  lpc54_wdtosc_powerup();
#warning "Missing WDTOSCCTRL setup"

  /* Enable the register interface (WWDT bus clock):  Set the WWDT bit in the
   * AHBCLKCTRL0 register.
   */

  lpc54_wwdt_enableclk();

  /* For waking up from a WWDT interrupt, enable the watchdog interrupt for
   * wake-up in the STARTER0 register.
   */

  /* Set watchdog mode register to zero */

  putreg32(0, LPC54_WWDT_MOD);

  /* Attach our watchdog interrupt handler (But don't enable it yet) */

  irq_attach(LPC54_IRQ_WDT, lpc54_wwdt_interrupt);

  /* Select an arbitrary initial timeout value.  But don't start the watchdog
   * yet. NOTE: If the "Hardware watchdog" feature is enabled through the
   * device option bits, the watchdog is automatically enabled at power-on.
   */

  lpc54_settimeout((FAR struct watchdog_lowerhalf_s *)priv,
                   CONFIG_LPC54_WWDT_DEFTIMOUT);

  /* Register the watchdog driver as /dev/watchdog0 */

  watchdog_register(devpath, (FAR struct watchdog_lowerhalf_s *)priv);
}

#endif /* CONFIG_WATCHDOG && CONFIG_LPC54_WWDT */
