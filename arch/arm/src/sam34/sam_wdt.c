/****************************************************************************
 * arch/arm/src/sam34/sam_wdt.c
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
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_wdt.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_SAM34_WDT)

/****************************************************************************
 * Pre-processor Definitions
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

#define WDT_FCLK        (BOARD_SCLK_FREQUENCY / 128)
#define WDT_MAXTIMEOUT  ((1000 * (WDT_MR_WDV_MAX+1)) / WDT_FCLK)

/* Configuration ************************************************************/

#ifndef CONFIG_SAM34_WDT_DEFTIMOUT
#  define CONFIG_SAM34_WDT_DEFTIMOUT WDT_MAXTIMEOUT
#endif

#ifndef CONFIG_DEBUG_WATCHDOG_INFO
#  undef CONFIG_SAM34_WDT_REGDEBUG
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
  const struct watchdog_ops_s  *ops;  /* Lower half operations */

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

#ifdef CONFIG_SAM34_WDT_REGDEBUG
static uint32_t sam34_getreg(uint32_t addr);
static void     sam34_putreg(uint32_t val, uint32_t addr);
#else
#  define       sam34_getreg(addr)     getreg32(addr)
#  define       sam34_putreg(val,addr) putreg32(val,addr)
#endif

/* Interrupt handling *******************************************************/

static int      sam34_interrupt(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int      sam34_start(struct watchdog_lowerhalf_s *lower);
static int      sam34_stop(struct watchdog_lowerhalf_s *lower);
static int      sam34_keepalive(struct watchdog_lowerhalf_s *lower);
static int      sam34_getstatus(struct watchdog_lowerhalf_s *lower,
                  struct watchdog_status_s *status);
static int      sam34_settimeout(struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);
static xcpt_t   sam34_capture(struct watchdog_lowerhalf_s *lower,
                  xcpt_t handler);
static int      sam34_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
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

#ifdef CONFIG_SAM34_WDT_REGDEBUG
static uint32_t sam34_getreg(uint32_t addr)
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
              wdinfo("...\n");
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

          wdinfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  wdinfo("%08x->%08x\n", addr, val);
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

#ifdef CONFIG_SAM34_WDT_REGDEBUG
static void sam34_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  wdinfo("%08x<-%08x\n", addr, val);

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
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

static int sam34_interrupt(int irq, void *context, void *arg)
{
  struct sam34_lowerhalf_s *priv = &g_wdgdev;
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

          priv->handler(irq, context, NULL);
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
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_start(struct watchdog_lowerhalf_s *lower)
{
  struct sam34_lowerhalf_s *priv = (struct sam34_lowerhalf_s *)lower;
  uint32_t mr_val = 0;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* The watchdog is always disabled after a reset. It is enabled by setting
   * the WDGA bit in the WWDG_CR register, then it cannot be disabled again
   * except by a reset.
   */

#if defined(CONFIG_SAM34_JTAG_FULL_ENABLE) || \
    defined(CONFIG_SAM34_JTAG_NOJNTRST_ENABLE) || \
    defined(CONFIG_SAM34_JTAG_SW_ENABLE)
    {
      mr_val |= (WDT_MR_WDDBGHLT | WDT_MR_WDIDLEHLT);
    }
#endif

  /* TODO: WDT_MR_WDFIEN if handler available? WDT_MR_WDRPROC? */

  mr_val |= (WDT_MR_WDD(priv->window) | WDT_MR_WDV(priv->reload) |
             WDT_MR_WDRSTEN);
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
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_stop(struct watchdog_lowerhalf_s *lower)
{
  /* The watchdog is always disabled after a reset. It is enabled by clearing
   * the WDDIS bit in the WDT_CR register, then it cannot be disabled again
   * except by a reset.
   */

  wdinfo("Entry\n");
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
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_keepalive(struct watchdog_lowerhalf_s *lower)
{
  wdinfo("Entry\n");

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
 *   lower  - A pointer the publicly visible representation of the
 *            "lower-half" driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_getstatus(struct watchdog_lowerhalf_s *lower,
                           struct watchdog_status_s *status)
{
  struct sam34_lowerhalf_s *priv = (struct sam34_lowerhalf_s *)lower;
  uint32_t elapsed;

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

  /* REVISIT: not sure if you can read this... */

  elapsed = ((sam34_getreg(SAM_WDT_MR) & WDT_MR_WDV_MASK) >>
             WDT_MR_WDV_SHIFT);

  status->timeleft = (priv->timeout * elapsed) / (priv->reload + 1);

  wdinfo("Status     : %08" PRIx32 "\n", sam34_getreg(SAM_WDT_SR));
  wdinfo("  flags    : %08" PRIx32 "\n", status->flags);
  wdinfo("  timeout  : %" PRId32 "\n", status->timeout);
  wdinfo("  timeleft : %" PRId32 "\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: sam34_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   timeout - The new timeout value in millisecnds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam34_settimeout(struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  struct sam34_lowerhalf_s *priv = (struct sam34_lowerhalf_s *)lower;
  uint32_t reload;

  DEBUGASSERT(priv);
  wdinfo("Entry: timeout=%" PRId32 "\n", timeout);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > WDT_MAXTIMEOUT)
    {
      wderr("ERROR: Cannot represent timeout=%" PRId32 " > %d\n",
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

  wdinfo("fwdt=%d reload=%" PRId32 " timeout=%" PRId32 "\n",
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
 *   lower      - A pointer the publicly visible representation of the
 *                "lower-half" driver state structure.
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

static xcpt_t sam34_capture(struct watchdog_lowerhalf_s *lower,
                            xcpt_t handler)
{
#if 0 /* TODO */
  struct sam34_lowerhalf_s *priv = (struct sam34_lowerhalf_s *)lower;
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

  regval = sam34_getreg(SAM_WDT_CFR);
  if (handler)
    {
      /* Attaching... Enable the EWI interrupt */

      regval |= WWDG_CFR_EWI;
      sam34_putreg(regval, SAM_WDT_CFR);

      up_enable_irq(SAM_IRQ_WWDG);
    }
  else
    {
      /* Detaching... Disable the EWI interrupt */

      regval &= ~WWDG_CFR_EWI;
      sam34_putreg(regval, SAM_WDT_CFR);

      up_disable_irq(SAM_IRQ_WWDG);
    }

  leave_critical_section(flags);
  return oldhandler;

#endif
  DEBUGPANIC();
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

static int sam34_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                       unsigned long arg)
{
  struct sam34_lowerhalf_s *priv = (struct sam34_lowerhalf_s *)lower;
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
          uint32_t window = (((priv->timeout - mintime) * WDT_FCLK) /
                             1000) - 1;
          DEBUGASSERT(window <= priv->reload);
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
 *   Initialize the WDT watchdog timer.  The watchdog timer is initialized
 *   and registers as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_WDT_DISABLE_ON_RESET
void sam_wdtinitialize(const char *devpath)
{
  struct sam34_lowerhalf_s *priv = &g_wdgdev;
  uint32_t mr_val;

  /* Enable watchdog with 5 sec timeout */

  mr_val = (WDT_MR_WDD((5) * WDT_FCLK) | WDT_MR_WDV((5) * WDT_FCLK) |
           WDT_MR_WDRSTEN);
  sam34_putreg(mr_val, SAM_WDT_MR);

  wdinfo("Entry: devpath=%s\n", devpath);

  /* NOTE we assume that clocking to the IWDG has already been provided by
   * the RCC initialization logic.
   */

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  priv->ops = &g_wdgops;

  /* Attach our EWI interrupt handler (But don't enable it yet) */

  irq_attach(SAM_IRQ_WDT, sam34_interrupt, NULL);

  /* Select an arbitrary initial timeout value.  But don't start the watchdog
   * yet. NOTE: If the "Hardware watchdog" feature is enabled through the
   * device option bits, the watchdog is automatically enabled at power-on.
   */

  sam34_settimeout((struct watchdog_lowerhalf_s *)priv,
                   CONFIG_WDT_TIMEOUT);

  /* Disable minimum time feature for now. */

  priv->window = priv->reload;

  /* Register the watchdog driver, usually at CONFIG_WATCHDOG_DEVPATH
   * (default /dev/watchdog0).
   */

  watchdog_register(devpath, (struct watchdog_lowerhalf_s *)priv);
}
#endif /* CONFIG_WDT_DISABLE_ON_RESET */

#endif /* CONFIG_WATCHDOG && CONFIG_SAM34_WDT */
