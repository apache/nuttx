/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_wdt.c
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

#include <inttypes.h>
#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "cxd56_clock.h"
#include "cxd56_wdt.h"
#include "cxd56_powermgr.h"

#ifdef CONFIG_CXD56_WDT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select the path to the registered watchdog timer device */

#ifdef CONFIG_WATCHDOG_DEVPATH
#define DEVPATH CONFIG_WATCHDOG_DEVPATH
#else
#define DEVPATH "/dev/watchdog0"
#endif

/* watchdog timeout maximum value */

#define WDT_MAX_TIMEOUT (40000) /* 40 sec */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct cxd56_lowerhalf_s
{
  const struct watchdog_ops_s *ops; /* Lower half operations */
#ifdef CONFIG_CXD56_WDT_INTERRUPT
  xcpt_t handler; /* Current WDT interrupt handler */
#endif
  uint32_t timeout; /* The actual timeout value (milliseconds) */
  uint32_t reload;  /* The 32-bit watchdog reload value */
  bool started;     /* The timer has been started */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations */

#if defined(CONFIG_CXD56_WDT_REGDEBUG) && defined(CONFIG_DEBUG_FEATURES)
static uint32_t cxd56_getreg(uintptr_t regaddr);
static void cxd56_putreg(uint32_t regval, uintptr_t regaddr);
#else
#  define cxd56_getreg(regaddr) getreg32(regaddr)
#  define cxd56_putreg(regval, regaddr) putreg32(regval, regaddr)
#endif

/* Interrupt handling *******************************************************/

#ifdef CONFIG_CXD56_WDT_INTERRUPT
static int cxd56_wdtinterrupt(int irq, void *context, void *arg);
#endif

/* "Lower half" driver methods **********************************************/

static int cxd56_start(struct watchdog_lowerhalf_s *lower);
static int cxd56_stop(struct watchdog_lowerhalf_s *lower);
static int cxd56_keepalive(struct watchdog_lowerhalf_s *lower);
static int cxd56_getstatus(struct watchdog_lowerhalf_s *lower,
                           struct watchdog_status_s *status);
static int cxd56_settimeout(struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout);
static xcpt_t cxd56_capture(struct watchdog_lowerhalf_s *lower,
                            xcpt_t handler);
static int cxd56_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                       unsigned long arg);
static int cxd56_pm_event(uint8_t id);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = cxd56_start,
  .stop       = cxd56_stop,
  .keepalive  = cxd56_keepalive,
  .getstatus  = cxd56_getstatus,
  .settimeout = cxd56_settimeout,
  .capture    = cxd56_capture,
  .ioctl      = cxd56_ioctl,
};

/* "Lower half" driver state */

static struct cxd56_lowerhalf_s g_wdtdev;

/* pm handle */

static void *pmhandle = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_getreg
 *
 * Description:
 *   Get the contents of an CXD56 register
 *
 ****************************************************************************/

#  if defined(CONFIG_CXD56_WDT_REGDEBUG) && defined(CONFIG_DEBUG_FEATURES)
static uint32_t cxd56_getreg(uintptr_t regaddr)
{
  static uint32_t prevaddr = 0;
  static uint32_t count    = 0;
  static uint32_t preval   = 0;

  /* Read the value from the register */

  uint32_t regval = getreg32(regaddr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register? If so, suppress some of the output.
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

          wdinfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = regaddr;
      preval   = regval;
      count    = 1;
    }

  /* Show the register value read */

  wdinfo("%08x->%08\n", regaddr, regval);
  return regval;
}
#endif

/****************************************************************************
 * Name: cxd56_putreg
 *
 * Description:
 *   Set the contents of an CXD56 register to a value
 *
 ****************************************************************************/

#if defined(CONFIG_CXD56_WDT_REGDEBUG) && defined(CONFIG_DEBUG_FEATURES)
static void cxd56_putreg(uint32_t regval, uintptr_t regaddr)
{
  /* Show the register value being written */

  wdinfo("%08x<-%08x\n", regaddr, regval);

  /* Write the value */

  putreg32(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: cxd56_wdtinterrupt
 *
 * Description:
 *   WDT early warning interrupt
 *
 * Input Parameters:
 *   Usual interrupt handler arguments.
 *
 * Returned Values:
 *   Always returns OK.
 *
 ****************************************************************************/

#ifdef CONFIG_CXD56_WDT_INTERRUPT
static int cxd56_wdtinterrupt(int irq, void *context, void *arg)
{
  struct cxd56_lowerhalf_s *priv = arg;

  /* Is there a registered handler? */

  if (priv->handler)
    {
      /* Yes... NOTE:  This interrupt service routine (ISR) must reload
       * the WDT counter to prevent the reset.  Otherwise, we will reset
       * upon return.
       */

      priv->handler(irq, context, NULL);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: cxd56_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_start(struct watchdog_lowerhalf_s *lower)
{
  struct cxd56_lowerhalf_s *priv = (struct cxd56_lowerhalf_s *)lower;

  wdinfo("Entry\n");

  cxd56_putreg(WDOGLOCK_UNLOCK_KEY, CXD56_WDT_WDOGLOCK);
  cxd56_putreg(WDOGCONTROL_RESEN | WDOGCONTROL_INTEN, CXD56_WDT_WDOGCONTROL);
  cxd56_putreg(0, CXD56_WDT_WDOGLOCK);

  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: cxd56_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_stop(struct watchdog_lowerhalf_s *lower)
{
  struct cxd56_lowerhalf_s *priv = (struct cxd56_lowerhalf_s *)lower;

  wdinfo("Entry\n");
  cxd56_putreg(WDOGLOCK_UNLOCK_KEY, CXD56_WDT_WDOGLOCK);
  cxd56_putreg(WDOGCONTROL_STOP, CXD56_WDT_WDOGCONTROL);
  priv->started = false;
  return OK;
}

/****************************************************************************
 * Name: cxd56_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the atchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_keepalive(struct watchdog_lowerhalf_s *lower)
{
  wdinfo("Entry\n");
  cxd56_putreg(WDOGLOCK_UNLOCK_KEY, CXD56_WDT_WDOGLOCK);
  cxd56_putreg(0, CXD56_WDT_WDOGINTCLR); /* reload by write any value */
  cxd56_putreg(0, CXD56_WDT_WDOGLOCK);
  return OK;
}

/****************************************************************************
 * Name: cxd56_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   stawtus - The location to return the watchdog status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_getstatus(struct watchdog_lowerhalf_s *lower,
                           struct watchdog_status_s *status)
{
  struct cxd56_lowerhalf_s *priv = (struct cxd56_lowerhalf_s *)lower;
  uint64_t remain;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

#ifdef CONFIG_CXD56_WDT_INTERRUPT
  if (priv->handler)
    {
      status->flags |= WDFLAGS_CAPTURE;
    }
#endif

  /* Return the actual timeout is milliseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the watchdog expires (in milliseconds) */

  remain           = (uint64_t)cxd56_getreg(CXD56_WDT_WDOGVALUE);
  status->timeleft = (uint32_t)(remain * 1000 / cxd56_get_cpu_baseclk());
  if (cxd56_getreg(CXD56_WDT_WDOGRIS) != WDOGRIS_RAWINT)
    {
      status->timeleft += status->timeout / 2;
    }

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08" PRIx32 "\n", status->flags);
  wdinfo("  timeout  : %" PRId32 "\n", status->timeout);
  wdinfo("  timeleft : %" PRId32 "\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: cxd56_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   timeout - The new timeout value in millisecnds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_settimeout(struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  struct cxd56_lowerhalf_s *priv = (struct cxd56_lowerhalf_s *)lower;
  uint32_t reload;
  uint32_t freq;
  uint64_t llreload;

  DEBUGASSERT(priv);
  wdinfo("Entry: timeout=%" PRId32 "\n", timeout);

  if ((timeout == 0) || (timeout > WDT_MAX_TIMEOUT))
    {
      return -EINVAL;
    }

  /* Calculate the reload value to achiee this (approximate) timeout. */

  freq = cxd56_get_cpu_baseclk();

  if (!freq)
    {
      return -EIO;
    }

  llreload = ((uint64_t)freq * (uint64_t)timeout + 500) / 1000;

  /* Actual register value is half of timeout value because watchdog makes
   * two laps until reset signal is asserted. At 1st lap, interrupt signal
   * is asserted. At 2nd lap, reset signal is asserted.
   */

  llreload /= 2;

  if (llreload >> 32)
    {
      return -ERANGE;
    }

  reload = (uint32_t)llreload;

  /* Calculate and save the actual timeout value in milliseconds */

  priv->timeout = (uint32_t)((llreload * 1000 + freq / 2) / freq) * 2;

  /* Remember the selected values */

  priv->reload = reload;

  wdinfo("reload=%" PRIu32 " timeout: %" PRId32 "->%" PRId32 "\n",
         reload, timeout, priv->timeout);

  /* Set the WDT register according to calculated value */

  cxd56_putreg(WDOGLOCK_UNLOCK_KEY, CXD56_WDT_WDOGLOCK);
  cxd56_putreg(reload, CXD56_WDT_WDOGLOAD);
  cxd56_putreg(WDOGCONTROL_RESEN | WDOGCONTROL_INTEN, CXD56_WDT_WDOGCONTROL);
  cxd56_putreg(0, CXD56_WDT_WDOGLOCK);

  priv->started = true;

  return OK;
}

/****************************************************************************
 * Name: cxd56_capture
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
 * Returned Values:
 *   The previous watchdog expiration function pointer or NULL is there was
 *   no previous function pointer, i.e., if the previous behavior was
 *   reset-on-expiration (NULL is also returned if an error occurs).
 *
 ****************************************************************************/

static xcpt_t cxd56_capture(struct watchdog_lowerhalf_s *lower,
                            xcpt_t handler)
{
#ifndef CONFIG_CXD56_WDT_INTERRUPT
  wderr("ERROR: Not configured for this mode\n");
  return NULL;
#else
  struct cxd56_lowerhalf_s *priv = (struct cxd56_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;

  DEBUGASSERT(priv);
  wdinfo("Entry: handler=%p\n", handler);

  /* Get the old handler return value */

  flags      = enter_critical_section();
  oldhandler = priv->handler;

  /* Save the new handler */

  priv->handler = handler;

  /* Are we attaching or detaching the handler? */

  if (handler)
    {
      /* Attaching... Enable the WDT interrupt */

      up_enable_irq(CXD56_IRQ_WDT_INT);
    }
  else
    {
      /* Detaching... Disable the WDT interrupt */

      up_disable_irq(CXD56_IRQ_WDT_INT);
    }

  leave_critical_section(flags);
  return oldhandler;
#endif
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
 *   cmd   - The ioctol command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                       unsigned long arg)
{
  wdinfo("cmd=%d arg=%ld\n", cmd, arg);

  /* No ioctls are supported */

  return -ENOTTY;
}

/****************************************************************************
 * Name: cxd56_pm_event
 *
 * Description:
 *   A callback function to receive events from power manager
 *
 * Input Parameters:
 *   id - A event identifier from power manager
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_pm_event(uint8_t id)
{
  struct cxd56_lowerhalf_s *priv = &g_wdtdev;

  switch (id)
    {
      case CXD56_PM_CALLBACK_ID_CLK_CHG_START:
      case CXD56_PM_CALLBACK_ID_HOT_SLEEP:

        /* do nothing, but watchdog is automatically stopped in hot sleep */

        break;

      case CXD56_PM_CALLBACK_ID_CLK_CHG_END:
      case CXD56_PM_CALLBACK_ID_HOT_BOOT:
        /* If watchdog has been already running before the clock is changed
         * or entering in hot sleep, re-start the watchdog timer with a
         * timeout value based on the new watchdog timer clock.
         */

        if (priv->started)
          {
            cxd56_settimeout((struct watchdog_lowerhalf_s *)priv,
                             priv->timeout);
          }
        break;

      default:
        break;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_wdt_initialize
 *
 * Description:
 *   Initialize the WDT watchdog time.  The watchdog timer is initialized and
 *   registered as 'devpath.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

int cxd56_wdt_initialize(void)
{
  struct cxd56_lowerhalf_s *priv = &g_wdtdev;

  /* set load value to max and lock */

  cxd56_putreg(0xffffffffu, CXD56_WDT_WDOGLOAD);
  cxd56_putreg(0, CXD56_WDT_WDOGLOCK);

  wdinfo("Entry: devpath=%s\n", DEVPATH);

  priv->ops = &g_wdgops;

#ifdef CONFIG_CXD56_WDT_INTERRUPT
  /* Attach our WDT interrupt handler (But don't enable it yet) */

  irq_attach(CXD56_IRQ_WDT_INT, cxd56_wdtinterrupt, priv);
#endif

  /* Register the watchdog driver as /dev/watchdog0 */

  watchdog_register(DEVPATH, (struct watchdog_lowerhalf_s *)priv);

  /* Register pm event callback */

  pmhandle = cxd56_pm_register_callback(PM_CLOCK_APP_CPU, cxd56_pm_event);

  return OK;
}

#endif /* CONFIG_CXD56_WDT */
