/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wdt_lowerhalf.c
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
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/watchdog.h>
#include <nuttx/spinlock.h>

#include "xtensa.h"

#include "hardware/esp32_soc.h"

#include "esp32_wdt.h"
#include "esp32_wdt_lowerhalf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PRE_VALUE                40000      /* 40000 * 12.5 ns = 500 us */

/* Number of Cycles to have 1 ms period
 * cycles = (1*10^-3 (s))*(f (Hz))
 */

#define MS_CYCLES_TIMER          2          /* 1 ms/(12.5 ns*PRE_VALUE) */
#define STAGE_0                  0
#define STAGE_1                  1
#define STAGE_2                  2
#define STAGE_3                  3
#define RESET_SYSTEM_RTC         4          /* Reset Main System + RTC */
#define RESET_SYSTEM_TIMER       3          /* Reset Main System */
#define INTERRUPT_ON_TIMEOUT     1
#define STAGES                   4
#define FULL_STAGE               0xffffffff /* ((2^32)-1) */
#define MAX_MWDT_TIMEOUT_MS      0x7fffffff /* ((2^32)-1)/cycles */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct esp32_wdt_lowerhalf_s
{
  const struct watchdog_ops_s *ops;       /* Lower half operations */
  struct esp32_wdt_dev_s      *wdt;       /* esp32 watchdog driver */
  uint32_t                    timeout;    /* The current timeout */
  enum wdt_peripherals        peripheral; /* Indicates if it is from RTC or Timer Module */
  uint32_t                    lastreset;  /* The last reset time */
  bool                        started;    /* True: Timer has been started */
  xcpt_t                      handler;    /* User Handler */
  void                        *upper;     /* Pointer to watchdog_upperhalf_s */
  spinlock_t                  lock;       /* Device specific lock */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt handling *******************************************************/

static int      esp32_wdt_handler(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int      esp32_wdt_start(struct watchdog_lowerhalf_s *lower);
static int      esp32_wdt_stop(struct watchdog_lowerhalf_s *lower);
static int      esp32_wdt_keepalive(struct watchdog_lowerhalf_s *lower);
static int      esp32_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                                    struct watchdog_status_s *status);
static int      esp32_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                                     uint32_t timeout);
static xcpt_t   esp32_wdt_capture(struct watchdog_lowerhalf_s *lower,
                                  xcpt_t handler);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_esp32_wdg_ops =
{
  .start      = esp32_wdt_start,
  .stop       = esp32_wdt_stop,
  .keepalive  = esp32_wdt_keepalive,
  .getstatus  = esp32_wdt_getstatus,
  .settimeout = esp32_wdt_settimeout,
  .capture    = esp32_wdt_capture,
  .ioctl      = NULL,
};

#ifdef CONFIG_ESP32_MWDT0
/* MWDT0 lower-half */

static struct esp32_wdt_lowerhalf_s g_esp32_mwdt0_lowerhalf =
{
  .ops = &g_esp32_wdg_ops,
};
#endif

#ifdef CONFIG_ESP32_MWDT1
/* MWDT1 lower-half */

static struct esp32_wdt_lowerhalf_s g_esp32_mwdt1_lowerhalf =
{
  .ops = &g_esp32_wdg_ops,
};
#endif

#ifdef CONFIG_ESP32_RWDT
/* RWDT lower-half */

static struct esp32_wdt_lowerhalf_s g_esp32_rwdt_lowerhalf =
{
  .ops = &g_esp32_wdg_ops,
};
#endif

/****************************************************************************
 * Name: esp32_wdt_start
 *
 * Description:
 *   Start the watchdog timer, register a callback if there is one and
 *   enables interrupt, otherwise, configure it to reset system on
 *   expiration.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp32_wdt_start(struct watchdog_lowerhalf_s *lower)
{
  struct esp32_wdt_lowerhalf_s *priv =
    (struct esp32_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry: started\n");
  DEBUGASSERT(priv);

  if (priv->started == true)
    {
      /* Return EBUSY to indicate that the timer was already running */

      return -EBUSY;
    }

  /* If WDT was not started yet */

  else
    {
      priv->started   = true;

      /* Unlock WDT */

      ESP32_WDT_UNLOCK(priv->wdt);

      /* No User Handler */

      if (priv->handler == NULL)
        {
             /* Then configure it to reset on wdt expiration */

            if (priv->peripheral == TIMER)
              {
                 ESP32_WDT_STG_CONF(priv->wdt, STAGE_0, RESET_SYSTEM_TIMER);
              }
            else
              {
                ESP32_WDT_STG_CONF(priv->wdt, STAGE_0, RESET_SYSTEM_RTC);
              }
        }

      /* User handler was already provided */

      else
        {
          /* Then configure it to call the user handler on wdt expiration */

          ESP32_WDT_STG_CONF(priv->wdt, STAGE_0, INTERRUPT_ON_TIMEOUT);

          /* Set the lower half handler and enable interrupt */

          flags = spin_lock_irqsave(&priv->lock);
          ESP32_WDT_SETISR(priv->wdt, esp32_wdt_handler, priv);
          spin_unlock_irqrestore(&priv->lock, flags);

          ESP32_WDT_ENABLEINT(priv->wdt);
        }

      flags = spin_lock_irqsave(&priv->lock);
      priv->lastreset = clock_systime_ticks();
      ESP32_WDT_START(priv->wdt);
      spin_unlock_irqrestore(&priv->lock, flags);

      /* Lock it again */

      ESP32_WDT_LOCK(priv->wdt);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_stop
 *
 * Description:
 *   Stop the watchdog timer. In case a callback was previously configured,
 *   unregister and deallocate it.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 ****************************************************************************/

static int esp32_wdt_stop(struct watchdog_lowerhalf_s *lower)
{
  struct esp32_wdt_lowerhalf_s *priv =
  (struct esp32_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  /* Unlock WDT */

  ESP32_WDT_UNLOCK(priv->wdt);

  /* Disable the WDT */

  ESP32_WDT_STOP(priv->wdt);

  /* In case there is some callback registered, disable and deallocate */

  if (priv->handler != NULL)
    {
      ESP32_WDT_DISABLEINT(priv->wdt);

      flags = spin_lock_irqsave(&priv->lock);
      ESP32_WDT_SETISR(priv->wdt, NULL, NULL);
      spin_unlock_irqrestore(&priv->lock, flags);
    }

  /* Lock it again */

  ESP32_WDT_LOCK(priv->wdt);

  priv->started = false;

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_keepalive
 *
 * Description:
 *   Reset the watchdog timer, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 *
 ****************************************************************************/

static int esp32_wdt_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct esp32_wdt_lowerhalf_s *priv =
    (struct esp32_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry\n");

  /* Unlock */

  ESP32_WDT_UNLOCK(priv->wdt);

  /* Feed the dog and updates the lastreset variable */

  flags = spin_lock_irqsave(&priv->lock);
  priv->lastreset = clock_systime_ticks();
  ESP32_WDT_FEED(priv->wdt);
  spin_unlock_irqrestore(&priv->lock, flags);

  /* Lock */

  ESP32_WDT_LOCK(priv->wdt);

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of
 *            the "lower-half" driver state structure.
 *   status - The location to return the watchdog status information.
 *
 ****************************************************************************/

static int esp32_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                           struct watchdog_status_s *status)
{
  struct esp32_wdt_lowerhalf_s *priv =
    (struct esp32_wdt_lowerhalf_s *)lower;
  uint32_t ticks;
  uint32_t elapsed;

  DEBUGASSERT(priv);

  /* Flags */

  status->flags = 0;
  /* If no handler was settled, then RESET on expiration.
   * Otherwise, call the user handler.
   */

  if (priv->handler == NULL)
    {
      status->flags |= WDFLAGS_RESET;
    }
  else
    {
      status->flags |= WDFLAGS_CAPTURE;
    }
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  /* Return the current timeout in milliseconds */

  status->timeout = priv->timeout;

  /* Get the elapsed time since the last ping */

  ticks   = clock_systime_ticks() - priv->lastreset;
  elapsed = (uint32_t)TICK2MSEC(ticks);

  if (elapsed < priv->timeout)
    {
      /* Return the approximate time until the watchdog timer expiration */

      status->timeleft = priv->timeout - elapsed;
    }
  else
    {
      status->timeleft = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of
 *             the "lower-half" driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp32_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  struct esp32_wdt_lowerhalf_s *priv =
    (struct esp32_wdt_lowerhalf_s *)lower;
  uint16_t rtc_cycles = 0;
  uint32_t rtc_ms_max = 0;

  wdinfo("Entry: timeout=%d\n", timeout);
  DEBUGASSERT(priv);

  /* Unlock WDT */

  ESP32_WDT_UNLOCK(priv->wdt);

  /* Write the timeout value */

  priv->timeout = timeout;

  /* Watchdog from Timer Module */

  if (priv->peripheral == TIMER)
    {
      /* Is this timeout a valid value for Timer's WDT? */

      if (timeout == 0 || timeout > MAX_MWDT_TIMEOUT_MS)
        {
          wderr("ERROR: Cannot represent timeout=%d > %d\n",
                timeout, MAX_MWDT_TIMEOUT_MS);
          return -ERANGE;
        }
      else
        {
          timeout = timeout*MS_CYCLES_TIMER;
          ESP32_WDT_STO(priv->wdt, timeout, STAGE_0);
        }
    }

  /* Watchdog from RTC Module */

  else
    {
      rtc_cycles = ESP32_RWDT_CLK(priv->wdt);
      rtc_ms_max = (uint32_t)(FULL_STAGE / rtc_cycles);

      /* Is this timeout a valid value for RTC WDT? */

      if (timeout == 0 || timeout > rtc_ms_max)
        {
          wderr("ERROR: Cannot represent timeout=%d > %d\n",
                timeout, rtc_ms_max);
          return -ERANGE;
        }
      else
        {
          timeout = timeout*rtc_cycles;
          ESP32_WDT_STO(priv->wdt, timeout, STAGE_0);
        }
    }

  /* Reset the wdt */

  ESP32_WDT_FEED(priv->wdt);

  /* Lock it again */

  ESP32_WDT_LOCK(priv->wdt);

  return OK;
}

/****************************************************************************
 * Name: esp32_wdt_capture
 *
 * Description:
 *   Don't reset on watchdog timer timeout; instead, call this user provider
 *   timeout handler.  NOTE:  Providing handler==NULL will restore the reset
 *   behavior.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the
 * "lower-half" driver state structure.
 *   newhandler - The new watchdog expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Value:
 *   The previous watchdog expiration function pointer or NULL if there was
 *   no previous function pointer, i.e., if the previous behavior was
 *   reset-on-expiration (NULL is also returned if an error occurs).
 *
 ****************************************************************************/

static xcpt_t esp32_wdt_capture(struct watchdog_lowerhalf_s *lower,
                            xcpt_t handler)
{
  struct esp32_wdt_lowerhalf_s *priv =
  (struct esp32_wdt_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;

  DEBUGASSERT(priv);
  wdinfo("Entry: handler=%p\n", handler);

  /* Get the old handler to return it */

  oldhandler = priv->handler;

  ESP32_WDT_UNLOCK(priv->wdt);

  flags = spin_lock_irqsave(&priv->lock);

  /* Save the new user handler */

  priv->handler = handler;

  /* There is a user callback and the timer has already been started.
   * The user wants to set a callback after starting the wdt or wants to
   * change the callback function once a callback has already been settled.
   */

  if (priv->handler != NULL && priv->started == true)
    {
      /* Deallocate the previous allocated interrupt
       * If there is a previous allocated interrupt.
       */

      if (oldhandler != NULL)
        {
          ESP32_WDT_SETISR(priv->wdt, NULL, NULL);
        }
      else
        {
          /* If it was previous configured to reset on timeout
           * then change to interrupt.
           */

          ESP32_WDT_STG_CONF(priv->wdt, STAGE_0, INTERRUPT_ON_TIMEOUT);
        }

      /* Set the lower half handler and enable interrupt */

      ESP32_WDT_SETISR(priv->wdt, esp32_wdt_handler, priv);
      ESP32_WDT_ENABLEINT(priv->wdt);
    }

  /* In case the user wants to disable the callback */

  else
    {
      ESP32_WDT_DISABLEINT(priv->wdt);
      ESP32_WDT_SETISR(priv->wdt, NULL, NULL);

      /* Then configure it to reset on wdt expiration */

      if (priv->peripheral == TIMER)
        {
          ESP32_WDT_STG_CONF(priv->wdt, STAGE_0, RESET_SYSTEM_TIMER);
        }
      else
        {
          ESP32_WDT_STG_CONF(priv->wdt, STAGE_0, RESET_SYSTEM_RTC);
        }
    }

  spin_unlock_irqrestore(&priv->lock, flags);
  ESP32_WDT_LOCK(priv->wdt);
  return oldhandler;
}

/* Interrupt handling *******************************************************/

static int    esp32_wdt_handler(int irq, void *context, void *arg)
{
  struct esp32_wdt_lowerhalf_s *priv = arg;

  ESP32_WDT_UNLOCK(priv->wdt);

  /* Updates last reset var and feed the dog to reload the counter and
   * to allow the application to continue executing.
   */
#ifdef CONFIG_DEBUG_WATCHDOG
  priv->lastreset = clock_systime_ticks();
#endif
  ESP32_WDT_FEED(priv->wdt);

  /* Run the user callback */

  priv->handler(irq, context, priv->upper);

  ESP32_WDT_ACKINT(priv->wdt);        /* Clear the Interrupt */
  ESP32_WDT_LOCK(priv->wdt);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_wdt_initialize
 *
 * Description:
 *   Initialize the WDT watchdog timer.  The watchdog timer is initialized
 *   and registered as 'devpath'.
 *
 * Input Parameters:
 *   devpath                 - The full path to the watchdog.  This should
 *                             be of the form/dev/watchdog0
 *   watchdog timer instance - A number do indicate which one is being used
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp32_wdt_initialize(const char *devpath, uint8_t wdt)
{
  struct esp32_wdt_lowerhalf_s *lower = NULL;
  int                             ret = OK;

  DEBUGASSERT(devpath);

  switch (wdt)
    {
#ifdef CONFIG_ESP32_MWDT0
      case 0:
        {
          lower = &g_esp32_mwdt0_lowerhalf;
          lower->peripheral = TIMER;
          break;
        }
#endif

#ifdef CONFIG_ESP32_MWDT1
      case 1:
        {
          lower = &g_esp32_mwdt1_lowerhalf;
          lower->peripheral = TIMER;
          break;
        }
#endif

#ifdef CONFIG_ESP32_RWDT
      case 2:
        {
          lower = &g_esp32_rwdt_lowerhalf;
          lower->peripheral = RTC;
          break;
        }
#endif

      default:
        {
          ret = -ENODEV;
          goto errout;
        }
    }

  /* Initialize the elements of lower half state structure */

  lower->handler = NULL;
  lower->timeout = 0;
  lower->wdt     = esp32_wdt_init(wdt);

  if (lower->wdt == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  lower->started = esp32_wdt_is_running(lower->wdt);

  ESP32_WDT_UNLOCK(lower->wdt);

  /* If it is a Main System Watchdog Timer configure the Prescale to
   * have a 500us period.
   */

  if (lower->peripheral == TIMER)
    {
      ESP32_WDT_PRE(lower->wdt, PRE_VALUE);
    }

  ESP32_WDT_LOCK(lower->wdt);

  /* Register the watchdog driver as /dev/watchdogX. If the registration goes
   * right the returned value from watchdog_register is a pointer to
   * watchdog_upperhalf_s that can be either used with watchdog_unregister()
   * or with the handler's arg.
   */

  lower->upper = watchdog_register(devpath,
                             (struct watchdog_lowerhalf_s *)lower);
  if (lower->upper == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the watchdog driver (such as if the
       * 'devpath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      ret = -EEXIST;
      goto errout;
    }

errout:
  return ret;
}
