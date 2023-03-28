/****************************************************************************
 * arch/risc-v/src/espressif/esp_wdt.c
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

#include <assert.h>
#include <debug.h>
#include <stdbool.h>

#include <nuttx/timers/watchdog.h>

#include "esp_irq.h"
#include "esp_wdt.h"

#include "hal/mwdt_ll.h"
#include "hal/wdt_hal.h"
#include "periph_ctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MWDT clock period in microseconds */

#define MWDT_CLK_PERIOD_US        (500)

/* Number of MWDT cycles per microseconds */

#define MWDT_CYCLES_PER_MS        (USEC_PER_MSEC / MWDT_CLK_PERIOD_US)

/* Convert MWDT timeout cycles to milliseconds */

#define MWDT_TIMEOUT_MS(t)        ((t) * MWDT_CYCLES_PER_MS)

/* Maximum number of MWDT cycles supported for timeout */

#define MWDT_MAX_TIMEOUT_MS       (UINT32_MAX / MWDT_CYCLES_PER_MS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure. This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct esp_wdt_lowerhalf_s
{
  const struct watchdog_ops_s *ops;       /* Lower half operations */
  uint32_t                     timeout;   /* The current timeout */
  wdt_stage_action_t           action;    /* The current action */
  uint32_t                     lastreset; /* The last reset time */
  bool                         started;   /* True: Timer has been started */
  xcpt_t                       handler;   /* User Handler */
  void                        *upper;     /* Pointer to watchdog_upperhalf_s */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt handling *******************************************************/

static int    wdt_handler(int irq, void *context, void *arg);

/* "Lower half" driver methods **********************************************/

static int    wdt_start(struct watchdog_lowerhalf_s *lower);
static int    wdt_stop(struct watchdog_lowerhalf_s *lower);
static int    wdt_keepalive(struct watchdog_lowerhalf_s *lower);
static int    wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                            struct watchdog_status_s *status);
static int    wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                             uint32_t timeout);
static xcpt_t wdt_capture(struct watchdog_lowerhalf_s *lower,
                          xcpt_t handler);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_esp_wdg_ops =
{
  .start      = wdt_start,
  .stop       = wdt_stop,
  .keepalive  = wdt_keepalive,
  .getstatus  = wdt_getstatus,
  .settimeout = wdt_settimeout,
  .capture    = wdt_capture,
  .ioctl      = NULL
};

/* MWDT0 lower-half */

static struct esp_wdt_lowerhalf_s g_esp_wdt_lowerhalf =
{
  .ops = &g_esp_wdg_ops,
  .timeout = MWDT_MAX_TIMEOUT_MS
};

/* Watchdog HAL context */

static wdt_hal_context_t wdt_hal_ctx;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wdt_start
 *
 * Description:
 *   Start the watchdog timer, register a callback if there is one and
 *   enables interrupt, otherwise, configure it to reset system on
 *   expiration.
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int wdt_start(struct watchdog_lowerhalf_s *lower)
{
  struct esp_wdt_lowerhalf_s *priv = (struct esp_wdt_lowerhalf_s *)lower;
  int ret = OK;
  irqstate_t flags;

  wdinfo("Entry: started\n");

  DEBUGASSERT(priv != NULL);

  if (priv->started)
    {
      /* Return EBUSY to indicate that the timer was already running */

      return -EBUSY;
    }

  priv->started = true;

  wdt_hal_write_protect_disable(&wdt_hal_ctx);

  if (priv->handler == NULL)
    {
      /* No user handler, so configure WDT to reset on timeout */

      priv->action = WDT_STAGE_ACTION_RESET_SYSTEM;

      wdt_hal_config_stage(&wdt_hal_ctx, WDT_STAGE0,
                           MWDT_TIMEOUT_MS(priv->timeout),
                           priv->action);
    }
  else
    {
      /* Configure WDT to call the user handler on timeout */

      priv->action = WDT_STAGE_ACTION_INT;

      wdt_hal_config_stage(&wdt_hal_ctx, WDT_STAGE0,
                           MWDT_TIMEOUT_MS(priv->timeout),
                           priv->action);

      /* Enable interrupt */

      mwdt_ll_set_intr_enable(wdt_hal_ctx.mwdt_dev, true);
    }

  flags = enter_critical_section();
  priv->lastreset = clock_systime_ticks();
  wdt_hal_enable(&wdt_hal_ctx);
  leave_critical_section(flags);

  wdt_hal_write_protect_enable(&wdt_hal_ctx);

  return ret;
}

/****************************************************************************
 * Name: wdt_stop
 *
 * Description:
 *   Stop the watchdog timer. In case a callback was previously configured,
 *   unregister and deallocate it.
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int wdt_stop(struct watchdog_lowerhalf_s *lower)
{
  struct esp_wdt_lowerhalf_s *priv = (struct esp_wdt_lowerhalf_s *)lower;

  wdt_hal_write_protect_disable(&wdt_hal_ctx);

  /* Disable the WDT */

  wdt_hal_disable(&wdt_hal_ctx);

  /* In case there is a callback registered, ensure WDT interrupts are
   * disabled.
   */

  if (priv->handler != NULL)
    {
      mwdt_ll_set_intr_enable(wdt_hal_ctx.mwdt_dev, false);
    }

  wdt_hal_write_protect_enable(&wdt_hal_ctx);

  priv->started = false;

  return OK;
}

/****************************************************************************
 * Name: wdt_keepalive
 *
 * Description:
 *   Reset the watchdog timer, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int wdt_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct esp_wdt_lowerhalf_s *priv = (struct esp_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  wdt_hal_write_protect_disable(&wdt_hal_ctx);

  /* Feed the dog and update the time of last reset */

  flags = enter_critical_section();
  priv->lastreset = clock_systime_ticks();
  wdt_hal_feed(&wdt_hal_ctx);
  leave_critical_section(flags);

  wdt_hal_write_protect_enable(&wdt_hal_ctx);

  return OK;
}

/****************************************************************************
 * Name: wdt_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *   status        - The location to return the watchdog status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                         struct watchdog_status_s *status)
{
  struct esp_wdt_lowerhalf_s *priv = (struct esp_wdt_lowerhalf_s *)lower;
  uint32_t ticks;
  uint32_t elapsed;

  DEBUGASSERT(priv != NULL);

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
 * Name: wdt_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *   timeout       - The new timeout value in milliseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                          uint32_t timeout)
{
  struct esp_wdt_lowerhalf_s *priv = (struct esp_wdt_lowerhalf_s *)lower;
  uint16_t rtc_cycles = 0;
  uint32_t rtc_ms_max = 0;

  wdinfo("Entry: timeout=%" PRIu32 "\n", timeout);

  DEBUGASSERT(priv != NULL);

  wdt_hal_write_protect_disable(&wdt_hal_ctx);

  priv->timeout = timeout;

  if (timeout == 0 || timeout > MWDT_MAX_TIMEOUT_MS)
    {
      wderr("ERROR: Cannot represent timeout=%" PRIu32 " > %" PRIu32 "\n",
            timeout, MWDT_MAX_TIMEOUT_MS);
      return -ERANGE;
    }

  wdt_hal_config_stage(&wdt_hal_ctx, WDT_STAGE0,
                       MWDT_TIMEOUT_MS(priv->timeout),
                       priv->action);

  wdt_hal_feed(&wdt_hal_ctx);

  wdt_hal_write_protect_enable(&wdt_hal_ctx);

  return OK;
}

/****************************************************************************
 * Name: wdt_capture
 *
 * Description:
 *   Don't reset on watchdog timer timeout; instead, call this user provider
 *   timeout handler.  NOTE:  Providing handler==NULL will restore the reset
 *   behavior.
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *   handler       - The new watchdog expiration function pointer. If this
 *                   function pointer is NULL, then the reset-on-expiration
 *                   behavior is restored.
 *
 * Returned Value:
 *   The previous watchdog expiration function pointer or NULL if there was
 *   no previous function pointer, i.e., if the previous behavior was
 *   reset-on-expiration (NULL is also returned if an error occurs).
 *
 ****************************************************************************/

static xcpt_t wdt_capture(struct watchdog_lowerhalf_s *lower, xcpt_t handler)
{
  struct esp_wdt_lowerhalf_s *priv = (struct esp_wdt_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;

  DEBUGASSERT(priv != NULL);

  wdinfo("Entry: handler=0x%" PRIxPTR "\n", (uintptr_t)handler);

  oldhandler = priv->handler;

  wdt_hal_write_protect_disable(&wdt_hal_ctx);

  flags = enter_critical_section();

  /* Save the new user handler */

  priv->handler = handler;

  /* There is a user callback and the timer has already been started.
   * The user wants to set a callback after starting the WDT or wants to
   * change the callback function once a callback has already been settled.
   */

  if (priv->handler != NULL && priv->started)
    {
      if (oldhandler == NULL)
        {
          /* If it was previous configured to reset on timeout
           * then change to interrupt.
           */

          priv->action = WDT_STAGE_ACTION_INT;

          wdt_hal_config_stage(&wdt_hal_ctx, WDT_STAGE0,
                               MWDT_TIMEOUT_MS(priv->timeout),
                               priv->action);
        }

      mwdt_ll_set_intr_enable(wdt_hal_ctx.mwdt_dev, true);
    }

  /* In case the user wants to disable the callback */

  else
    {
      mwdt_ll_set_intr_enable(wdt_hal_ctx.mwdt_dev, false);

      /* Then configure it to reset on WDT expiration */

      priv->action = WDT_STAGE_ACTION_RESET_SYSTEM;

      wdt_hal_config_stage(&wdt_hal_ctx, WDT_STAGE0,
                           MWDT_TIMEOUT_MS(priv->timeout),
                           priv->action);
    }

  leave_critical_section(flags);

  wdt_hal_write_protect_enable(&wdt_hal_ctx);

  return oldhandler;
}

/****************************************************************************
 * Name: wdt_handler
 *
 * Description:
 *   This is the WDT interrupt handler. It will be invoked when an
 *   interrupt is received on the device.
 *
 * Input Parameters:
 *   irq           - IRQ associated to that interrupt.
 *   context       - Interrupt register state save info.
 *   arg           - A pointer to the argument provided when the interrupt
 *                   was registered.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int wdt_handler(int irq, void *context, void *arg)
{
  struct esp_wdt_lowerhalf_s *priv = (struct esp_wdt_lowerhalf_s *)arg;

  /* Run the user callback */

  priv->handler(irq, context, priv->upper);

  /* Clear the Interrupt */

  wdt_hal_write_protect_disable(&wdt_hal_ctx);
  wdt_hal_handle_intr(&wdt_hal_ctx);
  wdt_hal_write_protect_enable(&wdt_hal_ctx);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_wdt_initialize
 *
 * Description:
 *   Initialize the watchdog timer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp_wdt_initialize(void)
{
  periph_module_enable(PERIPH_TIMG0_MODULE);
  wdt_hal_init(&wdt_hal_ctx, WDT_MWDT0, MWDT_LL_DEFAULT_CLK_PRESCALER, true);

  struct esp_wdt_lowerhalf_s *lower = &g_esp_wdt_lowerhalf;

  /* Initialize the elements of lower half state structure */

  lower->handler = NULL;
  lower->timeout = 0;
  lower->started = wdt_hal_is_enabled(&wdt_hal_ctx);

  /* Register the watchdog driver as /dev/watchdogX. If the registration goes
   * right the returned value from watchdog_register is a pointer to
   * watchdog_upperhalf_s that can be either used with watchdog_unregister()
   * or with the handler's arg.
   */

  lower->upper = watchdog_register(CONFIG_WATCHDOG_DEVPATH,
                                   (struct watchdog_lowerhalf_s *)lower);
  if (lower->upper == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the watchdog driver (such as if the
       * 'devpath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      return -EEXIST;
    }

  esp_setup_irq(TG0_WDT_LEVEL_INTR_SOURCE,
                ESP_IRQ_PRIORITY_DEFAULT,
                ESP_IRQ_TRIGGER_LEVEL);

  /* Attach the handler for the timer IRQ */

  irq_attach(ESP_IRQ_TG0_WDT_LEVEL, (xcpt_t)wdt_handler, lower);

  /* Enable the allocated CPU interrupt */

  up_enable_irq(ESP_IRQ_TG0_WDT_LEVEL);

  return OK;
}
