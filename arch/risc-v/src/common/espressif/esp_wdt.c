/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_wdt.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include "riscv_internal.h"

#include "esp_irq.h"
#include "esp_wdt.h"
#include "esp_clk.h"
#include "esp_rtc_gpio.h"

#include "hal/mwdt_ll.h"
#include "hal/rwdt_ll.h"
#ifdef CONFIG_ESPRESSIF_XTWDT
#include "hal/xt_wdt_ll.h"
#include "hal/xt_wdt_hal.h"
#include "esp_xt_wdt.h"
#include "esp_rom_sys.h"
#endif
#include "hal/wdt_hal.h"
#include "soc/rtc.h"
#include "periph_ctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MWDT clock period in microseconds */

#define MWDT_CLK_PERIOD_US               (500)

/* Number of MWDT cycles per microseconds */

#define MWDT_CYCLES_PER_MS               (USEC_PER_MSEC / MWDT_CLK_PERIOD_US)

/* Convert MWDT timeout cycles to milliseconds */

#define MWDT_TIMEOUT_MS(t)               ((t) * MWDT_CYCLES_PER_MS)

/* Maximum number of MWDT cycles supported for timeout */

#define MWDT_MAX_TIMEOUT_MS              (UINT32_MAX / MWDT_CYCLES_PER_MS)

/* Maximum number of cycles supported for a RWDT stage timeout */

#define RWDT_FULL_STAGE                  (UINT32_MAX)

/* Convert RWDT timeout cycles to milliseconds */

#define RWDT_TIMEOUT_MS(t)               (t * rtc_clk_slow_freq_get_hz() / 1000ULL)

#define WDT_INTR_ENABLE(timer, ctx, en)  (timer == RTC ?                               \
                                          rwdt_ll_set_intr_enable(ctx->rwdt_dev, en) : \
                                          mwdt_ll_set_intr_enable(ctx->mwdt_dev, en))

#define WDT_WP_DISABLE(dev)   do{ if (IS_XTWDT(dev) != true)                 \
                                    wdt_hal_write_protect_disable(dev->ctx); \
                                } while(0);

#define WDT_WP_ENABLE(dev)    do{ if (IS_XTWDT(dev) != true)                \
                                    wdt_hal_write_protect_enable(dev->ctx); \
                                } while(0);

/* Check whether the provided device is a XTAL32K Watchdog Timer */

#define IS_XTWDT(dev)    (((struct esp_wdt_lowerhalf_s *)dev)->peripheral == XTAL32K)

/* XTWDT clock period in nanoseconds */

#define XTWDT_CLK_PERIOD_NS        (30)

/* Maximum number of cycles supported for a XTWDT stage timeout */

#define XTWDT_FULL_STAGE          (UINT8_MAX)

/* Number of cycles for RTC_SLOW_CLK calibration */

#define XT_WDT_CLK_CAL_CYCLES     (500)

/* Helpers for converting from Q13.19 fixed-point format to float */

#define N             19
#define Q_TO_FLOAT(x) ((float)x/(float)(1<<N))

#if defined(CONFIG_ARCH_CHIP_ESP32C6) || defined(CONFIG_ARCH_CHIP_ESP32H2)
#define RTC_CORE_INTR_SOURCE  LP_WDT_INTR_SOURCE
#define ESP_IRQ_RTC_CORE      ESP_IRQ_LP_WDT
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum
{
    RTC,
    TIMER,
    XTAL32K,
} wdt_peripherals_t;

/* This structure provides the private representation of the "lower-half"
 * driver state structure. This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct esp_wdt_lowerhalf_s
{
  const struct watchdog_ops_s *ops;        /* Lower half operations */
  uint32_t                     timeout;    /* The current timeout */
  wdt_stage_action_t           action;     /* The current action */
  uint32_t                     lastreset;  /* The last reset time */
  bool                         started;    /* True: Timer has been started */
  wdt_peripherals_t            peripheral; /* Indicates if it is from RTC or Timer Module */
  xcpt_t                       handler;    /* User Handler */
  void                        *upper;      /* Pointer to watchdog_upperhalf_s */
  uint8_t                      periph;     /* Peripheral ID */
  uint8_t                      irq;        /* Interrupt ID */
  wdt_hal_context_t           *ctx;        /* Watchdog HAL context */
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
static int wdt_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                     unsigned long arg);
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
  .ioctl      = wdt_ioctl
};

#ifdef CONFIG_ESPRESSIF_MWDT0
/* Watchdog HAL context */

static wdt_hal_context_t mwdt0_hal_ctx;

/* MWDT0 lower-half */

static struct esp_wdt_lowerhalf_s g_esp_mwdt0_lowerhalf =
{
  .ops = &g_esp_wdg_ops,
  .timeout = MWDT_MAX_TIMEOUT_MS,
  .periph = TG0_WDT_LEVEL_INTR_SOURCE,
  .peripheral = TIMER,
  .irq = ESP_IRQ_TG0_WDT_LEVEL,
  .ctx = &mwdt0_hal_ctx
};
#endif

#ifdef CONFIG_ESPRESSIF_MWDT1
/* Watchdog HAL context */

static wdt_hal_context_t mwdt1_hal_ctx;

/* MWDT1 lower-half */

static struct esp_wdt_lowerhalf_s g_esp_mwdt1_lowerhalf =
{
  .ops = &g_esp_wdg_ops,
  .timeout = MWDT_MAX_TIMEOUT_MS,
  .periph = TG1_WDT_LEVEL_INTR_SOURCE,
  .peripheral = TIMER,
  .irq = ESP_IRQ_TG1_WDT_LEVEL,
  .ctx = &mwdt1_hal_ctx
};
#endif

#ifdef CONFIG_ESPRESSIF_RWDT
/* Watchdog HAL context */

static wdt_hal_context_t rwdt_hal_ctx = RWDT_HAL_CONTEXT_DEFAULT();

/* RWDT lower-half */

static struct esp_wdt_lowerhalf_s g_esp_rwdt_lowerhalf =
{
  .ops = &g_esp_wdg_ops,
  .timeout = RWDT_FULL_STAGE,
  .periph = RTC_CORE_INTR_SOURCE,
  .peripheral = RTC,
  .irq  = ESP_IRQ_RTC_CORE,
  .ctx = &rwdt_hal_ctx
};
#endif

#ifdef CONFIG_ESPRESSIF_XTWDT
static xt_wdt_hal_context_t xtwdt_hal_ctx;
static esp_xt_wdt_config_t cfg =
  {
    .timeout = CONFIG_ESPRESSIF_XTWDT_TIMEOUT,
#ifdef CONFIG_ESPRESSIF_XTWDT_BACKUP_CLK_ENABLE
    .auto_backup_clk_enable = true
#else
    .auto_backup_clk_enable = false
#endif
  };

struct esp_wdt_lowerhalf_s g_esp_xtwdt_lowerhalf =
{
  .ops    = &g_esp_wdg_ops,
  .periph = RTC_CORE_INTR_SOURCE,
  .peripheral = XTAL32K,
  .irq    = ESP_IRQ_RTC_XTAL32K_DEAD,
  .started = false,
  .ctx = (wdt_hal_context_t *)&xtwdt_hal_ctx
};
#endif

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
  uint32_t timeout;

  wdinfo("Entry: started\n");

  DEBUGASSERT(priv != NULL);

  if (priv->started)
    {
      /* Return EBUSY to indicate that the timer was already running */

      return -EBUSY;
    }

  priv->started = true;

  WDT_WP_DISABLE(priv);

  if (priv->handler == NULL)
    {
      /* No user handler, so configure WDT to reset on timeout */

      if (IS_XTWDT(priv) != true)
        {
          if (priv->peripheral == TIMER)
            {
              priv->action = WDT_STAGE_ACTION_RESET_SYSTEM;
              timeout = MWDT_TIMEOUT_MS(priv->timeout);
            }
          else
            {
              priv->action = WDT_STAGE_ACTION_RESET_RTC;
              timeout = RWDT_TIMEOUT_MS(priv->timeout);
            }

          wdt_hal_config_stage(priv->ctx, WDT_STAGE0,
                              timeout,
                              priv->action);
        }
    }
  else
    {
      /* Configure WDT to call the user handler on timeout */

      if (IS_XTWDT(priv) != true)
        {
          if (priv->peripheral == TIMER)
            {
              priv->action = WDT_STAGE_ACTION_INT;
              timeout = MWDT_TIMEOUT_MS(priv->timeout);
            }
          else
            {
              priv->action = WDT_STAGE_ACTION_INT;
              timeout = RWDT_TIMEOUT_MS(priv->timeout);
            }

          wdt_hal_config_stage(priv->ctx, WDT_STAGE0,
                              timeout,
                              priv->action);

          /* Enable interrupt */

          WDT_INTR_ENABLE(priv->peripheral, priv->ctx, true);
        }
#ifdef CONFIG_ESPRESSIF_XTWDT
      else
        {
          xt_wdt_ll_intr_enable(((xt_wdt_hal_context_t *)priv->ctx)->dev,
                                true);
        }
#endif
    }

  flags = enter_critical_section();
  priv->lastreset = clock_systime_ticks();

  if (IS_XTWDT(priv) != true)
    {
      wdt_hal_enable(priv->ctx);
    }
#ifdef CONFIG_ESPRESSIF_XTWDT
  else
    {
      xt_wdt_hal_enable((xt_wdt_hal_context_t *)priv->ctx, true);
    }
#endif

  leave_critical_section(flags);
  WDT_WP_ENABLE(priv);
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

  WDT_WP_DISABLE(priv);

  if (IS_XTWDT(priv) != true)
    {
      /* Disable the WDT */

      wdt_hal_disable(priv->ctx);

      /* In case there is a callback registered, ensure WDT interrupts are
       * disabled.
       */

      if (priv->handler != NULL)
        {
          WDT_INTR_ENABLE(priv->peripheral, priv->ctx, false);
        }
    }
#ifdef CONFIG_ESPRESSIF_XTWDT
  else
    {
      xt_wdt_hal_enable((xt_wdt_hal_context_t *)priv->ctx, false);
    }
#endif

  WDT_WP_ENABLE(priv);
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

  if (IS_XTWDT(priv) == true)
    {
      return OK;
    }

  wdt_hal_write_protect_disable(priv->ctx);

  /* Feed the dog and update the time of last reset */

  flags = enter_critical_section();
  priv->lastreset = clock_systime_ticks();
  wdt_hal_feed(priv->ctx);
  leave_critical_section(flags);

  wdt_hal_write_protect_enable(priv->ctx);

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
  uint32_t xtal32k_cycles = 0;
  uint32_t xtal32k_ms_max = 0;

  wdinfo("Entry: timeout=%" PRIu32 "\n", timeout);

  DEBUGASSERT(priv != NULL);

  priv->timeout = timeout;

  if (IS_XTWDT(priv) != true)
    {
      wdt_hal_write_protect_disable(priv->ctx);
      if (priv->peripheral == TIMER)
        {
          if (timeout == 0 || timeout > MWDT_MAX_TIMEOUT_MS)
            {
              wderr("ERROR: Cannot represent timeout=%"PRIu32"> %"PRIu32"\n",
                    timeout, MWDT_MAX_TIMEOUT_MS);
              return -ERANGE;
            }

          timeout = MWDT_TIMEOUT_MS(priv->timeout);
        }
      else if (priv->peripheral == RTC)
        {
          uint32_t period_13q19 = esp_clk_slowclk_cal_get();
          float period = Q_TO_FLOAT(period_13q19);
          rtc_cycles = 1000.0f / period;
          rtc_ms_max = (RWDT_FULL_STAGE / (uint32_t)rtc_cycles);

          /* Is this timeout a valid value for RTC WDT? */

          if (timeout == 0 || timeout > rtc_ms_max)
            {
              wderr("ERROR: Cannot represent timeout=%"PRIu32"> %"PRIu32"\n",
                    timeout, rtc_ms_max);
              return -ERANGE;
            }

          timeout = timeout * rtc_cycles;
        }

      wdt_hal_config_stage(priv->ctx, WDT_STAGE0,
                          timeout,
                          priv->action);

      wdt_hal_feed(priv->ctx);

      wdt_hal_write_protect_enable(priv->ctx);
    }
#ifdef CONFIG_ESPRESSIF_XTWDT
  else
    {
      xtal32k_cycles =  XTWDT_CLK_PERIOD_NS;
      xtal32k_ms_max = (XTWDT_FULL_STAGE * NSEC_PER_USEC
                        / (uint32_t)xtal32k_cycles);

      /* Is this timeout a valid value for RTC WDT? */

      if (timeout == 0 || timeout > xtal32k_ms_max)
        {
          wderr("Cannot represent timeout=%" PRIu32 " > %" PRIu32 "\n",
                timeout, rtc_ms_max);
          return -ERANGE;
        }

      timeout = timeout * xtal32k_cycles;
      xt_wdt_ll_set_timeout(((xt_wdt_hal_context_t *)priv->ctx)->dev,
                             timeout);
      return OK;
    }
#endif

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
  uint32_t timeout;

  DEBUGASSERT(priv != NULL);

  wdinfo("Entry: handler=%p\n", handler);

  oldhandler = priv->handler;

  WDT_WP_DISABLE(priv);

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

          if (IS_XTWDT(priv) != true)
            {
              if (priv->peripheral == TIMER)
                {
                  timeout = MWDT_TIMEOUT_MS(priv->timeout);
                }
              else
                {
                  timeout = RWDT_TIMEOUT_MS(priv->timeout);
                }

              priv->action = WDT_STAGE_ACTION_INT;

              wdt_hal_config_stage(priv->ctx, WDT_STAGE0,
                                  timeout,
                                  priv->action);
            }
        }

      if (IS_XTWDT(priv) != true)
        {
          WDT_INTR_ENABLE(priv->peripheral, priv->ctx, true);
        }
#ifdef CONFIG_ESPRESSIF_XTWDT
      else
        {
      xt_wdt_hal_enable((xt_wdt_hal_context_t *)priv->ctx, true);
        }
#endif
    }

  /* In case the user wants to disable the callback */

  else
    {
      if (IS_XTWDT(priv) != true)
        {
          if (priv->peripheral == TIMER)
            {
              timeout = MWDT_TIMEOUT_MS(priv->timeout);
              priv->action = WDT_STAGE_ACTION_RESET_SYSTEM;
            }
          else
            {
              timeout = RWDT_TIMEOUT_MS(priv->timeout);
              priv->action = WDT_STAGE_ACTION_RESET_RTC;
            }

          WDT_INTR_ENABLE(priv->peripheral, priv->ctx, false);

          /* Then configure it to reset on WDT expiration */

          wdt_hal_config_stage(priv->ctx, WDT_STAGE0,
                              timeout,
                              priv->action);
        }
#ifdef CONFIG_ESPRESSIF_XTWDT
      else
        {
          xt_wdt_hal_enable((xt_wdt_hal_context_t *)priv->ctx, false);
        }
#endif
    }

  leave_critical_section(flags);

  WDT_WP_ENABLE(priv);

  return oldhandler;
}

/****************************************************************************
 * Name: wdt_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *   cmd   - Command number to process.
 *   arg   - Argument that sent to the command.
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

static int wdt_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                     unsigned long arg)
{
  struct esp_wdt_lowerhalf_s *priv = (struct esp_wdt_lowerhalf_s *)lower;

  wdinfo("ioctl Call: cmd=0x%x arg=0x%lx", cmd, arg);

  /* Process the IOCTL command */

  switch (cmd)
    {
    case WDIOC_RSTCLK:
#ifdef CONFIG_ESPRESSIF_XTWDT
      if (IS_XTWDT(priv) == true)
        {
          xt_wdt_hal_enable((xt_wdt_hal_context_t *)priv->ctx, false);

          REG_CLR_BIT(RTC_CNTL_EXT_XTL_CONF_REG, RTC_CNTL_XPD_XTAL_32K);
          REG_SET_BIT(RTC_CNTL_EXT_XTL_CONF_REG, RTC_CNTL_XPD_XTAL_32K);

          /* Needs some time after switching to 32khz XTAL
          * before turning on WDT again
          */

          esp_rom_delay_us(300);

          xt_wdt_hal_enable((xt_wdt_hal_context_t *)priv->ctx, true);
        }
#endif
      break;

    default:
      return -ENOTTY;
    }

  return OK;
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

  WDT_WP_DISABLE(priv);

  /* Clear the Interrupt */

  if (IS_XTWDT(priv) != true)
    {
      wdt_hal_handle_intr(priv->ctx);
    }

  WDT_WP_ENABLE(priv);
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
 *   devpath - The full path to the watchdog.  This should
 *             be of the form /dev/watchdogX
 *   wdt_id  - A Watchdog Timer instance to be initialized.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp_wdt_initialize(const char *devpath, enum esp_wdt_inst_e wdt_id)
{
  struct esp_wdt_lowerhalf_s *lower = NULL;

  switch (wdt_id)
    {
#ifdef CONFIG_ESPRESSIF_MWDT0
      case ESP_WDT_MWDT0:
        {
          lower = &g_esp_mwdt0_lowerhalf;
          periph_module_enable(PERIPH_TIMG0_MODULE);
          wdt_hal_init(lower->ctx, WDT_MWDT0,
                       MWDT_LL_DEFAULT_CLK_PRESCALER, true);

          break;
        }

#endif

#ifdef CONFIG_ESPRESSIF_MWDT1
      case ESP_WDT_MWDT1:
        {
          lower = &g_esp_mwdt1_lowerhalf;
          periph_module_enable(PERIPH_TIMG1_MODULE);
          wdt_hal_init(lower->ctx, WDT_MWDT1,
                       MWDT_LL_DEFAULT_CLK_PRESCALER, true);

          break;
        }

#endif

#ifdef CONFIG_ESPRESSIF_RWDT
      case ESP_WDT_RWDT:
        {
          lower = &g_esp_rwdt_lowerhalf;
          wdt_hal_init(lower->ctx, WDT_RWDT, 0, true);
          esp_rtcioirqenable(lower->irq);
          break;
        }
#endif

#ifdef CONFIG_ESPRESSIF_XTWDT
      case ESP_WDT_XTAL32K:
        {
          lower = &g_esp_xtwdt_lowerhalf;
          esp_rtcioirqenable(lower->irq);
          xt_wdt_hal_init((xt_wdt_hal_context_t *)lower->ctx,
                          (xt_wdt_hal_config_t *)&cfg);
          if (cfg.auto_backup_clk_enable)
            {
              uint32_t rtc_clk_frequency_khz = rtc_clk_freq_cal(
                          rtc_clk_cal(RTC_CAL_INTERNAL_OSC, 500)) / 1000;
              xt_wdt_hal_enable_backup_clk((xt_wdt_hal_context_t *)
                                            lower->ctx,
                                            rtc_clk_frequency_khz);
            }
          break;
        }
#endif

      default:
        {
          wderr("ERROR: unsupported WDT %d\n", wdt_id);
          return ERROR;
        }
    }

  /* Initialize the elements of lower half state structure */

  lower->handler = NULL;
  lower->timeout = 0;

  if (IS_XTWDT(lower) != true)
    {
      lower->started = wdt_hal_is_enabled(lower->ctx);
    }

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

      return -EEXIST;
    }

  esp_setup_irq(lower->periph,
                ESP_IRQ_PRIORITY_DEFAULT,
                ESP_IRQ_TRIGGER_LEVEL);

  /* Attach the handler for the timer IRQ */

  irq_attach(lower->irq, (xcpt_t)wdt_handler, lower);

  /* Enable the allocated CPU interrupt */

  if (IS_XTWDT(lower) != true)
    {
      up_enable_irq(lower->irq);
    }
  else
    {
      up_enable_irq(ESP_IRQ_RTC_CORE);
    }

  return OK;
}
