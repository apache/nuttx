/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_wdt_lowerhalf.c
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
#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/watchdog.h>

#include "xtensa.h"
#include "esp32s3_wdt.h"
#include "esp32s3_rtc.h"
#include "esp32s3_wdt_lowerhalf.h"
#include "hardware/esp32s3_soc.h"

#include "soc/periph_defs.h"
#include "esp_private/periph_ctrl.h"

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

/* MWDT clock prescaler value */

#define MWDT_CLK_PRESCALER_VALUE  (MWDT_CLK_PERIOD_US * NSEC_PER_USEC / 12.5)

/* Maximum number of cycles supported for a RWDT stage timeout */

#define RWDT_FULL_STAGE           (UINT32_MAX)

/* XTWDT clock period in nanoseconds */

#define XTWDT_CLK_PERIOD_NS        (30)

/* Maximum number of cycles supported for a XTWDT stage timeout */

#define XTWDT_FULL_STAGE          (UINT8_MAX)

/* Number of cycles for RTC_SLOW_CLK calibration */

#define XT_WDT_CLK_CAL_CYCLES     (500)

/* Maximum number of divisor components
 * according to the frequency of RC_SLOW_CLK
 */

#define XT_WDT_DIV_COMP_N_MAX     8

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum wdt_peripheral_e
{
  RTC,
  TIMER,
  XTAL32K,
};

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct esp32s3_wdt_lowerhalf_s
{
  const struct watchdog_ops_s *ops;        /* Lower-half operations */
  struct esp32s3_wdt_dev_s *wdt;           /* ESP32-S3 watchdog driver */
  uint32_t timeout;                        /* The current timeout */
  enum wdt_peripheral_e peripheral;        /* Indicates if it is from RTC or Timer Module */
  uint32_t lastreset;                      /* The last reset time */
  bool started;                            /* True: Timer has been started */
  xcpt_t handler;                          /* User Handler */
  void *upper;                             /* Pointer to watchdog_upperhalf_s */
  spinlock_t lock;                         /* Device-specific lock */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt handling *******************************************************/

static int    wdt_handler(int irq, void *context, void *arg);

/* "Lower-half" driver methods **********************************************/

static int    wdt_lh_start(struct watchdog_lowerhalf_s *lower);
static int    wdt_lh_stop(struct watchdog_lowerhalf_s *lower);
static int    wdt_lh_keepalive(struct watchdog_lowerhalf_s *lower);
static int    wdt_lh_getstatus(struct watchdog_lowerhalf_s *lower,
                               struct watchdog_status_s *status);
static int    wdt_lh_settimeout(struct watchdog_lowerhalf_s *lower,
                                uint32_t timeout);
static xcpt_t wdt_lh_capture(struct watchdog_lowerhalf_s *lower,
                             xcpt_t handler);
static int wdt_lh_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                        unsigned long arg);
#ifdef CONFIG_ESP32S3_XTWDT_BACKUP_CLK_ENABLE
static uint32_t wdt_lh_xt_calculate(uint32_t rtc_clk_frequency_khz);
static uint32_t wdt_lh_clk_freq_cal(uint32_t rtc_clk_period_us);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower-half" driver methods */

static const struct watchdog_ops_s g_esp32s3_wdg_ops =
{
  .start      = wdt_lh_start,
  .stop       = wdt_lh_stop,
  .keepalive  = wdt_lh_keepalive,
  .getstatus  = wdt_lh_getstatus,
  .settimeout = wdt_lh_settimeout,
  .capture    = wdt_lh_capture,
  .ioctl      = wdt_lh_ioctl,
};

#ifdef CONFIG_ESP32S3_MWDT0
/* MWDT0 lower-half */

static struct esp32s3_wdt_lowerhalf_s g_esp32s3_mwdt0_lowerhalf =
{
  .ops = &g_esp32s3_wdg_ops,
};
#endif

#ifdef CONFIG_ESP32S3_MWDT1
/* MWDT1 lower-half */

static struct esp32s3_wdt_lowerhalf_s g_esp32s3_mwdt1_lowerhalf =
{
  .ops = &g_esp32s3_wdg_ops,
};
#endif

#ifdef CONFIG_ESP32S3_RWDT
/* RWDT lower-half */

static struct esp32s3_wdt_lowerhalf_s g_esp32s3_rwdt_lowerhalf =
{
  .ops = &g_esp32s3_wdg_ops,
};
#endif

#ifdef CONFIG_ESP32S3_XTWDT
/* XTWDT lower-half */

static struct esp32s3_wdt_lowerhalf_s g_esp32s3_xtwdt_lowerhalf =
{
  .ops = &g_esp32s3_wdg_ops,
};
#endif

/****************************************************************************
 * Name: wdt_lh_start
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

static int wdt_lh_start(struct watchdog_lowerhalf_s *lower)
{
  struct esp32s3_wdt_lowerhalf_s *priv =
    (struct esp32s3_wdt_lowerhalf_s *)lower;

  wdinfo("Entry: wdt_lh_start\n");

  DEBUGASSERT(priv);

  if (priv->started == true)
    {
      /* Return EBUSY to indicate that the timer was already running */

      return -EBUSY;
    }

  /* If WDT was not started yet */

  else
    {
      irqstate_t flags;

      priv->started = true;

      /* Unlock WDT */

      ESP32S3_WDT_UNLOCK(priv->wdt);

      /* No User Handler */

      if (priv->handler == NULL)
        {
             /* Then configure it to reset on wdt expiration */

            if (priv->peripheral == TIMER)
              {
                ESP32S3_WDT_STG_CONF(priv->wdt, ESP32S3_WDT_STAGE0,
                                     ESP32S3_WDT_STAGE_ACTION_RESET_SYSTEM);
              }
            else if (priv->peripheral == RTC)
              {
                ESP32S3_WDT_STG_CONF(priv->wdt, ESP32S3_WDT_STAGE0,
                                     ESP32S3_WDT_STAGE_ACTION_RESET_RTC);
              }
        }

      /* User handler was already provided */

      else
        {
          /* Then configure it to call the user handler on wdt expiration */

          ESP32S3_WDT_STG_CONF(priv->wdt, ESP32S3_WDT_STAGE0,
                               ESP32S3_WDT_STAGE_ACTION_INT);

          /* Set the lower-half handler and enable interrupt */

          flags = spin_lock_irqsave(&priv->lock);
          ESP32S3_WDT_SETISR(priv->wdt, wdt_handler, priv);
          spin_unlock_irqrestore(&priv->lock, flags);
          ESP32S3_WDT_ENABLEINT(priv->wdt);
        }

      flags = spin_lock_irqsave(&priv->lock);
      priv->lastreset = clock_systime_ticks();
      ESP32S3_WDT_START(priv->wdt);
      spin_unlock_irqrestore(&priv->lock, flags);

      /* Lock it again */

      ESP32S3_WDT_LOCK(priv->wdt);
    }

    return OK;
}

/****************************************************************************
 * Name: wdt_lh_stop
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

static int wdt_lh_stop(struct watchdog_lowerhalf_s *lower)
{
  struct esp32s3_wdt_lowerhalf_s *priv =
    (struct esp32s3_wdt_lowerhalf_s *)lower;

  /* Unlock WDT */

  ESP32S3_WDT_UNLOCK(priv->wdt);

  /* Disable the WDT */

  ESP32S3_WDT_STOP(priv->wdt);

  /* In case there is some callback registered, disable and deallocate */

  if (priv->handler != NULL)
    {
      irqstate_t flags;

      ESP32S3_WDT_DISABLEINT(priv->wdt);

      flags = spin_lock_irqsave(&priv->lock);
      ESP32S3_WDT_SETISR(priv->wdt, NULL, NULL);
      spin_unlock_irqrestore(&priv->lock, flags);
    }

  /* Lock it again */

  ESP32S3_WDT_LOCK(priv->wdt);

  priv->started = false;

  return OK;
}

/****************************************************************************
 * Name: wdt_lh_keepalive
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

static int wdt_lh_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct esp32s3_wdt_lowerhalf_s *priv =
    (struct esp32s3_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry\n");

  /* Unlock */

  ESP32S3_WDT_UNLOCK(priv->wdt);

  /* Feed the dog and updates the lastreset variable */

  flags = spin_lock_irqsave(&priv->lock);
  priv->lastreset = clock_systime_ticks();
  ESP32S3_WDT_FEED(priv->wdt);
  spin_unlock_irqrestore(&priv->lock, flags);

  /* Lock */

  ESP32S3_WDT_LOCK(priv->wdt);

  return OK;
}

/****************************************************************************
 * Name: wdt_lh_getstatus
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

static int wdt_lh_getstatus(struct watchdog_lowerhalf_s *lower,
                            struct watchdog_status_s *status)
{
  struct esp32s3_wdt_lowerhalf_s *priv =
    (struct esp32s3_wdt_lowerhalf_s *)lower;
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
 * Name: wdt_lh_settimeout
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

static int wdt_lh_settimeout(struct watchdog_lowerhalf_s *lower,
                             uint32_t timeout)
{
  struct esp32s3_wdt_lowerhalf_s *priv =
    (struct esp32s3_wdt_lowerhalf_s *)lower;
  uint16_t rtc_cycles = 0;
  uint32_t rtc_ms_max = 0;
  uint16_t xtal32k_cycles = 0;
  uint32_t xtal32k_ms_max = 0;

  wdinfo("Entry: timeout=%" PRIu32 "\n", timeout);
  DEBUGASSERT(priv);

  /* Unlock WDT */

  ESP32S3_WDT_UNLOCK(priv->wdt);

  /* Write the timeout value */

  priv->timeout = timeout;

  /* Watchdog from Timer Module */

  if (priv->peripheral == TIMER)
    {
      /* Is this timeout a valid value for Timer's WDT? */

      if (timeout == 0 || timeout > MWDT_MAX_TIMEOUT_MS)
        {
          wderr("Cannot represent timeout=%" PRIu32 " > %" PRIu32 "\n",
                timeout, MWDT_MAX_TIMEOUT_MS);
          return -ERANGE;
        }
      else
        {
          ESP32S3_WDT_STO(priv->wdt, MWDT_TIMEOUT_MS(timeout),
                          ESP32S3_WDT_STAGE0);
        }
    }

  /* Watchdog from RTC Module */

  else if (priv->peripheral == RTC)
    {
      rtc_cycles = ESP32S3_RWDT_CLK(priv->wdt);
      rtc_ms_max = (RWDT_FULL_STAGE / (uint32_t)rtc_cycles);

      /* Is this timeout a valid value for RTC WDT? */

      if (timeout == 0 || timeout > rtc_ms_max)
        {
          wderr("Cannot represent timeout=%" PRIu32 " > %" PRIu32 "\n",
                timeout, rtc_ms_max);
          return -ERANGE;
        }
      else
        {
          timeout = timeout * rtc_cycles;
          ESP32S3_WDT_STO(priv->wdt, timeout, ESP32S3_WDT_STAGE0);
        }
    }

    /* Watchdog from XTAL32K Module */

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
      else
        {
          timeout = timeout * xtal32k_cycles;
          ESP32S3_WDT_STO(priv->wdt, timeout, ESP32S3_WDT_STAGE0);
        }
    }

  /* Reset the wdt */

  ESP32S3_WDT_FEED(priv->wdt);

  /* Lock it again */

  ESP32S3_WDT_LOCK(priv->wdt);

  return OK;
}

/****************************************************************************
 * Name: wdt_lh_capture
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
 *   The previous watchdog expiration function pointer or NULL if there was
 *   no previous function pointer, i.e., if the previous behavior was
 *   reset-on-expiration (NULL is also returned if an error occurs).
 *
 ****************************************************************************/

static xcpt_t wdt_lh_capture(struct watchdog_lowerhalf_s *lower,
                             xcpt_t handler)
{
  struct esp32s3_wdt_lowerhalf_s *priv =
    (struct esp32s3_wdt_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;

  DEBUGASSERT(priv);

  wdinfo("Entry: handler=%p\n", handler);

  /* Get the old handler to return it */

  oldhandler = priv->handler;

  ESP32S3_WDT_UNLOCK(priv->wdt);

  flags = spin_lock_irqsave(&priv->lock);

  /* Save the new user handler */

  priv->handler = handler;

  /* There is a user callback and the timer has already been started.
   * The user wants to set a callback after starting the wdt or wants to
   * change the callback function once a callback has already been settled.
   */

  if (priv->handler != NULL && priv->started)
    {
      /* Deallocate the previous allocated interrupt
       * If there is a previous allocated interrupt.
       */

      if (oldhandler != NULL)
        {
          ESP32S3_WDT_SETISR(priv->wdt, NULL, NULL);
        }
      else
        {
          /* If it was previous configured to reset on timeout
           * then change to interrupt.
           */

          ESP32S3_WDT_STG_CONF(priv->wdt, ESP32S3_WDT_STAGE0,
                               ESP32S3_WDT_STAGE_ACTION_INT);
        }

      /* Set the lower-half handler and enable interrupt */

      ESP32S3_WDT_SETISR(priv->wdt, wdt_handler, priv);
      ESP32S3_WDT_ENABLEINT(priv->wdt);
    }

  /* In case the user wants to disable the callback */

  else
    {
      ESP32S3_WDT_DISABLEINT(priv->wdt);
      ESP32S3_WDT_SETISR(priv->wdt, NULL, NULL);

      /* Then configure it to reset on WDT expiration */

      if (priv->peripheral == TIMER)
        {
          ESP32S3_WDT_STG_CONF(priv->wdt, ESP32S3_WDT_STAGE0,
                               ESP32S3_WDT_STAGE_ACTION_RESET_SYSTEM);
        }
      else if (priv->peripheral == RTC)
        {
          ESP32S3_WDT_STG_CONF(priv->wdt, ESP32S3_WDT_STAGE0,
                               ESP32S3_WDT_STAGE_ACTION_RESET_RTC);
        }
    }

  spin_unlock_irqrestore(&priv->lock, flags);
  ESP32S3_WDT_LOCK(priv->wdt);
  return oldhandler;
}

/****************************************************************************
 * Name: wdt_lh_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the
 *                "lower-half" driver state structure.
 *   cmd        - Command number to process.
 *   arg        - Argument that sent to the command.
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

static int wdt_lh_ioctl(struct watchdog_lowerhalf_s *lower, int cmd,
                        unsigned long arg)
{
  struct esp32s3_wdt_lowerhalf_s *priv =
    (struct esp32s3_wdt_lowerhalf_s *)lower;

  wdinfo("ioctl Call: cmd=0x%x arg=0x%lx", cmd, arg);

  /* Process the IOCTL command */

  switch (cmd)
    {
    case WDIOC_RSTCLK:
      ESP32S3_XTWDT_RST_CLK(priv->wdt);
      break;

    default:
      return -ENOTTY;
    }

  return OK;
}

/* Interrupt handling *******************************************************/

static int wdt_handler(int irq, void *context, void *arg)
{
  struct esp32s3_wdt_lowerhalf_s *priv = arg;

  /* Run the user callback */

  priv->handler(irq, context, priv->upper);

  /* Clear the Interrupt */

  ESP32S3_WDT_UNLOCK(priv->wdt);

  ESP32S3_WDT_ACKINT(priv->wdt);

  ESP32S3_WDT_LOCK(priv->wdt);

  return OK;
}

/****************************************************************************
 * Name: wdt_lh_xt_calculate
 *
 * Description:
 *   Calculate the actual frequency of RC_SLOW_CLK to calibrate the backup
 *   RTC_SLOW_CLK. This is necessary to compensate for clock deviations.
 *
 * Input Parameters:
 *   rtc_clk_frequency_khz - Frequency of RTC SLOW CLK in khz
 *
 * Returned Values:
 *   The divisor of BACKUP32K_CLK used to calibrate the RTC_SLOW_CLK
 *   according to the actual frequency of the RC_SLOW_CLK.
 *
 ****************************************************************************/
#ifdef CONFIG_ESP32S3_XTWDT_BACKUP_CLK_ENABLE
static uint32_t wdt_lh_xt_calculate(uint32_t rtc_clk_frequency_khz)
{
    uint32_t xtal32k_clk_factor = 0;
    uint8_t divisor_comps[XT_WDT_DIV_COMP_N_MAX];

    uint8_t M = ((rtc_clk_frequency_khz / 32) / 2);
    uint32_t S = ((4 * rtc_clk_frequency_khz) / 32);

    memset(divisor_comps, M, XT_WDT_DIV_COMP_N_MAX);

    /* Calculate how far we are away from satisfying S = SUM(x_n) */

    uint8_t off = S - XT_WDT_DIV_COMP_N_MAX * M;

    /* Offset should never be this big */

    ASSERT(off <= XT_WDT_DIV_COMP_N_MAX);

    for (int i = 0; i < XT_WDT_DIV_COMP_N_MAX; i++)
      {
        if (off)
          {
            divisor_comps[i]++;
            off--;
          }

        /* Sum up all divisors */

        xtal32k_clk_factor |=  (divisor_comps[i] << 4 * i);
      }

    return xtal32k_clk_factor;
}

/****************************************************************************
 * Name: wdt_lh_clk_freq_cal
 *
 * Description:
 *   Calculate the clock frequency from period in microseconds.
 *
 * Input Parameters:
 *   rtc_clk_period_us - Average slow clock period in microseconds.
 *
 * Returned Values:
 *   Frequency of the clock in Hz.
 *
 ****************************************************************************/

static uint32_t wdt_lh_clk_freq_cal(uint32_t rtc_clk_period_us)
{
  if (rtc_clk_period_us == 0)
    {
      return 0;
    }

  return 1000000ULL * (1 << RTC_CLK_CAL_FRACT) / rtc_clk_period_us;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_wdt_initialize
 *
 * Description:
 *   Initialize the watchdog timer.  The watchdog timer is initialized
 *   and registered as 'devpath'.
 *
 * Input Parameters:
 *   devpath                 - The full path to the watchdog.  This should
 *                             be of the form /dev/watchdogX
 *   wdt                     - WDT instance to be initialized.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp32s3_wdt_initialize(const char *devpath, enum esp32s3_wdt_inst_e wdt)
{
  struct esp32s3_wdt_lowerhalf_s *lower = NULL;
  int                             ret = OK;

  DEBUGASSERT(devpath);

  /* Timer Group 0 and RTC are initialized on system start.
   * Timer Group 1 must be initialized for MWDT1.
   */

  switch (wdt)
    {
#ifdef CONFIG_ESP32S3_MWDT0
      case ESP32S3_WDT_MWDT0:
        {
          lower = &g_esp32s3_mwdt0_lowerhalf;
          lower->peripheral = TIMER;
          break;
        }
#endif

#ifdef CONFIG_ESP32S3_MWDT1
      case ESP32S3_WDT_MWDT1:
        {
          lower = &g_esp32s3_mwdt1_lowerhalf;
          lower->peripheral = TIMER;
          periph_module_enable(PERIPH_TIMG1_MODULE);
          break;
        }
#endif

#ifdef CONFIG_ESP32S3_RWDT
      case ESP32S3_WDT_RWDT:
        {
          lower = &g_esp32s3_rwdt_lowerhalf;
          lower->peripheral = RTC;
          break;
        }
#endif

#ifdef CONFIG_ESP32S3_XTWDT
      case ESP32S3_WDT_XTWDT:
        {
          lower = &g_esp32s3_xtwdt_lowerhalf;
          lower->peripheral = XTAL32K;
          break;
        }
#endif

      default:
        {
          ret = -ENODEV;
          goto errout;
        }
    }

  /* Initialize the elements of lower-half state structure */

  lower->handler = NULL;
  lower->timeout = 0;
  lower->wdt     = esp32s3_wdt_init(wdt);

  if (lower->wdt == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  lower->started = esp32s3_wdt_is_running(lower->wdt);

  ESP32S3_WDT_UNLOCK(lower->wdt);

  /* If it is a Main System Watchdog Timer configure the Prescale to
   * have a 500us period.
   */

  if (lower->peripheral == TIMER)
    {
      ESP32S3_MWDT_PRE(lower->wdt, MWDT_CLK_PRESCALER_VALUE);
    }

    /* Configure auto backup clock when XTAL32K fails */

#ifdef CONFIG_ESP32S3_XTWDT_BACKUP_CLK_ENABLE
  if (lower->peripheral == XTAL32K)
    {
      /* Estimate frequency of internal RTC oscillator */

      uint32_t rtc_clk_period_us =
        esp32s3_rtc_clk_cal(RTC_CAL_INTERNAL_OSC, XT_WDT_CLK_CAL_CYCLES);

      uint32_t rtc_clk_frequency_khz = wdt_lh_clk_freq_cal(rtc_clk_period_us)
        / 1000;

      uint32_t xtal32k_clk_factor =
        wdt_lh_xt_calculate(rtc_clk_frequency_khz);

      ESP32S3_XTWDT_PRE(lower->wdt, xtal32k_clk_factor);
    }
#endif

  ESP32S3_WDT_LOCK(lower->wdt);

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
