/****************************************************************************
 * arch/arm/src/imxrt/imxrt_wdog.c
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

#include <stdint.h>
#include <stdbool.h>

#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/spinlock.h>
#include <nuttx/timers/watchdog.h>

#include "arm_internal.h"
#include "hardware/imxrt_wdog.h"
#include "imxrt_wdog.h"

#include <arch/board/board.h> /* Include last:  has dependencies */

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_IMXRT_WDOG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select the path to the registered watchdog timer device */

#ifdef CONFIG_WATCHDOG_DEVPATH
#define DEVPATH CONFIG_WATCHDOG_DEVPATH
#else
#define DEVPATH "/dev/watchdog0"
#endif

/* Time out range is from 0 to 128 seconds in 0.5s intervals */

#define WDOG_MIN              (500)
#define WDOG_MAX              (128000)

#define WDOG_KEEP_ALIVE_KEY1  (0x5555u)
#define WDOG_KEEP_ALIVE_KEY2  (0xaaaau)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_wdog_lower
{
  const struct watchdog_ops_s  *ops;  /* Lower half operations */
  uint32_t     timeout;
  uint32_t     enabled;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Timeout to register field helper */

uint32_t imxrt_wdog_ms_to_reg(uint32_t timeout);

/* Lower half driver methods */

static int      imxrt_wdog_start(struct watchdog_lowerhalf_s *lower);
static int      imxrt_wdog_stop(struct watchdog_lowerhalf_s *lower);
static int      imxrt_wdog_keepalive(struct watchdog_lowerhalf_s *lower);
static int      imxrt_wdog_getstatus(struct watchdog_lowerhalf_s *lower,
                                     struct watchdog_status_s *status);
static int      imxrt_wdog_settimeout(struct watchdog_lowerhalf_s *lower,
                                      uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver ops */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = imxrt_wdog_start,
  .stop       = imxrt_wdog_stop,
  .keepalive  = imxrt_wdog_keepalive,
  .getstatus  = imxrt_wdog_getstatus,
  .settimeout = imxrt_wdog_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

/* "Lower half" driver state */

static struct imxrt_wdog_lower g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_wdog_ms_to_reg
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           lower-half driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

uint32_t imxrt_wdog_ms_to_reg(uint32_t ms)
{
  uint32_t reg = ms / 500; /* This gives the value needed for
                            * ms rounded up to the nearest 500ms of timeout
                            */

  if (reg != 0 && ms % 500 == 0)
    {
      /* If the number is divisible by 500ms we subtract 1.
       * Else we will set the timeout to the smallest achievable timeout
       * that is greater than the requested value.
       */

      reg--;
    }

  return reg;
}

/****************************************************************************
 * Name: imxrt_wdog
 *
 * Description:
 *   Start the watchdog timer, setting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           lower-half driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int imxrt_wdog_start(struct watchdog_lowerhalf_s *lower)
{
  struct imxrt_wdog_lower *priv = (struct imxrt_wdog_lower *)lower;
  uint16_t regval;

  if (priv->enabled == false)
    {
      priv->enabled = true;

      regval  = getreg16(IMXRT_WDOG1_WCR);
      regval |= WDOG_WCR_WT(imxrt_wdog_ms_to_reg(priv->timeout));
      putreg16(regval, IMXRT_WDOG1_WCR);

      /* Now that the timeout field is properly set, start the watchdog. */

      regval |= WDOG_WCR_WDE;
      putreg16(regval, IMXRT_WDOG1_WCR);
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_wdog_stop
 *
 * Description:
 *   Exists since it is a required function for the watchdog lower-half
 *   driver. On the IMXRT you cannot disable the watchdog once it is started.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           lower-half driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int imxrt_wdog_stop(struct watchdog_lowerhalf_s *lower)
{
  struct imxrt_wdog_lower *priv = (struct imxrt_wdog_lower *)lower;

  if (priv->enabled)
    {
      /* We cannot disable the watchdog once it is enabled. */

      wderr("ERROR: Cannot stop Wdog once started\n");
      return -ENOSYS;
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_wdog_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           lower-half driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int imxrt_wdog_keepalive(struct watchdog_lowerhalf_s *lower)
{
  irqstate_t flags = spin_lock_irqsave(NULL);

  putreg16(WDOG_KEEP_ALIVE_KEY1, IMXRT_WDOG1_WSR);
  putreg16(WDOG_KEEP_ALIVE_KEY2, IMXRT_WDOG1_WSR);

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

/****************************************************************************
 * Name: imxrt_wdog_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the
 *            lower-half driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int imxrt_wdog_getstatus(struct watchdog_lowerhalf_s *lower,
                                struct watchdog_status_s *status)
{
  struct imxrt_wdog_lower *priv = (struct imxrt_wdog_lower *)lower;

  status->flags = WDFLAGS_RESET;

  if (priv->enabled)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  status->timeout = priv->timeout;
  status->timeleft = 0; /* not supported for WDOG1 */

  return OK;
}

/****************************************************************************
 * Name: imxrt_wdog_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int imxrt_wdog_settimeout(struct watchdog_lowerhalf_s *lower,
                                 uint32_t timeout)
{
  uint32_t regval;
  struct imxrt_wdog_lower *priv = (struct imxrt_wdog_lower *)lower;

  if (timeout < WDOG_MIN || timeout > WDOG_MAX)
    {
      wderr("ERROR: Cannot represent timeout=%d. Range=[%d, %d]\n",
            timeout, WDOG_MIN, WDOG_MAX);
      return -ERANGE;
    }

  priv->timeout = timeout;

  irqstate_t flags = spin_lock_irqsave(NULL);

  /* write timer value to WCR WT register */

  regval  = getreg16(IMXRT_WDOG1_WCR);
  regval &= ~WDOG_WCR_WT_MASK;  /* clear previous count value */
  regval |= WDOG_WCR_WT(priv->timeout);
  putreg16(regval, IMXRT_WDOG1_WCR);

  /* reload the Wdog counter by petting it */

  putreg16(WDOG_KEEP_ALIVE_KEY1, IMXRT_WDOG1_WSR);
  putreg16(WDOG_KEEP_ALIVE_KEY2, IMXRT_WDOG1_WSR);

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_wdog_initialize
 *
 * Description:
 *   Initialize the watchdog time.  The watchdog timer is initialized and
 *   registered at devpath.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void imxrt_wdog_initialize(void)
{
  struct imxrt_wdog_lower *priv = &g_wdgdev;

  priv->ops = &g_wdgops;
  priv->timeout = WDOG_MIN;

  /* Register the watchdog driver at the path */

  wdinfo("Entry: devpath=%s\n", DEVPATH);
  watchdog_register(DEVPATH, (struct watchdog_lowerhalf_s *)priv);
}

#endif /* CONFIG_WATCHDOG && CONFIG_IMXRT_WDOG */

/****************************************************************************
 * Name: imxrt_wdog_disable
 *
 * Description:
 *   Disables all watchdogs
 *
 ****************************************************************************/

void imxrt_wdog_disable_all(void)
{
  uint32_t reg;
  irqstate_t flags;

  reg = getreg16(IMXRT_WDOG1_WCR);
  if (reg & WDOG_WCR_WDE)
    {
      reg &= ~WDOG_WCR_WDE;
      putreg16(reg, IMXRT_WDOG1_WCR);
    }

  reg = getreg16(IMXRT_WDOG2_WCR);
  if (reg & WDOG_WCR_WDE)
    {
      reg &= ~WDOG_WCR_WDE;
      putreg16(reg, IMXRT_WDOG2_WCR);
    }

  flags = enter_critical_section();
  putreg32(RTWDOG_UPDATE_KEY, IMXRT_RTWDOG_CNT);
  putreg32(0xffff, IMXRT_RTWDOG_TOVAL);
  modifyreg32(IMXRT_RTWDOG_CS, RTWDOG_CS_EN, RTWDOG_CS_UPDATE);
  leave_critical_section(flags);
}
