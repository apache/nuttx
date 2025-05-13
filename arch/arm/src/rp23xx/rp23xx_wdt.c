/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_wdt.c
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

#include "rp23xx_wdt.h"

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/chip/watchdog.h>

#include <hardware/rp23xx_ticks.h>
#include <hardware/rp23xx_watchdog.h>
#include <hardware/rp23xx_psm.h>

#include <arch/rp23xx/watchdog.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/timers/watchdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WD_ENABLE_BITS   (RP23XX_WATCHDOG_CTRL_ENABLE     \
                        | RP23XX_WATCHDOG_CTRL_PAUSE_DBG0 \
                        | RP23XX_WATCHDOG_CTRL_PAUSE_DBG1 \
                        | RP23XX_WATCHDOG_CTRL_PAUSE_JTAG)

#define WDT_MAX_TIMEOUT (0xffffff)  /* 16777215 ~ 16 sec */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

typedef struct rp23xx_watchdog_lowerhalf_s
{
  const struct watchdog_ops_s *ops;        /* Lower half operations */
  uint32_t                     timeout;    /* The current timeout */
  uint32_t                     lastreset;  /* The last reset time */
  bool                         started;    /* True: Timer has been started */
  xcpt_t                       handler;    /* User Handler */
  void                        *upper;      /* Pointer to watchdog_upperhalf_s */
} watchdog_lowerhalf_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* "Lower half" driver methods **********************************************/

static int  rp23xx_wdt_start      (struct watchdog_lowerhalf_s *lower);
static int  rp23xx_wdt_stop       (struct watchdog_lowerhalf_s *lower);
static int  rp23xx_wdt_keepalive  (struct watchdog_lowerhalf_s *lower);
static int  rp23xx_wdt_getstatus  (struct watchdog_lowerhalf_s *lower,
                               struct watchdog_status_s    *status);
static int  rp23xx_wdt_settimeout (struct watchdog_lowerhalf_s *lower,
                               uint32_t                     timeout);
static int  rp23xx_wdt_ioctl      (struct watchdog_lowerhalf_s *lower,
                               int                          cmd,
                               unsigned long                arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_rp23xx_wdg_ops =
{
  .start      = rp23xx_wdt_start,
  .stop       = rp23xx_wdt_stop,
  .keepalive  = rp23xx_wdt_keepalive,
  .getstatus  = rp23xx_wdt_getstatus,
  .settimeout = rp23xx_wdt_settimeout,
  .capture    = NULL,
  .ioctl      = rp23xx_wdt_ioctl,
};

static watchdog_lowerhalf_t g_rp23xx_watchdog_lowerhalf =
{
  .ops   = &g_rp23xx_wdg_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_wdt_start
 ****************************************************************************/

int rp23xx_wdt_start(struct watchdog_lowerhalf_s *lower)
{
  watchdog_lowerhalf_t *priv = (watchdog_lowerhalf_t *)lower;

  wdinfo("Entry\n");

  if (priv->started == true)
    {
      /* Return EBUSY to indicate that the timer was already running */

      return -EBUSY;
    }

  putreg32(priv->timeout * USEC_PER_MSEC,  RP23XX_WATCHDOG_LOAD);

  putreg32(RP23XX_PSM_WDSEL_BITS & ~(RP23XX_PSM_XOSC | RP23XX_PSM_ROSC),
           RP23XX_PSM_WDSEL);

  modreg32(WD_ENABLE_BITS, WD_ENABLE_BITS, RP23XX_WATCHDOG_CTRL);

  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: rp23xx_wdt_stop
 ****************************************************************************/

int rp23xx_wdt_stop(struct watchdog_lowerhalf_s *lower)
{
  watchdog_lowerhalf_t *priv = (watchdog_lowerhalf_t *)lower;

  wdinfo("Entry\n");

  modreg32(0, RP23XX_WATCHDOG_CTRL_ENABLE, RP23XX_WATCHDOG_CTRL);

  priv->started = false;
  return OK;
}

/****************************************************************************
 * Name: rp23xx_wdt_keepalive
 ****************************************************************************/

int rp23xx_wdt_keepalive(struct watchdog_lowerhalf_s *lower)
{
  watchdog_lowerhalf_t *priv = (watchdog_lowerhalf_t *)lower;

  wdinfo("Entry\n");

  putreg32(priv->timeout * USEC_PER_MSEC,  RP23XX_WATCHDOG_LOAD);

  return OK;
}

/****************************************************************************
 * Name: rp23xx_wdt_getstatus
 ****************************************************************************/

int rp23xx_wdt_getstatus(struct watchdog_lowerhalf_s  *lower,
                     struct watchdog_status_s     *status)
{
  watchdog_lowerhalf_t *priv = (watchdog_lowerhalf_t *)lower;
  uint32_t              ctrl = getreg32(RP23XX_WATCHDOG_CTRL);

  wdinfo("Entry\n");

  status->flags    =  (ctrl & RP23XX_WATCHDOG_CTRL_ENABLE) ? WDFLAGS_ACTIVE
                                                           : 0;

  status->timeout  =  priv->timeout;

  status->timeleft =  (ctrl & RP23XX_WATCHDOG_CTRL_TIME_MASK) /
                       USEC_PER_MSEC;
  wdinfo("Status     :\n");
  wdinfo("  flags    : %08" PRIx32 "\n", status->flags);
  wdinfo("  timeout  : %" PRId32 "\n", status->timeout);
  wdinfo("  timeleft : %" PRId32 "\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: rp23xx_wdt_settimeout
 ****************************************************************************/

int rp23xx_wdt_settimeout (struct watchdog_lowerhalf_s *lower,
                           uint32_t timeout)
{
  watchdog_lowerhalf_t *priv  = (watchdog_lowerhalf_t *)lower;

  wdinfo("Entry: timeout=%" PRId32 "\n", timeout);

  if ((timeout == 0) || (timeout > (WDT_MAX_TIMEOUT / USEC_PER_MSEC)))
    {
      return -EINVAL;
    }

  /* Load the watchdog timer. The maximum setting is 0xffffff which
   * corresponds to approximately 16 seconds
   */

  priv->timeout = timeout;

  putreg32(priv->timeout * USEC_PER_MSEC,  RP23XX_WATCHDOG_LOAD);

  return OK;
}

/****************************************************************************
 * Name: rp23xx_wdt_ioctl
 ****************************************************************************/

int rp23xx_wdt_ioctl(struct watchdog_lowerhalf_s *lower,
                 int                          cmd,
                 unsigned long                arg)
{
  if (cmd >= WDIOC_SET_SCRATCH0  &&  cmd <= WDIOC_SET_SCRATCH7)
    {
      int n = cmd - WDIOC_SET_SCRATCH0;

      putreg32((uint32_t) arg, RP23XX_WATCHDOG_SCRATCH(n));

      return OK;
    }

  if (cmd >= WDIOC_GET_SCRATCH0  &&  cmd <= WDIOC_GET_SCRATCH7)
    {
      int n = cmd - WDIOC_GET_SCRATCH0;

      *((uint32_t *)arg) = getreg32((uint32_t) RP23XX_WATCHDOG_SCRATCH(n));

      return OK;
    }

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_wdt_init
 ****************************************************************************/

int rp23xx_wdt_init(void)
{
  watchdog_lowerhalf_t *lower = &g_rp23xx_watchdog_lowerhalf;
  int                   ret   = OK;

  putreg32(WDT_MAX_TIMEOUT,  RP23XX_WATCHDOG_LOAD);
  modreg32(0, RP23XX_WATCHDOG_CTRL_ENABLE, RP23XX_WATCHDOG_CTRL);
  lower->timeout = WDT_MAX_TIMEOUT / USEC_PER_MSEC;

  lower->upper = watchdog_register(CONFIG_WATCHDOG_DEVPATH,
                                   (struct watchdog_lowerhalf_s *) lower);
  if (lower->upper == NULL)
    {
      ret = -EEXIST;
      goto errout;
    }

errout:
  return ret;
}
