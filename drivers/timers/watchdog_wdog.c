/****************************************************************************
 * drivers/timers/watchdog_wdog.c
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
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>
#include <nuttx/timers/watchdog.h>
#include <nuttx/timers/watchdog_wdog.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct watchdog_wdog_s
{
  struct watchdog_lowerhalf_s lower;   /* Must be first */
  struct wdog_s               wdog;    /* The timer that expires */
  uint32_t                    timeout; /* Timeout in milliseconds */
  bool                        started; /* Is the watchdog running */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int watchdog_wdog_start(FAR struct watchdog_lowerhalf_s *lower);
static int watchdog_wdog_stop(FAR struct watchdog_lowerhalf_s *lower);
static int watchdog_wdog_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int watchdog_wdog_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                                   FAR struct watchdog_status_s *status);
static int watchdog_wdog_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                                    uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct watchdog_ops_s g_watchdog_wdog_ops =
{
  .start      = watchdog_wdog_start,
  .stop       = watchdog_wdog_stop,
  .keepalive  = watchdog_wdog_keepalive,
  .getstatus  = watchdog_wdog_getstatus,
  .settimeout = watchdog_wdog_settimeout,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: watchdog_wdog_expire
 *
 * Description:
 *   The wdog handler: the watchdog was not fed in time.
 *
 ****************************************************************************/

static void watchdog_wdog_expire(wdparm_t arg)
{
  _alert("Watchdog expired\n");

#if defined(CONFIG_WATCHDOG_WDOG_REBOOT)
  board_reset(CONFIG_WATCHDOG_WDOG_STATUS);
#elif defined(CONFIG_WATCHDOG_WDOG_POWEROFF)
  board_power_off(CONFIG_WATCHDOG_WDOG_STATUS);
#else
  PANIC();
#endif
}

/****************************************************************************
 * Name: watchdog_wdog_arm
 ****************************************************************************/

static int watchdog_wdog_arm(FAR struct watchdog_wdog_s *priv)
{
  return wd_start(&priv->wdog, MSEC2TICK(priv->timeout),
                  watchdog_wdog_expire, (wdparm_t)priv);
}

static int watchdog_wdog_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct watchdog_wdog_s *priv = (FAR struct watchdog_wdog_s *)lower;
  int ret;

  ret = watchdog_wdog_arm(priv);
  if (ret >= 0)
    {
      priv->started = true;
    }

  return ret;
}

static int watchdog_wdog_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct watchdog_wdog_s *priv = (FAR struct watchdog_wdog_s *)lower;

  priv->started = false;
  wd_cancel(&priv->wdog);
  return OK;
}

static int watchdog_wdog_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct watchdog_wdog_s *priv = (FAR struct watchdog_wdog_s *)lower;

  return priv->started ? watchdog_wdog_arm(priv) : OK;
}

static int watchdog_wdog_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                                   FAR struct watchdog_status_s *status)
{
  FAR struct watchdog_wdog_s *priv = (FAR struct watchdog_wdog_s *)lower;

  status->flags    = priv->started ? WDFLAGS_ACTIVE | WDFLAGS_RESET : 0;
  status->timeout  = priv->timeout;
  status->timeleft = 0;

  if (priv->started)
    {
      /* wd_start() adds a tick, do not report more than the timeout */

      status->timeleft = TICK2MSEC(wd_gettime(&priv->wdog));
      if (status->timeleft > priv->timeout)
        {
          status->timeleft = priv->timeout;
        }
    }

  return OK;
}

static int watchdog_wdog_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                                    uint32_t timeout)
{
  FAR struct watchdog_wdog_s *priv = (FAR struct watchdog_wdog_s *)lower;

  if (timeout == 0)
    {
      return -EINVAL;
    }

  priv->timeout = timeout;
  return watchdog_wdog_keepalive(lower);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: watchdog_wdog_initialize
 ****************************************************************************/

int watchdog_wdog_initialize(FAR const char *path)
{
  FAR struct watchdog_wdog_s *priv;

  DEBUGASSERT(path != NULL);

  priv = kmm_zalloc(sizeof(struct watchdog_wdog_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->lower.ops = &g_watchdog_wdog_ops;
  priv->timeout   = CONFIG_WATCHDOG_WDOG_TIMEOUT;

  if (watchdog_register(path, &priv->lower) == NULL)
    {
      kmm_free(priv);
      return -EEXIST;
    }

  return OK;
}
