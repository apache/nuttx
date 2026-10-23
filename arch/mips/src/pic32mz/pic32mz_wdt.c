/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_wdt.c
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

#include <sys/types.h>

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/timers/watchdog.h>

#include "mips_internal.h"
#include "hardware/pic32mz_wdt.h"
#include "hardware/pic32mzw1_features.h"
#include "hardware/pic32mzw1_pmuclk.h"
#include "pic32mz_wdt.h"

#ifdef CONFIG_PIC32MZ_WDT

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pic32mz_wdt_lowerhalf_s
{
  const struct watchdog_ops_s *ops;      /* Lower half operations */
  clock_t                      lastkick; /* System tick of the last kick */
  uint32_t                     timeout;  /* Hardware time-out (ms) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pic32mz_wdt_start(struct watchdog_lowerhalf_s *lower);
static int pic32mz_wdt_stop(struct watchdog_lowerhalf_s *lower);
static int pic32mz_wdt_keepalive(struct watchdog_lowerhalf_s *lower);
static int pic32mz_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                                 struct watchdog_status_s *status);
static int pic32mz_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                                  uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct watchdog_ops_s g_wdtops =
{
  .start      = pic32mz_wdt_start,
  .stop       = pic32mz_wdt_stop,
  .keepalive  = pic32mz_wdt_keepalive,
  .getstatus  = pic32mz_wdt_getstatus,
  .settimeout = pic32mz_wdt_settimeout,
};

static struct pic32mz_wdt_lowerhalf_s g_wdt;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_wdt_timeout_ms
 *
 * Description:
 *   Hardware time-out in milliseconds.  The postscaler is the WDTPS field
 *   of CFGCON2, programmed from the device configuration; software cannot
 *   change it, so the time-out is fixed for a given image.  (WDTCON.RUNDIV
 *   mirrors it, but is not relied on when the watchdog is not enabled by
 *   the configuration.)
 *
 ****************************************************************************/

static uint32_t pic32mz_wdt_timeout_ms(void)
{
  uint32_t rundiv;
  uint64_t us;

  rundiv = (getreg32(PIC32MZ_CFGCON2) >> DEVCFG2_WDTPS_SHIFT) &
           DEVCFG2_WDTPS_MASK;
  if (rundiv > WDT_RUNDIV_MAX)
    {
      rundiv = WDT_RUNDIV_MAX;
    }

  us = (uint64_t)CONFIG_PIC32MZ_WDT_BASE_US << rundiv;
  return (uint32_t)((us + 999) / 1000);
}

/****************************************************************************
 * Name: pic32mz_wdt_hwkick
 *
 * Description:
 *   Clear the watchdog counter.  The datasheet requires the key to be
 *   written with a single 16-bit store to the upper half of WDTCON.
 *
 ****************************************************************************/

static void pic32mz_wdt_hwkick(struct pic32mz_wdt_lowerhalf_s *priv)
{
  putreg16(WDT_CLRKEY_VALUE, PIC32MZ_WDT_CLRKEY);
  priv->lastkick = clock_systime_ticks();
}

/****************************************************************************
 * Name: pic32mz_wdt_start
 *
 * Description:
 *   Start the watchdog.  When it is hardware-enabled by the device
 *   configuration (WDTEN) it is already running and this only kicks it.
 *
 ****************************************************************************/

static int pic32mz_wdt_start(struct watchdog_lowerhalf_s *lower)
{
  struct pic32mz_wdt_lowerhalf_s *priv =
    (struct pic32mz_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  pic32mz_wdt_hwkick(priv);
  putreg32(WDT_CON_ON, PIC32MZ_WDT_CONSET);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: pic32mz_wdt_stop
 *
 * Description:
 *   Stop the watchdog.  This is not possible when WDTEN is set in the
 *   device configuration: the ON bit is then ignored by the hardware.
 *
 ****************************************************************************/

static int pic32mz_wdt_stop(struct watchdog_lowerhalf_s *lower)
{
  irqstate_t flags;
  int ret = OK;

  flags = enter_critical_section();

  putreg32(WDT_CON_ON, PIC32MZ_WDT_CONCLR);

  if ((getreg32(PIC32MZ_WDT_CON) & WDT_CON_ON) != 0)
    {
      /* Still running: hardware-enabled by the device configuration */

      ret = -ENOTSUP;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: pic32mz_wdt_keepalive
 ****************************************************************************/

static int pic32mz_wdt_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct pic32mz_wdt_lowerhalf_s *priv =
    (struct pic32mz_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();
  pic32mz_wdt_hwkick(priv);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: pic32mz_wdt_getstatus
 ****************************************************************************/

static int pic32mz_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                                 struct watchdog_status_s *status)
{
  struct pic32mz_wdt_lowerhalf_s *priv =
    (struct pic32mz_wdt_lowerhalf_s *)lower;
  uint32_t elapsed;
  irqstate_t flags;

  flags = enter_critical_section();

  status->flags    = 0;
  status->timeout  = priv->timeout;

  if ((getreg32(PIC32MZ_WDT_CON) & WDT_CON_ON) != 0)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  /* The counter is not readable: derive the time left from the last kick */

  elapsed = TICK2MSEC(clock_systime_ticks() - priv->lastkick);
  status->timeleft = elapsed < priv->timeout ? priv->timeout - elapsed : 0;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: pic32mz_wdt_settimeout
 *
 * Description:
 *   The time-out is fixed by the device configuration.  A request for a
 *   time-out longer than the hardware one cannot be honored and fails;
 *   a shorter one is accepted (the watchdog then fires later than asked,
 *   which is safe for a caller that keeps alive within what it asked for).
 *   As the method specifies, the watchdog is also kicked.
 *
 ****************************************************************************/

static int pic32mz_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                                  uint32_t timeout)
{
  struct pic32mz_wdt_lowerhalf_s *priv =
    (struct pic32mz_wdt_lowerhalf_s *)lower;

  if (timeout == 0 || timeout > priv->timeout)
    {
      wdwarn("time-out %" PRIu32 " ms not possible, hardware is %" PRIu32
             " ms\n", timeout, priv->timeout);
      return -ERANGE;
    }

  return pic32mz_wdt_keepalive(lower);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_wdt_initialize
 ****************************************************************************/

int pic32mz_wdt_initialize(const char *devpath)
{
  struct pic32mz_wdt_lowerhalf_s *priv = &g_wdt;
  void *handle;

  if ((getreg32(PIC32MZ_RCON) & RCON_WDTO) != 0)
    {
      syslog(LOG_WARNING, "Last reset was caused by the watchdog\n");
      putreg32(RCON_WDTO, PIC32MZ_RCONCLR);
    }

  priv->ops      = &g_wdtops;
  priv->timeout  = pic32mz_wdt_timeout_ms();
  priv->lastkick = clock_systime_ticks();

  handle = watchdog_register(devpath,
                             (struct watchdog_lowerhalf_s *)priv);
  if (handle == NULL)
    {
      return -EEXIST;
    }

  /* Without the fuse it is stopped, and the application starts it.  With
   * the fuse it is running already: keep it alive until the application
   * takes over.
   */

  if ((getreg32(PIC32MZ_WDT_CON) & WDT_CON_ON) != 0)
    {
      pic32mz_wdt_hwkick(priv);
    }

  return OK;
}

#endif /* CONFIG_PIC32MZ_WDT */
