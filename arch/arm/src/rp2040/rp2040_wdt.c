/****************************************************************************
 * arch/arm/src/rp2040/rp2040_wdt.c
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

#include "rp2040_wdt.h"

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/chip/watchdog.h>

#include <hardware/rp2040_watchdog.h>
#include <hardware/rp2040_psm.h>

#include <arch/rp2040/watchdog.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/timers/watchdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WD_RESETS_BITS   (RP2040_PSM_CLOCKS              \
                        | RP2040_PSM_RESETS              \
                        | RP2040_PSM_BUSFABRIC           \
                        | RP2040_PSM_ROM                 \
                        | RP2040_PSM_SRAM0               \
                        | RP2040_PSM_SRAM1               \
                        | RP2040_PSM_SRAM2               \
                        | RP2040_PSM_SRAM3               \
                        | RP2040_PSM_SRAM4               \
                        | RP2040_PSM_SRAM5               \
                        | RP2040_PSM_XIP                 \
                        | RP2040_PSM_VREG_AND_CHIP_RESET \
                        | RP2040_PSM_SIO                 \
                        | RP2040_PSM_PROC0               \
                        | RP2040_PSM_PROC1)

#define WD_ENABLE_BITS   (RP2040_WATCHDOG_CTRL_ENABLE     \
                        | RP2040_WATCHDOG_CTRL_PAUSE_DBG0 \
                        | RP2040_WATCHDOG_CTRL_PAUSE_DBG1 \
                        | RP2040_WATCHDOG_CTRL_PAUSE_JTAG)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

typedef struct rp2040_watchdog_lowerhalf_s
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

static int  my_wdt_start      (struct watchdog_lowerhalf_s *lower);
static int  my_wdt_stop       (struct watchdog_lowerhalf_s *lower);
static int  my_wdt_keepalive  (struct watchdog_lowerhalf_s *lower);
static int  my_wdt_getstatus  (struct watchdog_lowerhalf_s *lower,
                               struct watchdog_status_s    *status);
static int  my_wdt_settimeout (struct watchdog_lowerhalf_s *lower,
                               uint32_t                     timeout);
static int  my_wdt_ioctl      (struct watchdog_lowerhalf_s *lower,
                               int                          cmd,
                               unsigned long                arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_rp2040_wdg_ops =
{
  .start      = my_wdt_start,
  .stop       = my_wdt_stop,
  .keepalive  = my_wdt_keepalive,
  .getstatus  = my_wdt_getstatus,
  .settimeout = my_wdt_settimeout,
  .capture    = NULL,
  .ioctl      = my_wdt_ioctl,
};

static watchdog_lowerhalf_t g_rp2040_watchdog_lowerhalf =
{
  .ops   = &g_rp2040_wdg_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: my_wdt_start
 ****************************************************************************/

int my_wdt_start(struct watchdog_lowerhalf_s *lower)
{
  watchdog_lowerhalf_t *priv = (watchdog_lowerhalf_t *)lower;

  /* Convert millisecond input to microseconds
   * Extra times 2 per errata RP2040-E1
   */

  putreg32(priv->timeout * 2000,  RP2040_WATCHDOG_LOAD);

  modreg32(WD_ENABLE_BITS, WD_ENABLE_BITS, RP2040_WATCHDOG_CTRL);

  modreg32(WD_RESETS_BITS, WD_RESETS_BITS, RP2040_PSM_WDSEL);

  return OK;
}

/****************************************************************************
 * Name: my_wdt_stop
 ****************************************************************************/

int my_wdt_stop(struct watchdog_lowerhalf_s *lower)
{
  modreg32(0, RP2040_WATCHDOG_CTRL_ENABLE, RP2040_WATCHDOG_CTRL);

  return OK;
}

/****************************************************************************
 * Name: my_wdt_keepalive
 ****************************************************************************/

int my_wdt_keepalive(struct watchdog_lowerhalf_s *lower)
{
  watchdog_lowerhalf_t *priv = (watchdog_lowerhalf_t *)lower;

  /* Convert millisecond input to microseconds
   * Extra times 2 per errata RP2040-E1
   */

  putreg32(priv->timeout * 2000,  RP2040_WATCHDOG_LOAD);

  return OK;
}

/****************************************************************************
 * Name: my_wdt_getstatus
 ****************************************************************************/

int my_wdt_getstatus(struct watchdog_lowerhalf_s  *lower,
                     struct watchdog_status_s     *status)
{
  watchdog_lowerhalf_t *priv = (watchdog_lowerhalf_t *)lower;
  uint32_t              ctrl = getreg32(RP2040_WATCHDOG_CTRL);

  status->flags    =  (ctrl & RP2040_WATCHDOG_CTRL_ENABLE) ? WDFLAGS_ACTIVE
                                                           : 0;

  status->timeout  =  priv->timeout;

  /* Convert microseconds to output microseconds.
   * Extra divide by 2 per errata RP2040-E1.
   */

  status->timeleft =  (ctrl & RP2040_WATCHDOG_CTRL_TIME_MASK) / 2000;

  /* WARNING: On (at least) version 2 RP2040 chips, the timeleft does
   *          not seem to be reliable.
   */

  return OK;
}

/****************************************************************************
 * Name: my_wdt_settimeout
 ****************************************************************************/

int my_wdt_settimeout (struct watchdog_lowerhalf_s *lower, uint32_t timeout)
{
  watchdog_lowerhalf_t *priv  = (watchdog_lowerhalf_t *)lower;

  priv->timeout = timeout > (0x7fffff / 1000) ? 0x7fffff : timeout;

  /* Convert millisecond input to microseconds
   * Extra times 2 per errata RP2040-E1
   */

  putreg32(priv->timeout * 2000,  RP2040_WATCHDOG_LOAD);
  return OK;
}

/****************************************************************************
 * Name: my_wdt_ioctl
 ****************************************************************************/

int my_wdt_ioctl(struct watchdog_lowerhalf_s *lower,
                 int                          cmd,
                 unsigned long                arg)
{
  if (cmd >= WDIOC_SET_SCRATCH0  &&  cmd <= WDIOC_SET_SCRATCH7)
    {
      int n = cmd - WDIOC_SET_SCRATCH0;

      putreg32((uint32_t) arg, RP2040_WATCHDOG_SCRATCH(n));

      return OK;
    }

  if (cmd >= WDIOC_GET_SCRATCH0  &&  cmd <= WDIOC_GET_SCRATCH7)
    {
      int n = cmd - WDIOC_GET_SCRATCH0;

      *((uint32_t *)arg) = getreg32((uint32_t) RP2040_WATCHDOG_SCRATCH(n));

      return OK;
    }

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_wdt_init
 ****************************************************************************/

int rp2040_wdt_init(void)
{
  watchdog_lowerhalf_t *lower = &g_rp2040_watchdog_lowerhalf;
  int                   ret   = OK;

  lower->upper = watchdog_register(CONFIG_WATCHDOG_DEVPATH,
                                   (struct watchdog_lowerhalf_s *) lower);
  if (lower->upper == NULL)
    {
      ret = -EEXIST;
      goto errout;
    }

  modreg32(0, RP2040_WATCHDOG_CTRL_ENABLE, RP2040_WATCHDOG_CTRL);

errout:
  return ret;
}
