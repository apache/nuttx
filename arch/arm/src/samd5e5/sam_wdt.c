/****************************************************************************
 * arch/arm/src/samd5e5/sam_wdt.c
 *
 *   Copyright 2020 Falker Automacao Agricola LTDA.
 *   Author: Leomar Mateus Radke <leomar@falker.com.br>
 *   Author: Ricardo Wartchow <wartchow@gmail.com>
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

#include <sys/types.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/timers/watchdog.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"
#include "sam_periphclks.h"
#include "sam_wdt.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_SAMD5E5_WDT)

#ifndef BOARD_SCLK_FREQUENCY
#  define BOARD_SCLK_FREQUENCY 32768
#endif

#define WDT_FCLK        (BOARD_SCLK_FREQUENCY / 128)
#define WDT_MAXTIMEOUT  ((1000 * (WDT_MR_WDV_MAX+1)) / WDT_FCLK)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/** N clock cycles */

#define WDT_CLK_8CYCLE      8
#define WDT_CLK_16CYCLE     16
#define WDT_CLK_32CYCLE     32
#define WDT_CLK_64CYCLE     64
#define WDT_CLK_128CYCLE    128
#define WDT_CLK_256CYCLE    256
#define WDT_CLK_512CYCLE    512
#define WDT_CLK_1024CYCLE   1024
#define WDT_CLK_2048CYCLE   2048
#define WDT_CLK_4096CYCLE   4096
#define WDT_CLK_8192CYCLE   8192
#define WDT_CLK_16384CYCLE  16384

/**
 * \brief Macro is used to indicate the rate of second/millisecond
 */

#define WDT_PERIOD_RATE 1000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/**
 * This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct sam_lowerhalf_s
{
  FAR const struct watchdog_ops_s  *ops;  /* Lower half operations */
  uint32_t timeout;                       /* The (actual) selected timeout */
  uint32_t lastreset;                     /* The last reset time */
  bool     started;                       /* true: The watchdog timer has been started */
  uint8_t  prescaler;                     /* Clock prescaler value */
  uint16_t reload;                        /* Timer reload value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

void sam_sync_wdt(int value);

/* "Lower half" driver methods **********************************************/

static int      sam_start(FAR struct watchdog_lowerhalf_s *lower);
static int      sam_stop(FAR struct watchdog_lowerhalf_s *lower);
static int      sam_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int      sam_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                  FAR struct watchdog_status_s *status);
static int      sam_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = sam_start,
  .stop       = sam_stop,
  .keepalive  = sam_keepalive,
  .getstatus  = sam_getstatus,
  .settimeout = sam_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

/* "Lower half" driver state */

static struct sam_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void sam_sync_wdt(int value)
{
  while ((getreg32(SAM_WDT_SYNCBUSY) & value) != 0);
}

void sam_wdt_dumpregs(void)
{
  wdinfo("WDT  Regs:\n");
  wdinfo("     INTENCLR:  %02x\n", getreg8(SAM_WDT_INTENCLR));
  wdinfo("     INTENSET:  %02x\n", getreg8(SAM_WDT_INTENSET));
  wdinfo("      INTFLAG:  %02x\n", getreg8(SAM_WDT_INTFLAG));
  wdinfo("        CTRLA:  %02x\n", getreg8(SAM_WDT_CTRLA));
  wdinfo("       CONFIG:  %02x\n", getreg8(SAM_WDT_CONFIG));
  wdinfo("          EWC:  %02x\n", getreg8(SAM_WDT_EWCTRL));
  wdinfo("        CLEAR:  %02x\n", getreg8(SAM_WDT_CLEAR));
}

/****************************************************************************
 * Name: sam_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *  lower - A pointer the publicly visible representation
 *  of the "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct sam_lowerhalf_s *priv = (FAR struct sam_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry: started=%d\n");
  DEBUGASSERT(priv);

  /* Have we already been started? */

  if (!priv->started)
    {
      /* Set up prescaler and reload value for the selected timeout before
       * starting the watchdog timer.
       */

      /* Enable IWDG (the LSI oscillator will be enabled by hardware).
       * If the "Hardware watchdog" feature is enabled
       * through the device option bits,
       * the watchdog is automatically enabled at power-on.
       */

      flags = enter_critical_section();

      putreg8(WDT_CTRLA_ENABLE, SAM_WDT_CTRLA);
      sam_sync_wdt(WDT_SYNCBUSY_ENABLE);
      priv->lastreset = clock_systime_ticks();
      priv->started   = true;

      leave_critical_section(flags);
    }

  return OK;
}

/****************************************************************************
 * Name: sam_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  /* The watchdog is always disabled after a reset. It is enabled by clearing
   * the WDDIS bit in the WDT_CR register, then it cannot be disabled again
   * except by a reset.
   */

  wdinfo("Entry\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: sam_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 *   The application program must write in the SAM_WDT_CLEAR register
 *   at regular intervals during normal operation to prevent an MCU reset.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation
 *           of the "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct sam_lowerhalf_s *priv = (FAR struct sam_lowerhalf_s *)lower;
  irqstate_t flags;

  /* Reload the WDT timer */

  flags = enter_critical_section();

  putreg32(WDT_CLEAR_CLEAR, SAM_WDT_CLEAR);
  priv->lastreset = clock_systime_ticks();

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of
 *            the "lower-half" driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                           FAR struct watchdog_status_s *status)
{
  FAR struct sam_lowerhalf_s *priv = (FAR struct sam_lowerhalf_s *)lower;
  uint32_t ticks;
  uint32_t elapsed;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  /* Return the actual timeout in milliseconds */

  status->timeout = priv->timeout;

  /* Get the elapsed time since the last ping */

  ticks   = clock_systime_ticks() - priv->lastreset;
  elapsed = (int32_t)TICK2MSEC(ticks);

  if (elapsed > priv->timeout)
    {
      elapsed = priv->timeout;
    }

  /* Return the approximate time until the watchdog timer expiration */

  status->timeleft = priv->timeout - elapsed;

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08x\n", status->flags);
  wdinfo("  timeout  : %d\n", status->timeout);
  wdinfo("  timeleft : %d\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: sam_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of
 *             the "lower-half" driver state structure.
 *   timeout - The new timeout value in millisecnds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int sam_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  FAR struct sam_lowerhalf_s *priv = (FAR struct sam_lowerhalf_s *)lower;
  uint64_t            tmp;
  uint32_t            period_cycles;
  uint8_t             timeout_period = WDT_CONFIG_PER_16K;
  irqstate_t flags;

  DEBUGASSERT(priv);
  wdinfo("Entry: timeout=%d\n", timeout);

  /* Can this timeout be represented? */

  flags = enter_critical_section();

  /* calc the period cycles corresponding to timeout period */

  tmp = (uint64_t)timeout * WDT_PERIOD_RATE;

  /* check whether overflow */

  if (tmp >> 32)
  return -ERANGE;

  period_cycles = (uint32_t)tmp;

  /* calc the register value corresponding to period cysles */

  switch (period_cycles)
  {
    case WDT_CLK_8CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_8;
    break;
    case WDT_CLK_16CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_16;
    break;
    case WDT_CLK_32CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_32;
    break;
    case WDT_CLK_64CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_64;
    break;
    case WDT_CLK_128CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_128;
    break;
    case WDT_CLK_256CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_256;
    break;
    case WDT_CLK_512CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_512;
    break;
    case WDT_CLK_1024CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_1K;
    break;
    case WDT_CLK_2048CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_2K;
    break;
    case WDT_CLK_4096CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_4K;
    break;
    case WDT_CLK_8192CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_8K;
    break;
    case WDT_CLK_16384CYCLE *WDT_PERIOD_RATE:
      timeout_period = WDT_CONFIG_PER_16K;
    break;
  }

  if (!priv->started)
    putreg8(WDT_CONFIG_PER_16K, SAM_WDT_CONFIG);
  else
    putreg8(timeout_period, SAM_WDT_CONFIG);

  priv->reload = timeout_period;
  wdinfo("fwdt=%d reload=%d timout=%d\n",
         WDT_FCLK, timeout_period, priv->timeout);
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_wdt_initialize
 *
 * Description:
 *   Initialize the WDT watchdog timer.  The watchdog timer
 *   is initialized and registers as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_wdt_initialize(FAR const char *devpath)
{
  FAR struct sam_lowerhalf_s *priv = &g_wdgdev;
  DEBUGASSERT((getreg8(SAM_WDT_CTRLA) & WDT_CTRLA_ENABLE) == 0);
  sam_apb_wdt_enableperiph();

  /* Initialize the driver state structure. */

  priv->ops = &g_wdgops;
  priv->started = false;
  sam_settimeout((FAR struct watchdog_lowerhalf_s *)priv,
                  BOARD_SCLK_FREQUENCY / 2);
  (void)watchdog_register(devpath, (FAR struct watchdog_lowerhalf_s *)priv);
}

#endif /* CONFIG_WATCHDOG && CONFIG__WDT */
