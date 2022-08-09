/****************************************************************************
 * arch/arm/src/phy62xx/phyplus_wdt.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include "phyplus_wdt.h"
#include "jump_function.h"
#include "mcu_phy_bumbee.h"
#include "clock.h"

#if defined(CONFIG_WATCHDOG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WATCHDOG_1000MS_EVENT    0x0001
#define WATCHDOG_1000MS_CYCLE    1000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

void __attribute__((used))  hal_WATCHDOG_IRQHandler(void)
{
    volatile uint32_t a;
    a = AP_WDT->EOI;
    AP_WDT->CRR = 0x76;
}

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* "Lower half" driver methods **********************************************/

static int    phyplus_wdt_start(struct watchdog_lowerhalf_s *lower);
static int    phyplus_wdt_stop(struct watchdog_lowerhalf_s *lower);
static int    phyplus_wdt_keepalive(struct watchdog_lowerhalf_s *lower);
static int    phyplus_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                                    struct watchdog_status_s *status);
static int    phyplus_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                                     uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = phyplus_wdt_start,
  .stop       = phyplus_wdt_stop,
  .keepalive  = phyplus_wdt_keepalive,
  .getstatus  = phyplus_wdt_getstatus,
  .settimeout = phyplus_wdt_settimeout,
  .capture    = NULL, /* phyplus_wdt_capture, */
  .ioctl      = NULL, /* phyplus_wdt_ioctl, */
};

struct phyplus_lowerhalf_s
{
  const struct watchdog_ops_s  *ops; /* Lower half operations */
  bool     started;                  /* true: The watchdog timer has been started */
  bool     intr_mode;                /* 0: not use intr_callback handle, 1: use intr_callback handle */
  WDG_CYCLE_Type_e wdt_cycle;
};

static struct phyplus_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pp_watchdog_init(void)
{
  wdinfo("pp_init\n");

  uint8_t delay;
  hal_clk_gate_enable(MOD_WDT);

  /* wdt_reset_en : software reset enable */

  /* 1---enable  0---disable */

  if ((AP_PCR->SW_RESET0 & 0x04) == 0)
    {
      AP_PCR->SW_RESET0 |= 0x04;
      delay = 20;

      while (delay-- > 0);
    }

  /* wdt software reset enable in 0xc */

  /* 1---enable  0---disable */

  if ((AP_PCR->SW_RESET2 & 0x04) == 0)
    {
      AP_PCR->SW_RESET2 |= 0x04;
      delay = 20;

      while (delay-- > 0);
    }

  /* watchdog software reset  */

  /* 1---normal  0---reset */

  AP_PCR->SW_RESET2 &= ~0x20;
  delay = 20;

  while (delay-- > 0);

  AP_PCR->SW_RESET2 |= 0x20;
  delay = 20;

  while (delay-- > 0);

  return 0;
}

static int pp_watchdog_start(bool interrupt_mode, WDG_CYCLE_Type_e cycle)
{
  wdinfo("pp_start\n");
  volatile uint32_t a;

  a = AP_WDT->EOI;
  AP_WDT->TORR = cycle;
  if (TRUE == interrupt_mode)
    {
      JUMP_FUNCTION(WDT_IRQ_HANDLER) = (uint32_t)&hal_WATCHDOG_IRQHandler;
      AP_WDT->CR = 0x1f;
      NVIC_SetPriority((IRQn_Type)WDT_IRQn, IRQ_PRIO_HAL);
      NVIC_EnableIRQ((IRQn_Type)WDT_IRQn);
    }
  else
    {
      JUMP_FUNCTION(WDT_IRQ_HANDLER) = 0;
      AP_WDT->CR = 0x1d;
      NVIC_DisableIRQ((IRQn_Type)WDT_IRQn);
    }
  AP_WDT->CRR = 0x76;

  return 0;
}

static int pp_watchdog_stop(void)
{
  wdinfo("pp_stop not support\n");

  /* phyplus current not support stop watchdog while watchdog is running */

  return 0;
}

static int __attribute__((used)) pp_watchdog_settimer(WDG_CYCLE_Type_e cycle)
{
  wdinfo("pp_settimer\n");
  AP_WDT->TORR = cycle;
  return 0;
}

static int pp_watchdog_feed(void)
{
  wdinfo("pp_feed\n");
  AP_WDT->CRR = 0x76;
  return 0;
}

/****************************************************************************
 * Name: phyplus_wdt_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int phyplus_wdt_start(struct watchdog_lowerhalf_s *lower)
{
  struct phyplus_lowerhalf_s *priv =
    (struct phyplus_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("wdt_start\n");

  DEBUGASSERT(priv);

  /* Have we already been started? */

  if (!priv->started)
    {
      /* Enable IWDG (the LSI oscillator will be enabled by hardware).  NOTE:
       * If the "Hardware watchdog" feature is enabled through the device
       * option bits, the watchdog is automatically enabled at power-on.
       */

      flags           = enter_critical_section();
      pp_watchdog_start(priv->intr_mode, priv->wdt_cycle);
      priv->started   = true;
      leave_critical_section(flags);
    }

  return OK;
}

/****************************************************************************
 * Name: phyplus_wdt_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int phyplus_wdt_stop(struct watchdog_lowerhalf_s *lower)
{
  struct phyplus_lowerhalf_s *priv =
    (struct phyplus_lowerhalf_s *)lower;

  wdinfo("wdt_stop\n");

  if (priv->started)
    {
      priv->started   = false;
      pp_watchdog_stop();
    }

  return -ENOSYS;
}

/****************************************************************************
 * Name: phyplus_wdt_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int phyplus_wdt_keepalive(struct watchdog_lowerhalf_s *lower)
{
  /* struct phyplus_lowerhalf_s *priv =
   * (struct phyplus_lowerhalf_s *)lower;
   */

  irqstate_t flags;

  wdinfo("wdt_feed\n");

  flags = enter_critical_section();

  pp_watchdog_feed();

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: phyplus_wdt_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the
 *            "lower-half" driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int phyplus_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                                 struct watchdog_status_s *status)
{
  struct phyplus_lowerhalf_s *priv =
    (struct phyplus_lowerhalf_s *)lower;
  DEBUGASSERT(priv);

  wdinfo("wdt getstatus\n");

  wdinfo("CR:%08x , TORR:%08lx , CCVR:%08lx , STAT:%08x\n", AP_WDT->CR ,
      AP_WDT->TORR , AP_WDT->CCVR , AP_WDT->STAT);

  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }
  else
    {
      status->flags |= WDFLAGS_RESET;
    }

  status->timeout = 0x0;
  status->timeleft = AP_WDT->CCVR;

  return OK;
}

/****************************************************************************
 * Name: phhyplus_wdt_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int phyplus_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                                  uint32_t timeout)
{
  struct phyplus_lowerhalf_s *priv =
    (struct phyplus_lowerhalf_s *)lower;

  wdinfo("wdt set timeout timeout=%lu\n", timeout);
  timeout = timeout / 1000;

  if (timeout < 4)
    {
      priv->wdt_cycle = WDG_2S;
    }
  else if((timeout < 8) && (timeout >= 4))
    {
      priv->wdt_cycle = WDG_4S;
    }
  else if((timeout < 16) && (timeout >= 8))
    {
      priv->wdt_cycle = WDG_8S;
    }
  else if((timeout < 32) && (timeout >= 16))
    {
      priv->wdt_cycle = WDG_16S;
    }
  else if((timeout < 64) && (timeout >= 32))
    {
      priv->wdt_cycle = WDG_32S;
    }
  else if((timeout < 128) && (timeout >= 64))
    {
      priv->wdt_cycle = WDG_64S;
    }
  else if((timeout < 256) && (timeout >= 128))
    {
      priv->wdt_cycle = WDG_128S;
    }
  else if(timeout > 256)
    {
      priv->wdt_cycle = WDG_256S;
    }

  AP_WDT->TORR = priv->wdt_cycle;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phyplus_wdt_initialize
 *
 * Description:
 *   Initialize the IWDG watchdog timer.  The watchdog timer is initialized
 *   and registers as 'devpath'.  The initial state of the watchdog timer is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *   lsifreq - The calibrated LSI clock frequency
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void phyplus_wdt_initialize(const char *devpath)
{
  struct phyplus_lowerhalf_s *priv = &g_wdgdev;

  wdinfo("wdt initialize\n");

  /* Initialize the driver state structure. */

  priv->ops     = &g_wdgops;
  priv->started = false;
  priv->intr_mode = false;
  priv->wdt_cycle = WDG_2S;

  pp_watchdog_init();

  watchdog_register(devpath, (struct watchdog_lowerhalf_s *)priv);
}

#endif /* CONFIG_WATCHDOG */

