/****************************************************************************
 * boards/risc-v/eic7700x/common/src/eic7700x_wdt.c
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

/* The watchdogs, and the question every reboot should answer: why?
 *
 * The chip keeps the cause of its last reset in a register until told to
 * forget.  On a board that is expected to reboot itself out of trouble,
 * that one byte is half the value of the whole feature: a watchdog bit
 * in the boot log turns "it seems to have rebooted at some point" into
 * "it hung at 03:12 and saved itself".  So the cause is read first,
 * said aloud, kept for boardctl to serve, and then cleared so the next
 * reset writes its own story.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <syslog.h>
#include <sys/boardctl.h>

#include "eic7700x_wdt.h"
#include "hardware/eic7700x_clk.h"
#include "riscv_internal.h"

#include "board_config.h"

#ifdef CONFIG_EIC7700X_WDT

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The raw cause byte, captured before it is cleared */

static uint32_t g_reset_cause;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_reset_cause
 *
 * Description:
 *   Serve the cached cause through the standard boardctl call, mapped
 *   onto the framework's names.  The raw byte rides along in the flag
 *   for anything that wants the chip's own encoding.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET_CAUSE
int board_reset_cause(FAR struct boardioc_reset_cause_s *cause)
{
  cause->flag = g_reset_cause;

  if ((g_reset_cause & RST_CAUSE_WATCHDOG) != 0)
    {
      cause->cause = BOARDIOC_RESETCAUSE_CORE_MWDT;
    }
  else if ((g_reset_cause & RST_CAUSE_SOFTWARE) != 0)
    {
      cause->cause = BOARDIOC_RESETCAUSE_CORE_SOFT;
    }
  else if ((g_reset_cause & RST_CAUSE_KEY) != 0)
    {
      cause->cause = BOARDIOC_RESETCAUSE_PIN;
    }
  else if ((g_reset_cause &
            (RST_CAUSE_POR_INTERNAL | RST_CAUSE_POR_EXTERNAL)) != 0)
    {
      cause->cause = BOARDIOC_RESETCAUSE_SYS_CHIPPOR;
    }
  else if (g_reset_cause != 0)
    {
      cause->cause = BOARDIOC_RESETCAUSE_UNKOWN;
    }
  else
    {
      cause->cause = BOARDIOC_RESETCAUSE_NONE;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: eic7700x_board_wdt_initialize
 *
 * Description:
 *   Report why the chip last reset, then register the watchdog
 *   instances the configuration asks for as /dev/watchdog0 and upwards.
 *
 * Returned Value:
 *   Zero on success; a negated errno from the last instance that failed
 *   to register.
 *
 ****************************************************************************/

int eic7700x_board_wdt_initialize(void)
{
  char devpath[20];
  int n;
  int ret;

  /* Why did the chip last reset?  Say so, remember it, and clear it so
   * the next reset is unambiguous: the register accumulates otherwise.
   */

  g_reset_cause = getreg32(EIC7700X_CLK_BASE + EIC7700X_DIE_STATUS) & 0xff;
  putreg32(1, EIC7700X_CLK_BASE + EIC7700X_CLR_RST_STATUS);

  syslog(LOG_INFO, "wdt: last reset:%s%s%s%s%s%s (%02x)\n",
         (g_reset_cause & (RST_CAUSE_POR_INTERNAL |
                           RST_CAUSE_POR_EXTERNAL)) ? " power-on" : "",
         (g_reset_cause & RST_CAUSE_KEY)      ? " key"      : "",
         (g_reset_cause & RST_CAUSE_WATCHDOG) ? " WATCHDOG" : "",
         (g_reset_cause & RST_CAUSE_SOFTWARE) ? " software" : "",
         (g_reset_cause & (RST_CAUSE_U84_NDRESET |
                           RST_CAUSE_SCPU_NDRESET)) ? " debug" : "",
         (g_reset_cause & RST_CAUSE_OTHER_DIE) ? " other-die" : "",
         (unsigned int)g_reset_cause);

  /* Register the instances the configuration asks for.  With the
   * auto-monitor on, each one is armed and fed by the kernel from here
   * until an application claims it.
   */

  ret = OK;
  n = 0;

#ifdef CONFIG_EIC7700X_WDT0
  snprintf(devpath, sizeof(devpath), "/dev/watchdog%d", n);
  ret = eic7700x_wdt_initialize(0, devpath);
  if (ret == OK)
    {
      n++;
    }
#endif

#ifdef CONFIG_EIC7700X_WDT1
  snprintf(devpath, sizeof(devpath), "/dev/watchdog%d", n);
  ret = eic7700x_wdt_initialize(1, devpath);
  if (ret == OK)
    {
      n++;
    }
#endif

#ifdef CONFIG_EIC7700X_WDT2
  snprintf(devpath, sizeof(devpath), "/dev/watchdog%d", n);
  ret = eic7700x_wdt_initialize(2, devpath);
  if (ret == OK)
    {
      n++;
    }
#endif

#ifdef CONFIG_EIC7700X_WDT3
  snprintf(devpath, sizeof(devpath), "/dev/watchdog%d", n);
  ret = eic7700x_wdt_initialize(3, devpath);
  if (ret == OK)
    {
      n++;
    }
#endif

  if (n > 0)
    {
      syslog(LOG_INFO, "wdt: %d watchdog%s"
#ifdef CONFIG_WATCHDOG_AUTOMONITOR
             ", auto-fed"
#endif
             "\n", n, n == 1 ? "" : "s");
    }

  return ret;
}

#endif /* CONFIG_EIC7700X_WDT */
