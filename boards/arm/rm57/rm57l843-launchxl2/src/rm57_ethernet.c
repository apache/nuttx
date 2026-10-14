/****************************************************************************
 * boards/arm/rm57/rm57l843-launchxl2/src/rm57_ethernet.c
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
#include <stdint.h>

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "hardware/rm57l843_memorymap.h"
#include "rm57_ethernet.h"
#include "rm57_gio.h"

#include "rm57l843-launchxl2.h"

#ifdef CONFIG_RM57_EMAC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PHY reset timing.  The low pulse only has to be about a microsecond,
 * but up_mdelay(1) is the smallest thing worth trusting here and costs
 * nothing at boot.  The DP83630 needs a few milliseconds after reset is
 * released before it answers MDIO, so allow a generous margin.
 */

#define PHY_RESET_ASSERT_MS   1
#define PHY_RESET_SETTLE_MS   10

/* Die ID registers (SPNU562A system module) - no CONFIG_RM57_EMAC-
 * independent consumer exists yet in the shared hardware/rm57_sys.h
 * header, so the addresses are computed inline here rather than adding
 * board-specific-use macros to a shared file.
 */

#define RM57_SYS_DIEIDL (RM57_SYS1_BASE + 0x007c)
#define RM57_SYS_DIEIDH (RM57_SYS1_BASE + 0x0080)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_phy_powerup
 *
 * Description:
 *   Bring the board's DP83630 out of power-down and out of reset.  Both
 *   of its control inputs default to the "off" state (see the
 *   GIO_PHY_PWRDOWN/GIO_PHY_RESET comments in rm57l843-launchxl2.h), so
 *   without this the PHY never runs: it does not answer MDIO, the RJ45
 *   link LED is only lit by leakage through its pull-up, and the EMAC
 *   driver reports MDIO_ALIVE=0 with every register otherwise correct.
 *
 *   The PHY's XIN reference clock comes from the MCU's ECLK1 terminal
 *   and is already running by the time this is called, from
 *   rm57_clockconfig() at boot (BOARD_ECLK1_DIV in board.h).  That
 *   ordering matters: the DP83630 latches its strap pins - including
 *   the PHY address - when reset is released, and it needs its clock to
 *   do so.
 *
 ****************************************************************************/

void rm57_phy_powerup(void)
{
  /* Leave power-down: the pinset drives GIOA[3] high, which has to
   * overcome the board's 2.2k pulldown on that net.
   */

  rm57_configgio(GIO_PHY_PWRDOWN);

  /* Assert reset (the pinset drives GIOA[4] low), hold, then release */

  rm57_configgio(GIO_PHY_RESET);
  up_mdelay(PHY_RESET_ASSERT_MS);

  rm57_giowrite(GIO_PHY_RESET, true);
  up_mdelay(PHY_RESET_SETTLE_MS);
}

#ifdef CONFIG_NETDEV_LATEINIT

/****************************************************************************
 * Name: rm57_eth_setup
 *
 * Description:
 *   Derive a locally-administered MAC address from the device's unique
 *   die ID and register the EMAC driver.
 *
 *   Only built under CONFIG_NETDEV_LATEINIT: otherwise
 *   arm_netinitialize() (rm57_netinitialize.c) already registers the
 *   EMAC with a fixed placeholder MAC address before rm57_bringup()
 *   ever runs, and calling rm57_ethinitialize() a second time here
 *   would attempt to register the same netdev twice.
 *
 ****************************************************************************/

int rm57_eth_setup(void)
{
  uint8_t mac[6];
  uint32_t dieidl;
  uint32_t dieidh;

  dieidl = getreg32(RM57_SYS_DIEIDL);
  dieidh = getreg32(RM57_SYS_DIEIDH);

  /* Locally-administered, unicast (bit 1 set, bit 0 clear of the first
   * octet) - avoids colliding with any IEEE-assigned OUI.
   */

  mac[0] = 0x02;
  mac[1] = (uint8_t)(dieidh >> 24);
  mac[2] = (uint8_t)(dieidh >> 16);
  mac[3] = (uint8_t)(dieidh >> 8);
  mac[4] = (uint8_t)(dieidl >> 8);
  mac[5] = (uint8_t)(dieidl >> 0);

  return rm57_ethinitialize(0, mac);
}

#endif /* CONFIG_NETDEV_LATEINIT */

#endif /* CONFIG_RM57_EMAC */
