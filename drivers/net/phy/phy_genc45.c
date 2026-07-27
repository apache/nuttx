/****************************************************************************
 * drivers/net/phy/phy_genc45.c
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

#include <debug.h>
#include <syslog.h>

#include <nuttx/net/phy.h>
#include <nuttx/net/mii.h>
#include <nuttx/ethtool.h>

#include "phy.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int genphy_c45_read_link(FAR struct phy_device_s *phydev);
static int genphy_c45_read_status(FAR struct phy_device_s *phydev);
static int genphy_c45_config_aneg(FAR struct phy_device_s *phydev);
static int genphy_c45_pma_read_abilities(FAR struct phy_device_s *phydev);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct phy_driver_s g_genphy_c45_driver =
{
  .name        = "Generic C45 PHY",
  .config_init = genphy_c45_pma_read_abilities,
  .config_aneg = genphy_c45_config_aneg,
  .read_status = genphy_c45_read_status,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: genphy_c45_read_link
 *
 * Description:
 *   Read link status from PCS (MMD3) status register. The link status
 *   bit is latched-low, so we read twice to get current state.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int genphy_c45_read_link(FAR struct phy_device_s *phydev)
{
  int val;

  /* Read PCS status - latched low, read twice */

  val = phy_read_mmd(phydev, MII_MMD_PCS, MII_MMD_STAT1);
  if (val < 0)
    {
      return val;
    }

  val = phy_read_mmd(phydev, MII_MMD_PCS, MII_MMD_STAT1);
  if (val < 0)
    {
      return val;
    }

  phydev->link = !!(val & MII_MMD_STAT1_LSTATUS);
  return OK;
}

/****************************************************************************
 * Name: genphy_c45_read_status
 *
 * Description:
 *   Read PHY link status, speed, and duplex from C45 MMD registers.
 *   Reads PCS for link, PMA for speed, AN for negotiation result.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int genphy_c45_read_status(FAR struct phy_device_s *phydev)
{
  int ret;
  int val;
  int ctrl2;

  /* Read link status from PCS */

  ret = genphy_c45_read_link(phydev);
  if (ret < 0)
    {
      return ret;
    }

  if (!phydev->link)
    {
      phydev->speed = 0;
      phydev->duplex = DUPLEX_UNKNOWN;
      return OK;
    }

  /* Read speed from PMA/PMD CTRL1 */

  val = phy_read_mmd(phydev, MII_MMD_PMAPMD, MII_MMD_CTRL1);
  if (val < 0)
    {
      return val;
    }

  /* Decode speed from CTRL1 speed selection bits 6 and 13 (mask 0x2040),
   * per IEEE 802.3 Clause 45 PMA/PMD control register 1.
   *   {13,6} = 00 -> 10M, 10 -> 100M, 01 -> 1000M, 11 -> 10G or higher
   */

  switch (val & 0x2040)
    {
      case 0x0000:                 /* bits {13,6} = 00 */
        phydev->speed = 10;
        break;

      case 0x2000:                 /* bits {13,6} = 10 (bit13 set) */
        phydev->speed = 100;
        break;

      case 0x0040:                 /* bits {13,6} = 01 (bit6 set) */
        phydev->speed = 1000;
        break;

      case 0x2040:                 /* bits {13,6} = 11 -> consult CTRL2 */
        ctrl2 = phy_read_mmd(phydev, MII_MMD_PMAPMD, MII_MMD_CTRL2);
        if (ctrl2 < 0)
          {
            return ctrl2;
          }

        if ((ctrl2 & MII_MMD_PMA_CTRL2_TYPE) == MII_MMD_PMA_CTRL2_10GBT)
          {
            phydev->speed = 10000;
          }
        else
          {
            phydev->speed = 0;
          }

        break;

      default:
        phydev->speed = 0;
        break;
    }

  /* C45 PHYs are typically full duplex */

  phydev->duplex = DUPLEX_FULL;

  return OK;
}

/****************************************************************************
 * Name: genphy_c45_config_aneg
 *
 * Description:
 *   Enable and restart auto-negotiation via AN MMD (MMD7).
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int genphy_c45_config_aneg(FAR struct phy_device_s *phydev)
{
  /* Enable and restart AN */

  return phy_write_mmd(phydev, MII_MMD_AN, MII_MMD_AN_CTRL1,
                       MII_MMD_AN_CTRL1_ENABLE | MII_MMD_AN_CTRL1_RESTART);
}

/****************************************************************************
 * Name: genphy_c45_pma_read_abilities
 *
 * Description:
 *   Read PMA extended abilities to determine supported speeds.
 *   Used as config_init for the generic C45 driver.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int genphy_c45_pma_read_abilities(FAR struct phy_device_s *phydev)
{
  int val;
  uint32_t supported = 0;

  /* Read PMA/PMD extended abilities (reg 11) */

  val = phy_read_mmd(phydev, MII_MMD_PMAPMD, MII_MMD_PMA_EXTABLE);
  if (val < 0)
    {
      /* If not available, assume the same basic 10/100/1000 capabilities
       * as the normal path below so the advertised set is consistent.
       */

      supported = SUPPORTED_10BASET_FULL |
                  SUPPORTED_100BASET_FULL |
                  SUPPORTED_1000BASET_FULL |
                  SUPPORTED_AUTONEG |
                  SUPPORTED_TP | SUPPORTED_MII;
      phydev->supported = supported;
      phydev->advertising = supported;
      return OK;
    }

  /* Always assume 10/100/1000 capable for C45 PHYs */

  supported = SUPPORTED_10BASET_FULL |
              SUPPORTED_100BASET_FULL |
              SUPPORTED_1000BASET_FULL |
              SUPPORTED_AUTONEG;

  if (val & MII_MMD_PMA_EXTABLE_10GBT)
    {
      supported |= SUPPORTED_10000BASET_FULL;
    }

  supported |= SUPPORTED_TP | SUPPORTED_MII;

  phydev->supported = supported;
  phydev->advertising = supported;

  return OK;
}
