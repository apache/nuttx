/****************************************************************************
 * drivers/net/phy/phy_gen.c
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

#include <nuttx/net/phy.h>
#include <nuttx/net/mii.h>
#include <nuttx/ethtool.h>

#include "phy.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int genphy_config_init(FAR struct phy_device_s *phydev);
static int genphy_config_aneg(FAR struct phy_device_s *phydev);
static int genphy_read_status(FAR struct phy_device_s *phydev);
static int genphy_config_power(FAR struct phy_device_s *phydev, bool enable);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct phy_driver_s g_genphy_driver =
{
  .name         = "Generic PHY",
  .config_init  = genphy_config_init,
  .config_aneg  = genphy_config_aneg,
  .read_status  = genphy_read_status,
  .config_power = genphy_config_power,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: genphy_config_init
 *
 * Description:
 *   Read MSR and ESTATUS to determine supported link modes.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int genphy_config_init(FAR struct phy_device_s *phydev)
{
  int msr;
  int estatus;
  uint32_t supported = 0;

  msr = phy_read(phydev, MII_MSR);
  if (msr < 0)
    {
      return msr;
    }

  if (msr & MII_MSR_10BASETXHALF)
    {
      supported |= SUPPORTED_10BASET_HALF;
    }

  if (msr & MII_MSR_10BASETXFULL)
    {
      supported |= SUPPORTED_10BASET_FULL;
    }

  if (msr & MII_MSR_100BASETXHALF)
    {
      supported |= SUPPORTED_100BASET_HALF;
    }

  if (msr & MII_MSR_100BASETXFULL)
    {
      supported |= SUPPORTED_100BASET_FULL;
    }

  if (msr & MII_MSR_ANEGABLE)
    {
      supported |= SUPPORTED_AUTONEG;
    }

  /* Check for extended status register for 1000BASE-T */

  if (msr & MII_MSR_ESTATEN)
    {
      estatus = phy_read(phydev, MII_ESTATUS);
      if (estatus >= 0)
        {
          if (estatus & MII_ESTATUS_1000BASETHALF)
            {
              supported |= SUPPORTED_1000BASET_HALF;
            }

          if (estatus & MII_ESTATUS_1000BASETFULL)
            {
              supported |= SUPPORTED_1000BASET_FULL;
            }
        }
    }

  supported |= SUPPORTED_TP | SUPPORTED_MII;

  phydev->supported = supported;
  phydev->advertising = supported;

  return OK;
}

/****************************************************************************
 * Name: genphy_config_aneg
 *
 * Description:
 *   Configure and restart auto-negotiation.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int genphy_config_aneg(FAR struct phy_device_s *phydev)
{
  uint16_t adv = MII_ADVERTISE_CSMA;
  uint16_t msctrl = 0;
  int ret;

  /* Build ADVERTISE register value from advertising bitmask */

  if (phydev->advertising & SUPPORTED_10BASET_HALF)
    {
      adv |= MII_ADVERTISE_10BASETXHALF;
    }

  if (phydev->advertising & SUPPORTED_10BASET_FULL)
    {
      adv |= MII_ADVERTISE_10BASETXFULL;
    }

  if (phydev->advertising & SUPPORTED_100BASET_HALF)
    {
      adv |= MII_ADVERTISE_100BASETXHALF;
    }

  if (phydev->advertising & SUPPORTED_100BASET_FULL)
    {
      adv |= MII_ADVERTISE_100BASETXFULL;
    }

  ret = phy_write(phydev, MII_ADVERTISE, adv);
  if (ret < 0)
    {
      return ret;
    }

  /* Write 1000BASE-T advertisement if supported */

  if (phydev->supported & (SUPPORTED_1000BASET_HALF |
                           SUPPORTED_1000BASET_FULL))
    {
      if (phydev->advertising & SUPPORTED_1000BASET_HALF)
        {
          msctrl |= MII_MSCONTROL_1000BASETHALF;
        }

      if (phydev->advertising & SUPPORTED_1000BASET_FULL)
        {
          msctrl |= MII_MSCONTROL_1000BASETFULL;
        }

      ret = phy_write(phydev, MII_MSCONTROL, msctrl);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Restart auto-negotiation: set ANENABLE and ANRESTART in MCR */

  return phy_modify(phydev, MII_MCR,
                    MII_MCR_ANENABLE | MII_MCR_ANRESTART,
                    MII_MCR_ANENABLE | MII_MCR_ANRESTART);
}

/****************************************************************************
 * Name: genphy_read_status
 *
 * Description:
 *   Read PHY link status, speed, and duplex from MII registers.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int genphy_read_status(FAR struct phy_device_s *phydev)
{
  int msr;
  int lpa;
  int msstatus;
  uint32_t common;

  /* Read MSR twice - link status is latched low */

  msr = phy_read(phydev, MII_MSR);
  if (msr < 0)
    {
      return msr;
    }

  msr = phy_read(phydev, MII_MSR);
  if (msr < 0)
    {
      return msr;
    }

  phydev->link = !!(msr & MII_MSR_LINKSTATUS);

  if (!phydev->link)
    {
      phydev->speed = 0;
      phydev->duplex = DUPLEX_UNKNOWN;
      return OK;
    }

  /* Read link partner ability */

  lpa = phy_read(phydev, MII_LPA);
  if (lpa < 0)
    {
      return lpa;
    }

  phydev->lp_advertising = 0;
  if (lpa & MII_LPA_10BASETXHALF)
    {
      phydev->lp_advertising |= SUPPORTED_10BASET_HALF;
    }

  if (lpa & MII_LPA_10BASETXFULL)
    {
      phydev->lp_advertising |= SUPPORTED_10BASET_FULL;
    }

  if (lpa & MII_LPA_100BASETXHALF)
    {
      phydev->lp_advertising |= SUPPORTED_100BASET_HALF;
    }

  if (lpa & MII_LPA_100BASETXFULL)
    {
      phydev->lp_advertising |= SUPPORTED_100BASET_FULL;
    }

  /* Check 1000BASE-T if supported */

  if (phydev->supported & (SUPPORTED_1000BASET_HALF |
                           SUPPORTED_1000BASET_FULL))
    {
      msstatus = phy_read(phydev, MII_MSSTATUS);
      if (msstatus >= 0)
        {
          if (msstatus & MII_MSSTATUS_LP1000BASETHALF)
            {
              phydev->lp_advertising |= SUPPORTED_1000BASET_HALF;
            }

          if (msstatus & MII_MSSTATUS_LP1000BASETFULL)
            {
              phydev->lp_advertising |= SUPPORTED_1000BASET_FULL;
            }
        }
    }

  /* Resolve speed and duplex from common capabilities */

  common = phydev->advertising & phydev->lp_advertising;

  if (common & SUPPORTED_1000BASET_FULL)
    {
      phydev->speed = 1000;
      phydev->duplex = DUPLEX_FULL;
    }
  else if (common & SUPPORTED_1000BASET_HALF)
    {
      phydev->speed = 1000;
      phydev->duplex = DUPLEX_HALF;
    }
  else if (common & SUPPORTED_100BASET_FULL)
    {
      phydev->speed = 100;
      phydev->duplex = DUPLEX_FULL;
    }
  else if (common & SUPPORTED_100BASET_HALF)
    {
      phydev->speed = 100;
      phydev->duplex = DUPLEX_HALF;
    }
  else if (common & SUPPORTED_10BASET_FULL)
    {
      phydev->speed = 10;
      phydev->duplex = DUPLEX_FULL;
    }
  else if (common & SUPPORTED_10BASET_HALF)
    {
      phydev->speed = 10;
      phydev->duplex = DUPLEX_HALF;
    }
  else
    {
      phydev->speed = 0;
      phydev->duplex = DUPLEX_UNKNOWN;
    }

  return OK;
}

/****************************************************************************
 * Name: genphy_config_power
 *
 * Description:
 *   Power the PHY up or down via the power-down bit in MCR.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *   enable - true to power up (clear PDOWN); false to power down (set PDOWN)
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int genphy_config_power(FAR struct phy_device_s *phydev, bool enable)
{
  return enable ? phy_clear_bits(phydev, MII_MCR, MII_MCR_PDOWN)
                : phy_set_bits(phydev, MII_MCR, MII_MCR_PDOWN);
}
