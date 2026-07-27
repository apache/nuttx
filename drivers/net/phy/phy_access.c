/****************************************************************************
 * drivers/net/phy/phy_access.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/net/phy.h>
#include <nuttx/net/mdio.h>

#include "phy.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy_read
 *
 * Description:
 *   Read a PHY register via the MDIO bus.
 *
 * Input Parameters:
 *   phydev  - Pointer to the PHY device structure
 *   regaddr - MII register address to read
 *
 * Returned Value:
 *   Register value (16-bit) on success; negated errno on failure.
 *
 ****************************************************************************/

int phy_read(FAR struct phy_device_s *phydev, uint8_t regaddr)
{
  uint16_t value;
  int ret;

  ret = mdio_read(phydev->bus, phydev->mdio_addr, regaddr, &value);
  if (ret < 0)
    {
      return ret;
    }

  return value;
}

/****************************************************************************
 * Name: phy_write
 *
 * Description:
 *   Write a value to a PHY register via the MDIO bus.
 *
 * Input Parameters:
 *   phydev  - Pointer to the PHY device structure
 *   regaddr - MII register address to write
 *   val     - 16-bit value to write
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int phy_write(FAR struct phy_device_s *phydev,
              uint8_t regaddr, uint16_t val)
{
  return mdio_write(phydev->bus, phydev->mdio_addr, regaddr, val);
}

/****************************************************************************
 * Name: phy_modify
 *
 * Description:
 *   Perform a read-modify-write on a PHY register. The bits selected
 *   by mask are cleared and then set according to the set parameter.
 *
 * Input Parameters:
 *   phydev  - Pointer to the PHY device structure
 *   regaddr - MII register address to modify
 *   mask    - Bitmask of bits to clear
 *   set     - Bitmask of bits to set (masked by mask)
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int phy_modify(FAR struct phy_device_s *phydev,
               uint8_t regaddr, uint16_t mask, uint16_t set)
{
  int ret;
  uint16_t value;

  ret = mdio_read(phydev->bus, phydev->mdio_addr, regaddr, &value);
  if (ret < 0)
    {
      return ret;
    }

  value = (value & ~mask) | (set & mask);
  return mdio_write(phydev->bus, phydev->mdio_addr, regaddr, value);
}

/****************************************************************************
 * Name: phy_set_bits
 *
 * Description:
 *   Set specific bits in a PHY register without affecting other bits.
 *
 * Input Parameters:
 *   phydev  - Pointer to the PHY device structure
 *   regaddr - MII register address to modify
 *   bits    - Bitmask of bits to set
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int phy_set_bits(FAR struct phy_device_s *phydev,
                 uint8_t regaddr, uint16_t bits)
{
  return phy_modify(phydev, regaddr, bits, bits);
}

/****************************************************************************
 * Name: phy_clear_bits
 *
 * Description:
 *   Clear specific bits in a PHY register without affecting other bits.
 *
 * Input Parameters:
 *   phydev  - Pointer to the PHY device structure
 *   regaddr - MII register address to modify
 *   bits    - Bitmask of bits to clear
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int phy_clear_bits(FAR struct phy_device_s *phydev,
                   uint8_t regaddr, uint16_t bits)
{
  return phy_modify(phydev, regaddr, bits, 0);
}

#ifdef CONFIG_MDIO_CLAUSE_45

/****************************************************************************
 * Name: phy_read_mmd
 *
 * Description:
 *   Read a PHY MMD (MDIO Manageable Device) register using Clause 45
 *   access over Clause 22.  Only available when CONFIG_MDIO_CLAUSE_45 set.
 *
 * Input Parameters:
 *   phydev  - Pointer to the PHY device structure
 *   devad   - MMD device address (e.g. MII_MMD_PCS)
 *   regaddr - Register address within the MMD device
 *
 * Returned Value:
 *   Register value (16-bit) on success; negated errno on failure.
 *
 ****************************************************************************/

int phy_read_mmd(FAR struct phy_device_s *phydev,
                 uint8_t devad, uint16_t regaddr)
{
  uint16_t value;
  int ret;

  ret = mdio_c45_read(phydev->bus, phydev->mdio_addr, devad, regaddr,
                      &value);
  if (ret < 0)
    {
      return ret;
    }

  return value;
}

/****************************************************************************
 * Name: phy_write_mmd
 *
 * Description:
 *   Write a PHY MMD (MDIO Manageable Device) register using Clause 45
 *   access over Clause 22.
 *
 * Input Parameters:
 *   phydev  - Pointer to the PHY device structure
 *   devad   - MMD device address (e.g. MII_MMD_PCS)
 *   regaddr - Register address within the MMD device
 *   val     - 16-bit value to write
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int phy_write_mmd(FAR struct phy_device_s *phydev,
                  uint8_t devad, uint16_t regaddr, uint16_t val)
{
  return mdio_c45_write(phydev->bus, phydev->mdio_addr,
                        devad, regaddr, val);
}

/****************************************************************************
 * Name: phy_modify_mmd
 *
 * Description:
 *   Read-modify-write a PHY MMD (MDIO Manageable Device) register using
 *   Clause 45 access over Clause 22.  The bits selected by mask are cleared
 *   and then set according to the set parameter.
 *
 * Input Parameters:
 *   phydev  - Pointer to the PHY device structure
 *   devad   - MMD device address (e.g. MII_MMD_PCS)
 *   regaddr - Register address within the MMD device
 *   mask    - Bitmask of bits to clear
 *   set     - Bitmask of bits to set (masked by mask)
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int phy_modify_mmd(FAR struct phy_device_s *phydev,
                   uint8_t devad, uint16_t regaddr,
                   uint16_t mask, uint16_t set)
{
  uint16_t value;
  int ret;

  ret = mdio_c45_read(phydev->bus, phydev->mdio_addr, devad, regaddr,
                      &value);
  if (ret < 0)
    {
      return ret;
    }

  value = (value & ~mask) | (set & mask);
  return mdio_c45_write(phydev->bus, phydev->mdio_addr, devad, regaddr,
                        value);
}

#endif /* CONFIG_MDIO_CLAUSE_45 */
