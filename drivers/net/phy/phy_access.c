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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phy_read_page_nolock
 *
 * Description:
 *   Read the currently selected page via the vendor driver's read_page
 *   callback.  Lockless: the caller must already hold the MDIO bus lock.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *
 * Returned Value:
 *   The current page (>= 0) on success; -ENOTSUP if the driver has no
 *   read_page callback; a negated errno on failure.
 *
 ****************************************************************************/

static inline int phy_read_page_nolock(FAR struct phy_device_s *phydev)
{
  if (phydev->drv->read_page == NULL)
    {
      return -ENOTSUP;
    }

  return phydev->drv->read_page(phydev);
}

/****************************************************************************
 * Name: phy_write_page_nolock
 *
 * Description:
 *   Select 'page' via the vendor driver's write_page callback.  Lockless:
 *   the caller must already hold the MDIO bus lock.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *   page   - Page number to select
 *
 * Returned Value:
 *   OK on success; -ENOTSUP if the driver has no write_page callback; a
 *   negated errno on failure.
 *
 ****************************************************************************/

static inline int phy_write_page_nolock(FAR struct phy_device_s *phydev,
                                        int page)
{
  if (phydev->drv->write_page == NULL)
    {
      return -ENOTSUP;
    }

  return phydev->drv->write_page(phydev, page);
}

/****************************************************************************
 * Name: phy_select_page
 *
 * Description:
 *   Take the MDIO bus lock, save the current page, and select the requested
 *   page.  On success the bus lock is held and the previous page (>= 0) is
 *   returned; the caller must release it via phy_restore_page().
 *
 *   On failure a negated errno is returned and the bus lock is NOT held:
 *   if acquiring the lock fails there is nothing to release, and if reading
 *   or switching the page fails this function releases the lock itself.
 *   The caller must therefore only call phy_restore_page() when
 *   phy_select_page() succeeds (returns >= 0).
 *
 *   Between phy_select_page() and phy_restore_page() the caller uses the
 *   lockless mdio_*_nolock() primitives to access registers on the selected
 *   page.
 *
 *   The page-select register and switching mechanism are vendor specific,
 *   so the actual page read/write is delegated to the driver's
 *   read_page/write_page callbacks; the core never hardcodes a page
 *   register.
 *
 * Input Parameters:
 *   phydev - Pointer to the PHY device structure
 *   page   - Page number to select
 *
 * Returned Value:
 *   The previous page number (>= 0, lock held) on success, or a negated
 *   errno (lock released) on failure.
 *
 ****************************************************************************/

static int phy_select_page(FAR struct phy_device_s *phydev, int page)
{
  int oldpage;
  int ret;

  ret = mdio_lock(phydev->bus);
  if (ret < 0)
    {
      /* Lock not acquired: nothing to release, and the caller must not
       * call phy_restore_page().
       */

      return ret;
    }

  oldpage = phy_read_page_nolock(phydev);
  if (oldpage < 0)
    {
      ret = oldpage;
      goto err_unlock;
    }

  if (oldpage != page)
    {
      ret = phy_write_page_nolock(phydev, page);
      if (ret < 0)
        {
          goto err_unlock;
        }
    }

  return oldpage;

err_unlock:

  /* The page could not be read or switched: drop the lock we took so the
   * caller can simply return the error without calling phy_restore_page().
   */

  mdio_unlock(phydev->bus);
  return ret;
}

/****************************************************************************
 * Name: phy_restore_page
 *
 * Description:
 *   Restore the page saved by phy_select_page() and release the MDIO bus
 *   lock.  Must only be called after a successful phy_select_page() (i.e.
 *   with the bus lock held); the page-selection failure paths release the
 *   lock themselves and must not reach here.
 *
 * Input Parameters:
 *   phydev  - Pointer to the PHY device structure
 *   oldpage - Page returned by a successful phy_select_page() (>= 0)
 *   ret     - Result of the operation performed on the selected page
 *
 * Returned Value:
 *   ret if it was negative; otherwise the page-restore error if that
 *   failed; otherwise ret.
 *
 ****************************************************************************/

static int phy_restore_page(FAR struct phy_device_s *phydev,
                            int oldpage, int ret)
{
  int ret2;

  /* Restore the saved page.  Propagate the restore error only if the
   * operation itself succeeded.
   */

  ret2 = phy_write_page_nolock(phydev, oldpage);
  if (ret >= 0 && ret2 < 0)
    {
      ret = ret2;
    }

  mdio_unlock(phydev->bus);
  return ret;
}

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

/****************************************************************************
 * Name: phy_read_paged
 *
 * Description:
 *   Read a register on a non-default PHY page.  The select-page ->
 *   read -> restore-page sequence runs atomically under the MDIO bus lock,
 *   so it cannot interleave with other bus users (e.g. MII ioctl).
 *
 * Input Parameters:
 *   phydev  - Pointer to the PHY device structure
 *   page    - Page number to select (via the driver's write_page callback)
 *   regaddr - MII register address to read within that page
 *   value   - Location to store the 16-bit value read
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int phy_read_paged(FAR struct phy_device_s *phydev,
                   int page, uint8_t regaddr, FAR uint16_t *value)
{
  int oldpage;
  int ret;

  oldpage = phy_select_page(phydev, page);
  if (oldpage < 0)
    {
      return oldpage;
    }

  ret = mdio_read_nolock(phydev->bus, phydev->mdio_addr, regaddr, value);

  return phy_restore_page(phydev, oldpage, ret);
}

/****************************************************************************
 * Name: phy_write_paged
 *
 * Description:
 *   Write a register on a non-default PHY page.  Atomic against other bus
 *   users (see phy_read_paged).
 *
 * Input Parameters:
 *   phydev  - Pointer to the PHY device structure
 *   page    - Page number to select (via the driver's write_page callback)
 *   regaddr - MII register address to write within that page
 *   value   - 16-bit value to write
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int phy_write_paged(FAR struct phy_device_s *phydev,
                    int page, uint8_t regaddr, uint16_t value)
{
  int oldpage;
  int ret;

  oldpage = phy_select_page(phydev, page);
  if (oldpage < 0)
    {
      return oldpage;
    }

  ret = mdio_write_nolock(phydev->bus, phydev->mdio_addr, regaddr, value);

  return phy_restore_page(phydev, oldpage, ret);
}

/****************************************************************************
 * Name: phy_modify_paged
 *
 * Description:
 *   Read-modify-write a register on a non-default PHY page, atomically
 *   under the MDIO bus lock.
 *
 * Input Parameters:
 *   phydev  - Pointer to the PHY device structure
 *   page    - Page number to select (via the driver's write_page callback)
 *   regaddr - MII register address to modify within that page
 *   mask    - Bitmask of bits to clear
 *   set     - Bitmask of bits to set (applied after mask)
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int phy_modify_paged(FAR struct phy_device_s *phydev,
                     int page, uint8_t regaddr,
                     uint16_t mask, uint16_t set)
{
  uint16_t value;
  int oldpage;
  int ret;

  oldpage = phy_select_page(phydev, page);
  if (oldpage < 0)
    {
      return oldpage;
    }

  ret = mdio_read_nolock(phydev->bus, phydev->mdio_addr, regaddr, &value);
  if (ret >= 0)
    {
      value = (value & ~mask) | (set & mask);
      ret = mdio_write_nolock(phydev->bus, phydev->mdio_addr, regaddr,
                              value);
    }

  return phy_restore_page(phydev, oldpage, ret);
}

