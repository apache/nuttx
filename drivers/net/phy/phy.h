/****************************************************************************
 * drivers/net/phy/phy.h
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

#ifndef __DRIVERS_NET_PHY_PHY_H
#define __DRIVERS_NET_PHY_PHY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/net/phy.h>

#ifdef CONFIG_NET_PHYLIB

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* PHY driver callbacks.
 *
 * This is the interface a vendor PHY driver implements and hands to
 * phy_driver_register().  It is internal to the phylib core and the vendor
 * PHY drivers under drivers/net/phy; it is not part of the public API in
 * include/nuttx/net/phy.h (which carries only a forward declaration for the
 * phydev->drv back-pointer).
 */

struct phy_driver_s
{
  FAR const char *name;
  uint32_t phy_id;
  uint32_t phy_id_mask;

  CODE int (*config_init)(FAR struct phy_device_s *phydev);
  CODE int (*config_aneg)(FAR struct phy_device_s *phydev);
  CODE int (*read_status)(FAR struct phy_device_s *phydev);

  /* config_power - Power the PHY hardware up or down.  'enable' selects
   *                power up (true) or power down (false).  Optional: a
   *                driver with no power-management support leaves this NULL
   *                (treated as a no-op).
   */

  CODE int (*config_power)(FAR struct phy_device_s *phydev, bool enable);

  /* Interrupt support (optional).
   *
   * config_intr      - Enable or disable the PHY's interrupt output (INTB),
   *                    e.g. program the PHY interrupt-enable register.
   *                    'enable' selects enable (true) or disable (false).
   *                    Should also clear any pending interrupts before
   *                    enabling and after disabling.
   *
   * handle_interrupt - PHY interrupt handler.  The driver reads its own
   *                    vendor interrupt-status register, decodes ALL pending
   *                    sources, services/acknowledges them, and decides what
   *                    to do.  Return value:
   *                      >0 : a link-relevant event is pending -> the state
   *                           machine then re-reads link status.
   *                       0 : no link-relevant event           -> link state
   *                           left untouched (e.g. a pure diagnostic event
   *                           handled internally by the driver).
   *                      <0 : MDIO/HW failure (PHY unreachable) -> the state
   *                           machine enters PHY_ERROR.
   *                    Non-link events (undervoltage / overtemp / jabber /
   *                    remote-fault) are handled inside this callback and
   *                    must NOT be pushed into the link state machine; if
   *                    such an event also drops the link, return >0 so link
   *                    status is re-read and reflected truthfully.
   *
   * Both NULL => the PHY can only be used in polling mode.
   */

  CODE int (*config_intr)(FAR struct phy_device_s *phydev, bool enable);
  CODE int (*handle_interrupt)(FAR struct phy_device_s *phydev);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Generic PHY driver instances shared across the phylib core (used by
 * phy_find_driver() as the C22/C45 fallbacks).
 */

extern const struct phy_driver_s g_genphy_driver;
#ifdef CONFIG_MDIO_CLAUSE_45
extern const struct phy_driver_s g_genphy_c45_driver;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* PHY driver registration (phy_device.c).  Called by vendor PHY drivers to
 * register their phy_driver_s callback table; not part of the public API.
 */

int phy_driver_register(FAR const struct phy_driver_s *drv);

/* PHY register utilities (phy_access.c).  Internal to the phylib core and
 * the vendor PHY drivers under drivers/net/phy; not part of the public API.
 */

int phy_read(FAR struct phy_device_s *phydev, uint8_t regaddr);
int phy_write(FAR struct phy_device_s *phydev,
              uint8_t regaddr, uint16_t val);
int phy_modify(FAR struct phy_device_s *phydev,
               uint8_t regaddr, uint16_t mask, uint16_t set);
int phy_set_bits(FAR struct phy_device_s *phydev,
                 uint8_t regaddr, uint16_t bits);
int phy_clear_bits(FAR struct phy_device_s *phydev,
                   uint8_t regaddr, uint16_t bits);
#ifdef CONFIG_MDIO_CLAUSE_45
int phy_read_mmd(FAR struct phy_device_s *phydev,
                 uint8_t devad, uint16_t regaddr);
int phy_write_mmd(FAR struct phy_device_s *phydev,
                  uint8_t devad, uint16_t regaddr, uint16_t val);
int phy_modify_mmd(FAR struct phy_device_s *phydev,
                   uint8_t devad, uint16_t regaddr,
                   uint16_t mask, uint16_t set);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_PHYLIB */
#endif /* __DRIVERS_NET_PHY_PHY_H */
