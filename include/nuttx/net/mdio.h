/****************************************************************************
 * include/nuttx/net/mdio.h
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

#ifndef __INCLUDE_NUTTX_NET_MDIO_H
#define __INCLUDE_NUTTX_NET_MDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <stdint.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Forward references */

struct mdio_bus_s;
struct mdio_lowerhalf_s;

/* This structure defines the interface for the MDIO lower-half driver.
 * These methods are called by the upper-half MDIO driver.
 */

struct mdio_ops_s
{
  /* Clause 22 MDIO Read. The first argument is a reference to the
   * lower-half driver's private state.
   */

  int (*read)(FAR struct mdio_lowerhalf_s *lower, uint8_t phyaddr,
              uint8_t regaddr, FAR uint16_t *value);

  /* Clause 22 MDIO Write */

  int (*write)(FAR struct mdio_lowerhalf_s *lower, uint8_t phyaddr,
               uint8_t regaddr, uint16_t value);

#ifdef CONFIG_MDIO_CLAUSE_45
  /* Native Clause 45 MDIO read.  Optional: when NULL, mdio_c45_read()
   * falls back to indirect access via Clause 22 registers 13/14.
   */

  int (*read_c45)(FAR struct mdio_lowerhalf_s *lower, uint8_t phyaddr,
                  uint8_t devad, uint16_t regaddr, FAR uint16_t *value);

  /* Native Clause 45 MDIO write.  Optional: when NULL, mdio_c45_write()
   * falls back to indirect access via Clause 22 registers 13/14.
   */

  int (*write_c45)(FAR struct mdio_lowerhalf_s *lower, uint8_t phyaddr,
                   uint8_t devad, uint16_t regaddr, uint16_t value);
#endif

  /* PHY hardware reset line control. Optional.
   *
   * controls the PHY's RESET_N line by
   * logical value rather than performing a one-shot pulse:
   *   value != 0 -> assert reset  (drive the PHY into reset)
   *   value == 0 -> deassert reset (release the PHY)
   * The lower half owns pin polarity (active-low RESET_N is handled inside),
   * and may sleep for the datasheet-mandated assert/deassert settle time
   * before returning.  The upper half (mdio_reset) takes care of idempotency
   * (skips the call when the requested state is already in effect).
   */

  int (*reset)(FAR struct mdio_lowerhalf_s *lower, uint8_t phyaddr,
               int value);
};

/* This structure defines the state of the MDIO lower-half driver.
 * The chip-specific MDIO driver must allocate and initialize one instance
 * of this structure.
 */

struct mdio_lowerhalf_s
{
  /* The vtable of MDIO lower-half operations.
   * This must be the first field.
   */

  FAR const struct mdio_ops_s *ops;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: mdio_register
 *
 * Description:
 *   Register a new MDIO bus instance.
 *
 * Input Parameters:
 *   lower - An instance of the lower-half MDIO driver, with the ops vtable
 *           as the first member.
 *
 * Returned Value:
 *   A non-NULL handle on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct mdio_bus_s *mdio_register(FAR struct mdio_lowerhalf_s *lower);

/****************************************************************************
 * Name: mdio_unregister
 *
 * Description:
 *   Unregister an MDIO bus instance.
 *
 * Input Parameters:
 *   dev - The MDIO bus handle returned by mdio_register.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_unregister(FAR struct mdio_bus_s *dev);

/****************************************************************************
 * Name: mdio_read
 *
 * Description:
 *   Read a 16-bit value from a PHY register on the MDIO bus.
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31).
 *   regaddr - The PHY register address (0-31).
 *   value   - A pointer to the location to store the read value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_read(FAR struct mdio_bus_s *dev, uint8_t phyaddr, uint8_t regaddr,
              FAR uint16_t *value);

/****************************************************************************
 * Name: mdio_write
 *
 * Description:
 *   Write a 16-bit value to a PHY register on the MDIO bus.
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31).
 *   regaddr - The PHY register address (0-31).
 *   value   - The value to write.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_write(FAR struct mdio_bus_s *dev, uint8_t phyaddr, uint8_t regaddr,
               uint16_t value);

/****************************************************************************
 * Name: mdio_c45_read / mdio_c45_write
 *
 * Description:
 *   Clause 45 MMD register access.  Uses the lower-half native C45 ops if
 *   provided; otherwise falls back to indirect access over Clause 22
 *   registers 13/14.  Only available when CONFIG_MDIO_CLAUSE_45 is set.
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31).
 *   devad   - The MMD device address.
 *   regaddr - The 16-bit MMD register address.
 *   value   - Read: pointer to store the value.  Write: value to write.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MDIO_CLAUSE_45
int mdio_c45_read(FAR struct mdio_bus_s *dev, uint8_t phyaddr,
                  uint8_t devad, uint16_t regaddr, FAR uint16_t *value);

int mdio_c45_write(FAR struct mdio_bus_s *dev, uint8_t phyaddr,
                   uint8_t devad, uint16_t regaddr, uint16_t value);
#endif

/****************************************************************************
 * Name: mdio_reset
 *
 * Description:
 *   Control the hardware reset line of a PHY on the MDIO bus. This is a
 *   level control, not a one-shot pulse.  Callers assert then deassert
 *   to perform a full reset:
 *
 *     mdio_reset(dev, phyaddr, 1);   // assert (into reset)
 *     mdio_reset(dev, phyaddr, 0);   // deassert (release)
 *
 *   The call is idempotent: requesting a state that is already in effect is
 *   a no-op.  The per-PHY assert/deassert settle delay is handled by the
 *   lower-half driver.
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31) to reset.
 *   value   - Reset line state: 1 to assert reset, 0 to deassert.
 *
 * Returned Value:
 *   Zero (OK) on success; -ENOSYS if no reset op is provided; a negated
 *   errno value on failure.
 *
 ****************************************************************************/

int mdio_reset(FAR struct mdio_bus_s *dev, uint8_t phyaddr, int value);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_MDIO_H */
