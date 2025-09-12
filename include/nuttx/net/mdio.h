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

  /* PHY Reset. Optional. */

  int (*reset)(FAR struct mdio_lowerhalf_s *lower, uint8_t phyaddr);
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

  FAR const struct mdio_ops_s ops;

  /* Opaque pointer to the lower-half driver's private state */

  FAR void *priv;
};

/* This is the opaque handle used by application code to access the
 * MDIO bus.
 */

struct mdio_bus_s
{
  /* Pointer to the lower-half driver's state */

  FAR struct mdio_lowerhalf_s *d_lower;

  /* For exclusive access to the bus */

  mutex_t lock;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
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
 * Name: mdio_reset
 *
 * Description:
 *   Reset a PHY on the MDIO bus.
 *
 * Input Parameters:
 *   dev     - The MDIO bus handle.
 *   phyaddr - The PHY address (0-31) to reset.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mdio_reset(FAR struct mdio_bus_s *dev, uint8_t phyaddr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_MDIO_H */
