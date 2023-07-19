/****************************************************************************
 * drivers/regmap/internal.h
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

#ifndef __DRIVERS_REGMAP_INTERNAL_H
#define __DRIVERS_REGMAP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/regmap/regmap.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef CODE void (*regmap_lock_t)(FAR void *);
typedef CODE void (*regmap_unlock_t)(FAR void *);

/* Configuration for the register map of a device.
 * This structure is only used inside regmap.
 */

struct regmap_s
{
  regmap_lock_t lock;
  regmap_unlock_t unlock;

  bool disable_locking;

  /* Number of bits in a register address, from regmap_config_s. */

  int reg_bytes;

  /* Number of bits in a register value, from regmap_config_s. */

  int val_bytes;

  /* Device bus interface, used internally. */

  FAR struct regmap_bus_s *bus;

  /* Regmap bus control handle.
   * reg_read_t/reg_write_t single-byte register reading.
   * read_t/write_t block data reading.
   * Note:The transmitted data must be in a data format supported by the bus.
   */

  reg_read_t  reg_read;
  reg_write_t reg_write;
  read_t  read;
  write_t write;

  /* The register address stride. Valid register addresses are a multiple
   * of this value. If set to 0, a value of 1 will be used.
   */

  int reg_stride;

  /* Prevent fragmentation */

  mutex_t mutex[0];
};

#endif /* __DRIVERS_REGMAP_INTERNAL_H */
