/****************************************************************************
 * include/nuttx/1wire/ds28e17.h
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

#ifndef __INCLUDE_NUTTX_1WIRE_DS28E17_H
#define __INCLUDE_NUTTX_1WIRE_DS28E17_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/1wire/1wire.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default how many converters in a bus. */

#define DS_DEFAULT_MAXSLAVES         10

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ds28e17_dev_s;
struct i2c_master_s;

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: ds28e17_search
 *
 * Description:
 *   Search all DS28E17 devices from a 1-wire network.
 *
 * Input Parameters:
 *   priv      - Pointer to the associated DS28E17
 *   cb_search - Callback to call on each device found
 *   arg       - Argument passed to cb_search
 *
 * Return Value:
 *   Number of DS28E17 devices present.
 *
 ****************************************************************************/

int ds28e17_search(FAR struct ds28e17_dev_s *priv,
                   CODE void (*cb_search)(int family,
                                          uint64_t romcode,
                                          FAR void *arg),
                   FAR void *arg);

/****************************************************************************
 * Name: ds28e17_lower_half
 *
 * Description:
 *   Initialize the lower half of the DS28E17 by creating a i2c_master_s
 *   for the virtual i2c master and link it to the associated DS28E17 and
 *   its port.
 *
 * Input Parameters:
 *   priv    - Pointer to the associated DS28E17
 *   romcode - The unique 64-bit address in 1-wire network.
 *             Use zero for skip-ROM mode.
 *
 * Returned Value:
 *   i2c device instance; NULL on failure.
 *
 ****************************************************************************/

FAR struct i2c_master_s *ds28e17_lower_half(FAR struct ds28e17_dev_s *priv,
                                            uint64_t romcode);

/****************************************************************************
 * Name: ds28e17_lower_half_unregister
 *
 * Description:
 *   Put back the lower half of the DS28E17.
 *
 * Input Parameters:
 *   priv    - Pointer to the associated DS28E17
 *   i2cdev  - i2c device instance from ds28e17_lower_half()
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ds28e17_lower_half_unregister(FAR struct ds28e17_dev_s *priv,
                                  FAR struct i2c_master_s *i2cdev);

/****************************************************************************
 * Name: ds28e17_initialize
 *
 * Description:
 *   Returns a common DS28E17 device from 1-wire lower half device
 *
 * Input Parameters:
 *   dev - The allocated 1-wire lower half
 *
 ****************************************************************************/

FAR struct ds28e17_dev_s *ds28e17_initialize(FAR struct onewire_dev_s *dev);

#endif /* __INCLUDE_NUTTX_1WIRE_DS28E17_H */
