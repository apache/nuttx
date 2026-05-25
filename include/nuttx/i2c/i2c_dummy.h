/****************************************************************************
 * include/nuttx/i2c/i2c_dummy.h
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

#ifndef __INCLUDE_NUTTX_I2C_I2C_DUMMY_H
#define __INCLUDE_NUTTX_I2C_I2C_DUMMY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_dummy_device_s
{
  CODE int (*transfer)(FAR struct i2c_dummy_device_s *dev,
                       FAR struct i2c_msg_s *msg);
#ifdef CONFIG_I2C_RESET
  CODE void (*reset)(FAR struct i2c_dummy_device_s *dev);
#endif
  uint16_t addr;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: i2c_dummy_initialize
 *
 * Description:
 *   Initialize a dummy I2C bus for transport and command-line validation.
 *   The device pointer table must remain valid for the lifetime of the bus.
 *
 * Returned Value:
 *   A valid I2C lower-half instance on success; NULL on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_DUMMY
FAR struct i2c_master_s *
i2c_dummy_initialize(FAR struct i2c_dummy_device_s * const *devices,
                     int ndevices);

#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_I2C_I2C_DUMMY_H */
