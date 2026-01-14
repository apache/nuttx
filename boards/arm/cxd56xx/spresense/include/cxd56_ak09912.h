/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_ak09912.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_AK09912_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_AK09912_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_ak09912_initialize
 *
 * Description:
 *   Initialize AK09912 i2c driver and register the AK09912 device.
 *
 ****************************************************************************/

#if defined (CONFIG_SENSORS_AK09912) || defined (CONFIG_SENSORS_AK09912_SCU)
int board_ak09912_initialize(const char *devpath, int bus);
#endif

#ifdef CONFIG_SENSORS_AK09912_SCU
/****************************************************************************
 * Name: ak09912_init
 *
 * Description:
 *   Initialize AK09912 magnetometer device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             AK09912
 *   port    - I2C port number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ak09912_init(struct i2c_master_s *i2c, int port);

/****************************************************************************
 * Name: ak09912_scu_register
 *
 * Description:
 *   Register the AK09912 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mag0"
 *   minor   - The number of sequencer
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             AK09912
 *   port    - I2C port number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ak09912_scu_register(const char *devpath, int minor,
                         struct i2c_master_s *i2c, int port);

#endif /* CONFIG_SENSORS_AK09912_SCU */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_AK09912_H */
