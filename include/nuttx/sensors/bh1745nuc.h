/****************************************************************************
 * include/nuttx/sensors/bh1745nuc.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BH1745NUC_H
#define __INCLUDE_NUTTX_SENSORS_BH1745NUC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_I2C) && (defined(CONFIG_SENSORS_BH1745NUC) || \
                            defined(CONFIG_SENSORS_BH1745NUC_SCU))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *
 * CONFIG_SENSORS_BH1745NUC
 *   Enables support for the BH1745NUC uORB driver
 * CONFIG_SENSORS_BH1745NUC_SCU
 *   Enables support for the BH1745NUC SCU character driver (cxd56xx)
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef CONFIG_SENSORS_BH1745NUC_SCU
/****************************************************************************
 * Name: bh1745nuc_init
 *
 * Description:
 *   Initialize BH1745NUC color sensor device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BH1745NUC
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bh1745nuc_init(FAR struct i2c_master_s *i2c, int port);

/****************************************************************************
 * Name: bh1745nuc_register
 *
 * Description:
 *   Register the BH1745NUC character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/color0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BH1745NUC
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bh1745nuc_register(FAR const char *devpath, int minor,
                       FAR struct i2c_master_s *i2c, int port);
#endif /* CONFIG_SENSORS_BH1745NUC_SCU */

#ifdef CONFIG_SENSORS_BH1745NUC
/****************************************************************************
 * Name: bh1745nuc_register_uorb
 *
 * Description:
 *   Register the BH1745NUC color sensor as a uORB device.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path.
 *   i2c   - The I2C bus driver instance.
 *   addr  - The I2C address of the BH1745NUC.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bh1745nuc_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                            uint8_t addr);
#endif /* CONFIG_SENSORS_BH1745NUC */

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && (SENSORS_BH1745NUC || SENSORS_BH1745NUC_SCU) */
#endif /* __INCLUDE_NUTTX_SENSORS_BH1745NUC_H */
