/****************************************************************************
 * include/nuttx/i2c/pca9540bdp.h
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
 *
 * References:
 *   "PCA9540B 2-channel I2C-bus multiplexer product datasheet",
 *   31 October 2016, NXP
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_I2C_PCA9540BDP_H
#define __INCLUDE_NUTTX_I2C_PCA9540BDP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifndef CONFIG_PCA9540BDP_BASEADDR
#  define CONFIG_PCA9540BDP_BASEADDR        0x70
#endif

#define PCA9540BDP_SEL_PORT0                0x0
#define PCA9540BDP_SEL_PORT1                0x1

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct pca9540bdp_dev_s;

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
 * Name: pca9540bdp_lower_half
 *
 * Description:
 *   Initialize the lower half of the PCA9540BDP by creating a i2c_master_s
 *   for the virtual i2c master and link it to the associated PCA9540BDP and
 *   its port.
 *
 * Input Parameters:
 *   dev  - Pointer to the associated PCA9540BDP
 *   port - The port number as defined in pca9540bdp.h
 *
 * Returned Value:
 *   Common i2c multiplexer device instance; NULL on failure.
 *
 ****************************************************************************/

FAR struct i2c_master_s *
  pca9540bdp_lower_half(FAR struct pca9540bdp_dev_s *dev, uint8_t port);

/****************************************************************************
 * Name: pca9540bdp_initialize
 *
 * Description:
 *   Initialize the PCA9540BDP device.
 *
 * Input Parameters:
 *   i2c  - An instance of the I2C interface to use to communicate with
 *          PCA9540BDP
 *   addr - The I2C address of the PCA9540BDP.  The base I2C address of the
 *          PCA9540BDP is 0x70.
 *
 * Returned Value:
 *   Common i2c multiplexer device instance; NULL on failure.
 *
 ****************************************************************************/

FAR struct pca9540bdp_dev_s *
  pca9540bdp_initialize(FAR struct i2c_master_s *i2c, uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_I2C_PCA9540BDP_H */
