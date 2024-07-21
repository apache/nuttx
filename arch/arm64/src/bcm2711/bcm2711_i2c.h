/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_i2c.h
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#ifndef __ARCH_ARM64_SRC_BCM2711_BCM2711_I2C_H
#define __ARCH_ARM64_SRC_BCM2711_BCM2711_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/bcm2711_bsc.h"
#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device for the BCM2711.
 *
 * Input parameters:
 *     port - The bus number for the I2C interface.
 *
 ****************************************************************************/

struct i2c_master_s *bcm2711_i2cbus_initialize(int port);

/****************************************************************************
 * Name: bcm2711_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C device on the BCM2711.
 *
 * Input parameters;
 *     dev - The device to uninitialize.
 *
 ****************************************************************************/

int bcm2711_i2cbus_uninitialize(struct i2c_master_s *dev);

#endif // __ARCH_ARM64_SRC_BCM2711_BCM2711_I2C_H
