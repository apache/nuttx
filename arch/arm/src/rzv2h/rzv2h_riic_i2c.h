/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_riic_i2c.h
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

#ifndef __ARCH_ARM_SRC_RZV2H_RZV2H_RIIC_I2C_H
#define __ARCH_ARM_SRC_RZV2H_RZV2H_RIIC_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RZ_RIIC_MASTER_DIV_TIME_NS (1000000000.0)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_riic_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus.  On first call for a given port, initializes
 *   the hardware via rzv2h_riic_i2c_init().  Increments the reference count
 *   on subsequent calls.
 *
 * Input Parameters:
 *   port - I2C port number (0-8)
 *
 * Returned Value:
 *   I2C master device instance on success, NULL on failure
 *
 ****************************************************************************/

struct i2c_master_s *rzv2h_riic_i2cbus_initialize(int port);

/****************************************************************************
 * Name: rzv2h_riic_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus.  Decrements the reference count and shuts
 *   down the hardware when the last reference is released.
 *
 * Input Parameters:
 *   dev - I2C master device instance
 *
 * Returned Value:
 *   OK on success, ERROR if reference count underflow
 *
 ****************************************************************************/

int rzv2h_riic_i2cbus_uninitialize(struct i2c_master_s *dev);

#endif /* __ARCH_ARM_SRC_RZV2H_RZV2H_RIIC_I2C_H */
