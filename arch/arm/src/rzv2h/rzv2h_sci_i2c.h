/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_sci_i2c.h
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

#ifndef __ARCH_ARM_SRC_RZV2H_RZV2H_SCI_I2C_H
#define __ARCH_ARM_SRC_RZV2H_RZV2H_SCI_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of SCI channels that can operate in simple I2C mode */

#define RZV2H_SCI_I2C_NCHANNELS     10

/* Maximum bit rate supported by SCI simple I2C mode (fast mode) */

#define RZV2H_SCI_I2C_MAX_FREQUENCY I2C_SPEED_FAST

/* NuttX bus number used by the board logic when registering SCI
 * channel 'ch' as an I2C character driver (/dev/i2cN).
 */

#define RZV2H_SCI_I2C_BUS(ch) (CONFIG_RZV2H_SCI_I2C_BUS_OFFSET + (ch))

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_sci_i2cbus_initialize
 *
 * Description:
 *   Initialize one SCI channel in simple I2C master mode.  On the first
 *   call for a given channel, attaches the interrupts, opens the FSP
 *   r_sci_b_i2c module and then configures the pins.
 *
 * Input Parameters:
 *   channel - SCI channel number (0-9)
 *
 * Returned Value:
 *   I2C master device instance on success, NULL on failure
 *
 ****************************************************************************/

struct i2c_master_s *rzv2h_sci_i2cbus_initialize(int channel);

/****************************************************************************
 * Name: rzv2h_sci_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an SCI simple I2C bus.  Decrements the reference count
 *   and shuts down the hardware when the last reference is released.
 *
 * Input Parameters:
 *   dev - I2C master device instance
 *
 * Returned Value:
 *   OK on success, ERROR if reference count underflow
 *
 ****************************************************************************/

int rzv2h_sci_i2cbus_uninitialize(struct i2c_master_s *dev);

#endif /* __ARCH_ARM_SRC_RZV2H_RZV2H_SCI_I2C_H */
