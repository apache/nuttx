/****************************************************************************
 * arch/arm/src/stm32/stm32_i2c.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_I2C_H
#define __ARCH_ARM_SRC_STM32_STM32_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#include "chip.h"
#include "hardware/stm32_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If a dynamic timeout is selected, then a non-negative, non-zero micro-
 * seconds per byte value must be provided as well.
 */

#ifdef CONFIG_STM32_I2C_DYNTIMEO
#  if CONFIG_STM32_I2C_DYNTIMEO_USECPERBYTE < 1
#    warning "Ignoring CONFIG_STM32_I2C_DYNTIMEO because of CONFIG_STM32_I2C_DYNTIMEO_USECPERBYTE"
#    undef CONFIG_STM32_I2C_DYNTIMEO
#  endif
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a unique instance of struct
 *   struct i2c_master_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2c_master_s *stm32_i2cbus_initialize(int port);

/****************************************************************************
 * Name: stm32_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port, and power down the device.
 *
 * Input Parameters:
 *   Device structure as returned by the stm32_i2cbus_initialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int stm32_i2cbus_uninitialize(struct i2c_master_s *dev);

#endif /* __ARCH_ARM_SRC_STM32_STM32_I2C_H */
