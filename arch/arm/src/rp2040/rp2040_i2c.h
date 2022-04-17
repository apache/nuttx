/****************************************************************************
 * arch/arm/src/rp2040/rp2040_i2c.h
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

#ifndef __ARCH_ARM_SRC_RP2040_RP2040_I2C_H
#define __ARCH_ARM_SRC_RP2040_RP2040_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include "hardware/rp2040_i2c.h"

#ifndef __ASSEMBLY__
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
 * Name: rp2040_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a unique instance of struct
 *   struct i2c_master_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameter:
 *   Port number (for hardware that has multiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2c_master_s *rp2040_i2cbus_initialize(int port);

/****************************************************************************
 * Name: rp2040_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port, and power down the device.
 *
 * Input Parameter:
 *   Device structure as returned by the rp2040_i2cbus_initialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int rp2040_i2cbus_uninitialize(struct i2c_master_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP2040_RP2040_I2C_H */
