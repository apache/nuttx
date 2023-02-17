/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_lpi2c_slave.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_LPI2C_SLAVE_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_LPI2C_SLAVE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/i2c/i2c_slave.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
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
 * Name: s32k1xx_i2cbus_slave_initialize
 *
 * Description:
 *   Initialize the I2C slave device and increase the reference counter.
 *   If the device has already been initialized only the reference counter
 *   will be increased.
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple I2C interfaces).
 *
 * Returned Value:
 *   A valid I2C device structure reference on success; a NULL on failure.
 *
 ****************************************************************************/

struct i2c_slave_s *s32k1xx_i2cbus_slave_initialize(int port);

/****************************************************************************
 * Name: s32k1xx_i2cbus_slave_uninitialize
 *
 * Description:
 *   Decrease the reference counter of the I2C slave device.  When there are
 *   no more references left the I2C slave device is uninitialized.
 *
 * Input Parameters:
 *   dev - Device structure as returned by s32k1xx_i2cbus_slave_initialize().
 *
 * Returned Value:
 *   OK on success, ERROR when there is an internal reference count mismatch
 *   or dev points to an invalid hardware device.
 *
 ****************************************************************************/

int s32k1xx_i2cbus_slave_uninitialize(struct i2c_slave_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_LPI2C_SLAVE_H */
