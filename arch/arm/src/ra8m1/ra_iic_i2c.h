/****************************************************************************
 * arch/arm/src/ra8m1/ra_iic_i2c.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_RA_IIC_I2C_H
#define __ARCH_ARM_SRC_RA8M1_RA_IIC_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra_iic_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected IIC channel (0 or 1) as the generic NuttX
 *   I2C master interface.  Returns NULL if that channel is not enabled
 *   (CONFIG_RA_IICn) or bus is out of range.  Only ra_i2c.c (the
 *   bus-number dispatcher) calls this directly.
 *
 ****************************************************************************/

FAR struct i2c_master_s *ra_iic_i2cbus_initialize(int bus);

#endif /* __ARCH_ARM_SRC_RA8M1_RA_IIC_I2C_H */
