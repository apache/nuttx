/****************************************************************************
 * include/nuttx/power/act8945a.h
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

#ifndef __INCLUDE_NUTTX_POWER_BATTERY_ACT8945A_H
#define __INCLUDE_NUTTX_POWER_BATTERY_ACT8945A_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACT8945A_NUM_REGS                         7

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_REGULATOR_ACT8945A)

#ifdef __cplusplus
extern "C"
{
#endif

struct i2c_master_s;
int act8945a_initialize(FAR struct i2c_master_s *i2c, unsigned int vsel);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_I2C_ACT8945A */
#endif

