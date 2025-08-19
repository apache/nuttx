/****************************************************************************
 * boards/arm/rp2040/common/include/rp2040_tmp112.h
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

#ifndef __BOARDS_ARM_RP2040_COMMON_INCLUDE_RP2040_TMP112_H
#define __BOARDS_ARM_RP2040_COMMON_INCLUDE_RP2040_TMP112_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_tmp112_initialize
 *
 * Description:
 *   Initialize and register the TMP112 temperature sensor driver.
 *
 * Input Parameters:
 *   i2c   - An instance of the I2C interface to use.
 *   devno - The device number, used to build the device path as /dev/tempN.
 *   addr  - The I2C address to use.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_tmp112_initialize(FAR struct i2c_master_s *i2c, int devno,
                            uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_RP2040_COMMON_INCLUDE_RP2040_TMP112_H */
