/****************************************************************************
 * include/nuttx/sensors/tc74ax.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_TC74AX_H
#define __INCLUDE_NUTTX_SENSORS_TC74AX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_TC74AX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TC74AX_CMD_READ_TEMP 0
#define TC74AX_CMD_READ_CONFIG 1

#define TC74AX_CONFIG_DATA_READY

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

typedef enum
{
  TC74AX_OPERATION_MODE_OPERATING,
  TC74AX_OPERATION_MODE_STANDBY
} tc74ax_operation_mode_e;

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
 * Name: tc74ax_register
 *
 * Description:
 *   Registers TC74Ax temperature sensor at specified pathname.
 *
 * Input Parameters:
 *   devpath - full pathname where the driver will be registered
 *             (example: /dev/therm0)
 *   i2c     - instance of I2C driver controlling the bus this sensor
 *             is attached to.
 *   addr    - I2C address of the sensor. Corresponds to the chip name
 *             with x replaced by a digit in range of 0-7. Minimum
 *             is 72 for TC74A0, maximum is 79 for TC74A7.
 *
 * Returned Value:
 *   OK on success or negated errno on failure.
 *
 ****************************************************************************/

int tc74ax_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* CONFIG_I2C && CONFIG_SENSORS_TC74AX */

#endif /* __INCLUDE_NUTTX_SENSORS_TC74AX_H */
