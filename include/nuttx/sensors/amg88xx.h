/****************************************************************************
 * include/nuttx/sensors/amg88xx.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_AMG88XX_H
#define __INCLUDE_NUTTX_SENSORS_AMG88XX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AMG88XX_PIXELS                           (64)      /* hex 0x40     */
#define AMG88XX_PIXELS_ARRAY_LENGTH              (127)

#define AMG88XX_THERMISTOR_RESOLUTION            (0.0625)
#define AMG88XX_THERMISTOR_MAX_VALUE             (0x7FF)   /* +127.9375 *c */
#define AMG88XX_THERMISTOR_MIN_VALUE             (0xBBB)   /*  -59.6875 *c */

#define AMG88XX_PIXEL_RESOLUTION                 (0.25)
#define AMG88XX_PIXEL_MAX_VALUE                  (0x1F4)   /*  +125     *c */
#define AMG88XX_PIXEL_MIN_VALUE                  (0xF24)   /*  -55      *c */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Data returned by sensor read typedef  */

typedef uint8_t amg88xx_pixels_temp_t[AMG88XX_PIXELS_ARRAY_LENGTH];

/* Typedef used to interact with ioctl syscall to set sensor operation mode */

typedef enum
{
  op_mode_normal = 0,
  op_mode_sleep = 1
} amg88xx_operation_mode_e;

/* Typedef used to interact with ioctl syscall to set sensor fps */

typedef enum
{
  fps_one = 0,
  fps_ten = 1
} amg88xx_fps_e;

/* Runtime configuration struct sent to the sensor driver */

struct amg88xx_config_s
{
  uint8_t addr;
  uint32_t speed;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: amg88xx_register
 *
 * Description:
 *   Register the amg88xx character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/irm0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             AMG88xx
 *   config  - Initial configuration of the sensor
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int amg88xx_register(FAR const char *devpath,
                     FAR struct i2c_master_s *i2c,
                     FAR struct amg88xx_config_s *config);

#endif /* __INCLUDE_NUTTX_SENSORS_AMG88XX_H */
