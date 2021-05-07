/****************************************************************************
 * include/nuttx/sensors/bh1750fvi.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BH1750FVI_H
#define __INCLUDE_NUTTX_SENSORS_BH1750FVI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_SENSORS_BH1750FVI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device I2C Address */

#define BH1750FVI_I2C_ADDR        0x23

/* Instruction Set Architecture */

#define BH1750FVI_POWERDOWN       0x00
#define BH1750FVI_POWERON         0x01
#define BH1750FVI_RESET           0x07
#define BH1750FVI_CONTINUOUS_HRM  0x10 /* Continuously H-Resolution Mode */
#define BH1750FVI_CONTINUOUS_HRM2 0x11 /* Continuously H-Resolution Mode 2 */
#define BH1750FVI_CONTINUOUS_LRM  0x12 /* Continuously L-Resolution Mode */
#define BH1750FVI_ONETIME_HRM     0x20 /* One Time H-Resolution Mode */
#define BH1750FVI_ONETIME_HRM2    0x21 /* One Time H-Resolution Mode 2 */
#define BH1750FVI_ONETIME_LRM     0x23 /* One Time L-Resolution Mode */
#define BH1750FVI_MEASURE_TIMEH   0x40 /* Change Measure Time 01000_MT[7,6,5] */
#define BH1750FVI_MEASURE_TIMEL   0x60 /* Change Measute Time 011_MT[4,3,2,1,0] */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: bh1750fvi_register
 *
 * Description:
 *   Register the BH1750FVI character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/light0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *              BH1750FVI
 *   addr    - The I2C address of the BH1750FVI.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct i2c_master_s; /* Forward reference */
int bh1750fvi_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_BH1750FVI */
#endif /* __INCLUDE_NUTTX_SENSORS_BH1750FVI_H */
