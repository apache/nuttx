/****************************************************************************
 * include/nuttx/video/imx335.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_IMX335_H
#define __INCLUDE_NUTTX_VIDEO_IMX335_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgsensor.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx335_initialize
 *
 * Description:
 *   Create an IMX335 image sensor instance.  The sensor must be powered
 *   and out of reset; it is probed when the capture driver is registered.
 *
 * Input Parameters:
 *   i2c  - I2C bus the sensor is attached to (address 0x1a)
 *   inck - Sensor input clock in Hz: 6, 18, 24, 27 or 74.25 MHz
 *
 * Returned Value:
 *   Image sensor instance on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct imgsensor_s *imx335_initialize(FAR struct i2c_master_s *i2c,
                                          uint32_t inck);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_IMX335_H */
