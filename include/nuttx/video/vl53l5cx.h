/****************************************************************************
 * include/nuttx/video/vl53l5cx.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_VL53L5CX_H
#define __INCLUDE_NUTTX_VIDEO_VL53L5CX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

#ifdef CONFIG_VIDEO_VL53L5CX

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/* Board configuration.  INT is open drain and falls when a frame is
 * ready.
 */

struct vl53l5cx_config_s
{
  uint8_t addr;                       /* 7-bit I2C address, 0x29 */

  /* Attach and enable the INT handler, or disable it with isr NULL */

  CODE int (*attach)(FAR const struct vl53l5cx_config_s *config,
                     xcpt_t isr, FAR void *arg);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: vl53l5cx_register
 *
 * Description:
 *   Check that a VL53L5CX answers on the bus and register it as a V4L2
 *   depth capture device (V4L2_PIX_FMT_Z16, 8x8 or 4x4).  The firmware
 *   and the default configuration are written at the first stream start.
 *
 * Input Parameters:
 *   devpath - The capture device path, e.g. "/dev/video1"
 *   i2c     - The I2C bus the sensor is on
 *   config  - The board configuration, kept by the driver
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int vl53l5cx_register(FAR const char *devpath,
                      FAR struct i2c_master_s *i2c,
                      FAR const struct vl53l5cx_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_VIDEO_VL53L5CX */
#endif /* __INCLUDE_NUTTX_VIDEO_VL53L5CX_H */
