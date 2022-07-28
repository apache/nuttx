/****************************************************************************
 * include/nuttx/audio/aw88266a.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_FS1818U_H
#define __INCLUDE_NUTTX_AUDIO_FS1818U_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/audio/audio.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct fs1818u_lower_s
{
  uint32_t frequency;                 /* I2C frequency */
  uint8_t address;                    /* I2C device address */
  int bclk_factor;                    /* blck factor */
  CODE void (*reset)(FAR const struct fs1818u_lower_s *lower);
  CODE int (*power_en)(bool flag);
  CODE int (*reset_en)(bool flag);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: fs1818_initialize
 *
 * Description:
 *   Initializa fs1818u smart pa
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *
fs1818_initialize(FAR struct i2c_master_s *i2c,
                  FAR struct fs1818u_lower_s * lower);

#endif /* __INCLUDE_NUTTX_AUDIO_FS1818U_H */
