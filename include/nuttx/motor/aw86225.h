/****************************************************************************
 * include/nuttx/motor/aw86225.h
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

#ifndef __INCLUDE_NUTTX_MOTOR_AW86225_H
#define __INCLUDE_NUTTX_MOTOR_AW86225_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_MOTOR_UPPER_HAVE_PATTERN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aw86225_pattern_s
{
  uint8_t patternid[8];
  uint8_t waveloop[8];
  uint8_t mainloop;
  float strength;
  uint32_t duration; /* in millisecond */
};

struct aw86225_patterns_s
{
  uint32_t count;
  uint8_t repeatable;
  struct aw86225_pattern_s pattern[1];
};

struct aw86225_config_s
{
  uint8_t addr;                        /* I2C address */
  int freq;                            /* I2C frequency */
  FAR struct i2c_master_s *i2c;        /* I2C interface */
};

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
 * Name: aw86225_register
 *
 * Description:
 *   Register the aw86225 character device as 'devname'
 *
 * Input Parameters:
 *   devname - The user specifies device path.
 *   config  - The board config function for the device.
 *
 * Returned Value:
 *   Return 0 if the driver was successfully initialize; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none
 *
 ****************************************************************************/

int aw86225_register(FAR const char *devname,
                     FAR const struct aw86225_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* CONFIG_MOTOR_UPPER_HAVE_PATTERN */
#endif /* __INCLUDE_NUTTX_MOTOR_AW86225_H */
