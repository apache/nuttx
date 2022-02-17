/****************************************************************************
 * include/nuttx/sensors/hall3ph.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_HALL3PH_H
#define __INCLUDE_NUTTX_SENSORS_HALL3PH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* 120-degree Hall effect sensors positions */

enum hall3_120deg_position_e
{
  HALL3_120DEG_POS_1 = 0b001,
  HALL3_120DEG_POS_2 = 0b011,
  HALL3_120DEG_POS_3 = 0b010,
  HALL3_120DEG_POS_4 = 0b110,
  HALL3_120DEG_POS_5 = 0b100,
  HALL3_120DEG_POS_6 = 0b101
};

/* 60-degree Hall effect sensors positions */

enum hall3_60deg_position_e
{
  HALL3_60DEG_POS_1 = 0b000,
  HALL3_60DEG_POS_2 = 0b001,
  HALL3_60DEG_POS_3 = 0b101,
  HALL3_60DEG_POS_4 = 0b111,
  HALL3_60DEG_POS_5 = 0b110,
  HALL3_60DEG_POS_6 = 0b010
};

/* This structure provides the "lower-half" driver operations available to
 * the "upper-half" driver.
 */

struct hall3_lowerhalf_s;
struct hall3_ops_s
{
  /* Configure the 3-phase Hall effect sensor device */

  CODE int (*setup)(FAR struct hall3_lowerhalf_s *lower);

  /* Disable the 3-phase Hall effect sensor device */

  CODE int (*shutdown)(FAR struct hall3_lowerhalf_s *lower);

  /* Return the current 3-phase Hall effect sensor position */

  CODE int (*position)(FAR struct hall3_lowerhalf_s *lower,
                       FAR uint8_t *pos);
};

/* This structure provides the publicly visible representation of the
 * "lower-half" driver state structure.  "lower half" drivers will have an
 * internal structure definition that will be cast-compatible with this
 * structure definitions.
 */

struct hall3_lowerhalf_s
{
  FAR const struct hall3_ops_s *ops;
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hall3_register
 *
 * Description:
 *   Register the 3-phase Hall effect sensor lower half device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/hall0"
 *   lower - An instance of the lower half interface
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.  The following
 *   possible error values may be returned (most are returned by
 *   register_driver()):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int hall3_register(FAR const char *devpath,
                   FAR struct hall3_lowerhalf_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_HALL3PH_H */
