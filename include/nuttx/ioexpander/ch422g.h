/****************************************************************************
 * include/nuttx/ioexpander/ch422g.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_CH422G_H
#define __INCLUDE_NUTTX_IOEXPANDER_CH422G_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pin numbering used by the ioexpander interface.
 *
 * The CH422G offers eight bi-directional pins, IO0-IO7, and four
 * open-drain outputs, OC0-OC3.  They are presented as one contiguous pin
 * space so that a single ioexpander_dev_s covers the whole chip.
 */

#define CH422G_IO0              0
#define CH422G_IO1              1
#define CH422G_IO2              2
#define CH422G_IO3              3
#define CH422G_IO4              4
#define CH422G_IO5              5
#define CH422G_IO6              6
#define CH422G_IO7              7
#define CH422G_OC0              8
#define CH422G_OC1              9
#define CH422G_OC2              10
#define CH422G_OC3              11

#define CH422G_NPINS            12
#define CH422G_NIO              8    /* IO0-IO7 */
#define CH422G_NOC              4    /* OC0-OC3 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the CH422G
 * driver when the driver is instantiated.  Memory for this structure is
 * provided by the caller.  It is not copied by the driver and is presumed
 * to persist while the driver is active.
 */

struct ch422g_config_s
{
  uint32_t frequency;  /* I2C frequency */
};

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
 * Name: ch422g_initialize
 *
 * Description:
 *   Instantiate and configure the CH422G device driver to use the provided
 *   I2C device instance.
 *
 * Input Parameters:
 *   i2c    - An I2C driver instance
 *   config - Persistent board configuration data
 *
 * Returned Value:
 *   An ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *
ch422g_initialize(FAR struct i2c_master_s *i2c,
                  FAR struct ch422g_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_IOEXPANDER_CH422G_H */
