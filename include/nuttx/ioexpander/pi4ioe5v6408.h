/****************************************************************************
 * include/nuttx/ioexpander/pi4ioe5v6408.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_PI4IOE5V6408_H
#define __INCLUDE_NUTTX_IOEXPANDER_PI4IOE5V6408_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_PI4IOE5V6408_INT_ENABLE
#  include <nuttx/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PI4IOE5V6408_I2C_ADDRESS_LOW   0x43
#define PI4IOE5V6408_I2C_ADDRESS_HIGH  0x44
#define PI4IOE5V6408_NPINS             8

/* Power-up / soft-reset defaults.
 * Direction: 1 = output, 0 = input.
 * Pull select: 1 = pull-up, 0 = pull-down.
 */

#define PI4IOE5V6408_DIR_DEFAULT       0xff
#define PI4IOE5V6408_OUT_DEFAULT       0x00
#define PI4IOE5V6408_HIGHZ_DEFAULT     0xff
#define PI4IOE5V6408_PULLEN_DEFAULT    0xff
#define PI4IOE5V6408_PULLSEL_DEFAULT   0x00

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;
struct ioexpander_dev_s;

struct pi4ioe5v6408_config_s
{
  uint8_t address;
  uint32_t frequency;

#ifdef CONFIG_PI4IOE5V6408_INT_ENABLE
  CODE int (*attach)(FAR struct pi4ioe5v6408_config_s *config,
                     xcpt_t isr, FAR void *arg);
  CODE void (*enable)(FAR struct pi4ioe5v6408_config_s *config, bool enable);
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: pi4ioe5v6408_initialize
 *
 * Description:
 *   Instantiate and initialize a PI4IOE5V6408 I2C I/O expander.
 *
 * Input Parameters:
 *   i2c    - I2C controller instance
 *   config - Persistent board configuration
 *
 * Returned Value:
 *   An ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *
pi4ioe5v6408_initialize(FAR struct i2c_master_s *i2c,
                        FAR struct pi4ioe5v6408_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_IOEXPANDER_PI4IOE5V6408_H */
