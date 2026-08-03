/****************************************************************************
 * include/nuttx/input/st7123.h
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

/* The ST7123 is an I2C capacitive touchscreen controller.  This header
 * exposes the board-facing registration API used to bind the driver to an
 * I2C bus and GPIO interrupt.  Once registered, the device appears as
 * /dev/inputN and reports multi-touch samples through the standard
 * touchscreen upper half.
 */

#ifndef __INCLUDE_NUTTX_INPUT_ST7123_H
#define __INCLUDE_NUTTX_INPUT_ST7123_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>

#ifdef CONFIG_INPUT_ST7123

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Board-specific configuration.  The board must supply attach(), which
 * wires the ST7123 INT pin to the given handler.  Memory for this structure
 * is provided by the caller, is not copied by the driver, and must persist
 * while the driver is active.
 */

struct st7123_config_s
{
  CODE int (*attach)(FAR const struct st7123_config_s *config,
                     xcpt_t isr, FAR void *arg);
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
 * Name: st7123_register
 *
 * Description:
 *   Probe the ST7123 over I2C, configure the touchscreen lower half from
 *   the reported resolution and touch-area count, register it as
 *   /dev/inputN where N is the given minor number, and attach the board
 *   interrupt via config->attach().
 *
 *   I2C frequency and slave address are taken from
 *   CONFIG_INPUT_ST7123_I2C_FREQUENCY and CONFIG_INPUT_ST7123_I2C_ADDRESS.
 *
 * Input Parameters:
 *   i2c    - I2C master used to talk to the ST7123
 *   minor  - Device minor number used to form /dev/inputN
 *   config - Persistent board configuration; config->attach must be valid
 *
 * Returned Value:
 *   Zero (OK) on success.  Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

int st7123_register(FAR struct i2c_master_s *i2c, uint8_t minor,
                    FAR const struct st7123_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT_ST7123 */
#endif /* __INCLUDE_NUTTX_INPUT_ST7123_H */
