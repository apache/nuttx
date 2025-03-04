/****************************************************************************
 * include/nuttx/ioexpander/tca64xx.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_TCA64XX_H
#define __INCLUDE_NUTTX_IOEXPANDER_TCA64XX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Identifies supported TCA64xx parts (as well as the number of supported
 * parts).
 */

enum tca64xx_part_e
{
  TCA6408_PART = 0,
  TCA6416_PART,
  TCA6424_PART,
  TCA64_NPARTS
};

#ifdef CONFIG_TCA64XX_INT_ENABLE
/* This is the type of the TCA64xx interrupt handler */

typedef CODE void (*tca64_handler_t)(FAR void *arg);
#endif

/* A reference to a structure of this type must be passed to the TCA64xx
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the TCA64xx and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by
 * the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct tca64_config_s
{
  /* Device characterization */

  uint8_t address;     /* 7-bit I2C address (only bits 0-6 used) */
  uint8_t part;        /* See enum tca64xx_part_e */
  uint32_t frequency;  /* I2C or SPI frequency */

#ifdef CONFIG_TCA64XX_INT_ENABLE
  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the TCA64xx driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the TCA64xx interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   */

  CODE int  (*attach)(FAR struct tca64_config_s *state,
                      tca64_handler_t handler, FAR void *arg);
  CODE void (*enable)(FAR struct tca64_config_s *state, bool enable);
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tca64_initialize
 *
 * Description:
 *   Instantiate and configure the TCA64xx device driver to use the provided
 *   I2C device instance.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   minor   - The device i2c address
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   an ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

struct i2c_master_s;
FAR struct ioexpander_dev_s *tca64_initialize(FAR struct i2c_master_s *i2c,
                                        FAR struct tca64_config_s *config);

#endif /* __INCLUDE_NUTTX_IOEXPANDER_TCA64XX_H */
