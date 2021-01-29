/****************************************************************************
 * include/nuttx/ioexpander/pcf8574.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_PCF8574_H
#define __INCLUDE_NUTTX_IOEXPANDER_PCF8574_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_PCF8574_INT_ENABLE
/* This is the type of the PCF8574xx interrupt handler */

typedef CODE void (*pcf8574_handler_t)(FAR void *arg);
#endif

/* A reference to a structure of this type must be passed to the PCF8574xx
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the PCF8574xx and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by
 * the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct pcf8574_config_s
{
  /* Device characterization */

  uint8_t address;     /* 7-bit I2C address (only bits 0-6 used) */
  uint32_t frequency;  /* I2C frequency */

#ifdef CONFIG_PCF8574_INT_ENABLE
  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the PCF8574xx driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the PCF8574xx interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   */

  CODE int  (*attach)(FAR struct pcf8574_config_s *state,
                      pcf8574_handler_t handler, FAR void *arg);
  CODE void (*enable)(FAR struct pcf8574_config_s *state, bool enable);
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pcf8574_initialize
 *
 * Description:
 *   Instantiate and configure the PCF8574xx device driver to use the
 *   provided I2C device instance.
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
FAR struct ioexpander_dev_s *pcf8574_initialize(FAR struct i2c_master_s *i2c,
                                        FAR struct pcf8574_config_s *config);

#endif /* __INCLUDE_NUTTX_IOEXPANDER_PCF8574_H */
