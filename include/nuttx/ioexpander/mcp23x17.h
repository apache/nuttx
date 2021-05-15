/****************************************************************************
 * include/nuttx/ioexpander/mcp23x17.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_MCP23X17_H
#define __INCLUDE_NUTTX_IOEXPANDER_MCP23X17_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the mcp23x17
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the mcp23x17 and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by
 * the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct mcp23x17_config_s
{
  /* Device characterization */

  uint8_t address;     /* 7-bit I2C address (only bits 0-6 used) */
  uint32_t frequency;  /* I2C or SPI frequency */

  /* Sets the state of the MCP23X17's nReset pin */

  CODE void (*set_nreset_pin)(bool state);

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  /* If multiple mcp23x17 devices are supported, then an IRQ number must
   * be provided for each so that their interrupts can be distinguished.
   */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the mcp23x17 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the mcp23x17 interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   */

  CODE int  (*attach)(FAR struct mcp23x17_config_s *state, xcpt_t isr,
                      FAR void *arg);
  CODE void (*enable)(FAR struct mcp23x17_config_s *state, bool enable);
#endif
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
 * Name: mcp23x17_initialize
 *
 * Description:
 *   Initialize a MCP23X17 I2C device.
 *
 * TODO: Add support for more than one device.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *mcp23x17_initialize(
                              FAR struct i2c_master_s *i2cdev,
                              FAR struct mcp23x17_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_IOEXPANDER_MCP23X17_H */
