/****************************************************************************
 * include/nuttx/ioexpander/pca9538.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_PCA9538_H
#define __INCLUDE_NUTTX_IOEXPANDER_PCA9538_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the pca9538
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the pca9538 and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by
 * the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct pca9538_config_s
{
  /* Device characterization */

  uint8_t address;     /* 7-bit I2C address (only bits 0-6 used) */
  uint32_t frequency;  /* I2C or SPI frequency */

  /* Sets the state of the PCA9538's nReset pin */

  CODE void (*set_nreset_pin)(bool state);

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  /* If multiple pca9538 devices are supported, then an IRQ number must
   * be provided for each so that their interrupts can be distinguished.
   */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the pca9538 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the pca9538 interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   */

  CODE int  (*attach)(FAR struct pca9538_config_s *state, xcpt_t isr,
                      FAR void *arg);
  CODE void (*enable)(FAR struct pca9538_config_s *state, bool enable);
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
 * Name: pca9538_initialize
 *
 * Description:
 *   Instantiate and configure the pca9538 device driver to use the provided
 *   I2C device
 *   instance.
 *
 * Input Parameters:
 *   dev     - An I2C driver instance
 *   minor   - The device i2c address
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   an ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *pca9538_initialize(FAR struct i2c_master_s *dev,
                                        FAR struct pca9538_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_IOEXPANDER_PCA9538_H */
