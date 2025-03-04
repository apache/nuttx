/****************************************************************************
 * include/nuttx/ioexpander/sx1509.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_SX1509_H
#define __INCLUDE_NUTTX_IOEXPANDER_SX1509_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the sx1509
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the sx1509 and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by
 * the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct sx1509_config_s
{
  /* Device characterization */

  uint8_t  address;   /* 7-bit I2C address (only bits 0-6 used) */
  uint32_t frequency; /* I2C or SPI frequency */

#ifdef CONFIG_SX1509_LED_ENABLE
  uint8_t  led_pre;   /* LED driver frequency prescaler */
#endif

  /* Sets the state of nReset pin */

  CODE void (*set_nreset)(bool state);

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  CODE int  (*attach)(FAR struct sx1509_config_s *state, xcpt_t isr,
                      FAR void *arg);
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
 * Name: sx1509_initialize
 *
 * Description:
 *   Initialize a SX1509 I2C device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   config  - Persistent board configuration data
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *sx1509_initialize(
                              FAR struct i2c_master_s *i2cdev,
                              FAR struct sx1509_config_s *config);

#ifdef CONFIG_SX1509_LED_ENABLE
/****************************************************************************
 * Name: sx1509_leds_initialize
 *
 * Description:
 *   Initialize the LED driver for a given SX1509 device.
 *
 ****************************************************************************/

int sx1509_leds_initialize(FAR struct ioexpander_dev_s *ioe,
                           FAR const char *devname);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_IOEXPANDER_SX1509_H */
