/****************************************************************************
 * drivers/ioexpander/pcf8575.h
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

#ifndef __DRIVERS_IOEXPANDER_PCF8575_H
#define __DRIVERS_IOEXPANDER_PCF8575_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/mutex.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/pcf8575.h>

#include <nuttx/i2c/i2c_master.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_PCF8575)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *   CONFIG_I2C
 *     I2C support is required
 *   CONFIG_IOEXPANDER
 *     Enables I/O expander support
 *
 * CONFIG_IOEXPANDER_PCF8575
 *   Enables support for the PCF8575 driver (Needs CONFIG_INPUT)
 * CONFIG_PCF8575_MULTIPLE
 *   Can be defined to support multiple PCF8575 devices on board.
 */

#ifndef CONFIG_I2C
#  error "CONFIG_I2C is required by PCF8575"
#endif

/* PCF8575 Definitions ******************************************************/

#define PCF8575_I2C_MAXFREQUENCY  400000       /* 400KHz */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure represents the state of the PCF8575 driver */

struct pcf8575_dev_s
{
  struct ioexpander_dev_s dev;         /* Nested structure to allow casting as public gpio
                                        * expander. */
  FAR struct pcf8575_config_s *config; /* Board configuration data */
  FAR struct i2c_master_s *i2c;        /* Saved I2C driver instance */
  mutex_t lock;                        /* Mutual exclusion */
  uint16_t inpins;                     /* Set of input pins */
  uint16_t outstate;                   /* State of all output pins */
};

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_PCF8575 */
#endif /* __DRIVERS_IOEXPANDER_PCF8575_H */
