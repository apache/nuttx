/****************************************************************************
 * drivers/ioexpander/pca9555.h
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

/* References:
 *   "16-bit I2C-bus and SMBus I/O port with interrupt product datasheet",
 *   Rev. 08 - 22 October 2009, NXP
 */

#ifndef __DRIVERS_IOEXPANDER_PCA9555_H
#define __DRIVERS_IOEXPANDER_PCA9555_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/pca9555.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_PCA9555)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *   CONFIG_I2C
 *     I2C support is required
 *   CONFIG_IOEXPANDER
 *     Enables support for the PCA9555 I/O expander
 *
 * CONFIG_IOEXPANDER_PCA9555
 *   Enables support for the PCA9555 driver (Needs CONFIG_INPUT)
 * CONFIG_PCA9555_MULTIPLE
 *   Can be defined to support multiple PCA9555 devices on board.
 * CONFIG_PCA9555_INT_NCALLBACKS
 *   Maximum number of supported pin interrupt callbacks.
 */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_PCA9555_INT_NCALLBACKS
#    define CONFIG_PCA9555_INT_NCALLBACKS 4
#  endif
#endif

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_SCHED_WORKQUEUE
#    error Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected.
#  endif
#endif

#undef CONFIG_PCA9555_REFCNT

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/input[n] device driver path.
 *  It defined here
 * so that it will be used consistently in all places.
 */

/* PCA9555 Resources ********************************************************/

#define PCA9555_GPIO_NPINS 16 /* All pins can be used as GPIOs */

#ifndef CONFIG_I2C
#error "CONFIG_I2C is required by PCA9555"
#endif

#define PCA9555_MAXDEVS             8

/* I2C frequency */

#define PCA9555_I2C_MAXFREQUENCY    400000       /* 400KHz */

/* PCA9555 Registers ********************************************************/

/* Register Addresses */

#define PCA9555_REG_INPUT  0x00
#define PCA9555_REG_OUTPUT 0x02
#define PCA9555_REG_POLINV 0x04
#define PCA9555_REG_CONFIG 0x06

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* This type represents on registered pin interrupt callback */

struct pca9555_callback_s
{
  ioe_pinset_t pinset;                 /* Set of pin interrupts that will generate
                                        * the callback. */
  ioe_callback_t cbfunc;               /* The saved callback function pointer */
  FAR void *cbarg;                     /* Callback argument */
};
#endif

/* This structure represents the state of the PCA9555 driver */

struct pca9555_dev_s
{
  struct ioexpander_dev_s      dev;     /* Nested structure to allow casting as public gpio
                                         * expander. */
#ifdef CONFIG_PCA9555_SHADOW_MODE
  uint8_t sreg[8];                      /* Shadowed registers of the PCA9555 */
#endif
#ifdef CONFIG_PCA9555_MULTIPLE
  FAR struct pca9555_dev_s    *flink;   /* Supports a singly linked list of drivers */
#endif
  FAR struct pca9555_config_s *config;  /* Board configuration data */
  FAR struct i2c_master_s     *i2c;     /* Saved I2C driver instance */
  sem_t                        exclsem; /* Mutual exclusion */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  struct work_s work;                   /* Supports the interrupt handling "bottom half" */

  /* Saved callback information for each I/O expander client */

  struct pca9555_callback_s cb[CONFIG_PCA9555_INT_NCALLBACKS];
#endif
};

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_PCA9555 */
#endif /* __DRIVERS_IOEXPANDER_PCA9555_H */
