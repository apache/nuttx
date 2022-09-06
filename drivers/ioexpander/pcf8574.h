/****************************************************************************
 * drivers/ioexpander/pcf8574.h
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

#ifndef __DRIVERS_IOEXPANDER_PCF8574_H
#define __DRIVERS_IOEXPANDER_PCF8574_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/pcf8574.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_PCF8574)

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
 * CONFIG_IOEXPANDER_PCF8574
 *   Enables support for the PCF8574 driver (Needs CONFIG_INPUT)
 * CONFIG_PCF8574_MULTIPLE
 *   Can be defined to support multiple PCF8574 devices on board.
 * CONFIG_PCF8574_INT_NCALLBACKS
 *   Maximum number of supported pin interrupt callbacks.
 * CONFIG_PCF8574_INT_POLL
 *   Enables a poll for missed interrupts
 * CONFIG_PCF8574_INT_POLLDELAY
 *   If CONFIG_PCF8574_INT_POLL=y, then this is the delay in microseconds
 *   between polls for missed interrupts.
 */

#ifndef CONFIG_I2C
#  error "CONFIG_I2C is required by PCF8574"
#endif

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_PCF8574_INT_NCALLBACKS
#    define CONFIG_PCF8574_INT_NCALLBACKS 4
#  endif
#endif

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_SCHED_WORKQUEUE
#    error Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected.
#  endif
#endif

#ifndef CONFIG_PCF8574_INT_POLLDELAY
#  define CONFIG_PCF8574_INT_POLLDELAY 500000
#endif

/* PCF8574 Definitions ******************************************************/

#define PCF8574_I2C_MAXFREQUENCY  400000       /* 400KHz */
#define PCF8574_POLLDELAY         (CONFIG_PCF8574_INT_POLLDELAY / USEC_PER_TICK)

#define PCF8574_LEVEL_SENSITIVE(d,p) \
  (((d)->trigger  & ((ioe_pinset_t)1 << (p))) == 0)
#define PCF8574_LEVEL_HIGH(d,p) \
  (((d)->level[0] & ((ioe_pinset_t)1 << (p))) != 0)
#define PCF8574_LEVEL_LOW(d,p) \
  (((d)->level[1] & ((ioe_pinset_t)1 << (p))) != 0)

#define PCF8574_EDGE_SENSITIVE(d,p) \
  (((d)->trigger  & ((ioe_pinset_t)1 << (p))) != 0)
#define PCF8574_EDGE_RISING(d,p) \
  (((d)->level[0] & ((ioe_pinset_t)1 << (p))) != 0)
#define PCF8574_EDGE_FALLING(d,p) \
  (((d)->level[1] & ((ioe_pinset_t)1 << (p))) != 0)
#define PCF8574_EDGE_BOTH(d,p) \
  (PCF8574_LEVEL_RISING(d,p) && PCF8574_LEVEL_FALLING(d,p))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* This type represents on registered pin interrupt callback */

struct pcf8574_callback_s
{
  ioe_pinset_t pinset;                 /* Set of pin interrupts that will generate
                                        * the callback. */
  ioe_callback_t cbfunc;               /* The saved callback function pointer */
  FAR void *cbarg;                     /* Callback argument */
};
#endif

/* This structure represents the state of the PCF8574 driver */

struct pcf8574_dev_s
{
  struct ioexpander_dev_s dev;         /* Nested structure to allow casting as public gpio
                                        * expander. */
  FAR struct pcf8574_config_s *config; /* Board configuration data */
  FAR struct i2c_master_s *i2c;        /* Saved I2C driver instance */
  mutex_t lock;                        /* Mutual exclusion */
  uint8_t inpins;                      /* Set of input pins */
  uint8_t outstate;                    /* State of all output pins */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#ifdef CONFIG_PCF8574_INT_POLL
  struct wdog_s wdog;                  /* Timer used to poll for missed interrupts */
#endif

  uint8_t input;                       /* Last input registers */
  uint8_t intstat;                     /* Pending interrupts */
  uint8_t trigger;                     /* Bit encoded: 0=level 1=edge */
  uint8_t level[2];                    /* Bit encoded: 01=high/rising, 10 low/falling, 11 both */
  struct work_s work;                  /* Supports the interrupt handling "bottom half" */

  /* Saved callback information for each I/O expander client */

  struct pcf8574_callback_s cb[CONFIG_PCF8574_INT_NCALLBACKS];
#endif
};

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_PCF8574 */
#endif /* __DRIVERS_IOEXPANDER_PCF8574_H */
