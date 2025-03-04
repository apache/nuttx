/****************************************************************************
 * drivers/ioexpander/mcp23x08.h
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

#ifndef __DRIVERS_IOEXPANDER_MCP23X08_H
#define __DRIVERS_IOEXPANDER_MCP23X08_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/mcp23x08.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_MCP23X08)

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
 * CONFIG_IOEXPANDER_MCP23X08
 *   Enables support for the MCP23X08 driver (Needs CONFIG_INPUT)
 * CONFIG_MCP23X08_MULTIPLE
 *   Can be defined to support multiple MCP23X08 devices on board.
 * CONFIG_MCP23X08_INT_NCALLBACKS
 *   Maximum number of supported pin interrupt callbacks.
 * CONFIG_MCP23X08_INT_POLL
 *   Enables a poll for missed interrupts
 * CONFIG_MCP23X08_INT_POLLDELAY
 *   If CONFIG_MCP23X08_INT_POLL=y, then this is the delay in microseconds
 *   between polls for missed interrupts.
 */

#ifndef CONFIG_I2C
#  error "CONFIG_I2C is required by MCP23X08"
#endif

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_MCP23X08_INT_NCALLBACKS
#    define CONFIG_MCP23X08_INT_NCALLBACKS 4
#  endif
#endif

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_SCHED_WORKQUEUE
#    error Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected.
#  endif
#endif

#ifndef CONFIG_MCP23X08_INT_POLLDELAY
#  define CONFIG_MCP23X08_INT_POLLDELAY 500000
#endif

/* MCP23X08 Definitions *****************************************************/

/* I2C frequency */

#define MCP23X08_I2C_MAXFREQUENCY        400000       /* 400KHz */

/* MCP23X08 *****************************************************************/

/* If IOCON.BANK = 0 Addressing Mode */

#define MCP23X08_IODIR                  0x00
#define MCP23X08_IPOL                   0x01
#define MCP23X08_GPINTEN                0x02
#define MCP23X08_DEFVAL                 0x03
#define MCP23X08_INTCON                 0x04
#define MCP23X08_IOCON                  0x05
#define MCP23X08_GPPU                   0x06
#define MCP23X08_INTF                   0x07
#define MCP23X08_INTCAP                 0x08
#define MCP23X08_GPIO                   0x09
#define MCP23X08_OLAT                   0x0a

#define MCP23X08_IOCON_INTPOL           (1 << 1) /* Polarity of INT output pin */
#define MCP23X08_IOCON_ODR              (1 << 2) /* Config INT pin as open-drain */
#define MCP23X08_IOCON_HAEN             (1 << 3) /* HW Address enable bit */
#define MCP23X08_IOCON_DISSLW           (1 << 4) /* Disable Slew Rate for SDA output */
#define MCP23X08_IOCON_SEQOP            (1 << 5) /* Disable Sequential Operation */

#define MCP23X08_NR_GPIO_MAX            8

#define MCP23X08_POLLDELAY       (CONFIG_MCP23X08_INT_POLLDELAY / USEC_PER_TICK)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* This type represents on registered pin interrupt callback */

struct mcp23x08_callback_s
{
  ioe_pinset_t pinset;              /* Set of pin interrupts that will generate
                                     * the callback. */
  ioe_callback_t cbfunc;            /* The saved callback function pointer */
  FAR void *cbarg;                  /* Callback argument */
};
#endif

/* This structure represents the state of the MCP23X08 driver */

struct mcp23x08_dev_s
{
  struct ioexpander_dev_s dev;          /* Nested structure to allow casting
                                         * as public gpio expander.
                                         */
#ifdef CONFIG_MCP23X08_MULTIPLE
  FAR struct mcp23x08_dev_s *flink;     /* Supports a singly linked list of drivers */
#endif

  FAR struct mcp23x08_config_s *config; /* Board configuration data */
  FAR struct i2c_master_s *i2c;         /* Saved I2C driver instance */
  mutex_t lock;                         /* Mutual exclusion */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#ifdef CONFIG_MCP23X08_INT_POLL
  struct wdog_s wdog;                /* Timer used to poll for missed interrupts */
#endif

  ioe_pinset_t input;                /* Last input registers */
  ioe_pinset_t intstat;              /* Pending interrupts */
  struct work_s work;                /* Supports the interrupt handling "bottom half" */

  /* Saved callback information for each I/O expander client */

  struct mcp23x08_callback_s cb[CONFIG_MCP23X08_INT_NCALLBACKS];
#endif
};

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_MCP23X08 */
#endif /* __DRIVERS_IOEXPANDER_MCP23X08_H */
