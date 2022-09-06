/****************************************************************************
 * drivers/ioexpander/mcp23x17.h
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

#ifndef __DRIVERS_IOEXPANDER_MCP23X17_H
#define __DRIVERS_IOEXPANDER_MCP23X17_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/mcp23x17.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_MCP23X17)

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
 * CONFIG_IOEXPANDER_MCP23X17
 *   Enables support for the MCP23X17 driver (Needs CONFIG_INPUT)
 * CONFIG_MCP23X17_MULTIPLE
 *   Can be defined to support multiple MCP23X17 devices on board.
 * CONFIG_MCP23X17_INT_NCALLBACKS
 *   Maximum number of supported pin interrupt callbacks.
 * CONFIG_MCP23X17_INT_POLL
 *   Enables a poll for missed interrupts
 * CONFIG_MCP23X17_INT_POLLDELAY
 *   If CONFIG_MCP23X17_INT_POLL=y, then this is the delay in microseconds
 *   between polls for missed interrupts.
 */

#ifndef CONFIG_I2C
#  error "CONFIG_I2C is required by MCP23X17"
#endif

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_MCP23X17_INT_NCALLBACKS
#    define CONFIG_MCP23X17_INT_NCALLBACKS 4
#  endif
#endif

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_SCHED_WORKQUEUE
#    error Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected.
#  endif
#endif

#ifndef CONFIG_MCP23X17_INT_POLLDELAY
#  define CONFIG_MCP23X17_INT_POLLDELAY 500000
#endif

/* MCP23X17 Definitions *****************************************************/

/* I2C frequency */

#define MCP23X17_I2C_MAXFREQUENCY        400000       /* 400KHz */

/* MCP23X17 *****************************************************************/

/* If IOCON.BANK = 0 Addressing Mode */

#define MCP23X17_IODIRA                 0x00
#define MCP23X17_IODIRB                 0x01
#define MCP23X17_IPOLA                  0x02
#define MCP23X17_IPOLB                  0x03
#define MCP23X17_GPINTENA               0x04
#define MCP23X17_GPINTENB               0x05
#define MCP23X17_DEFVALA                0x06
#define MCP23X17_DEFVALB                0x07
#define MCP23X17_INTCONA                0x08
#define MCP23X17_INTCONB                0x09
#define MCP23X17_IOCON                  0x0a
#define MCP23X17_IOCON_2                0x0b
#define MCP23X17_GPPUA                  0x0c
#define MCP23X17_GPPUB                  0x0d
#define MCP23X17_INTFA                  0x0e
#define MCP23X17_INTFB                  0x0f
#define MCP23X17_INTCAPA                0x10
#define MCP23X17_INTCAPB                0x11
#define MCP23X17_GPIOA                  0x12
#define MCP23X17_GPIOB                  0x13
#define MCP23X17_OLATA                  0x14
#define MCP23X17_OLATB                  0x15

#define MCP23X17_IOCON_INTPOL           (1 << 1) /* Polarity of INT output pin */
#define MCP23X17_IOCON_ODR              (1 << 2) /* Config INT pin as open-drain */
#define MCP23X17_IOCON_HAEN             (1 << 3) /* HW Address enable bit */
#define MCP23X17_IOCON_DISSLW           (1 << 4) /* Disable Slew Rate for SDA output */
#define MCP23X17_IOCON_SEQOP            (1 << 5) /* Disable Sequential Operation */
#define MCP23X17_IOCON_MIRROR           (1 << 6) /* Mirror INT pins */
#define MCP23X17_IOCON_BANK             (1 << 7) /* Configure how to address register */

#define MCP23X17_NR_GPIO_MAX            16

#define MCP23X17_POLLDELAY       (CONFIG_MCP23X17_INT_POLLDELAY / USEC_PER_TICK)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* This type represents on registered pin interrupt callback */

struct mcp23x17_callback_s
{
  ioe_pinset_t pinset;              /* Set of pin interrupts that will generate
                                     * the callback. */
  ioe_callback_t cbfunc;            /* The saved callback function pointer */
  FAR void *cbarg;                  /* Callback argument */
};
#endif

/* This structure represents the state of the MCP23X17 driver */

struct mcp23x17_dev_s
{
  struct ioexpander_dev_s dev;          /* Nested structure to allow casting
                                         * as public gpio expander.
                                         */
  FAR struct mcp23x17_config_s *config; /* Board configuration data */
  FAR struct i2c_master_s *i2c;         /* Saved I2C driver instance */
  mutex_t lock;                         /* Mutual exclusion */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#ifdef CONFIG_MCP23X17_INT_POLL
  struct wdog_s wdog;                /* Timer used to poll for missed interrupts */
#endif

  ioe_pinset_t input;                /* Last input registers */
  ioe_pinset_t intstat;              /* Pending interrupts */
  struct work_s work;                /* Supports the interrupt handling "bottom half" */

  /* Saved callback information for each I/O expander client */

  struct mcp23x17_callback_s cb[CONFIG_MCP23X17_INT_NCALLBACKS];
#endif
};

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_MCP23X17 */
#endif /* __DRIVERS_IOEXPANDER_MCP23X17_H */
