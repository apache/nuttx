/****************************************************************************
 * drivers/input/stmpe811.h
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
 *   "STMPE811 S-Touch advanced resistive touchscreen controller with 8-bit
 *    GPIO expander," Doc ID 14489 Rev 6, CD00186725, STMicroelectronics"
 */

#ifndef __DRIVERS_INPUT_STMPE811_H
#define __DRIVERS_INPUT_STMPE811_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/input/stmpe811.h>

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_STMPE811)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Reference counting is partially implemented, but not needed in the current
 * design.
 */

#undef CONFIG_STMPE811_REFCNT

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/input[n] device driver path.
 * It defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/input%d"
#define DEV_NAMELEN  16

/* STMPE811 Resources *******************************************************/

#ifndef CONFIG_STMPE811_TSC_DISABLE
#  define STMPE811_ADC_NPINS  4 /* Only pins 0-3 can be used for ADC */
#  define STMPE811_GPIO_NPINS 4 /* Only pins 0-3 can be used as GPIOs */
#else
#  define STMPE811_ADC_NPINS  8 /* All pins can be used for ADC */
#  define STMPE811_GPIO_NPINS 8 /* All pins can be used as GPIOs */
#endif

/* Driver flags */

#define STMPE811_FLAGS_TSC_INITIALIZED  (1 << 0) /* 1: The TSC block has been initialized */
#define STMPE811_FLAGS_GPIO_INITIALIZED (1 << 1) /* 1: The GIO block has been initialized */
#define STMPE811_FLAGS_ADC_INITIALIZED  (1 << 2) /* 1: The ADC block has been initialized */
#define STMPE811_FLAGS_TS_INITIALIZED   (1 << 3) /* 1: The TS block has been initialized */

/* Timeout to detect missing pen up events */

#define STMPE811_PENUP_TICKS            MSEC2TICK(100)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes the state of one contact */

enum stmpe811_contact_3
{
  CONTACT_NONE = 0,                    /* No contact */
  CONTACT_DOWN,                        /* First contact */
  CONTACT_MOVE,                        /* Same contact, possibly different position */
  CONTACT_UP,                          /* Contact lost */
};

/* This structure describes the results of one STMPE811 sample */

struct stmpe811_sample_s
{
  uint8_t  id;                         /* Sampled touch point ID */
  uint8_t  contact;                    /* Contact state (see enum stmpe811_contact_e) */
  bool     valid;                      /* True: x,y,z contain valid, sampled data */
  uint16_t x;                          /* Measured X position */
  uint16_t y;                          /* Measured Y position */
  uint8_t  z;                          /* Measured Z index */
};

/* This structure represents the state of the STMPE811 driver */

struct stmpe811_dev_s
{
#ifdef CONFIG_STMPE811_MULTIPLE
  FAR struct stmpe811_dev_s *flink;     /* Supports a singly linked list of drivers */
#endif

  /* Common fields */

  FAR struct stmpe811_config_s *config; /* Board configuration data */
  mutex_t lock;                         /* Manages exclusive access to this structure */
#ifdef CONFIG_STMPE811_SPI
  FAR struct spi_dev_s *spi;            /* Saved SPI driver instance */
#else
  FAR struct i2c_master_s *i2c;         /* Saved I2C driver instance */
#endif

  uint8_t inuse;                        /* STMPE811 pins in use */
  uint8_t flags;                        /* See STMPE811_FLAGS_* definitions */
  struct work_s work;                   /* Supports the interrupt handling "bottom half" */

  /* Fields that may be disabled to save size if touchscreen support
   * is not used.
   */

#ifndef CONFIG_STMPE811_TSC_DISABLE
#ifdef CONFIG_STMPE811_REFCNT
  uint8_t crefs;                        /* Number of times the device has been opened */
#endif
  uint8_t nwaiters;                     /* Number of threads waiting for STMPE811 data */
  uint8_t id;                           /* Current touch point ID */
  uint8_t minor;                        /* Touchscreen minor device number */
  volatile bool penchange;              /* An unreported event is buffered */

  uint16_t threshx;                     /* Thresholded X value */
  uint16_t threshy;                     /* Thresholded Y value */
  sem_t waitsem;                        /* Used to wait for the availability of data */

  struct work_s timeout;                /* Supports tiemeout work */
  struct wdog_s wdog;                   /* Timeout to detect missing pen down events */
  struct stmpe811_sample_s sample;      /* Last sampled touch point data */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_STMPE811_NPOLLWAITERS];
#endif

  /* Fields that may be disabled to save size of GPIO support is not used */

#if !defined(CONFIG_STMPE811_GPIO_DISABLE) && !defined(CONFIG_STMPE811_GPIOINT_DISABLE)
  stmpe811_handler_t handlers[STMPE811_GPIO_NPINS]; /* GPIO "interrupt handlers" */
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe811_getreg8
 *
 * Description:
 *   Read from an 8-bit STMPE811 register
 *
 ****************************************************************************/

uint8_t stmpe811_getreg8(FAR struct stmpe811_dev_s *priv, uint8_t regaddr);

/****************************************************************************
 * Name: stmpe811_putreg8
 *
 * Description:
 *   Write a value to an 8-bit STMPE811 register
 *
 ****************************************************************************/

void stmpe811_putreg8(FAR struct stmpe811_dev_s *priv,
                      uint8_t regaddr, uint8_t regval);

/****************************************************************************
 * Name: stmpe811_getreg16
 *
 * Description:
 *   Read 16-bits of data from an STMPE-11 register
 *
 ****************************************************************************/

uint16_t stmpe811_getreg16(FAR struct stmpe811_dev_s *priv, uint8_t regaddr);

/****************************************************************************
 * Name: stmpe811_tscint
 *
 * Description:
 *   Handle touchscreen interrupt events (this function actually executes in
 *   the context of the worker thread).
 *
 ****************************************************************************/

#ifndef CONFIG_STMPE811_TSC_DISABLE
void stmpe811_tscworker(FAR struct stmpe811_dev_s *priv,
                         uint8_t intsta) weak_function;
#endif

/****************************************************************************
 * Name: stmpe811_gpioworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

#if !defined(CONFIG_STMPE811_GPIO_DISABLE) && !defined(CONFIG_STMPE811_GPIOINT_DISABLE)
void stmpe811_gpioworker(FAR struct stmpe811_dev_s *priv) weak_function;
#endif

#endif /* CONFIG_INPUT && CONFIG_INPUT_STMPE811 */
#endif /* __DRIVERS_INPUT_STMPE811_H */
