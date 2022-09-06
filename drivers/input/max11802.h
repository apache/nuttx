/****************************************************************************
 * drivers/input/max11802.h
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
 *   "Low-Power, Ultra-Small Resistive Touch-Screen Controllers
 *    with I2C/SPI Interface" Maxim IC, Rev 3, 10/2010
 */

#ifndef __DRIVERS_INPUT_MAX11802_H
#define __DRIVERS_INPUT_MAX11802_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <poll.h>
#include <nuttx/wqueue.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/input/max11802.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MAX11802 Interfaces ******************************************************/

/* LSB of register addresses specifies read (1) or write (0). */

#define MAX11802_CMD_XPOSITION ((0x52 << 1) | 1)
#define MAX11802_CMD_YPOSITION ((0x54 << 1) | 1)
#define MAX11802_CMD_MEASUREXY (0x70 << 1)
#define MAX11802_CMD_MODE_WR   (0x0B << 1)
#define MAX11802_CMD_MODE_RD   ((0x0B << 1) | 1)
#define MAX11802_CMD_AVG_WR    (0x03 << 1)
#define MAX11802_CMD_SAMPLE_WR (0x04 << 1)
#define MAX11802_CMD_TIMING_WR (0x05 << 1)
#define MAX11802_CMD_DELAY_WR  (0x06 << 1)
#define MAX11802_CMD_PULL_WR   (0x07 << 1)

/* Register values to set */

#define MAX11802_MODE    0x06
#define MAX11802_AVG     0x55
#define MAX11802_SAMPLE  0xAA
#define MAX11802_TIMING  0x77
#define MAX11802_DELAY   0x55
#define MAX11802_PULL    0x33

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/input[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/input%d"
#define DEV_NAMELEN  16

/* Poll the pen position while the pen is down at this rate (50MS): */

#define MAX11802_WDOG_DELAY     MSEC2TICK(50)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes the state of one contact */

enum max11802_contact_3
{
  CONTACT_NONE = 0,                    /* No contact */
  CONTACT_DOWN,                        /* First contact */
  CONTACT_MOVE,                        /* Same contact, possibly different position */
  CONTACT_UP,                          /* Contact lost */
};

/* This structure describes the results of one MAX11802 sample */

struct max11802_sample_s
{
  uint8_t  id;                          /* Sampled touch point ID */
  uint8_t  contact;                     /* Contact state (see enum ads7843e_contact_e) */
  bool     valid;                       /* True: x,y contain valid, sampled data */
  uint16_t x;                           /* Measured X position */
  uint16_t y;                           /* Measured Y position */
};

/* This structure describes the state of one MAX11802 driver instance */

struct max11802_dev_s
{
#ifdef CONFIG_ADS7843E_MULTIPLE
  FAR struct ads7843e_dev_s *flink;     /* Supports a singly linked list of drivers */
#endif
  uint8_t nwaiters;                     /* Number of threads waiting for MAX11802 data */
  uint8_t id;                           /* Current touch point ID */
  volatile bool penchange;              /* An unreported event is buffered */
  uint16_t threshx;                     /* Thresholding X value */
  uint16_t threshy;                     /* Thresholding Y value */
  mutex_t devlock;                      /* Manages exclusive access to this structure */
  sem_t waitsem;                        /* Used to wait for the availability of data */

  FAR struct max11802_config_s *config; /* Board configuration data */
  FAR struct spi_dev_s *spi;            /* Saved SPI driver instance */
  struct work_s work;                   /* Supports the interrupt handling "bottom half" */
  struct max11802_sample_s sample;      /* Last sampled touch point data */
  struct wdog_s wdog;                   /* Poll the position while the pen is down */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_MAX11802_NPOLLWAITERS];
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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_INPUT_ADS7843E_H */
