/****************************************************************************
 * drivers/input/ads7843e.h
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
 *   "Touch Screen Controller, ADS7843," Burr-Brown Products from Texas
 *    Instruments, SBAS090B, September 2000, Revised May 2002"
 *
 * See also:
 *   "Low Voltage I/O Touch Screen Controller, TSC2046," Burr-Brown Products
 *    from Texas Instruments, SBAS265F, October 2002, Revised August 2007."
 *
 *   "XPT2046 Data Sheet," Shenzhen XPTek Technology Co., Ltd, 2007
 */

#ifndef __DRIVERS_INPUT_ADS7843E_H
#define __DRIVERS_INPUT_ADS7843E_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <poll.h>
#include <nuttx/wqueue.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/input/ads7843e.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Reference counting is partially implemented, but not needed in the current
 * design.
 */

#undef CONFIG_ADS7843E_REFCNT

/* ADS7843E Interfaces ******************************************************/

/* ADS7843E command bit settings */

#define ADS7843E_CMD_PD0          (1 << 0)  /* Bit 0: Power down mode select bit 0 */
#define ADS7843E_CMD_PD1          (1 << 1)  /* Bit 1: Power down mode select bit 1 */
#define ADS7843E_CMD_SER          (1 << 2)  /* Bit 2: SER/DFR\: 0:DFR 1:SER */
#define ADS7843E_CMD_MODE8        (1 << 3)  /* Bit 3: Mode 1:8-bits 0:12-bits */
#define ADS7843E_CMD_CHAN_SHIFT   (4)       /* Bits 4-6: Channel select bits */
#define ADS7843E_CMD_CHAN_MASK    (7 << ADS7843E_CMD_CHAN_SHIFT)
#define ADS7843E_CMD_START        (1 << 7)  /* Bit 7: Start Bit */

/* ADS7843E Commands */

#define ADS7843_CMD_YPOSITION \
  ((1 << ADS7843E_CMD_CHAN_SHIFT)| ADS7843E_CMD_START | ADS7843E_CMD_PD0 | ADS7843E_CMD_PD1)
#define ADS7843_CMD_XPOSITION \
  ((5 << ADS7843E_CMD_CHAN_SHIFT)| ADS7843E_CMD_START | ADS7843E_CMD_PD0 | ADS7843E_CMD_PD1)
#define ADS7843_CMD_ENABPENIRQ \
  ((1 << ADS7843E_CMD_CHAN_SHIFT)| ADS7843E_CMD_START)

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/input[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/input%d"
#define DEV_NAMELEN  16

/* Poll the pen position while the pen is down at this rate (50MS): */

#define ADS7843E_WDOG_DELAY       MSEC2TICK(50)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes the state of one contact */

enum ads7843e_contact_e
{
  CONTACT_NONE = 0,                    /* No contact */
  CONTACT_DOWN,                        /* First contact */
  CONTACT_MOVE,                        /* Same contact, possibly different position */
  CONTACT_UP,                          /* Contact lost */
};

/* This structure describes the results of one ADS7843E sample */

struct ads7843e_sample_s
{
  uint8_t  id;                          /* Sampled touch point ID */
  uint8_t  contact;                     /* Contact state (see enum ads7843e_contact_e) */
  bool     valid;                       /* True: x,y contain valid, sampled data */
  uint16_t x;                           /* Measured X position */
  uint16_t y;                           /* Measured Y position */
};

/* This structure describes the state of one ADS7843E driver instance */

struct ads7843e_dev_s
{
#ifdef CONFIG_ADS7843E_MULTIPLE
  FAR struct ads7843e_dev_s *flink;     /* Supports a singly linked list of drivers */
#endif
#ifdef CONFIG_ADS7843E_REFCNT
  uint8_t crefs;                        /* Number of times the device has been opened */
#endif
  uint8_t nwaiters;                     /* Number of threads waiting for ADS7843E data */
  uint8_t id;                           /* Current touch point ID */
  volatile bool penchange;              /* An unreported event is buffered */
  uint16_t threshx;                     /* Thresholding X value */
  uint16_t threshy;                     /* Thresholding Y value */
  sem_t devsem;                         /* Manages exclusive access to this structure */
  sem_t waitsem;                        /* Used to wait for the availability of data */

  FAR struct ads7843e_config_s *config; /* Board configuration data */
  FAR struct spi_dev_s *spi;            /* Saved SPI driver instance */
  struct work_s work;                   /* Supports the interrupt handling "bottom half" */
  struct ads7843e_sample_s sample;      /* Last sampled touch point data */
  struct wdog_s wdog;                   /* Poll the position while the pen is down */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_ADS7843E_NPOLLWAITERS];
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
