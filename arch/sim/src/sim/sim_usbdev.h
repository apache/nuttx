/****************************************************************************
 * arch/sim/src/sim/sim_usbdev.h
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

#ifndef __ARCH_SIM_SRC_SIM_USB_DEV_H
#define __ARCH_SIM_SRC_SIM_USB_DEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef __SIM__
#include "config.h"
#endif

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* NuttX Endpoint descriptor */

struct host_usb_epdesc_s
{
  uint8_t   len;              /* Descriptor length */
  uint8_t   type;             /* Descriptor type */
  uint8_t   addr;             /* Endpoint address */
  uint8_t   attr;             /* Endpoint attributes */
  uint16_t  mxpacketsize;     /* Maximum packet size */
  uint8_t   interval;         /* Interval */
};

/* This structure is used to send control requests to a USB device. */

struct host_usb_ctrlreq_s
{
  uint8_t   type;             /* Matches request type */
  uint8_t   req;              /* Matches request field */
  uint16_t  value;
  uint16_t  index;
  uint16_t  len;
  uint8_t   data[0];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Host USB Interface */

int host_usbdev_init(uint32_t speed);
int host_usbdev_deinit(void);
int host_usbdev_pullup(bool enable);
int host_usbdev_epconfig(uint8_t epno,
                         const struct host_usb_epdesc_s *epdesc);
int host_usbdev_epdisable(uint8_t epno);
int host_usbdev_epstall(uint8_t epno, bool resume);
int host_usbdev_epcancel(uint8_t epno);
int host_usbdev_epwrite(uint8_t epno, uint8_t flags,
                        uint8_t *data, uint16_t len);
struct host_usb_ctrlreq_s *host_usbdev_ep0read(void);
uint8_t *host_usbdev_epread(uint8_t epno, uint16_t *len);
void host_usbdev_epread_end(uint8_t epno);

#endif /* __ARCH_SIM_SRC_SIM_USB_DEV_H */
