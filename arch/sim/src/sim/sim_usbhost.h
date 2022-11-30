/****************************************************************************
 * arch/sim/src/sim/sim_usbhost.h
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

#ifndef __ARCH_SIM_SRC_SIM_USB_HOST_H
#define __ARCH_SIM_SRC_SIM_USB_HOST_H

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
};

struct host_usb_datareq_s
{
  struct host_usb_datareq_s  *flink;
  uint8_t                     addr;
  uint8_t                     xfrtype;
  uint8_t                    *data;
  uint16_t                    len;
  uint16_t                    xfer;
  bool                        success;
  void                       *priv;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Host USB host Interface */

int host_usbhost_init(void);
bool host_usbhost_getconnstate(void);
int host_usbhost_open(void);
void host_usbhost_close(void);
int host_usbhost_ep0trans(struct host_usb_ctrlreq_s *ctrlreq,
                          struct host_usb_datareq_s *datareq);
int host_usbhost_eptrans(struct host_usb_datareq_s *datareq);
struct host_usb_datareq_s *host_usbhost_getcomplete(void);

#endif /* __ARCH_SIM_SRC_SIM_USB_HOST_H */
