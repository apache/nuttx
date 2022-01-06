/****************************************************************************
 * include/nuttx/wireless/ieee802154/xbee.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE802154_XBEE_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE802154_XBEE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <nuttx/arch.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The XBee provides interrupts (ATTN) to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanism for controlling
 * the XBee GPIO interrupt.
 *
 * The XBee interrupt is active low.
 */

struct xbee_lower_s
{
  void (*reset)(FAR const struct xbee_lower_s *lower);
  int  (*attach)(FAR const struct xbee_lower_s *lower, xcpt_t handler,
                 FAR void *arg);
  void (*enable)(FAR const struct xbee_lower_s *lower, bool state);
  bool (*poll)(FAR const struct xbee_lower_s *lower);
};

/* This is an opaque reference to the XBee's internal private state.  It is
 * returned by xbee_init() when it is created.  It may then be used
 * at other interfaces in order to interact with the XBee MAC.
 */

typedef FAR void *XBEEHANDLE;

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: xbee_init
 *
 * Description:
 *   Initialize an XBee driver.  The XBee device is assumed to be
 *   in the post-reset state upon entry to this function.
 *
 * Input Parameters:
 *   spi   - A reference to the platform's SPI driver for the XBee
 *   lower - The MCU-specific interrupt used to control low-level MCU
 *           functions (i.e., XBee GPIO interrupts).
 *   devno - If more than one XBee is supported, then this is the
 *           zero based number that identifies the XBee;
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

struct spi_dev_s; /* Forward reference */
XBEEHANDLE xbee_init(FAR struct spi_dev_s *spi,
                     FAR const struct xbee_lower_s *lower);

/****************************************************************************
 * Name: xbee_netdev_register
 *
 * Description:
 *   Register XBee network device.  The network device is what binds the XBee
 *   MAC to the network layer (6LoWPAN, PF_IEEE802154).
 *
 * Input Parameters:
 *   xbee   - A reference to the XBee Mac driver
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int xbee_netdev_register(XBEEHANDLE xbee);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE802154_XBEE_H */
