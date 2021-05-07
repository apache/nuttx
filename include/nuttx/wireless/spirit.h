/****************************************************************************
 * include/nuttx/wireless/spirit.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_SPIRIT_H
#define __INCLUDE_NUTTX_WIRELESS_SPIRIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* Special multicast and broadcast addresses */

#define SPIRIT_MCAST_ADDRESS 0xee
#define SPIRIT_BCAST_ADDRESS 0xff

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The Spirit provides interrupts to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanixm for controlling
 * the Spirit GPIO interrupt.
 */

struct spirit_lower_s
{
  int  (*reset)(FAR const struct spirit_lower_s *lower);
  int  (*attach)(FAR const struct spirit_lower_s *lower, xcpt_t handler,
                 FAR void *arg);
  void (*enable)(FAR const struct spirit_lower_s *lower, bool state);
};

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

struct spi_dev_s; /* Forward reference */

/****************************************************************************
 * Function: spirit_netdev_initialize
 *
 * Description:
 *   Initialize the IEEE802.15.4 driver and register it as a network device.
 *
 * Input Parameters:
 *   spi   - A reference to the platform's SPI driver for the spirit
 *   lower - The MCU-specific interrupt used to control low-level MCU
 *           functions (i.e., spirit GPIO interrupts).
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int spirit_netdev_initialize(FAR struct spi_dev_s *spi,
                             FAR const struct spirit_lower_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_IEEE802154__AT86RF23X_H */
