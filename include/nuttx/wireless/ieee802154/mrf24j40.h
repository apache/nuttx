/****************************************************************************
 * include/nuttx/wireless/ieee802154/mrf24j40.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE802154_MRF24J40_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE802154_MRF24J40_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <nuttx/arch.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The MRF24J40 provides interrupts to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanism for controlling
 * the MRF24J40 GPIO interrupt.
 *
 * The MRF24J40 interrupt is an active low, *level* interrupt.
 * From Datasheet:
 * "Note 1: The INTEDGE polarity defaults to:
 *
 *   0 = Falling Edge. Ensure that the interrupt polarity matches the
 *       interrupt pin polarity of the host microcontroller.
 *
 *  Note 2: The INT pin will remain high or low, depending on INTEDGE
 *   polarity setting, until INTSTAT register is read."
 */

struct mrf24j40_lower_s
{
  int  (*attach)(FAR const struct mrf24j40_lower_s *lower, xcpt_t handler,
                 FAR void *arg);
  void (*enable)(FAR const struct mrf24j40_lower_s *lower, bool state);
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

/****************************************************************************
 * Name: mrf24j40_init
 *
 * Description:
 *   Initialize the IEEE802.15.4 driver. The MRF24J40 device is assumed to
 *   be in the post-reset state upon entry to this function.
 *
 * Input Parameters:
 *   spi   - A reference to the platform's SPI driver for the MRF24J40
 *   lower - The MCU-specific interrupt used to control low-level MCU
 *           functions (i.e., MRF24J40 GPIO interrupts).
 *   devno - If more than one MRF24J40 is supported, then this is the
 *           zero based number that identifies the MRF24J40;
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

struct spi_dev_s; /* Forward reference */
FAR struct ieee802154_radio_s *
  mrf24j40_init(FAR struct spi_dev_s *spi,
                FAR const struct mrf24j40_lower_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE802154_MRF24J40_H */
