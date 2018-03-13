/****************************************************************************
 * include/nuttx/wireless/ieee802154/xbee.h
 *
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author: Anthony Merlino <anthony@vergeaero.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE802154_XBEE_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE802154_XBEE_H

/****************************************************************************
 * Included files
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
 *   Register XBee network device.  The network device is what binds the XBee MAC
 *   to the network layer (6LoWPAN, PF_IEEE802154).
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
