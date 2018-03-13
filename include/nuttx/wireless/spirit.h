/****************************************************************************
 * include/nuttx/wireless/spirit.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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
