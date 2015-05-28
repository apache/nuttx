/****************************************************************************
 * net/arp/arp_poll.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/net/netdev.h>

#include "devif/devif.h"
#include "arp/arp.h"

#ifdef CONFIG_NET_ARP_SEND

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: arp_poll
 *
 * Description:
 *   Poll all pending transfer for ARP requests to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   devif_poll() and devif_timer() and may be called from the timer
 *   interrupt/watchdog handler level.
 *
 ****************************************************************************/

int arp_poll(FAR struct net_driver_s *dev, devif_poll_callback_t callback)
{
  /* Setup for the ARP callback (most of these do not apply) */

  dev->d_appdata = NULL;
  dev->d_len     = 0;
  dev->d_sndlen  = 0;

  /* Perform the ARP callbacks */

  (void)devif_conn_event(dev, NULL, ARP_POLL, dev->d_conncb);

  /* Call back into the driver */

  return callback(dev);
}

#endif /* CONFIG_NET_ARP_SEND */
