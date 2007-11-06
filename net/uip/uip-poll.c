/****************************************************************************
 * net/uip/uip-poll.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
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
#ifdef CONFIG_NET

#include <sys/types.h>
#include <debug.h>

#include <net/uip/uipopt.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>

#include "uip-internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_udppoll()
 *
 * Description:
 *   Periodic processing for a UDP connection identified by its number.
 *   This function does the necessary periodic processing (timers,
 *   polling) for a uIP TCP conneciton, and should be called by the UIP
 *   device driver when the periodic uIP timer goes off. It should be
 *   called for every connection, regardless of whether they are open of
 *   closed.
 *
 * Assumptions:
 *   This function is called from the CAN device driver may be called from
 *   the timer interrupt/watchdog handle level.
 *
 ****************************************************************************/

static inline void uip_udppoll(struct uip_driver_s *dev, unsigned int conn)
{
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: uip-poll
 *
 * Description:
 *   This function will traverse each active uIP connection structure and
 *   perform uip_interrupt with the specified event.  After each polling each
 *   active uIP connection structure, this function will call the provided
 *   callback function if the poll resulted in new data to be send.  The poll
 *   will continue until all connections have been polled or until the user-
 *   suplied function returns a non-zero value (which is would do only if
 *   it cannot accept further write data).
 *
 *   This function should be called periodically with event == UIP_TIMER for
 *   periodic processing.  This function may also be called with UIP_POLL to
 *   perform necessary periodic processing of TCP connections.
 *
 *   When the callback function is called, there may be an outbound packet
 *   waiting for service in the uIP packet buffer, and if so the d_len field
 *   is set to a value larger than zero. The device driver should be called to
 *   send out the packet.
 *
 * Assumptions:
 *   This function is called from the CAN device driver and may be called from
 *   the timer interrupt/watchdog handle level.
 *
 ****************************************************************************/

int uip_poll(struct uip_driver_s *dev, uip_poll_callback_t callback, int event)
{
  struct uip_conn *conn;
#ifdef CONFIG_NET_UDP
  struct uip_udp_conn *udp_conn;
#endif
  irqstate_t flags;

  /* Interrupts must be disabled while traversing the active connection list */

  flags = irqsave();

  /* Traverse all of the active TCP connections and perform the poll action */

  conn = NULL;
  while ((conn = uip_nexttcpconn(conn)))
    {
      uip_conn = conn;
      uip_interrupt(dev, event);
      if (callback(dev))
        {
          irqrestore(flags);
          return 1;
        }
    }
  uip_conn = NULL;

#ifdef CONFIG_NET_UDP
  /* Traverse all of the allocated UDP connections and perform a poll action */

  udp_conn = NULL;
  while ((udp_conn = uip_nextudpconn(udp_conn)))
    {
      uip_udp_conn = udp_conn;
      uip_interrupt(dev, UIP_UDP_POLL);
      if (callback(dev))
        {
          irqrestore(flags);
          return 1;
        }
    }
  uip_udp_conn = NULL;
#endif /* CONFIG_NET_UDP */

  irqrestore(flags);
  return 0;
}

#endif /* CONFIG_NET */
