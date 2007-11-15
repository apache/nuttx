/****************************************************************************
 * net/uip/uip-tcpcallback.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: uip_tcpcallback
 *
 * Description:
 *   Inform the application holding the TCP socket of a change in state.
 *
 * Assumptions:
 *   This function is called at the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

uint8 uip_tcpcallback(struct uip_driver_s *dev, struct uip_conn *conn, uint8 flags)
{
  /* Preserve the UIP_ACKDATA, UIP_CLOSE, and UIP_ABORT in the response.
   * These is needed by uIP to handle responses and buffer state.  The
   * UIP_NEWDATA indication will trigger the ACK response, but must be
   * explicitly set in the callback.
   */

  uint8 ret = flags;

  vdbg("flags: %02x\n", flags);

  /* Check if there is a data callback */

  if (conn->data_event)
    {
      /* Perform the callback.  Callback function normally returns the input flags,
       * however, the implemenation may set one of the following:
       *
       *   UIP_CLOSE   - Gracefully close the current connection
       *   UIP_ABORT   - Abort (reset) the current connection on an error that
       *                 prevents UIP_CLOSE from working.
       *
       * Or clear the following:
       *
       *   UIP_NEWDATA - May be cleared to suppress returning the ACK response.
       *                 (dev->d_len should also be set to zero in this case).
       */

      ret = conn->data_event(dev, conn, flags);
    }
  else if ((flags & UIP_CONN_EVENTS) == 0)
    {
      /* There is no handler to receive new data in place and this is not a
       * connection event (which may also include new data that must be ACKed).
       * In this case, clear the UIP_NEWDATA bit so that no ACK will be sent
       * and drop the packet.
       */

      dbg("No listener on connection\n");

#ifdef CONFIG_NET_STATISTICS
      uip_stat.tcp.drop++;
#endif

      ret       &= ~UIP_NEWDATA;
      dev->d_len = 0;
    }

  /* Check if there is a connection-related event and a connection
   * callback.
   */

  if (((flags & UIP_CONN_EVENTS) != 0) && conn->connection_event)
    {
      /* Perform the callback */

      conn->connection_event(conn, flags);
    }

  return ret;
}

#endif /* CONFIG_NET */
