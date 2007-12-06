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
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)

#include <sys/types.h>
#include <string.h>
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
 * Function: uip_readahead
 *
 * Description:
 *   Copy as much received data as possible into the readahead buffer
 *
 * Assumptions:
 *   This function is called at the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
static int uip_readahead(struct uip_readahead_s *readahead, uint8 *buf, int len)
{
  int available = CONFIG_NET_TCP_READAHEAD_BUFSIZE - readahead->rh_nbytes;
  int recvlen   = 0;

  if (len > 0 && available > 0)
    {
      /* Get the length of the data to buffer. */

      if (len > available)
        {
          recvlen = available;
        }
      else
        {
          recvlen = len;
        }

      /* Copy the new appdata into the read-ahead buffer */

      memcpy(&readahead->rh_buffer[readahead->rh_nbytes], buf, recvlen);
      readahead->rh_nbytes += recvlen;
    }
  return recvlen;
}
#endif

/****************************************************************************
 * Function: uip_dataevent
 *
 * Description:
 *   This is the default data_event handler that is called when there is no
 *   use data handler in place
 *
 * Assumptions:
 *   This function is called at the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static inline uint8
uip_dataevent(struct uip_driver_s *dev, struct uip_conn *conn, uint8 flags)
{
  uint8 ret = flags;

  /* Is there new data?  With non-zero length?  (Certain connection events
   * can have zero-length with UIP_NEWDATA set just to cause an ACK).
   */

  if ((flags & UIP_NEWDATA) != 0 && dev->d_len > 0)
    {
#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
      struct uip_readahead_s *readahead1;
      struct uip_readahead_s *readahead2 = NULL;
      uint16 recvlen = 0;
      uint8 *buf     = dev->d_appdata;
      int    buflen  = dev->d_len;

      /* First, we need to determine if we have space to buffer the data.  This
       * needs to be verified before we actually begin buffering the data. We
       * will use any remaining space in the last allocated readahead buffer
       * plus as much one additional buffer.  It is expected that the size of
       * readahead buffers are tuned so that one full packet will always fit
       * into one readahead buffer (for example if the buffer size is 420, then
       * a readahead buffer of 366 will hold a full packet of TCP data).
       */

      readahead1 = (struct uip_readahead_s*)conn->readahead.tail;
      if ((readahead1 &&
          (CONFIG_NET_TCP_READAHEAD_BUFSIZE - readahead1->rh_nbytes) > buflen) ||
          (readahead2 = uip_tcpreadaheadalloc()) != NULL)
        {
          /* We have buffer space.  Now try to append add as much data as possible
           * to the last readahead buffer attached to this connection.
           */

          if (readahead1)
            {
              recvlen = uip_readahead(readahead1, buf, buflen);
              if (recvlen > 0)
                {
                  buf    += recvlen;
                  buflen -= recvlen;
                }
            }

          /* Do we need to buffer into the newly allocated buffer as well? */

          if (readahead2)
            {
              (void)uip_readahead(readahead2, buf, buflen);

              /* Save the readahead buffer in the connection structure where
               * it can be found with recv() is called.
               */

              sq_addlast(&readahead2->rh_node, &conn->readahead);
            }

          /* Indicate that all of the data in the buffer has been consumed */

          nvdbg("Buffered %d bytes\n", dev->d_len);
          dev->d_len = 0;
        }
      else
#endif
        {
          /* There is no handler to receive new data and there are no free
           * read-ahead buffers to retain the data.  In this case, clear the
           * UIP_NEWDATA bit so that no ACK will be sent and drop the packet.
           */

#ifdef CONFIG_NET_STATISTICS
          uip_stat.tcp.syndrop++;
          uip_stat.tcp.drop++;
#endif
          ret       &= ~UIP_NEWDATA;
          dev->d_len = 0;
        }
    }

  return ret;
}

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

  nvdbg("flags: %02x\n", flags);

  /* Check if there is a data callback */

  if (conn->data_event)
    {
      /* Perform the callback.  Callback function normally returns the input flags,
       * however, the implementation may set one of the following:
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

  /* If there is no data callback -OR- if the data callback does not handle the
   * newdata event, then there is no handler in place to handle new incoming data.
   */

  if (!conn->data_event || (conn->data_flags & UIP_NEWDATA) == 0)
    {
      /* In either case, we will take a default newdata action */

      nvdbg("No listener on connection\n");
      ret = uip_dataevent(dev, conn, ret);
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

#endif /* CONFIG_NET && CONFIG_NET_TCP */
