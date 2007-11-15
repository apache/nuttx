/****************************************************************************
 * net/uip/uip-tcpappsend.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
 * Definitions
 ****************************************************************************/

#define BUF ((struct uip_tcpip_hdr *)&dev->d_buf[UIP_LLH_LEN])

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_tcpappsend
 *
 * Description:
 *   Handle application response
 *
 * Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   result - App result event sent
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void uip_tcpappsend(struct uip_driver_s *dev, struct uip_conn *conn, uint8 result)
{
  /* Handle the result based on the application response */

  vdbg("result: %02x\n", result);

  /* Check for connection aborted */

  if (result & UIP_ABORT)
    {
      dev->d_sndlen = 0;
      conn->tcpstateflags = UIP_CLOSED;
      vdbg("TCP state: UIP_CLOSED\n");

      uip_tcpsend(dev, conn, TCP_RST | TCP_ACK, UIP_IPTCPH_LEN);
    }

  /* Check for connection closed */

  else if (result & UIP_CLOSE)
    {
      conn->tcpstateflags = UIP_FIN_WAIT_1;
      conn->len = 1;
      conn->nrtx = 0;
      vdbg("TCP state: UIP_FIN_WAIT_1\n");

      dev->d_sndlen = 0;
      uip_tcpsend(dev, conn, TCP_FIN | TCP_ACK, UIP_IPTCPH_LEN);
    }

  /* None of the above */

  else
    {
      /* If d_sndlen > 0, the application has data to be sent. */

      if (dev->d_sndlen > 0)
        {
          /* If the connection has acknowledged data, the contents of
           * the ->len variable should be discarded.
           */

          if (result & UIP_ACKDATA)
            {
              conn->len = 0;
            }

          /* If the ->len variable is non-zero the connection has
           * already data in transit and cannot send anymore right
           * now.
           */

          if (conn->len == 0)
            {
              /* The application cannot send more than what is
               * allowed by the mss (the minumum of the MSS and the
               * available window).
               */

              if (dev->d_sndlen > conn->mss)
                {
                  dev->d_sndlen = conn->mss;
                }

              /* Remember how much data we send out now so that we
               * know when everything has been acknowledged.
               */

              conn->len = dev->d_sndlen;
            }
          else
            {
              /* If the application already had unacknowledged data,
               * we make sure that the application does not send
               * (i.e., retransmit) out more than it previously sent
               * out.
               */

              dev->d_sndlen = conn->len;
            }
        }

      /* Then handle the rest of the operation just as for the rexmit case */

      conn->nrtx = 0;
      uip_tcprexmit(dev, conn, result);
    }
}

/****************************************************************************
 * Name: uip_tcprexmit
 *
 * Description:
 *   Handle application retransmission
 *
 * Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   result - App result event sent
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void uip_tcprexmit(struct uip_driver_s *dev, struct uip_conn *conn, uint8 result)
{
  vdbg("result: %02x\n", result);

  dev->d_appdata = dev->d_snddata;

  /* If the application has data to be sent, or if the incoming packet had
   * new data in it, we must send out a packet.
   */

  if (dev->d_sndlen > 0 && conn->len > 0)
    {
      /* We always set the ACK flag in response packets adding the length of
       * the IP and TCP headers.
       */

      uip_tcpsend(dev, conn, TCP_ACK | TCP_PSH, conn->len + UIP_TCPIP_HLEN);
    }

  /* If there is no data to send, just send out a pure ACK if there is newdata. */

  else if (result & UIP_NEWDATA)
    {
      uip_tcpsend(dev, conn, TCP_ACK, UIP_TCPIP_HLEN);
    }

  /* There is nothing to do -- drop the packet */

  else
    {
      dev->d_len = 0;
    }
}
#endif /* CONFIG_NET */
