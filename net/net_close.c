/****************************************************************************
 * net/net_close.c
 *
 *   Copyright (C) 2007-2013 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_NET

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/net/uip/uip-arch.h>

#include "net_internal.h"
#include "uip/uip_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: netclose_interrupt
 *
 * Description:
 *   Handle uIP callback events.
 *
 * Parameters:
 *   conn - uIP TCP connection structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from normal user-level logic
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static uint16_t netclose_interrupt(FAR struct uip_driver_s *dev,
                                   FAR void *pvconn, FAR void *pvpriv,
                                   uint16_t flags)
{
  FAR struct uip_conn *conn = (FAR struct uip_conn*)pvconn;

  DEBUGASSERT(conn != NULL);

  nlldbg("conn: %p flags: %04x\n", conn, flags);

  /* UIP_CLOSE:    The remote host has closed the connection
   * UIP_ABORT:    The remote host has aborted the connection
   * UIP_TIMEDOUT: The remote did not respond, the connection timed out
   */

  if ((flags & (UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT)) != 0)
    {
      /* Free connection resources */

      uip_tcpfree(conn);

      /* Stop further callbacks */

      flags = 0;
    }
  else
    {
      /* Drop data received in this state and make sure that UIP_CLOSE
       * is set in the response
       */

      dev->d_len = 0;
      flags = (flags & ~UIP_NEWDATA) | UIP_CLOSE;
    }

  return flags;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Function: netclose_disconnect
 *
 * Description:
 *   Break any current TCP connection
 *
 * Parameters:
 *   conn - uIP TCP connection structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from normal user-level logic
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static inline void netclose_disconnect(FAR struct socket *psock)
{
  FAR struct uip_callback_s *cb;
  uip_lock_t flags;

  /* Interrupts are disabled here to avoid race conditions */

  flags = uip_lock();

  struct uip_conn *conn = (struct uip_conn*)psock->s_conn;

  /* There shouldn't be any callbacks registered */

  DEBUGASSERT(conn->list == NULL);

  /* Check for the case where the host beat us and disconnected first */

  if (conn->tcpstateflags == UIP_ESTABLISHED && 
      (cb = uip_tcpcallbackalloc(conn)) != NULL)
    {
      /* Set up to receive TCP data event callbacks */

      cb->flags = UIP_NEWDATA|UIP_POLL|UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT;
      cb->event = netclose_interrupt;

      /* Notify the device driver of the availaibilty of TX data */

      netdev_txnotify(conn->ripaddr);
    }
  else
    {
      uip_tcpfree(conn);
    }

  uip_unlock(flags);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_close
 *
 * Description:
 *   Performs the close operation on a socket instance
 *
 * Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 * Assumptions:
 *
 ****************************************************************************/

int psock_close(FAR struct socket *psock)
{
  int err;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* We perform the uIP close operation only if this is the last count on the socket.
   * (actually, I think the socket crefs only takes the values 0 and 1 right now).
   */

  if (psock->s_crefs <= 1)
    {
      /* Perform uIP side of the close depending on the protocol type */

      switch (psock->s_type)
        {
#ifdef CONFIG_NET_TCP
          case SOCK_STREAM:
            {
              struct uip_conn *conn = psock->s_conn;

              /* Is this the last reference to the connection structure (there
               * could be more if the socket was dup'ed).
               */

              if (conn->crefs <= 1)
                {
                  /* Yes... then perform the disconnection now */

                  uip_unlisten(conn);          /* No longer accepting connections */
                  conn->crefs = 0;             /* No more references on the connection */
                  netclose_disconnect(psock);  /* Break any current connections */
                }
              else
                {
                  /* No.. Just decrement the reference count */

                  conn->crefs--;
                }
            }
            break;
#endif

#ifdef CONFIG_NET_UDP
          case SOCK_DGRAM:
            {
              struct uip_udp_conn *conn = psock->s_conn;

              /* Is this the last reference to the connection structure (there
               * could be more if the socket was dup'ed.
               */

              if (conn->crefs <= 1)
                {
                  /* Yes... free the connection structure */

                  conn->crefs = 0;             /* No more references on the connection */
                  uip_udpfree(psock->s_conn);  /* Free uIP resources */
                }
              else
                {
                  /* No.. Just decrement the reference count */

                  conn->crefs--;
                }
            }
            break;
#endif

          default:
            err = EBADF;
            goto errout;
        }
    }

  /* Then release our reference on the socket structure containing the connection */

  sock_release(psock);
  return OK;

errout:
  errno = err;
  return ERROR;
}

/****************************************************************************
 * Function: net_close
 *
 * Description:
 *   Performs the close operation on socket descriptors
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 * Assumptions:
 *
 ****************************************************************************/

int net_close(int sockfd)
{
  return psock_close(sockfd_socket(sockfd));
}

#endif /* CONFIG_NET */
