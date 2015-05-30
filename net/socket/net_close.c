/****************************************************************************
 * net/socket/net_close.c
 *
 *   Copyright (C) 2007-2015 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <arch/irq.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/tcp.h>
#include <nuttx/net/udp.h>

#ifdef CONFIG_NET_SOLINGER
#  include <nuttx/clock.h>
#endif

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"
#include "local/local.h"
#include "socket/socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
struct tcp_close_s
{
  FAR struct devif_callback_s *cl_cb; /* Reference to TCP callback instance */
#ifdef CONFIG_NET_SOLINGER
  FAR struct socket       *cl_psock;  /* Reference to the TCP socket */
  sem_t                    cl_sem;    /* Signals disconnect completion */
  int                      cl_result; /* The result of the close */
  uint32_t                 cl_start;  /* Time close started (in ticks) */
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: close_timeout
 *
 * Description:
 *   Check for a timeout on a lingering close.
 *
 * Parameters:
 *   pstate - close state structure
 *
 * Returned Value:
 *   TRUE:timeout FALSE:no timeout
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#if defined(CONFIG_NET_TCP) && defined(CONFIG_NET_SOLINGER)
static inline int close_timeout(FAR struct tcp_close_s *pstate)
{
  FAR struct socket *psock = 0;

  /* Make sure that we are performing a lingering close */

  if (pstate)
    {
     /* Yes Check for a timeout configured via setsockopts(SO_LINGER).
      * If none... we well let the send wait forever.
      */

     psock = pstate->cl_psock;
     if (psock && psock->s_linger != 0)
       {
         /* Check if the configured timeout has elapsed */

         return net_timeo(pstate->cl_start, psock->s_linger);
       }
    }

  /* No timeout */

  return FALSE;
}
#endif /* CONFIG_NET_SOCKOPTS && CONFIG_NET_SOLINGER */

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
static uint16_t netclose_interrupt(FAR struct net_driver_s *dev,
                                   FAR void *pvconn, FAR void *pvpriv,
                                   uint16_t flags)
{
#ifdef CONFIG_NET_SOLINGER
  FAR struct tcp_close_s *pstate = (FAR struct tcp_close_s *)pvpriv;
#endif
  FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)pvconn;

  DEBUGASSERT(conn != NULL);

  nllvdbg("conn: %p flags: %04x\n", conn, flags);

  /* TCP_DISCONN_EVENTS:
   *   TCP_CLOSE:    The remote host has closed the connection
   *   TCP_ABORT:    The remote host has aborted the connection
   *   TCP_TIMEDOUT: The remote did not respond, the connection timed out
   *   NETDEV_DOWN:  The network device went down
   */

  if ((flags & TCP_DISCONN_EVENTS) != 0)
    {
      /* The disconnection is complete */

#ifdef CONFIG_NET_SOLINGER
      /* pstate non-NULL means that we are performing a LINGERing close.*/

      if (pstate)
        {
          /* Wake up the waiting thread with a successful result */

          pstate->cl_result = OK;
          goto end_wait;
        }

      /* Otherwise, nothing is waiting on the close event and we can perform
       * the completion actions here.
       */

      else
#endif
        {
          /* Free connection resources */

          tcp_free(conn);

          /* Stop further callbacks */

          flags = 0;
        }
    }

#ifdef CONFIG_NET_SOLINGER
  /* Check for a timeout. */

  else if (pstate && close_timeout(pstate))
    {
      /* Yes.. Wake up the waiting thread and report the timeout */

      nlldbg("CLOSE timeout\n");
      pstate->cl_result = -ETIMEDOUT;
      goto end_wait;
    }

#endif /* CONFIG_NET_SOLINGER */

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  /* Check if all outstanding bytes have been ACKed */

  else if (conn->unacked != 0)
    {
      /* No... we are still waiting for ACKs.  Drop any received data, but
       * do not yet report TCP_CLOSE in the response.
       */

      dev->d_len = 0;
      flags = (flags & ~TCP_NEWDATA);
    }

#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

  else
    {
      /* Drop data received in this state and make sure that TCP_CLOSE
       * is set in the response
       */

      dev->d_len = 0;
      flags = (flags & ~TCP_NEWDATA) | TCP_CLOSE;
    }

  return flags;

#ifdef CONFIG_NET_SOLINGER
end_wait:
  pstate->cl_cb->flags = 0;
  pstate->cl_cb->priv  = NULL;
  pstate->cl_cb->event = NULL;
  sem_post(&pstate->cl_sem);

  nllvdbg("Resuming\n");
  return 0;
#endif
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Function: netclose_txnotify
 *
 * Description:
 *   Notify the appropriate device driver that we are have data ready to
 *   be send (TCP)
 *
 * Parameters:
 *   psock - Socket state structure
 *   conn  - The TCP connection structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static inline void netclose_txnotify(FAR struct socket *psock,
                                     FAR struct tcp_conn_s *conn)
{
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  /* If both IPv4 and IPv6 support are enabled, then we will need to select
   * the device driver using the appropriate IP domain.
   */

  if (psock->s_domain == PF_INET)
#endif
    {
      /* Notify the device driver that send data is available */

#ifdef CONFIG_NETDEV_MULTINIC
      netdev_ipv4_txnotify(conn->u.ipv4.laddr, conn->u.ipv4.raddr);
#else
      netdev_ipv4_txnotify(conn->u.ipv4.raddr);
#endif
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else /* if (psock->s_domain == PF_INET6) */
#endif /* CONFIG_NET_IPv4 */
    {
      /* Notify the device driver that send data is available */

      DEBUGASSERT(psock->s_domain == PF_INET6);
#ifdef CONFIG_NETDEV_MULTINIC
      netdev_ipv6_txnotify(conn->u.ipv6.laddr, conn->u.ipv6.raddr);
#else
      netdev_ipv6_txnotify(conn->u.ipv6.raddr);
#endif
    }
#endif /* CONFIG_NET_IPv6 */
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
static inline int netclose_disconnect(FAR struct socket *psock)
{
  struct tcp_close_s state;
  FAR struct tcp_conn_s *conn;
  net_lock_t flags;
#ifdef CONFIG_NET_SOLINGER
  bool linger;
#endif
  int ret = OK;

  /* Interrupts are disabled here to avoid race conditions */

  flags = net_lock();
  conn = (FAR struct tcp_conn_s *)psock->s_conn;

  /* If we have a semi-permanent write buffer callback in place, then
   * release it now.
   */

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  if (psock->s_sndcb)
    {
      tcp_callback_free(conn, psock->s_sndcb);
      psock->s_sndcb = NULL;
    }
#endif

  /* There shouldn't be any callbacks registered. */

  DEBUGASSERT(conn && conn->list == NULL);

  /* Check for the case where the host beat us and disconnected first */

  if (conn->tcpstateflags == TCP_ESTABLISHED &&
      (state.cl_cb = tcp_callback_alloc(conn)) != NULL)
    {
      /* Set up to receive TCP data event callbacks */

      state.cl_cb->flags = (TCP_NEWDATA | TCP_POLL | TCP_DISCONN_EVENTS);
      state.cl_cb->event = netclose_interrupt;

#ifdef CONFIG_NET_SOLINGER
      /* Check for a lingering close */

      linger = _SO_GETOPT(psock->s_options, SO_LINGER);

      /* Has a lingering close been requested */

      if (linger)
        {
          /* A non-NULL value of the priv field means that lingering is
           * enabled.
           */

          state.cl_cb->priv  = (FAR void *)&state;

          /* Set up for the lingering wait */

          state.cl_psock     = psock;
          state.cl_result    = -EBUSY;
          sem_init(&state.cl_sem, 0, 0);

          /* Record the time that we started the wait (in ticks) */

          state.cl_start = clock_systimer();
        }
      else
#endif /* CONFIG_NET_SOLINGER */

        {
          /* We will close immediately. The NULL priv field signals this */

          state.cl_cb->priv  = NULL;

          /* No further references on the connection */

          conn->crefs = 0;
        }

      /* Notify the device driver of the availability of TX data */

      netclose_txnotify(psock, conn);

#ifdef CONFIG_NET_SOLINGER
      /* Wait only if we are lingering */

      if (linger)
        {
          /* Wait for the disconnect event */

          (void)net_lockedwait(&state.cl_sem);

          /* We are now disconnected */

          sem_destroy(&state.cl_sem);
          tcp_callback_free(conn, state.cl_cb);

          /* Free the connection */

          conn->crefs = 0;          /* No more references on the connection */
          tcp_free(conn);           /* Free uIP resources */

          /* Get the result of the close */

          ret = state.cl_result;
        }
#endif /* CONFIG_NET_SOLINGER */
    }
  else
    {
      tcp_free(conn);
    }

  net_unlock(flags);
  return ret;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Function: local_close
 *
 * Description:
 *   Performs the close operation on a local socket instance
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

#ifdef CONFIG_NET_LOCAL
static void local_close(FAR struct socket *psock)
{
  FAR struct local_conn_s *conn = psock->s_conn;

  /* Is this the last reference to the connection structure (there could
   * be more if the socket was dup'ed).
   */

  if (conn->lc_crefs <= 1)
    {
      conn->lc_crefs = 0;
      local_release(conn);
    }
  else
    {
      /* No.. Just decrement the reference count */

      conn->lc_crefs--;
    }
}
#endif /* CONFIG_NET_LOCAL */

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

  /* We perform the uIP close operation only if this is the last count on
   * the socket. (actually, I think the socket crefs only takes the values
   * 0 and 1 right now).
   */

  if (psock->s_crefs <= 1)
    {
      /* Perform uIP side of the close depending on the protocol type */

      switch (psock->s_type)
        {
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_LOCAL_STREAM)
          case SOCK_STREAM:
            {
#ifdef CONFIG_NET_LOCAL_STREAM
#ifdef CONFIG_NET_TCP
              if (psock->s_domain == PF_LOCAL)
#endif
                {
                  /* Release our reference to the local connection structure */

                  local_close(psock);
                }
#endif /* CONFIG_NET_LOCAL_STREAM */

#ifdef CONFIG_NET_TCP
#ifdef CONFIG_NET_LOCAL_STREAM
              else
#endif
                {
                  FAR struct tcp_conn_s *conn = psock->s_conn;

                  /* Is this the last reference to the connection structure
                   * (there could be more if the socket was dup'ed).
                   */

                  if (conn->crefs <= 1)
                    {
                      /* Yes... then perform the disconnection now */

                      tcp_unlisten(conn); /* No longer accepting connections */
                      conn->crefs = 0;    /* Discard our reference to the connection */

                      /* Break any current connections */

                      err = netclose_disconnect(psock);
                      if (err < 0)
                        {
                          /* This would normally occur only if there is a
                           * timeout from a lingering close.
                           */

                          goto errout_with_psock;
                        }

                      /* Stop the network monitor */

                      net_stopmonitor(conn);
                    }
                  else
                    {
                      /* No.. Just decrement the reference count */

                      conn->crefs--;
                    }
                }
#endif /* CONFIG_NET_TCP || CONFIG_NET_LOCAL_STREAM */
            }
            break;
#endif

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_LOCAL_DGRAM)
          case SOCK_DGRAM:
            {
#ifdef CONFIG_NET_LOCAL_DGRAM
#ifdef CONFIG_NET_UDP
              if (psock->s_domain == PF_LOCAL)
#endif
                {
                  /* Release our reference to the local connection structure */

                  local_close(psock);
                }
#endif /* CONFIG_NET_LOCAL_DGRAM */

#ifdef CONFIG_NET_UDP
#ifdef CONFIG_NET_LOCAL_DGRAM
              else
#endif
                {
                  FAR struct udp_conn_s *conn = psock->s_conn;

                  /* Is this the last reference to the connection structure
                   * (there could be more if the socket was dup'ed).
                   */

                  if (conn->crefs <= 1)
                    {
                      /* Yes... free the connection structure */

                      conn->crefs = 0;
                      udp_free(psock->s_conn);
                    }
                  else
                    {
                      /* No.. Just decrement the reference count */

                      conn->crefs--;
                    }
                }
#endif /* CONFIG_NET_UDP || CONFIG_NET_LOCAL_DGRAM */
            }
            break;
#endif

#ifdef CONFIG_NET_PKT
          case SOCK_RAW:
            {
              FAR struct pkt_conn_s *conn = psock->s_conn;

              /* Is this the last reference to the connection structure (there
               * could be more if the socket was dup'ed).
               */

              if (conn->crefs <= 1)
                {
                  /* Yes... free the connection structure */

                  conn->crefs = 0;          /* No more references on the connection */
                  pkt_free(psock->s_conn);  /* Free uIP resources */
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

#ifdef CONFIG_NET_TCP
errout_with_psock:
  sock_release(psock);
#endif

errout:
  set_errno(err);
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
