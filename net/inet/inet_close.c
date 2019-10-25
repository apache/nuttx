/****************************************************************************
 * net/inet/inet_close.c
 *
 *   Copyright (C) 2007-2017, 2019 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <arch/irq.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/tcp.h>
#include <nuttx/net/udp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"
#include "local/local.h"
#include "socket/socket.h"
#include "usrsock/usrsock.h"
#include "inet/inet.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef NET_TCP_HAVE_STACK
struct tcp_close_s
{
  FAR struct devif_callback_s *cl_cb;     /* Reference to TCP callback instance */
  FAR struct socket           *cl_psock;  /* Reference to the TCP socket */
  sem_t                        cl_sem;    /* Signals disconnect completion */
  int                          cl_result; /* The result of the close */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_close_eventhandler
 *
 * Description:
 *   Handle network callback events.
 *
 * Input Parameters:
 *   conn - TCP connection structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from normal user-level logic
 *
 ****************************************************************************/

#ifdef NET_TCP_HAVE_STACK
static uint16_t tcp_close_eventhandler(FAR struct net_driver_s *dev,
                                       FAR void *pvconn, FAR void *pvpriv,
                                       uint16_t flags)
{
  FAR struct tcp_close_s *pstate = (FAR struct tcp_close_s *)pvpriv;
  FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)pvconn;

  DEBUGASSERT(pstate != NULL && conn != NULL);

  ninfo("conn: %p flags: %04x\n", conn, flags);

  /* TCP_DISCONN_EVENTS:
   *   TCP_CLOSE:    The remote host has closed the connection
   *   TCP_ABORT:    The remote host has aborted the connection
   *   TCP_TIMEDOUT: The remote did not respond, the connection timed out
   *   NETDEV_DOWN:  The network device went down
   */

  if ((flags & TCP_DISCONN_EVENTS) != 0)
    {
      /* The disconnection is complete.  Wake up the waiting thread with an
       * appropriate result.  Success is returned in these cases:
       *
       * * TCP_CLOSE indicates normal successful closure.  The TCP_CLOSE
       *   event is sent when the remote ACKs the outgoing FIN in the
       *   FIN_WAIT_1 state.  That is the appropriate time for the
       *   application to close the socket.
       *
       *   NOTE:  The underlying connection, however, will persist, waiting
       *   for the FIN to be returned by the remote in the TIME_WAIT state.
       *
       * * TCP_ABORT is less likely but still means that the socket was
       *   closed, albeit abnormally due to a RST from the remote.
       *
       * * TCP_TIMEDOUT would be reported in this context if there is no
       *   ACK response to the FIN in the FIN_WAIT_2 state.  The socket will
       *   again be closed abnormally.
       *
       * This is the only true error case.
       *
       * * NETDEV_DOWN would indicate that the network went down before the
       *   close completed.  A non-standard ENODEV error will be returned
       *   in this case.  The socket will be left in a limbo state if the
       *   network is taken down but should recover later when the
       *   NETWORK_DOWN event is processed further.
       */

      if ((flags & NETDEV_DOWN) != 0)
        {
          pstate->cl_result = -ENODEV;
        }
      else
        {
          pstate->cl_result = OK;
        }

      goto end_wait;
    }

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  /* Check if all outstanding bytes have been ACKed */

  else if (conn->unacked != 0 || !sq_empty(&conn->write_q))
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

  UNUSED(conn);           /* May not be used */
  return flags;

end_wait:
  pstate->cl_cb->flags = 0;
  pstate->cl_cb->priv  = NULL;
  pstate->cl_cb->event = NULL;
  nxsem_post(&pstate->cl_sem);

  ninfo("Resuming\n");
  UNUSED(conn);           /* May not be used */
  return 0;
}
#endif /* NET_TCP_HAVE_STACK */

/****************************************************************************
 * Name: tcp_close_txnotify
 *
 * Description:
 *   Notify the appropriate device driver that we are have data ready to
 *   be send (TCP)
 *
 * Input Parameters:
 *   psock - Socket state structure
 *   conn  - The TCP connection structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef NET_TCP_HAVE_STACK
static inline void tcp_close_txnotify(FAR struct socket *psock,
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

      netdev_ipv4_txnotify(conn->u.ipv4.laddr, conn->u.ipv4.raddr);
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else /* if (psock->s_domain == PF_INET6) */
#endif /* CONFIG_NET_IPv4 */
    {
      /* Notify the device driver that send data is available */

      DEBUGASSERT(psock->s_domain == PF_INET6);
      netdev_ipv6_txnotify(conn->u.ipv6.laddr, conn->u.ipv6.raddr);
    }
#endif /* CONFIG_NET_IPv6 */
}
#endif /* NET_TCP_HAVE_STACK */

/****************************************************************************
 * Name: tcp_close_disconnect
 *
 * Description:
 *   Break any current TCP connection
 *
 * Input Parameters:
 *   conn - TCP connection structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from normal user-level logic
 *
 ****************************************************************************/

#ifdef NET_TCP_HAVE_STACK
static inline int tcp_close_disconnect(FAR struct socket *psock)
{
  struct tcp_close_s state;
  FAR struct tcp_conn_s *conn;
#ifdef CONFIG_NET_SOLINGER
  struct timespec abstime;
  bool linger;
#endif
  int ret = OK;

  /* Interrupts are disabled here to avoid race conditions */

  net_lock();

  conn = (FAR struct tcp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

#ifdef CONFIG_NET_SOLINGER
  /* SO_LINGER
   *   Lingers on a close() if data is present. This option controls the
   *   action taken when unsent messages queue on a socket and close() is
   *   performed. If SO_LINGER is set, the system shall block the calling
   *   thread during close() until it can transmit the data or until the
   *   time expires. If SO_LINGER is not specified, and close() is issued,
   *   the system handles the call in a way that allows the calling thread
   *   to continue as quickly as possible. This option takes a linger
   *   structure, as defined in the <sys/socket.h> header, to specify the
   *   state of the option and linger interval.
   */

  linger = _SO_GETOPT(psock->s_options, SO_LINGER);
  if (linger)
    {
      /* Get the current time */

      ret = clock_gettime(CLOCK_REALTIME, &abstime);
      if (ret >= 0)
        {
          /* NOTE: s_linger's unit is deciseconds so we don't need to update
           * abstime.tv_nsec here.
           */

          abstime.tv_sec += psock->s_linger / DSEC_PER_SEC;

          /* Wait until abstime for the buffered TX data to be sent. */

          ret = tcp_txdrain(psock, &abstime);
          if (ret < 0)
            {
              /* tcp_txdrain may fail, but that won't stop us from closing
               * the socket.
               */

              nerr("ERROR: tcp_txdrain() failed: %d\n", ret);
            }
        }
    }
#endif

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  /* If we have a semi-permanent write buffer callback in place, then
   * is needs to be be nullified.
   *
   * Commit f1ef2c6cdeb032eaa1833cc534a63b50c5058270:
   * "When a socket is closed, it should make sure that any pending write
   *  data is sent before the FIN is sent.  It already would wait for all
   *  sent data to be acked, however it would discard any pending write
   *  data that had not been sent at least once.
   *
   * "This change adds a check for pending write data in addition to unacked
   *  data.  However, to be able to actually send any new data, the send
   *  callback must be left.  The callback should be freed later when the
   *  socket is actually destroyed."
   *
   * REVISIT:  Where and how exactly is s_sndcb ever freed?  Is there a
   * memory leak here?
   */

  psock->s_sndcb = NULL;
#endif

  /* Check for the case where the host beat us and disconnected first */

  if (conn->tcpstateflags == TCP_ESTABLISHED &&
      (state.cl_cb = tcp_callback_alloc(conn)) != NULL)
    {
      /* Set up to receive TCP data event callbacks */

      state.cl_cb->flags = (TCP_NEWDATA | TCP_POLL | TCP_DISCONN_EVENTS);
      state.cl_cb->event = tcp_close_eventhandler;

      /* A non-NULL value of the priv field means that lingering is
       * enabled.
       */

      state.cl_cb->priv  = (FAR void *)&state;

      /* Set up for the lingering wait */

      state.cl_psock     = psock;
      state.cl_result    = -EBUSY;

      /* This semaphore is used for signaling and, hence, should not have
       * priority inheritance enabled.
       */

      nxsem_init(&state.cl_sem, 0, 0);
      nxsem_setprotocol(&state.cl_sem, SEM_PRIO_NONE);

      /* Notify the device driver of the availability of TX data */

      tcp_close_txnotify(psock, conn);

      /* Wait for the disconnect event */

      (void)net_lockedwait(&state.cl_sem);

      /* We are now disconnected */

      nxsem_destroy(&state.cl_sem);
      tcp_callback_free(conn, state.cl_cb);

      /* Free the connection
       * No more references on the connection
       */

      conn->crefs = 0;

      /* Get the result of the close */

      ret = state.cl_result;
    }

  /* Free network resources */

  tcp_free(conn);

  net_unlock();
  return ret;
}
#endif /* NET_TCP_HAVE_STACK */

/****************************************************************************
 * Name: udp_close
 *
 * Description:
 *   Break any current UDP connection
 *
 * Input Parameters:
 *   conn - UDP connection structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from normal user-level logic
 *
 ****************************************************************************/

#ifdef NET_UDP_HAVE_STACK
static inline int udp_close(FAR struct socket *psock)
{
  FAR struct udp_conn_s *conn;
#ifdef CONFIG_NET_SOLINGER
  struct timespec abstime;
  bool linger;
#endif

  /* Interrupts are disabled here to avoid race conditions */

  net_lock();

  conn = (FAR struct udp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

#ifdef CONFIG_NET_SOLINGER
  /* SO_LINGER
   *   Lingers on a close() if data is present. This option controls the
   *   action taken when unsent messages queue on a socket and close() is
   *   performed. If SO_LINGER is set, the system shall block the calling
   *   thread during close() until it can transmit the data or until the
   *   time expires. If SO_LINGER is not specified, and close() is issued,
   *   the system handles the call in a way that allows the calling thread
   *   to continue as quickly as possible. This option takes a linger
   *   structure, as defined in the <sys/socket.h> header, to specify the
   *   state of the option and linger interval.
   */

  linger = _SO_GETOPT(psock->s_options, SO_LINGER);
  if (linger)
    {
      int ret;

      /* Get the current time */

      ret = clock_gettime(CLOCK_REALTIME, &abstime);
      if (ret >= 0)
        {
          /* NOTE: s_linger's unit is deciseconds so we don't need to update
           * abstime.tv_nsec here.
           */

          abstime.tv_sec += psock->s_linger / DSEC_PER_SEC;

          /* Wait until abstime for the buffered TX data to be sent. */

          ret = udp_txdrain(psock, &abstime);
          if (ret < 0)
            {
              /* udp_txdrain may fail, but that won't stop us from closing
               * the socket.
               */

              nerr("ERROR: udp_txdrain() failed: %d\n", ret);
            }
        }
    }
#endif

#ifdef CONFIG_NET_UDP_WRITE_BUFFERS
  /* Free any semi-permanent write buffer callback in place. */

  if (psock->s_sndcb != NULL)
    {
      udp_callback_free(conn->dev, conn, psock->s_sndcb);
      psock->s_sndcb = NULL;
    }
#endif

  /* And free the connection structure */

  conn->crefs = 0;
  udp_free(psock->s_conn);
  net_unlock();
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_close
 *
 * Description:
 *   Performs the close operation on an AF_INET or AF_INET6 socket instance
 *
 * Input Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 * Assumptions:
 *
 ****************************************************************************/

int inet_close(FAR struct socket *psock)
{
  /* Perform some pre-close operations for the AF_INET/AF_INET6 address
   * types.
   */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        {
#ifdef NET_TCP_HAVE_STACK
          FAR struct tcp_conn_s *conn = psock->s_conn;
          int ret;

          /* Is this the last reference to the connection structure (there
           * could be more if the socket was dup'ed).
           */

          if (conn->crefs <= 1)
            {
              /* Yes... then perform the disconnection now */

              tcp_unlisten(conn); /* No longer accepting connections */
              conn->crefs = 0;    /* Discard our reference to the connection */

              /* Break any current connections and close the socket */

              ret = tcp_close_disconnect(psock);
              if (ret < 0)
                {
                  /* This would normally occur only if there is a timeout
                   * from a lingering close.
                   */

                  nerr("ERROR: tcp_close_disconnect failed: %d\n", ret);
                  return ret;
                }

              /* Stop the network monitor for all sockets */

              tcp_stop_monitor(conn, TCP_CLOSE);
            }
          else
            {
              /* No.. Just decrement the reference count */

              conn->crefs--;

              /* Stop monitor for this socket only */

              tcp_close_monitor(psock);
            }
#else
        nwarn("WARNING: SOCK_STREAM support is not available in this "
              "configuration\n");
        return -EAFNOSUPPORT;
#endif /* NET_TCP_HAVE_STACK */
        }
        break;
#endif /* CONFIG_NET_TCP */

#ifdef CONFIG_NET_UDP
      case SOCK_DGRAM:
        {
#ifdef NET_UDP_HAVE_STACK
          FAR struct udp_conn_s *conn = psock->s_conn;
          int ret;

          /* Is this the last reference to the connection structure (there
           * could be more if the socket was dup'ed).
           */

          if (conn->crefs <= 1)
            {
              /* Yes... Clost the socket */

              ret = udp_close(psock);
              if (ret < 0)
                {
                  /* This would normally occur only if there is a timeout
                   * from a lingering close.
                   */

                  nerr("ERROR: udp_close failed: %d\n", ret);
                  return ret;
                }
            }
          else
            {
              /* No.. Just decrement the reference count */

              conn->crefs--;
            }
#else
          nwarn("WARNING: SOCK_DGRAM support is not available in this "
                "configuration\n");
          return -EAFNOSUPPORT;
#endif /* NET_UDP_HAVE_STACK */
        }
        break;
#endif /* CONFIG_NET_UDP */

      default:
        return -EBADF;
    }

  return OK;
}
