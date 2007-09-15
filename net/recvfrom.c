/****************************************************************************
 * net/recvfrom.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <string.h>
#include <errno.h>
#include <arch/irq.h>
#include <nuttx/clock.h>
#include <net/uip/uip-arch.h>

#include "net-internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define TCP_TIMEO 10  /* Deciseconds after data received before recv() returns */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct recvfrom_s
{
#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
  FAR struct socket *rf_sock;       /* The parent socket structure */
#endif
  sem_t              rf_sem;        /* Semaphore signals recv completion */
  size_t             rf_buflen;     /* Length of receive buffer */
  char              *rf_buffer;     /* Pointer to receive buffer */
#ifndef CONFIG_DISABLE_CLOCK
  uint32             rf_starttime;  /* rcv start time for determining timeout */
#endif
  size_t             rf_recvlen;    /* The received length */
  int                rf_result;     /* OK on success, otherwise a negated errno. */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: recvfrom_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   receive operation via by the uIP layer.
 *
 * Parameters:
 *   dev      The sructure of the network driver that caused the interrupt
 *   private  An instance of struct recvfrom_s cast to void*
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

static void recvfrom_interrupt(struct uip_driver_s *dev, void *private)
{
  struct recvfrom_s *pstate = (struct recvfrom_s *)private;
  FAR struct socket *psock;
  size_t recvlen;

  /* 'private' might be null in some race conditions (?) */

  if (pstate)
    {
      /* Get the socket reference from the private data */

      psock = pstate->rf_sock;

      /* If new data is available, then complete the read action. */

      if (uip_newdata())
        {
          /* Get the length of the data to return */
          if (dev->d_len > pstate->rf_buflen)
            {
              recvlen = pstate->rf_buflen;
            }
          else
            {
              recvlen = dev->d_len;
            }

          /* Copy the new appdata into the user buffer */

          memcpy(pstate->rf_buffer, dev->d_appdata, recvlen);

          /* Update the accumulated size of the data read */

          pstate->rf_recvlen += recvlen;
          pstate->rf_buffer  += recvlen;
          pstate->rf_buflen  -= recvlen;

          /* Are we finished?  If this is a UDP socket or if the user
         * buffer has been filled, then we are finished.
         */

#ifdef CONFIG_NET_UDP
          if (psock->s_type == SOCK_DGRAM)
            {
              struct uip_udp_conn *udp_conn;

              /* Don't allow any further UDP call backs. */

              udp_conn          = (struct uip_udp_conn *)psock->s_conn;
              udp_conn->private = NULL;
              udp_conn->event   = NULL;

              /* Wake up the waiting thread, returning the number of bytes
               * actually read.
               */

              sem_post(&pstate->rf_sem);
            }
          else
#endif
          if (pstate->rf_buflen == 0)
            {
              struct uip_conn *conn;

              /* The TCP receive buffer is full.  Return now, perhaps truncating
               * the received data (need to fix that).
               *
               * Don't allow any further TCP call backs.
               */

              conn               = (struct uip_conn *)psock->s_conn;
              conn->data_private = NULL;
              conn->data_event   = NULL;

              /* Wake up the waiting thread, returning the number of bytes
               * actually read.
               */

              sem_post(&pstate->rf_sem);
            }

            /* Reset the timeout.  We will want a short timeout to terminate
               * the TCP receive.
               */

#ifndef CONFIG_DISABLE_CLOCK
            pstate->rf_starttime = g_system_timer;
#endif
        }

      /* Check for a loss of connection */

      else if ((uip_flags & (UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT)) != 0)
        {
          /* Stop further callbacks */

#ifdef CONFIG_NET_UDP
          if (psock->s_type == SOCK_DGRAM)
            {
              struct uip_udp_conn *udp_conn = (struct uip_udp_conn *)psock->s_conn;
              udp_conn->private  = NULL;
              udp_conn->event    = NULL;
            }
          else
#endif
            {
              struct uip_conn *conn = (struct uip_conn *)psock->s_conn;
              conn->data_private = NULL;
              conn->data_event   = NULL;
            }

          /* Report not connected */

          pstate->rf_result = -ENOTCONN;

          /* Wake up the waiting thread */

          sem_post(&pstate->rf_sem);
        }

      /* No data has been received -- this is some other event... probably a
       * poll -- check for a timeout.
       */

#ifndef CONFIG_DISABLE_CLOCK
      else
        {
          socktimeo_t timeo;

          /* If this is a TCP socket that has already received some data,
           * than we will always use a short timeout.
           */

          if (pstate->rf_recvlen > 0)
            {
              /* Use the short timeout */

              timeo = TCP_TIMEO;
            }

          /* No.. check for a timeout configured via setsockopts(SO_RCVTIMEO).
           * If non... we well let the read hang forever.
           */

          else
            {
#ifdef CONFIG_NET_SOCKOPTS
              timeo = psock->s_rcvtimeo;
#else
              timeo = 0;
#endif
            }

          /* Is there an effective timeout? */

          if (timeo)
            {
              /* Yes.. Check if the timeout has elapsed */

              if (net_timeo(pstate->rf_starttime, timeo))
                {
                  /* Yes.. the timeout has elapsed... do not allow any further
                   * callbacks
                   */

#ifdef CONFIG_NET_UDP
                  if (psock->s_type == SOCK_DGRAM)
                    {
                      struct uip_udp_conn *udp_conn;

                      /* Stop further callbacks */

                      udp_conn          = (struct uip_udp_conn *)psock->s_conn;
                      udp_conn->private = NULL;
                      udp_conn->event   = NULL;

                      /* Report a timeout error */

                      pstate->rf_result = -EAGAIN;
                    }
                  else
#endif
                    {
                      struct uip_conn *conn;

                      conn               = (struct uip_conn *)psock->s_conn;
                      conn->data_private = NULL;
                      conn->data_event   = NULL;

                      /* Report an error only if no data has been received */

                      if (pstate->rf_recvlen == 0)
                        {
                          /* Report the timeout error */

                          pstate->rf_result = -EAGAIN;
                        }
                    }

                  /* Wake up the waiting thread, returning either the error -EAGAIN
                   * that signals the timeout event or the data received up to
                   * the point tht the timeout occured (no error).
                   */

                  sem_post(&pstate->rf_sem);
                }
            }
        }
#endif
    }
}

/****************************************************************************
 * Function: recvfrom_init
 *
 * Description:
 *   Initialize the state structure
 *
 * Parameters:
 *   psock    Pointer to the socket structure for the socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void recvfrom_init(FAR struct socket *psock, FAR void *buf, size_t len,
                          struct recvfrom_s *pstate)
{
  /* Initialize the state structure. */

  memset(pstate, 0, sizeof(struct recvfrom_s));
  (void)sem_init(&pstate->rf_sem, 0, 0); /* Doesn't really fail */
  pstate->rf_sock   = psock;
  pstate->rf_buflen = len;
  pstate->rf_buffer = buf;

#ifndef CONFIG_DISABLE_CLOCK
  /* Set up the start time for the timeout */

  pstate->rf_starttime = g_system_timer;
#endif
}

/****************************************************************************
 * Function: recvfrom_result
 *
 * Description:
 *   Evaluate the result of the recv operations
 *
 * Parameters:
 *   result   The result of the sem_wait operation (may indicate EINTR)
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   The result of the recv operation with errno set appropriately
 *
 * Assumptions:
 *
 ****************************************************************************/

static ssize_t recvfrom_result(int result, struct recvfrom_s *pstate)
{
  int save_errno = *get_errno_ptr(); /* In case something we do changes it */

  /* Release semaphore in the state structure */

  sem_destroy(&pstate->rf_sem);

  /* Check for a error/timeout detected by the interrupt handler.  Errors are
   * signaled by negative errno values for the rcv length
   */

  if (pstate->rf_result < 0)
    {
      /* Return EGAIN on a timeout or ENOTCONN on loss of connection */

      return pstate->rf_result;
    }

  /* If sem_wait failed, then we were probably reawakened by a signal. In
   * this case, sem_wait will have set errno appropriately.
   */

  if (result < 0)
    {
      return -save_errno;
    }

  return pstate->rf_recvlen;
}

/****************************************************************************
 * Function: udp_recvfrom
 *
 * Description:
 *   Perform the recvfrom operation for a UDP SOCK_DGRAM
 *
 * Parameters:
 *   psock    Pointer to the socket structure for the SOCK_DRAM socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   infrom   INET ddress of source
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -errno is returned (see recvfrom for list of errnos).
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
#ifdef CONFIG_NET_IPv6
static ssize_t udp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                            FAR const struct sockaddr_in6 *infrom )
#else
static ssize_t udp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                            FAR const struct sockaddr_in *infrom )
#endif
{
  struct uip_udp_conn *udp_conn;
  struct recvfrom_s state;
  irqstate_t save;
  int ret;

  /* Perform the UDP recvfrom() operation */

  /* Initialize the state structure.  This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save = irqsave();
  recvfrom_init(psock, buf, len, &state);

  /* Setup the UDP socket */

  ret = uip_udpconnect(psock->s_conn, NULL);
  if (ret < 0)
    {
      irqrestore(save);
      return ret;
    }

  /* Set up the callback in the connection */

  udp_conn          = (struct uip_udp_conn *)psock->s_conn;
  udp_conn->private = (void*)&state;
  udp_conn->event   = recvfrom_interrupt;

  /* Enable the UDP socket */

  uip_udpenable(psock->s_conn);

  /* Wait for either the receive to complete or for an error/timeout to occur.
   * NOTES:  (1) sem_wait will also terminate if a signal is received, (2)
   * interrupts are disabled!  They will be re-enabled while the task sleeps
   * and automatically re-enabled when the task restarts.
   */

  ret = sem_wait(&state. rf_sem);

  /* Make sure that no further interrupts are processed */

  uip_udpdisable(psock->s_conn);
  udp_conn->private = NULL;
  udp_conn->event   = NULL;
  irqrestore(save);

#warning "Needs to return server address"
  return recvfrom_result(ret, &state);
}
#endif /* CONFIG_NET_UDP */

/****************************************************************************
 * Function: tcp_recvfrom
 *
 * Description:
 *   Perform the recvfrom operation for a TCP/IP SOCK_STREAM
 *
 * Parameters:
 *   psock    Pointer to the socket structure for the SOCK_DRAM socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   infrom   INET ddress of source
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -errno is returned (see recvfrom for list of errnos).
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static ssize_t tcp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                            FAR const struct sockaddr_in6 *infrom )
#else
static ssize_t tcp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                            FAR const struct sockaddr_in *infrom )
#endif
{
  struct uip_conn *conn;
  struct recvfrom_s state;
  irqstate_t save;
  int ret;

  /* Verify that the SOCK_STREAM has been connected */

  if (_SS_ISCONNECTED(psock->s_flags))
    {
      /* The SOCK_STREAM must be connect in order to recive */

      return -ENOTCONN;
    }


  /* Initialize the state structure.  This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save = irqsave();
  recvfrom_init(psock, buf, len, &state);

  /* Set up the callback in the connection */

  conn               = (struct uip_conn *)psock->s_conn;
  conn->data_private = (void*)&state;
  conn->data_event   = recvfrom_interrupt;

  /* Wait for either the receive to complete or for an error/timeout to occur.
   * NOTES:  (1) sem_wait will also terminate if a signal is received, (2)
   * interrupts are disabled!  They will be re-enabled while the task sleeps
   * and automatically re-enabled when the task restarts.
   */

  ret = sem_wait(&state. rf_sem);

  /* Make sure that no further interrupts are processed */

  conn->data_private = NULL;
  conn->data_event   = NULL;
  irqrestore(save);

#warning "Needs to return server address"
  return recvfrom_result(ret, &state);
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: recvfrom
 *
 * Description:
 *   recvfrom() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN
 *     The socket is marked non-blocking and the receive operation would block,
 *     or a receive timeout had been set and the timeout expired before data
 *     was received.
 *   EBADF
 *     The argument sockfd is an invalid descriptor.
 *   ECONNREFUSED
 *     A remote host refused to allow the network connection (typically because
 *     it is not running the requested service).
 *   EFAULT
 *     The receive buffer pointer(s) point outside the process's address space.
 *   EINTR
 *     The receive was interrupted by delivery of a signal before any data were
 *     available.
 *   EINVAL
 *     Invalid argument passed.
 *   ENOMEM
 *     Could not allocate memory.
 *   ENOTCONN
 *     The socket is associated with a connection-oriented protocol and has
 *     not been connected.
 *   ENOTSOCK
 *     The argument sockfd does not refer to a socket.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t recvfrom(int sockfd, FAR void *buf, size_t len, int flags, FAR struct sockaddr *from,
                 FAR socklen_t *fromlen)
{
  FAR struct socket *psock;
#ifdef CONFIG_NET_IPv6
  FAR const struct sockaddr_in6 *infrom = (const struct sockaddr_in6 *)from;
#else
  FAR const struct sockaddr_in *infrom = (const struct sockaddr_in *)from;
#endif
  ssize_t ret;
  int err;

  /* Verify that non-NULL pointers were passed */

  if (!buf || !from || !fromlen)
    {
      err = EINVAL;
      goto errout;
    }

  /* Get the underlying socket structure */
  /* Verify that the sockfd corresponds to valid, allocated socket */

  psock = sockfd_socket(sockfd);
  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* Verify that a valid address has been provided */

#ifdef CONFIG_NET_IPv6
  if (from->sa_family != AF_INET6 || *fromlen < sizeof(struct sockaddr_in6))
#else
  if (from->sa_family != AF_INET || *fromlen < sizeof(struct sockaddr_in))
#endif
  {
      err = EBADF;
      goto errout;
  }

  /* Set the socket state to receiving */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_RECV);

  /* Perform the TCP/IP or UDP recv() operation */

#ifdef CONFIG_NET_UDP
  if (psock->s_type == SOCK_STREAM)
#endif
    {
      ret = tcp_recvfrom(psock, buf, len, infrom);
    }
#ifdef CONFIG_NET_UDP
  else
    {
      ret = udp_recvfrom(psock, buf, len, infrom);
    }
#endif

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Handle returned errors */

  if (ret < 0)
    {
      err = -ret;
      goto errout;
    }

  /* Success return */

  return ret;

errout:
  *get_errno_ptr() = err;
  return ERROR;
}

#endif /* CONFIG_NET */
