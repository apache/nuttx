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

#include "net_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct recvfrom_s
{
  sem_t   rf_sem;
  uint16  rf_buflen;
  char  * rf_buffer;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void recvfrom_interrupt(void *private)
{
  struct recvfrom_s *pstate = (struct recvfrom_s *)private;
  size_t recvlen;

  /* If new data is available and we are correctly intialized, then complete
   * the read action.  We could also check for POLL events here in order to
   * implement SO_RECVTIMEO.
   */

  if (uip_newdata() && private)
    {
      /* Get the length of the data to return */
      if (uip_len > pstate-> rf_buflen)
        {
          recvlen = pstate-> rf_buflen;
        }
      else
        {
          recvlen = uip_len;
        }

      /* Copy the appdate into the user data and send it */

      memcpy(pstate->rf_buffer, uip_appdata, recvlen);

      /* Don't allow any further call backs. */

      uip_conn->private = NULL;
      uip_conn->callback = NULL;

      /* Wake up the waiting thread, returning the number of bytes
       * actually read.
       */

      pstate->rf_buflen = recvlen;
      sem_post(&pstate-> rf_sem);
    }
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
 *     Could not allocate memory for recvmsg().
 *   ENOTCONN
 *     The socket is associated with a connection-oriented protocol and has
 *     not been connected.
 *   ENOTSOCK
 *     The argument sockfd does not refer to a socket.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t recvfrom(int sockfd, void *buf, size_t len, int flags, struct sockaddr *from,
                 socklen_t *fromlen)
{
  FAR struct socket *psock;
#ifdef CONFIG_NET_IPv6
  FAR const struct sockaddr_in6 *infrom = (const struct sockaddr_in6 *)from;
#else
  FAR const struct sockaddr_in *infrom = (const struct sockaddr_in *)from;
#endif
#ifdef CONFIG_NET_UDP
  struct uip_udp_conn *udp_conn;
  struct recvfrom_s state;
  irqstate_t save;
#endif
  int err;
  int ret;

  /* Get the underlying socket structure */
  /* Verify that the sockfd corresponds to valid, allocated socket */

  psock = sockfd_socket(sockfd);
  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* Perform the TCP/IP recv() operation */

  if (psock->s_type == SOCK_STREAM)
    {
#warning "TCP/IP recv not implemented"
      err = ENOSYS;
      goto errout;
    }

  /* Perform the UDP recvfrom() operation */

#ifdef CONFIG_NET_UDP
  /* Initialize the state structure.  This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save = irqsave();
  memset(&state, 0, sizeof(struct recvfrom_s));
  (void)sem_init(&state. rf_sem, 0, 0); /* Doesn't really fail */
  state. rf_buflen = len;
  state. rf_buffer = buf;

  /* Setup the UDP socket */

  ret = uip_udpconnect(psock->s_conn, NULL);
  if (ret < 0)
    {
      irqrestore(save);
      err = -ret;
      goto errout;
    }

  /* Set up the callback in the connection */

  udp_conn = (struct uip_udp_conn *)psock->s_conn;
  udp_conn->private  = (void*)&state;
  udp_conn->callback = recvfrom_interrupt;

  /* Wait for either the read to complete:  NOTES:  (1) sem_wait will also
   * terminate if a signal is received, (2) interrupts are disabled!  They
   * will be re-enabled while the task sleeps and automatically re-enabled
   * when the task restarts.
   */

  ret = sem_wait(&state. rf_sem);

  /* Make sure that no further interrupts are processed */

  uip_conn->private = NULL;
  uip_conn->callback = NULL;
  sem_destroy(&state. rf_sem);
  irqrestore(save);

  /* If sem_wait failed, then we were probably reawakened by a signal. In
   * this case, sem_wait will have set errno appropriately.
   */

  if (ret < 0)
    {
      return ERROR;
    }

  return state.rf_buflen;
#warning "Needs to return server address"
#else
  err = ENOSYS;
#endif

errout:
  *get_errno_ptr() = err;
  return ERROR;
}

#endif /* CONFIG_NET */
