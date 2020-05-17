/****************************************************************************
 * net/pkt/pkt_recvfrom.c
 *
 *   Copyright (C) 2007-2009, 2011-2017, 2020 Gregory Nutt. All rights
 *     reserved.
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
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <arch/irq.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "pkt/pkt.h"
#include "socket/socket.h"
#include <netpacket/packet.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pkt_recvfrom_s
{
  FAR struct devif_callback_s *pr_cb;  /* Reference to callback instance */
  sem_t        pr_sem;                 /* Semaphore signals recv completion */
  size_t       pr_buflen;              /* Length of receive buffer */
  FAR uint8_t *pr_buffer;              /* Pointer to receive buffer */
  ssize_t      pr_recvlen;             /* The received length */
  int          pr_result;              /* Success:OK, failure:negated errno */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_add_recvlen
 *
 * Description:
 *   Update information about space available for new data and update size
 *   of data in buffer,  This logic accounts for the case where
 *   recvfrom_udpreadahead() sets state.pr_recvlen == -1 .
 *
 * Input Parameters:
 *   pstate   recvfrom state structure
 *   recvlen  size of new data appended to buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void pkt_add_recvlen(FAR struct pkt_recvfrom_s *pstate,
                                   size_t recvlen)
{
  if (pstate->pr_recvlen < 0)
    {
      pstate->pr_recvlen = 0;
    }

  pstate->pr_recvlen += recvlen;
  pstate->pr_buffer  += recvlen;
  pstate->pr_buflen  -= recvlen;
}

/****************************************************************************
 * Name: pkt_recvfrom_newdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Input Parameters:
 *   dev      The structure of the network driver that caused the event.
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void pkt_recvfrom_newdata(FAR struct net_driver_s *dev,
                                 FAR struct pkt_recvfrom_s *pstate)
{
  size_t recvlen;

  if (dev->d_len > pstate->pr_buflen)
    {
      recvlen = pstate->pr_buflen;
    }
  else
    {
      recvlen = dev->d_len;
    }

  /* Copy the new packet data into the user buffer */

  memcpy(pstate->pr_buffer, dev->d_buf, recvlen);
  ninfo("Received %d bytes (of %d)\n", (int)recvlen, (int)dev->d_len);

  /* Update the accumulated size of the data read */

  pkt_add_recvlen(pstate, recvlen);
}

/****************************************************************************
 * Name: pkt_recvfrom_sender
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void pkt_recvfrom_sender(FAR struct net_driver_s *dev,
                                       FAR struct pkt_recvfrom_s *pstate)
{
}

/****************************************************************************
 * Name: pkt_recvfrom_eventhandler
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 * Assumptions:
 *
 ****************************************************************************/

static uint16_t pkt_recvfrom_eventhandler(FAR struct net_driver_s *dev,
                                          FAR void *pvconn,
                                          FAR void *pvpriv, uint16_t flags)
{
  struct pkt_recvfrom_s *pstate = (struct pkt_recvfrom_s *)pvpriv;

  ninfo("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* If a new packet is available, then complete the read action. */

      if ((flags & PKT_NEWDATA) != 0)
        {
          /* Copy the packet */

          pkt_recvfrom_newdata(dev, pstate);

          /* We are finished. */

          ninfo("PKT done\n");

          /* Don't allow any further call backs. */

          pstate->pr_cb->flags   = 0;
          pstate->pr_cb->priv    = NULL;
          pstate->pr_cb->event   = NULL;

          /* Save the sender's address in the caller's 'from' location */

          pkt_recvfrom_sender(dev, pstate);

          /* indicate that the data has been consumed */

          flags &= ~PKT_NEWDATA;

          /* Wake up the waiting thread, returning the number of bytes
           * actually read.
           */

          nxsem_post(&pstate->pr_sem);
        }
    }

  return flags;
}

/****************************************************************************
 * Name: pkt_recvfrom_initialize
 *
 * Description:
 *   Initialize the state structure
 *
 * Input Parameters:
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

static void pkt_recvfrom_initialize(FAR struct socket *psock, FAR void *buf,
                                    size_t len, FAR struct sockaddr *infrom,
                                    FAR socklen_t *fromlen,
                                    FAR struct pkt_recvfrom_s *pstate)
{
  /* Initialize the state structure. */

  memset(pstate, 0, sizeof(struct pkt_recvfrom_s));

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&pstate->pr_sem, 0, 0); /* Doesn't really fail */
  nxsem_set_protocol(&pstate->pr_sem, SEM_PRIO_NONE);

  pstate->pr_buflen = len;
  pstate->pr_buffer = buf;
}

/* The only un-initialization that has to be performed is destroying the
 * semaphore.
 */

#define pkt_recvfrom_uninitialize(s) nxsem_destroy(&(s)->pr_sem)

/****************************************************************************
 * Name: pkt_recvfrom_result
 *
 * Description:
 *   Evaluate the result of the recv operations
 *
 * Input Parameters:
 *   result   The result of the net_lockedwait operation (may indicate EINTR)
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   The result of the recv operation with errno set appropriately
 *
 * Assumptions:
 *
 ****************************************************************************/

static ssize_t pkt_recvfrom_result(int result,
                                   FAR struct pkt_recvfrom_s *pstate)
{
  /* Check for a error/timeout detected by the event handler.  Errors are
   * signaled by negative errno values for the rcv length
   */

  if (pstate->pr_result < 0)
    {
      /* This might return EAGAIN on a timeout or ENOTCONN on loss of
       * connection (TCP only)
       */

      return pstate->pr_result;
    }

  /* If net_lockedwait failed, then we were probably reawakened by a signal.
   * In this case, net_lockedwait will have returned negated errno
   * appropriately.
   */

  if (result < 0)
    {
      return result;
    }

  return pstate->pr_recvlen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_recvfrom
 *
 * Description:
 *   Implements the socket recvfrom interface for the case of the AF_INET
 *   and AF_INET6 address families.  pkt_recvfrom() receives messages from
 *   a socket, and may be used to receive data on a socket whether or not it
 *   is connection-oriented.
 *
 *   If 'from' is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in.  The argument 'fromlen' is
 *   initialized to the size of the buffer associated with from, and
 *   modified on return to indicate the actual size of the address stored
 *   there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters received.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recv() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvfrom() for the list of appropriate error values).
 *
 ****************************************************************************/

ssize_t pkt_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                     int flags, FAR struct sockaddr *from,
                     FAR socklen_t *fromlen)
{
  FAR struct pkt_conn_s *conn = (FAR struct pkt_conn_s *)psock->s_conn;
  FAR struct net_driver_s *dev;
  struct pkt_recvfrom_s state;
  ssize_t ret;

  /* If a 'from' address has been provided, verify that it is large
   * enough to hold this address family.
   */

  if (from != NULL && *fromlen < sizeof(sa_family_t))
    {
      return -EINVAL;
    }

  if (psock->s_type != SOCK_RAW)
    {
      nerr("ERROR: Unsupported socket type: %d\n", psock->s_type);
      ret = -ENOSYS;
    }

  /* Perform the packet recvfrom() operation */

  /* Initialize the state structure.  This is done with the network
   * locked because we don't want anything to happen until we are ready.
   */

  net_lock();
  pkt_recvfrom_initialize(psock, buf, len, from, fromlen, &state);

  /* Get the device driver that will service this transfer */

  dev  = pkt_find_device(conn);
  if (dev == NULL)
    {
      ret = -ENODEV;
      goto errout_with_state;
    }

  /* TODO pkt_recvfrom_initialize() expects from to be of type sockaddr_in,
   * but in our case is sockaddr_ll
   */

#if 0
  ret = pkt_connect(conn, NULL);
  if (ret < 0)
    {
      goto errout_with_state;
    }
#endif

  /* Set up the callback in the connection */

  state.pr_cb = pkt_callback_alloc(dev, conn);
  if (state.pr_cb)
    {
      state.pr_cb->flags  = (PKT_NEWDATA | PKT_POLL);
      state.pr_cb->priv   = (FAR void *)&state;
      state.pr_cb->event  = pkt_recvfrom_eventhandler;

      /* Wait for either the receive to complete or for an error/timeout to
       * occur. NOTES:  (1) net_lockedwait will also terminate if a signal
       * is received, (2) the network is locked!  It will be un-locked while
       * the task sleeps and automatically re-locked when the task restarts.
       */

      ret = net_lockedwait(&state.pr_sem);

      /* Make sure that no further events are processed */

      pkt_callback_free(dev, conn, state.pr_cb);
      ret = pkt_recvfrom_result(ret, &state);
    }
  else
    {
      ret = -EBUSY;
    }

errout_with_state:
  net_unlock();
  pkt_recvfrom_uninitialize(&state);
  return ret;
}

#endif /* CONFIG_NET */
