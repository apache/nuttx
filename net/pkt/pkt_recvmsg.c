/****************************************************************************
 * net/pkt/pkt_recvmsg.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
  unsigned int offset;
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

  offset = (dev->d_appdata - dev->d_iob->io_data) - dev->d_iob->io_offset;

  recvlen = iob_copyout(pstate->pr_buffer, dev->d_iob, recvlen, offset);

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
                                          FAR void *pvpriv, uint16_t flags)
{
  struct pkt_recvfrom_s *pstate = pvpriv;

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
  nxsem_init(&pstate->pr_sem, 0, 0); /* Doesn't really fail */

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
 *   result   The result of the net_sem_wait operation (may indicate EINTR)
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

  /* If net_sem_wait failed, then we were probably reawakened by a signal.
   * In this case, net_sem_wait will have returned negated errno
   * appropriately.
   */

  if (result < 0)
    {
      return result;
    }

  return pstate->pr_recvlen;
}

/****************************************************************************
 * Name: pkt_readahead
 *
 * Description:
 *   Copy the buffered read-ahead data to the user buffer.
 *
 * Input Parameters:
 *   conn  -  PKT socket connection structure containing the read-
 *            ahead data.
 *   buf      target buffer.
 *
 * Returned Value:
 *   Number of bytes copied to the user buffer
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static inline ssize_t pkt_readahead(FAR struct pkt_conn_s *conn,
                                    FAR void *buf, size_t buflen)
{
  FAR struct iob_s *iob;
  ssize_t ret = -ENODATA;

  /* Check there is any packets already buffered in a read-ahead buffer. */

  if ((iob = iob_peek_queue(&conn->readahead)) != NULL)
    {
      DEBUGASSERT(iob->io_pktlen > 0);

      /* Copy to user */

      ret = iob_copyout(buf, iob, buflen, 0);

      ninfo("Received %zd bytes (of %u)\n", ret, iob->io_pktlen);

      /* Remove the I/O buffer chain from the head of the read-ahead
       * buffer queue.
       */

      iob_remove_queue(&conn->readahead);

      /* And free the I/O buffer chain */

      iob_free_chain(iob);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_recvmsg
 *
 * Description:
 *   Implements the socket recvmsg interface for the case of the AF_INET
 *   and AF_INET6 address families.  pkt_recvmsg() receives messages from
 *   a socket, and may be used to receive data on a socket whether or not it
 *   is connection-oriented.
 *
 *   If 'msg_name' is not NULL, and the underlying protocol provides the
 *   source address, this source address is filled in.  The argument
 *   'msg_namelen' is initialized to the size of the buffer associated with
 *   msg_name, and modified on return to indicate the actual size of the
 *   address stored there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive the message
 *   flags    Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters received. If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recvmsg() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvmsg() for the list of appropriate error values).
 *
 ****************************************************************************/

ssize_t pkt_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                    int flags)
{
  FAR void *buf = msg->msg_iov->iov_base;
  size_t len = msg->msg_iov->iov_len;
  FAR struct sockaddr *from = msg->msg_name;
  FAR socklen_t *fromlen = &msg->msg_namelen;
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

  /* Check if there is buffered read-ahead data for this socket.  We may have
   * already received the response to previous command.
   */

  if (!IOB_QEMPTY(&conn->readahead))
    {
      ret = pkt_readahead(conn, buf, len);
    }
  else if (_SS_ISNONBLOCK(conn->sconn.s_flags) ||
           (flags & MSG_DONTWAIT) != 0)
    {
      /* Handle non-blocking PKT sockets */

      ret = -EAGAIN;
    }
  else
    {
      pkt_recvfrom_initialize(psock, buf, len, from, fromlen, &state);

      /* Get the device driver that will service this transfer */

      dev  = pkt_find_device(conn);
      if (dev == NULL)
        {
          ret = -ENODEV;
          goto errout_with_state;
        }

      /* TODO pkt_recvfrom_initialize() expects from to be of type
       * sockaddr_in, but in our case is sockaddr_ll
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

          /* Wait for either the receive to complete or for an error/timeout
           * to occur. NOTES:  (1) net_sem_wait will also terminate if a
           * signal is received, (2) the network is locked!  It will be
           * un-locked while the task sleeps and automatically re-locked when
           * the task restarts.
           */

          ret = net_sem_wait(&state.pr_sem);

          /* Make sure that no further events are processed */

          pkt_callback_free(dev, conn, state.pr_cb);
          ret = pkt_recvfrom_result(ret, &state);
        }
      else
        {
          ret = -EBUSY;
        }

errout_with_state:
      pkt_recvfrom_uninitialize(&state);
    }

  net_unlock();
  return ret;
}

#endif /* CONFIG_NET */
