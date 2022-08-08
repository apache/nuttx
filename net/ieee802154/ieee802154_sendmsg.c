/****************************************************************************
 * net/ieee802154/ieee802154_sendmsg.c
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

#include <sys/types.h>
#include <sys/socket.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/semaphore.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/net.h>

#include "utils/utils.h"
#include "netdev/netdev.h"
#include "devif/devif.h"
#include "socket/socket.h"
#include "ieee802154/ieee802154.h"

#ifdef CONFIG_NET_IEEE802154

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon by the event handler.
 */

struct ieee802154_sendto_s
{
  FAR struct socket *is_sock;            /* Points to the parent socket structure */
  FAR struct devif_callback_s *is_cb;    /* Reference to callback instance */
  struct ieee802154_saddr_s is_destaddr; /* Frame destination address */
  sem_t is_sem;                          /* Used to wake up the waiting thread */
  FAR const uint8_t *is_buffer;          /* User buffer of data to send */
  size_t is_buflen;                      /* Number of bytes in the is_buffer */
  ssize_t is_sent;                       /* The number of bytes sent (or error) */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ieee802154_anyaddrnull
 *
 * Description:
 *   If the destination address is all zero in the MAC header buf, then it is
 *   broadcast on the 802.15.4 network.
 *
 * Input Parameters:
 *   addr    - The address to check
 *   addrlen - The length of the address in bytes
 *
 * Returned Value:
 *   True if the address is all zero.
 *
 ****************************************************************************/

static bool ieee802154_anyaddrnull(FAR const uint8_t *addr, uint8_t addrlen)
{
  while (addrlen-- > 0)
    {
      if (addr[addrlen] != 0x00)
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: ieee802154_saddrnull
 *
 * Description:
 *   If the destination address is all zero in the MAC header buf, then it is
 *   broadcast on the 802.15.4 network.
 *
 * Input Parameters:
 *   eaddr - The short address to check
 *
 * Returned Value:
 *   The address length associated with the address mode.
 *
 ****************************************************************************/

static inline bool ieee802154_saddrnull(FAR const uint8_t *saddr)
{
  return ieee802154_anyaddrnull(saddr, IEEE802154_SADDRSIZE);
}

/****************************************************************************
 * Name: ieee802154_eaddrnull
 *
 * Description:
 *   If the destination address is all zero in the MAC header buf, then it is
 *   broadcast on the 802.15.4 network.
 *
 * Input Parameters:
 *   eaddr - The extended address to check
 *
 * Returned Value:
 *   The address length associated with the address mode.
 *
 ****************************************************************************/

static inline bool ieee802154_eaddrnull(FAR const uint8_t *eaddr)
{
  return ieee802154_anyaddrnull(eaddr, IEEE802154_EADDRSIZE);
}

/****************************************************************************
 * Name: ieee802154_meta_data
 *
 * Description:
 *   Based on the collected attributes and addresses, construct the MAC meta
 *   data structure that we need to interface with the IEEE 802.15.4 MAC.
 *
 * Input Parameters:
 *   radio   - Radio network driver state instance.
 *   pstate  - Send state structure instance
 *   meta    - Location to return the corresponding meta data.
 *   paylen  - The size of the data payload to be sent.
 *
 * Returned Value:
 *   Ok is returned on success; Otherwise a negated errno value is returned.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

static void ieee802154_meta_data(FAR struct radio_driver_s *radio,
                                 FAR struct ieee802154_sendto_s *pstate,
                                 FAR struct ieee802154_frame_meta_s *meta)
{
  FAR struct ieee802154_saddr_s *destaddr;
  FAR struct ieee802154_saddr_s *srcaddr;
  FAR struct ieee802154_conn_s *conn;
  FAR struct socket *psock;
  bool rcvrnull;

  DEBUGASSERT(radio != NULL && pstate != NULL && pstate->is_sock != NULL &&
              meta != NULL);

  psock    = pstate->is_sock;
  DEBUGASSERT(psock->s_conn != NULL);

  conn     = (FAR struct ieee802154_conn_s *)psock->s_conn;
  srcaddr  = &conn->laddr;
  destaddr = &pstate->is_destaddr;

  DEBUGASSERT(srcaddr->s_mode != IEEE802154_ADDRMODE_NONE &&
              destaddr->s_mode != IEEE802154_ADDRMODE_NONE);

  /* Initialize all settings to all zero */

  memset(meta, 0, sizeof(struct ieee802154_frame_meta_s));

  /* Source address mode */

  meta->srcmode = srcaddr->s_mode;

  /* Check for a broadcast destination address (all zero) */

  if (destaddr->s_mode == IEEE802154_ADDRMODE_EXTENDED)
    {
      /* Extended destination address mode */

      rcvrnull = ieee802154_eaddrnull(destaddr->s_eaddr);
    }
  else
    {
      /* Short destination address mode */

      rcvrnull = ieee802154_saddrnull(destaddr->s_saddr);
    }

  if (rcvrnull)
    {
      meta->flags.ackreq = TRUE;
    }

  /* Destination address.
   *
   * If the output address is NULL, then it is broadcast on the 802.15.4
   * network.
   */

  if (rcvrnull)
    {
      /* Broadcast requires short address mode. */

      meta->destaddr.mode     = IEEE802154_ADDRMODE_SHORT;
      IEEE802154_PANIDCOPY(meta->destaddr.panid, destaddr->s_panid);
      meta->destaddr.saddr[0] = 0xff;
      meta->destaddr.saddr[1] = 0xff;
      memset(meta->destaddr.eaddr, 0, IEEE802154_EADDRSIZE);
    }
  else
    {
      /* Destination address. */

      meta->destaddr.mode     = destaddr->s_mode;
      IEEE802154_PANIDCOPY(meta->destaddr.panid, destaddr->s_panid);

      if (destaddr->s_mode == IEEE802154_ADDRMODE_SHORT)
        {
          IEEE802154_SADDRCOPY(meta->destaddr.saddr, destaddr->s_eaddr);
          memset(meta->destaddr.eaddr, 0, IEEE802154_EADDRSIZE);
        }
      else
        {
          IEEE802154_EADDRCOPY(meta->destaddr.eaddr, destaddr->s_eaddr);
          memset(meta->destaddr.saddr, 0, IEEE802154_SADDRSIZE);
        }
    }

  /* Handle associated with MSDU.  Will increment once per packet, not
   * necessarily per frame:  The same MSDU handle will be used for each
   * fragment of a disassembled packet.
   */

  meta->handle = radio->r_msdu_handle++;

#ifdef CONFIG_IEEE802154_SECURITY
#  warning CONFIG_IEEE802154_SECURITY not yet supported
#endif

#ifdef CONFIG_IEEE802154_UWB
#  warning CONFIG_IEEE802154_UWB not yet supported
#endif

  /* Ranging left zero */
}

/****************************************************************************
 * Name: ieee802154_sendto_eventhandler
 ****************************************************************************/

static uint16_t ieee802154_sendto_eventhandler(FAR struct net_driver_s *dev,
                                               FAR void *pvconn,
                                               FAR void *pvpriv,
                                               uint16_t flags)
{
  FAR struct radio_driver_s *radio;
  FAR struct ieee802154_sendto_s *pstate;
  struct ieee802154_frame_meta_s meta;
  FAR struct iob_s *iob;
  int hdrlen;
  int ret;

  DEBUGASSERT(pvpriv != NULL && dev != NULL);

  /* Ignore polls from non IEEE 802.15.4 network drivers */

  if (dev->d_lltype != NET_LL_IEEE802154)
    {
      return flags;
    }

  /* Make sure that this is the driver to which the socket is connected. */

#warning Missing logic

  pstate = (FAR struct ieee802154_sendto_s *)pvpriv;
  radio  = (FAR struct radio_driver_s *)dev;

  ninfo("flags: %04x sent: %zd\n", flags, pstate->is_sent);

  if (pstate != NULL && (flags & IEEE802154_POLL) != 0)
    {
      /* Initialize the meta data */

      ieee802154_meta_data(radio, pstate, &meta);

      /* Get the IEEE 802.15.4 MAC header length */

      hdrlen = radio->r_get_mhrlen(radio, &meta);
      if (hdrlen < 0)
        {
          nerr("ERROR: Failed to get header length: %d\n", hdrlen);
          ret = hdrlen;
          goto errout;
        }

      /* Verify that the user buffer can fit within the frame with this
       * MAC header.
       */

      DEBUGASSERT(CONFIG_NET_IEEE802154_FRAMELEN <= CONFIG_IOB_BUFSIZE);
      if (pstate->is_buflen + hdrlen > IEEE802154_FRAMELEN)
        {
          nerr("ERROR: User buffer will not fit into the frame: %u > %u\n",
               (unsigned int)(pstate->is_buflen + hdrlen),
               (unsigned int)CONFIG_IOB_BUFSIZE);
          ret = -E2BIG;
          goto errout;
        }

      /* Allocate an IOB to hold the frame data */

      iob = net_ioballoc(false);
      if (iob == NULL)
        {
          nwarn("WARNING: Failed to allocate IOB\n");
          return flags;
        }

      /* Initialize the IOB */

      iob->io_offset = hdrlen;
      iob->io_len    = pstate->is_buflen + hdrlen;
      iob->io_pktlen = pstate->is_buflen + hdrlen;

      /* Copy the user data into the IOB */

      memcpy(&iob->io_data[hdrlen], pstate->is_buffer, pstate->is_buflen);

      /* And submit the IOB to the network driver */

      ret = radio->r_req_data(radio, &meta, iob);
      if (ret < 0)
        {
          nerr("ERROR: r_req_data() failed: %d\n", ret);
          goto errout;
        }

      /* Save the successful result */

      pstate->is_sent = pstate->is_buflen;

      /* Don't allow any further call backs. */

      pstate->is_cb->flags    = 0;
      pstate->is_cb->priv     = NULL;
      pstate->is_cb->event    = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->is_sem);
    }

  return flags;

errout:

  /* Don't allow any further call backs. */

  pstate->is_cb->flags    = 0;
  pstate->is_cb->priv     = NULL;
  pstate->is_cb->event    = NULL;
  pstate->is_sent         = ret;

  /* Wake up the waiting thread */

  nxsem_post(&pstate->is_sem);
  return flags;
}

/****************************************************************************
 * Name: ieee802154_sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is retruend.  See sendto() for the complete list
 *   of return values.
 *
 ****************************************************************************/

static ssize_t ieee802154_sendto(FAR struct socket *psock,
                                 FAR const void *buf,
                                 size_t len, int flags,
                                 FAR const struct sockaddr *to,
                                 socklen_t tolen)
{
  FAR struct sockaddr_ieee802154_s *destaddr;
  FAR struct radio_driver_s *radio;
  FAR struct ieee802154_conn_s *conn;
  struct ieee802154_sendto_s state;
  int ret = OK;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      return -EBADF;
    }

  /* Only SOCK_DGRAM is supported (because the MAC header is stripped) */

  if (psock->s_type != SOCK_DGRAM)
    {
      /* EDESTADDRREQ.  Signifies that the socket is not connection-mode and
       * no peer address is set.
       */

      return -EDESTADDRREQ;
    }

  conn = (FAR struct ieee802154_conn_s *)psock->s_conn;

  /* Verify that the address is large enough to be a valid PF_IEEE802154
   * address.
   */

  if (tolen < sizeof(struct ieee802154_saddr_s))
    {
      return -EDESTADDRREQ;
    }

  /* Get the device driver that will service this transfer */

  radio = ieee802154_find_device(conn, &conn->laddr);
  if (radio == NULL)
    {
      return -ENODEV;
    }

  /* Perform the send operation */

  /* Initialize the state structure. This is done with the network
   * locked because we don't want anything to happen until we are
   * ready.
   */

  net_lock();
  memset(&state, 0, sizeof(struct ieee802154_sendto_s));

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&state.is_sem, 0, 0); /* Doesn't really fail */
  nxsem_set_protocol(&state.is_sem, SEM_PRIO_NONE);

  state.is_sock   = psock;          /* Socket descriptor to use */
  state.is_buflen = len;            /* Number of bytes to send */
  state.is_buffer = buf;            /* Buffer to send from */

  /* Copy the destination address */

  destaddr = (FAR struct sockaddr_ieee802154_s *)to;
  memcpy(&state.is_destaddr, &destaddr->sa_addr,
         sizeof(struct ieee802154_saddr_s));

  if (len > 0)
    {
      /* Allocate resource to receive a callback */

      state.is_cb = ieee802154_callback_alloc(&radio->r_dev, conn);
      if (state.is_cb)
        {
          /* Set up the callback in the connection */

          state.is_cb->flags = PKT_POLL;
          state.is_cb->priv  = (FAR void *)&state;
          state.is_cb->event = ieee802154_sendto_eventhandler;

          /* Notify the device driver that new TX data is available. */

          netdev_txnotify_dev(&radio->r_dev);

          /* Wait for the send to complete or an error to occur.
           * net_lockedwait will also terminate if a signal is received.
           */

          ret = net_lockedwait(&state.is_sem);

          /* Make sure that no further events are processed */

          ieee802154_callback_free(&radio->r_dev, conn, state.is_cb);
        }
    }

  nxsem_destroy(&state.is_sem);
  net_unlock();

  /* Check for a errors, Errors are signaled by negative errno values
   * for the send length
   */

  if (state.is_sent < 0)
    {
      return state.is_sent;
    }

  /* If net_lockedwait failed, then we were probably reawakened by a signal.
   * In this case, net_lockedwait will have returned negated errno
   * appropriately.
   */

  if (ret < 0)
    {
      return ret;
    }

  /* Return the number of bytes actually sent */

  return state.is_sent;
}

/****************************************************************************
 * Name: ieee802154_send
 *
 * Description:
 *   Socket send() method for the PF_IEEE802154 socket.
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see send() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

static ssize_t ieee802154_send(FAR struct socket *psock, FAR const void *buf,
                               size_t len, int flags)
{
  struct sockaddr_ieee802154_s to;
  FAR struct ieee802154_conn_s *conn;
  ssize_t ret;

  DEBUGASSERT(psock != NULL || buf != NULL);
  conn = (FAR struct ieee802154_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

  /* Only SOCK_DGRAM is supported (because the MAC header is stripped) */

  if (psock->s_type == SOCK_DGRAM)
    {
      /* send() may be used only if the socket has been connected. */

      if (!_SS_ISCONNECTED(conn->sconn.s_flags) ||
          conn->raddr.s_mode == IEEE802154_ADDRMODE_NONE)
        {
          ret = -ENOTCONN;
        }
      else
        {
          to.sa_family = AF_IEEE802154;
          memcpy(&to.sa_addr, &conn->raddr,
                 sizeof(struct ieee802154_saddr_s));

          /* Then perform the send() as sendto() */

          ret = ieee802154_sendto(psock, buf, len, flags,
                                  (FAR const struct sockaddr *)&to,
                                  sizeof(struct sockaddr_ieee802154_s));
        }
    }
  else
    {
      /* EDESTADDRREQ.  Signifies that the socket is not connection-mode and
       * no peer address is set.
       */

      ret = -EDESTADDRREQ;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ieee802154_sendmsg
 *
 * Description:
 *   Socket sendmsg() method for the PF_IEEE802154 socket.
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   msg      Message to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On error, a negated
 *   errno value is returned (see sendmsg() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

ssize_t ieee802154_sendmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                           int flags)
{
  FAR const void *buf = msg->msg_iov->iov_base;
  size_t len = msg->msg_iov->iov_len;
  FAR const struct sockaddr *to = msg->msg_name;
  socklen_t tolen = msg->msg_namelen;

  /* Validity check, only single iov supported */

  if (msg->msg_iovlen != 1)
    {
      return -ENOTSUP;
    }

  return to ? ieee802154_sendto(psock, buf, len, flags, to, tolen) :
              ieee802154_send(psock, buf, len, flags);
}

#endif /* CONFIG_NET_IEEE802154 */
