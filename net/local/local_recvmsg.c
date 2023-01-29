/****************************************************************************
 * net/local/local_recvmsg.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_LOCAL)

#include <sys/types.h>
#include <sys/socket.h>
#include <inttypes.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "local/local.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_fifo_read
 *
 * Description:
 *   A thin layer around local_fifo_read that handles socket-related loss-of-
 *   connection events.
 *
 ****************************************************************************/

static int psock_fifo_read(FAR struct socket *psock, FAR void *buf,
                           FAR size_t *readlen, bool once)
{
  FAR struct local_conn_s *conn = (FAR struct local_conn_s *)psock->s_conn;
  int ret;

  ret = local_fifo_read(&conn->lc_infile, buf, readlen, once);
  if (ret < 0)
    {
      /* -ECONNRESET is a special case.  We may or not have received
       * data, then the peer closed the connection.
       */

      if (ret == -ECONNRESET)
        {
          nerr("ERROR: Lost connection: %d\n", ret);

          /* Report an ungraceful loss of connection.  This should
           * eventually be reported as ENOTCONN.
           */

          conn->lc_conn.s_flags &= ~(_SF_CONNECTED | _SF_CLOSED);
          conn->lc_state = LOCAL_STATE_DISCONNECTED;

          /* Did we receive any data? */

          if (*readlen <= 0)
            {
              /* No.. return the ECONNRESET error now.  Otherwise,
               * process the received data and return ENOTCONN the
               * next time that psock_recvfrom() is called.
               */

              return ret;
            }
        }
      else
        {
          nerr("ERROR: Failed to read packet: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: local_recvctl
 *
 * Description:
 *   Handle the socket message conntrol field
 *
 * Input Parameters:
 *   conn     Local connection instance
 *   msg      Message to send
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_SCM
static void local_recvctl(FAR struct local_conn_s *conn,
                          FAR struct msghdr *msg)
{
  FAR struct local_conn_s *peer;
  struct cmsghdr *cmsg;
  int count;
  int *fds;
  int i;

  net_lock();

  cmsg  = CMSG_FIRSTHDR(msg);
  count = (cmsg->cmsg_len - sizeof(struct cmsghdr)) / sizeof(int);
  cmsg->cmsg_len = 0;

  if (count == 0)
    {
      goto out;
    }

  if (conn->lc_peer == NULL)
    {
      peer = local_peerconn(conn);
      if (peer == NULL)
        {
          goto out;
        }
    }
  else
    {
      peer = conn;
    }

  if (peer->lc_cfpcount == 0)
    {
      goto out;
    }

  fds = (int *)CMSG_DATA(cmsg);

  count = count > peer->lc_cfpcount ?
                  peer->lc_cfpcount : count;
  for (i = 0; i < count; i++)
    {
      fds[i] = file_dup(peer->lc_cfps[i], 0);
      file_close(peer->lc_cfps[i]);
      kmm_free(peer->lc_cfps[i]);
      peer->lc_cfps[i] = NULL;
      peer->lc_cfpcount--;
      if (fds[i] < 0)
        {
          i++;
          break;
        }
    }

  if (i > 0)
    {
      if (peer->lc_cfpcount)
        {
          memmove(peer->lc_cfps[0], peer->lc_cfps[i],
                  sizeof(FAR void *) * peer->lc_cfpcount);
        }

      cmsg->cmsg_len   = CMSG_LEN(sizeof(int) * i);
      cmsg->cmsg_level = SOL_SOCKET;
      cmsg->cmsg_type  = SCM_RIGHTS;
    }

out:
  net_unlock();
}
#endif /* CONFIG_NET_LOCAL_SCM */

/****************************************************************************
 * Name: psock_stream_recvfrom
 *
 * Description:
 *   psock_stream_recvfrom() receives messages from a local stream socket.
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
 *   recv() will return 0.  Otherwise, on errors, -1 is returned, and errno
 *   is set appropriately (see receive from for the complete list).
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
static inline ssize_t
psock_stream_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                      int flags, FAR struct sockaddr *from,
                      FAR socklen_t *fromlen)
{
  FAR struct local_conn_s *conn = (FAR struct local_conn_s *)psock->s_conn;
  size_t readlen = len;
  int ret;

  /* Verify that this is a connected peer socket */

  if (conn->lc_state != LOCAL_STATE_CONNECTED ||
      conn->lc_infile.f_inode == NULL)
    {
      if (conn->lc_state == LOCAL_STATE_CONNECTING)
        {
          return -EAGAIN;
        }

      nerr("ERROR: not connected\n");
      return -ENOTCONN;
    }

  /* Read the packet */

  ret = psock_fifo_read(psock, buf, &readlen, true);
  if (ret < 0)
    {
      return ret;
    }

  /* Return the address family */

  if (from)
    {
      ret = local_getaddr(conn, from, fromlen);
      if (ret < 0)
        {
          return ret;
        }
    }

  return readlen;
}
#endif /* CONFIG_NET_LOCAL_STREAM */

/****************************************************************************
 * Name: psock_dgram_recvfrom
 *
 * Description:
 *   psock_dgram_recvfrom() receives messages from a local datagram socket.
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
 *   On success, returns the number of characters received.  Otherwise, on
 *   errors, -1 is returned, and errno is set appropriately (see receive
 *   from for the complete list).
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_DGRAM
static inline ssize_t
psock_dgram_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                     int flags, FAR struct sockaddr *from,
                     FAR socklen_t *fromlen)
{
  FAR struct local_conn_s *conn = (FAR struct local_conn_s *)psock->s_conn;
  uint16_t pktlen;
  size_t readlen;
  int ret;

  /* We keep packet sizes in a uint16_t, so there is a upper limit to the
   * 'len' that can be supported.
   */

  DEBUGASSERT(len <= UINT16_MAX);

  /* Verify that this is a bound, un-connected peer socket */

  if (conn->lc_state != LOCAL_STATE_BOUND)
    {
      /* Either not bound to address or it is connected */

      nerr("ERROR: Connected or not bound\n");
      return -EISCONN;
    }

  /* The incoming FIFO should not be open */

  DEBUGASSERT(conn->lc_infile.f_inode == NULL);

  /* Make sure that half duplex FIFO has been created */

  ret = local_create_halfduplex(conn, conn->lc_path);
  if (ret < 0)
    {
      nerr("ERROR: Failed to create FIFO for %s: %d\n",
           conn->lc_path, ret);
      return ret;
    }

  /* Open the receiving side of the transfer */

  ret = local_open_receiver(conn, _SS_ISNONBLOCK(conn->lc_conn.s_flags) ||
                            (flags & MSG_DONTWAIT) != 0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to open FIFO for %s: %d\n",
           conn->lc_path, ret);
      goto errout_with_halfduplex;
    }

  /* Sync to the start of the next packet in the stream and get the size of
   * the next packet.
   */

  ret = local_sync(&conn->lc_infile);
  if (ret < 0)
    {
      nerr("ERROR: Failed to get packet length: %d\n", ret);
      goto errout_with_infd;
    }
  else if (ret > UINT16_MAX)
    {
      nerr("ERROR: Packet is too big: %d\n", ret);
      goto errout_with_infd;
    }

  pktlen = ret;

  /* Read the packet */

  readlen = MIN(pktlen, len);
  ret     = psock_fifo_read(psock, buf, &readlen, false);
  if (ret < 0)
    {
      goto errout_with_infd;
    }

  /* If there are unread bytes remaining in the packet, flush the remainder
   * of the packet to the bit bucket.
   */

  DEBUGASSERT(readlen <= pktlen);
  if (readlen < pktlen)
    {
      uint8_t bitbucket[32];
      uint16_t remaining;
      size_t tmplen;

      remaining = pktlen - readlen;
      do
        {
          /* Read 32 bytes into the bit bucket */

          tmplen = MIN(remaining, 32);
          ret     = psock_fifo_read(psock, bitbucket, &tmplen, false);
          if (ret < 0)
            {
              goto errout_with_infd;
            }

          /* Adjust the number of bytes remaining to be read from the
           * packet
           */

          DEBUGASSERT(tmplen <= remaining);
          remaining -= tmplen;
          readlen += tmplen;
        }
      while (remaining > 0);
    }

  /* Now we can close the read-only file descriptor */

  file_close(&conn->lc_infile);
  conn->lc_infile.f_inode = NULL;

  /* Release our reference to the half duplex FIFO */

  local_release_halfduplex(conn);

  /* Return the address family */

  if (from)
    {
      ret = local_getaddr(conn, from, fromlen);
      if (ret < 0)
        {
          return ret;
        }
    }

  return readlen;

errout_with_infd:

  /* Close the read-only file descriptor */

  file_close(&conn->lc_infile);
  conn->lc_infile.f_inode = NULL;

errout_with_halfduplex:

  /* Release our reference to the half duplex FIFO */

  local_release_halfduplex(conn);
  return ret;
}
#endif /* CONFIG_NET_LOCAL_STREAM */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_recvmsg
 *
 * Description:
 *   recvmsg() receives messages from a local socket and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive the message
 *   flags    Receive flags (ignored for now)
 *
 * Returned Value:
 *   On success, returns the number of characters received. If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recvmsg() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvmsg() for the list of appropriate error values).
 *
 ****************************************************************************/

ssize_t local_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                      int flags)
{
  FAR socklen_t *fromlen = &msg->msg_namelen;
  FAR struct sockaddr *from = msg->msg_name;
  FAR void *buf = msg->msg_iov->iov_base;
  size_t len = msg->msg_iov->iov_len;

  DEBUGASSERT(psock && psock->s_conn && buf);

  /* Check for a stream socket */

#ifdef CONFIG_NET_LOCAL_STREAM
  if (psock->s_type == SOCK_STREAM)
    {
      len = psock_stream_recvfrom(psock, buf, len, flags, from, fromlen);
    }
  else
#endif

#ifdef CONFIG_NET_LOCAL_DGRAM
  if (psock->s_type == SOCK_DGRAM)
    {
      len = psock_dgram_recvfrom(psock, buf, len, flags, from, fromlen);
    }
  else
#endif
    {
      DEBUGPANIC();
      nerr("ERROR: Unrecognized socket type: %" PRIu8 "\n", psock->s_type);
      len = -EINVAL;
    }

#ifdef CONFIG_NET_LOCAL_SCM
  /* Receive the control message */

  if (len >= 0 && msg->msg_control &&
      msg->msg_controllen > sizeof(struct cmsghdr))
    {
      local_recvctl(psock->s_conn, msg);
    }
#endif /* CONFIG_NET_LOCAL_SCM */

  return len;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
