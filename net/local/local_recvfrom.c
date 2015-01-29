/****************************************************************************
 * net/local/local_recvfrom.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_LOCAL)

#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

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
                           FAR size_t *readlen)
{
  FAR struct local_conn_s *conn = (FAR struct local_conn_s *)psock->s_conn;
  int ret;

  ret = local_fifo_read(conn->lc_infd, buf, readlen);
  if (ret < 0)
    {
      /* -ECONNRESET is a special case.  We may or not have received
       * data, then the peer closed the connection.
       */

      if (ret == -ECONNRESET)
        {
          ndbg("ERROR: Lost connection: %d\n", ret);

          /* Report an ungraceful loss of connection.  This should
           * eventually be reported as ENOTCONN.
           */

          psock->s_flags &= ~(_SF_CONNECTED |_SF_CLOSED);
          conn->lc_state  = LOCAL_STATE_DISCONNECTED;

          /* Did we receive any data? */

          if (*readlen <= 0)
            {
              /* No.. return the ECONNRESET error now.  Otherwise,
               * process the received data and return ENOTCONN the
               * next time that psock_recvfrom() is calle.
               */

              return ret;
            }
        }
      else
        {
          ndbg("ERROR: Failed to read packet: %d\n", ret);
          return ret;
        }
    }
 
  return OK;
}

/****************************************************************************
 * Function: psock_stream_recvfrom
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

static inline ssize_t
psock_stream_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                      int flags, FAR struct sockaddr *from,
                      FAR socklen_t *fromlen)
{
  FAR struct local_conn_s *conn = (FAR struct local_conn_s *)psock->s_conn;
  size_t readlen;
  int ret;

  /* Verify that this is a connected peer socket */

  if (conn->lc_state != LOCAL_STATE_CONNECTED)
    {
      ndbg("ERROR: not connected\n");
      return -ENOTCONN;
    }

  /* The incoming FIFO should be open */

  DEBUGASSERT(conn->lc_infd >= 0);

  /* Are there still bytes in the FIFO from the last packet? */

  if (conn->u.peer.lc_remaining == 0)
    {
      /* No.. Sync to the start of the next packet in the stream and get
       * the size of the next packet.
       */

      ret = local_sync(conn->lc_infd);
      if (ret < 0)
        {
          ndbg("ERROR: Failed to get packet length: %d\n", ret);
          return ret;
        }
      else if (ret > UINT16_MAX)
        {
          ndbg("ERROR: Packet is too big: %d\n", ret);
          return -E2BIG;
        }

      conn->u.peer.lc_remaining = (uint16_t)ret;
    }

  /* Read the packet */

  readlen = MIN(conn->u.peer.lc_remaining, len);
  ret     = psock_fifo_read(psock, buf, &readlen);
  if (ret < 0)
    {
      return ret;
    }

  /* Adjust the number of bytes remaining to be read from the packet */

  DEBUGASSERT(readlen <= conn->u.peer.lc_remaining);
  conn->u.peer.lc_remaining -= readlen;

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

/****************************************************************************
 * Function: psock_dgram_recvfrom
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

      ndbg("ERROR: Connected or not bound\n");
      return -EISCONN;
    }

  /* The incoming FIFO should not be open */

  DEBUGASSERT(conn->lc_infd < 0);

  /* Make sure that half duplex FIFO has been created */

  ret = local_create_halfduplex(conn, conn->lc_path);
  if (ret < 0)
    {
      ndbg("ERROR: Failed to create FIFO for %s: %d\n",
           conn->lc_path, ret);
      return ret;
    }

  /* Open the receiving side of the transfer */

  ret = local_open_receiver(conn);
  if (ret < 0)
    {
      ndbg("ERROR: Failed to open FIFO for %s: %d\n",
           conn->lc_path, ret);
      return ret;
    }

  /* Sync to the start of the next packet in the stream and get the size of
   * the next packet.
   */

  ret = local_sync(conn->lc_infd);
  if (ret < 0)
    {
      ndbg("ERROR: Failed to get packet length: %d\n", ret);
      goto errout_with_infd;
    }
  else if (ret > UINT16_MAX)
    {
      ndbg("ERROR: Packet is too big: %d\n", ret);
      goto errout_with_infd;
    }

  pktlen = ret;

  /* Read the packet */

  readlen = MIN(pktlen, len);
  ret     = psock_fifo_read(psock, buf, &readlen);
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

          readlen = MIN(remaining, 32);
          ret     = psock_fifo_read(psock, bitbucket, &tmplen);
          if (ret < 0)
            {
              goto errout_with_infd;
            }

          /* Adjust the number of bytes remaining to be read from the packet */

          DEBUGASSERT(tmplen <= remain);
          remaining -= tmplen;
        }
      while (remaining > 0);
    }

  /* Now we can close the read-only socket descriptor */

  close(conn->lc_infd);
  conn->lc_infd = -1;

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
  close(conn->lc_infd);
  conn->lc_infd = -1;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_local_recvfrom
 *
 * Description:
 *   psock_local_recvfrom() receives messages from a local socket and may be
 *   used to receive data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
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

ssize_t psock_local_recvfrom(FAR struct socket *psock, FAR void *buf,
                             size_t len, int flags, FAR struct sockaddr *from,
                             FAR socklen_t *fromlen)
{
  DEBUGASSERT(psock && psock->s_conn && buf);

  /* Check for a stream socket */

  if (psock->s_type == SOCK_STREAM)
    {
      return psock_stream_recvfrom(psock, buf, len, flags, from, fromlen);
    }
  else if (psock->s_type == SOCK_DGRAM)
    {
      return psock_dgram_recvfrom(psock, buf, len, flags, from, fromlen);
    }
  else
    {
      DEBUGPANIC();
      ndbg("ERROR: Unrecognized socket type: %s\n", psock->s_type);
      return -EINVAL;
    }
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
