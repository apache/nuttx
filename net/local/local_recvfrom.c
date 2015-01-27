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
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: psock_fifo_read
 *
 * Description:
 *   A thin layer aroudn local_fifo_read that handles socket-related loss-of-
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_recvfrom
 *
 * Description:
 *   recvfrom() receives messages from a local socket, and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recv() will return 0.  Otherwise, on errors, -1 is returned, and errno
 *   is set appropriately (see receive from for the complete list).
 *
 ****************************************************************************/

ssize_t psock_local_recvfrom(FAR struct socket *psock, FAR void *buf,
                             size_t len, int flags, FAR struct sockaddr *from,
                             FAR socklen_t *fromlen)
{
  FAR struct local_conn_s *conn;
  size_t readlen;
  int ret;

  DEBUGASSERT(psock && psock->s_conn && buf);
  conn = (FAR struct local_conn_s *)psock->s_conn;

  /* Verify that this is a connected peer socket and that it has opened the
   * incoming FIFO for read-only access.
   */

  if (conn->lc_type != LOCAL_STATE_CONNECTED ||
      conn->lc_infd < 0)
    {
      ndbg("ERROR: not connected\n");
      return -ENOTCONN;
    }

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
      else if (ret > 0xffff)
        {
          ndbg("ERROR: Packet is too big: %d\n", ret);
          return -E2BIG;
        }

      conn->u.peer.lc_remaining = (uint16_t)ret;
    }

  /* Read the packet */
  /* REVISIT:  Does this make sense if the socket is SOCK_DGRAM? */

  readlen = MIN(conn->u.peer.lc_remaining, len);
  ret     = psock_fifo_read(psock, buf, &readlen);
  if (ret < 0)
    {
      return ret;
    }

  /* Adjust the number of bytes remaining to be read from the packet */

  DEBUGASSERT(readlen <= conn->u.peer.lc_remaining);
  conn->u.peer.lc_remaining -= readlen;

  /* If this is a SOCK_STREAM socket and there are unread bytes remaining
   * in the packet, we will get those bytes the next time recv is called.
   * What if this is a SOCK_DRAM?  REVISIT: Here we flush the remainder of
   * the packet to the bit bucket.
   */

  if (psock->s_type == SOCK_DGRAM && conn->u.peer.lc_remaining > 0)
    {
      uint8_t bitbucket[32];
      size_t tmplen;

      do
        {
          /* Read 32 bytes into the bit bucket */

          tmplen = MIN(conn->u.peer.lc_remaining, 32);
          ret    = psock_fifo_read(psock, bitbucket, &tmplen);
          if (ret < 0)
            {
              return ret;
            }

          /* Adjust the number of bytes remaining to be read from the packet */

          DEBUGASSERT(tmplen <= conn->u.peer.lc_remaining);
          conn->u.peer.lc_remaining -= tmplen;
        }
      while (conn->u.peer.lc_remaining > 0);
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

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
