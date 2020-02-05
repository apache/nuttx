/****************************************************************************
 * net/local/local_sendto.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_LOCAL_DGRAM)

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_local_sendto
 *
 * Description:
 *   This function implements the Unix domain-specific logic of the
 *   standard sendto() socket operation.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 *   NOTE: All input parameters were verified by sendto() before this
 *   function was called.
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is returned.  See the description in
 *   net/socket/sendto.c for the list of appropriate return value.
 *
 ****************************************************************************/

ssize_t psock_local_sendto(FAR struct socket *psock, FAR const void *buf,
                           size_t len, int flags, FAR const struct sockaddr *to,
                           socklen_t tolen)
{
  FAR struct local_conn_s *conn = (FAR struct local_conn_s *)psock->s_conn;
  FAR struct sockaddr_un *unaddr = (FAR struct sockaddr_un *)to;
  ssize_t nsent;
  int ret;

  /* We keep packet sizes in a uint16_t, so there is a upper limit to the
   * 'len' that can be supported.
   */

  DEBUGASSERT(buf && len <= UINT16_MAX);

  /* Verify that this is not a connected peer socket.  It need not be
   * bound, however.  If unbound, recvfrom will see this as a nameless
   * connection.
   */

  if (conn->lc_state != LOCAL_STATE_UNBOUND &&
      conn->lc_state != LOCAL_STATE_BOUND)
    {
      /* Either not bound to address or it is connected */

      nerr("ERROR: Connected state\n");
      return -EISCONN;
    }

  /* The outgoing FIFO should not be open */

  DEBUGASSERT(conn->lc_outfile.f_inode == 0);

  /* At present, only standard pathname type address are support */

  if (tolen < sizeof(sa_family_t) + 2)
    {
      /* EFAULT - An invalid user space address was specified for a parameter */

      return -EFAULT;
    }

  /* Make sure that half duplex FIFO has been created.
   * REVISIT:  Or should be just make sure that it already exists?
   */

  ret = local_create_halfduplex(conn, unaddr->sun_path);
  if (ret < 0)
    {
      nerr("ERROR: Failed to create FIFO for %s: %d\n",
           conn->lc_path, ret);
      return ret;
    }

  /* Open the sending side of the transfer */

  ret = local_open_sender(conn, unaddr->sun_path,
                          _SS_ISNONBLOCK(psock->s_flags) ||
                          (flags & MSG_DONTWAIT) != 0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to open FIFO for %s: %d\n",
           unaddr->sun_path, ret);

      nsent = ret;
      goto errout_with_halfduplex;
    }

  /* Send the packet */

  nsent = local_send_packet(&conn->lc_outfile, buf, len);
  if (nsent < 0)
    {
      nerr("ERROR: Failed to send the packet: %d\n", ret);
    }
  else
    {
      /* local_send_packet returns 0 if all 'len' bytes were sent */

      nsent = len;
    }

  /* Now we can close the write-only socket descriptor */

  file_close(&conn->lc_outfile);
  conn->lc_outfile.f_inode = NULL;

errout_with_halfduplex:

  /* Release our reference to the half duplex FIFO */

  local_release_halfduplex(conn);
  return nsent;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL_DGRAM */
