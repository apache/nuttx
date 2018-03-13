/****************************************************************************
 * net/local/local_send.c
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

#include <sys/types.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "local/local.h"

#ifdef CONFIG_NET_LOCAL_STREAM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_local_send
 *
 * Description:
 *   Send a local packet as a stream.
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags (ignored for now)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately (see send() for the
 *   list of errno numbers).
 *
 ****************************************************************************/

ssize_t psock_local_send(FAR struct socket *psock, FAR const void *buf,
                         size_t len, int flags)
{
  FAR struct local_conn_s *peer;
  int ret;

  DEBUGASSERT(psock && psock->s_conn && buf);
  peer = (FAR struct local_conn_s *)psock->s_conn;

  /* Verify that this is a connected peer socket and that it has opened the
   * outgoing FIFO for write-only access.
   */

  if (peer->lc_state != LOCAL_STATE_CONNECTED ||
      peer->lc_outfile.f_inode == NULL)
    {
      nerr("ERROR: not connected\n");
      return -ENOTCONN;
    }

  /* Send the packet */

  ret = local_send_packet(&peer->lc_outfile, (FAR uint8_t *)buf, len);

  /* If the send was successful, then the full packet will have been sent */

  return ret < 0 ? ret : len;
}

#endif /* CONFIG_NET_LOCAL_STREAM */
