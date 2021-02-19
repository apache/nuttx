/****************************************************************************
 * net/local/local_send.c
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
