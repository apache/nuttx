/****************************************************************************
 * net/local/local_bind.c
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

#include <sys/socket.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "local/local.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_bind
 *
 * Description:
 *   This function implements the low-level parts of the standard local
 *   bind()operation.
 *
 ****************************************************************************/

int psock_local_bind(FAR struct socket *psock,
                     FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR struct local_conn_s *conn;
  FAR const struct sockaddr_un *unaddr =
    (FAR const struct sockaddr_un *)addr;

  DEBUGASSERT(unaddr->sun_family == AF_LOCAL);

  if (addrlen <= sizeof(sa_family_t) + 1)
    {
      return -EINVAL;
    }

  conn = psock->s_conn;

  /* Save the address family */

  conn->lc_proto = psock->s_type;
  conn->lc_instance_id = -1;

  /* Now determine the type of the Unix domain socket by comparing the size
   * of the address description.
   */

  if (unaddr->sun_path[0] == '\0')
    {
      /* Zero-length sun_path... This is an abstract Unix domain socket */

      conn->lc_type = LOCAL_TYPE_ABSTRACT;

      /* Copy the path into the connection structure */

      strlcpy(conn->lc_path, &unaddr->sun_path[1], sizeof(conn->lc_path));
    }
  else
    {
      /* Check if local address is already in use */

      if (local_active(conn, unaddr) != NULL)
        {
          return -EADDRINUSE;
        }

      /* This is an normal, pathname Unix domain socket */

      conn->lc_type = LOCAL_TYPE_PATHNAME;

      /* Copy the path into the connection structure */

      strlcpy(conn->lc_path, unaddr->sun_path, sizeof(conn->lc_path));
    }

  conn->lc_state = LOCAL_STATE_BOUND;

#ifdef CONFIG_NET_LOCAL_DGRAM
  int ret;

  if (psock->s_type == SOCK_DGRAM)
    {
      /* Make sure that half duplex FIFO has been created */

      ret = local_create_halfduplex(conn, conn->lc_path);
      if (ret < 0)
        {
          nerr("ERROR: Failed to create FIFO for %s: %d\n",
               conn->lc_path, ret);
          return ret;
        }

      /* Open the receiving side of the transfer */

      ret = local_open_receiver(conn, _SS_ISNONBLOCK(conn->lc_conn.s_flags));
      if (ret < 0)
        {
          nerr("ERROR: Failed to open FIFO for %s: %d\n",
               conn->lc_path, ret);

          /* Release our reference to the half duplex FIFO */

          local_release_halfduplex(conn);
          return ret;
        }
    }
#endif

  nxsem_init(&conn->lc_sendsem, 0, 0);

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
