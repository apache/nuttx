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
  int namelen;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL &&
              unaddr != NULL && unaddr->sun_family == AF_LOCAL &&
              addrlen >= sizeof(sa_family_t));

  conn = (FAR struct local_conn_s *)psock->s_conn;

  /* Save the address family */

  conn->lc_proto = psock->s_type;

  /* Now determine the type of the Unix domain socket by comparing the size
   * of the address description.
   */

  if (addrlen == sizeof(sa_family_t))
    {
      /* No sun_path... This is an un-named Unix domain socket */

      conn->lc_type = LOCAL_TYPE_UNNAMED;
    }
  else
    {
      namelen = strnlen(unaddr->sun_path, UNIX_PATH_MAX - 1);
      if (namelen <= 0)
        {
          /* Zero-length sun_path... This is an abstract Unix domain socket */

          conn->lc_type    = LOCAL_TYPE_ABSTRACT;
          conn->lc_path[0] = '\0';
        }
      else
        {
          /* This is an normal, pathname Unix domain socket */

          conn->lc_type = LOCAL_TYPE_PATHNAME;

          /* Copy the path into the connection structure */

          strlcpy(conn->lc_path, unaddr->sun_path, sizeof(conn->lc_path));
          conn->lc_instance_id = -1;
        }
    }

  conn->lc_state = LOCAL_STATE_BOUND;
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
