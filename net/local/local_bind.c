/****************************************************************************
 * net/local/local_bind.c
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

          strncpy(conn->lc_path, unaddr->sun_path, UNIX_PATH_MAX - 1);
          conn->lc_path[UNIX_PATH_MAX - 1] = '\0';
          conn->lc_instance_id = -1;
        }
    }

  conn->lc_state = LOCAL_STATE_BOUND;
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
