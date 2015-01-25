/****************************************************************************
 * net/local/local_conn.c
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

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "local/local.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_initialize
 *
 * Description:
 *   Initialize the local, Unix domain connection structures.  Called once
 *   and only from the common network initialization logic.
 *
 ****************************************************************************/

/* void local_initialize(void) */

/****************************************************************************
 * Name: local_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized Unix domain socket connection structure.
 *   This is normally something done by the implementation of the socket()
 *   API
 *
 ****************************************************************************/

FAR struct local_conn_s *local_alloc(void)
{
  FAR struct local_conn_s *conn =
    (FAR struct local_conn_s *)kmm_zalloc(sizeof(struct local_conn_s));

  if (conn)
    {
      /* Make sure that the pipe is marked closed */

      conn->lc_fd = -1;
    }

  return conn;
}

/****************************************************************************
 * Name: local_free()
 *
 * Description:
 *   Free a packet Unix domain connection structure that is no longer in use.
 *   This should be done by the implementation of close().
 *
 ****************************************************************************/

void local_free(FAR struct local_conn_s *conn)
{
  DEBUGASSERT(conn != NULL);
  free(conn);
}

/****************************************************************************
 * Name: local_bind
 *
 * Description:
 *   This function implements the low-level parts of the standard local
 *   bind()operation.
 *
 ****************************************************************************/

int local_bind(FAR struct local_conn_s *conn,
               FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR const struct sockaddr_un *unaddr =
    (FAR const struct sockaddr_un *)addr;
  int namelen;

  DEBUGASSERT(conn && unaddr && unaddr->sun_family == AF_LOCAL &&
              addrlen >= sizeof(sa_family_t));

  if (addrlen == sizeof(sa_family_t))
    {
      /* No sun_path... This is an un-named Unix domain socket */

      conn->lc_type = LOCAL_TYPE_UNNAMED;
    }
  else
    {
      namelen = strnlen(unaddr->sun_path, UNIX_PATH_MAX-1);
      if (namelen <= 0)
        {
          /* Zero-length sun_path... This is an abstract Unix domain socket */

          conn->lc_type    = LOCAL_TYPE_ABSTRACT;
          conn->lc_path[0] = '\0';
        }
      else
        {
          /* This is an normal, pathname Unix domain socket */

          conn->lc_type    = LOCAL_TYPE_PATHNAME;

          /* Copy the path into the connection structure */

          (void)strncpy(conn->lc_path, unaddr->sun_path, UNIX_PATH_MAX-1);
          conn->lc_path[UNIX_PATH_MAX-1] = '\0';
        }
    }

  return OK;
}


#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
