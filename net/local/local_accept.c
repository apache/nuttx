/****************************************************************************
 * net/local/local_accept.c
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
#include <errno.h>
#include <assert.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "local/local.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_local_accept
 *
 * Description:
 *   This function implements accept() for Unix domain sockets.  See the
 *   description of accept() for further information.
 *
 * Parameters:
 *   psock    The listening Unix domain socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *   newconn  The new, accepted  Unix domain connection structure
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *   See the description of accept of the possible errno values in the
 *   description of accept().
 *
 * Assumptions:
 *   Network is NOT locked.
 *
 ****************************************************************************/

int psock_local_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                       FAR socklen_t *addrlen, FAR void **newconn)

{
  FAR struct local_conn_s *server;
  FAR struct local_conn_s *client;
  FAR struct local_conn_s *conn;
  int ret;

  /* Some sanity checks */

  DEBUGASSERT(psock && psock->s_conn);
  server = (FAR struct local_conn_s *)psock->s_conn;

  if (server->lc_family != SOCK_STREAM ||
      server->lc_state != LOCAL_STATE_LISTENING ||
      server->lc_type != LOCAL_TYPE_PATHNAME)
    {
      return -EOPNOTSUPP;
    }

  /* Loop as necessary if we have to wait for a connection */

  for (;;)
    {
      /* Are there pending connections.  Remove the client from the
       * head of the waiting list.
       */

      client = (FAR struct local_conn_s *)
        dq_remfirst(&server->u.server.lc_waiters);

      if (client)
        {
          /* Decrement the number of pending clients */

          DEBUGASSERT(server->u.server.lc_pending > 0);
          server->u.server.lc_pending--;

          /* Create a new connection structure for the server side of the
           * connection.
           */

          conn = local_alloc();
          if (!conn)
            {
              ndbg("ERROR:  Failed to allocate new connection structure\n");
              ret = -ENOMEM;
            }
          else
            {
              /* Initialize the new connection structure */

              conn->lc_crefs  = 1;
              conn->lc_family = SOCK_STREAM;
              conn->lc_type   = LOCAL_TYPE_PATHNAME;
              conn->lc_state   = LOCAL_STATE_CONNECTED;

              strncpy(conn->lc_path, client->lc_path, UNIX_PATH_MAX-1);
              conn->lc_path[UNIX_PATH_MAX-1] = '\0';

              /* Open the server-side write-only FIFO.  This should not
               * block.
               */

              ret = local_open_server_tx(conn);
              if (ret < 0)
                {
                   ndbg("ERROR: Failed to open write-only FIFOs for %s: %d\n",
                        conn->lc_path, ret);
                }
            }

          /* Do we have a connection?  Is the write-side FIFO opened? */

          if (ret == OK)
            {
              DEBUGASSERT(conn->lc_outfd >= 0);

              /* Open the server-side read-only FIFO.  This should not
               * block because the client side has already opening it
               * for writing.
               */

              ret = local_open_server_rx(conn);
              if (ret < 0)
                {
                   ndbg("ERROR: Failed to open read-only FIFOs for %s: %d\n",
                        conn->lc_path, ret);
                }
            }

          /* Do we have a connection?  Are the FIFOs opened? */

          if (ret == OK)
            {
              DEBUGASSERT(conn->lc_infd >= 0);

              /* Return the address family */

              if (addr)
                {
                  FAR struct sockaddr_un *unaddr;
                  int totlen;
                  int pathlen;

                  /* If an address is provided, then the length must also be
                   * provided.
                   */

                  DEBUGASSERT(addrlen);

                  /* Get the length of the path (minus the NUL terminator)
                   * and the length of the whole client address.
                   */

                  pathlen = strnlen(client->lc_path, UNIX_PATH_MAX-1);
                  totlen  = sizeof(sa_family_t) + pathlen + 1;

                  /* If the length of the whole client address is larger
                   * than the buffer provided by the caller, then truncate
                   * the address to fit.
                   */

                  if (totlen > *addrlen)
                    {
                      pathlen    -= (totlen - *addrlen);
                      totlen      = *addrlen;
                    }

                  /* Copy the Unix domain address */

                  unaddr = (FAR struct sockaddr_un *)addr;
                  unaddr->sun_family = AF_LOCAL;
                  memcpy(unaddr->sun_path, client->lc_path, pathlen);
                  unaddr->sun_path[pathlen] = '\0';

                  /* Return the Unix domain address size */

                  *addrlen = totlen;
                }

              /* Return the client connection structure */

              *newconn = (FAR void *)conn;
            }

          /* Signal the client with the result of the connection */

          client->u.client.lc_result = ret;
          sem_post(&client->lc_waitsem);
          return ret;
        }

      /* No.. then there should be no pending connections */

      DEBUGASSERT(server->u.server.lc_pending == 0);

      /* Was the socket opened non-blocking? */

      if (_SS_ISNONBLOCK(psock->s_flags))
        {
          /* Yes.. return EAGAIN */

          return -EAGAIN;
        }

      /* Otherwise, listen for a connection and try again. */

      ret = local_listen(server, server->u.server.lc_backlog);
      if (ret < 0)
        {
          return ret;
        }
    }
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
