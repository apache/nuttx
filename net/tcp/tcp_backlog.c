/****************************************************************************
 * net/tcp/tcp_backlog.c
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

#include <nuttx/net/netconfig.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP) && defined(CONFIG_NET_TCPBACKLOG)

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/net/net.h>

#include "devif/devif.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_backlogcreate
 *
 * Description:
 *   Called from the listen() logic to setup the backlog as specified in the
 *   the listen arguments.
 *
 * Assumptions:
 *   Called from normal task logic.  The network may or may not be locked.
 *
 ****************************************************************************/

int tcp_backlogcreate(FAR struct tcp_conn_s *conn, int nblg)
{
  FAR struct tcp_backlog_s     *bls = NULL;
  FAR struct tcp_blcontainer_s *blc;
  int size;
  int offset;
  int i;

  ninfo("conn=%p nblg=%d\n", conn, nblg);

#ifdef CONFIG_DEBUG_FEATURES
  if (!conn)
    {
      return -EINVAL;
    }
#endif

  /* Then allocate the backlog as requested */

  if (nblg > 0)
    {
      /* nblog value must less than SOMAXCONN */

      if (nblg > SOMAXCONN)
        {
          nblg = SOMAXCONN;
        }

      /* Align the list of backlog structures to 32-bit boundaries.  This
       * may be excessive on 24-16-bit address machines; and insufficient
       * on 64-bit address machines -- REVISIT
       */

      offset = (sizeof(struct tcp_backlog_s) + 3) & ~3;

      /* Then determine the full size of the allocation include the
       * tcp_backlog_s, a pre-allocated array of struct tcp_blcontainer_s
       * and alignment padding
       */

      size = offset + nblg * sizeof(struct tcp_blcontainer_s);

      /* Then allocate that much */

      bls = (FAR struct tcp_backlog_s *)kmm_zalloc(size);
      if (!bls)
        {
          nerr("ERROR: Failed to allocate backlog\n");
          return -ENOMEM;
        }

      /* Then add all of the pre-allocated containers to the free list */

      blc = (FAR struct tcp_blcontainer_s *)(((FAR uint8_t *)bls) + offset);
      for (i = 0; i < nblg; i++)
        {
          sq_addfirst(&blc->bc_node, &bls->bl_free);
          blc++;
        }
    }

  /* Destroy any existing backlog (shouldn't be any) */

  net_lock();
  tcp_backlogdestroy(conn);

  /* Now install the backlog tear-off in the connection.  NOTE that bls may
   * actually be NULL if nblg is <= 0;  In that case, we are disabling
   * backlog support.  Since the network is locked, destroying the old
   * backlog and replace it with the new is an atomic operation
   */

  conn->backlog = bls;
  net_unlock();
  return OK;
}

/****************************************************************************
 * Name: tcp_backlogdestroy
 *
 * Description:
 *   (1) Called from tcp_free() whenever a connection is freed.
 *   (2) Called from tcp_backlogcreate() to destroy any old backlog
 *
 *   NOTE: This function may re-enter tcp_free when a connection that
 *   is freed that has pending connections.
 *
 * Assumptions:
 *   Called from network socket logic with the network locked
 *
 ****************************************************************************/

int tcp_backlogdestroy(FAR struct tcp_conn_s *conn)
{
  FAR struct tcp_backlog_s     *blg;
  FAR struct tcp_blcontainer_s *blc;
  FAR struct tcp_conn_s        *blconn;

  ninfo("conn=%p\n", conn);

#ifdef CONFIG_DEBUG_FEATURES
  if (!conn)
    {
      return -EINVAL;
    }
#endif

  /* Make sure that the connection has a backlog to be destroyed */

  if (conn->backlog)
    {
      /* Remove the backlog structure reference from the connection */

      blg           = conn->backlog;
      conn->backlog = NULL;

      /* Handle any pending connections in the backlog */

      while ((blc = (FAR struct tcp_blcontainer_s *)
                    sq_remfirst(&blg->bl_pending)) != NULL)
        {
          blconn = blc->bc_conn;
          if (blconn)
            {
              /* REVISIT
               * -- such connections really need to be gracefully closed
               */

              blconn->blparent = NULL;
              blconn->backlog  = NULL;
              blconn->crefs    = 0;
              tcp_free(blconn);
            }
        }

      /* Then free the entire backlog structure */

      kmm_free(blg);
    }

  return OK;
}

/****************************************************************************
 * Name: tcp_backlogadd
 *
 * Description:
 *  Called tcp_listen when a new connection is made with a listener socket
 *  but when there is no accept() in place to receive the connection.  This
 *  function adds the new connection to the backlog.
 *
 * Assumptions:
 *   Called from network socket logic with the network locked
 *
 ****************************************************************************/

int tcp_backlogadd(FAR struct tcp_conn_s *conn,
                   FAR struct tcp_conn_s *blconn)
{
  FAR struct tcp_backlog_s     *bls;
  FAR struct tcp_blcontainer_s *blc;
  int ret = -EINVAL;

  ninfo("conn=%p blconn=%p\n", conn, blconn);

#ifdef CONFIG_DEBUG_FEATURES
  if (!conn)
    {
      return -EINVAL;
    }
#endif

  bls = conn->backlog;
  if (bls && blconn)
    {
      /* Get a container for the connection from the free list */

      blc = (FAR struct tcp_blcontainer_s *)sq_remfirst(&bls->bl_free);
      if (!blc)
        {
          nerr("ERROR: There are no free containers for TCP BACKLOG!\n");
          ret = -ENOMEM;
        }
      else
        {
          /* Save the connection reference in the container and put the
           * container at the end of the pending connection list (FIFO).
           */

          blc->bc_conn = blconn;
          sq_addlast(&blc->bc_node, &bls->bl_pending);
          ret = OK;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: tcp_backlogremove
 *
 * Description:
 *  Called from poll().  Before waiting for a new connection, poll will
 *  call this API to see if there are pending connections in the backlog.
 *
 * Assumptions:
 *   Called from network socket logic with the network locked
 *
 ****************************************************************************/

bool tcp_backlogavailable(FAR struct tcp_conn_s *conn)
{
  return (conn && conn->backlog && !sq_empty(&conn->backlog->bl_pending));
}

/****************************************************************************
 * Name: tcp_backlogremove
 *
 * Description:
 *  Called from accept().  Before waiting for a new connection, accept will
 *  call this API to see if there are pending connections in the backlog.
 *
 * Assumptions:
 *   Called from network socket logic with the network locked
 *
 ****************************************************************************/

FAR struct tcp_conn_s *tcp_backlogremove(FAR struct tcp_conn_s *conn)
{
  FAR struct tcp_backlog_s     *bls;
  FAR struct tcp_blcontainer_s *blc;
  FAR struct tcp_conn_s        *blconn = NULL;

#ifdef CONFIG_DEBUG_FEATURES
  if (!conn)
    {
      return NULL;
    }
#endif

  bls = conn->backlog;
  if (bls)
    {
      /* Remove the a container at the head of the pending connection list
       * (FIFO)
       */

      blc = (FAR struct tcp_blcontainer_s *)sq_remfirst(&bls->bl_pending);
      if (blc)
        {
          /* Extract the connection reference from the container and put
           * container in the free list
           */

          blconn       = blc->bc_conn;
          blc->bc_conn = NULL;
          sq_addlast(&blc->bc_node, &bls->bl_free);
        }
    }

  ninfo("conn=%p, returning %p\n", conn, blconn);
  return blconn;
}

/****************************************************************************
 * Name: tcp_backlogdelete
 *
 * Description:
 *  Called from tcp_free() when a connection is freed that this also
 *  retained in the pending connection list of a listener.  We simply need
 *  to remove the defunct connection from the list.
 *
 * Assumptions:
 *   Called from network socket logic with the network locked
 *
 ****************************************************************************/

int tcp_backlogdelete(FAR struct tcp_conn_s *conn,
                      FAR struct tcp_conn_s *blconn)
{
  FAR struct tcp_backlog_s     *bls;
  FAR struct tcp_blcontainer_s *blc;
  FAR struct tcp_blcontainer_s *prev;

  ninfo("conn=%p blconn=%p\n", conn, blconn);

#ifdef CONFIG_DEBUG_FEATURES
  if (!conn)
    {
      return -EINVAL;
    }
#endif

  bls = conn->backlog;
  if (bls)
    {
      /* Find the container hold the connection */

      for (blc = (FAR struct tcp_blcontainer_s *)sq_peek(&bls->bl_pending),
           prev = NULL;
           blc;
           prev = blc,
            blc = (FAR struct tcp_blcontainer_s *)sq_next(&blc->bc_node))
        {
          if (blc->bc_conn == blconn)
            {
              if (prev)
                {
                  /* Remove the a container from the middle of the list of
                   * pending connections
                   */

                  sq_remafter(&prev->bc_node, &bls->bl_pending);
                }
              else
                {
                  /* Remove the a container from the head of the list of
                   * pending connections
                   */

                  sq_remfirst(&bls->bl_pending);
                }

              /* Put container in the free list */

              blc->bc_conn = NULL;
              sq_addlast(&blc->bc_node, &bls->bl_free);
              return OK;
            }
        }

      nerr("ERROR: Failed to find pending connection\n");
      return -EINVAL;
    }

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_TCP && CONFIG_NET_TCPBACKLOG */
