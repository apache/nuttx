/****************************************************************************
 * net/socket/net_sockets.c
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

#include <string.h>
#include <assert.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>

#include "socket/socket.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void _net_semtake(FAR struct socketlist *list)
{
  net_lockedwait_uninterruptible(&list->sl_sem);
}

#define _net_semgive(list) nxsem_post(&list->sl_sem)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_initlist
 *
 * Description:
 *   Initialize a list of sockets for a new task
 *
 * Input Parameters:
 *   list -- A reference to the pre-allocated socket list to be initialized.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void net_initlist(FAR struct socketlist *list)
{
  /* Initialize the list access mutex */

  nxsem_init(&list->sl_sem, 0, 1);
}

/****************************************************************************
 * Name: net_releaselist
 *
 * Description:
 *   Release resources held by the socket list
 *
 * Input Parameters:
 *   list - A reference to the pre-allocated socket list to be un-
 *          initialized.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void net_releaselist(FAR struct socketlist *list)
{
  int ndx;

  DEBUGASSERT(list);

  /* Close each open socket in the list. */

  for (ndx = 0; ndx < CONFIG_NSOCKET_DESCRIPTORS; ndx++)
    {
      FAR struct socket *psock = &list->sl_sockets[ndx];
      if (psock->s_crefs > 0)
        {
          psock_close(psock);
        }
    }

  /* Destroy the semaphore */

  nxsem_destroy(&list->sl_sem);
}

/****************************************************************************
 * Name: sockfd_allocate
 *
 * Description:
 *   Allocate a socket descriptor
 *
 * Input Parameters:
 *   Lowest socket descriptor index to be used.
 *
 * Returned Value:
 *   On success, a socket descriptor >= minsd is returned.  A negated errno
 *   value is returned on failure.
 *
 ****************************************************************************/

int sockfd_allocate(int minsd)
{
  FAR struct socketlist *list;
  int i;

  /* Get the socket list for this task/thread */

  list = nxsched_get_sockets();
  if (list)
    {
      /* Search for a socket structure with no references */

      _net_semtake(list);
      for (i = minsd; i < CONFIG_NSOCKET_DESCRIPTORS; i++)
        {
          /* Are there references on this socket? */

          if (!list->sl_sockets[i].s_crefs)
            {
              /* No take the reference and return the index + an offset
               * as the socket descriptor.
               */

              memset(&list->sl_sockets[i], 0, sizeof(struct socket));
              list->sl_sockets[i].s_crefs = 1;
              _net_semgive(list);
              return i + __SOCKFD_OFFSET;
            }
        }

      _net_semgive(list);
    }

  return ERROR;
}

/****************************************************************************
 * Name: psock_release
 *
 * Description:
 *   Free a socket.
 *
 * Input Parameters:
 *   psock - A reference to the socket instance to be freed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void psock_release(FAR struct socket *psock)
{
  if (psock != NULL)
    {
      /* Decrement the count if there the socket will persist
       * after this.
       */

      if (psock->s_crefs > 1)
        {
          psock->s_crefs--;
        }
      else
        {
          /* The socket will not persist... reset it */

          memset(psock, 0, sizeof(struct socket));
        }
    }
}

/****************************************************************************
 * Name: sockfd_release
 *
 * Description:
 *   Free the socket by its socket descriptor.
 *
 * Input Parameters:
 *   sockfd - Socket descriptor identifies the socket to be released.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sockfd_release(int sockfd)
{
  /* Get the socket structure for this sockfd */

  FAR struct socket *psock = sockfd_socket(sockfd);

  if (psock)
    {
      /* Take the list semaphore so that there will be no accesses
       * to this socket structure.
       */

      FAR struct socketlist *list = nxsched_get_sockets();
      if (list)
        {
          _net_semtake(list);
          psock_release(psock);
          _net_semgive(list);
        }
    }
}

/****************************************************************************
 * Name: sockfd_socket
 *
 * Description:
 *   Given a socket descriptor, return the underlying socket structure.
 *
 * Input Parameters:
 *   sockfd - The socket descriptor index to use.
 *
 * Returned Value:
 *   On success, a reference to the socket structure associated with the
 *   the socket descriptor is returned.  NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct socket *sockfd_socket(int sockfd)
{
  FAR struct socketlist *list;
  int ndx = sockfd - __SOCKFD_OFFSET;

  if (ndx >= 0 && ndx < CONFIG_NSOCKET_DESCRIPTORS)
    {
      list = nxsched_get_sockets();
      if (list)
        {
          return &list->sl_sockets[ndx];
        }
    }

  return NULL;
}
