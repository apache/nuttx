/****************************************************************************
 * net/socket/net_sockets.c
 *
 *   Copyright (C) 2007-2009, 2011-2014, 2016 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/kmalloc.h>

#include "socket/socket.h"

#if CONFIG_NSOCKET_DESCRIPTORS > 0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void _net_semtake(FAR struct socketlist *list)
{
  int ret;

  /* Take the semaphore (perhaps waiting) */

  while ((ret = net_lockedwait(&list->sl_sem)) < 0)
    {
      /* The only case that an error should occr here is if
       * the wait was awakened by a signal.
       */

      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
    }

  UNUSED(ret);
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

  (void)nxsem_init(&list->sl_sem, 0, 1);
}

/****************************************************************************
 * Name: net_releaselist
 *
 * Description:
 *   Release resources held by the socket list
 *
 * Input Parameters:
 *   list -- A reference to the pre-allocated socket list to be un-initialized.
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
          (void)psock_close(psock);
        }
    }

  /* Destroy the semaphore */

  (void)nxsem_destroy(&list->sl_sem);
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

  list = sched_getsockets();
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
 * Name: sock_release
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

void sock_release(FAR struct socket *psock)
{
  if (psock != NULL)
    {
      /* Take the list semaphore so that there will be no accesses
       * to this socket structure.
       */

      FAR struct socketlist *list = sched_getsockets();
      if (list)
        {
          /* Decrement the count if there the socket will persist
           * after this.
           */

          _net_semtake(list);
          if (psock->s_crefs > 1)
            {
              psock->s_crefs--;
            }
          else
            {
              /* The socket will not persist... reset it */

              memset(psock, 0, sizeof(struct socket));
            }

          _net_semgive(list);
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

  /* Get the socket structure for this sockfd */

  if (psock)
    {
      sock_release(psock);
    }
}

/****************************************************************************
 * Name: sockfd_socket
 *
 * Description:
 *   Given a socket descriptor, return the underlying socket structure.
 *
 * Input Parameters:
 *   sockfd - The socket descriptor index o use.
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
      list = sched_getsockets();
      if (list)
        {
          return &list->sl_sockets[ndx];
        }
    }

  return NULL;
}

#endif /* CONFIG_NSOCKET_DESCRIPTORS > 0 */
