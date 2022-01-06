/****************************************************************************
 * fs/vfs/fs_select.c
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

#include <sys/select.h>
#include <sys/time.h>

#include <string.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: select
 *
 * Description:
 *   select() allows a program to monitor multiple file descriptors, waiting
 *   until one or more of the file descriptors become "ready" for some class
 *   of I/O operation (e.g., input possible).  A file descriptor is
 *   considered  ready if it is possible to perform the corresponding I/O
 *   operation (e.g., read(2)) without blocking.
 *
 *   NOTE: poll() is the fundamental API for performing such monitoring
 *   operation under NuttX.  select() is provided for compatibility and
 *   is simply a layer of added logic on top of poll().  As such, select()
 *   is more wasteful of resources and poll() is the recommended API to be
 *   used.
 *
 * Input Parameters:
 *   nfds - the maximum fd number (+1) of any descriptor in any of the
 *     three sets.
 *   readfds - the set of descriptions to monitor for read-ready events
 *   writefds - the set of descriptions to monitor for write-ready events
 *   exceptfds - the set of descriptions to monitor for error events
 *   timeout - Return at this time if none of these events of interest
 *     occur.
 *
 *  Returned Value:
 *   0: Timer expired
 *  >0: The number of bits set in the three sets of descriptors
 *  -1: An error occurred (errno will be set appropriately)
 *
 ****************************************************************************/

int select(int nfds, FAR fd_set *readfds, FAR fd_set *writefds,
           FAR fd_set *exceptfds, FAR struct timeval *timeout)
{
  struct pollfd *pollset = NULL;
  int errcode = OK;
  int fd;
  int npfds;
  int msec;
  int ndx;
  int ret;

  /* select() is cancellation point */

  enter_cancellation_point();

  if (nfds < 0)
    {
      errcode = EINVAL;
      goto errout;
    }

  /* How many pollfd structures do we need to allocate? */

  /* Initialize the descriptor list for poll() */

  for (fd = 0, npfds = 0; fd < nfds; fd++)
    {
      /* Check if any monitor operation is requested on this fd */

      if ((readfds   && FD_ISSET(fd, readfds))  ||
          (writefds  && FD_ISSET(fd, writefds)) ||
          (exceptfds && FD_ISSET(fd, exceptfds)))
        {
          /* Yes.. increment the count of pollfds structures needed */

          npfds++;
        }
    }

  /* Allocate the descriptor list for poll() */

  if (npfds > 0)
    {
      pollset = (FAR struct pollfd *)
        kmm_zalloc(npfds * sizeof(struct pollfd));

      if (pollset == NULL)
        {
          errcode = ENOMEM;
          goto errout;
        }
    }

  /* Initialize the descriptor list for poll() */

  for (fd = 0, ndx = 0; fd < nfds; fd++)
    {
      int incr = 0;

      /* The readfs set holds the set of FDs that the caller can be assured
       * of reading from without blocking.  Note that POLLHUP is included as
       * a read-able condition.  POLLHUP will be reported at the end-of-file
       * or when a connection is lost.  In either case, the read() can then
       * be performed without blocking.
       */

      if (readfds && FD_ISSET(fd, readfds))
        {
          pollset[ndx].fd      = fd;
          pollset[ndx].events |= POLLIN;
          incr                 = 1;
        }

      /* The writefds set holds the set of FDs that the caller can be assured
       * of writing to without blocking.
       */

      if (writefds && FD_ISSET(fd, writefds))
        {
          pollset[ndx].fd      = fd;
          pollset[ndx].events |= POLLOUT;
          incr                 = 1;
        }

      /* The exceptfds set holds the set of FDs that are watched for
       * exceptions
       */

      if (exceptfds && FD_ISSET(fd, exceptfds))
        {
          pollset[ndx].fd      = fd;
          incr                  = 1;
        }

      ndx += incr;
    }

  DEBUGASSERT(ndx == npfds);

  /* Convert the timeout to milliseconds */

  if (timeout)
    {
      /* Calculate the timeout in milliseconds */

      msec = timeout->tv_sec * 1000 + timeout->tv_usec / 1000;
    }
  else
    {
      /* Any negative value of msec means no timeout */

      msec = -1;
    }

  /* Then let poll do all of the real work. */

  ret = nx_poll(pollset, npfds, msec);
  if (ret < 0)
    {
      /* poll() failed! Save the errno value */

      errcode = -ret;
    }

  /* Now set up the return values */

  if (readfds)
    {
      memset(readfds, 0, sizeof(fd_set));
    }

  if (writefds)
    {
      memset(writefds, 0, sizeof(fd_set));
    }

  if (exceptfds)
    {
      memset(exceptfds, 0, sizeof(fd_set));
    }

  /* Convert the poll descriptor list back into selects 3 bitsets */

  if (ret > 0)
    {
      ret = 0;
      for (ndx = 0; ndx < npfds; ndx++)
        {
          /* Check for read conditions.  Note that POLLHUP is included as a
           * read condition.  POLLHUP will be reported when no more data will
           * be available (such as when a connection is lost).  In either
           * case, the read() can then be performed without blocking.
           */

          if (readfds)
            {
              if (pollset[ndx].revents & (POLLIN | POLLHUP))
                {
                  FD_SET(pollset[ndx].fd, readfds);
                  ret++;
                }
            }

          /* Check for write conditions */

          if (writefds)
            {
              if (pollset[ndx].revents & (POLLOUT | POLLHUP))
                {
                  FD_SET(pollset[ndx].fd, writefds);
                  ret++;
                }
            }

          /* Check for exceptions */

          if (exceptfds)
            {
              if (pollset[ndx].revents & POLLERR)
                {
                  FD_SET(pollset[ndx].fd, exceptfds);
                  ret++;
                }
            }
        }
    }

  kmm_free(pollset);

  /* Did poll() fail above? */

  if (ret >= 0)
    {
      leave_cancellation_point();
      return ret;
    }

errout:
  set_errno(errcode);
  leave_cancellation_point();
  return ERROR;
}
