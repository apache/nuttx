/****************************************************************************
 * netutils/thttpd/timers.c
 * FD watcher routines for poll()
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Derived from the file of the same name in the original THTTPD package:
 *
 *   Copyright © 1999,2000 by Jef Poskanzer <jef@mail.acme.com>.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdlib.h>
#include <debug.h>

#if 0
#include <sys/time.h>

#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#endif
#include <poll.h>

#include "config.h"
#include "fdwatch.h"

#ifdef CONFIG_THTTPD

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
static long           nwatches;
#endif

static int           *fd_rw;
static void         **fd_data;
static struct pollfd *pollfds;
static int           *poll_pollndx;
static int           *poll_rfdidx;
static int            npoll_fds;
static int            nreturned;
static int            next_rfndx;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Initialize the fdwatch data structures.  Returns -1 on failure. */

int fdwatch_initialize(void)
{
  int i;

  /* Initialize the fdwatch data structures. */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
  nwatches = 0;
#endif

  fd_rw   = (int *)malloc(sizeof(int) * CONFIG_NSOCKET_DESCRIPTORS);
  if (!fd_rw)
    {
      goto errout;
    }

  fd_data = (void **)malloc(sizeof(void*) * CONFIG_NSOCKET_DESCRIPTORS);
  if (!fd_data)
    {
      goto errout_with_fd_rw;
    }

  for (i = 0; i < CONFIG_NSOCKET_DESCRIPTORS; ++i)
    {
      fd_rw[i] = -1;
    }

  pollfds = (struct pollfd *)malloc(sizeof(struct pollfd) * CONFIG_NSOCKET_DESCRIPTORS);
  if (!pollfds)
    {
      goto errout_with_fd_data;
    }

  poll_pollndx = (int *)malloc(sizeof(int) * CONFIG_NSOCKET_DESCRIPTORS);
  if (!poll_pollndx)
    {
      goto errout_with_pollfds;
    }

  poll_rfdidx  = (int *)malloc(sizeof(int) * CONFIG_NSOCKET_DESCRIPTORS);
  if (!poll_rfdidx)
    {
      goto errout_with_poll_pollndx;
    }

  for (i = 0; i < CONFIG_NSOCKET_DESCRIPTORS; i++)
    {
      pollfds[i].fd = poll_pollndx[i] = -1;
    }

  return OK;

errout_with_poll_pollndx:
  free(poll_pollndx);
errout_with_pollfds:
  free(pollfds);
errout_with_fd_data:
  free(fd_data);
errout_with_fd_rw:
  free(fd_rw);
errout:
  return ERROR;
}

/* Add a descriptor to the watch list.  rw is either FDW_READ or FDW_WRITE.  */

void fdwatch_add_fd(int fd, void *client_data, int rw)
{
  int fdndx;

#ifdef CONFIG_DEBUG
  if (fd < CONFIG_NFILE_DESCRIPTORS ||
      fd >= CONFIG_NFILE_DESCRIPTORS+CONFIG_NSOCKET_DESCRIPTORS ||
      fd_rw[fd-CONFIG_NFILE_DESCRIPTORS] != -1)
    {
      ndbg("Received bad fd (%d)\n", fd);
      return;
    }
#endif

  if (npoll_fds >= CONFIG_NSOCKET_DESCRIPTORS)
    {
      ndbg("too many fds\n");
      return;
    }

  /* Get the socket index associated with the fd */

  fdndx = fd-CONFIG_NFILE_DESCRIPTORS;

  /* Save the new fd at the end of the list */

  pollfds[npoll_fds].fd = fd;
  switch (rw)
    {
    default:
    case FDW_READ:
      pollfds[npoll_fds].events = POLLIN;
      break;

    case FDW_WRITE:
      pollfds[npoll_fds].events = POLLOUT;
      break;
    }

  /* Save the new index and increment the cound of watched descriptors */

  poll_pollndx[fdndx] = npoll_fds;
  npoll_fds++;

  fd_rw[fdndx]        = rw;
  fd_data[fdndx]      = client_data;
}

/* Remove a descriptor from the watch list. */

void fdwatch_del_fd(int fd)
{
  int fdndx;
  int pollndx;
  int tmpndx;

#ifdef CONFIG_DEBUG
  if (fd < CONFIG_NFILE_DESCRIPTORS ||
      fd >= CONFIG_NFILE_DESCRIPTORS+CONFIG_NSOCKET_DESCRIPTORS ||
      fd_rw[fd-CONFIG_NFILE_DESCRIPTORS] == -1)
    {
      ndbg("Received bad fd: %d\n", fd);
      return;
    }
#endif

  /* Get the socket index associated with the fd and then the poll
   * index associated with that.
   */

  fdndx   = fd-CONFIG_NFILE_DESCRIPTORS;
  pollndx = poll_pollndx[fdndx];

#ifdef CONFIG_DEBUG
  if (pollndx < 0 || pollndx >= CONFIG_NSOCKET_DESCRIPTORS)
    {
      ndbg("Bad poll index: %d\n", pollndx);
      return;
    }
#endif

  /* Decrement the number of fds in the poll table */

  npoll_fds--;

  /* Replace the deleted one with the one at the the end
   * of the list.
   */

  tmpndx                = pollfds[pollndx].fd - CONFIG_NFILE_DESCRIPTORS;
  pollfds[pollndx]      = pollfds[npoll_fds];
  poll_pollndx[tmpndx]    =  poll_pollndx[fdndx];;
  pollfds[npoll_fds].fd = -1;
  poll_pollndx[fdndx]     = -1;

  fd_rw[fdndx]          = -1;
  fd_data[fdndx]        = NULL;
}

/* Do the watch.  Return value is the number of descriptors that are ready,
 * or 0 if the timeout expired, or -1 on errors.  A timeout of INFTIM means
 * wait indefinitely.
 */

int fdwatch(long timeout_msecs)
{
  int rfndx;
  int ret;
  int i;

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
  nwatches++;
#endif

  ret = poll(pollfds, npoll_fds, (int)timeout_msecs);
  if (ret <= 0)
    {
      return ret;
    }

  rfndx = 0;
  for (i = 0; i < npoll_fds; i++)
    {
      if (pollfds[i].revents & (POLLIN | POLLOUT | POLLERR | POLLHUP | POLLNVAL))
        {
          poll_rfdidx[rfndx++] = pollfds[i].fd;
          if (rfndx == ret)
            {
              break;
            }
        }
    }

  next_rfndx = 0;
  return rfndx;
}

/* Check if a descriptor was ready. */

int fdwatch_check_fd(int fd)
{
  int fdndx;
  int pollndx;

#ifdef CONFIG_DEBUG
  if (fd < CONFIG_NFILE_DESCRIPTORS ||
      fd >= CONFIG_NFILE_DESCRIPTORS+CONFIG_NSOCKET_DESCRIPTORS ||
      fd_rw[fd-CONFIG_NFILE_DESCRIPTORS] == -1)
    {
      ndbg("Bad fd: %d\n", fd);
      return 0;
    }
#endif

  /* Get the socket index associated with the fd and then the poll
   * index associated with that.
   */

  fdndx   = fd-CONFIG_NFILE_DESCRIPTORS;
  pollndx = poll_pollndx[fdndx];

#ifdef CONFIG_DEBUG
  if (pollndx < 0 || pollndx >= CONFIG_NSOCKET_DESCRIPTORS)
    {
      ndbg("Bad poll index: %d\n", pollndx);
      return 0;
    }
#endif

  if (pollfds[pollndx].revents & POLLERR)
    {
        return 0;
    }

  switch (fd_rw[fdndx])
    {
    case FDW_READ:
      return pollfds[pollndx].revents & (POLLIN | POLLHUP | POLLNVAL);

    case FDW_WRITE:
      return pollfds[pollndx].revents & (POLLOUT | POLLHUP | POLLNVAL);

    default:
      break;
    }
  return 0;
}

void *fdwatch_get_next_client_data(void)
{
  int rfndx;
  int fdndx;
  int fd;

  if (next_rfndx >= nreturned)
    {
      return (void*)-1;
    }

  rfndx = next_rfndx++;

#ifdef CONFIG_DEBUG
  if (rfndx < 0 || rfndx >= CONFIG_NSOCKET_DESCRIPTORS)
    {
      ndbg("Bad rfndx: %d\n", rfndx);
      return NULL;
    }
#endif

  fd   = poll_rfdidx[rfndx];
  fdndx = fd-CONFIG_NFILE_DESCRIPTORS;
  if (fdndx < 0 || fdndx >= CONFIG_NSOCKET_DESCRIPTORS)
    {
      return NULL;
    }
  return fd_data[fdndx];
}

/* Generate debugging statistics ndbg message. */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
void fdwatch_logstats(long secs)
{
  if (secs > 0)
    ndbg("fdwatch - %ld polls (%g/sec)\n", nwatches, (float)nwatches / secs);
  nwatches = 0;
}
#endif

#endif /* CONFIG_THTTPD */

