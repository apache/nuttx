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
static long nwatches = 0;
#endif

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

struct fdwatch_s *fdwatch_initialize(int nfds)
{
  struct fdwatch_s *fw;
  int i;

  /* Allocate the fdwatch data structure */

  fw = (struct fdwatch_s*)zalloc(sizeof(struct fdwatch_s));
  if (!fw)
    {
      ndbg("Failed to allocate fdwatch\n");
      return NULL;
    }

  /* Initialize the fdwatch data structures. */

  fw->nfds = nfds;

  fw->fd_rw = (int*)malloc(sizeof(int) * nfds);
  if (!fw->fd_rw)
    {
      goto errout_with_allocations;
    }

  fw->fd_data = (void**)malloc(sizeof(void*) * nfds);
  if (!fw->fd_data)
    {
      goto errout_with_allocations;
    }

  for (i = 0; i < nfds; ++i)
    {
      fw->fd_rw[i] = -1;
    }

  fw->pollfds = (struct pollfd*)malloc(sizeof(struct pollfd) * nfds);
  if (!fw->pollfds)
    {
      goto errout_with_allocations;
    }

  fw->poll_pollndx = (int*)malloc(sizeof(int) * nfds);
  if (!fw->poll_pollndx)
    {
      goto errout_with_allocations;
    }

  fw->poll_rfdidx  = (int*)malloc(sizeof(int) * nfds);
  if (!fw->poll_rfdidx)
    {
      goto errout_with_allocations;
    }

  for (i = 0; i < nfds; i++)
    {
      fw->pollfds[i].fd = fw->poll_pollndx[i] = -1;
    }

  return fw;

errout_with_allocations:
  fdwatch_uninitialize(fw);
  return NULL;
}

/* Uninitialize the fwdatch data structure */

void fdwatch_uninitialize(struct fdwatch_s *fw)
{
  if (fw)
    {
      if (fw->fd_rw)
        {
          free(fw->fd_rw);
        }

      if (fw->fd_data)
        {
          free(fw->fd_data);
        }

      if (fw->pollfds)
        {
          free(fw->pollfds);
        }

      if (fw->poll_pollndx)
        {
          free(fw->poll_pollndx);
        }

      if (fw->poll_rfdidx)
        {
          free(fw->poll_rfdidx);
        }

      free(fw);
    }
}

/* Add a descriptor to the watch list.  rw is either FDW_READ or FDW_WRITE.  */

void fdwatch_add_fd(struct fdwatch_s *fw, int fd, void *client_data, int rw)
{
  int fdndx;

#ifdef CONFIG_DEBUG
  if (fd < CONFIG_NFILE_DESCRIPTORS ||
      fd >= CONFIG_NFILE_DESCRIPTORS+fw->nfds ||
      fw->fd_rw[fd-CONFIG_NFILE_DESCRIPTORS] != -1)
    {
      ndbg("Received bad fd (%d)\n", fd);
      return;
    }
#endif

  if (fw->npoll_fds >= fw->nfds)
    {
      ndbg("too many fds\n");
      return;
    }

  /* Get the socket index associated with the fd */

  fdndx = fd-CONFIG_NFILE_DESCRIPTORS;

  /* Save the new fd at the end of the list */

  fw->pollfds[fw->npoll_fds].fd = fd;
  switch (rw)
    {
    default:
    case FDW_READ:
      fw->pollfds[fw->npoll_fds].events = POLLIN;
      break;

    case FDW_WRITE:
      fw->pollfds[fw->npoll_fds].events = POLLOUT;
      break;
    }

  /* Save the new index and increment the cound of watched descriptors */

  fw->poll_pollndx[fdndx] = fw->npoll_fds;
  fw->npoll_fds++;

  fw->fd_rw[fdndx]        = rw;
  fw->fd_data[fdndx]      = client_data;
}

/* Remove a descriptor from the watch list. */

void fdwatch_del_fd(struct fdwatch_s *fw, int fd)
{
  int fdndx;
  int pollndx;
  int tmpndx;

#ifdef CONFIG_DEBUG
  if (fd < CONFIG_NFILE_DESCRIPTORS ||
      fd >= CONFIG_NFILE_DESCRIPTORS+fw->nfds ||
      fw->fd_rw[fd-CONFIG_NFILE_DESCRIPTORS] == -1)
    {
      ndbg("Received bad fd: %d\n", fd);
      return;
    }
#endif

  /* Get the socket index associated with the fd and then the poll
   * index associated with that.
   */

  fdndx   = fd-CONFIG_NFILE_DESCRIPTORS;
  pollndx = fw->poll_pollndx[fdndx];

#ifdef CONFIG_DEBUG
  if (pollndx < 0 || pollndx >= fw->nfds)
    {
      ndbg("Bad poll index: %d\n", pollndx);
      return;
    }
#endif

  /* Decrement the number of fds in the poll table */

  fw->npoll_fds--;

  /* Replace the deleted one with the one at the the end
   * of the list.
   */

  tmpndx                = fw->pollfds[pollndx].fd - CONFIG_NFILE_DESCRIPTORS;
  fw->pollfds[pollndx]      = fw->pollfds[fw->npoll_fds];
  fw->poll_pollndx[tmpndx]    =  fw->poll_pollndx[fdndx];;
  fw->pollfds[fw->npoll_fds].fd = -1;
  fw->poll_pollndx[fdndx]     = -1;

  fw->fd_rw[fdndx]          = -1;
  fw->fd_data[fdndx]        = NULL;
}

/* Do the watch.  Return value is the number of descriptors that are ready,
 * or 0 if the timeout expired, or -1 on errors.  A timeout of INFTIM means
 * wait indefinitely.
 */

int fdwatch(struct fdwatch_s *fw, long timeout_msecs)
{
  int rfndx;
  int ret;
  int i;

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
  nwatches++;
#endif

  ret = poll(fw->pollfds, fw->npoll_fds, (int)timeout_msecs);
  if (ret <= 0)
    {
      return ret;
    }

  rfndx = 0;
  for (i = 0; i < fw->npoll_fds; i++)
    {
      if (fw->pollfds[i].revents & (POLLIN | POLLOUT | POLLERR | POLLHUP | POLLNVAL))
        {
          fw->poll_rfdidx[rfndx++] = fw->pollfds[i].fd;
          if (rfndx == ret)
            {
              break;
            }
        }
    }

  return rfndx;
}

/* Check if a descriptor was ready. */

int fdwatch_check_fd(struct fdwatch_s *fw, int fd)
{
  int fdndx;
  int pollndx;

#ifdef CONFIG_DEBUG
  if (fd < CONFIG_NFILE_DESCRIPTORS ||
      fd >= CONFIG_NFILE_DESCRIPTORS+fw->nfds ||
      fw->fd_rw[fd-CONFIG_NFILE_DESCRIPTORS] == -1)
    {
      ndbg("Bad fd: %d\n", fd);
      return 0;
    }
#endif

  /* Get the socket index associated with the fd and then the poll
   * index associated with that.
   */

  fdndx   = fd-CONFIG_NFILE_DESCRIPTORS;
  pollndx = fw->poll_pollndx[fdndx];

#ifdef CONFIG_DEBUG
  if (pollndx < 0 || pollndx >= fw->nfds)
    {
      ndbg("Bad poll index: %d\n", pollndx);
      return 0;
    }
#endif

  if (fw->pollfds[pollndx].revents & POLLERR)
    {
        return 0;
    }

  switch (fw->fd_rw[fdndx])
    {
    case FDW_READ:
      return fw->pollfds[pollndx].revents & (POLLIN | POLLHUP | POLLNVAL);

    case FDW_WRITE:
      return fw->pollfds[pollndx].revents & (POLLOUT | POLLHUP | POLLNVAL);

    default:
      break;
    }
  return 0;
}

void *fdwatch_get_next_client_data(struct fdwatch_s *fw)
{
  int rfndx;
  int fdndx;
  int fd;

#ifdef CONFIG_DEBUG
  if (rfndx < 0 || rfndx >= fw->nfds)
    {
      ndbg("Bad rfndx: %d\n", rfndx);
      return NULL;
    }
#endif

  fd   = fw->poll_rfdidx[rfndx];
  fdndx = fd-CONFIG_NFILE_DESCRIPTORS;
  if (fdndx < 0 || fdndx >= fw->nfds)
    {
      return NULL;
    }
  return fw->fd_data[fdndx];
}

/* Generate debugging statistics ndbg message. */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
void fdwatch_logstats(struct fdwatch_s *fw, long secs)
{
  if (secs > 0)
    ndbg("fdwatch - %ld polls (%g/sec)\n", nwatches, (float)nwatches / secs);
  nwatches = 0;
}
#endif

#endif /* CONFIG_THTTPD */

