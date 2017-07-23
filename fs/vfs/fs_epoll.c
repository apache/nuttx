/****************************************************************************
 * fs/vfs/fs_epoll.c
 *
 *   Copyright (C) 2015 Anton D. Kachalov. All rights reserved.
 *   Author: Anton D. Kachalov <mouse@mayc.ru>
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

#include <sys/epoll.h>

#include <stdint.h>
#include <poll.h>
#include <errno.h>
#include <string.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#ifndef CONFIG_DISABLE_POLL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: epoll_create
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int epoll_create(int size)
{
  FAR struct epoll_head *eph =
    (FAR struct epoll_head *)kmm_malloc(sizeof(struct epoll_head));

  eph->size = size;
  eph->occupied = 0;
  eph->evs = kmm_malloc(sizeof(struct epoll_event) * eph->size);

  /* REVISIT: This will not work on machines where:
   * sizeof(struct epoll_head *) > sizeof(int)
   */

  return (int)((intptr_t)eph);
}

/****************************************************************************
 * Name: epoll_close
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void epoll_close(int epfd)
{
  /* REVISIT: This will not work on machines where:
   * sizeof(struct epoll_head *) > sizeof(int)
   */

  FAR struct epoll_head *eph = (FAR struct epoll_head *)((intptr_t)epfd);

  kmm_free(eph->evs);
  kmm_free(eph);
}

/****************************************************************************
 * Name: epoll_ctl
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int epoll_ctl(int epfd, int op, int fd, struct epoll_event *ev)
{
  /* REVISIT: This will not work on machines where:
   * sizeof(struct epoll_head *) > sizeof(int)
   */

  FAR struct epoll_head *eph = (FAR struct epoll_head *)((intptr_t)epfd);

  switch (op)
    {
      case EPOLL_CTL_ADD:
        finfo("%08x CTL ADD(%d): fd=%d ev=%08x\n",
              epfd, eph->occupied, fd, ev->events);

        eph->evs[eph->occupied].events = ev->events | POLLERR | POLLHUP;
        eph->evs[eph->occupied++].data.fd = fd;
        return 0;

      case EPOLL_CTL_DEL:
        {
          int i;

          for (i = 0; i < eph->occupied; i++)
            {
              if (eph->evs[i].data.fd == fd)
                {
                  if (i != eph->occupied-1)
                    {
                      memmove(&eph->evs[i], &eph->evs[i + 1],
                              eph->occupied - i);
                    }

                  eph->occupied--;
                  return 0;
                }
            }

          return -ENOENT;
        }

      case EPOLL_CTL_MOD:
        {
          int i;

          finfo("%08x CTL MOD(%d): fd=%d ev=%08x\n",
                epfd, eph->occupied, fd, ev->events);

          for (i = 0; i < eph->occupied; i++)
            {
              if (eph->evs[i].data.fd == fd)
                {
                  eph->evs[i].events = ev->events | POLLERR | POLLHUP;
                  return 0;
                }
            }

          return -ENOENT;
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: epoll_wait
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int epoll_wait(int epfd, FAR struct epoll_event *evs, int maxevents,
               int timeout)
{
  /* REVISIT: This will not work on machines where:
   * sizeof(struct epoll_head *) > sizeof(int)
   */

  FAR struct epoll_head *eph = (FAR struct epoll_head *)((intptr_t)epfd);
  int counter;
  int rc;
  int i;

  rc = poll((FAR struct pollfd *)eph->evs, eph->occupied, timeout);

  if (rc <= 0)
    {
      if (rc < 0)
        {
          ferr("ERROR: %08x poll fail: %d for %d, %d msecs\n",
               epfd, rc, eph->occupied, timeout);

          for (i = 0; i < eph->occupied; i++)
            {
              ferr("  %02d: fd=%d\n", i, eph->evs[i].data.fd);
            }
        }

      return rc;
    }

  /* Iterate over non NULL event fds */

  for (i = 0, counter = 0; i < rc && counter < eph->size; counter++)
    {
      if (eph->evs[counter].revents != 0)
        {
          evs[i].data.fd = eph->evs[counter].data.fd;
          evs[i].events  = eph->evs[counter].revents;
          i += 1;
        }
    }

  return i;
}

#endif /* CONFIG_DISABLE_POLL */
