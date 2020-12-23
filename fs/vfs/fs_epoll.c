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

#include <inttypes.h>
#include <stdint.h>
#include <poll.h>
#include <errno.h>
#include <string.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct epoll_head
{
  int size;
  int occupied;
  struct file fp;
  struct inode in;
  FAR epoll_data_t *data;
  FAR struct pollfd *poll;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int epoll_poll(FAR struct file *filep,
                      FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_epoll_ops =
{
  .poll = epoll_poll
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int epoll_poll(FAR struct file *filep,
                      FAR struct pollfd *fds, bool setup)
{
  return OK;
}

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
  FAR struct epoll_head *eph;
  int reserve = size + 1;

  eph = (FAR struct epoll_head *)
        kmm_zalloc(sizeof(struct epoll_head) +
                   sizeof(epoll_data_t) * reserve +
                   sizeof(struct pollfd) * reserve);

  eph->size = size;
  eph->data = (FAR epoll_data_t *)(eph + 1);
  eph->poll = (FAR struct pollfd *)(eph->data + reserve);

  INODE_SET_DRIVER(&eph->in);
  eph->in.u.i_ops = &g_epoll_ops;
  eph->fp.f_inode = &eph->in;
  eph->in.i_private = eph;

  eph->poll[0].ptr = &eph->fp;
  eph->poll[0].events = POLLIN | POLLFILE;

  /* REVISIT: This will not work on machines where:
   * sizeof(struct epoll_head *) > sizeof(int)
   */

  return (int)((intptr_t)eph);
}

/****************************************************************************
 * Name: epoll_create1
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int epoll_create1(int flags)
{
  /* For current implementation, Close-on-exec is a default behavior,
   * the handle of epoll(2) is not a real file handle.
   */

  if (flags != EPOLL_CLOEXEC)
    {
      set_errno(EINVAL);
      return -1;
    }

  return epoll_create(CONFIG_FS_NEPOLL_DESCRIPTORS);
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
  int i;

  switch (op)
    {
      case EPOLL_CTL_ADD:
        finfo("%08x CTL ADD(%d): fd=%d ev=%08" PRIx32 "\n",
              epfd, eph->occupied, fd, ev->events);
        if (eph->occupied >= eph->size)
          {
            set_errno(ENOMEM);
            return -1;
          }

        for (i = 1; i <= eph->occupied; i++)
          {
            if (eph->poll[i].fd == fd)
              {
                set_errno(EEXIST);
                return -1;
              }
          }

        eph->data[++eph->occupied]      = ev->data;
        eph->poll[eph->occupied].events = ev->events | POLLERR | POLLHUP;
        eph->poll[eph->occupied].fd     = fd;

        break;

      case EPOLL_CTL_DEL:
        for (i = 1; i <= eph->occupied; i++)
          {
            if (eph->poll[i].fd == fd)
              {
                if (i != eph->occupied - 1)
                  {
                    memmove(&eph->data[i], &eph->data[i + 1],
                            sizeof(epoll_data_t) * (eph->occupied - i));
                    memmove(&eph->poll[i], &eph->poll[i + 1],
                            sizeof(struct pollfd) * (eph->occupied - i));
                  }

                eph->occupied--;
                break;
              }
          }

        set_errno(ENOENT);
        return -1;

      case EPOLL_CTL_MOD:
        finfo("%08x CTL MOD(%d): fd=%d ev=%08" PRIx32 "\n",
              epfd, eph->occupied, fd, ev->events);
        for (i = 1; i <= eph->occupied; i++)
          {
            if (eph->poll[i].fd == fd)
              {
                eph->data[i]        = ev->data;
                eph->poll[i].events = ev->events | POLLERR | POLLHUP;
                break;
              }
          }

        set_errno(ENOENT);
        return -1;

      default:
        set_errno(EINVAL);
        return -1;
    }

  if (eph->poll[0].sem)
    {
      eph->poll[0].revents |= eph->poll[0].events;
      nxsem_post(eph->poll[0].sem);
    }

  return 0;
}

/****************************************************************************
 * Name: epoll_pwait
 ****************************************************************************/

int epoll_pwait(int epfd, FAR struct epoll_event *evs,
                int maxevents, int timeout, FAR const sigset_t *sigmask)
{
  /* REVISIT: This will not work on machines where:
   * sizeof(struct epoll_head *) > sizeof(int)
   */

  FAR struct epoll_head *eph = (FAR struct epoll_head *)((intptr_t)epfd);
  struct timespec expire;
  struct timespec curr;
  struct timespec diff;
  int counter;
  int rc;
  int i;

  if (timeout >= 0)
    {
      expire.tv_sec  = timeout / 1000;
      expire.tv_nsec = timeout % 1000 * 1000;

#ifdef CONFIG_CLOCK_MONOTONIC
      clock_gettime(CLOCK_MONOTONIC, &curr);
#else
      clock_gettime(CLOCK_REALTIME, &curr);
#endif

      clock_timespec_add(&curr, &expire, &expire);
    }

again:
  if (timeout < 0)
    {
      rc = ppoll(eph->poll, eph->occupied + 1, NULL, sigmask);
    }
  else
    {
#ifdef CONFIG_CLOCK_MONOTONIC
      clock_gettime(CLOCK_MONOTONIC, &curr);
#else
      clock_gettime(CLOCK_REALTIME, &curr);
#endif
      clock_timespec_subtract(&expire, &curr, &diff);

      rc = ppoll(eph->poll, eph->occupied + 1, &diff, sigmask);
    }

  if (rc <= 0)
    {
      return rc;
    }
  else if (eph->poll[0].revents != 0)
    {
      if (--rc == 0)
        {
          goto again;
        }
    }

  if (rc > maxevents)
    {
      rc = maxevents;
    }

  /* Iterate over non NULL event fds */

  for (i = 0, counter = 1; i < rc && counter <= eph->occupied; counter++)
    {
      if (eph->poll[counter].revents != 0)
        {
          evs[i].data     = eph->data[counter];
          evs[i++].events = eph->poll[counter].revents;
          if (eph->poll[counter].events & EPOLLONESHOT)
            {
              eph->poll[counter].events = 0; /* Disable oneshot internally */
            }
        }
    }

  return i;
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

int epoll_wait(int epfd, FAR struct epoll_event *evs,
               int maxevents, int timeout)
{
  return epoll_pwait(epfd, evs, maxevents, timeout, NULL);
}
