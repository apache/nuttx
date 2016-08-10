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

#ifndef __INCLUDE_SYS_EPOLL_H
#define __INCLUDE_SYS_EPOLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <poll.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EPOLL_CTL_ADD 1 /* Add a file descriptor to the interface.  */
#define EPOLL_CTL_DEL 2 /* Remove a file descriptor from the interface.  */
#define EPOLL_CTL_MOD 3 /* Change file descriptor epoll_event structure.  */

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum EPOLL_EVENTS
  {
    EPOLLIN = POLLIN,
#define EPOLLIN EPOLLIN
    EPOLLPRI = POLLPRI,
#define EPOLLPRI EPOLLPRI
    EPOLLOUT = POLLOUT,
#define EPOLLOUT EPOLLOUT
    EPOLLRDNORM = POLLRDNORM,
#define EPOLLRDNORM EPOLLRDNORM
    EPOLLRDBAND = POLLRDBAND,
#define EPOLLRDBAND EPOLLRDBAND
    EPOLLWRNORM = POLLWRNORM,
#define EPOLLWRNORM EPOLLWRNORM
    EPOLLWRBAND = POLLWRBAND,
#define EPOLLWRBAND EPOLLWRBAND
    EPOLLERR = POLLERR,
#define EPOLLERR EPOLLERR
    EPOLLHUP = POLLHUP,
#define EPOLLHUP EPOLLHUP
  };

typedef union poll_data
{
  int          fd;       /* The descriptor being polled */
} epoll_data_t;

struct epoll_event
{
  epoll_data_t data;
  FAR sem_t   *sem;      /* Pointer to semaphore used to post output event */
  pollevent_t  events;   /* The input event flags */
  pollevent_t  revents;  /* The output event flags */
  FAR void    *priv;     /* For use by drivers */
};

struct epoll_head
{
  int size;
  int occupied;
  FAR struct epoll_event *evs;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int epoll_create(int size);
int epoll_ctl(int epfd, int op, int fd, struct epoll_event *ev);
int epoll_wait(int epfd, struct epoll_event *evs, int maxevents, int timeout);

void epoll_close(int epfd);

#endif /* __INCLUDE_SYS_EPOLL_H */
