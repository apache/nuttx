/****************************************************************************
 * include/sys/epoll.h
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

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <poll.h>
#include <fcntl.h>

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
    EPOLLRDHUP = 0x2000,
#define EPOLLRDHUP EPOLLRDHUP
    EPOLLWAKEUP = 1u << 29,
#define EPOLLWAKEUP EPOLLWAKEUP
    EPOLLONESHOT = 1u << 30,
#define EPOLLONESHOT EPOLLONESHOT
  };

/* Flags to be passed to epoll_create1.  */

enum
{
  EPOLL_CLOEXEC = O_CLOEXEC
#define EPOLL_CLOEXEC EPOLL_CLOEXEC
};

union epoll_data
{
  FAR void    *ptr;
  int          fd;
  uint32_t     u32;
#ifdef CONFIG_HAVE_LONG_LONG
  uint64_t     u64;
#endif
};

typedef union epoll_data epoll_data_t;

struct epoll_event
{
  uint32_t     events;
  epoll_data_t data;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int epoll_create(int size);
int epoll_create1(int flags);
int epoll_ctl(int epfd, int op, int fd, FAR struct epoll_event *ev);
int epoll_wait(int epfd, FAR struct epoll_event *evs,
               int maxevents, int timeout);
int epoll_pwait(int epfd, FAR struct epoll_event *evs,
                int maxevents, int timeout, FAR const sigset_t *sigmask);

void epoll_close(int epfd);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_EPOLL_H */
