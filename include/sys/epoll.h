#ifndef _EPOLL__H
#define _EPOLL__H

#include <poll.h>

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

#define EPOLL_CTL_ADD 1 /* Add a file descriptor to the interface.  */
#define EPOLL_CTL_DEL 2 /* Remove a file descriptor from the interface.  */
#define EPOLL_CTL_MOD 3 /* Change file descriptor epoll_event structure.  */

typedef union poll_data
{
  int         fd;       /* The descriptor being polled */
} epoll_data_t;

struct epoll_event
{
  epoll_data_t data;
  sem_t       *sem;      /* Pointer to semaphore used to post output event */
  pollevent_t  events;   /* The input event flags */
  pollevent_t  revents;  /* The output event flags */
  FAR void    *priv;     /* For use by drivers */
};

struct epoll_head
{
  int size;
  int occupied;
  struct epoll_event *evs;
};

int epoll_create(int size);
int epoll_ctl(int epfd, int op, int fd, struct epoll_event *ev);
int epoll_wait(int epfd, struct epoll_event *evs, int maxevents, int timeout);

void epoll_close(int epfd);

#endif
