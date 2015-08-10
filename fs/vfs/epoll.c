#include <stdint.h>
#include <poll.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <sys/epoll.h>

int epoll_create(int size)
{
  struct epoll_head *eph = malloc(sizeof(struct epoll_head));

  eph->size = size;
  eph->occupied = 0;
  eph->evs = malloc(sizeof(struct epoll_event) * eph->size);

  return (int)eph;
}

void epoll_close(int epfd)
{
  struct epoll_head *eph = (struct epoll_head *)epfd;

  free(eph->evs);
  free(eph);
}

int epoll_ctl(int epfd, int op, int fd, struct epoll_event *ev)
{
  struct epoll_head *eph = (struct epoll_head *)epfd;

  switch (op)
    {
      case EPOLL_CTL_ADD:
        printf("%08x CTL ADD(%d): fd=%d ev=%08x\n", epfd, eph->occupied, fd, ev->events);
        eph->evs[eph->occupied].events = ev->events | POLLERR | POLLHUP;
        eph->evs[eph->occupied++].data.fd = fd;
        return 0;

      case EPOLL_CTL_DEL:
        {
          int i;
          for (i=0; i < eph->occupied; i++)
            {
              if (eph->evs[i].data.fd == fd)
                {
                  if (i != eph->occupied-1)
                    memmove(&eph->evs[i], &eph->evs[i + 1], eph->occupied - i);
                  eph->occupied--;
                  return 0;
                }
            }
          return -ENOENT;
        }

      case EPOLL_CTL_MOD:
        {
          int i;
          printf("%08x CTL MOD(%d): fd=%d ev=%08x\n", epfd, eph->occupied, fd, ev->events);
          for (i=0; i < eph->occupied; i++)
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

int epoll_wait(int epfd, struct epoll_event *evs, int maxevents, int timeout)
{
  int i;
  int rc;
  struct epoll_head *eph = (struct epoll_head *)epfd;

  rc = poll((struct pollfd *)eph->evs, eph->occupied, timeout);

  if (rc <= 0)
    {
      if (rc < 0) {
      printf("%08x poll fail: %d for %d, %d msecs\n", epfd, rc, eph->occupied, timeout);
      for (i=0; i < eph->occupied; i++)
        {
          printf("%02d: fd=%d\n", i, eph->evs[i].data.fd);
        }
      }
      return rc;
    }

  for (i=0; i < rc; i++)
    {
      evs[i].data.fd = (pollevent_t)eph->evs[i].data.fd;
      evs[i].events = (pollevent_t)eph->evs[i].revents;
    }

  return rc;
}
