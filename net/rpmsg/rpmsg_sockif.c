/****************************************************************************
 * net/rpmsg/rpmsg_sockif.c
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

#include <assert.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mm/circbuf.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/semaphore.h>

#include <netinet/in.h>
#include <netpacket/rpmsg.h>

#include "socket/socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define RPMSG_SOCKET_CMD_SYNC       1
#define RPMSG_SOCKET_CMD_DATA       2

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct rpmsg_socket_sync_s
{
  uint32_t                       cmd;
  uint32_t                       size;
} end_packed_struct;

begin_packed_struct struct rpmsg_socket_data_s
{
  uint32_t                       cmd;
  uint32_t                       pos;

  /* Act data len, don't include len itself when SOCK_DGRAM */

  uint32_t                       len;
  char                           data[0];
} end_packed_struct;

struct rpmsg_socket_conn_s
{
  struct rpmsg_endpoint          ept;

  struct sockaddr_rpmsg          rpaddr;
  uint16_t                       crefs;

  FAR struct pollfd              *fds[CONFIG_NET_RPMSG_NPOLLWAITERS];

  sem_t                          sendsem;
  sem_t                          sendlock;

  sem_t                          recvsem;
  sem_t                          recvlock;

  FAR void                       *recvdata;
  uint32_t                       recvlen;
  FAR struct circbuf_s           recvbuf;

  FAR struct rpmsg_socket_conn_s *next;

  /* server listen-scoket listening: backlog > 0;
   * server listen-scoket closed: backlog = -1;
   * others: backlog = 0;
   */

  int                            backlog;

  /* Flow control, descript send side */

  uint32_t                       sendsize;
  uint32_t                       sendpos;
  uint32_t                       ackpos;

  /* Flow control, descript recv side */

  uint32_t                       recvpos;
  uint32_t                       lastpos;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        rpmsg_socket_setup(FAR struct socket *psock, int protocol);
static sockcaps_t rpmsg_socket_sockcaps(FAR struct socket *psock);
static void       rpmsg_socket_addref(FAR struct socket *psock);
static int        rpmsg_socket_bind(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen);
static int        rpmsg_socket_getsockname(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen);
static int        rpmsg_socket_getconnname(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen);
static int        rpmsg_socket_listen(FAR struct socket *psock, int backlog);
static int        rpmsg_socket_connect(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen);
static int        rpmsg_socket_accept(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen,
                    FAR struct socket *newsock);
static int        rpmsg_socket_poll(FAR struct socket *psock,
                    FAR struct pollfd *fds, bool setup);
static ssize_t    rpmsg_socket_sendmsg(FAR struct socket *psock,
                    FAR struct msghdr *msg, int flags);
static ssize_t    rpmsg_socket_recvmsg(FAR struct socket *psock,
                    FAR struct msghdr *msg, int flags);
static int        rpmsg_socket_close(FAR struct socket *psock);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_rpmsg_sockif =
{
  rpmsg_socket_setup,       /* si_setup */
  rpmsg_socket_sockcaps,    /* si_sockcaps */
  rpmsg_socket_addref,      /* si_addref */
  rpmsg_socket_bind,        /* si_bind */
  rpmsg_socket_getsockname, /* si_getsockname */
  rpmsg_socket_getconnname, /* si_getconnname */
  rpmsg_socket_listen,      /* si_listen */
  rpmsg_socket_connect,     /* si_connect */
  rpmsg_socket_accept,      /* si_accept */
  rpmsg_socket_poll,        /* si_poll */
  rpmsg_socket_sendmsg,     /* si_sendmsg */
  rpmsg_socket_recvmsg,     /* si_recvmsg */
  rpmsg_socket_close        /* si_close */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static unsigned int g_rpmsg_id;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void rpmsg_socket_lock(FAR sem_t *sem)
{
  net_lockedwait_uninterruptible(sem);
}

static inline void rpmsg_socket_unlock(FAR sem_t *sem)
{
  nxsem_post(sem);
}

static inline void rpmsg_socket_post(FAR sem_t *sem)
{
  int semcount;

  nxsem_get_value(sem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(sem);
    }
}

static void rpmsg_socket_pollnotify(FAR struct rpmsg_socket_conn_s *conn,
                                    pollevent_t eventset)
{
  int i;

  for (i = 0; i < CONFIG_NET_RPMSG_NPOLLWAITERS; i++)
    {
      FAR struct pollfd *fds = conn->fds[i];

      if (fds)
        {
          fds->revents |= (fds->events & eventset);

          if (fds->revents != 0)
            {
              rpmsg_socket_post(fds->sem);
            }
        }
    }
}

static FAR struct rpmsg_socket_conn_s *rpmsg_socket_alloc(void)
{
  FAR struct rpmsg_socket_conn_s *conn;

  conn = kmm_zalloc(sizeof(struct rpmsg_socket_conn_s));
  if (!conn)
    {
      return NULL;
    }

  circbuf_init(&conn->recvbuf, NULL, 0);

  nxsem_init(&conn->sendlock, 0, 1);
  nxsem_init(&conn->recvlock, 0, 1);
  nxsem_init(&conn->sendsem, 0, 0);
  nxsem_init(&conn->recvsem, 0, 0);
  nxsem_set_protocol(&conn->sendsem, SEM_PRIO_NONE);
  nxsem_set_protocol(&conn->recvsem, SEM_PRIO_NONE);

  conn->crefs = 1;
  return conn;
}

static void rpmsg_socket_free(FAR struct rpmsg_socket_conn_s *conn)
{
  circbuf_uninit(&conn->recvbuf);

  nxsem_destroy(&conn->recvlock);
  nxsem_destroy(&conn->sendlock);
  nxsem_destroy(&conn->sendsem);
  nxsem_destroy(&conn->recvsem);

  kmm_free(conn);
}

static int rpmsg_socket_wakeup(FAR struct rpmsg_socket_conn_s *conn)
{
  struct rpmsg_socket_data_s msg;
  uint32_t space;
  int ret = 0;

  space = conn->recvpos - conn->lastpos;

  if (space > circbuf_size(&conn->recvbuf) / 2)
    {
      msg.cmd = RPMSG_SOCKET_CMD_DATA;
      msg.pos = conn->recvpos;
      msg.len = 0;
      ret = rpmsg_send(&conn->ept, &msg, sizeof(msg));
      if (ret >= 0)
        {
          conn->lastpos = conn->recvpos;
        }
    }

  return ret;
}

static int rpmsg_socket_sync(FAR struct rpmsg_socket_conn_s *conn,
                             unsigned int timeout)
{
  struct rpmsg_socket_sync_s msg;
  int ret;

  msg.cmd  = RPMSG_SOCKET_CMD_SYNC;
  msg.size = circbuf_size(&conn->recvbuf);

  ret = rpmsg_send(&conn->ept, &msg, sizeof(msg));
  if (ret < 0)
    {
      return ret;
    }

  if (conn->sendsize == 0)
    {
      ret = net_timedwait(&conn->sendsem, timeout);
    }

  return ret;
}

static inline uint32_t rpmsg_socket_get_space(
                        FAR struct rpmsg_socket_conn_s *conn)
{
  return conn->sendsize - (conn->sendpos - conn->ackpos);
}

static int rpmsg_socket_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len, uint32_t src,
                               FAR void *priv)
{
  FAR struct rpmsg_socket_conn_s *conn = ept->priv;
  FAR struct rpmsg_socket_sync_s *head = data;

  if (head->cmd == RPMSG_SOCKET_CMD_SYNC)
    {
      conn->sendsize = head->size;
      rpmsg_socket_post(&conn->sendsem);
    }
  else
    {
      FAR struct rpmsg_socket_data_s *msg = data;
      FAR uint8_t *buf = (FAR uint8_t *)msg->data;

      rpmsg_socket_lock(&conn->sendlock);

      conn->ackpos = msg->pos;

      if (rpmsg_socket_get_space(conn) > 0)
        {
          rpmsg_socket_post(&conn->sendsem);
          rpmsg_socket_pollnotify(conn, POLLOUT);
        }

      rpmsg_socket_unlock(&conn->sendlock);

      if (len > sizeof(*msg))
        {
          len -= sizeof(*msg);

          DEBUGASSERT(len == msg->len || len == msg->len + sizeof(uint32_t));

          rpmsg_socket_lock(&conn->recvlock);

          if (conn->recvdata)
            {
              conn->recvlen = MIN(conn->recvlen, msg->len);

              if (len == msg->len)
                {
                  /* SOCOK_STREAM */

                  conn->recvpos += conn->recvlen;
                  memcpy(conn->recvdata, buf, conn->recvlen);
                  buf += conn->recvlen;
                  len -= conn->recvlen;
                }
              else
                {
                  /* SOCOK_DGRAM */

                  conn->recvpos += len;
                  memcpy(conn->recvdata, buf + sizeof(uint32_t),
                         conn->recvlen);
                  len = 0;
                }

              conn->recvdata = NULL;
              nxsem_post(&conn->recvsem);
            }

          if (len > 0)
            {
              ssize_t written;

              written = circbuf_write(&conn->recvbuf, buf, len);
              if (written != len)
                {
                  nerr("circbuf_write overflow, %d, %d\n", written, len);
                }

              rpmsg_socket_pollnotify(conn, POLLIN);
            }

          rpmsg_socket_unlock(&conn->recvlock);
        }
    }

  return 0;
}

static inline void rpmsg_socket_destroy_ept(
                    FAR struct rpmsg_socket_conn_s *conn)
{
  if (!conn)
    {
      return;
    }

  rpmsg_socket_lock(&conn->recvlock);

  if (conn->ept.rdev)
    {
      if (conn->backlog)
        {
          /* Listen socket */

          conn->backlog = -1;
        }

      rpmsg_destroy_ept(&conn->ept);
      rpmsg_socket_post(&conn->sendsem);
      rpmsg_socket_post(&conn->recvsem);
      rpmsg_socket_pollnotify(conn, POLLIN | POLLOUT);
    }

  rpmsg_socket_unlock(&conn->recvlock);
}

static void rpmsg_socket_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  rpmsg_socket_destroy_ept(ept->priv);
}

static void rpmsg_socket_device_created(FAR struct rpmsg_device *rdev,
                                        FAR void *priv)
{
  FAR struct rpmsg_socket_conn_s *conn = priv;

  if (conn->ept.rdev)
    {
      return;
    }

  if (strcmp(conn->rpaddr.rp_cpu, rpmsg_get_cpuname(rdev)) == 0)
    {
      conn->ept.priv = conn;

      rpmsg_create_ept(&conn->ept, rdev, conn->rpaddr.rp_name,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       rpmsg_socket_ept_cb, rpmsg_socket_ns_unbind);
    }
}

static void rpmsg_socket_device_destroy(FAR struct rpmsg_device *rdev,
                                        FAR void *priv)
{
  FAR struct rpmsg_socket_conn_s *conn = priv;

  if (strcmp(conn->rpaddr.rp_cpu, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_socket_destroy_ept(conn);
    }
}

static void rpmsg_socket_ns_bind(FAR struct rpmsg_device *rdev,
                                 FAR void *priv, FAR const char *name,
                                 uint32_t dest)
{
  FAR struct rpmsg_socket_conn_s *server = priv;
  FAR struct rpmsg_socket_conn_s *tmp;
  FAR struct rpmsg_socket_conn_s *new;
  int ret;
  int i = 0;

  if (strncmp(name, server->rpaddr.rp_name,
              strlen(server->rpaddr.rp_name)))
    {
      return;
    }

  if (strlen(server->rpaddr.rp_cpu) &&
          strcmp(server->rpaddr.rp_cpu, rpmsg_get_cpuname(rdev)))
    {
      /* Bind specific CPU, then only listen that CPU */

      return;
    }

  for (tmp = server; tmp->next; tmp = tmp->next)
    {
      if (++i > server->backlog)
        {
          return;
        }
    }

  new = rpmsg_socket_alloc();
  if (!new)
    {
      return;
    }

  ret = circbuf_resize(&new->recvbuf, CONFIG_NET_RPMSG_RXBUF_SIZE);
  if (ret < 0)
    {
      rpmsg_socket_free(new);
      return;
    }

  new->ept.priv = new;
  ret = rpmsg_create_ept(&new->ept, rdev, name,
                         RPMSG_ADDR_ANY, dest,
                         rpmsg_socket_ept_cb, rpmsg_socket_ns_unbind);
  if (ret < 0)
    {
      rpmsg_socket_free(new);
      return;
    }

  strcpy(new->rpaddr.rp_cpu, rpmsg_get_cpuname(rdev));
  strcpy(new->rpaddr.rp_name, name);

  tmp->next = new;

  rpmsg_socket_post(&server->recvsem);
  rpmsg_socket_pollnotify(server, POLLIN);
}

static int rpmsg_socket_getaddr(FAR struct rpmsg_socket_conn_s *conn,
                                FAR struct sockaddr *addr,
                                FAR socklen_t *addrlen)
{
  if (!addr || *addrlen < sizeof(struct sockaddr_rpmsg))
    {
      return -EINVAL;
    }

  memcpy(addr, &conn->rpaddr, sizeof(struct sockaddr_rpmsg));
  *addrlen = sizeof(struct sockaddr_rpmsg);

  return 0;
}

static int rpmsg_socket_setaddr(FAR struct rpmsg_socket_conn_s *conn,
                                FAR const struct sockaddr *addr,
                                socklen_t addrlen, bool suffix)
{
  FAR struct sockaddr_rpmsg *rpaddr = (FAR struct sockaddr_rpmsg *)addr;

  if (rpaddr->rp_family != AF_RPMSG ||
      addrlen != sizeof(struct sockaddr_rpmsg))
    {
      return -EINVAL;
    }

  memcpy(&conn->rpaddr, rpaddr, sizeof(struct sockaddr_rpmsg));

  if (suffix)
    {
      size_t len;

      rpaddr = &conn->rpaddr;
      len = strlen(rpaddr->rp_name);
      snprintf(&rpaddr->rp_name[len], RPMSG_SOCKET_NAME_SIZE - len - 1,
               "%u", g_rpmsg_id++);
    }

  return 0;
}

static int rpmsg_socket_setup(FAR struct socket *psock, int protocol)
{
  FAR struct rpmsg_socket_conn_s *conn;

  conn = rpmsg_socket_alloc();
  if (!conn)
    {
      return -ENOMEM;
    }

  psock->s_conn = conn;
  return 0;
}

static sockcaps_t rpmsg_socket_sockcaps(FAR struct socket *psock)
{
  return SOCKCAP_NONBLOCKING;
}

static void rpmsg_socket_addref(FAR struct socket *psock)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;

  conn->crefs++;
}

static int rpmsg_socket_bind(FAR struct socket *psock,
                            FAR const struct sockaddr *addr,
                            socklen_t addrlen)
{
  return rpmsg_socket_setaddr(psock->s_conn, addr, addrlen, false);
}

static int rpmsg_socket_getsockname(FAR struct socket *psock,
                                   FAR struct sockaddr *addr,
                                   FAR socklen_t *addrlen)
{
  return rpmsg_socket_getaddr(psock->s_conn, addr, addrlen);
}

static int rpmsg_socket_getconnname(FAR struct socket *psock,
                                    FAR struct sockaddr *addr,
                                    FAR socklen_t *addrlen)
{
  return rpmsg_socket_getsockname(psock, addr, addrlen);
}

static int rpmsg_socket_listen(FAR struct socket *psock, int backlog)
{
  FAR struct rpmsg_socket_conn_s *server = psock->s_conn;

  if (psock->s_type != SOCK_STREAM)
    {
      return -ENOSYS;
    }

  if (!_SS_ISBOUND(psock->s_flags) || backlog <= 0)
    {
      return -EINVAL;
    }

  server->backlog = backlog;
  return rpmsg_register_callback(server,
                                NULL,
                                NULL,
                                rpmsg_socket_ns_bind);
}

static int rpmsg_socket_connect_internal(FAR struct socket *psock)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  unsigned int timeout;
  unsigned int tc;
  int ret;

  ret = circbuf_resize(&conn->recvbuf, CONFIG_NET_RPMSG_RXBUF_SIZE);
  if (ret < 0)
    {
      return ret;
    }

  ret = rpmsg_register_callback(conn,
                                rpmsg_socket_device_created,
                                rpmsg_socket_device_destroy,
                                NULL);
  if (ret < 0)
    {
      return ret;
    }

  ret     = -ETIMEDOUT;
  timeout = _SO_TIMEOUT(psock->s_rcvtimeo);

  for (tc = 0; tc < timeout * 1000; )
    {
      ret = rpmsg_socket_sync(conn, timeout);
      if (ret != RPMSG_ERR_ADDR)
        {
          break;
        }

      if (timeout != UINT_MAX)
        {
          tc += RPMSG_TICK_COUNT;
        }
    }

  if (ret < 0)
    {
      rpmsg_unregister_callback(conn,
                                rpmsg_socket_device_created,
                                rpmsg_socket_device_destroy,
                                NULL);
    }

  return ret;
}

static int rpmsg_socket_connect(FAR struct socket *psock,
                                FAR const struct sockaddr *addr,
                                socklen_t addrlen)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  int ret;

  if (_SS_ISCONNECTED(psock->s_flags))
    {
      return -EISCONN;
    }

  ret = rpmsg_socket_setaddr(conn, addr, addrlen,
                             psock->s_type == SOCK_STREAM);
  if (ret < 0)
    {
      return ret;
    }

  return rpmsg_socket_connect_internal(psock);
}

static int rpmsg_socket_accept(FAR struct socket *psock,
                               FAR struct sockaddr *addr,
                               FAR socklen_t *addrlen,
                               FAR struct socket *newsock)
{
  FAR struct rpmsg_socket_conn_s *server = psock->s_conn;
  int ret = 0;

  if (server->backlog == -1)
    {
      return -ECONNRESET;
    }

  if (!_SS_ISLISTENING(psock->s_flags))
    {
      return -EINVAL;
    }

  while (1)
    {
      if (server->next)
        {
          FAR struct rpmsg_socket_conn_s *conn = server->next;

          server->next = conn->next;
          conn->next = NULL;

          rpmsg_socket_sync(conn, _SO_TIMEOUT(psock->s_rcvtimeo));

          rpmsg_register_callback(conn,
                                  rpmsg_socket_device_created,
                                  rpmsg_socket_device_destroy,
                                  NULL);

          newsock->s_domain = psock->s_domain;
          newsock->s_sockif = psock->s_sockif;
          newsock->s_type   = SOCK_STREAM;
          newsock->s_conn   = conn;

          rpmsg_socket_getaddr(conn, addr, addrlen);

          break;
        }
      else
        {
          if (_SS_ISNONBLOCK(psock->s_flags))
            {
              ret = -EAGAIN;
              break;
            }
          else
            {
              ret = net_lockedwait(&server->recvsem);
              if (server->backlog == -1)
                {
                  ret = -ECONNRESET;
                }

              if (ret < 0)
                {
                  break;
                }
            }
        }
    }

  return ret;
}

static int rpmsg_socket_poll(FAR struct socket *psock,
                             FAR struct pollfd *fds, bool setup)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  pollevent_t eventset = 0;
  int ret = 0;
  int i;

  if (setup)
    {
      for (i = 0; i < CONFIG_NET_RPMSG_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!conn->fds[i])
            {
              /* Bind the poll structure and this slot */

              conn->fds[i] = fds;
              fds->priv    = &conn->fds[i];
              break;
            }
        }

      if (i >= CONFIG_NET_RPMSG_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret       = -EBUSY;
          goto errout;
        }

      /* Immediately notify on any of the requested events */

      if (_SS_ISLISTENING(psock->s_flags))
        {
          if (conn->backlog == -1)
            {
              ret = -ECONNRESET;
              goto errout;
            }

          if (conn->next)
            {
              eventset |= (fds->events & POLLIN);
            }
        }
      else if (_SS_ISCONNECTED(psock->s_flags))
        {
          if (!conn->ept.rdev)
            {
              ret = -ECONNRESET;
              goto errout;
            }

          rpmsg_socket_lock(&conn->sendlock);

          if (rpmsg_socket_get_space(conn) > 0)
            {
              eventset |= (fds->events & POLLOUT);
            }

          rpmsg_socket_unlock(&conn->sendlock);

          rpmsg_socket_lock(&conn->recvlock);

          if (!circbuf_is_empty(&conn->recvbuf))
            {
              eventset |= (fds->events & POLLIN);
            }

          rpmsg_socket_unlock(&conn->recvlock);
        }
      else
        {
          ret = -EPERM;
          goto errout;
        }

      if (eventset)
        {
          rpmsg_socket_pollnotify(conn, eventset);
        }
    }
  else if (fds->priv != NULL)
    {
      for (i = 0; i < CONFIG_NET_RPMSG_NPOLLWAITERS; i++)
        {
          if (fds == conn->fds[i])
            {
              conn->fds[i] = NULL;
              fds->priv = NULL;
              break;
            }
        }
    }

errout:
  return ret;
}

static ssize_t rpmsg_socket_send_continuous(FAR struct socket *psock,
                                            FAR const void *buf,
                                            size_t len)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  FAR const char *cur = buf;
  size_t written = 0;
  int ret = 0;

  rpmsg_socket_lock(&conn->sendlock);

  while (written < len)
    {
      uint32_t block = MIN(len - written, rpmsg_socket_get_space(conn));
      FAR struct rpmsg_socket_data_s *msg;
      uint32_t ipcsize;

      if (block == 0)
        {
          if (!_SS_ISNONBLOCK(psock->s_flags))
            {
              rpmsg_socket_unlock(&conn->sendlock);

              ret = net_timedwait(&conn->sendsem,
                                  _SO_TIMEOUT(psock->s_sndtimeo));
              if (!conn->ept.rdev)
                {
                  ret = -ECONNRESET;
                }

              rpmsg_socket_lock(&conn->sendlock);

              if (ret < 0)
                {
                  break;
                }
            }
          else
            {
              ret = -EAGAIN;
              break;
            }
        }

      msg = rpmsg_get_tx_payload_buffer(&conn->ept, &ipcsize, true);
      if (!msg)
        {
          ret = -EINVAL;
          break;
        }

      block = MIN(block, ipcsize - sizeof(*msg));

      msg->cmd = RPMSG_SOCKET_CMD_DATA;
      msg->pos = conn->recvpos;
      msg->len = block;
      memcpy(msg->data, cur, block);

      conn->lastpos  = conn->recvpos;
      conn->sendpos += msg->len;

      ret = rpmsg_send_nocopy(&conn->ept, msg, block + sizeof(*msg));
      if (ret < 0)
        {
          break;
        }

      written += block;
      cur     += block;
    }

  rpmsg_socket_unlock(&conn->sendlock);

  return written ? written : ret;
}

static ssize_t rpmsg_socket_send_single(FAR struct socket *psock,
                                        FAR const void *buf, size_t len)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  FAR struct rpmsg_socket_data_s *msg;
  uint32_t total = len + sizeof(*msg) + sizeof(uint32_t);
  uint32_t ipcsize;
  int ret;

  if (total > conn->sendsize)
    {
      return -EFBIG;
    }

  rpmsg_socket_lock(&conn->sendlock);

  while (total - sizeof(*msg) > rpmsg_socket_get_space(conn))
    {
      if (!_SS_ISNONBLOCK(psock->s_flags))
        {
          rpmsg_socket_unlock(&conn->sendlock);

          ret = net_timedwait(&conn->sendsem,
                              _SO_TIMEOUT(psock->s_sndtimeo));
          if (!conn->ept.rdev)
            {
              ret = -ECONNRESET;
            }

          rpmsg_socket_lock(&conn->sendlock);

          if (ret < 0)
            {
              goto out;
            }
        }
      else
        {
          ret = -EAGAIN;
          goto out;
        }
    }

  msg = rpmsg_get_tx_payload_buffer(&conn->ept, &ipcsize, true);
  if (!msg)
    {
      ret = -EINVAL;
      goto out;
    }

  if (total > ipcsize)
    {
      total = ipcsize;
      len   = ipcsize - sizeof(*msg) + sizeof(uint32_t);
    }

  /* SOCK_DGRAM need write len to buffer */

  msg->cmd = RPMSG_SOCKET_CMD_DATA;
  msg->pos = conn->recvpos;
  msg->len = len;
  memcpy(msg->data, &len, sizeof(uint32_t));
  memcpy(msg->data + sizeof(uint32_t), buf, len);

  conn->lastpos  = conn->recvpos;
  conn->sendpos += len + sizeof(uint32_t);

  ret = rpmsg_send_nocopy(&conn->ept, msg, total);
out:
  rpmsg_socket_unlock(&conn->sendlock);
  return ret > 0 ? len : ret;
}

static ssize_t rpmsg_socket_send_internal(FAR struct socket *psock,
                                          FAR const void *buf,
                                          size_t len, int flags)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;

  if (!conn->ept.rdev)
    {
      /* return ECONNRESET if lower IPC closed */

      return -ECONNRESET;
    }

  if (psock->s_type == SOCK_STREAM)
    {
      return rpmsg_socket_send_continuous(psock, buf, len);
    }
  else
    {
      return rpmsg_socket_send_single(psock, buf, len);
    }
}

static ssize_t rpmsg_socket_sendmsg(FAR struct socket *psock,
                                    FAR struct msghdr *msg, int flags)
{
  const FAR void *buf = msg->msg_iov->iov_base;
  size_t len = msg->msg_iov->iov_len;
  const FAR struct sockaddr *to = msg->msg_name;
  socklen_t tolen = msg->msg_namelen;
  ssize_t ret;

  /* Validity check, only single iov supported */

  if (msg->msg_iovlen != 1)
    {
      return -ENOTSUP;
    }

  if (!_SS_ISCONNECTED(psock->s_flags))
    {
      if (to == NULL)
        {
          return -ENOTCONN;
        }

      ret = rpmsg_socket_connect(psock, to, tolen);
      if (ret < 0)
        {
          return ret;
        }
    }

  return rpmsg_socket_send_internal(psock, buf, len, flags);
}

static ssize_t rpmsg_socket_recvmsg(FAR struct socket *psock,
                                    FAR struct msghdr *msg, int flags)
{
  FAR void *buf = msg->msg_iov->iov_base;
  size_t len = msg->msg_iov->iov_len;
  FAR struct sockaddr *from = msg->msg_name;
  FAR socklen_t *fromlen = &msg->msg_namelen;
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  ssize_t ret;

  if (psock->s_type == SOCK_DGRAM && _SS_ISBOUND(psock->s_flags)
          && !_SS_ISCONNECTED(psock->s_flags))
    {
      ret = rpmsg_socket_connect_internal(psock);
      if (ret < 0)
        {
          return ret;
        }

      psock->s_flags |= _SF_CONNECTED;
    }

  if (!_SS_ISCONNECTED(psock->s_flags))
    {
      return -EISCONN;
    }

  rpmsg_socket_lock(&conn->recvlock);

  if (psock->s_type == SOCK_DGRAM)
    {
      uint32_t datalen;

      ret = circbuf_read(&conn->recvbuf, &datalen, sizeof(uint32_t));
      if (ret > 0)
        {
          ret = circbuf_read(&conn->recvbuf, buf, MIN(datalen, len));
          if (ret > 0 && datalen > ret)
            {
              circbuf_skip(&conn->recvbuf, datalen - ret);
            }

          conn->recvpos += datalen + sizeof(uint32_t);
        }
    }
  else
    {
      ret = circbuf_read(&conn->recvbuf, buf, len);
      conn->recvpos +=  ret > 0 ? ret : 0;
    }

  if (ret > 0)
    {
      rpmsg_socket_wakeup(conn);
      goto out;
    }

  if (!conn->ept.rdev)
    {
      /* return EOF if lower IPC closed */

      ret = 0;
      goto out;
    }

  if (_SS_ISNONBLOCK(psock->s_flags))
    {
      ret = -EAGAIN;
      goto out;
    }

  conn->recvdata = buf;
  conn->recvlen  = len;

  rpmsg_socket_unlock(&conn->recvlock);

  ret = net_timedwait(&conn->recvsem, _SO_TIMEOUT(psock->s_rcvtimeo));
  if (!conn->ept.rdev)
    {
      ret = -ECONNRESET;
    }

  rpmsg_socket_lock(&conn->recvlock);

  if (!conn->recvdata)
    {
      ret = conn->recvlen;
      rpmsg_socket_wakeup(conn);
    }
  else
    {
      conn->recvdata = NULL;
    }

out:
  rpmsg_socket_unlock(&conn->recvlock);

  if (ret > 0)
    {
      rpmsg_socket_getaddr(conn, from, fromlen);
    }

  return ret;
}

static int rpmsg_socket_close(FAR struct socket *psock)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;

  if (conn->crefs > 1)
    {
      conn->crefs--;
      return 0;
    }

  if (conn->backlog)
    {
      rpmsg_unregister_callback(conn,
                                NULL,
                                NULL,
                                rpmsg_socket_ns_bind);
    }
  else
    {
      rpmsg_unregister_callback(conn,
                                rpmsg_socket_device_created,
                                rpmsg_socket_device_destroy,
                                NULL);
    }

  rpmsg_socket_destroy_ept(conn);
  rpmsg_socket_free(conn);
  return 0;
}
