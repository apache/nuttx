/****************************************************************************
 * net/rpmsg/rpmsg_sockif.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <limits.h>
#include <nuttx/debug.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/socket.h>

#include <nuttx/atomic.h>
#include <nuttx/kmalloc.h>
#include <nuttx/circbuf.h>
#include <nuttx/crc32.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/mutex.h>
#include <nuttx/percpu.h>
#include <nuttx/poll.h>
#include <nuttx/queue.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/ioctl.h>

#include <netinet/in.h>
#include <netpacket/rpmsg.h>

#include "socket/socket.h"
#include "utils/utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_SOCKET_CMD_SYNC           1
#define RPMSG_SOCKET_CMD_DATA           2
#define RPMSG_SOCKET_CMD_SHUTDOWN       3
#define RPMSG_SOCKET_NAME_PREFIX        "s:"
#define RPMSG_SOCKET_NAME_PREFIX_LEN    2
#define RPMSG_SOCKET_NAME_LEN           10

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct rpmsg_socket_sync_s
{
  uint32_t                       cmd;
  uint32_t                       size;
  uint32_t                       pid;
  uint32_t                       uid;
  uint32_t                       gid;
} end_packed_struct;

begin_packed_struct struct rpmsg_socket_data_s
{
  uint32_t                       cmd;
  uint32_t                       pos;

  /* Act data len, don't include len itself when SOCK_DGRAM */

  uint32_t                       len;
  uint8_t                        data[0];
} end_packed_struct;

begin_packed_struct struct rpmsg_socket_shutdown_s
{
  uint32_t                       cmd;
  uint32_t                       how;
} end_packed_struct;

struct rpmsg_socket_conn_s
{
  /* Common prologue of all connection structures. */

  struct socket_conn_s           sconn;

  atomic_t                       unbind;
  struct rpmsg_endpoint          ept;

  struct sockaddr_rpmsg          rpaddr;
  char                           nameid[RPMSG_SOCKET_NAME_LEN];
  uint16_t                       crefs;
  uint32_t                       how;

  FAR struct pollfd_s            *fds[CONFIG_NET_RPMSG_NPOLLWAITERS];
  mutex_t                        polllock;

  sem_t                          sendsem;
  mutex_t                        sendlock;

  sem_t                          recvsem;
  mutex_t                        recvlock;

  FAR void                       *recvdata;
  uint32_t                       recvlen;
  FAR struct circbuf_s           recvbuf;

  FAR struct rpmsg_socket_conn_s *server;
  FAR struct rpmsg_socket_conn_s *next;

  /* Connections that have been created by rpmsg_socket_ns_bind() but have
   * not yet sent their SYNC message, so are not on the accept queue yet.
   * The listening socket owns this list so that it can detach them if it
   * is closed first.  Only valid on a listening socket (backlog > 0).
   */

  FAR struct rpmsg_socket_conn_s *pendlist;
  FAR struct rpmsg_socket_conn_s *pendnext;

  /* Entry in g_rpmsg_socket_server while the socket is listening */

  struct dq_entry_s              snode;

  /* server listen-scoket listening: backlog > 0;
   * others: backlog = 0;
   * pending is current number of pending connections;
   */

  int                            backlog;
  int                            pending;

  /* The remote connection's credentials */

  struct ucred                   cred;

  /* Recvbuf size */

  uint32_t                       recvsize;

  /* Flow control, descript send side */

  uint32_t                       sendsize;
  uint32_t                       sendpos;
  uint32_t                       ackpos;

  /* Flow control, descript recv side */

  uint32_t                       recvpos;
  uint32_t                       lastpos;

  uint8_t                        priority;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        rpmsg_socket_setup(FAR struct socket *psock);
static sockcaps_t rpmsg_socket_sockcaps(FAR struct socket *psock);
static void       rpmsg_socket_pend_add(
                    FAR struct rpmsg_socket_conn_s *server,
                    FAR struct rpmsg_socket_conn_s *conn);
static bool       rpmsg_socket_pend_remove(
                    FAR struct rpmsg_socket_conn_s *server,
                    FAR struct rpmsg_socket_conn_s *conn);
static void       rpmsg_socket_addref(FAR struct socket *psock);
static int        rpmsg_socket_bind(FAR struct socket *psock,
                                    FAR const struct sockaddr *addr,
                                    socklen_t addrlen);
static int        rpmsg_socket_getsockname(FAR struct socket *psock,
                                           FAR struct sockaddr *addr,
                                           FAR socklen_t *addrlen);
static int        rpmsg_socket_getpeername(FAR struct socket *psock,
                                           FAR struct sockaddr *addr,
                                           FAR socklen_t *addrlen);
static int        rpmsg_socket_listen(FAR struct socket *psock, int backlog);
static int        rpmsg_socket_connect(FAR struct socket *psock,
                                       FAR const struct sockaddr *addr,
                                       socklen_t addrlen);
static int        rpmsg_socket_accept(FAR struct socket *psock,
                                      FAR struct sockaddr *addr,
                                      FAR socklen_t *addrlen,
                                      FAR struct socket *newsock, int flags);
static int        rpmsg_socket_poll(FAR struct socket *psock,
                                    FAR struct pollfd_s *fds, bool setup);
static ssize_t    rpmsg_socket_sendmsg(FAR struct socket *psock,
                                       FAR const struct msghdr *msg,
                                       int flags);
static ssize_t    rpmsg_socket_recvmsg(FAR struct socket *psock,
                                       FAR struct msghdr *msg, int flags);
static int        rpmsg_socket_close(FAR struct socket *psock);
static int        rpmsg_socket_ioctl(FAR struct socket *psock,
                                     int cmd, unsigned long arg);
static int        rpmsg_socket_shutdown(FAR struct socket *psock, int how);
static int        rpmsg_socket_mmap(FAR struct socket *psock,
                                    FAR struct mm_map_entry_s *mmap);
#ifdef CONFIG_NET_SOCKOPTS
static int        rpmsg_socket_getsockopt(FAR struct socket *psock,
                                          int level, int option,
                                          FAR void *value,
                                          FAR socklen_t *value_len);
static int        rpmsg_socket_setsockopt(FAR struct socket *psock,
                                          int level, int option,
                                          FAR const void *value,
                                          socklen_t value_len);
#endif

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
  rpmsg_socket_getpeername, /* si_getpeername */
  rpmsg_socket_listen,      /* si_listen */
  rpmsg_socket_connect,     /* si_connect */
  rpmsg_socket_accept,      /* si_accept */
  rpmsg_socket_poll,        /* si_poll */
  rpmsg_socket_sendmsg,     /* si_sendmsg */
  rpmsg_socket_recvmsg,     /* si_recvmsg */
  rpmsg_socket_close,       /* si_close */
  rpmsg_socket_ioctl,       /* si_ioctl */
  NULL,                     /* si_socketpair */
  rpmsg_socket_shutdown     /* si_shutdown */
#ifdef CONFIG_NET_SOCKOPTS
  , rpmsg_socket_getsockopt /* si_getsockopt */
  , rpmsg_socket_setsockopt /* si_setsockopt */
#endif
#ifdef CONFIG_NET_SENDFILE
  , NULL                    /* si_sendfile */
#endif
  , rpmsg_socket_mmap       /* si_mmap */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static DEFINE_PER_CPU_BSS_BMP(atomic_t, g_rpmsg_id);
#define g_rpmsg_id this_cpu_var_bmp(g_rpmsg_id)

/* Listening sockets.  rpmsg_socket_ns_bind() holds the lock while it uses
 * the server it found, so that close() cannot free that server meanwhile.
 */

static DEFINE_PER_CPU_BMP(mutex_t, g_rpmsg_socket_lock) =
  NXMUTEX_INITIALIZER;
#define g_rpmsg_socket_lock this_cpu_var_bmp(g_rpmsg_socket_lock)

static DEFINE_PER_CPU_BSS_BMP(dq_queue_t, g_rpmsg_socket_server);
#define g_rpmsg_socket_server this_cpu_var_bmp(g_rpmsg_socket_server)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void rpmsg_socket_post(FAR sem_t *sem)
{
  int semcount;

  nxsem_get_value(sem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(sem);
    }
}

static inline void
rpmsg_socket_poll_notify(FAR struct rpmsg_socket_conn_s *conn,
                         pollevent_t eventset)
{
  nxmutex_lock(&conn->polllock);
  poll_notify(conn->fds, CONFIG_NET_RPMSG_NPOLLWAITERS, eventset);
  nxmutex_unlock(&conn->polllock);
}

static FAR struct rpmsg_socket_conn_s *rpmsg_socket_alloc(void)
{
  FAR struct rpmsg_socket_conn_s *conn;
  int ret;

  conn = kmm_zalloc(sizeof(struct rpmsg_socket_conn_s));
  if (!conn)
    {
      return NULL;
    }

  ret = circbuf_init(&conn->recvbuf, NULL, 0);
  if (ret < 0)
    {
      kmm_free(conn);
      return NULL;
    }

  conn_init(&conn->sconn);
  nxmutex_init(&conn->polllock);
  nxmutex_init(&conn->sendlock);
  nxmutex_init(&conn->recvlock);
  nxsem_init(&conn->sendsem, 0, 0);
  nxsem_init(&conn->recvsem, 0, 0);

  conn->recvsize = CONFIG_NET_RPMSG_RXBUF_SIZE;
  conn->crefs = 1;
  conn->priority = CONFIG_NET_RPMSG_EPT_PRIORITY;
  return conn;
}

static void rpmsg_socket_free(FAR struct rpmsg_socket_conn_s *conn)
{
  circbuf_uninit(&conn->recvbuf);

  nxmutex_destroy(&conn->polllock);
  nxmutex_destroy(&conn->recvlock);
  nxmutex_destroy(&conn->sendlock);
  nxsem_destroy(&conn->sendsem);
  nxsem_destroy(&conn->recvsem);
  conn_uninit(&conn->sconn);

  kmm_free(conn);
}

static int rpmsg_socket_wakeup(FAR struct rpmsg_socket_conn_s *conn)
{
  FAR struct rpmsg_socket_data_s *msg;
  uint32_t len = sizeof(*msg);
  uint32_t space;
  int ret;

  if (!conn->ept.rdev || atomic_read(&conn->unbind))
    {
      return 0;
    }

  nxmutex_lock(&conn->recvlock);
  space = conn->recvpos - conn->lastpos;
  nxmutex_unlock(&conn->recvlock);

  if (space <= circbuf_size(&conn->recvbuf) / 2)
    {
      return 0;
    }

  /* Reserve the tx buffer without holding recvlock, since it may block
   * waiting for a free vring buffer.
   */

  msg = rpmsg_get_tx_payload_buffer(&conn->ept, &len, true);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len >= sizeof(*msg));
  nxmutex_lock(&conn->recvlock);

  /* The peer may have destroyed its endpoint meanwhile.  rpmsg_send()
   * would then wait the whole RPMSG_TICK_COUNT for dest_addr to become
   * valid again, so check and send under recvlock, which
   * rpmsg_socket_ns_unbind() takes too.
   */

  if (!is_rpmsg_ept_ready(&conn->ept) || atomic_read(&conn->unbind))
    {
      nxmutex_unlock(&conn->recvlock);
      rpmsg_release_tx_buffer(&conn->ept, msg);
      return -ECONNRESET;
    }

  conn->lastpos = conn->recvpos;
  msg->cmd = RPMSG_SOCKET_CMD_DATA;
  msg->pos = conn->recvpos;
  msg->len = 0;
  ret = rpmsg_send_offchannel_nocopy(&conn->ept, conn->ept.addr,
                                     conn->ept.dest_addr, msg,
                                     sizeof(*msg));

  nxmutex_unlock(&conn->recvlock);
  return ret;
}

static inline uint32_t
rpmsg_socket_get_space(FAR struct rpmsg_socket_conn_s *conn)
{
  return conn->sendsize - (conn->sendpos - conn->ackpos);
}

static int rpmsg_socket_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len, uint32_t src,
                               FAR void *priv)
{
  FAR struct rpmsg_socket_conn_s *conn = ept->priv;
  FAR struct rpmsg_socket_sync_s *head = data;
  FAR struct rpmsg_socket_conn_s *server;

  if (head->cmd == RPMSG_SOCKET_CMD_SYNC)
    {
      nxmutex_lock(&conn->sendlock);
      conn->sendsize = head->size;
      conn->cred.pid = head->pid;
      conn->cred.uid = head->uid;
      conn->cred.gid = head->gid;

      conn->sconn.s_flags &= ~_SF_CONNECTING;
      conn->sconn.s_flags |= _SF_CONNECTED;
      _SO_CONN_SETERRNO(conn, OK);

      /* SYNC means the peer (re)established the connection.  Clear any
       * stale unbind flag left by a previous NS_DESTROY so that connect()
       * does not wrongly fail.
       */

      atomic_set_release(&conn->unbind, 0);
      rpmsg_socket_post(&conn->sendsem);
      rpmsg_socket_poll_notify(conn, POLLOUT);
      nxmutex_unlock(&conn->sendlock);

      /* Accept new connection and complete the final handshake.
       *
       * conn->server is only valid while the listening socket still owns
       * this connection on its pending list.  Take recvlock and re-check
       * that we are still on that list: if the listener was closed (or the
       * remote unbound) in the meantime it has already detached us and
       * conn->server is NULL.
       */

      nxmutex_lock(&conn->recvlock);
      server = conn->server;
      nxmutex_unlock(&conn->recvlock);

      if (server != NULL)
        {
          FAR struct rpmsg_socket_conn_s *tmp;

          nxmutex_lock(&server->recvlock);
          if (!rpmsg_socket_pend_remove(server, conn))
            {
              /* Already detached by the listener. */

              nxmutex_unlock(&server->recvlock);
              return 0;
            }

          for (tmp = server; tmp->next; tmp = tmp->next);
          tmp->next = conn;
          rpmsg_socket_post(&server->recvsem);
          rpmsg_socket_poll_notify(server, POLLIN);
          nxmutex_unlock(&server->recvlock);

          /* The listener no longer owns this connection, so drop the back
           * pointer:  the listening socket may be closed and freed at any
           * time from now on, and rpmsg_socket_ns_unbind() must not take
           * its recvlock afterwards.
           */

          nxmutex_lock(&conn->recvlock);
          conn->server = NULL;
          nxmutex_unlock(&conn->recvlock);
        }
    }
  else if (head->cmd == RPMSG_SOCKET_CMD_DATA)
    {
      FAR struct rpmsg_socket_data_s *msg = data;
      FAR uint8_t *buf = msg->data;

      nxmutex_lock(&conn->sendlock);
      conn->ackpos = msg->pos;
      if (rpmsg_socket_get_space(conn) > 0)
        {
          rpmsg_socket_post(&conn->sendsem);
          rpmsg_socket_poll_notify(conn, POLLOUT);
        }

      nxmutex_unlock(&conn->sendlock);
      if (len > sizeof(*msg))
        {
          len -= sizeof(*msg);
          DEBUGASSERT(len == msg->len || len == msg->len + sizeof(uint32_t));

          nxmutex_lock(&conn->recvlock);
          if (conn->recvdata)
            {
              conn->recvlen = MIN(conn->recvlen, msg->len);
              if (len == msg->len)
                {
                  /* SOCK_STREAM */

                  conn->recvpos += conn->recvlen;
                  memcpy(conn->recvdata, buf, conn->recvlen);
                  buf += conn->recvlen;
                  len -= conn->recvlen;
                }
              else
                {
                  /* SOCK_DGRAM */

                  conn->recvpos += len;
                  memcpy(conn->recvdata, buf + sizeof(uint32_t),
                         conn->recvlen);
                  len = 0;
                }

              conn->recvdata = NULL;
              rpmsg_socket_post(&conn->recvsem);
            }

          if (len > 0)
            {
              ssize_t written;

              written = circbuf_write(&conn->recvbuf, buf, len);
              if (written != len)
                {
                  nerr("circbuf_write overflow, %zu, %zu\n", written, len);
                }

              rpmsg_socket_poll_notify(conn, POLLIN);
            }

          nxmutex_unlock(&conn->recvlock);
        }
    }
  else if (head->cmd == RPMSG_SOCKET_CMD_SHUTDOWN)
    {
      FAR struct rpmsg_socket_shutdown_s *msg = data;

      if (msg->how & SHUT_WR)
        {
          nxmutex_lock(&conn->recvlock);
          conn->how |= SHUT_RD;
          rpmsg_socket_post(&conn->recvsem);
          rpmsg_socket_poll_notify(conn, POLLIN | POLLHUP);
          nxmutex_unlock(&conn->recvlock);
        }

      if (msg->how & SHUT_RD)
        {
          nxmutex_lock(&conn->sendlock);
          conn->how |= SHUT_WR;
          rpmsg_socket_post(&conn->sendsem);
          rpmsg_socket_poll_notify(conn, POLLOUT | POLLHUP);
          nxmutex_unlock(&conn->sendlock);
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

  nxmutex_lock(&conn->recvlock);
  nxmutex_lock(&conn->sendlock);

  if (conn->ept.rdev)
    {
      rpmsg_socket_post(&conn->sendsem);
      rpmsg_socket_post(&conn->recvsem);
      rpmsg_socket_poll_notify(conn, POLLIN | POLLOUT);
    }

  nxmutex_unlock(&conn->sendlock);
  nxmutex_unlock(&conn->recvlock);
  rpmsg_destroy_ept(&conn->ept);
}

/****************************************************************************
 * Name: rpmsg_socket_pend_add
 *
 * Description:
 *   Track a connection that is waiting for its SYNC message on the
 *   listening socket, so that the listener can detach it if it is closed
 *   before the SYNC arrives.
 *
 * Assumptions:
 *   server->recvlock is held by the caller.
 *
 ****************************************************************************/

static void rpmsg_socket_pend_add(FAR struct rpmsg_socket_conn_s *server,
                                  FAR struct rpmsg_socket_conn_s *conn)
{
  conn->pendnext   = server->pendlist;
  server->pendlist = conn;
}

/****************************************************************************
 * Name: rpmsg_socket_pend_remove
 *
 * Description:
 *   Take a connection off the listening socket's waiting-for-SYNC list.
 *   Returns true if it was found, i.e. if the listener had not already
 *   detached it.
 *
 * Assumptions:
 *   server->recvlock is held by the caller.
 *
 ****************************************************************************/

static bool rpmsg_socket_pend_remove(FAR struct rpmsg_socket_conn_s *server,
                                     FAR struct rpmsg_socket_conn_s *conn)
{
  FAR struct rpmsg_socket_conn_s **pp;

  for (pp = &server->pendlist; *pp != NULL; pp = &(*pp)->pendnext)
    {
      if (*pp == conn)
        {
          *pp = conn->pendnext;
          conn->pendnext = NULL;
          return true;
        }
    }

  return false;
}

static void rpmsg_socket_ns_bound(struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_socket_conn_s *conn = ept->priv;
  FAR struct rpmsg_socket_sync_s *msg;
  uint32_t len = sizeof(*msg);

  if (!ept->rdev)
    {
      return;
    }

  /* Reserve the tx buffer without holding recvlock, since it may block
   * waiting for a free vring buffer.
   */

  msg = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (msg == NULL)
    {
      return;
    }

  DEBUGASSERT(len >= sizeof(*msg));

  msg->cmd = RPMSG_SOCKET_CMD_SYNC;
  msg->pid = nxsched_getpid();
  msg->uid = getuid();
  msg->gid = getgid();

  nxmutex_lock(&conn->recvlock);

  /* This runs in the rx callback context, where a blocking rpmsg_send()
   * on an endpoint unregistered meanwhile would wait the whole
   * RPMSG_TICK_COUNT and stall every inbound message.
   *
   * conn->unbind is not checked:  a stale flag left by an earlier
   * NS_DESTROY is cleared only when a SYNC is received, so bailing out on
   * it would leave a reconnecting peer without our recvbuf size.
   */

  if (!is_rpmsg_ept_ready(ept))
    {
      nxmutex_unlock(&conn->recvlock);
      rpmsg_release_tx_buffer(ept, msg);
      return;
    }

  msg->size = circbuf_size(&conn->recvbuf);
  rpmsg_send_offchannel_nocopy(ept, ept->addr, ept->dest_addr,
                               msg, sizeof(*msg));

  nxmutex_unlock(&conn->recvlock);
}

static void rpmsg_socket_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_socket_conn_s *conn = ept->priv;
  FAR struct rpmsg_socket_conn_s *server;

  nxmutex_lock(&conn->recvlock);
  server = conn->server;
  conn->server = NULL;
  atomic_set_release(&conn->unbind, 1);
  rpmsg_socket_post(&conn->sendsem);
  rpmsg_socket_post(&conn->recvsem);
  rpmsg_socket_poll_notify(conn, POLLIN | POLLOUT | POLLHUP | POLLERR);

  nxmutex_unlock(&conn->recvlock);

  /* If the remote unbound before sending its SYNC, this connection is
   * still owned by the listener and will never be accepted.  Release its
   * backlog slot and drop it.
   */

  if (server != NULL)
    {
      nxmutex_lock(&server->recvlock);
      if (rpmsg_socket_pend_remove(server, conn))
        {
          server->pending--;
          nxmutex_unlock(&server->recvlock);

          /* conn->ept is still registered:  openamp drops its reference
           * after this callback returns, and release_cb then frees the
           * connection.  Clear crefs so that it actually does, and do not
           * free here.
           */

          conn->crefs = 0;
          rpmsg_destroy_ept(&conn->ept);
          return;
        }

      nxmutex_unlock(&server->recvlock);
    }
}

static void rpmsg_socket_ept_release(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_socket_conn_s *conn = ept->priv;

  /* Detach the ept from its rpmsg device: the conn may outlive the device
   * when a file descriptor is still open, and a stale rdev would keep all
   * the "if (conn->ept.rdev)" guards passing on freed memory.
   */

  ept->rdev = NULL;

  /* Release conn only when close is called */

  if (!conn->crefs)
    {
      rpmsg_socket_free(conn);
    }
}

static void rpmsg_socket_format_name(FAR struct rpmsg_socket_conn_s *conn,
                                     FAR char *namebuf)
{
  if (strlen(conn->rpaddr.rp_name) > RPMSG_SOCKET_NAME_LEN)
    {
      uint32_t crc = crc32((const uint8_t *)conn->rpaddr.rp_name,
                           strlen(conn->rpaddr.rp_name));

      snprintf(namebuf, RPMSG_NAME_SIZE, "%s%.2s%08" PRIx32 "%s",
               RPMSG_SOCKET_NAME_PREFIX, conn->rpaddr.rp_name,
               crc, conn->nameid);
    }
  else
    {
      snprintf(namebuf, RPMSG_NAME_SIZE, "%s%.*s%s",
               RPMSG_SOCKET_NAME_PREFIX, RPMSG_SOCKET_NAME_LEN,
               conn->rpaddr.rp_name, conn->nameid);
    }
}

static void rpmsg_socket_device_created(FAR struct rpmsg_device *rdev,
                                        FAR void *priv)
{
  FAR struct rpmsg_socket_conn_s *conn = priv;
  char namebuf[RPMSG_NAME_SIZE];

  if (conn->ept.rdev)
    {
      return;
    }

  if (strcmp(conn->rpaddr.rp_cpu, rpmsg_get_cpuname(rdev)) == 0)
    {
      conn->ept.priv = conn;
      conn->ept.ns_bound_cb = rpmsg_socket_ns_bound;
      conn->ept.release_cb = rpmsg_socket_ept_release;
      rpmsg_socket_format_name(conn, namebuf);

      rpmsg_set_priority(&conn->ept, conn->priority);
      rpmsg_create_ept(&conn->ept, rdev, namebuf,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       rpmsg_socket_ept_cb, rpmsg_socket_ns_unbind);
    }
}

static bool rpmsg_socket_ns_match(FAR struct rpmsg_device *rdev,
                                  FAR void *priv, FAR const char *name,
                                  uint32_t dest)
{
  /* Only a stream client names its endpoint with a ":<id>" suffix.  The
   * two ends of a datagram socket use the same name, and the first
   * announcement has to wait in the rpmsg bind list until the other end
   * creates its endpoint, so leave those alone.
   */

  return strncmp(name, RPMSG_SOCKET_NAME_PREFIX,
                 RPMSG_SOCKET_NAME_PREFIX_LEN) == 0 &&
         strchr(name + RPMSG_SOCKET_NAME_PREFIX_LEN, ':') != NULL;
}

/****************************************************************************
 * Name: rpmsg_socket_find_server
 *
 * Assumptions:
 *   g_rpmsg_socket_lock is held by the caller.
 *
 ****************************************************************************/

static FAR struct rpmsg_socket_conn_s *
rpmsg_socket_find_server(FAR struct rpmsg_device *rdev,
                         FAR const char *name)
{
  FAR struct rpmsg_socket_conn_s *server;
  char namebuf[RPMSG_NAME_SIZE];

  dq_for_every_entry(&g_rpmsg_socket_server, server,
                     struct rpmsg_socket_conn_s, snode)
    {
      rpmsg_socket_format_name(server, namebuf);
      if (strncmp(name, namebuf, strlen(namebuf)))
        {
          continue;
        }

      if (server->rpaddr.rp_cpu[0] &&
          strcmp(server->rpaddr.rp_cpu, rpmsg_get_cpuname(rdev)))
        {
          /* Bind specific CPU, then only listen that CPU */

          continue;
        }

      return server;
    }

  return NULL;
}

static void rpmsg_socket_ns_destroy(FAR struct rpmsg_device *rdev,
                                    FAR const char *name,
                                    uint32_t dest)
{
  struct rpmsg_ns_msg ns_msg;

  strlcpy(ns_msg.name, name, RPMSG_NAME_SIZE);
  ns_msg.addr  = RPMSG_ADDR_ANY;
  ns_msg.flags = RPMSG_NS_DESTROY;

  rpmsg_send_offchannel_raw(&rdev->ns_ept, rdev->ns_ept.addr,
                            RPMSG_NS_EPT_ADDR, &ns_msg, sizeof(ns_msg),
                            true);
}

static void rpmsg_socket_ns_accept(FAR struct rpmsg_socket_conn_s *server,
                                   FAR struct rpmsg_device *rdev,
                                   FAR const char *name, uint32_t dest)
{
  FAR struct rpmsg_socket_conn_s *new;
  int ret;

  new = rpmsg_socket_alloc();
  if (!new)
    {
      return;
    }

  ret = circbuf_resize(&new->recvbuf, server->recvsize);
  if (ret < 0)
    {
      rpmsg_socket_free(new);
      return;
    }

  new->server   = server;
  new->priority = server->priority;
  new->ept.priv = new;
  new->ept.release_cb = rpmsg_socket_ept_release;

  /* Take the backlog slot and join the listener's pending list before the
   * endpoint exists.  rpmsg_create_ept() sends the NS ack, after which the
   * remote may answer with its SYNC at once;  the handler must already be
   * able to find this connection on the list.
   */

  nxmutex_lock(&server->recvlock);
  if (server->pending >= server->backlog)
    {
      /* Reject the connection.  The endpoint has to be created even so:
       * destroying it is what sends NS_DESTROY to the client, which is how
       * the client learns the connection was refused.  Without it connect()
       * would block until its send timeout expires.  Drop the back pointer
       * first, this connection never joins the listener's pending list.
       */

      nerr("Reject the connection from %s (ept:%s), max listen %d\n",
           rpmsg_get_cpuname(rdev), name, server->backlog);
      nxmutex_unlock(&server->recvlock);

      new->server = NULL;
      if (rpmsg_create_ept(&new->ept, rdev, name, RPMSG_ADDR_ANY, dest,
                           rpmsg_socket_ept_cb,
                           rpmsg_socket_ns_unbind) >= 0)
        {
          rpmsg_destroy_ept(&new->ept);
        }

      rpmsg_socket_free(new);
      return;
    }

  /* The slot is released by accept(), by rpmsg_socket_ns_unbind() or when
   * the listening socket is closed.
   */

  server->pending++;
  rpmsg_socket_pend_add(server, new);
  nxmutex_unlock(&server->recvlock);

  /* The accepted connection inherits the listener's priority, so that
   * getsockopt(SO_PRIORITY) on it reports the priority actually in use.
   */

  ret = rpmsg_set_priority(&new->ept, new->priority);
  if (ret < 0)
    {
      goto errout_with_pending;
    }

  ret = rpmsg_create_ept(&new->ept, rdev, name,
                         RPMSG_ADDR_ANY, dest,
                         rpmsg_socket_ept_cb, rpmsg_socket_ns_unbind);
  if (ret < 0)
    {
      goto errout_with_pending;
    }

  new->rpaddr.rp_family = AF_RPMSG;
  strlcpy(new->rpaddr.rp_cpu, rpmsg_get_cpuname(rdev),
          sizeof(new->rpaddr.rp_cpu));
  strlcpy(new->rpaddr.rp_name, name + RPMSG_SOCKET_NAME_PREFIX_LEN,
          sizeof(new->rpaddr.rp_name));

  rpmsg_socket_ns_bound(&new->ept);
  return;

errout_with_pending:

  /* The endpoint was never created, so nothing will call release_cb.
   * Give the backlog slot back and drop the connection.
   */

  nxmutex_lock(&server->recvlock);
  if (rpmsg_socket_pend_remove(server, new))
    {
      server->pending--;
    }

  nxmutex_unlock(&server->recvlock);
  rpmsg_socket_free(new);
}

static void rpmsg_socket_ns_bind(FAR struct rpmsg_device *rdev,
                                 FAR void *priv, FAR const char *name,
                                 uint32_t dest)
{
  FAR struct rpmsg_socket_conn_s *server;

  nxmutex_lock(&g_rpmsg_socket_lock);
  server = rpmsg_socket_find_server(rdev, name);
  if (server != NULL)
    {
      rpmsg_socket_ns_accept(server, rdev, name, dest);
    }
  else
    {
      /* Nobody listens on this name:  refuse the connection, so that the
       * client fails with ECONNREFUSED instead of waiting for a listener.
       */

      rpmsg_socket_ns_destroy(rdev, name, dest);
    }

  nxmutex_unlock(&g_rpmsg_socket_lock);
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
      addrlen < sizeof(struct sockaddr_rpmsg))
    {
      return -EINVAL;
    }

  memcpy(&conn->rpaddr, rpaddr, sizeof(struct sockaddr_rpmsg));

  if (suffix)
    {
      snprintf(conn->nameid, sizeof(conn->nameid), ":%" PRIx32,
               atomic_add(&g_rpmsg_id, 1));
    }
  else
    {
      conn->nameid[0] = 0;
    }

  return 0;
}

static int rpmsg_socket_setup(FAR struct socket *psock)
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
  int ret;

  ret = rpmsg_socket_getaddr(psock->s_conn, addr, addrlen);
  if (ret >= 0)
    {
      strlcpy(((struct sockaddr_rpmsg *)addr)->rp_cpu,
              CONFIG_RPMSG_LOCAL_CPUNAME, RPMSG_SOCKET_CPU_SIZE);
    }

  return ret;
}

static int rpmsg_socket_getpeername(FAR struct socket *psock,
                                    FAR struct sockaddr *addr,
                                    FAR socklen_t *addrlen)
{
  return rpmsg_socket_getaddr(psock->s_conn, addr, addrlen);
}

static int rpmsg_socket_listen(FAR struct socket *psock, int backlog)
{
  FAR struct rpmsg_socket_conn_s *server = psock->s_conn;

  if (psock->s_type != SOCK_STREAM)
    {
      return -ENOSYS;
    }

  if (!_SS_ISBOUND(server->sconn.s_flags) || backlog <= 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&g_rpmsg_socket_lock);
  if (server->backlog == 0)
    {
      dq_addlast(&server->snode, &g_rpmsg_socket_server);
    }

  server->backlog = backlog;
  nxmutex_unlock(&g_rpmsg_socket_lock);
  return 0;
}

static int rpmsg_socket_connect_internal(FAR struct socket *psock)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  int ret;

  ret = circbuf_resize(&conn->recvbuf, conn->recvsize);
  if (ret < 0)
    {
      return ret;
    }

  ret = rpmsg_register_callback(conn,
                                rpmsg_socket_device_created,
                                NULL,
                                NULL,
                                NULL);
  if (ret < 0)
    {
      return ret;
    }

  nxmutex_lock(&conn->sendlock);
  if (conn->sendsize == 0)
    {
      nxmutex_unlock(&conn->sendlock);
      if (_SS_ISNONBLOCK(conn->sconn.s_flags))
        {
          return -EINPROGRESS;
        }

      if (!conn->ept.rdev)
        {
          return -ECONNREFUSED;
        }

      ret = net_sem_timedwait(&conn->sendsem,
                              _SO_TIMEOUT(conn->sconn.s_sndtimeo));
      if (!conn->ept.rdev || atomic_read(&conn->unbind))
        {
          ret = -ECONNRESET;
        }
    }
  else
    {
      nxmutex_unlock(&conn->sendlock);
    }

  return ret;
}

static int rpmsg_socket_connect(FAR struct socket *psock,
                                FAR const struct sockaddr *addr,
                                socklen_t addrlen)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  int ret;

  if (_SS_ISCONNECTED(conn->sconn.s_flags))
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
                               FAR struct socket *newsock,
                               int flags)
{
  FAR struct rpmsg_socket_conn_s *server = psock->s_conn;
  int ret = 0;

  if (!_SS_ISLISTENING(server->sconn.s_flags))
    {
      return -EINVAL;
    }

  while (1)
    {
      FAR struct rpmsg_socket_conn_s *conn = NULL;

      nxmutex_lock(&server->recvlock);

      if (server->next)
        {
          conn = server->next;
          server->next = conn->next;
          conn->next = NULL;
          server->pending--;
        }

      nxmutex_unlock(&server->recvlock);

      if (conn && conn->sendsize > 0)
        {
          newsock->s_domain = psock->s_domain;
          newsock->s_sockif = psock->s_sockif;
          newsock->s_type   = SOCK_STREAM;
          newsock->s_conn   = conn;

          rpmsg_socket_getaddr(conn, addr, addrlen);
          break;
        }
      else
        {
          if (_SS_ISNONBLOCK(server->sconn.s_flags))
            {
              ret = -EAGAIN;
              break;
            }
          else
            {
              ret = net_sem_wait(&server->recvsem);
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
                             FAR struct pollfd_s *fds, bool setup)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  pollevent_t eventset = 0;
  int i;

  if (setup)
    {
      nxmutex_lock(&conn->polllock);
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

      nxmutex_unlock(&conn->polllock);
      if (i >= CONFIG_NET_RPMSG_NPOLLWAITERS)
        {
          fds->priv = NULL;
          return -EBUSY;
        }

      /* Immediately notify on any of the requested events */

      if (_SS_ISLISTENING(conn->sconn.s_flags))
        {
          nxmutex_lock(&conn->recvlock);
          if (conn->next)
            {
              eventset |= POLLIN;
            }

          nxmutex_unlock(&conn->recvlock);
        }
      else if (_SS_ISCONNECTED(conn->sconn.s_flags))
        {
          if (!conn->ept.rdev || atomic_read(&conn->unbind))
            {
              eventset |= POLLHUP;
            }

          nxmutex_lock(&conn->sendlock);
          if (rpmsg_socket_get_space(conn) > 0)
            {
              eventset |= POLLOUT;
            }

          nxmutex_unlock(&conn->sendlock);

          nxmutex_lock(&conn->recvlock);
          if (!circbuf_is_empty(&conn->recvbuf))
            {
              eventset |= POLLIN;
            }

          nxmutex_unlock(&conn->recvlock);
        }
      else /* !_SS_ISCONNECTED(conn->sconn.s_flags) */
        {
          if (atomic_read(&conn->unbind) ||
              (!conn->ept.rdev && !_SS_INITD(conn->sconn.s_flags)))
            {
              eventset |= POLLHUP | POLLERR;
            }
        }

      nxmutex_lock(&conn->polllock);
      poll_notify(&fds, 1, eventset);
      nxmutex_unlock(&conn->polllock);
    }
  else
    {
      nxmutex_lock(&conn->polllock);
      if (fds->priv != NULL)
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

      nxmutex_unlock(&conn->polllock);
    }

  return OK;
}

static ssize_t rpmsg_socket_get_iovlen(FAR const struct iovec *buf,
                                       size_t iovcnt)
{
  size_t len = 0;

  while (iovcnt--)
    {
      if (buf->iov_len > MIN(SSIZE_MAX, UINT32_MAX) - len)
        {
          return -EINVAL;
        }

      len += buf->iov_len;
      buf++;
    }

  return len;
}

static ssize_t rpmsg_socket_send_continuous(FAR struct socket *psock,
                                            FAR const struct iovec *buf,
                                            size_t iovcnt, bool nonblock)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  ssize_t ret;
  uint32_t len;
  uint32_t written = 0;
  uint32_t offset = 0;

  ret = rpmsg_socket_get_iovlen(buf, iovcnt);
  if (ret < 0)
    {
      return ret;
    }

  len = ret;
  ret = 0;

  while (written < len)
    {
      FAR struct rpmsg_socket_data_s *msg;
      uint32_t block_written = 0;
      uint32_t ipcsize;
      uint32_t block;

      nxmutex_lock(&conn->sendlock);
      block = MIN(len - written, rpmsg_socket_get_space(conn));
      nxmutex_unlock(&conn->sendlock);

      if (block == 0)
        {
          if (nonblock)
            {
              ret = -EAGAIN;
              break;
            }

          ret = net_sem_timedwait(&conn->sendsem,
                                  _SO_TIMEOUT(conn->sconn.s_sndtimeo));
          if (!conn->ept.rdev || atomic_read(&conn->unbind))
            {
              ret = -ECONNRESET;
            }

          if (ret < 0)
            {
              break;
            }

          continue;
        }

      msg = rpmsg_get_tx_payload_buffer(&conn->ept, &ipcsize, true);
      if (!msg)
        {
          ret = -EINVAL;
          break;
        }

      nxmutex_lock(&conn->sendlock);
      block = MIN(len - written, rpmsg_socket_get_space(conn));
      block = MIN(block, ipcsize - sizeof(*msg));

      msg->cmd = RPMSG_SOCKET_CMD_DATA;
      msg->pos = conn->recvpos;
      msg->len = block;
      while (block_written < block)
        {
          uint32_t chunk = MIN(block - block_written, buf->iov_len - offset);

          memcpy(msg->data + block_written,
                 (FAR const uint8_t *)buf->iov_base + offset, chunk);
          offset += chunk;
          if (offset == buf->iov_len)
            {
              buf++;
              offset = 0;
            }

          block_written += chunk;
        }

      conn->lastpos  = conn->recvpos;
      conn->sendpos += msg->len;

      ret = rpmsg_sendto_nocopy(&conn->ept, msg, block + sizeof(*msg),
                                conn->ept.dest_addr);
      nxmutex_unlock(&conn->sendlock);
      if (ret < 0)
        {
          rpmsg_release_tx_buffer(&conn->ept, msg);
          break;
        }

      written += block;
    }

  return written ? written : ret;
}

static ssize_t rpmsg_socket_send_single(FAR struct socket *psock,
                                        FAR const struct iovec *buf,
                                        size_t iovcnt, bool nonblock)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  FAR struct rpmsg_socket_data_s *msg;
  FAR uint8_t *msgpos;
  ssize_t ret;
  uint32_t len;
  uint32_t total;
  uint32_t written = 0;
  uint32_t ipcsize;
  uint32_t space;

  ret = rpmsg_socket_get_iovlen(buf, iovcnt);
  if (ret < 0)
    {
      return ret;
    }

  len = ret;
  if (conn->sendsize < sizeof(uint32_t) ||
      len > conn->sendsize - sizeof(uint32_t))
    {
      return -EMSGSIZE;
    }

  total = len + sizeof(uint32_t);

  while (1)
    {
      nxmutex_lock(&conn->sendlock);
      space = rpmsg_socket_get_space(conn);
      nxmutex_unlock(&conn->sendlock);

      if (space < total)
        {
          if (nonblock)
            {
              return -EAGAIN;
            }

          ret = net_sem_timedwait(&conn->sendsem,
                                  _SO_TIMEOUT(conn->sconn.s_sndtimeo));
          if (!conn->ept.rdev || atomic_read(&conn->unbind))
            {
              ret = -ECONNRESET;
            }

          if (ret < 0)
            {
              return ret;
            }

          continue;
        }

      msg = rpmsg_get_tx_payload_buffer(&conn->ept, &ipcsize, true);
      if (!msg)
        {
          return -EINVAL;
        }

      if (ipcsize < sizeof(*msg) || total > ipcsize - sizeof(*msg))
        {
          rpmsg_release_tx_buffer(&conn->ept, msg);
          return -EMSGSIZE;
        }

      nxmutex_lock(&conn->sendlock);
      if (total > rpmsg_socket_get_space(conn))
        {
          nxmutex_unlock(&conn->sendlock);
          rpmsg_release_tx_buffer(&conn->ept, msg);
          if (nonblock)
            {
              return -EAGAIN;
            }

          continue;
        }

      break;
    }

  /* SOCK_DGRAM need write len to buffer */

  msg->cmd = RPMSG_SOCKET_CMD_DATA;
  msg->pos = conn->recvpos;
  msg->len = len;
  memcpy(msg->data, &len, sizeof(uint32_t));
  msgpos = msg->data + sizeof(uint32_t);
  while (written < len)
    {
      if (len - written < buf->iov_len)
        {
          memcpy(msgpos, buf->iov_base, len - written);
          written = len;
        }
      else
        {
          memcpy(msgpos, buf->iov_base, buf->iov_len);
          written += buf->iov_len;
          msgpos  += buf->iov_len;
          buf++;
        }
    }

  conn->lastpos  = conn->recvpos;
  conn->sendpos += total;

  ret = rpmsg_sendto_nocopy(&conn->ept, msg, total + sizeof(*msg),
                            conn->ept.dest_addr);
  nxmutex_unlock(&conn->sendlock);
  if (ret < 0)
    {
      rpmsg_release_tx_buffer(&conn->ept, msg);
    }

  return ret > 0 ? len : ret;
}

static ssize_t rpmsg_socket_sendmsg(FAR struct socket *psock,
                                    FAR const struct msghdr *msg, int flags)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  FAR const struct iovec *buf = msg->msg_iov;
  size_t len = msg->msg_iovlen;
  FAR const struct sockaddr *to = msg->msg_name;
  socklen_t tolen = msg->msg_namelen;
  bool nonblock;
  ssize_t ret;

  if (!_SS_ISCONNECTED(conn->sconn.s_flags))
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

  if (!conn->ept.rdev || atomic_read(&conn->unbind))
    {
      /* return ECONNRESET if lower IPC closed */

      return -ECONNRESET;
    }

  if (conn->how & SHUT_WR)
    {
      return -EPIPE;
    }

  nonblock = _SS_ISNONBLOCK(conn->sconn.s_flags) ||
             (flags & MSG_DONTWAIT) != 0;

  if (psock->s_type == SOCK_STREAM)
    {
      return rpmsg_socket_send_continuous(psock, buf, len, nonblock);
    }
  else
    {
      return rpmsg_socket_send_single(psock, buf, len, nonblock);
    }
}

static ssize_t rpmsg_socket_recvmsg(FAR struct socket *psock,
                                    FAR struct msghdr *msg, int flags)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  FAR struct sockaddr *from = msg->msg_name;
  FAR socklen_t *fromlen = &msg->msg_namelen;
  FAR void *buf = msg->msg_iov->iov_base;
  size_t len = msg->msg_iov->iov_len;
  ssize_t ret;

  if (msg->msg_iovlen != 1)
    {
      return -ENOTSUP;
    }

  if (psock->s_type != SOCK_STREAM &&
      _SS_ISBOUND(conn->sconn.s_flags) &&
      !_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      ret = rpmsg_socket_connect_internal(psock);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (!_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      return -EISCONN;
    }

  if (conn->how & SHUT_RD)
    {
      return 0;
    }

  nxmutex_lock(&conn->recvlock);
  if (psock->s_type != SOCK_STREAM)
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
      conn->recvpos += ret > 0 ? ret : 0;
    }

  if (ret > 0)
    {
      goto out;
    }

  if (!conn->ept.rdev || atomic_read(&conn->unbind))
    {
      /* return EOF if lower IPC closed */

      ret = 0;
      goto out;
    }

  if (_SS_ISNONBLOCK(conn->sconn.s_flags) || (flags & MSG_DONTWAIT) != 0)
    {
      ret = -EAGAIN;
      goto out;
    }

  conn->recvdata = buf;
  conn->recvlen  = len;

  nxsem_reset(&conn->recvsem, 0);
  nxmutex_unlock(&conn->recvlock);

  ret = net_sem_timedwait(&conn->recvsem,
                          _SO_TIMEOUT(conn->sconn.s_rcvtimeo));
  if (!conn->ept.rdev || atomic_read(&conn->unbind))
    {
      ret = 0;
    }

  nxmutex_lock(&conn->recvlock);
  if (!conn->recvdata)
    {
      ret = conn->recvlen;
    }
  else
    {
      conn->recvdata = NULL;
    }

out:
  nxmutex_unlock(&conn->recvlock);
  if (ret > 0)
    {
      rpmsg_socket_wakeup(conn);
      rpmsg_socket_getaddr(conn, from, fromlen);
    }

  return ret;
}

static int rpmsg_socket_close(FAR struct socket *psock)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;

  if (--conn->crefs)
    {
      return 0;
    }

  if (conn->backlog > 0)
    {
      FAR struct rpmsg_socket_conn_s *child;
      FAR struct rpmsg_socket_conn_s *pend;

      nxmutex_lock(&g_rpmsg_socket_lock);
      dq_rem(&conn->snode, &g_rpmsg_socket_server);
      nxmutex_unlock(&g_rpmsg_socket_lock);

      /* Detach any connection still waiting for its SYNC: they hold a
       * pointer to this listening socket which is about to be freed.
       */

      nxmutex_lock(&conn->recvlock);
      while ((pend = conn->pendlist) != NULL)
        {
          conn->pendlist = pend->pendnext;
          pend->pendnext = NULL;
          conn->pending--;

          nxmutex_lock(&pend->recvlock);
          pend->server = NULL;
          nxmutex_unlock(&pend->recvlock);

          nxmutex_unlock(&conn->recvlock);

          /* A pending connection still holds the reference taken by
           * rpmsg_socket_alloc(), so clear it before destroying the
           * endpoint;  otherwise release_cb will not free it.
           */

          pend->crefs = 0;
          rpmsg_socket_destroy_ept(pend);
          nxmutex_lock(&conn->recvlock);
        }

      /* Drop the connections that completed the handshake but were never
       * accepted.  Nothing else refers to them.
       */

      child = conn->next;
      conn->next = NULL;
      nxmutex_unlock(&conn->recvlock);

      while (child != NULL)
        {
          FAR struct rpmsg_socket_conn_s *next = child->next;

          child->next  = NULL;
          child->crefs = 0;
          rpmsg_socket_destroy_ept(child);
          child = next;
        }
    }
  else
    {
      rpmsg_unregister_callback(conn,
                                rpmsg_socket_device_created,
                                NULL,
                                NULL,
                                NULL);
    }

  if (conn->ept.rdev)
    {
      rpmsg_socket_destroy_ept(conn);
    }
  else
    {
      rpmsg_socket_free(conn);
    }

  return 0;
}

static void rpmsg_socket_path(FAR struct rpmsg_socket_conn_s *conn,
                              FAR char *buf, size_t len)
{
  if (conn->backlog) /* Server */
    {
      snprintf(buf, len,
               "rpmsg:[%s:[%s%s]<->%s]",
               CONFIG_RPMSG_LOCAL_CPUNAME, conn->rpaddr.rp_name,
               conn->nameid, conn->rpaddr.rp_cpu);
    }
  else /* Client */
    {
      snprintf(buf, len,
               "rpmsg:[%s<->%s:[%s%s]]",
               CONFIG_RPMSG_LOCAL_CPUNAME, conn->rpaddr.rp_cpu,
               conn->rpaddr.rp_name, conn->nameid);
    }
}

static int rpmsg_socket_ioctl(FAR struct socket *psock,
                              int cmd, unsigned long arg)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  int ret = OK;

  switch (cmd)
    {
      case FIONREAD:
        *(FAR int *)((uintptr_t)arg) = circbuf_used(&conn->recvbuf);
        break;

      case FIONSPACE:
        *(FAR int *)((uintptr_t)arg) = rpmsg_socket_get_space(conn);
        break;

      case FIOC_FILEPATH:
        rpmsg_socket_path(conn, (FAR char *)(uintptr_t)arg, PATH_MAX);
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static int rpmsg_socket_shutdown(FAR struct socket *psock, int how)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  FAR struct rpmsg_socket_shutdown_s *msg;
  uint32_t len = sizeof(*msg);
  int ret;

  if (!conn->ept.rdev || atomic_read(&conn->unbind))
    {
      return -ENOTCONN;
    }

  conn->how |= how;

  /* Reserve the tx buffer without holding recvlock, since it may block
   * waiting for a free vring buffer.
   */

  msg = rpmsg_get_tx_payload_buffer(&conn->ept, &len, true);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len >= sizeof(*msg));
  nxmutex_lock(&conn->recvlock);

  /* See rpmsg_socket_wakeup() */

  if (!is_rpmsg_ept_ready(&conn->ept) || atomic_read(&conn->unbind))
    {
      nxmutex_unlock(&conn->recvlock);
      rpmsg_release_tx_buffer(&conn->ept, msg);
      return -ENOTCONN;
    }

  msg->cmd = RPMSG_SOCKET_CMD_SHUTDOWN;
  msg->how = how;
  ret = rpmsg_send_offchannel_nocopy(&conn->ept, conn->ept.addr,
                                     conn->ept.dest_addr, msg,
                                     sizeof(*msg));

  nxmutex_unlock(&conn->recvlock);
  return ret < 0 ? ret : OK;
}

static int rpmsg_socket_munmap(FAR struct task_group_s *group,
                               FAR struct mm_map_entry_s *mmap,
                               FAR void *start, size_t length)
{
  FAR struct rpmsg_device *rdev = mmap->priv.p;
  FAR void *addr = mmap->vaddr;

  mm_map_remove(get_current_mm(), mmap);
  rpmsg_free_buf(rdev, addr);

  return OK;
}

static int rpmsg_socket_mmap(FAR struct socket *psock,
                             FAR struct mm_map_entry_s *mmap)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;
  int ret = OK;

  if (rpmsg_support_alloc_buf(conn->ept.rdev))
    {
      FAR void *addr;

      addr = rpmsg_alloc_buf(conn->ept.rdev, mmap->length, 8);
      if (addr == NULL)
        {
          ret = -ENOMEM;
        }
      else
        {
          mmap->vaddr = addr;
          mmap->priv.p = conn->ept.rdev;
          mmap->munmap = rpmsg_socket_munmap;
          mm_map_add(get_current_mm(), mmap);
        }
    }
  else
    {
      ret = -ENOSYS;
    }

  return ret;
}

#ifdef CONFIG_NET_SOCKOPTS
static int rpmsg_socket_getsockopt(FAR struct socket *psock, int level,
                                   int option, FAR void *value,
                                   FAR socklen_t *value_len)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;

  if (level == SOL_SOCKET)
    {
      switch (option)
        {
          case SO_PEERCRED:
            {
              if (!value || *value_len != sizeof(struct ucred))
                {
                  return -EINVAL;
                }

              memcpy(value, &conn->cred, sizeof(struct ucred));
              return OK;
            }

          case SO_RCVBUF:
            {
              if (*value_len != sizeof(int))
                {
                  return -EINVAL;
                }

              *(FAR int *)value = conn->recvsize;
              return OK;
            }

          case SO_SNDBUF:
            {
              if (*value_len != sizeof(int))
                {
                  return -EINVAL;
                }

              *(FAR int *)value = conn->sendsize;
              return OK;
            }

          case SO_PRIORITY:
            {
              if (value == NULL || *value_len == 0)
                {
                  return -EINVAL;
                }

              /* Write back only as many bytes as the caller provided, so
               * that both an int and a char argument work.
               */

              if (*value_len >= sizeof(int))
                {
                  *(FAR int *)value = conn->priority;
                  *value_len = sizeof(int);
                }
              else
                {
                  *(FAR unsigned char *)value = conn->priority;
                  *value_len = sizeof(unsigned char);
                }

              return OK;
            }
        }
    }

  return -ENOPROTOOPT;
}

static int rpmsg_socket_setsockopt(FAR struct socket *psock, int level,
                                   int option, FAR const void *value,
                                   socklen_t value_len)
{
  FAR struct rpmsg_socket_conn_s *conn = psock->s_conn;

  if (level == SOL_SOCKET)
    {
      switch (option)
        {
          case SO_RCVBUF:
            {
              if (value_len < sizeof(int))
                {
                  return -EINVAL;
                }

              if (_SS_ISCONNECTED(conn->sconn.s_flags))
                {
                  return -EISCONN;
                }

              conn->recvsize = *(FAR const int *)value;
              return OK;
            }

          case SO_PRIORITY:
            {
              int priority;

              if (value == NULL || value_len == 0)
                {
                  return -EINVAL;
                }

              if (_SS_ISCONNECTED(conn->sconn.s_flags))
                {
                  return -EISCONN;
                }

              /* SO_PRIORITY is an int in socket(7), but accept a smaller
               * value as well so that a char or uint8_t argument also
               * works.  Read only as many bytes as the caller supplied.
               */

              priority = (value_len >= sizeof(int)) ?
                *(FAR const int *)value :
                (int)*(FAR const unsigned char *)value;

              /* RPMSG_PRIO_RT is reserved for internal real-time traffic
               * and must not be requested by applications.
               */

              if (priority < RPMSG_PRIO_MIN || priority >= RPMSG_PRIO_RT)
                {
                  return -EINVAL;
                }

              conn->priority = (uint8_t)priority;
              return OK;
            }
        }
    }

  return -ENOPROTOOPT;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_socket_initialize
 *
 * Description:
 *   Catch the name service announcements of all rpmsg socket clients, so
 *   that a connection to a name nobody listens on is refused at once.
 *
 ****************************************************************************/

void rpmsg_socket_initialize(void)
{
  rpmsg_register_callback(NULL, NULL, NULL,
                          rpmsg_socket_ns_match,
                          rpmsg_socket_ns_bind);
}
