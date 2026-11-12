/****************************************************************************
 * net/vsock/vsock_sockif.c
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
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include <netinet/in.h>
#include <nuttx/circbuf.h>
#include <nuttx/fs/uio.h>
#include <nuttx/nuttx.h>
#include <nuttx/spinlock.h>

#include "socket/socket.h"
#include "vsock/vsock.h"
#include "vsock/vsock_virtio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Vsock notify type */

#define VSOCK_NOTIFY_ERROR       0
#define VSOCK_NOTIFY_LISTEN      1
#define VSOCK_NOTIFY_RECV        2
#define VSOCK_NOTIFY_SEND        3
#define VSOCK_NOTIFY_SHUTDOWN    4

#define VSOCK_IS_SHUTDOWN_RD(s)  ((s) & SHUT_RD)
#define VSOCK_IS_SHUTDOWN_WR(s)  ((s) & SHUT_WR)
#define VSOCK_IS_SHUTDOWN(s)     ((s) == SHUT_RDWR)

#define VSOCK_LAST_RESERVED_PORT 1024

#define VSOCK_CLOSE_TIMEOUT      SEC2TICK(8)

/* Virtio Transport function */

#define VSOCK_GET_LOCALCID(t)    (t)->ops->get_localcid(t)
#define VSOCK_GET_REMOTECID(t)   (t)->ops->get_remotecid(t)
#define VSOCK_ALLOC_PKT(t,p,c)   (t)->ops->alloc_pkt(t,p,c)
#define VSOCK_SEND_PKT(t,p)      (t)->ops->send_pkt(t,p)

#ifdef VSOCK_DEBUG
#  define vsockdbg               _err
#else
#  define vsockdbg(fmt, ...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vsock_msg_s
{
  struct uio                    uio;

  /* Recv msg: total capacity to receive data
   * Send msg: total data length to send
   */

  size_t                        len;
};

struct vsock_conn_s
{
  /* Common prologue of all connection structures. */

  struct socket_conn_s          sconn;

  FAR struct vsock_transport_s *transport;
  struct sockaddr_vm            local_addr;
  struct sockaddr_vm            remote_addr;
  atomic_t                      ref;

  /* Save bound or connected list node */

  struct list_node              node;

  /* For accept connection, this is the list node in server child list
   */

  struct list_node              child;

  spinlock_t                    poll_lock;
  FAR struct pollfd_s          *fds[CONFIG_NET_VSOCK_NPOLLWAITERS];

  /* Rx */

  rmutex_t                      rx_lock;
  sem_t                         rx_sem;
  struct circbuf_s              rx_buf;
  FAR struct vsock_msg_s       *rx_msg;
  size_t                        rx_lowat;

  spinlock_t                    cnt_lock;
  uint32_t                      rx_fwd_cnt;
  uint32_t                      rx_last_fwd_cnt;
  uint32_t                      rx_buf_alloc;

  /* Tx */

  uint32_t                      tx_cnt;
  uint32_t                      tx_fwd_cnt;
  uint32_t                      tx_buf_alloc;

  rmutex_t                      tx_lock;
  sem_t                         tx_sem;

  /* Shutdown flags */

  uint32_t                      shutdown;

  /* Close timeout work */

  struct work_s                 close_work;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef VSOCK_DEBUG
static FAR const char *g_vsock_op[] =
{
  [VIRTIO_VSOCK_OP_INVALID]        = "INVALID",
  [VIRTIO_VSOCK_OP_REQUEST]        = "REQUEST",
  [VIRTIO_VSOCK_OP_RESPONSE]       = "RESPONSE",
  [VIRTIO_VSOCK_OP_RST]            = "RST",
  [VIRTIO_VSOCK_OP_SHUTDOWN]       = "SHUTDOWN",
  [VIRTIO_VSOCK_OP_RW]             = "RW",
  [VIRTIO_VSOCK_OP_CREDIT_UPDATE]  = "CREDIT_UPDATE",
  [VIRTIO_VSOCK_OP_CREDIT_REQUEST] = "CREDIT_REQUEST",
};
#endif

static_assert(SHUT_RD == VIRTIO_VSOCK_SHUTDOWN_RECV, "");
static_assert(SHUT_WR == VIRTIO_VSOCK_SHUTDOWN_SEND, "");

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        vsock_setup(FAR struct socket *psock);
static sockcaps_t vsock_sockcaps(FAR struct socket *psock);
static void       vsock_addref(FAR struct socket *psock);
static int        vsock_bind(FAR struct socket *psock,
                             FAR const struct sockaddr *addr,
                             socklen_t addrlen);
static int        vsock_getsockname(FAR struct socket *psock,
                                    FAR struct sockaddr *addr,
                                    FAR socklen_t *addrlen);
static int        vsock_getpeername(FAR struct socket *psock,
                                    FAR struct sockaddr *addr,
                                    FAR socklen_t *addrlen);
static int        vsock_listen(FAR struct socket *psock, int backlog);
static int        vsock_connect(FAR struct socket *psock,
                                FAR const struct sockaddr *addr,
                                socklen_t addrlen);
static int        vsock_accept(FAR struct socket *psock,
                               FAR struct sockaddr *addr,
                               FAR socklen_t *addrlen,
                               FAR struct socket *newsock, int flags);
static int        vsock_poll(FAR struct socket *psock,
                             FAR struct pollfd_s *fds, bool setup);
static ssize_t    vsock_sendmsg(FAR struct socket *psock,
                                FAR const struct msghdr *msg, int flags);
static ssize_t    vsock_recvmsg(FAR struct socket *psock,
                                FAR struct msghdr *msg, int flags);
static int        vsock_close(FAR struct socket *psock);
static int        vsock_ioctl(FAR struct socket *psock, int cmd,
                              unsigned long arg);
static int        vsock_shutdown(FAR struct socket *psock, int how);
#ifdef CONFIG_NET_SOCKOPTS
static int        vsock_getsockopt(FAR struct socket *psock, int level,
                                   int option, FAR void *value,
                                   FAR socklen_t *value_len);
static int        vsock_setsockopt(FAR struct socket *psock, int level,
                                   int option, FAR const void *value,
                                   socklen_t value_len);
#endif

#ifdef VSOCK_DEBUG
static void vsock_dump_pkt(FAR struct vsock_pkt_s *pkt,
                           FAR const char *msg, bool dumpbuf);
#else
#  define vsock_dump_pkt(pkt, msg, dumpbuf)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_vsock_sockif =
{
  vsock_setup,       /* si_setup */
  vsock_sockcaps,    /* si_sockcaps */
  vsock_addref,      /* si_addref */
  vsock_bind,        /* si_bind */
  vsock_getsockname, /* si_getsockname */
  vsock_getpeername, /* si_getpeername */
  vsock_listen,      /* si_listen */
  vsock_connect,     /* si_connect */
  vsock_accept,      /* si_accept */
  vsock_poll,        /* si_poll */
  vsock_sendmsg,     /* si_sendmsg */
  vsock_recvmsg,     /* si_recvmsg */
  vsock_close,       /* si_close */
  vsock_ioctl,       /* si_ioctl */
  NULL,              /* si_socketpair */
  vsock_shutdown     /* si_shutdown */
#ifdef CONFIG_NET_SOCKOPTS
  , vsock_getsockopt /* si_getsockopt */
  , vsock_setsockopt /* si_setsockopt */
#endif
#ifdef CONFIG_NET_SENDFILE
  , NULL             /* si_sendfile */
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static atomic_t g_vsock_port = VSOCK_LAST_RESERVED_PORT;

static FAR struct kwork_wqueue_s *g_vsock_wqueue = NULL;

/* List and their locks */

static struct list_node g_vsock_conn = LIST_INITIAL_VALUE(g_vsock_conn);
static struct list_node g_vsock_transport =
  LIST_INITIAL_VALUE(g_vsock_transport);

static spinlock_t g_vsock_conn_lock = SP_UNLOCKED;
static spinlock_t g_vsock_transport_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsock_lock/unlock
 ****************************************************************************/

static inline_function void vsock_lock(FAR struct vsock_conn_s *conn)
{
  nxrmutex_lock(&conn->rx_lock);
  nxrmutex_lock(&conn->tx_lock);
}

static inline_function void vsock_unlock(FAR struct vsock_conn_s *conn)
{
  nxrmutex_unlock(&conn->tx_lock);
  nxrmutex_unlock(&conn->rx_lock);
}

/****************************************************************************
 * Name: vsock_rx_lock/unlock
 ****************************************************************************/

static inline_function void vsock_rx_lock(FAR struct vsock_conn_s *conn)
{
  nxrmutex_lock(&conn->rx_lock);
}

static inline_function void vsock_rx_unlock(FAR struct vsock_conn_s *conn)
{
  nxrmutex_unlock(&conn->rx_lock);
}

/****************************************************************************
 * Name: vsock_rx_lock/unlock
 ****************************************************************************/

static inline_function void vsock_tx_lock(FAR struct vsock_conn_s *conn)
{
  nxrmutex_lock(&conn->tx_lock);
}

static inline_function void vsock_tx_unlock(FAR struct vsock_conn_s *conn)
{
  nxrmutex_unlock(&conn->tx_lock);
}

/****************************************************************************
 * Name: vsock_breaklock/vsock_restorelock
 ****************************************************************************/

static inline_function void vsock_breaklock(FAR struct vsock_conn_s *conn,
                                            FAR unsigned int *rxcount,
                                            FAR unsigned int *txcount)
{
  nxrmutex_breaklock(&conn->tx_lock, txcount);
  nxrmutex_breaklock(&conn->rx_lock, rxcount);
}

static inline_function void vsock_restorelock(FAR struct vsock_conn_s *conn,
                                              unsigned int rxcount,
                                              unsigned int txcount)
{
  nxrmutex_restorelock(&conn->rx_lock, rxcount);
  nxrmutex_restorelock(&conn->tx_lock, txcount);
}

/****************************************************************************
 * Name: vsock_get_rx_size
 *
 * Description:
 *   Helper function for vsock poll, notify the polling thread
 *
 ****************************************************************************/

static inline_function size_t
vsock_get_rx_size(FAR struct vsock_conn_s *conn)
{
  size_t rx_size = 0;

  if (_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      rx_size = circbuf_used(&conn->rx_buf);
      if (conn->rx_msg != NULL)
        {
          rx_size += conn->rx_msg->len - conn->rx_msg->uio.uio_resid;
        }
    }
  else
    {
      vrterr("not connected\n");
    }

  return rx_size;
}

/****************************************************************************
 * Name: vsock_poll_notify
 *
 * Description:
 *   Helper function for vsock poll, notify the polling thread
 *
 ****************************************************************************/

static inline_function void vsock_poll_notify(FAR struct vsock_conn_s *conn,
                                              pollevent_t eventset)
{
  irqstate_t flags;

  flags = spin_lock_irqsave_nopreempt(&conn->poll_lock);
  poll_notify(conn->fds, CONFIG_NET_VSOCK_NPOLLWAITERS, eventset);
  spin_unlock_irqrestore_nopreempt(&conn->poll_lock, flags);
}

static void vsock_notify(FAR struct vsock_conn_s *conn, int type)
{
  pollevent_t eventset = 0;

  switch (type)
    {
      case VSOCK_NOTIFY_ERROR:
        vsock_post(&conn->rx_sem);
        vsock_post(&conn->tx_sem);
        eventset = POLLERR | POLLHUP;
        break;
      case VSOCK_NOTIFY_LISTEN:
        vsock_post(&conn->rx_sem);
        eventset = POLLIN;
        break;
      case VSOCK_NOTIFY_SEND:
        vsock_post(&conn->tx_sem);
        eventset = POLLOUT;
        break;
      case VSOCK_NOTIFY_RECV:
        if (vsock_get_rx_size(conn) >= conn->rx_lowat)
          {
            vsock_post(&conn->rx_sem);
            eventset = POLLIN;
          }
        break;
      case VSOCK_NOTIFY_SHUTDOWN:
        if (conn->shutdown & SHUT_RD)
          {
            vsock_post(&conn->rx_sem);
            eventset |= POLLIN;
          }

        if (conn->shutdown & SHUT_WR)
          {
            vsock_post(&conn->tx_sem);
            eventset |= POLLOUT;
          }

        if (VSOCK_IS_SHUTDOWN(conn->shutdown))
          {
            eventset |= POLLHUP;
          }
        break;
    }

  vsock_poll_notify(conn, eventset);
}

/****************************************************************************
 * Name: vsock_assgin_transport
 *
 * Description:
 *   Assign the vsock transport to the vsock connection and call the
 *   transport init to initialize the transport vsock connection.
 *
 ****************************************************************************/

static int vsock_assgin_transport(FAR struct vsock_conn_s *conn)
{
  FAR struct vsock_transport_s *t;
  uint64_t remote_cid;
  irqstate_t flags;

  remote_cid = conn->remote_addr.svm_cid;
  flags = spin_lock_irqsave(&g_vsock_transport_lock);
  list_for_every_entry(&g_vsock_transport, t, struct vsock_transport_s, node)
    {
      if (VSOCK_GET_REMOTECID(t) == remote_cid)
        {
          conn->transport = t;
          break;
        }
    }

  spin_unlock_irqrestore(&g_vsock_transport_lock, flags);

  if (conn->transport == NULL)
    {
      vrterr("Transport not found\n");
      return -ENOTSUP;
    }

  return circbuf_resize(&conn->rx_buf, conn->rx_buf_alloc);
}

/****************************************************************************
 * Name: vsock_free
 *
 * Description:
 *   Directly free the vsock connection. Before call this function, other
 *   connection resource should has been freed, such as transport uninit
 *
 ****************************************************************************/

static void vsock_free(FAR struct vsock_conn_s *conn)
{
  vsockdbg("vsock free, conn=%p\n", conn);
  nxrmutex_destroy(&conn->rx_lock);
  nxrmutex_destroy(&conn->tx_lock);
  nxsem_destroy(&conn->rx_sem);
  nxsem_destroy(&conn->tx_sem);
  circbuf_uninit(&conn->rx_buf);
  kmm_free(conn);
}

/****************************************************************************
 * Name: vsock_add/sub_ref
 *
 * Description:
 *   The virtio transport receive worker/thread may use this the virtual
 *   socket connection at any time. And the User may call vsock_close to free
 *   the connection at any time too. So every virtual socket connection
 *   maintains a reference count for safe free the connection.
 *   vsock_add/sub_ref() will increase/decrease the reference count, and will
 *   cleanup the virtual socket connection resource when the reference count
 *   equals to 0.
 *
 ****************************************************************************/

static inline_function void vsock_add_ref(FAR struct vsock_conn_s *conn)
{
  atomic_fetch_add(&conn->ref, 1);
}

static inline_function void vsock_sub_ref(FAR struct vsock_conn_s *conn)
{
  if (atomic_fetch_sub(&conn->ref, 1) <= 1)
    {
      vsock_free(conn);
    }
}

/****************************************************************************
 * Name: vsock_alloc
 *
 * Description:
 *   Alloc a vsock connection and initialize it.
 *
 ****************************************************************************/

static FAR struct vsock_conn_s *vsock_alloc(void)
{
  FAR struct vsock_conn_s *conn;
  int ret;

  conn = kmm_zalloc(sizeof(*conn));
  if (conn == NULL)
    {
      vrterr("Alloc vsock conn failed\n");
      return NULL;
    }

  ret = circbuf_init(&conn->rx_buf, NULL, 0);
  if (ret < 0)
    {
      kmm_free(conn);
      vrterr("circbuf_init failed %d\n", ret);
      return NULL;
    }

  conn->rx_lowat = 1;
  conn->rx_buf_alloc = CONFIG_NET_VSOCK_RX_BUFSIZE;
  list_initialize(&conn->child);

  list_initialize(&conn->node);
  vsock_addr_init(&conn->local_addr, VMADDR_CID_ANY, VMADDR_PORT_ANY);
  vsock_addr_init(&conn->remote_addr, VMADDR_CID_ANY, VMADDR_PORT_ANY);

  nxrmutex_init(&conn->rx_lock);
  nxrmutex_init(&conn->tx_lock);
  spin_lock_init(&conn->poll_lock);
  spin_lock_init(&conn->cnt_lock);
  nxsem_init(&conn->rx_sem, 0, 0);
  nxsem_init(&conn->tx_sem, 0, 0);
  vsock_add_ref(conn);
  return conn;
}

/****************************************************************************
 * Name: vsock_find_bound/connected_conn
 *
 * Description:
 *   Find the bound or connected connection in corresponding list.
 *   When received a packet from peer, transport layer call these functions
 *   to get the corresponding connection.
 *
 *   vsock_find_bound_conn(): Find bound connection
 *   vsock_find_connected_conn(): Find connection connection
 *
 ****************************************************************************/

static inline_function FAR struct vsock_conn_s *
vsock_find_bound_conn(FAR const struct sockaddr_vm *addr)
{
  FAR const struct sockaddr_vm *local;
  FAR struct vsock_conn_s *conn;
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_vsock_conn_lock);
  list_for_every_entry(&g_vsock_conn, conn, struct vsock_conn_s, node)
    {
      local = &conn->local_addr;
      if (local->svm_port == addr->svm_port &&
          (local->svm_cid == addr->svm_cid ||
          local->svm_cid == VMADDR_CID_ANY ||
          addr->svm_cid == VMADDR_CID_ANY))
        {
          vsock_add_ref(conn);
          spin_unlock_irqrestore(&g_vsock_conn_lock, flags);
          return conn;
        }
    }

  spin_unlock_irqrestore(&g_vsock_conn_lock, flags);
  return NULL;
}

static inline_function FAR struct vsock_conn_s *
vsock_find_connected_conn(FAR const struct sockaddr_vm *remote_addr,
                          FAR const struct sockaddr_vm *local_addr)
{
  FAR struct vsock_conn_s *conn;
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_vsock_conn_lock);
  list_for_every_entry(&g_vsock_conn, conn, struct vsock_conn_s, node)
    {
      if (conn->remote_addr.svm_cid == remote_addr->svm_cid &&
          conn->remote_addr.svm_port == remote_addr->svm_port &&
          conn->local_addr.svm_port == local_addr->svm_port &&
          (conn->local_addr.svm_cid == VMADDR_CID_ANY ||
          conn->local_addr.svm_cid == local_addr->svm_cid))
        {
          vsock_add_ref(conn);
          spin_unlock_irqrestore(&g_vsock_conn_lock, flags);
          return conn;
        }
    }

  spin_unlock_irqrestore(&g_vsock_conn_lock, flags);
  return NULL;
}

/****************************************************************************
 * Name: vsock_insert/remove_conn
 *
 * Description:
 *   The bound list maintained the connections that listen (server) for
 *   connections or want to connect to peer connection (client).
 *   The transport layer will find the connection in this list to handle
 *   connection connected event.
 *
 *   vsock_insert_conn(): Insert a connection to the bound list
 *   vsock_remove_conn(): Remove a connection from the bound list
 *
 ****************************************************************************/

static inline_function void vsock_insert_conn(FAR struct vsock_conn_s *conn)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_vsock_conn_lock);
  list_add_tail(&g_vsock_conn, &conn->node);
  spin_unlock_irqrestore(&g_vsock_conn_lock, flags);
}

static inline_function void vsock_remove_conn(FAR struct vsock_conn_s *conn)
{
  irqstate_t flags;

  vsockdbg("Remove bound, conn=%p\n", conn);
  flags = spin_lock_irqsave(&g_vsock_conn_lock);
  if (list_in_list(&conn->node))
    {
      list_delete(&conn->node);
      list_clear_node(&conn->node);
    }

  spin_unlock_irqrestore(&g_vsock_conn_lock, flags);
}

static inline_function bool vsock_check_conn(FAR struct vsock_conn_s *conn)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_vsock_conn_lock);
  if (list_in_list(&conn->node))
    {
      spin_unlock_irqrestore(&g_vsock_conn_lock, flags);
      return true;
    }

  spin_unlock_irqrestore(&g_vsock_conn_lock, flags);
  return false;
}

/****************************************************************************
 * Name: vsock_bind_internal
 *
 * Description:
 *   Get the vsock connection port and add it to the bound connection list.
 *
 ****************************************************************************/

static int vsock_bind_internal(FAR struct vsock_conn_s *conn,
                               FAR const struct sockaddr_vm *addr)
{
  struct sockaddr_vm new_addr;

  vsock_addr_init(&new_addr, addr->svm_cid, addr->svm_port);

  if (addr->svm_port == VMADDR_PORT_ANY)
    {
      /* Simple method to find a port that not used by other */

      for (; ; )
        {
          new_addr.svm_port = atomic_fetch_add(&g_vsock_port, 1);
          if (vsock_find_bound_conn(&new_addr) == NULL)
            {
              goto out;
            }
        }

      return -EADDRNOTAVAIL;
    }
  else
    {
      if (addr->svm_port < VSOCK_LAST_RESERVED_PORT)
        {
          return -EACCES;
        }

      if (vsock_find_bound_conn(&new_addr) != NULL)
        {
          return -EADDRINUSE;
        }
    }

out:
  vsock_addr_init(&conn->local_addr, new_addr.svm_cid, new_addr.svm_port);
  vsock_insert_conn(conn);
  return OK;
}

/****************************************************************************
 * Name: vsock_unbind_internal
 *
 * Description:
 *   Remove the connection from the bound connection list and init the err
 *   number and connection state
 *
 ****************************************************************************/

static inline_function void
vsock_unbind_internal(FAR struct vsock_conn_s *conn)
{
  conn->sconn.s_error = 0;
  vsock_remove_conn(conn);
}

/****************************************************************************
 * Name: vsock_alloc_tx_credit
 ****************************************************************************/

static inline_function void
vsock_alloc_tx_credit(FAR struct vsock_conn_s *conn, uint32_t credit)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&conn->cnt_lock);
  conn->tx_cnt += credit;
  spin_unlock_irqrestore(&conn->cnt_lock, flags);
}

/****************************************************************************
 * Name: vsock_free_tx_credit
 ****************************************************************************/

static unused_code inline_function void
vsock_free_tx_credit(FAR struct vsock_conn_s *conn, uint32_t credit)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&conn->cnt_lock);
  conn->tx_cnt -= credit;
  spin_unlock_irqrestore(&conn->cnt_lock, flags);
}

/****************************************************************************
 * Name: vsock_get_tx_credit
 ****************************************************************************/

static inline_function size_t
vsock_get_tx_credit(FAR struct vsock_conn_s *conn)
{
  irqstate_t flags;
  size_t len;

  flags = spin_lock_irqsave(&conn->cnt_lock);
  len = conn->tx_buf_alloc - (conn->tx_cnt - conn->tx_fwd_cnt);
  spin_unlock_irqrestore(&conn->cnt_lock, flags);
  return len;
}

/****************************************************************************
 * Name: vsock_update_tx_credit
 ****************************************************************************/

static inline_function void
vsock_update_tx_credit(FAR struct vsock_conn_s *conn,
                       FAR struct vsock_pkt_s *pkt)
{
  FAR struct vsock_hdr_s *hdr = vsock_pkt2hdr(pkt);
  irqstate_t flags;

  /* Listener sockets are not associated with any transport, so we are
   * not able to take the state to see if there is space available in the
   * remote peer, but since they are only used to receive requests, we
   * can assume that there is always space available in the other peer.
   */

  flags = spin_lock_irqsave(&conn->cnt_lock);
  conn->tx_buf_alloc = hdr->buf_alloc;
  conn->tx_fwd_cnt = hdr->fwd_cnt;
  spin_unlock_irqrestore(&conn->cnt_lock, flags);
}

/****************************************************************************
 * Name: vsock_msg_done
 *
 * Description:
 *   Judge whether the virtual msg is full (RX) or empty (TX).
 *
 ****************************************************************************/

static inline_function bool vsock_msg_done(FAR const struct vsock_msg_s *msg)
{
  return msg->uio.uio_resid == 0;
}

/****************************************************************************
 * Name: vsock_copy_pkt2msg
 *
 * Description:
 *   Copy the rx pkt data in the rx queue to the msg until the msg full or
 *   ran out of the rx pkt data.
 *
 ****************************************************************************/

static inline_function size_t
vsock_copy_pkt2msg(FAR struct vsock_conn_s *conn,
                   FAR struct vsock_msg_s *msg, FAR struct vsock_pkt_s *pkt)
{
  size_t ret = 0;

  vsockdbg("pkt vbcnt=%zu vbidx=%zu vboff=%zu vb[0].len=%u msg used=%zu\n",
           pkt->vbcnt, pkt->vbidx, pkt->vboff, pkt->vb[0].len,
           msg->len - msg->uio.uio_resid);

  while (!vsock_msg_done(msg) && pkt->vbidx < pkt->vbcnt &&
         pkt->vboff <= pkt->vb[pkt->vbidx].len)
    {
      FAR const char *buf = pkt->vb[pkt->vbidx].buf + pkt->vboff;
      size_t len = pkt->vb[pkt->vbidx].len - pkt->vboff;
      size_t copied = MIN(len, msg->uio.uio_resid);

      uio_copyfrom(&msg->uio, 0, buf, copied);
      uio_advance(&msg->uio, copied);
      if (copied >= len)
        {
          pkt->vbidx++;
          pkt->vboff = 0;
        }
      else
        {
          pkt->vboff += copied;
        }

      ret += copied;
    }

  return ret;
}

/****************************************************************************
 * Name: vsock_copy_circ2msg
 *
 * Description:
 *   Copy data from the circular buffer to the virtual socket message until
 *   circular buffer empty or virtual socket message full.
 *
 ****************************************************************************/

static size_t vsock_copy_circ2msg(FAR struct vsock_conn_s *conn,
                                  FAR struct vsock_msg_s *msg)
{
  FAR struct circbuf_s *circ = &conn->rx_buf;
  size_t ret = 0;

  while (!circbuf_is_empty(circ) && !vsock_msg_done(msg))
    {
      size_t len;

      FAR const char *buf = circbuf_get_readptr(circ, &len);

      len = MIN(len, msg->uio.uio_resid);

      uio_copyfrom(&msg->uio, 0, buf, len);
      uio_advance(&msg->uio, len);
      circbuf_readcommit(circ, len);
      ret += len;
    }

  return ret;
}

/****************************************************************************
 * Name: vsock_copy_msg2pkt
 ****************************************************************************/

static size_t vsock_copy_msg2pkt(FAR struct vsock_pkt_s *pkt,
                                FAR struct vsock_msg_s *msg, size_t total)
{
  size_t ret = 0;

  DEBUGASSERT(total <= msg->uio.uio_resid);

  while (pkt != NULL)
    {
      while (total > 0 && pkt->vbidx < pkt->vbcnt &&
             pkt->vboff <= pkt->vb[pkt->vbidx].len)
        {
          FAR void *buf = pkt->vb[pkt->vbidx].buf + pkt->vboff;
          size_t len = pkt->vb[pkt->vbidx].len - pkt->vboff;

          size_t copied = MIN(len, total);

          uio_copyto(&msg->uio, 0, buf, copied);
          uio_advance(&msg->uio, copied);
          if (copied >= len)
            {
              pkt->vbidx++;
              pkt->vboff = 0;
            }
          else
            {
              pkt->vboff += copied;
            }

          total -= copied;
          ret += copied;
        }

      pkt = pkt->next;
    }

  return ret;
}

/****************************************************************************
 * Name: vsock_copy_pkt2circ
 *
 * Description:
 *   Copy the rx pkt data in the rx queue to the rx circular buffer until
 *   the rx buffer full or ran out of the rx pkt data.
 *
 ****************************************************************************/

static inline_function size_t
vsock_copy_pkt2circ(FAR struct vsock_conn_s *conn,
                    FAR struct vsock_pkt_s *pkt)
{
  FAR struct circbuf_s *circ = &conn->rx_buf;
  size_t ret = 0;

  vsockdbg("pkt vbcnt=%zu vbidx=%zu vboff=%zu "
           "vb[0].len=%" PRIu32 " rxbuf used=%" PRIu32 "\n",
           pkt->vbcnt, pkt->vbidx, pkt->vboff, pkt->vb[0].len,
           circbuf_used(circ));

  while (!circbuf_is_full(circ) && pkt->vbidx < pkt->vbcnt &&
         pkt->vboff <= pkt->vb[pkt->vbidx].len)
    {
      FAR const char *buf = pkt->vb[pkt->vbidx].buf + pkt->vboff;
      size_t len = pkt->vb[pkt->vbidx].len - pkt->vboff;

      size_t copied = circbuf_write(circ, buf, len);

      if (copied >= len)
        {
          DEBUGASSERT(copied == len);
          pkt->vbidx++;
          pkt->vboff = 0;
        }
      else
        {
          DEBUGASSERT(circbuf_is_full(circ));
          pkt->vboff += copied;
        }

      ret += copied;
    }

  return ret;
}

/****************************************************************************
 * Name: vsock_send_pkt
 ****************************************************************************/

static int vsock_send_pkt(FAR struct vsock_conn_s *conn,
                          FAR struct vsock_msg_s *msg,
                          uint16_t op, uint32_t hdrflags,
                          uint64_t dst_cid, uint32_t dst_port)
{
  FAR struct vsock_pkt_s *tmp;
  struct vsock_pkt_s pkt;
  struct vsock_hdr_s *hdr;
  unsigned int rxcount;
  unsigned int txcount;
  irqstate_t flags;
  uint32_t len;
  int ret;

  /* vsock_get_tx_credit might return less than len credit */

  len = msg ? MIN(vsock_get_tx_credit(conn), msg->uio.uio_resid) : 0;

  /* Do not send zero length OP_RW pkt */

  if (len == 0 && op == VIRTIO_VSOCK_OP_RW)
    {
      return len;
    }

  memset(&pkt, 0, sizeof(struct vsock_pkt_s));

  vsock_breaklock(conn, &rxcount, &txcount);
  ret = VSOCK_ALLOC_PKT(conn->transport, &pkt, len);
  vsock_restorelock(conn, rxcount, txcount);
  if (ret < 0)
    {
      vrterr("Alloc pkt and buffer failed %d\n", ret);
      return ret;
    }

  if (len != 0)
    {
      len = vsock_copy_msg2pkt(&pkt, msg, len);
      vsock_alloc_tx_credit(conn, len);
    }

  for (tmp = &pkt; tmp != NULL; tmp = tmp->next)
    {
      hdr = vsock_pkt2hdr(tmp);
      if (dst_cid == 0)
        {
          hdr->dst_cid  = conn->remote_addr.svm_cid;
          hdr->dst_port = conn->remote_addr.svm_port;
        }
      else
        {
          hdr->dst_cid  = dst_cid;
          hdr->dst_port = dst_port;
        }

      hdr->type     = VIRTIO_VSOCK_TYPE_STREAM;
      hdr->op       = op;
      hdr->src_cid  = VSOCK_GET_LOCALCID(conn->transport);
      hdr->src_port = conn->local_addr.svm_port;
      hdr->flags    = hdrflags;
      hdr->len      = tmp->len - VIRTIO_VSOCK_HDR_LEN;

      flags = spin_lock_irqsave(&conn->cnt_lock);
      conn->rx_last_fwd_cnt = conn->rx_fwd_cnt;
      hdr->fwd_cnt = conn->rx_fwd_cnt;
      hdr->buf_alloc = conn->rx_buf_alloc;
      spin_unlock_irqrestore(&conn->cnt_lock, flags);
    }

  vsock_dump_pkt(&pkt, "Send", false);
  return VSOCK_SEND_PKT(conn->transport, &pkt);
}

/****************************************************************************
 * Name: vsock_send_credit
 ****************************************************************************/

static inline_function int vsock_send_credit(FAR struct vsock_conn_s *conn)
{
  return vsock_send_pkt(conn, NULL, VIRTIO_VSOCK_OP_CREDIT_UPDATE, 0, 0, 0);
}

/****************************************************************************
 * Name: vsock_update_rx_credit
 ****************************************************************************/

static inline_function void
vsock_update_rx_credit(FAR struct vsock_conn_s *conn, size_t size)
{
  irqstate_t flags;
  uint32_t delta;

  flags = spin_lock_irqsave(&conn->cnt_lock);
  conn->rx_fwd_cnt += size;
  delta = conn->rx_fwd_cnt - conn->rx_last_fwd_cnt;
  spin_unlock_irqrestore(&conn->cnt_lock, flags);

  vsockdbg("used=%" PRIu32 " alloc=%" PRIu32 " fwd=%" PRIu32
           " last_fwd=%" PRIu32 "\n", delta, conn->rx_buf_alloc,
           conn->rx_fwd_cnt, conn->rx_last_fwd_cnt);
  if (delta >= conn->rx_buf_alloc / 2)
    {
      vsockdbg("Update the credit\n");
      vsock_send_credit(conn);
    }
}

/****************************************************************************
 * Name: vsock_reset
 *
 * Description:
 *  Normally packets are associated with a socket.  There may be no socket
 *  if an attempt was made to connect to a socket that does not exist.
 *
 ****************************************************************************/

static int vsock_reset(FAR struct vsock_conn_s *conn,
                       FAR struct vsock_pkt_s *pkt)
{
  if (pkt)
    {
      FAR struct vsock_hdr_s *hdr = vsock_pkt2hdr(pkt);

      if (hdr->op == VIRTIO_VSOCK_OP_RST)
        {
          return 0;
        }
    }

  /* Send RST only if the original pkt is not a RST pkt or pkt is null */

  return vsock_send_pkt(conn, NULL, VIRTIO_VSOCK_OP_RST, 0, 0, 0);
}

/****************************************************************************
 * Name: vsock_recv_connecting
 ****************************************************************************/

static int vsock_recv_connecting(FAR struct vsock_conn_s *conn,
                                 FAR struct vsock_pkt_s *pkt)
{
  FAR struct vsock_hdr_s *hdr = vsock_pkt2hdr(pkt);
  int ret = -EINVAL;

  switch (hdr->op)
    {
      case VIRTIO_VSOCK_OP_RESPONSE:
        vsockdbg("conn=%p, VSOCK_CONNECTED\n", conn);
        conn->sconn.s_flags |= _SF_CONNECTED;
        vsock_notify(conn, VSOCK_NOTIFY_SEND);
        return 0;
      case VIRTIO_VSOCK_OP_RST:
        vrterr("conn=%p, VSOCK_RST\n", conn);
        ret = -ECONNRESET;
        break;
      default:
        vrterr("conn=%p, VSOCK_ERROR, op=%u\n", conn, hdr->op);
        break;
    }

  vsock_reset(conn, pkt);
  conn->sconn.s_error = ret;
  vsock_notify(conn, VSOCK_NOTIFY_ERROR);
  return ret;
}

/****************************************************************************
 * Name: vsock_reset_no_conn
 *
 * Description:
 *  Normally packets are associated with a socket.  There may be no socket
 *  if an attempt was made to connect to a socket that does not exist.
 *
 ****************************************************************************/

static int vsock_reset_no_conn(FAR struct vsock_transport_s *t,
                               FAR struct vsock_pkt_s *pkt)
{
  FAR struct vsock_hdr_s *hdr = vsock_pkt2hdr(pkt);
  FAR struct vsock_hdr_s *rhdr;
  FAR struct vsock_pkt_s reply;
  int ret;

  /* Send RST only if the original pkt is not a RST pkt */

  if (hdr->op == VIRTIO_VSOCK_OP_RST)
    {
      return 0;
    }

  ret = VSOCK_ALLOC_PKT(t, &reply, 0);
  if (ret < 0)
    {
      nwarn("WARNING: Failed to allocate vsock RST reply packet: %d\n", ret);
      return ret;
    }

  rhdr           = vsock_pkt2hdr(&reply);
  rhdr->type     = VIRTIO_VSOCK_TYPE_STREAM;
  rhdr->op       = VIRTIO_VSOCK_OP_RST;
  rhdr->src_cid  = hdr->dst_cid;
  rhdr->src_port = hdr->dst_port;
  rhdr->dst_cid  = hdr->src_cid;
  rhdr->dst_port = hdr->src_port;
  rhdr->flags    = 0;
  rhdr->len      = 0;

  vsock_dump_pkt(&reply, "Send", false);
  return VSOCK_SEND_PKT(t, &reply);
}

/****************************************************************************
 * Name: vsock_recv_listening
 ****************************************************************************/

static int vsock_recv_listening(FAR struct vsock_conn_s *conn,
                                FAR struct vsock_transport_s *t,
                                FAR struct vsock_pkt_s *pkt)
{
  FAR struct vsock_hdr_s *hdr = vsock_pkt2hdr(pkt);
  FAR struct vsock_conn_s *newconn;
  int ret;

  if (hdr->op != VIRTIO_VSOCK_OP_REQUEST)
    {
      ret = -EINVAL;
      goto reset;
    }

  newconn = vsock_alloc();
  if (newconn == NULL)
    {
      nwarn("WARNING: Failed to allocate vsock connection for the incoming"
           " request\n");
      ret = -ENOMEM;
      goto reset;
    }

  vsock_addr_init(&newconn->local_addr, hdr->dst_cid, hdr->dst_port);
  vsock_addr_init(&newconn->remote_addr, hdr->src_cid, hdr->src_port);
  newconn->transport = t;
  ret = circbuf_resize(&newconn->rx_buf, newconn->rx_buf_alloc);
  if (ret < 0)
    {
      nwarn("WARNING: Failed to resize rx buffer for new vsock"
           " connection: %d\n", ret);
      vsock_free(newconn);
      goto reset;
    }

  list_add_tail(&conn->child, &newconn->child);
  vsockdbg("create conn=%p dst_port=%" PRIu32 "\n", newconn, hdr->dst_port);

  vsock_update_tx_credit(newconn, pkt);
  newconn->sconn.s_flags |= _SF_CONNECTED;
  vsock_insert_conn(newconn);
  vsock_notify(conn, VSOCK_NOTIFY_LISTEN);

  /* Send VSOCK RESPONSE to peer */

  return vsock_send_pkt(newconn, NULL, VIRTIO_VSOCK_OP_RESPONSE, 0,
                        hdr->src_cid, hdr->src_port);

reset:
  vsock_reset_no_conn(t, pkt);
  return ret;
}

/****************************************************************************
 * Name: vsock_recv_enqueue
 ****************************************************************************/

static void vsock_recv_enqueue(FAR struct vsock_conn_s *conn,
                              FAR struct vsock_pkt_s *pkt)
{
  FAR struct vsock_hdr_s *hdr = vsock_pkt2hdr(pkt);
  uint32_t len = hdr->len;
  size_t copied = 0;

  DEBUGASSERT(conn != NULL);

  /* If the remote peer doesn't respect the credit information
   * (rx_buf_alloc, rx_fwd_cnt), sending more data than it can send,
   * we should drop the packets to prevent a malicious peer
   * from using all of our memory.
   *
   * This is patch follows the VIRTIO spec: "VIRTIO_VSOCK_OP_RW data
   * packets MUST only be transmitted when the peer has sufficient
   * free buffer space for the payload"
   */

  if (len > conn->rx_buf_alloc)
    {
      vrterr("ERROR: pkt len larger than rx_buf_alloc\n");
      return;
    }

  /* Copy buffer to the user msg if user is waiting */

  if (conn->rx_msg != NULL)
    {
      copied = vsock_copy_pkt2msg(conn, conn->rx_msg, pkt);
    }

  if (copied > 0)
    {
      vsock_update_rx_credit(conn, copied);
    }

  /* If still has data in rx pkt, copy to socket buffer */

  copied += vsock_copy_pkt2circ(conn, pkt);
  DEBUGASSERT(copied == len);
}

/****************************************************************************
 * Name: vsock_recv_connected
 ****************************************************************************/

static int vsock_recv_connected(FAR struct vsock_conn_s *conn,
                                FAR struct vsock_pkt_s *pkt)
{
  FAR struct vsock_hdr_s *hdr = vsock_pkt2hdr(pkt);

  switch (hdr->op)
    {
      case VIRTIO_VSOCK_OP_RW:
        vsock_recv_enqueue(conn, pkt);
        vsock_notify(conn, VSOCK_NOTIFY_RECV);
        break;
      case VIRTIO_VSOCK_OP_CREDIT_REQUEST:
        vsock_send_credit(conn);
        break;
      case VIRTIO_VSOCK_OP_CREDIT_UPDATE:
        vsock_notify(conn, VSOCK_NOTIFY_SEND);
        break;
      case VIRTIO_VSOCK_OP_SHUTDOWN:
        if (hdr->flags & VIRTIO_VSOCK_SHUTDOWN_RECV)
          {
            conn->shutdown |= SHUT_WR;
          }

        if (hdr->flags & VIRTIO_VSOCK_SHUTDOWN_SEND)
          {
            conn->shutdown |= SHUT_RD;
          }

        if (VSOCK_IS_SHUTDOWN(conn->shutdown))
          {
            vsockdbg("Recv SHUTDOWN, flags=0x%" PRIx32 "\n", hdr->flags);
            vsock_reset(conn, pkt);
          }

        vsock_notify(conn, VSOCK_NOTIFY_SHUTDOWN);
        break;
      case VIRTIO_VSOCK_OP_RST:
        vsockdbg("Recv reset\n");
        conn->shutdown |= SHUT_RDWR;
        vsock_notify(conn, VSOCK_NOTIFY_SHUTDOWN);
        break;
      default:
        return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: vsock_recv_closing
 ****************************************************************************/

static void vsock_recv_closing(FAR struct vsock_conn_s *conn,
                               FAR struct vsock_pkt_s *pkt)
{
  FAR struct vsock_hdr_s *hdr = vsock_pkt2hdr(pkt);

  if (hdr->op == VIRTIO_VSOCK_OP_SHUTDOWN)
    {
      vsock_reset(conn, pkt);
    }
  else if (hdr->op == VIRTIO_VSOCK_OP_RST)
    {
      work_cancel_sync_wq(g_vsock_wqueue, &conn->close_work);
      vsock_remove_conn(conn);
      if (_SS_ISCLOSED(conn->sconn.s_flags))
        {
          vsock_sub_ref(conn);
        }
    }
}

/****************************************************************************
 * Name: vsock_close_work
 ****************************************************************************/

static void vsock_close_work(FAR void *arg)
{
  FAR struct vsock_conn_s *conn = arg;

  vsockdbg("close_work conn, conn=%p\n", conn);
  vsock_lock(conn);

  /* Send RST to the peer */

  vsock_reset(conn, NULL);
  vsock_remove_conn(conn);

  vsock_unlock(conn);
  vsock_sub_ref(conn);
}

/****************************************************************************
 * Name: vsock_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 *   Only support socket type SOCK_STREAM will be supported
 *   later.
 *
 * Input Parameters:
 *   psock    A pointer to a user allocated socket structure
 *            to be initialized.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int vsock_setup(FAR struct socket *psock)
{
  FAR struct vsock_conn_s *conn;

  if (psock->s_type == SOCK_STREAM)
    {
      conn = vsock_alloc();
      if (conn == NULL)
        {
          nwarn("WARNING: Failed to allocate vsock connection\n");
          return -ENOMEM;
        }

      psock->s_conn = conn;
      return OK;
    }

  nwarn("WARNING: Unsupported vsock socket type: %d\n", psock->s_type);
  return -ENOTSUP;
}

/****************************************************************************
 * Name: vsock_sockcaps
 *
 * Description:
 *   Return the bit encoded capabilities of this socket.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose capabilities are being
 *           queried.
 *
 * Returned Value:
 *   The set of socket capabilities is returned.
 *
 ****************************************************************************/

static sockcaps_t vsock_sockcaps(FAR struct socket *psock)
{
  return SOCKCAP_NONBLOCKING;
}

/****************************************************************************
 * Name: vsock_addref
 *
 * Description:
 *   Increment the reference count on the underlying connection structure.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose reference count will be
 *           incremented.
 *
 ****************************************************************************/

static void vsock_addref(FAR struct socket *psock)
{
  FAR struct vsock_conn_s *conn = psock->s_conn;

  vsock_add_ref(conn);
}

/****************************************************************************
 * Name: vsock_bind
 *
 * Description:
 *   Validate the sockaddr and call vsock_bind_internal() to get vm port and
 *   add connection to the bound list to wait peer connect()
 *
 * Input Parameters:
 *   psock    Socket structure of the socket to bind
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success;  A negated errno value is returned on failure.  See
 *   bind() for a list a appropriate error values.
 *
 ****************************************************************************/

static int vsock_bind(FAR struct socket *psock,
                      FAR const struct sockaddr *addr,
                      socklen_t addrlen)
{
  FAR const struct sockaddr_vm *vmaddr =
    (FAR const struct sockaddr_vm *)addr;
  FAR struct vsock_conn_s *conn = psock->s_conn;
  int ret;

  ret = vsock_addr_is_valid(addr, addrlen);
  if (ret < 0)
    {
      nerr("ERROR: Invalid vsock bind address: %d\n", ret);
      return ret;
    }

  vsock_lock(conn);
  ret = vsock_bind_internal(conn, vmaddr);
  vsock_unlock(conn);
  return ret;
}

/****************************************************************************
 * Name: vsock_getsockname
 *
 * Description:
 *   The vsock_getsockname() function retrieves the locally-bound name of
 *   the specified virtual socket, stores this address in the sockaddr
 *   structure pointed to by the 'addr' argument, and stores the length of
 *   this address in the object pointed to by the 'addrlen' argument.
 *
 * Input Parameters:
 *   psock    Socket structure of the socket to be queried
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address.  Otherwise, a negated errno value is returned.  See
 *   getsockname() for the list of appropriate error numbers.
 *
 ****************************************************************************/

static int vsock_getsockname(FAR struct socket *psock,
                             FAR struct sockaddr *addr,
                             FAR socklen_t *addrlen)
{
  FAR struct vsock_conn_s *conn = psock->s_conn;
  int ret;

  ret = vsock_addr_get(&conn->local_addr, addr, addrlen);
  if (ret >= 0 && conn->transport != NULL)
    {
      FAR struct sockaddr_vm *vmaddr = (FAR struct sockaddr_vm *)addr;

      vmaddr->svm_cid = VSOCK_GET_LOCALCID(conn->transport);
    }

  return ret;
}

/****************************************************************************
 * Name: vsock_getpeername
 *
 * Description:
 *   The vsock_getpeername() function retrieves the remote-connected name of
 *   the specified virtual socket, stores this address in the sockaddr
 *   structure pointed to by the 'addr' argument, and stores the length of
 *   this address in the object pointed to by the 'addrlen' argument.
 *
 * Parameters:
 *   psock    Socket structure of the socket to be queried
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address.  Otherwise, a negated errno value is returned.  See
 *   getpeername() for the list of appropriate error numbers.
 *
 ****************************************************************************/

static int vsock_getpeername(FAR struct socket *psock,
                             FAR struct sockaddr *addr,
                             FAR socklen_t *addrlen)
{
  FAR struct vsock_conn_s *conn = psock->s_conn;

  vsock_addr_get(&conn->remote_addr, addr, addrlen);
  return 0;
}

/****************************************************************************
 * Name: vsock_listen
 *
 * Description:
 *   To accept connections, a socket is first created with psock_socket(), a
 *   willingness to accept incoming connections and a queue limit for
 *   incoming connections are specified with psock_listen(), and then the
 *   connections are accepted with psock_accept().  For the case of local
 *   Unix sockets, psock_listen() calls this function.  The psock_listen()
 *   call applies only to sockets of type SOCK_STREAM.
 *
 * Input Parameters:
 *   psock    Reference to an internal, boound socket structure.
 *   backlog  The maximum length the queue of pending connections may grow.
 *            If a connection request arrives with the queue full, the client
 *            may receive an error with an indication of ECONNREFUSED or,
 *            if the underlying protocol supports retransmission, the request
 *            may be ignored so that retries succeed.
 *
 * Returned Value:
 *   On success, zero is returned. On error, a negated errno value is
 *   returned.  See listen() for the set of appropriate error values.
 *
 ****************************************************************************/

static int vsock_listen(FAR struct socket *psock, int backlog)
{
  FAR struct vsock_conn_s *conn = psock->s_conn;

  if (psock->s_type != SOCK_STREAM)
    {
      nerr("ERROR: vsock listen only supports SOCK_STREAM\n");
      return -ENOSYS;
    }

  if (!_SS_ISBOUND(conn->sconn.s_flags) || backlog <= 0)
    {
      nerr("ERROR: vsock listen requires bound socket and backlog > 0\n");
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: vsock_connect
 *
 * Description:
 *   vsock_connect() connects the virtual socket referred to by the structure
 *   'psock' to the address specified by 'addr'. The addrlen argument
 *   specifies the size of 'addr'.  The format of the address in 'addr' is
 *   determined by the address space of the socket 'psock'.
 *
 * Input Parameters:
 *   psock     Pointer to a socket structure initialized by psock_socket()
 *   addr      Server address (form depends on type of socket)
 *   addrlen   Length of actual 'addr'
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.  See connect() for the
 *   list of appropriate errno values to be returned.
 *
 ****************************************************************************/

static int vsock_connect(FAR struct socket *psock,
                         FAR const struct sockaddr *addr,
                         socklen_t addrlen)
{
  FAR struct vsock_conn_s *conn = psock->s_conn;
  FAR struct sockaddr_vm vmaddr;
  int ret;

  if (_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      nwarn("WARNING: vsock already connected\n");
      return -EISCONN;
    }

  /* Set the remote address */

  ret = vsock_addr_set(&conn->remote_addr, addr, addrlen);
  if (ret < 0)
    {
      nerr("ERROR: Invalid vsock connect address: %d\n", ret);
      return ret;
    }

  vsock_lock(conn);

  /* Get the local port and insert to the bound list */

  vsock_addr_init(&vmaddr, VMADDR_CID_ANY, VMADDR_PORT_ANY);
  ret = vsock_bind_internal(conn, &vmaddr);
  if (ret < 0)
    {
      nerr("ERROR: Failed to auto-bind vsock local port: %d\n", ret);
      goto out;
    }

  conn->sconn.s_flags |= _SF_BOUND;

  /* Get transport function */

  ret = vsock_assgin_transport(conn);
  if (ret < 0)
    {
      nerr("ERROR: Failed to assign vsock transport: %d\n", ret);
      goto err_with_bind;
    }

  /* Send connect msg */

  ret = vsock_send_pkt(conn, NULL, VIRTIO_VSOCK_OP_REQUEST, 0, 0, 0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to send vsock connect request: %d\n", ret);
      goto err_with_bind;
    }

  conn->sconn.s_flags |= _SF_CONNECTING;
  if (_SS_ISNONBLOCK(conn->sconn.s_flags))
    {
      ret = -EINPROGRESS;
      goto out;
    }

  vsockdbg("conn=%p\n", conn);
  vsock_unlock(conn);
  ret = net_sem_timedwait(&conn->tx_sem,
                          _SO_TIMEOUT(conn->sconn.s_sndtimeo));
  vsock_lock(conn);
  if (conn->sconn.s_error < 0)
    {
      ret = conn->sconn.s_error;
      goto err_with_bind;
    }
  else if (ret >= 0)
    {
      goto out;
    }

err_with_bind:
  vsock_unbind_internal(conn);
out:
  vsock_unlock(conn);
  return ret;
}

/****************************************************************************
 * Name: vsock_accept
 *
 * Description:
 *   This function implements accept() for virtual sockets.  See the
 *   description of accept() for further information.
 *
 * Input Parameters:
 *   psock    The listening Virtual domain socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr',
 *            Return: returned size of 'addr'
 *   newconn  The new, accepted Virtual domain connection structure
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *   See the description of accept of the possible errno values in the
 *   description of accept().
 *
 ****************************************************************************/

static int vsock_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                        FAR socklen_t *addrlen, FAR struct socket *newsock,
                        int flags)
{
  FAR struct vsock_conn_s *server = psock->s_conn;
  int ret = 0;

  if (!_SS_ISLISTENING(server->sconn.s_flags))
    {
      return -EINVAL;
    }

  for (; ; )
    {
      FAR struct vsock_conn_s *conn = NULL;

      vsock_lock(server);
      while (!list_is_empty(&server->child))
        {
          conn = list_first_entry(&server->child, struct vsock_conn_s,
                                  child);
          list_delete(&conn->child);

          vsock_lock(conn);
          if (VSOCK_IS_SHUTDOWN(conn->shutdown))
            {
              vsock_remove_conn(conn);
              vsock_unlock(conn);
              vsock_sub_ref(conn);
              conn = NULL;
            }
          else
            {
              vsock_unlock(conn);
              break;
            }
        }

      vsock_unlock(server);

      if (conn)
        {
          newsock->s_domain = psock->s_domain;
          newsock->s_sockif = psock->s_sockif;
          newsock->s_type = SOCK_STREAM;
          newsock->s_conn = conn;

          vsockdbg("conn=%p\n", conn);
          vsock_addr_get(&conn->remote_addr, addr, addrlen);
          break;
        }

      if (_SS_ISNONBLOCK(server->sconn.s_flags))
        {
          ret = -EAGAIN;
          break;
        }

      ret = net_sem_wait(&server->rx_sem);
      if (ret < 0)
        {
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: vsock_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to local_poll which, indiectly, calls to function.
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

static int vsock_poll(FAR struct socket *psock,
                      FAR struct pollfd_s *fds, bool setup)
{
  FAR struct vsock_conn_s *conn = psock->s_conn;
  pollevent_t eventset = 0;
  irqstate_t flags;
  int ret = 0;
  int i;

  if (setup)
    {
      flags = spin_lock_irqsave(&conn->poll_lock);

      for (i = 0; i < CONFIG_NET_VSOCK_NPOLLWAITERS; i++)
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

      spin_unlock_irqrestore(&conn->poll_lock, flags);

      if (i >= CONFIG_NET_VSOCK_NPOLLWAITERS)
        {
          nwarn("WARNING: No available vsock poll slots\n");
          fds->priv = NULL;
          return -EBUSY;
        }

      /* Immediately notify on any of the requested events */

      vsock_lock(conn);
      if (_SS_ISLISTENING(conn->sconn.s_flags))
        {
          if (!list_is_empty(&conn->child))
            {
              eventset |= POLLIN;
            }
        }
      else if (_SS_ISCONNECTED(conn->sconn.s_flags))
        {
          if (VSOCK_IS_SHUTDOWN_WR(conn->shutdown))
            {
              eventset |= POLLHUP;
            }

          if (vsock_get_tx_credit(conn) > 0)
            {
              eventset |= POLLOUT;
            }

          if (VSOCK_IS_SHUTDOWN_RD(conn->shutdown) ||
              vsock_get_rx_size(conn) >= conn->rx_lowat)
            {
              eventset |= POLLIN;
            }
        }

      vsock_unlock(conn);
      vsock_poll_notify(conn, eventset);
    }
  else
    {
      flags = spin_lock_irqsave(&conn->poll_lock);

      if (fds->priv != NULL)
        {
          for (i = 0; i < CONFIG_NET_VSOCK_NPOLLWAITERS; i++)
            {
              if (fds == conn->fds[i])
                {
                  conn->fds[i] = NULL;
                  fds->priv = NULL;
                  break;
                }
            }
        }

      spin_unlock_irqrestore(&conn->poll_lock, flags);
    }

  return ret;
}

/****************************************************************************
 * Name: vsock_msg_init
 *
 * Description:
 *   Virtual socket message init function
 *
 ****************************************************************************/

static void vsock_msg_init(FAR struct vsock_msg_s *msg,
                            FAR const struct msghdr *hdr)
{
  uio_init(&msg->uio, hdr->msg_iov, hdr->msg_iovlen);
  msg->len = msg->uio.uio_resid;
}

/****************************************************************************
 * Name: vsock_sendmsg
 *
 * Description:
 *   Implements the sendmsg() operation for the case of the Virtual socket
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      msg to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see sendmsg() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

static ssize_t vsock_sendmsg(FAR struct socket *psock,
                             FAR const struct msghdr *hdr, int flags)
{
  FAR struct vsock_conn_s *conn = psock->s_conn;
  FAR const struct sockaddr *to = hdr->msg_name;
  struct vsock_msg_s msg;
  ssize_t ret = 0;
  bool nonblock;

  vsock_tx_lock(conn);

  if (!_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      if (to == NULL)
        {
          ret = -ENOTCONN;
          goto out;
        }

      ret = vsock_connect(psock, to, hdr->msg_namelen);
      if (ret < 0)
        {
          nwarn("WARNING: Vsock auto-connect in sendmsg failed: %zd\n", ret);
          goto out;
        }
    }

  if (conn->shutdown & SHUT_WR)
    {
      ret = -EPIPE;
      goto out;
    }

  nonblock = _SS_ISNONBLOCK(conn->sconn.s_flags) ||
             (flags & MSG_DONTWAIT) != 0;

  vsock_msg_init(&msg, hdr);
  while (!vsock_msg_done(&msg))
    {
      if (conn->shutdown & SHUT_WR)
        {
          ret = -EPIPE;
          goto out;
        }

      if (vsock_get_tx_credit(conn) > 0)
        {
          ret = vsock_send_pkt(conn, &msg, VIRTIO_VSOCK_OP_RW, 0, 0, 0);
          break;
        }

      if (nonblock)
        {
          ret = -EAGAIN;
          break;
        }

      vsock_tx_unlock(conn);
      ret = net_sem_timedwait(&conn->tx_sem,
                              _SO_TIMEOUT(conn->sconn.s_sndtimeo));
      vsock_tx_lock(conn);

      if (ret < 0)
        {
          vrterr("vsock_sendmsg: tx_credit timeout %zd\n", ret);
          break;
        }
    }

out:
  vsock_tx_unlock(conn);
  return ret;
}

/****************************************************************************
 * Name: vsock_recvmsg
 *
 * Description:
 *   recvmsg() receives messages from a local socket and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive the message
 *   flags    Receive flags (ignored for now)
 *
 * Returned Value:
 *   On success, returns the number of characters received. If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recvmsg() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvmsg() for the list of appropriate error values).
 *
 ****************************************************************************/

static ssize_t vsock_recvmsg(FAR struct socket *psock,
                             FAR struct msghdr *hdr, int flags)
{
  FAR struct vsock_conn_s *conn = psock->s_conn;
  FAR struct sockaddr *from = hdr->msg_name;
  FAR socklen_t *fromlen = &hdr->msg_namelen;
  struct vsock_msg_s msg;
  ssize_t ret;

  if (psock->s_type != SOCK_STREAM)
    {
      return -ENOTSUP;
    }

  if (!_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      return -EISCONN;
    }

  vsock_msg_init(&msg, hdr);
  vsock_rx_lock(conn);

  /* Try to recv the buffer from rx_buf */

  ret = vsock_copy_circ2msg(conn, &msg);
  if (ret > 0)
    {
      vsock_update_rx_credit(conn, ret);
    }
  else if (VSOCK_IS_SHUTDOWN_RD(conn->shutdown))
    {
      ret = 0;
    }
  else if (_SS_ISNONBLOCK(conn->sconn.s_flags) ||
           (flags & MSG_DONTWAIT) != 0)
    {
      ret = -EAGAIN;
    }
  else /* ret == 0 */
    {
      /* No data recv, setup the user msg and wait */

      DEBUGASSERT(ret == 0);
      conn->rx_msg = &msg;
      nxsem_reset(&conn->rx_sem, 0);
      vsock_rx_unlock(conn);
      ret = net_sem_timedwait(&conn->rx_sem,
                              _SO_TIMEOUT(conn->sconn.s_rcvtimeo));
      vsock_rx_lock(conn);
      conn->rx_msg = NULL;
      if (msg.len > msg.uio.uio_resid)
        {
          ret = msg.len - msg.uio.uio_resid;
        }
    }

  vsock_rx_unlock(conn);
  if (ret > 0)
    {
      vsock_addr_get(&conn->remote_addr, from, fromlen);
    }

  return ret;
}

/****************************************************************************
 * Name: vsock_close
 *
 * Description:
 *   Performs the close operation on a Virtual socket instance
 *
 * Input Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *   0 on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int vsock_close(FAR struct socket *psock)
{
  FAR struct vsock_conn_s *conn = psock->s_conn;
  int ret = OK;

  vsock_lock(conn);

  conn->sconn.s_flags |= _SF_CLOSED;

  /* Condition 1: socket()  created connection, bind or connect waiting
   * Condition 2: socket()  created connection, listen
   * Condition 3: accept()  created connection, must in connected state
   */

  if (VSOCK_IS_SHUTDOWN(conn->shutdown))
    {
      vsockdbg("Peer has closed conn=%p s_flags=0x%x\n",
               conn, conn->sconn.s_flags);
      vsock_remove_conn(conn);
    }
  else if (_SS_ISCONNECTED(conn->sconn.s_flags) ||
           _SS_ISCONNECTING(conn->sconn.s_flags))
    {
      vsock_add_ref(conn);
      conn->shutdown = SHUT_RDWR;

      /* Follow Virtio Spec 1.2 to do the clean disconnect:
       * Clean disconnect is achieved by one or more
       * VIRTIO_VSOCK_OP_SHUTDOWN packets that indicate no more data will
       * be sent and received, followed by a VIRTIO_VSOCK_OP_RST response
       * from the peer. If no VIRTIO_VSOCK_OP_RST response is received within
       * an implementation-specific amount of time, a VIRTIO_VSOCK_OP_RST
       * packet is sent to forcibly disconnect the socket.
       */

      vsockdbg("send shutdown to peer conn=%p\n", conn);
      ret = vsock_send_pkt(conn, NULL, VIRTIO_VSOCK_OP_SHUTDOWN,
                           VIRTIO_VSOCK_SHUTDOWN, 0, 0);
      if (ret < 0)
        {
          vsockdbg("send shutdown failed conn=%p\n", conn);
          vsock_unlock(conn);
          vsock_sub_ref(conn);
          return ret;
        }

      /* Queue the close timeout work */

      if (vsock_check_conn(conn))
        {
          vsockdbg("queue the close work conn=%p\n", conn);
          ret = vsock_queue_work(&conn->close_work, vsock_close_work, conn,
                                 VSOCK_CLOSE_TIMEOUT);
        }
    }
  else if (_SS_ISBOUND(conn->sconn.s_flags) ||
           _SS_ISLISTENING(conn->sconn.s_flags))
    {
      vsock_unbind_internal(conn);
    }

  vsock_unlock(conn);
  vsock_sub_ref(conn);
  return ret;
}

/****************************************************************************
 * Name: vsock_shutdown
 *
 * Description:
 *   The shutdown() function will cause all or part of connection on the
 *   socket associated with the file descriptor socket to be shut down.
 *
 *   The shutdown() function disables subsequent send and/or receive
 *   operations on a socket, depending on the value of the how argument.
 *
 * Parameters:
 *   sockfd - Specifies the file descriptor of the socket.
 *   how    - Specifies the type of shutdown. The values are as follows:
 *
 *     SHUT_RD   - Disables further receive operations.
 *     SHUT_WR   - Disables further send operations.
 *     SHUT_RDWR - Disables further send and receive operations.
 *
 ****************************************************************************/

static int vsock_shutdown(FAR struct socket *psock, int how)
{
  FAR struct vsock_conn_s *conn = psock->s_conn;
  int ret;

  vsock_lock(conn);
  vsockdbg("send shutdown to peer conn=%p\n", conn);
  conn->shutdown |= how;
  ret = vsock_send_pkt(conn, NULL, VIRTIO_VSOCK_OP_SHUTDOWN, how, 0, 0);
  if (ret >= 0)
    {
      vsock_notify(conn, VSOCK_NOTIFY_SHUTDOWN);
    }

  vsock_unlock(conn);
  return ret;
}

/****************************************************************************
 * Name: vsock_ioctl
 *
 * Description:
 *   This function performs virtual socket specific operations.
 *
 * Parameters:
 *   psock    A reference to the socket structure of the socket
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *
 ****************************************************************************/

static int vsock_ioctl(FAR struct socket *psock, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: vsock_getsockopt
 ****************************************************************************/

#ifdef CONFIG_NET_SOCKOPTS
static int vsock_getsockopt(FAR struct socket *psock, int level, int option,
                            FAR void *value, FAR socklen_t *value_len)
{
  FAR struct vsock_conn_s *conn = psock->s_conn;
  int ret = -ENOPROTOOPT;

  if (level != SOL_SOCKET)
    {
      return ret;
    }

  switch (option)
    {
      case SO_RCVLOWAT:
        {
          FAR unsigned long *rcvlowat = value;

          if (value == NULL || value_len == NULL)
            {
              return -EINVAL;
            }

          vsock_rx_lock(conn);
          *rcvlowat = conn->rx_lowat;
          *value_len = sizeof(unsigned long);
          vsock_rx_unlock(conn);
          ret = OK;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: vsock_setsockopt
 ****************************************************************************/

static int vsock_setsockopt(FAR struct socket *psock, int level, int option,
                            FAR const void *value, socklen_t value_len)
{
  FAR struct vsock_conn_s *conn = psock->s_conn;
  int ret = -ENOPROTOOPT;

  if (level != SOL_SOCKET)
    {
      return ret;
    }

  switch (option)
    {
      case SO_RCVLOWAT:
        if (value == NULL || value_len != sizeof(unsigned long))
          {
            return -EINVAL;
          }

        vsock_rx_lock(conn);
        conn->rx_lowat = MIN(*(FAR unsigned long *)value,
                             conn->rx_buf_alloc);
        vsock_rx_unlock(conn);
        ret = OK;
        break;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: vsock_dump_pkt
 ****************************************************************************/

#ifdef VSOCK_DEBUG
static void vsock_dump_pkt(FAR struct vsock_pkt_s *pkt,
                           FAR const char *msg, bool dumpbuf)
{
  FAR struct vsock_hdr_s *hdr = vsock_pkt2hdr(pkt);
  size_t i;

  _alert("%s pkt: %p src: cid=%" PRIu64 " port=%" PRIu32
         " dst: cid=%" PRIu64 " port=%" PRIu32
         " op=%s type=%" PRIu16 " len=%" PRIu32 " flags=0x%" PRIx32
         " rx_buf_alloc=%" PRIu32 " rx_fwd_cnt=%" PRIu32 " \n",
         msg, pkt, hdr->src_cid, hdr->src_port, hdr->dst_cid, hdr->dst_port,
         g_vsock_op[hdr->op], hdr->type, hdr->len, hdr->flags,
         hdr->buf_alloc, hdr->fwd_cnt);

  if (dumpbuf)
    {
      for (i = 0; i < pkt->vbcnt; i++)
        {
          lib_dumpbuffer("Virt Buffer:", pkt->vb[i].buf, pkt->vb[i].len);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsock_recv_pkt
 ****************************************************************************/

void vsock_recv_pkt(FAR struct vsock_transport_s *t,
                    FAR struct vsock_pkt_s *pkt)
{
  FAR struct vsock_hdr_s *hdr = vsock_pkt2hdr(pkt);
  FAR struct vsock_conn_s *conn;
  struct sockaddr_vm remote_addr;
  struct sockaddr_vm local_addr;

  vsock_dump_pkt(pkt, "Recv", false);

  /* Type check */

  if (hdr->type != VIRTIO_VSOCK_TYPE_STREAM)
    {
      vrterr("Operation Type Error\n");
      vsock_reset_no_conn(t, pkt);
      return;
    }

  /* Find the connected or bound socket in the list, the refcnt will be
   * added when a connection is found in bound or connected list.
   */

  vsock_addr_init(&remote_addr, hdr->src_cid, hdr->src_port);
  vsock_addr_init(&local_addr, hdr->dst_cid, hdr->dst_port);
  conn = vsock_find_connected_conn(&remote_addr, &local_addr);
  if (conn == NULL)
    {
      conn = vsock_find_bound_conn(&local_addr);
      if (conn == NULL)
        {
          vsock_reset_no_conn(t, pkt);
          vrterr("ERROR: No connection found\n");
          return;
        }
    }

  vsock_lock(conn);

  /* Update the window */

  vsock_update_tx_credit(conn, pkt);

  /* Handle the receiving data */

  if (VSOCK_IS_SHUTDOWN(conn->shutdown))
    {
      vsock_recv_closing(conn, pkt);
    }
  else if (_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      vsock_recv_connected(conn, pkt);
    }
  else if (_SS_ISLISTENING(conn->sconn.s_flags))
    {
      vsock_recv_listening(conn, t, pkt);
    }
  else if (_SS_ISBOUND(conn->sconn.s_flags))
    {
      vsock_recv_connecting(conn, pkt);
    }
  else
    {
      vsock_reset_no_conn(t, pkt);
    }

  vsock_unlock(conn);
  vsock_sub_ref(conn);
}

/****************************************************************************
 * Name: vsock_queue_work
 *
 * Description:
 *   Work queue for the virtual socket layer.
 *
 ****************************************************************************/

int vsock_queue_work(FAR struct work_s *work, worker_t worker, FAR void *arg,
                     clock_t delay)
{
  return work_queue_wq(g_vsock_wqueue, work, worker, arg, delay);
}

/****************************************************************************
 * Name: vsock_conn_dump
 *
 * Description:
 *   Dump vsock connections owned by the specified transport.
 *
 ****************************************************************************/

void vsock_conn_dump(FAR struct vsock_transport_s *t)
{
  FAR struct vsock_conn_s *conn;
  irqstate_t flags;
  int i = 0;

  flags = spin_lock_irqsave(&g_vsock_conn_lock);
  list_for_every_entry(&g_vsock_conn, conn, struct vsock_conn_s, node)
    {
      if (conn->transport != t)
        {
          continue;
        }

      syslog(LOG_EMERG, "    conn[%d]: %s, local=%u:%u"
                        ", remote=%u:%u, flags=0x%x"
                        ", rx=%zu/%zu, tx_credit=%zu\n",
             i++,
             _SS_ISLISTENING(conn->sconn.s_flags) ? "server" : "client",
             conn->local_addr.svm_cid, conn->local_addr.svm_port,
             conn->remote_addr.svm_cid, conn->remote_addr.svm_port,
             conn->sconn.s_flags,
             circbuf_used(&conn->rx_buf),
             circbuf_size(&conn->rx_buf),
             vsock_get_tx_credit(conn));
    }

  if (i == 0)
    {
      syslog(LOG_EMERG, "    (no connections)\n");
    }

  spin_unlock_irqrestore(&g_vsock_conn_lock, flags);
}

/****************************************************************************
 * Name: vsock_transport_register
 *
 * Description:
 *   Register the virtual socket transport.
 *
 ****************************************************************************/

void vsock_transport_register(FAR struct vsock_transport_s *t)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_vsock_transport_lock);
  list_add_tail(&g_vsock_transport, &t->node);
  spin_unlock_irqrestore(&g_vsock_transport_lock, flags);
}

/****************************************************************************
 * Name: vsock_initialize
 *
 * Description:
 *   Initialize the virtual socket layer.
 *
 ****************************************************************************/

int vsock_initialize(void)
{
  int ret = OK;

  g_vsock_wqueue = work_queue_create("vsock_work",
                                     CONFIG_NET_VSOCK_WORK_PRIORITY,
                                     NULL,
                                     CONFIG_NET_VSOCK_WORK_STACKSIZE, 1);
  if (g_vsock_wqueue == NULL)
    {
      vrterr("Create vsock wqueue failed\n");
      return -ENOMEM;
    }

#ifdef CONFIG_NET_VSOCK_VIRTIO
  ret = vsock_virtio_initialize();
  if (ret < 0)
    {
      vrterr("ERROR: vsock_virtio_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_NET_VSOCK_VHOST
  ret = vsock_vhost_initialize();
  if (ret < 0)
    {
      vhosterr("ERROR: vsock_vhost_initialize failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_NET_VSOCK_LOCAL
  ret = vsock_local_initialize();
  if (ret < 0)
    {
      vrterr("ERROR: vsock_local_initialize failed, ret=%d\n", ret);
    }
#endif

  return ret;
}
