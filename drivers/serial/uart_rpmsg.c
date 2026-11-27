/****************************************************************************
 * drivers/serial/uart_rpmsg.c
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

#include <errno.h>
#include <inttypes.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <sys/param.h>
#include <sys/types.h>
#include <termios.h>

#include <nuttx/debug.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>
#include <nuttx/ring.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/serial/serial.h>
#include <nuttx/serial/tioctl.h>
#include <nuttx/serial/uart_rpmsg.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_RPMSG_DEV_CONSOLE          "/dev/console"
#define UART_RPMSG_DEV_PREFIX           "/dev/tty"
#define UART_RPMSG_EPT_PREFIX           "rpmsg-tty"
#define UART_RPMSG_NAME_SIZE            (RPMSG_NAME_SIZE - 10)

#define UART_RPMSG_SYNC                 0  /* Handshake: pos == recv cap */
#define UART_RPMSG_DATA                 1  /* Payload and/or position sync */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Counter-based flow control protocol.  Each frame carries an explicit
 * command:
 *
 *   - SYNC : sent once after ns_bound, len == 0, pos == local recv
 *            capacity (recv.size - 1).  Establishes the peer send window.
 *   - DATA : pos always piggybacks the sender's consumed position
 *            (recvpos) to acknowledge the reverse stream.  len > 0 carries
 *            payload; len == 0 is a pure position sync (an ack with no
 *            payload).
 */

begin_packed_struct struct uart_rpmsg_msg_s
{
  uint32_t cmd;                  /* UART_RPMSG_SYNC / DATA */
  uint32_t pos;                  /* recvpos (piggyback ack); for SYNC:
                                  * local recv buffer capacity */
  uint32_t len;                  /* Payload length; 0 means position-only */
  char     data[];
} end_packed_struct;

struct uart_rpmsg_priv_s
{
  struct rpmsg_endpoint ept;
  char                  devname[UART_RPMSG_NAME_SIZE];
  char                  cpuname[UART_RPMSG_NAME_SIZE];

  uint32_t              sendsize;       /* Peer recv capacity; 0 means not
                                         * connected */
  spinlock_t            lock;           /* Flow-control counters / TX path lock */
  uint16_t              crefs;          /* open()/close() reference count */

  uint32_t              sendpos;        /* Cumulative bytes we have sent */
  uint32_t              ackpos;         /* Cumulative bytes the peer consumed */
  uint32_t              received;       /* Cumulative bytes written to recv */
  uint32_t              lastpos;        /* recvpos at the last ack we sent */
  sem_t                 sendsem;        /* dmasend waits, ept_cb posts */
  sem_t                 recvsem;        /* read() blocks here; ept_cb posts */

  struct uart_termios_s termios;        /* c_iflag/c_oflag/c_lflag */

#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGTSTP)
  pid_t                 pid;            /* Foreground task PID for iob control */
#endif
  FAR struct pollfd_s   *fds[CONFIG_RPMSG_UART_NPOLLWAITERS];
  struct ring_s         ring;           /* Ring handle */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     uart_rpmsg_open(FAR struct file *filep);
static int     uart_rpmsg_close(FAR struct file *filep);
static ssize_t uart_rpmsg_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen);
static ssize_t uart_rpmsg_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen);
static int     uart_rpmsg_ioctl(FAR struct file *filep, int cmd,
                                unsigned long arg);
static int     uart_rpmsg_poll(FAR struct file *filep,
                               FAR struct pollfd_s *fds, bool setup);

static void    uart_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                         FAR void *priv_);
static void    uart_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                         FAR void *priv_);
static int     uart_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len, uint32_t src,
                                 FAR void *priv_);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_uart_rpmsg_fops =
{
  uart_rpmsg_open,
  uart_rpmsg_close,
  uart_rpmsg_read,
  uart_rpmsg_write,
  NULL,               /* seek */
  uart_rpmsg_ioctl,
  NULL,               /* mmap */
  NULL,               /* truncate */
  uart_rpmsg_poll,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Post the send semaphore without letting its count grow unbounded. */

static void uart_rpmsg_post(FAR sem_t *sem)
{
  int semcount;

  nxsem_get_value(sem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(sem);
    }
}

/* Bytes we may still send before the peer recv ring is full.  Returns 0 when
 * not connected: sendsize, sendpos and ackpos are all reset to 0 on
 * disconnect, so the subtraction naturally yields 0.  Must be called with
 * dev->lock held.
 */

static inline_function
uint32_t uart_rpmsg_get_space(FAR struct uart_rpmsg_priv_s *priv)
{
  return priv->sendsize - (priv->sendpos - priv->ackpos);
}

/* Send a payload-less ACK so the peer can advance its send window.  Thread
 * context only; must be called without priv->lock held, since rpmsg_send
 * may block.  Returns rpmsg_send status, -ENOTCONN if down.
 */

static int uart_rpmsg_send_ack(FAR struct uart_rpmsg_priv_s *priv)
{
  struct uart_rpmsg_msg_s msg;
  uint32_t received;
  irqstate_t flags;
  int ret;

  if (!is_rpmsg_ept_ready(&priv->ept))
    {
      return -ENOTCONN;
    }

  flags = spin_lock_irqsave(&priv->lock);
  received = priv->received;
  spin_unlock_irqrestore(&priv->lock, flags);

  msg.cmd = UART_RPMSG_DATA;
  msg.pos = received;
  msg.len = 0;

  ret = rpmsg_send(&priv->ept, &msg, sizeof(msg));
  if (ret < 0)
    {
      return ret;
    }

  flags = spin_lock_irqsave(&priv->lock);

  if ((int32_t)(received - priv->lastpos) > 0)
    {
      priv->lastpos = received;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  return ret;
}

static int uart_rpmsg_open(FAR struct file *filep)
{
  FAR struct uart_rpmsg_priv_s *priv = filep->f_inode->i_private;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  /* Reference count only: the endpoint is owned by the rpmsg framework, so
   * opening never brings the link up.
   */

  DEBUGASSERT(priv->crefs < UINT16_MAX);
  priv->crefs++;

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

static int uart_rpmsg_close(FAR struct file *filep)
{
  FAR struct uart_rpmsg_priv_s *priv = filep->f_inode->i_private;
  uint32_t dropped = 0;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  DEBUGASSERT(priv->crefs > 0);
  if (--priv->crefs == 0)
    {
      /* Last close: drop unread bytes so a fresh session starts clean, but
       * count them as consumed or the peer window shrinks for good.
       */

      dropped = ring_reset(&priv->ring);
      priv->received += dropped;

#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGTSTP)
      priv->pid = INVALID_PROCESS_ID;
#endif
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  if (dropped > 0)
    {
      uart_rpmsg_send_ack(priv);
    }

  return OK;
}

static ssize_t uart_rpmsg_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen)
{
  FAR struct uart_rpmsg_priv_s *priv = filep->f_inode->i_private;
  struct ring_buf_s rbuf;
  ssize_t ret = 0;
  irqstate_t flags;

  if (buflen == 0)
    {
      return 0;
    }

  flags = spin_lock_irqsave(&priv->lock);

  for (; ; )
    {
      ring_spsc_dequeue_start(&priv->ring, buflen, &rbuf);

      ret = 0;
      if (rbuf.len1 != 0)
        {
          ret += uart_termios_in_copy(&priv->termios, buffer,
                                      rbuf.ptr1, rbuf.len1);
        }

      if (rbuf.len2 != 0)
        {
          ret += uart_termios_in_copy(&priv->termios, &buffer[ret],
                                      rbuf.ptr2, rbuf.len2);
        }

      priv->received += rbuf.cnt;
      ring_spsc_dequeue_finish(&priv->ring, &rbuf);

      if (ret > 0)
        {
          break;
        }
      else if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          break;
        }
      else
        {
          spin_unlock_irqrestore(&priv->lock, flags);
          ret = nxsem_wait(&priv->recvsem);
          flags = spin_lock_irqsave(&priv->lock);
          if (ret < 0)
            {
              break;
            }
        }
    }

  if ((priv->termios.lflag & ECHO) && ret > 0)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      uart_rpmsg_write(filep, buffer, ret);
    }
  else if (priv->received - priv->lastpos > ring_capacity(&priv->ring) / 2)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      uart_rpmsg_send_ack(priv);
    }
  else
    {
      spin_unlock_irqrestore(&priv->lock, flags);
    }

  return ret;
}

static int uart_rpmsg_wait_space(FAR struct file *filep,
                                 FAR struct uart_rpmsg_priv_s *priv)
{
  if (filep->f_oflags & O_NONBLOCK)
    {
      return -EAGAIN;
    }

  return rpmsg_wait(&priv->ept, &priv->sendsem);
}

static ssize_t uart_rpmsg_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen)
{
  FAR struct uart_rpmsg_priv_s *priv = filep->f_inode->i_private;
  size_t nwritten = 0;
  irqstate_t flags;
  ssize_t ret = 0;

  if (buflen == 0)
    {
      return 0;
    }

  while (nwritten < buflen)
    {
      FAR struct uart_rpmsg_msg_s *msg;
      uint32_t received;
      uint32_t space;
      size_t used;
      size_t len;

      flags = spin_lock_irqsave(&priv->lock);

      /* Drop data until the peer SYNC arrives: waiting here deadlocks a core
       * that writes this tty before booting the peer it is connected to.
       */

      if (priv->sendsize == 0)
        {
          spin_unlock_irqrestore(&priv->lock, flags);
          ret = -ENOTCONN;
          break;
        }

      space = uart_rpmsg_get_space(priv);
      spin_unlock_irqrestore(&priv->lock, flags);

      if (space == 0)
        {
          ret = uart_rpmsg_wait_space(filep, priv);
          if (ret < 0)
            {
              break;
            }

          continue;
        }

      msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
      if (msg == NULL)
        {
          ret = -ENOMEM;
          break;
        }

      flags = spin_lock_irqsave(&priv->lock);

      space -= sizeof(*msg);
      space = MIN(uart_rpmsg_get_space(priv), space);

      len = uart_termios_out_copy(&priv->termios, msg->data, space,
                                  &buffer[nwritten], buflen - nwritten,
                                  &used);

      received = priv->received;
      priv->sendpos += len;

      spin_unlock_irqrestore(&priv->lock, flags);

      if (len == 0)
        {
          rpmsg_release_tx_buffer(&priv->ept, msg);
          ret = uart_rpmsg_wait_space(filep, priv);
          if (ret < 0)
            {
              break;
            }

          continue;
        }

      msg->cmd = UART_RPMSG_DATA;
      msg->pos = received;
      msg->len = len;

      ret = rpmsg_send_nocopy(&priv->ept, msg, sizeof(*msg) + len);

      flags = spin_lock_irqsave(&priv->lock);

      if (ret < 0)
        {
          /* Nothing reached the wire, so the peer will never ack these bytes
           * and the reservation has to be given back.
           */

          priv->sendpos -= len;
        }
      else if ((int32_t)(received - priv->lastpos) > 0)
        {
          priv->lastpos = received;
        }

      spin_unlock_irqrestore(&priv->lock, flags);

      if (ret < 0)
        {
          rpmsg_release_tx_buffer(&priv->ept, msg);
          break;
        }

      nwritten += used;
    }

  return nwritten > 0 ? nwritten : ret;
}

static int uart_rpmsg_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  FAR struct uart_rpmsg_priv_s *priv = filep->f_inode->i_private;
  irqstate_t flags;
  int ret = OK;

  switch (cmd)
    {
      case TCGETS:
        {
          FAR struct termios *termiosp =
            (FAR struct termios *)(uintptr_t)arg;

          if (termiosp == NULL)
            {
              return -EINVAL;
            }

          flags = spin_lock_irqsave(&priv->lock);

          uart_termios_getattr(&priv->termios, termiosp);
          spin_unlock_irqrestore(&priv->lock, flags);
        }
        break;

      case TCSETS:
      case TCSETSW:
      case TCSETSF:
        {
          FAR struct termios *termiosp =
            (FAR struct termios *)(uintptr_t)arg;

          if (termiosp == NULL)
            {
              return -EINVAL;
            }

          flags = spin_lock_irqsave(&priv->lock);

          uart_termios_setattr(&priv->termios, termiosp);
          spin_unlock_irqrestore(&priv->lock, flags);
        }
        break;

      case FIONREAD:
        {
          FAR int *nbytes = (FAR int *)(uintptr_t)arg;

          if (nbytes == NULL)
            {
              return -EINVAL;
            }

          *nbytes = ring_count(&priv->ring);
        }
        break;

#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGTSTP)
      case TIOCSCTTY:
        {
          if (arg < 0)
            {
              return -EINVAL;
            }

          priv->pid = (pid_t)arg;
        }
        break;

      case TIOCNOTTY:
        {
          priv->pid = INVALID_PROCESS_ID;
        }
        break;
#endif

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static int uart_rpmsg_poll(FAR struct file *filep, FAR struct pollfd_s *fds,
                           bool setup)
{
  FAR struct uart_rpmsg_priv_s *priv = filep->f_inode->i_private;
  pollevent_t eventset = 0;
  irqstate_t flags;
  int ret = OK;
  int i;

  flags = spin_lock_irqsave_nopreempt(&priv->lock);

  if (setup)
    {
      for (i = 0; i < CONFIG_RPMSG_UART_NPOLLWAITERS; i++)
        {
          if (priv->fds[i] == NULL)
            {
              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_RPMSG_UART_NPOLLWAITERS)
        {
          ret = -EBUSY;
          goto errout;
        }

      if (ring_count(&priv->ring) > 0)
        {
          eventset |= POLLIN;
        }

      if (uart_rpmsg_get_space(priv) > 0)
        {
          eventset |= POLLOUT;
        }

      if (eventset != 0)
        {
          poll_notify(&fds, 1, eventset);
        }
    }
  else
    {
      FAR struct pollfd_s **slot = (FAR struct pollfd_s **)fds->priv;

      if (slot != NULL)
        {
          *slot     = NULL;
          fds->priv = NULL;
        }
    }

errout:
  spin_unlock_irqrestore_nopreempt(&priv->lock, flags);
  return ret;
}

/****************************************************************************
 * RPMsg endpoint callbacks
 ****************************************************************************/

static void uart_rpmsg_ns_bound(FAR struct rpmsg_endpoint *ept)
{
  FAR struct uart_rpmsg_priv_s *priv = ept->priv;
  struct uart_rpmsg_msg_s msg;

  msg.cmd = UART_RPMSG_SYNC;
  msg.pos = ring_capacity(&priv->ring);
  msg.len = 0;

  rpmsg_send(ept, &msg, sizeof(msg));
}

static void uart_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                      FAR void *priv_)
{
  FAR struct uart_rpmsg_priv_s *priv = priv_;
  char eptname[RPMSG_NAME_SIZE];

  if (strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      /* Create the ept at default priority so the first callback (the peer
       * SYNC) runs in the rpmsg work-queue thread, where dmasend may block
       * to flush data queued before the link came up.  ept_cb raises it to
       * RT once the initial backlog is drained.
       */

      priv->ept.priv = priv;
      priv->ept.ns_bound_cb = uart_rpmsg_ns_bound;
      snprintf(eptname, sizeof(eptname), "%s%s",
               UART_RPMSG_EPT_PREFIX, priv->devname);
      rpmsg_create_ept(&priv->ept, rdev, eptname,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       uart_rpmsg_ept_cb, NULL);
    }
}

static void uart_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                      FAR void *priv_)
{
  FAR struct uart_rpmsg_priv_s *priv = priv_;
  irqstate_t flags;

  if (priv->ept.priv != NULL &&
      strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      /* Zero the counters before destroying so a woken writer cannot mistake
       * stale state for an open window, then wake it to let it bail out.
       */

      flags = spin_lock_irqsave(&priv->lock);
      priv->sendsize = 0;
      priv->sendpos  = 0;
      priv->ackpos   = 0;
      spin_unlock_irqrestore(&priv->lock, flags);

      /* Destroying may wait on the endpoint refcount, so it stays outside */

      rpmsg_destroy_ept(&priv->ept);

      /* Kick both wait mechanisms so nobody is left blocked on a link that
       * is gone.  POLLHUP reaches every waiter, subscribed or not.  Same
       * reason as ept_cb for holding the lock across poll_notify.
       */

      flags = spin_lock_irqsave_nopreempt(&priv->lock);
      uart_rpmsg_post(&priv->sendsem);
      uart_rpmsg_post(&priv->recvsem);
      poll_notify(priv->fds, CONFIG_RPMSG_UART_NPOLLWAITERS, POLLHUP);
      spin_unlock_irqrestore_nopreempt(&priv->lock, flags);
    }
}

#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGTSTP)
static void uart_rpmsg_signal_check(FAR struct uart_rpmsg_priv_s *priv,
                                    FAR const char *data, size_t len)
{
  pid_t pid = priv->pid;

  if (pid <= 0)
    {
      return;
    }

  while (len > 0)
    {
      FAR const char *special;
      int signo;

      special = uart_termios_isig_find(&priv->termios, data, len, &signo);
      if (special == NULL)
        {
          break;
        }

      nxsig_kill(pid, signo);

      len -= special - data + 1;
      data = special + 1;
    }
}
#else
#  define uart_rpmsg_signal_check(priv, data, len)
#endif

static int uart_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept, FAR void *data,
                             size_t len, uint32_t src, FAR void *priv_)
{
  FAR struct uart_rpmsg_priv_s *priv = ept->priv;
  FAR struct uart_rpmsg_msg_s *msg = data;
  irqstate_t flags;

  flags = spin_lock_irqsave_nopreempt(&priv->lock);

  if (msg->cmd == UART_RPMSG_SYNC)
    {
      /* Handshake: record the peer recv capacity as our send window and
       * reset the position counters.
       *
       * This first callback runs in the rpmsg work-queue thread (the ept is
       * created at default priority), so we can flush inline any data that
       * was queued into the xmit ring before the link came up: dmasend may
       * block on rpmsg_wait, whose recursive dispatch keeps servicing acks
       * on the same work queue.  Once the backlog is drained, raise the ept
       * to RT priority so steady-state tty traffic is dispatched
       * synchronously with low latency.
       */

      priv->sendsize = msg->pos;
      priv->sendpos  = 0;
      priv->ackpos   = 0;
      priv->received = 0;
      priv->lastpos  = 0;
      rpmsg_set_priority(&priv->ept, RPMSG_PRIO_RT);
    }
  else
    {
      /* The peer position always arrives; payload is optional.  Only DATA
       * frames carry a payload (SYNC is always len 0), so the whole receive
       * path lives in this branch.
       */

      priv->ackpos = msg->pos;

      if (msg->len > 0)
        {
          ring_spsc_enqueue(&priv->ring, msg->data, msg->len);
          uart_rpmsg_signal_check(priv, msg->data, msg->len);
          uart_rpmsg_post(&priv->recvsem);
          poll_notify(priv->fds, CONFIG_RPMSG_UART_NPOLLWAITERS, POLLIN);
        }
    }

  if (uart_rpmsg_get_space(priv) > 0)
    {
      uart_rpmsg_post(&priv->sendsem);
      poll_notify(priv->fds, CONFIG_RPMSG_UART_NPOLLWAITERS, POLLOUT);
    }

  spin_unlock_irqrestore_nopreempt(&priv->lock, flags);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int uart_rpmsg_init(FAR const char *cpuname, FAR const char *devname,
                    int buf_size, bool isconsole)
{
  FAR struct uart_rpmsg_priv_s *priv;
  char dev_name[32];
  FAR char *buf;
  int ret;

  if (buf_size <= 0)
    {
      return -EINVAL;
    }

  /* One allocation backs both the ring blob and priv.  The blob needs a
   * RING_BLOB_ALIGN base (cache-line isolation + init assert), so it goes
   * first and kmm_memalign covers it with no padding.  priv follows the
   * blob, whose size is the requested capacity plus the blob header, rounded
   * up just enough to keep priv aligned; those few bytes only add ring
   * capacity.
   */

  buf_size = ALIGN_UP(sizeof(struct ring_blob_s) + buf_size,
                      sizeof(uintptr_t));

  buf = kmm_memalign(RING_BLOB_ALIGN, buf_size + sizeof(*priv));
  if (buf == NULL)
    {
      return -ENOMEM;
    }

  priv = (FAR struct uart_rpmsg_priv_s *)(buf + buf_size);
  memset(priv, 0, sizeof(*priv));
  memset(buf, 0, sizeof(struct ring_blob_s));

  ring_spsc_init(&priv->ring, buf, buf_size, 0, false);

  spin_lock_init(&priv->lock);
  nxsem_init(&priv->recvsem, 0, 0);
  nxsem_init(&priv->sendsem, 0, 0);

  /* Only the console side cooks and echoes; the cu side stays raw to avoid
   * echoing back what the console already echoed across cores.  cu sets its
   * own OPOST|ONLCR via ioctl.
   */

  if (isconsole)
    {
      uart_termios_init(&priv->termios, ICRNL, OPOST | ONLCR, ECHO | ISIG);
    }
  else
    {
      uart_termios_init(&priv->termios, 0, 0, 0);
    }

#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGTSTP)
  priv->pid = INVALID_PROCESS_ID;
#endif

  strlcpy(priv->cpuname, cpuname, sizeof(priv->cpuname));
  strlcpy(priv->devname, devname, sizeof(priv->devname));

  ret = rpmsg_register_callback(priv,
                                uart_rpmsg_device_created,
                                uart_rpmsg_device_destroy,
                                NULL,
                                NULL);
  if (ret < 0)
    {
      goto fail_with_priv;
    }

  snprintf(dev_name, sizeof(dev_name), "%s%s",
           UART_RPMSG_DEV_PREFIX, devname);
  ret = register_driver(dev_name, &g_uart_rpmsg_fops, 0666, priv);
  if (ret < 0)
    {
      rpmsgerr("register %s failed, ret=%d\n", dev_name, ret);
      goto fail_with_rpmsg;
    }

  if (isconsole)
    {
      ret = register_driver(UART_RPMSG_DEV_CONSOLE, &g_uart_rpmsg_fops,
                            0666, priv);
      if (ret < 0)
        {
          rpmsgerr("register console failed, ret=%d\n", ret);
          unregister_driver(dev_name);
          goto fail_with_rpmsg;
        }
    }

  return OK;

fail_with_rpmsg:
  rpmsg_unregister_callback(priv,
                            uart_rpmsg_device_created,
                            uart_rpmsg_device_destroy,
                            NULL,
                            NULL);
fail_with_priv:
  nxsem_destroy(&priv->recvsem);
  nxsem_destroy(&priv->sendsem);
  kmm_free(buf);
  return ret;
}
