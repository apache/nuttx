/****************************************************************************
 * drivers/serial/uart_ram.c
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
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/serial/tioctl.h>
#include <nuttx/serial/uart_ram.h>
#include <nuttx/wdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_RAMBUF_SECTION(n) \
        locate_data(CONFIG_RAM_UART_BUFFER_SECTION "." #n)

#define UART_RAM_INITIALIZER(buf) \
  { \
    .rxlock = NXMUTEX_INITIALIZER, \
    .txlock = NXMUTEX_INITIALIZER, \
    .lock = SP_UNLOCKED, \
    .recvsem = SEM_INITIALIZER(0), \
    .sendsem = SEM_INITIALIZER(0), \
    .rambuf = (buf), \
  }

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The peer raises no interrupt, so the wdog stands in for it: it binds the
 * slave and wakes waiters.
 */

struct uart_ram_s
{
  struct wdog_s         wdog;
  mutex_t               rxlock;   /* Single consumer on rx */
  mutex_t               txlock;   /* Single producer on tx */
  spinlock_t            lock;     /* Shared with the wdog */
  sem_t                 recvsem;
  sem_t                 sendsem;
  uint16_t              crefs;
  FAR struct pollfd_s   *fds[CONFIG_RAM_UART_NPOLLWAITERS];
  struct ring_s         tx;
  struct ring_s         rx;
  FAR uart_rambuf_t     *rambuf;  /* [0] master writes, [1] slave */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     uart_ram_open(FAR struct file *filep);
static int     uart_ram_close(FAR struct file *filep);
static ssize_t uart_ram_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t uart_ram_write(FAR struct file *filep,
                              FAR const char *buffer, size_t buflen);
static int     uart_ram_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg);
static int     uart_ram_poll(FAR struct file *filep,
                             FAR struct pollfd_s *fds, bool setup);

static void    uart_ram_wdog(wdparm_t arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_uart_ram_fops =
{
  uart_ram_open,
  uart_ram_close,
  uart_ram_read,
  uart_ram_write,
  NULL,               /* seek */
  uart_ram_ioctl,
  NULL,               /* mmap */
  NULL,               /* truncate */
  uart_ram_poll,
};

#ifdef CONFIG_RAM_UART0
static uart_rambuf_t UART_RAMBUF_SECTION(0) g_uart_rambuf0[2];
static struct uart_ram_s g_uart_ram0 = UART_RAM_INITIALIZER(g_uart_rambuf0);
#endif

#ifdef CONFIG_RAM_UART1
static uart_rambuf_t UART_RAMBUF_SECTION(1) g_uart_rambuf1[2];
static struct uart_ram_s g_uart_ram1 = UART_RAM_INITIALIZER(g_uart_rambuf1);
#endif

#ifdef CONFIG_RAM_UART2
static uart_rambuf_t UART_RAMBUF_SECTION(2) g_uart_rambuf2[2];
static struct uart_ram_s g_uart_ram2 = UART_RAM_INITIALIZER(g_uart_rambuf2);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_ram_post
 ****************************************************************************/

static void uart_ram_post(FAR sem_t *sem)
{
  int semcount;

  /* The wdog posts on every tick */

  nxsem_get_value(sem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(sem);
    }
}

/****************************************************************************
 * Name: uart_ram_wait_ready
 ****************************************************************************/

static int uart_ram_wait_ready(FAR struct file *filep,
                               FAR struct ring_s *ring, FAR sem_t *sem)
{
  FAR struct uart_ram_s *priv = filep->f_inode->i_private;
  irqstate_t flags;
  bool ready;
  int ret;

  /* A ring never goes back to unbound, so this is checked once */

  for (; ; )
    {
      flags = spin_lock_irqsave(&priv->lock);
      ready = ring_is_ready(ring);
      spin_unlock_irqrestore(&priv->lock, flags);

      if (ready)
        {
          return OK;
        }
      else if (filep->f_oflags & O_NONBLOCK)
        {
          return -EAGAIN;
        }

      ret = nxsem_wait(sem);
      if (ret < 0)
        {
          return ret;
        }
    }
}

/****************************************************************************
 * Name: uart_ram_open
 ****************************************************************************/

static int uart_ram_open(FAR struct file *filep)
{
  FAR struct uart_ram_s *priv = filep->f_inode->i_private;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  DEBUGASSERT(priv->crefs < UINT16_MAX);
  if (priv->crefs++ == 0)
    {
      wd_start(&priv->wdog, USEC2TICK(CONFIG_RAM_UART_POLLING_INTERVAL),
               uart_ram_wdog, (wdparm_t)priv);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: uart_ram_close
 ****************************************************************************/

static int uart_ram_close(FAR struct file *filep)
{
  FAR struct uart_ram_s *priv = filep->f_inode->i_private;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  DEBUGASSERT(priv->crefs > 0);
  if (--priv->crefs == 0)
    {
      wd_cancel(&priv->wdog);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: uart_ram_read
 ****************************************************************************/

static ssize_t uart_ram_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct uart_ram_s *priv = filep->f_inode->i_private;
  ssize_t ret;

  if (buflen == 0)
    {
      return 0;
    }

  ret = nxmutex_lock(&priv->rxlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = uart_ram_wait_ready(filep, &priv->rx, &priv->recvsem);

  while (ret >= 0)
    {
      ret = ring_spsc_dequeue(&priv->rx, buffer, buflen);
      if (ret > 0)
        {
          break;
        }

      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          break;
        }

      ret = nxsem_wait(&priv->recvsem);
    }

  nxmutex_unlock(&priv->rxlock);
  return ret;
}

/****************************************************************************
 * Name: uart_ram_write
 ****************************************************************************/

static ssize_t uart_ram_write(FAR struct file *filep,
                              FAR const char *buffer, size_t buflen)
{
  FAR struct uart_ram_s *priv = filep->f_inode->i_private;
  size_t nwritten = 0;
  ssize_t ret;

  if (buflen == 0)
    {
      return 0;
    }

  ret = nxmutex_lock(&priv->txlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = uart_ram_wait_ready(filep, &priv->tx, &priv->sendsem);

  while (ret >= 0 && nwritten < buflen)
    {
      ret = ring_spsc_enqueue(&priv->tx, &buffer[nwritten],
                              buflen - nwritten);
      if (ret > 0)
        {
          nwritten += ret;
          ret = nwritten;
          continue;
        }

      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = nwritten > 0 ? (ssize_t)nwritten : -EAGAIN;
          break;
        }

      ret = nxsem_wait(&priv->sendsem);
    }

  nxmutex_unlock(&priv->txlock);
  return nwritten > 0 ? (ssize_t)nwritten : ret;
}

/****************************************************************************
 * Name: uart_ram_ioctl
 ****************************************************************************/

static int uart_ram_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg)
{
  FAR struct uart_ram_s *priv = filep->f_inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      case FIONREAD:
        {
          FAR int *nbytes = (FAR int *)(uintptr_t)arg;

          if (nbytes == NULL)
            {
              ret = -EINVAL;
              break;
            }

          *nbytes = ring_is_ready(&priv->rx) ? ring_count(&priv->rx) : 0;
        }
        break;

      /* Nothing to release; rpmsg_port_uart sends it on peer poweroff */

      case TIOCVHANGUP:
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: uart_ram_poll
 ****************************************************************************/

static int uart_ram_poll(FAR struct file *filep,
                         FAR struct pollfd_s *fds,
                         bool setup)
{
  FAR struct uart_ram_s *priv = filep->f_inode->i_private;
  pollevent_t eventset = 0;
  irqstate_t flags;
  int ret = OK;
  int i;

  flags = spin_lock_irqsave_nopreempt(&priv->lock);

  if (setup)
    {
      for (i = 0; i < CONFIG_RAM_UART_NPOLLWAITERS; i++)
        {
          if (priv->fds[i] == NULL)
            {
              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_RAM_UART_NPOLLWAITERS)
        {
          ret = -EBUSY;
          goto errout;
        }

      if (ring_is_ready(&priv->rx) && ring_count(&priv->rx) > 0)
        {
          eventset |= POLLIN;
        }

      if (ring_is_ready(&priv->tx) && ring_space(&priv->tx) > 0)
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
 * Name: uart_ram_wdog
 ****************************************************************************/

static void uart_ram_wdog(wdparm_t arg)
{
  FAR struct uart_ram_s *priv = (FAR struct uart_ram_s *)arg;
  FAR struct ring_s *tx = &priv->tx;
  FAR struct ring_s *rx = &priv->rx;
  pollevent_t eventset = 0;
  irqstate_t flags;

  flags = spin_lock_irqsave_nopreempt(&priv->lock);

  /* No-op once bound; the slave binds once the master is up */

  ring_spsc_init(tx, priv->rambuf[1], sizeof(priv->rambuf[1]), 0, true);
  ring_spsc_init(rx, priv->rambuf[0], sizeof(priv->rambuf[0]), 0, true);

  if (ring_is_ready(tx) && ring_is_ready(rx))
    {
      if (ring_count(rx) > 0)
        {
          uart_ram_post(&priv->recvsem);
          eventset |= POLLIN;
        }

      if (ring_space(tx) > 0)
        {
          uart_ram_post(&priv->sendsem);
          eventset |= POLLOUT;
        }

      /* fds is written by poll() under the same lock, and the callback
       * may wake a task, so the lock must span poll_notify()
       */

      if (eventset != 0)
        {
          poll_notify(priv->fds, CONFIG_RAM_UART_NPOLLWAITERS, eventset);
        }
    }

  /* Under the lock, so that a racing last close() stops it */

  if (priv->crefs > 0)
    {
      wd_start(&priv->wdog, USEC2TICK(CONFIG_RAM_UART_POLLING_INTERVAL),
               uart_ram_wdog, arg);
    }

  spin_unlock_irqrestore_nopreempt(&priv->lock, flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_ram_register
 ****************************************************************************/

int uart_ram_register(FAR const char *devname, uart_rambuf_t rambuf[2],
                      bool slave)
{
  FAR struct uart_ram_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct uart_ram_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  nxmutex_init(&priv->rxlock);
  nxmutex_init(&priv->txlock);
  spin_lock_init(&priv->lock);
  nxsem_init(&priv->recvsem, 0, 0);
  nxsem_init(&priv->sendsem, 0, 0);
  priv->rambuf = rambuf;

  /* Only the master resets the rings; the slave binds from the wdog */

  if (!slave)
    {
      ring_spsc_init(&priv->tx, rambuf[0], sizeof(rambuf[0]), 0, false);
      ring_spsc_init(&priv->rx, rambuf[1], sizeof(rambuf[1]), 0, false);
    }

  ret = register_driver(devname, &g_uart_ram_fops, 0666, priv);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->rxlock);
      nxmutex_destroy(&priv->txlock);
      nxsem_destroy(&priv->recvsem);
      nxsem_destroy(&priv->sendsem);
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: ram_serialinit
 ****************************************************************************/

void ram_serialinit(void)
{
#ifdef CONFIG_RAM_UART0
#  ifndef CONFIG_RAM_UART0_SLAVE
  ring_spsc_init(&g_uart_ram0.tx, g_uart_rambuf0[0],
                 sizeof(g_uart_rambuf0[0]), 0, false);
  ring_spsc_init(&g_uart_ram0.rx, g_uart_rambuf0[1],
                 sizeof(g_uart_rambuf0[1]), 0, false);
#  endif
  register_driver("/dev/tty0", &g_uart_ram_fops, 0666, &g_uart_ram0);
#endif

#ifdef CONFIG_RAM_UART1
#  ifndef CONFIG_RAM_UART1_SLAVE
  ring_spsc_init(&g_uart_ram1.tx, g_uart_rambuf1[0],
                 sizeof(g_uart_rambuf1[0]), 0, false);
  ring_spsc_init(&g_uart_ram1.rx, g_uart_rambuf1[1],
                 sizeof(g_uart_rambuf1[1]), 0, false);
#  endif
  register_driver("/dev/tty1", &g_uart_ram_fops, 0666, &g_uart_ram1);
#endif

#ifdef CONFIG_RAM_UART2
#  ifndef CONFIG_RAM_UART2_SLAVE
  ring_spsc_init(&g_uart_ram2.tx, g_uart_rambuf2[0],
                 sizeof(g_uart_rambuf2[0]), 0, false);
  ring_spsc_init(&g_uart_ram2.rx, g_uart_rambuf2[1],
                 sizeof(g_uart_rambuf2[1]), 0, false);
#  endif
  register_driver("/dev/tty2", &g_uart_ram_fops, 0666, &g_uart_ram2);
#endif
}
