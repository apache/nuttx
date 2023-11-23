/****************************************************************************
 * drivers/syslog/ramlog.c
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

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <ctype.h>
#include <sys/boardctl.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/syslog/ramlog.h>
#include <nuttx/compiler.h>
#include <nuttx/list.h>
#include <nuttx/irq.h>

#ifdef CONFIG_RAMLOG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RAMLOG_MAGIC_NUMBER 0x12345678

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ramlog_header_s
{
  uint32_t          rl_magic;    /* The rl_magic number for ramlog buffer init */
  volatile uint32_t rl_head;     /* The head index (where data is added,natural growth) */
  char              rl_buffer[]; /* Circular RAM buffer */
};

struct ramlog_user_s
{
  struct list_node  rl_node;       /* The list_node of reader */
  volatile uint32_t rl_tail;       /* The tail index (where data is removed) */
  uint32_t          rl_threashold; /* The threashold of the reader to read log */
#ifndef CONFIG_RAMLOG_NONBLOCKING
  sem_t             rl_waitsem;    /* Used to wait for data */
#endif

  /* The following the poll structures of threads waiting for driver events.
   * The 'struct pollfd' reference for each open is also  retained in the
   * f_priv field of the 'struct file'.
   */

  FAR struct pollfd *rl_fds;
};

struct ramlog_dev_s
{
  /* The following is the header of the RAM buffer,
   * Store the RAM BUFFER init rl_magic number,
   * and read/write pointers
   */

  FAR struct ramlog_header_s *rl_header;

  mutex_t                    rl_lock;    /* Enforces mutually exclusive access */
  uint32_t                   rl_bufsize; /* Size of the Circular RAM buffer */
  struct list_node           rl_list;    /* The head of ramlog_user_s list */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

#ifndef CONFIG_RAMLOG_NONBLOCKING
static void    ramlog_readnotify(FAR struct ramlog_dev_s *priv);
#endif
static void    ramlog_pollnotify(FAR struct ramlog_dev_s *priv);
static void    ramlog_addchar(FAR struct ramlog_dev_s *priv, char ch);

/* Character driver methods */

static int     ramlog_file_open(FAR struct file *filep);
static int     ramlog_file_close(FAR struct file *filep);
static ssize_t ramlog_file_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen);
static ssize_t ramlog_file_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen);
static int     ramlog_file_ioctl(FAR struct file *filep, int cmd,
                                 unsigned long arg);
static int     ramlog_file_poll(FAR struct file *filep,
                                FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ramlogfops =
{
  ramlog_file_open,  /* open */
  ramlog_file_close, /* close */
  ramlog_file_read,  /* read */
  ramlog_file_write, /* write */
  NULL,              /* seek */
  ramlog_file_ioctl, /* ioctl */
  NULL,              /* mmap */
  NULL,              /* truncate */
  ramlog_file_poll   /* poll */
};

/* This is the pre-allocated buffer used for the console RAM log and/or
 * for the syslogging function.
 */

#ifdef CONFIG_RAMLOG_SYSLOG
#  ifdef RAMLOG_BUFFER_SECTION
static uint32_t g_sysbuffer[CONFIG_RAMLOG_BUFSIZE / 4]
                       locate_data(RAMLOG_BUFFER_SECTION);
#  else
static uint32_t g_sysbuffer[CONFIG_RAMLOG_BUFSIZE / 4];
#  endif

/* This is the device structure for the console or syslogging function.  It
 * must be statically initialized because the RAMLOG ramlog_putc function
 * could be called before the driver initialization logic executes.
 */

static struct ramlog_dev_s g_sysdev =
{
  (FAR struct ramlog_header_s *)g_sysbuffer,            /* rl_buffer */
  NXMUTEX_INITIALIZER,                                  /* rl_lock */
  sizeof(g_sysbuffer) - sizeof(struct ramlog_header_s), /* rl_bufsize */
  LIST_INITIAL_VALUE(g_sysdev.rl_list)                  /* rl_list */
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ramlog_bufferused
 ****************************************************************************/

static uint32_t ramlog_bufferused(FAR struct ramlog_dev_s *priv,
                                  FAR struct ramlog_user_s *upriv)
{
  uint32_t used = priv->rl_header->rl_head - upriv->rl_tail;
  return used > priv->rl_bufsize ? priv->rl_bufsize : used;
}

/****************************************************************************
 * Name: ramlog_readnotify
 ****************************************************************************/

#ifndef CONFIG_RAMLOG_NONBLOCKING
static void ramlog_readnotify(FAR struct ramlog_dev_s *priv)
{
  FAR struct ramlog_user_s *upriv;
  irqstate_t flags;

  /* Notify all waiting readers that they can read from the FIFO */

  flags = enter_critical_section();
  list_for_every_entry(&priv->rl_list, upriv, struct ramlog_user_s, rl_node)
    {
      for (; ; )
        {
          int semcount = 0;

          nxsem_get_value(&upriv->rl_waitsem, &semcount);
          if (semcount >= 0)
            {
              break;
            }

          nxsem_post(&upriv->rl_waitsem);
        }
    }

    leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: ramlog_pollnotify
 ****************************************************************************/

static void ramlog_pollnotify(FAR struct ramlog_dev_s *priv)
{
  FAR struct ramlog_user_s *upriv;
  irqstate_t flags;

  /* This function may be called from an interrupt handler */

  flags = enter_critical_section();
  list_for_every_entry(&priv->rl_list, upriv, struct ramlog_user_s, rl_node)
    {
      if (ramlog_bufferused(priv, upriv) >= upriv->rl_threashold)
        {
          /* Notify all poll/select waiters that they can read from
           * the FIFO
           */

          poll_notify(&upriv->rl_fds, 1, POLLIN);
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ramlog_addchar
 ****************************************************************************/

static void ramlog_addchar(FAR struct ramlog_dev_s *priv, char ch)
{
  FAR struct ramlog_header_s *header = priv->rl_header;
  irqstate_t flags;

  /* Disable interrupts (in case we are NOT called from interrupt handler) */

  flags = enter_critical_section();

#ifdef CONFIG_RAMLOG_SYSLOG
  if (priv == &g_sysdev && header->rl_magic != RAMLOG_MAGIC_NUMBER)
    {
      memset(header, 0, sizeof(g_sysbuffer));
      header->rl_magic = RAMLOG_MAGIC_NUMBER;
    }
#endif

#ifdef CONFIG_RAMLOG_CRLF
  /* Ignore carriage returns */

  if (ch == '\r')
    {
      leave_critical_section(flags);
      return;
    }

  /* Pre-pend a carriage before a linefeed */

  if (ch == '\n')
    {
      ch = '\r';
    }

again:
#endif

  header->rl_buffer[header->rl_head++ % priv->rl_bufsize] = ch;

#ifdef CONFIG_RAMLOG_CRLF
  if (ch == '\r')
    {
      ch = '\n';
      goto again;
    }
#endif

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ramlog_addbuf
 ****************************************************************************/

static ssize_t ramlog_addbuf(FAR struct ramlog_dev_s *priv,
                             FAR const char *buffer, size_t len)
{
  size_t nwritten;
  char ch;
  int ret;

  ret = nxmutex_lock(&priv->rl_lock);
  if (ret < 0)
    {
      return ret;
    }

  for (nwritten = 0; nwritten < len; nwritten++)
    {
      /* Get the next character to output */

      ch = buffer[nwritten];

      /* Then output the character */

      ramlog_addchar(priv, ch);
    }

  /* Was anything written? */

  if (nwritten > 0)
    {
#ifndef CONFIG_RAMLOG_NONBLOCKING
      /* Are there threads waiting for read data? */

      ramlog_readnotify(priv);
#endif
      /* Notify all poll/select waiters that they can read from the FIFO */

      ramlog_pollnotify(priv);
    }

  /* We always have to return the number of bytes requested and NOT the
   * number of bytes that were actually written.  Otherwise, callers
   * probably retry, causing same error condition again.
   */

  nxmutex_unlock(&priv->rl_lock);
  return len;
}

/****************************************************************************
 * Name: ramlog_read
 ****************************************************************************/

static ssize_t ramlog_file_read(FAR struct file *filep, FAR char *buffer,
                                size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv = inode->i_private;
  FAR struct ramlog_header_s *header = priv->rl_header;
  FAR struct ramlog_user_s *upriv = filep->f_priv;
  uint32_t tail;
  ssize_t nread;
  char ch;
  int ret;

  /* If the circular buffer is empty, then wait for something to be written
   * to it.  This function may NOT be called from an interrupt handler.
   */

  DEBUGASSERT(!up_interrupt_context());

  /* Get exclusive access to the rl_tail index */

  ret = nxmutex_lock(&priv->rl_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Determine whether the read pointer is overwritten */

  if (header->rl_head - upriv->rl_tail > priv->rl_bufsize)
    {
      upriv->rl_tail = header->rl_head - priv->rl_bufsize;
    }

  tail = upriv->rl_tail % priv->rl_bufsize;

  /* Loop until something is read */

  for (nread = 0; (size_t)nread < len; )
    {
      /* Get the next byte from the buffer */

      if (header->rl_head == upriv->rl_tail)
        {
          /* The circular buffer is empty. */

#ifdef CONFIG_RAMLOG_NONBLOCKING
          /* Return what we have (with zero mean the end-of-file) */

          break;
#else
          /* Did we read anything? */

          if (nread > 0)
            {
              /* Yes.. break out to return what we have. */

              break;
            }

          /* If the driver was opened with O_NONBLOCK option, then don't
           * wait.
           */

          if (filep->f_oflags & O_NONBLOCK)
            {
              nread = -EAGAIN;
              break;
            }

          /* Otherwise, wait for something to be written to the circular
           * buffer. Increment the number of waiters so that the
           * ramlog_file_write() will note that it needs to post the
           * semaphore to wake us up.
           */

          nxmutex_unlock(&priv->rl_lock);

          /* We may now be pre-empted!  But that should be okay because we
           * have already incremented nwaiters.  Pre-emptions is disabled
           * but will be re-enabled while we are waiting.
           */

          ret = nxsem_wait(&upriv->rl_waitsem);

          /* Did we successfully get the rl_waitsem? */

          if (ret >= 0)
            {
              /* Yes... then retake the mutual exclusion mutex */

              ret = nxmutex_lock(&priv->rl_lock);
            }

          /* Was the mutex wait successful? Did we successful re-take the
           * mutual exclusion mutex?
           */

          if (ret < 0)
            {
              /* No.. One of the two nxsem_wait's failed. */

              /* Return the error. We did handle the case where we read
               * anything already before waiting.
               */

              return ret;
            }
#endif /* CONFIG_RAMLOG_NONBLOCKING */
        }
      else
        {
          /* The circular buffer is not empty, get the next byte from the
           * tail index.
           */

          ch = header->rl_buffer[tail];

          /* Increment the tail index. */

          upriv->rl_tail++;
          if (++tail >= priv->rl_bufsize)
            {
              tail = 0;
            }

          /* Add the character to the user buffer. */

          buffer[nread] = ch;
          nread++;
        }
    }

  /* Relinquish the mutual exclusion mutex */

  nxmutex_unlock(&priv->rl_lock);

  /* Return the number of characters actually read */

  return nread;
}

/****************************************************************************
 * Name: ramlog_file_write
 ****************************************************************************/

static ssize_t ramlog_file_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv = inode->i_private;

  return ramlog_addbuf(priv, buffer, len);
}

/****************************************************************************
 * Name: ramlog_file_ioctl
 ****************************************************************************/

static int ramlog_file_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv = inode->i_private;
  FAR struct ramlog_user_s *upriv = filep->f_priv;
  int ret;

  ret = nxmutex_lock(&priv->rl_lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case FIONREAD:
        *(FAR int *)((uintptr_t)arg) = ramlog_bufferused(priv, upriv);
        break;
      case PIPEIOC_POLLINTHRD:
        upriv->rl_threashold = (uint32_t)arg;
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->rl_lock);
  return ret;
}

/****************************************************************************
 * Name: ramlog_file_poll
 ****************************************************************************/

static int ramlog_file_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv = inode->i_private;
  FAR struct ramlog_user_s *upriv = filep->f_priv;
  pollevent_t eventset = POLLOUT;
  irqstate_t flags;
  int ret;

  /* Get exclusive access to the poll structures */

  ret = nxmutex_lock(&priv->rl_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference.
       */

      if (!upriv->rl_fds)
        {
          upriv->rl_fds = fds;
          fds->priv     = &upriv->rl_fds;
        }

      /* Should immediately notify on any of the requested events? */

      flags = enter_critical_section();

      /* Check if the receive buffer is not empty. */

      if (ramlog_bufferused(priv, upriv) >= upriv->rl_threashold)
        {
          eventset |= POLLIN;
        }

      leave_critical_section(flags);

      poll_notify(&fds, 1, eventset);
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

  nxmutex_unlock(&priv->rl_lock);
  return ret;
}

/****************************************************************************
 * Name: ramlog_file_open
 ****************************************************************************/

static int ramlog_file_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv = inode->i_private;
  FAR struct ramlog_header_s *header = priv->rl_header;
  FAR struct ramlog_user_s *upriv;
  int ret;

  /* Get exclusive access to the rl_tail index */

  upriv = kmm_zalloc(sizeof(FAR struct ramlog_user_s));
  if (upriv == NULL)
    {
      return -ENOMEM;
    }

  upriv->rl_threashold = CONFIG_RAMLOG_POLLTHRESHOLD;
#ifndef CONFIG_RAMLOG_NONBLOCKING
  nxsem_init(&upriv->rl_waitsem, 0, 0);
#endif

  ret = nxmutex_lock(&priv->rl_lock);
  if (ret < 0)
    {
#ifndef CONFIG_RAMLOG_NONBLOCKING
      nxsem_destroy(&upriv->rl_waitsem);
#endif
      kmm_free(upriv);
      return ret;
    }

  list_add_tail(&priv->rl_list, &upriv->rl_node);

  upriv->rl_tail = header->rl_head > priv->rl_bufsize ?
                   header->rl_head - priv->rl_bufsize : 0;
  filep->f_priv = upriv;
  nxmutex_unlock(&priv->rl_lock);
  return ret;
}

/****************************************************************************
 * Name: ramlog_file_close
 ****************************************************************************/

static int ramlog_file_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv = inode->i_private;
  FAR struct ramlog_user_s *upriv = filep->f_priv;
  int ret;

  /* Get exclusive access to the rl_tail index */

  ret = nxmutex_lock(&priv->rl_lock);
  if (ret < 0)
    {
      return ret;
    }

#ifndef CONFIG_RAMLOG_NONBLOCKING
  nxsem_destroy(&upriv->rl_waitsem);
#endif

  list_delete(&upriv->rl_node);
  kmm_free(upriv);
  nxmutex_unlock(&priv->rl_lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ramlog_register
 *
 * Description:
 *   Create the RAM logging device and register it at the specified path.
 *
 ****************************************************************************/

int ramlog_register(FAR const char *devpath, FAR char *buffer, size_t buflen)
{
  FAR struct ramlog_dev_s *priv;
  int ret = -ENOMEM;

  /* Sanity checking */

  DEBUGASSERT(devpath && buffer && buflen > sizeof(struct ramlog_header_s));

  /* Allocate a RAM logging device structure */

  priv = kmm_zalloc(sizeof(struct ramlog_dev_s));
  if (priv != NULL)
    {
      /* Initialize the non-zero values in the RAM logging device structure */

      nxmutex_init(&priv->rl_lock);
      list_initialize(&priv->rl_list);
      priv->rl_bufsize = buflen - sizeof(struct ramlog_header_s);
      priv->rl_header = (FAR struct ramlog_header_s *)buffer;

      /* Register the character driver */

      ret = register_driver(devpath, &g_ramlogfops, 0666, priv);
      if (ret < 0)
        {
          nxmutex_destroy(&priv->rl_lock);
          kmm_free(priv);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ramlog_syslog_register
 *
 * Description:
 *   Use a pre-allocated RAM logging device and register it at the path
 *   specified by CONFIG_RAMLOG_SYSLOG
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_SYSLOG
void ramlog_syslog_register(void)
{
  /* Register the syslog character driver */

  register_driver(CONFIG_SYSLOG_DEVPATH, &g_ramlogfops, 0666, &g_sysdev);
}
#endif

/****************************************************************************
 * Name: ramlog_putc
 *
 * Description:
 *   This is the low-level system logging interface.
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_SYSLOG
int ramlog_putc(FAR struct syslog_channel_s *channel, int ch)
{
  FAR struct ramlog_dev_s *priv = &g_sysdev;

  UNUSED(channel);

  /* Add the character to the RAMLOG */

  ramlog_addchar(priv, ch);

#ifndef CONFIG_RAMLOG_NONBLOCKING
  /* Are there threads waiting for read data? */

  ramlog_readnotify(priv);
#endif

  /* Notify all poll/select waiters that they can read from the FIFO */

  ramlog_pollnotify(priv);

  /* Return the character added on success */

  return ch;
}

ssize_t ramlog_write(FAR struct syslog_channel_s *channel,
                     FAR const char *buffer, size_t buflen)
{
  FAR struct ramlog_dev_s *priv = &g_sysdev;

  return ramlog_addbuf(priv, buffer, buflen);
}
#endif

#endif /* CONFIG_RAMLOG */
