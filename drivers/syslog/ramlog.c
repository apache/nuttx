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
#include <nuttx/spinlock.h>
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

  /* Notify all waiting readers that they can read from the FIFO */

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
}
#endif

/****************************************************************************
 * Name: ramlog_pollnotify
 ****************************************************************************/

static void ramlog_pollnotify(FAR struct ramlog_dev_s *priv)
{
  FAR struct ramlog_user_s *upriv;

  /* This function may be called from an interrupt handler */

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
}

/****************************************************************************
 * Name: ramlog_flush
 ****************************************************************************/

static void ramlog_bufferflush(FAR struct ramlog_dev_s *priv)
{
  FAR struct ramlog_user_s *upriv;

  priv->rl_header->rl_head = 0;
  list_for_every_entry(&priv->rl_list, upriv, struct ramlog_user_s, rl_node)
    {
      upriv->rl_tail = 0;
    }
}

/****************************************************************************
 * Name: ramlog_copybuf
 ****************************************************************************/

static void ramlog_copybuf(FAR struct ramlog_dev_s *priv,
                          FAR const char *buffer, size_t len)
{
  FAR struct ramlog_header_s *header = priv->rl_header;
  FAR char *buf = header->rl_buffer;
  uint32_t offset;
  uint32_t tail;

  if (len <= 0)
    {
      return;
    }

  offset = header->rl_head % priv->rl_bufsize;
  tail = priv->rl_bufsize - offset;

  if (len > tail)
    {
      memcpy(&buf[offset], buffer, tail);
      memcpy(buf, buffer + tail, len - tail);
    }
  else
    {
      memcpy(&buf[offset], buffer, len);
    }

  header->rl_head += len;
}

/****************************************************************************
 * Name: ramlog_addbuf
 ****************************************************************************/

static ssize_t ramlog_addbuf(FAR struct ramlog_dev_s *priv,
                             FAR const char *buffer, size_t len)
{
#if defined(CONFIG_RAMLOG_SYSLOG) || defined(CONFIG_RAMLOG_CRLF)
  FAR struct ramlog_header_s *header = priv->rl_header;
#endif
  size_t buflen = len;
  irqstate_t flags;
#ifdef CONFIG_RAMLOG_CRLF
  FAR const char *end;
  FAR const char *pos;
  FAR char *buf;
#endif

  /* Disable interrupts (in case we are NOT called from interrupt handler) */

  flags = enter_critical_section();

#ifdef CONFIG_RAMLOG_SYSLOG
  if (header->rl_magic != RAMLOG_MAGIC_NUMBER && priv == &g_sysdev)
    {
      memset(header, 0, sizeof(g_sysbuffer));
      header->rl_magic = RAMLOG_MAGIC_NUMBER;
    }
#endif

  if (buflen > priv->rl_bufsize)
    {
      buffer += buflen - priv->rl_bufsize;
      buflen = priv->rl_bufsize;
    }

#ifdef CONFIG_RAMLOG_CRLF
  buf = header->rl_buffer;
  end = buffer + buflen;
  pos = buffer;

  do
    {
      /* Ignore carriage returns */

      if (*pos == '\r' || *pos == '\n')
        {
          ramlog_copybuf(priv, buffer, pos - buffer);
          buffer = pos + 1;
        }

      /* Pre-pend a carriage before a linefeed */

      if (*pos == '\n')
        {
          buf[header->rl_head++ % priv->rl_bufsize] = '\r';
          buf[header->rl_head++ % priv->rl_bufsize] = '\n';
        }
    }
  while (++pos != end);

  ramlog_copybuf(priv, buffer, pos - buffer);
#else
  ramlog_copybuf(priv, buffer, buflen);
#endif

  /* Was anything written? */

  if (len > 0)
    {
      /* Lock the scheduler do NOT switch out */

      if (!up_interrupt_context())
        {
          sched_lock();
        }

#ifndef CONFIG_RAMLOG_NONBLOCKING
      /* Are there threads waiting for read data? */

      ramlog_readnotify(priv);
#endif
      /* Notify all poll/select waiters that they can read from the FIFO */

      ramlog_pollnotify(priv);

      /* Unlock the scheduler */

      if (!up_interrupt_context())
        {
          sched_unlock();
        }
    }

  /* We always have to return the number of bytes requested and NOT the
   * number of bytes that were actually written.  Otherwise, callers
   * probably retry, causing same error condition again.
   */

  leave_critical_section(flags);
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
  irqstate_t flags;
  uint32_t ncopy;
  ssize_t nread;
  uint32_t tail;
  uint32_t pos;

  /* If the circular buffer is empty, then wait for something to be written
   * to it.  This function may NOT be called from an interrupt handler.
   */

  DEBUGASSERT(!up_interrupt_context());

  /* Get exclusive access to the rl_tail index */

  flags = enter_critical_section();

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
          int ret;

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

          /* We may now be pre-empted!  But that should be okay because we
           * have already incremented nwaiters.  Pre-emptions is disabled
           * but will be re-enabled while we are waiting.
           */

          ret = nxsem_wait(&upriv->rl_waitsem);

          /* Did we successfully get the rl_waitsem? */

          if (ret < 0)
            {
              /* No.. nxsem_wait's failed. */

              /* Return the error. We did handle the case where we read
               * anything already before waiting.
               */

              return ret;
            }
#endif /* CONFIG_RAMLOG_NONBLOCKING */
        }
      else
        {
          /* Determine whether the read pointer is overwritten */

          if (header->rl_head - upriv->rl_tail > priv->rl_bufsize)
            {
              upriv->rl_tail = header->rl_head - priv->rl_bufsize;
            }

          /* The circular buffer is not empty, get the next byte from the
           * tail index.
           */

          pos = upriv->rl_tail % priv->rl_bufsize;
          ncopy = header->rl_head - upriv->rl_tail;
          tail = priv->rl_bufsize - pos;

          if (ncopy > len - nread)
            {
              ncopy = len - nread;
            }

          if (ncopy > tail)
            {
              memcpy(&buffer[nread], &header->rl_buffer[pos], tail);
              memcpy(&buffer[nread + tail], header->rl_buffer, ncopy - tail);
            }
          else
            {
              memcpy(&buffer[nread], &header->rl_buffer[pos], ncopy);
            }

          upriv->rl_tail += ncopy;
          nread += ncopy;
        }
    }

  leave_critical_section(flags);

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
  irqstate_t flags;
  int ret = 0;

  flags = enter_critical_section();

  switch (cmd)
    {
      case FIONREAD:
        *(FAR int *)((uintptr_t)arg) = ramlog_bufferused(priv, upriv);
        break;
      case PIPEIOC_POLLINTHRD:
        upriv->rl_threashold = (uint32_t)arg;
        break;
      case BIOC_FLUSH:
        ramlog_bufferflush(priv);
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  leave_critical_section(flags);
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

  /* Get exclusive access to the poll structures */

  flags = enter_critical_section();

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

      /* Check if the receive buffer is not empty. */

      if (ramlog_bufferused(priv, upriv) >= upriv->rl_threashold)
        {
          eventset |= POLLIN;
        }

      poll_notify(&fds, 1, eventset);
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

  leave_critical_section(flags);
  return 0;
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
  irqstate_t flags;

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

  flags = enter_critical_section();
  list_add_tail(&priv->rl_list, &upriv->rl_node);
  upriv->rl_tail = header->rl_head > priv->rl_bufsize ?
                   header->rl_head - priv->rl_bufsize : 0;
  leave_critical_section(flags);

  filep->f_priv = upriv;
  return 0;
}

/****************************************************************************
 * Name: ramlog_file_close
 ****************************************************************************/

static int ramlog_file_close(FAR struct file *filep)
{
  FAR struct ramlog_user_s *upriv = filep->f_priv;
  irqstate_t flags;

  /* Get exclusive access to the rl_tail index */

  flags = enter_critical_section();
  list_delete(&upriv->rl_node);
  leave_critical_section(flags);

#ifndef CONFIG_RAMLOG_NONBLOCKING
  nxsem_destroy(&upriv->rl_waitsem);
#endif
  kmm_free(upriv);
  return 0;
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

      list_initialize(&priv->rl_list);
      priv->rl_bufsize = buflen - sizeof(struct ramlog_header_s);
      priv->rl_header = (FAR struct ramlog_header_s *)buffer;

      /* Register the character driver */

      ret = register_driver(devpath, &g_ramlogfops, 0666, priv);
      if (ret < 0)
        {
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
int ramlog_putc(FAR syslog_channel_t *channel, int ch)
{
  char cch = ch;

  UNUSED(channel);

  /* Add the character to the RAMLOG */

  ramlog_addbuf(&g_sysdev, &cch, 1);

  /* Return the character added on success */

  return ch;
}

ssize_t ramlog_write(FAR syslog_channel_t *channel,
                     FAR const char *buffer, size_t buflen)
{
  return ramlog_addbuf(&g_sysdev, buffer, buflen);
}
#endif

#endif /* CONFIG_RAMLOG */
