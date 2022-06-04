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
#include <wchar.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/syslog/ramlog.h>
#include <nuttx/compiler.h>

#include <nuttx/irq.h>

#ifdef CONFIG_RAMLOG

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ramlog_dev_s
{
#ifndef CONFIG_RAMLOG_NONBLOCKING
  volatile uint8_t  rl_nwaiters; /* Number of threads waiting for data */
#endif
  volatile size_t   rl_head;     /* The head index (where data is added) */
  volatile size_t   rl_tail;     /* The tail index (where data is removed) */
  sem_t             rl_exclsem;  /* Enforces mutually exclusive access */
#ifndef CONFIG_RAMLOG_NONBLOCKING
  sem_t             rl_waitsem;  /* Used to wait for data */
#endif
  size_t            rl_bufsize;  /* Size of the RAM buffer */
  FAR char         *rl_buffer;   /* Circular RAM buffer */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  FAR struct pollfd *rl_fds[CONFIG_RAMLOG_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

#ifndef CONFIG_RAMLOG_NONBLOCKING
static int     ramlog_readnotify(FAR struct ramlog_dev_s *priv);
#endif
static void    ramlog_pollnotify(FAR struct ramlog_dev_s *priv,
                                 pollevent_t eventset);
static int     ramlog_addchar(FAR struct ramlog_dev_s *priv, char ch);

/* Character driver methods */

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
  NULL,              /* open */
  NULL,              /* close */
  ramlog_file_read,  /* read */
  ramlog_file_write, /* write */
  NULL,              /* seek */
  ramlog_file_ioctl, /* ioctl */
  ramlog_file_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL             /* unlink */
#endif
};

/* This is the pre-allocated buffer used for the console RAM log and/or
 * for the syslogging function.
 */

#ifdef CONFIG_RAMLOG_SYSLOG
static char g_sysbuffer[CONFIG_RAMLOG_BUFSIZE]
#ifdef CONFIG_RAMLOG_BUFFER_SECTION
                               locate_data(CONFIG_RAMLOG_BUFFER_SECTION)
#endif
;

/* This is the device structure for the console or syslogging function.  It
 * must be statically initialized because the RAMLOG ramlog_putc function
 * could be called before the driver initialization logic executes.
 */

static struct ramlog_dev_s g_sysdev =
{
#ifndef CONFIG_RAMLOG_NONBLOCKING
  0,                             /* rl_nwaiters */
#endif
  CONFIG_RAMLOG_BUFSIZE,         /* rl_head */
  CONFIG_RAMLOG_BUFSIZE,         /* rl_tail */
  SEM_INITIALIZER(1),            /* rl_exclsem */
#ifndef CONFIG_RAMLOG_NONBLOCKING
  SEM_INITIALIZER(0),            /* rl_waitsem */
#endif
  CONFIG_RAMLOG_BUFSIZE,         /* rl_bufsize */
  g_sysbuffer                    /* rl_buffer */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ramlog_bufferused
 ****************************************************************************/

static size_t ramlog_bufferused(FAR struct ramlog_dev_s *priv)
{
  return (priv->rl_bufsize + priv->rl_head - priv->rl_tail) %
         priv->rl_bufsize;
}

/****************************************************************************
 * Name: ramlog_readnotify
 ****************************************************************************/

#ifndef CONFIG_RAMLOG_NONBLOCKING
static int ramlog_readnotify(FAR struct ramlog_dev_s *priv)
{
  irqstate_t flags;
  int i;

  /* Notify all waiting readers that they can read from the FIFO */

  flags = enter_critical_section();
  for (i = 0; i < priv->rl_nwaiters; i++)
    {
      nxsem_post(&priv->rl_waitsem);
    }

  leave_critical_section(flags);

  /* Return number of notified readers. */

  return i;
}
#endif

/****************************************************************************
 * Name: ramlog_pollnotify
 ****************************************************************************/

static void ramlog_pollnotify(FAR struct ramlog_dev_s *priv,
                              pollevent_t eventset)
{
  FAR struct pollfd *fds;
  irqstate_t flags;
  int i;

  /* This function may be called from an interrupt handler */

  for (i = 0; i < CONFIG_RAMLOG_NPOLLWAITERS; i++)
    {
      flags = enter_critical_section();
      fds = priv->rl_fds[i];
      if (fds)
        {
          fds->revents |= (fds->events & eventset);
          if (fds->revents != 0)
            {
              nxsem_post(fds->sem);
            }
        }

      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: ramlog_initbuf
 *
 * Description:
 *  Initialize g_sysdev based on the current system ramlog buffer.
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_SYSLOG
static void ramlog_initbuf(void)
{
  FAR struct ramlog_dev_s *priv = &g_sysdev;
  FAR const char *tmp;
  char prev;
  size_t i;

  if (priv->rl_head != CONFIG_RAMLOG_BUFSIZE ||
      priv->rl_tail != CONFIG_RAMLOG_BUFSIZE)
    {
      return;
    }

  prev = priv->rl_buffer[priv->rl_bufsize - 1];
  for (i = 0; i < priv->rl_bufsize; i++)
    {
      if (!prev && priv->rl_buffer[i])
        {
          tmp = priv->rl_buffer + i;
          if ((ssize_t)mbsnrtowcs(NULL, &tmp, priv->rl_bufsize - i, 0, NULL) < 0)
            {
              break;
            }
          else if (tmp == NULL)
            {
              priv->rl_head = i + strlen(priv->rl_buffer + i);
              priv->rl_tail = i;
              return;
            }
          else if ((ssize_t)mbstowcs(NULL, priv->rl_buffer, 0) < 0)
            {
              break;
            }
          else
            {
              priv->rl_head = strlen(priv->rl_buffer);
              priv->rl_tail = i;
              return;
            }
        }

      prev = priv->rl_buffer[i];
    }

  memset(priv->rl_buffer, 0, priv->rl_bufsize);
}
#endif

/****************************************************************************
 * Name: ramlog_addchar
 ****************************************************************************/

static int ramlog_addchar(FAR struct ramlog_dev_s *priv, char ch)
{
  irqstate_t flags;
  size_t nexthead;

#ifdef CONFIG_RAMLOG_SYSLOG
  if (priv == &g_sysdev)
    {
      ramlog_initbuf();
    }
#endif

  /* Disable interrupts (in case we are NOT called from interrupt handler) */

  flags = enter_critical_section();

#ifdef CONFIG_RAMLOG_CRLF
  /* Ignore carriage returns */

  if (ch == '\r')
    {
      leave_critical_section(flags);
      return OK;
    }

  /* Pre-pend a carriage before a linefeed */

  if (ch == '\n')
    {
      ch = '\r';
    }

again:
#endif

  /* Calculate the write index AFTER the next byte is written */

  nexthead = priv->rl_head + 1;
  if (nexthead >= priv->rl_bufsize)
    {
      nexthead = 0;
    }

  /* Would the next write overflow the circular buffer? */

  if (nexthead == priv->rl_tail)
    {
#ifdef CONFIG_RAMLOG_OVERWRITE
      /* Yes... Overwrite with the latest log in the circular buffer */

      priv->rl_buffer[priv->rl_tail] = '\0';
      priv->rl_tail += 1;
      if (priv->rl_tail >= priv->rl_bufsize)
        {
          priv->rl_tail = 0;
        }
#else
      /* Yes... Return an indication that nothing was saved in the buffer. */

      leave_critical_section(flags);
      return -EBUSY;
#endif
    }

  /* No... copy the byte and re-enable interrupts */

  priv->rl_buffer[priv->rl_head] = ch;
  priv->rl_head = nexthead;

#ifdef CONFIG_RAMLOG_CRLF
  if (ch == '\r')
    {
      ch = '\n';
      goto again;
    }
#endif

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ramlog_addbuf
 ****************************************************************************/

static ssize_t ramlog_addbuf(FAR struct ramlog_dev_s *priv,
                             FAR const char *buffer, size_t len)
{
  int readers_waken;
  ssize_t nwritten;
  char ch;
  int ret;

  ret = nxsem_wait(&priv->rl_exclsem);
  if (ret < 0)
    {
      return ret;
    }

  for (nwritten = 0; (size_t)nwritten < len; nwritten++)
    {
      /* Get the next character to output */

      ch = buffer[nwritten];

      /* Then output the character */

      ret = ramlog_addchar(priv, ch);
      if (ret < 0)
        {
          /* The buffer is full and nothing was saved.  The remaining
           * data to be written is dropped on the floor.
           */

          break;
        }
    }

  /* Was anything written? */

  if (nwritten > 0)
    {
      readers_waken = 0;

#ifndef CONFIG_RAMLOG_NONBLOCKING
      /* Are there threads waiting for read data? */

      readers_waken = ramlog_readnotify(priv);
#endif

      /* If there are multiple readers, some of them might block despite
       * POLLIN because first reader might read all data. Favor readers
       * and notify poll waiters only if no reader was awakened, even if
       * the latter may starve.
       *
       * This also implies we do not have to make these two notify
       * operations a critical section.
       */

      if (readers_waken == 0 &&
          ramlog_bufferused(priv) >= CONFIG_RAMLOG_POLLTHRESHOLD)
        {
          /* Notify all poll/select waiters that they can read from the
           * FIFO.
           */

          ramlog_pollnotify(priv, POLLIN);
        }
    }

  /* We always have to return the number of bytes requested and NOT the
   * number of bytes that were actually written.  Otherwise, callers
   * probably retry, causing same error condition again.
   */

  nxsem_post(&priv->rl_exclsem);
  return len;
}

/****************************************************************************
 * Name: ramlog_read
 ****************************************************************************/

static ssize_t ramlog_file_read(FAR struct file *filep, FAR char *buffer,
                                size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv;
  ssize_t nread;
  char ch;
  int ret;

  /* Some sanity checking */

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct ramlog_dev_s *)inode->i_private;

  /* If the circular buffer is empty, then wait for something to be written
   * to it.  This function may NOT be called from an interrupt handler.
   */

  DEBUGASSERT(!up_interrupt_context());

  /* Get exclusive access to the rl_tail index */

  ret = nxsem_wait(&priv->rl_exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Loop until something is read */

  for (nread = 0; (size_t)nread < len; )
    {
      /* Get the next byte from the buffer */

      if (priv->rl_head == priv->rl_tail)
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

          sched_lock();
          priv->rl_nwaiters++;
          nxsem_post(&priv->rl_exclsem);

          /* We may now be pre-empted!  But that should be okay because we
           * have already incremented nwaiters.  Pre-emptions is disabled
           * but will be re-enabled while we are waiting.
           */

          ret = nxsem_wait(&priv->rl_waitsem);

          /* Interrupts will be disabled when we return.  So the decrementing
           * rl_nwaiters here is safe.
           */

          priv->rl_nwaiters--;
          sched_unlock();

          /* Did we successfully get the rl_waitsem? */

          if (ret >= 0)
            {
              /* Yes... then retake the mutual exclusion semaphore */

              ret = nxsem_wait(&priv->rl_exclsem);
            }

          /* Was the semaphore wait successful? Did we successful re-take the
           * mutual exclusion semaphore?
           */

          if (ret < 0)
            {
              /* No.. One of the two nxsem_wait's failed. */

              /* Return the error. We did handle the case where we read
               * anything already before waiting.
               */

              nread = ret;

              /* Break out to return what we have.  Note, we can't exactly
               * "break" out because whichever error occurred, we do not hold
               * the exclusion semaphore.
               */

              goto errout_without_sem;
            }
#endif /* CONFIG_RAMLOG_NONBLOCKING */
        }
      else
        {
          /* The circular buffer is not empty, get the next byte from the
           * tail index.
           */

          ch = priv->rl_buffer[priv->rl_tail];
          priv->rl_buffer[priv->rl_tail] = '\0';

          /* Increment the tail index. */

          if (++priv->rl_tail >= priv->rl_bufsize)
            {
              priv->rl_tail = 0;
            }

          /* Add the character to the user buffer. */

          buffer[nread] = ch;
          nread++;
        }
    }

  /* Relinquish the mutual exclusion semaphore */

  nxsem_post(&priv->rl_exclsem);

  /* Notify all poll/select waiters that they can write to the FIFO */

#ifndef CONFIG_RAMLOG_NONBLOCKING
errout_without_sem:
#endif

  if (nread > 0)
    {
      ramlog_pollnotify(priv, POLLOUT);
    }

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
  FAR struct ramlog_dev_s *priv;

  /* Some sanity checking */

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct ramlog_dev_s *)inode->i_private;

  return ramlog_addbuf(priv, buffer, len);
}

/****************************************************************************
 * Name: ramlog_file_ioctl
 ****************************************************************************/

static int ramlog_file_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct ramlog_dev_s *)inode->i_private;

  ret = nxsem_wait(&priv->rl_exclsem);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case FIONREAD:
        *(FAR int *)((uintptr_t)arg) = ramlog_bufferused(priv);
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&priv->rl_exclsem);

  return ret;
}

/****************************************************************************
 * Name: ramlog_file_poll
 ****************************************************************************/

static int ramlog_file_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv;
  pollevent_t eventset;
  irqstate_t flags;
  size_t next_head;
  int ret;
  int i;

  /* Some sanity checking */

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct ramlog_dev_s *)inode->i_private;

  /* Get exclusive access to the poll structures */

  ret = nxsem_wait(&priv->rl_exclsem);
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

      for (i = 0; i < CONFIG_RAMLOG_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->rl_fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->rl_fds[i] = fds;
              fds->priv       = &priv->rl_fds[i];
              break;
            }
        }

      if (i >= CONFIG_RAMLOG_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should immediately notify on any of the requested events? */

      eventset = 0;

      flags = enter_critical_section();
      next_head = priv->rl_head + 1;
      if (next_head >= priv->rl_bufsize)
        {
          next_head = 0;
        }

      /* First, check if the receive buffer is not full. */

      if (next_head != priv->rl_tail)
        {
          eventset |= POLLOUT;
        }

      /* Check if the receive buffer is not empty. */

      if (ramlog_bufferused(priv) >= CONFIG_RAMLOG_POLLTHRESHOLD)
        {
          eventset |= POLLIN;
        }

      leave_critical_section(flags);

      if (eventset)
        {
          ramlog_pollnotify(priv, eventset);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;

#ifdef CONFIG_DEBUG_FEATURES
      if (!slot)
        {
          ret = -EIO;
          goto errout;
        }
#endif

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

errout:
  nxsem_post(&priv->rl_exclsem);
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

  DEBUGASSERT(devpath && buffer && buflen > 1);

  /* Allocate a RAM logging device structure */

  priv = (struct ramlog_dev_s *)kmm_zalloc(sizeof(struct ramlog_dev_s));
  if (priv != NULL)
    {
      /* Initialize the non-zero values in the RAM logging device structure */

      nxsem_init(&priv->rl_exclsem, 0, 1);
#ifndef CONFIG_RAMLOG_NONBLOCKING
      nxsem_init(&priv->rl_waitsem, 0, 0);

      /* The rl_waitsem semaphore is used for signaling and, hence, should
       * not have priority inheritance enabled.
       */

      nxsem_set_protocol(&priv->rl_waitsem, SEM_PRIO_NONE);
#endif

      priv->rl_bufsize = buflen;
      priv->rl_buffer  = buffer;

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
int ramlog_putc(FAR struct syslog_channel_s *channel, int ch)
{
  FAR struct ramlog_dev_s *priv = &g_sysdev;
  int readers_waken = 0;
  int ret;

  UNUSED(channel);

  /* Add the character to the RAMLOG */

  ret = ramlog_addchar(priv, ch);
  if (ret < 0)
    {
      /* The buffer is full and 'ch' was not saved. */

      return ret;
    }

#ifndef CONFIG_RAMLOG_NONBLOCKING
  /* Are there threads waiting for read data? */

  readers_waken = ramlog_readnotify(priv);
#endif

  /* If there are multiple readers, some of them might block despite
   * POLLIN because first reader might read all data. Favor readers
   * and notify poll waiters only if no reader was awakened, even if
   * the latter may starve.
   *
   * This also implies we do not have to make these two notify
   * operations a critical section.
   */

  if (readers_waken == 0 &&
      ramlog_bufferused(priv) >= CONFIG_RAMLOG_POLLTHRESHOLD)
    {
      /* Notify all poll/select waiters that they can read from the FIFO */

      ramlog_pollnotify(priv, POLLIN);
    }

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
