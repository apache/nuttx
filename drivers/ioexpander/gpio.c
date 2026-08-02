/****************************************************************************
 * drivers/ioexpander/gpio.c
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

#include <sys/types.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <assert.h>
#include <nuttx/debug.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>

#include <nuttx/fs/fs.h>
#include <nuttx/spinlock.h>
#include <nuttx/ioexpander/gpio.h>

#ifdef CONFIG_GPIO_PROCFS
#  include <sys/stat.h>
#  include <sys/param.h>
#  include <fcntl.h>
#  include <nuttx/kmalloc.h>
#  include <nuttx/list.h>
#  include <nuttx/mutex.h>
#  include <nuttx/fs/procfs.h>
#endif

#ifdef CONFIG_DEV_GPIO

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_GPIO_PROCFS

/* One registered pin.  struct gpio_dev_s belongs to the lower half, which
 * usually embeds it in a larger private structure of its own, so the list
 * node lives here rather than being added to it.
 */

/* Long enough for any name gpio_pin_register() accepts: it builds
 * "/dev/" plus the name in a 32 byte buffer.
 */

#define GPIO_PROCFS_NAMELEN 28

struct gpio_entry_s
{
  struct list_node node;
  FAR struct gpio_dev_s *dev;
  char name[GPIO_PROCFS_NAMELEN];
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     gpio_handler(FAR struct gpio_dev_s *dev, uint8_t pin);
static int     gpio_open(FAR struct file *filep);
static ssize_t gpio_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t gpio_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static off_t   gpio_seek(FAR struct file *filep, off_t offset, int whence);
static int     gpio_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
#ifdef CONFIG_GPIO_PROCFS
static int     gpio_procfs_open(FAR struct file *filep,
                                FAR const char *relpath,
                                int oflags, mode_t mode);
static int     gpio_procfs_close(FAR struct file *filep);
static ssize_t gpio_procfs_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen);
static int     gpio_procfs_dup(FAR const struct file *oldp,
                               FAR struct file *newp);
static int     gpio_procfs_stat(FAR const char *relpath,
                                FAR struct stat *buf);
static void    gpio_procfs_add(FAR struct gpio_dev_s *dev,
                               FAR const char *pinname);
static void    gpio_procfs_remove(FAR struct gpio_dev_s *dev);
#endif
static int     gpio_poll(FAR struct file *filep,
                         FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_gpio_drvrops =
{
  gpio_open,   /* open */
  NULL,        /* close */
  gpio_read,   /* read */
  gpio_write,  /* write */
  gpio_seek,   /* seek */
  gpio_ioctl,  /* ioctl */
  NULL,        /* mmap */
  NULL,        /* truncate */
  gpio_poll,   /* poll */
};

#ifdef CONFIG_GPIO_PROCFS

static struct list_node g_gpio_list = LIST_INITIAL_VALUE(g_gpio_list);
static mutex_t g_gpio_lock = NXMUTEX_INITIALIZER;
static bool g_gpio_procfs_added;

/* Indexed by enum gpio_pintype_e.  The enum's own comment warns that a
 * table like this has to be extended with it; the assertion below turns
 * forgetting into a build error rather than a pin type with no name.
 */

static const FAR char *g_gpio_typename[] =
{
  "INPUT",            /* GPIO_INPUT_PIN                  */
  "INPUT_PU",         /* GPIO_INPUT_PIN_PULLUP           */
  "INPUT_PD",         /* GPIO_INPUT_PIN_PULLDOWN         */
  "OUTPUT",           /* GPIO_OUTPUT_PIN                 */
  "OUTPUT_OD",        /* GPIO_OUTPUT_PIN_OPENDRAIN       */
  "INT",              /* GPIO_INTERRUPT_PIN              */
  "INT_HIGH",         /* GPIO_INTERRUPT_HIGH_PIN         */
  "INT_LOW",          /* GPIO_INTERRUPT_LOW_PIN          */
  "INT_RISING",       /* GPIO_INTERRUPT_RISING_PIN       */
  "INT_FALLING",      /* GPIO_INTERRUPT_FALLING_PIN      */
  "INT_BOTH",         /* GPIO_INTERRUPT_BOTH_PIN         */
  "INT_WAKE",         /* GPIO_INTERRUPT_PIN_WAKEUP       */
  "INT_HIGH_WAKE",    /* GPIO_INTERRUPT_HIGH_PIN_WAKEUP  */
  "INT_LOW_WAKE",     /* GPIO_INTERRUPT_LOW_PIN_WAKEUP   */
  "INT_RISING_WAKE",  /* GPIO_INTERRUPT_RISING_PIN_WAKEUP */
  "INT_FALLING_WAKE", /* GPIO_INTERRUPT_FALLING_PIN_WAKEUP */
  "INT_BOTH_WAKE",    /* GPIO_INTERRUPT_BOTH_PIN_WAKEUP  */
};

static_assert(nitems(g_gpio_typename) == GPIO_NPINTYPES,
              "pin type name table does not match enum gpio_pintype_e");

static const struct procfs_operations g_gpio_procfs_ops =
{
  gpio_procfs_open,   /* open */
  gpio_procfs_close,  /* close */
  gpio_procfs_read,   /* read */
  NULL,               /* write */
  NULL,               /* poll */

  gpio_procfs_dup,    /* dup */

  NULL,               /* opendir */
  NULL,               /* closedir */
  NULL,               /* readdir */
  NULL,               /* rewinddir */

  gpio_procfs_stat,   /* stat */
};

static const struct procfs_entry_s g_gpio_procfs =
{
  "gpio", &g_gpio_procfs_ops, PROCFS_FILE_TYPE
};

#endif /* CONFIG_GPIO_PROCFS */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_GPIO_PROCFS

/****************************************************************************
 * Name: gpio_procfs_open
 *
 * Description:
 *   Open /proc/gpio.  The entry is read only, and holds no state of its
 *   own beyond the position accounting procfs does for every file.
 *
 * Input Parameters:
 *   filep   - The file structure to attach the open file to
 *   relpath - The path below /proc being opened
 *   oflags  - Open flags; anything but read only is refused
 *   mode    - Ignored, the entry cannot be created
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int gpio_procfs_open(FAR struct file *filep, FAR const char *relpath,
                            int oflags, mode_t mode)
{
  FAR struct procfs_file_s *priv;

  if ((oflags & O_ACCMODE) != O_RDONLY)
    {
      return -EACCES;
    }

  priv = kmm_zalloc(sizeof(struct procfs_file_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  filep->f_priv = priv;
  return OK;
}

/****************************************************************************
 * Name: gpio_procfs_close
 *
 * Description:
 *   Close /proc/gpio and free what open() allocated.
 *
 * Input Parameters:
 *   filep - The open file
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int gpio_procfs_close(FAR struct file *filep)
{
  kmm_free(filep->f_priv);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: gpio_procfs_read
 *
 * Description:
 *   List every registered pin, in registration order, with what the upper
 *   half knows about it.  A lower half that supplies go_describe adds its
 *   own fields to the same line.
 *
 *   Each read renders from the start and skips what earlier reads already
 *   returned, so a file longer than the caller's buffer still comes out
 *   whole across successive reads.
 *
 * Input Parameters:
 *   filep  - The open file, carrying the offset reached so far
 *   buffer - Where to return the text
 *   buflen - Size of buffer
 *
 * Returned Value:
 *   The number of bytes returned, zero at end of file, or a negated errno
 *   on failure.
 *
 ****************************************************************************/

static ssize_t gpio_procfs_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  FAR struct gpio_entry_s *entry;
  size_t remaining = buflen;
  FAR char *dest = buffer;
  off_t pos = filep->f_pos;
  char extra[48];
  char line[128];
  bool value;
  size_t n;
  int ret;

  ret = nxmutex_lock(&g_gpio_lock);
  if (ret < 0)
    {
      return ret;
    }

  list_for_every_entry(&g_gpio_list, entry, struct gpio_entry_s, node)
    {
      FAR const char *type = "-";

      if (remaining == 0)
        {
          break;
        }

      /* A lower half is free to invent a pin type this table has never
       * heard of, so the index is bounded here rather than trusted.
       */

      if (entry->dev->gp_pintype < nitems(g_gpio_typename))
        {
          type = g_gpio_typename[entry->dev->gp_pintype];
        }

      /* A pin that cannot be read reports -, so that a failed read is
       * not shown as a low level.
       */

      if (entry->dev->gp_ops->go_read != NULL &&
          entry->dev->gp_ops->go_read(entry->dev, &value) >= 0)
        {
          n = snprintf(line, sizeof(line), "%-12s type:%-18s val:%u",
                       entry->name, type, value);
        }
      else
        {
          n = snprintf(line, sizeof(line), "%-12s type:%-18s val:-",
                       entry->name, type);
        }

      n += snprintf(line + n, sizeof(line) - n,
                    " regs:%u ints:%" PRIuPTR,
                    entry->dev->register_count, entry->dev->int_count);

      extra[0] = '\0';
      if (entry->dev->gp_ops->go_describe != NULL)
        {
          entry->dev->gp_ops->go_describe(entry->dev, extra, sizeof(extra));
          extra[sizeof(extra) - 1] = '\0';
        }

      if (extra[0] != '\0')
        {
          n += snprintf(line + n, sizeof(line) - n, " %s", extra);
        }

      n += snprintf(line + n, sizeof(line) - n, "\n");

      /* snprintf() reports the length it wanted, so a line longer than the
       * buffer would otherwise carry n past it.
       */

      if (n >= sizeof(line))
        {
          n = sizeof(line) - 1;
          line[n - 1] = '\n';
        }

      n = procfs_memcpy(line, n, dest, remaining, &pos);
      dest += n;
      remaining -= n;
    }

  nxmutex_unlock(&g_gpio_lock);

  filep->f_pos += (dest - buffer);
  return dest - buffer;
}

/****************************************************************************
 * Name: gpio_procfs_dup
 *
 * Description:
 *   Duplicate an open /proc/gpio, copying the position reached so that the
 *   new file continues where the old one had got to.
 *
 * Input Parameters:
 *   oldp - The open file being duplicated
 *   newp - The file structure to attach the duplicate to
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int gpio_procfs_dup(FAR const struct file *oldp,
                           FAR struct file *newp)
{
  FAR struct procfs_file_s *priv;

  priv = kmm_zalloc(sizeof(struct procfs_file_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  memcpy(priv, oldp->f_priv, sizeof(struct procfs_file_s));
  newp->f_priv = priv;
  return OK;
}

/****************************************************************************
 * Name: gpio_procfs_stat
 *
 * Description:
 *   Report /proc/gpio as a read only regular file.
 *
 * Input Parameters:
 *   relpath - The path below /proc being queried
 *   buf     - Where to return the status
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int gpio_procfs_stat(FAR const char *relpath, FAR struct stat *buf)
{
  buf->st_mode    = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;
  return OK;
}

/****************************************************************************
 * Name: gpio_procfs_add
 *
 * Description:
 *   Remember a pin, and create /proc/gpio when the first one appears.
 *   procfs_register() has to run before procfs is mounted, which holds
 *   for the pins a board registers during start up.  The entry is claimed
 *   once for the lifetime of the system, since procfs_register() appends
 *   without checking for duplicates.
 *
 * Input Parameters:
 *   dev     - The pin being registered
 *   pinname - The name it was registered under, used as the /proc/gpio
 *             label; the caller's string must outlive the pin
 *
 * Returned Value:
 *   None.  A pin that cannot be listed is still a working pin, so a
 *   failure here does not fail the registration.
 *
 ****************************************************************************/

static void gpio_procfs_add(FAR struct gpio_dev_s *dev,
                            FAR const char *pinname)
{
  FAR struct gpio_entry_s *entry;

  entry = kmm_zalloc(sizeof(struct gpio_entry_s));
  if (entry == NULL)
    {
      return;
    }

  entry->dev = dev;
  strlcpy(entry->name, pinname, sizeof(entry->name));

  nxmutex_lock(&g_gpio_lock);

  /* procfs_register() appends without checking for a duplicate, so the
   * entry is claimed once for the lifetime of the system rather than
   * whenever the list is empty.  Pins come and go at run time.
   */

  if (!g_gpio_procfs_added)
    {
      procfs_register(&g_gpio_procfs);
      g_gpio_procfs_added = true;
    }

  list_add_tail(&g_gpio_list, &entry->node);
  nxmutex_unlock(&g_gpio_lock);
}

/****************************************************************************
 * Name: gpio_procfs_remove
 *
 * Description:
 *   Forget a pin.  /proc/gpio stays, since procfs has no way to withdraw
 *   an entry, and lists nothing once the last pin has gone.
 *
 * Input Parameters:
 *   dev - The pin being unregistered
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void gpio_procfs_remove(FAR struct gpio_dev_s *dev)
{
  FAR struct gpio_entry_s *entry;

  nxmutex_lock(&g_gpio_lock);

  list_for_every_entry(&g_gpio_list, entry, struct gpio_entry_s, node)
    {
      if (entry->dev == dev)
        {
          list_delete(&entry->node);
          kmm_free(entry);
          break;
        }
    }

  nxmutex_unlock(&g_gpio_lock);
}

#endif /* CONFIG_GPIO_PROCFS */

/****************************************************************************
 * Name: gpio_handler
 *
 * Description:
 *   GPIO interrupt callback function.
 *
 ****************************************************************************/

static int gpio_handler(FAR struct gpio_dev_s *dev, uint8_t pin)
{
#if CONFIG_DEV_GPIO_NSIGNALS > 0
  int i;
#endif

  DEBUGASSERT(dev != NULL);

  dev->int_count++;

#if CONFIG_DEV_GPIO_NPOLLWAITERS > 0
  poll_notify(dev->fds, CONFIG_DEV_GPIO_NPOLLWAITERS, POLLIN);
#endif

#if CONFIG_DEV_GPIO_NSIGNALS > 0
  for (i = 0; i < CONFIG_DEV_GPIO_NSIGNALS; i++)
    {
      FAR struct gpio_signal_s *signal = &dev->gp_signals[i];

      if (signal->gp_pid == 0)
        {
          break;
        }

      nxsig_notification(signal->gp_pid, &signal->gp_event,
                         SI_QUEUE, &signal->gp_work);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: gpio_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int gpio_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct gpio_dev_s *dev;

  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  filep->f_priv = (FAR void *)dev->int_count;
  return OK;
}

/****************************************************************************
 * Name: gpio_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t gpio_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct inode *inode;
  FAR struct gpio_dev_s *dev;
  int ret;

  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  DEBUGASSERT(buffer != NULL);
  if (buflen == 0)
    {
      return 0;  /* Zero will be interpreted as the End-of-File. */
    }

  /* Check for End-of-File.
   *
   * REVISIT:  Returning End-of-File after one byte has been written permits
   * you to cat or echo the GPIO.  This, however, is an un-natural use of a
   * file position since there is no file position associated with a GPIO.
   * It also makes the read() method difficult to use programmatically.
   */

  if (filep->f_pos > 0)
    {
      return 0;
    }

  /* Update interrupt count and read the GPIO value */

  filep->f_priv = (FAR void *)dev->int_count;

  ret = dev->gp_ops->go_read(dev, (FAR bool *)&buffer[0]);
  if (ret < 0)
    {
      return ret;
    }

  /* Convert the GPIO value to ASCII and increment the file position */

  buffer[0]    += '0';
  filep->f_pos  = 1;
  return 1;
}

/****************************************************************************
 * Name: gpio_write
 *
 * Description:
 *   Standard character driver write method.
 *
 *   REVISIT:  The read() method obeys the semantics of a file position and
 *   requires re-opening the driver or seeking to address 0.  The write()
 *   method does not.  This is an inconsistency.
 *
 ****************************************************************************/

static ssize_t gpio_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode;
  FAR struct gpio_dev_s *dev;
  int ret;
  bool val;

  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  DEBUGASSERT(buffer != NULL);

  /* Check if this pin is write-able */

  if (dev->gp_pintype != GPIO_OUTPUT_PIN &&
      dev->gp_pintype != GPIO_OUTPUT_PIN_OPENDRAIN)
    {
      return -EACCES;
    }

  /* Verify that a buffer containing data was provided */

  DEBUGASSERT(buffer != NULL);
  if (buflen != 0)
    {
      /* Only values '0' and '1' can be written */

      if (buffer[0] == '0')
        {
          val = 0;
        }
      else if (buffer[0] == '1')
        {
          val = 1;
        }
      else
        {
          return -EINVAL;
        }

      /* Write the GPIO value */

      ret = dev->gp_ops->go_write(dev, val);
      if (ret < 0)
        {
          return ret;
        }

      /* One byte written */

      return 1;
    }

  return 0;
}

/****************************************************************************
 * Name: gpio_seek
 *
 * Description:
 *   Reset read flag on seek to 0
 *
 *   REVISIT:  Seeking address zero is required to return addition GPIO
 *   values from read().  This, however, is an un-natural use of a file
 *   position since there is no file position associated with a GPIO.  It
 *   also makes the read() method difficult to use programmatically.
 *
 ****************************************************************************/

static off_t gpio_seek(FAR struct file *filep, off_t offset, int whence)
{
  /* Only SEEK_SET is supported, return ENOSYS for other valid options */

  if (whence == SEEK_CUR || whence == SEEK_END)
    {
      return -ENOSYS;
    }

  /* Only Offset zero makes sense,  POSIX permits setting the file position
   * beyond the end of the file, but that makes little sense here.
   */

  if (whence == SEEK_SET && offset == 0)
    {
      filep->f_pos = 0;
      return 0;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: gpio_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int gpio_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct gpio_dev_s *dev;
  irqstate_t flags;
  int ret = OK;
#if CONFIG_DEV_GPIO_NSIGNALS > 0
  pid_t pid;
  int i;
  int j;
#endif

  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;
  DEBUGASSERT(dev->gp_ops != NULL);

  switch (cmd)
    {
      /* Command:     GPIOC_WRITE
       * Description: Set the value of an output GPIO
       * Argument:    0=output a low value; 1=output a high value
       */

      case GPIOC_WRITE:
        if (dev->gp_pintype == GPIO_OUTPUT_PIN ||
            dev->gp_pintype == GPIO_OUTPUT_PIN_OPENDRAIN)
          {
            DEBUGASSERT(arg == 0ul || arg == 1ul);
            DEBUGASSERT(dev->gp_ops->go_write != NULL);
            ret = dev->gp_ops->go_write(dev, (bool)arg);
          }
        else
          {
            ret = -EACCES;
          }
        break;

      /* Command:     GPIOC_READ
       * Description: Read the value of an input or output GPIO
       * Argument:    A pointer to an bool value to receive the result:
       *              false=low value; true=high value.
       */

      case GPIOC_READ:
        {
          FAR bool *ptr = (FAR bool *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          filep->f_priv = (FAR void *)dev->int_count;

          DEBUGASSERT(dev->gp_ops->go_read != NULL);
          ret = dev->gp_ops->go_read(dev, ptr);
          DEBUGASSERT(ret < 0 || *ptr == 0 || *ptr == 1);
        }
        break;

      /* Command:     GPIOC_PINTYPE
       * Description: Return the GPIO pin type.
       * Argument:    A pointer to an instance of type enum gpio_pintype_e
       */

      case GPIOC_PINTYPE:
        {
          FAR enum gpio_pintype_e *ptr =
            (FAR enum gpio_pintype_e *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = (enum gpio_pintype_e)dev->gp_pintype;
        }
        break;

      /* Command:     GPIOC_REGISTER
       * Description: Register to receive a signal whenever there an
       *              interrupt is received on an input gpio pin.  This
       *              feature, of course, depends upon interrupt GPIO
       *              support from the platform.
       * Argument:    The number of signal to be generated when the
       *              interrupt occurs.
       */

      case GPIOC_REGISTER:
        if (dev->gp_pintype >= GPIO_INTERRUPT_PIN)
          {
            flags = enter_critical_section();
#if CONFIG_DEV_GPIO_NSIGNALS > 0
            if (arg)
              {
                pid = nxsched_getpid();
                for (i = 0; i < CONFIG_DEV_GPIO_NSIGNALS; i++)
                  {
                    FAR struct gpio_signal_s *signal = &dev->gp_signals[i];

                    if (signal->gp_pid == 0 || signal->gp_pid == pid)
                      {
                        memcpy(&signal->gp_event, (FAR void *)arg,
                               sizeof(signal->gp_event));
                        signal->gp_pid = pid;
                        break;
                      }
                  }

                if (i == CONFIG_DEV_GPIO_NSIGNALS)
                  {
                    leave_critical_section(flags);
                    ret = -EBUSY;
                    break;
                  }
              }
#endif

            if (dev->register_count++ > 0)
              {
                leave_critical_section(flags);
                break;
              }

            leave_critical_section(flags);

            /* Register our handler */

            DEBUGASSERT(dev->gp_ops->go_attach != NULL);
            ret = dev->gp_ops->go_attach(dev,
                                         (pin_interrupt_t)gpio_handler);
            if (ret >= 0)
              {
                /* Enable pin interrupts */

                DEBUGASSERT(dev->gp_ops->go_enable != NULL);
                ret = dev->gp_ops->go_enable(dev, true);
              }
          }
        else
          {
            ret = -EACCES;
          }
        break;

      /* Command:     GPIOC_UNREGISTER
       * Description: Stop receiving signals for pin interrupts.
       * Argument:    None.
       */

      case GPIOC_UNREGISTER:
        if (dev->gp_pintype >= GPIO_INTERRUPT_PIN)
          {
            flags = enter_critical_section();
#if CONFIG_DEV_GPIO_NSIGNALS > 0
            pid = nxsched_getpid();
            for (i = 0; i < CONFIG_DEV_GPIO_NSIGNALS; i++)
              {
                if (pid == dev->gp_signals[i].gp_pid)
                  {
                    for (j = i + 1; j < CONFIG_DEV_GPIO_NSIGNALS; j++)
                      {
                        if (dev->gp_signals[j].gp_pid == 0)
                          {
                            break;
                          }
                      }

                    if (i != --j)
                      {
                        memcpy(&dev->gp_signals[i], &dev->gp_signals[j],
                               sizeof(dev->gp_signals[i]));
                      }

                    dev->gp_signals[j].gp_pid = 0;
                    nxsig_cancel_notification(&dev->gp_signals[j].gp_work);
                    break;
                  }
              }
#endif

            if (--dev->register_count > 0)
              {
                leave_critical_section(flags);
                break;
              }

            leave_critical_section(flags);

            /* Make sure that the pin interrupt is disabled */

            DEBUGASSERT(dev->gp_ops->go_enable != NULL);
            ret = dev->gp_ops->go_enable(dev, false);
            if (ret >= 0)
              {
                /* Detach the handler */

                DEBUGASSERT(dev->gp_ops->go_attach != NULL);
                ret = dev->gp_ops->go_attach(dev, NULL);
              }
          }
        else
          {
            ret = -EACCES;
          }
        break;

      /* Command:     GPIOC_SETPINTYPE
       * Description: Set the GPIO pin type.
       * Argument:    The enum gpio_pintype_e type.
       */

      case GPIOC_SETPINTYPE:
        {
          enum gpio_pintype_e pintype = (enum gpio_pintype_e)arg;

          /* Check if the argument is a valid pintype */

          if (pintype >= GPIO_NPINTYPES)
            {
              ret = -EINVAL;
              break;
            }

          /* Check if the pintype actually needs to be changed */

          if (dev->gp_pintype == pintype)
            {
              /* Pintype remains the same, no need to change anything */

              break;
            }

          /* Disable interrupt if pin had an interrupt pintype previously */

          if (dev->gp_pintype >= GPIO_INTERRUPT_PIN)
            {
              DEBUGASSERT(dev->gp_ops->go_enable != NULL);
              ret = dev->gp_ops->go_enable(dev, false);
              if (ret < 0)
                {
                  break;
                }
            }

          /* Change pintype */

          DEBUGASSERT(dev->gp_ops->go_setpintype != NULL);
          ret = dev->gp_ops->go_setpintype(dev, pintype);
          if (ret < 0)
            {
              break;
            }

          /* Additional DEBUGASSERTs to make sure the right operations are
           * available after the pintype has been changed.
           */

          DEBUGASSERT(dev->gp_pintype == pintype);
          DEBUGASSERT(dev->gp_ops != NULL);
          DEBUGASSERT(dev->gp_ops->go_read != NULL);
          DEBUGASSERT(dev->gp_ops->go_setpintype != NULL);

          if (pintype >= GPIO_OUTPUT_PIN && pintype < GPIO_INTERRUPT_PIN)
            {
              DEBUGASSERT(dev->gp_ops->go_write != NULL);
            }
          else if (pintype >= GPIO_INTERRUPT_PIN)
            {
              DEBUGASSERT(dev->gp_ops->go_attach != NULL);
              DEBUGASSERT(dev->gp_ops->go_enable != NULL);
            }
        }
        break;

      /* Command:     GPIOC_SETDEBOUNCE
       * Description: Set the GPIO pin debounce duration.
       * Argument:    The duration of the channel debounce, uint is ns.
       */

      case GPIOC_SETDEBOUNCE:
        {
          DEBUGASSERT(dev->gp_ops->go_setdebounce != NULL);
          ret = dev->gp_ops->go_setdebounce(dev, arg);
          break;
        }

      /* Command:     GPIOC_SETMASK
       * Description: Mask or unmask the GPIO interrupt without disabling it.
       *              When masked, the interrupt is suppressed but the
       *              interrupt source remains enabled.
       * Argument:    true to mask the interrupt;
       *              false to unmask the interrupt.
       */

    case GPIOC_IRQ_SETMASK:
        {
          bool mask = (bool)arg;
          DEBUGASSERT(dev->gp_ops->go_setmask != NULL);
          ret = dev->gp_ops->go_setmask(dev, mask);
          break;
        }

      /* Unrecognized command */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: gpio_poll
 *
 * Description:
 *   Poll method for gpio device.
 *
 ****************************************************************************/

static int gpio_poll(FAR struct file *filep,
                     FAR struct pollfd *fds, bool setup)
{
#if CONFIG_DEV_GPIO_NPOLLWAITERS > 0
  FAR struct inode *inode = filep->f_inode;
  FAR struct gpio_dev_s *dev = inode->i_private;
  int i;
#endif

  irqstate_t flags;
  int ret = OK;

  /* Are we setting up the poll?  Or tearing it down? */

  flags = enter_critical_section();
  if (setup)
    {
#if CONFIG_DEV_GPIO_NPOLLWAITERS > 0
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_DEV_GPIO_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (dev->fds[i] == NULL)
            {
              /* Bind the poll structure and this slot */

              dev->fds[i] = fds;
              fds->priv   = &dev->fds[i];

              /* Report if a event is pending */

              if (dev->int_count != (uintptr_t)(filep->f_priv))
                {
                  poll_notify(&fds, 1, POLLIN);
                }

              break;
            }
        }

      if (i >= CONFIG_DEV_GPIO_NPOLLWAITERS)
#endif
        {
          fds->priv = NULL;
          ret       = -EBUSY;
        }
    }
  else if (fds->priv != NULL)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_pin_register
 *
 * Description:
 *   Register GPIO pin device driver at /dev/gpioN, where N is the provided
 *   minor number.
 *
 * Input Parameters:
 *   dev    - A pointer to a gpio_dev_s
 *   minor  - An integer value to be concatenated with '/dev/gpio'
 *            to form the device name.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on a failure
 *   all error values returned by inode_reserve:
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int gpio_pin_register(FAR struct gpio_dev_s *dev, int minor)
{
  char devname[16];

  snprintf(devname, sizeof(devname), "gpio%u", (unsigned int)minor);
  return gpio_pin_register_byname(dev, devname);
}

/****************************************************************************
 * Name: gpio_pin_register_byname
 *
 * Description:
 *   Register GPIO pin device driver with it's pin name.
 *
 * Input Parameters:
 *   dev      - A pointer to a gpio_dev_s
 *   pinname  - A pointer to the name to be concatenated with '/dev/'
 *              to form the device name.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on a failure
 *   all error values returned by inode_reserve:
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int gpio_pin_register_byname(FAR struct gpio_dev_s *dev,
                             FAR const char *pinname)
{
  char devname[32];
  int ret;

  DEBUGASSERT(dev != NULL && dev->gp_ops != NULL && pinname != NULL);

  switch (dev->gp_pintype)
    {
      case GPIO_INPUT_PIN:
      case GPIO_INPUT_PIN_PULLUP:
      case GPIO_INPUT_PIN_PULLDOWN:
        {
          DEBUGASSERT(dev->gp_ops->go_read != NULL);
        }
        break;

      case GPIO_OUTPUT_PIN:
      case GPIO_OUTPUT_PIN_OPENDRAIN:
        {
          DEBUGASSERT(dev->gp_ops->go_read != NULL &&
                      dev->gp_ops->go_write != NULL);
        }
        break;

      default:
        {
          DEBUGASSERT(dev->gp_ops->go_read != NULL &&
                      dev->gp_ops->go_attach != NULL &&
                      dev->gp_ops->go_enable != NULL);

          /* Make sure that the pin interrupt is disabled */

          ret = dev->gp_ops->go_enable(dev, false);
          if (ret < 0)
            {
              return ret;
            }
        }
        break;
    }

  snprintf(devname, sizeof(devname), "/dev/%s", pinname);

  gpioinfo("Registering %s\n", devname);

  ret = register_driver(devname, &g_gpio_drvrops, 0600, dev);

#ifdef CONFIG_GPIO_PROCFS
  if (ret >= 0)
    {
      gpio_procfs_add(dev, pinname);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: gpio_pin_unregister
 *
 * Description:
 *   Unregister GPIO pin device driver at /dev/gpioN, where N is the provided
 *   minor number.
 *
 * Input Parameters:
 *   dev    - A pointer to a gpio_dev_s
 *   minor  - An integer value to be concatenated with '/dev/gpio'
 *            to form the device name.
 *
 * Returned Value:
 *   Zero on success; A negated value is returned on a failure
 *   (all error values returned by inode_remove):
 *
 *   ENOENT - path does not exist.
 *   EBUSY  - Ref count is not 0;
 *
 ****************************************************************************/

int gpio_pin_unregister(FAR struct gpio_dev_s *dev, int minor)
{
  char devname[16];
  snprintf(devname, sizeof(devname), "gpio%u", (unsigned int)minor);
  return gpio_pin_unregister_byname(dev, devname);
}

/****************************************************************************
 * Name: gpio_pin_unregister_byname
 *
 * Description:
 *   Unregister GPIO pin device driver at /dev/pinname.
 *
 * Input Parameters:
 *   dev      - A pointer to a gpio_dev_s
 *   pinname  - A pointer to the name to be concatenated with '/dev/'
 *              to form the device name.
 *
 *
 * Returned Value:
 *   Zero on success; A negated value is returned on a failure
 *   (all error values returned by inode_remove):
 *
 *   ENOENT - path does not exist.
 *   EBUSY  - Ref count is not 0;
 ****************************************************************************/

int gpio_pin_unregister_byname(FAR struct gpio_dev_s *dev,
                               FAR const char *pinname)
{
  char devname[32];

  snprintf(devname, sizeof(devname), "/dev/%s", pinname);

  gpioinfo("Unregistering %s\n", devname);

#ifdef CONFIG_GPIO_PROCFS
  gpio_procfs_remove(dev);
#endif

  return unregister_driver(devname);
}

#endif /* CONFIG_DEV_GPIO */
