/****************************************************************************
 * drivers/pinctrl/pinctrl.c
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
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/pinctrl/pinctrl.h>

#ifdef CONFIG_PINCTRL_PROCFS
#  include <sys/stat.h>
#  include <fcntl.h>
#  include <nuttx/kmalloc.h>
#  include <nuttx/list.h>
#  include <nuttx/mutex.h>
#  include <nuttx/fs/procfs.h>
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_PINCTRL_PROCFS

/* One registered controller.  struct pinctrl_dev_s belongs to the caller
 * and holds only an operations pointer, so the list node lives here.
 */

struct pinctrl_entry_s
{
  struct list_node node;
  FAR struct pinctrl_dev_s *dev;
  int minor;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     pinctrl_open(FAR struct file *filep);
static int     pinctrl_close(FAR struct file *filep);
static ssize_t pinctrl_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t pinctrl_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     pinctrl_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

#ifdef CONFIG_PINCTRL_PROCFS
static int     pinctrl_procfs_open(FAR struct file *filep,
                                   FAR const char *relpath,
                                   int oflags, mode_t mode);
static int     pinctrl_procfs_close(FAR struct file *filep);
static ssize_t pinctrl_procfs_read(FAR struct file *filep,
                                   FAR char *buffer, size_t buflen);
static int     pinctrl_procfs_dup(FAR const struct file *oldp,
                                  FAR struct file *newp);
static int     pinctrl_procfs_stat(FAR const char *relpath,
                                   FAR struct stat *buf);
static void    pinctrl_procfs_add(FAR struct pinctrl_dev_s *dev, int minor);
static void    pinctrl_procfs_remove(FAR struct pinctrl_dev_s *dev,
                                     int minor);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pinctrl_drvrops =
{
  pinctrl_open,  /* open */
  pinctrl_close, /* close */
  pinctrl_read,  /* read */
  pinctrl_write, /* write */
  NULL,          /* seek */
  pinctrl_ioctl  /* ioctl */
};

#ifdef CONFIG_PINCTRL_PROCFS

static struct list_node g_pinctrl_list =
                        LIST_INITIAL_VALUE(g_pinctrl_list);
static mutex_t g_pinctrl_lock = NXMUTEX_INITIALIZER;
static bool g_pinctrl_procfs_added;

static const struct procfs_operations g_pinctrl_procfs_ops =
{
  pinctrl_procfs_open,   /* open */
  pinctrl_procfs_close,  /* close */
  pinctrl_procfs_read,   /* read */
  NULL,                  /* write */
  NULL,                  /* poll */

  pinctrl_procfs_dup,    /* dup */

  NULL,                  /* opendir */
  NULL,                  /* closedir */
  NULL,                  /* readdir */
  NULL,                  /* rewinddir */

  pinctrl_procfs_stat,   /* stat */
};

static const struct procfs_entry_s g_pinctrl_procfs =
{
  "pinctrl", &g_pinctrl_procfs_ops, PROCFS_FILE_TYPE
};

#endif /* CONFIG_PINCTRL_PROCFS */

#ifdef CONFIG_PINCTRL_PROCFS

/****************************************************************************
 * Name: pinctrl_procfs_open
 *
 * Description:
 *   Open /proc/pinctrl.  The entry is read only, and holds no state of
 *   its own beyond the position accounting procfs does for every
 *   file.
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

static int pinctrl_procfs_open(FAR struct file *filep,
                               FAR const char *relpath,
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
 * Name: pinctrl_procfs_close
 *
 * Description:
 *   Close /proc/pinctrl and free what open() allocated.
 *
 * Input Parameters:
 *   filep - The open file
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int pinctrl_procfs_close(FAR struct file *filep)
{
  kmm_free(filep->f_priv);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: pinctrl_procfs_append
 *
 * Description:
 *   Append formatted text at offset n, clamping to the buffer.  snprintf
 *   returns the length it wanted, so an unclamped sum would carry the
 *   offset past the buffer and wrap the remaining size.  The offset
 *   returned never exceeds len - 1.
 *
 * Input Parameters:
 *   line - The line being built
 *   len  - Size of line
 *   n    - Offset to append at
 *   fmt  - Format string, followed by its arguments
 *
 * Returned Value:
 *   The offset after the text, never more than len - 1.
 *
 ****************************************************************************/

static size_t pinctrl_procfs_append(FAR char *line, size_t len, size_t n,
                                    FAR const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  n += vsnprintf(line + n, len - n, fmt, ap);
  va_end(ap);

  return n < len ? n : len - 1;
}

/****************************************************************************
 * Name: pinctrl_procfs_field
 *
 * Description:
 *   Append one key:value token, or key:- when the field's validity bit is
 *   clear, so every line carries the same tokens and absence is explicit.
 *
 * Input Parameters:
 *   line - The line being built
 *   len  - Size of line
 *   n    - Offset to append at
 *   key  - Token name
 *   have - The pad's PINCTRL_HAVE_* validity bits
 *   bit  - The bit that makes this field meaningful
 *   val  - The value, used only when that bit is set
 *
 * Returned Value:
 *   The offset after the token.
 *
 ****************************************************************************/

static size_t pinctrl_procfs_field(FAR char *line, size_t len, size_t n,
                                   FAR const char *key, uint32_t have,
                                   uint32_t bit, uint32_t val)
{
  if ((have & bit) != 0)
    {
      return pinctrl_procfs_append(line, len, n, "%s:%" PRIu32 " ",
                                   key, val);
    }

  return pinctrl_procfs_append(line, len, n, "%s:- ", key);
}

/****************************************************************************
 * Name: pinctrl_procfs_pad
 *
 * Description:
 *   Render one pad as a single line of key:value tokens, every line the
 *   same tokens in the same order, then the controller's extra fields.
 *
 * Input Parameters:
 *   line - Where to render the line
 *   len  - Size of line
 *   pin  - The pad's number
 *   info - What the controller reported for it
 *
 * Returned Value:
 *   The length of the rendered line.
 *
 ****************************************************************************/

static size_t pinctrl_procfs_pad(FAR char *line, size_t len, uint32_t pin,
                                 FAR const struct pinctrl_padinfo_s *info)
{
  size_t n;

  n = pinctrl_procfs_append(line, len, 0, "%-4" PRIu32 " %-20s ", pin,
                            info->name[0] != '\0' ? info->name : "-");

  n = pinctrl_procfs_field(line, len, n, "func", info->have,
                           PINCTRL_HAVE_FUNCTION, info->function);
  n = pinctrl_procfs_append(line, len, n, "sel:%-16s ",
                            info->funcname[0] != '\0' ?
                            info->funcname : "-");
  n = pinctrl_procfs_field(line, len, n, "ds", info->have,
                           PINCTRL_HAVE_STRENGTH, info->strength);
  n = pinctrl_procfs_field(line, len, n, "pu", info->have,
                           PINCTRL_HAVE_PULL, info->pullup);
  n = pinctrl_procfs_field(line, len, n, "pd", info->have,
                           PINCTRL_HAVE_PULL, info->pulldown);
  n = pinctrl_procfs_field(line, len, n, "ie", info->have,
                           PINCTRL_HAVE_INPUT, info->input);
  n = pinctrl_procfs_field(line, len, n, "smt", info->have,
                           PINCTRL_HAVE_SCHMITT, info->schmitt);
  n = pinctrl_procfs_field(line, len, n, "slew", info->have,
                           PINCTRL_HAVE_SLEWRATE, info->slewrate);

  /* The fields end with a separator; take it back so a line with no extra
   * text does not end in a blank.
   */

  if (info->extra[0] != '\0')
    {
      n = pinctrl_procfs_append(line, len, n, "%s\n", info->extra);
    }
  else
    {
      if (n > 0 && line[n - 1] == ' ')
        {
          n--;
        }

      n = pinctrl_procfs_append(line, len, n, "\n");
    }

  /* A truncated line still has to end the record */

  if (line[n - 1] != '\n')
    {
      line[n - 1] = '\n';
    }

  return n;
}

/****************************************************************************
 * Name: pinctrl_procfs_read
 *
 * Description:
 *   Ask every registered controller to describe each of its pads, in
 *   registration order.  A controller with no get_pad method contributes
 *   its name and a note.
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

static ssize_t pinctrl_procfs_read(FAR struct file *filep,
                                   FAR char *buffer, size_t buflen)
{
  struct pinctrl_padinfo_s info;
  FAR struct pinctrl_entry_s *entry;
  size_t remaining = buflen;
  FAR char *dest = buffer;
  off_t pos = filep->f_pos;
  char line[192];
  uint32_t pin;
  size_t n;
  int ret;

  ret = nxmutex_lock(&g_pinctrl_lock);
  if (ret < 0)
    {
      return ret;
    }

  list_for_every_entry(&g_pinctrl_list, entry, struct pinctrl_entry_s, node)
    {
      if (remaining == 0)
        {
          break;
        }

      n = snprintf(line, sizeof(line), "pinctrl%d: %" PRIu32 " pads\n",
                   entry->minor, entry->dev->npins);
      n = procfs_memcpy(line, n, dest, remaining, &pos);
      dest += n;
      remaining -= n;

      if (entry->dev->ops->get_pad == NULL)
        {
          n = snprintf(line, sizeof(line),
                       "  no detail, configuration is write only\n");
          n = procfs_memcpy(line, n, dest, remaining, &pos);
          dest += n;
          remaining -= n;
          continue;
        }

      for (pin = 0; pin < entry->dev->npins && remaining > 0; pin++)
        {
          memset(&info, 0, sizeof(info));
          if (entry->dev->ops->get_pad(entry->dev, pin, &info) < 0)
            {
              continue;
            }

          n = pinctrl_procfs_pad(line, sizeof(line), pin, &info);
          n = procfs_memcpy(line, n, dest, remaining, &pos);
          dest += n;
          remaining -= n;
        }
    }

  nxmutex_unlock(&g_pinctrl_lock);

  filep->f_pos += (dest - buffer);
  return dest - buffer;
}

/****************************************************************************
 * Name: pinctrl_procfs_dup
 *
 * Description:
 *   Duplicate an open /proc/pinctrl, copying the position reached so
 *   that the new file continues where the old one had got to.
 *
 * Input Parameters:
 *   oldp - The open file being duplicated
 *   newp - The file structure to attach the duplicate to
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int pinctrl_procfs_dup(FAR const struct file *oldp,
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
 * Name: pinctrl_procfs_stat
 *
 * Description:
 *   Report /proc/pinctrl as a read only regular file.
 *
 * Input Parameters:
 *   relpath - The path below /proc being queried
 *   buf     - Where to return the status
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int pinctrl_procfs_stat(FAR const char *relpath, FAR struct stat *buf)
{
  buf->st_mode    = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;
  return OK;
}

/****************************************************************************
 * Name: pinctrl_procfs_add
 *
 * Description:
 *   Remember a controller, and create /proc/pinctrl the first time one
 *   appears.  procfs_register() requires that procfs is not yet mounted,
 *   which holds because controllers register during board or architecture
 *   start up.
 *
 * Input Parameters:
 *   dev   - The controller being registered
 *   minor - Its /dev/pinctrl number, used as the /proc/pinctrl label
 *
 * Returned Value:
 *   None.  A controller that cannot be listed is still usable, so a
 *   failure here does not fail the registration.
 *
 ****************************************************************************/

static void pinctrl_procfs_add(FAR struct pinctrl_dev_s *dev, int minor)
{
  FAR struct pinctrl_entry_s *entry;

  entry = kmm_zalloc(sizeof(struct pinctrl_entry_s));
  if (entry == NULL)
    {
      return;
    }

  entry->dev   = dev;
  entry->minor = minor;

  nxmutex_lock(&g_pinctrl_lock);

  /* procfs_register() appends without checking for a duplicate, so the
   * entry is claimed once for the lifetime of the system rather than
   * whenever the list is empty.
   */

  if (!g_pinctrl_procfs_added)
    {
      procfs_register(&g_pinctrl_procfs);
      g_pinctrl_procfs_added = true;
    }

  list_add_tail(&g_pinctrl_list, &entry->node);
  nxmutex_unlock(&g_pinctrl_lock);
}

/****************************************************************************
 * Name: pinctrl_procfs_remove
 *
 * Description:
 *   Forget a controller.  /proc/pinctrl stays, since procfs has no
 *   way to withdraw an entry, and lists nothing once the last
 *   controller has gone.
 *
 * Input Parameters:
 *   dev   - The controller being unregistered
 *   minor - Its /dev/pinctrl number
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pinctrl_procfs_remove(FAR struct pinctrl_dev_s *dev, int minor)
{
  FAR struct pinctrl_entry_s *entry;

  nxmutex_lock(&g_pinctrl_lock);

  list_for_every_entry(&g_pinctrl_list, entry, struct pinctrl_entry_s, node)
    {
      if (entry->dev == dev && entry->minor == minor)
        {
          list_delete(&entry->node);
          kmm_free(entry);
          break;
        }
    }

  nxmutex_unlock(&g_pinctrl_lock);
}

#endif /* CONFIG_PINCTRL_PROCFS */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinctrl_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int pinctrl_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: pinctrl_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int pinctrl_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: pinctrl_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t pinctrl_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: pinctrl_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t pinctrl_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return buflen;
}

/****************************************************************************
 * Name: pinctrl_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int pinctrl_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct pinctrl_dev_s *dev;
  FAR struct pinctrl_param_s *param;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  switch (cmd)
    {
     /* Command:     PINCTRLC_SETFUNCTION
      * Description: Set the mux function of the pinctrl pin
      * Argument:    A pointer to an instance of struct pinctrl_param_s
      */

      case PINCTRLC_SETFUNCTION:
        {
          param = (FAR struct pinctrl_param_s *)((uintptr_t)arg);
          ret = PINCTRL_SETFUNCTION(dev, param->pin, param->para.function);
        }
      break;

     /* Command:     PINCTRLC_SETSTRENGTH
      * Description: Set the driver strength of the pinctrl pin
      * Argument:    A pointer to an instance of struct pinctrl_param_s
      */

      case PINCTRLC_SETSTRENGTH:
        {
          param = (FAR struct pinctrl_param_s *)((uintptr_t)arg);
          ret = PINCTRL_SETSTRENGTH(dev, param->pin, param->para.strength);
        }
      break;

     /* Command:     PINCTRLC_SETDRIVER
      * Description: Set the driver type of the pinctrl pin
      * Argument:    A pointer to an instance of struct pinctrl_param_s
      */

      case PINCTRLC_SETDRIVER:
        {
          param = (FAR struct pinctrl_param_s *)((uintptr_t)arg);
          ret = PINCTRL_SETDRIVER(dev, param->pin, param->para.type);
        }
      break;

     /* Command:     PINCTRLC_SETSLEWRATE
      * Description: Set slewrate of the pinctrl pin
      * Argument:    A pointer to an instance of struct pinctrl_param_s
      */

      case PINCTRLC_SETSLEWRATE:
        {
          param = (FAR struct pinctrl_param_s *)((uintptr_t)arg);
          ret = PINCTRL_SETSLEWRATE(dev, param->pin, param->para.slewrate);
        }
      break;

     /* Command:     PINCTRLC_SELECTGPIO
      * Description: Select gpio function of pinctrl pin
      * Argument:    The uint32_t pinctrl number
      */

      case PINCTRLC_SELECTGPIO:
        {
          ret = PINCTRL_SELECTGPIO(dev, arg);
        }
      break;

     /* Command:     PINCTRLC_GETPAD
      * Description: Describe the current configuration of one pad
      * Argument:    A pointer to an instance of struct pinctrl_getpad_s
      */

      case PINCTRLC_GETPAD:
        {
          FAR struct pinctrl_getpad_s *getpad =
            (FAR struct pinctrl_getpad_s *)((uintptr_t)arg);

          if (dev->ops->get_pad == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          memset(&getpad->info, 0, sizeof(getpad->info));
          ret = dev->ops->get_pad(dev, getpad->pin, &getpad->info);
        }
      break;

      /* Unrecognized command */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinctrl_register
 *
 * Description:
 *   Register PINCTRL device driver.
 *
 ****************************************************************************/

int pinctrl_register(FAR struct pinctrl_dev_s *dev, int minor)
{
  char devname[32];
  int ret;

  snprintf(devname, 16, "/dev/pinctrl%u", (unsigned int)minor);
  ret = register_driver(devname, &g_pinctrl_drvrops, 0600, dev);

#ifdef CONFIG_PINCTRL_PROCFS
  if (ret >= 0)
    {
      pinctrl_procfs_add(dev, minor);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: pinctrl_unregister
 *
 * Description:
 *   Unregister PINCTRL device driver.
 *
 ****************************************************************************/

void pinctrl_unregister(FAR struct pinctrl_dev_s *dev, int minor)
{
  char devname[32];

#ifdef CONFIG_PINCTRL_PROCFS
  pinctrl_procfs_remove(dev, minor);
#endif

  snprintf(devname, 16, "/dev/pinctrl%u", (unsigned int)minor);
  (void)unregister_driver(devname);
}
