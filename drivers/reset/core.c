/****************************************************************************
 * drivers/reset/core.c
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

#include <errno.h>
#include <nuttx/debug.h>
#include <string.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/reset/reset.h>
#include <nuttx/reset/reset-controller.h>

#ifdef CONFIG_RESET_PROCFS
#  include <sys/stat.h>
#  include <fcntl.h>
#  include <stdarg.h>
#  include <stdio.h>
#  include <string.h>
#  include <nuttx/fs/procfs.h>
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct list_node
g_reset_controller_list = LIST_INITIAL_VALUE(g_reset_controller_list);
static mutex_t g_reset_list_mutex = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* struct reset_control_array - an array of reset controls
 * @base: reset control for compatibility with reset control API functions
 * @num_rstcs: number of reset controls
 * @rstc: array of reset controls
 */

struct reset_control_array
{
  struct reset_control base;
  unsigned int num_rstcs;
  FAR struct reset_control *rstc[];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline FAR struct reset_control_array *
rstc_to_array(FAR struct reset_control *rstc)
{
  return container_of(rstc, struct reset_control_array, base);
}

/****************************************************************************
 * Name: reset_control_array_reset
 *
 * Description:
 *   This function is used to perform reset operation of a reset control
 *   array.
 *
 * Input Parameters:
 *   resets - An instance of a reset_control_array.
 *
 * Returned Value:
 *   A 0 in case of success, otherwise error.
 ****************************************************************************/

static int reset_control_array_reset(FAR struct reset_control_array *resets)
{
  int ret = 0;
  unsigned int i;

  for (i = 0; i < resets->num_rstcs; i++)
    {
      ret = reset_control_reset(resets->rstc[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: reset_control_array_assert
 *
 * Description:
 *   This function is used to perform assert operation of a reset control
 *   array.
 *
 * Input Parameters:
 *    resets - An instance of a reset_control_array.
 *
 * Returned Value:
 *   A 0 in case of success, otherwise error.
 ****************************************************************************/

static int reset_control_array_assert(FAR struct reset_control_array *resets)
{
  int ret = 0;
  unsigned int i;

  for (i = 0; i < resets->num_rstcs; i++)
    {
      ret = reset_control_assert(resets->rstc[i]);
      if (ret < 0)
        {
          while (i--)
            {
              reset_control_deassert(resets->rstc[i]);
            }

          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: reset_control_array_deassert
 *
 * Description:
 *   This function is used to perform deassert operation of a reset control
 *   array.
 *
 * Input Parameters:
 *   resets - An instance of a reset_control_array.
 *
 * Returned Value:
 *   A 0 in case of success, an negative error code otherwise.
 ****************************************************************************/

static int
reset_control_array_deassert(FAR struct reset_control_array *resets)
{
  int ret = 0;
  unsigned int i;

  for (i = 0; i < resets->num_rstcs; i++)
    {
      ret = reset_control_deassert(resets->rstc[i]);
      if (ret < 0)
        {
          while (i--)
            {
              reset_control_assert(resets->rstc[i]);
            }

          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: reset_control_array_acquire
 *
 * Description:
 *   This function is used to perform acquire operation of a reset control
 *   array.
 *
 * Input Parameters:
 *    resets - An instance of a reset_control_array.
 *
 * Returned Value:
 *   A 0 in case of success, an negative error code otherwise.
 ****************************************************************************/

static int
reset_control_array_acquire(FAR struct reset_control_array *resets)
{
  unsigned int i;
  unsigned int ret = 0;

  for (i = 0; i < resets->num_rstcs; i++)
    {
      ret = reset_control_acquire(resets->rstc[i]);
      if (ret < 0)
        {
          while (i--)
            {
              reset_control_release(resets->rstc[i]);
            }

          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: reset_control_array_release
 *
 * Description:
 *   This function is used to perform release operation of a reset control
 *   array.
 *
 * Input Parameters:
 *   resets - An instance of a reset_control_array.
 ****************************************************************************/

static void
reset_control_array_release(FAR struct reset_control_array *resets)
{
  unsigned int i;

  for (i = 0; i < resets->num_rstcs; i++)
    {
      reset_control_release(resets->rstc[i]);
    }
}

/****************************************************************************
 * Name: reset_control_get_internal
 *
 * Description:
 *   This function is used to return a reset_control by rcdev,index,shared,
 *   acquired parameters.
 *
 * Input Parameters:
 *   rcdev    - An instance of reset_controller_dev type.
 *   index    - ID of the reset controller in the reset controller device.
 *   shared   - Is this a shared (1), or an exclusive (0) reset_control.
 *   acquired - Only one reset_control may be acquired for a given rcdev and
 *   index.
 *
 * Returned Value:
 *   Return reset_control if success, others NULL.
 ****************************************************************************/

static FAR struct reset_control *
reset_control_get_internal(FAR struct reset_controller_dev *rcdev,
                           unsigned int index, bool shared, bool acquired)
{
  FAR struct reset_control *rstc;

  DEBUGASSERT(nxmutex_is_locked(&g_reset_list_mutex));

  list_for_every_entry(&rcdev->reset_control_head, rstc,
                       struct reset_control, list)
    {
      if (rstc->id == index)
        {
          /* Allow creating a secondary exclusive reset_control
           * that is initially not acquired for an already
           * controlled reset line.
           */

          if (!rstc->shared && !shared && !acquired)
            {
              break;
            }

          /* shared reset controller */

          if (!rstc->shared || !shared)
            {
              rsterr("not shared reset control\n");
              return NULL;
            }

          atomic_fetch_add(&rstc->refcnt, 1);
          return rstc;
        }
    }

  rstc = kmm_zalloc(sizeof(*rstc));
  if (!rstc)
    {
      return NULL;
    }

#if defined(CONFIG_RESET_RPMSG)

  /* Only client defines this function */

  if (rcdev->ops->acquire)
    {
      int ret = rcdev->ops->acquire(rcdev, index, shared, acquired);

      if (ret < 0)
        {
          kmm_free(rstc);
          return NULL;
        }
    }
#endif

  rstc->rcdev = rcdev;
  list_add_after(&rcdev->reset_control_head, &rstc->list);
  rstc->id = index;
  atomic_set(&rstc->refcnt, 1);
  rstc->acquired = acquired;
  rstc->shared = shared;

  return rstc;
}

/****************************************************************************
 * Name: reset_control_put_internal
 *
 * Description:
 *   This is used to free a reset_control getted by reset_control_get.
 *
 * Input Parameters:
 *   rstc - An reset control
 *
 ****************************************************************************/

static void reset_control_put_internal(FAR struct reset_control *rstc)
{
  DEBUGASSERT(nxmutex_is_locked(&g_reset_list_mutex));

  if (atomic_fetch_sub(&rstc->refcnt, 1) == 1)
    {
      DEBUGASSERT(nxmutex_is_locked(&g_reset_list_mutex));
      list_delete(&rstc->list);

#if defined(CONFIG_RESET_RPMSG)

      /* Only client defines this function */

      if (rstc->rcdev->ops->release)
        {
          rstc->rcdev->ops->release(rstc->rcdev, rstc->id);
        }
#endif

      kmm_free(rstc);
    }
}

/****************************************************************************
 * Name: reset_control_array_put
 *
 * Description:
 *   This function is used to perform a free operation of reset
 *   control array.
 *
 * Input Parameters:
 *   resets - An instance of reset_control_array.
 *
 ****************************************************************************/

static void reset_control_array_put(FAR struct reset_control_array *resets)
{
  unsigned int i;

  nxmutex_lock(&g_reset_list_mutex);
  for (i = 0; i < resets->num_rstcs; i++)
    {
      reset_control_put_internal(resets->rstc[i]);
    }

  nxmutex_unlock(&g_reset_list_mutex);
  kmm_free(resets);
}

/****************************************************************************
 * Name: reset_controller_get_by_name
 *
 * Description:
 *   This function is used to get a reset_controller_dev instance register
 *   by name.
 *
 * Input Parameters:
 *   name - An name of reset_controller_dev var register.
 *
 * Returned Value:
 *   Return reset_controller_dev if success, others failed.
 ****************************************************************************/

static FAR struct reset_controller_dev *
reset_controller_get_by_name(FAR const char *name)
{
  FAR struct reset_controller_dev *rcdev;

  nxmutex_lock(&g_reset_list_mutex);
  list_for_every_entry(&g_reset_controller_list, rcdev,
                       struct reset_controller_dev, list)
  {
    if (!strcmp(name, rcdev->name))
      {
        nxmutex_unlock(&g_reset_list_mutex);
        return rcdev;
      }
  }

  nxmutex_unlock(&g_reset_list_mutex);

#if defined(CONFIG_RESET_RPMSG)
  if (strchr(name, '/'))
    {
      return reset_rpmsg_get(name);
    }
#endif

  return NULL;
}

#ifdef CONFIG_RESET_PROCFS

/****************************************************************************
 * Name: reset_procfs_open
 *
 * Description:
 *   Open /proc/reset.  The entry is read only, and holds no state of
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

static int reset_procfs_open(FAR struct file *filep,
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
 * Name: reset_procfs_close
 *
 * Description:
 *   Close /proc/reset and free what open() allocated.
 *
 * Input Parameters:
 *   filep - The open file
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int reset_procfs_close(FAR struct file *filep)
{
  kmm_free(filep->f_priv);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: reset_procfs_append
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

static size_t reset_procfs_append(FAR char *line, size_t len, size_t n,
                                  FAR const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  n += vsnprintf(line + n, len - n, fmt, ap);
  va_end(ap);

  return n < len ? n : len - 1;
}

/****************************************************************************
 * Name: reset_procfs_line
 *
 * Description:
 *   Render one reset line as key:value tokens, every line the same tokens
 *   in the same order, then the controller's extra fields.  The state
 *   comes from status(), which a controller need not implement either; it
 *   renders as - when absent or when the call fails.
 *
 * Input Parameters:
 *   line  - Where to render the line
 *   len   - Size of line
 *   rcdev - The controller owning the line
 *   id    - The line's id within that controller
 *   info  - What the controller reported for it
 *
 * Returned Value:
 *   The length of the rendered line.
 *
 ****************************************************************************/

static size_t reset_procfs_line(FAR char *line, size_t len,
                                FAR struct reset_controller_dev *rcdev,
                                unsigned int id,
                                FAR const struct reset_lineinfo_s *info)
{
  FAR const char *state = "-";
  size_t n;
  int ret;

  if (rcdev->ops->status != NULL)
    {
      ret = rcdev->ops->status(rcdev, id);
      if (ret >= 0)
        {
          state = ret > 0 ? "asserted" : "released";
        }
    }

  n = reset_procfs_append(line, len, 0, "%-4u %-24s state:%-8s", id,
                          info->name[0] != '\0' ? info->name : "-", state);

  if (info->extra[0] != '\0')
    {
      n = reset_procfs_append(line, len, n, " %s", info->extra);
    }

  n = reset_procfs_append(line, len, n, "\n");

  /* A truncated line still has to end the record */

  if (line[n - 1] != '\n')
    {
      line[n - 1] = '\n';
    }

  return n;
}

/****************************************************************************
 * Name: reset_procfs_read
 *
 * Description:
 *   Describe every registered controller's lines, in registration order.
 *   A controller with no get_line method contributes its name and a note.
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

static ssize_t reset_procfs_read(FAR struct file *filep,
                                 FAR char *buffer, size_t buflen)
{
  FAR struct reset_controller_dev *rcdev;
  struct reset_lineinfo_s info;
  size_t remaining = buflen;
  FAR char *dest = buffer;
  off_t pos = filep->f_pos;
  char line[96];
  unsigned int id;
  size_t n;
  int ret;

  ret = nxmutex_lock(&g_reset_list_mutex);
  if (ret < 0)
    {
      return ret;
    }

  list_for_every_entry(&g_reset_controller_list, rcdev,
                       struct reset_controller_dev, list)
    {
      if (remaining == 0)
        {
          break;
        }

      /* The name alone: nlines is an id space rather than a count of
       * real lines, so reporting it here would overstate what follows.
       */

      n = snprintf(line, sizeof(line), "%s:\n", rcdev->name);
      n = procfs_memcpy(line, n, dest, remaining, &pos);
      dest += n;
      remaining -= n;

      if (rcdev->ops->get_line == NULL)
        {
          n = snprintf(line, sizeof(line),
                       "  no detail, lines are not enumerable\n");
          n = procfs_memcpy(line, n, dest, remaining, &pos);
          dest += n;
          remaining -= n;
          continue;
        }

      for (id = 0; id < rcdev->nlines && remaining > 0; id++)
        {
          memset(&info, 0, sizeof(info));
          if (rcdev->ops->get_line(rcdev, id, &info) < 0)
            {
              continue;
            }

          n = reset_procfs_line(line, sizeof(line), rcdev, id, &info);
          n = procfs_memcpy(line, n, dest, remaining, &pos);
          dest += n;
          remaining -= n;
        }
    }

  nxmutex_unlock(&g_reset_list_mutex);

  filep->f_pos += (dest - buffer);
  return dest - buffer;
}

/****************************************************************************
 * Name: reset_procfs_dup
 *
 * Description:
 *   Duplicate an open /proc/reset, copying the position reached so
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

static int reset_procfs_dup(FAR const struct file *oldp,
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
 * Name: reset_procfs_stat
 *
 * Description:
 *   Report /proc/reset as a read only regular file.
 *
 * Input Parameters:
 *   relpath - The path below /proc being queried
 *   buf     - Where to return the status
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int reset_procfs_stat(FAR const char *relpath, FAR struct stat *buf)
{
  buf->st_mode    = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;
  return OK;
}

static const struct procfs_operations g_reset_procfs_ops =
{
  reset_procfs_open,     /* open */
  reset_procfs_close,    /* close */
  reset_procfs_read,     /* read */
  NULL,                  /* write */
  NULL,                  /* poll */

  reset_procfs_dup,      /* dup */

  NULL,                  /* opendir */
  NULL,                  /* closedir */
  NULL,                  /* readdir */
  NULL,                  /* rewinddir */

  reset_procfs_stat,     /* stat */
};

static const struct procfs_entry_s g_reset_procfs =
{
  "reset", &g_reset_procfs_ops, PROCFS_FILE_TYPE
};

static bool g_reset_procfs_added;

#endif /* CONFIG_RESET_PROCFS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: reset_control_get
 *
 * Description:
 *   This function is used to get a reset control by reset controller name.
 *
 *   Firstly, get a reset controller device from list, and then call
 *   reset_control_get_internal function by index, shared or acquired
 *   parameters return a reset control.
 *
 * Input Parameters:
 *   name     - The reset controller name
 *   index    - The reset controller in reset controller device
 *   shared   - Is this a shared (1), or an exclusive (0) reset_control
 *   acquired - Flags that used to get a exclusive reset control
 *
 * Returned Value:
 *   Return reset_control if success, others return NULL if failed
 ****************************************************************************/

FAR struct reset_control *
reset_control_get(FAR const char *name, int index, bool shared,
                  bool acquired)
{
  FAR struct reset_control *rstc;
  FAR struct reset_controller_dev *rcdev;

  if (name == NULL)
    {
      return NULL;
    }

  /* ID of the reset controller in the reset controller device */

  if (index < 0)
    {
      rsterr("ID of the reset controller is invalid\n");
      return NULL;
    }

  if (shared && acquired)
    {
      rsterr("shared && acquired exist meanwhile\n");
      return NULL;
    }

  rcdev = reset_controller_get_by_name(name);
  if (!rcdev)
    {
      return NULL;
    }

  nxmutex_lock(&g_reset_list_mutex);

  /* g_reset_list_mutex also protects the rcdev's reset_control list */

  rstc = reset_control_get_internal(rcdev, index, shared, acquired);
  nxmutex_unlock(&g_reset_list_mutex);

  return rstc;
}

/****************************************************************************
 * Name: reset_control_reset
 *
 * Description:
 *   On a shared reset line the actual reset pulse is only triggered once for
 *   the lifetime of the reset_control instance: for all but the first caller
 *   this is a no-op.
 *   Consumers must not use reset_control_(de)assert on shared reset lines
 *   when reset_control_reset has been used.
 *
 * Input Parameters:
 *   rstc - Reset controller
 *
 * Returned Value:
 *   Returns a negative errno if not supported, a positive value if the
 *   reset line is asserted.
 ****************************************************************************/

int reset_control_reset(FAR struct reset_control *rstc)
{
  int ret;

  rstinfo("Enter: reset_control_reset\n");

  if (rstc == NULL)
    {
      rsterr("rstc is null\n");
      return -EINVAL;
    }

  if (rstc->array)
    {
      return reset_control_array_reset(rstc_to_array(rstc));
    }

  if (!rstc->rcdev->ops->reset)
    {
      rsterr("rstc callback is null\n");
      return -ENOTSUP;
    }

  if (rstc->shared)
    {
      if (atomic_read(&rstc->deassert_count) != 0)
        {
          return -EINVAL;
        }

      if (atomic_fetch_add(&rstc->triggered_count, 1) != 0)
        {
          return 0;
        }
    }
  else
    {
      if (!rstc->acquired)
        {
          return -EPERM;
        }
    }

  ret = rstc->rcdev->ops->reset(rstc->rcdev, rstc->id);

  /* shared:1 and reset failed, triggered_count subtract 1 */

  if (rstc->shared && ret < 0)
    {
      atomic_fetch_sub(&rstc->triggered_count, 1);
    }

  return ret;
}

/****************************************************************************
 * Name: reset_control_assert
 *
 * Description:
 *   Asserts the reset line.
 *
 *   Calling this on an exclusive reset controller guarantees that the reset
 *   will be asserted. When called on a shared reset controller the line may
 *   still be deasserted, as long as other users keep it so.
 *
 *   For shared reset controls a driver cannot expect the hw's registers and
 *   internal state to be reset, but must be prepared for this to happen.
 *   Consumers must not use reset_control_reset on shared reset lines when
 *   reset_control_(de)assert has been used.
 *   return -EBUSY.
 *
 * Input Parameters:
 *   rstc - Reset controller
 *
 * Returned Value:
 *   Returns a negative errno if not supported, a positive value if the
 *   reset line is asserted.
 ****************************************************************************/

int reset_control_assert(FAR struct reset_control *rstc)
{
  rstinfo("Enter: reset_control_assert\n");
  if (rstc == NULL)
    {
      rsterr("rstc is null\n");
      return -EINVAL;
    }

  if (rstc->array)
    {
      return reset_control_array_assert(rstc_to_array(rstc));
    }

  if (rstc->shared)
    {
      if (atomic_read(&rstc->triggered_count) != 0)
        {
          return -EINVAL;
        }

      if (atomic_read(&rstc->deassert_count) == 0)
        {
          rsterr("deassert_count = 0, invalid value\n");
          return -EINVAL;
        }

      if (atomic_fetch_sub(&rstc->deassert_count, 1) != 1)
        {
          return 0;
        }

      /* Shared reset controls allow the reset line to be in any state
       * after this call, so doing nothing is a valid option.
       */

      if (!rstc->rcdev->ops->assert)
        {
          return -EBUSY;
        }
    }
  else
    {
      /* If the reset controller does not implement .assert(), there
       * is no way to guarantee that the reset line is asserted after
       * this call.
       */

      if (!rstc->rcdev->ops->assert)
        {
          return -ENOTSUP;
        }

      if (!rstc->acquired)
        {
           rsterr("reset %s (ID: %u) is not acquired\n",
               rstc->rcdev->name, rstc->id);
          return -EPERM;
        }
    }

#undef assert
  return rstc->rcdev->ops->assert(rstc->rcdev, rstc->id);
}

/****************************************************************************
 * Name: reset_control_deassert
 *
 * Description:
 *   Deasserts the reset line.
 *   After calling this function, the reset is guaranteed to be deasserted.
 *   Consumers must not use reset_control_reset on shared reset lines when
 *   reset_control_(de)assert has been used.
 *   return -EBUSY.
 *
 * Input Parameters:
 *   rstc - Reset controller
 *
 * Returned Value:
 *   Returns a negative errno if not supported, a positive value if the
 *   reset line is asserted.
 *
 ****************************************************************************/

int reset_control_deassert(FAR struct reset_control *rstc)
{
  rstinfo("Enter: reset_control_deassert\n");
  if (!rstc)
    {
      rsterr("rstc is null\n");
      return -EINVAL;
    }

  if (rstc->array)
    {
      return reset_control_array_deassert(rstc_to_array(rstc));
    }

  if (rstc->shared)
    {
      if (atomic_read(&rstc->triggered_count) != 0)
        {
          rsterr("triggered_count != 0, invalid value\n");
          return -EINVAL;
        }

      if (atomic_fetch_add(&rstc->deassert_count, 1) != 0)
        {
          return 0;
        }
    }
  else
    {
      if (!rstc->acquired)
        {
          rsterr("reset %s (ID: %u) is not acquired\n",
               rstc->rcdev->name, rstc->id);
          return -EPERM;
        }
    }

  /* If the reset controller does not implement .deassert(), we assume
   * that it handles self-deasserting reset lines via .reset(). In that
   * case, the reset lines are deasserted by default. If that is not the
   * case, the reset controller driver should implement .deassert() and
   * return -ENOTSUP.
   */

  if (!rstc->rcdev->ops->deassert)
    {
      return -ESRCH ;
    }

  return rstc->rcdev->ops->deassert(rstc->rcdev, rstc->id);
}

/****************************************************************************
 * Name: reset_control_status
 *
 * Description:
 *   Get the reset line status.
 *
 * Input Parameters:
 *   rstc - Reset controller
 *
 * Returned Value:
 *   Returns a negative errno if not supported, a positive value if the
 *   reset line is asserted.
 ****************************************************************************/

int reset_control_status(FAR struct reset_control *rstc)
{
  if (rstc == NULL)
    {
      return -EINVAL;
    }

  if (rstc->array)
    {
      return -EINVAL;
    }

  if (rstc->rcdev->ops->status)
    {
      return rstc->rcdev->ops->status(rstc->rcdev, rstc->id);
    }

  return -ENOTSUP;
}

/****************************************************************************
 * Name: reset_control_acquire()
 *
 * Description:
 *   Acquires a reset control for exclusive use.
 *   This is used to explicitly acquire a reset control for exclusive use.
 *   Note that exclusive resets are requested as acquired by default.
 *   In order for a second consumer to be able to control the reset, the
 *   first consumer has to release it first.
 *   Typically the easiest way to achieve this is to call the
 *   reset_control_get_exclusive_released() to obtain an instance of the
 *   reset control. Such reset controls are not acquired by default.
 *
 *   Consumers implementing shared access to an exclusive reset need to
 *   follow a specific protocol in order to work together. Before consumers
 *   can change a reset they must acquire exclusive access using
 *   reset_control_acquire().
 *   After they are done operating the reset, they must release exclusive
 *   access with a call to reset_control_release(). Consumers are not
 *   granted exclusive access to the reset as long as another consumer
 *   hasn't released a reset.
 *   See also: reset_control_release()
 *
 * Input Parameters:
 *   rstc - Reset control
 *
 * Returned Value:
 *   Returns a negative errno if not supported, a positive value if the
 *   reset line is asserted.
 ****************************************************************************/

int reset_control_acquire(FAR struct reset_control *rstc)
{
  FAR struct reset_control *rc;

  if (rstc == NULL)
    {
      return 0;
    }

  if (rstc->array)
    {
      return reset_control_array_acquire(rstc_to_array(rstc));
    }

  nxmutex_lock(&g_reset_list_mutex);

  if (rstc->acquired)
    {
      nxmutex_unlock(&g_reset_list_mutex);
      return 0;
    }

  list_for_every_entry(&rstc->rcdev->reset_control_head, rc,
                        struct reset_control, list)
    {
      if (rstc != rc && rstc->id == rc->id)
        {
          if (rc->acquired)
            {
              nxmutex_unlock(&g_reset_list_mutex);
              return -EBUSY;
            }
        }
    }

  rstc->acquired = true;

  nxmutex_unlock(&g_reset_list_mutex);
  return 0;
}

/****************************************************************************
 * Name: reset_control_release()
 *
 * Description:
 *   Releases exclusive access to a reset control.
 *
 *   Releases exclusive access right to a reset control previously obtained
 *   by a call to reset_control_acquire(). Until a consumer calls this
 *   function, no other consumers will be granted exclusive access.
 *
 *   See also: reset_control_acquire()
 *
 * Input Parameters:
 *   rstc - Reset control
 ****************************************************************************/

void reset_control_release(FAR struct reset_control *rstc)
{
  if (rstc == NULL)
    {
      return;
    }

  if (rstc->array)
    {
      reset_control_array_release(rstc_to_array(rstc));
    }
  else
    {
      rstc->acquired = false;
    }
}

/****************************************************************************
 * Name: reset_control_put
 *
 * Description:
 *   Free the reset control.
 *
 * Input Parameters:
 *   rstc - Reset controller
 ****************************************************************************/

void reset_control_put(FAR struct reset_control *rstc)
{
  if (!rstc)
    {
      return;
    }

  if (rstc->array)
    {
      reset_control_array_put(rstc_to_array(rstc));
      return;
    }

  nxmutex_lock(&g_reset_list_mutex);
  reset_control_put_internal(rstc);
  nxmutex_unlock(&g_reset_list_mutex);
}

/****************************************************************************
 * Name: reset_control_array_get
 *
 * Description:
 *   Get a list of reset controls using device node.
 *
 * Input Parameters:
 *   name     - The reset controller name
 *   shared   - Whether reset controls are shared or not
 *   acquired - Only one reset control may be acquired for a given controller
 *   and ID
 *
 * Returned Value:
 *   Returns pointer to allocated reset_control on success or error on
 *   failure
 ****************************************************************************/

FAR struct reset_control *
reset_control_array_get(FAR const char *name, const int id[],
                        const unsigned int num, bool shared, bool acquired)
{
  FAR struct reset_control_array *resets;
  FAR struct reset_control *rstc;
  unsigned int i;

  resets = kmm_zalloc(sizeof(struct reset_control_array) +
                      sizeof(struct reset_control) * num);
  if (!resets)
    {
      return NULL;
    }

  for (i = 0; i < num; i++)
    {
      rstc = reset_control_get(name, id[i], shared, acquired);
      if (rstc == NULL)
        {
          goto err_rst;
        }

      resets->rstc[i] = rstc;
    }

  resets->num_rstcs = num;
  resets->base.array = true;

  return &resets->base;

err_rst:
  nxmutex_lock(&g_reset_list_mutex);
  while (i--)
    {
      reset_control_put_internal(resets->rstc[i]);
    }

  nxmutex_unlock(&g_reset_list_mutex);
  kmm_free(resets);

  return rstc;
}

/****************************************************************************
 * Name: reset_control_device_reset
 *
 * Description:
 *   Find reset controller associated with the device and perform reset,
 *   finally free the reset control.
 *   Convenience wrapper for reset_control_get() and reset_control_reset().
 *   This is useful for the common case of devices with single, dedicated
 *   reset lines.
 *
 * Input Parameters:
 *   name - The controller name
 *
 * Returned Value:
 *   Returns a negative errno if failed, otherwise success.
 ****************************************************************************/

int reset_control_device_reset(FAR const char *name)
{
  FAR struct reset_control *rstc;
  int ret;

  rstc = reset_control_get(name, 0, false, true);
  if (!rstc)
    {
      rsterr("get rstc failed.\n");
      return -EINVAL;
    }

  ret = reset_control_reset(rstc);
  reset_control_put(rstc);

  return ret;
}

/****************************************************************************
 * Name: reset_controller_register
 *
 * Description:
 *   Register a reset controller device
 *
 * Input Parameters:
 *   rcdev - A pointer to the initialized reset controller device
 *
 * Returned Value:
 *   Returns 0 if success, otherwise failed.
 ****************************************************************************/

int reset_controller_register(FAR struct reset_controller_dev *rcdev)
{
  if (rcdev == NULL || rcdev->name == NULL || rcdev->ops == NULL)
    {
      rsterr("rcdev is null\n");
      return -EINVAL;
    }

  list_initialize(&rcdev->reset_control_head);

  nxmutex_lock(&g_reset_list_mutex);

#ifdef CONFIG_RESET_PROCFS
  /* procfs_register() wants to run before procfs is mounted, which holds:
   * controllers register during board or architecture start up.  It
   * appends without checking for a duplicate, so the entry is claimed once
   * for the lifetime of the system rather than whenever the list is empty.
   */

  if (!g_reset_procfs_added)
    {
      procfs_register(&g_reset_procfs);
      g_reset_procfs_added = true;
    }
#endif

  list_add_after(&g_reset_controller_list, &rcdev->list);
  nxmutex_unlock(&g_reset_list_mutex);

  return 0;
}

/****************************************************************************
 * Name: reset_controller_unregister
 *
 * Description:
 *   Unregister a reset controller device
 *
 * Input Parameters:
 *   rcdev - A pointer to the reset controller device
 ****************************************************************************/

void reset_controller_unregister(FAR struct reset_controller_dev *rcdev)
{
  nxmutex_lock(&g_reset_list_mutex);
  list_delete(&rcdev->list);
  nxmutex_unlock(&g_reset_list_mutex);
}

