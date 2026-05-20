/****************************************************************************
 * drivers/timers/pulsecount.c
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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/timers/pulsecount.h>

#ifdef CONFIG_PULSECOUNT

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pulsecount_upperhalf_s
{
  uint8_t crefs;                    /* The number of times the device has
                                      * been opened */
  volatile bool started;            /* True: pulse output is active */
  volatile bool waiting;            /* True: a thread is waiting for expiry */
  mutex_t lock;                     /* Supports mutual exclusion */
  sem_t waitsem;                    /* Waits for finite pulse completion */
  struct pulsecount_info_s info;    /* Pulse output characteristics */

  /* Lower half state */

  FAR struct pulsecount_lowerhalf_s *dev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pulsecount_open(FAR struct file *filep);
static int pulsecount_close(FAR struct file *filep);
static ssize_t pulsecount_read(FAR struct file *filep,
                               FAR char *buffer, size_t buflen);
static ssize_t pulsecount_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen);
static int pulsecount_start(FAR struct pulsecount_upperhalf_s *upper,
                            unsigned int oflags);
static int pulsecount_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pulsecount_fops =
{
  pulsecount_open,   /* open */
  pulsecount_close,  /* close */
  pulsecount_read,   /* read */
  pulsecount_write,  /* write */
  NULL,              /* seek */
  pulsecount_ioctl,  /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void pulsecount_dump(FAR const char *msg,
                            FAR const struct pulsecount_info_s *info,
                            bool started)
{
  _info("%s: high: %" PRIu32 " ns low: %" PRIu32
          " ns count: %" PRIu32 " started: %d\n",
          msg, info->high_ns, info->low_ns, info->count, started);
}

static int pulsecount_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct pulsecount_upperhalf_s *upper = inode->i_private;
  FAR struct pulsecount_lowerhalf_s *lower;
  uint8_t tmp;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      ret = -EMFILE;
      goto errout_with_lock;
    }

  if (tmp == 1)
    {
      lower = upper->dev;
      DEBUGASSERT(lower->ops->setup != NULL);

      ret = lower->ops->setup(lower);
      if (ret < 0)
        {
          goto errout_with_lock;
        }
    }

  upper->crefs = tmp;
  ret = OK;

errout_with_lock:
  nxmutex_unlock(&upper->lock);
  return ret;
}

static int pulsecount_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct pulsecount_upperhalf_s *upper = inode->i_private;
  FAR struct pulsecount_lowerhalf_s *lower;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (upper->crefs > 1)
    {
      upper->crefs--;
    }
  else
    {
      lower = upper->dev;
      upper->crefs = 0;

      DEBUGASSERT(lower->ops->shutdown != NULL);
      lower->ops->shutdown(lower);
    }

  nxmutex_unlock(&upper->lock);
  return OK;
}

static ssize_t pulsecount_read(FAR struct file *filep,
                               FAR char *buffer, size_t buflen)
{
  return 0;
}

static ssize_t pulsecount_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen)
{
  return 0;
}

static int pulsecount_start(FAR struct pulsecount_upperhalf_s *upper,
                            unsigned int oflags)
{
  FAR struct pulsecount_lowerhalf_s *lower;
  int ret = OK;

  DEBUGASSERT(upper != NULL);
  lower = upper->dev;
  DEBUGASSERT(lower != NULL && lower->ops->start != NULL);

  if (upper->info.count == 0)
    {
      return -EINVAL;
    }

  if (!upper->started)
    {
      upper->waiting = (oflags & O_NONBLOCK) == 0;
      upper->started = true;

      ret = lower->ops->start(lower, &upper->info, upper);
      if (ret == OK)
        {
          while (upper->waiting)
            {
              ret = nxsem_wait_uninterruptible(&upper->waitsem);
              if (ret < 0)
                {
                  upper->started = false;
                  upper->waiting = false;
                }
            }
        }
      else
        {
          upper->started = false;
          upper->waiting = false;
        }
    }

  return ret;
}

static int pulsecount_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct pulsecount_upperhalf_s *upper = inode->i_private;
  FAR struct pulsecount_lowerhalf_s *lower = upper->dev;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case PULSECOUNTIOC_SETCHARACTERISTICS:
        {
          FAR const struct pulsecount_info_s *info =
            (FAR const struct pulsecount_info_s *)((uintptr_t)arg);
          DEBUGASSERT(info != NULL);

          if (info->high_ns == 0 || info->low_ns == 0 ||
              pulsecount_frequency(info) == 0 || info->count == 0)
            {
              ret = -EINVAL;
              break;
            }

          pulsecount_dump("PULSECOUNTIOC_SETCHARACTERISTICS", info,
                          upper->started);
          memcpy(&upper->info, info, sizeof(struct pulsecount_info_s));

          if (upper->started)
            {
              ret = lower->ops->start(lower, &upper->info, upper);
            }
        }
        break;

      case PULSECOUNTIOC_GETCHARACTERISTICS:
        {
          FAR struct pulsecount_info_s *info =
            (FAR struct pulsecount_info_s *)((uintptr_t)arg);
          DEBUGASSERT(info != NULL);

          memcpy(info, &upper->info, sizeof(struct pulsecount_info_s));
          pulsecount_dump("PULSECOUNTIOC_GETCHARACTERISTICS", info,
                          upper->started);
        }
        break;

      case PULSECOUNTIOC_START:
        {
          pulsecount_dump("PULSECOUNTIOC_START", &upper->info,
                          upper->started);
          ret = pulsecount_start(upper, filep->f_oflags);
        }
        break;

      case PULSECOUNTIOC_STOP:
        {
          if (upper->started)
            {
              ret = lower->ops->stop(lower);
              upper->started = false;
              upper->waiting = false;
            }
        }
        break;

      default:
        {
          DEBUGASSERT(lower->ops->ioctl != NULL);
          ret = lower->ops->ioctl(lower, cmd, arg);
        }
        break;
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pulsecount_register(FAR const char *path,
                        FAR struct pulsecount_lowerhalf_s *dev)
{
  FAR struct pulsecount_upperhalf_s *upper;

  upper = kmm_zalloc(sizeof(struct pulsecount_upperhalf_s));
  if (upper == NULL)
    {
      _err("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  nxmutex_init(&upper->lock);
  nxsem_init(&upper->waitsem, 0, 0);

  upper->dev = dev;

  _info("Registering %s\n", path);
  return register_driver(path, &g_pulsecount_fops, 0666, upper);
}

void pulsecount_expired(FAR void *handle)
{
  FAR struct pulsecount_upperhalf_s *upper =
    (FAR struct pulsecount_upperhalf_s *)handle;

  if (upper->started)
    {
      if (upper->waiting)
        {
          upper->waiting = false;
          nxsem_post(&upper->waitsem);
        }

      upper->started = false;
    }
}

#endif /* CONFIG_PULSECOUNT */
