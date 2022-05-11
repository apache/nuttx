/****************************************************************************
 * drivers/sensors/sensor.c
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <poll.h>
#include <fcntl.h>
#include <nuttx/list.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/circbuf.h>
#include <nuttx/mutex.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device naming ************************************************************/

#define ROUND_DOWN(x, y)    (((x) / (y)) * (y))
#define DEVNAME_FMT         "/dev/sensor/sensor_%s%s%d"
#define DEVNAME_UNCAL       "_uncal"
#define TIMING_BUF_ESIZE    (sizeof(unsigned long))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes sensor info */

struct sensor_info_s
{
  unsigned long esize;
  FAR char *name;
};

/* This structure describes user info of sensor, the user may be
 * advertiser or subscriber
 */

struct sensor_user_s
{
  /* The common info */

  struct list_node node;       /* Node of users list */
  struct pollfd   *fds;        /* The poll structure of thread waiting events */
  bool             changed;    /* This is used to indicate event happens and need to
                                * asynchronous notify other users
                                */
  sem_t            buffersem;  /* Wakeup user waiting for data in circular buffer */
  size_t           bufferpos;  /* The index of user generation in buffer */

  /* The subscriber info
   * Support multi advertisers to subscribe their own data when they
   * appear in dual role
   */

  unsigned long    generation; /* Last generation subscriber has seen */
  unsigned long    interval;   /* The interval for subscriber */
  unsigned long    latency;    /* The bactch latency for subscriber */
};

/* This structure describes the state of the upper half driver */

struct sensor_upperhalf_s
{
  FAR struct sensor_lowerhalf_s *lower;  /* The handle of lower half driver */
  struct sensor_state_s          state;  /* The state of sensor device */
  struct circbuf_s   timing;             /* The circular buffer of generation */
  struct circbuf_s   buffer;             /* The circular buffer of data */
  rmutex_t           lock;               /* Manages exclusive access to file operations */
  struct list_node   userlist;           /* List of users */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    sensor_pollnotify(FAR struct sensor_upperhalf_s *upper,
                                 pollevent_t eventset);
static int     sensor_open(FAR struct file *filep);
static int     sensor_close(FAR struct file *filep);
static ssize_t sensor_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t sensor_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     sensor_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
static int     sensor_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup);
static ssize_t sensor_push_event(FAR void *priv, FAR const void *data,
                                 size_t bytes);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_info_s g_sensor_info[] =
{
  {0,                                     NULL},
  {sizeof(struct sensor_accel),           "accel"},
  {sizeof(struct sensor_mag),             "mag"},
  {sizeof(struct sensor_gyro),            "gyro"},
  {sizeof(struct sensor_light),           "light"},
  {sizeof(struct sensor_baro),            "baro"},
  {sizeof(struct sensor_prox),            "prox"},
  {sizeof(struct sensor_humi),            "humi"},
  {sizeof(struct sensor_temp),            "temp"},
  {sizeof(struct sensor_rgb),             "rgb"},
  {sizeof(struct sensor_hall),            "hall"},
  {sizeof(struct sensor_ir),              "ir"},
  {sizeof(struct sensor_gps),             "gps"},
  {sizeof(struct sensor_uv),              "uv"},
  {sizeof(struct sensor_noise),           "noise"},
  {sizeof(struct sensor_pm25),            "pm25"},
  {sizeof(struct sensor_pm1p0),           "pm1p0"},
  {sizeof(struct sensor_pm10),            "pm10"},
  {sizeof(struct sensor_co2),             "co2"},
  {sizeof(struct sensor_hcho),            "hcho"},
  {sizeof(struct sensor_tvoc),            "tvoc"},
  {sizeof(struct sensor_ph),              "ph"},
  {sizeof(struct sensor_dust),            "dust"},
  {sizeof(struct sensor_hrate),           "hrate"},
  {sizeof(struct sensor_hbeat),           "hbeat"},
  {sizeof(struct sensor_ecg),             "ecg"},
  {sizeof(struct sensor_ppgd),            "ppgd"},
  {sizeof(struct sensor_ppgq),            "ppgq"},
  {sizeof(struct sensor_impd),            "impd"},
  {sizeof(struct sensor_ots),             "ots"},
  {sizeof(struct sensor_gps_satellite),   "gps_satellite"},
  {sizeof(struct sensor_wake_gesture),    "wake_gesture"},
  {sizeof(struct sensor_cap),             "cap"},
};

static const struct file_operations g_sensor_fops =
{
  sensor_open,    /* open  */
  sensor_close,   /* close */
  sensor_read,    /* read  */
  sensor_write,   /* write */
  NULL,           /* seek  */
  sensor_ioctl,   /* ioctl */
  sensor_poll     /* poll  */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sensor_lock(FAR void *priv)
{
  FAR struct sensor_upperhalf_s *upper = priv;
  nxrmutex_lock(&upper->lock);
}

static void sensor_unlock(FAR void *priv)
{
  FAR struct sensor_upperhalf_s *upper = priv;
  nxrmutex_unlock(&upper->lock);
}

static int sensor_update_interval(FAR struct file *filep,
                                  FAR struct sensor_upperhalf_s *upper,
                                  FAR struct sensor_user_s *user,
                                  unsigned long interval)
{
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *tmp;
  unsigned long min_interval = interval;
  unsigned long min_latency = interval != ULONG_MAX ?
                              user->latency : ULONG_MAX;
  int ret = 0;

  if (interval == user->interval)
    {
      return 0;
    }

  list_for_every_entry(&upper->userlist, tmp, struct sensor_user_s, node)
    {
      if (tmp == user || tmp->interval == ULONG_MAX)
        {
          continue;
        }

      if (min_interval > tmp->interval)
        {
          min_interval = tmp->interval;
        }

      if (min_latency > tmp->latency)
        {
          min_latency = tmp->latency;
        }
    }

  if (lower->ops->set_interval)
    {
      if (min_interval != ULONG_MAX &&
          min_interval != upper->state.min_interval)
        {
          unsigned long expected_interval = min_interval;
          ret = lower->ops->set_interval(lower, filep, &min_interval);
          if (ret < 0)
            {
              return ret;
            }
          else if (min_interval > expected_interval)
            {
              return -EINVAL;
            }
        }

      if (min_latency == ULONG_MAX)
        {
          min_latency = 0;
        }

      if (lower->ops->batch &&
          (min_latency != upper->state.min_latency ||
          (min_interval != upper->state.min_interval && min_latency)))
        {
          ret = lower->ops->batch(lower, filep, &min_latency);
          if (ret >= 0)
            {
              upper->state.min_latency = min_latency;
            }
        }
    }

  upper->state.min_interval = min_interval;
  user->interval = interval;
  sensor_pollnotify(upper, POLLPRI);
  return ret;
}

static int sensor_update_latency(FAR struct file *filep,
                                 FAR struct sensor_upperhalf_s *upper,
                                 FAR struct sensor_user_s *user,
                                 unsigned long latency)
{
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *tmp;
  unsigned long min_latency = latency;
  int ret = 0;

  if (latency == user->latency)
    {
      return 0;
    }

  if (user->interval == ULONG_MAX)
    {
      user->latency = latency;
      return 0;
    }

  if (latency <= upper->state.min_latency)
    {
      goto update;
    }

  list_for_every_entry(&upper->userlist, tmp, struct sensor_user_s, node)
    {
      if (tmp == user || tmp->interval == ULONG_MAX)
        {
          continue;
        }

      if (min_latency > tmp->latency)
        {
          min_latency = tmp->latency;
        }
    }

update:
  if (min_latency == ULONG_MAX)
    {
      min_latency = 0;
    }

  if (min_latency == upper->state.min_latency)
    {
      user->latency = latency;
      return ret;
    }

  if (lower->ops->batch)
    {
      ret = lower->ops->batch(lower, filep, &min_latency);
      if (ret < 0)
        {
          return ret;
        }
    }

  upper->state.min_latency = min_latency;
  user->latency = latency;
  sensor_pollnotify(upper, POLLPRI);
  return ret;
}

static void sensor_generate_timing(FAR struct sensor_upperhalf_s *upper,
                                   unsigned long nums)
{
  unsigned long interval = upper->state.min_interval != ULONG_MAX ?
                           upper->state.min_interval : 1;
  while (nums-- > 0)
    {
      upper->state.generation += interval;
      circbuf_overwrite(&upper->timing, &upper->state.generation,
                        TIMING_BUF_ESIZE);
    }
}

static bool sensor_is_updated(FAR struct sensor_upperhalf_s *upper,
                              FAR struct sensor_user_s *user)
{
  long delta = upper->state.generation - user->generation;

  if (delta <= 0)
    {
      return false;
    }
  else if (user->interval == ULONG_MAX)
    {
      return true;
    }
  else
    {
      /* Check whether next generation user want in buffer.
       * generation     next generation(not published yet)
       * ____v_____________v
       * ////|//////^      |
       *         ^ middle point
       *   next generation user want
       */

      return delta >= user->interval - (upper->state.min_interval >> 1);
    }
}

static void sensor_catch_up(FAR struct sensor_upperhalf_s *upper,
                            FAR struct sensor_user_s *user)
{
  unsigned long generation;
  long delta;

  circbuf_peek(&upper->timing, &generation, TIMING_BUF_ESIZE);
  delta = generation - user->generation;
  if (delta > 0)
    {
      user->bufferpos = upper->timing.tail / TIMING_BUF_ESIZE;
      if (user->interval == ULONG_MAX)
        {
          user->generation = generation - 1;
        }
      else
        {
          delta -= upper->state.min_interval >> 1;
          user->generation += ROUND_DOWN(delta, user->interval);
        }
    }
}

static ssize_t sensor_do_samples(FAR struct sensor_upperhalf_s *upper,
                                 FAR struct sensor_user_s *user,
                                 FAR char *buffer, size_t len)
{
  unsigned long generation;
  ssize_t ret = 0;
  size_t nums;
  size_t pos;
  size_t end;

  sensor_catch_up(upper, user);
  nums = upper->timing.head / TIMING_BUF_ESIZE - user->bufferpos;
  if (len < nums * upper->state.esize)
    {
      nums = len / upper->state.esize;
    }

  len = nums * upper->state.esize;

  /* Take samples continuously */

  if (user->interval == ULONG_MAX)
    {
      ret = circbuf_peekat(&upper->buffer,
                           user->bufferpos * upper->state.esize,
                           buffer, len);
      user->bufferpos += nums;
      circbuf_peekat(&upper->timing,
                     (user->bufferpos - 1) * TIMING_BUF_ESIZE,
                     &user->generation, TIMING_BUF_ESIZE);
      return ret;
    }

  /* Take samples one-bye-one, to determine whether a sample needed:
   *
   * If user's next generation is on the left side of middle point,
   * we should copy this sample for user.
   *                      next_generation(or end)
   *                ________________v____
   * timing buffer: //|//////.      |
   *                  ^   middle
   *              generation
   *                        next sample(or end)
   *                ________________v____
   *  data  buffer:   |             |
   *                  ^
   *                sample
   */

  pos = user->bufferpos;
  end = upper->timing.head / TIMING_BUF_ESIZE;
  circbuf_peekat(&upper->timing, pos * TIMING_BUF_ESIZE,
                 &generation, TIMING_BUF_ESIZE);
  while (pos++ != end)
    {
      unsigned long next_generation;
      long delta;

      if (pos * TIMING_BUF_ESIZE == upper->timing.head)
        {
          next_generation = upper->state.generation +
                            upper->state.min_interval;
        }
      else
        {
          circbuf_peekat(&upper->timing, pos * TIMING_BUF_ESIZE,
                         &next_generation, TIMING_BUF_ESIZE);
        }

      delta = next_generation + generation -
              ((user->generation + user->interval) << 1);
      if (delta >= 0)
        {
          ret += circbuf_peekat(&upper->buffer,
                                (pos - 1) * upper->state.esize,
                                buffer + ret, upper->state.esize);
          user->bufferpos = pos;
          user->generation += user->interval;
          if (ret >= len)
            {
              break;
            }
        }

      generation = next_generation;
    }

  return ret;
}

static void sensor_pollnotify_one(FAR struct sensor_user_s *user,
                                  pollevent_t eventset)
{
  int semcount;

  if (eventset == POLLPRI)
    {
      user->changed = true;
    }

  if (!user->fds)
    {
      return;
    }

  user->fds->revents |= (user->fds->events & eventset);
  if (user->fds->revents != 0)
    {
      sninfo("Report events: %08" PRIx32 "\n", user->fds->revents);
      nxsem_get_value(user->fds->sem, &semcount);
      if (semcount < 1)
        {
          nxsem_post(user->fds->sem);
        }
    }
}

static void sensor_pollnotify(FAR struct sensor_upperhalf_s *upper,
                              pollevent_t eventset)
{
  FAR struct sensor_user_s *user;

  list_for_every_entry(&upper->userlist, user, struct sensor_user_s, node)
    {
      sensor_pollnotify_one(user, eventset);
    }
}

static int sensor_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *user;
  int ret = 0;

  nxrmutex_lock(&upper->lock);
  user = kmm_zalloc(sizeof(struct sensor_user_s));
  if (user == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_sem;
    }

  if (lower->ops->open)
    {
      ret = lower->ops->open(lower, filep);
      if (ret < 0)
        {
          goto errout_with_user;
        }
    }

  if (filep->f_oflags & O_RDOK)
    {
      if (upper->state.nsubscribers == 0 && lower->ops->activate)
        {
          ret = lower->ops->activate(lower, filep, true);
          if (ret < 0)
            {
              goto errout_with_open;
            }
        }

      upper->state.nsubscribers++;
    }

  if (filep->f_oflags & O_WROK)
    {
      upper->state.nadvertisers++;
      if (filep->f_oflags & SENSOR_PERSIST)
        {
          lower->persist = true;
        }
    }

  if (upper->state.generation && lower->persist)
    {
      user->generation = upper->state.generation - 1;
      user->bufferpos  = upper->timing.head / TIMING_BUF_ESIZE - 1;
    }
  else
    {
      user->generation = upper->state.generation;
      user->bufferpos  = upper->timing.head / TIMING_BUF_ESIZE;
    }

  user->interval = ULONG_MAX;
  nxsem_init(&user->buffersem, 0, 0);
  nxsem_set_protocol(&user->buffersem, SEM_PRIO_NONE);
  list_add_tail(&upper->userlist, &user->node);

  /* The new user generation, notify to other users */

  sensor_pollnotify(upper, POLLPRI);

  filep->f_priv = user;
  goto errout_with_sem;

errout_with_open:
  if (lower->ops->close)
    {
      lower->ops->close(lower, filep);
    }

errout_with_user:
  kmm_free(user);
errout_with_sem:
  nxrmutex_unlock(&upper->lock);
  return ret;
}

static int sensor_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *user = filep->f_priv;
  int ret = 0;

  nxrmutex_lock(&upper->lock);
  if (lower->ops->close)
    {
      ret = lower->ops->close(lower, filep);
      if (ret < 0)
        {
          nxrmutex_unlock(&upper->lock);
          return ret;
        }
    }

  if (filep->f_oflags & O_RDOK)
    {
      upper->state.nsubscribers--;
      if (upper->state.nsubscribers == 0 && lower->ops->activate)
        {
          lower->ops->activate(lower, filep, false);
        }
    }

  if (filep->f_oflags & O_WROK)
    {
      upper->state.nadvertisers--;
    }

  list_delete(&user->node);
  sensor_update_latency(filep, upper, user, ULONG_MAX);
  sensor_update_interval(filep, upper, user, ULONG_MAX);
  nxsem_destroy(&user->buffersem);

  /* The user is closed, notify to other users */

  sensor_pollnotify(upper, POLLPRI);
  nxrmutex_unlock(&upper->lock);

  kmm_free(user);
  return ret;
}

static ssize_t sensor_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *user = filep->f_priv;
  ssize_t ret;

  if (!buffer || !len)
    {
      return -EINVAL;
    }

  nxrmutex_lock(&upper->lock);
  if (lower->ops->fetch)
    {
      if (!(filep->f_oflags & O_NONBLOCK))
        {
          nxrmutex_unlock(&upper->lock);
          ret = nxsem_wait_uninterruptible(&user->buffersem);
          if (ret < 0)
            {
              return ret;
            }

          nxrmutex_lock(&upper->lock);
        }
      else if (!upper->state.nsubscribers)
        {
          ret = -EAGAIN;
          goto out;
        }

        ret = lower->ops->fetch(lower, filep, buffer, len);
    }
  else if (circbuf_is_empty(&upper->buffer))
    {
      ret = -ENODATA;
    }
  else if (sensor_is_updated(upper, user))
    {
      ret = sensor_do_samples(upper, user, buffer, len);
    }
  else if (lower->persist)
    {
      /* Persistent device can get latest old data if not updated. */

      ret = circbuf_peekat(&upper->buffer,
                           (user->bufferpos - 1) * upper->state.esize,
                           buffer, upper->state.esize);
    }
  else
    {
      ret = -ENODATA;
    }

out:
  nxrmutex_unlock(&upper->lock);
  return ret;
}

static ssize_t sensor_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;

  return lower->push_event(lower->priv, buffer, buflen);
}

static int sensor_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *user = filep->f_priv;
  int ret = 0;

  sninfo("cmd=%x arg=%08lx\n", cmd, arg);

  switch (cmd)
    {
      case SNIOC_GET_STATE:
        {
          nxrmutex_lock(&upper->lock);
          memcpy((FAR void *)(uintptr_t)arg,
                 &upper->state, sizeof(upper->state));
          user->changed = false;
          nxrmutex_unlock(&upper->lock);
        }
        break;

      case SNIOC_SET_INTERVAL:
        {
          nxrmutex_lock(&upper->lock);
          ret = sensor_update_interval(filep, upper, user,
                                       arg ? arg : ULONG_MAX);
          nxrmutex_unlock(&upper->lock);
        }
        break;

      case SNIOC_BATCH:
        {
          nxrmutex_lock(&upper->lock);
          ret = sensor_update_latency(filep, upper, user, arg);
          nxrmutex_unlock(&upper->lock);
        }
        break;

      case SNIOC_SELFTEST:
        {
          if (lower->ops->selftest == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          ret = lower->ops->selftest(lower, filep, arg);
        }
        break;

      case SNIOC_SET_CALIBVALUE:
        {
          if (lower->ops->set_calibvalue == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          ret = lower->ops->set_calibvalue(lower, filep, arg);
        }
        break;

      case SNIOC_CALIBRATE:
        {
          if (lower->ops->calibrate == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          ret = lower->ops->calibrate(lower, filep, arg);
        }
        break;

      case SNIOC_SET_USERPRIV:
        {
          nxrmutex_lock(&upper->lock);
          upper->state.priv = (FAR void *)(uintptr_t)arg;
          nxrmutex_unlock(&upper->lock);
        }
        break;

      case SNIOC_SET_BUFFER_NUMBER:
        {
          nxrmutex_lock(&upper->lock);
          if (!circbuf_is_init(&upper->buffer))
            {
              if (arg >= lower->nbuffer)
                {
                  lower->nbuffer = arg;
                }
              else
                {
                  ret = -ERANGE;
                }
            }
          else
            {
              ret = -EBUSY;
            }

          nxrmutex_unlock(&upper->lock);
        }
        break;

      case SNIOC_UPDATED:
        {
          nxrmutex_lock(&upper->lock);
          *(FAR bool *)(uintptr_t)arg = sensor_is_updated(upper, user);
          nxrmutex_unlock(&upper->lock);
        }
        break;

      default:

        /* Lowerhalf driver process other cmd. */

        if (lower->ops->control)
          {
            ret = lower->ops->control(lower, filep, cmd, arg);
          }
        else
          {
            ret = -ENOTTY;
          }

        break;
    }

  return ret;
}

static int sensor_poll(FAR struct file *filep,
                       FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *user = filep->f_priv;
  pollevent_t eventset = 0;
  int semcount;
  int ret = 0;

  nxrmutex_lock(&upper->lock);
  if (setup)
    {
      /* Don't have enough space to store fds */

      if (user->fds)
        {
          ret = -ENOSPC;
          goto errout;
        }

      user->fds = fds;
      fds->priv = filep;
      if (lower->ops->fetch)
        {
          /* Always return POLLIN for fetch data directly(non-block) */

          if (filep->f_oflags & O_NONBLOCK)
            {
              eventset |= (fds->events & POLLIN);
            }
          else
            {
              nxsem_get_value(&user->buffersem, &semcount);
              if (semcount > 0)
                {
                  eventset |= (fds->events & POLLIN);
                }
            }
        }
      else if (sensor_is_updated(upper, user))
        {
          eventset |= (fds->events & POLLIN);
        }

      if (user->changed)
        {
          eventset |= (fds->events & POLLPRI);
        }

      if (eventset)
        {
          sensor_pollnotify_one(user, eventset);
        }
    }
  else
    {
      user->fds = NULL;
      fds->priv = NULL;
    }

errout:
  nxrmutex_unlock(&upper->lock);
  return ret;
}

static ssize_t sensor_push_event(FAR void *priv, FAR const void *data,
                                 size_t bytes)
{
  FAR struct sensor_upperhalf_s *upper = priv;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *user;
  unsigned long envcount;
  int semcount;
  int ret;

  envcount = bytes / upper->state.esize;
  if (!envcount || bytes != envcount * upper->state.esize)
    {
      return -EINVAL;
    }

  nxrmutex_lock(&upper->lock);
  if (!circbuf_is_init(&upper->buffer))
    {
      /* Initialize sensor buffer when data is first generated */

      ret = circbuf_init(&upper->buffer, NULL, lower->nbuffer *
                         upper->state.esize);
      if (ret < 0)
        {
          nxrmutex_unlock(&upper->lock);
          return ret;
        }

      ret = circbuf_init(&upper->timing, NULL, lower->nbuffer *
                         TIMING_BUF_ESIZE);
      if (ret < 0)
        {
          circbuf_uninit(&upper->buffer);
          nxrmutex_unlock(&upper->lock);
          return ret;
        }
    }

  circbuf_overwrite(&upper->buffer, data, bytes);
  sensor_generate_timing(upper, envcount);
  list_for_every_entry(&upper->userlist, user, struct sensor_user_s, node)
    {
      if (sensor_is_updated(upper, user))
        {
          nxsem_get_value(&user->buffersem, &semcount);
          if (semcount < 1)
            {
              nxsem_post(&user->buffersem);
            }

          sensor_pollnotify_one(user, POLLIN);
        }
    }

  nxrmutex_unlock(&upper->lock);
  return bytes;
}

static void sensor_notify_event(FAR void *priv)
{
  FAR struct sensor_upperhalf_s *upper = priv;
  FAR struct sensor_user_s *user;
  int semcount;

  nxrmutex_lock(&upper->lock);
  list_for_every_entry(&upper->userlist, user, struct sensor_user_s, node)
    {
      nxsem_get_value(&user->buffersem, &semcount);
      if (semcount < 1)
        {
          nxsem_post(&user->buffersem);
        }

      sensor_pollnotify_one(user, POLLIN);
    }

  nxrmutex_unlock(&upper->lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sensor_register
 *
 * Description:
 *   This function binds an instance of a "lower half" Sensor driver with the
 *   "upper half" Sensor device and registers that device so that can be used
 *   by application code.
 *
 *   We will register the chararter device by node name format based on the
 *   type of sensor. Multiple types of the same type are distinguished by
 *   numbers. eg: accel0, accel1
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persists as long
 *           as the driver persists.
 *   devno - The user specifies which device of this type, from 0. If the
 *           devno alerady exists, -EEXIST will be returned.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int sensor_register(FAR struct sensor_lowerhalf_s *lower, int devno)
{
  char path[PATH_MAX];

  DEBUGASSERT(lower != NULL);

  snprintf(path, PATH_MAX, DEVNAME_FMT,
           g_sensor_info[lower->type].name,
           lower->uncalibrated ? DEVNAME_UNCAL : "",
           devno);
  return sensor_custom_register(lower, path,
                                g_sensor_info[lower->type].esize);
}

/****************************************************************************
 * Name: sensor_custom_register
 *
 * Description:
 *   This function binds an instance of a "lower half" Sensor driver with the
 *   "upper half" Sensor device and registers that device so that can be used
 *   by application code.
 *
 *   You can register the character device type by specific path and esize.
 *   This API corresponds to the sensor_custom_unregister.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persists as long
 *           as the driver persists.
 *   path  - The user specifies path of device. ex: /dev/sensor/xxx.
 *   esize - The element size of intermediate circular buffer.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int sensor_custom_register(FAR struct sensor_lowerhalf_s *lower,
                           FAR const char *path, unsigned long esize)
{
  FAR struct sensor_upperhalf_s *upper;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL);

  if (lower->type >= SENSOR_TYPE_COUNT || !esize)
    {
      snerr("ERROR: type is invalid\n");
      return ret;
    }

  /* Allocate the upper-half data structure */

  upper = kmm_zalloc(sizeof(struct sensor_upperhalf_s));
  if (!upper)
    {
      snerr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the upper-half data structure */

  list_initialize(&upper->userlist);
  upper->state.esize = esize;
  upper->state.min_interval = ULONG_MAX;
  if (lower->ops->activate)
    {
      upper->state.nadvertisers = 1;
    }

  nxrmutex_init(&upper->lock);

  /* Bind the lower half data structure member */

  lower->priv          = upper;
  lower->sensor_lock   = sensor_lock;
  lower->sensor_unlock = sensor_unlock;

  if (!lower->ops->fetch)
    {
      if (!lower->nbuffer)
        {
          lower->nbuffer = 1;
        }

      lower->push_event = sensor_push_event;
    }
  else
    {
      lower->notify_event = sensor_notify_event;
      lower->nbuffer = 0;
    }

#ifdef CONFIG_SENSORS_RPMSG
  lower = sensor_rpmsg_register(lower, path);
  if (lower == NULL)
    {
      ret = -EIO;
      goto drv_err;
    }
#endif

  upper->state.nbuffer = lower->nbuffer;
  upper->lower = lower;
  sninfo("Registering %s\n", path);
  ret = register_driver(path, &g_sensor_fops, 0666, upper);
  if (ret)
    {
      goto drv_err;
    }

  return ret;

drv_err:
  nxrmutex_destroy(&upper->lock);

  kmm_free(upper);

  return ret;
}

/****************************************************************************
 * Name: sensor_unregister
 *
 * Description:
 *   This function unregister character node and release all resource about
 *   upper half driver.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persists as long
 *           as the driver persists.
 *   devno - The user specifies which device of this type, from 0.
 ****************************************************************************/

void sensor_unregister(FAR struct sensor_lowerhalf_s *lower, int devno)
{
  char path[PATH_MAX];

  snprintf(path, PATH_MAX, DEVNAME_FMT,
           g_sensor_info[lower->type].name,
           lower->uncalibrated ? DEVNAME_UNCAL : "",
           devno);
  sensor_custom_unregister(lower, path);
}

/****************************************************************************
 * Name: sensor_custom_unregister
 *
 * Description:
 *   This function unregister character node and release all resource about
 *   upper half driver. This API corresponds to the sensor_custom_register.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persists as long
 *           as the driver persists.
 *   path  - The user specifies path of device, ex: /dev/sensor/xxx
 ****************************************************************************/

void sensor_custom_unregister(FAR struct sensor_lowerhalf_s *lower,
                              FAR const char *path)
{
  FAR struct sensor_upperhalf_s *upper;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(lower->priv != NULL);

  upper = lower->priv;

  sninfo("UnRegistering %s\n", path);
  unregister_driver(path);

#ifdef CONFIG_SENSORS_RPMSG
  sensor_rpmsg_unregister(lower);
#endif

  nxrmutex_destroy(&upper->lock);
  if (circbuf_is_init(&upper->buffer))
    {
      circbuf_uninit(&upper->buffer);
      circbuf_uninit(&upper->timing);
    }

  kmm_free(upper);
}
