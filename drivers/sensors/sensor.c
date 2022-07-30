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
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device naming ************************************************************/

#define ROUNDUP(x, esize)  ((x + (esize - 1)) / (esize)) * (esize)
#define DEVNAME_FMT        "/dev/sensor/sensor_%s%s%d"
#define DEVNAME_UNCAL      "_uncal"

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
  struct circbuf_s   buffer;             /* The circular buffer of sensor device */
  sem_t              exclsem;            /* Manages exclusive access to file operations */
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

static bool sensor_in_range(unsigned long left, unsigned long value,
                            unsigned long right)
{
  if (left < right)
    {
      return left <= value && value < right;
    }
  else
    {
      /* Maybe the data overflowed and a wraparound occurred */

      return left <= value || value < right;
    }
}

static bool sensor_is_updated(unsigned long generation,
                              unsigned long ugeneration)
{
  return generation > ugeneration;
}

static int sensor_update_interval(FAR struct file *filep,
                                  FAR struct sensor_upperhalf_s *upper,
                                  FAR struct sensor_user_s *user,
                                  unsigned long interval)
{
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *tmp;
  unsigned long min_interval = interval;
  int ret = 0;

  if (interval == user->interval)
    {
      return 0;
    }

  if (interval <= upper->state.min_interval)
    {
      goto update;
    }

  list_for_every_entry(&upper->userlist, tmp, struct sensor_user_s, node)
    {
      if (tmp == user)
        {
          continue;
        }

      if (min_interval > tmp->interval)
        {
          min_interval = tmp->interval;
        }
    }

update:
  if (min_interval == upper->state.min_interval)
    {
      user->interval = interval;
      return ret;
    }

  if (min_interval != ULONG_MAX && lower->ops->set_interval)
    {
      ret = lower->ops->set_interval(lower, filep, &min_interval);
      if (ret < 0)
        {
          return ret;
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

  if (upper->state.min_interval == 0)
    {
      return -EINVAL;
    }

  if (latency == user->latency)
    {
      return 0;
    }

  if (latency <= upper->state.min_latency)
    {
      goto update;
    }

  list_for_every_entry(&upper->userlist, tmp, struct sensor_user_s, node)
    {
      if (tmp == user)
        {
          continue;
        }

      if (min_latency > tmp->latency)
        {
          min_latency = tmp->latency;
        }
    }

update:
  if (min_latency == upper->state.min_latency)
    {
      user->latency = latency;
      return ret;
    }

  if (min_latency != ULONG_MAX && lower->ops->batch)
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
  int ret;

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

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
    }

  user->interval   = ULONG_MAX;
  user->latency    = ULONG_MAX;
  user->generation = upper->state.generation;
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
  nxsem_post(&upper->exclsem);
  return ret;
}

static int sensor_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR struct sensor_user_s *user = filep->f_priv;
  int ret;

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (lower->ops->close)
    {
      ret = lower->ops->close(lower, filep);
      if (ret < 0)
        {
          nxsem_post(&upper->exclsem);
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
  nxsem_post(&upper->exclsem);

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
  unsigned long nums;
  ssize_t ret;

  if (!buffer || !len)
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (lower->ops->fetch)
    {
      if (!(filep->f_oflags & O_NONBLOCK))
        {
          nxsem_post(&upper->exclsem);
          ret = nxsem_wait_uninterruptible(&user->buffersem);
          if (ret < 0)
            {
              return ret;
            }

          ret = nxsem_wait(&upper->exclsem);
          if (ret < 0)
            {
              return ret;
            }
        }
      else if (!upper->state.nsubscribers)
        {
          ret = -EAGAIN;
          goto out;
        }

        ret = lower->ops->fetch(lower, filep, buffer, len);
    }
  else
    {
      /* We must make sure that when the semaphore is equal to 1, there must
       * be events available in the buffer, so we use a while statement to
       * synchronize this case that other read operations consume events
       * that have just entered the buffer.
       */

      while (circbuf_is_empty(&upper->buffer))
        {
          if (filep->f_oflags & O_NONBLOCK)
            {
              ret = -EAGAIN;
              goto out;
            }
          else
            {
              nxsem_post(&upper->exclsem);
              ret = nxsem_wait_uninterruptible(&user->buffersem);
              if (ret < 0)
                {
                  return ret;
                }

              ret = nxsem_wait(&upper->exclsem);
              if (ret < 0)
                {
                  return ret;
                }
            }
        }

      /* Always read the last data in the circbuffer as initial value
       * for new users when the sensor device has not yet generated
       * new data.
       */

      if (user->generation == upper->state.generation)
        {
          user->generation--;
        }

      /* If user's generation isn't within circbuffer range, the
       * oldest data in circbuffer are returned to the users.
       */

      else if (!sensor_in_range(upper->state.generation - lower->nbuffer,
                                user->generation, upper->state.generation))

        {
          user->generation = upper->state.generation - lower->nbuffer;
        }

      nums = upper->state.generation - user->generation;
      if (len < nums * upper->state.esize)
        {
          nums = len / upper->state.esize;
        }

      len = nums * upper->state.esize;
      ret = circbuf_peekat(&upper->buffer,
                           user->generation * upper->state.esize,
                           buffer, len);
      user->generation += nums;
    }

out:
  nxsem_post(&upper->exclsem);
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
  int ret;

  sninfo("cmd=%x arg=%08lx\n", cmd, arg);

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case SNIOC_GET_STATE:
        {
          memcpy((FAR void *)(uintptr_t)arg,
                 &upper->state, sizeof(upper->state));
          user->changed = false;
        }
        break;

      case SNIOC_SET_INTERVAL:
        {
          ret = sensor_update_interval(filep, upper, user, arg);
        }
        break;

      case SNIOC_BATCH:
        {
          ret = sensor_update_latency(filep, upper, user, arg);
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
          upper->state.priv = (FAR void *)(uintptr_t)arg;
        }
        break;

      case SNIOC_SET_BUFFER_NUMBER:
        {
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

  nxsem_post(&upper->exclsem);
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
  int ret;

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

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
      else if (sensor_is_updated(upper->state.generation, user->generation))
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
  nxsem_post(&upper->exclsem);
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

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (!circbuf_is_init(&upper->buffer))
    {
      /* Initialize sensor buffer when data is first generated */

      ret = circbuf_init(&upper->buffer, NULL, lower->nbuffer *
                         upper->state.esize);
      if (ret < 0)
        {
          nxsem_post(&upper->exclsem);
          return ret;
        }
    }

  circbuf_overwrite(&upper->buffer, data, bytes);
  upper->state.generation += envcount;
  list_for_every_entry(&upper->userlist, user, struct sensor_user_s, node)
    {
      if (sensor_is_updated(upper->state.generation, user->generation))
        {
          nxsem_get_value(&user->buffersem, &semcount);
          if (semcount < 1)
            {
              nxsem_post(&user->buffersem);
            }

          sensor_pollnotify_one(user, POLLIN);
        }
    }

  nxsem_post(&upper->exclsem);
  return bytes;
}

static void sensor_notify_event(FAR void *priv)
{
  FAR struct sensor_upperhalf_s *upper = priv;
  FAR struct sensor_user_s *user;
  int semcount;

  if (nxsem_wait(&upper->exclsem) < 0)
    {
      return;
    }

  list_for_every_entry(&upper->userlist, user, struct sensor_user_s, node)
    {
      nxsem_get_value(&user->buffersem, &semcount);
      if (semcount < 1)
        {
          nxsem_post(&user->buffersem);
        }

      sensor_pollnotify_one(user, POLLIN);
    }

  nxsem_post(&upper->exclsem);
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
  upper->state.min_latency = ULONG_MAX;
  if (lower->ops->activate)
    {
      upper->state.nadvertisers = 1;
    }

  nxsem_init(&upper->exclsem, 0, 1);

  /* Bind the lower half data structure member */

  lower->priv = upper;

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

  upper->state.nbuffer = lower->nbuffer;
  sninfo("Registering %s\n", path);
  ret = register_driver(path, &g_sensor_fops, 0666, upper);
  if (ret)
    {
      goto drv_err;
    }

#ifdef CONFIG_SENSORS_RPMSG
  lower = sensor_rpmsg_register(lower, path);
  if (lower == NULL)
    {
      ret = -EIO;
      goto drv_err;
    }
#endif

  upper->lower = lower;
  return ret;

drv_err:
  nxsem_destroy(&upper->exclsem);

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

  nxsem_destroy(&upper->exclsem);
  if (circbuf_is_init(&upper->buffer))
    {
      circbuf_uninit(&upper->buffer);
    }

  kmm_free(upper);
}
