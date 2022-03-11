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
#include <nuttx/kmalloc.h>
#include <nuttx/mm/circbuf.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device naming ************************************************************/

#define ROUNDUP(x, esize)  ((x + (esize - 1)) / (esize)) * (esize)
#define DEVNAME_FMT        "/dev/sensor/%s%s%d"
#define DEVNAME_MAX        64
#define DEVNAME_UNCAL      "_uncal"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes sensor info */

struct sensor_info
{
  uint8_t   esize;
  FAR char *name;
};

/* This structure describes the state of the upper half driver */

struct sensor_upperhalf_s
{
  /* poll structures of threads waiting for driver events. */

  FAR struct pollfd             *fds[CONFIG_SENSORS_NPOLLWAITERS];
  FAR struct sensor_lowerhalf_s *lower;  /* the handle of lower half driver */
  struct circbuf_s   buffer;             /* The circular buffer of sensor device */
  uint8_t            esize;              /* The element size of circular buffer */
  uint8_t            crefs;              /* Number of times the device has been opened */
  sem_t              exclsem;            /* Manages exclusive access to file operations */
  sem_t              buffersem;          /* Wakeup user waiting for data in circular buffer */
  bool               enabled;            /* The status of sensor enable or disable */
  unsigned int       interval;           /* The sample interval for sensor, in us */
  unsigned int       latency;            /* The batch latency for sensor, in us */
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

static const struct sensor_info g_sensor_info[] =
{
  {0,                                           NULL},
  {sizeof(struct sensor_event_accel),           "accel"},
  {sizeof(struct sensor_event_mag),             "mag"},
  {sizeof(struct sensor_event_gyro),            "gyro"},
  {sizeof(struct sensor_event_light),           "light"},
  {sizeof(struct sensor_event_baro),            "baro"},
  {sizeof(struct sensor_event_prox),            "prox"},
  {sizeof(struct sensor_event_humi),            "humi"},
  {sizeof(struct sensor_event_temp),            "temp"},
  {sizeof(struct sensor_event_rgb),             "rgb"},
  {sizeof(struct sensor_event_hall),            "hall"},
  {sizeof(struct sensor_event_ir),              "ir"},
  {sizeof(struct sensor_event_gps),             "gps"},
  {sizeof(struct sensor_event_uv),              "uv"},
  {sizeof(struct sensor_event_noise),           "noise"},
  {sizeof(struct sensor_event_pm25),            "pm25"},
  {sizeof(struct sensor_event_pm1p0),           "pm1p0"},
  {sizeof(struct sensor_event_pm10),            "pm10"},
  {sizeof(struct sensor_event_co2),             "co2"},
  {sizeof(struct sensor_event_hcho),            "hcho"},
  {sizeof(struct sensor_event_tvoc),            "tvoc"},
  {sizeof(struct sensor_event_ph),              "ph"},
  {sizeof(struct sensor_event_dust),            "dust"},
  {sizeof(struct sensor_event_hrate),           "hrate"},
  {sizeof(struct sensor_event_hbeat),           "hbeat"},
  {sizeof(struct sensor_event_ecg),             "ecg"},
  {sizeof(struct sensor_event_ppgd),            "ppgd"},
  {sizeof(struct sensor_event_ppgq),            "ppgq"},
  {sizeof(struct sensor_event_impd),            "impd"},
  {sizeof(struct sensor_event_ots),             "ots"},
  {sizeof(struct sensor_event_gps_satellite),   "gps_satellite"},
  {sizeof(struct sensor_event_wake_gesture),    "wake_gesture"},
  {sizeof(struct sensor_event_cap),             "cap"},
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

static void sensor_pollnotify(FAR struct sensor_upperhalf_s *upper,
                              pollevent_t eventset)
{
  FAR struct pollfd *fd;
  int semcount;
  int i;

  for (i = 0; i < CONFIG_SENSORS_NPOLLWAITERS; i++)
    {
      fd = upper->fds[i];
      if (fd)
        {
          fd->revents |= (fd->events & eventset);

          if (fd->revents != 0)
            {
              sninfo("Report events: %08" PRIx32 "\n", fd->revents);

              nxsem_get_value(fd->sem, &semcount);
              if (semcount < 1)
                {
                  nxsem_post(fd->sem);
                }
            }
        }
    }
}

static int sensor_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  uint8_t tmp;
  int ret;

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto err;
    }
  else if (tmp == 1)
    {
      /* Initialize sensor buffer */

      ret = circbuf_init(&upper->buffer, NULL, lower->buffer_number *
                         upper->esize);
      if (ret < 0)
        {
          goto err;
        }
    }

  upper->crefs = tmp;
err:
  nxsem_post(&upper->exclsem);
  return ret;
}

static int sensor_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  int ret;

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (--upper->crefs <= 0 && upper->enabled)
    {
      ret = lower->ops->activate ?
            lower->ops->activate(lower, false) : -ENOTSUP;
      if (ret >= 0)
        {
          upper->enabled = false;
          circbuf_uninit(&upper->buffer);
        }
    }

  nxsem_post(&upper->exclsem);
  return ret;
}

static ssize_t sensor_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
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
          ret = nxsem_wait_uninterruptible(&upper->buffersem);
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
      else if (!upper->enabled)
        {
          ret = -EAGAIN;
          goto out;
        }

        ret = lower->ops->fetch(lower, buffer, len);
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
              ret = nxsem_wait_uninterruptible(&upper->buffersem);
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

      ret = circbuf_read(&upper->buffer, buffer, len);

      /* Release some buffer space when current mode isn't batch mode
       * and last mode is batch mode, and the number of bytes available
       * in buffer is less than the number of bytes origin.
       */

      uint32_t buffer_size = lower->buffer_number * upper->esize;
      if (upper->latency == 0 &&
          circbuf_size(&upper->buffer) > buffer_size &&
          circbuf_used(&upper->buffer) <= buffer_size)
        {
          ret = circbuf_resize(&upper->buffer, buffer_size);
        }
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

  return sensor_push_event(upper, buffer, buflen);
}

static int sensor_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR unsigned int *val = (unsigned int *)(uintptr_t)arg;
  int ret;

  sninfo("cmd=%x arg=%08lx\n", cmd, arg);

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case SNIOC_ACTIVATE:
        {
          if (upper->enabled == !!arg)
            {
              break;
            }

          ret = lower->ops->activate ?
                lower->ops->activate(lower, !!arg) : -ENOTSUP;
          if (ret >= 0)
            {
              upper->enabled = !!arg;
              if (!upper->enabled)
                {
                  upper->interval = 0;
                  upper->latency = 0;
                  ret = circbuf_resize(&upper->buffer, lower->buffer_number *
                                                       upper->esize);
                }
            }
        }
        break;

      case SNIOC_SET_INTERVAL:
        {
          if (lower->ops->set_interval == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          if (upper->interval == *val)
            {
              break;
            }

          ret = lower->ops->set_interval(lower, val);
          if (ret >= 0)
            {
              upper->interval = *val;
            }
        }
        break;

      case SNIOC_BATCH:
        {
          if (lower->ops->batch == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          if (upper->interval == 0)
            {
              ret = -EINVAL;
              break;
            }

          if (upper->latency == *val)
            {
              break;
            }

          ret = lower->ops->batch(lower, val);
          if (ret >= 0)
            {
              upper->latency = *val;
              if (*val != 0)
                {
                  /* Adjust length of buffer in batch mode */

                  uint32_t buffer_size = (ROUNDUP(*val, upper->interval) /
                                         upper->interval +
                                         lower->buffer_number) *
                                         upper->esize;

                  ret = circbuf_resize(&upper->buffer, buffer_size);
                }
            }
        }
        break;

      case SNIOC_SELFTEST:
        {
          if (lower->ops->selftest == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          ret = lower->ops->selftest(lower, arg);
        }
        break;

      case SNIOC_SET_CALIBVALUE:
        {
          if (lower->ops->set_calibvalue == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          ret = lower->ops->set_calibvalue(lower, arg);
        }
        break;

      case SNIOC_CALIBRATE:
        {
          if (lower->ops->calibrate == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          ret = lower->ops->calibrate(lower, arg);
        }
        break;

      case SNIOC_GET_NEVENTBUF:
        {
          *val = lower->buffer_number + lower->batch_number;
        }
        break;

      case SNIOC_SET_BUFFER_NUMBER:
        {
          if (arg != 0)
            {
              lower->buffer_number = arg;
              ret = circbuf_resize(&upper->buffer, arg * upper->esize);
            }
        }
        break;

      default:

        /* Lowerhalf driver process other cmd. */

        if (lower->ops->control)
          {
            ret = lower->ops->control(lower, cmd, arg);
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
                       struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  pollevent_t eventset = 0;
  int semcount;
  int ret;
  int i;

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      for (i = 0; i < CONFIG_SENSORS_NPOLLWAITERS; i++)
        {
          if (NULL == upper->fds[i])
            {
              upper->fds[i] = fds;
              fds->priv = &upper->fds[i];
              break;
            }
        }

      /* Don't have enough space to store fds */

      if (i == CONFIG_SENSORS_NPOLLWAITERS)
        {
          ret = -ENOSPC;
          goto errout;
        }

      if (lower->ops->fetch)
        {
          /* Always return POLLIN for fetch data directly(non-block) */

          if (filep->f_oflags & O_NONBLOCK)
            {
              eventset |= (fds->events & POLLIN);
            }
          else
            {
              nxsem_get_value(&upper->buffersem, &semcount);
              if (semcount > 0)
                {
                  eventset |= (fds->events & POLLIN);
                }
            }
        }
      else if (!circbuf_is_empty(&upper->buffer))
        {
          eventset |= (fds->events & POLLIN);
        }

      if (eventset)
        {
          sensor_pollnotify(upper, eventset);
        }
    }
  else if (fds->priv != NULL)
    {
      for (i = 0; i < CONFIG_SENSORS_NPOLLWAITERS; i++)
        {
          if (fds == upper->fds[i])
            {
              upper->fds[i] = NULL;
              fds->priv = NULL;
              break;
            }
        }
    }

errout:
  nxsem_post(&upper->exclsem);
  return ret;
}

static ssize_t sensor_push_event(FAR void *priv, FAR const void *data,
                                 size_t bytes)
{
  FAR struct sensor_upperhalf_s *upper = priv;
  int semcount;
  int ret;

  if (!bytes)
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  circbuf_overwrite(&upper->buffer, data, bytes);
  sensor_pollnotify(upper, POLLIN);
  nxsem_get_value(&upper->buffersem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(&upper->buffersem);
    }

  nxsem_post(&upper->exclsem);
  return bytes;
}

static void sensor_notify_event(FAR void *priv)
{
  FAR struct sensor_upperhalf_s *upper = priv;
  int semcount;

  if (nxsem_wait(&upper->exclsem) < 0)
    {
      return;
    }

  sensor_pollnotify(upper, POLLIN);
  nxsem_get_value(&upper->buffersem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(&upper->buffersem);
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
  char path[DEVNAME_MAX];

  DEBUGASSERT(lower != NULL);

  snprintf(path, DEVNAME_MAX, DEVNAME_FMT,
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
                           FAR const char *path, uint8_t esize)
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

  upper->lower = lower;
  upper->esize = esize;

  nxsem_init(&upper->exclsem, 0, 1);
  nxsem_init(&upper->buffersem, 0, 0);

  nxsem_set_protocol(&upper->buffersem, SEM_PRIO_NONE);

  /* Bind the lower half data structure member */

  lower->priv = upper;

  if (!lower->ops->fetch)
    {
      if (!lower->buffer_number)
        {
          lower->buffer_number = 1;
        }

      lower->push_event = sensor_push_event;
    }
  else
    {
      lower->notify_event = sensor_notify_event;
      lower->buffer_number = 0;
    }

  sninfo("Registering %s\n", path);
  ret = register_driver(path, &g_sensor_fops, 0666, upper);
  if (ret)
    {
      goto drv_err;
    }

  return ret;

drv_err:
  nxsem_destroy(&upper->exclsem);
  nxsem_destroy(&upper->buffersem);

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
  char path[DEVNAME_MAX];

  snprintf(path, DEVNAME_MAX, DEVNAME_FMT,
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

  nxsem_destroy(&upper->exclsem);
  nxsem_destroy(&upper->buffersem);

  kmm_free(upper);
}
