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
#include <errno.h>
#include <debug.h>

#include <poll.h>
#include <fcntl.h>
#include <nuttx/kmalloc.h>
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
  uint8_t         idx;
  const uint8_t   esize;
  FAR const char *name;
};

/* This structure describes sensor circular buffer */

struct sensor_buffer_s
{
  uint32_t  head;
  uint32_t  tail;
  uint32_t  size;
  FAR void *data;
};

/* This structure describes the state of the upper half driver */

struct sensor_upperhalf_s
{
  FAR struct sensor_lowerhalf_s *lower;  /* the handle of lower half driver */
  FAR struct sensor_buffer_s    *buffer; /* The circualr buffer of sensor device */
  FAR struct pollfd *fds;                /* poll structures of threads waiting for driver events. */
  uint8_t            idx;                /* The index number of node path */
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
static int     sensor_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
static int     sensor_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sensor_info g_sensor_info[] =
{
  {0, sizeof(struct sensor_event_accel), "accel"},
  {0, sizeof(struct sensor_event_mag),   "mag"},
  {0, sizeof(struct sensor_event_gyro),  "gyro"},
  {0, sizeof(struct sensor_event_light), "light"},
  {0, sizeof(struct sensor_event_baro),  "baro"},
  {0, sizeof(struct sensor_event_prox),  "prox"},
  {0, sizeof(struct sensor_event_humi),  "humi"},
  {0, sizeof(struct sensor_event_temp),  "temp"},
  {0, sizeof(struct sensor_event_rgb),   "rgb"},
  {0, sizeof(struct sensor_event_hall),  "hall"},
  {0, sizeof(struct sensor_event_ir),    "ir"},
  {0, sizeof(struct sensor_event_gps),   "gps"},
  {0, sizeof(struct sensor_event_uv),    "uv"},
  {0, sizeof(struct sensor_event_noise), "noise"},
  {0, sizeof(struct sensor_event_pm25),  "pm25"},
  {0, sizeof(struct sensor_event_pm1p0), "pm1p0"},
  {0, sizeof(struct sensor_event_pm10),  "pm10"},
  {0, sizeof(struct sensor_event_co2),   "co2"},
  {0, sizeof(struct sensor_event_hcho),  "hcho"},
  {0, sizeof(struct sensor_event_tvoc),  "tvoc"},
  {0, sizeof(struct sensor_event_ph),    "ph"},
  {0, sizeof(struct sensor_event_dust),  "dust"},
  {0, sizeof(struct sensor_event_hrate), "hrate"},
  {0, sizeof(struct sensor_event_hbeat), "hbeat"},
};

static const struct file_operations g_sensor_fops =
{
  sensor_open,    /* open  */
  sensor_close,   /* close */
  sensor_read,    /* read  */
  NULL,           /* write */
  NULL,           /* seek  */
  sensor_ioctl,   /* ioctl */
  sensor_poll     /* poll  */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool sensor_buffer_is_empty(FAR struct sensor_buffer_s *buffer)
{
  return buffer->head == buffer->tail;
}

static uint32_t sensor_buffer_len(FAR struct sensor_buffer_s *buffer)
{
  return buffer->head - buffer->tail;
}

static uint32_t sensor_buffer_unused(FAR struct sensor_buffer_s *buffer)
{
  return buffer->size - sensor_buffer_len(buffer);
}

static void sensor_buffer_reset(FAR struct sensor_buffer_s *buffer)
{
  buffer->head = buffer->tail = 0;
}

static void sensor_buffer_push(FAR struct sensor_buffer_s *buffer,
                               FAR const void *data, uint32_t bytes)
{
  uint32_t space = sensor_buffer_unused(buffer);
  uint32_t off = buffer->head % buffer->size;
  uint32_t overwrite = 0;

  /* If buffer is full or there is not enough space, overwriting of old
   * data will occur, we should move tail point after pushing data
   * completely.
   */

  if (bytes > buffer->size)
    {
      data += bytes - buffer->size;
      bytes = buffer->size;
    }

  if (bytes > space)
    {
      overwrite = bytes - space;
    }

  space = buffer->size - off;
  if (bytes < space)
    {
      space = bytes;
    }

  memcpy(buffer->data + off, data, space);
  memcpy(buffer->data, data + space, bytes - space);
  buffer->head += bytes;
  buffer->tail += overwrite;
}

static uint32_t sensor_buffer_pop(FAR struct sensor_buffer_s *buffer,
                                  FAR void *data, uint32_t bytes)
{
  uint32_t len = sensor_buffer_len(buffer);
  uint32_t off;

  if (bytes > len)
    {
      bytes = len;
    }

  if (!data)
    {
      goto skip;
    }

  off = buffer->tail % buffer->size;
  len = buffer->size - off;
  if (bytes < len)
    {
      len = bytes;
    }

  memcpy(data, buffer->data + off, len);
  memcpy(data + len, buffer->data, bytes - len);

skip:
  buffer->tail += bytes;

  return bytes;
}

static int sensor_buffer_resize(FAR struct sensor_buffer_s **buffer,
                                int type, uint32_t bytes)
{
  FAR struct sensor_buffer_s *tmp;
  int len = sensor_buffer_len(*buffer);
  int skipped;

  bytes = ROUNDUP(bytes, g_sensor_info[type].esize);
  tmp = kmm_malloc(sizeof(*tmp) + bytes);
  if (!tmp)
    {
      snerr("Faild to alloc memory for circular buffer\n");
      return -ENOMEM;
    }

  tmp->data = tmp + 1;

  skipped = (bytes > len) ? 0 : len - bytes;
  len -= skipped;
  sensor_buffer_pop(*buffer, NULL, skipped);
  sensor_buffer_pop(*buffer, tmp->data, len);

  tmp->size = bytes;
  tmp->head = len;
  tmp->tail = 0;

  kmm_free(*buffer);
  *buffer = tmp;

  return 0;
}

static int sensor_buffer_create(FAR struct sensor_buffer_s **buffer,
                                int type, uint32_t bytes)
{
  FAR struct sensor_buffer_s *tmp;

  bytes = ROUNDUP(bytes, g_sensor_info[type].esize);

  tmp = kmm_malloc(sizeof(*tmp) + bytes);
  if (!tmp)
    {
      snerr("Faild to malloc memory for circular buffer\n");
      return -ENOMEM;
    }

  tmp->size = bytes;
  tmp->data = tmp + 1;
  tmp->head = 0;
  tmp->tail = 0;

  *buffer = tmp;

  return 0;
}

static void sensor_buffer_release(FAR struct sensor_buffer_s *buffer)
{
  kmm_free(buffer);
}

static void sensor_pollnotify(FAR struct sensor_upperhalf_s *upper,
                              pollevent_t eventset)
{
  int semcount;

  if (upper->fds)
    {
      upper->fds->revents |= (upper->fds->events & eventset);

      if (upper->fds->revents != 0)
        {
          sninfo("Report events: %02x\n", upper->fds->revents);

          nxsem_get_value(upper->fds->sem, &semcount);
          if (semcount < 1)
            {
              nxsem_post(upper->fds->sem);
            }
        }
    }
}

static int sensor_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  int ret;

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (upper->crefs)
    {
      ret = -EBUSY;
    }
  else
    {
      upper->crefs++;
      upper->fds = NULL;
      sensor_buffer_reset(upper->buffer);
    }

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

  /* We must make sure that when the semaphore is equal to 1, there must
   * be events avaliable in the buffer, so we use a while statement to
   * synchronize this case that other read operations consume events
   * that have just entered the buffer.
   */

  while (sensor_buffer_is_empty(upper->buffer))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto again;
        }
      else
        {
          nxsem_post(&upper->exclsem);
          ret = nxsem_wait_uninterruptible(&upper->buffersem);
          if (ret)
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

  ret = sensor_buffer_pop(upper->buffer, buffer, len);

  /* Release some buffer space when current mode isn't batch mode and last
   * mode is batch mode, and the number of bytes avaliable in buffer is
   * less than the number of bytes origin.
   */

  if (upper->latency == 0 &&
      upper->buffer->size > lower->buffer_bytes &&
      sensor_buffer_len(upper->buffer) <= lower->buffer_bytes)
    {
      sensor_buffer_resize(&upper->buffer, lower->type, lower->buffer_bytes);
    }

again:
  nxsem_post(&upper->exclsem);
  return ret;
}

static int sensor_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sensor_upperhalf_s *upper = inode->i_private;
  FAR struct sensor_lowerhalf_s *lower = upper->lower;
  FAR unsigned int *val = (unsigned int *)(uintptr_t)arg;
  int ret;

  sninfo("cmd=%x arg=%08x\n", cmd, arg);

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
            }
        }
        break;

      case SNIOC_SET_INTERVAL:
        {
          if (upper->interval == *val)
            {
              break;
            }

          ret = lower->ops->set_interval ?
                lower->ops->set_interval(lower, val) : -ENOTSUP;
          if (ret >= 0)
            {
              upper->interval = *val;
            }
        }
        break;

      case SNIOC_BATCH:
        {
          if (upper->interval == 0)
            {
              ret = -EINVAL;
              break;
            }

          if (upper->latency == *val)
            {
              break;
            }

          ret = lower->ops->batch ?
                lower->ops->batch(lower, val) : -ENOTSUP;
          if (ret >= 0)
            {
              upper->latency = *val;
              if (*val != 0)
                {
                  /* Adjust length of buffer in batch mode */

                  sensor_buffer_resize(&upper->buffer, lower->type,
                                       lower->buffer_bytes +
                                       ROUNDUP(*val, upper->interval) /
                                       upper->interval *
                                       g_sensor_info[lower->type].esize);
                }
            }
        }
        break;

      case SNIOC_GET_NEVENTBUF:
        {
          *val = lower->buffer_bytes / g_sensor_info[lower->type].esize;
        }
        break;

      default:
        ret = -ENOTTY;
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
  pollevent_t eventset = 0;
  int ret;

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      if (upper->fds)
        {
          ret = -EBUSY;
          goto errout;
        }

      upper->fds = fds;
      fds->priv = &upper->fds;

      if (!sensor_buffer_is_empty(upper->buffer))
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
      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      if (!slot)
        {
          ret = -EIO;
          goto errout;
        }

      *slot = NULL;
      fds->priv = NULL;
    }

errout:
  nxsem_post(&upper->exclsem);
  return ret;
}

static void sensor_push_event(FAR void *priv, FAR const void *data,
                                     uint32_t bytes)
{
  FAR struct sensor_upperhalf_s *upper = priv;
  int semcount;

  if (!bytes || nxsem_wait(&upper->exclsem) < 0)
    {
      return;
    }

  sensor_buffer_push(upper->buffer, data, bytes);
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
 *   dev  - A pointer to an instance of lower half sensor driver. This
 *          instance is bound to the sensor driver and must persists as long
 *          as the driver persists.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int sensor_register(FAR struct sensor_lowerhalf_s *lower)
{
  FAR struct sensor_upperhalf_s *upper;
  char path[DEVNAME_MAX];
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL);

  if (lower->type >= SENSOR_TYPE_COUNT)
    {
      snerr("ERROR: Type is invalid\n");
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

  nxsem_init(&upper->exclsem, 0, 1);
  nxsem_init(&upper->buffersem, 0, 0);

  nxsem_set_protocol(&upper->buffersem, SEM_PRIO_NONE);

  /* Bind the lower half data structure member */

  lower->priv = upper;
  lower->push_event = sensor_push_event;

  if (!lower->buffer_bytes)
    {
      lower->buffer_bytes = g_sensor_info[lower->type].esize;
    }

  /* Initialize sensor buffer */

  ret = sensor_buffer_create(&upper->buffer,
                             lower->type, lower->buffer_bytes);
  if (ret)
    {
      goto buf_err;
    }

  upper->idx = g_sensor_info[lower->type].idx++;
  snprintf(path, DEVNAME_MAX, DEVNAME_FMT,
           g_sensor_info[lower->type].name,
           lower->uncalibrated ? DEVNAME_UNCAL : "",
           upper->idx);
  sninfo("Registering %s\n", path);

  ret = register_driver(path, &g_sensor_fops, 0666, upper);
  if (ret)
    {
      goto drv_err;
    }

  return ret;

drv_err:
  sensor_buffer_release(upper->buffer);
  g_sensor_info[lower->type].idx--;
buf_err:
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
 *   dev  - A pointer to an instance of lower half sensor driver. This
 *          instance is bound to the sensor driver and must persists as long
 *          as the driver persists.
 ****************************************************************************/

void sensor_unregister(FAR struct sensor_lowerhalf_s *lower)
{
  FAR struct sensor_upperhalf_s *upper;
  char path[DEVNAME_MAX];

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(lower->priv != NULL);

  upper = lower->priv;

  snprintf(path, DEVNAME_MAX, DEVNAME_FMT,
           g_sensor_info[lower->type].name,
           lower->uncalibrated ? DEVNAME_UNCAL : "",
           upper->idx);
  sninfo("UnRegistering %s\n", path);
  unregister_driver(path);

  nxsem_destroy(&upper->exclsem);
  nxsem_destroy(&upper->buffersem);

  sensor_buffer_release(upper->buffer);
  kmm_free(upper);
}
