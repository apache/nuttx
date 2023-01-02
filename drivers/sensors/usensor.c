/****************************************************************************
 * drivers/sensors/usensor.c
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
#include <nuttx/fs/fs.h>
#include <nuttx/list.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define USENSOR_PATH       "/dev/usensor"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int usensor_open(FAR struct file *filep);
static int usensor_close(FAR struct file *filep);
static ssize_t usensor_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t usensor_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int usensor_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes context of usensor */

struct usensor_context_s
{
  mutex_t          lock;      /* Manages exclusive access to file operations */
  struct list_node list;      /* List of node registered */
};

/* This structure describes lowerhalf driver for an usensor registered */

struct usensor_lowerhalf_s
{
  struct list_node          node;    /* node in all usensor list */
  struct sensor_lowerhalf_s driver;  /* The lowerhalf driver of user sensor */
  char                      path[1]; /* The path of usensor character device */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_usensor_fops =
{
  usensor_open,  /* open  */
  usensor_close, /* close */
  usensor_read,  /* read  */
  usensor_write, /* write */
  NULL,          /* seek  */
  usensor_ioctl, /* ioctl */
  NULL,          /* truncate */
  NULL,          /* mmap */
  NULL,          /* poll  */
};

static const struct sensor_ops_s g_usensor_ops =
{
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int usensor_register(FAR struct usensor_context_s *usensor,
                            FAR const struct sensor_reginfo_s *info)
{
  FAR struct usensor_lowerhalf_s *lower;
  int ret;

  lower = kmm_zalloc(sizeof(*lower) + strlen(info->path));
  if (!lower)
    {
      return -ENOMEM;
    }

  lower->driver.nbuffer = info->nbuffer;
  lower->driver.persist = info->persist;
  lower->driver.ops = &g_usensor_ops;
  strcpy(lower->path, info->path);
  ret = sensor_custom_register(&lower->driver, lower->path, info->esize);
  if (ret < 0)
    {
      goto errout_with_lower;
    }

  ret = nxmutex_lock(&usensor->lock);
  if (ret < 0)
    {
      goto errout_with_register;
    }

  list_add_tail(&usensor->list, &lower->node);
  nxmutex_unlock(&usensor->lock);

  return ret;

errout_with_register:
  sensor_custom_unregister(&lower->driver, info->path);
errout_with_lower:
  kmm_free(lower);
  return ret;
}

static int usensor_unregister(FAR struct usensor_context_s *usensor,
                              FAR const char *path)
{
  FAR struct usensor_lowerhalf_s *lower;
  int ret;

  ret = nxmutex_lock(&usensor->lock);
  if (ret < 0)
    {
      return ret;
    }

  list_for_every_entry(&usensor->list, lower, struct usensor_lowerhalf_s,
                       node)
    {
      if (strcmp(path, lower->path) == 0)
        {
          list_delete(&lower->node);
          nxmutex_unlock(&usensor->lock);
          sensor_custom_unregister(&lower->driver, path);
          kmm_free(lower);
          return 0;
        }
    }

  nxmutex_unlock(&usensor->lock);
  return -ENOENT;
}

static int usensor_open(FAR struct file *filep)
{
  return 0;
}

static int usensor_close(FAR struct file *filep)
{
  return 0;
}

static ssize_t usensor_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return buflen;
}

static ssize_t usensor_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  return buflen;
}

static int usensor_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usensor_context_s *usensor = inode->i_private;
  int ret = -ENOTTY;

  if (cmd == SNIOC_REGISTER)
    {
      FAR const struct sensor_reginfo_s *info;

      info = (FAR const struct sensor_reginfo_s *)(uintptr_t)arg;
      ret = usensor_register(usensor, info);
    }
  else if (cmd == SNIOC_UNREGISTER)
    {
      FAR const char *path;

      path = (FAR const char *)(uintptr_t)arg;
      ret = usensor_unregister(usensor, path);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usensor_initialize
 *
 * Description:
 *   This function registers usensor character node "/dev/usensor", so that
 *   application can register user sensor by this node. The node will
 *   manager all user sensors in this character dirver.
 ****************************************************************************/

int usensor_initialize(void)
{
  FAR struct usensor_context_s *usensor;
  int ret;

  usensor = kmm_zalloc(sizeof(*usensor));
  if (!usensor)
    {
      return -ENOMEM;
    }

  nxmutex_init(&usensor->lock);
  list_initialize(&usensor->list);

  ret = register_driver(USENSOR_PATH, &g_usensor_fops, 0666, usensor);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  return ret;

errout_with_lock:
  nxmutex_destroy(&usensor->lock);
  kmm_free(usensor);
  return ret;
}
