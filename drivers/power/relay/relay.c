/****************************************************************************
 * drivers/power/relay/relay.c
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

#include <errno.h>

#include <nuttx/mutex.h>
#include <nuttx/power/relay.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t relay_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t relay_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     relay_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_relay_ops =
{
  NULL,         /* open */
  NULL,         /* close */
  relay_read,   /* read */
  relay_write,  /* write */
  NULL,         /* seek */
  relay_ioctl,  /* ioctl */
  NULL          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t relay_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  return 0;
}

static ssize_t relay_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return 0;
}

static int relay_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct relay_dev_s *dev;
  FAR struct inode *inode;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case RELAYIOC_SET:
        {
          FAR bool *enable = (FAR bool *)((uintptr_t)arg);
          DEBUGASSERT(enable != NULL);
          ret = dev->ops->set(dev, *enable);
        }
        break;

      case RELAYIOC_GET:
        DEBUGASSERT(arg != 0);
        ret = dev->ops->get(dev, (FAR bool *)((uintptr_t)arg));
        break;

      default:
        if (dev->ops->ioctl != NULL)
          {
            ret = dev->ops->ioctl(dev, cmd, arg);
          }
        else
          {
            ret = -ENOTSUP;
          }
        break;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: relay_register
 *
 * Description:
 *   Register the relay driver to the vfs.
 *
 * Input Parameters:
 *   dev     - the relay device
 *   devname - the relay device name
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int relay_register(FAR struct relay_dev_s *dev, FAR const char *devname)
{
  DEBUGASSERT(devname != NULL);
  DEBUGASSERT(dev != NULL && dev->ops != NULL);

  if (dev->ops->set == NULL || dev->ops->get == NULL)
    {
      return -EINVAL;
    }

  nxmutex_init(&dev->lock);

  return register_driver(devname, &g_relay_ops, 0666, dev);
}
