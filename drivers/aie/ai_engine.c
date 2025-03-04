/****************************************************************************
 * drivers/aie/ai_engine.c
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

#include <debug.h>

#include <nuttx/aie/ai_engine.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct aie_upperhalf_s
{
  FAR struct aie_lowerhalf_s *lower;  /* The handle of lower half driver */
  mutex_t                    lock;    /* Mutual exclusion */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int aie_open(FAR struct file *filep);

static int aie_close(FAR struct file *filep);

static int aie_ioctl(FAR struct file *filep, int cmd,
                     unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_aie_ops =
{
  aie_open,     /* open */
  aie_close,    /* close */
  NULL,         /* read */
  NULL,         /* write */
  NULL,         /* seek */
  aie_ioctl,    /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aie_open
 *
 * Description:
 *   A open method to open the ai engine.
 *
 ****************************************************************************/

static int aie_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct aie_upperhalf_s *upper = inode->i_private;
  FAR struct aie_lowerhalf_s *lower;

  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  return OK;
}

/****************************************************************************
 * Name: aie_close
 *
 * Description:
 *   A close method to close the ai engine.
 *
 ****************************************************************************/

static int aie_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct aie_upperhalf_s *upper = inode->i_private;
  FAR struct aie_lowerhalf_s *lower;
  int                         ret;

  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = (int)(intptr_t)filep->f_priv;
  if (ret > 0)
    {
      lower->ops->deinit(lower, ret);
    }

  nxmutex_unlock(&upper->lock);

  return OK;
}

/****************************************************************************
 * Name: aie_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *   This is where ALL of the aie work is done.
 *
 ****************************************************************************/

static int aie_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct aie_upperhalf_s *upper = inode->i_private;
  FAR struct aie_lowerhalf_s *lower = NULL;
  int                         ret;

  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  _info("cmd: %d arg: %lu\n", cmd, arg);

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case AIE_CMD_LOAD:
        ret = (int)(intptr_t)filep->f_priv;
        if (ret > 0)
          {
            ret = -EINVAL; /* Double load is not allowed */
          }
        else
          {
            ret = lower->ops->init(lower, arg /* model */);
            if (ret > 0)
              {
                filep->f_priv = (void *)(intptr_t)ret;
                ret = OK;
              }
          }
        break;

      case AIE_CMD_FEED_INPUT:
        ret = lower->ops->feed_input(lower, (int)(intptr_t)filep->f_priv,
                                     arg /* input */);
        break;

      case AIE_CMD_GET_OUTPUT:
        ret = lower->ops->get_output(lower, (int)(intptr_t)filep->f_priv,
                                     arg /* output */);
        break;

      default:
        if (lower->ops->control)
          {
            ret = lower->ops->control(lower, (int)(intptr_t)filep->f_priv,
                                      cmd, arg);
          }
        else
          {
            ret = -ENOSYS;
          }
        break;
    }

  nxmutex_unlock(&upper->lock);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aie_register
 *
 * Description:
 *   Register a aie driver.
 *
 ****************************************************************************/

int aie_register(FAR const char *path,
                 FAR struct aie_lowerhalf_s *lower)
{
  FAR struct aie_upperhalf_s *upper = NULL;
  int                         ret   = -ENOMEM;

  DEBUGASSERT(path);

  /* Allocate the upper-half data structure */

  upper = (FAR struct aie_upperhalf_s *)
          kmm_malloc(sizeof(struct aie_upperhalf_s));
  if (!upper)
    {
      _err("Upper half allocation failed\n");
      goto errout;
    }

  /* Initialize the aie device structure */

  upper->lower = lower;
  nxmutex_init(&upper->lock);

  /* Register the aie device */

  ret = register_driver(path, &g_aie_ops, 0666, upper);
  if (ret < 0)
    {
      _err("register aie driver failed: %d\n", ret);
      nxmutex_destroy(&upper->lock);
      kmm_free(upper);
    }

errout:
  return ret;
}
