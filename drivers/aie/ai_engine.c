/****************************************************************************
 * drivers/aie/ai_engine.c
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

enum aie_state_e
{
  AIE_STATE_NOP = 0,
  AIE_STATE_INITED,
  AIE_STATE_FEEDED,
  AIE_STATE_INFERENCED,
};

/* This structure describes the state of the upper half driver */

struct aie_upperhalf_s
{
  FAR struct aie_lowerhalf_s *lower;  /* The handle of lower half driver */
  volatile enum aie_state_e  state;   /* The device state */
  mutex_t                    lock;    /* Mutual exclusion */
  uint8_t                    crefs;   /* The number of times the engine
                                       * has been opend. */
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
 *   A open method to increase the crefs.
 *
 ****************************************************************************/

static int aie_open(FAR struct file *filep)
{
  FAR struct inode              *inode = filep->f_inode;
  FAR struct aie_upperhalf_s    *upper = inode->i_private;
  int                            ret;

  DEBUGASSERT(upper != NULL);
  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (upper->crefs == 0)
    {
      upper->crefs++;
      ret = OK;
    }
  else
    {
      ret = -EBUSY;
    }

  nxmutex_unlock(&upper->lock);

  return ret;
}

/****************************************************************************
 * Name: aie_close
 *
 * Description:
 *   A close method to decrease the crefs.
 *
 ****************************************************************************/

static int aie_close(FAR struct file *filep)
{
  FAR struct inode              *inode = filep->f_inode;
  FAR struct aie_upperhalf_s    *upper = inode->i_private;
  FAR struct aie_lowerhalf_s    *lower = NULL;
  int                            ret;

  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (upper->crefs == 1)
    {
      ret = lower->ops->deinit(lower, filep);
      if (ret == OK)
        {
          upper->state = AIE_STATE_NOP;
          upper->crefs--;
        }
    }
  else
    {
      ret = OK;
    }

  nxmutex_unlock(&upper->lock);

  return ret;
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
  FAR struct inode              *inode = filep->f_inode;
  FAR struct aie_upperhalf_s    *upper = inode->i_private;
  FAR struct aie_lowerhalf_s    *lower = NULL;
  int                            ret;

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
      case AIE_CMD_INIT:
        ret = lower->ops->init(lower, filep, arg);
        if (ret != OK)
          {
            break;
          }

        if (lower->workspace_len)
          {
            lower->workspace = kmm_malloc(lower->workspace_len);
            if (!lower->workspace)
              {
                ret = -ENOMEM;
                break;
              }
          }

        upper->state = AIE_STATE_INITED;
        break;

      case AIE_CMD_FEED_INPUT:
        if (upper->state == AIE_STATE_INITED ||
            upper->state == AIE_STATE_INFERENCED)
          {
            lower->input = (uintptr_t)arg;
            ret = lower->ops->feed_input(lower, filep);
            if (ret == OK)
              {
                upper->state = AIE_STATE_FEEDED;
              }
          }
        else
          {
            ret = -EPERM;
          }
        break;

      case AIE_CMD_GET_OUTPUT:
        if (upper->state == AIE_STATE_FEEDED)
          {
            lower->output = (uintptr_t)arg;
            ret = lower->ops->get_output(lower, filep);
            if (ret == OK)
              {
                upper->state = AIE_STATE_INFERENCED;
              }
          }
        else
          {
            ret = -EPERM;
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
  FAR struct aie_upperhalf_s  *upper = NULL;
  int                          ret   = -ENOMEM;

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

  upper->lower   = lower;
  upper->state   = AIE_STATE_NOP;
  upper->crefs   = 0;
  lower->priv    = upper;

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
