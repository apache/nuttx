/****************************************************************************
 * drivers/safety/safety.c
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

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <debug.h>

#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/safety/safety.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Structure for each safety module user */

struct safety_user_s
{
  struct list_node   node;
  FAR struct pollfd *fds;       /* Poll file descriptors */
  bool               has_data;  /* Flag indicating new data available */
};

/* Safety module private data */

struct safety_priv_s
{
  struct safety_lowerhalf_s *lower;       /* Lower half driver structure */
  struct list_node           users;       /* User contexts */
  mutex_t                    mutex;       /* Mutex for exclusive access */
  spinlock_t                 spinlock;    /* Spinlock for atomic operations */
  bool                       has_data;    /* Data available before open */
  size_t                     result_size; /* Size of result data */
  uint8_t                    data[0];     /* Flexible array for result data */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     safety_open(FAR struct file *filep);
static int     safety_close(FAR struct file *filep);
static ssize_t safety_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static int     safety_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
static int     safety_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup);
static int     safety_handler(FAR void *arg, FAR void *result,
                              off_t offset, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: safety_handler
 *
 * Description:
 *   Generic handler for safety module fault notification. Stores the result
 *   data for later retrieval via read operation.
 ****************************************************************************/

static int safety_handler(FAR void *arg, FAR void *result,
                          off_t offset, size_t len)
{
  FAR struct safety_priv_s *priv = arg;
  FAR struct safety_user_s *user;
  irqstate_t flags;
  int ret = -EINVAL;

  if ((size_t)offset + len <= priv->result_size)
    {
      flags = spin_lock_irqsave_nopreempt(&priv->spinlock);

      priv->has_data = true;
      memcpy(priv->data + offset, result, len);

      /* Notify all active users */

      list_for_every_entry(&priv->users, user, struct safety_user_s, node)
        {
          user->has_data = true;
          if (user->fds)
            {
              poll_notify(&user->fds, 1, POLLIN);
            }
        }

      spin_unlock_irqrestore_nopreempt(&priv->spinlock, flags);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: safety_open
 ****************************************************************************/

static int safety_open(FAR struct file *filep)
{
  FAR struct inode              *inode = filep->f_inode;
  FAR struct safety_priv_s      *priv  = inode->i_private;
  FAR struct safety_lowerhalf_s *lower = priv->lower;
  FAR struct safety_user_s      *user;
  irqstate_t flags;
  int ret = OK;

  /* Allocate new user context */

  user = kmm_zalloc(sizeof(struct safety_user_s));
  if (user != NULL)
    {
      nxmutex_lock(&priv->mutex);

      user->has_data = priv->has_data;

      /* Initialize module on first open */

      if (list_is_empty(&priv->users))
        {
          if (lower->ops->setup != NULL)
            {
              ret = lower->ops->setup(lower);
              if (ret < 0)
                {
                  kmm_free(user);
                }
            }
        }

      if (ret >= 0)
        {
          flags = spin_lock_irqsave(&priv->spinlock);
          list_add_tail(&priv->users, &user->node);
          filep->f_priv = user;
          spin_unlock_irqrestore(&priv->spinlock, flags);
        }

      nxmutex_unlock(&priv->mutex);
    }
  else
    {
      ret = -ENOMEM;
    }

  return ret;
}

/****************************************************************************
 * Name: safety_close
 *
 * Description:
 *   Close a safety module device.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int safety_close(FAR struct file *filep)
{
  FAR struct inode              *inode = filep->f_inode;
  FAR struct safety_priv_s      *priv  = inode->i_private;
  FAR struct safety_lowerhalf_s *lower = priv->lower;
  FAR struct safety_user_s      *user  = filep->f_priv;
  irqstate_t flags;
  bool empty;

  nxmutex_lock(&priv->mutex);

  flags = spin_lock_irqsave(&priv->spinlock);
  list_delete(&user->node);
  empty = list_is_empty(&priv->users);
  spin_unlock_irqrestore(&priv->spinlock, flags);

  /* Shutdown module on last close */

  if (empty)
    {
      if (lower->ops->shutdown != NULL)
        {
          lower->ops->shutdown(lower);
        }
    }

  nxmutex_unlock(&priv->mutex);

  kmm_free(user);
  return OK;
}

/****************************************************************************
 * Name: safety_read
 *
 * Description:
 *   Read safety module fault result data.
 *
 * Input Parameters:
 *   filep  - File structure pointer
 *   buffer - User provided buffer for result data
 *   len    - Length of buffer in bytes
 *
 * Returned Value:
 *   Number of bytes read on success; a negated errno value on failure.
 *   -EINVAL if buffer is NULL or too small
 *   -EAGAIN if no data is available
 ****************************************************************************/

static ssize_t safety_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct safety_priv_s *priv  = inode->i_private;
  FAR struct safety_user_s *user  = filep->f_priv;
  irqstate_t flags;
  ssize_t ret = -EINVAL;

  if (len == priv->result_size && buffer != NULL)
    {
      flags = spin_lock_irqsave(&priv->spinlock);

      if (priv->has_data)
        {
          /* Copy result data to user buffer */

          memcpy(buffer, priv->data, priv->result_size);
          user->has_data = false;
          ret = (ssize_t)priv->result_size;
        }
      else
        {
          ret = -EAGAIN;
        }

        spin_unlock_irqrestore(&priv->spinlock, flags);
    }

  return ret;
}

/****************************************************************************
 * Name: safety_ioctl
 *
 * Description:
 *   Handle IOCTL commands for the safety module device.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   cmd   - The IOCTL command
 *   arg   - The argument of the IOCTL command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int safety_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode              *inode = filep->f_inode;
  FAR struct safety_priv_s      *priv  = inode->i_private;
  FAR struct safety_lowerhalf_s *lower = priv->lower;
  int                            ret   = -ENOTSUP;

  nxmutex_lock(&priv->mutex);

  switch (cmd)
    {
      case SAFETYIOC_INJECT:
        {
          if (lower->ops->inject)
            {
              ret = lower->ops->inject(lower, (FAR char *)((uintptr_t)arg));
            }
        }
        break;

      case SAFETYIOC_SELFTEST:
        {
          if (lower->ops->selftest)
            {
              ret = lower->ops->selftest(lower);
            }
        }
        break;

      default:

        /* Forward undefined commands to module specific handler */

        if (lower->ops->ioctl)
          {
            ret = lower->ops->ioctl(lower, cmd, arg);
          }
        else
          {
            ret = -ENOTTY;
          }

        break;
    }

  nxmutex_unlock(&priv->mutex);
  return ret;
}

/****************************************************************************
 * Name: safety_poll
 *
 * Description:
 *   Handle poll operations for the safety module device.
 *
 * Input Parameters:
 *   filep - File structure pointer
 *   fds   - The structure describing the events to be monitored
 *   setup - true: Setup the poll; false: Teardown the poll
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure
 ****************************************************************************/

static int safety_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct safety_priv_s *priv  = inode->i_private;
  FAR struct safety_user_s *user  = filep->f_priv;
  irqstate_t flags;
  int ret = OK;

  flags = spin_lock_irqsave_nopreempt(&priv->spinlock);

  if (setup)
    {
      /* Save the poll structure for use by safety_handler */

      if (user->fds)
        {
          ret = -EBUSY;
        }
      else
        {
          user->fds = fds;
          fds->priv = user;

          /* Check for data already available */

          if (user->has_data)
            {
              poll_notify(&fds, 1, POLLIN);
            }
        }
    }
  else if (fds->priv == user)
    {
      /* Release our saved state */

      user->fds = NULL;
      fds->priv = NULL;
    }

  spin_unlock_irqrestore_nopreempt(&priv->spinlock, flags);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: safety_register
 ****************************************************************************/

int safety_register(FAR struct safety_lowerhalf_s *lower,
                    enum safety_module_e module,
                    size_t result_size)
{
  static FAR const char *g_safety_modules[] =
  {
    "clock",
    "power",
    "cpu",
    "flash",
    "ram",
    "mpu",
    "bus",
    "temperature",
    "error_report",
    "smu",
    "reg"
  };

  static const struct file_operations g_safety_fops =
  {
    safety_open,    /* open  */
    safety_close,   /* close */
    safety_read,    /* read  */
    NULL,           /* write */
    NULL,           /* seek  */
    safety_ioctl,   /* ioctl */
    NULL,           /* mmap */
    NULL,           /* truncate */
    safety_poll     /* poll  */
  };

  FAR struct safety_priv_s *priv;
  char path[32];
  int ret = OK;

  /* Parameter check */

  if (lower != NULL && lower->ops != NULL &&
      module < SAFETY_MODULE_MAX)
    {
      priv = kmm_zalloc(sizeof(struct safety_priv_s) + result_size);
      if (priv != NULL)
        {
          /* Initialize private data */

          spin_lock_init(&priv->spinlock);
          nxmutex_init(&priv->mutex);
          list_initialize(&priv->users);
          priv->lower = lower;
          priv->result_size = result_size;

          /* Set the fault handler */

          if (lower->ops->set_callback)
            {
              ret = lower->ops->set_callback(lower, safety_handler, priv);
              if (ret < 0)
                {
                  saerr("set_callback failed!\n");
                  nxmutex_destroy(&priv->mutex);
                  kmm_free(priv);
                }
            }

          /* Register device node */

          if (ret >= 0)
            {
              snprintf(path, sizeof(path), "/dev/safety/%s",
                       g_safety_modules[module]);
              ret = register_driver(path, &g_safety_fops, 0666, priv);
              if (ret < 0)
                {
                  saerr("register_driver failed!\n");
                  nxmutex_destroy(&priv->mutex);
                  kmm_free(priv);
                }
            }
        }
      else
        {
          ret = -ENOMEM;
        }
    }
  else
    {
      ret = -EINVAL;
    }

  return ret;
}
