/****************************************************************************
 * drivers/timers/capture.c
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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/timers/capture.h>

#include <arch/irq.h>

#ifdef CONFIG_CAPTURE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct cap_upperhalf_s
{
  uint8_t                    crefs;    /* The number of times the device has been opened */
  uint8_t                    nchan;    /* The number of channels, only invalid for multi channels */
  mutex_t                    lock;     /* Supports mutual exclusion */
  FAR struct cap_lowerhalf_s **lower;  /* lower-half state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     cap_open(FAR struct file *filep);
static int     cap_close(FAR struct file *filep);
static ssize_t cap_read(FAR struct file *filep, FAR char *buffer,
                       size_t buflen);
static ssize_t cap_write(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen);
static int cap_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_capops =
{
  cap_open,  /* open */
  cap_close, /* close */
  cap_read,  /* read */
  cap_write, /* write */
  NULL,      /* seek */
  cap_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cap_open
 *
 * Description:
 *   This function is called whenever the PWM Capture device is opened.
 *
 ****************************************************************************/

static int cap_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct cap_upperhalf_s *upper = inode->i_private;
  uint8_t                     tmp;
  int                         ret;

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      goto errout;
    }

  /* Increment the count of references to the device.  If this is the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_lock;
    }

  /* Check if this is the first time that the driver has been opened. */

  if (tmp == 1)
    {
      FAR struct cap_lowerhalf_s **lower = upper->lower;
      uint8_t i;

      for (i = 0; i < upper->nchan; i++)
        {
          ret = lower[i]->ops->start(lower[i]);
          if (ret < 0)
            {
              while (i-- > 0)
                {
                  lower[i]->ops->stop(lower[i]);
                }

              goto errout_with_lock;
            }
        }
    }

  /* Save the new open count on success */

  upper->crefs = tmp;
  ret = OK;

errout_with_lock:
  nxmutex_unlock(&upper->lock);

errout:
  return ret;
}

/****************************************************************************
 * Name: cap_close
 *
 * Description:
 *   This function is called when the PWM Capture device is closed.
 *
 ****************************************************************************/

static int cap_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct cap_upperhalf_s *upper = inode->i_private;
  int                         ret;

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 1)
    {
      upper->crefs--;
    }
  else
    {
      FAR struct cap_lowerhalf_s **lower = upper->lower;
      uint8_t i;

      /* There are no more references to the port */

      upper->crefs = 0;

      for (i = 0; i < upper->nchan; i++)
        {
          /* Disable the PWM Capture device */

          lower[i]->ops->stop(lower[i]);
        }
    }

  nxmutex_unlock(&upper->lock);
  ret = OK;

errout:
  return ret;
}

/****************************************************************************
 * Name: cap_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t cap_read(FAR struct file *filep,
                       FAR char *buffer,
                       size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: cap_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t cap_write(FAR struct file *filep,
                        FAR const char *buffer,
                        size_t buflen)
{
  /* Return a failure */

  return -EPERM;
}

/****************************************************************************
 * Name: cap_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *   This is where ALL of the PWM Capture work is done.
 *
 ****************************************************************************/

static int cap_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct cap_upperhalf_s *upper;
  FAR struct cap_lowerhalf_s **lower;
  int                        ret;
  uint8_t                    i;

  cpinfo("cmd: %d arg: %ld\n", cmd, arg);
  upper = inode->i_private;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
      /* CAPIOC_DUTYCYCLE - Get the pwm duty from the capture.
       * Argument: int8_t pointer to the location to return the duty.
       */

      case CAPIOC_DUTYCYCLE:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr);
          for (i = 0; i < upper->nchan; i++)
            {
              DEBUGASSERT(lower[i]->ops->getduty != NULL);
              ret = lower[i]->ops->getduty(lower[i], &ptr[i]);
              if (ret < 0)
                {
                  break;
                }
            }
        }
        break;

      /* CAPIOC_FREQUENCE - Get the pulse frequence from the capture.
       * Argument: int32_t pointer to the location to return the frequence.
       */

      case CAPIOC_FREQUENCE:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr);
          for (i = 0; i < upper->nchan; i++)
            {
              DEBUGASSERT(lower[i]->ops->getfreq != NULL);
              ret = lower[i]->ops->getfreq(lower[i], &ptr[i]);
              if (ret < 0)
                {
                  break;
                }
            }
        }
        break;

      /* CAPIOC_EDGES - Get the pwm edges from the capture.
       * Argument: int32_t pointer to the location to return the edges.
       */

      case CAPIOC_EDGES:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr);
          for (i = 0; i < upper->nchan; i++)
            {
              DEBUGASSERT(lower[i]->ops->getedges != NULL);
              ret = lower[i]->ops->getedges(lower[i], &ptr[i]);
              if (ret < 0)
                {
                  break;
                }
            }
        }
        break;

      /* CAPIOC_ALL - Get the pwm duty, pulse frequence, pwm edges, from
       * the capture.
       * Argument: A reference to struct cap_all_s.
       */

      case CAPIOC_ALL:
        {
          FAR struct cap_all_s *ptr =
                     (FAR struct cap_all_s *)((uintptr_t)arg);
          DEBUGASSERT(ptr);
          for (i = 0 ; i < upper->nchan ; i++)
            {
              ret = lower[i]->ops->getduty(lower[i], &ptr[i].duty);
              if (ret < 0)
                {
                  break;
                }

              ret = lower[i]->ops->getfreq(lower[i], &ptr[i].freq);
              if (ret < 0)
                {
                  break;
                }

              ret = lower[i]->ops->getedges(lower[i], &ptr[i].edges);
              if (ret < 0)
                {
                  break;
                }
            }
        }
        break;

      /* Any unrecognized IOCTL commands might be platform-specific ioctl
       * commands
       */

      default:
        {
          cperr("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
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
 * Name: cap_register
 *
 * Description:
 *   Register the PWM Capture lower half device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/cap0"
 *   lower - An instance of the lower half interface
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.  The following
 *   possible error values may be returned (most are returned by
 *   register_driver()):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int cap_register(FAR const char *devpath, FAR struct cap_lowerhalf_s *lower)
{
  FAR struct cap_upperhalf_s *upper;

  /* Allocate the upper-half data structure */

  upper = (FAR struct cap_upperhalf_s *)
           kmm_zalloc(sizeof(struct cap_upperhalf_s) +
                      sizeof(FAR struct cap_lowerhalf_s *));
  if (!upper)
    {
      return -ENOMEM;
    }

  /* Initialize the PWM Capture device structure
   * (it was already zeroed by kmm_zalloc())
   */

  nxmutex_init(&upper->lock);
  upper->lower = (FAR struct cap_lowerhalf_s **)(upper + 1);
  upper->lower[0] = lower;
  upper->nchan = 1;

  /* Register the PWM Capture device */

  return register_driver(devpath, &g_capops, 0666, upper);
}

int cap_register_multiple(FAR const char *devpath,
                          FAR struct cap_lowerhalf_s **lower, int n)
{
  FAR struct cap_upperhalf_s *upper;

  /* Allocate the upper-half data structure */

  upper = (FAR struct cap_upperhalf_s *)
           kmm_zalloc(sizeof(struct cap_upperhalf_s));
  if (!upper)
    {
      return -ENOMEM;
    }

  /* Initialize the PWM Capture device structure
   * (it was already zeroed by kmm_zalloc())
   */

  nxmutex_init(&upper->lock);
  upper->lower = lower;
  upper->nchan = n;

  /* Register the PWM Capture device */

  return register_driver(devpath, &g_capops, 0666, upper);
}

#endif /* CONFIG_CAPTURE */
