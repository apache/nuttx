/****************************************************************************
 * drivers/motor/stepper.c
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

/* Upper-half, character driver for stepper control */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/motor/stepper.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct stepper_upperhalf_s
{
  FAR struct stepper_lowerhalf_s *lower; /* lower half driver */
  int refs;                              /* Reference count */
  mutex_t lock;                          /* Only one thread can access at a time */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     stepper_open(FAR struct file *filep);
static int     stepper_close(FAR struct file *filep);
static ssize_t stepper_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t stepper_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     stepper_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_stepper_fops =
{
  stepper_open,   /* open */
  stepper_close,  /* close */
  stepper_read,   /* read */
  stepper_write,  /* write */
  NULL,           /* seek */
  stepper_ioctl,  /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stepper_open
 *
 * Description:
 *   This function is called whenever the stepper device is opened.
 *
 ****************************************************************************/

static int stepper_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct stepper_upperhalf_s *stepper = inode->i_private;
  FAR struct stepper_lowerhalf_s *lower = stepper->lower;
  int ret;

  ret = nxmutex_lock(&stepper->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then
   * initialize the device.
   */

  stepper->refs++;
  if (stepper->refs == 1)
    {
      ret = lower->ops->setup(lower);
      if (ret < 0)
        {
          stepper->refs--;  /* Something bad happened: open failure */
        }
    }

  nxmutex_unlock(&stepper->lock);
  return ret;
}

/****************************************************************************
 * Name: stepper_close
 *
 * Description:
 *   This routine is called when the stepper device is closed.
 *
 ****************************************************************************/

static int stepper_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct stepper_upperhalf_s *stepper = inode->i_private;
  FAR struct stepper_lowerhalf_s *lower = stepper->lower;
  int ret;

  ret = nxmutex_lock(&stepper->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  stepper->refs--;
  if (stepper->refs == 0)
    {
      ret = lower->ops->shutdown(lower); /* Disable the stepper */
    }

  nxmutex_unlock(&stepper->lock);

  return ret;
}

/****************************************************************************
 * Name: stepper_read
 ****************************************************************************/

static ssize_t stepper_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct stepper_upperhalf_s *stepper = inode->i_private;
  FAR struct stepper_lowerhalf_s *lower = stepper->lower;
  FAR struct stepper_status_s *status;
  int ret;

  if (buflen != sizeof(struct stepper_status_s))
    {
      return -EINVAL;
    }

  status = (FAR struct stepper_status_s *)buffer;
  ret = lower->ops->update_status(lower);
  if (ret < 0)
    {
      stperr("Update stepper state failed: %d\n", ret);
      return ret;
    }

  *status = lower->status;
  return sizeof(struct stepper_status_s);
}

/****************************************************************************
 * Name: stepper_write
 ****************************************************************************/

static ssize_t stepper_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct stepper_upperhalf_s *stepper = inode->i_private;
  FAR struct stepper_lowerhalf_s *lower = stepper->lower;
  FAR struct stepper_job_s const *job;
  int ret;

  if (buflen != sizeof(struct stepper_job_s))
    {
      return -EINVAL;
    }

  job = (FAR struct stepper_job_s const *)buffer;
  ret = lower->ops->work(lower, job);
  if (ret < 0)
    {
      stperr("Stepper work failed: %d\n", ret);
      return ret;
    }

  return sizeof(struct stepper_job_s);
}

/****************************************************************************
 * Name: stepper_ioctl
 ****************************************************************************/

static int stepper_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct stepper_upperhalf_s *stepper = inode->i_private;
  FAR struct stepper_lowerhalf_s *lower = stepper->lower;
  int ret = 0;

  switch (cmd)
    {
      case STEPIOC_IDLE:
        {
          uint8_t idle = (uint8_t)arg;

          ret = lower->ops->idle(lower, idle);
          if (ret < 0)
            {
              stperr("STEPIOC_IDLE failed: %d\n", ret);
            }
        }
        break;

      case STEPIOC_CLEAR_FAULT:
        {
          uint8_t fault = (uint8_t)arg;

          ret = lower->ops->clear(lower, fault);
          if (ret < 0)
            {
              stperr("STEPIOC_CLEAR_FAULT failed: %d\n", ret);
            }
        }
        break;

      case STEPIOC_MICROSTEPPING:
        {
          uint16_t resolution = (uint16_t)arg;

          ret = lower->ops->microstepping(lower, resolution);
        }
        break;

      case STEPIOC_SET_CURRENT_POS:
        {
          lower->status.position = (int32_t)arg;
          ret = 0;
          stpinfo("STEPIOC_SET_CURRENT_POS: new position is %ld\n",
                  lower->status.position);
        }
        break;

      default:
        {
          ret = lower->ops->ioctl(lower, cmd, arg);
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stepper_register
 *
 * Description:
 *   This function binds an instance of a "lower half" stepper driver with
 *   the "upper half" stepper device and registers that device so that can
 *   be used by application code.
 *
 *   We will register the character device with specified path.
 *
 * Input Parameters:
 *   path  - The user specifies path name.
 *   lower - A pointer to an instance of lower half stepper driver. This
 *           instance is bound to the stepper driver and must persists
 *           as long as the driver persists.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int stepper_register(FAR const char *path,
                     FAR struct stepper_lowerhalf_s *lower)
{
  FAR struct stepper_upperhalf_s *stepper;
  int ret = 0;

  /* Sanity check */

  DEBUGASSERT(lower != NULL);

  /* Initialize the upper-half data structure */

  stepper = kmm_zalloc(sizeof(struct stepper_upperhalf_s));
  if (stepper == NULL)
    {
      stperr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Initialize mutex */

  nxmutex_init(&stepper->lock);

  /* Connect stepper driver with lower level interface */

  stepper->lower = lower;
  stepper->refs = 0;

  /* Register the stepper character driver */

  ret = register_driver(path, &g_stepper_fops, 0666, stepper);
  if (ret < 0)
    {
      nxmutex_destroy(&stepper->lock);
      kmm_free(stepper);
    }

  return ret;
}
