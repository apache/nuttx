/****************************************************************************
 * drivers/timers/pwm.c
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

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/timers/pwm.h>

#ifdef CONFIG_PWM

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct pwm_upperhalf_s
{
  uint8_t           crefs;          /* The number of times the device has
                                     * been opened */
  volatile bool     started;        /* True: pulsed output is being
                                     * generated */
  mutex_t           lock;           /* Supports mutual exclusion */
  struct pwm_info_s info;           /* Pulsed output characteristics */
  FAR struct pwm_lowerhalf_s *dev;  /* lower-half state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    pwm_dump(FAR const char *msg,
                        FAR const struct pwm_info_s *info,
                        bool started);
static int     pwm_open(FAR struct file *filep);
static int     pwm_close(FAR struct file *filep);
static ssize_t pwm_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t pwm_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen);
static int     pwm_start(FAR struct pwm_upperhalf_s *upper,
                         unsigned int oflags);
static int     pwm_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pwmops =
{
  pwm_open,  /* open */
  pwm_close, /* close */
  pwm_read,  /* read */
  pwm_write, /* write */
  NULL,      /* seek */
  pwm_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_dump
 ****************************************************************************/

static void pwm_dump(FAR const char *msg, FAR const struct pwm_info_s *info,
                     bool started)
{
  int i;

  pwminfo("%s: frequency: %" PRId32 "\n", msg, info->frequency);

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      pwminfo(" channel: %d duty: %08" PRIx32 "\n",
              info->channels[i].channel, info->channels[i].duty);
    }

  pwminfo(" started: %d\n", started);
}

/****************************************************************************
 * Name: pwm_open
 *
 * Description:
 *   This function is called whenever the PWM device is opened.
 *
 ****************************************************************************/

static int pwm_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct pwm_upperhalf_s *upper = inode->i_private;
  uint8_t                     tmp;
  int                         ret;

  pwminfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      goto errout;
    }

  /* Increment the count of references to the device.  If this the first
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
      FAR struct pwm_lowerhalf_s *lower = upper->dev;

      /* Yes.. perform one time hardware initialization. */

      DEBUGASSERT(lower->ops->setup != NULL);
      pwminfo("calling setup\n");

      ret = lower->ops->setup(lower);
      if (ret < 0)
        {
          goto errout_with_lock;
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
 * Name: pwm_close
 *
 * Description:
 *   This function is called when the PWM device is closed.
 *
 ****************************************************************************/

static int pwm_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct pwm_upperhalf_s *upper = inode->i_private;
  int                         ret;

  pwminfo("crefs: %d\n", upper->crefs);

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
      FAR struct pwm_lowerhalf_s *lower = upper->dev;

      /* There are no more references to the port */

      upper->crefs = 0;

      /* Disable the PWM device */

      DEBUGASSERT(lower->ops->shutdown != NULL);
      pwminfo("calling shutdown\n");

      lower->ops->shutdown(lower);
    }

  ret = OK;
  nxmutex_unlock(&upper->lock);

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t pwm_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: pwm_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t pwm_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   Handle the PWMIOC_START ioctl command
 *
 ****************************************************************************/

static int pwm_start(FAR struct pwm_upperhalf_s *upper, unsigned int oflags)
{
  FAR struct pwm_lowerhalf_s *lower;
  int ret = OK;

  UNUSED(oflags);

  DEBUGASSERT(upper != NULL);
  lower = upper->dev;
  DEBUGASSERT(lower != NULL && lower->ops->start != NULL);

  /* Verify that the PWM is not already running */

  if (!upper->started)
    {
      /* Invoke the bottom half method to start the pulse train */

      ret = lower->ops->start(lower, &upper->info);

      /* A return value of zero means that the pulse train was started
       * successfully.
       */

      if (ret == OK)
        {
          /* Indicate that the pulse train has started */

          upper->started = true;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the PWM work is done.
 *
 ****************************************************************************/

static int pwm_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct pwm_upperhalf_s *upper = inode->i_private;
  FAR struct pwm_lowerhalf_s *lower = upper->dev;
  int                         ret;

  pwminfo("cmd: %d arg: %ld\n", cmd, arg);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
      /* PWMIOC_SETCHARACTERISTICS - Set the characteristics of the next
       *   pulsed output and start the pulsed output. It will change the
       *   characteristics of the pulsed output on the fly if the timer is
       *   already started.
       *
       *   ioctl argument:  A read-only reference to struct pwm_info_s that
       *   provides the characteristics of the pulsed output.
       */

      case PWMIOC_SETCHARACTERISTICS:
        {
          FAR const struct pwm_info_s *info =
            (FAR const struct pwm_info_s *)((uintptr_t)arg);
          DEBUGASSERT(info != NULL && lower->ops->start != NULL);

          pwm_dump("PWMIOC_SETCHARACTERISTICS", info, upper->started);

          /* Save the pulse train characteristics */

          memcpy(&upper->info, info, sizeof(struct pwm_info_s));

          /* If PWM is already running, then re-start it with the new
           * characteristics.
           */

          if (upper->started)
            {
              ret = lower->ops->start(lower, &upper->info);
            }
        }
        break;

      /* PWMIOC_GETCHARACTERISTICS - Get the currently selected
       * characteristics of the pulsed output (independent of whether the
       * output is start or stopped).
       *
       *   ioctl argument:  A reference to struct pwm_info_s to receive the
       *   characteristics of the pulsed output.
       */

      case PWMIOC_GETCHARACTERISTICS:
        {
          FAR struct pwm_info_s *info =
            (FAR struct pwm_info_s *)((uintptr_t)arg);
          DEBUGASSERT(info != NULL);

          memcpy(info, &upper->info, sizeof(struct pwm_info_s));

          pwm_dump("PWMIOC_GETCHARACTERISTICS", info, upper->started);
        }
        break;

      /* PWMIOC_START - Start the pulsed output.  The
       *   PWMIOC_SETCHARACTERISTICS  command must have previously been sent.
       *
       *   ioctl argument:  None
       */

      case PWMIOC_START:
        {
          pwm_dump("PWMIOC_START", &upper->info, upper->started);
          DEBUGASSERT(lower->ops->start != NULL);

          /* Start the pulse train */

          ret = pwm_start(upper, filep->f_oflags);
        }
        break;

      /* PWMIOC_STOP - Stop the pulsed output.
       *
       *   ioctl argument:  None
       */

      case PWMIOC_STOP:
        {
          pwminfo("PWMIOC_STOP: started: %d\n", upper->started);
          DEBUGASSERT(lower->ops->stop != NULL);

          if (upper->started)
            {
              ret = lower->ops->stop(lower);
              upper->started = false;
            }
        }
        break;

      /* Any unrecognized IOCTL commands might be platform-specific ioctl
       * commands.
       */

      default:
        {
          pwminfo("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
          DEBUGASSERT(lower->ops->ioctl != NULL);
          ret = lower->ops->ioctl(lower, cmd, arg);
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
 * Name: pwm_register
 *
 * Description:
 *   This function binds an instance of a "lower half" timer driver with the
 *   "upper half" PWM device and registers that device so that can be used
 *   by application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   reset state (as if the shutdown() method had already been called).
 *
 * Input Parameters:
 *   path - The full path to the driver to be registered in the NuttX pseudo-
 *     filesystem.  The recommended convention is to name all PWM drivers
 *     as "/dev/pwm0", "/dev/pwm1", etc.  where the driver path differs only
 *     in the "minor" number at the end of the device name.
 *   dev - A pointer to an instance of lower half timer driver.  This
 *     instance is bound to the PWM driver and must persists as long as the
 *     driver persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pwm_register(FAR const char *path, FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct pwm_upperhalf_s *upper;

  /* Allocate the upper-half data structure */

  upper = (FAR struct pwm_upperhalf_s *)
    kmm_zalloc(sizeof(struct pwm_upperhalf_s));
  if (!upper)
    {
      pwmerr("Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the PWM device structure (it was already zeroed by
   * kmm_zalloc()).
   */

  nxmutex_init(&upper->lock);

  upper->dev = dev;

  /* Register the PWM device */

  pwminfo("Registering %s\n", path);
  return register_driver(path, &g_pwmops, 0666, upper);
}

#endif /* CONFIG_PWM */
