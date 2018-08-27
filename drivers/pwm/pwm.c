/****************************************************************************
 * drivers/pwm/pwm.c
 *
 *   Copyright (C) 2011-2013, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/pwm.h>

#include <nuttx/irq.h>

#ifdef CONFIG_PWM

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct pwm_upperhalf_s
{
  uint8_t           crefs;    /* The number of times the device has been opened */
  volatile bool     started;  /* True: pulsed output is being generated */
#ifdef CONFIG_PWM_PULSECOUNT
  volatile bool     waiting;  /* True: Caller is waiting for the pulse count to expire */
#endif
  sem_t             exclsem;  /* Supports mutual exclusion */
#ifdef CONFIG_PWM_PULSECOUNT
  sem_t             waitsem;  /* Used to wait for the pulse count to expire */
#endif
  struct pwm_info_s info;     /* Pulsed output characteristics */
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
  0,         /* seek */
  pwm_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0        /* poll */
#endif
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
#ifdef CONFIG_PWM_MULTICHAN
  int i;
#endif

  pwminfo("%s: frequency: %d", msg, info->frequency);

#ifdef CONFIG_PWM_MULTICHAN
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      pwminfo(" channel: %d duty: %08x",
              info->channels[i].channel, info->channels[i].duty);
    }
#else
  pwminfo(" duty: %08x", info->duty);
#endif

#ifdef CONFIG_PWM_PULSECOUNT
  pwminfo(" count: %d\n", info->count);
#endif

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

  ret = nxsem_wait(&upper->exclsem);
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
      goto errout_with_sem;
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
          goto errout_with_sem;
        }
    }

  /* Save the new open count on success */

  upper->crefs = tmp;
  ret = OK;

errout_with_sem:
  nxsem_post(&upper->exclsem);

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

  ret = nxsem_wait(&upper->exclsem);
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
      pwminfo("calling shutdown: %d\n");

      lower->ops->shutdown(lower);
    }

  ret = OK;
  nxsem_post(&upper->exclsem);

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

#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_start(FAR struct pwm_upperhalf_s *upper, unsigned int oflags)
{
  FAR struct pwm_lowerhalf_s *lower;
  irqstate_t flags;
  int ret = OK;

  DEBUGASSERT(upper != NULL);
  lower = upper->dev;
  DEBUGASSERT(lower != NULL && lower->ops->start != NULL);

  /* Verify that the PWM is not already running */

  if (!upper->started)
    {
      /* Disable interrupts to avoid race conditions */

      flags = enter_critical_section();

      /* Indicate that if will be waiting for the pulse count to complete.
       * Note that we will only wait if a non-zero pulse count is specified
       * and if the PWM driver was opened in normal, blocking mode.  Also
       * assume for now that the pulse train will be successfully started.
       *
       * We do these things before starting the PWM to avoid race conditions.
       */

      upper->waiting = (upper->info.count > 0) && ((oflags & O_NONBLOCK) == 0);
      upper->started = true;

      /* Invoke the bottom half method to start the pulse train */

      ret = lower->ops->start(lower, &upper->info, upper);

      /* A return value of zero means that the pulse train was started
       * successfully.
       */

      if (ret == OK)
        {
          /* Should we wait for the pulse output to complete?  Loop in
           * in case the wakeup form nxsem_wait() is a false alarm.
           */

          while (upper->waiting)
            {
              /* Wait until we are awakened by pwm_expired().  When
               * pwm_expired is called, it will post the waitsem and
               * clear the waiting flag.
               */

              int tmp = nxsem_wait(&upper->waitsem);
              DEBUGASSERT(tmp == OK || tmp == -EINTR);
            }
        }
      else
        {
          /* Looks like we won't be waiting after all */

          pwminfo("start failed: %d\n", ret);
          upper->started = false;
          upper->waiting = false;
        }

      leave_critical_section(flags);
    }

  return ret;
}
#else
static int pwm_start(FAR struct pwm_upperhalf_s *upper, unsigned int oflags)
{
  FAR struct pwm_lowerhalf_s *lower;
  int ret = OK;

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
#endif

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

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
      /* PWMIOC_SETCHARACTERISTICS - Set the characteristics of the next pulsed
       *   output.  This command will neither start nor stop the pulsed output.
       *   It will either setup the configuration that will be used when the
       *   output is started; or it will change the characteristics of the pulsed
       *   output on the fly if the timer is already started.
       *
       *   ioctl argument:  A read-only reference to struct pwm_info_s that provides
       *   the characteristics of the pulsed output.
       */

      case PWMIOC_SETCHARACTERISTICS:
        {
          FAR const struct pwm_info_s *info = (FAR const struct pwm_info_s *)((uintptr_t)arg);
          DEBUGASSERT(info != NULL && lower->ops->start != NULL);

          pwm_dump("PWMIOC_SETCHARACTERISTICS", info, upper->started);

          /* Save the pulse train characteristics */

          memcpy(&upper->info, info, sizeof(struct pwm_info_s));

          /* If PWM is already running, then re-start it with the new characteristics */

          if (upper->started)
            {
#ifdef CONFIG_PWM_PULSECOUNT
              ret = lower->ops->start(lower, &upper->info, upper);
#else
              ret = lower->ops->start(lower, &upper->info);
#endif
            }
        }
        break;

      /* PWMIOC_GETCHARACTERISTICS - Get the currently selected characteristics of
       *   the pulsed output (independent of whether the output is start or stopped).
       *
       *   ioctl argument:  A reference to struct pwm_info_s to receive the
       *   characteristics of the pulsed output.
       */

      case PWMIOC_GETCHARACTERISTICS:
        {
          FAR struct pwm_info_s *info = (FAR struct pwm_info_s *)((uintptr_t)arg);
          DEBUGASSERT(info != NULL);

          memcpy(info, &upper->info, sizeof(struct pwm_info_s));

          pwm_dump("PWMIOC_GETCHARACTERISTICS", info, upper->started);
        }
        break;

      /* PWMIOC_START - Start the pulsed output.  The PWMIOC_SETCHARACTERISTICS
       *   command must have previously been sent.
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
#ifdef CONFIG_PWM_PULSECOUNT
              if (upper->waiting)
                {
                  upper->waiting = false;
                }
#endif
            }
        }
        break;

      /* Any unrecognized IOCTL commands might be platform-specific ioctl commands */

      default:
        {
          pwminfo("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
          DEBUGASSERT(lower->ops->ioctl != NULL);
          ret = lower->ops->ioctl(lower, cmd, arg);
        }
        break;
    }

  nxsem_post(&upper->exclsem);
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
 *   dev - A pointer to an instance of lower half timer driver.  This instance
 *     is bound to the PWM driver and must persists as long as the driver
 *     persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pwm_register(FAR const char *path, FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct pwm_upperhalf_s *upper;

  /* Allocate the upper-half data structure */

  upper = (FAR struct pwm_upperhalf_s *)kmm_zalloc(sizeof(struct pwm_upperhalf_s));
  if (!upper)
    {
      pwmerr("Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the PWM device structure (it was already zeroed by kmm_zalloc()) */

  nxsem_init(&upper->exclsem, 0, 1);
#ifdef CONFIG_PWM_PULSECOUNT
  nxsem_init(&upper->waitsem, 0, 0);

  /* The wait semaphore is used for signaling and, hence, should not have priority
   * inheritance enabled.
   */

  nxsem_setprotocol(&upper->waitsem, SEM_PRIO_NONE);
#endif

  upper->dev = dev;

  /* Register the PWM device */

  pwminfo("Registering %s\n", path);
  return register_driver(path, &g_pwmops, 0666, upper);
}

/****************************************************************************
 * Name: pwm_expired
 *
 * Description:
 *   If CONFIG_PWM_PULSECOUNT is defined and the pulse count was configured
 *   to a non-zero value, then the "upper half" driver will wait for the
 *   pulse count to expire.  The sequence of expected events is as follows:
 *
 *   1. The upper half driver calls the start method, providing the lower
 *      half driver with the pulse train characteristics.  If a fixed
 *      number of pulses is required, the 'count' value will be nonzero.
 *   2. The lower half driver's start() methoc must verify that it can
 *      support the request pulse train (frequency, duty, AND pulse count).
 *      If it cannot, it should return an error.  If the pulse count is
 *      non-zero, it should set up the hardware for that number of pulses
 *      and return success.  NOTE:  That is CONFIG_PWM_PULSECOUNT is
 *      defined, the start() method receives an additional parameter
 *      that must be used in this callback.
 *   3. When the start() method returns success, the upper half driver
 *      will "sleep" until the pwm_expired method is called.
 *   4. When the lower half detects that the pulse count has expired
 *      (probably through an interrupt), it must call the pwm_expired
 *      interface using the handle that was previously passed to the
 *      start() method
 *
 * Input Parameters:
 *   handle - This is the handle that was provided to the lower-half
 *     start() method.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM_PULSECOUNT
void pwm_expired(FAR void *handle)
{
  FAR struct pwm_upperhalf_s *upper = (FAR struct pwm_upperhalf_s *)handle;

  pwminfo("started: %d waiting: %d\n", upper->started, upper->waiting);

  /* Make sure that the PWM is started */

  if (upper->started)
    {
      /* Is there a thread waiting for the pulse train to complete? */

      if (upper->waiting)
        {
          /* Yes.. clear the waiting flag and awakened the waiting thread */

          upper->waiting = false;
          nxsem_post(&upper->waitsem);
        }

      /* The PWM is now stopped */

      upper->started = false;
    }
}
#endif

#endif /* CONFIG_PWM */
