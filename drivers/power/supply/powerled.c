/****************************************************************************
 * drivers/power/supply/powerled.c
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

/* Upper-half, character driver for high power LED driver. */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/power/powerled.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int powerled_open(FAR struct file *filep);
static int powerled_close(FAR struct file *filep);
static int powerled_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations powerled_fops =
{
  powerled_open,                /* open */
  powerled_close,               /* close */
  NULL,                         /* read */
  NULL,                         /* write */
  NULL,                         /* seek */
  powerled_ioctl,               /* ioctl */
  NULL                          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: powerled_open
 *
 * Description:
 *   This function is called whenever the POWERLED device is opened.
 *
 ****************************************************************************/

static int powerled_open(FAR struct file *filep)
{
  FAR struct inode *inode        = filep->f_inode;
  FAR struct powerled_dev_s *dev = inode->i_private;
  uint8_t tmp;
  int ret;

  /* If the port is the middle of closing, wait until the close is finished */

  ret = nxmutex_lock(&dev->closelock);
  if (ret >= 0)
    {
      /* Increment the count of references to the device.  If this the first
       * time that the driver has been opened for this device, then
       * initialize the device.
       */

      tmp = dev->ocount + 1;
      if (tmp == 0)
        {
          /* More than 255 opens; uint8_t overflows to zero */

          ret = -EMFILE;
        }
      else
        {
          /* Check if this is the first time that the driver has been
           * opened.
           */

          if (tmp == 1)
            {
              /* Yes.. perform one time hardware initialization. */

              irqstate_t flags = enter_critical_section();
              ret = dev->ops->setup(dev);
              if (ret == OK)
                {
                  /* Save the new open count on success */

                  dev->ocount = tmp;
                }

              leave_critical_section(flags);
            }
        }

      nxmutex_unlock(&dev->closelock);
    }

  return OK;
}

/****************************************************************************
 * Name: powerled_close
 *
 * Description:
 *   This routine is called when the POWERLED device is closed.
 *   It waits for the last remaining data to be sent.
 *
 ****************************************************************************/

static int powerled_close(FAR struct file *filep)
{
  FAR struct inode *inode        = filep->f_inode;
  FAR struct powerled_dev_s *dev = inode->i_private;
  irqstate_t flags;
  int ret;

  ret = nxmutex_lock(&dev->closelock);
  if (ret >= 0)
    {
      /* Decrement the references to the driver.  If the reference count will
       * decrement to 0, then uninitialize the driver.
       */

      if (dev->ocount > 1)
        {
          dev->ocount--;
          nxmutex_unlock(&dev->closelock);
        }
      else
        {
          /* There are no more references to the port */

          dev->ocount = 0;

          /* Free the IRQ and disable the POWERLED device */

          flags = enter_critical_section();      /* Disable interrupts */
          dev->ops->shutdown(dev);               /* Disable the POWERLED */
          leave_critical_section(flags);

          nxmutex_unlock(&dev->closelock);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: powerled_ioctl
 ****************************************************************************/

static int powerled_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode    = filep->f_inode;
  FAR struct powerled_dev_s *dev = inode->i_private;
  FAR struct powerled_s *powerled    = (FAR struct powerled_s *)dev->priv;
  int ret;

  switch (cmd)
    {
      case PWRIOC_START:
        {
          /* Allow powerled start only when limits set and structure is
           * locked
           */

          if (powerled->limits.lock == false ||
              powerled->limits.current <= 0)
            {
              pwrerr("ERROR: powerled limits must be set"
                     " and locked before start\n");

              ret = -EPERM;
              goto errout;
            }

          /* Check powerled mode */

          if (powerled->opmode != POWERLED_OPMODE_CONTINUOUS &&
              powerled->opmode != POWERLED_OPMODE_FLASH)
            {
              pwrerr("ERROR: unsupported powerled mode!");

              ret = -EPERM;
              goto errout;
            }

          /* Finally, call start from lower-half driver */

          ret = dev->ops->start(dev);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_START failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_STOP:
        {
          ret = dev->ops->stop(dev);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_STOP failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_SET_MODE:
        {
          uint8_t mode = ((uint8_t)arg);

          ret = dev->ops->mode_set(dev, mode);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_SET_MODE failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_SET_LIMITS:
        {
          FAR struct powerled_limits_s *limits =
            (FAR struct powerled_limits_s *)((uintptr_t)arg);

          if (powerled->limits.lock == true)
            {
              pwrerr("ERROR: PWRRLED limits locked!\n");

              ret = -EPERM;
              goto errout;
            }

          /* NOTE: this call must set the powerled_limits_s structure */

          ret = dev->ops->limits_set(dev, limits);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_SET_LIMITS failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_GET_STATE:
        {
          FAR struct powerled_state_s *state =
            (FAR struct powerled_state_s *)((uintptr_t)arg);

          ret = dev->ops->state_get(dev, state);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_GET_STATE failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_SET_FAULT:
        {
          uint8_t fault = ((uint8_t)arg);

          ret = dev->ops->fault_set(dev, fault);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_SET_FAULT failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_GET_FAULT:
        {
          FAR uint8_t *fault = ((FAR uint8_t *)arg);

          ret = dev->ops->fault_get(dev, fault);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_GET_FAULT failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_CLEAN_FAULT:
        {
          uint8_t fault = ((uint8_t)arg);

          ret = dev->ops->fault_clean(dev, fault);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_CLEAN_FAULT failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_SET_PARAMS:
        {
          FAR struct powerled_params_s *params =
            (FAR struct powerled_params_s *)((uintptr_t)arg);

          if (powerled->param.lock == true)
            {
              pwrerr("ERROR: powerled params locked!\n");

              ret = -EPERM;
              goto errout;
            }

          if (params->brightness < 0.0 || params->brightness > 100.0 ||
              params->frequency < 0.0 || params->duty < 0.0 ||
              params->duty > 100.0)
            {
              pwrerr("ERROR: powerled invalid parameters %f %f %f\n",
                     params->brightness, params->frequency, params->duty);

              ret = -EPERM;
              goto errout;
            }

          ret = dev->ops->params_set(dev, params);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_SET_PARAMS failed %d\n", ret);
            }
          break;
        }

      default:
        {
          pwrinfo("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
          ret = dev->ops->ioctl(dev, cmd, arg);
          break;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: powerled_register
 ****************************************************************************/

int powerled_register(FAR const char *path, FAR struct powerled_dev_s *dev,
                      FAR void *lower)
{
  int ret;

  DEBUGASSERT(path != NULL && dev != NULL && lower != NULL);
  DEBUGASSERT(dev->ops != NULL);

  /* For safety reason, when some necessary low-level logic is not provided,
   * system should fail before low-level hardware initialization, so:
   *   - all ops are checked here, before character driver registration
   *   - all ops must be provided, even if not used
   */

  DEBUGASSERT(dev->ops->setup != NULL);
  DEBUGASSERT(dev->ops->shutdown != NULL);
  DEBUGASSERT(dev->ops->stop != NULL);
  DEBUGASSERT(dev->ops->start != NULL);
  DEBUGASSERT(dev->ops->params_set != NULL);
  DEBUGASSERT(dev->ops->mode_set != NULL);
  DEBUGASSERT(dev->ops->limits_set != NULL);
  DEBUGASSERT(dev->ops->fault_set != NULL);
  DEBUGASSERT(dev->ops->state_get != NULL);
  DEBUGASSERT(dev->ops->fault_get != NULL);
  DEBUGASSERT(dev->ops->fault_clean != NULL);
  DEBUGASSERT(dev->ops->ioctl != NULL);

  /* Initialize the powerled device structure */

  dev->ocount = 0;

  /* Initialize mutex */

  nxmutex_init(&dev->closelock);

  /* Connect POWERLED driver with lower level interface */

  dev->lower = lower;

  /* Register the POWERLED character driver */

  ret = register_driver(path, &powerled_fops, 0666, dev);
  if (ret < 0)
    {
      nxmutex_destroy(&dev->closelock);
    }

  return ret;
}
