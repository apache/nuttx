/****************************************************************************
 * drivers/motor/foc/foc_dev.c
 * Upper-half FOC controller logic
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

#include <stdio.h>
#include <fcntl.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/motor/motor_ioctl.h>
#include <nuttx/motor/foc/foc_lower.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int foc_open(FAR struct file *filep);
static int foc_close(FAR struct file *filep);
static int foc_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int foc_lower_ops_assert(FAR struct foc_lower_ops_s *ops);

static int foc_setup(FAR struct foc_dev_s *dev);
static int foc_shutdown(FAR struct foc_dev_s *dev);
static int foc_stop(FAR struct foc_dev_s *dev);
static int foc_start(FAR struct foc_dev_s *dev);
static int foc_cfg_set(FAR struct foc_dev_s *dev, FAR struct foc_cfg_s *cfg);
static int foc_state_get(FAR struct foc_dev_s *dev,
                         FAR struct foc_state_s *state);
static int foc_params_set(FAR struct foc_dev_s *dev,
                          FAR struct foc_params_s *params);
static int foc_fault_clear(FAR struct foc_dev_s *dev);
static int foc_info_get(FAR struct foc_dev_s *dev,
                        FAR struct foc_info_s *info);

static int foc_notifier(FAR struct foc_dev_s *dev,
                        FAR foc_current_t *current);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* File operations */

static const struct file_operations g_foc_fops =
{
  foc_open,                     /* open */
  foc_close,                    /* close */
  NULL,                         /* read */
  NULL,                         /* write */
  NULL,                         /* seek */
  foc_ioctl,                    /* ioctl */
  NULL                          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                        /* unlink */
#endif
};

/* FOC callbacks from the lower-half implementation to this driver */

static struct foc_callbacks_s g_foc_callbacks =
{
  foc_notifier
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_open
 *
 * Description:
 *   This function is called whenever the foc device is opened.
 *
 ****************************************************************************/

static int foc_open(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct foc_dev_s *dev   = inode->i_private;
  uint8_t               tmp   = 0;
  int                   ret   = OK;

  /* Non-blocking operations not supported */

  if (filep->f_oflags & O_NONBLOCK)
    {
      ret = -EPERM;
      goto errout;
    }

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
          /* Check if this is the first time that the driver has been opened
           */

          if (tmp == 1)
            {
              ret = foc_setup(dev);
              if (ret == OK)
                {
                  /* Save the new open count on success */

                  dev->ocount = tmp;
                }
            }
          else
            {
              /* Save the incremented open count */

              dev->ocount = tmp;
            }
        }

      nxmutex_unlock(&dev->closelock);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_close
 *
 * Description:
 *   This routine is called when the foc device is closed.
 *
 ****************************************************************************/

static int foc_close(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct foc_dev_s *dev   = inode->i_private;
  int                   ret   = 0;

  ret = nxmutex_lock(&dev->closelock);
  if (ret >= 0)
    {
      /* Decrement the references to the driver. If the reference count will
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

          /* Shutdown the device */

          ret = foc_shutdown(dev);
          nxmutex_unlock(&dev->closelock);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: foc_ioctl
 *
 * Description:
 *   Supported IOCTLs:
 *
 *   MTRIOC_START:        Start the FOC device,
 *                        arg: none
 *
 *   MTRIOC_STOP:         Stop the FOC device,
 *                        arg: none
 *
 *   MTRIOC_GET_STATE:    Get the FOC device state,
 *                        arg: struct foc_state_s pointer
 *                        This is a blocking operation that is used to
 *                        synchronize the user space application with
 *                        a FOC worker.
 *
 *   MTRIOC_CLEAR_FAULT: Clear the FOC device fault state,
 *                        arg: none
 *
 *   MTRIOC_SET_PARAMS:   Set the FOC device operation parameters,
 *                        arg: struct foc_params_s pointer
 *
 *   MTRIOC_SET_CONFIG:   Set the FOC device configuration,
 *                        arg: struct foc_cfg_s pointer
 *
 *   MTRIOC_GET_INFO:     Get the FOC device info,
 *                        arg: struct foc_info_s pointer
 *
 ****************************************************************************/

static int foc_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct foc_dev_s *dev   = inode->i_private;
  int                   ret   = 0;

  switch (cmd)
    {
      /* Start the FOC device */

      case MTRIOC_START:
        {
          ret = foc_start(dev);
          if (ret != OK)
            {
              mtrerr("MTRIOC_START failed %d\n", ret);
            }

          break;
        }

      /* Stop the FOC device */

      case MTRIOC_STOP:
        {
          ret = foc_stop(dev);
          if (ret != OK)
            {
              mtrerr("MTRIOC_STOP failed %d\n", ret);
            }

          break;
        }

      /* Get device state */

      case MTRIOC_GET_STATE:
        {
          FAR struct foc_state_s *state = (FAR struct foc_state_s *)arg;

          DEBUGASSERT(state != NULL);

          ret = foc_state_get(dev, state);
          if (ret != OK)
            {
              mtrerr("MTRIOC_GET_STATE failed %d\n", ret);
            }

          break;
        }

      /* Clear fault state */

      case MTRIOC_CLEAR_FAULT:
        {
          DEBUGASSERT(arg == 0);

          ret = foc_fault_clear(dev);
          if (ret != OK)
            {
              mtrerr("MTRIOC_CLEAR_FAULT failed %d\n", ret);
            }

          break;
        }

      /* Set device parameters */

      case MTRIOC_SET_PARAMS:
        {
          FAR struct foc_params_s *params = (FAR struct foc_params_s *)arg;

          DEBUGASSERT(params != NULL);

          ret = foc_params_set(dev, params);
          if (ret != OK)
            {
              mtrerr("MTRIOC_SET_PARAMS failed %d\n", ret);
            }

          break;
        }

      /* Set the device configuration */

      case MTRIOC_SET_CONFIG:
        {
          FAR struct foc_cfg_s *cfg = (FAR struct foc_cfg_s *)arg;

          DEBUGASSERT(cfg != NULL);

          ret = foc_cfg_set(dev, cfg);
          if (ret != OK)
            {
              mtrerr("MTRIOC_SET_CONFIG failed %d\n", ret);
            }

          break;
        }

      /* Get the FOC device info */

      case MTRIOC_GET_INFO:
        {
          FAR struct foc_info_s *info = (struct foc_info_s *)arg;

          DEBUGASSERT(info != NULL);

          ret = foc_info_get(dev, info);
          if (ret != OK)
            {
              mtrerr("MTRIOC_GET_INFO failed %d\n", ret);
            }

          break;
        }

      /* Not supported */

      default:
        {
          mtrinfo("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);

          /* Call lower-half logic */

          ret = FOC_OPS_IOCTL(dev, cmd, arg);

          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: foc_lower_ops_assert
 *
 * Description:
 *   Assert the lower-half FOC operations
 *
 ****************************************************************************/

static int foc_lower_ops_assert(FAR struct foc_lower_ops_s *ops)
{
  DEBUGASSERT(ops->configure);
  DEBUGASSERT(ops->setup);
  DEBUGASSERT(ops->shutdown);
  DEBUGASSERT(ops->start);
  DEBUGASSERT(ops->ioctl);
  DEBUGASSERT(ops->bind);
  DEBUGASSERT(ops->fault_clear);
#ifdef CONFIG_MOTOR_FOC_TRACE
  DEBUGASSERT(ops->trace);
#endif

  UNUSED(ops);

  return OK;
}

/****************************************************************************
 * Name: foc_lower_bind
 *
 * Description:
 *   Bind the upper-half with the lower-half FOC logic
 *
 ****************************************************************************/

static int foc_lower_bind(FAR struct foc_dev_s *dev)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(g_foc_callbacks.notifier);

  return FOC_OPS_BIND(dev, &g_foc_callbacks);
}

/****************************************************************************
 * Name: foc_setup
 *
 * Description:
 *   Setup the FOC device
 *
 ****************************************************************************/

static int foc_setup(FAR struct foc_dev_s *dev)
{
  int ret = OK;

  DEBUGASSERT(dev);

  mtrinfo("FOC SETUP\n");

  /* Reset device data */

  memset(&dev->cfg, 0, sizeof(struct foc_cfg_s));
  memset(&dev->state, 0, sizeof(struct foc_state_s));

  /* Bind the upper-half with the lower-half FOC logic */

  ret = foc_lower_bind(dev);
  if (ret < 0)
    {
      mtrerr("foc_lower_bind failed %d\n", ret);
      goto errout;
    }

  /* Call lower-half setup */

  ret = FOC_OPS_SETUP(dev);
  if (ret < 0)
    {
      mtrerr("FOC_OPS_SETUP failed %d\n", ret);
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_shutdown
 *
 * Description:
 *   Shutdown the FOC device
 *
 ****************************************************************************/

int foc_shutdown(FAR struct foc_dev_s *dev)
{
  int ret = OK;

  DEBUGASSERT(dev);

  mtrinfo("FOC SHUTDOWN\n");

  /* Call the lower-half shutdown */

  ret = FOC_OPS_SHUTDOWN(dev);

  return ret;
}

/****************************************************************************
 * Name: foc_start
 *
 * Description:
 *   Start the FOC device
 *
 ****************************************************************************/

static int foc_start(FAR struct foc_dev_s *dev)
{
  int ret = OK;

  DEBUGASSERT(dev);

  mtrinfo("FOC START\n");

  /* Reset the notifier semaphore */

  ret = nxsem_reset(&dev->statesem, 0);
  if (ret < 0)
    {
      mtrerr("nxsem_reset failed %d\n", ret);
      goto errout;
    }

  /* Start the FOC */

  ret = FOC_OPS_START(dev, true);
  if (ret < 0)
    {
      mtrerr("FOC_OPS_START failed %d !\n", ret);
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_stop
 *
 * Description:
 *   Stop the FOC device
 *
 ****************************************************************************/

static int foc_stop(FAR struct foc_dev_s *dev)
{
  foc_duty_t d_zero[CONFIG_MOTOR_FOC_PHASES];
  int        ret = OK;

  DEBUGASSERT(dev);

  mtrinfo("FOC STOP\n");

  /* Zero duty cycle */

  memset(&d_zero, 0, CONFIG_MOTOR_FOC_PHASES * sizeof(foc_duty_t));

  /* Reset duty cycle */

  ret = FOC_OPS_DUTY(dev, d_zero);
  if (ret < 0)
    {
      mtrerr("FOC_OPS_DUTY failed %d\n", ret);
    }

  /* Stop the FOC */

  ret = FOC_OPS_START(dev, false);
  if (ret < 0)
    {
      mtrerr("FOC_OPS_START failed %d\n", ret);
    }

  /* Reset device data */

  memset(&dev->state, 0, sizeof(struct foc_state_s));

  return ret;
}

/****************************************************************************
 * Name: foc_cfg_set
 *
 * Description:
 *   Set the FOC device configuration
 *
 ****************************************************************************/

static int foc_cfg_set(FAR struct foc_dev_s *dev, FAR struct foc_cfg_s *cfg)
{
  int ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(cfg);

  DEBUGASSERT(cfg->pwm_freq > 0);
  DEBUGASSERT(cfg->notifier_freq > 0);

  /* Copy common configuration */

  memcpy(&dev->cfg, cfg, sizeof(struct foc_cfg_s));

  mtrinfo("FOC PWM=%" PRIu32 " notifier=%" PRIu32 "\n",
          dev->cfg.pwm_freq, dev->cfg.notifier_freq);

  /* Call arch configuration */

  ret = FOC_OPS_CONFIGURE(dev, &dev->cfg);
  if (ret < 0)
    {
      mtrerr("FOC_OPS_CONFIGURE failed %d\n", ret);
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_state_get
 *
 * Description:
 *   Get the FOC device state
 *
 ****************************************************************************/

static int foc_state_get(FAR struct foc_dev_s *dev,
                         FAR struct foc_state_s *state)
{
  int ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(state);

  /* Signal trace */

#ifdef CONFIG_MOTOR_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_STATE, true);
#endif

  /* Wait for notification if blocking */

  ret = nxsem_wait_uninterruptible(&dev->statesem);
  if (ret < 0)
    {
      goto errout;
    }

#ifdef CONFIG_MOTOR_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_STATE, false);
#endif

  /* Copy state */

  memcpy(state, &dev->state, sizeof(struct foc_state_s));

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_fault_clear
 *
 * Description:
 *   Clear the FOC device fault state
 *
 ****************************************************************************/

static int foc_fault_clear(FAR struct foc_dev_s *dev)
{
  int ret = OK;

  /* Call lower-half logic */

  ret = FOC_OPS_FAULT_CLEAR(dev);
  if (ret < 0)
    {
      mtrerr("FOC_OPS_FAULT_CLEAR failed %d\n", ret);
      goto errout;
    }

  /* Clear all faults */

  dev->state.fault = FOC_FAULT_NONE;

errout:
  return ret;
}

/****************************************************************************
 * Name: foc_params_set
 *
 * Description:
 *   Set the FOC device parameters
 *
 ****************************************************************************/

static int foc_params_set(FAR struct foc_dev_s *dev,
                          FAR struct foc_params_s *params)
{
  int ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(params);

#ifdef CONFIG_MOTOR_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_PARAMS, true);
#endif

  /* Set new duty */

  ret = FOC_OPS_DUTY(dev, params->duty);

#ifdef CONFIG_MOTOR_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_PARAMS, false);
#endif

  return ret;
}

/****************************************************************************
 * Name: foc_info_get
 *
 * Description:
 *   Get the FOC device info
 *
 ****************************************************************************/

static int foc_info_get(FAR struct foc_dev_s *dev,
                        FAR struct foc_info_s *info)
{
  /* Copy data from device */

  memcpy(info, &dev->info, sizeof(struct foc_info_s));

  return OK;
}

/****************************************************************************
 * Name: foc_notifier
 *
 * Description:
 *   Notify the user-space and provide the phase current samples
 *
 ****************************************************************************/

static int foc_notifier(FAR struct foc_dev_s *dev,
                        FAR foc_current_t *current)
{
  int ret  = OK;
  int sval = 0;

  DEBUGASSERT(dev != NULL);

#ifdef CONFIG_MOTOR_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_NOTIFIER, true);
#endif

  /* Copy currents */

  memcpy(&dev->state.curr,
         current,
         sizeof(foc_current_t) * CONFIG_MOTOR_FOC_PHASES);

  /* Check if the previous cycle was handled */

  ret = nxsem_get_value(&dev->statesem, &sval);
  if (ret != OK)
    {
      ret = -EINVAL;
    }
  else
    {
      if (sval < -dev->ocount)
        {
          /* This is a critical fault */

          DEBUGPANIC();

          /* Set timeout fault if not in debug mode */

          dev->state.fault |= FOC_FAULT_TIMEOUT;

          /* Reset semaphore */

          nxsem_reset(&dev->statesem, 0);
        }
      else
        {
          /* Loop until all of the waiting threads have been restarted. */

          while (sval < 0)
            {
              /* Post semaphore */

              nxsem_post(&dev->statesem);

              /* Increment the semaphore count (as was done by the
               * above post).
               */

              sval += 1;
            }
        }
    }

#ifdef CONFIG_MOTOR_FOC_TRACE
  FOC_OPS_TRACE(dev, FOC_TRACE_NOTIFIER, false);
#endif

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foc_register
 *
 * Description:
 *   Register the FOC character device as 'path'
 *
 * Input Parameters:
 *   path  - The full path to the driver to register
 *   dev   - An instance of the FOC device
 *
 ****************************************************************************/

int foc_register(FAR const char *path, FAR struct foc_dev_s *dev)
{
  int ret = OK;

  DEBUGASSERT(path != NULL);
  DEBUGASSERT(dev != NULL);

  /* Lower-half must be initialized */

  DEBUGASSERT(dev->lower);
  DEBUGASSERT(dev->lower->ops);
  DEBUGASSERT(dev->lower->data);

  /* Reset counter */

  dev->ocount = 0;

  /* Assert the lower-half interface */

  ret = foc_lower_ops_assert(dev->lower->ops);
  if (ret < 0)
    {
      goto errout;
    }

  /* Initialize mutex & semaphores */

  nxmutex_init(&dev->closelock);
  nxsem_init(&dev->statesem, 0, 0);

  /* Register the FOC character driver */

  ret = register_driver(path, &g_foc_fops, 0666, dev);
  if (ret < 0)
    {
      nxmutex_destroy(&dev->closelock);
      nxsem_destroy(&dev->statesem);
      goto errout;
    }

errout:
  return ret;
}
