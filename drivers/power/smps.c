/****************************************************************************
 * drivers/power/smps.c
 * Upper-half, character driver for SMPS (switched-mode power supply)
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
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/power/smps.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     smps_open(FAR struct file *filep);
static int     smps_close(FAR struct file *filep);
static ssize_t smps_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t smps_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static int     smps_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations smps_fops =
{
  smps_open,                    /* open */
  smps_close,                   /* close */
  smps_read,                    /* read */
  smps_write,                   /* write */
  NULL,                         /* seek */
  smps_ioctl,                   /* ioctl */
  NULL                          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: smps_open
 *
 * Description:
 *   This function is called whenever the SMPS device is opened.
 *
 ****************************************************************************/

static int smps_open(FAR struct file *filep)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct smps_dev_s *dev   = inode->i_private;
  uint8_t                tmp;
  int                    ret;

  /* If the port is the middle of closing, wait until the close is finished */

  ret = nxsem_wait(&dev->closesem);
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

      nxsem_post(&dev->closesem);
    }

  return OK;
}

/****************************************************************************
 * Name: smps_close
 *
 * Description:
 *   This routine is called when the SMPS device is closed.
 *   It waits for the last remaining data to be sent.
 *
 ****************************************************************************/

static int smps_close(FAR struct file *filep)
{
  FAR struct inode     *inode  = filep->f_inode;
  FAR struct smps_dev_s *dev   = inode->i_private;
  irqstate_t            flags;
  int                   ret;

  ret = nxsem_wait(&dev->closesem);
  if (ret >= 0)
    {
      /* Decrement the references to the driver.  If the reference count will
       * decrement to 0, then uninitialize the driver.
       */

      if (dev->ocount > 1)
        {
          dev->ocount--;
          nxsem_post(&dev->closesem);
        }
      else
        {
          /* There are no more references to the port */

          dev->ocount = 0;

          /* Free the IRQ and disable the SMPS device */

          flags = enter_critical_section();      /* Disable interrupts */
          dev->ops->shutdown(dev);               /* Disable the SMPS */
          leave_critical_section(flags);

          nxsem_post(&dev->closesem);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: smps_read
 ****************************************************************************/

static ssize_t smps_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  return 1;
}

/****************************************************************************
 * Name: smps_write
 ****************************************************************************/

static ssize_t smps_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return 1;
}

/****************************************************************************
 * Name: smps_ioctl
 ****************************************************************************/

static int smps_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode    = filep->f_inode;
  FAR struct smps_dev_s *dev = inode->i_private;
  FAR struct smps_s *smps    = (FAR struct smps_s *)dev->priv;
  int ret;

  switch (cmd)
    {
      case PWRIOC_START:
        {
          /* Allow SMPS start only when some limits available
           * and structure is locked.
           * REVISIT: not sure if it is needed here
           */

          if ((smps->limits.lock == false) ||
              (smps->limits.v_in <= 0 && smps->limits.v_out <= 0 &&
               smps->limits.i_in <= 0 && smps->limits.i_out <= 0 &&
               smps->limits.p_in <= 0 && smps->limits.p_out <= 0))
            {
              pwrerr("ERROR: SMPS limits data must be set"
                     " and locked before SMPS start\n");

              ret = -EPERM;
              goto errout;
            }

          /* Check SMPS mode */

          if (smps->opmode == SMPS_OPMODE_INIT)
            {
              pwrerr("ERROR: SMPS operation mode not specified\n");

              ret = -EPERM;
              goto errout;
            }

          /* When constan current mode, then output current must be
           * provided
           */

          if (smps->opmode == SMPS_OPMODE_CC && smps->param.i_out <= 0)
            {
              pwrerr("ERROR: CC selected but i_out not specified!\n");

              ret = -EPERM;
              goto errout;
            }

          /* When constan voltage mode, then output voltage must be
           * provided
           */

          if (smps->opmode == SMPS_OPMODE_CV && smps->param.v_out <= 0)
            {
              pwrerr("ERROR: CV selected but v_out not specified!\n");

              ret = -EPERM;
              goto errout;
            }

          /* When constan power mode, then output power must be provided */

          if (smps->opmode == SMPS_OPMODE_CP && smps->param.p_out <= 0)
            {
              pwrerr("ERROR: CP selected but p_out not specified!\n");

              ret = -EPERM;
              goto errout;
            }

          /* REVISIT: something else ?? */

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
          FAR struct smps_limits_s *limits =
            (FAR struct smps_limits_s *)((uintptr_t)arg);

          if (smps->limits.lock == true)
            {
              pwrerr("ERROR: SMPS limits locked!\n");

              ret = -EPERM;
              goto errout;
            }

          /* NOTE: this call must set the smps_limits_s structure */

          ret = dev->ops->limits_set(dev, limits);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_SET_LIMITS failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_GET_STATE:
        {
          FAR struct smps_state_s *state =
            (FAR struct smps_state_s *)((uintptr_t)arg);

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
          FAR struct smps_params_s *params =
            (FAR struct smps_params_s *)((uintptr_t)arg);

          if (smps->param.lock == true)
            {
              pwrerr("ERROR: SMPS params locked!\n");

              ret = -EPERM;
              goto errout;
            }

          if ((smps->limits.lock == false) ||
              (smps->limits.v_in <= 0 && smps->limits.v_out <= 0 &&
               smps->limits.i_in <= 0 && smps->limits.i_out <= 0 &&
               smps->limits.p_in <= 0 && smps->limits.p_out <= 0))
            {
              pwrerr("ERROR: limits must be set prior to params!\n");

              ret = -EPERM;
              goto errout;
            }

          /* Check output voltage configuration */

          if (smps->limits.v_out > 0 && params->v_out > smps->limits.v_out)
            {
              pwrerr("ERROR: params->v_out > limits.v_out: %.2f > %.2f\n",
                     params->v_out, smps->limits.v_out);

              ret = -EPERM;
              goto errout;
            }

          /* Check output current configuration */

          if (smps->limits.i_out > 0 && params->i_out > smps->limits.i_out)
            {
              pwrerr("ERROR: params->i_out > limits.i_out: %.2f > %.2f\n",
                     params->i_out, smps->limits.i_out);

              ret = -EPERM;
              goto errout;
            }

          /* Check output power configuration */

          if (smps->limits.p_out > 0 && params->p_out > smps->limits.p_out)
            {
              pwrerr("ERROR: params->p_out > limits.p_out: %.2f > %.2f\n",
                     params->p_out, smps->limits.p_out);

              ret = -EPERM;
              goto errout;
            }

          /* TODO: limits */

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
 * Name: smps_register
 ****************************************************************************/

int smps_register(FAR const char *path, FAR struct smps_dev_s *dev,
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

  /* Initialize the SMPS device structure */

  dev->ocount = 0;

  /* Initialize semaphores */

  nxsem_init(&dev->closesem, 0, 1);

  /* Connect SMPS driver with lower level interface */

  dev->lower = lower;

  /* Register the SMPS character driver */

  ret = register_driver(path, &smps_fops, 0444, dev);
  if (ret < 0)
    {
      nxsem_destroy(&dev->closesem);
    }

  return ret;
}
