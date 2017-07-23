/****************************************************************************
 * drivers/power/smps.c
 * Upper-half, character driver for SMPS (switched-mode power supply)
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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
#include <unistd.h>
#include <semaphore.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
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
static int     smps_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

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
  smps_ioctl                    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL                        /* poll */
#endif
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
  int                    ret   = OK;

  /* If the port is the middle of closing, wait until the close is finished */

  if (sem_wait(&dev->closesem) != OK)
    {
      ret = -errno;
    }
  else
    {
      /* Increment the count of references to the device.  If this the first
       * time that the driver has been opened for this device, then initialize
       * the device.
       */

      tmp = dev->ocount + 1;
      if (tmp == 0)
        {
          /* More than 255 opens; uint8_t overflows to zero */

          ret = -EMFILE;
        }
      else
        {
          /* Check if this is the first time that the driver has been opened. */

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

      sem_post(&dev->closesem);
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
  FAR struct inode     *inode = filep->f_inode;
  FAR struct smps_dev_s *dev   = inode->i_private;
  irqstate_t            flags;
  int                   ret = OK;

  if (sem_wait(&dev->closesem) != OK)
    {
      ret = -errno;
    }
  else
    {
      /* Decrement the references to the driver.  If the reference count will
       * decrement to 0, then uninitialize the driver.
       */

      if (dev->ocount > 1)
        {
          dev->ocount--;
          sem_post(&dev->closesem);
        }
      else
        {
          /* There are no more references to the port */

          dev->ocount = 0;

          /* Free the IRQ and disable the SMPS device */

          flags = enter_critical_section();       /* Disable interrupts */
          dev->ops->shutdown(dev);               /* Disable the SMPS */
          leave_critical_section(flags);

          sem_post(&dev->closesem);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: smps_read
 ****************************************************************************/

static ssize_t smps_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  return 1;
}

/****************************************************************************
 * Name: smps_write
 ****************************************************************************/

static ssize_t smps_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
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
  int ret;

  switch (cmd)
    {
      case PWRIOC_START:
        {
          /* TODO: sanity checking:
           *        - absolute limits
           *        - SMPS parameters
           *        - SMPS mode
           */

          ret = dev->ops->start(dev);
          if (ret != OK)
            {
              pwrerr("ERROR: SMPSC_START failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_STOP:
        {
          ret = dev->ops->stop(dev);
          if (ret != OK)
            {
              pwrerr("ERROR: SMPSC_STOP failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_SET_MODE:
        {
          uint8_t mode = ((uint8_t)arg);

          ret = dev->ops->mode_set(dev, mode);
          if (ret != OK)
            {
              pwrerr("ERROR: SMPSC_SET_MODE failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_SET_LIMITS:
        {
          FAR struct smps_limits_s *limits =
            (FAR struct smps_limits_s *)((uintptr_t)arg);

          /* TODO: lock mechanism */

          ret = dev->ops->limits_set(dev, limits);
          if (ret != OK)
            {
              pwrerr("ERROR: SMPSC_SET_LIMITS failed %d\n", ret);
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
              pwrerr("ERROR: SMPSC_GET_STATE failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_SET_FAULT:
        {
          uint8_t fault = ((uint8_t)arg);

          ret = dev->ops->fault_set(dev, fault);
          if (ret != OK)
            {
              pwrerr("ERROR: SMPSC_SET_FAULT failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_GET_FAULT:
        {
          uint8_t *fault = ((uint8_t*)arg);

          ret = dev->ops->fault_get(dev, fault);
          if (ret != OK)
            {
              pwrerr("ERROR: SMPSC_GET_FAULT failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_CLEAN_FAULT:
        {
          uint8_t fault = ((uint8_t)arg);

          ret = dev->ops->fault_clean(dev, fault);
          if (ret != OK)
            {
              pwrerr("ERROR: SMPSC_CLEAN_FAULT failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_SET_PARAMS:
        {
          FAR struct smps_params_s *params =
            (FAR struct smps_params_s *)((uintptr_t)arg);

          /* TODO: lock mechanism */
          /* TODO: limits */

          ret = dev->ops->params_set(dev, params);
          if (ret != OK)
            {
              pwrerr("ERROR: SMPSC_SET_PARAMS failed %d\n", ret);
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

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: smps_register
 ****************************************************************************/

int smps_register(FAR const char *path, FAR struct smps_dev_s *dev, FAR void *lower)
{
  int ret;

  /* Initialize the HRTIM device structure */

  dev->ocount = 0;

  /* Initialize semaphores */

  sem_init(&dev->closesem, 0, 1);

  /* Connect SMPS driver with lower level interface */

  dev->lower = lower;

  /* Register the SMPS character driver */

  ret =  register_driver(path, &smps_fops, 0444, dev);
  if (ret < 0)
    {
      sem_destroy(&dev->closesem);
    }

  return ret;
}
