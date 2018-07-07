/****************************************************************************
 * drivers/power/motor.c
 * Upper-half, character driver for motor control
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#include <nuttx/power/motor.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     motor_open(FAR struct file *filep);
static int     motor_close(FAR struct file *filep);
static ssize_t motor_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t motor_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static int     motor_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations motor_fops =
{
  motor_open,                    /* open */
  motor_close,                   /* close */
  motor_read,                    /* read */
  motor_write,                   /* write */
  NULL,                          /* seek */
  motor_ioctl                    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL                         /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                         /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: motor_open
 *
 * Description:
 *   This function is called whenever the motor device is opened.
 *
 ****************************************************************************/

static int motor_open(FAR struct file *filep)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct motor_dev_s *dev   = inode->i_private;
  uint8_t                 tmp;
  int                     ret;

  /* If the port is the middle of closing, wait until the close is finished */

  ret = nxsem_wait(&dev->closesem);
  if (ret >= 0)
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

      nxsem_post(&dev->closesem);
    }

  return OK;
}

/****************************************************************************
 * Name: motor_close
 *
 * Description:
 *   This routine is called when the motor device is closed.
 *
 ****************************************************************************/

static int motor_close(FAR struct file *filep)
{
  FAR struct inode      *inode  = filep->f_inode;
  FAR struct motor_dev_s *dev   = inode->i_private;
  irqstate_t             flags;
  int                    ret;

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

          /* Free the IRQ and disable the motor device */

          flags = enter_critical_section();       /* Disable interrupts */
          dev->ops->shutdown(dev);               /* Disable the motor */
          leave_critical_section(flags);

          nxsem_post(&dev->closesem);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: motor_read
 ****************************************************************************/

static ssize_t motor_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  return 1;
}

/****************************************************************************
 * Name: motor_write
 ****************************************************************************/

static ssize_t motor_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  return 1;
}

/****************************************************************************
 * Name: motor_ioctl
****************************************************************************/

static int motor_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct motor_dev_s *dev   = inode->i_private;
  FAR struct motor_s     *motor = (FAR struct motor_s *)dev->priv;
  int ret;

  switch (cmd)
    {
      case PWRIOC_START:
        {
          /* Allow motor start only when some limits available
           * and strucutre is locked.
           */

          if ((motor->limits.lock == false) ||
              (
#ifdef CONFIG_MOTOR_HAVE_POSITION
                motor->limits.position <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_HAVE_SPEED
               motor->limits.speed <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_HAVE_TORQUE
               motor->limits.torque <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_HAVE_FORCE
                motor->limits.force <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
               motor->limits.v_in <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
               motor->limits.i_in <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
               motor->limits.p_in <= 0.0 &&
#endif
                1))
            {
              pwrerr("ERROR: motor limits data must be set"
                     " and locked before motor start\n");

              ret = -EPERM;
              goto errout;
            }

          /* Check motor mode */

          if (motor->opmode == MOTOR_OPMODE_INIT)
            {
              pwrerr("ERROR: motor operation mode not specified\n");

              ret = -EPERM;
              goto errout;
            }

          /* REVISIT: do we need some parameters assertions here ? */

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
          /* Call stop from lower-half driver */

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
          FAR struct motor_limits_s *limits =
            (FAR struct motor_limits_s *)((uintptr_t)arg);

          if (motor->limits.lock == true)
            {
              pwrerr("ERROR: motor limits locked!\n");

              ret = -EPERM;
              goto errout;
            }

          /* NOTE: this call must set the motor_limits_s structure */

          ret = dev->ops->limits_set(dev, limits);
          if (ret != OK)
            {
              pwrerr("ERROR: PWRIOC_SET_LIMITS failed %d\n", ret);
            }
          break;
        }

      case PWRIOC_GET_STATE:
        {
          FAR struct motor_state_s *state =
            (FAR struct motor_state_s *)((uintptr_t)arg);

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
          uint8_t *fault = ((uint8_t*)arg);

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
          FAR struct motor_params_s *params =
            (FAR struct motor_params_s *)((uintptr_t)arg);

          if (motor->param.lock == true)
            {
              pwrerr("ERROR: motor params locked!\n");

              ret = -EPERM;
              goto errout;
            }

          if ((motor->limits.lock == false) ||
              (
#ifdef CONFIG_MOTOR_HAVE_POSITION
                motor->limits.position <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_HAVE_SPEED
                motor->limits.speed <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_HAVE_TORQUE
                motor->limits.torque <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_HAVE_FORCE
                motor->limits.force <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
                motor->limits.v_in <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
                motor->limits.i_in <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
                motor->limits.p_in <= 0.0 &&
#endif
                1))
            {
              pwrerr("ERROR: limits must be set prior to params!\n");

              ret = -EPERM;
              goto errout;
            }

#ifdef CONFIG_MOTOR_HAVE_DIRECTION
          /* Check direction configuration */

          if (params->direction != MOTOR_DIR_CCW &&
              params->direction != MOTOR_DIR_CW)
            {
              pwrerr("ERROR: invalid direction value %d\n",
                     params->direction);

              ret = -EPERM;
              goto errout;
            }
#endif

#ifdef CONFIG_MOTOR_HAVE_POSITION
          /* Check position configuration */

          if (params->position < 0.0 ||
              params->position > motor->limits.position)
            {
              pwrerr("ERROR: params->position > limits.position: %.2f > %.2f\n",
                     params->position, motor->limits.position);

              ret = -EPERM;
              goto errout;
            }
#endif

#ifdef CONFIG_MOTOR_HAVE_SPEED
          /* Check speed configuration */

          if (motor->limits.speed > 0.0 && params->speed > motor->limits.speed)
            {
              pwrerr("ERROR: params->speed > limits.speed: %.2f > %.2f\n",
                     params->speed, motor->limits.speed);

              ret = -EPERM;
              goto errout;
            }
#endif

#ifdef CONFIG_MOTOR_HAVE_TORQUE
          /* Check torque configuration */

          if (motor->limits.torque > 0.0 && params->torque > motor->limits.torque)
            {
              pwrerr("ERROR: params->torque > limits.torque: %.2f > %.2f\n",
                     params->torque, motor->limits.torque);

              ret = -EPERM;
              goto errout;
            }
#endif

#ifdef CONFIG_MOTOR_HAVE_FORCE
          /* Check force configuration */

          if (motor->limits.force > 0.0 && params->force > motor->limits.force)
            {
              pwrerr("ERROR: params->force > limits.force: %.2f > %.2f\n",
                     params->force, motor->limits.force);

              ret = -EPERM;
              goto errout;
            }
#endif

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
 * Name: motor_register
 ****************************************************************************/

int motor_register(FAR const char *path, FAR struct motor_dev_s *dev, FAR void *lower)
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

  /* Initialize the motor device structure */

  dev->ocount = 0;

  /* Initialize semaphores */

  nxsem_init(&dev->closesem, 0, 1);

  /* Connect motor driver with lower level interface */

  dev->lower = lower;

  /* Register the motor character driver */

  ret = register_driver(path, &motor_fops, 0444, dev);
  if (ret < 0)
    {
      nxsem_destroy(&dev->closesem);
    }

  return ret;
}
