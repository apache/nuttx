/****************************************************************************
 * drivers/motor/motor.c
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

/* Upper-half, character driver for motor control */

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
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/motor/motor.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct motor_upperhalf_s
{
  FAR struct motor_lowerhalf_s *lower; /* the handle of lower half driver */
  uint8_t ocount;                      /* The number of times the device has been opened */
  sem_t closesem;                      /* Locks out new opens while close is in progress */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     motor_open(FAR struct file *filep);
static int     motor_close(FAR struct file *filep);
static ssize_t motor_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t motor_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static int     motor_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

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
  motor_ioctl,                   /* ioctl */
  NULL                           /* poll */
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
  FAR struct inode *inode = filep->f_inode;
  FAR struct motor_upperhalf_s *upper = inode->i_private;
  FAR struct motor_lowerhalf_s *lower = upper->lower;
  uint8_t tmp;
  int ret;

  /* If the port is the middle of closing, wait until the close is finished */

  ret = nxsem_wait(&upper->closesem);
  if (ret >= 0)
    {
      /* Increment the count of references to the device.  If this the first
       * time that the driver has been opened for this device, then
       * initialize the device.
       */

      tmp = upper->ocount + 1;
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

              ret = lower->ops->setup(lower);
              if (ret == OK)
                {
                  /* Save the new open count on success */

                  upper->ocount = tmp;
                }
            }
        }

      nxsem_post(&upper->closesem);
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
  FAR struct inode *inode = filep->f_inode;
  FAR struct motor_upperhalf_s *upper = inode->i_private;
  FAR struct motor_lowerhalf_s *lower = upper->lower;
  int ret;

  ret = nxsem_wait(&upper->closesem);
  if (ret >= 0)
    {
      /* Decrement the references to the driver.  If the reference count will
       * decrement to 0, then uninitialize the driver.
       */

      if (upper->ocount > 1)
        {
          upper->ocount--;
          nxsem_post(&upper->closesem);
        }
      else
        {
          /* There are no more references to the port */

          upper->ocount = 0;

          /* Free the IRQ and disable the motor device */

          lower->ops->shutdown(lower);           /* Disable the motor */
          nxsem_post(&upper->closesem);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: motor_read
 ****************************************************************************/

static ssize_t motor_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  return 1;
}

/****************************************************************************
 * Name: motor_write
 ****************************************************************************/

static ssize_t motor_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return 1;
}

/****************************************************************************
 * Name: motor_ioctl
 ****************************************************************************/

static int motor_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct motor_upperhalf_s *upper = inode->i_private;
  FAR struct motor_lowerhalf_s *lower = upper->lower;
  int ret;

  switch (cmd)
    {
      case MTRIOC_START:
        {
          /* Allow motor start only when some limits available
           * and structure is locked.
           */

          if ((lower->limits.lock == false) ||
              (
#ifdef CONFIG_MOTOR_UPPER_HAVE_POSITION
                lower->limits.position <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_SPEED
                lower->limits.speed <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_TORQUE
                lower->limits.torque <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_FORCE
                lower->limits.force <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_VOLTAGE
                lower->limits.v_in <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_CURRENT
                lower->limits.i_in <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_POWER
                lower->limits.p_in <= 0.0 &&
#endif
                1))
            {
              mtrerr("Motor limits data must be set"
                     " and locked before motor start\n");

              ret = -EPERM;
              goto errout;
            }

          /* Check motor mode */

          if (lower->opmode == MOTOR_OPMODE_INIT)
            {
              mtrerr("Motor operation mode not specified\n");

              ret = -EPERM;
              goto errout;
            }

          /* REVISIT: do we need some parameters assertions here ? */

          /* Finally, call start from lower-half driver */

          ret = lower->ops->start(lower);
          if (ret != OK)
            {
              mtrerr("MTRIOC_START failed %d\n", ret);
            }
          break;
        }

      case MTRIOC_STOP:
        {
          /* Call stop from lower-half driver */

          ret = lower->ops->stop(lower);
          if (ret != OK)
            {
              mtrerr("MTRIOC_STOP failed %d\n", ret);
            }
          break;
        }

      case MTRIOC_SET_MODE:
        {
          uint8_t mode = ((uint8_t)arg);

          ret = lower->ops->mode_set(lower, mode);
          if (ret != OK)
            {
              mtrerr("MTRIOC_SET_MODE failed %d\n", ret);
            }
          break;
        }

      case MTRIOC_SET_LIMITS:
        {
          FAR struct motor_limits_s *limits =
            (FAR struct motor_limits_s *)((uintptr_t)arg);

          if (lower->limits.lock == true)
            {
              mtrerr("Motor limits locked!\n");

              ret = -EPERM;
              goto errout;
            }

          /* NOTE: this call must set the motor_limits_s structure */

          ret = lower->ops->limits_set(lower, limits);
          if (ret != OK)
            {
              mtrerr("MTRIOC_SET_LIMITS failed %d\n", ret);
            }
          break;
        }

      case MTRIOC_GET_STATE:
        {
          FAR struct motor_state_s *state =
            (FAR struct motor_state_s *)((uintptr_t)arg);

          ret = lower->ops->state_get(lower, state);
          if (ret != OK)
            {
              mtrerr("MTRIOC_GET_STATE failed %d\n", ret);
            }
          break;
        }

      case MTRIOC_SET_FAULT:
        {
          uint8_t fault = ((uint8_t)arg);

          ret = lower->ops->fault_set(lower, fault);
          if (ret != OK)
            {
              mtrerr("MTRIOC_SET_FAULT failed %d\n", ret);
            }
          break;
        }

      case MTRIOC_GET_FAULT:
        {
          FAR uint8_t *fault = ((FAR uint8_t *)arg);

          ret = lower->ops->fault_get(lower, fault);
          if (ret != OK)
            {
              mtrerr("MTRIOC_GET_FAULT failed %d\n", ret);
            }
          break;
        }

      case MTRIOC_CLEAR_FAULT:
        {
          uint8_t fault = ((uint8_t)arg);

          ret = lower->ops->fault_clear(lower, fault);
          if (ret != OK)
            {
              mtrerr("MTRIOC_CLEAR_FAULT failed %d\n", ret);
            }
          break;
        }

      case MTRIOC_SET_PARAMS:
        {
          FAR struct motor_params_s *params =
            (FAR struct motor_params_s *)((uintptr_t)arg);

          if (lower->param.lock == true)
            {
              mtrerr("Motor params locked!\n");

              ret = -EPERM;
              goto errout;
            }

          if ((lower->limits.lock == false) ||
              (
#ifdef CONFIG_MOTOR_UPPER_HAVE_POSITION
                lower->limits.position <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_SPEED
                lower->limits.speed <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_TORQUE
                lower->limits.torque <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_FORCE
                lower->limits.force <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_VOLTAGE
                lower->limits.v_in <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_CURRENT
                lower->limits.i_in <= 0.0 &&
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_POWER
                lower->limits.p_in <= 0.0 &&
#endif
                1))
            {
              mtrerr("Limits must be set prior to params!\n");

              ret = -EPERM;
              goto errout;
            }

#ifdef CONFIG_MOTOR_UPPER_HAVE_DIRECTION
          /* Check direction configuration */

          if (params->direction != MOTOR_DIR_CCW &&
              params->direction != MOTOR_DIR_CW)
            {
              mtrerr("Invalid direction value %d\n",
                     params->direction);

              ret = -EPERM;
              goto errout;
            }
#endif

#ifdef CONFIG_MOTOR_UPPER_HAVE_POSITION
          /* Check position configuration */

          if (params->position < 0.0 ||
              params->position > lower->limits.position)
            {
              mtrerr("params->position > limits.position: "
                     "%.2f > %.2f\n",
                     params->position, lower->limits.position);

              ret = -EPERM;
              goto errout;
            }
#endif

#ifdef CONFIG_MOTOR_UPPER_HAVE_SPEED
          /* Check speed configuration */

          if (lower->limits.speed > 0.0 &&
              params->speed > lower->limits.speed)
            {
              mtrerr("params->speed > limits.speed: %.2f > %.2f\n",
                     params->speed, lower->limits.speed);

              ret = -EPERM;
              goto errout;
            }
#endif

#ifdef CONFIG_MOTOR_UPPER_HAVE_TORQUE
          /* Check torque configuration */

          if (lower->limits.torque > 0.0 &&
              params->torque > lower->limits.torque)
            {
              mtrerr("params->torque > limits.torque: %.2f > %.2f\n",
                     params->torque, lower->limits.torque);

              ret = -EPERM;
              goto errout;
            }
#endif

#ifdef CONFIG_MOTOR_UPPER_HAVE_FORCE
          /* Check force configuration */

          if (lower->limits.force > 0.0 &&
              params->force > lower->limits.force)
            {
              mtrerr("params->force > limits.force: %.2f > %.2f\n",
                     params->force, lower->limits.force);

              ret = -EPERM;
              goto errout;
            }
#endif

          ret = lower->ops->params_set(lower, params);
          if (ret != OK)
            {
              mtrerr("MTRIOC_SET_PARAMS failed %d\n", ret);
            }
          break;
        }

      default:
        {
          mtrinfo("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
          ret = lower->ops->ioctl(lower, cmd, arg);
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
 *
 * Description:
 *   This function binds an instance of a "lower half" motor driver with the
 *   "upper half" motor device and registers that device so that can be used
 *   by application code.
 *
 *   We will register the chararter device with specified path.
 *
 * Input Parameters:
 *   path  - The user specifies path name.
 *   lower - A pointer to an instance of lower half motor driver. This
 *           instance is bound to the motor driver and must persists as long
 *           as the driver persists.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int motor_register(FAR const char *path,
                   FAR struct motor_lowerhalf_s *lower)
{
  FAR struct motor_upperhalf_s *upper;
  int ret;

  DEBUGASSERT(path != NULL && lower != NULL);
  DEBUGASSERT(lower->ops != NULL);

  /* For safety reason, when some necessary low-level logic is not provided,
   * system should fail before low-level hardware initialization, so:
   *   - all ops are checked here, before character driver registration
   *   - all ops must be provided, even if not used
   */

  DEBUGASSERT(lower->ops->setup != NULL);
  DEBUGASSERT(lower->ops->shutdown != NULL);
  DEBUGASSERT(lower->ops->stop != NULL);
  DEBUGASSERT(lower->ops->start != NULL);
  DEBUGASSERT(lower->ops->params_set != NULL);
  DEBUGASSERT(lower->ops->mode_set != NULL);
  DEBUGASSERT(lower->ops->limits_set != NULL);
  DEBUGASSERT(lower->ops->fault_set != NULL);
  DEBUGASSERT(lower->ops->state_get != NULL);
  DEBUGASSERT(lower->ops->fault_get != NULL);
  DEBUGASSERT(lower->ops->fault_clear != NULL);
  DEBUGASSERT(lower->ops->ioctl != NULL);

  /* Allocate the upper-half data structure */

  upper = kmm_zalloc(sizeof(struct motor_upperhalf_s));
  if (!upper)
    {
      mtrerr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize semaphores */

  nxsem_init(&upper->closesem, 0, 1);

  /* Connect motor driver with lower level interface */

  upper->lower = lower;

  /* Register the motor character driver */

  ret = register_driver(path, &motor_fops, 0444, upper);
  if (ret < 0)
    {
      nxsem_destroy(&upper->closesem);
      kmm_free(upper);
    }

  return ret;
}
