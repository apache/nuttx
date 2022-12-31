/****************************************************************************
 * drivers/sensors/qencoder.c
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
#include <nuttx/sensors/qencoder.h>

#include <arch/irq.h>

#ifdef CONFIG_SENSORS_QENCODER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct qe_upperhalf_s
{
  uint8_t                    crefs;    /* The number of times the device has been opened */
  mutex_t                    lock;     /* Supports mutual exclusion */
  FAR struct qe_lowerhalf_s *lower;    /* lower-half state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     qe_open(FAR struct file *filep);
static int     qe_close(FAR struct file *filep);
static ssize_t qe_read(FAR struct file *filep, FAR char *buffer,
                       size_t buflen);
static ssize_t qe_write(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen);
static int     qe_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_qeops =
{
  qe_open,  /* open */
  qe_close, /* close */
  qe_read,  /* read */
  qe_write, /* write */
  NULL,     /* seek */
  qe_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qe_open
 *
 * Description:
 *   This function is called whenever the QEncoder device is opened.
 *
 ****************************************************************************/

static int qe_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct qe_upperhalf_s *upper = inode->i_private;
  uint8_t                     tmp;
  int                         ret;

  sninfo("crefs: %d\n", upper->crefs);

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
      FAR struct qe_lowerhalf_s *lower = upper->lower;

      /* Yes.. perform one time hardware initialization. */

      DEBUGASSERT(lower->ops->setup != NULL);
      sninfo("calling setup\n");

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
 * Name: qe_close
 *
 * Description:
 *   This function is called when the QEncoder device is closed.
 *
 ****************************************************************************/

static int qe_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct qe_upperhalf_s *upper = inode->i_private;
  int                         ret;

  sninfo("crefs: %d\n", upper->crefs);

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
      FAR struct qe_lowerhalf_s *lower = upper->lower;

      /* There are no more references to the port */

      upper->crefs = 0;

      /* Disable the QEncoder device */

      DEBUGASSERT(lower->ops->shutdown != NULL);
      sninfo("calling shutdown\n");

      lower->ops->shutdown(lower);
    }

  nxmutex_unlock(&upper->lock);
  ret = OK;

errout:
  return ret;
}

/****************************************************************************
 * Name: qe_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t qe_read(FAR struct file *filep,
                       FAR char *buffer,
                       size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: qe_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t qe_write(FAR struct file *filep,
                        FAR const char *buffer,
                        size_t buflen)
{
  /* Return a failure */

  return -EPERM;
}

/****************************************************************************
 * Name: qe_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *   This is where ALL of the QEncoder work is done.
 *
 ****************************************************************************/

static int qe_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode          *inode = filep->f_inode;
  FAR struct qe_upperhalf_s *upper;
  FAR struct qe_lowerhalf_s *lower;
  int                        ret;

  sninfo("cmd: %d arg: %ld\n", cmd, arg);
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
      /* QEIOC_POSITION - Get the current position from the encoder.
       *   Argument: int32_t pointer to the location to return the position.
       */

      case QEIOC_POSITION:
        {
          FAR int32_t *ptr = (FAR int32_t *)((uintptr_t)arg);
          DEBUGASSERT(lower->ops->position != NULL && ptr);
          ret = lower->ops->position(lower, ptr);
        }
        break;

      /* QEIOC_RESET - Reset the position to zero.
       *   Argument: None
       */

      case QEIOC_RESET:
        {
          DEBUGASSERT(lower->ops->reset != NULL);
          ret = lower->ops->reset(lower);
        }
        break;

      /* QEIOC_SETPOSMAX - Set the maximum encoder position.
       *   Argument: uint32
       */

      case QEIOC_SETPOSMAX:
        {
          uint32_t maxpos = (uint32_t)arg;
          if (lower->ops->setposmax != NULL)
            {
              ret = lower->ops->setposmax(lower, maxpos);
            }
          else
            {
              ret = -ENOTTY;
            }
        }
        break;

      /* QEIOC_SETINDEX - Set the index pin position
       *   Argument: uint32
       */

      case QEIOC_SETINDEX:
        {
          uint32_t indexpos = (uint32_t)arg;
          if (lower->ops->setindex != NULL)
            {
              ret = lower->ops->setindex(lower, indexpos);
            }
          else
            {
              ret = -ENOTTY;
            }
        }
        break;

      /* Any unrecognized IOCTL commands might be platform-specific ioctl
       * commands
       */

      default:
        {
          sninfo("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
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
 * Name: qe_register
 *
 * Description:
 *   Register the Quadrature Encoder lower half device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
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

int qe_register(FAR const char *devpath, FAR struct qe_lowerhalf_s *lower)
{
  FAR struct qe_upperhalf_s *upper;

  /* Allocate the upper-half data structure */

  upper = (FAR struct qe_upperhalf_s *)
           kmm_zalloc(sizeof(struct qe_upperhalf_s));
  if (!upper)
    {
      snerr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the QEncoder device structure
   * (it was already zeroed by kmm_zalloc())
   */

  nxmutex_init(&upper->lock);
  upper->lower = lower;

  /* Register the QEncoder device */

  sninfo("Registering %s\n", devpath);
  return register_driver(devpath, &g_qeops, 0666, upper);
}

#endif /* CONFIG_SENSORS_QENCODER */
