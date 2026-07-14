/****************************************************************************
 * drivers/analog/pot.c
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
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/analog/pot.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pot_open(FAR struct file *filep);
static int pot_close(FAR struct file *filep);
static int pot_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pot_fops =
{
  pot_open,                      /* open */
  pot_close,                     /* close */
  NULL,                          /* read */
  NULL,                          /* write */
  NULL,                          /* seek */
  pot_ioctl,                     /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pot_open
 *
 * Description:
 *   This function is called whenever the potentiometer device is opened.
 *
 ****************************************************************************/

static int pot_open(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct pot_dev_s *dev   = inode->i_private;
  uint8_t               tmp;
  int                   ret;

  /* If the port is the middle of closing, wait until the close is
   * finished.
   */

  ret = nxmutex_lock(&dev->pd_closelock);
  if (ret >= 0)
    {
      /* Increment the count of references to the device.  If this is the
       * first time that the driver has been opened for this device, then
       * initialize the device.
       */

      tmp = dev->pd_ocount + 1;
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

              ret = dev->pd_ops->po_setup(dev);
              if (ret == OK)
                {
                  /* Save the new open count on success */

                  dev->pd_ocount = tmp;
                }
            }
        }

      nxmutex_unlock(&dev->pd_closelock);
    }

  return ret;
}

/****************************************************************************
 * Name: pot_close
 *
 * Description:
 *   This routine is called when the potentiometer device is closed.
 *
 ****************************************************************************/

static int pot_close(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct pot_dev_s *dev   = inode->i_private;
  int                   ret;

  ret = nxmutex_lock(&dev->pd_closelock);
  if (ret >= 0)
    {
      /* Decrement the references to the driver.  If the reference count
       * will decrement to 0, then uninitialize the driver.
       */

      if (dev->pd_ocount > 1)
        {
          dev->pd_ocount--;
        }
      else
        {
          /* There are no more references to the port */

          dev->pd_ocount = 0;

          /* Disable the potentiometer device */

          dev->pd_ops->po_shutdown(dev);
        }

      nxmutex_unlock(&dev->pd_closelock);
    }

  return ret;
}

/****************************************************************************
 * Name: pot_ioctl
 ****************************************************************************/

static int pot_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct pot_dev_s *dev   = inode->i_private;
  int                   ret   = OK;

  ret = nxmutex_lock(&dev->pd_lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      /* Get the device properties */

      case POTIOC_GET_INFO:
        {
          FAR struct pot_info_s *info =
            (FAR struct pot_info_s *)((uintptr_t)arg);

          DEBUGASSERT(info != NULL);
          info->nwipers = dev->pd_nwipers;
          info->max     = dev->pd_max;
          info->rab     = dev->pd_rab;
          info->caps    = 0;

          if (dev->pd_ops->po_getwiper)
            {
              info->caps |= POT_CAP_READBACK;
            }

          if (dev->pd_ops->po_move)
            {
              info->caps |= POT_CAP_MOVE;
            }

          if (dev->pd_ops->po_enable)
            {
              info->caps |= POT_CAP_ENABLE;
            }

          if (dev->pd_ops->po_store || dev->pd_ops->po_recall)
            {
              info->caps |= POT_CAP_NV;
            }

          break;
        }

      /* Set a wiper value */

      case POTIOC_SET_WIPER:
        {
          FAR struct pot_wiper_s *wiper =
            (FAR struct pot_wiper_s *)((uintptr_t)arg);

          DEBUGASSERT(wiper != NULL);
          if (wiper->wiper >= dev->pd_nwipers || wiper->val > dev->pd_max)
            {
              ret = -EINVAL;
            }
          else if (dev->pd_ops->po_setwiper == NULL)
            {
              ret = -ENOTSUP;
            }
          else
            {
              ret = dev->pd_ops->po_setwiper(dev, wiper->wiper, wiper->val);
            }

          break;
        }

      /* Get a wiper value */

      case POTIOC_GET_WIPER:
        {
          FAR struct pot_wiper_s *wiper =
            (FAR struct pot_wiper_s *)((uintptr_t)arg);

          DEBUGASSERT(wiper != NULL);
          if (wiper->wiper >= dev->pd_nwipers)
            {
              ret = -EINVAL;
            }
          else if (dev->pd_ops->po_getwiper == NULL)
            {
              ret = -ENOTSUP;
            }
          else
            {
              ret = dev->pd_ops->po_getwiper(dev, wiper->wiper, &wiper->val);
            }

          break;
        }

      /* Move a wiper by a number of steps */

      case POTIOC_MOVE_WIPER:
        {
          FAR struct pot_move_s *move =
            (FAR struct pot_move_s *)((uintptr_t)arg);

          DEBUGASSERT(move != NULL);
          if (move->wiper >= dev->pd_nwipers)
            {
              ret = -EINVAL;
            }
          else if (dev->pd_ops->po_move == NULL)
            {
              ret = -ENOTSUP;
            }
          else
            {
              ret = dev->pd_ops->po_move(dev, move->wiper, move->steps);
            }

          break;
        }

      /* Enable or force a wiper into shutdown */

      case POTIOC_SET_ENABLE:
        {
          FAR struct pot_enable_s *enable =
            (FAR struct pot_enable_s *)((uintptr_t)arg);

          DEBUGASSERT(enable != NULL);
          if (enable->wiper >= dev->pd_nwipers)
            {
              ret = -EINVAL;
            }
          else if (dev->pd_ops->po_enable == NULL)
            {
              ret = -ENOTSUP;
            }
          else
            {
              ret = dev->pd_ops->po_enable(dev, enable->wiper,
                                           enable->enable);
            }

          break;
        }

      /* Store a wiper value to a non-volatile slot */

      case POTIOC_STORE_WIPER:
        {
          FAR struct pot_nv_s *nv = (FAR struct pot_nv_s *)((uintptr_t)arg);

          DEBUGASSERT(nv != NULL);
          if (nv->wiper >= dev->pd_nwipers)
            {
              ret = -EINVAL;
            }
          else if (dev->pd_ops->po_store == NULL)
            {
              ret = -ENOTSUP;
            }
          else
            {
              ret = dev->pd_ops->po_store(dev, nv->wiper, nv->slot);
            }

          break;
        }

      /* Recall a wiper value from a non-volatile slot */

      case POTIOC_RECALL_WIPER:
        {
          FAR struct pot_nv_s *nv = (FAR struct pot_nv_s *)((uintptr_t)arg);

          DEBUGASSERT(nv != NULL);
          if (nv->wiper >= dev->pd_nwipers)
            {
              ret = -EINVAL;
            }
          else if (dev->pd_ops->po_recall == NULL)
            {
              ret = -ENOTSUP;
            }
          else
            {
              ret = dev->pd_ops->po_recall(dev, nv->wiper, nv->slot);
            }

          break;
        }

      /* Forward any unrecognized commands to the lower half */

      default:
        {
          if (dev->pd_ops->po_ioctl)
            {
              ret = dev->pd_ops->po_ioctl(dev, cmd, arg);
            }
          else
            {
              ret = -ENOTTY;
            }

          break;
        }
    }

  nxmutex_unlock(&dev->pd_lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pot_register
 ****************************************************************************/

int pot_register(FAR const char *path, FAR struct pot_dev_s *dev)
{
  int ret;

  DEBUGASSERT(dev != NULL && dev->pd_ops != NULL);

  /* Initialize the potentiometer device structure */

  dev->pd_ocount = 0;

  /* Initialize mutexes */

  nxmutex_init(&dev->pd_closelock);
  nxmutex_init(&dev->pd_lock);

  /* Register the potentiometer character driver */

  ret = register_driver(path, &g_pot_fops, 0666, dev);
  if (ret < 0)
    {
      nxmutex_destroy(&dev->pd_closelock);
      nxmutex_destroy(&dev->pd_lock);
    }

  return ret;
}
