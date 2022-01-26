/****************************************************************************
 * drivers/analog/opamp.c
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
#include <nuttx/analog/opamp.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     opamp_open(FAR struct file *filep);
static int     opamp_close(FAR struct file *filep);
static int     opamp_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations opamp_fops =
{
  opamp_open,                    /* open */
  opamp_close,                   /* close */
  NULL,                          /* read */
  NULL,                          /* write */
  NULL,                          /* seek */
  opamp_ioctl,                   /* ioctl */
  NULL                           /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                         /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: opamp_open
 *
 * Description:
 *   This function is called whenever the OPAMP device is opened.
 *
 ****************************************************************************/

static int opamp_open(FAR struct file *filep)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct opamp_dev_s *dev   = inode->i_private;
  uint8_t                tmp;
  int                    ret;

  /* If the port is the middle of closing, wait until the close is
   * finished.
   */

  ret = nxsem_wait(&dev->ad_closesem);
  if (ret >= 0)
    {
      /* Increment the count of references to the device.  If this is the
       * first time that the driver has been opened for this device, then
       * initialize the device.
       */

      tmp = dev->ad_ocount + 1;
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
              ret = dev->ad_ops->ao_setup(dev);
              if (ret == OK)
                {
                  /* Save the new open count on success */

                  dev->ad_ocount = tmp;
                }

              leave_critical_section(flags);
            }
        }

      nxsem_post(&dev->ad_closesem);
    }

  return ret;
}

/****************************************************************************
 * Name: opamp_close
 *
 * Description:
 *   This routine is called when the OPAMP device is closed.
 *   It waits for the last remaining data to be sent.
 *
 ****************************************************************************/

static int opamp_close(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct opamp_dev_s *dev   = inode->i_private;
  irqstate_t            flags;
  int                   ret;

  ret = nxsem_wait(&dev->ad_closesem);
  if (ret >= 0)
    {
      /* Decrement the references to the driver.  If the reference count will
       * decrement to 0, then uninitialize the driver.
       */

      if (dev->ad_ocount > 1)
        {
          dev->ad_ocount--;
          nxsem_post(&dev->ad_closesem);
        }
      else
        {
          /* There are no more references to the port */

          dev->ad_ocount = 0;

          /* Free the IRQ and disable the OPAMP device */

          flags = enter_critical_section();       /* Disable interrupts */
          dev->ad_ops->ao_shutdown(dev);          /* Disable the OPAMP */
          leave_critical_section(flags);

          nxsem_post(&dev->ad_closesem);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: opamp_ioctl
 ****************************************************************************/

static int opamp_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct opamp_dev_s *dev = inode->i_private;
  int ret;

  ret = dev->ad_ops->ao_ioctl(dev, cmd, arg);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: opamp_register
 ****************************************************************************/

int opamp_register(FAR const char *path, FAR struct opamp_dev_s *dev)
{
  int ret;

  /* Initialize the OPAMP device structure */

  dev->ad_ocount = 0;

  /* Initialize semaphores */

  nxsem_init(&dev->ad_closesem, 0, 1);

  /* Register the OPAMP character driver */

  ret = register_driver(path, &opamp_fops, 0444, dev);
  if (ret < 0)
    {
      nxsem_destroy(&dev->ad_closesem);
    }

  return ret;
}
