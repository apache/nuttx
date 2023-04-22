/****************************************************************************
 * drivers/analog/dac.c
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
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/analog/dac.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HALF_SECOND_MSEC 500
#define HALF_SECOND_USEC 500000L

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     dac_open(FAR struct file *filep);
static int     dac_close(FAR struct file *filep);
static ssize_t dac_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen);
static int     dac_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_dac_fops =
{
  dac_open,       /* open */
  dac_close,      /* close */
  NULL,           /* read */
  dac_write,      /* write */
  NULL,           /* seek */
  dac_ioctl,      /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dac_open
 *
 * Description:
 *   This function is called whenever the DAC device is opened.
 *
 ****************************************************************************/

static int dac_open(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct dac_dev_s *dev   = inode->i_private;
  uint8_t               tmp;
  int                   ret;

  /* If the port is in the middle of closing, wait until the close is
   * finished.
   */

  ret = nxmutex_lock(&dev->ad_closelock);
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
                  /* Mark the FIFOs empty */

                  dev->ad_xmit.af_head = 0;
                  dev->ad_xmit.af_tail = 0;

                  /* Save the new open count on success */

                  dev->ad_ocount = tmp;
                }

              leave_critical_section(flags);
            }
        }

      nxmutex_unlock(&dev->ad_closelock);
    }

  return ret;
}

/****************************************************************************
 * Name: dac_close
 *
 * Description:
 *   This routine is called when the DAC device is closed.
 *   It waits for the last remaining data to be sent.
 *
 ****************************************************************************/

static int dac_close(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct dac_dev_s *dev   = inode->i_private;
  irqstate_t            flags;
  int                   ret;

  ret = nxmutex_lock(&dev->ad_closelock);
  if (ret >= 0)
    {
      /* Decrement the references to the driver.  If the reference count will
       * decrement to 0, then uninitialize the driver.
       */

      if (dev->ad_ocount > 1)
        {
          dev->ad_ocount--;
          nxmutex_unlock(&dev->ad_closelock);
        }
      else
        {
          /* There are no more references to the port */

          dev->ad_ocount = 0;

          /* Now we wait for the transmit FIFO to clear */

          while (dev->ad_xmit.af_head != dev->ad_xmit.af_tail)
            {
               nxsig_usleep(HALF_SECOND_USEC);
            }

          /* Free the IRQ and disable the DAC device */

          flags = enter_critical_section();    /* Disable interrupts */
          dev->ad_ops->ao_shutdown(dev);       /* Disable the DAC */
          leave_critical_section(flags);

          nxmutex_unlock(&dev->ad_closelock);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: dac_xmit
 *
 * Description:
 *   Send the message at the head of the ad_xmit FIFO
 *
 * Assumptions:
 *   Called with interrupts disabled
 *
 ****************************************************************************/

static int dac_xmit(FAR struct dac_dev_s *dev)
{
  bool enable = false;
  int ret = OK;

  /* Check if the xmit FIFO is empty */

  if (dev->ad_xmit.af_head != dev->ad_xmit.af_tail)
    {
      /* Send the next message at the head of the FIFO */

      ret = dev->ad_ops->ao_send(dev,
        &dev->ad_xmit.af_buffer[dev->ad_xmit.af_head]);

      /* Make sure the TX done interrupts are enabled */

      enable = (ret == OK ? true : false);
    }

  dev->ad_ops->ao_txint(dev, enable);
  return ret;
}

/****************************************************************************
 * Name: dac_write
 ****************************************************************************/

static ssize_t dac_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct dac_dev_s  *dev   = inode->i_private;
  FAR struct dac_fifo_s *fifo  = &dev->ad_xmit;
  FAR struct dac_msg_s  *msg;
  bool                   empty;
  ssize_t                nsent = 0;
  irqstate_t             flags;
  int                    nexttail;
  int                    msglen;
  int                    ret   = 0;

  /* Interrupts must be disabled throughout the following */

  flags = enter_critical_section();

  /* Check if the TX FIFO was empty when we started.  That is a clue that we
   * have to kick off a new TX sequence.
   */

  empty = (fifo->af_head == fifo->af_tail);

  /* Add the messages to the FIFO.  Ignore any trailing messages that are
   * shorter than the minimum.
   */

  if (buflen % 5 == 0)
    {
      msglen = 5;
    }
  else if (buflen % 4 == 0)
    {
      msglen = 4;
    }
  else if (buflen % 3 == 0)
    {
      msglen = 3;
    }
  else if (buflen % 2 == 0)
    {
      msglen = 2;
    }
  else if (buflen == 1)
    {
      msglen = 1;
    }
  else
    {
      msglen = 5;
    }

  while ((buflen - nsent) >= msglen)
    {
      /* Check if adding this new message would over-run the drivers ability
       * to enqueue xmit data.
       */

      nexttail = fifo->af_tail + 1;
      if (nexttail >= CONFIG_DAC_FIFOSIZE)
        {
          nexttail = 0;
        }

      /* If the XMIT FIFO becomes full, then wait for space to become
       * available.
       */

      while (nexttail == fifo->af_head)
        {
          /* The transmit FIFO is full -- was non-blocking mode selected? */

          if (filep->f_oflags & O_NONBLOCK)
            {
              if (nsent == 0)
                {
                  ret = -EAGAIN;
                }
              else
                {
                  ret = nsent;
                }

              goto return_with_irqdisabled;
            }

          /* If the FIFO was empty when we started, then we will have to
           * start the XMIT sequence to clear the FIFO.
           */

          if (empty)
            {
              dac_xmit(dev);
            }

          /* Wait for a message to be sent */

          ret = nxsem_wait_uninterruptible(&fifo->af_sem);
          if (ret < 0)
            {
              goto return_with_irqdisabled;
            }

          /* Re-check the FIFO state */

          empty = (fifo->af_head == fifo->af_tail);
        }

      /* We get here if there is space at the end of the FIFO.  Add the new
       * DAC message at the tail of the FIFO.
       */

      if (msglen == 5)
        {
          msg = (FAR struct dac_msg_s *)&buffer[nsent];
          memcpy(&fifo->af_buffer[fifo->af_tail], msg, msglen);
        }
      else if (msglen == 4)
        {
          fifo->af_buffer[fifo->af_tail].am_channel = buffer[nsent];
          fifo->af_buffer[fifo->af_tail].am_data =
            *(FAR uint32_t *)&buffer[nsent];
          fifo->af_buffer[fifo->af_tail].am_data &= 0xffffff00;
        }
      else if (msglen == 3)
        {
          fifo->af_buffer[fifo->af_tail].am_channel = buffer[nsent];
          fifo->af_buffer[fifo->af_tail].am_data =
            (*(FAR uint16_t *)&buffer[nsent + 1]);
          fifo->af_buffer[fifo->af_tail].am_data <<= 16;
        }
      else if (msglen == 2)
        {
          fifo->af_buffer[fifo->af_tail].am_channel = 0;
          fifo->af_buffer[fifo->af_tail].am_data =
            (*(FAR uint16_t *)&buffer[nsent]);
          fifo->af_buffer[fifo->af_tail].am_data <<= 16;
        }
      else if (msglen == 1)
        {
          fifo->af_buffer[fifo->af_tail].am_channel = 0;
          fifo->af_buffer[fifo->af_tail].am_data    = buffer[nsent];
          fifo->af_buffer[fifo->af_tail].am_data  <<= 24;
        }

      /* Increment the tail of the circular buffer */

      fifo->af_tail = nexttail;

      /* Increment the number of bytes that were sent */

      nsent += msglen;
    }

  /* We get here after all messages have been added to the FIFO.  Check if
   * we need to kick of the XMIT sequence.
   */

  if (empty)
    {
      dac_xmit(dev);
    }

  /* Return the number of bytes that were sent */

  ret = nsent;

return_with_irqdisabled:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: dac_ioctl
 ****************************************************************************/

static int dac_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct dac_dev_s *dev = inode->i_private;
  int ret;

  ret = dev->ad_ops->ao_ioctl(dev, cmd, arg);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dac_txdone
 *
 * Description:
 *   Called from the DAC interrupt handler at the completion of a send
 *   operation.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 ****************************************************************************/

int dac_txdone(FAR struct dac_dev_s *dev)
{
  int ret = -ENOENT;
  int sval;

  /* Verify that the xmit FIFO is not empty */

  if (dev->ad_xmit.af_head != dev->ad_xmit.af_tail)
    {
      /* Remove the message at the head of the xmit FIFO */

      if (++dev->ad_xmit.af_head >= CONFIG_DAC_FIFOSIZE)
        {
          dev->ad_xmit.af_head = 0;
        }

      /* Send the next message in the FIFO */

      ret = dac_xmit(dev);
      if (ret == OK)
        {
          /* Inform any waiting threads that new xmit space is available */

          ret = nxsem_get_value(&dev->ad_xmit.af_sem, &sval);
          if (ret == OK && sval <= 0)
            {
              ret = nxsem_post(&dev->ad_xmit.af_sem);
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: dac_register
 *
 * Description:
 *   Register a dac driver.
 *
 * Input Parameters:
 *    path - The full path to the DAC device to be registered.  This could
 *      be, as an example, "/dev/dac0"
 *    dev - An instance of the device-specific DAC interface
 *
 * Returned Value:
 *    Zero on success; A negated errno value on failure.
 *
 ****************************************************************************/

int dac_register(FAR const char *path, FAR struct dac_dev_s *dev)
{
  /* Initialize the DAC device structure */

  dev->ad_ocount = 0;

  /* Initialize semaphores & mutex */

  nxsem_init(&dev->ad_xmit.af_sem, 0, 0);
  nxmutex_init(&dev->ad_closelock);

  dev->ad_ops->ao_reset(dev);

  return register_driver(path, &g_dac_fops, 0222, dev);
}
