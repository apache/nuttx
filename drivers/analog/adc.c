/****************************************************************************
 * drivers/analog/adc.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/random.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     adc_open(FAR struct file *filep);
static int     adc_close(FAR struct file *filep);
static ssize_t adc_read(FAR struct file *fielp, FAR char *buffer,
                        size_t buflen);
static int     adc_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int     adc_reset(FAR struct adc_dev_s *dev);
static int     adc_receive(FAR struct adc_dev_s *dev, uint8_t ch,
                           int32_t data);
static void    adc_notify(FAR struct adc_dev_s *dev);
static int     adc_poll(FAR struct file *filep, struct pollfd *fds,
                        bool setup);
static int     adc_reset_fifo(FAR struct adc_dev_s *dev);
static int     adc_samples_on_read(FAR struct adc_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_adc_fops =
{
  adc_open,     /* open */
  adc_close,    /* close */
  adc_read,     /* read */
  NULL,         /* write */
  NULL,         /* seek */
  adc_ioctl,    /* ioctl */
  adc_poll      /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL        /* unlink */
#endif
};

static const struct adc_callback_s g_adc_callback =
{
  adc_receive,    /* au_receive */
  adc_reset       /* au_reset */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_open
 *
 * Description:
 *   This function is called whenever the ADC device is opened.
 *
 ****************************************************************************/

static int adc_open(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct adc_dev_s *dev   = inode->i_private;
  uint8_t               tmp;
  int                   ret;

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
                  /* Mark the FIFOs empty */

                  dev->ad_recv.af_head = 0;
                  dev->ad_recv.af_tail = 0;

                  /* Clear overrun indicator */

                  dev->ad_isovr = false;

                  /* Finally, Enable the ADC RX interrupt */

                  dev->ad_ops->ao_rxint(dev, true);
                }

              leave_critical_section(flags);
            }

          /* Save the new open count on success */

          dev->ad_ocount = tmp;
        }

      nxsem_post(&dev->ad_closesem);
    }

  return ret;
}

/****************************************************************************
 * Name: adc_close
 *
 * Description:
 *   This routine is called when the ADC device is closed.
 *   It waits for the last remaining data to be sent.
 *
 ****************************************************************************/

static int adc_close(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct adc_dev_s *dev   = inode->i_private;
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

          /* Free the IRQ and disable the ADC device */

          flags = enter_critical_section();    /* Disable interrupts */
          dev->ad_ops->ao_shutdown(dev);       /* Disable the ADC */
          leave_critical_section(flags);

          nxsem_post(&dev->ad_closesem);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: adc_read
 ****************************************************************************/

static ssize_t adc_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct adc_dev_s *dev   = inode->i_private;
  size_t                nread;
  irqstate_t            flags;
  int                   ret   = 0;
  int                   msglen;

  ainfo("buflen: %d\n", (int)buflen);

  /* Determine the size of the messages to return.
   *
   * REVISIT:  What if buflen is 8 does that mean 4 messages of size 2?  Or
   * 2 messages of size 4?  What if buflen is 12.  Does that mean 3 at size
   * 4?  Or 4 at size 3?  The form of the return data should probably really
   * be specified via IOCTL.
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

  if (buflen >= msglen)
    {
      /* Interrupts must be disabled while accessing the ad_recv FIFO */

      flags = enter_critical_section();
      while (dev->ad_recv.af_head == dev->ad_recv.af_tail)
        {
          /* Check if there was an overrun, if set we need to return EIO */

          if (dev->ad_isovr)
            {
              dev->ad_isovr = false;
              ret = -EIO;
              goto return_with_irqdisabled;
            }

          /* The receive FIFO is empty -- was non-blocking mode selected? */

          if (filep->f_oflags & O_NONBLOCK)
            {
              ret = -EAGAIN;
              goto return_with_irqdisabled;
            }

          /* Wait for a message to be received */

          dev->ad_nrxwaiters++;
          ret = nxsem_wait(&dev->ad_recv.af_sem);
          dev->ad_nrxwaiters--;
          if (ret < 0)
            {
              goto return_with_irqdisabled;
            }
        }

      /* The ad_recv FIFO is not empty.  Copy all buffered data that will fit
       * in the user buffer.
       */

      nread = 0;
      do
        {
          FAR struct adc_msg_s *msg =
            &dev->ad_recv.af_buffer[dev->ad_recv.af_head];

          /* Will the next message in the FIFO fit into the user buffer? */

          if (nread + msglen > buflen)
            {
              /* No.. break out of the loop now with nread equal to the
               * actual number of bytes transferred.
               */

              break;
            }

          /* Feed ADC data to entropy pool */

          add_sensor_randomness(msg->am_data);

          /* Copy the message to the user buffer */

          if (msglen == 1)
            {
              /* Only one channel, return MS 8-bits of the sample. */

              buffer[nread] = msg->am_data >> 24;
            }
          else if (msglen == 2)
            {
              /* Only one channel, return only the MS 16-bits of the sample.
               */

              int16_t data16 = msg->am_data >> 16;
              memcpy(&buffer[nread], &data16, 2);
            }
          else if (msglen == 3)
            {
              int16_t data16;

              /* Return the channel and the MS 16-bits of the sample. */

              buffer[nread] = msg->am_channel;
              data16 = msg->am_data >> 16;
              memcpy(&buffer[nread + 1], &data16, 2);
            }
          else if (msglen == 4)
            {
              int32_t data24;

#ifdef CONFIG_ENDIAN_BIG
              /* In the big endian case, we simply copy the MS three bytes
               * which are indices: 0-2.
               */

              data24 = msg->am_data;
#else
              /* In the little endian case, indices 0-2 correspond to the
               * the three LS bytes.
               */

              data24 = msg->am_data >> 8;
#endif

              /* Return the channel and the most significant 24-bits */

              buffer[nread] = msg->am_channel;
              memcpy(&buffer[nread + 1], &data24, 3);
            }
          else
            {
              /* Return the channel and all four bytes of the sample */

              buffer[nread] = msg->am_channel;
              memcpy(&buffer[nread + 1], &msg->am_data, 4);
            }

          nread += msglen;

          /* Increment the head of the circular message buffer */

          if (++dev->ad_recv.af_head >= CONFIG_ADC_FIFOSIZE)
            {
              dev->ad_recv.af_head = 0;
            }
        }
      while (dev->ad_recv.af_head != dev->ad_recv.af_tail);

      /* All of the messages have been transferred.  Return the number of
       * bytes that were read.
       */

      ret = nread;

return_with_irqdisabled:
      leave_critical_section(flags);
    }

  ainfo("Returning: %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: adc_ioctl
 ****************************************************************************/

static int adc_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adc_dev_s *dev = inode->i_private;
  int ret;

  switch (cmd)
    {
      case ANIOC_RESET_FIFO:
        {
          ret = adc_reset_fifo(dev);
        }
        break;

      case ANIOC_SAMPLES_ON_READ:
        {
          ret = adc_samples_on_read(dev);
        }
        break;

      default:
        {
          /* Those IOCTLs might be used in arch specific section */

          ret = dev->ad_ops->ao_ioctl(dev, cmd, arg);
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: adc_reset
 ****************************************************************************/

static int adc_reset(FAR struct adc_dev_s *dev)
{
  /* Set overrun flag to give read a chance to recover */

  dev->ad_isovr = true;

  /* No need to notify here. The adc_receive callback will be called next.
   * If an ADC overrun occurs then there must be at least one conversion.
   */

  return OK;
}

/****************************************************************************
 * Name: adc_receive
 ****************************************************************************/

static int adc_receive(FAR struct adc_dev_s *dev, uint8_t ch, int32_t data)
{
  FAR struct adc_fifo_s *fifo = &dev->ad_recv;
  int                    nexttail;
  int                    errcode = -ENOMEM;

  /* Check if adding this new message would over-run the drivers ability to
   * enqueue read data.
   */

  nexttail = fifo->af_tail + 1;
  if (nexttail >= CONFIG_ADC_FIFOSIZE)
    {
      nexttail = 0;
    }

  /* Refuse the new data if the FIFO is full */

  if (nexttail != fifo->af_head)
    {
      /* Add the new, decoded ADC sample at the tail of the FIFO */

      fifo->af_buffer[fifo->af_tail].am_channel = ch;
      fifo->af_buffer[fifo->af_tail].am_data    = data;

      /* Increment the tail of the circular buffer */

      fifo->af_tail = nexttail;

      adc_notify(dev);

      errcode = OK;
    }

  return errcode;
}

/****************************************************************************
 * Name: adc_pollnotify
 ****************************************************************************/

static void adc_pollnotify(FAR struct adc_dev_s *dev, uint32_t type)
{
  int i;

  for (i = 0; i < CONFIG_ADC_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = dev->fds[i];
      if (fds)
        {
          fds->revents |= type;
          nxsem_post(fds->sem);
        }
    }
}

/****************************************************************************
 * Name: adc_notify
 ****************************************************************************/

static void adc_notify(FAR struct adc_dev_s *dev)
{
  FAR struct adc_fifo_s *fifo = &dev->ad_recv;

  /* If there are threads waiting on poll() for data to become available,
   * then wake them up now.
   */

  adc_pollnotify(dev, POLLIN);

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (dev->ad_nrxwaiters > 0)
    {
      nxsem_post(&fifo->af_sem);
    }
}

/****************************************************************************
 * Name: adc_poll
 ****************************************************************************/

static int adc_poll(FAR struct file *filep, struct pollfd *fds, bool setup)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct adc_dev_s *dev   = inode->i_private;
  irqstate_t flags;
  int ret = 0;
  int i;

  /* Interrupts must be disabled while accessing the list of poll structures
   * and ad_recv FIFO.
   */

  flags = enter_critical_section();

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto return_with_irqdisabled;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_ADC_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!dev->fds[i])
            {
              /* Bind the poll structure and this slot */

              dev->fds[i] = fds;
              fds->priv   = &dev->fds[i];
              break;
            }
        }

      if (i >= CONFIG_ADC_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto return_with_irqdisabled;
        }

      /* Should we immediately notify on any of the requested events? */

      if (dev->ad_recv.af_head != dev->ad_recv.af_tail)
        {
          adc_pollnotify(dev, POLLIN);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

return_with_irqdisabled:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: adc_reset_fifo
 ****************************************************************************/

static int adc_reset_fifo(FAR struct adc_dev_s *dev)
{
  irqstate_t flags;
  FAR struct adc_fifo_s *fifo = &dev->ad_recv;

  /* Interrupts must be disabled while accessing the ad_recv FIFO */

  flags = enter_critical_section();

  fifo->af_head = fifo->af_tail;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: adc_samples_on_read
 ****************************************************************************/

static int adc_samples_on_read(FAR struct adc_dev_s *dev)
{
  irqstate_t flags;
  FAR struct adc_fifo_s *fifo = &dev->ad_recv;
  int16_t ret;

  /* Interrupts must be disabled while accessing the ad_recv FIFO */

  flags = enter_critical_section();

  ret = fifo->af_tail - fifo->af_head;

  leave_critical_section(flags);

  if (ret < 0)
    {
      /* Increment return value by the size of FIFO */

      ret += CONFIG_ADC_FIFOSIZE;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_register
 ****************************************************************************/

int adc_register(FAR const char *path, FAR struct adc_dev_s *dev)
{
  int ret;

  DEBUGASSERT(path != NULL && dev != NULL);

  /* Bind the upper-half callbacks to the lower half ADC driver */

  DEBUGASSERT(dev->ad_ops != NULL && dev->ad_ops->ao_bind != NULL);
  ret = dev->ad_ops->ao_bind(dev, &g_adc_callback);
  if (ret < 0)
    {
      aerr("ERROR: Failed to bind callbacks: %d\n", ret);
      return ret;
    }

  /* Initialize the ADC device structure */

  dev->ad_ocount = 0;

  /* Initialize semaphores */

  nxsem_init(&dev->ad_recv.af_sem, 0, 0);
  nxsem_init(&dev->ad_closesem, 0, 1);

  /* The receive semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_set_protocol(&dev->ad_recv.af_sem, SEM_PRIO_NONE);

  /* Reset the ADC hardware */

  DEBUGASSERT(dev->ad_ops->ao_reset != NULL);
  dev->ad_ops->ao_reset(dev);

  /* Register the ADC character driver */

  ret = register_driver(path, &g_adc_fops, 0444, dev);
  if (ret < 0)
    {
      nxsem_destroy(&dev->ad_recv.af_sem);
      nxsem_destroy(&dev->ad_closesem);
    }

  return ret;
}
