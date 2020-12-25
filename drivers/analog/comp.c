/****************************************************************************
 * drivers/analog/comp.c
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
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/analog/comp.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     comp_open(FAR struct file *filep);
static int     comp_close(FAR struct file *filep);
static ssize_t comp_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static int     comp_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
static int     comp_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup);
static int     comp_notify(FAR struct comp_dev_s *dev, uint8_t val);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations comp_fops =
{
  comp_open,                    /* open */
  comp_close,                   /* close */
  comp_read,                    /* read */
  NULL,                         /* write */
  NULL,                         /* seek */
  comp_ioctl,                   /* ioctl */
  comp_poll                     /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                        /* unlink */
#endif
};

static const struct comp_callback_s g_comp_callback =
  {
    comp_notify   /* au_notify */
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: comp_pollnotify
 *
 * Description:
 *   This function is called to notify any waiters of poll-reated events.
 *
 ****************************************************************************/

static void comp_pollnotify(FAR struct comp_dev_s *dev,
                            pollevent_t eventset)
{
  int i;

  if (eventset & POLLERR)
    {
      eventset &= ~(POLLOUT | POLLIN);
    }

  for (i = 0; i < CONFIG_DEV_COMP_NPOLLWAITERS; i++)
    {
      FAR struct pollfd *fds = dev->d_fds[i];

      if (fds)
        {
          fds->revents |= eventset & (fds->events | POLLERR | POLLHUP);

          if ((fds->revents & (POLLOUT | POLLHUP)) == (POLLOUT | POLLHUP))
            {
              /* POLLOUT and POLLHUP are mutually exclusive. */

              fds->revents &= ~POLLOUT;
            }

          if (fds->revents != 0)
            {
              ainfo("Report events: %02x\n", fds->revents);
              nxsem_post(fds->sem);
            }
        }
    }
}

/****************************************************************************
 * Name: comp_semtake
 ****************************************************************************/

static int comp_semtake(FAR sem_t *sem)
{
  return nxsem_wait_uninterruptible(sem);
}

/****************************************************************************
 * Name: comp_poll
 ****************************************************************************/

static int comp_poll(FAR struct file *filep, FAR struct pollfd *fds,
                     bool setup)
{
  FAR struct inode      *inode    = filep->f_inode;
  FAR struct comp_dev_s *dev      = inode->i_private;
  int                    ret      = OK;
  int                    i;

  DEBUGASSERT(dev && fds);

  /* Are we setting up the poll?  Or tearing it down? */

  ret = comp_semtake(&dev->ad_sem);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_DEV_COMP_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!dev->d_fds[i])
            {
              /* Bind the poll structure and this slot */

              dev->d_fds[i] = fds;
              fds->priv     = &dev->d_fds[i];
              break;
            }
        }

      if (i >= CONFIG_DEV_COMP_NPOLLWAITERS)
        {
          fds->priv   = NULL;
          ret         = -EBUSY;
          goto errout;
        }
    }
  else
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

#ifdef CONFIG_DEBUG_FEATURES
      if (!slot)
        {
          ret              = -EIO;
          goto errout;
        }
#endif

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  nxsem_post(&dev->ad_sem);
  return ret;
}

/****************************************************************************
 * Name: comp_notify
 *
 * Description:
 *   This function is called from the lower half driver to notify the change
 *   of the comparator output.
 *
 ****************************************************************************/

static int comp_notify(FAR struct comp_dev_s *dev, uint8_t val)
{
  /* TODO: store values in FIFO? */

  dev->val = val;

  comp_pollnotify(dev, POLLIN);
  nxsem_post(&dev->ad_readsem);

  return 0;
}

/****************************************************************************
 * Name: comp_open
 *
 * Description:
 *   This function is called whenever the COMP device is opened.
 *
 ****************************************************************************/

static int comp_open(FAR struct file *filep)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct comp_dev_s *dev   = inode->i_private;
  uint8_t                tmp;
  int                    ret;

  /* If the port is the middle of closing, wait until the close is
   * finished.
   */

  ret = nxsem_wait(&dev->ad_sem);
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

      nxsem_post(&dev->ad_sem);
    }

  return ret;
}

/****************************************************************************
 * Name: comp_close
 *
 * Description:
 *   This routine is called when the COMP device is closed.
 *   It waits for the last remaining data to be sent.
 *
 ****************************************************************************/

static int comp_close(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct comp_dev_s *dev   = inode->i_private;
  irqstate_t            flags;
  int                   ret;

  ret = nxsem_wait(&dev->ad_sem);
  if (ret >= 0)
    {
      /* Decrement the references to the driver.  If the reference count will
       * decrement to 0, then uninitialize the driver.
       */

      if (dev->ad_ocount > 1)
        {
          dev->ad_ocount--;
          nxsem_post(&dev->ad_sem);
        }
      else
        {
          /* There are no more references to the port */

          dev->ad_ocount = 0;

          /* Free the IRQ and disable the COMP device */

          flags = enter_critical_section();       /* Disable interrupts */
          dev->ad_ops->ao_shutdown(dev);          /* Disable the COMP */
          leave_critical_section(flags);

          nxsem_post(&dev->ad_sem);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: comp_read
 ****************************************************************************/

static ssize_t comp_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct comp_dev_s *dev   = inode->i_private;
  int                    ret;

  /* If non-blocking read, read the value immediately and return. */

  if (filep->f_oflags & O_NONBLOCK)
    {
      ret = dev->ad_ops->ao_read(dev);
      buffer[0] = (uint8_t)ret;
      return 1;
    }

  ret = nxsem_wait(&dev->ad_readsem);
  if (ret < 0)
    {
      aerr("nxsem_wait() failed: %d\n", ret);
      return ret;
    }

  buffer[0] = dev->val;

  return 1;
}

/****************************************************************************
 * Name: comp_ioctl
 ****************************************************************************/

static int comp_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct comp_dev_s *dev = inode->i_private;
  int ret;

  ret = dev->ad_ops->ao_ioctl(dev, cmd, arg);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: comp_register
 ****************************************************************************/

int comp_register(FAR const char *path, FAR struct comp_dev_s *dev)
{
  int ret;

  /* Initialize the COMP device structure */

  dev->ad_ocount = 0;

  /* Initialize semaphores */

  nxsem_init(&dev->ad_sem, 0, 1);
  nxsem_set_protocol(&dev->ad_sem, SEM_PRIO_NONE);

  nxsem_init(&dev->ad_readsem, 0, 0);
  nxsem_set_protocol(&dev->ad_readsem, SEM_PRIO_NONE);

  /* Bind the upper-half callbacks to the lower half COMP driver */

  DEBUGASSERT(dev->ad_ops != NULL);

  if (dev->ad_ops->ao_bind != NULL)
    {
      ret = dev->ad_ops->ao_bind(dev, &g_comp_callback);
      if (ret < 0)
        {
          aerr("ERROR: Failed to bind callbacks: %d\n", ret);
          return ret;
        }
    }

  /* Register the COMP character driver */

  ret =  register_driver(path, &comp_fops, 0444, dev);
  if (ret < 0)
    {
      nxsem_destroy(&dev->ad_sem);
    }

  return ret;
}
