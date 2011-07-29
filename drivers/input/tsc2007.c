/****************************************************************************
 * drivers/input/tsc2007.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * References:
 *   "1.2V to 3.6V, 12-Bit, Nanopower, 4-Wire Micro TOUCH SCREEN CONTROLLER
 *    with I2C Interface," SBAS405A March 2007, Revised, March 2009, Texas
 *    Instruments Incorporated
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

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/fs.h>
#include <nuttx/i2c.h>
#include <nuttx/input/tsc2007.h>

#include "tsc2007.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Driver support ***********************************************************/
/* This format is used to construct the /dev/skel[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/input%d"
#define DEV_NAMELEN  16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tsc2007_dev_s
{
  uint8_t               crefs;    /* Number of times the device has been opened */
  sem_t                 devsem;   /* Manages exclusive access to this structure */

  FAR struct tsc2007_config_s *config; /* Board configuration data */
  FAR struct i2c_dev_s *i2c;      /* Saved I2C driver instance */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd        *fds[CONFIG_TSC2007_NPOLLWAITERS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
#ifndef CONFIG_DISABLE_POLL
static void tsc2007_pollnotify(FAR struct tsc2007_dev_s *priv,
                               pollevent_t eventset);
#endif
static int tsc2007_transfer(FAR struct tsc2007_dev_s *priv, uint8_t cmd);
static void tsc2007_sample(FAR struct tsc2007_dev_s *tsc,
                           FAR struct tsc2007_sample_s *sample);
static int tsc2007_interrupt(int irq, FAR void *context);

/* Character driver methods */

static int tsc2007_open(FAR struct file *filep);
static int tsc2007_close(FAR struct file *filep);
static ssize_t tsc2007_read(FAR struct file *filep, FAR char *buffer, size_t len);
static ssize_t tsc2007_write(FAR struct file *filep, FAR const char *buffer, size_t len);
static int tsc2007_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int tsc2007_poll(FAR struct file *filep, struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations tsc2007_fops =
{
  tsc2007_open,    /* open */
  tsc2007_close,   /* close */
  tsc2007_read,    /* read */
  tsc2007_write,   /* write */
  0,               /* seek */
  tsc2007_ioctl    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , tsc2007_poll   /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tsc2007_pollnotify
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static void tsc2007_pollnotify(FAR struct tsc2007_dev_s *priv,
                               pollevent_t eventset)
{
  int i;

  for (i = 0; i < CONFIG_TSC2007_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= (fds->events & eventset);
          if (fds->revents != 0)
            {
              ivdbg("Report events: %02x\n", fds->revents);
              sem_post(fds->sem);
            }
        }
    }
}
#else
#  define tsc2007_pollnotify(priv,event)
#endif

/****************************************************************************
 * Name: tsc2007_transfer
 ****************************************************************************/

static int tsc2007_transfer(FAR struct tsc2007_dev_s *priv, uint8_t cmd)
{
   struct i2c_msg_s msg;
   uint8_t data12[2];
   int ret;

  /* "A conversion/write cycle begins when the master issues the address
   *  byte containing the slave address of the TSC2007, with the eighth bit
   *  equal to a 0 (R/W = 0)... Once the eighth bit has been received...
   *  the TSC2007 issues an acknowledge.
   *
   * "When the master receives the acknowledge bit from the TSC2007, the
   *  master writes the command byte to the slave... After the command byte
   *  is received by the slave, the slave issues another acknowledge bit.
   *  The master then ends the write cycle by issuing a repeated START or a
   *  STOP condition...
   */

   msg.addr   = priv->config->address; /* 7-bit address */
   msg.flags  = 0;                     /* Write transaction, beginning with START */
   msg.buffer = &cmd;                  /* Transfer from this address */
   msg.length = 1;                     /* Send one byte following the address */
 
   ret = I2C_TRANSFER(priv->i2c, &msg, 1);
   if (ret < 0)
     {
       idbg("I2C_TRANSFER failed: %d\n", ret);
       return ret;
     }

  /* "The input multiplexer channel for the A/D converter is selected when
   *  bits C3 through C0 are clocked in. If the selected channel is an X-,Y-,
   *  or Z-position measurement, the appropriate drivers turn on once the
   *  acquisition period begins.
   *
   * "... the input sample acquisition period starts on the falling edge of
   *  SCL when the C0 bit of the command byte has been latched, and ends
   *  when a STOP or repeated START condition has been issued. A/D conversion
   *  starts immediately after the acquisition period...
   *
   * "For best performance, the I2C bus should remain in an idle state while
   *  an A/D conversion is taking place. ... The master should wait for at
   *  least 10ms before attempting to read data from the TSC2007...
   */

  usleep(10*1000);

  /* "Data access begins with the master issuing a START condition followed
   *  by the address byte ... with R/W = 1.
   *
   * "When the eighth bit has been received and the address matches, the
   *  slave issues an acknowledge. The first byte of serial data then follows
   *  (D11-D4, MSB first).
   *
   * "After the first byte has been sent by the slave, it releases the SDA line
   *  for the master to issue an acknowledge.  The slave responds with the
   *  second byte of serial data upon receiving the acknowledge from the master
   *  (D3-D0, followed by four 0 bits). The second byte is followed by a NOT
   *  acknowledge bit (ACK = 1) from the master to indicate that the last
   *  data byte has been received...
   */

   msg.addr   = priv->config->address; /* 7-bit address */
   msg.flags  = I2C_M_READ;            /* Read transaction, beginning with START */
   msg.buffer = data12;                /* Transfer two this address */
   msg.length = 2;                     /* Read two bytes following the address */
 
   ret = I2C_TRANSFER(priv->i2c, &msg, 1);
   if (ret < 0)
     {
       idbg("I2C_TRANSFER failed: %d\n", ret);
       return ret;
     }

   ret = (unsigned int)data12[0] << 4 | (unsigned int)data12[1];
   ivdbg(&tsc->client->dev, "data: 0x%03x\n", ret);
   return ret;
}

/****************************************************************************
 * Name: tsc2007_sample
 ****************************************************************************/

static void tsc2007_sample(FAR struct tsc2007_dev_s *tsc,
                           FAR struct tsc2007_sample_s *sample)
{
  /* ADC is on... Disable interrupts and read Y and X positions */

  sample->y = tsc2007_transfer(tsc, (TSC2007_CMD_12BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_YPOS));
  sample->x = tsc2007_transfer(tsc, (TSC2007_CMD_12BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_XPOS));

  /* Read Z1 and Z2 positions */

  sample->z1 = tsc2007_transfer(tsc, (TSC2007_CMD_12BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_Z1POS));
  sample->z2 = tsc2007_transfer(tsc, (TSC2007_CMD_12BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_Z2POS));

  /* Power down ADC and enable PENIRQ */

  (void)tsc2007_transfer(tsc, (TSC2007_CMD_12BIT | TSC2007_CMD_PWRDN_IRQEN));
}

/****************************************************************************
 * Name: tsc2007_interrupt
 ****************************************************************************/

static int tsc2007_interrupt(int irq, FAR void *context)
{
# warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: tsc2007_open
 ****************************************************************************/

static int tsc2007_open(FAR struct file *filep)
{
  FAR struct inode         *inode;
  FAR struct tsc2007_dev_s *priv;
  uint8_t                   tmp;
  int                       ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tsc2007_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Increment the reference count */

  tmp = priv->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Check if this is the first time that the driver has been opened. */

  if (tmp == 1)
    {
      irqstate_t flags = irqsave();

      /* Perform one time hardware initialization */

#warning "Missing logic"
      irqrestore(flags);
    }

  /* Save the new open count on success */

  priv->crefs = tmp;

errout_with_sem:
  sem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: tsc2007_close
 ****************************************************************************/

static int tsc2007_close(FAR struct file *filep)
{
  FAR struct inode         *inode;
  FAR struct tsc2007_dev_s *priv;
  int                       ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tsc2007_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Decrement the reference count unless it would decrement to zero */
  if (priv->crefs > 1)
    {
      priv->crefs--;
      sem_post(&priv->devsem);
      return OK;
    }

  /* There are no more references to the port */

  priv->crefs = 0;

  /* Perform driver teardown */
#warning "Missing logic"

  sem_post(&priv->devsem);
  return OK;
}

/****************************************************************************
 * Name: tsc2007_read
 ****************************************************************************/

static ssize_t tsc2007_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode         *inode;
  FAR struct tsc2007_dev_s *priv;
  int                       ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tsc2007_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

#warning "Not implemented"

  sem_post(&priv->devsem);
  return -ENOSYS;
}

/****************************************************************************
 * Name: tsc2007_write
 ****************************************************************************/

static ssize_t tsc2007_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  FAR struct inode         *inode;
  FAR struct tsc2007_dev_s *priv;
  int                       ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tsc2007_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

#warning "Not implemented"
  sem_post(&priv->devsem);
  return -ENOSYS;
}

/****************************************************************************
 * Name:tsc2007_ioctl
 ****************************************************************************/

static int tsc2007_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode         *inode;
  FAR struct tsc2007_dev_s *priv;
  int                       ret;

  ivdbg("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tsc2007_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      /* Add support for ioctl commands here */

      default:
        ret = -ENOTTY;
        break;
    }

  sem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: tsc2007_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int tsc2007_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode         *inode;
  FAR struct tsc2007_dev_s *priv;
  pollevent_t               eventset;
  int                       ndx;
  int                       ret = OK;
  int                       i;

  ivdbg("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tsc2007_dev_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_TSC2007_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i]  = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_TSC2007_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should immediately notify on any of the requested events? */
#warning "Missing logic"


    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;

#ifdef CONFIG_DEBUG
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
  sem_post(&priv->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tsc2007_register
 *
 * Description:
 *   Configure the TSC2007 to use the provided I2C device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   dev     - An I2C driver instance
 *   config  - Persistant board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int tsc2007_register(FAR struct i2c_dev_s *dev,
                     FAR struct tsc2007_config_s *config, int minor)
{
  FAR struct tsc2007_dev_s *priv;
  char devname[DEV_NAMELEN];
  int ret;

  ivdbg("dev: %p minor: %d\n", dev, minor);
  DEBUGASSERT(dev != NULL && config != NULL && minor > 0 && minor < 100);
  DEBUGASSERT((config->address & 0xfc) == 0x48);

  /* Create and initialize a TSC2007 device driver instance */

  priv = (FAR struct tsc2007_dev_s *)kmalloc(sizeof(struct tsc2007_dev_s));
  if (!priv)
    {
      idbg("kmalloc(%d) failed\n", sizeof(struct tsc2007_dev_s));
      return -ENOMEM;
    }

  /* Initialize the TSC2007 device driver instance */

  memset(priv, 0, sizeof(struct tsc2007_dev_s));
  priv->i2c    = dev;             /* Save the I2C device handle */
  priv->config = config;          /* Save the board configuration */
  sem_init(&priv->devsem,  0, 1); /* Initialize device structure semaphore */

  /* Attach the interrupt handler */

  ret = config->attach(config, tsc2007_interrupt);
  if (ret < 0)
    {
      idbg("Failed to attach interrupt\n");
      goto errout_with_priv;
    }

  /* Power down the ADC and enable PENIRQ.  This is the normal state while
   * waiting for a touch event.
   */

  ret = tsc2007_transfer(priv, (TSC2007_CMD_12BIT | TSC2007_CMD_PWRDN_IRQEN));
  if (ret < 0)
    {
      idbg("tsc2007_transfer failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* Register the device as an input device */

  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, minor);
  ivdbg("Registering %s\n", devname);
  ret = register_driver(devname, &tsc2007_fops, 0666, priv);
  if (ret < 0)
    {
      idbg("register_driver() failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* Enable the interrupt */

  ret = config->enable(config, true);
  if (ret < 0)
    {
      idbg("Failed enable interrupt\n");
      goto errout_with_priv;
    }
  return ret;

errout_with_priv:
  sem_destroy(&priv->devsem);
  kfree(priv);
  return ret;
}
