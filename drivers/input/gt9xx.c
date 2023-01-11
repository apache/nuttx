/****************************************************************************
 * drivers/input/gt9xx.c
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

/* Reference:
 * "NuttX RTOS for PinePhone: Touch Panel"
 * https://lupyuen.github.io/articles/touch2
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/gt9xx.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Default I2C Frequency is 400 kHz */

#ifndef CONFIG_INPUT_GT9XX_I2C_FREQUENCY
#  define CONFIG_INPUT_GT9XX_I2C_FREQUENCY 400000
#endif

/* Default Number of Poll Waiters is 1 */

#ifndef CONFIG_INPUT_GT9XX_NPOLLWAITERS
#  define CONFIG_INPUT_GT9XX_NPOLLWAITERS 1
#endif

/* I2C Registers for Goodix GT9XX Touch Panel */

#define GTP_REG_VERSION    0x8140  /* Product ID */
#define GTP_READ_COOR_ADDR 0x814e  /* Touch Panel Status */
#define GTP_POINT1         0x8150  /* Touch Point 1 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Touch Panel Device */

struct gt9xx_dev_s
{
  /* I2C bus and address for device */

  struct i2c_master_s *i2c;
  uint8_t addr;

  /* Callback for Board-Specific Operations */

  const struct gt9xx_board_s *board;

  /* Device State */

  mutex_t devlock;  /* Mutex to prevent concurrent reads */
  uint8_t cref;     /* Reference Counter for device */
  bool int_pending; /* True if a Touch Interrupt is pending processing */
  uint16_t x;       /* X Coordinate of Last Touch Point */
  uint16_t y;       /* Y Coordinate of Last Touch Point */
  uint8_t flags;    /* Touch Up or Touch Down for Last Touch Point */

  /* Poll Waiters for device */

  struct pollfd *fds[CONFIG_INPUT_GT9XX_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gt9xx_open(FAR struct file *filep);
static int gt9xx_close(FAR struct file *filep);
static ssize_t gt9xx_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static int gt9xx_poll(FAR struct file *filep, FAR struct pollfd *fds,
                      bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* File Operations for Touch Panel */

static const struct file_operations g_gt9xx_fileops =
{
  gt9xx_open,   /* open */
  gt9xx_close,  /* close */
  gt9xx_read,   /* read */
  NULL,         /* write */
  NULL,         /* seek */
  NULL,         /* ioctl */
  NULL,         /* truncate */
  NULL,         /* mmap */
  gt9xx_poll    /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gt9xx_i2c_read
 *
 * Description:
 *   Read a Touch Panel Register over I2C.
 *
 * Input Parameters:
 *   dev    - Touch Panel Device
 *   reg    - I2C Register to be read
 *   buf    - Receive Buffer
 *   buflen - Number of bytes to be read
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_i2c_read(FAR struct gt9xx_dev_s *dev,
                          uint16_t reg,
                          uint8_t *buf,
                          size_t buflen)
{
  int ret;

  /* Send the Register Address, MSB first */

  uint8_t regbuf[2] =
  {
    reg >> 8,   /* First Byte: MSB */
    reg & 0xff  /* Second Byte: LSB */
  };

  /* Compose the I2C Messages */

  struct i2c_msg_s msgv[2] =
  {
    {
      /* Send the I2C Register Address */

      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = regbuf,
      .length    = sizeof(regbuf)
    },
    {
      /* Receive the I2C Register Values */

      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = I2C_M_READ,
      .buffer    = buf,
      .length    = buflen
    }
  };

  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);

  iinfo("reg=0x%x, buflen=%ld\n", reg, buflen);
  DEBUGASSERT(dev && dev->i2c && buf);

  /* Execute the I2C Transfer */

  ret = I2C_TRANSFER(dev->i2c, msgv, msgv_len);
  if (ret < 0)
    {
      ierr("I2C Read failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_DEBUG_INPUT_INFO
  iinfodumpbuffer("gt9xx_i2c_read", buf, buflen);
#endif /* CONFIG_DEBUG_INPUT_INFO */

  return OK;
}

/****************************************************************************
 * Name: gt9xx_i2c_write
 *
 * Description:
 *   Write to a Touch Panel Register over I2C.
 *
 * Input Parameters:
 *   dev - Touch Panel Device
 *   reg - I2C Register to be written
 *   val - Value to be written
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_i2c_write(FAR struct gt9xx_dev_s *dev,
                           uint16_t reg,
                           uint8_t val)
{
  int ret;

  /* Send the Register Address, MSB first */

  uint8_t regbuf[2] =
  {
    reg >> 8,   /* First Byte: MSB */
    reg & 0xff  /* Second Byte: LSB */
  };

  /* Send the Register Value */

  uint8_t buf[1] =
  {
    val  /* Value to be written */
  };

  /* Compose the I2C Messages */

  struct i2c_msg_s msgv[2] =
  {
    {
      /* Send the I2C Register Address */

      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = regbuf,
      .length    = sizeof(regbuf)
    },
    {
      /* Send the I2C Register Value */

      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = I2C_M_NOSTART,
      .buffer    = buf,
      .length    = sizeof(buf)
    }
  };

  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);

  iinfo("reg=0x%x, val=%d\n", reg, val);
  DEBUGASSERT(dev && dev->i2c);

  /* Execute the I2C Transfer */

  ret = I2C_TRANSFER(dev->i2c, msgv, msgv_len);
  if (ret < 0)
    {
      ierr("I2C Write failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: gt9xx_probe_device
 *
 * Description:
 *   Read the Product ID from the Touch Panel over I2C.
 *
 * Input Parameters:
 *   dev - Touch Panel Device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_probe_device(FAR struct gt9xx_dev_s *dev)
{
  int ret;
  uint8_t id[4];

  /* Read the Product ID */

  ret = gt9xx_i2c_read(dev, GTP_REG_VERSION, id, sizeof(id));
  if (ret < 0)
    {
      ierr("I2C Probe failed: %d\n", ret);
      return ret;
    }

  /* For GT917S: Product ID will be 39 31 37 53, i.e. "917S" */

#ifdef CONFIG_DEBUG_INPUT_INFO
  iinfodumpbuffer("gt9xx_probe_device", id, sizeof(id));
#endif /* CONFIG_DEBUG_INPUT_INFO */

  return OK;
}

/****************************************************************************
 * Name: gt9xx_set_status
 *
 * Description:
 *   Set the Touch Panel Status over I2C.
 *
 * Input Parameters:
 *   dev    - Touch Panel Device
 *   status - Status value to be set
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_set_status(FAR struct gt9xx_dev_s *dev, uint8_t status)
{
  int ret;

  iinfo("status=%d\n", status);
  DEBUGASSERT(dev);

  /* Write to the Status Register over I2C */

  ret = gt9xx_i2c_write(dev, GTP_READ_COOR_ADDR, status);
  if (ret < 0)
    {
      ierr("Set Status failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: gt9xx_read_touch_data
 *
 * Description:
 *   Read a Touch Sample from Touch Panel. Returns either 0 or 1
 *   Touch Points.
 *
 * Input Parameters:
 *   dev    - Touch Panel Device
 *   sample - Returned Touch Sample (0 or 1 Touch Points)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_read_touch_data(FAR struct gt9xx_dev_s *dev,
                                 FAR struct touch_sample_s *sample)
{
  uint8_t status[1];
  uint8_t status_code;
  uint8_t touched_points;
  uint8_t touch[6];
  uint16_t x;
  uint16_t y;
  uint8_t flags;
  int ret;

  /* Erase the Touch Sample and Touch Point */

  iinfo("\n");
  DEBUGASSERT(dev && sample);
  memset(sample, 0, sizeof(*sample));

  /* Read the Touch Panel Status */

  ret = gt9xx_i2c_read(dev, GTP_READ_COOR_ADDR, status, sizeof(status));
  if (ret < 0)
    {
      ierr("Read Touch Panel Status failed: %d\n", ret);
      return ret;
    }

  /* Decode the Status Code and the Touched Points */

  status_code = status[0] & 0x80;
  touched_points = status[0] & 0x0f;

  /* If Touch Panel Status is OK and Touched Points is 1 or more */

  if (status_code != 0 && touched_points >= 1)
    {
      /* Read the First Touch Point (6 bytes) */

      ret = gt9xx_i2c_read(dev, GTP_POINT1, touch, sizeof(touch));
      if (ret < 0)
        {
          ierr("Read Touch Point failed: %d\n", ret);
          return ret;
        }

      /* Decode the Touch Coordinates */

      x = touch[0] + (touch[1] << 8);
      y = touch[2] + (touch[3] << 8);

      /* Return the Touch Coordinates as Touch Down */

      flags = TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID;
      sample->npoints = 1;
      sample->point[0].id = 0;
      sample->point[0].x = x;
      sample->point[0].y = y;
      sample->point[0].flags = flags;
      iinfo("touch down x=%d, y=%d\n", x, y);
    }

  /* Set the Touch Panel Status to 0 */

  ret = gt9xx_set_status(dev, 0);
  if (ret < 0)
    {
      ierr("Set Touch Panel Status failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: gt9xx_read
 *
 * Description:
 *   Read a Touch Sample from Touch Panel. Returns either 0 or 1
 *   Touch Points.
 *
 * Input Parameters:
 *   dev    - Touch Panel Device
 *   buffer - Returned Touch Sample (0 or 1 Touch Points)
 *   buflen - Size of buffer
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static ssize_t gt9xx_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  struct touch_sample_s sample;
  const size_t outlen = sizeof(sample);
  irqstate_t flags;
  int ret;

  /* Returned Touch Sample will have 0 or 1 Touch Points */

  iinfo("buflen=%ld\n", buflen);
  if (buflen < outlen)
    {
      ierr("Buffer should be at least %ld bytes, got %ld bytes\n",
           outlen, buflen);
      return -EINVAL;
    }

  /* Get the Touch Panel Device */

  DEBUGASSERT(filep);
  inode = filep->f_inode;
  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  /* Begin Mutex: Lock to prevent concurrent reads */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;

  /* If waiting for Touch Up, return the Last Touch Point as Touch Up */

  if (priv->flags & TOUCH_DOWN)
    {
      /* Begin Critical Section */

      flags = enter_critical_section();

      /* Mark the Last Touch Point as Touch Up */

      priv->flags = TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID;

      /* End Critical Section */

      leave_critical_section(flags);

      /* Return the Last Touch Point, changed to Touch Up */

      memset(&sample, 0, sizeof(sample));
      sample.npoints = 1;
      sample.point[0].id = 0;
      sample.point[0].x = priv->x;
      sample.point[0].y = priv->y;
      sample.point[0].flags = priv->flags;
      memcpy(buffer, &sample, sizeof(sample));
      ret = OK;
      iinfo("touch up x=%d, y=%d\n", priv->x, priv->y);
    }
  else
    {
      /* Otherwise read the Touch Point over I2C */

      ret = gt9xx_read_touch_data(priv, &sample);

      /* Skip duplicates */

      if (sample.npoints >= 1 &&
          priv->x == sample.point[0].x &&
          priv->y == sample.point[0].y)
        {
          memset(&sample, 0, sizeof(sample));
          sample.npoints = 0;
          iinfo("skip duplicate x=%d, y=%d\n", priv->x, priv->y);
        }

      /* Return the Touch Point */

      memcpy(buffer, &sample, sizeof(sample));

      /* Begin Critical Section */

      flags = enter_critical_section();

      /* Clear the Interrupt Pending Flag */

      priv->int_pending = false;

      /* Remember the Last Touch Point */

      if (sample.npoints >= 1)
        {
          priv->x = sample.point[0].x;
          priv->y = sample.point[0].y;
          priv->flags = sample.point[0].flags;
        }

      /* End Critical Section */

      leave_critical_section(flags);
    }

  /* End Mutex: Unlock to allow next read */

  nxmutex_unlock(&priv->devlock);
  return (ret < 0) ? ret : outlen;
}

/****************************************************************************
 * Name: gt9xx_open
 *
 * Description:
 *   Open the Touch Panel Device.  If this is the first open, we power on
 *   the Touch Panel, probe for the Touch Panel and enable Touch Panel
 *   Interrupts.
 *
 * Input Parameters:
 *   filep - File Struct for Touch Panel
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  unsigned int use_count;
  int ret;

  /* Get the Touch Panel Device */

  iinfo("\n");
  DEBUGASSERT(filep);
  inode = filep->f_inode;
  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  /* Begin Mutex: Lock to prevent concurrent update to Reference Count */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("Lock Mutex failed: %d\n", ret);
      return ret;
    }

  /* Get next Reference Count */

  use_count = priv->cref + 1;
  DEBUGASSERT(use_count < UINT8_MAX && use_count > priv->cref);
  if (use_count == 1)
    {
      /* If first user, power on the Touch Panel */

      DEBUGASSERT(priv->board->set_power != NULL);
      ret = priv->board->set_power(priv->board, true);
      if (ret < 0)
        {
          goto out_lock;
        }

      /* Let Touch Panel power up before probing */

      nxsig_usleep(100 * 1000);

      /* Check that Touch Panel exists on I2C */

      ret = gt9xx_probe_device(priv);
      if (ret < 0)
        {
          /* No such device, power off the Touch Panel */

          priv->board->set_power(priv->board, false);
          goto out_lock;
        }

      /* Enable Touch Panel Interrupts */

      DEBUGASSERT(priv->board->irq_enable);
      priv->board->irq_enable(priv->board, true);
    }

  /* Set the Reference Count */

  priv->cref = use_count;

  /* End Mutex: Unlock to allow update to Reference Count */

out_lock:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: gt9xx_close
 *
 * Description:
 *   Close the Touch Panel Device.  If this is the final close, we disable
 *   Touch Panel Interrupts and power off the Touch Panel.
 *
 * Input Parameters:
 *   filep - File Struct for Touch Panel
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  int use_count;
  int ret;

  /* Get the Touch Panel Device */

  iinfo("\n");
  DEBUGASSERT(filep);
  inode = filep->f_inode;
  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  /* Begin Mutex: Lock to prevent concurrent update to Reference Count */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("Lock Mutex failed: %d\n", ret);
      return ret;
    }

  /* Decrement the Reference Count */

  use_count = priv->cref - 1;
  DEBUGASSERT(use_count >= 0);
  if (use_count == 0)
    {
      /* If final user, disable Touch Panel Interrupts */

      DEBUGASSERT(priv->board && priv->board->irq_enable);
      priv->board->irq_enable(priv->board, false);

      /* Power off the Touch Panel */

      DEBUGASSERT(priv->board->set_power);
      priv->board->set_power(priv->board, false);
    }

  /* Set the Reference Count */

  priv->cref = use_count;

  /* End Mutex: Unlock to allow update to Reference Count */

  nxmutex_unlock(&priv->devlock);
  return OK;
}

/****************************************************************************
 * Name: gt9xx_poll
 *
 * Description:
 *   Setup or teardown a poll for the Touch Panel Device.
 *
 * Input Parameters:
 *   filep - File Struct for Touch Panel
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *   setup - true: Setup the poll; false: Teardown the poll
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_poll(FAR struct file *filep, FAR struct pollfd *fds,
                      bool setup)
{
  FAR struct gt9xx_dev_s *priv;
  FAR struct inode *inode;
  bool pending;
  int ret = 0;
  int i;

  /* Get the Touch Panel Device */

  iinfo("setup=%d\n", setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;
  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct gt9xx_dev_s *)inode->i_private;

  /* Begin Mutex: Lock to prevent concurrent update to Poll Waiters */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("Lock Mutex failed: %d\n", ret);
      return ret;
    }

  if (setup)
    {
      /* If Poll Setup: Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* Find an available slot for the Poll Waiter */

      for (i = 0; i < CONFIG_INPUT_GT9XX_NPOLLWAITERS; i++)
        {
          /* Found an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_INPUT_GT9XX_NPOLLWAITERS)
        {
          /* No slots available */

          fds->priv = NULL;
          ret = -EBUSY;
        }
      else
        {
          /* If Interrupt Pending is set, notify the Poll Waiters */

          pending = priv->int_pending;
          if (pending)
            {
              poll_notify(priv->fds,
                          CONFIG_INPUT_GT9XX_NPOLLWAITERS,
                          POLLIN);
            }
        }
    }
  else if (fds->priv)
    {
      /* If Poll Teardown: Remove the poll setup */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      *slot = NULL;
      fds->priv = NULL;
    }

  /* End Mutex: Unlock to allow update to Poll Waiters */

out:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: gt9xx_isr_handler
 *
 * Description:
 *   Interrupt Handler for Touch Panel.
 *
 * Input Parameters:
 *   irq     - IRQ Number
 *   context - IRQ Context
 *   arg     - Touch Panel Device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_isr_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct gt9xx_dev_s *priv = (FAR struct gt9xx_dev_s *)arg;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* Begin Critical Section */

  flags = enter_critical_section();

  /* Set the Interrupt Pending Flag */

  priv->int_pending = true;

  /* End Critical Section */

  leave_critical_section(flags);

  /* Notify the Poll Waiters */

  poll_notify(priv->fds, CONFIG_INPUT_GT9XX_NPOLLWAITERS, POLLIN);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gt9xx_register
 *
 * Description:
 *   Register the driver for Goodix GT9XX Touch Panel.  Attach the
 *   Interrupt Handler for the Touch Panel and disable Touch Interrupts.
 *
 * Input Parameters:
 *   devpath      - Device Path (e.g. "/dev/input0")
 *   dev          - I2C Bus
 *   i2c_devaddr  - I2C Address of Touch Panel
 *   board_config - Callback for Board-Specific Operations
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int gt9xx_register(FAR const char *devpath,
                   FAR struct i2c_master_s *i2c_dev,
                   uint8_t i2c_devaddr,
                   const struct gt9xx_board_s *board_config)
{
  struct gt9xx_dev_s *priv;
  int ret = 0;

  iinfo("devpath=%s, i2c_devaddr=%d\n", devpath, i2c_devaddr);
  DEBUGASSERT(devpath != NULL && i2c_dev != NULL && board_config != NULL);

  /* Allocate the Touch Panel Device Structure */

  priv = kmm_zalloc(sizeof(struct gt9xx_dev_s));
  if (!priv)
    {
      ierr("GT9XX Memory Allocation failed\n");
      return -ENOMEM;
    }

  /* Setup the Touch Panel Device Structure */

  priv->addr = i2c_devaddr;
  priv->i2c = i2c_dev;
  priv->board = board_config;
  nxmutex_init(&priv->devlock);

  /* Register the Touch Input Driver */

  ret = register_driver(devpath, &g_gt9xx_fileops, 0666, priv);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      ierr("GT9XX Registration failed: %d\n", ret);
      return ret;
    }

  /* Attach the Interrupt Handler */

  DEBUGASSERT(priv->board->irq_attach);
  priv->board->irq_attach(priv->board, gt9xx_isr_handler, priv);

  /* Disable Touch Panel Interrupts */

  DEBUGASSERT(priv->board->irq_enable);
  priv->board->irq_enable(priv->board, false);

  iinfo("GT9XX Touch Panel registered\n");
  return OK;
}
