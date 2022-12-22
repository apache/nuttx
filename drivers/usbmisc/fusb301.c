/****************************************************************************
 * drivers/usbmisc/fusb301.c
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
#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/usb/fusb301.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FUSB301
#  define fusb301_err(x, ...)        _err(x, ##__VA_ARGS__)
#  define fusb301_info(x, ...)       _info(x, ##__VA_ARGS__)
#else
#  define fusb301_err(x, ...)        uerr(x, ##__VA_ARGS__)
#  define fusb301_info(x, ...)       uinfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_FUSB301_I2C_FREQUENCY
#  define CONFIG_FUSB301_I2C_FREQUENCY 400000
#endif

/* Other macros */

#define FUSB301_I2C_RETRIES  10

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct fusb301_dev_s
{
  FAR struct i2c_master_s *i2c;         /* I2C interface */
  uint8_t addr;                         /* I2C address */
  volatile bool int_pending;            /* Interrupt received but handled */
  mutex_t devlock;                      /* Manages exclusive access */
  FAR struct fusb301_config_s *config;  /* Platform specific configuration */
  FAR struct pollfd *fds[CONFIG_FUSB301_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

static int fusb301_open(FAR struct file *filep);
static int fusb301_close(FAR struct file *filep);
static ssize_t fusb301_read(FAR struct file *, FAR char *, size_t);
static ssize_t fusb301_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int fusb301_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int fusb301_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fusb301ops =
{
  fusb301_open,  /* open */
  fusb301_close, /* close */
  fusb301_read,  /* read */
  fusb301_write, /* write */
  NULL,          /* seek */
  fusb301_ioctl, /* ioctl */
  NULL,          /* truncate */
  fusb301_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fusb301_getreg
 *
 * Description:
 *   Read from an 8-bit FUSB301 register
 *
 * Input Parameters:
 *   priv   - pointer to FUSB301 Private Structure
 *   reg    - register to read
 *
 * Returned Value:
 *   Returns positive register value in case of success, otherwise ERROR
 ****************************************************************************/

static int fusb301_getreg(FAR struct fusb301_dev_s *priv, uint8_t reg)
{
  int ret = -EIO;
  int retries;
  uint8_t regval;
  struct i2c_msg_s msg[2];

  DEBUGASSERT(priv);

  msg[0].frequency = CONFIG_FUSB301_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_FUSB301_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  /* Perform the transfer */

  for (retries = 0; retries < FUSB301_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->i2c, msg, 2);
      if (ret >= 0)
        {
          fusb301_info("reg:%02X, value:%02X\n", reg, regval);
          return regval;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == FUSB301_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              fusb301_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  fusb301_info("reg:%02X, error:%d\n", reg, ret);
  return ret;
}

/****************************************************************************
 * Name: fusb301_putreg
 *
 * Description:
 *   Write a value to an 8-bit FUSB301 register
 *
 * Input Parameters:
 *   priv    - pointer to FUSB301 Private Structure
 *   regaddr - register to read
 *   regval  - value to be written
 *
 * Returned Value:
 *   None
 ****************************************************************************/

static int fusb301_putreg(FAR struct fusb301_dev_s *priv, uint8_t regaddr,
                          uint8_t regval)
{
  int ret = -EIO;
  int retries;
  struct i2c_msg_s msg;
  uint8_t txbuffer[2];

  /* Setup to the data to be transferred (register address and data). */

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  /* Setup 8-bit FUSB301 address write message */

  msg.frequency = CONFIG_FUSB301_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = 2;

  /* Perform the transfer */

  for (retries = 0; retries < FUSB301_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->i2c, &msg, 1);
      if (ret == OK)
        {
          fusb301_info("reg:%02X, value:%02X\n", regaddr, regval);

          return OK;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == FUSB301_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              fusb301_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  fusb301_err("ERROR: failed reg:%02X, value:%02X, error:%d\n",
              regaddr, regval, ret);
  return ret;
}

/****************************************************************************
 * Name: fusb301_read_device_id
 *
 * Description:
 *   Read device version and revision IDs
 *
 ****************************************************************************/

static int fusb301_read_device_id(FAR struct fusb301_dev_s * priv,
                                  FAR uint8_t * arg)
{
  int ret;

  ret = fusb301_getreg(priv, FUSB301_DEV_ID_REG);
  if (ret < 0)
    {
      fusb301_err("ERROR: Failed to read device ID\n");
      return -EIO;
    }

  *arg = ret;
  return OK;
}

/****************************************************************************
 * Name: fusb301_clear_interrupts
 *
 * Description:
 *   Clear interrupts from FUSB301 chip
 *
 ****************************************************************************/

static int fusb301_clear_interrupts(FAR struct fusb301_dev_s *priv)
{
  int ret = OK;

  ret = fusb301_getreg(priv, FUSB301_INTERRUPT_REG);
  if (ret < 0)
    {
      fusb301_err("ERROR: Failed to clear interrupts\n");
      return -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: fusb301_setup
 *
 * Description:
 *   Setup FUSB301 chip
 *
 ****************************************************************************/

static int fusb301_setup(FAR struct fusb301_dev_s *priv,
                         struct fusb301_setup_s *setup)
{
  int ret = OK;

  fusb301_info("drp_tgl:%02X, host_curr:%02X, global_int:%X, mask:%02X\n",
    setup->drp_toggle_timing, setup->host_current, setup->global_int_mask,
    setup->int_mask);

  ret = fusb301_putreg(priv, FUSB301_CONTROL_REG, setup->drp_toggle_timing |
    setup->host_current | setup->global_int_mask);

  if (ret < 0)
    {
      fusb301_err("ERROR: Failed to write control register\n");
      goto err_out;
    }

  ret = fusb301_putreg(priv, FUSB301_MASK_REG, setup->int_mask);
  if (ret < 0)
    {
      fusb301_err("ERROR: Failed to write mask register\n");
    }

err_out:
  return ret;
}

/****************************************************************************
 * Name: fusb301_set_mode
 *
 * Description:
 *   Configure supported device modes (sink, source, DRP, accessory)
 *
 ****************************************************************************/

static int fusb301_set_mode(FAR struct fusb301_dev_s *priv,
                            enum fusb301_mode_e mode)
{
  int ret = OK;

  if (mode > MODE_DRP_ACC)
    {
      return -EINVAL;
    }

  ret = fusb301_putreg(priv, FUSB301_MODE_REG, mode);
  if (ret < 0)
    {
      fusb301_err("ERROR: Failed to write mode register\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: fusb301_set_state
 *
 * Description:
 *   Force device in specified state
 *
 ****************************************************************************/

static int fusb301_set_state(FAR struct fusb301_dev_s *priv,
                             enum fusb301_manual_e state)
{
  int ret = OK;

  if (state > MANUAL_UNATT_SNK)
    {
      return -EINVAL;
    }

  ret = fusb301_putreg(priv, FUSB301_MANUAL_REG, state);
  if (ret < 0)
    {
      fusb301_err("ERROR: Failed to write manual register\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: fusb301_read_status
 *
 * Description:
 *   Read status register
 *
 ****************************************************************************/

static int fusb301_read_status(FAR struct fusb301_dev_s *priv,
                               FAR uint8_t *arg)
{
  int ret;

  ret = fusb301_getreg(priv, FUSB301_STATUS_REG);
  if (ret < 0)
    {
      fusb301_err("ERROR: Failed to read status\n");
      return -EIO;
    }

  *arg = ret;
  return OK;
}

/****************************************************************************
 * Name: fusb301_read_devtype
 *
 * Description:
 *   Read type of attached device
 *
 ****************************************************************************/

static int fusb301_read_devtype(FAR struct fusb301_dev_s *priv,
                                FAR uint8_t *arg)
{
  int ret;

  ret = fusb301_getreg(priv, FUSB301_TYPE_REG);
  if (ret < 0)
    {
      fusb301_err("ERROR: Failed to read type\n");
      return -EIO;
    }

  *arg = ret;
  return OK;
}

/****************************************************************************
 * Name: fusb301_reset
 *
 * Description:
 *   Reset FUSB301 HW and clear I2C registers
 *
 ****************************************************************************/

static int fusb301_reset(FAR struct fusb301_dev_s *priv)
{
  int ret = OK;

  ret = fusb301_putreg(priv, FUSB301_RESET_REG, RESET_SW_RES);
  if (ret < 0)
    {
      fusb301_err("ERROR: Failed to write reset register\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: fusb301_open
 *
 * Description:
 *   This function is called whenever the FUSB301 device is opened.
 *
 ****************************************************************************/

static int fusb301_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fusb301_dev_s *priv = inode->i_private;
  int ret = OK;

  /* Probe device */

  ret = fusb301_getreg(priv, FUSB301_DEV_ID_REG);
  if (ret < 0)
    {
      fusb301_err("ERROR: No response at given address 0x%02X\n",
                  priv->addr);
      ret = -EFAULT;
    }
  else
    {
      fusb301_info("device id: 0x%02X\n", ret);

      fusb301_clear_interrupts(priv);
      priv->config->irq_enable(priv->config, true);
    }

  /* Error exit */

  return ret;
}

/****************************************************************************
 * Name: fusb301_close
 *
 * Description:
 *   This routine is called when the FUSB301 device is closed.
 *
 ****************************************************************************/

static int fusb301_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fusb301_dev_s *priv = inode->i_private;

  priv->config->irq_enable(priv->config, false);

  return OK;
}

/****************************************************************************
 * Name: fusb301_read
 * Description:
 *   This routine is called when the FUSB301 device is read.
 ****************************************************************************/

static ssize_t fusb301_read(FAR struct file *filep,
                            FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fusb301_dev_s *priv = inode->i_private;
  FAR struct fusb301_result_s *ptr;
  irqstate_t flags;
  int ret;

  if (buflen < sizeof(struct fusb301_result_s))
    {
      return 0;
    }

  ptr = (struct fusb301_result_s *)buffer;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();
  priv->int_pending = false;
  leave_critical_section(flags);

  fusb301_clear_interrupts(priv);

  ptr->status = fusb301_getreg(priv, FUSB301_STATUS_REG);
  ptr->dev_type = fusb301_getreg(priv, FUSB301_TYPE_REG);

  nxmutex_unlock(&priv->devlock);
  return sizeof(struct fusb301_result_s);
}

/****************************************************************************
 * Name: fusb301_write
 * Description:
 *   This routine is called when the FUSB301 device is written to.
 ****************************************************************************/

static ssize_t fusb301_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  ssize_t length = 0;

  return length;
}

/****************************************************************************
 * Name: fusb301_ioctl
 * Description:
 *   This routine is called when ioctl function call is performed for
 *   the FUSB301 device.
 ****************************************************************************/

static int fusb301_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fusb301_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  fusb301_info("cmd: 0x%02X, arg:%lu\n", cmd, arg);

  switch (cmd)
  {
  case USBCIOC_READ_DEVID:
    {
      ret = fusb301_read_device_id(priv, (uint8_t *)arg);
    }
    break;

  case USBCIOC_SETUP:
    {
      ret = fusb301_setup(priv, (struct fusb301_setup_s *)arg);
    }
    break;

  case USBCIOC_SET_MODE:
    {
      ret = fusb301_set_mode(priv, (uint8_t)arg);
    }
    break;

  case USBCIOC_SET_STATE:
    {
      ret = fusb301_set_state(priv, (uint8_t)arg);
    }
    break;

  case USBCIOC_READ_STATUS:
    {
      ret = fusb301_read_status(priv, (uint8_t *)arg);
    }
    break;

  case USBCIOC_READ_DEVTYPE:
    {
      ret = fusb301_read_devtype(priv, (uint8_t *)arg);
    }
    break;

  case USBCIOC_RESET:
    {
      ret = fusb301_reset(priv);
    }
    break;

  default:
    {
      fusb301_err("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
    }
    break;
  }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: fusb301_poll
 * Description:
 *   This routine is called during FUSB301 device poll
 ****************************************************************************/

static int fusb301_poll(FAR struct file *filep,
                        FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode;
  FAR struct fusb301_dev_s *priv;
  irqstate_t flags;
  int ret = OK;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct fusb301_dev_s *)inode->i_private;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* This is a request to set up the poll. Find an available
       * slot for the poll structure reference.
       */

      for (i = 0; i < CONFIG_FUSB301_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_FUSB301_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }

      flags = enter_critical_section();
      if (priv->int_pending)
        {
          poll_notify(priv->fds, CONFIG_FUSB301_NPOLLWAITERS, POLLIN);
        }

      leave_critical_section(flags);
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

out:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: fusb301_callback
 *
 * Description:
 *   FUSB301 interrupt handler
 *
 ****************************************************************************/

static int fusb301_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct fusb301_dev_s *priv = (FAR struct fusb301_dev_s *)arg;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  flags = enter_critical_section();
  priv->int_pending = true;

  poll_notify(priv->fds, CONFIG_FUSB301_NPOLLWAITERS, POLLIN);
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int fusb301_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr, FAR struct fusb301_config_s *config)
{
  FAR struct fusb301_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL && config != NULL);

  /* Initialize the FUSB301 device structure */

  priv = (FAR struct fusb301_dev_s *)
                kmm_zalloc(sizeof(struct fusb301_dev_s));
  if (!priv)
    {
      fusb301_err("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Initialize device structure mutex */

  nxmutex_init(&priv->devlock);

  priv->int_pending = false;
  priv->i2c         = i2c;
  priv->addr        = addr;
  priv->config      = config;

  /* Register the character driver */

  ret = register_driver(devpath, &g_fusb301ops, 0666, priv);
  if (ret < 0)
    {
      fusb301_err("ERROR: Failed to register driver: %d\n", ret);
      goto errout_with_priv;
    }

  /* Prepare interrupt line and handler. */

  if (priv->config->irq_clear)
    {
      priv->config->irq_clear(config);
    }

  priv->config->irq_attach(config, fusb301_int_handler, priv);
  priv->config->irq_enable(config, false);

  return OK;

errout_with_priv:
  nxmutex_destroy(&priv->devlock);
  kmm_free(priv);
  return ret;
}
