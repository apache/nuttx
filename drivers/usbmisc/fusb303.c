/****************************************************************************
 * drivers/usbmisc/fusb303.c
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

#include <nuttx/compiler.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/usb/fusb303.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FUSB303
#  define fusb303_err(x, ...)        _err(x, ##__VA_ARGS__)
#  define fusb303_info(x, ...)       _info(x, ##__VA_ARGS__)
#else
#  define fusb303_err(x, ...)        uerr(x, ##__VA_ARGS__)
#  define fusb303_info(x, ...)       uinfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_FUSB303_I2C_FREQUENCY
#  define CONFIG_FUSB303_I2C_FREQUENCY 400000
#endif

/* Other macros */

#define FUSB303_I2C_RETRIES  10

#define FUSB303_ALL_INTR     (INTERRUPT_ATTACH | INTERRUPT_DETACH | \
                              INTERRUPT_BC_LVL | INTERRUPT_AUTOSNK | \
                              INTERRUPT_VBUS_CHG | INTERRUPT_FAULT | \
                              INTERRUPT_ORIENT)

#define FUSB303_ALL_INTR1    (INTERRUPT1_REMEDY | INTERRUPT1_FRC_SUCC | \
                              INTERRUPT1_FRC_FAIL | INTERRUPT1_REM_FAIL | \
                              INTERRUPT1_REM_VBON | INTERRUPT1_REM_VBOFF)

/* Debug */

#ifdef CONFIG_DEBUG_FUSB303
#  define DUMPREG(priv, x) \
    do \
      { \
        int ret = fusb303_getreg((priv), (x)); \
        if (ret < 0) \
          { \
            fusb303_err("ERROR: Failed to read %s(0x%02X)\n", #x, (x)); \
          } \
        else \
          { \
            fusb303_info("%s(0x%02X): 0x%02X\n", #x, (x), ret); \
          } \
      } \
    while(0)
#endif

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct fusb303_dev_s
{
  FAR struct i2c_master_s *i2c;         /* I2C interface */
  uint8_t addr;                         /* I2C address */
  volatile bool int_pending;            /* Interrupt received but handled */
  mutex_t devlock;                      /* Manages exclusive access */
  FAR struct fusb303_config_s *config;  /* Platform specific configuration */
  FAR struct pollfd *fds[CONFIG_FUSB303_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FUSB303
static int fusb303_dumpregs(FAR const char *funcname,
                            FAR struct fusb303_dev_s *priv);
#endif
static int fusb303_open(FAR struct file *filep);
static int fusb303_close(FAR struct file *filep);
static ssize_t fusb303_read(FAR struct file *, FAR char *, size_t);
static ssize_t fusb303_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int fusb303_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int fusb303_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fusb303ops =
{
  fusb303_open,  /* open */
  fusb303_close, /* close */
  fusb303_read,  /* read */
  fusb303_write, /* write */
  NULL,          /* seek */
  fusb303_ioctl, /* ioctl */
  NULL,          /* truncate */
  fusb303_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fusb303_getreg
 *
 * Description:
 *   Read from an 8-bit FUSB303 register
 *
 * Input Parameters:
 *   priv   - pointer to FUSB303 Private Structure
 *   reg    - register to read
 *
 * Returned Value:
 *   Returns positive register value in case of success, otherwise ERROR
 *
 ****************************************************************************/

static int fusb303_getreg(FAR struct fusb303_dev_s *priv, uint8_t reg)
{
  int ret = -EIO;
  int retries;
  uint8_t regval;
  struct i2c_msg_s msg[2];

  DEBUGASSERT(priv);

  msg[0].frequency = CONFIG_FUSB303_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_FUSB303_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  /* Perform the transfer */

  for (retries = 0; retries < FUSB303_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->i2c, msg, 2);
      if (ret >= 0)
        {
          fusb303_info("reg:%02X, value:%02X\n", reg, regval);
          return regval;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == FUSB303_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              fusb303_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  fusb303_info("reg:%02X, error:%d\n", reg, ret);
  return ret;
}

/****************************************************************************
 * Name: fusb303_putreg
 *
 * Description:
 *   Write a value to an 8-bit FUSB303 register
 *
 * Input Parameters:
 *   priv    - pointer to FUSB303 Private Structure
 *   regaddr - register to read
 *   regval  - value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int fusb303_putreg(FAR struct fusb303_dev_s *priv, uint8_t regaddr,
                          uint8_t regval)
{
  int ret = -EIO;
  int retries;
  struct i2c_msg_s msg;
  uint8_t txbuffer[2];

  /* Setup to the data to be transferred (register address and data). */

  txbuffer[0]   = regaddr;
  txbuffer[1]   = regval;

  /* Setup 8-bit FUSB303 address write message */

  msg.frequency = CONFIG_FUSB303_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = 2;

  /* Perform the transfer */

  for (retries = 0; retries < FUSB303_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->i2c, &msg, 1);
      if (ret == OK)
        {
          fusb303_info("reg:%02X, value:%02X\n", regaddr, regval);

          return OK;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == FUSB303_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              fusb303_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  fusb303_err("ERROR: failed reg:%02X, value:%02X, error:%d\n",
              regaddr, regval, ret);
  return ret;
}

/****************************************************************************
 * Name: fusb303_dumpregs
 *
 * Description:
 *   Dump FUSB303 registers
 *
 * Input Parameters:
 *   priv    - pointer to FUSB303 Private Structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FUSB303
static int noinline_function fusb303_dumpregs(FAR const char *funcname,
                                              FAR struct fusb303_dev_s *priv)
{
  fusb303_info("'%s':\n", funcname);
  DUMPREG(priv, FUSB303_DEV_ID_REG);
  DUMPREG(priv, FUSB303_DEV_TYPE_REG);
  DUMPREG(priv, FUSB303_PORTROLE_REG);
  DUMPREG(priv, FUSB303_CONTROL_REG);
  DUMPREG(priv, FUSB303_CONTROL1_REG);
  DUMPREG(priv, FUSB303_MANUAL_REG);
  DUMPREG(priv, FUSB303_RESET_REG);
  DUMPREG(priv, FUSB303_MASK_REG);
  DUMPREG(priv, FUSB303_MASK1_REG);
  DUMPREG(priv, FUSB303_STATUS_REG);
  DUMPREG(priv, FUSB303_STATUS1_REG);
  DUMPREG(priv, FUSB303_TYPE_REG);
  DUMPREG(priv, FUSB303_INTERRUPT_REG);
  DUMPREG(priv, FUSB303_INTERRUPT1_REG);
  return OK;
}
#endif /* CONFIG_DEBUG_FUSB303 */

/****************************************************************************
 * Name: fusb303_read_device_id
 *
 * Description:
 *   Read device version, revision ID and type.
 *
 ****************************************************************************/

static int fusb303_read_device_id(FAR struct fusb303_dev_s *priv,
                                  FAR uint8_t *dev_id, FAR uint8_t *dev_type)
{
  int ret;

  ret = fusb303_getreg(priv, FUSB303_DEV_ID_REG);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to read device ID\n");
      return -EIO;
    }

  if (dev_id != NULL)
    {
      *dev_id = ret;
    }

  ret = fusb303_getreg(priv, FUSB303_DEV_TYPE_REG);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to read device type\n");
      return -EIO;
    }

  if (dev_type != NULL)
    {
      *dev_type = ret;
    }

  return ret;
}

/****************************************************************************
 * Name: fusb303_clear_interrupts
 *
 * Description:
 *   Clear interrupts from FUSB303 chip
 *
 ****************************************************************************/

static int fusb303_clear_interrupts(FAR struct fusb303_dev_s *priv)
{
  int ret;

  ret = fusb303_putreg(priv, FUSB303_INTERRUPT_REG, FUSB303_ALL_INTR);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to clear interrupts\n");
      return -EIO;
    }

  ret = fusb303_putreg(priv, FUSB303_INTERRUPT1_REG, FUSB303_ALL_INTR1);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to clear interrupts\n");
      return -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: fusb303_setup
 *
 * Description:
 *   Setup FUSB303 chip
 *
 ****************************************************************************/

static int fusb303_setup(FAR struct fusb303_dev_s *priv,
                         struct fusb303_setup_s *setup)
{
  int ret = OK;
  uint8_t regval;

  fusb303_info("drp_tgl:%02X, host_curr:%02X\n"
               "dcable_en: %d, remedy_en: %d, auto_snk_en: %d\n"
               "global_int: %d, mask: %02X, mask1: %02X\n",
               setup->drp_toggle_timing, setup->host_current,
               (int)setup->dcable_en, (int)setup->remedy_en,
               (int)setup->auto_snk_en, (int)setup->global_int_mask,
               setup->int_mask, setup->int_mask1);

  /* Enable chip in I2C mode. */

  ret = fusb303_getreg(priv, FUSB303_CONTROL1_REG);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to enable chip\n");
      goto err_out;
    }

  /* TODO: no way to change AUTO_SNK_TH or TCCDEB at the moment. */

  regval = (uint8_t)ret | CONTROL1_ENABLE;
  if (setup->auto_snk_en)
    {
      regval |= CONTROL1_AUTO_SNK_EN;
    }
  else
    {
      regval &= ~CONTROL1_AUTO_SNK_EN;
    }

  if (setup->remedy_en)
    {
      regval |= CONTROL1_REMEDY_EN;
    }
  else
    {
      regval &= ~CONTROL1_REMEDY_EN;
    }

  ret = fusb303_putreg(priv, FUSB303_CONTROL1_REG, regval);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to enable chip\n");
      goto err_out;
    }

  /* Setup the interrupt masks and remaining settings. */

  ret = fusb303_putreg(priv, FUSB303_MASK_REG, setup->int_mask);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to write mask register\n");
      goto err_out;
    }

  ret = fusb303_putreg(priv, FUSB303_MASK1_REG, setup->int_mask1);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to write mask register\n");
      goto err_out;
    }

  /* Interrupts can happen only after unmasking global_int_mask */

  regval = setup->drp_toggle_timing | setup->host_current |
           setup->global_int_mask;
  if (setup->dcable_en)
    {
      regval |= CONTROL_DCABLE_EN;
    }

  ret = fusb303_putreg(priv, FUSB303_CONTROL_REG, regval);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to write control register\n");
      goto err_out;
    }

err_out:
#ifdef CONFIG_DEBUG_FUSB303
  fusb303_dumpregs("fusb303_setup", priv);
#endif
  return ret;
}

/****************************************************************************
 * Name: fusb303_set_mode
 *
 * Description:
 *   Configure supported device modes (sink, source, DRP, accessory)
 *
 ****************************************************************************/

static int fusb303_set_mode(FAR struct fusb303_dev_s *priv,
                            enum fusb303_mode_e mode)
{
  int ret;

  if (mode > MODE_ORIENTDEB)
    {
      return -EINVAL;
    }

  ret = fusb303_putreg(priv, FUSB303_PORTROLE_REG, mode);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to set portrole\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: fusb303_set_state
 *
 * Description:
 *   Force device in specified state
 *
 ****************************************************************************/

static int fusb303_set_state(FAR struct fusb303_dev_s *priv,
                             enum fusb303_manual_e state)
{
  int ret;

  if (state > MANUAL_FORCE_SRC)
    {
      return -EINVAL;
    }

  ret = fusb303_putreg(priv, FUSB303_MANUAL_REG, state);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to set state\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: fusb303_read_status
 *
 * Description:
 *   Read status register
 *
 ****************************************************************************/

static int fusb303_read_status(FAR struct fusb303_dev_s *priv,
                               FAR uint8_t *arg)
{
  int ret;

  ret = fusb303_getreg(priv, FUSB303_STATUS_REG);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to read status\n");
      return -EIO;
    }

  *arg = ret;
  return OK;
}

/****************************************************************************
 * Name: fusb303_read_devtype
 *
 * Description:
 *   Read type of attached device
 *
 ****************************************************************************/

static int fusb303_read_devtype(FAR struct fusb303_dev_s *priv,
                                FAR uint8_t *arg)
{
  int ret;

  ret = fusb303_getreg(priv, FUSB303_TYPE_REG);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to read type\n");
      return -EIO;
    }

  *arg = ret;
  return OK;
}

/****************************************************************************
 * Name: fusb303_reset
 *
 * Description:
 *   Reset FUSB303 HW and clear I2C registers
 *
 ****************************************************************************/

static int fusb303_reset(FAR struct fusb303_dev_s *priv)
{
  int ret;

  ret = fusb303_putreg(priv, FUSB303_RESET_REG, RESET_SW_RES);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to reset chip\n");
      ret = -EIO;
    }

  /* tRESET max 100 ms. */

  up_mdelay(100);

  return ret;
}

/****************************************************************************
 * Name: fusb303_open
 *
 * Description:
 *   This function is called whenever the FUSB303 device is opened.
 *
 ****************************************************************************/

static int fusb303_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fusb303_dev_s *priv = inode->i_private;
  uint8_t dev_id;
  uint8_t dev_type;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Probe device */

  ret = fusb303_read_device_id(priv, &dev_id, &dev_type);
  if (ret < 0)
    {
      fusb303_err("ERROR: No response at given address 0x%02X\n",
                  priv->addr);
      ret = -EFAULT;
    }
  else
    {
      fusb303_info("device id: 0x%02X type: 0x%02X\n", dev_id, dev_type);

      fusb303_clear_interrupts(priv);
      priv->config->irq_enable(priv->config, true);
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: fusb303_close
 *
 * Description:
 *   This routine is called when the FUSB303 device is closed.
 *
 ****************************************************************************/

static int fusb303_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fusb303_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  priv->config->irq_enable(priv->config, false);

  nxmutex_unlock(&priv->devlock);
  return OK;
}

/****************************************************************************
 * Name: fusb303_read
 *
 * Description:
 *   This routine is called when the FUSB303 device is read.
 *
 ****************************************************************************/

static ssize_t fusb303_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fusb303_dev_s *priv = inode->i_private;
  FAR struct fusb303_result_s *ptr;
  irqstate_t flags;
  int ret;

  if (buflen < sizeof(struct fusb303_result_s))
    {
      return 0;
    }

  ptr = (struct fusb303_result_s *)buffer;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();
  priv->int_pending = false;
  leave_critical_section(flags);

  ptr->status = fusb303_getreg(priv, FUSB303_STATUS_REG);
  ptr->status1 = fusb303_getreg(priv, FUSB303_STATUS1_REG);
  ptr->dev_type = fusb303_getreg(priv, FUSB303_TYPE_REG);

#ifdef CONFIG_DEBUG_FUSB303
  fusb303_dumpregs("fusb303_read", priv);
#endif

  fusb303_clear_interrupts(priv);

  nxmutex_unlock(&priv->devlock);
  return sizeof(struct fusb303_result_s);
}

/****************************************************************************
 * Name: fusb303_write
 *
 * Description:
 *   This routine is called when the FUSB303 device is written to.
 *
 ****************************************************************************/

static ssize_t fusb303_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  ssize_t length = 0;

  return length;
}

/****************************************************************************
 * Name: fusb303_ioctl
 *
 * Description:
 *   This routine is called when ioctl function call is performed for
 *   the FUSB303 device.
 *
 ****************************************************************************/

static int fusb303_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fusb303_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  fusb303_info("cmd: 0x%02X, arg:%lu\n", cmd, arg);

  switch (cmd)
  {
  case USBCIOC_READ_DEVID:
    {
      ret = fusb303_read_device_id(priv, (uint8_t *)arg, NULL);
    }
    break;

  case USBCIOC_SETUP:
    {
      ret = fusb303_setup(priv, (struct fusb303_setup_s *)arg);
    }
    break;

  case USBCIOC_SET_MODE:
    {
      ret = fusb303_set_mode(priv, (uint8_t)arg);
    }
    break;

  case USBCIOC_SET_STATE:
    {
      ret = fusb303_set_state(priv, (uint8_t)arg);
    }
    break;

  case USBCIOC_READ_STATUS:
    {
      ret = fusb303_read_status(priv, (uint8_t *)arg);
    }
    break;

  case USBCIOC_READ_DEVTYPE:
    {
      ret = fusb303_read_devtype(priv, (uint8_t *)arg);
    }
    break;

  case USBCIOC_RESET:
    {
      ret = fusb303_reset(priv);
    }
    break;

  default:
    {
      fusb303_err("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
    }
    break;
  }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: fusb303_poll
 *
 * Description:
 *   This routine is called during FUSB303 device poll
 *
 ****************************************************************************/

static int fusb303_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode;
  FAR struct fusb303_dev_s *priv;
  irqstate_t flags;
  int ret = OK;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct fusb303_dev_s *)inode->i_private;

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

      for (i = 0; i < CONFIG_FUSB303_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_FUSB303_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }

      flags = enter_critical_section();
      if (priv->int_pending)
        {
          poll_notify(priv->fds, CONFIG_FUSB303_NPOLLWAITERS, POLLIN);
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
 * Name: fusb303_callback
 *
 * Description:
 *   FUSB303 interrupt handler
 *
 ****************************************************************************/

static int fusb303_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct fusb303_dev_s *priv = (FAR struct fusb303_dev_s *)arg;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  flags = enter_critical_section();
  priv->int_pending = true;

  poll_notify(priv->fds, CONFIG_FUSB303_NPOLLWAITERS, POLLIN);
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int fusb303_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr, FAR struct fusb303_config_s *config)
{
  FAR struct fusb303_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL && config != NULL);

  /* Initialize the FUSB303 device structure */

  priv = (FAR struct fusb303_dev_s *)
                             kmm_zalloc(sizeof(struct fusb303_dev_s));
  if (!priv)
    {
      fusb303_err("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Initialize device structure mutex */

  nxmutex_init(&priv->devlock);

  priv->int_pending = false;
  priv->i2c         = i2c;
  priv->addr        = addr;
  priv->config      = config;

  /* Register the character driver */

  ret = register_driver(devpath, &g_fusb303ops, 0666, priv);
  if (ret < 0)
    {
      fusb303_err("ERROR: Failed to register driver: %d\n", ret);
      goto errout_with_priv;
    }

  /* Prepare interrupt line and handler. */

  if (priv->config->irq_clear)
    {
      priv->config->irq_clear(config);
    }

  priv->config->irq_attach(config, fusb303_int_handler, priv);
  priv->config->irq_enable(config, false);

  return OK;

errout_with_priv:
  nxmutex_destroy(&priv->devlock);
  kmm_free(priv);
  return ret;
}
