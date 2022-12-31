/****************************************************************************
 * drivers/sensors/kxtj9.c
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
#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mutex.h>
#include <nuttx/sensors/kxtj9.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_KXTJ9)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_KXTJ9_I2C_BUS_SPEED
#  define CONFIG_KXTJ9_I2C_BUS_SPEED 400000
#endif

/* Register Definitions *****************************************************/

/* Output registers */

#define XOUT_L          0x06
#define WHO_AM_I        0x0f
#define DCST_RESP       0x0c

/* Control registers */

#define INT_REL         0x1a
#define CTRL_REG1       0x1b
#define INT_CTRL1       0x1e
#define DATA_CTRL       0x21
#define CTRL_REG2       0x1d

/* Control register 1 bits */

#define PC1_OFF         0x7f
#define PC1_ON          (1 << 7)

/* CTRL_REG1: set resolution, g-range, data ready enable */

/* Output resolution: 8-bit valid or 12-bit valid */

#define RES_8BIT        0
#define RES_12BIT       (1 << 6)

/* Data ready function enable bit: set during probe if using irq mode */

#define DRDYE           (1 << 5)

/* Output g-range: +/-2g, 4g, or 8g */

#define KXTJ9_G_2G      0
#define KXTJ9_G_4G      (1 << 3)
#define KXTJ9_G_8G      (1 << 4)

/* Interrupt control register 1 bits */

/* Set these during probe if using irq mode */

#define KXTJ9_IEL       (1 << 3)
#define KXTJ9_IEA       (1 << 4)
#define KXTJ9_IEN       (1 << 5)

#define KXTJ9_SRST      0x80
#define WHO_AM_I_KXCJ9  0x0a

#define KXTJ9_CTRL1_CONFIG  (RES_12BIT | KXTJ9_G_2G | DRDYE)

/* Misc. driver definitions *************************************************/

#define ACCEL_NUM_RETRIES   5

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of one KXTJ9 device */

struct kxtj9_dev_s
{
  FAR struct i2c_master_s *i2c;
  mutex_t lock;
  bool enable;
  bool power_enabled;
  uint8_t address;
  uint8_t shift;
  uint8_t ctrl_reg1;
  uint8_t data_ctrl;
  uint8_t int_ctrl;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C helpers */

static int     kxtj9_reg_read(FAR struct kxtj9_dev_s *priv, uint8_t regaddr,
                              FAR uint8_t *regval, unsigned int len);
static int     kxtj9_reg_write(FAR struct kxtj9_dev_s *priv,
                               uint8_t regaddr,  uint8_t regval);

/* KXTJ9 helpers */

static int     kxtj9_configure(FAR struct kxtj9_dev_s *priv, uint8_t odr);
static int     kxtj9_enable(FAR struct kxtj9_dev_s *priv, bool on);
static int     kxtj9_read_sensor_data(FAR struct kxtj9_dev_s *priv,
                 FAR struct kxtj9_sensor_data *sensor_data);
static void    kxtj9_soft_reset(FAR struct kxtj9_dev_s *priv);
static void    kxtj9_set_mode_standby(FAR struct kxtj9_dev_s *priv);

/* Character driver methods */

static ssize_t kxtj9_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t kxtj9_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     kxtj9_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fops =
{
  NULL,            /* open */
  NULL,            /* close */
  kxtj9_read,      /* read */
  kxtj9_write,     /* write */
  NULL,            /* seek */
  kxtj9_ioctl,     /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kxtj9_reg_read
 *
 * Description:
 *   Read from multiple KXTJ9 registers.
 *
 ****************************************************************************/

static int kxtj9_reg_read(FAR struct kxtj9_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *regval, unsigned int len)
{
  struct i2c_msg_s msg[2];
  uint8_t buf[1];
  int retries = ACCEL_NUM_RETRIES;
  int ret;

  do
    {
      /* Format two messages: The first is a write containing the register
       * address
       */

      buf[0]           = regaddr;

      msg[0].frequency = CONFIG_KXTJ9_I2C_BUS_SPEED,
      msg[0].addr      = priv->address;
      msg[0].flags     = 0;
      msg[0].buffer    = buf;
      msg[0].length    = 1;

      /* The second is a read with a restart containing the register data */

      msg[1].frequency = CONFIG_KXTJ9_I2C_BUS_SPEED,
      msg[1].addr      = priv->address;
      msg[1].flags     = I2C_M_READ;
      msg[1].buffer    = regval;
      msg[1].length    = len;

      /* Then perform the transfer. */

      ret = I2C_TRANSFER(priv->i2c, msg, 2);
    }
  while (ret < 0 && retries-- > 0);

  return ret;
}

/****************************************************************************
 * Name: kxtj9_reg_write
 *
 * Description:
 *   Write a value to a single KXTJ9 register
 *
 ****************************************************************************/

static int kxtj9_reg_write(FAR struct kxtj9_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_msg_s msg;
  uint8_t buf[2];
  int ret;
  int retries = ACCEL_NUM_RETRIES;

  do
    {
      /* Setup for the transfer */

      buf[0]        = regaddr;
      buf[1]        = regval;

      msg.frequency = CONFIG_KXTJ9_I2C_BUS_SPEED,
      msg.addr      = priv->address;
      msg.flags     = 0;
      msg.buffer    = buf;
      msg.length    = 2;

      /* Then perform the transfer. */

      ret = I2C_TRANSFER(priv->i2c, &msg, 1);
    }
  while (ret < 0 && retries-- > 0);

  return ret;
}

/****************************************************************************
 * Name: kxtj9_soft_reset
 *
 * Description:
 *   Configure the KXTJ9 device.  Handler the SNIOC_CONFIGURE IOCTL command.
 *
 ****************************************************************************/

static void kxtj9_soft_reset(FAR struct kxtj9_dev_s *priv)
{
  uint8_t wbuf[1];

  /* Set accel into standby and known state by disabling PC1 */

  wbuf[0] = KXTJ9_CTRL1_CONFIG;
  kxtj9_reg_write(priv, CTRL_REG1, wbuf[0]);

  /* Send the reset command */

  kxtj9_reg_read(priv, CTRL_REG2, &wbuf[0], 1);

  wbuf[0] |= KXTJ9_SRST;
  kxtj9_reg_write(priv, CTRL_REG2, wbuf[0]);

  /* Delay 10ms for the accel parts to re-initialize */

  nxsig_usleep(10000);
}

/****************************************************************************
 * Name: kxtj9_set_mode_standby
 *
 * Description:
 *   Configure the KXTJ9 device.  Handler the SNIOC_CONFIGURE IOCTL command.
 *
 ****************************************************************************/

static void kxtj9_set_mode_standby(FAR struct kxtj9_dev_s *priv)
{
  uint8_t wbuf[1];

  /* Set Accel into standby and known state by disabling PC1 */

  wbuf[0] = KXTJ9_CTRL1_CONFIG;
  kxtj9_reg_write(priv, CTRL_REG1, wbuf[0]);

  /* Clear interrupts */

  wbuf[0] = 0;
  kxtj9_reg_write(priv, INT_CTRL1, wbuf[0]);
}

/****************************************************************************
 * Name: kxtj9_configure
 *
 * Description:
 *   Configure the KXTJ9 device.  Handler the SNIOC_CONFIGURE IOCTL command.
 *
 ****************************************************************************/

static int kxtj9_configure(FAR struct kxtj9_dev_s *priv, uint8_t odr)
{
  uint8_t wbuf[0];
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  kxtj9_soft_reset(priv);
  kxtj9_set_mode_standby(priv);

  /* Read WHO_AM_I register, should return 0x0a */

  kxtj9_reg_read(priv, WHO_AM_I, &wbuf[0], 1);
  if (wbuf[0] != WHO_AM_I_KXCJ9)
    {
      snerr("ERROR: Not KXCJ9 chipset, WHO_AM_I register is 0x%2x.\n",
            wbuf[0]);
    }

  /* Ensure that PC1 is cleared before updating control registers */

  kxtj9_reg_write(priv, CTRL_REG1, 0);

  /* 12Bit Res and -2G~+2G range */

  priv->ctrl_reg1 = KXTJ9_CTRL1_CONFIG;
  kxtj9_reg_write(priv, CTRL_REG1, priv->ctrl_reg1);

  priv->data_ctrl = odr;
  kxtj9_reg_write(priv, DATA_CTRL, priv->data_ctrl);

  /* In irq mode, populate INT_CTRL */

  priv->int_ctrl = KXTJ9_IEN | KXTJ9_IEA | KXTJ9_IEL;
  kxtj9_reg_write(priv, INT_CTRL1, priv->int_ctrl);

  nxmutex_unlock(&priv->lock);
  return 0;
}

/****************************************************************************
 * Name: kxtj9_enable
 *
 * Description:
 *   Enable or disable the KXTJ9 device.  Handler the SNIOC_ENABLE and
 *   SNIOC_DISABLE IOCTL commands.
 *
 ****************************************************************************/

static int kxtj9_enable(FAR struct kxtj9_dev_s *priv, bool on)
{
  uint8_t wbuf[1];
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!on && priv->power_enabled)
    {
      priv->ctrl_reg1 &= PC1_OFF;
      kxtj9_reg_write(priv, CTRL_REG1, priv->ctrl_reg1);

      priv->power_enabled = false;
      sninfo("KXTJ9 in disabled mode\n");
    }
  else if (on && !priv->power_enabled)
    {
      /* Turn on outputs */

      priv->ctrl_reg1 |= PC1_ON;
      kxtj9_reg_write(priv, CTRL_REG1, priv->ctrl_reg1);

      /* Clear initial interrupt if in irq mode */

      kxtj9_reg_read(priv, INT_REL, wbuf, 1);
      priv->power_enabled = true;
      sninfo("KXTJ9 in operating mode\n");
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: kxtj9_read_sensor_data
 *
 * Description:
 *   Read sensor data.  This supports the standard driver read() method.
 *
 ****************************************************************************/

static int kxtj9_read_sensor_data(FAR struct kxtj9_dev_s *priv,
                                  FAR struct kxtj9_sensor_data *sensor_data)
{
  int16_t acc_data[3];
  uint8_t data;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  kxtj9_reg_read(priv, XOUT_L, (uint8_t *)acc_data, 6);

  /* 12 bit resolution, get rid of the lowest 4 bits */

  sensor_data->x = acc_data[0] >> 4;
  sensor_data->y = acc_data[1] >> 4;
  sensor_data->z = acc_data[2] >> 4;

  /* Read INT_REL to clear interrupt status */

  kxtj9_reg_read(priv, INT_REL, &data, 1);
  nxmutex_unlock(&priv->lock);

  /* Feed sensor data to entropy pool */

  add_sensor_randomness((acc_data[0] << 16) ^ (acc_data[1] << 8) ^
                        acc_data[2]);

  return OK;
}

/****************************************************************************
 * Name: kxtj9_read
 *
 * Description:
 *   The standard read method.
 *
 ****************************************************************************/

static ssize_t kxtj9_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode;
  FAR struct kxtj9_dev_s *priv;
  size_t nsamples;
  size_t i;
  int ret;

  /* How many samples will fit in the buffer? */

  nsamples = buflen / sizeof(struct kxtj9_sensor_data);

  /* If the provided buffer is not large enough to return a single sample,
   * then return an error.
   */

  if (nsamples < 1)
    {
      snerr("ERROR: Buffer too small %lu < %u\n",
            buflen, sizeof(struct kxtj9_sensor_data));
      return (ssize_t)-EINVAL;
    }

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL && buffer != NULL);
  inode = filep->f_inode;

  priv = (FAR struct kxtj9_dev_s *)inode->i_private;
  DEBUGASSERT(priv != NULL  && priv->i2c != NULL);

  /* Return all of the samples that will fit in the user-provided buffer */

  for (i = 0; i < nsamples; i++)
    {
      /* Get the next sample data */

      ret = kxtj9_read_sensor_data(priv,
                                   (FAR struct kxtj9_sensor_data *)buffer);
      if (ret < 0)
        {
          snerr("ERROR: kxtj9_read_sensor_data failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Set up for the next sample */

      buffer += sizeof(struct kxtj9_sensor_data);
    }

  return (ssize_t)(nsamples * sizeof(struct kxtj9_sensor_data));
}

/****************************************************************************
 * Name: kxtj9_write
 *
 * Description:
 *   A dummy write method.
 *
 ****************************************************************************/

static ssize_t kxtj9_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: kxtj9_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/

static int kxtj9_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct kxtj9_dev_s *priv;
  int  ret;

  /* Sanity check */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct kxtj9_dev_s *)inode->i_private;
  DEBUGASSERT(priv != NULL && priv->i2c != NULL);

  /* Handle ioctl commands */

  switch (cmd)
    {
      /* Start converting. Arg: None. */

      case SNIOC_ENABLE:
        ret = kxtj9_enable(priv, true);
        break;

      /* Stop converting. Arg: None. */

      case SNIOC_DISABLE:
        ret = kxtj9_enable(priv, false);
        break;

      /* Configure the KXTJ9. Arg: enum kxtj9_odr_e value. */

      case SNIOC_CONFIGURE:
        {
          DEBUGASSERT(arg <= UINT8_MAX);
          ret = kxtj9_configure(priv, (uint8_t)arg);
          sninfo("SNIOC_CONFIGURE: ODR=%u ret=%d\n",
                 (unsigned int)arg, ret);
        }
        break;

      /* Unrecognized commands */

      default:
        snerr("ERROR: Unrecognized cmd: %d arg: %lu\n", cmd, arg);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kxtj9_register
 *
 * Description:
 *   Register the KXTJ9 accelerometer device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/accel0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the KXTJ9 accelerometer, gyroscope or
 *             magnetometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int kxtj9_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t address)
{
  FAR struct kxtj9_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL && i2c != NULL);

  /* Initialize the device's structure */

  priv = (FAR struct kxtj9_dev_s *)kmm_zalloc(sizeof(struct kxtj9_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate driver instance\n");
      return -ENOMEM;
    }

  priv->i2c     = i2c;
  priv->address = address;
  nxmutex_init(&priv->lock);

  /* Register the character driver */

  ret = register_driver(devpath, &g_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_KXTJ9 */
