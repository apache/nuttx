/****************************************************************************
 * drivers/sensors/bh1749nuc.c
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

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/sensors/bh1749nuc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BH1749NUC_I2C_FREQ          400000

#define BH1749NUC_MANUFACTID        0xE0    /* Manufact ID */
#define BH1749NUC_PARTID            0x0D    /* Part ID */

/* BH1749NUC Registers */

#define BH1749NUC_SYSTEM_CONTROL    0x40
#define BH1749NUC_MODE_CONTROL1     0x41
#define BH1749NUC_MODE_CONTROL2     0x42
#define BH1749NUC_RED_DATA_LSB      0x50
#define BH1749NUC_GREEN_DATA_LSB    0x52
#define BH1749NUC_BLUE_DATA_LSB     0x54
#define BH1749NUC_IR_DATA_LSB       0x58
#define BH1749NUC_GREEN2_DATA_LSB   0x5a
#define BH1749NUC_MANUFACTURER_ID   0x92

/* Register SYSTEM_CONTROL */

#define BH1749NUC_SYSTEM_CONTROL_SW_RESET      (1 << 7)
#define BH1749NUC_SYSTEM_CONTROL_INT_RESET     (1 << 6)

/* Register MODE_CONTROL1 */

#define BH1749NUC_MODE_CONTROL1_IR_GAIN_X1     (0x20)
#define BH1749NUC_MODE_CONTROL1_IR_GAIN_X32    (0x60)
#define BH1749NUC_MODE_CONTROL1_RGB_GAIN_X1    (0x08)
#define BH1749NUC_MODE_CONTROL1_RGB_GAIN_X32   (0x18)
#define BH1749NUC_MODE_CONTROL1_MEAS_TIME160MS (0x02)

/* Register MODE_CONTROL2 */

#define BH1749NUC_MODE_CONTROL2_RGBI_EN        (1 << 4)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Structure for bh1749nuc device */

struct bh1749nuc_dev_s
{
  FAR struct i2c_master_s *i2c;   /* I2C interface */
  int                      freq;  /* Frequency */
  uint8_t                  addr;  /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int bh1749nuc_open(FAR struct file *filep);
static int bh1749nuc_close(FAR struct file *filep);
static ssize_t bh1749nuc_read(FAR struct file *filep,
                              FAR char *buffer,
                              size_t buflen);
static ssize_t bh1749nuc_write(FAR struct file *filep,
                               FAR const char *buffer,
                               size_t buflen);
static int bh1749nuc_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bh1749nucfops =
{
  bh1749nuc_open,              /* open */
  bh1749nuc_close,             /* close */
  bh1749nuc_read,              /* read */
  bh1749nuc_write,             /* write */
  NULL,                        /* seek */
  bh1749nuc_ioctl,             /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bh1749nuc_getreg8
 *
 * Description:
 *   Read from an 8-bit BH1749NUC register
 *
 ****************************************************************************/

static uint8_t bh1749nuc_getreg8(FAR struct bh1749nuc_dev_s *priv,
                                 uint8_t regaddr)
{
  struct i2c_msg_s msg[2];
  uint8_t regval = 0;
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return regval;
}

/****************************************************************************
 * Name: bh1749nuc_read16
 *
 * Description:
 *   Read 16-bit register
 *
 ****************************************************************************/

static uint16_t bh1749nuc_read16(FAR struct bh1749nuc_dev_s *priv,
                                 uint8_t regaddr)
{
  struct i2c_msg_s msg[2];
  uint8_t regval[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = (uint8_t *)&regval;
  msg[1].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return regval[1] << 8 | regval[0] << 0;
}

/****************************************************************************
 * Name: bh1749nuc_putreg8
 *
 * Description:
 *   Write to an 8-bit BH1749NUC register
 *
 ****************************************************************************/

static void bh1749nuc_putreg8(FAR struct bh1749nuc_dev_s *priv,
                              uint8_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg[2];
  uint8_t txbuffer[2];
  int ret;

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = txbuffer;
  msg[0].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: bh1749nuc_checkid
 *
 * Description:
 *   Read and verify the BH1749NUC chip ID
 *
 ****************************************************************************/

static int bh1749nuc_checkid(FAR struct bh1749nuc_dev_s *priv)
{
  uint8_t id;

  /* Read Manufact ID */

  id = bh1749nuc_getreg8(priv, BH1749NUC_MANUFACTURER_ID);
  if (id != BH1749NUC_MANUFACTID)
    {
      /* Manufact ID is not Correct */

      snerr("Wrong Manufact ID! %02x\n", id);
      return -ENODEV;
    }

  /* Read Part ID */

  id = bh1749nuc_getreg8(priv, BH1749NUC_SYSTEM_CONTROL);
  if ((id & 0x3f) != BH1749NUC_PARTID)
    {
      /* Part ID is not Correct */

      snerr("Wrong Part ID! %02x\n", id);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: bh1749nuc_open
 *
 * Description:
 *   This function is called whenever the BH1749NUC device is opened.
 *
 ****************************************************************************/

static int bh1749nuc_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct bh1749nuc_dev_s *priv  = inode->i_private;
  uint8_t                     val;

  /* MODE_CONTROL1 */

  val = (BH1749NUC_MODE_CONTROL1_MEAS_TIME160MS |
         BH1749NUC_MODE_CONTROL1_IR_GAIN_X1 |
         BH1749NUC_MODE_CONTROL1_RGB_GAIN_X1);
  bh1749nuc_putreg8(priv, BH1749NUC_MODE_CONTROL1, val);

  /* MODE_CONTROL2 */

  val = BH1749NUC_MODE_CONTROL2_RGBI_EN;
  bh1749nuc_putreg8(priv, BH1749NUC_MODE_CONTROL2, val);

  return OK;
}

/****************************************************************************
 * Name: bh1749nuc_close
 *
 * Description:
 *   This routine is called when the BH1749NUC device is closed.
 *
 ****************************************************************************/

static int bh1749nuc_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct bh1749nuc_dev_s *priv  = inode->i_private;
  uint8_t                     val;

  /* Stop sampling */

  val = (BH1749NUC_SYSTEM_CONTROL_SW_RESET |
         BH1749NUC_SYSTEM_CONTROL_INT_RESET);
  bh1749nuc_putreg8(priv, BH1749NUC_SYSTEM_CONTROL, val);

  return OK;
}

/****************************************************************************
 * Name: bh1749nuc_read
 ****************************************************************************/

static ssize_t bh1749nuc_read(FAR struct file *filep, FAR char *buffer,
                              size_t len)
{
  FAR struct inode            *inode = filep->f_inode;
  FAR struct bh1749nuc_dev_s  *priv  = inode->i_private;
  FAR struct bh1749nuc_data_s *data  =
    (FAR struct bh1749nuc_data_s *) buffer;

  if (len < sizeof(struct bh1749nuc_data_s))
    {
      snerr("ERROR: Not enough memory to read data sample.\n");
      return -ENOBUFS;
    }

  /* Get data */

  data->red    = bh1749nuc_read16(priv, BH1749NUC_RED_DATA_LSB);
  data->green  = bh1749nuc_read16(priv, BH1749NUC_GREEN_DATA_LSB);
  data->blue   = bh1749nuc_read16(priv, BH1749NUC_BLUE_DATA_LSB);
  data->ir     = bh1749nuc_read16(priv, BH1749NUC_IR_DATA_LSB);
  data->green2 = bh1749nuc_read16(priv, BH1749NUC_GREEN2_DATA_LSB);

  return sizeof(struct bh1749nuc_data_s);
}

/****************************************************************************
 * Name: bh1749nuc_write
 ****************************************************************************/

static ssize_t bh1749nuc_write(FAR struct file *filep,
                               FAR const char *buffer,
                               size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bh1749nuc_ioctl
 ****************************************************************************/

static int bh1749nuc_ioctl(FAR struct file *filep,
                           int cmd,
                           unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bh1749nuc_register
 *
 * Description:
 *   Register the BH1749NUC character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/color0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BH1749NUC
 *   addr    - I2C address
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bh1749nuc_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr)
{
  FAR struct bh1749nuc_dev_s *priv;
  int                         ret;

  /* Initialize the BH1749NUC device structure */

  priv = (FAR struct bh1749nuc_dev_s *)
    kmm_malloc(sizeof(struct bh1749nuc_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c   = i2c;
  priv->addr  = addr;
  priv->freq  = BH1749NUC_I2C_FREQ;

  /* Check Device ID */

  ret = bh1749nuc_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      return ret;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_bh1749nucfops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BH1749NUC driver loaded successfully!\n");

  return ret;
}
