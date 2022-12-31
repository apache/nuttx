/****************************************************************************
 * drivers/sensors/fxos8700cq.c
 * Driver for Motion Sensor FXOS8700CQ (NXP)
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
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/sensors/fxos8700cq.h>

#if defined(CONFIG_SENSORS_FXOS8700CQ)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FXOS8700CQ_I2C_ADDR 0x1d
#define FXOS8700CQ_I2C_FREQ 400000

/* I2C Regs */

#define FXOS8700CQ_STATUS 0x00
#define FXOS8700CQ_WHOAMI 0x0D
#define FXOS8700CQ_XYZ_DATA_CFG 0x0E
#define FXOS8700CQ_CTRL_REG1 0x2A
#define FXOS8700CQ_CTRL_REG2 0x2B
#define FXOS8700CQ_CTRL_REG3 0x2C
#define FXOS8700CQ_CTRL_REG4 0x2D
#define FXOS8700CQ_CTRL_REG5 0x2E

#define FXOS8700CQ_M_CTRL_REG1 0x5B
#define FXOS8700CQ_M_CTRL_REG2 0x5C

#define FXOS8700CQ_PULSE_CFG (0x21)
#define FXOS8700CQ_PULSE_SRC (0x22)
#define FXOS8700CQ_PULSE_THSX (0x23)

/* Values */
#define FXOS8700CQ_WHOAMI_VAL 0xC7

/** status byte + x,y,z for accelerometer and magnetometer */
#define FXOS8700CQ_READ_LEN ((8 + (16 * 3 + 16 * 3)) / 8)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fxos8700cq_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  int freq;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t fxos8700cq_getreg8(FAR struct fxos8700cq_dev_s *priv,
                                  uint8_t regaddr);

static int fxos8700cq_putreg8(FAR struct fxos8700cq_dev_s *priv,
                              uint8_t regaddr, uint8_t regval);

static int fxos8700cq_getregs(FAR struct fxos8700cq_dev_s *priv,
                              uint8_t regaddr, uint8_t *regval, int len);

/* Character driver methods */

static int fxos8700cq_open(FAR struct file *filep);
static int fxos8700cq_close(FAR struct file *filep);
static ssize_t fxos8700cq_read(FAR struct file *filep,
                               FAR char *buffer, size_t len);
static int fxos8700cq_checkid(FAR struct fxos8700cq_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/** vtable that supports the character driver interface */

static const struct file_operations g_fxos8700cqfops =
{
  fxos8700cq_open,    /* open */
  fxos8700cq_close,   /* close */
  fxos8700cq_read,    /* read */
};

/****************************************************************************
 * Name: fxos8700cq_getreg8
 *
 * Description:
 *   Read from an 8-bit FXOS8700CQ register
 *
 ****************************************************************************/

static uint8_t fxos8700cq_getreg8(FAR struct fxos8700cq_dev_s *priv,
                                  uint8_t regaddr)
{
  uint8_t regval = 0;

  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = &regaddr;
  msg[0].length = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr = priv->addr;
  msg[1].flags = I2C_M_READ;
  msg[1].buffer = &regval;
  msg[1].length = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);

  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return regval;
}

/****************************************************************************
 * Name: fxos8700cq_putreg8
 *
 * Description:
 *   Write a value to an 8-bit FXOS8700CQ register
 *
 ****************************************************************************/

static int fxos8700cq_putreg8(FAR struct fxos8700cq_dev_s *priv,
                              uint8_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg[2];
  int ret;
  uint8_t txbuffer[2];

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = priv->freq;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = txbuffer;
  msg[0].length = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: fxos8700cq_getregs
 *
 * Description:
 *   Read cnt bytes from specified dev_addr and reg_addr
 *
 ****************************************************************************/

static int fxos8700cq_getregs(FAR struct fxos8700cq_dev_s *priv,
                              uint8_t regaddr, uint8_t *regval, int len)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = &regaddr;
  msg[0].length = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr = priv->addr;
  msg[1].flags = I2C_M_READ;
  msg[1].buffer = regval;
  msg[1].length = len;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: fxos8700cq_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int fxos8700cq_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fxos8700cq_dev_s *priv = inode->i_private;

  uint8_t regval;
  int ret = 0;
  ret = fxos8700cq_getreg8(priv, FXOS8700CQ_WHOAMI);
  if (ret != FXOS8700CQ_WHOAMI_VAL)
    {
      snerr("ERROR: Invalid device identification %02x\n", regval);
      return -ENODEV;
    }

  regval = 0x00; /* active: standbye */
  ret = fxos8700cq_putreg8(priv, FXOS8700CQ_CTRL_REG1, regval);

  if (ret < 0)
    {
      snerr("ERROR: %02x\n", regval);
      return (ret);
    }

  /* m_hms[1:0]: Hybrid mode, both accelerometer and magnetometer
   * m_os[2:0]: Oversample ratio (OSR) for magnetometer data
   * m_ost: One-shot triggered magnetic measurement mode
   * m_rst: One-shot magnetic reset degauss control bit
   * m_acal: Magnetic hard-iron offset auto-calibration enable
   */

  regval = 0
    | (0b11 << 0)
    | (0b11 << 3)
    | (0b1 << 5)
    | (0b0 << 6)
    | (0b0 << 7)
    ;
  ret = fxos8700cq_putreg8(priv, FXOS8700CQ_M_CTRL_REG1, regval);
  if (ret < 0)
    {
      return (ret);
    }

  /* hyb_autoinc_mode */

  regval = 0b1 << 5;
  ret = fxos8700cq_putreg8(priv, FXOS8700CQ_M_CTRL_REG2, regval);
  if (ret < 0)
    {
      return (ret);
    }

  /* fs[1:0]: Accelerometer full-scale range : 4G */

  regval = 0b01;
  ret = fxos8700cq_putreg8(priv, FXOS8700CQ_XYZ_DATA_CFG, regval);
  if (ret < 0)
    {
      return (ret);
    }

  /* active ,  lnoise ,  dr[2:0] */

  regval = 0b1
    | (0b1 < 2)
    | (0b001 < 5)
    ;
  ret = fxos8700cq_putreg8(priv, FXOS8700CQ_CTRL_REG1, regval);
  if (ret < 0)
    {
      return (ret);
    }

  return OK;
}

/****************************************************************************
 * Name: fxos8700cq_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int fxos8700cq_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fxos8700cq_dev_s *priv = inode->i_private;

  uint8_t regval = 0b0; /* active */
  int ret = fxos8700cq_putreg8(priv, FXOS8700CQ_CTRL_REG1, regval);
  if (ret < 0)
    {
      return (ret);
    }

  return OK;
}

/****************************************************************************
 * Name: fxos8700cq_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t fxos8700cq_read(FAR struct file *filep, FAR char *buffer,
                               size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fxos8700cq_dev_s *priv = inode->i_private;

  FAR fxos8700cq_data * p = (FAR fxos8700cq_data *) buffer;
  if (len < sizeof(fxos8700cq_data))
    {
      snerr("Expected buffer size is %d\n", sizeof(fxos8700cq_data));
      return 0;
    }

  uint8_t data[FXOS8700CQ_READ_LEN];
  int status = fxos8700cq_getregs(priv, FXOS8700CQ_STATUS,
                                  (FAR uint8_t *)data, FXOS8700CQ_READ_LEN);
  if (status < 0)
    {
      snerr("Error: read status=%d", status);
      return 0;
    }

  /* Copy the 14 bit accelerometer byte data into 16 bit words */

  p->accel.x = (int16_t)(((data[1] << 8) | data[2])) >> 2;
  p->accel.y = (int16_t)(((data[3] << 8) | data[4])) >> 2;
  p->accel.z = (int16_t)(((data[5] << 8) | data[6])) >> 2;

  /* Copy the magnetometer byte data into 16 bit words */

  p->magn.x = (int16_t)(((data[7] << 8) | data[8]));
  p->magn.y = (int16_t)(((data[9] << 8) | data[10]));
  p->magn.z = (int16_t)(((data[11] << 8) | data[12]));

  return len;
}

/****************************************************************************
 * Name: fxos8700cq_checkid
 *
 * Description:
 *   Read and verify the FXOS8700CQ chip ID
 *
 ****************************************************************************/

static int fxos8700cq_checkid(FAR struct fxos8700cq_dev_s *priv)
{
  uint8_t devid = 0;
  devid = fxos8700cq_getreg8(priv, FXOS8700CQ_WHOAMI);
  sninfo("devid: %04x\n", devid);
  if (devid != (uint16_t) FXOS8700CQ_WHOAMI_VAL)
    {
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: fxos8700cq_register
 *
 * Description:
 *   Register the FXOS8700CQ character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/accel0"
 *   dev - An instance of the I2C interface to communicate with device
 *
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int fxos8700cq_register(FAR const char *devpath,
                        FAR struct i2c_master_s *dev)
{
  FAR struct fxos8700cq_dev_s *priv;
  int ret;

  priv = (FAR struct fxos8700cq_dev_s *)
    kmm_malloc(sizeof(struct fxos8700cq_dev_s));

  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = dev;
  priv->addr = FXOS8700CQ_I2C_ADDR;
  priv->freq = FXOS8700CQ_I2C_FREQ;

  ret = fxos8700cq_checkid(priv);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  ret = register_driver(devpath, &g_fxos8700cqfops, 0444, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("FXOS8700CQ driver loaded successfully!\n");
  return OK;
}

#endif /* CONFIG_SENSORS_FXOS8700CQ */
