/****************************************************************************
 * drivers/sensors/ina260.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ina260.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_I2C)
#  error i2c support required
#endif

#define I2C_NOSTARTSTOP_MSGS              2
#define I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX 0
#define I2C_NOSTARTSTOP_DATA_MSG_INDEX    1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ina260_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t addr;                  /* I2C address */
  uint16_t config;               /* INA260 config shadow */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     ina260_write16(FAR struct ina260_dev_s *priv, uint8_t regaddr,
                              FAR uint16_t regvalue);
static int     ina260_read16(FAR struct ina260_dev_s *priv, uint8_t regaddr,
                             FAR uint16_t *regvalue);
static int     ina260_readpower(FAR struct ina260_dev_s *priv,
                                FAR struct ina260_s *buffer);

/* Character driver methods */

static int     ina260_open(FAR struct file *filep);
static int     ina260_close(FAR struct file *filep);
static ssize_t ina260_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t ina260_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ina260fops =
{
  ina260_open,     /* open */
  ina260_close,    /* close */
  ina260_read,     /* read */
  ina260_write,    /* write */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ina260_access(FAR struct ina260_dev_s *priv,
                         uint8_t start_register_address, bool reading,
                         FAR uint8_t *register_value, uint8_t data_length)
{
  struct i2c_msg_s msg[I2C_NOSTARTSTOP_MSGS];
  int ret;

  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].frequency =
                                        CONFIG_INA260_I2C_FREQUENCY;

  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].addr = priv->addr;
  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].flags = 0;
  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].buffer = &start_register_address;
  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].length = 1;

  msg[I2C_NOSTARTSTOP_DATA_MSG_INDEX].addr =
                                 msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].addr;
  msg[I2C_NOSTARTSTOP_DATA_MSG_INDEX].flags = reading ? I2C_M_READ : 0;
  msg[I2C_NOSTARTSTOP_DATA_MSG_INDEX].buffer = register_value;
  msg[I2C_NOSTARTSTOP_DATA_MSG_INDEX].length = data_length;

  ret = I2C_TRANSFER(priv->i2c, msg, I2C_NOSTARTSTOP_MSGS);

  sninfo("start_register_address: "
         "0x%02X data_length: %d register_value: 0x%02x (0x%04x) ret: %d\n",
         start_register_address, data_length, *register_value,
         *((FAR uint16_t *)register_value), ret);

  return ret;
}

static int ina260_read16(FAR struct ina260_dev_s *priv, uint8_t regaddr,
                         FAR uint16_t *regvalue)
{
  uint8_t buf[2];

  int ret = ina260_access(priv, regaddr, true, buf, 2);

  if (ret == 0)
    {
      *regvalue = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    }

  return ret;
}

static int ina260_write16(FAR struct ina260_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t regvalue)
{
  uint8_t buf[2];

  int ret;

  buf[0] = (regvalue >> 8) & 0xff;
  buf[1] =  regvalue       & 0xff;

  ret = ina260_access(priv, regaddr, false, buf, 2);

  return ret;
}

/****************************************************************************
 * Name: ina260_readpower
 *
 * Description:
 *   Read the current, voltage, and power registers.
 *   The INA260 has an integrated shunt resistor: current and power are
 *   read directly from hardware registers (no software computation).
 *
 ****************************************************************************/

static int ina260_readpower(FAR struct ina260_dev_s *priv,
                            FAR struct ina260_s *buffer)
{
  uint16_t reg;
  int ret;

  /* Read the bus voltage register (LSB = 1.25 mV) */

  ret = ina260_read16(priv, INA260_REG_BUS_VOLTAGE, &reg);
  if (ret < 0)
    {
      snerr("ERROR: ina260_read16 failed: %d\n", ret);
      return ret;
    }

  /* Convert to millivolts: val * 1.25 = val * 125 / 100 */

  buffer->voltage = (uint32_t)reg * 125u / 100u;

  /* Read the current register (LSB = 1.25 mA, signed) */

  ret = ina260_read16(priv, INA260_REG_CURRENT, &reg);
  if (ret < 0)
    {
      snerr("ERROR: ina260_read16 failed: %d\n", ret);
      return ret;
    }

  /* Convert to milliamperes: val * 1.25 = val * 125 / 100 */

  buffer->current = (int32_t)(int16_t)reg * 125 / 100;

  /* Read the power register (LSB = 10 mW, unsigned) */

  ret = ina260_read16(priv, INA260_REG_POWER, &reg);
  if (ret < 0)
    {
      snerr("ERROR: ina260_read16 failed: %d\n", ret);
      return ret;
    }

  buffer->power = (uint32_t)reg * 10u;

  return OK;
}

/****************************************************************************
 * Name: ina260_open
 *
 * Description:
 *   This function is called whenever the INA260 device is opened.
 *
 ****************************************************************************/

static int ina260_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ina260_dev_s *priv = inode->i_private;

  return ina260_write16(priv, INA260_REG_CONFIG,
                        priv->config | INA260_CONFIG_MODE_SBCONT);
}

/****************************************************************************
 * Name: ina260_close
 *
 * Description:
 *   This routine is called when the INA260 device is closed.
 *
 ****************************************************************************/

static int ina260_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ina260_dev_s *priv = inode->i_private;

  return ina260_write16(priv, INA260_REG_CONFIG,
                        priv->config | INA260_CONFIG_MODE_PWRDOWN);
}

/****************************************************************************
 * Name: ina260_read
 ****************************************************************************/

static ssize_t ina260_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ina260_dev_s *priv = inode->i_private;
  FAR struct ina260_s *ptr;
  ssize_t nsamples;
  int i;
  int ret;

  /* How many samples were requested to get? */

  nsamples = buflen / sizeof(struct ina260_s);
  ptr      = (FAR struct ina260_s *)buffer;

  sninfo("buflen: %d nsamples: %d\n", buflen, nsamples);

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      struct ina260_s pwr;

      /* Read the next struct ina260_s measurement */

      ret = ina260_readpower(priv, &pwr);
      if (ret < 0)
        {
          snerr("ERROR: ina260_readpower failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Save the measurement in the user buffer */

      *ptr++ = pwr;
    }

  return nsamples * sizeof(struct ina260_s);
}

/****************************************************************************
 * Name: ina260_write
 ****************************************************************************/

static ssize_t ina260_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ina260_register
 *
 * Description:
 *   Register the INA260 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/pwrmntr0"
 *   i2c - An instance of the I2C interface to use to communicate with INA260
 *   addr - The I2C address of the INA260.
 *   config - a combination of the constants defined earlier in this file
 *            (averaging, conversion times). Operating mode bits are ignored.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ina260_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr, uint16_t config)
{
  FAR struct ina260_dev_s *priv;
  int ret = 0;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the ina260 device structure */

  priv = kmm_malloc(sizeof(struct ina260_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  /* Save the config (except opmode) */

  priv->config = config & ~INA260_CONFIG_MODE_MASK;

  /* Issue a software reset to return all registers to POR defaults */

  ret = ina260_write16(priv, INA260_REG_CONFIG, INA260_CONFIG_RST);
  if (ret < 0)
    {
      snerr("ERROR: Failed to reset INA260: %d\n", ret);
      goto errout;
    }

  /* Apply config, keep chip switched off */

  ret = ina260_write16(priv, INA260_REG_CONFIG,
                       priv->config | INA260_CONFIG_MODE_PWRDOWN);
  if (ret < 0)
    {
      snerr("ERROR: Failed to apply config: %d\n", ret);
      goto errout;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_ina260fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      goto errout;
    }

  sninfo("(addr=0x%02x) registered at %s\n", priv->addr, devpath);
  return ret;

errout:
  kmm_free(priv);
  return ret;
}
