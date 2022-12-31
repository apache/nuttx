/****************************************************************************
 * drivers/sensors/ina219.c
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

#include <inttypes.h>
#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ina219.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_I2C)
#  error i2c support required
#endif

#define INA219_REG_CONFIG            0  /* See below */
#define INA219_REG_SHUNT_VOLTAGE     1  /* Shunt voltage in 10 uV units */
#define INA219_REG_BUS_VOLTAGE       2  /* Bus votlage in 4 mV units */
#define INA219_REG_POWER             3  /* Requires prior calibration */
#define INA219_REG_CURRENT           4  /* Requires prior calibration */
#define INA219_REG_CALIBRATION       5  /* Calibration value to compute current */

/* Operating modes - not controllable by user */

#define INA219_CONFIG_OPMODE_SHIFT   0
#define INA219_CONFIG_OPMODE_MASK    (7 << INA219_CONFIG_OPMODE_SHIFT)
#define INA219_CONFIG_OPMODE_PWRDOWN (0 << INA219_CONFIG_OPMODE_SHIFT)
#define INA219_CONFIG_OPMODE_STRIG   (1 << INA219_CONFIG_OPMODE_SHIFT)
#define INA219_CONFIG_OPMODE_BTRIG   (2 << INA219_CONFIG_OPMODE_SHIFT)
#define INA219_CONFIG_OPMODE_SBTRIG  (3 << INA219_CONFIG_OPMODE_SHIFT)
#define INA219_CONFIG_OPMODE_OFF     (4 << INA219_CONFIG_OPMODE_SHIFT)
#define INA219_CONFIG_OPMODE_SCONT   (5 << INA219_CONFIG_OPMODE_SHIFT)
#define INA219_CONFIG_OPMODE_BCONT   (6 << INA219_CONFIG_OPMODE_SHIFT)
#define INA219_CONFIG_OPMODE_SBCONT  (7 << INA219_CONFIG_OPMODE_SHIFT)

#ifndef CONFIG_INA219_I2C_FREQUENCY
#  define CONFIG_INA219_I2C_FREQUENCY 400000
#endif

#define I2C_NOSTARTSTOP_MSGS              2
#define I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX 0
#define I2C_NOSTARTSTOP_DATA_MSG_INDEX    1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ina219_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t addr;                  /* I2C address */
  uint16_t config;               /* INA 219 config shadow */
  int32_t shunt_resistor_value;  /* micro-ohms, max 2.15 kohms */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     ina219_write16(FAR struct ina219_dev_s *priv, uint8_t regaddr,
                              FAR uint16_t regvalue);
static int     ina219_read16(FAR struct ina219_dev_s *priv, uint8_t regaddr,
                             FAR uint16_t *regvalue);
static int     ina219_readpower(FAR struct ina219_dev_s *priv,
                                FAR struct ina219_s *buffer);

/* Character driver methods */

static int     ina219_open(FAR struct file *filep);
static int     ina219_close(FAR struct file *filep);
static ssize_t ina219_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t ina219_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ina219fops =
{
  ina219_open,     /* open */
  ina219_close,    /* close */
  ina219_read,     /* read */
  ina219_write,    /* write */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ina219_access(FAR struct ina219_dev_s *priv,
                         uint8_t start_register_address, bool reading,
                         FAR uint8_t *register_value, uint8_t data_length)
{
  struct i2c_msg_s msg[I2C_NOSTARTSTOP_MSGS];
  int ret;

  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].frequency =
    CONFIG_INA219_I2C_FREQUENCY;

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

static int ina219_read16(FAR struct ina219_dev_s *priv, uint8_t regaddr,
                         FAR uint16_t *regvalue)
{
  uint8_t buf[2];

  int ret = ina219_access(priv, regaddr, true, buf, 2);

  if (ret == 0)
    {
      *regvalue = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    }

  return ret;
}

static int ina219_write16(FAR struct ina219_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t regvalue)
{
  uint8_t buf[2];

  int ret;

  buf[0] = (regvalue >> 8) & 0xff;
  buf[1] =  regvalue       & 0xff;

  ret = ina219_access(priv, regaddr, false, buf, 2);

  return ret;
}

/****************************************************************************
 * Name: ina219_readpower
 *
 * Description:
 *   Read the current and voltage register with special scaling
 *
 ****************************************************************************/

static int ina219_readpower(FAR struct ina219_dev_s *priv,
                             FAR struct ina219_s *buffer)
{
  uint16_t reg;
  int64_t  tmp;

  int ret;

  /* Read the raw bus voltage */

  ret = ina219_read16(priv, INA219_REG_BUS_VOLTAGE, &reg);
  if (ret < 0)
    {
      snerr("ERROR: ina219_read16 failed: %d\n", ret);
      return ret;
    }

  /* Convert register value to bus voltage */

  reg >>= 3; /* 3 LSB of reg contains status bits */
  buffer->voltage = ((uint32_t)reg) * 4000LU;

  /* Read the raw shunt voltage */

  ret = ina219_read16(priv, INA219_REG_SHUNT_VOLTAGE, &reg);
  if (ret < 0)
    {
      snerr("ERROR: ina219_read16 failed: %d\n", ret);
      return ret;
    }

  /* Convert register value to shunt voltage */

  tmp = ((int64_t)(int16_t)reg) * 10LL; /* micro volts across shunt */

  /* Convert shunt voltage to current across the shunt resistor.
   * I(uA) = U(uV)/R(ohms)
   *       = U(uV)/(R(uohms)/1000000)
   *       = U(uV) * 1000000 / R(uohms)
   * We use a temporary 64-bit accumulator to avoid overflows.
   */

  tmp = tmp * 1000000LL;
  tmp = tmp / (int64_t)priv->shunt_resistor_value;

  buffer->current = (int32_t)tmp;

  sninfo("Voltage: %" PRIu32 " uV, Current: %" PRId32 " uA\n",
         buffer->voltage, buffer->current);

  return OK;
}

/****************************************************************************
 * Name: ina219_open
 *
 * Description:
 *   This function is called whenever the INA219 device is opened.
 *
 ****************************************************************************/

static int ina219_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ina219_dev_s *priv   = inode->i_private;

  return ina219_write16(priv, INA219_REG_CONFIG,
                        priv->config | INA219_CONFIG_OPMODE_SBCONT);
}

/****************************************************************************
 * Name: ina219_close
 *
 * Description:
 *   This routine is called when the INA219 device is closed.
 *
 ****************************************************************************/

static int ina219_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ina219_dev_s *priv   = inode->i_private;

  return ina219_write16(priv, INA219_REG_CONFIG,
                        priv->config | INA219_CONFIG_OPMODE_OFF);
}

/****************************************************************************
 * Name: ina219_read
 ****************************************************************************/

static ssize_t ina219_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ina219_dev_s *priv   = inode->i_private;
  FAR struct ina219_s *ptr;
  ssize_t nsamples;
  int i;
  int ret;

  /* How many samples were requested to get? */

  nsamples = buflen / sizeof(struct ina219_s);
  ptr      = (FAR struct ina219_s *)buffer;

  sninfo("buflen: %d nsamples: %d\n", buflen, nsamples);

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      struct ina219_s pwr;

      /* Read the next struct ina219_s power value */

      ret = ina219_readpower(priv, &pwr);
      if (ret < 0)
        {
          snerr("ERROR: ina219_readpower failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Save the temperature value in the user buffer */

      *ptr++ = pwr;
    }

  return nsamples * sizeof(struct ina219_s);
}

/****************************************************************************
 * Name: ina219_write
 ****************************************************************************/

static ssize_t ina219_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ina219_register
 *
 * Description:
 *   Register the INA219 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/pwrmntr0"
 *   i2c - An instance of the I2C interface to use to communicate with INA219
 *   addr - The I2C address of the INA219.
 *   shuntval - the shunt resistor value in micro-ohms.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ina219_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr, int32_t shuntval, uint16_t config)
{
  FAR struct ina219_dev_s *priv;
  int ret = 0;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the ina219 device structure */

  priv = (FAR struct ina219_dev_s *)kmm_malloc(sizeof(struct ina219_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;
  priv->shunt_resistor_value = shuntval;

  /* Save the config (except opmode) */

  priv->config = config & ~INA219_CONFIG_OPMODE_MASK;

  /* Apply config, keep chip switched off */

  ret = ina219_write16(priv, INA219_REG_CONFIG,
                        priv->config | INA219_CONFIG_OPMODE_OFF);
  if (ret < 0)
    {
      snerr("ERROR: Failed to apply config: %d\n", ret);
      goto errout;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_ina219fops, 0666, priv);
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
