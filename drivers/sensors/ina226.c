/****************************************************************************
 * drivers/sensors/ina226.c
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
#include <nuttx/sensors/ina226.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_I2C)
#  error i2c support required
#endif

#ifndef CONFIG_INA226_I2C_FREQUENCY
#  define CONFIG_INA226_I2C_FREQUENCY 400000
#endif

#define I2C_NOSTARTSTOP_MSGS              2
#define I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX 0
#define I2C_NOSTARTSTOP_DATA_MSG_INDEX    1

#define BV_LSB 1250LU /* Bus voltage LSB in microvolts */
#define SV_LSB 250LU  /* Shunt voltage LSB in microvolts times 10 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ina226_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t addr;                  /* I2C address */
  uint16_t config;               /* INA226 config shadow */
  int32_t shunt_resistor_value;  /* micro-ohms, max 2.15 kohms */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     ina226_write16(FAR struct ina226_dev_s *priv, uint8_t regaddr,
                              FAR uint16_t regvalue);
static int     ina226_read16(FAR struct ina226_dev_s *priv, uint8_t regaddr,
                             FAR uint16_t *regvalue);
static int     ina226_readpower(FAR struct ina226_dev_s *priv,
                                FAR struct ina226_s *buffer);

/* Character driver methods */

static int     ina226_open(FAR struct file *filep);
static int     ina226_close(FAR struct file *filep);
static ssize_t ina226_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t ina226_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ina226fops =
{
  ina226_open,     /* open */
  ina226_close,    /* close */
  ina226_read,     /* read */
  ina226_write,    /* write */
  NULL,            /* seek */
  NULL,            /* ioctl */
  NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ina226_access(FAR struct ina226_dev_s *priv,
                         uint8_t start_register_address, bool reading,
                         FAR uint8_t *register_value, uint8_t data_length)
{
  struct i2c_msg_s msg[I2C_NOSTARTSTOP_MSGS];
  int ret;

  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].frequency =
                                        CONFIG_INA226_I2C_FREQUENCY;

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

static int ina226_read16(FAR struct ina226_dev_s *priv, uint8_t regaddr,
                         FAR uint16_t *regvalue)
{
  uint8_t buf[2];

  int ret = ina226_access(priv, regaddr, true, buf, 2);

  if (ret == 0)
    {
      *regvalue = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    }

  return ret;
}

static int ina226_write16(FAR struct ina226_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t regvalue)
{
  uint8_t buf[2];

  int ret;

  buf[0] = (regvalue >> 8) & 0xff;
  buf[1] =  regvalue       & 0xff;

  ret = ina226_access(priv, regaddr, false, buf, 2);

  return ret;
}

/****************************************************************************
 * Name: ina226_readpower
 *
 * Description:
 *   Read the current and voltage register with special scaling
 *
 ****************************************************************************/

static int ina226_readpower(FAR struct ina226_dev_s *priv,
                             FAR struct ina226_s *buffer)
{
  uint16_t reg;
  int64_t  tmp;

  int ret;

  /* Read the raw bus voltage */

  ret = ina226_read16(priv, INA226_REG_BUS_VOLTAGE, &reg);
  if (ret < 0)
    {
      snerr("ERROR: ina226_read16 failed: %d\n", ret);
      return ret;
    }

  /* Convert register value to bus voltage */

  buffer->voltage = ((uint32_t)reg) * BV_LSB; /* 1 LSB 1,25mV */

  /* Read the raw shunt voltage */

  ret = ina226_read16(priv, INA226_REG_SHUNT_VOLTAGE, &reg);
  if (ret < 0)
    {
      snerr("ERROR: ina226_read16 failed: %d\n", ret);
      return ret;
    }

  tmp = ((int64_t)(int16_t)reg);

  /* Convert shunt voltage to current across the shunt resistor.
   * I(uA) = U(uV)/R(ohms)
   *       = U(uV)/(R(uohms)/1000000)
   *       = U(uV) * 1000000 / R(uohms)
   *
   * U(uV) = tmp*2,5
   *
   * We use a temporary 64-bit accumulator to avoid overflows.
   */

  tmp = tmp * 2500000LL;
  tmp = tmp / (int64_t)priv->shunt_resistor_value;

  buffer->current = (int32_t)tmp;

  return OK;
}

/****************************************************************************
 * Name: ina226_open
 *
 * Description:
 *   This function is called whenever the INA226 device is opened.
 *
 ****************************************************************************/

static int ina226_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ina226_dev_s *priv   = inode->i_private;

  return ina226_write16(priv, INA226_REG_CONFIG,
                        priv->config | INA226_CONFIG_MODE_SBCONT);
}

/****************************************************************************
 * Name: ina226_close
 *
 * Description:
 *   This routine is called when the INA226 device is closed.
 *
 ****************************************************************************/

static int ina226_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ina226_dev_s *priv   = inode->i_private;

  return ina226_write16(priv, INA226_REG_CONFIG,
                        priv->config | INA226_CONFIG_MODE_PWRDOWN);
}

/****************************************************************************
 * Name: ina226_read
 ****************************************************************************/

static ssize_t ina226_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ina226_dev_s *priv   = inode->i_private;
  FAR struct ina226_s *ptr;
  ssize_t nsamples;
  int i;
  int ret;

  /* How many samples were requested to get? */

  nsamples = buflen / sizeof(struct ina226_s);
  ptr      = (FAR struct ina226_s *)buffer;

  sninfo("buflen: %d nsamples: %d\n", buflen, nsamples);

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      struct ina226_s pwr;

      /* Read the next struct ina226_s power value */

      ret = ina226_readpower(priv, &pwr);
      if (ret < 0)
        {
          snerr("ERROR: ina226_readpower failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Save the power value in the user buffer */

      *ptr++ = pwr;
    }

  return nsamples * sizeof(struct ina226_s);
}

/****************************************************************************
 * Name: ina226_write
 ****************************************************************************/

static ssize_t ina226_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ina226_register
 *
 * Description:
 *   Register the INA226 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/pwrmntr0"
 *   i2c - An instance of the I2C interface to use to communicate with INA226
 *   addr - The I2C address of the INA226.
 *   shuntval - the shunt resistor value in micro-ohms.
 *   config - a combination of the constants defined earlier in this file.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ina226_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr, int32_t shuntval, uint16_t config)
{
  FAR struct ina226_dev_s *priv;
  int ret = 0;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the ina226 device structure */

  priv = (FAR struct ina226_dev_s *)kmm_malloc(sizeof(struct ina226_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;
  priv->shunt_resistor_value = shuntval;

  /* Save the config (except opmode) */

  priv->config = config & ~INA226_CONFIG_MODE_MASK;

  /* Apply config, keep chip switched off */

  ret = ina226_write16(priv, INA226_REG_CONFIG,
                       priv->config | INA226_CONFIG_MODE_PWRDOWN);
  if (ret < 0)
    {
      snerr("ERROR: Failed to apply config: %d\n", ret);
      goto errout;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_ina226fops, 0666, priv);
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
