/****************************************************************************
 * drivers/sensors/ina3221.c
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
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ina3221.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_I2C)
#  error i2c support required
#endif

#define INA3221_REG_CONFIG              0x00  /* See below */
#define INA3221_REG_CH1_SHUNT_VOLTAGE   0x01  /* Shunt voltage in 40 uV units */
#define INA3221_REG_CH1_BUS_VOLTAGE     0x02  /* Bus votlage in 8 mV units */
#define INA3221_REG_CH2_SHUNT_VOLTAGE   0x03  /* Shunt voltage in 40 uV units */
#define INA3221_REG_CH2_BUS_VOLTAGE     0x04  /* Bus votlage in 8 mV units */
#define INA3221_REG_CH3_SHUNT_VOLTAGE   0x05  /* Shunt voltage in 40 uV units */
#define INA3221_REG_CH3_BUS_VOLTAGE     0x06  /* Bus votlage in 8 mV units */
#define INA3221_REG_CH1_CRIT_LIMIT      0x07  /* Critical Alert limit */
#define INA3221_REG_CH1_WARN_LIMIT      0x08  /* Warning Alert limit */
#define INA3221_REG_CH2_CRIT_LIMIT      0x09  /* Critical Alert limit */
#define INA3221_REG_CH2_WARN_LIMIT      0x0A  /* Warning Alert limit */
#define INA3221_REG_CH3_CRIT_LIMIT      0x0B  /* Critical Alert limit */
#define INA3221_REG_CH3_WARN_LIMIT      0x0C  /* Warning Alert limit */
#define INA3221_REG_VSHUNT_SUM          0x0D  /* Shunt Voltage Sum */
#define INA3221_REG_VSHUNT_SUM_LIMIT    0x0E  /* Shunt Voltage Sum Limit */
#define INA3221_REG_MASKENABLE          0x0F  /* Mask/Enable register */
#define INA3221_REG_POWERVALID_UPPER    0x10  /* Power-Valid Upper Limit */
#define INA3221_REG_POWERVALID_LOWER    0x11  /* Power-Valid Lower Limit */
#define INA3221_REG_MANUFACTURER_ID     0xFE  /* Always 0x5449 */
#define INA3221_REG_DIE_ID              0xFF  /* Always 0x3220 */

#define INA3221_CONFIG_RST (1 << 15)

#ifndef CONFIG_INA3221_I2C_FREQUENCY
#  define CONFIG_INA3221_I2C_FREQUENCY  400000
#endif

#define I2C_NOSTARTSTOP_MSGS              2
#define I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX 0
#define I2C_NOSTARTSTOP_DATA_MSG_INDEX    1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ina3221_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t addr;                  /* I2C address */
  uint16_t config;               /* INA 3221 config shadow */
  int32_t shunt_resistor[3];     /* micro-ohms */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     ina3221_write16(FAR struct ina3221_dev_s *priv,
                               uint8_t regaddr,
                               FAR uint16_t regvalue);
static int     ina3221_read16(FAR struct ina3221_dev_s *priv,
                              uint8_t regaddr,
                              FAR uint16_t *regvalue);
static int     ina3221_readpower(FAR struct ina3221_dev_s *priv,
                                 FAR struct ina3221_s *buffer);

/* Character driver methods */

static ssize_t ina3221_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t ina3221_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ina3221fops =
{
  NULL,            /* open */
  NULL,            /* close */
  ina3221_read,    /* read */
  ina3221_write,   /* write */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ina3221_access(FAR struct ina3221_dev_s *priv,
                          uint8_t start_register_address, bool reading,
                          FAR uint8_t *register_value, uint8_t data_length)
{
  struct i2c_msg_s msg[I2C_NOSTARTSTOP_MSGS];
  int ret;

  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].frequency =
                                 CONFIG_INA3221_I2C_FREQUENCY;

  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].addr      = priv->addr;
  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].flags     = 0;
  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].buffer    = &start_register_address;
  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].length    = 1;

  msg[I2C_NOSTARTSTOP_DATA_MSG_INDEX].addr         =
                                 msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].addr;
  msg[I2C_NOSTARTSTOP_DATA_MSG_INDEX].flags        =
                                reading ? I2C_M_READ : 0;
  msg[I2C_NOSTARTSTOP_DATA_MSG_INDEX].buffer       = register_value;
  msg[I2C_NOSTARTSTOP_DATA_MSG_INDEX].length       = data_length;

  ret = I2C_TRANSFER(priv->i2c, msg, I2C_NOSTARTSTOP_MSGS);

  sninfo("start_register_address: "
         "0x%02X data_length: %d register_value: 0x%02x (0x%04x) ret: %d\n",
         start_register_address, data_length, *register_value,
         *((FAR uint16_t *)register_value), ret);

  return ret;
}

static int ina3221_read16(FAR struct ina3221_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t *regvalue)
{
  uint8_t buf[2];

  int ret = ina3221_access(priv, regaddr, true, buf, 2);

  if (ret == 0)
    {
      *regvalue = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    }

  return ret;
}

static int ina3221_write16(FAR struct ina3221_dev_s *priv, uint8_t regaddr,
                           FAR uint16_t regvalue)
{
  uint8_t buf[2];

  int ret;

  buf[0] = (regvalue >> 8) & 0xff;
  buf[1] =  regvalue       & 0xff;

  ret = ina3221_access(priv, regaddr, false, buf, 2);

  return ret;
}

/****************************************************************************
 * Name: ina3221_readpower
 *
 * Description:
 *   Read the current and voltage register with special scaling
 *
 ****************************************************************************/

static int ina3221_readpower(FAR struct ina3221_dev_s *priv,
                             FAR struct ina3221_s *buffer)
{
  uint16_t reg;
  int16_t sreg;
  int64_t tmp;
  int i;

  int ret;

  /* Loop for all 3 channels on the device */

  for (i = 0; i < 3; i++)
    {
      if (priv->config & (INA3221_CONFIG_CH1_EN >> i))
        {
          /* Read the raw bus voltage */

          ret = ina3221_read16(priv, (INA3221_REG_CH1_BUS_VOLTAGE + i * 2),
                               &reg);
          if (ret < 0)
            {
              snerr("ERROR: ina3221_read16 failed: %d\n", ret);
              return ret;
            }

          /* Convert register value to bus voltage */

          sreg   = (int16_t)reg;
          sreg >>= 3;                               /* 3 LSB of reg are not used,
                                                     * but the value is signed */
          buffer->ch[i].voltage = ((uint32_t)sreg) * 8000LU;

          /* Read the raw shunt voltage */

          ret = ina3221_read16(priv, (INA3221_REG_CH1_SHUNT_VOLTAGE + i * 2),
                               &reg);
          if (ret < 0)
            {
              snerr("ERROR: ina3221_read16 failed: %d\n", ret);
              return ret;
            }

          /* Convert register value to shunt voltage */

          sreg   = (int16_t)reg;
          sreg >>= 3;                               /* 3 LSB of reg are not used,
                                                     * but the value is signed */
          tmp    = ((int64_t)(int16_t)sreg) * 40LL; /* micro volts across shunt */

          /* Convert shunt voltage to current across the shunt resistor.
           * I(uA) = U(uV)/R(ohms)
           *       = U(uV)/(R(uohms)/1000000)
           *       = U(uV) * 1000000 / R(uohms)
           * We use a temporary 64-bit accumulator to avoid overflows.
           */

          tmp = tmp * 1000000LL;
          tmp = tmp / (int64_t)priv->shunt_resistor[i];

          buffer->ch[i].current = (int32_t)tmp;

          sninfo("Voltage: %u uV, Current: %d uA\n",
                 buffer->ch[i].voltage, buffer->ch[i].current);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ina3221_read
 ****************************************************************************/

static ssize_t ina3221_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ina3221_dev_s *priv = inode->i_private;
  FAR struct ina3221_s *ptr;
  ssize_t nsamples;
  int i;
  int ret;

  /* How many samples were requested to get? */

  nsamples = buflen / sizeof(struct ina3221_s);
  ptr      = (FAR struct ina3221_s *)buffer;

  sninfo("buflen: %d nsamples: %d\n", buflen, nsamples);

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      struct ina3221_s pwr;

      /* Read the next struct ina3221_s power value */

      ret = ina3221_readpower(priv, &pwr);
      if (ret < 0)
        {
          snerr("ERROR: ina3221_readpower failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Save the temperature value in the user buffer */

      *ptr++ = pwr;
    }

  return nsamples * sizeof(struct ina3221_s);
}

/****************************************************************************
 * Name: ina3221_write
 ****************************************************************************/

static ssize_t ina3221_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ina3221_register
 *
 * Description:
 *   Register the INA3221 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/pwrmntr0"
 *   i2c - An instance of the I2C interface to use to communicate with
 *         INA3221
 *   config - Configuration including I2C address, shunt values, and INA3221
 *            configuration mask.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ina3221_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     FAR const struct ina3221_config_s *config)
{
  FAR struct ina3221_dev_s *priv;
  int ret = 0;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the ina3221 device structure */

  priv = (FAR struct ina3221_dev_s *)
                      kmm_malloc(sizeof(struct ina3221_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = config->addr;

  /* Save the config (except opmode) */

  priv->config = config->cfgreg & ~(INA3221_CONFIG_RST);

  /* Copy the shunt resistor values */

  memcpy(priv->shunt_resistor, config->shunt_resistor, 3 * sizeof(int32_t));

  /* Apply config, keep chip switched off */

  ret = ina3221_write16(priv, INA3221_REG_CONFIG, priv->config);
  if (ret < 0)
    {
      snerr("ERROR: Failed to apply config: %d\n", ret);
      goto errout;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_ina3221fops, 0666, priv);
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
