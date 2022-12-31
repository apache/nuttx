/****************************************************************************
 * drivers/sensors/lm75.c
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
#include <fixedmath.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lm75.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_LM75_I2C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LM75_I2C_FREQUENCY
#  define CONFIG_LM75_I2C_FREQUENCY 100000
#endif

/* Centigrade to Fahrenheit conversion:  F = 9*C/5 + 32 */

#define B16_9DIV5  (9 * 65536 / 5)
#define B16_32     (32 * 65536)

/****************************************************************************
 * Private
 ****************************************************************************/

struct lm75_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  bool fahrenheit;              /* true: temperature will be reported in fahrenheit */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     lm75_i2c_write(FAR struct lm75_dev_s *priv,
                              FAR const uint8_t *buffer, int buflen);
static int     lm75_i2c_read(FAR struct lm75_dev_s *priv,
                             FAR uint8_t *buffer, int buflen);
static int     lm75_readb16(FAR struct lm75_dev_s *priv, uint8_t regaddr,
                            FAR b16_t *regvalue);
static int     lm75_writeb16(FAR struct lm75_dev_s *priv, uint8_t regaddr,
                             b16_t regval);
static int     lm75_readtemp(FAR struct lm75_dev_s *priv, FAR b16_t *temp);
static int     lm75_readconf(FAR struct lm75_dev_s *priv, FAR uint8_t *conf);
static int     lm75_writeconf(FAR struct lm75_dev_s *priv, uint8_t conf);

/* Character driver methods */

static ssize_t lm75_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t lm75_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static int     lm75_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lm75fops =
{
  NULL,            /* open */
  NULL,            /* close */
  lm75_read,       /* read */
  lm75_write,      /* write */
  NULL,            /* seek */
  lm75_ioctl,      /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm75_i2c_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static int lm75_i2c_write(FAR struct lm75_dev_s *priv,
                          FAR const uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_LM75_I2C_FREQUENCY,
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)buffer;  /* Override const */
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: lm75_i2c_read
 *
 * Description:
 *   Read from the I2C device.
 *
 ****************************************************************************/

static int lm75_i2c_read(FAR struct lm75_dev_s *priv,
                         FAR uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_LM75_I2C_FREQUENCY,
  msg.addr      = priv->addr,
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: lm75_readb16
 *
 * Description:
 *   Read a 16-bit register (LM75_TEMP_REG, LM75_THYS_REG, or LM75_TOS_REG)
 *
 ****************************************************************************/

static int lm75_readb16(FAR struct lm75_dev_s *priv, uint8_t regaddr,
                        FAR b16_t *regvalue)
{
  uint8_t buffer[2];
  int ret;

  /* Write the register address */

  ret = lm75_i2c_write(priv, &regaddr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 16-bits from the register (discarding 7) */

  ret = lm75_i2c_read(priv, buffer, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* Data format is:  TTTTTTTT Txxxxxxx where TTTTTTTTT is a nine-bit,
   * signed temperature value with LSB = 0.5 degrees centigrade.  So the
   * raw data is b8_t
   */

  *regvalue = b8tob16((b8_t)buffer[0] << 8 | (b8_t)buffer[1]);
  sninfo("addr: %02x value: %08" PRIx32 " ret: %d\n",
         regaddr, *regvalue, ret);
  return OK;
}

/****************************************************************************
 * Name: lm75_writeb16
 *
 * Description:
 *   Write to a 16-bit register (LM75_TEMP_REG, LM75_THYS_REG, or
 *   LM75_TOS_REG)
 *
 ****************************************************************************/

static int lm75_writeb16(FAR struct lm75_dev_s *priv, uint8_t regaddr,
                         b16_t regval)
{
  uint8_t buffer[3];
  b8_t regb8;

  sninfo("addr: %02x value: %08" PRIx32 "\n", regaddr, regval);

  /* Set up a 3 byte message to send */

  buffer[0] = regaddr;

  regb8 = b16tob8(regval);
  buffer[1] = (uint8_t)(regb8 >> 8);
  buffer[2] = (uint8_t)regb8;

  /* Write the register address followed by the data (no RESTART) */

  return lm75_i2c_write(priv, buffer, 3);
}

/****************************************************************************
 * Name: lm75_readtemp
 *
 * Description:
 *   Read the temperature register with special scaling (LM75_TEMP_REG)
 *
 ****************************************************************************/

static int lm75_readtemp(FAR struct lm75_dev_s *priv, FAR b16_t *temp)
{
  b16_t temp16;
  int ret;

  /* Read the raw temperature data (b16_t) */

  ret = lm75_readb16(priv, LM75_TEMP_REG, &temp16);
  if (ret < 0)
    {
      snerr("ERROR: lm75_readb16 failed: %d\n", ret);
      return ret;
    }

  add_sensor_randomness(temp16);

  sninfo("Centigrade: %08" PRIx32 "\n", temp16);

  /* Was fahrenheit requested? */

  if (priv->fahrenheit)
    {
      /* Centigrade to Fahrenheit conversion:  F = 9*C/5 + 32 */

      temp16 =  b16mulb16(temp16, B16_9DIV5) + B16_32;
      sninfo("Fahrenheit: %08" PRIx32 "\n", temp16);
    }

  *temp = temp16;
  return OK;
}

/****************************************************************************
 * Name: lm75_readconf
 *
 * Description:
 *   Read the 8-bit LM75 configuration register
 *
 ****************************************************************************/

static int lm75_readconf(FAR struct lm75_dev_s *priv, FAR uint8_t *conf)
{
  uint8_t buffer;
  int ret;

  /* Write the configuration register address */

  buffer = LM75_CONF_REG;

  ret = lm75_i2c_write(priv, &buffer, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8-bits from the register */

  ret = lm75_i2c_read(priv, conf, 1);
  sninfo("conf: %02x ret: %d\n", *conf, ret);
  return ret;
}

/****************************************************************************
 * Name: lm75_writeconf
 *
 * Description:
 *   Write to a 8-bit LM75 configuration register.
 *
 ****************************************************************************/

static int lm75_writeconf(FAR struct lm75_dev_s *priv, uint8_t conf)
{
  uint8_t buffer[2];

  sninfo("conf: %02x\n", conf);

  /* Set up a 2 byte message to send */

  buffer[0] = LM75_CONF_REG;
  buffer[1] = conf;

  /* Write the register address followed by the data (no RESTART) */

  return lm75_i2c_write(priv, buffer, 2);
}

/****************************************************************************
 * Name: lm75_read
 ****************************************************************************/

static ssize_t lm75_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct lm75_dev_s *priv   = inode->i_private;
  FAR b16_t             *ptr;
  ssize_t                nsamples;
  int                    i;
  int                    ret;

  /* How many samples were requested to get? */

  nsamples = buflen / sizeof(b16_t);
  ptr      = (FAR b16_t *)buffer;

  sninfo("buflen: %d nsamples: %d\n", buflen, nsamples);

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      b16_t temp = 0;

      /* Read the next b16_t temperature value */

      ret = lm75_readtemp(priv, &temp);
      if (ret < 0)
        {
          snerr("ERROR: lm75_readtemp failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Save the temperature value in the user buffer */

      *ptr++ = temp;
    }

  return nsamples * sizeof(b16_t);
}

/****************************************************************************
 * Name: lm75_write
 ****************************************************************************/

static ssize_t lm75_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lm75_ioctl
 ****************************************************************************/

static int lm75_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct lm75_dev_s *priv  = inode->i_private;
  int                    ret   = OK;

  switch (cmd)
    {
      /* Read from the configuration register. Arg: uint8_t* pointer */

      case SNIOC_READCONF:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = lm75_readconf(priv, ptr);
          sninfo("conf: %02x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write to the configuration register. Arg:  uint8_t value */

      case SNIOC_WRITECONF:
        ret = lm75_writeconf(priv, (uint8_t)arg);
        sninfo("conf: %02x ret: %d\n", *(FAR uint8_t *)arg, ret);
        break;

      /* Shutdown the LM75, Arg: None */

      case SNIOC_SHUTDOWN:
        {
          uint8_t conf;
          ret = lm75_readconf(priv, &conf);
          if (ret == OK)
            {
              ret = lm75_writeconf(priv, conf | LM75_CONF_SHUTDOWN);
            }

          sninfo("conf: %02x ret: %d\n", conf | LM75_CONF_SHUTDOWN, ret);
        }
        break;

      /* Powerup the LM75, Arg: None */

      case SNIOC_POWERUP:
        {
          uint8_t conf;
          ret = lm75_readconf(priv, &conf);
          if (ret == OK)
            {
              ret = lm75_writeconf(priv, conf & ~LM75_CONF_SHUTDOWN);
            }

          sninfo("conf: %02x ret: %d\n", conf & ~LM75_CONF_SHUTDOWN, ret);
        }
        break;

      /* Report samples in Fahrenheit */

      case SNIOC_FAHRENHEIT:
        priv->fahrenheit = true;
        sninfo("Fahrenheit\n");
        break;

      /* Report Samples in Centigrade */

      case SNIOC_CENTIGRADE:
        priv->fahrenheit = false;
        sninfo("Centigrade\n");
        break;

      /* Read THYS temperature register.  Arg: b16_t* pointer */

      case SNIOC_READTHYS:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = lm75_readb16(priv, LM75_THYS_REG, ptr);
          sninfo("THYS: %08" PRIx32 " ret: %d\n", *ptr, ret);
        }
        break;

      /* Write THYS temperature register. Arg: b16_t value */

      case SNIOC_WRITETHYS:
        ret = lm75_writeb16(priv, LM75_THYS_REG, (b16_t)arg);
        sninfo("THYS: %08" PRIx32 " ret: %d\n", (b16_t)arg, ret);
        break;

      /* Read TOS (Over-temp Shutdown Threshold) Register.
       * Arg: b16_t* pointer
       */

      case SNIOC_READTOS:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = lm75_readb16(priv, LM75_TOS_REG, ptr);
          sninfo("TOS: %08" PRIx32 " ret: %d\n", *ptr, ret);
        }
        break;

      /* Write TOS (Over-temp Shutdown Threshold) Register.
       * Arg: b16_t value
       */

      case SNIOC_WRITETOS:
        ret = lm75_writeb16(priv, LM75_TOS_REG, (b16_t)arg);
        sninfo("TOS: %08" PRIx32 " ret: %d\n", (b16_t)arg, ret);
        break;

      default:
        sninfo("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm75_register
 *
 * Description:
 *   Register the LM-75 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c - An instance of the I2C interface to use to communicate with LM75
 *   addr - The I2C address of the LM-75.  The base I2C address of the LM75
 *   is 0x48.  Bits 0-3 can be controlled to get 8 unique addresses from 0x48
 *   through 0x4f.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lm75_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                  uint8_t addr)
{
  FAR struct lm75_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == CONFIG_LM75_ADDR0 || addr == CONFIG_LM75_ADDR1 ||
              addr == CONFIG_LM75_ADDR2 || addr == CONFIG_LM75_ADDR3 ||
              addr == CONFIG_LM75_ADDR4 || addr == CONFIG_LM75_ADDR5 ||
              addr == CONFIG_LM75_ADDR6 || addr == CONFIG_LM75_ADDR7);

  /* Initialize the LM-75 device structure */

  priv = (FAR struct lm75_dev_s *)kmm_malloc(sizeof(struct lm75_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->fahrenheit = false;

  /* Register the character driver */

  ret = register_driver(devpath, &g_lm75fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_LM75_I2C */
