/****************************************************************************
 * drivers/sensors/ltc4151.c
 * Character driver for the LTC 4151 Power Sensor
 *
 *   Copyright (C) 2017 Giorgio Groß. All rights reserved.
 *   Author: Giorgio Groß <giorgio.gross@robodev.eu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ltc4151.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LTC4151)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LTC4151_I2C_FREQUENCY
#  define CONFIG_LTC4151_I2C_FREQUENCY 400000
#endif

#define I2C_NOSTARTSTOP_MSGS 2
#define I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX 0
#define I2C_NOSTARTSTOP_DATA_MSG_INDEX 1

/****************************************************************************
 * Private
 ****************************************************************************/

struct ltc4151_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  float shunt_resistor_value;   /* [ohm] */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* I2C Helpers */

static int     ltc4151_read16(FAR struct ltc4151_dev_s *priv, uint8_t regaddr,
                              FAR uint16_t *regvalue);
static int     ltc4151_readpower(FAR struct ltc4151_dev_s *priv,
                                 FAR struct ltc4151_s *buffer);

/* Character driver methods */

static int     ltc4151_open(FAR struct file *filep);
static int     ltc4151_close(FAR struct file *filep);
static ssize_t ltc4151_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t ltc4151_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     ltc4151_ioctl(FAR struct file *filep,int cmd,unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ltc4151fops =
{
  ltc4151_open,
  ltc4151_close,
  ltc4151_read,
  ltc4151_write,
  NULL,
  ltc4151_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ltc4151_read_reg(FAR struct ltc4151_dev_s *priv,
                            uint8_t start_register_address,
                            FAR uint8_t* register_value, uint8_t data_length)
{
  struct i2c_msg_s msg[I2C_NOSTARTSTOP_MSGS];
  int ret;

  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].frequency = CONFIG_LTC4151_I2C_FREQUENCY;
  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].addr = priv->addr;
  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].flags = 0;
  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].buffer = &start_register_address;
  msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].length = 1;
  msg[I2C_NOSTARTSTOP_DATA_MSG_INDEX].addr = msg[I2C_NOSTARTSTOP_ADDRESS_MSG_INDEX].addr;
  msg[I2C_NOSTARTSTOP_DATA_MSG_INDEX].flags = I2C_M_READ;
  msg[I2C_NOSTARTSTOP_DATA_MSG_INDEX].buffer = register_value;
  msg[I2C_NOSTARTSTOP_DATA_MSG_INDEX].length = data_length;

  ret = I2C_TRANSFER(priv->i2c, msg, I2C_NOSTARTSTOP_MSGS);
  sninfo("start_register_address: "
         "0x%02X data_length: %d register_value: 0x%02x (0x%04x) ret: %d\n",
         start_register_address, data_length, *register_value,
         *((uint16_t*)register_value), ret);
  return ret;
}

static int ltc4151_read16(FAR struct ltc4151_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t* regvalue)
{
  int ret = ltc4151_read_reg(priv, regaddr, (uint8_t*)regvalue,
                             sizeof(*regvalue));

  /* Bytes are 8bit_msb.4bit_0.4bit_lsb = 16bit */

  *regvalue = ((*regvalue & LTC4151_VALUE_MSB_MASK) << 4) |
              ((*regvalue & LTC4151_VALUE_LSB_MASK) >> 8);
  return ret;
}

/****************************************************************************
 * Name: ltc4151_readpower
 *
 * Description:
 *   Read the current and voltage register with special scaling
 *
 ****************************************************************************/

static int ltc4151_readpower(FAR struct ltc4151_dev_s *priv,
                             FAR struct ltc4151_s *buffer)
{
  float float_current;
  float float_voltage;
  uint16_t current_reg;
  uint16_t volt_reg;
  int ret;

  /* Read the raw temperature data (b16_t) */

  ret = ltc4151_read16(priv, LTC4151_CURR_REG, &current_reg);
  if (ret < 0)
  {
    snerr("ERROR: ltc4151_read16 failed: %d\n", ret);
    return ret;
  }

  ret = ltc4151_read16(priv, LTC4151_VOLT_REG, &volt_reg);
  if (ret < 0)
  {
    snerr("ERROR: ltc4151_read16 failed: %d\n", ret);
    return ret;
  }

  /* Current is passed as delta voltage, to get the current divide it by the
   * used resistors resistance.
   *
   * float_current = register in 20 micro volt, shunt in ohm -> result in
   * milliampere
   */

  float_current   = (((float)current_reg) * 20.0 /
                     (1000.0 * priv->shunt_resistor_value));
  buffer->current = ftob16(float_current);

  sninfo("current_reg=0x%04x float_current=%d\n",
         current_reg, (int)float_current);

  /* fload_voltage =  register in 25 micro volt -> result in volt */

  float_voltage   = ((float)volt_reg) * 25.0 / 1000.0;
  buffer->voltage = ftob16(float_voltage);

  sninfo("volt_reg=0x%04x float_voltage=%d\n",
         volt_reg, (int)float_voltage);

  return OK;
}

/****************************************************************************
 * Name: ltc4151_open
 *
 * Description:
 *   This function is called whenever the LTC4151 device is opened.
 *
 ****************************************************************************/

static int ltc4151_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ltc4151_close
 *
 * Description:
 *   This routine is called when the LTC4151 device is closed.
 *
 ****************************************************************************/

static int ltc4151_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ltc4151_read
 ****************************************************************************/

static ssize_t ltc4151_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ltc4151_dev_s *priv   = inode->i_private;
  FAR struct ltc4151_s *ptr;
  ssize_t nsamples;
  int i;
  int ret;

  /* How many samples were requested to get? */

  nsamples = buflen / sizeof(struct ltc4151_s);
  ptr      = (FAR struct ltc4151_s *)buffer;

  sninfo("buflen: %d nsamples: %d\n", buflen, nsamples);

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      struct ltc4151_s pwr;

      /* Read the next struct ltc4151_s power value */

      ret = ltc4151_readpower(priv, &pwr);
      if (ret < 0)
        {
          snerr("ERROR: ltc4151_readtemp failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Save the temperature value in the user buffer */

      *ptr++ = pwr;
    }

  return nsamples * sizeof(struct ltc4151_s);
}

/****************************************************************************
 * Name: ltc4151_write
 ****************************************************************************/

static ssize_t ltc4151_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ltc4151_ioctl
 ****************************************************************************/

static int ltc4151_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ltc4151_register
 *
 * Description:
 *   Register the LTC4151 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/pwrmntr0"
 *   i2c - An instance of the I2C interface to use to communicate with LTC4151
 *   addr - The I2C address of the LTC4151.  The base I2C address of the LTC4151
 *   is 0x6f.  Bits 0-3 can be controlled to get 10 unique addresses from 0x66
 *   through 0x6f.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ltc4151_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr, float shunt_resistor_value)
{
  FAR struct ltc4151_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == CONFIG_LTC4151_ADDR0 || addr == CONFIG_LTC4151_ADDR1 ||
              addr == CONFIG_LTC4151_ADDR2 || addr == CONFIG_LTC4151_ADDR3 ||
              addr == CONFIG_LTC4151_ADDR4 || addr == CONFIG_LTC4151_ADDR5 ||
              addr == CONFIG_LTC4151_ADDR6 || addr == CONFIG_LTC4151_ADDR7 ||
              addr == CONFIG_LTC4151_ADDR8 || addr == CONFIG_LTC4151_ADDR9);

  /* Initialize the ltc4151 device structure */

  priv = (FAR struct ltc4151_dev_s *)kmm_malloc(sizeof(struct ltc4151_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;
  priv->shunt_resistor_value = shunt_resistor_value;

  /* Register the character driver */

  ret = register_driver(devpath, &g_ltc4151fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("ltc4151 (addr=0x%02x) registered at %s\n", priv->addr, devpath);

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_SENSORS_LTC4151 */
