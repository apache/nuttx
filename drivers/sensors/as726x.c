/****************************************************************************
 * drivers/sensors/as726x.c
 * Character driver for the AS7263 6-Ch NIR Spectral Sensing Engine
 * and AS7262 Consumer Grade Smart 6-Channel VIS Sensor
 *
 *   Copyright (C) 2019 Fabian Justi. All rights reserved.
 *   Author: Fabian Justi <Fabian.Justi@gmx.de> and
 *           Andreas Kurz <andreas.kurz@methodpark.de>
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

#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/as726x.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_AS726X)

/****************************************************************************
 * Pre-process Definitions
 ****************************************************************************/

#ifndef CONFIG_AS726X_I2C_FREQUENCY
#  define CONFIG_AS726X_I2C_FREQUENCY 100000
#endif

#define AS726X_INTEGRATION_TIME 50
#define AS726X_GAIN             0b01   /* Set gain to 64x */
#define AS726X_MEASURMENT_MODE  0b10   /* One-shot reading of VBGYOR or RSTUVW */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct as726x_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static float as726x_getcalibrated(FAR struct as726x_dev_s *priv,
                                  uint8_t regaddr);
static int16_t as726x_getchannel(FAR struct as726x_dev_s *priv,
                                 uint8_t regaddr);

/* I2C Helpers */

static uint8_t read_register(FAR struct as726x_dev_s *priv, uint8_t addr);
static uint8_t as726x_read8(FAR struct as726x_dev_s *priv, uint8_t regval);
static void write_register(FAR struct as726x_dev_s *priv, uint8_t addr,
                          uint8_t val);
static void as726x_write8(FAR struct as726x_dev_s *priv, uint8_t regaddr,
                          uint8_t regval);

/* Character driver methods */

static int as726x_open(FAR struct file *filep);
static int as726x_close(FAR struct file *filep);
static ssize_t as726x_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t as726x_write(FAR struct file *filep,
                            FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_as726x_fops =
{
  as726x_open,                  /* open */
  as726x_close,                 /* close */
  as726x_read,                  /* read */
  as726x_write,                 /* write */
  NULL,                         /* seek */
  NULL,                         /* ioctl */
  NULL                          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: as726x_getcalibrated
 *
 * Description:
 *   Read calibrated value
 *
 ****************************************************************************/

static float as726x_getcalibrated(FAR struct as726x_dev_s *priv,
                                  uint8_t regaddr)
{
  uint8_t byte0 = as726x_read8(priv, regaddr + 0);
  uint8_t byte1 = as726x_read8(priv, regaddr + 1);
  uint8_t byte2 = as726x_read8(priv, regaddr + 2);
  uint8_t byte3 = as726x_read8(priv, regaddr + 3);

  uint32_t colourdata = ((uint32_t) byte0 << (8 * 3));
  colourdata |= ((uint32_t) byte1 << (8 * 2));
  colourdata |= ((uint32_t) byte2 << (8 * 1));
  colourdata |= ((uint32_t) byte3 << (8 * 0));

  return *((float *)(&colourdata));
}

/****************************************************************************
 * Name: as726x_getchannel
 *
 * Description:
 *   Read colour channel
 *
 ****************************************************************************/

static int16_t as726x_getchannel(FAR struct as726x_dev_s *priv,
                                 uint8_t regaddr)
{
  int16_t colourdata = as726x_read8(priv, regaddr) << 8;
  colourdata |= as726x_read8(priv, regaddr + 1);
  return colourdata;
}

/****************************************************************************
 * Name: as726x_read8
 *
 * Description:
 *   Read 8-bit register
 *
 ****************************************************************************/

static uint8_t read_register(FAR struct as726x_dev_s *priv, uint8_t addr)
{
  struct i2c_config_s config;
  uint8_t regval = 0;
  int ret;

  config.frequency = CONFIG_AS726X_I2C_FREQUENCY;
  config.address = AS726X_I2C_ADDR;
  config.addrlen = 7;

  ret = i2c_write(priv->i2c, &config, &addr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  ret = i2c_read(priv->i2c, &config, &regval, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  return regval;
}

static uint8_t as726x_read8(FAR struct as726x_dev_s *priv, uint8_t regaddr)
{
  uint8_t status;

  status = read_register(priv, AS72XX_SLAVE_STATUS_REG);
  if ((status & AS72XX_SLAVE_RX_VALID) != 0)
    {
      /* There is data to be read.
       * Read the byte but do nothing with it.
       */

      read_register(priv, AS72XX_SLAVE_READ_REG);
    }

  /* Wait for WRITE flag to clear */

  while (1)
    {
      status = read_register(priv, AS72XX_SLAVE_STATUS_REG);
      if ((status & AS72XX_SLAVE_TX_VALID) == 0)
        {
          break;  /* If TX bit is clear, it is ok to write */
        }

      nxsig_usleep(AS726X_POLLING_DELAY);
    }

  /* Send the virtual register address (bit 7 should be 0 to indicate we are
   * reading a register).
   */

  write_register(priv, AS72XX_SLAVE_WRITE_REG, regaddr);

  /* Wait for READ flag to be set */

  while (1)
    {
      status = read_register(priv, AS72XX_SLAVE_STATUS_REG);
      if ((status & AS72XX_SLAVE_RX_VALID) != 0)
        {
          break;  /* Read data is ready. */
        }

      nxsig_usleep(AS726X_POLLING_DELAY);
    }

  uint8_t incoming = read_register(priv, AS72XX_SLAVE_READ_REG);
  return incoming;
}

/****************************************************************************
 * Name: as726x_write8
 *
 * Description:
 *   Write from an 8-bit register
 *
 ****************************************************************************/

static void write_register(FAR struct as726x_dev_s *priv, uint8_t addr,
                           uint8_t val)
{
  struct i2c_config_s config;
  uint8_t msg[2] =
  {
    0
  };

  int ret;

  config.frequency = CONFIG_AS726X_I2C_FREQUENCY;
  config.address = AS726X_I2C_ADDR;
  config.addrlen = 7;

  msg[0] = addr;
  msg[1] = val;

  ret = i2c_write(priv->i2c, &config, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return;
    }
}

static void as726x_write8(FAR struct as726x_dev_s *priv, uint8_t regaddr,
                          uint8_t regval)
{
  uint8_t status;

  while (1)
    {
      status = read_register(priv, AS72XX_SLAVE_STATUS_REG);
      if ((status & AS72XX_SLAVE_TX_VALID) == 0)
        {
          /* No inbound TX pending at slave. Okay to write now. */

          break;
        }

      nxsig_usleep(AS726X_POLLING_DELAY);
    }

  /* Send the virtual register address (setting bit 7 to indicate we are
   * writing to a register).
   */

  write_register(priv, AS72XX_SLAVE_WRITE_REG, (regaddr | 0x80));

  /* Wait for WRITE register to be empty */

  while (1)
    {
      status = read_register(priv, AS72XX_SLAVE_STATUS_REG);
      if ((status & AS72XX_SLAVE_TX_VALID) == 0)
        {
          /* No inbound TX pending at slave. Okay to write now. */

          break;
        }

      nxsig_usleep(AS726X_POLLING_DELAY);
    }

  /* Send the data to complete the operation. */

  write_register(priv, AS72XX_SLAVE_WRITE_REG, regval);
}

/****************************************************************************
 * Name: as726x_open
 *
 * Description:
 *   This function is called whenever the AS726X device is opened.
 *
 ****************************************************************************/

static int as726x_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: as726x_close
 *
 * Description:
 *   This routine is called when the AS726X device is closed.
 *
 ****************************************************************************/

static int as726x_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: as726x_read
 ****************************************************************************/

static ssize_t as726x_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct as726x_dev_s *priv = inode->i_private;
  FAR struct as726x_sensor_data_s *data =
    (FAR struct as726x_sensor_data_s *)buffer;

  /* Check if the user is reading the right size */

  if (buflen < sizeof(FAR struct as726x_sensor_data_s))
    {
      snerr("ERROR: Not enough memory for reading all channels.\n");
      return -ENOSYS;
    }
  else
    {
      data->v_r_value = as726x_getchannel(priv, AS726X_V_R);
      data->b_s_value = as726x_getchannel(priv, AS726X_B_S);
      data->g_t_value = as726x_getchannel(priv, AS726X_G_T);
      data->y_u_value = as726x_getchannel(priv, AS726X_Y_U);
      data->o_v_value = as726x_getchannel(priv, AS726X_O_V);
      data->r_w_value = as726x_getchannel(priv, AS726X_R_W);
    }

  return buflen;
}

/****************************************************************************
 * Name: as726x_write
 ****************************************************************************/

static ssize_t as726x_write(FAR struct file *filep,
                            FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: as726x_register
 *
 * Description:
 *   Register the AS726X character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/spectr0"
 *   i2c - An instance of the I2C interface to use to communicate with AS726X
 *   addr - The I2C address of the AS726X.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int as726x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  uint8_t _sensor_version;
  uint8_t value;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the AS726X device structure */

  FAR struct as726x_dev_s *priv =
    (FAR struct as726x_dev_s *)kmm_malloc(sizeof(struct as726x_dev_s));

  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = AS726X_I2C_ADDR;

  /* Check HW version for AS7262 and AS7263 */

  _sensor_version = as726x_read8(priv, AS726x_HW_VERSION);
  if (_sensor_version != 0x3e && _sensor_version != 0x3f)
    {
      snerr("ID (should be 0x3e or 0x3f): 0x %d\n", ret);
    }

  /* Initialize the device
   * Read the register value toggle and disable led
   */

  ret = as726x_read8(priv, AS726x_LED_CONTROL);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize the AS726X!\n");
      return ret;
    }

  value  = ret;
  value &= ~(1 << 0);           /* Clear the bit */

  as726x_write8(priv, AS726x_LED_CONTROL, value);

  /* If you use Mode 2 or 3 (all the colors) then integration time is double.
   * 140*2 = 280ms between readings.
   * 50 * 2.8ms = 140ms. 0 to 255 is valid.
   */

  as726x_write8(priv, AS726x_INT_T, AS726X_INTEGRATION_TIME);

  ret = as726x_read8(priv, AS726x_CONTROL_SETUP);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize the AS726X!\n");
      return ret;
    }

  value  = ret;
  value &= 0b11001111;         /* Clear GAIN bits */
  value |= (AS726X_GAIN << 4);  /* Set GAIN bits with user's choice */

  as726x_write8(priv, AS726x_CONTROL_SETUP, value);

  ret = as726x_read8(priv, AS726x_CONTROL_SETUP);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize the AS726X!\n");
      return ret;
    }

  value  = ret;
  value &= 0b11110011;                     /* Clear BANK bits */
  value |= (AS726X_MEASURMENT_MODE << 2);   /* Set BANK bits with user's
                                             * choice */

  as726x_write8(priv, AS726x_CONTROL_SETUP, value);

  /* Register the character driver */

  ret = register_driver(devpath, &g_as726x_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_AS726X */
