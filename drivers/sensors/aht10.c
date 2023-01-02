/****************************************************************************
 * drivers/sensors/aht10.c
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

#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/aht10.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_AHT10)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_AHT10_I2C_FREQUENCY
#  define CONFIG_AHT10_I2C_FREQUENCY 400000
#endif

/* I2C command bytes */

#define AHT10_SOFT_INIT              0xe1
#define AHT10_MEAS_TRIG              0xac
#define AHT10_MEAS_READ              0x71
#define AHT10_NORMAL_CMD             0xa8

/****************************************************************************
 * Private
 ****************************************************************************/

struct aht10_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;                /* True, driver has been unlinked */
#endif
  int temperature;              /* Cached temperature */
  int humidity;                 /* Cached humidity */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  int16_t crefs;                /* Number of open references */
#endif
  mutex_t devlock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int aht10_writereg(FAR struct aht10_dev_s *priv,
                          uint8_t reg, FAR uint8_t *data);
static int aht10_readregs(FAR struct aht10_dev_s *priv,
                          FAR uint8_t *buf, uint8_t len);

/* Character driver methods */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     aht10_open(FAR struct file *filep);
static int     aht10_close(FAR struct file *filep);
#endif
static ssize_t aht10_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t aht10_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     aht10_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     aht10_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_aht10fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  aht10_open,     /* open */
  aht10_close,    /* close */
#else
  NULL,           /* open */
  NULL,           /* close */
#endif
  aht10_read,     /* read */
  aht10_write,    /* write */
  NULL,           /* seek */
  aht10_ioctl,    /* ioctl */
  NULL,           /* truncate */
  NULL,           /* mmap */
  NULL            /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , aht10_unlink /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aht10_writereg
 *
 * Description:
 *   I2C write access helper.
 *
 ****************************************************************************/

static int aht10_writereg(FAR struct aht10_dev_s *priv,
                          uint8_t reg, FAR uint8_t *data)
{
  struct i2c_msg_s msg;
  uint8_t buf[3];
  int ret;

  buf[0] = reg;
  buf[1] = data[0];
  buf[2] = data[1];

  msg.frequency = CONFIG_AHT10_I2C_FREQUENCY;
  msg.addr = priv->addr;
  msg.flags = 0;
  msg.buffer = buf;
  msg.length = 3;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);

  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: aht10_readregs
 *
 * Description:
 *   I2C read access helper.
 *
 ****************************************************************************/

static int aht10_readregs(FAR struct aht10_dev_s *priv,
                          FAR uint8_t *buf, uint8_t len)
{
  struct i2c_msg_s msg;
  int ret;

  msg.frequency = CONFIG_AHT10_I2C_FREQUENCY;
  msg.addr = priv->addr;
  msg.flags = I2C_M_READ;
  msg.buffer = buf;
  msg.length = len;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);

  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: aht10_initialize
 *
 * Description:
 *   Initialize the AHT10 sensor. This takes less than 20 ms.
 *
 ****************************************************************************/

static int aht10_initialize(FAR struct aht10_dev_s *priv)
{
  int ret;
  uint8_t buf[2];

  buf[0] = 0x00;
  buf[1] = 0x00;

  ret = aht10_writereg(priv, AHT10_NORMAL_CMD, buf);
  if (ret < 0)
    {
      snerr("ERROR: write normal command failed: %d\n", ret);
      return ret;
    }

  /* wait at least 300ms */

  nxsig_usleep(300000);

  buf[0] = 0x08;
  buf[1] = 0x00;

  ret = aht10_writereg(priv, AHT10_SOFT_INIT, buf);
  if (ret < 0)
    {
      snerr("ERROR: send calibrate command failed: %d\n", ret);
      return ret;
    }

  /* wait at least 300ms */

  nxsig_usleep(300000);

  return ret;
}

/****************************************************************************
 * Name: aht10_temp_to_mcelsius
 *
 * Description:
 *   Convert raw temperature value to milli celsius.
 *
 ****************************************************************************/

static inline int aht10_temp_to_mcelsius(uint32_t raw)
{
  /* Formula T = -50 + 200 * (ST / 2^20) from datasheet,
   * converted to integer fixed point (3 digits) representation.
   */

  return ((raw * 3125) >> 14) - 50000;
}

/****************************************************************************
 * Name: aht10_rh_to_pcm
 *
 * Description:
 *   Convert raw humidity value to one-thousandths of a percent
 *   (per cent mille) relative humidity.
 *
 ****************************************************************************/

static inline int aht10_rh_to_pcm(uint32_t raw)
{
  /* Formula RH = (SRH / 2^20) * 100% from datasheet,
   * converted to integer fixed point (3 digits) representation.
   */

  return (raw >> 15) * 3125;
}

/****************************************************************************
 * Name: aht10_read_values
 *
 * Description: read raw data from the sensor.
 *
 ****************************************************************************/

static int aht10_read_values(FAR struct aht10_dev_s *priv, FAR int *temp,
                             FAR int *rh)
{
  uint8_t buf[6];
  uint32_t temp20;
  uint32_t rh20;
  int ret;

  /* Sample command */

  buf[0] = 0x33;
  buf[1] = 0x00;

  ret = aht10_writereg(priv, AHT10_MEAS_TRIG, buf);
  if (ret < 0)
    {
      snerr("ERROR: aht10_writereg failed: %d\n", ret);
      return ret;
    }

  /* Read the raw temperature data and humidity data */

  ret = aht10_readregs(priv, buf, 6);
  if (ret < 0)
    {
      snerr("ERROR: aht10_readregs failed: %d\n", ret);
      return ret;
    }

  /* Check the AHT10 has been calibrated. */

  if ((buf[0] & 0x68) != 0x08)
    {
      aht10_initialize(priv);
      snwarn("WARNING: aht10 are not calibrated.\n");
    }

  /* Humidity data (20bits). */

  rh20 = ((buf[1] << 12) | (buf[2] << 4) |
         ((buf[3] & 0xf0) >> 4)) & 0x000fffff;

  /* Temperature data (20bits). */

  temp20 = (((buf[3] & 0x0f) << 16) |
             (buf[4] << 8) | buf[5]) & 0x000fffff;

  add_sensor_randomness((int)temp20 << 16 | rh20);

  /* Sensor data convert to reality */

  priv->temperature = aht10_temp_to_mcelsius(temp20);
  priv->humidity = aht10_rh_to_pcm(rh20);

  *temp = priv->temperature;
  *rh = priv->humidity;
  return OK;
}

/****************************************************************************
 * Name: aht10_open
 *
 * Description:
 *   This function is called whenever the AHT10 device is opened.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int aht10_open(FAR struct file *filep)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct aht10_dev_s *priv  = inode->i_private;

  /* Get exclusive access */

  nxmutex_lock(&priv->devlock);

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  nxmutex_unlock(&priv->devlock);
  return OK;
}
#endif

/****************************************************************************
 * Name: aht10_close
 *
 * Description:
 *   This routine is called when the SHT2x device is closed.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int aht10_close(FAR struct file *filep)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct aht10_dev_s *priv  = inode->i_private;

  /* Get exclusive access */

  nxmutex_lock(&priv->devlock);

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero and the driver has been unlinked,
   * then free memory now.
   */

  if (priv->crefs <= 0 && priv->unlinked)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return OK;
    }

  nxmutex_unlock(&priv->devlock);
  return OK;
}
#endif

/****************************************************************************
 * Name: aht10_read
 ****************************************************************************/

static ssize_t aht10_read(FAR struct file *filep,
                          FAR char *buffer, size_t buflen)
{
  FAR struct inode       *inode  = filep->f_inode;
  FAR struct aht10_dev_s *priv   = inode->i_private;
  ssize_t                 length = 0;
  int temp;
  int rh;
  int ret;

  /* Get exclusive access */

  nxmutex_lock(&priv->devlock);

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked sensors. This allows
       * sensor use on hot swappable I2C bus.
       */

      nxmutex_unlock(&priv->devlock);
      return -ENODEV;
    }
#endif

  ret = aht10_read_values(priv, &temp, &rh);
  if (ret < 0)
    {
      sninfo("cannot read data: %d\n", ret);
    }
  else
    {
      /* This interface is mainly intended for easy debugging in nsh. */

      length = snprintf(buffer, buflen, "%d %d\n", temp, rh);
      if (length > buflen)
        {
          length = buflen;
        }
    }

  nxmutex_unlock(&priv->devlock);
  return length;
}

/****************************************************************************
 * Name: aht10_write
 ****************************************************************************/

static ssize_t aht10_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: aht10_ioctl
 ****************************************************************************/

static int aht10_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct aht10_dev_s *priv = inode->i_private;
  int ret;

  /* Get exclusive access */

  nxmutex_lock(&priv->devlock);

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked sensors. This allows
       * sensor use on hot swappable I2C bus.
       */

      nxmutex_unlock(&priv->devlock);
      return -ENODEV;
    }
#endif

  switch (cmd)
    {
      /* Soft reset the AHT10, Arg: None */

      case SNIOC_RESET:
        {
          ret = aht10_initialize(priv);
          sninfo("softreset ret: %d\n", ret);
        }
        break;

      case SNIOC_READ_RAW_DATA:
      case SNIOC_READ_CONVERT_DATA:
        {
          int temp;
          int rh;

          ret = aht10_read_values(priv, &temp, &rh);
          if (ret < 0)
            {
              sninfo("cannot read data: %d\n", ret);
            }
          else
            {
              FAR struct aht10_conv_data_s *data =
                (FAR struct aht10_conv_data_s *)arg;

              data->temperature = temp;
              data->humidity = rh;
            }
        }
        break;

      default:
          sninfo("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: aht10_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int aht10_unlink(FAR struct inode *inode)
{
  FAR struct aht10_dev_s *priv;
  int ret;

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = (FAR struct aht10_dev_s *)inode->i_private;

  /* Get exclusive access */

  nxmutex_lock(&priv->devlock);

  /* Are there open references to the driver data structure? */

  if (priv->crefs <= 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return OK;
    }

  /* No... just mark the driver as unlinked and free the resources when
   * the last client closes their reference to the driver.
   */

  priv->unlinked = true;
  nxmutex_unlock(&priv->devlock);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aht10_register
 *
 * Description:
 *   Register the AHT10 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the AHT10
 *   addr    - The I2C address of the AHT10 (always 0x38).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int aht10_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t addr)
{
  FAR struct aht10_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == CONFIG_AHT10_ADDR);

  /* Initialize the device structure */

  priv = (FAR struct aht10_dev_s *)kmm_zalloc(sizeof(struct aht10_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  nxmutex_init(&priv->devlock);

  ret = aht10_initialize(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize AHT10: %d\n", ret);
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return ret;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_aht10fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_SENSORS_AHT10 */
