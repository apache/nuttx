/****************************************************************************
 * drivers/sensors/sht21.c
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
#include <time.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sht21.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_SHT21)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SHT21_DEBUG
#  define sht21_dbg(x, ...)    _info(x, ##__VA_ARGS__)
#else
#  define sht21_dbg(x, ...)    sninfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_SHT21_I2C_FREQUENCY
#  define CONFIG_SHT21_I2C_FREQUENCY 400000
#endif

/* I2C command bytes */

#define SHT21_TRIG_T_MEAS_HM         0xe3
#define SHT21_TRIG_RH_MEAS_HM        0xe5
#define SHT21_WRITE_USERREG          0xe6
#define SHT21_READ_USERREG           0xe7
#define SHT21_SOFT_RESET             0xfe

/****************************************************************************
 * Private
 ****************************************************************************/

struct sht21_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  bool valid;                   /* If cached readings are valid */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;                /* True, driver has been unlinked */
#endif
  struct timespec last_update;  /* Last time when sensor was read */
  int temperature;              /* Cached temperature */
  int humidity;                 /* Cached humidity */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  int16_t crefs;                /* Number of open references */
#endif
  sem_t devsem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int sht21_access(FAR struct sht21_dev_s *priv,
                        uint8_t reg_addr, bool read,
                        FAR uint8_t *reg_value, uint8_t len);
#ifdef CONFIG_SHT21_DEBUG
static int sht21_read8(FAR struct sht21_dev_s *priv, uint8_t regaddr,
                       FAR uint8_t *regvalue);
#endif
static int sht21_read16(FAR struct sht21_dev_s *priv, uint8_t regaddr,
                        FAR uint16_t *regvalue);

/* Character driver methods */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     sht21_open(FAR struct file *filep);
static int     sht21_close(FAR struct file *filep);
#endif
static ssize_t sht21_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t sht21_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     sht21_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     sht21_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_sht21fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  sht21_open,     /* open */
  sht21_close,    /* close */
#else
  NULL,           /* open */
  NULL,           /* close */
#endif
  sht21_read,     /* read */
  sht21_write,    /* write */
  NULL,           /* seek */
  sht21_ioctl,    /* ioctl */
  NULL            /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , sht21_unlink /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sht21_access
 *
 * Description:
 *   I2C access helper.
 *
 ****************************************************************************/

static int sht21_access(FAR struct sht21_dev_s *priv,
                        uint8_t reg_addr, bool read,
                        FAR uint8_t *reg_value, uint8_t len)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = CONFIG_SHT21_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = &reg_addr;
  msg[0].length = 1;

  msg[1].frequency = CONFIG_SHT21_I2C_FREQUENCY;
  msg[1].addr = priv->addr;
  msg[1].flags = read ? I2C_M_READ : I2C_M_NOSTART;
  msg[1].buffer = reg_value;
  msg[1].length = len;

  ret = I2C_TRANSFER(priv->i2c, msg, (len > 0) ? 2 : 1);

  sht21_dbg("reg_addr: 0x%02X len: %d reg_value: 0x%02x ret: %d\n",
            reg_addr, len, reg_value ? *reg_value : 0, ret);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: sht21_read8
 *
 * Description:
 *   Read 8-bit value from the I2C device.
 *
 ****************************************************************************/

#ifdef CONFIG_SHT21_DEBUG
static int sht21_read8(FAR struct sht21_dev_s *priv, uint8_t regaddr,
                       FAR uint8_t *regvalue)
{
  uint8_t buf[1];
  int ret;

  ret = sht21_access(priv, regaddr, true, buf, 1);
  if (ret == 0)
    {
      *regvalue = buf[0];
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: sht21_read16
 *
 * Description:
 *   Read 16-bit value from the I2C device (MSB first).
 *
 ****************************************************************************/

static int sht21_read16(FAR struct sht21_dev_s *priv, uint8_t regaddr,
                        FAR uint16_t *regvalue)
{
  uint8_t buf[2];
  int ret;

  ret = sht21_access(priv, regaddr, true, buf, 2);
  if (ret == 0)
    {
      *regvalue = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    }

  return ret;
}

/****************************************************************************
 * Name: sht21_softreset
 *
 * Description:
 *   Reset the SHT2x sensor. This takes less than 15 ms.
 *
 ****************************************************************************/

static inline int sht21_softreset(FAR struct sht21_dev_s *priv)
{
  return sht21_access(priv, SHT21_SOFT_RESET, false, NULL, 0);
}

/****************************************************************************
 * Name: has_time_passed
 *
 * Description:
 *   Return true if curr >= start + secs_since_start
 *
 ****************************************************************************/

static bool has_time_passed(struct timespec curr,
                            struct timespec start,
                            unsigned int secs_since_start)
{
  if ((long)((start.tv_sec + secs_since_start) - curr.tv_sec) == 0)
    {
      return start.tv_nsec <= curr.tv_nsec;
    }

  return (long)((start.tv_sec + secs_since_start) - curr.tv_sec) <= 0;
}

/****************************************************************************
 * Name: sht21_temp_to_mcelsius
 *
 * Description:
 *   Convert raw temperature value to milli celsius.
 *
 ****************************************************************************/

static inline int sht21_temp_to_mcelsius(int raw)
{
  /* Clear status bits. */

  raw &= ~0x03;

  /* Formula T = -46.85 + 175.72 * ST / 2^16 from datasheet 6.2,
   * converted to integer fixed point (3 digits) representation.
   */

  return (((175720 >> 3) * raw) >> 13) - 46850;
}

/****************************************************************************
 * Name: sht21_rh_to_pcm
 *
 * Description:
 *   Convert raw humidity value to one-thousandths of a percent
 *   (per cent mille) relative humidity.
 *
 ****************************************************************************/

static inline int sht21_rh_to_pcm(int raw)
{
  /* Clear status bits. */

  raw &= ~0x03;

  /* Formula RH = -6.0 + 125.0 * SRH / 2^16 from datasheet 6.1,
   * converted to integer fixed point (3 digits) representation.
   */

  return (((125000 >> 3) * raw) >> 13) - 6000;
}

/****************************************************************************
 * Name: sht21_read_values
 *
 * Description:
 *
 ****************************************************************************/

static int sht21_read_values(FAR struct sht21_dev_s *priv, FAR int *temp,
                             FAR int *rh)
{
  uint16_t temp16;
  uint16_t rh16;
  struct timespec ts;
  int ret;

  clock_gettime(CLOCK_REALTIME, &ts);

  /* Datasheet section 2.3: "To keep self heating below 0.1°C, SHT2x
   * should not be active for more than 10% of the time – e.g. maximum
   * two measurements per second at 12bit accuracy shall be made."
   * We limit to one measurement per second to keep arithmetic simple.
   */

  if (!priv->valid || has_time_passed(ts, priv->last_update, 1))
    {
      /* Read the raw temperature data */

      ret = sht21_read16(priv, SHT21_TRIG_T_MEAS_HM, &temp16);
      if (ret < 0)
        {
          snerr("ERROR: sht21_read16 failed: %d\n", ret);
          return ret;
        }

      /* Read the raw humidity data */

      ret = sht21_read16(priv, SHT21_TRIG_RH_MEAS_HM, &rh16);
      if (ret < 0)
        {
          snerr("ERROR: sht21_read16 failed: %d\n", ret);
          return ret;
        }

      add_sensor_randomness(ts.tv_nsec ^ ((int)temp16 << 16 | rh16));

      priv->temperature = sht21_temp_to_mcelsius(temp16);
      priv->humidity = sht21_rh_to_pcm(rh16);
      priv->last_update = ts;
      priv->valid = true;
    }

  *temp = priv->temperature;
  *rh = priv->humidity;
  return OK;
}

/****************************************************************************
 * Name: sht21_open
 *
 * Description:
 *   This function is called whenever the SHT2x device is opened.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sht21_open(FAR struct file *filep)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct sht21_dev_s *priv  = inode->i_private;
  int ret;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  nxsem_post(&priv->devsem);
  return OK;
}
#endif

/****************************************************************************
 * Name: sht21_close
 *
 * Description:
 *   This routine is called when the SHT2x device is closed.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sht21_close(FAR struct file *filep)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct sht21_dev_s *priv  = inode->i_private;
  int ret;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero and the driver has been unlinked,
   * then free memory now.
   */

  if (priv->crefs <= 0 && priv->unlinked)
    {
      nxsem_destroy(&priv->devsem);
      kmm_free(priv);
      return OK;
    }

  nxsem_post(&priv->devsem);
  return OK;
}
#endif

/****************************************************************************
 * Name: sht21_read
 ****************************************************************************/

static ssize_t sht21_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode       *inode  = filep->f_inode;
  FAR struct sht21_dev_s *priv   = inode->i_private;
  ssize_t                 length = 0;
  int temp;
  int rh;
  int ret;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked sensors. This allows
       * sensor use on hot swappable I2C bus.
       */

      nxsem_post(&priv->devsem);
      return -ENODEV;
    }
#endif

  ret = sht21_read_values(priv, &temp, &rh);
  if (ret < 0)
    {
      sht21_dbg("cannot read data: %d\n", ret);
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

  nxsem_post(&priv->devsem);
  return length;
}

/****************************************************************************
 * Name: sht21_write
 ****************************************************************************/

static ssize_t sht21_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: sht21_ioctl
 ****************************************************************************/

static int sht21_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct sht21_dev_s *priv = inode->i_private;
  int ret;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked sensors. This allows
       * sensor use on hot swappable I2C bus.
       */

      nxsem_post(&priv->devsem);
      return -ENODEV;
    }
#endif

  switch (cmd)
    {
      /* Soft reset the SHT2x, Arg: None */

      case SNIOC_RESET:
        {
          ret = sht21_softreset(priv);
          sht21_dbg("softreset ret: %d\n", ret);
        }
        break;

      case SNIOC_READ_RAW_DATA:
      case SNIOC_READ_CONVERT_DATA:
        {
          int temp;
          int rh;

          ret = sht21_read_values(priv, &temp, &rh);
          if (ret < 0)
            {
              sht21_dbg("cannot read data: %d\n", ret);
            }
          else
            {
              FAR struct sht21_conv_data_s *data =
                (FAR struct sht21_conv_data_s *)arg;

              data->temperature = temp;
              data->humidity = rh;
            }
        }
        break;

#ifdef CONFIG_SHT21_DEBUG
      case SNIOC_DUMP_REGS:
        {
          uint8_t userreg = 0;
          ret = sht21_read8(priv, SHT21_READ_USERREG, &userreg);
          sht21_dbg("read8 ret = %d, userreg = %d\n", ret, userreg);
        }
        break;
#endif

      default:
        sht21_dbg("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: sht21_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sht21_unlink(FAR struct inode *inode)
{
  FAR struct sht21_dev_s *priv;
  int ret;

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = (FAR struct sht21_dev_s *)inode->i_private;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Are there open references to the driver data structure? */

  if (priv->crefs <= 0)
    {
      nxsem_destroy(&priv->devsem);
      kmm_free(priv);
      return OK;
    }

  /* No... just mark the driver as unlinked and free the resources when
   * the last client closes their reference to the driver.
   */

  priv->unlinked = true;
  nxsem_post(&priv->devsem);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sht21_register
 *
 * Description:
 *   Register the SHT2x character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the SHT2x
 *   addr    - The I2C address of the SHT2x. The I2C address of both SHT20
 *             and SHT21 is always 0x40.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sht21_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t addr)
{
  FAR struct sht21_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == CONFIG_SHT21_ADDR);

  /* Initialize the device structure */

  priv = (FAR struct sht21_dev_s *)kmm_zalloc(sizeof(struct sht21_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  nxsem_init(&priv->devsem, 0, 1);

  /* Register the character driver */

  ret = register_driver(devpath, &g_sht21fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_SENSORS_SHT21 */
