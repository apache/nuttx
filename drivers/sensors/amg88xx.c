/****************************************************************************
 * drivers/sensors/amg88xx.c
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
#include <nuttx/mutex.h>
#include <nuttx/sensors/amg88xx.h>

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ioctl.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register bits definitions */

/* Power Control Register */

#define AMG88XX_POWER_CONTROL_REGISTER              (0x00)
#define AMG88XX_NORMAL_MODE_CMD                     (0x00)
#define AMG88XX_SLEEP_MODE_CMD                      (0x10)

/* Reset Register */

#define AMG88XX_RESET_REGISTER                      (0x01)
#define AMG88XX_FLAG_RESET_CMD                      (0x30)
#define AMG88XX_INITIAL_RESET_CMD                   (0x30)

/* Frame Rate Register */

#define AMG88XX_FRAME_RATE_REGISTER                 (0x02)
#define AMG88XX_FPS_BIT                             (0x00)
#define AMG88XX_1FPS                                (0x01)
#define AMG88XX_10FPS                               (0x00)

/* Interrupt Control Register */

#define AMG88XX_INTERRUPT_CONTROL_REGISTER          (0x03)
#define AMG88XX_INTMOD_BIT                          (0x01)
#define AMG88XX_INTMOD_ABS_MODE                     (0x01)
#define AMG88XX_INTMOD_DIFF_MODE                    (0x01)
#define AMG88XX_INTEN_BIT                           (0x01)
#define AMG88XX_INTEN_ACTIVE                        (0x01)
#define AMG88XX_INTEN_INATIVE                       (0x00)

/* Status Register */

#define AMG88XX_STATUS_REGISTER                     (0x04)
#define AMG88XX_OVF_IRS                             (0x02)
#define AMG88XX_INTF                                (0x01)

/* Status Clear Register */

#define AMG88XX_STATUS_CLEAR_REGISTER               (0x05)
#define AMG88XX_OVF_CLR                             (0x02)
#define AMG88XX_INT_CLR                             (0x01)

/* Average Register */

#define AMG88XX_AVERAGE_REGISTER                    (0x07)
#define AMG88XX_MAMOD_BIT                           (0x05)
#define AMG88XX_MAMOD_ENABLE                        (0x01)
#define AMG88XX_MAMOD_DISABLE                       (0x00)

/* Setting average mode
 *
 * There is an difference between
 * average register and average mode register
 * Check the datasheet for clarification
 */

#define AMG88XX_AVERAGE_MODE_REGISTER               (0x1F)
#define AMG88XX_AVEAGE_MODE_CMD_1                   (0x50)
#define AMG88XX_AVEAGE_MODE_CMD_2                   (0x45)
#define AMG88XX_AVEAGE_MODE_CMD_3                   (0x57)
#define AMG88XX_AVEAGE_MODE_CMD_4                   (0x00)

/* Interrupt Level Register */

#define AMG88XX_INT_HIGH_L                          (0x08)
#define AMG88XX_INT_HIGH_H                          (0x09)
#define AMG88XX_INT_LOW_L                           (0x0A)
#define AMG88XX_INT_LOW_H                           (0x0B)
#define AMG88XX_INT_HYST_L                          (0x0C)
#define AMG88XX_INT_HYST_H                          (0x0D)
#define AMG88XX_INT_MASK_H                          (0x0F)

/* Thermistor Register */

#define AMG88XX_THERMISTOR_L                        (0x0E)
#define AMG88XX_THERMISTOR_H                        (0x0F)

/* Interrupt Table Register */

#define AMG88XX_INTERRUPT_TABLE_REGISTER            (0x10)
#define AMG88XX_INTERRUPT_TABLE_LENGTH              (0x08)

/* Temperature Register */

#define AMG88XX_PIXEL_TABLE_BASE                    (0x80)
#define AMG88XX_PIXEL_TABLE_WIDTH                   (0x7F)
#define AMG88XX_PIXEL_WIDTH                         (0x02)
#define AMG88XX_PIXEL_MASK_H                        (0x0F)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct amg88xx_dev_s
{
  FAR struct i2c_master_s         *i2c;        /* I2C interface            */
  FAR struct amg88xx_config_s     *config;     /* sensor config structure  */
  mutex_t                          devlock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int amg88xx_i2c_write_addr(FAR struct amg88xx_dev_s *priv,
                                         uint8_t regaddr,
                                         uint8_t regval);

static inline int amg88xx_i2c_read_addr(FAR struct amg88xx_dev_s *priv,
                                        uint8_t regaddr,
                                        uint8_t *regval);

/* Character driver methods */

static int amg88xx_open(FAR struct file *filep);

static int amg88xx_close(FAR struct file *filep);

static ssize_t amg88xx_read(FAR struct file *filep,
                            FAR char *buffer,
                            size_t buflen);

static int amg88xx_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_amg88xx_fops =
{
  amg88xx_open,       /* open   */
  amg88xx_close,      /* close  */
  amg88xx_read,       /* read   */
  NULL,               /* write  */
  NULL,               /* seek   */
  amg88xx_ioctl       /* ioctl  */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: amg88xx_i2c_write_addr
 *
 * Description:
 *   Write to an amg88xx register transaction pattern:
 *   Write COMMAND at REGISTER
 *   Sensor Address / W - Register Address - Command
 ****************************************************************************/

static inline int amg88xx_i2c_write_addr(FAR struct amg88xx_dev_s *priv,
                                         uint8_t regaddr,
                                         uint8_t regval)
{
  /* Write to an amg88xx register transaction pattern:
   * Sensor Address - Register Address - Command
   */

  int                         ret;
  struct i2c_msg_s            msg;
  uint8_t                     data[2];

  data[0]             = regaddr;
  data[1]             = regval;

  msg.addr         = priv->config->addr;
  msg.frequency    = priv->config->speed;
  msg.flags        = I2C_M_NOSTOP;
  msg.buffer       = data;
  msg.length       = sizeof(data);

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);

  return ret;
}

/****************************************************************************
 * Name: amg88xx_i2c_read_addr
 *
 * Description:
 *   Read to an amg88xx register transaction pattern:
 *   Read COMMAND from REGISTER
 *   Sensor Address / W - Register Address - Sensor Address / R - Command
 ****************************************************************************/

static inline int amg88xx_i2c_read_addr(FAR struct amg88xx_dev_s *priv,
                                        uint8_t regaddr,
                                        FAR uint8_t *regval)
{
  int                         ret;
  struct i2c_msg_s            msg[2];

  msg[0].addr         = priv->config->addr;
  msg[0].frequency    = priv->config->speed;
  msg[0].flags        = I2C_M_NOSTOP;
  msg[0].buffer       = &regaddr;
  msg[0].length       = sizeof(regaddr);

  msg[1].addr         = priv->config->addr;
  msg[1].frequency    = priv->config->speed;
  msg[1].flags        = I2C_M_READ;
  msg[1].buffer       = regval;
  msg[1].length       = sizeof(*regval);

  ret = I2C_TRANSFER(priv->i2c, msg, 2);

  return ret;
}

/****************************************************************************
 * Name: amg88xx_open
 *
 * Description:
 *   Character device open call.
 *   Opening the device will not ensure that data can be read immediately.
 *   Some time need to pass before data is available in the data registers.
 *   By default the device start with 10 FPS, but still, without not delay
 *   between open and read the data will be full 0x00.
 *
 * Input Parameters:
 *   filep   - Pointer to struct file
 *
 * Returned Value:
 *   OK      - Sensor device was opened successfully.
 *
 ****************************************************************************/

static int amg88xx_open(FAR struct file *filep)
{
  FAR struct inode          *inode    = filep->f_inode;
  FAR struct amg88xx_dev_s  *priv     = inode->i_private;
  uint8_t                    sleep;
  int                        ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  amg88xx_i2c_read_addr(priv,
                       (uint8_t)AMG88XX_POWER_CONTROL_REGISTER,
                       (uint8_t *)&sleep);

  /* Set the sensor to normal operation mode */

  ret = amg88xx_i2c_write_addr(priv,
                              (uint8_t)AMG88XX_POWER_CONTROL_REGISTER,
                              (uint8_t)AMG88XX_NORMAL_MODE_CMD);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->devlock);
      return ret;
    }

  if (sleep == AMG88XX_SLEEP_MODE_CMD)
    {
      /* Hardcoded value specified in the datasheet */

      usleep(50000);

      /* If sensor was in sleep mode, we write initial reset register
       * same things as flag reset and read nonvolatile memory
       * which store adjustment value of sensor.
       */

      ret = amg88xx_i2c_write_addr(priv,
                                  (uint8_t)AMG88XX_RESET_REGISTER,
                                  (uint8_t)AMG88XX_INITIAL_RESET_CMD);
      if (ret < 0)
        {
          nxmutex_unlock(&priv->devlock);
          return ret;
        }

      /* Hardcoded value specified in the datasheet */

      usleep(2000);
    }

  ret = amg88xx_i2c_write_addr(priv,
                               (uint8_t)AMG88XX_RESET_REGISTER,
                               (uint8_t)AMG88XX_FLAG_RESET_CMD);

  if (ret < 0)
    {
      nxmutex_unlock(&priv->devlock);
      return ret;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: amg88xx_close
 *
 * Description:
 *   Character device close call.
 *
 * Input Parameters:
 *   filep - Pointer to struct file
 *
 * Returned Value:
 *   OK - Sensor device was closed successfully.
 *
 ****************************************************************************/

static int amg88xx_close(FAR struct file *filep)
{
  FAR struct inode          *inode    = filep->f_inode;
  FAR struct amg88xx_dev_s  *priv     = inode->i_private;
  int                        ret;

  DEBUGASSERT(priv != NULL);

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the sensor to sleep operation mode */

  ret = amg88xx_i2c_write_addr(priv,
                               (uint8_t)AMG88XX_POWER_CONTROL_REGISTER,
                               (uint8_t)AMG88XX_SLEEP_MODE_CMD);

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: amg88xx_read
 *
 * Description:
 *   Character device read call. Blocks until requested buffer size is full.
 *
 * Input Parameters:
 *   filep - Pointer to struct file
 *   buffer - Pointer to user buffer
 *   buflen - Size of user buffer in bytes
 *
 * Returned Value:
 *   Returns the number of bytes written to the buffer.
 *   -EINVAL - Supplied buffer length invalid
 *
 ****************************************************************************/

static ssize_t amg88xx_read(FAR struct file *filep,
                            FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode          *inode    = filep->f_inode;
  FAR struct amg88xx_dev_s  *priv     = inode->i_private;
  uint8_t                    regval   = AMG88XX_PIXEL_TABLE_BASE;
  int                        ret;

  DEBUGASSERT(priv != NULL);

  if (buffer == NULL)
    {
      snerr("ERROR: buffer should not be NULL.\n");
      return -EINVAL;
    }

  if (buflen != AMG88XX_PIXELS_ARRAY_LENGTH)
    {
      snerr("ERROR: buflen is not sufficient to store sensor data.\n");
      return -EINVAL;
    }

  struct i2c_msg_s msg[2];

  msg[0].addr         = priv->config->addr;
  msg[0].frequency    = priv->config->speed;
  msg[0].flags        = I2C_M_NOSTOP;
  msg[0].buffer       = &regval;
  msg[0].length       = 1;

  msg[1].addr         = priv->config->addr;
  msg[1].frequency    = priv->config->speed;
  msg[1].flags        = I2C_M_READ;
  msg[1].buffer       = (uint8_t *)buffer;
  msg[1].length       = buflen;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);

  return ret;
}

static int amg88xx_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct inode         *inode    = filep->f_inode;
  FAR struct amg88xx_dev_s *priv     = inode->i_private;
  int                       ret      = -EINVAL;
  uint8_t                   regval   = 0;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
    case SNIOC_SET_OPERATIONAL_MODE:
      switch (*(amg88xx_operation_mode_e *)arg)
        {
        case op_mode_normal:
          ret = amg88xx_i2c_write_addr(
                priv,
                (uint8_t)AMG88XX_POWER_CONTROL_REGISTER,
                (uint8_t)AMG88XX_NORMAL_MODE_CMD);
          break;
        case op_mode_sleep:
          ret = amg88xx_i2c_write_addr(
                priv,
                (uint8_t)AMG88XX_POWER_CONTROL_REGISTER,
                (uint8_t)AMG88XX_SLEEP_MODE_CMD);
          break;
        default:
          ret = -EINVAL;
          break;
        }
    break;

    case SNIOC_SET_FRAMERATE:
      switch (*(amg88xx_fps_e *)arg)
        {
        case fps_one:
          regval = (AMG88XX_1FPS << AMG88XX_FPS_BIT);
          ret = amg88xx_i2c_write_addr(priv,
                                       (uint8_t)AMG88XX_FRAME_RATE_REGISTER,
                                       regval);
          break;
        case fps_ten:
          regval = (AMG88XX_10FPS << AMG88XX_FPS_BIT);
          ret = amg88xx_i2c_write_addr(priv,
                                       (uint8_t)AMG88XX_FRAME_RATE_REGISTER,
                                       regval);
          break;
        default:
          ret = -EINVAL;
          break;
        }
    break;

    /* Enabling and disabling the moving average requires following
     * a precedure described in the i2c communication interface manual
     */

    case SNIOC_SET_MOVING_AVG:
      ret = amg88xx_i2c_write_addr(
            priv,
            (uint8_t)AMG88XX_AVERAGE_MODE_REGISTER,
            (uint8_t)AMG88XX_AVEAGE_MODE_CMD_1);
      if (ret < 0)
        {
          return ret;
        }

      ret = amg88xx_i2c_write_addr(
            priv,
            (uint8_t)AMG88XX_AVERAGE_MODE_REGISTER,
            (uint8_t)AMG88XX_AVEAGE_MODE_CMD_2);
      if (ret < 0)
        {
          return ret;
        }

      ret = amg88xx_i2c_write_addr(
            priv,
            (uint8_t)AMG88XX_AVERAGE_MODE_REGISTER,
            (uint8_t)AMG88XX_AVEAGE_MODE_CMD_3);
      if (ret < 0)
        {
          return ret;
        }

      regval = (bool)arg                                    ?
               (AMG88XX_MAMOD_ENABLE << AMG88XX_MAMOD_BIT)  :
               (AMG88XX_MAMOD_DISABLE << AMG88XX_MAMOD_BIT) ;

      ret = amg88xx_i2c_write_addr(
            priv,
            (uint8_t)AMG88XX_AVERAGE_REGISTER,
            regval);

      if (ret < 0)
        {
          return ret;
        }

      ret = amg88xx_i2c_write_addr(
            priv,
            (uint8_t)AMG88XX_AVERAGE_MODE_REGISTER,
            (uint8_t)AMG88XX_AVEAGE_MODE_CMD_4);
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: amg88xx_register
 *
 * Description:
 *   Register the amg88xx character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/irm0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             AMG88xx
 *   config  - Initial configuration of the sensor
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int amg88xx_register(FAR const char *devpath,
                     FAR struct i2c_master_s *i2c,
                     FAR struct amg88xx_config_s *config)
{
  FAR struct amg88xx_dev_s *priv;
  int                       ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(config != NULL);

  /* Initialize the AMG88xx device structure */

  priv = (FAR struct amg88xx_dev_s *)
    kmm_malloc(sizeof(struct amg88xx_dev_s));

  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->config = config;
  nxmutex_init(&priv->devlock);

  /* Register the character driver */

  ret = register_driver(devpath, &g_amg88xx_fops, 0666, priv);

  if (ret < 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      snerr("ERROR: Failed to register driver: %d\n", ret);
    }

  return ret;
}
