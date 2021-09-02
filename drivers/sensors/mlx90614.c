/****************************************************************************
 * drivers/sensors/mlx90614.c
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

/* Character driver for the Melexis MLX90614 Infrared Thermometer */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdlib.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/mlx90614.h>
#include <nuttx/random.h>

#ifdef CONFIG_MLX90614_CRC
#include <crc8.h>
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MLX90614)

/****************************************************************************
 * Pre-process Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mlx90614_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     mlx90614_read_word(FAR struct mlx90614_dev_s *priv,
                                  uint8_t cmd, FAR uint16_t *regval);
static int     mlx90614_write_word(FAR struct mlx90614_dev_s *priv,
                                   uint8_t cmd, uint16_t regval);

/* Character driver methods */

static ssize_t mlx90614_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t mlx90614_write(FAR struct file *filep,
                  FAR const char *buffer, size_t buflen);
static int     mlx90614_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_mlx90614_fops =
{
  NULL,            /* open */
  NULL,            /* close */
  mlx90614_read,   /* read */
  mlx90614_write,  /* write */
  NULL,            /* seek */
  mlx90614_ioctl,  /* ioctl */
  NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL            /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mlx90614_read_word
 *
 * Description:
 *   Read 16-bit register
 *
 ****************************************************************************/

static int mlx90614_read_word(FAR struct mlx90614_dev_s *priv, uint8_t cmd,
                              FAR uint16_t *regval)
{
  struct i2c_config_s config;
#ifdef CONFIG_MLX90614_CRC
  uint8_t checkcrc[6];
  uint8_t *buffer;
#else
  uint8_t buffer[3];
#endif
  uint8_t crc;
  int ret;

#ifdef CONFIG_MLX90614_CRC
  /* Recreate the I2C byte sequence to check the CRC PEC field */

  checkcrc[0] = I2C_WRITEADDR8(priv->addr);
  checkcrc[1] = cmd;
  checkcrc[2] = I2C_READADDR8(priv->addr);

  /* Point "buffer" to checkcrc[3] to fill it with received bytes */

  buffer = (uint8_t *) &checkcrc[3];
#endif

  /* Set up the I2C configuration */

  config.frequency = 100000;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the Command and read 16-bits + PEC from the device */

  ret = i2c_writeread(priv->i2c, &config, (FAR const uint8_t *) &cmd, 1,
                      buffer, 3);
  if (ret < 0)
    {
      snerr ("i2c_writeread failed: %d\n", ret);
      return ret;
    }

  /* Copy the content of the buffer to the location of the uint16_t pointer */

  *regval = (uint16_t)(buffer[0] | (buffer[1] << 8));

  sninfo("value[0]: %02x | value[1]: %02x | value[2]: %02x | ret: %d\n",
         buffer[0], buffer[1], buffer[2], ret);

#ifdef CONFIG_MLX90614_CRC
  crc = crc8ccitt(checkcrc, 5);
  sninfo("Calculated CRC: 0x%02X\n", crc);

  if (crc != buffer[2])
    {
      snerr("Invalid CRC! Expected: 0x%02X read: 0x%02X\n", crc, buffer[2]);
      return -EFAULT;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: mlx90614_write_word
 *
 * Description:
 *   Write to a 16-bit register
 *
 ****************************************************************************/

static int mlx90614_write_word(FAR struct mlx90614_dev_s *priv, uint8_t cmd,
                               uint16_t regval)
{
  struct i2c_config_s config;
  uint8_t pkgcrc[5];
  uint8_t *buffer;
  uint8_t crc;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = 100000;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* First write 0x00 0x00 to erase the EEPROM */

  pkgcrc[0] = I2C_WRITEADDR8(priv->addr);
  pkgcrc[1] = cmd;
  pkgcrc[2] = 0x00;
  pkgcrc[3] = 0x00;

  /* Generate the CRC to put in the PEC field */

  crc = crc8ccitt(pkgcrc, 4);
  sninfo("Generated CRC: 0x%02X\n", crc);

  pkgcrc[4] = crc;

  buffer = &(pkgcrc[1]);

  /* Write the Command + 2 Bytes + PEC to device */

  ret = i2c_write(priv->i2c, &config, (FAR const uint8_t *) buffer, 4);
  if (ret < 0)
    {
      snerr ("i2c_writeread failed: %d\n", ret);
      return ret;
    }

  /* Wait the EEPROM erase */

  nxsig_usleep(10 * 1000);

  /* Create the I2C command that will be sent to device */

  pkgcrc[0] = I2C_WRITEADDR8(priv->addr);
  pkgcrc[1] = cmd;
  pkgcrc[2] = (regval & 0xff);
  pkgcrc[3] = (regval & 0xff00) >> 8;

  /* Generate the CRC to put in the PEC field */

  crc = crc8ccitt(pkgcrc, 4);
  sninfo("Generated CRC: 0x%02X\n", crc);

  pkgcrc[4] = crc;

  /* Write the Command + 2 Bytes + PEC to device */

  ret = i2c_write(priv->i2c, &config, (FAR const uint8_t *) buffer, 4);
  if (ret < 0)
    {
      snerr ("i2c_writeread failed: %d\n", ret);
      return ret;
    }

  sninfo("New address 0x%02x stored correctly!\n", regval);

  return OK;
}

/****************************************************************************
 * Name: mlx90614_read
 ****************************************************************************/

static ssize_t mlx90614_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  FAR struct inode *inode;
  FAR struct mlx90614_dev_s *priv;
  FAR struct mlx90614_temp_s *temp = (FAR struct mlx90614_temp_s *)buffer;
  uint8_t cmd;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct mlx90614_dev_s *)inode->i_private;

  /* Check if the user is reading the right size */

  if (buflen != sizeof(struct mlx90614_temp_s))
    {
      snerr("ERROR: You need to read 2 bytes from this sensor!\n");
      return -EINVAL;
    }

  /* Read Ambient Temperature sensor */

  cmd = MLX90614_CMD_RAM_ACCESS | MLX90614_TA;

  ret = mlx90614_read_word(priv, cmd, &(temp->ta));
  if (ret < 0)
    {
      snerr("ERROR: Error reading thermometer sensor!\n");
      return ret;
    }

  /* Read Object1 Temperature sensor */

  cmd = MLX90614_CMD_RAM_ACCESS | MLX90614_TOBJ1;

  ret = mlx90614_read_word(priv, cmd, &(temp->tobj1));
  if (ret < 0)
    {
      snerr("ERROR: Error reading thermometer sensor!\n");
      return ret;
    }

  /* Read Object2 Temperature sensor - some sensor doesn't have it */

  cmd = MLX90614_CMD_RAM_ACCESS | MLX90614_TOBJ2;

  ret = mlx90614_read_word(priv, cmd, &(temp->tobj2));
  if (ret < 0)
    {
      snerr("ERROR: Error reading thermometer sensor!\n");
      return ret;
    }

  return buflen;
}

/****************************************************************************
 * Name: mlx90614_write
 ****************************************************************************/

static ssize_t mlx90614_write(FAR struct file *filep,
                              FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: mlx90614_ioctl
 ****************************************************************************/

static int mlx90614_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mlx90614_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      /* Change the device to use a new i2c/smbus address */

      case SNIOC_CHANGE_SMBUSADDR:
        {
          FAR struct mlx90614_dev_s newdev;
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          uint16_t newaddr;
          uint8_t smbcmd;

          /* EEPROM reflash only happen when sent at I2C address 0x00 */

          newdev.i2c  = priv->i2c;
          newdev.addr = 0x00;

          /* The I2C address is put in the high byte position */

          newaddr = (uint16_t) *ptr;
          sninfo("Setting new address: 0x%02x\n", newaddr);

          smbcmd = MLX90614_CMD_EEPROM_ACCESS | MLX90614_SMBUS_ADDR;

          ret = mlx90614_write_word(&newdev, smbcmd, newaddr);
          if (ret < 0)
            {
              snerr("ERROR: Failed to change the I2C/SMBus address!\n");
            }
        }
        break;

      default:
        snerr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mlx90614_register
 *
 * Description:
 *   Register the MLX90614 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/therm0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             MLX90614
 *   addr    - The I2C address of the MLX90614.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mlx90614_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t addr)
{
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the MLX90614 device structure */

  FAR struct mlx90614_dev_s *priv =
    (FAR struct mlx90614_dev_s *)kmm_malloc(sizeof(struct mlx90614_dev_s));

  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  /* Register the character driver */

  ret = register_driver(devpath, &g_mlx90614_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_MLX90614 */
