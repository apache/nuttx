/****************************************************************************
 * drivers/power/supply/husb238.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/power/husb238.h>
#include <nuttx/power/power_ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HUSB238_REG_PD_STATUS0   0x00
#define HUSB238_REG_PD_STATUS1   0x01
#define HUSB238_REG_SRC_PDO_5V   0x02
#define HUSB238_REG_SRC_PDO_9V   0x03
#define HUSB238_REG_SRC_PDO_12V  0x04
#define HUSB238_REG_SRC_PDO_15V  0x05
#define HUSB238_REG_SRC_PDO_18V  0x06
#define HUSB238_REG_SRC_PDO_20V  0x07
#define HUSB238_REG_SRC_PDO      0x08
#define HUSB238_REG_GO_COMMAND   0x09

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct husb238_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* HUSB238 I2C address */
  int freq;                     /* HUSB238 Frequency */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int husb238_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_husb238fops =
{
  NULL,         /* open */
  NULL,         /* close */
  NULL,         /* read */
  NULL,         /* write */
  NULL,         /* seek */
  husb238_ioctl /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int husb238_getreg8(FAR struct husb238_dev_s *priv, uint8_t regaddr)
{
  struct i2c_config_s config;
  uint8_t regval = 0;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  /* Register to read */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      pwrerr("ERROR: i2c_write failed: %s (%d)\n", strerror(-ret), ret);
      return ret;
    }

  /* Read register */

  ret = i2c_read(priv->i2c, &config, (FAR uint8_t *)&regval, 1);
  if (ret < 0)
    {
      pwrerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  return regval;
}

static int husb238_putreg8(FAR struct husb238_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t data[2];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address = priv->addr;
  config.addrlen = 7;

  data[0] = regaddr;
  data[1] = regval;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (FAR uint8_t *)&data, 2);
  if (ret < 0)
    {
      pwrerr("ERROR: i2c_write failed: %s (%d)\n", strerror(-ret), ret);
      return ret;
    }

  return OK;
}

static int husb238_get_pdstatus_0(FAR struct husb238_dev_s *priv,
                                  FAR struct husb238_pdstatus_0_s *status)
{
  int ret;
  uint8_t regval;

  ret = husb238_getreg8(priv, HUSB238_REG_PD_STATUS0);
  if (ret < 0)
    {
      return ret;
    }

  regval = ret;
  status->src_voltage = (regval >> 4) & 0x0f;
  status->src_current = regval & 0x0f;

  return OK;
}

static int husb238_get_pdstatus_1(FAR struct husb238_dev_s *priv,
                                  FAR struct husb238_pdstatus_1_s *status)
{
  int ret;
  uint8_t regval;

  ret = husb238_getreg8(priv, HUSB238_REG_PD_STATUS1);
  if (ret < 0)
    {
      return ret;
    }

  regval = ret;
  status->cc_dir = (regval >> 7) & 0x1;
  status->attached = (regval >> 6) & 0x1;
  status->pd_response = (regval >> 3) & 0x7;
  status->voltage_5v = (regval >> 2) & 0x1;
  status->current_5v = regval & 0x3;

  return OK;
}

static int husb238_get_src_pdos(FAR struct husb238_dev_s *priv,
                                FAR struct husb238_src_pdos_s *pdos)
{
  int ret;
  uint8_t pdo;

  /* Read the SRC_PDO_5V register */

  ret = husb238_getreg8(priv, HUSB238_REG_SRC_PDO_5V);
  if (ret < 0)
    {
      return ret;
    }

  pdo = ret;
  pdos->detected_5v = (pdo >> 7) & 0x1;
  pdos->current_5v = pdo & 0x7;

  /* Read the SRC_PDO_9V register */

  ret = husb238_getreg8(priv, HUSB238_REG_SRC_PDO_9V);
  if (ret < 0)
    {
      return ret;
    }

  pdo = ret;
  pdos->detected_9v = (pdo >> 7) & 0x1;
  pdos->current_9v = pdo & 0x7;

  /* Read the SRC_PDO_12V register */

  ret = husb238_getreg8(priv, HUSB238_REG_SRC_PDO_12V);
  if (ret < 0)
    {
      return ret;
    }

  pdo = ret;
  pdos->detected_12v = (pdo >> 7) & 0x1;
  pdos->current_12v = pdo & 0x7;

  /* Read the SRC_PDO_15V register */

  ret = husb238_getreg8(priv, HUSB238_REG_SRC_PDO_15V);
  if (ret < 0)
    {
      return ret;
    }

  pdo = ret;
  pdos->detected_15v = (pdo >> 7) & 0x1;
  pdos->current_15v = pdo & 0x7;

  /* Read the SRC_PDO_18V register */

  ret = husb238_getreg8(priv, HUSB238_REG_SRC_PDO_18V);
  if (ret < 0)
    {
      return ret;
    }

  pdo = ret;
  pdos->detected_18v = (pdo >> 7) & 0x1;
  pdos->current_18v = pdo & 0x7;

  /* Read the SRC_PDO_20V register */

  ret = husb238_getreg8(priv, HUSB238_REG_SRC_PDO_20V);
  if (ret < 0)
    {
      return ret;
    }

  pdo = ret;
  pdos->detected_20v = (pdo >> 7) & 0x1;
  pdos->current_20v = pdo & 0x7;

  return OK;
}

static int husb238_get_selected_pdo(FAR struct husb238_dev_s *priv,
                                    uint8_t *pdo)
{
  int ret;

  ret = husb238_getreg8(priv, HUSB238_REG_SRC_PDO);
  if (ret < 0)
    {
      return ret;
    }

  *pdo = (ret >> 4) & 0x0f;

  return OK;
}

static int husb238_set_selected_pdo(FAR struct husb238_dev_s *priv,
                                    uint8_t pdo_select)
{
  int ret;

  if (pdo_select != HUSB238_SRC_PDO_NONE &&
      pdo_select != HUSB238_SRC_PDO_5V &&
      pdo_select != HUSB238_SRC_PDO_9V &&
      pdo_select != HUSB238_SRC_PDO_12V &&
      pdo_select != HUSB238_SRC_PDO_15V &&
      pdo_select != HUSB238_SRC_PDO_18V &&
      pdo_select != HUSB238_SRC_PDO_20V)
    {
      syslog(LOG_ERR, "Unsupported PDO_SELECT specified\n");
      return -EINVAL;
    }

  ret = husb238_putreg8(priv, HUSB238_REG_SRC_PDO, pdo_select << 4);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

static int husb238_send_command(FAR struct husb238_dev_s *priv,
                                uint8_t command)
{
  int ret;

  if (command != HUSB238_CMD_REQUEST_PDO &&
      command != HUSB238_CMD_SEND_GETSRCCAP &&
      command != HUSB238_CMD_SEND_HARDRESET)
    {
      syslog(LOG_ERR, "Unsupported command specified\n");
      return -EINVAL;
    }

  ret = husb238_putreg8(priv, HUSB238_REG_GO_COMMAND, command);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

static int husb238_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct husb238_dev_s *priv = inode->i_private;

  int ret = OK;

  switch (cmd)
    {
      /* Read and parse the contents of the PD_STATUS0 register */

      case PWRIOC_HUSB238_GET_PDSTATUS_0:
        {
          FAR struct husb238_pdstatus_0_s *pdstatus_0 =
            (FAR struct husb238_pdstatus_0_s *)((uintptr_t)arg);
          ret = husb238_get_pdstatus_0(priv, pdstatus_0);
          break;
        }

      /* Read and parse the contents of the PD_STATUS1 register */

      case PWRIOC_HUSB238_GET_PDSTATUS_1:
        {
          FAR struct husb238_pdstatus_1_s *pdstatus_1 =
            (FAR struct husb238_pdstatus_1_s *)((uintptr_t)arg);
          ret = husb238_get_pdstatus_1(priv, pdstatus_1);
          break;
        }

      /* Read and parse the contents of all the SRC_PDO_* registers */

      case PWRIOC_HUSB238_GET_SRC_PDOS:
        {
          FAR struct husb238_src_pdos_s *src_pdos =
            (FAR struct husb238_src_pdos_s *)((uintptr_t)arg);
          ret = husb238_get_src_pdos(priv, src_pdos);
          break;
        }

      /* Read and parse the contents of the SRC_PDO register */

      case PWRIOC_HUSB238_GET_SELECTED_PDO:
        {
          FAR uint8_t *pdo = (FAR uint8_t *)((uintptr_t)arg);
          ret = husb238_get_selected_pdo(priv, pdo);
          break;
        }

      /* Write the contents of the SRC_PDO register */

      case PWRIOC_HUSB238_SET_SELECTED_PDO:
        {
          FAR uint8_t *pdo = (FAR uint8_t *)((uintptr_t)arg);
          ret = husb238_set_selected_pdo(priv, *pdo);
          break;
        }

      /* Write the contents of the GO_COMMAND register */

      case PWRIOC_HUSB238_SEND_COMMAND:
        {
          FAR uint8_t *command = (FAR uint8_t *)((uintptr_t)arg);
          ret = husb238_send_command(priv, *command);
          break;
        }

      /* Command was not recognized */

      default:
        {
          pwrerr("ERROR: Unrecognized cmd: %d\n", cmd);
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: husb238_register
 *
 * Description:
 *   Register the HUSB238 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/usbpd0"
 *   i2c     - An instance of the I2C interface to use.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int husb238_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct husb238_dev_s *priv;
  int ret;

  /* Initialize the HUSB238 device structure */

  priv = kmm_malloc(sizeof(struct husb238_dev_s));
  if (priv == NULL)
    {
      pwrerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = 0x08;
  priv->freq = CONFIG_USBPD_HUSB238_I2C_FREQUENCY;

  /* Register the character driver */

  ret = register_driver(devpath, &g_husb238fops, 0666, priv);
  if (ret < 0)
    {
      pwrerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
