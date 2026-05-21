/****************************************************************************
 * drivers/crypto/rng90.c
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

#include <string.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/mutex.h>
#include <nuttx/crypto/rng90.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng90_dev_s
{
  FAR struct i2c_master_s *i2c;  /* handle do barramento I2C */
  uint8_t addr;                   /* endereço I2C do chip */
  mutex_t lock;                   /* serializa chamadas concorrentes */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool rng90_wakeup(FAR struct rng90_dev_s *priv);
static bool rng90_sleep(FAR struct rng90_dev_s *priv);
static bool rng90_genrnd(FAR struct rng90_dev_s *priv, FAR uint8_t *buffer,
                          size_t buflen);
static int rng90_selftest(FAR struct rng90_dev_s *priv, uint8_t mode,
                          FAR uint8_t *result);

/* Character driver methods */

static int rng90_open(FAR struct file *filep);
static int rng90_close(FAR struct file *filep);
static ssize_t rng90_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static int rng90_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_rng90_fops =
{
  rng90_open, rng90_close, rng90_read, NULL,
  NULL,       rng90_ioctl, NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rng90_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rng90_dev_s *priv = inode->i_private;
  bool ret;

  /* In close event the IC will be put to sleep, therefore wake it up */
  /* Don't worry run wakeup routine when the device is already awake */

  nxmutex_lock(&priv->lock);
  ret = rng90_wakeup(priv);
  nxmutex_unlock(&priv->lock);

  return ret ? OK : ERROR;
}

static int rng90_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rng90_dev_s *priv = inode->i_private;

  nxmutex_lock(&priv->lock);
  if (rng90_sleep(priv) == false)
  {
    crypterr("ERROR: Failed to put rng90 device to sleep\n");
  }

  nxmutex_unlock(&priv->lock);

  return OK;
}

static ssize_t rng90_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rng90_dev_s *priv = inode->i_private;
  bool ret;

  nxmutex_lock(&priv->lock);
  ret = rng90_genrnd(priv, (FAR uint8_t *)buffer, buflen);
  nxmutex_unlock(&priv->lock);

  /* rng90 always returns 32 bytes */

  return ret ? (ssize_t)MIN(buflen, 32u) : -EIO;
}

static int rng90_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rng90_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret != 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case RNG90_IOC_WAKEUP:
        ret = rng90_wakeup(priv) ? OK : ERROR;
        break;

      case RNG90_IOC_SLEEP:
        ret = rng90_sleep(priv) ? OK : ERROR;
        break;

      case RNG90_IOC_GENRND:
        ret = rng90_genrnd(priv, (FAR uint8_t *)arg, 32) ? OK : ERROR;
        break;

      case RNG90_IOC_SELFTEST:
        {
          FAR struct rng90_selftest_s *st = (FAR struct rng90_selftest_s *)arg;
          ret = rng90_selftest(priv, st->mode, &st->result);
        }
        break;

      default:
        crypterr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

    nxmutex_unlock(&priv->lock);
    return ret;
}

static bool rng90_wakeup(FAR struct rng90_dev_s *priv)
{
  struct i2c_config_s config;
  uint8_t response[4];
  int ret;

  config.frequency = RNG90_I2C_FREQ;
  config.address   = priv->addr;
  config.addrlen   = 7;

  i2c_write(priv->i2c, &config, NULL, 0);
  up_mdelay(1);

  ret = i2c_read(priv->i2c, &config, response, 4);
  if (ret < 0)
    {
      return false;
    }

  return (response[1] == 0x11);
}

static bool rng90_sleep(FAR struct rng90_dev_s *priv)
{
  struct i2c_config_s config;
  uint8_t word_address = 0x01;
  int ret;

  config.frequency = RNG90_I2C_FREQ;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Sleep sequence: device address + word address 0x01 + Stop condition */

  ret = i2c_write(priv->i2c, &config, &word_address, 1);
  return (ret >= 0);
}

static uint16_t rng90_crc16(FAR const uint8_t *data, size_t len)
{
  uint16_t crc = 0x0000;
  size_t i;
  int bit;

  for (i = 0; i < len; i++)
    {
      uint8_t byte = data[i];
      for (bit = 0; bit < 8; bit++)
        {
          uint8_t mix = (crc ^ byte) & 0x01;
          crc >>= 1;
          if (mix)
            {
              crc ^= 0x8005;
            }

          byte >>= 1;
        }
    }

  return crc;
}

static bool rng90_genrnd(FAR struct rng90_dev_s *priv, FAR uint8_t *buffer,
                          size_t buflen)
{
  struct i2c_config_s config;
  int ret;

  /* Random command packet (datasheet section 6.2):
   *
   * I2C write frame:
   *   [word_addr=0x03][count=27][opcode=0x16][param1=0x00]
   *   [param2_lo=0x00][param2_hi=0x00][data=20 bytes][crc_lo][crc_hi]
   *
   * count = 1(count) + 1(opcode) + 1(param1) + 2(param2)
   *       + 20(data) + 2(crc) = 27
   *
   * CRC is calculated over count + packet bytes (not word address).
   */

  uint8_t pkt[28];
  uint16_t crc;

  pkt[0]  = 0x03;   /* word address: command */
  pkt[1]  = 27;     /* count */
  pkt[2]  = 0x16;   /* opcode: Random */
  pkt[3]  = 0x00;   /* param1 */
  pkt[4]  = 0x00;   /* param2 lo */
  pkt[5]  = 0x00;   /* param2 hi */
  memset(&pkt[6], 0x00, 20); /* data: 20 bytes, any value */

  /* CRC over count + packet (pkt[1] through pkt[25]) */

  crc     = rng90_crc16(&pkt[1], 25);
  pkt[26] = (uint8_t)(crc & 0xff);  /* CRC lo */
  pkt[27] = (uint8_t)(crc >> 8);    /* CRC hi */

  config.frequency = RNG90_I2C_FREQ;
  config.address   = priv->addr;
  config.addrlen   = 7;

  ret = i2c_write(priv->i2c, &config, pkt, sizeof(pkt));
  if (ret < 0)
    {
      return false;
    }

  /* Wait for command execution (datasheet section 5.6.2):
   * First execution: 72ms max (includes self-test)
   * Subsequent:      25ms max
   * Using max values to guarantee completion without polling.
   */

  up_mdelay(72);

  /* Response: [count=35][32 random bytes][crc_lo][crc_hi] */

  uint8_t response[35];
  ret = i2c_read(priv->i2c, &config, response, sizeof(response));
  if (ret < 0)
    {
      return false;
    }

  /* Copy up to 32 bytes into caller's buffer */

  memcpy(buffer, &response[1], MIN(buflen, 32u));
  return true;
}

static int rng90_selftest(FAR struct rng90_dev_s *priv, uint8_t mode,
                          FAR uint8_t *result)
{
  struct i2c_config_s config;
  uint8_t pkt[8];
  uint8_t response[4];
  uint16_t crc;
  int ret;

  /* SelfTest command packet (datasheet section 6.4):
   *
   * I2C write frame:
   *   [word_addr=0x03][count=7][opcode=0x77][param1=mode]
   *   [param2_lo=0x00][param2_hi=0x00][crc_lo][crc_hi]
   *
   * count = 1(count) + 1(opcode) + 1(param1) + 2(param2) + 2(crc) = 7
   * No data bytes for SelfTest.
   *
   * Execution times (datasheet section 5.6.2):
   *   mode=0x00 (Status):       0.4ms max
   *   mode=0x01 (DRBG):        31.8ms max
   *   mode=0x20 (SHA256):      14.5ms max
   * Using 32ms to cover all modes safely.
   */

  pkt[0] = 0x03;    /* word address: command */
  pkt[1] = 7;       /* count */
  pkt[2] = 0x77;    /* opcode: SelfTest */
  pkt[3] = mode;    /* param1: test mode */
  pkt[4] = 0x00;    /* param2 lo */
  pkt[5] = 0x00;    /* param2 hi */

  /* CRC over count + packet (pkt[1] through pkt[5]) */

  crc    = rng90_crc16(&pkt[1], 5);
  pkt[6] = (uint8_t)(crc & 0xff);  /* CRC lo */
  pkt[7] = (uint8_t)(crc >> 8);    /* CRC hi */

  config.frequency = RNG90_I2C_FREQ;
  config.address   = priv->addr;
  config.addrlen   = 7;

  ret = i2c_write(priv->i2c, &config, pkt, sizeof(pkt));
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(32);

  /* Response: [count=4][status_byte][crc_lo][crc_hi] */

  ret = i2c_read(priv->i2c, &config, response, sizeof(response));
  if (ret < 0)
    {
      return ret;
    }

  /* Return the status byte to the caller */

  *result = response[1];
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rng90_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t addr)
{
  int ret;
  FAR struct rng90_dev_s *priv;
  bool wake_successful;

  /* Sanity check */
  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(i2c != NULL);

  /* Initialize the device's structure */

  priv = (FAR struct rng90_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      crypterr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = addr;
  nxmutex_init(&priv->lock);

  /* Check rng90 availability */

  wake_successful = rng90_wakeup(priv);
  if (!wake_successful)
    {
      crypterr("ERROR: Failed to wake rng90 device\n");
      kmm_free(priv);
      return -ENODEV;
    }

  /* Register the character driver */
  ret = register_driver(devpath, &g_rng90_fops, 0666, priv);
  if (ret < 0)
    {
      crypterr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return ret;
    }
  return OK;
}