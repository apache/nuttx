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

#include <errno.h>
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
 * Pre-processor Definitions
 ****************************************************************************/

#define RNG90_WAKE_I2C_ADDR       0x00
#define RNG90_WAKE_I2C_MAXFREQ    100000
#define RNG90_WAKE_RETRIES        3
#define RNG90_WAKE_DELAY_MS       5

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng90_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  mutex_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool rng90_wakeup(FAR struct rng90_dev_s *priv);
static bool rng90_sleep(FAR struct rng90_dev_s *priv);
static void rng90_send_wake_token(FAR struct rng90_dev_s *priv);
static void rng90_i2c_config(FAR struct i2c_config_s *config,
                             uint32_t frequency, uint8_t address);
static bool rng90_genrnd(FAR struct rng90_dev_s *priv, FAR uint8_t *buffer,
                          size_t buflen);

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

  /* The close path puts the device into explicit sleep, so wake it here. */

  nxmutex_lock(&priv->lock);
  if (!rng90_wakeup(priv))
    {
      crypterr("ERROR: Failed to wake rng90 device\n");
    }

  nxmutex_unlock(&priv->lock);

  return OK;
}

static int rng90_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rng90_dev_s *priv = inode->i_private;

  nxmutex_lock(&priv->lock);
  if (!rng90_sleep(priv))
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

      default:
        crypterr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static void rng90_i2c_config(FAR struct i2c_config_s *config,
                             uint32_t frequency, uint8_t address)
{
  config->frequency = frequency;
  config->address   = address;
  config->addrlen   = 7;
}

static bool rng90_is_wake_response(FAR const uint8_t *response)
{
  return response[0] == 0x04 && response[1] == 0x11 &&
         response[2] == 0x33 && response[3] == 0x43;
}

static void rng90_send_wake_token(FAR struct rng90_dev_s *priv)
{
  struct i2c_config_s config;
  uint8_t wake_token = 0x01;
  uint32_t wake_freq;

  /* Wake is an SDA-low pulse, not a normal transaction with an ACK. */

  wake_freq = MIN(RNG90_I2C_FREQ, RNG90_WAKE_I2C_MAXFREQ);
  rng90_i2c_config(&config, wake_freq, RNG90_WAKE_I2C_ADDR);
  i2c_write(priv->i2c, &config, &wake_token, 1);
  up_mdelay(RNG90_WAKE_DELAY_MS);
}

static bool rng90_wakeup(FAR struct rng90_dev_s *priv)
{
  struct i2c_config_s config;
  uint8_t response[4];
  int attempt;
  int ret;

  for (attempt = 0; attempt < RNG90_WAKE_RETRIES; attempt++)
    {
      rng90_send_wake_token(priv);

      rng90_i2c_config(&config, RNG90_I2C_FREQ, priv->addr);
      ret = i2c_read(priv->i2c, &config, response, sizeof(response));
      if (ret >= 0 && rng90_is_wake_response(response))
        {
          return true;
        }
    }

  return false;
}

static bool rng90_sleep(FAR struct rng90_dev_s *priv)
{
  struct i2c_config_s config;
  uint8_t word_address = 0x01;
  int ret;

  rng90_i2c_config(&config, RNG90_I2C_FREQ, priv->addr);

  /* Sleep sequence: device address + word address 0x01 + Stop condition */

  ret = i2c_write(priv->i2c, &config, &word_address, 1);
  return (ret >= 0);
}

static uint16_t rng90_crc16(FAR const uint8_t *data, size_t len)
{
  uint16_t crc = 0x0000;
  size_t i;

  for (i = 0; i < len; i++)
    {
      uint8_t byte = data[i];
      uint8_t shift;

      for (shift = 0x01; shift != 0; shift <<= 1)
        {
          bool dbit = (byte & shift) != 0;
          bool cbit = (crc & 0x8000) != 0;

          crc <<= 1;
          if (dbit != cbit)
            {
              crc ^= 0x8005;
            }
        }
    }

  return crc;
}

static bool rng90_genrnd(FAR struct rng90_dev_s *priv, FAR uint8_t *buffer,
                          size_t buflen)
{
  struct i2c_config_s config;
  uint8_t response[35];
  int ret;
  int attempt;

  /* Random command packet (datasheet section 6.2) */

  uint8_t pkt[28];
  uint16_t crc;

  pkt[0]  = 0x03;   /* word address: command */
  pkt[1]  = 27;     /* count */
  pkt[2]  = 0x16;   /* opcode: Random */
  pkt[3]  = 0x00;   /* param1 */
  pkt[4]  = 0x00;   /* param2 lo */
  pkt[5]  = 0x00;   /* param2 hi */

  /* data: 20 bytes, any value */

  memset(&pkt[6], 0x00, 20);

  /* CRC over count + packet (pkt[1] through pkt[25]) */

  crc     = rng90_crc16(&pkt[1], 25);
  pkt[26] = (uint8_t)(crc & 0xff);  /* CRC lo */
  pkt[27] = (uint8_t)(crc >> 8);    /* CRC hi */

  rng90_i2c_config(&config, RNG90_I2C_FREQ, priv->addr);

  for (attempt = 0; attempt < 2; attempt++)
    {
      if (attempt > 0)
        {
          rng90_wakeup(priv);
        }

      ret = i2c_write(priv->i2c, &config, pkt, sizeof(pkt));
      if (ret < 0)
        {
          continue;
        }

      /* Wait for command execution (datasheet section 5.6.2) */

      up_mdelay(72);

      /* Response: [count=35][32 random bytes][crc_lo][crc_hi] */

      ret = i2c_read(priv->i2c, &config, response, sizeof(response));
      if (ret >= 0)
        {
          memcpy(buffer, &response[1], MIN(buflen, 32u));
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rng90_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t addr)
{
  int ret;
  FAR struct rng90_dev_s *priv;

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
