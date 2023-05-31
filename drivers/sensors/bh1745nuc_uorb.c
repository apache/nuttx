/****************************************************************************
 * drivers/sensors/bh1745nuc_uorb.c
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
#include <debug.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/random.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/bh1745nuc.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Registers */

#define BH1745NUC_SYSTEM_CONTROL    (0x40)
#define BH1745NUC_MODE_CONTROL1     (0x41)
#define BH1745NUC_MODE_CONTROL2     (0x42)
#define BH1745NUC_MODE_CONTROL3     (0x44)
#define BH1745NUC_RED_DATA_LSB      (0x50)
#define BH1745NUC_RED_DATA_MSB      (0x51)
#define BH1745NUC_GREEN_DATA_LSB    (0x52)
#define BH1745NUC_GREEN_DATA_MSB    (0x53)
#define BH1745NUC_BLUE_DATA_LSB     (0x54)
#define BH1745NUC_BLUE_DATA_MSB     (0x55)
#define BH1745NUC_CLEAR_DATA_LSB    (0x56)
#define BH1745NUC_CLEAR_DATA_MSB    (0x57)
#define BH1745NUC_DINT_DATA_LSB     (0x58)
#define BH1745NUC_DINT_DATA_MSB     (0x59)
#define BH1745NUC_INTERRUPT         (0x60)
#define BH1745NUC_PERSISTENCE       (0x61)
#define BH1745NUC_TH_LSB            (0x62)
#define BH1745NUC_TH_MSB            (0x63)
#define BH1745NUC_TL_LSB            (0x64)
#define BH1745NUC_TL_MSB            (0x65)
#define BH1745NUC_MANUFACTURER_ID   (0x92)

/* ID values */

#define BH1745NUC_MANUFACTID        (0xe0)
#define BH1745NUC_PARTID            (0x0b)

/* SYSTEM_CONTROL bits */

#define BH1745NUC_SYSTEM_CONTROL_PART_ID_MASK  (0x3f)
#define BH1745NUC_SYSTEM_CONTROL_INT_RESET     (1 << 6)
#define BH1745NUC_SYSTEM_CONTROL_SW_RESET      (1 << 7)

/* MODE_CONTROL1 bits */

#define BH1745NUC_MODE_CONTROL1_MEAS_TIME160MS  (0x00)
#define BH1745NUC_MODE_CONTROL1_MEAS_TIME320MS  (0x01)
#define BH1745NUC_MODE_CONTROL1_MEAS_TIME640MS  (0x02)
#define BH1745NUC_MODE_CONTROL1_MEAS_TIME1280MS (0x03)
#define BH1745NUC_MODE_CONTROL1_MEAS_TIME2560MS (0x04)
#define BH1745NUC_MODE_CONTROL1_MEAS_TIME5120MS (0x05)

/* MODE_CONTROL2 bits */

#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X1    (0x00)
#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X2    (0x01)
#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X16   (0x02)
#define BH1745NUC_MODE_CONTROL2_RGBC_EN        (1 << 4)
#define BH1745NUC_MODE_CONTROL2_VALID          (1 << 7)

/* MODE_CONTROL3 must always hold this value */

#define BH1745NUC_MODE_CONTROL3_VAL            (0x02)

/* Measurement time used by this driver and the matching VALID poll */

#define BH1745NUC_MEAS_TIME_US                 (160000)
#define BH1745NUC_VALID_POLL_US                (10000)
#define BH1745NUC_VALID_RETRIES                (2 * BH1745NUC_MEAS_TIME_US / \
                                                BH1745NUC_VALID_POLL_US)

/* Burst read of RED, GREEN, BLUE and CLEAR data */

#define BH1745NUC_DATA_BYTES                   (8)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bh1745nuc_dev_s
{
  struct sensor_lowerhalf_s lower;    /* Lower-half sensor driver */
  FAR struct i2c_master_s  *i2c;      /* I2C interface */
  mutex_t                   lock;     /* Serialize device access */
  uint8_t                   addr;     /* I2C address */
  bool                      enabled;  /* Sensor activated */
  unsigned long             interval; /* Polling interval in us */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bh1745nuc_read_reg(FAR struct bh1745nuc_dev_s *priv,
                              uint8_t regaddr, FAR uint8_t *regval);
static int bh1745nuc_read_regs(FAR struct bh1745nuc_dev_s *priv,
                               uint8_t regaddr, FAR uint8_t *regvals,
                               int len);
static int bh1745nuc_write_reg(FAR struct bh1745nuc_dev_s *priv,
                               uint8_t regaddr, uint8_t regval);
static int bh1745nuc_checkid(FAR struct bh1745nuc_dev_s *priv);
static int bh1745nuc_measure(FAR struct bh1745nuc_dev_s *priv,
                             FAR struct sensor_rgb *rgb);

/* Sensor lower-half operations */

static int bh1745nuc_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable);
static int bh1745nuc_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR uint32_t *period_us);
static int bh1745nuc_fetch(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, FAR char *buffer,
                           size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_bh1745nuc_ops =
{
  .activate     = bh1745nuc_activate,
  .set_interval = bh1745nuc_set_interval,
  .fetch        = bh1745nuc_fetch,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bh1745nuc_read_reg
 ****************************************************************************/

static int bh1745nuc_read_reg(FAR struct bh1745nuc_dev_s *priv,
                              uint8_t regaddr, FAR uint8_t *regval)
{
  return bh1745nuc_read_regs(priv, regaddr, regval, 1);
}

/****************************************************************************
 * Name: bh1745nuc_read_regs
 *
 * Description:
 *   Read a block of registers.  The BH1745NUC auto-increments the register
 *   address on multi-byte reads.
 *
 ****************************************************************************/

static int bh1745nuc_read_regs(FAR struct bh1745nuc_dev_s *priv,
                               uint8_t regaddr, FAR uint8_t *regvals,
                               int len)
{
  struct i2c_config_s config;

  config.frequency = CONFIG_BH1745NUC_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  return i2c_writeread(priv->i2c, &config, &regaddr, 1, regvals, len);
}

/****************************************************************************
 * Name: bh1745nuc_write_reg
 ****************************************************************************/

static int bh1745nuc_write_reg(FAR struct bh1745nuc_dev_s *priv,
                               uint8_t regaddr, uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buf[2];

  config.frequency = CONFIG_BH1745NUC_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  buf[0] = regaddr;
  buf[1] = regval;

  return i2c_write(priv->i2c, &config, buf, 2);
}

/****************************************************************************
 * Name: bh1745nuc_checkid
 *
 * Description:
 *   Read and verify the BH1745NUC manufacturer and part ID
 *
 ****************************************************************************/

static int bh1745nuc_checkid(FAR struct bh1745nuc_dev_s *priv)
{
  uint8_t id = 0;
  int ret;

  ret = bh1745nuc_read_reg(priv, BH1745NUC_MANUFACTURER_ID, &id);
  if (ret < 0)
    {
      return ret;
    }

  if (id != BH1745NUC_MANUFACTID)
    {
      snerr("ERROR: Wrong manufacturer ID: 0x%02x\n", id);
      return -ENODEV;
    }

  ret = bh1745nuc_read_reg(priv, BH1745NUC_SYSTEM_CONTROL, &id);
  if (ret < 0)
    {
      return ret;
    }

  if ((id & BH1745NUC_SYSTEM_CONTROL_PART_ID_MASK) != BH1745NUC_PARTID)
    {
      snerr("ERROR: Wrong part ID: 0x%02x\n", id);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: bh1745nuc_measure
 *
 * Description:
 *   Wait for a completed RGBC measurement and read the RGB channels.
 *   The channel values are the raw 16-bit ADC counts.
 *
 ****************************************************************************/

static int bh1745nuc_measure(FAR struct bh1745nuc_dev_s *priv,
                             FAR struct sensor_rgb *rgb)
{
  uint8_t data[BH1745NUC_DATA_BYTES];
  uint8_t regval;
  uint16_t r;
  uint16_t g;
  uint16_t b;
  int retries = BH1745NUC_VALID_RETRIES;
  int ret;

  /* Wait for new data.  Reading MODE_CONTROL2 clears the VALID bit. */

  do
    {
      ret = bh1745nuc_read_reg(priv, BH1745NUC_MODE_CONTROL2, &regval);
      if (ret < 0)
        {
          return ret;
        }

      if (regval & BH1745NUC_MODE_CONTROL2_VALID)
        {
          break;
        }

      nxsig_usleep(BH1745NUC_VALID_POLL_US);
    }
  while (--retries);

  if (retries == 0)
    {
      snerr("ERROR: BH1745NUC measurement timeout\n");
      return -ETIMEDOUT;
    }

  /* Burst read RED, GREEN, BLUE and CLEAR data */

  ret = bh1745nuc_read_regs(priv, BH1745NUC_RED_DATA_LSB, data,
                            BH1745NUC_DATA_BYTES);
  if (ret < 0)
    {
      return ret;
    }

  r = (uint16_t)data[1] << 8 | data[0];
  g = (uint16_t)data[3] << 8 | data[2];
  b = (uint16_t)data[5] << 8 | data[4];

  add_sensor_randomness(r ^ g ^ b);

  rgb->timestamp = sensor_get_timestamp();
  rgb->r         = sensor_data_itof(r);
  rgb->g         = sensor_data_itof(g);
  rgb->b         = sensor_data_itof(b);

  return OK;
}

/****************************************************************************
 * Name: bh1745nuc_activate
 ****************************************************************************/

static int bh1745nuc_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable)
{
  FAR struct bh1745nuc_dev_s *priv =
      container_of(lower, FAR struct bh1745nuc_dev_s, lower);
  int ret;

  nxmutex_lock(&priv->lock);

  if (enable)
    {
      /* 160ms measurement time */

      ret = bh1745nuc_write_reg(priv, BH1745NUC_MODE_CONTROL1,
                                BH1745NUC_MODE_CONTROL1_MEAS_TIME160MS);
      if (ret < 0)
        {
          goto errout;
        }

      /* Enable RGBC measurement with x16 gain */

      ret = bh1745nuc_write_reg(priv, BH1745NUC_MODE_CONTROL2,
                                BH1745NUC_MODE_CONTROL2_RGBC_EN |
                                BH1745NUC_MODE_CONTROL2_ADC_GAIN_X16);
      if (ret < 0)
        {
          goto errout;
        }

      /* MODE_CONTROL3 must be set to 0x02 */

      ret = bh1745nuc_write_reg(priv, BH1745NUC_MODE_CONTROL3,
                                BH1745NUC_MODE_CONTROL3_VAL);
      if (ret < 0)
        {
          goto errout;
        }
    }
  else
    {
      /* Stop RGBC measurement */

      ret = bh1745nuc_write_reg(priv, BH1745NUC_MODE_CONTROL2, 0);
      if (ret < 0)
        {
          goto errout;
        }
    }

  priv->enabled = enable;

errout:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: bh1745nuc_set_interval
 ****************************************************************************/

static int bh1745nuc_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR uint32_t *period_us)
{
  FAR struct bh1745nuc_dev_s *priv =
      container_of(lower, FAR struct bh1745nuc_dev_s, lower);

  priv->interval = *period_us;
  return OK;
}

/****************************************************************************
 * Name: bh1745nuc_fetch
 ****************************************************************************/

static int bh1745nuc_fetch(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct bh1745nuc_dev_s *priv =
      container_of(lower, FAR struct bh1745nuc_dev_s, lower);
  struct sensor_rgb rgb;
  int ret;

  if (buflen != sizeof(rgb))
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);

  if (!priv->enabled)
    {
      ret = -EACCES;
      goto errout;
    }

  ret = bh1745nuc_measure(priv, &rgb);
  if (ret < 0)
    {
      goto errout;
    }

  memcpy(buffer, &rgb, sizeof(rgb));
  ret = sizeof(rgb);

errout:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bh1745nuc_register_uorb
 *
 * Description:
 *   Register the BH1745NUC color sensor as a uORB device.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path.
 *   i2c   - The I2C bus driver instance.
 *   addr  - The I2C address of the BH1745NUC.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bh1745nuc_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                            uint8_t addr)
{
  FAR struct bh1745nuc_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_zalloc(sizeof(struct bh1745nuc_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate BH1745NUC instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->interval   = 1000000;
  priv->lower.ops  = &g_bh1745nuc_ops;
  priv->lower.type = SENSOR_TYPE_RGB;
  nxmutex_init(&priv->lock);

  /* Probe the device */

  ret = bh1745nuc_checkid(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to identify BH1745NUC: %d\n", ret);
      goto errout;
    }

  /* Reset the device to a known state */

  ret = bh1745nuc_write_reg(priv, BH1745NUC_SYSTEM_CONTROL,
                            BH1745NUC_SYSTEM_CONTROL_SW_RESET |
                            BH1745NUC_SYSTEM_CONTROL_INT_RESET);
  if (ret < 0)
    {
      snerr("ERROR: Failed to reset BH1745NUC: %d\n", ret);
      goto errout;
    }

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register BH1745NUC driver: %d\n", ret);
      goto errout;
    }

  sninfo("BH1745NUC registered\n");
  return OK;

errout:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}
