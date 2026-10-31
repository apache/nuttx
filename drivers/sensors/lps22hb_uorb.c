/****************************************************************************
 * drivers/sensors/lps22hb_uorb.c
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
#include <nuttx/random.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/lps22hb.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Registers */

#define LPS22HB_INTERRUPT_CFG (0x0b)
#define LPS22HB_THS_P_L       (0x0c)
#define LPS22HB_THS_P_H       (0x0d)
#define LPS22HB_WHO_AM_I      (0x0f)
#define LPS22HB_CTRL_REG1     (0x10)
#define LPS22HB_CTRL_REG2     (0x11)
#define LPS22HB_CTRL_REG3     (0x12)
#define LPS22HB_FIFO_CTRL     (0x14)
#define LPS22HB_REF_P_XL      (0x15)
#define LPS22HB_REF_P_L       (0x16)
#define LPS22HB_REF_P_H       (0x17)
#define LPS22HB_RPDS_L        (0x18)
#define LPS22HB_RPDS_H        (0x19)
#define LPS22HB_RES_CONF      (0x1a)
#define LPS22HB_INT_SOURCE    (0x25)
#define LPS22HB_FIFO_STATUS   (0x26)
#define LPS22HB_STATUS        (0x27)
#define LPS22HB_PRESS_OUT_XL  (0x28)
#define LPS22HB_PRESS_OUT_L   (0x29)
#define LPS22HB_PRESS_OUT_H   (0x2a)
#define LPS22HB_TEMP_OUT_L    (0x2b)
#define LPS22HB_TEMP_OUT_H    (0x2c)
#define LPS22HB_LPFP_RES      (0x33)

/* WHO_AM_I value */

#define LPS22HB_DEVID         (0xb1)

/* CTRL_REG1 bits */

#define LPS22HB_CTRL_REG1_BDU (1 << 1)

/* CTRL_REG2 bits */

#define LPS22HB_CTRL_REG2_ONE_SHOT   (1 << 0)
#define LPS22HB_CTRL_REG2_IF_ADD_INC (1 << 4)

/* STATUS bits */

#define LPS22HB_STATUS_P_DA   (1 << 0)
#define LPS22HB_STATUS_T_DA   (1 << 1)

/* Sensitivity: pressure 4096 LSB/hPa, temperature 100 LSB/degC */

#define LPS22HB_PRESS_DIVIDER (4096)
#define LPS22HB_TEMP_DIVIDER  (100)

/* One-shot conversion timeout */

#define LPS22HB_ONE_SHOT_RETRIES 10

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lps22hb_dev_s
{
  FAR struct sensor_lowerhalf_s lower;    /* Lower-half sensor driver */
  FAR struct i2c_master_s      *i2c;      /* I2C interface */
  uint8_t                       addr;     /* I2C address */
  bool                          enabled;  /* Sensor activated */
  unsigned long                 interval; /* Polling interval in us */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lps22hb_read_reg(FAR struct lps22hb_dev_s *priv, uint8_t regaddr,
                            FAR uint8_t *regval);
static int lps22hb_write_reg(FAR struct lps22hb_dev_s *priv, uint8_t regaddr,
                             uint8_t regval);
static int lps22hb_read_regs(FAR struct lps22hb_dev_s *priv, uint8_t regaddr,
                             FAR uint8_t *regvals, int len);
static int lps22hb_checkid(FAR struct lps22hb_dev_s *priv);
static int lps22hb_measure(FAR struct lps22hb_dev_s *priv,
                           FAR struct sensor_baro *baro);

/* Sensor lower-half operations */

static int lps22hb_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);
static int lps22hb_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us);
static int lps22hb_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, FAR char *buffer,
                         size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_lps22hb_ops =
{
  .activate     = lps22hb_activate,
  .set_interval = lps22hb_set_interval,
  .fetch        = lps22hb_fetch,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lps22hb_read_reg
 ****************************************************************************/

static int lps22hb_read_reg(FAR struct lps22hb_dev_s *priv, uint8_t regaddr,
                            FAR uint8_t *regval)
{
  return lps22hb_read_regs(priv, regaddr, regval, 1);
}

/****************************************************************************
 * Name: lps22hb_read_regs
 *
 * Description:
 *   Read a block of registers.  The LPS22HB auto-increments the register
 *   address on multi-byte reads (IF_ADD_INC is set by default).
 *
 ****************************************************************************/

static int lps22hb_read_regs(FAR struct lps22hb_dev_s *priv, uint8_t regaddr,
                             FAR uint8_t *regvals, int len)
{
  struct i2c_config_s config;

  config.frequency = CONFIG_LPS22HB_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  return i2c_writeread(priv->i2c, &config, &regaddr, 1, regvals, len);
}

/****************************************************************************
 * Name: lps22hb_write_reg
 ****************************************************************************/

static int lps22hb_write_reg(FAR struct lps22hb_dev_s *priv, uint8_t regaddr,
                             uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buf[2];

  config.frequency = CONFIG_LPS22HB_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  buf[0] = regaddr;
  buf[1] = regval;

  return i2c_write(priv->i2c, &config, buf, 2);
}

/****************************************************************************
 * Name: lps22hb_checkid
 *
 * Description:
 *   Read and verify the LPS22HB chip ID
 *
 ****************************************************************************/

static int lps22hb_checkid(FAR struct lps22hb_dev_s *priv)
{
  uint8_t devid = 0;
  int ret;

  ret = lps22hb_read_reg(priv, LPS22HB_WHO_AM_I, &devid);
  if (ret < 0)
    {
      return ret;
    }

  if (devid != LPS22HB_DEVID)
    {
      snerr("ERROR: Wrong device ID: 0x%02x\n", devid);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: lps22hb_measure
 *
 * Description:
 *   Trigger a one-shot measurement and read pressure and temperature.
 *
 ****************************************************************************/

static int lps22hb_measure(FAR struct lps22hb_dev_s *priv,
                           FAR struct sensor_baro *baro)
{
  uint8_t data[5];
  uint8_t regval;
  int32_t press_raw;
  int16_t temp_raw;
  int retries = LPS22HB_ONE_SHOT_RETRIES;
  int whole;
  int frac;
  int ret;

  /* Enable block data update and set one-shot (ODR = 0) mode */

  ret = lps22hb_write_reg(priv, LPS22HB_CTRL_REG1, LPS22HB_CTRL_REG1_BDU);
  if (ret < 0)
    {
      return ret;
    }

  /* Trigger a one-shot conversion.  Keep IF_ADD_INC set so that the burst
   * read below auto-increments the register address.
   */

  ret = lps22hb_write_reg(priv, LPS22HB_CTRL_REG2,
                          LPS22HB_CTRL_REG2_IF_ADD_INC |
                          LPS22HB_CTRL_REG2_ONE_SHOT);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for pressure and temperature data to be available */

  do
    {
      nxsig_usleep(5000);
      ret = lps22hb_read_reg(priv, LPS22HB_STATUS, &regval);
      if (ret < 0)
        {
          return ret;
        }
    }
  while (((regval & (LPS22HB_STATUS_P_DA | LPS22HB_STATUS_T_DA)) !=
          (LPS22HB_STATUS_P_DA | LPS22HB_STATUS_T_DA)) && --retries);

  if (retries == 0)
    {
      snerr("ERROR: LPS22HB conversion timeout\n");
      return -ETIMEDOUT;
    }

  /* Burst read pressure (3 bytes) and temperature (2 bytes) */

  ret = lps22hb_read_regs(priv, LPS22HB_PRESS_OUT_XL, data, 5);
  if (ret < 0)
    {
      return ret;
    }

  press_raw = data[2] << 16 | data[1] << 8 | data[0];

  /* Sign-extend the 24-bit pressure value */

  if (press_raw & 0x00800000)
    {
      press_raw |= 0xff000000;
    }

  temp_raw = data[4] << 8 | data[3];

  add_sensor_randomness(press_raw ^ temp_raw);

  /* Pressure is a 24-bit count in 1/4096 hPa.  Split the integer and
   * fractional hPa so that the operands stay within the b16 range.
   */

  whole = press_raw / LPS22HB_PRESS_DIVIDER;
  frac  = press_raw % LPS22HB_PRESS_DIVIDER;

  baro->timestamp   = sensor_get_timestamp();
  baro->pressure    =
    sensor_data_add(sensor_data_itof(whole),
                    sensor_data_divi(sensor_data_itof(frac),
                                     LPS22HB_PRESS_DIVIDER));
  baro->temperature = sensor_data_divi(sensor_data_itof(temp_raw),
                                       LPS22HB_TEMP_DIVIDER);

  return OK;
}

/****************************************************************************
 * Name: lps22hb_activate
 ****************************************************************************/

static int lps22hb_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct lps22hb_dev_s *priv =
      container_of(lower, FAR struct lps22hb_dev_s, lower);

  priv->enabled = enable;
  return OK;
}

/****************************************************************************
 * Name: lps22hb_set_interval
 ****************************************************************************/

static int lps22hb_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us)
{
  FAR struct lps22hb_dev_s *priv =
      container_of(lower, FAR struct lps22hb_dev_s, lower);

  priv->interval = *period_us;
  return OK;
}

/****************************************************************************
 * Name: lps22hb_fetch
 ****************************************************************************/

static int lps22hb_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct lps22hb_dev_s *priv =
      container_of(lower, FAR struct lps22hb_dev_s, lower);
  struct sensor_baro baro;
  int ret;

  if (buflen != sizeof(baro))
    {
      return -EINVAL;
    }

  ret = lps22hb_measure(priv, &baro);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer, &baro, sizeof(baro));
  return sizeof(baro);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lps22hb_register_uorb
 *
 * Description:
 *   Register the LPS22HB barometer sensor as a UORB device.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path.
 *   i2c   - The I2C bus driver instance.
 *   addr  - The I2C address of the LPS22HB.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lps22hb_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                          uint8_t addr)
{
  FAR struct lps22hb_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_zalloc(sizeof(struct lps22hb_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate LPS22HB instance\n");
      return -ENOMEM;
    }

  priv->i2c            = i2c;
  priv->addr           = addr;
  priv->interval       = 1000000;
  priv->lower.ops      = &g_lps22hb_ops;
  priv->lower.type     = SENSOR_TYPE_BAROMETER;

  /* Probe the device */

  ret = lps22hb_checkid(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to identify LPS22HB: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register LPS22HB driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  sninfo("LPS22HB registered\n");
  return OK;
}
