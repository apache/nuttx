/****************************************************************************
 * drivers/sensors/icp10125.c
 * Character driver for icp10125.
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

#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>
#include <nuttx/nuttx.h>
#include <nuttx/wqueue.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/icp10125.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C address len */

#define ICP10125_I2C_ADDR_LEN          7        /* I2C address length */

/* Array count func define */

#define ICP10125_COUNTOF(arr)          (sizeof(arr) / sizeof((arr)[0]))
                                                /* Array count function */

/* Eanble/disable define */

#define ICP10125_ENABLE                1        /* Function enable */
#define ICP10125_DISABLE               0        /* Function disable */

/* Reset time define */

#define ICP10125_RESET_DELAY           500      /* Reset time(us) */
#define ICP10125_RESET_MAX_COUNT       3        /* Max reset count */

/* Device default macros */

#define ICP10125_DEFAULT_INTERVAL      100000   /* Default interval */
#define ICP10125_DEFAULT_BUFFER_NUMBER 1        /* Default buffer number */

/* Base struct related */

#define ICP10125_BASE_P_CALI_VALUE_0   45000.0f /* Pressure cali value 0 */
#define ICP10125_BASE_P_CALI_VALUE_1   80000.0f /* Pressure cali value 1 */
#define ICP10125_BASE_P_CALI_VALUE_2   105000.f /* Pressure cali value 2 */
#define ICP10125_BASE_LUT_COE_SHIFT    (1 << 20)
                                                /* LUT coefficient shift */
#define ICP10125_BASE_LUT_LOWER_COE    3.5f     /* LUT lower coefficient */
#define ICP10125_BASE_LUT_UPPER_COE    11.5f    /* LUT lower coefficient */
#define ICP10125_BASE_QUADR_FACTOR     (1 / 16777216.0f)
                                                /* Quadr factor */
#define ICP10125_BASE_OFFSET_FACTOR    2048.0f  /* Offset factor */

/* Temperature conversion related */

#define ICP10125_TEMPERATURE_FACTOR0   -45.0f   /* Temperature factor 0 */
#define ICP10125_TEMPERATURE_FACTOR1   175.0f   /* Temperature factor 1 */
#define ICP10125_TEMPERATURE_SENS      65536.0f /* Temperature sensitivity */

/* Conversion related */

#define ICP10125_TEMPERATURE_LSB_CONST 32768    /* Temperature lsb constant */

/* Command map */

#define ICP10125_WHOAMI_VALUE          0x0008   /* Valid WHOAMI value */

#define ICP10125_CMD_WHOAMI            0xefc8   /* Command: who am i */
#define ICP10125_CMD_RESET             0x805d   /* Command: soft reset */
#define ICP10125_CMD_NORMAL_MODE       0x48a3   /* Command: normal mode */
#define ICP10125_CMD_LOWPOWER_MODE     0x401a   /* Command: lowpower mode */
#define ICP10125_CMD_LOWNOISE_MODE     0x5059   /* Command: lownoise mode */
#define ICP10125_CMD_ULTRA_LN_MODE     0x58e0   /* Command:
                                                   ultra lownoise mode */
#define ICP10125_CMD_MOVE_POINTER      0xc595   /* Command: move pointer
                                                   in address register */
#define ICP10125_CMD_SEND_OTP_ADDR     0x00669c /* Command: send OTP addr */
#define ICP10125_CMD_INC_OTP_ADDR      0xc7f7   /* Command: inc OTP addr */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Sensor ODR enum */

enum icp10125_odr_e
{
  ICP10125_1_Hz,                                /* Sampling freq is 1hz */
  ICP10125_10_Hz,                               /* Sampling freq is 10hz */
  ICP10125_25_Hz                                /* Sampling freq is 25hz */
};

/* Sensor mode enum */

enum icp10125_mode_e
{
  ICP10125_MODE_NORMAL,                         /* Normal mode */
  ICP10125_MODE_LOWPOWER,                       /* Low power mode */
  ICP10125_MODE_LOWNOISE,                       /* Low noise mode */
  ICP10125_MODE_ULTRA_LOWNOISE                  /* Utra low noise mode */
};

/* Base struct */

struct icp10125_base_s
{
  float sensor_constants[4];                    /* Formula constants */
  float p_pa_calib[3];                          /* Pressure calibvalue(in Pa) */
  float lut_lower;                              /* LUT lower half */
  float lut_upper;                              /* LUT upper half */
  float quadr_factor;                           /* Quadratic factor */
  float offst_factor;                           /* Offset factor */
};

/* Sensor struct */

struct icp10125_sensor_s
{
  struct sensor_lowerhalf_s lower;              /* Lower half sensor driver */
  struct icp10125_base_s    base;               /* Sensor base  */
  bool                      activated;          /* Sensor working state */
  uint32_t                  interval;           /* Sensor interval */
  float                     offset;             /* Sensor offset value */
  uint64_t                  timestamp;          /* Units is microseconds */
};

/* Device struct */

struct icp10125_dev_s
{
  struct icp10125_sensor_s           dev;       /* Sensor struct */
  FAR const struct icp10125_config_s *config;   /* The board config */
  struct work_s                      work;      /* Interrupt handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C functions */

static int icp10125_writeread(FAR struct icp10125_dev_s *priv,
                              FAR uint8_t *tx, uint8_t tx_len,
                              FAR uint8_t *rx, uint8_t rx_len);
static int icp10125_write(FAR struct icp10125_dev_s *priv,
                          FAR uint8_t *tx, uint8_t tx_len);
static int icp10125_read(FAR struct icp10125_dev_s *priv,
                         FAR uint8_t *rx, uint8_t rx_len);

/* Sensor handle functions */

/* icp10125 handle functions */

static int icp10125_readdevid(FAR struct icp10125_dev_s *priv);
static int icp10125_reset(FAR struct icp10125_dev_s *priv);
static int icp10125_read_otp(FAR struct icp10125_dev_s *priv, FAR int *out);
static void icp10125_init_base(FAR struct icp10125_dev_s *priv, FAR int *otp);
void icp10125_calculate_conversion_constants(FAR struct icp10125_dev_s *priv,
                                             FAR float *p_pa,
                                             FAR float *p_lut,
                                             FAR float *out);
static int icp10125_setmode(FAR struct icp10125_dev_s *priv,
                            enum icp10125_mode_e mode);
static float icp10125_from_lsb_to_hpa(FAR struct icp10125_dev_s *priv,
                                      int32_t p_lsb, int16_t t_lsb);
static int icp10125_getdata(FAR struct icp10125_dev_s *priv,
                            FAR struct sensor_baro *baro);
static int icp10125_read_push(FAR struct icp10125_dev_s *priv);
static uint8_t icp10125_findodr(FAR struct icp10125_dev_s *priv,
                                FAR uint32_t *interval);
static int icp10125_initchip(FAR struct icp10125_dev_s *priv);

/* Sensor ops functions */

static int icp10125_set_interval(FAR struct file *filep,
                                 FAR struct sensor_lowerhalf_s *lower,
                                 FAR unsigned long *period_us);
static int icp10125_activate(FAR struct file *filep,
                             FAR struct sensor_lowerhalf_s *lower,
                             bool enable);
static int icp10125_selftest(FAR struct file *filep,
                             FAR struct sensor_lowerhalf_s *lower,
                             unsigned long arg);
static int icp10125_set_calibvalue(FAR struct file *filep,
                                   FAR struct sensor_lowerhalf_s *lower,
                                   unsigned long arg);

/* Sensor worker functions */

static void icp10125_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Sensor ops */

static const struct sensor_ops_s g_icp10125_ops =
{
  .set_interval   = icp10125_set_interval,  /* Set output data period */
  .activate       = icp10125_activate,      /* Enable/disable sensor */
  .selftest       = icp10125_selftest,      /* Sensor selftest */
  .set_calibvalue = icp10125_set_calibvalue /* Sensor set calibvalue */
};

/* Sensor ODR */

static const uint32_t g_icp10125_odr[] =
{
  1000000,                                  /* Sampling interval is 1000ms */
  100000,                                   /* Sampling interval is 100ms */
  40000                                     /* Sampling interval is 40ms */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* I2C functions */

/****************************************************************************
 * Name: icp10125_writeread
 *
 * Description:
 *   Read data.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   rx     - Rx buffer.
 *   rx_len - Rx length.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int icp10125_writeread(FAR struct icp10125_dev_s *priv,
                              FAR uint8_t *tx, uint8_t tx_len,
                              FAR uint8_t *rx, uint8_t rx_len)
{
  struct i2c_config_s config;

  /* Set up the I2C configuration. */

  config.frequency = priv->config->freq;
  config.address   = priv->config->addr;
  config.addrlen   = ICP10125_I2C_ADDR_LEN;

  /* I2c write and then read bytes. */

  return i2c_writeread(priv->config->i2c, &config, tx, tx_len, rx, rx_len);
}

/****************************************************************************
 * Name: icp10125_write
 *
 * Description:
 *   Write data.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   tx     - Tx buffer.
 *   tx_len - Tx length.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int icp10125_write(FAR struct icp10125_dev_s *priv,
                          FAR uint8_t *tx, uint8_t tx_len)
{
  struct i2c_config_s config;

  /* Set up the I2C configuratiion. */

  config.frequency = priv->config->freq;
  config.address   = priv->config->addr;
  config.addrlen   = ICP10125_I2C_ADDR_LEN;

  /* I2c read bytes. */

  return i2c_write(priv->config->i2c, &config, tx, tx_len);
}

/****************************************************************************
 * Name: icp10125_read
 *
 * Description:
 *   Write data.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   rx     - Rx buffer.
 *   rx_len - Rx length.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int icp10125_read(FAR struct icp10125_dev_s *priv,
                         FAR uint8_t *rx, uint8_t rx_len)
{
  struct i2c_config_s config;

  /* Set up the I2C configuration. */

  config.frequency = priv->config->freq;
  config.address   = priv->config->addr;
  config.addrlen   = ICP10125_I2C_ADDR_LEN;

  /* I2c read bytes. */

  return i2c_read(priv->config->i2c, &config, rx, rx_len);
}

/* Sensor handle functions */

/****************************************************************************
 * Name: icp10125_readdevid
 *
 * Description:
 *   Read the device ID.
 *
 * Input Parameters:
 *   priv - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int icp10125_readdevid(FAR struct icp10125_dev_s *priv)
{
  uint8_t buff[4];
  uint16_t regval;
  int ret;

  buff[0] = (ICP10125_CMD_WHOAMI & 0xff00) >> 8;
  buff[1] = ICP10125_CMD_WHOAMI & 0x00ff;

  ret = icp10125_writeread(priv, buff, 2, &buff[2], 2);
  if (ret < 0)
    {
      snerr("Failed to read device id: %d\n", ret);
      return ret;
    }

  regval = ((uint16_t)buff[2]) << 8 | (buff[3]);
  if ((regval & 0x003f) != ICP10125_WHOAMI_VALUE)
    {
      snerr("Wrong device ID: %x\n", regval);
      ret = -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: icp10125_reset
 *
 * Description:
 *   Software reset the sensor.
 *
 * Input Parameters:
 *   priv - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int icp10125_reset(FAR struct icp10125_dev_s *priv)
{
  uint8_t buff[2];
  int ret;

  /* software reset. */

  buff[0] = (ICP10125_CMD_RESET & 0xff00) >> 8;
  buff[1] = ICP10125_CMD_RESET & 0x00ff;

  ret = icp10125_write(priv, buff, 2);
  if (ret < 0)
    {
      snerr("Failed to software reset: %d\n", ret);
    }

  /* Delay for reset booting. */

  up_udelay(ICP10125_RESET_DELAY);

  return ret;
}

/****************************************************************************
 * Name: icp10125_read_otp
 *
 * Description:
 *   Read calibration parameters OTP0, ..., OTP3.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   out  - Buffer for output value.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int icp10125_read_otp(FAR struct icp10125_dev_s *priv, FAR int *out)
{
  uint8_t wbuffer[5];
  uint8_t rbuffer[3];
  int ret;
  int i;

  /* Set chip in OTP read mode. */

  wbuffer[0] = (ICP10125_CMD_MOVE_POINTER & 0xff00) >> 8;
  wbuffer[1] = ICP10125_CMD_MOVE_POINTER & 0x00ff;
  wbuffer[2] = (ICP10125_CMD_SEND_OTP_ADDR & 0xff0000) >> 16;
  wbuffer[3] = (ICP10125_CMD_SEND_OTP_ADDR & 0x00ff00) >> 8;
  wbuffer[4] = ICP10125_CMD_SEND_OTP_ADDR & 0x0000ff;

  ret = icp10125_write(priv, wbuffer, 5);
  if (ret < 0)
    {
      snerr("Failed to set OTP read mode: %d\n", ret);
      return ret;
    }

  /* Read OTP values. */

  for (i = 0; i < 4; i++)
    {
      /* Increase address for read-out of OTP. */

      wbuffer[0] = (ICP10125_CMD_INC_OTP_ADDR & 0xff00) >> 8;
      wbuffer[1] = ICP10125_CMD_INC_OTP_ADDR & 0x00ff;

      ret = icp10125_write(priv, wbuffer, 2);
      if (ret < 0)
        {
          snerr("Failed to increase OTP address: %d\n", ret);
          return ret;
        }

      /* Read OTP values. */

      ret = icp10125_read(priv, rbuffer, 3);
      if (ret < 0)
        {
          snerr("Failed to read OTP values %d\n", ret);
          return ret;
        }

      out[i] = (((uint16_t)rbuffer[0] << 8 | rbuffer[1]) + 32768) %
               65536 - 32768;
    }

  return ret;
}

/****************************************************************************
 * Name: icp10125_init_base
 *
 * Description:
 *   Initialize pressure data struct.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   otp  - Input OTP values.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void icp10125_init_base(FAR struct icp10125_dev_s *priv, FAR int *otp)
{
  int i;

  for (i = 0; i < 4; i++)
    {
      priv->dev.base.sensor_constants[i] = otp[i];
    }

  priv->dev.base.p_pa_calib[0] = ICP10125_BASE_P_CALI_VALUE_0;
  priv->dev.base.p_pa_calib[1] = ICP10125_BASE_P_CALI_VALUE_1;
  priv->dev.base.p_pa_calib[2] = ICP10125_BASE_P_CALI_VALUE_2;
  priv->dev.base.lut_lower = ICP10125_BASE_LUT_LOWER_COE *
                             ICP10125_BASE_LUT_COE_SHIFT;
  priv->dev.base.lut_upper = ICP10125_BASE_LUT_UPPER_COE *
                             ICP10125_BASE_LUT_COE_SHIFT;
  priv->dev.base.quadr_factor = ICP10125_BASE_QUADR_FACTOR;
  priv->dev.base.offst_factor = ICP10125_BASE_OFFSET_FACTOR;
}

/****************************************************************************
 * Name: icp10125_calculate_conversion_constants
 *
 * Description:
 *   Calculate conversion constants.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   p_Pa  - Pressure data in Pa.
 *   p_LUT - P_LUT values.
 *   out   - Output conversion constants.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

void icp10125_calculate_conversion_constants(FAR struct icp10125_dev_s *priv,
                                             FAR float *p_pa,
                                             FAR float *p_lut,
                                             FAR float *out)
{
  out[2] = (p_lut[0] * p_lut[1] * (p_pa[0] - p_pa[1]) +
           p_lut[1] * p_lut[2] * (p_pa[1] - p_pa[2]) +
           p_lut[2] * p_lut[0] * (p_pa[2] - p_pa[0])) /
           (p_lut[2] * (p_pa[0] - p_pa[1]) +
           p_lut[0] * (p_pa[1] - p_pa[2]) +
           p_lut[1] * (p_pa[2] - p_pa[0]));
  out[0] = (p_pa[0] * p_lut[0] - p_pa[1] * p_lut[1] -
           (p_pa[1] - p_pa[0]) * out[2]) / (p_lut[0] - p_lut[1]);
  out[1] = (p_pa[0] - out[0]) * (p_lut[0] + out[2]);
}

/****************************************************************************
 * Name: icp10125_setmode
 *
 * Description:
 *   Set working mode.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   mode - Working mode.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int icp10125_setmode(FAR struct icp10125_dev_s *priv,
                            enum icp10125_mode_e mode)
{
  uint8_t mode_buff[2];
  int ret = -EINVAL;

  switch (mode)
    {
      case ICP10125_MODE_NORMAL:
        {
          mode_buff[0] = (ICP10125_CMD_NORMAL_MODE & 0xff00) >> 8;
          mode_buff[1] = ICP10125_CMD_NORMAL_MODE & 0x00ff;
          ret = icp10125_write(priv, mode_buff, 2);
          if (ret < 0)
            {
              snerr("ERROR: Failed to set chip in normal mode\n");
              return ret;
            }
        }
        break;

      case ICP10125_MODE_LOWPOWER:
        {
          mode_buff[0] = (ICP10125_CMD_LOWPOWER_MODE & 0xff00) >> 8;
          mode_buff[1] = ICP10125_CMD_LOWPOWER_MODE & 0x00ff;
          ret = icp10125_write(priv, mode_buff, 2);
          if (ret < 0)
            {
              snerr("ERROR: Failed to set chip in lowpower mode\n");
              return ret;
            }
        }
        break;

      case ICP10125_MODE_LOWNOISE:
        {
          mode_buff[0] = (ICP10125_CMD_LOWNOISE_MODE & 0xff00) >> 8;
          mode_buff[1] = ICP10125_CMD_LOWNOISE_MODE & 0x00ff;
          ret = icp10125_write(priv, mode_buff, 2);
          if (ret < 0)
            {
              snerr("ERROR: Failed to set chip in lowpower mode\n");
              return ret;
            }
        }
        break;

      case ICP10125_MODE_ULTRA_LOWNOISE:
        {
          mode_buff[0] = (ICP10125_CMD_ULTRA_LN_MODE & 0xff00) >> 8;
          mode_buff[1] = ICP10125_CMD_ULTRA_LN_MODE & 0x00ff;
          ret = icp10125_write(priv, mode_buff, 2);
          if (ret < 0)
            {
              snerr("ERROR: Failed to set chip in lowpower mode\n");
              return ret;
            }
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: icp10125_from_lsb_to_hpa
 *
 * Description:
 *   Convert raw-data into engineering units.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   p_lsb - Pressure raw data.
 *   t_lsb - Temperature raw data.
 *
 * Returned Value:
 *   Return to the converted data(uints in hPa).
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static float icp10125_from_lsb_to_hpa(FAR struct icp10125_dev_s *priv,
                                      int32_t p_lsb, int16_t t_lsb)
{
  FAR struct icp10125_base_s *base = &(priv->dev.base);
  float out[3];
  float in[3];
  float t;

  t = (float)t_lsb - ICP10125_TEMPERATURE_LSB_CONST;
  in[0] = base->lut_lower + base->sensor_constants[0] * t * t *
          base->quadr_factor;
  in[1] = base->offst_factor * base->sensor_constants[3] +
          base->sensor_constants[1] * t * t * base->quadr_factor;
  in[2] = base->lut_upper + base->sensor_constants[2] * t * t *
          base->quadr_factor;

  icp10125_calculate_conversion_constants(priv, base->p_pa_calib, in, out);

  return (out[0] + out[1] / (out[2] + p_lsb)) / 100.0f;
}

/****************************************************************************
 * Name: icp10125_getdata
 *
 * Description:
 *   Read the ots data.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   baro - Store event data.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int icp10125_getdata(FAR struct icp10125_dev_s *priv,
                            FAR struct sensor_baro *baro)
{
  uint8_t rbuffer[9];
  int32_t pressure;
  int16_t temperature;
  int ret;

  /* Read raw data of pressure and temperature. */

  ret = icp10125_read(priv, rbuffer, 9);
  if (ret < 0)
    {
      snerr("Failed to read raw data: %d\n", ret);
      return ret;
    }

  temperature = (int16_t)(rbuffer[6] << 8 | rbuffer[7]);
  pressure = (int32_t)((rbuffer[0] << 16) | (rbuffer[1] << 8) |
                       rbuffer[3]);

  baro->temperature = ICP10125_TEMPERATURE_FACTOR0 +
                      ICP10125_TEMPERATURE_FACTOR1 /
                      ICP10125_TEMPERATURE_SENS * temperature;
  baro->pressure = icp10125_from_lsb_to_hpa(priv, pressure, temperature) -
                   priv->dev.offset;

  /* Set sensor mode after every data reading. */

  ret = icp10125_setmode(priv, ICP10125_MODE_LOWNOISE);
  if (ret < 0)
    {
      snerr("Failed to set mode: low noise: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: icp10125_read_push
 *
 * Description:
 *   Read the ots data.
 *   Then push data to upper half.
 *
 * Input Parameters:
 *   priv - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int icp10125_read_push(FAR struct icp10125_dev_s *priv)
{
  struct sensor_baro baro;
  int ret;

  /* Get pressure and temperature data. */

  ret = icp10125_getdata(priv, &baro);
  if (ret < 0)
    {
      snerr("Failed to get raw data: %d\n", ret);
      return ret;
    }

  /* Add timestamp. */

  baro.timestamp = priv->dev.timestamp;

  /* Push data to upper half. */

  priv->dev.lower.push_event(priv->dev.lower.priv, &baro,
                             sizeof(struct sensor_baro));

  return ret;
}

/****************************************************************************
 * Name: icp10125_findodr
 *
 * Description:
 *   Find the best matching odr for sensor.
 *
 * Input Parameters:
 *   priv     - Device struct.
 *   interval - Interval to be judged.
 *
 * Returned Value:
 *   Index of the best fit ODR.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static uint8_t icp10125_findodr(FAR struct icp10125_dev_s *priv,
                                FAR uint32_t *interval)
{
  uint8_t idx;

  for (idx = 0; idx < ICP10125_COUNTOF(g_icp10125_odr); idx++)
    {
      if (*interval >= g_icp10125_odr[idx])
        {
          return idx;
        }
    }

  return idx - 1;
}

/****************************************************************************
 * Name: icp10125_initchip
 *
 * Description:
 *   Initialized chip and enter into lowpower mode.
 *
 * Input Parameters:
 *   priv - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int icp10125_initchip(FAR struct icp10125_dev_s *priv)
{
  int otp[4];
  int ret;

  /* Read chip id. */

  ret = icp10125_readdevid(priv);
  if (ret < 0)
    {
      snerr("Failed to reading chip id: %d\n", ret);
      return ret;
    }

  /* Software reset. */

  ret = icp10125_reset(priv);
  if (ret < 0)
    {
      snerr("Failed to reset chip: %d\n", ret);
      return ret;
    }

  /* Read presure calibration parametes: OTP0, ..., OTP3. */

  ret = icp10125_read_otp(priv, otp);
  if (ret < 0)
    {
      snerr("Failed to read otp values: %d\n", ret);
      return ret;
    }

  /* Initialize base structure. */

  icp10125_init_base(priv, otp);
  if (ret < 0)
    {
      snerr("Failed to initialize base structure: %d\n", ret);
    }

  return ret;
}

/* Sensor ops functions */

/****************************************************************************
 * Name: icp10125_set_interval
 *
 * Description:
 *   Set the sensor output data period in microseconds for a given sensor.
 *   If *period_us > max_delay it will be truncated to max_delay and if
 *   *period_us < min_delay it will be replaced by min_delay.
 *
 * Input Parameters:
 *   filep      - The pointer of file, represents each user using the sensor.
 *   lower      - The instance of lower half sensor driver.
 *   period_us  - The time between report data, in us. It may by overwrite
 *                by lower half driver.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int icp10125_set_interval(FAR struct file *filep,
                                 FAR struct sensor_lowerhalf_s *lower,
                                 FAR unsigned long *period_us)
{
  FAR struct icp10125_dev_s *priv = (FAR struct icp10125_dev_s *)lower;
  uint8_t idx;

  /* Sanity check. */

  DEBUGASSERT(priv != NULL && period_us != NULL);

  /* Find best matching interval. */

  idx = icp10125_findodr(priv, (FAR uint32_t *)period_us);
  priv->dev.interval = g_icp10125_odr[idx];

  if (priv->dev.activated)
    {
      /* Change ODR. */

      work_queue(HPWORK, &priv->work, icp10125_worker, priv,
                 priv->dev.interval / USEC_PER_TICK);
    }

  return OK;
}

/****************************************************************************
 * Name: icp10125_activate
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   filep  - The pointer of file, represents each user using the sensor.
 *   lower  - The instance of lower half sensor driver.
 *   enable - true(enable) and false(disable).
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int icp10125_activate(FAR struct file *filep,
                             FAR struct sensor_lowerhalf_s *lower,
                             bool enable)
{
  FAR struct icp10125_dev_s *priv = (FAR struct icp10125_dev_s *)lower;
  int ret = OK;

  /* Sanity check. */

  DEBUGASSERT(priv != NULL);

  /* Check activated flag is updated. */

  if (priv->dev.activated == enable)
    {
      return OK;
    }

  if (enable)
    {
      /* Set chip in low noise mode and read pressure data first. */

      ret = icp10125_setmode(priv, ICP10125_MODE_LOWNOISE);
      if (ret < 0)
        {
          snerr("Failed to set mode: low noise mode: %d\n", ret);
          return ret;
        }

      work_queue(HPWORK, &priv->work, icp10125_worker, priv,
                 priv->dev.interval / USEC_PER_TICK);
    }
  else
    {
      /* If disabled, stop sampling. */

      work_cancel(HPWORK, &priv->work);
    }

  priv->dev.activated = enable;

  return ret;
}

/****************************************************************************
 * Name: icp10125_selftest
 *
 * Description:
 *   Mainly used in the self-test link, including device ID inspection
 *   and device functional inspection.
 *
 * Input Parameters:
 *   filep      - The pointer of file, represents each user using the sensor.
 *   lower      - The instance of lower half sensor driver.
 *   arg        - The parameters associated with cmd.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *   -ENOTTY    - The cmd don't support.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int icp10125_selftest(FAR struct file *filep,
                             FAR struct sensor_lowerhalf_s *lower,
                             unsigned long arg)
{
  FAR struct icp10125_dev_s *priv = (FAR struct icp10125_dev_s *)lower;
  int ret = -ENOTTY;

  DEBUGASSERT(lower != NULL);

  /* Process ioctl commands. */

  switch (arg)
    {
      case SNIOC_SIMPLE_CHECK: /* Simple communication check. */
        {
          ret = icp10125_readdevid(priv);
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: icp10125_set_calibvalue
 *
 * Description:
 *   The calibration value to be written in the dedicated registers. At each
 * power-on, so that the values read from the sensor are already corrected.
 * When the device is calibrated, the absolute accuracy will be better than
 * before.
 *
 * Input Parameters:
 *   filep - The pointer of file, represents each user using the sensor.
 *   lower - The instance of lower half sensor driver.
 *   arg   - The parameters associated with calibration value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *   -ENOTTY - The cmd don't support.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int icp10125_set_calibvalue(FAR struct file *filep,
                                   FAR struct sensor_lowerhalf_s *lower,
                                   unsigned long arg)
{
  FAR struct icp10125_dev_s *priv = (FAR struct icp10125_dev_s *)lower;

  DEBUGASSERT(lower != NULL);

  priv->dev.offset = atof((FAR char *)arg);

  return OK;
}

/* Sensor worker functions */

/****************************************************************************
 * Name: icp10125_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data. We should not do
 *   this in a interrupt since it might take too long. Also we cannot lock
 *   the I2C bus from within an interrupt.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void icp10125_worker(FAR void *arg)
{
  FAR struct icp10125_dev_s *priv = arg;

  /* Sanity check. */

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Get the timestamp. */

  priv->dev.timestamp = sensor_get_timestamp();

  /* Prepare for next reading. */

  work_queue(HPWORK, &priv->work, icp10125_worker, priv,
             priv->dev.interval / USEC_PER_TICK);

  /* Read the latest sensor data and push to upper half. */

  icp10125_read_push(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icp10125_register
 *
 * Description:
 *   Register the icp10125 character device as 'devno'
 *
 * Input Parameters:
 *   devno  - The device number, used to build the device path
 *              as /dev/sensor/baroN
 *   config - The board config function for the device.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int icp10125_register(int devno, FAR const struct icp10125_config_s *config)
{
  FAR struct icp10125_dev_s *priv;
  int ret;

  /* Sanity check. */

  DEBUGASSERT(config != NULL);

  /* Initialize the icp10125 device structure. */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->dev.lower.type    = SENSOR_TYPE_BAROMETER;
  priv->dev.lower.nbuffer = ICP10125_DEFAULT_BUFFER_NUMBER;
  priv->dev.lower.ops     = &g_icp10125_ops;
  priv->dev.interval      = ICP10125_DEFAULT_INTERVAL;
  priv->config            = config;

  /* Initialize chip and enter into lowpower mode. */

  ret = icp10125_initchip(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to init chip: %d\n", ret);
      goto err;
    }

  /* Register the character driver. */

  ret = sensor_register(&priv->dev.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      goto err;
    }

  return ret;

err:
  kmm_free(priv);
  return ret;
}
