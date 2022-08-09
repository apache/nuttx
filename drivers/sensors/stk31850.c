/****************************************************************************
 * drivers/sensors/stk31850.c
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

#include <stdio.h>
#include <stdlib.h>
#include <debug.h>
#include <errno.h>
#include <math.h>
#include <assert.h>

#include <nuttx/nuttx.h>
#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/sensors/stk31850.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STK31850_DEFAULT_INTERVAL   100000        /* Default conversion interval. */
#define STK31850_DEVICE_ID          0x82          /* Device ID. */
#define STK31850_RESET_CODE         0xff          /* Software reset code. */
#define STK31850_FSM_SET            0x01          /* FSM set value. */
#define STK31850_DISABLE            0x00          /* Pre-define of disable. */
#define STK31850_ENABLE             0x01          /* Pre-define of enable. */
#define STK31850_DEFAULT_WAIT       0x40          /* Defalut wait period set. */
#define STK31850_DEFAULT_SPEC1      0x09          /* Defalut spec1 set. */
#define STK31850_DEFAULT_SPEC2      0x03          /* Defalut spec2 set. */
#define STK31850_LONG_INTEG         0x00          /* Define ALS long integration time. */
#define STK31850_RESET_TIME         30000         /* Reset delay time(us). */
#define STK31850_ALS_THD_H          60000         /* ALS high threshold. */
#define STK31850_ALS_THD_L          1200          /* ALS low threshold. */
#define STK31850_FREQ_SET           20            /* ALS frequency set. */
#define STK31850_GAIN_SET_MAX       3             /* Gain setting index max. */
#define STK31850_WAIT_COUNT         5             /* Data read wait count. */
#define STK31850_WAIT_TIME          5000          /* Wait for data ready(us). */

/* FIFO mode. */

#define STK31850_FIFO_BYPASS        0x01          /* FIFO bypass mode. */

/* Sensor ODR */

#define STK31850_UNIT_TIME          1000000.0f    /* Unit time 1000000us */

#define STK31850_ODR_320Hz          0x00          /* ALS integration time 3.125ms. */
#define STK31850_ODR_160Hz          0x01          /* ALS integration time 6.25ms. */
#define STK31850_ODR_80Hz           0x02          /* ALS integration time 12.5ms. */
#define STK31850_ODR_40Hz           0x03          /* ALS integration time 25ms. */
#define STK31850_ODR_20Hz           0x04          /* ALS integration time 50ms. */
#define STK31850_ODR_10Hz           0x05          /* ALS integration time 100ms. */
#define STK31850_ODR_5Hz            0x06          /* ALS integration time 200ms. */
#define STK31850_ODR_2p5Hz          0x07          /* ALS integration time 400ms. */

/* Device Register */

#define STK31850_STATE              0x00          /* ALS state register. */
#define STK31850_PDT_ID             0x3e          /* Product ID(TBD) register. */
#define STK31850_ALSCTRL1           0x02          /* ALS control 1 register. */
#define STK31850_INTCTRL1           0x04          /* Interrupt control 1 register. */
#define STK31850_WAIT               0x05          /* Wait register. */
#define STK31850_FLAG               0x10          /* Flag register. */
#define STK31850_DATA1_ALS_F_REG    0x13          /* ALS data 1 register. */
#define STK31850_DATA2_ALS_F_REG    0x14          /* ALS data 2 register. */
#define STK31850_DATA1_C_REG        0x15          /* C data 1 register. */
#define STK31850_DATA2_C_REG        0x16          /* C data 2 register. */
#define STK31850_GAINCTRL           0x4e          /* Gain control register. */
#define STK31850_FSM                0x5f          /* FSM register. */
#define STK31850_ALSCTRL2           0x6f          /* ALS control 2 register. */
#define STK31850_PDCTRL1            0xa1          /* PD control 1 register. */
#define STK31850_INTCTRL2           0xa5          /* Interrupt control 2 register. */
#define STK31850_AGAINCTRL1         0xdb          /* A gain control register. */
#define STK31850_FIFOCTRL1          0x60          /* FIFO control 1 register. */
#define STK31850_FIFO1_WM_LV        0x61          /* FIFO watermark level 1 register. */
#define STK31850_FIFO2_WM_LV        0x62          /* FIFO watermark level 2 register. */
#define STK31850_FIFOCTRL2          0x63          /* FIFO control 2 register. */
#define STK31850_SOFT_RESET         0x80          /* Software reset register. */
#define STK31850_SPEC1              0xf6          /* SPEC1 register. */
#define STK31850_SPEC2              0x81          /* SPEC2 register. */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Device Register Bit */

struct stk31850_state_s
{
  uint8_t not_used_01              : 1;
  uint8_t en_als                   : 1;
  uint8_t en_wait                  : 1;
  uint8_t not_used_02              : 5;
};

typedef struct stk31850_state_s stk31850_state_t;

struct stk31850_alsctl1_s
{
  uint8_t it_als                   : 4;
  uint8_t gain_als                 : 2;
  uint8_t prst_als                 : 2;
};

typedef struct stk31850_alsctl1_s stk31850_alsctl1_t;

struct stk31850_intctl1_s
{
  uint8_t not_used_01              : 3;
  uint8_t en_als_int               : 1;
  uint8_t not_used_02              : 4;
};

typedef struct stk31850_intctl1_s stk31850_intctl1_t;

struct stk31850_wait_s
{
  uint8_t wait                      : 8;
};

typedef struct stk31850_wait_s stk31850_wait_t;

struct stk31850_flag_s
{
  uint8_t flg_nf                    : 1;   /* Read only */
  uint8_t not_used_01               : 1;
  uint8_t flg_als_sat               : 1;   /* Read only */
  uint8_t not_used_02               : 2;
  uint8_t flg_als_int               : 1;
  uint8_t not_used_03               : 1;
  uint8_t flg_als_dr                : 1;
};

typedef struct stk31850_flag_s stk31850_flag_t;

struct stk31850_gainctrl_s
{
  uint8_t not_used_01               : 1;
  uint8_t gain_als_dx128            : 1;
  uint8_t gain_c_dx128              : 1;
  uint8_t not_used_02               : 1;
  uint8_t gain_c                    : 2;
  uint8_t not_used_03               : 2;
};

typedef struct stk31850_gainctrl_s stk31850_gainctrl_t;

struct stk31850_pdctrl1_s
{
  uint8_t c                         : 1;
  uint8_t als                       : 1;
  uint8_t not_used_01               : 6;
};

typedef struct stk31850_pdctrl1_s stk31850_pdctrl1_t;

struct stk31850_alsctl2_s
{
  uint8_t it2_als                   : 5;
  uint8_t not_used_01               : 2;
  uint8_t it_als_sel                : 1;
};

typedef struct stk31850_alsctl2_s stk31850_alsctl2_t;

struct stk31850_intctl2_s
{
  uint8_t not_used_01               : 1;
  uint8_t en_als_dr_int             : 1;
  uint8_t not_used_02               : 6;
};

typedef struct stk31850_intctl2_s stk31850_intctl2_t;

struct stk31850_againctrl_s
{
  uint8_t not_used_01                : 2;
  uint8_t als_ci                     : 2;
  uint8_t c_ci                       : 2;
  uint8_t not_used_02                : 2;
};

typedef struct stk31850_againctrl_s stk31850_againctrl_t;

struct stk31850_fifoctrl1_s
{
  uint8_t fifo_data_sel             : 2;
  uint8_t not_used_01               : 2;
  uint8_t fifo_mode                 : 2;
  uint8_t not_used_02               : 2;
};

typedef struct stk31850_fifoctrl1_s stk31850_fifoctrl1_t;

struct stk31850_fifo1wm_s
{
  uint8_t fifo_wm_lv                : 2;
  uint8_t not_used_01               : 6;
};

typedef struct stk31850_fifo1wm_s stk31850_fifo1wm_t;

struct stk31850_fifo2wm_s
{
  uint8_t fifo_wm_lv                : 8;
};

typedef struct stk31850_fifo2wm_s stk31850_fifo2wm_t;

struct stk31850_fifoctrl2_s
{
  uint8_t fifo_full_en              : 1;
  uint8_t fifo_thd_en               : 1;
  uint8_t fifo_ovr_en               : 1;
  uint8_t not_used_01               : 5;
};

typedef struct stk31850_fifoctrl2_s stk31850_fifoctrl2_t;

struct stk31850_spec1_s
{
  uint8_t value                     : 8;
};

typedef struct stk31850_spec1_s stk31850_spec1_t;

struct stk31850_spec2_s
{
  uint8_t value                     : 8;
};

typedef struct stk31850_spec2_s stk31850_spec2_t;

/* Sensor ODR */

struct stk31850_odr_s
{
  uint8_t regval;                                 /* the data of register */
  float odr;                                      /* the unit is Hz */
};

/* Light gain */

struct stk31850_gain_s
{
  uint8_t pow;                                       /* ALS stage pow */
  uint8_t gain_als;                                  /* ALS gain setting */
  uint8_t gain_c;                                    /* C channel gain setting */
  uint8_t gain_c_128;                                /* C channel gain dx128 */
  uint8_t gain_als_128;                              /* ALS gain gain dx128 */
};

/* Device struct */

struct stk31850_dev_s
{
  /* sensor_lowerhalf_s must be in the first line. */

  struct sensor_lowerhalf_s lower;                /* Lower half sensor driver. */
  bool activated;                                 /* Sensor working state. */
  unsigned long interval;                         /* Sensor acquisition interval. */
  FAR const struct stk31850_config_s *config;     /* The board config function. */
  struct work_s work;                             /* Interrupt handler worker. */
  float last_lux;                                 /* Last light data. */
  float last_ir;                                  /* Last ir data. */
  uint8_t gain_idx;                               /* Gain set index. */
  uint8_t wait_cnt;                               /* worker wait times */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C functions */

static int stk31850_i2c_read(FAR struct stk31850_dev_s *priv,
                             uint8_t regaddr,
                             FAR uint8_t *regval);
static int stk31850_i2c_write(FAR struct stk31850_dev_s *priv,
                              uint8_t regaddr, FAR uint8_t *value);
static int stk31850_i2c_readword(FAR struct stk31850_dev_s *priv,
                                 uint8_t regaddr,
                                 FAR uint16_t *regval);

/* Sensor handle functions */

static int stk31850_readdevid(FAR struct stk31850_dev_s *priv);
static int stk31850_reset(FAR struct stk31850_dev_s *priv);
static int stk31850_enable(FAR struct stk31850_dev_s *priv,
                           bool enable);
static int stk31850_findodr(FAR float *freq);
static int stk31850_setodr(FAR struct stk31850_dev_s *priv, uint8_t value);
static int stk31850_setgain(FAR struct stk31850_dev_s *priv);
static int stk31850_setintmode(FAR struct stk31850_dev_s *priv,
                               uint8_t value);
static int stk31850_setpd(FAR struct stk31850_dev_s *priv,
                          uint8_t value);
static int stk31850_setwait(FAR struct stk31850_dev_s *priv,
                            uint8_t value);
static int stk31850_setfifo(FAR struct stk31850_dev_s *priv);
static int stk31850_setspec(FAR struct stk31850_dev_s *priv);
static int stk31850_checkgain(FAR struct stk31850_dev_s *priv,
                              uint16_t data_f, uint16_t data_c);
static int stk31850_setstate(FAR struct stk31850_dev_s *priv, uint8_t value);
static bool stk31850_check_dataready(FAR struct stk31850_dev_s *priv);
static int stk31850_readlux(FAR struct stk31850_dev_s *priv,
                            FAR float *lux, FAR float *ir);

/* Sensor ops functions */

static int stk31850_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR unsigned long *period_us);
static int stk31850_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             bool enable);
static int stk31850_selftest(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             unsigned long arg);

/* Sensor poll functions */

static void stk31850_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct stk31850_odr_s g_stk31850_odr[] =
{
  {STK31850_ODR_2p5Hz, 2.5},                 /* Sampling interval is 400ms. */
  {STK31850_ODR_5Hz, 5},                     /* Sampling interval is 200ms. */
  {STK31850_ODR_10Hz, 10},                   /* Sampling interval is 100ms. */
  {STK31850_ODR_20Hz, 20},                   /* Sampling interval is 50ms. */
  {STK31850_ODR_40Hz, 40},                   /* Sampling interval is 25ms. */
  {STK31850_ODR_80Hz, 80},                   /* Sampling interval is 12.5ms. */
  {STK31850_ODR_160Hz, 160},                 /* Sampling interval is 6.25ms. */
  {STK31850_ODR_320Hz, 320},                 /* Sampling interval is 3.125ms. */
};

static const struct stk31850_gain_s g_stk31850_gain[] =
{
  {1, 0x03, 0x03, 0x01, 0x01},               /* Gain set x128 times. */
  {2, 0x03, 0x03, 0x00, 0x00},               /* Gain set x64 times. */
  {8, 0x02, 0x02, 0x00, 0x00},               /* Gain set x32 times. */
  {32, 0x01, 0x01, 0x00, 0x00},              /* Gain set x16 times. */
};

static const struct sensor_ops_s g_stk31850_ops =
{
  .activate = stk31850_activate,             /* Enable/disable sensor. */
  .set_interval = stk31850_set_interval,     /* Set output data period. */
  .selftest = stk31850_selftest              /* Sensor selftest function */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* I2C functions */

/****************************************************************************
 * Name: stk31850_i2c_read
 *
 * Description:
 *   Read 8-bit register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Register address.
 *   regval  - Register value.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int stk31850_i2c_read(FAR struct stk31850_dev_s *priv,
                             uint8_t regaddr,
                             FAR uint8_t *regval)
{
  struct i2c_config_s config;
  int ret;

  DEBUGASSERT(regval != NULL);

  /* Set up the I2C configuration */

  config.frequency = priv->config->freq;
  config.address = priv->config->addr;
  config.addrlen = 7;

  /* Write 8 bits to device, then read 8 bits from the device */

  ret = i2c_writeread(priv->config->i2c, &config, &regaddr, 1, regval, 1);
  if (ret < 0)
    {
      snerr("stk31850 I2C r fail:%d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: stk31850_i2c_write
 *
 * Description:
 *   write 8-bit register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Register address.
 *   value   - To be write value.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int stk31850_i2c_write(FAR struct stk31850_dev_s *priv,
                             uint8_t regaddr, FAR uint8_t *value)
{
  struct i2c_config_s config;
  int ret;
  uint8_t buf[2];

  /* Set up the I2C configuration. */

  config.frequency = priv->config->freq;
  config.address = priv->config->addr;
  config.addrlen = 7;

  /* Set up the Modbus write request. */

  buf[0] = regaddr;
  buf[1] = *value;

  /* Write the Modbus write request. */

  ret = i2c_write(priv->config->i2c, &config, buf, sizeof(buf));
  if (ret < 0)
    {
      snerr("stk31850 I2C w fail:%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: stk31850_i2c_readword
 *
 * Description:
 *   Read 16-bit register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Register address.
 *   regval  - Register value.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int stk31850_i2c_readword(FAR struct stk31850_dev_s *priv,
                                 uint8_t regaddr,
                                 FAR uint16_t *regval)
{
  struct i2c_config_s config;
  int ret;
  uint8_t buffer[2];

  DEBUGASSERT(regval != NULL);

  /* Set up the I2C configuration */

  config.frequency = priv->config->freq;
  config.address = priv->config->addr;
  config.addrlen = 7;

  /* Write 8 bits to device, then read 16-bits from the device */

  ret = i2c_writeread(priv->config->i2c, &config, &regaddr, 1, buffer, 2);
  if (ret < 0)
    {
      snerr("stk31850 I2C r fail:%d\n", ret);
      return ret;
    }

  /* Copy the content of the buffer to the location of the uint16_t pointer */

  *regval = (uint16_t)((buffer[0] << 8) | (buffer[1]));

  return ret;
}

/* Sensor handle functions */

/****************************************************************************
 * Name: stk31850_readdevid
 *
 * Description:
 *   Read the device ID.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int stk31850_readdevid(FAR struct stk31850_dev_s *priv)
{
  int ret;
  uint8_t regval;

  ret = stk31850_i2c_read(priv, STK31850_PDT_ID, &regval);
  if (ret < 0 || regval != STK31850_DEVICE_ID)
    {
      snerr("stk31850 ID error %d\n", regval);
      ret = -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: stk31850_reset
 *
 * Description:
 *   Reset stk31850.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int stk31850_reset(FAR struct stk31850_dev_s *priv)
{
  int ret;
  uint8_t value;

  value = STK31850_RESET_CODE;

  ret = stk31850_i2c_write(priv, STK31850_SOFT_RESET, &value);
  if (ret < 0)
    {
      snerr("stk31850 rst fail\n");
      return ret;
    }

  up_udelay(STK31850_RESET_TIME);

  return ret;
}

/****************************************************************************
 * Name: stk31850_enable
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   priv   - The instance of lower half sensor driver
 *   enable - true(enable) and false(disable)
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int stk31850_enable(FAR struct stk31850_dev_s *priv,
                           bool enable)
{
  int ret;

  if (enable)
    {
      /* Disable interrupt. */

      ret = stk31850_setintmode(priv, STK31850_DISABLE);
      if (ret < 0)
        {
          snerr("stk31850 disable intrpt fail\n");
          return ret;
        }

      /* Set default gain. */

      ret = stk31850_setgain(priv);
      if (ret < 0)
        {
          return ret;
        }

      /* Set pd. */

      ret = stk31850_setpd(priv, STK31850_ENABLE);
      if (ret < 0)
        {
          snerr("stk31850 set pd fail\n");
          return ret;
        }

      /* Set wait. */

      ret = stk31850_setwait(priv, STK31850_DEFAULT_WAIT);
      if (ret < 0)
        {
          snerr("stk31850 set wait fail\n");
          return ret;
        }

      /* Disable fifo. */

      ret = stk31850_setfifo(priv);
      if (ret < 0)
        {
          snerr("stk31850 disale fifo fail\n");
          return ret;
        }

      /* Set spec. */

      ret = stk31850_setspec(priv);
      if (ret < 0)
        {
          snerr("stk31850 set spec fail: %d\n");
          return ret;
        }

      /* Enable sensor. */

      ret = stk31850_setstate(priv, STK31850_ENABLE);
      if (ret < 0)
        {
          return ret;
        }

      work_queue(HPWORK, &priv->work,
                 stk31850_worker, priv,
                 priv->interval / USEC_PER_TICK);
    }
  else
    {
      /* Set to shut down */

      priv->last_lux = 0;
      work_cancel(HPWORK, &priv->work);
      ret = stk31850_setstate(priv, STK31850_DISABLE);
      if (ret < 0)
        {
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: stk31850_findodr
 *
 * Description:
 *   Find the period that matches best.
 *
 * Input Parameters:
 *   freq  - Desired frequency.
 *
 * Returned Value:
 *   Index of the best fit ODR.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int stk31850_findodr(FAR float *freq)
{
  int i;
  int num = sizeof(g_stk31850_odr) / sizeof(struct stk31850_odr_s);

  for (i = 0; i < num; i++)
    {
      if (*freq <= g_stk31850_odr[i].odr)
        {
          *freq = g_stk31850_odr[i].odr;
          return i;
        }
    }

  return num - 1;
}

/****************************************************************************
 * Name: stk31850_setodr
 *
 * Description:
 *   Set odr for stk31850.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - The value set to register.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int stk31850_setodr(FAR struct stk31850_dev_s *priv, uint8_t value)
{
  stk31850_alsctl1_t reg_ctl1;
  stk31850_alsctl2_t reg_ctl2;
  int ret;

  ret = stk31850_i2c_read(priv, STK31850_ALSCTRL2, (FAR uint8_t *)&reg_ctl2);
  if (ret < 0)
    {
      return ret;
    }

  ret = stk31850_i2c_read(priv, STK31850_ALSCTRL1, (FAR uint8_t *)&reg_ctl1);
  if (ret < 0)
    {
      return ret;
    }

  reg_ctl2.it_als_sel = STK31850_LONG_INTEG;
  reg_ctl2.it2_als = 0;

  reg_ctl1.it_als = value;
  reg_ctl1.prst_als = 0;

  ret = stk31850_i2c_write(priv, STK31850_ALSCTRL2,
                           (FAR uint8_t *)&reg_ctl2);
  if (ret < 0)
    {
      return ret;
    }

  ret = stk31850_i2c_write(priv, STK31850_ALSCTRL1,
                           (FAR uint8_t *)&reg_ctl1);

  return ret;
}

/****************************************************************************
 * Name: stk31850_setgain
 *
 * Description:
 *   Set gain for stk31850.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int stk31850_setgain(FAR struct stk31850_dev_s *priv)
{
  stk31850_gainctrl_t reg_gain;
  stk31850_againctrl_t reg_again;
  stk31850_alsctl1_t reg_alsctrl1;
  int ret;

  ret = stk31850_i2c_read(priv, STK31850_GAINCTRL, (FAR uint8_t *)&reg_gain);
  if (ret < 0)
    {
      goto exit;
    }

  ret = stk31850_i2c_read(priv, STK31850_AGAINCTRL1,
                          (FAR uint8_t *)&reg_again);
  if (ret < 0)
    {
      goto exit;
    }

  ret = stk31850_i2c_read(priv, STK31850_ALSCTRL1,
                          (FAR uint8_t *)&reg_alsctrl1);
  if (ret < 0)
    {
      goto exit;
    }

  reg_gain.gain_c = g_stk31850_gain[priv->gain_idx].gain_c;
  reg_gain.gain_als_dx128 = g_stk31850_gain[priv->gain_idx].gain_als_128;
  reg_gain.gain_c_dx128 = g_stk31850_gain[priv->gain_idx].gain_c_128;

  reg_again.als_ci = 0;
  reg_again.c_ci = 0;

  reg_alsctrl1.gain_als = g_stk31850_gain[priv->gain_idx].gain_als;

  ret = stk31850_i2c_write(priv, STK31850_GAINCTRL,
                           (FAR uint8_t *)&reg_gain);
  if (ret < 0)
    {
      goto exit;
    }

  ret = stk31850_i2c_write(priv, STK31850_AGAINCTRL1,
                           (FAR uint8_t *)&reg_again);
  if (ret < 0)
    {
      goto exit;
    }

  ret = stk31850_i2c_write(priv, STK31850_ALSCTRL1,
                           (FAR uint8_t *)&reg_alsctrl1);

exit:
  if (ret < 0)
    {
      snerr("stk31850 set gain fail\n");
    }

  return ret;
}

/****************************************************************************
 * Name: stk31850_setintmode
 *
 * Description:
 *   Set interrupt mode.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   value   - Register value.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   when the two threshold low register MSBs are set to 11b.
 *
 ****************************************************************************/

static int stk31850_setintmode(FAR struct stk31850_dev_s *priv,
                               uint8_t value)
{
  stk31850_intctl1_t reg_initctl1;
  stk31850_intctl2_t reg_initctl2;
  stk31850_flag_t reg_flag;
  int ret;

  ret = stk31850_i2c_read(priv, STK31850_INTCTRL1,
                          (FAR uint8_t *)&reg_initctl1);
  if (ret < 0)
    {
      return ret;
    }

  ret = stk31850_i2c_read(priv, STK31850_INTCTRL2,
                          (FAR uint8_t *)&reg_initctl2);
  if (ret < 0)
    {
      return ret;
    }

  reg_initctl1.en_als_int = value;
  reg_initctl2.en_als_dr_int = 1;  /* TODO */

  ret = stk31850_i2c_write(priv, STK31850_INTCTRL1,
                           (FAR uint8_t *)&reg_initctl1);
  if (ret < 0)
    {
      return ret;
    }

  ret = stk31850_i2c_write(priv, STK31850_INTCTRL2,
                           (FAR uint8_t *)&reg_initctl2);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear flags. */

  reg_flag.flg_als_int = 0;
  reg_flag.flg_als_dr = 0;

  ret = stk31850_i2c_write(priv, STK31850_FLAG, (FAR uint8_t *)&reg_flag);

  return ret;
}

/****************************************************************************
 * Name: stk31850_setpd
 *
 * Description:
 *   Enable/disable als c and als.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Register value.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   Read lux data with interrupt.
 *
 ****************************************************************************/

static int stk31850_setpd(FAR struct stk31850_dev_s *priv,
                          uint8_t value)
{
  stk31850_pdctrl1_t reg;

  reg.c = value;
  reg.als = value;

  return stk31850_i2c_write(priv, STK31850_PDCTRL1, (FAR uint8_t *)&reg);
}

/****************************************************************************
 * Name: stk31850_setwait
 *
 * Description:
 *   Set wait time.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Register value.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   Read lux data with interrupt.
 *
 ****************************************************************************/

static int stk31850_setwait(FAR struct stk31850_dev_s *priv,
                            uint8_t value)
{
  stk31850_wait_t reg;

  return stk31850_i2c_write(priv, STK31850_WAIT, (FAR uint8_t *)&reg);
}

/****************************************************************************
 * Name: stk31850_setfifo
 *
 * Description:
 *   Disable fifo.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Register value.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   Read lux data with interrupt.
 *
 ****************************************************************************/

static int stk31850_setfifo(FAR struct stk31850_dev_s *priv)
{
  stk31850_fifoctrl1_t reg_fifoctrl1;
  stk31850_fifoctrl2_t reg_fifoctrl2;
  stk31850_fifo1wm_t reg_fifo1wm;
  stk31850_fifo2wm_t reg_fifo2wm;
  int ret;

  reg_fifoctrl1.fifo_data_sel = 0;
  reg_fifoctrl1.fifo_mode = STK31850_FIFO_BYPASS;

  reg_fifoctrl2.fifo_full_en = 0;
  reg_fifoctrl2.fifo_ovr_en = 0;
  reg_fifoctrl2.fifo_thd_en = 0;

  reg_fifo1wm.fifo_wm_lv = 0;
  reg_fifo2wm.fifo_wm_lv = 0;

  ret = stk31850_i2c_write(priv, STK31850_FIFOCTRL1,
                           (FAR uint8_t *)&reg_fifoctrl1);
  if (ret < 0)
    {
      return ret;
    }

  ret = stk31850_i2c_write(priv, STK31850_FIFOCTRL2,
                           (FAR uint8_t *)&reg_fifoctrl2);
  if (ret < 0)
    {
      return ret;
    }

  ret = stk31850_i2c_write(priv, STK31850_FIFO1_WM_LV,
                           (FAR uint8_t *)&reg_fifo1wm);
  if (ret < 0)
    {
      return ret;
    }

  ret = stk31850_i2c_write(priv, STK31850_FIFO2_WM_LV,
                           (FAR uint8_t *)&reg_fifo2wm);

  return ret;
}

/****************************************************************************
 * Name: stk31850_setspec
 *
 * Description:
 *   Set spec.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   Read lux data with interrupt.
 *
 ****************************************************************************/

static int stk31850_setspec(FAR struct stk31850_dev_s *priv)
{
  stk31850_spec1_t reg1;
  stk31850_spec2_t reg2;
  int ret;

  reg1.value = STK31850_DEFAULT_SPEC1;

  ret = stk31850_i2c_write(priv, STK31850_SPEC1, (FAR uint8_t *)&reg1);
  if (ret < 0)
    {
      return ret;
    }

  reg2.value = STK31850_DEFAULT_SPEC2;

  ret = stk31850_i2c_write(priv, STK31850_SPEC2, (FAR uint8_t *)&reg2);

  return ret;
}

/****************************************************************************
 * Name: stk31850_checkgain
 *
 * Description:
 *   Check the gain.
 *
 * Input Parameters:
 *   priv     - Device struct.
 *   data_f   - als data.
 *   data_c   - c_data.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure; A positive value
 *   indicates the pow after switching.
 *
 * Assumptions/Limitations:
 *
 ****************************************************************************/

static int stk31850_checkgain(FAR struct stk31850_dev_s *priv,
                              uint16_t data_f, uint16_t data_c)
{
  uint8_t fsm_reg = STK31850_FSM_SET;
  unsigned int temp_idx = 0;
  int ret = 0;

  temp_idx = priv->gain_idx;

  if (data_f > STK31850_ALS_THD_H || data_c > STK31850_ALS_THD_H)
    {
      /* Greater than the current range. */

      if (priv->gain_idx < STK31850_GAIN_SET_MAX)
        {
          priv->gain_idx++;
        }
    }
  else if (data_f < STK31850_ALS_THD_L || data_c < STK31850_ALS_THD_L)
    {
      /* Less than the current range. */

      if (priv->gain_idx > 0)
        {
          priv->gain_idx--;
        }
    }

  /* Need to change range. */

  if (temp_idx != priv->gain_idx)
    {
      ret = stk31850_setgain(priv);
      if (ret < 0)
        {
          return ret;
        }

      ret = stk31850_i2c_write(priv, STK31850_FSM, &fsm_reg);
      if (ret < 0)
        {
          return ret;
        }

      return g_stk31850_gain[priv->gain_idx].pow;
    }

  return ret;
}

/****************************************************************************
 * Name: stk31850_setstate
 *
 * Description:
 *   Set sensor state.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Register value.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   Read lux data with interrupt.
 *
 ****************************************************************************/

static int stk31850_setstate(FAR struct stk31850_dev_s *priv, uint8_t value)
{
  stk31850_state_t reg;
  int ret;

  reg.en_als = value;
  reg.en_wait = value;

  ret = stk31850_i2c_write(priv, STK31850_STATE, (FAR uint8_t *)&reg);
  if (ret < 0)
    {
      snerr("stk31850 set state fail\n");
    }

  return ret;
}

/****************************************************************************
 * Name: stk31850_check_dataready
 *
 * Description:
 *   Check if the data is ready (ADC conversion compeleted).
 *
 * Input Parameters:
 *   priv  - Device struct.
 *
 * Returned Value:
 *   True - Data is ready. False - Data is not ready, or failed to check.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static bool stk31850_check_dataready(FAR struct stk31850_dev_s *priv)
{
  stk31850_flag_t reg_flag;
  int ret;

  ret = stk31850_i2c_read(priv, STK31850_FLAG, (FAR uint8_t *)&reg_flag);
  if (ret < 0)
    {
      return false;
    }

  return (reg_flag.flg_als_dr == 1);
}

/****************************************************************************
 * Name: stk31850_readlux
 *
 * Description:
 *   Read the light data.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   lux   - Output data.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   Read lux data with interrupt.
 *
 ****************************************************************************/

static int stk31850_readlux(FAR struct stk31850_dev_s *priv,
                            FAR float *lux, FAR float *ir)
{
  uint16_t value_f;
  uint16_t value_c;
  int ret;

  ret = stk31850_i2c_readword(priv, STK31850_DATA1_ALS_F_REG, &value_f);
  if (ret < 0)
    {
      return ret;
    }

  ret = stk31850_i2c_readword(priv, STK31850_DATA1_C_REG, &value_c);
  if (ret < 0)
    {
      return ret;
    }

  ret = stk31850_checkgain(priv, value_f, value_c);
  if (ret < 0)
    {
      snerr("stk31850 check gain fail\n");
      return ret;
    }
  else if (ret == 0)
    {
      *lux = value_f * g_stk31850_gain[priv->gain_idx].pow;
      *ir = value_c * g_stk31850_gain[priv->gain_idx].pow;
      priv->last_lux = *lux;
      priv->last_ir = *ir;
    }
  else
    {
      *lux = priv->last_lux;
      *ir = priv->last_ir;
    }

  return ret;
}

/* Sensor ops functions */

/****************************************************************************
 * Name: stk31850_set_interval
 *
 * Description:
 *   Set the sensor output data period in microseconds for a given sensor.
 *   If *period_us > max_delay it will be truncated to max_delay and if
 *   *period_us < min_delay it will be replaced by min_delay.
 *
 * Input Parameters:
 *   lower     - The instance of lower half sensor driver.
 *   filep     - The pointer of file, represents each user using the sensor.
 *   period_us - The time between report data, in us. It may by overwrite
 *                by lower half driver.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int stk31850_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR unsigned long *period_us)
{
  FAR struct stk31850_dev_s *priv = (FAR struct stk31850_dev_s *)lower;
  float freq;
  int ret;
  int idx;

  /* Sanity check. */

  DEBUGASSERT(priv != NULL && period_us != NULL);

  freq = STK31850_FREQ_SET;

  /* Find the period that matches best.  */

  idx = stk31850_findodr(&freq);
  ret = stk31850_setodr(priv, g_stk31850_odr[idx].regval);
  if (ret < 0)
    {
      snerr("stk31850 set intrvl fail\n");
      return ret;
    }

  *period_us = STK31850_UNIT_TIME / freq;
  priv->interval = *period_us;

  return ret;
}

/****************************************************************************
 * Name: stk31850_activate
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   lower  - The instance of lower half sensor driver.
 *   filep  - The pointer of file, represents each user using the sensor.
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

static int stk31850_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             bool enable)
{
  FAR struct stk31850_dev_s *priv = (FAR struct stk31850_dev_s *)lower;
  int ret;

  if (lower->type == SENSOR_TYPE_LIGHT)
    {
      if (priv->activated != enable)
        {
          ret = stk31850_enable(priv, enable);
          if (ret < 0)
            {
              return ret;
            }

          priv->activated = enable;
        }
    }
  else
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: stk31850_selftest
 *
 * Selftest allows for the testing of the mechanical and electrical
 * portions of the sensors. When the selftest is activated, the
 * electronics cause the sensors to be actuated and produce an output
 * signal. The output signal is used to observe the selftest response.
 * When the selftest response exceeds the min/max values,
 * the part is deemed to have failed selftest.
 *
 * Input Parameters:
 *   lower - The instance of lower half sensor driver.
 *   filep - The pointer of file, represents each user using the sensor.
 *   arg   - The parameters associated with selftest.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int stk31850_selftest(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             unsigned long arg)
{
  FAR struct stk31850_dev_s * priv = (FAR struct stk31850_dev_s *)lower;
  int ret;

  DEBUGASSERT(lower != NULL);

  /* Process ioctl commands. */

  switch (arg)
    {
      case SNIOC_SIMPLE_CHECK:    /* Simple check tag */
        {
          /* Read device ID. */

          ret = stk31850_readdevid(priv);
        }
        break;

      default:                      /* Other cmd tag */
        {
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}

/* Sensor poll functions */

/****************************************************************************
 * Name: stk31850_worker
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

static void stk31850_worker(FAR void *arg)
{
  FAR struct stk31850_dev_s *priv = arg;
  struct sensor_light light;

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp if this is not a repeat worker. */

  if (priv->wait_cnt == 0)
    {
      light.timestamp = sensor_get_timestamp();
    }

  /* If data is ready or time is out, try to read out and push it. */

  if (stk31850_check_dataready(priv) ||
      priv->wait_cnt >= STK31850_WAIT_COUNT)
    {
      priv->wait_cnt = 0;
      if (stk31850_readlux(priv, &light.light, &light.ir) == 0)
        {
          priv->lower.push_event(priv->lower.priv, &light, sizeof(light));
        }

      /* Set next worker for reading */

      if (priv->activated)
        {
          work_queue(HPWORK, &priv->work, stk31850_worker, priv,
                     priv->interval / USEC_PER_TICK);
        }
    }

  /* If data is not ready, wait several times till time is out. */

  else
    {
      priv->wait_cnt++;
      if (priv->activated)
        {
          work_queue(HPWORK, &priv->work, stk31850_worker, priv,
                     STK31850_WAIT_TIME / USEC_PER_TICK);
        }
      else
        {
          priv->wait_cnt = 0;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stk31850_register
 *
 * Description:
 *   Register the sensor.
 *
 * Input Parameters:
 *   devno   - Sensor device number.
 *   i2c     - I2C master handler.
 *   config  - Interrupt fuctions.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

int stk31850_register(int devno, FAR const struct stk31850_config_s *config)
{
  FAR struct stk31850_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(config != NULL);

  /* Initialize the STK31850 device structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->config = config;
  priv->lower.ops = &g_stk31850_ops;
  priv->lower.type = SENSOR_TYPE_LIGHT;
  priv->lower.uncalibrated = true;
  priv->interval = STK31850_DEFAULT_INTERVAL;
  priv->gain_idx = STK31850_GAIN_SET_MAX;
  priv->lower.nbuffer = CONFIG_SENSORS_STK31850_BUFFER_NUMBER;

  /* Read and verify the deviceid */

  ret = stk31850_readdevid(priv);
  if (ret < 0)
    {
      goto err;
    }

  ret = stk31850_reset(priv);
  if (ret < 0)
    {
      goto err;
    }

  ret = stk31850_setstate(priv, STK31850_DISABLE);
  if (ret < 0)
    {
      goto err;
    }

  /* Register the character driver */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      goto err;
    }

  return ret;

err:
  kmm_free(priv);
  return ret;
}
