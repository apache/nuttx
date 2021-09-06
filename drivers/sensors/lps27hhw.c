/****************************************************************************
 * drivers/sensors/lps27hhw.c
 * Character driver for lps27hhw.
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

#include <nuttx/nuttx.h>
#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wqueue.h>
#include <nuttx/random.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/lps27hhw.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C address len */

#define LPS27HHW_I2C_ADDR_LEN          (7)

/* Bit func define */

#define LPS27HHW_BIT(n)                (1u << (n))

/* Array count func define */

#define LPS27HHW_COUNTOF(arr)          (sizeof(arr) / sizeof((arr)[0]))

/* Roundup func define */

#define LPS27HHW_ROUNDUP(x, esize)     (((x) + (esize) - 1) \
                                       / (esize) * (esize))

/* Eanble/disable define */

#define LPS27HHW_ENABLE                (1u)
#define LPS27HHW_DISABLE               (0u)

/* Data width define */

#define LPS27HHW_PRESSURE_WIDTH        (3)
#define LPS27HHW_TEMP_WIDTH            (2)

/* Reset time define */

#define LPS27HHW_RESET_DELAY           (5)       /* Reset time */
#define LPS27HHW_RESET_MAX_COUNT       (3)       /* Max reset count */

/* Sensor sensitivity define */

#define LPS27HHW_PRESSURE_SENS         (4096.0f) /* LSB/hPa */
#define LPS27HHW_TEMP_SENS             (100.0f)  /* LSB/°C */

/* Device default macros */

#define LPS27HHW_DEFAULT_INTERVAL      (10000)   /* Default interval */
#define LPS27HHW_DEFAULT_LATENCY       (10000)   /* Default latency */
#define LPS27HHW_DEFAULT_FIFOWTM       (1)       /* Default fifo watermark */
#define LPS27HHW_DEFAULT_BUFFER_NUMBER (1)       /* Default buffer number */
#define LPS27HHW_FIFO_SLOTS_NUMBER     (128)     /* Default FIFO slots number */

/* Register map */

#define LPS27HHW_REG_WHOAMI            0x0f      /* WHOAMI register */
#define LPS27HHW_DEVICE_ID             0xb3      /* Device id */

#define LPS27HHW_REG_INT_CFG           0x0b      /* INTERRUPT_CFG register */
#define LPS27HHW_MASK_INT_CFG_LIR      LPS27HHW_BIT(2)
#define LPS27HHW_SHIFT_INT_CFG_LIR     2

#define LPS27HHW_REG_CTRL1             0x10      /* CTRL_REG1 register */
#define LPS27HHW_MASK_CTRL1_ODR        (LPS27HHW_BIT(6) | LPS27HHW_BIT(5) | \
                                       LPS27HHW_BIT(4))
#define LPS27HHW_SHIFT_CTRL1_ODR       4
#define LPS27HHW_MASK_CTRL1_EN_LPFP    LPS27HHW_BIT(3)
#define LPS27HHW_SHIFT_CTRL1_EN_LPFP   3
#define LPS27HHW_MASK_CTRL1_BDU        LPS27HHW_BIT(1)
#define LPS27HHW_SHIFT_CTRL1_BDU       1

#define LPS27HHW_REG_CTRL2             0x11      /* CTRL_REG2 register */
#define LPS27HHW_MASK_CTRL2_BOOT       LPS27HHW_BIT(7)
#define LPS27HHW_SHIFT_CTRL2_BOOT      7
#define LPS27HHW_MASK_CTRL2_INT_HL     LPS27HHW_BIT(6)
#define LPS27HHW_SHIFT_CTRL2_INT_HL    6
#define LPS27HHW_MASK_CTRL2_PP_OD      LPS27HHW_BIT(5)
#define LPS27HHW_SHIFT_CTRL2_PP_OD     5
#define LPS27HHW_MASK_CTRL2_SWRESET    LPS27HHW_BIT(2)
#define LPS27HHW_SHIFT_CTRL2_SWRESET   2

#define LPS27HHW_REG_CTRL3             0x12      /* CTRL_REG3 register */
#define LPS27HHW_MASK_CTRL3_INT_FFULL  LPS27HHW_BIT(5)
#define LPS27HHW_SHIFT_CTRL3_INT_FFULL 5
#define LPS27HHW_MASK_CTRL3_INT_FWTM   LPS27HHW_BIT(4)
#define LPS27HHW_SHIFT_CTRL3_INT_FWTM  4
#define LPS27HHW_MASK_CTRL3_INT_FOVER  LPS27HHW_BIT(3)
#define LPS27HHW_SHIFT_CTRL3_INT_FOVER 3
#define LPS27HHW_MASK_CTRL3_DRDY       LPS27HHW_BIT(2)
#define LPS27HHW_SHIFT_CTRL3_DRDY      2

#define LPS27HHW_REG_FCTRL             0x13      /* FIFO_CTRL register */
#define LPS27HHW_MASK_FCTRL_STOP_WTM   LPS27HHW_BIT(3)
#define LPS27HHW_SHIFT_FCTRL_STOP_WTM  3
#define LPS27HHW_MASK_FCTRL_TRIG_MODE  LPS27HHW_BIT(2)
#define LPS27HHW_MASK_FCTRL_F_MODE     (LPS27HHW_BIT(1) | LPS27HHW_BIT(0))
#define LPS27HHW_SHIFT_FCTRL_F_MODE    0

#define LPS27HHW_REG_FIFO_WTM          0x14      /* FIFO_WTM register */
#define LPS27HHW_MASK_FIFO_WTM         0xff
#define LPS27HHW_SHIFT_FIFO_WTM        0

#define LPS27HHW_REG_F_STATUS1         0x25      /* FIFO_STATUS1 register */
#define LPS27HHW_MASK_F_STATUS1_FSS    0xff
#define LPS27HHW_SHIFT_F_STATUS1_FSS   0

#define LPS27HHW_REG_STATUS            0x27      /* STATUS register */
#define LPS27HHW_MASK_STATUS_T_OR      LPS27HHW_BIT(5)
#define LPS27HHW_SHIFT_STATUS_T_OR     5
#define LPS27HHW_MASK_STATUS_P_OR      LPS27HHW_BIT(4)
#define LPS27HHW_SHIFT_STATUS_P_OR     4
#define LPS27HHW_MASK_STATUS_T_DA      LPS27HHW_BIT(1)
#define LPS27HHW_SHIFT_STATUS_T_DA     1
#define LPS27HHW_MASK_STATUS_P_DA      LPS27HHW_BIT(0)
#define LPS27HHW_SHIFT_STATUS_P_DA     0

#define LPS27HHW_REG_PRESS_OUT_XL      0x28      /* PRESS_OUT_XL register */

#define LPS27HHW_REG_TEMP_OUT_L        0x2b      /* TEMP_OUT_L register */

#define LPS27HHW_REG_F_OUT_PRESS_XL    0x78      /* FIFO_DATA_OUT_PRESS_XL register */

#define LPS27HHW_REG_FIFO_OUT_TEMP_L   0x7b      /* FIFO_DATA_OUT_TEMP_L register */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Sensor ODR enum */

enum lps27hhw_odr_e
{
  LPS27HHW_ONE_SHOT,                             /* Used for powerdwon mode */
  LPS27HHW_1_Hz,                                 /* Sampling freq is 1hz */
  LPS27HHW_10_Hz,                                /* Sampling freq is 10hz */
  LPS27HHW_25_Hz,                                /* Sampling freq is 25hz */
  LPS27HHW_50_Hz,                                /* Sampling freq is 50hz */
  LPS27HHW_75_Hz,                                /* Sampling freq is 75hz */
  LPS27HHW_100_Hz,                               /* Sampling freq is 100hz */
  LPS27HHW_200_Hz                                /* Sampling freq is 200hz */
};

/* Sensor INT type enum */

enum lps27hhw_int_e
{
  LPS27HHW_INT_F_FULL,                           /* FIFO full int type */
  LPS27HHW_INT_F_WTM,                            /* FIFO watermark int type */
  LPS27HHW_INT_F_OVR,                            /* FIFO overrun int type */
  LPS27HHW_INT_DRDY                              /* Date ready int type */
};

/* Sensor mode enum */

enum lps27hhw_mode_e
{
  LPS27HHW_MODE_BYPASS,                          /* Bypass mode */
  LPS27HHW_MODE_FIFO,                            /* FIFO mode */
  LPS27HHW_MODE_STREAM,                          /* Dynamic-Stream mode */
  LPS27HHW_MODE_BYPASS_TO_FIFO,                  /* Continuous-to-FIFO mode */
  LPS27HHW_MODE_BYPASS_TO_STREAM,                /* Bypass-to-Continuous mode */
  LPS27HHW_MODE_STREAM_TO_FIFO,                  /* Bypass-to-FIFO mode */
  LPS27HHW_MODE_POWER_DOWN                       /* Powerdown mode */
};

/* Sensor INT polarity enum */

enum lps27hhw_polarity_e
{
  LPS27HHW_ACTIVE_HIGH,                          /* Int pin active-high */
  LPS27HHW_ACTIVE_LOW                            /* Int pin active-low */
};

/* Sensor INT latch enum */

enum lps27hhw_lir_e
{
  LPS27HHW_INT_PULSED,                           /* Int request pulsed */
  LPS27HHW_INT_LATCHED                           /* Int request latched */
};

/* Sensor PP_OD enum */

enum lps27hhw_ppod_e
{
  LPS27HHW_PUSH_PULL,                            /* Int pad push-pull */
  LPS27HHW_OPEN_DRAIN                            /* Int pad open-drain */
};

/* Sensor struct */

struct lps27hhw_sensor_s
{
  struct sensor_lowerhalf_s lower;               /* Lower half sensor driver */
  bool                      fifoen;              /* Sensor fifo enable */
  bool                      activated;           /* Sensor working state */
  bool                      interval_updated;    /* Sensor interval updated flag */
  bool                      latency_updated;     /* Sensor latency updated flag */
  uint32_t                  interval;            /* Sensor interval */
  uint32_t                  latency;             /* Sensor batch latency */
  uint32_t                  fifowtm;             /* Sensor fifo water marker */
  uint64_t                  timestamp;           /* Units is microseconds */
};

/* Device struct */

struct lps27hhw_dev_s
{
  struct lps27hhw_sensor_s           dev;        /* Sensor struct */
  FAR const struct lps27hhw_config_s *config;    /* The board config */
  struct work_s                      work;       /* Interrupt handler */
};

/* Sensor ODR struct */

struct lps27hhw_odr_s
{
  enum lps27hhw_odr_e regval;                    /* The data register */
  uint32_t            interval;                  /* The interval in us */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C functions */

static int lps27hhw_read(FAR struct lps27hhw_dev_s *priv, uint8_t regaddr,
                         FAR uint8_t *value, uint8_t len);
static int lps27hhw_readreg(FAR struct lps27hhw_dev_s *priv, uint8_t regaddr,
                            FAR uint8_t *regval);
static int lps27hhw_writereg(FAR struct lps27hhw_dev_s *priv,
                             uint8_t regaddr, uint8_t regval);
static int lps27hhw_updatereg(FAR struct lps27hhw_dev_s *priv,
                              uint8_t regaddr, uint8_t mask,
                              uint8_t regval);

/* Sensor handle functions */

/* Lps27hhw handle functions */

static int lps27hhw_readdevid(FAR struct lps27hhw_dev_s *priv);
static int lps27hhw_reset(FAR struct lps27hhw_dev_s *priv);
static uint8_t lps27hhw_findodr(FAR struct lps27hhw_dev_s *priv,
                                FAR unsigned int *interval);
static int lps27hhw_setodr(FAR struct lps27hhw_dev_s *priv,
                           enum lps27hhw_odr_e odr);
static int lps27hhw_setlpfilter(FAR struct lps27hhw_dev_s *priv,
                                uint8_t value);
static int lps27hhw_setupdate(FAR struct lps27hhw_dev_s *priv,
                              uint8_t value);
static int lps27hhw_setint(FAR struct lps27hhw_dev_s *priv,
                           enum lps27hhw_int_e type);
static int lps27hhw_setmode(FAR struct lps27hhw_dev_s *priv,
                            enum lps27hhw_mode_e mode);
static int lps27hhw_isready(FAR struct lps27hhw_dev_s *priv,
                            FAR uint8_t *value);
static int lps27hhw_getdata(FAR struct lps27hhw_dev_s *priv,
                            FAR struct sensor_event_baro *baro);
static int lps27hhw_read_push(FAR struct lps27hhw_dev_s *priv, bool push);
static int lps27hhw_initchip(FAR struct lps27hhw_dev_s *priv);

/* Sensor ops functions */

static int lps27hhw_batch(FAR struct sensor_lowerhalf_s *lower,
                          FAR unsigned int *latency_us);
static int lps27hhw_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR unsigned int *period_us);
static int lps27hhw_activate(FAR struct sensor_lowerhalf_s *lower,
                             bool enable);

/* Sensor interrupt functions */

static int lps27hhw_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                      ioe_pinset_t pinset, FAR void *arg);
static void lps27hhw_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Sensor ops */

static const struct sensor_ops_s g_lps27hhw_ops =
{
  .activate = lps27hhw_activate,         /* Enable/disable sensor */
  .set_interval = lps27hhw_set_interval, /* Set output data period */
  .batch = lps27hhw_batch                /* Set maximum report latency */
};

/* Sensor ODR */

static const struct lps27hhw_odr_s g_lps27hhw_odr[] =
{
  {LPS27HHW_1_Hz,   1000000},            /* Sampling interval is 1000ms */
  {LPS27HHW_10_Hz,  100000},             /* Sampling interval is 100ms */
  {LPS27HHW_25_Hz,  40000},              /* Sampling interval is 40ms */
  {LPS27HHW_50_Hz,  20000},              /* Sampling interval is 20ms */
  {LPS27HHW_75_Hz,  13333},              /* Sampling interval is 13.3ms */
  {LPS27HHW_100_Hz, 10000},              /* Sampling interval is 10ms */
  {LPS27HHW_200_Hz, 5000},               /* Sampling interval is 5ms */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* I2C functions */

/****************************************************************************
 * Name: lps27hhw_read
 *
 * Description:
 *   Read data.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Register address.
 *   value   - Burst read value.
 *   len     - Burst read length.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_read(FAR struct lps27hhw_dev_s *priv, uint8_t regaddr,
                         FAR uint8_t *value, uint8_t len)
{
  struct i2c_config_s config;

  /* Set up the I2C configuration */

  config.frequency = priv->config->freq;
  config.address   = priv->config->addr;
  config.addrlen   = LPS27HHW_I2C_ADDR_LEN;

  /* I2c read bytes */

  return i2c_writeread(priv->config->i2c, &config, &regaddr,
                       sizeof(regaddr), value, len);
}

/****************************************************************************
 * Name: lps27hhw_readreg
 *
 * Description:
 *   Read data from a register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Register address.
 *   regval  - Register value.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_readreg(FAR struct lps27hhw_dev_s *priv, uint8_t regaddr,
                            FAR uint8_t *regval)
{
  struct i2c_config_s config;

  /* Set up the I2C configuration */

  config.frequency = priv->config->freq;
  config.address   = priv->config->addr;
  config.addrlen   = LPS27HHW_I2C_ADDR_LEN;

  /* I2c read one byte */

  return i2c_writeread(priv->config->i2c, &config, &regaddr,
                       sizeof(regaddr), regval, sizeof(*regval));
}

/****************************************************************************
 * Name: lps27hhw_writereg
 *
 * Description:
 *   Write data to a register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Register address.
 *   regval  - Register value.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_writereg(FAR struct lps27hhw_dev_s *priv,
                             uint8_t regaddr, uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t tx_buff[2];

  /* Set up the I2C configuration */

  config.frequency = priv->config->freq;
  config.address   = priv->config->addr;
  config.addrlen   = LPS27HHW_I2C_ADDR_LEN;

  /* Setup tx buffer */

  tx_buff[0] = regaddr;
  tx_buff[1] = regval;

  /* I2c reg write one byte */

  return i2c_write(priv->config->i2c, &config, tx_buff, 2);
}

/****************************************************************************
 * Name: lps27hhw_updatereg
 *
 * Description:
 *   Update data for a register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   regaddr - Register address.
 *   regval  - Register value.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_updatereg(FAR struct lps27hhw_dev_s *priv,
                              uint8_t regaddr, uint8_t mask,
                              uint8_t regval)
{
  uint8_t old_value;
  uint8_t new_value;
  int ret;

  /* Read old value of the register */

  ret = lps27hhw_readreg(priv, regaddr, &old_value);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read register\n");
      return ret;
    }

  /* Setup new value of the register */

  new_value = (old_value & ~mask) | (regval & mask);
  if (new_value == old_value)
    {
      return OK;
    }

  /* Write new value to the register */

  return lps27hhw_writereg(priv, regaddr, new_value);
}

/* Sensor handle functions */

/****************************************************************************
 * Name: lps27hhw_readdevid
 *
 * Description:
 *   Read the device ID.
 *
 * Input Parameters:
 *   priv - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int lps27hhw_readdevid(FAR struct lps27hhw_dev_s *priv)
{
  uint8_t regval;
  int ret;

  ret = lps27hhw_readreg(priv, LPS27HHW_REG_WHOAMI, &regval);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read device id\n");
      return ret;
    }

  if (regval != LPS27HHW_DEVICE_ID)
    {
      snerr("ERROR: Wrong device ID: %x\n", regval);
      ret = -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: lps27hhw_reset
 *
 * Description:
 *   Software reset the sensor.
 *
 * Input Parameters:
 *   priv - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_reset(FAR struct lps27hhw_dev_s *priv)
{
  uint8_t count = 0;
  uint8_t regval;
  uint8_t status;
  int ret;

  do
    {
      /* software reset */

      ret = lps27hhw_updatereg(priv, LPS27HHW_REG_CTRL2,
                               LPS27HHW_MASK_CTRL2_BOOT,
                               LPS27HHW_ENABLE
                               << LPS27HHW_SHIFT_CTRL2_BOOT);
      if (ret < 0)
        {
          snerr("ERROR: Failed to reset lps27hhw\n");
          return ret;
        }

      /* Delay for reset booting */

      up_mdelay(LPS27HHW_RESET_DELAY);

      /* Check reset is done */

      ret = lps27hhw_readreg(priv, LPS27HHW_REG_CTRL2, &regval);
      if (ret < 0)
        {
          snerr("ERROR: Failed to read reset status\n");
          return ret;
        }

      status = (regval && LPS27HHW_MASK_CTRL2_SWRESET)
               >> LPS27HHW_SHIFT_CTRL2_SWRESET;
      count++;

      /* Check reset is timeout */

      if (count > LPS27HHW_RESET_MAX_COUNT)
        {
          snerr("ERROR: Reset timeout: %d\n", count);
          return -EIO;
        }
    }
  while (status);

  return ret;
}

/****************************************************************************
 * Name: lps27hhw_findodr
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

static uint8_t lps27hhw_findodr(FAR struct lps27hhw_dev_s *priv,
                                FAR unsigned int *interval)
{
  uint8_t idx;

  for (idx = 0; idx < LPS27HHW_COUNTOF(g_lps27hhw_odr); idx++)
    {
      if (*interval >= g_lps27hhw_odr[idx].interval)
        {
          return idx;
        }
    }

  return (LPS27HHW_COUNTOF(g_lps27hhw_odr) - 1);
}

/****************************************************************************
 * Name: lps27hhw_setodr
 *
 * Description:
 *   Set odr for sensor.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   odr  - Set odr.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_setodr(FAR struct lps27hhw_dev_s *priv,
                           enum lps27hhw_odr_e odr)
{
  int ret;

  ret = lps27hhw_updatereg(priv, LPS27HHW_REG_CTRL1,
                         LPS27HHW_MASK_CTRL1_ODR, odr
                         << LPS27HHW_SHIFT_CTRL1_ODR);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set data rate\n");
    }

  return ret;
}

/****************************************************************************
 * Name: lps27hhw_setlpfilter
 *
 * Description:
 *   Sensor low-pass filter selection on output.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Set value.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_setlpfilter(FAR struct lps27hhw_dev_s *priv,
                                uint8_t value)
{
  int ret;

  ret = lps27hhw_updatereg(priv, LPS27HHW_REG_CTRL1,
                         LPS27HHW_MASK_CTRL1_EN_LPFP, value
                         << LPS27HHW_SHIFT_CTRL1_EN_LPFP);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set low-pass filter\n");
    }

  return ret;
}

/****************************************************************************
 * Name: lps27hhw_setupdate
 *
 * Description:
 *   Set block data update mode.
 *   0: continuous update;
 *   1: output registers are not updated until MSB and LSB have been read.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - The mode of block data update.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_setupdate(FAR struct lps27hhw_dev_s *priv, uint8_t value)
{
  int ret;

  ret = lps27hhw_updatereg(priv, LPS27HHW_REG_CTRL1,
                           LPS27HHW_MASK_CTRL1_BDU, value
                           << LPS27HHW_SHIFT_CTRL1_BDU);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set block data update\n");
    }

  return ret;
}

/****************************************************************************
 * Name: lps27hhw_setint
 *
 * Description:
 *   Set interrupt type for the sensor.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   type - INT type.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_setint(FAR struct lps27hhw_dev_s *priv,
                           enum lps27hhw_int_e type)
{
  int ret;

  /* Clear int types beforing setting */

  ret = lps27hhw_writereg(priv, LPS27HHW_REG_CTRL3, 0x00);
  if (ret < 0)
    {
      snerr("ERROR: Failed to clear int types\n");
      return ret;
    }

  /* Set interrupt type */

  switch (type)
    {
      case LPS27HHW_INT_F_FULL:
        {
          ret = lps27hhw_updatereg(priv, LPS27HHW_REG_CTRL3,
                                   LPS27HHW_MASK_CTRL3_INT_FFULL,
                                   LPS27HHW_ENABLE
                                   << LPS27HHW_SHIFT_CTRL3_INT_FFULL);
          if (ret < 0)
            {
              snerr("ERROR: Failed to set full FIFO int\n");
              return ret;
            }
        }
        break;

      case LPS27HHW_INT_F_WTM:
        {
          ret = lps27hhw_updatereg(priv, LPS27HHW_REG_CTRL3,
                                   LPS27HHW_MASK_CTRL3_INT_FWTM,
                                   LPS27HHW_ENABLE
                                   << LPS27HHW_SHIFT_CTRL3_INT_FWTM);
          if (ret < 0)
           {
              snerr("ERROR: Failed to set FIFO watermark int\n");
              return ret;
            }
        }
        break;

      case LPS27HHW_INT_F_OVR:
        {
          ret = lps27hhw_updatereg(priv, LPS27HHW_REG_CTRL3,
                                   LPS27HHW_MASK_CTRL3_INT_FOVER,
                                   LPS27HHW_ENABLE
                                   << LPS27HHW_SHIFT_CTRL3_INT_FOVER);
          if (ret < 0)
            {
              snerr("ERROR: Failed to set fifo overflow int\n");
              return ret;
            }
        }
        break;

      case LPS27HHW_INT_DRDY:
        {
          ret = lps27hhw_updatereg(priv, LPS27HHW_REG_CTRL3,
                                   LPS27HHW_MASK_CTRL3_DRDY,
                                   LPS27HHW_ENABLE
                                   << LPS27HHW_SHIFT_CTRL3_DRDY);
          if (ret < 0)
            {
              snerr("ERROR: Failed to set data ready int\n");
              return ret;
            }
        }
        break;

      default:
        {
          snerr("ERROR: Int type is not supported\n");
          return -EINVAL;
        }
    }

  /* Configure FIFO watermark settings related to int type */

  ret = lps27hhw_updatereg(priv, LPS27HHW_REG_FCTRL,
                           LPS27HHW_MASK_FCTRL_STOP_WTM,
                           priv->dev.fifoen
                           << LPS27HHW_SHIFT_FCTRL_STOP_WTM);
  if (ret < 0)
    {
      snerr("ERROR: Failed to enable/disable FIFO watermark\n");
      return ret;
    }

  ret = lps27hhw_updatereg(priv, LPS27HHW_REG_FIFO_WTM,
                           LPS27HHW_MASK_FIFO_WTM,
                           priv->dev.fifowtm
                           << LPS27HHW_SHIFT_FIFO_WTM);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set FIFO watermark\n");
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: lsp27hhw_setmode
 *
 * Description:
 *   Set working mode.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   mode - Working mode.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_setmode(FAR struct lps27hhw_dev_s *priv,
                            enum lps27hhw_mode_e mode)
{
  int ret;

  switch (mode)
    {
      case LPS27HHW_MODE_BYPASS:           /* Share the same setting */
      case LPS27HHW_MODE_FIFO:             /* Share the same setting */
      case LPS27HHW_MODE_STREAM:           /* Share the same setting */
      case LPS27HHW_MODE_BYPASS_TO_FIFO:   /* Share the same setting */
      case LPS27HHW_MODE_BYPASS_TO_STREAM: /* Share the same setting */
      case LPS27HHW_MODE_STREAM_TO_FIFO:   /* Share the same setting */
        {
          /* FIFO mode selection */

          ret = lps27hhw_updatereg(priv, LPS27HHW_REG_FCTRL,
                                   LPS27HHW_MASK_FCTRL_F_MODE |
                                   LPS27HHW_MASK_FCTRL_TRIG_MODE, mode
                                   << LPS27HHW_SHIFT_FCTRL_F_MODE);
          if (ret < 0)
            {
              snerr("ERROR: Failed to set fifo mode\n");
              return ret;
            }
        }
        break;

      case LPS27HHW_MODE_POWER_DOWN:
        {
          /* Odr used for powerdown mode */

          ret = lps27hhw_setodr(priv, LPS27HHW_ONE_SHOT);
          if (ret < 0)
            {
              snerr("ERROR: Failed to set powwerdown mode\n");
              return ret;
            }
        }
        break;

      default:
        {
          snerr("ERROR: Mode type is not supported\n");
          return -EINVAL;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: lps27hhw_isready
 *
 * Description:
 *   Read the sensor data ready flag.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Data ready flag.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_isready(FAR struct lps27hhw_dev_s *priv,
                            FAR uint8_t *value)
{
  uint8_t status;
  int ret;

  if (priv->dev.fifoen)
    {
      ret = lps27hhw_readreg(priv, LPS27HHW_REG_F_STATUS1, &status);
      if (ret < 0)
        {
          snerr("ERROR: Failed to read fifo status1\n");
          return ret;
        }

      *value = (status & LPS27HHW_MASK_F_STATUS1_FSS)
               >> LPS27HHW_SHIFT_F_STATUS1_FSS;
    }
  else
    {
      ret = lps27hhw_readreg(priv, LPS27HHW_REG_STATUS, &status);
      if (ret < 0)
        {
          snerr("ERROR: Failed to read status\n");
          return ret;
        }

      *value = (((status & LPS27HHW_MASK_STATUS_T_DA)
               >> LPS27HHW_SHIFT_STATUS_T_DA)
               || ((status & LPS27HHW_MASK_STATUS_T_OR)
               >> LPS27HHW_SHIFT_STATUS_T_OR)
               || ((status & LPS27HHW_MASK_STATUS_P_DA)
               >> LPS27HHW_SHIFT_STATUS_P_DA)
               || ((status & LPS27HHW_MASK_STATUS_P_OR)
               >> LPS27HHW_SHIFT_STATUS_P_OR))
               ? LPS27HHW_ENABLE : LPS27HHW_DISABLE;
    }

  return ret;
}

/****************************************************************************
 * Name: lps27hhw_getdata
 *
 * Description:
 *   Read the pressure and temperature data.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   baro - Store event data.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_getdata(FAR struct lps27hhw_dev_s *priv,
                            FAR struct sensor_event_baro *baro)
{
  uint8_t pressure_buff[LPS27HHW_PRESSURE_WIDTH];
  uint8_t temperature_buff[LPS27HHW_TEMP_WIDTH];
  int32_t pressure;
  int32_t temperature;
  int ret;

  if (true == priv->dev.fifoen)
    {
      ret = lps27hhw_read(priv, LPS27HHW_REG_F_OUT_PRESS_XL,
                          pressure_buff, LPS27HHW_PRESSURE_WIDTH);
      if (ret < 0)
        {
          snerr("ERROR: Failed to get pressure\n");
          return ret;
        }

      ret = lps27hhw_read(priv, LPS27HHW_REG_FIFO_OUT_TEMP_L,
                          temperature_buff, LPS27HHW_TEMP_WIDTH);
      if (ret < 0)
        {
          snerr("ERROR: Failed to get temperature\n");
          return ret;
        }
    }
  else
    {
      ret = lps27hhw_read(priv, LPS27HHW_REG_PRESS_OUT_XL,
                          pressure_buff, LPS27HHW_PRESSURE_WIDTH);
      if (ret < 0)
        {
          snerr("ERROR: Failed to get fifo pressure\n");
          return ret;
        }

      ret = lps27hhw_read(priv, LPS27HHW_REG_TEMP_OUT_L,
                          temperature_buff, LPS27HHW_TEMP_WIDTH);
      if (ret < 0)
        {
          snerr("ERROR: Failed to get fifo temperature\n");
          return ret;
        }
    }

  pressure = (pressure_buff[0] & 0x000000ff) | ((pressure_buff[1] << 8)
             & 0x0000ff00) | ((pressure_buff[2] << 16) & 0x00ff0000);
  temperature = (temperature_buff[0] & 0x00ff) | ((temperature_buff[1] << 8)
                & 0xff00);

  baro->pressure = pressure / LPS27HHW_PRESSURE_SENS;
  baro->temperature = temperature / LPS27HHW_TEMP_SENS;

  return ret;
}

/****************************************************************************
 * Name: lps27hhw_read_push
 *
 * Description:
 *   Read the pressure and temperature data.
 *   Then push data to upper half.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   push - Flag of pushing data to upper half.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_read_push(FAR struct lps27hhw_dev_s *priv, bool push)
{
  struct sensor_event_baro baro[LPS27HHW_FIFO_SLOTS_NUMBER];
  uint8_t status;
  int i = 0;
  int ret;

  do
    {
      /* Check data is ready */

      ret = lps27hhw_isready(priv, &status);
      if (ret < 0)
        {
          snerr("ERROR: Failed to check is ready\n");
          return ret;
        }

      /* If ready, get data */

      if (status)
        {
          ret = lps27hhw_getdata(priv,
                                 (FAR struct sensor_event_baro *)
                                 &baro[i]);
          if (ret < 0)
            {
              snerr("ERROR: Failed to get data\n");
              return ret;
            }

          baro[i].timestamp = priv->dev.timestamp -
                              (status - 1) * priv->dev.interval;
          i++;
        }

      /* Check reading is over FIFO watermark */

      if (i > LPS27HHW_FIFO_SLOTS_NUMBER)
        {
          snerr("ERROR: Reading is  over max FIFO slots\n");
          return -EIO;
        }
    }
  while (status);

  /* If there is any data, check the data is needed pushing to upper */

  if (push && (i > 0))
    {
      priv->dev.lower.push_event(priv->dev.lower.priv, baro,
                                 i * sizeof(struct sensor_event_baro));
    }

  return ret;
}

/****************************************************************************
 * Name: lps27hhw_initchip
 *
 * Description:
 *   Initialized chip and enter into lowpower mode.
 *
 * Input Parameters:
 *   priv - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_initchip(FAR struct lps27hhw_dev_s *priv)
{
  int ret;

  /* Read chip id */

  ret = lps27hhw_readdevid(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to reading chip id\n");
      return ret;
    }

  /* Perform chip reset */

  ret = lps27hhw_reset(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to reset chip\n");
      return ret;
    }

  /* Set low-pass filter */

  ret = lps27hhw_setlpfilter(priv, LPS27HHW_DISABLE);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set low-pass filter\n");
      return ret;
    }

  /* Configure int pin:
   * pulsed, active low and open-drain.
   */

  ret = lps27hhw_updatereg(priv, LPS27HHW_REG_INT_CFG,
                           LPS27HHW_MASK_INT_CFG_LIR,
                           LPS27HHW_INT_PULSED
                           << LPS27HHW_SHIFT_INT_CFG_LIR);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set int pin: puslsed\n");
      return ret;
    }

  ret = lps27hhw_updatereg(priv, LPS27HHW_REG_CTRL2,
                           LPS27HHW_MASK_CTRL2_INT_HL,
                           LPS27HHW_ACTIVE_LOW
                           << LPS27HHW_SHIFT_CTRL2_INT_HL);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure int pint polarity\n");
      return ret;
    }

  ret = lps27hhw_updatereg(priv, LPS27HHW_REG_CTRL2,
                           LPS27HHW_MASK_CTRL2_PP_OD,
                           LPS27HHW_OPEN_DRAIN
                           << LPS27HHW_SHIFT_CTRL2_PP_OD);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure int pint mode\n");
      return ret;
    }

  /* Set interrupt type: data ready */

  ret = lps27hhw_setint(priv, LPS27HHW_INT_DRDY);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set int type: data ready\n");
      return ret;
    }

  /* Set block data update: output registers not updated
   * until MSB and LSB have been read
   */

  ret = lps27hhw_setupdate(priv, LPS27HHW_ENABLE);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set block data update\n");
      return ret;
    }

  /* Set chip into lowpower mode */

  ret = lps27hhw_setmode(priv, LPS27HHW_MODE_POWER_DOWN);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set mode: powerdown\n");
      return ret;
    }

  return ret;
}

/* Sensor ops functions */

/****************************************************************************
 * Name: lps27hhw_batch
 *
 * Description:
 *   Set sensor's maximum report latency in microseconds.
 *
 * Input Parameters:
 *   lower      - The instance of lower half sensor driver.
 *   latency_us - the time between batch data, in us. It may by overwrite
 *                by lower half driver.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int lps27hhw_batch(FAR struct sensor_lowerhalf_s *lower,
                          FAR unsigned int *latency_us)
{
  FAR struct lps27hhw_dev_s *priv = (FAR struct lps27hhw_dev_s *)lower;
  uint32_t max_latency;
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL && latency_us != NULL);

  /* Verify batch latency */

  max_latency = priv->dev.lower.batch_number * priv->dev.interval;
  if (*latency_us > max_latency)
    {
      *latency_us = max_latency;
    }
  else if ((*latency_us < priv->dev.interval) && *latency_us > 0)
    {
      *latency_us = priv->dev.interval;
    }

  /* Find best matching FIFO watermark */

  priv->dev.fifowtm = LPS27HHW_ROUNDUP(*latency_us, priv->dev.interval)
                      / priv->dev.interval;
  *latency_us = priv->dev.fifowtm * priv->dev.interval;

  /* Check latency is updated */

  if (priv->dev.latency == *latency_us)
    {
      priv->dev.latency_updated = false;
      return OK;
    }

  /* If latency is updated, renew chip mode and int type */

  priv->dev.latency_updated = true;
  priv->dev.latency = *latency_us;
  priv->dev.fifoen = priv->dev.fifowtm > 1 ? true : false;

  ret = lps27hhw_setmode(priv, priv->dev.fifoen
                         ? LPS27HHW_MODE_STREAM
                         : LPS27HHW_MODE_BYPASS);
  if (ret < 0)
    {
      snerr("ERROR: Failed to renew chip mode\n");
      return ret;
    }

  ret = lps27hhw_setint(priv, priv->dev.fifoen
                        ? LPS27HHW_INT_F_WTM :
                        LPS27HHW_INT_DRDY);
  if (ret < 0)
    {
      snerr("ERROR: Failed to renew int type\n");
    }

  return ret;
}

/****************************************************************************
 * Name: lps27hhw_set_interval
 *
 * Description:
 *   Set the sensor output data period in microseconds for a given sensor.
 *   If *period_us > max_delay it will be truncated to max_delay and if
 *   *period_us < min_delay it will be replaced by min_delay.
 *
 * Input Parameters:
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

static int lps27hhw_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR unsigned int *period_us)
{
  FAR struct lps27hhw_dev_s *priv = (FAR struct lps27hhw_dev_s *)lower;
  uint8_t idx;
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL && period_us != NULL);

  /* Find best matching interval */

  idx = lps27hhw_findodr(priv, period_us);

  /* Check interval is updated */

  if (priv->dev.interval == g_lps27hhw_odr[idx].interval)
    {
      priv->dev.interval_updated = false;
      return OK;
    }

  /* If interval is updated, renew odr */

  priv->dev.interval_updated = true;
  priv->dev.interval = g_lps27hhw_odr[idx].interval;

  ret = lps27hhw_setodr(priv, g_lps27hhw_odr[idx].regval);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set odr\n");
    }

  return ret;
}

/****************************************************************************
 * Name: lps27hhw_activate
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   lower  - The instance of lower half sensor driver
 *   enable - true(enable) and false(disable)
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int lps27hhw_activate(FAR struct sensor_lowerhalf_s *lower,
                             bool enable)
{
  FAR struct lps27hhw_dev_s *priv = (FAR struct lps27hhw_dev_s *)lower;
  uint8_t idx;
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Check activated flag is updated */

  if (priv->dev.activated == enable)
    {
      return OK;
    }

  /* If activated flag is updated, readout data register or FIFO.
   * If enabled, readout but not push is used for clearing data.
   * If disabled, readout and push to upper before into lowpower mode.
   */

  ret = lps27hhw_read_push(priv, !enable);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read push\n");
      return ret;
    }

  if (enable)
    {
      /* If enabled, check interval and latency are updated  */

      /* If interval is not updated, set odr */

      if (!priv->dev.interval_updated)
        {
          idx = lps27hhw_findodr(priv, &(priv->dev.interval));
          ret = lps27hhw_setodr(priv, g_lps27hhw_odr[idx].regval);
          if (ret < 0)
            {
              snerr("ERROR: Failed to set odr\n");
              return ret;
            }
        }
      else
        {
          /* If interval is updated, Clear flag */

          priv->dev.interval_updated = false;
        }

      /* If latency is not updated, set chip mode and int type */

      if (!(priv->dev.latency_updated))
        {
          ret = lps27hhw_setmode(priv, priv->dev.fifoen
                                 ? LPS27HHW_MODE_STREAM
                                 : LPS27HHW_MODE_BYPASS);
          if (ret < 0)
            {
              snerr("ERROR: Failed to renew chip mode\n");
              return ret;
            }

          ret = lps27hhw_setint(priv, priv->dev.fifoen
                                ? LPS27HHW_INT_F_WTM :
                                LPS27HHW_INT_DRDY);
          if (ret < 0)
            {
              snerr("ERROR: Failed to renew int type\n");
              return ret;
            }
        }
      else
        {
          /* If latency is updated, Clear flag */

          priv->dev.latency_updated = false;
        }
    }
  else
    {
      /* If disabled, set chip enter into lowpower mode */

      ret = lps27hhw_setmode(priv, LPS27HHW_MODE_POWER_DOWN);
      if (ret < 0)
        {
          snerr("ERROR: Failed to set chip powerdown\n");
          return ret;
        }
    }

  /* Enable/disable interrupt */

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                  IOEXPANDER_OPTION_INTCFG,
                  enable ? IOEXPANDER_VAL_FALLING : IOEXPANDER_VAL_DISABLE);
  priv->dev.activated = enable;

  return ret;
}

/* Sensor interrupt functions */

/****************************************************************************
 * Name: lps27hhw_interrupt_handler
 *
 * Description:
 *   Handle the sensor interrupt.
 *
 * Input Parameters:
 *   dev     - ioexpander device.
 *   pinset  - Interrupt pin.
 *   arg     - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int lps27hhw_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                      ioe_pinset_t pinset, FAR void *arg)
{
  /* This function should be called upon a rising edge on the LSM6DSO
   * new data interrupt pin since it signals that new data has
   * been measured.
   */

  FAR struct lps27hhw_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp */

  priv->dev.timestamp = sensor_get_timestamp();

  /* Task the worker with retrieving the latest sensor data. We should not
   * do this in a interrupt since it might take too long. Also we cannot lock
   * the I2C bus from within an interrupt.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  work_queue(LPWORK, &priv->work, lps27hhw_worker, priv, 0);

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                  IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_DISABLE);

  return OK;
}

/****************************************************************************
 * Name: lps27hhw_worker
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

static void lps27hhw_worker(FAR void *arg)
{
  FAR struct lps27hhw_dev_s *priv = arg;

  /* Sanity check */

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Enable interrupt */

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                  IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_FALLING);

  /* Read the latest sensor data and push to uppe half */

  lps27hhw_read_push(priv, true);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lps27hhw_register
 *
 * Description:
 *   Register the LPS27HHW character device as 'devno'
 *
 * Input Parameters:
 *   devno  - The device number, used to build the device path
 *              as /dev/sensor/baroN
 *   config - The board config function for the device.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int lps27hhw_register(int devno, FAR const struct lps27hhw_config_s *config)
{
  FAR struct lps27hhw_dev_s *priv;
  void *ioephandle;
  int ret;

  /* Sanity check */

  DEBUGASSERT(config != NULL);

  /* Initialize the LPS27HHW device structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->dev.lower.type          = SENSOR_TYPE_BAROMETER;
  priv->dev.lower.buffer_number = LPS27HHW_DEFAULT_BUFFER_NUMBER;
  priv->dev.lower.batch_number  = LPS27HHW_FIFO_SLOTS_NUMBER;
  priv->dev.lower.ops           = &g_lps27hhw_ops;
  priv->dev.interval            = LPS27HHW_DEFAULT_INTERVAL;
  priv->dev.latency             = LPS27HHW_DEFAULT_LATENCY;
  priv->dev.fifowtm             = LPS27HHW_DEFAULT_FIFOWTM;
  priv->config                  = config;

  /* Initialize chip and enter into lowpower mode */

  ret = lps27hhw_initchip(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to init chip: %d\n", ret);
      goto err;
    }

  /* Interrupt register */

  ret = IOEXP_SETDIRECTION(priv->config->ioedev, priv->config->pin,
                           IOEXPANDER_DIRECTION_IN_PULLUP);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set direction: %d\n", ret);
      goto err;
    }

  ioephandle = IOEP_ATTACH(priv->config->ioedev, 1 << (priv->config->pin),
                           lps27hhw_interrupt_handler, (void *)priv);
  if (ioephandle == NULL)
    {
      ret = -EIO;
      snerr("ERROR: Failed to attach: %d\n", ret);
      goto err;
    }

  ret = IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                        IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_DISABLE);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set option: %d\n", ret);
      goto irq_err;
    }

  /* Register the character driver */

  ret = sensor_register(&priv->dev.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      sensor_unregister(&priv->dev.lower, devno);
      goto irq_err;
    }

  return ret;

irq_err:
  IOEP_DETACH(priv->config->ioedev, lps27hhw_interrupt_handler);

err:
  kmm_free(priv);
  return ret;
}
