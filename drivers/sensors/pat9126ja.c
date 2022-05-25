/****************************************************************************
 * drivers/sensors/pat9126ja.c
 * Character driver for pat9126ja.
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
#include <nuttx/sensors/pat9126ja.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C address len */

#define PAT9126JA_I2C_ADDR_LEN          7

/* Bit func define */

#define PAT9126JA_BIT(n)                (1 << (n))

/* Array count func define */

#define PAT9126JA_COUNTOF(arr)          (sizeof(arr) / sizeof((arr)[0]))

/* Roundup func define */

#define PAT9126JA_ROUNDUP(x, esize)     (((x) + (esize) - 1) \
                                        / (esize) * (esize))

/* Eanble/disable define */

#define PAT9126JA_ENABLE                1
#define PAT9126JA_DISABLE               0

/* Reset time define */

#define PAT9126JA_RESET_DELAY           3       /* Reset time */
#define PAT9126JA_RESET_MAX_COUNT       3       /* Max reset count */

/* Read polling time and count define */

#define PAT9126JA_POLLING_TIME          16667
#define PAT9126JA_POLLING_COUNT         10

/* Device default macros */

#define PAT9126JA_DEFAULT_BUFFER_NUMBER 1       /* Default buffer number */

/* Sensor resolution */

#define PAT9126JA_X_RESOLUTION          0x64    /* X axis resolution(in cpi) */
#define PAT9126JA_Y_RESOLUTION          0x00    /* Y axis resolution(in cpi) */

/* Register map */

#define PAT9126JA_WHOAMI_VALUE          0x31    /* Valid WHOAMI register value */

/* Non-Bank Register */

#define PAT9126JA_REG_BANK              0x7F    /* Bank register */
#define PAT9126JA_BANK0                 0x00    /* Bank 0x00 */
#define PAT9126JA_BANK1                 0x01    /* Bank 0x01 */

/* Internal Registers in Bank 0x00 */

#define PAT9126JA_REG_WHOAMI            0x00    /* WHOAMI register */

#define PAT9126JA_REG_PID2              0x01    /* Product_ID2 register */
#define PAT9126JA_PID2_VALUE            0x92    /* Product_ID2: 0x92 */

#define PAT9126JA_REG_MOTION_STATUS     0x02    /* Motion_Status register */
#define PAT9126JA_MASK_MOTION_STATUS_M  PAT9126JA_BIT(7)
#define PAT9126JA_SHIFT_MOTION_STATUS_M 7

#define PAT9126JA_REG_DELTA_X_LO        0x03    /* Delta_X_Lo register */
#define PAT9126JA_MASK_DELTA_X_LO       0xff
#define PAT9126JA_SHIFT_DELTA_X_LO      0

#define PAT9126JA_REG_DELTA_Y_LO        0x04    /* Delta_Y_Lo register */
#define PAT9126JA_MASK_DELTA_Y_LO       0xff
#define PAT9126JA_SHIFT_DELTA_Y_LO      0

#define PAT9126JA_REG_MODE              0x05    /* OP_Mode register */
#define PAT9126JA_MASK_MODE_SLP_ENH     PAT9126JA_BIT(4)
#define PAT9126JA_SHIFT_MODE_SLP_ENH    4
#define PAT9126JA_MASK_MODE_SLP2_ENH    PAT9126JA_BIT(3)
#define PAT9126JA_SHIFT_MODE_SLP2_ENH   3
#define PAT9126JA_MASK_MODE_SLP2MU_ENH  PAT9126JA_BIT(2)
#define PAT9126JA_SHIFT_MODE_SLP2MU_ENH 2
#define PAT9126JA_MASK_MODE_SLP1MU_ENH  PAT9126JA_BIT(1)
#define PAT9126JA_SHIFT_MODE_SLP1MU_ENH 1
#define PAT9126JA_MASK_MODE_WAKEUP      PAT9126JA_BIT(0)
#define PAT9126JA_SHIFT_MODE_WAKEUP     0
#define PAT9126JA_MODE_DIS_SLP1_SLP2    0xa0
#define PAT9126JA_MODE_EN_SLP1_DIS_SLP2 0xb0
#define PAT9126JA_MODE_EN_SLP1_SLP2     0xb8
#define PAT9126JA_MODE_FORCE_SLP2       0xbC
#define PAT9126JA_MODE_FORCE_SLP1       0xb2
#define PAT9126JA_MODE_WAKEUP_TO_RUN    0xa1

#define PAT9126JA_REG_CONFIG            0x06    /* CONFIGURATION register */
#define PAT9126JA_MASK_CONFIG_RESET     PAT9126JA_BIT(7)
#define PAT9126JA_SHIFT_CONFIG_RESET    7
#define PAT9126JA_MASK_CONFIG_PD_ENH    PAT9126JA_BIT(3)
#define PAT9126JA_SHIFT_CONFIG_PD_ENH   3
#define PAT9126JA_CONFIG_RESET          0x97
#define PAT9126JA_CONFIG_RESET_OK       0x17
#define PAT9126JA_CONFIG_POWERDOWN      0x1F

#define PAT9126JA_REG_W_PROTECT         0x09    /* Write_Protect register */
#define PAT9126JA_MASK_W_PROTECT        0xff
#define PAT9126JA_SHIFT_W_PROTECT       0
#define PAT9126JA_ENABLE_W_PROTECT      0x00
#define PAT9126JA_DISABLE_W_PROTECT     0x5a

#define PAT9126JA_REG_SLEEP1            0x0a    /* Sleep1 register */
#define PAT9126JA_MASK_SLEEP1_FREQ      0xf0
#define PAT9126JA_SHIFT_SLEEP1_FREQ     4
#define PAT9126JA_MASK_SLEEP1_ETM       0x0f
#define PAT9126JA_SHIFT_SLEEP1_ETM      0

#define PAT9126JA_REG_SLEEP2            0x0b    /* Sleep2 register */
#define PAT9126JA_MASK_SLEEP2_FREQ      0xf0
#define PAT9126JA_SHIFT_SLEEP2_FREQ     4
#define PAT9126JA_MASK_SLEEP2_ETM       0x0f
#define PAT9126JA_SHIFT_SLEEP2_ETM      0

#define PAT9126JA_REG_RES_X             0x0d    /* RES_X register */

#define PAT9126JA_REG_RES_Y             0x0e    /* RES_Y register */

#define PAT9126JA_REG_DELTA_XY_HI       0x12    /* Delta_XY_Hi register */
#define PAT9126JA_MASK_DELTA_XY_HIX     0xf0
#define PAT9126JA_SHIFT_DELTA_XY_HIX    4
#define PAT9126JA_MASK_DELTA_XY_HIY     0x0f
#define PAT9126JA_SHIFT_DELTA_XY_HIY    0

#define PAT9126JA_REG_DATAFORMAT        0x19    /* Data format register */
#define PAT9126JA_DATAFORMAT_12_BIT     0x04

#define PAT9126JA_REG_LOWVOLT_CONFIG    0x4b    /* LowVolt_Config register */
#define PAT9126JA_MASK_LOWVOLT_CONFIG   0xff
#define PAT9126JA_SHIFT_LOWVOLT_CONFIG  0
#define PAT9126JA_HIGHVOLT_SEGMENT      0x00
#define PAT9126JA_LOWVOLT_SEGMENT       0x04

#define PAT9126JA_REG_FEATURE1          0x7c    /* Feature1 register */
#define PAT9126JA_FEATURE1_VALUE        0x82

#define PAT9126JA_REG_FEATURE2          0x2b    /* Feature2 register */
#define PAT9126JA_FEATURE2_VALUE        0x6d

#define PAT9126JA_REG_FEATURE3          0x2d    /* Feature3 register */
#define PAT9126JA_FEATURE3_VALUE        0x00

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Sensor mode enum */

enum pat9126ja_mode_e
{
  PAT9126JA_MODE_NORMAL,                        /* Normal mode */
  PAT9126JA_MODE_LOWPOWER1,                     /* Lowpower1 mode */
  PAT9126JA_MODE_LOWPOWER2,                     /* Lowpower2 mode */
  PAT9126JA_MODE_POWERDOWN                      /* Powerdown mode */
};

/* Sensor struct */

struct pat9126ja_sensor_s
{
  struct sensor_lowerhalf_s lower;              /* Lower half sensor driver */
  bool                      activated;          /* Sensor working state */
  uint64_t                  timestamp;          /* Units is microseconds */
  int32_t                   x_position;         /* Position of x-axis */
  uint32_t                  polling_count;      /* Units is counts */
  uint8_t                   x_resolution;       /* Resolution of x-axis */
};

/* Device struct */

struct pat9126ja_dev_s
{
  struct pat9126ja_sensor_s           dev;     /* Sensor struct */
  FAR const struct pat9126ja_config_s *config; /* The board config */
  struct work_s                       work;    /* Interrupt handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C functions */

static int pat9126ja_readreg(FAR struct pat9126ja_dev_s *priv,
                             uint8_t regaddr, FAR uint8_t *regval);
static int pat9126ja_writereg(FAR struct pat9126ja_dev_s *priv,
                              uint8_t regaddr, uint8_t regval);
static int pat9126ja_updatereg(FAR struct pat9126ja_dev_s *priv,
                               uint8_t regaddr, uint8_t mask,
                               uint8_t regval);

/* Sensor handle functions */

/* pat9126ja handle functions */

static int pat9126ja_readdevid(FAR struct pat9126ja_dev_s *priv);
static int pat9126ja_writeprotect(FAR struct pat9126ja_dev_s *priv,
                                  uint8_t enable);
static int pat9126ja_reset(FAR struct pat9126ja_dev_s *priv);
static int pat9126ja_setmode(FAR struct pat9126ja_dev_s *priv,
                             enum pat9126ja_mode_e mode);
static int pat9126ja_isready(FAR struct pat9126ja_dev_s *priv,
                             FAR uint8_t *value);
static int pat9126ja_getdata(FAR struct pat9126ja_dev_s *priv,
                             FAR struct sensor_ots *ots);
static int pat9126ja_read_push(FAR struct pat9126ja_dev_s *priv, bool push);
static int pat9126ja_initchip(FAR struct pat9126ja_dev_s *priv);

/* Sensor ops functions */

static int pat9126ja_activate(FAR struct file *filep,
                              FAR struct sensor_lowerhalf_s *lower,
                              bool enable);
#ifdef CONFIG_FACTEST_SENSORS_PAT9126JA
static int pat9126ja_selftest(FAR struct file *filep,
                              FAR struct sensor_lowerhalf_s *lower,
                              unsigned long arg);
static int pat9126ja_calibrate(FAR struct file *filep,
                               FAR struct sensor_lowerhalf_s *lower,
                               unsigned long arg);
static int pat9126ja_control(FAR struct sensor_lowerhalf_s *lower,
                             int cmd, unsigned long arg);
#endif
static int pat9126ja_set_calibvalue(FAR struct file *filep,
                                    FAR struct sensor_lowerhalf_s *lower,
                                    unsigned long arg);

/* Sensor interrupt functions */

static int pat9126ja_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                       ioe_pinset_t pinset, FAR void *arg);
static void pat9126ja_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Sensor ops */

static const struct sensor_ops_s g_pat9126ja_ops =
{
  .activate       = pat9126ja_activate,       /* Enable/disable sensor */
#ifdef CONFIG_FACTEST_SENSORS_PAT9126JA
  .selftest       = pat9126ja_selftest,       /* Sensor selftest */
  .calibrate      = pat9126ja_calibrate,      /* Sensor calibrate */
  .control        = pat9126ja_control,        /* Set special config for sensor */
#endif
  .set_calibvalue = pat9126ja_set_calibvalue  /* Sensor set calibvalue */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* I2C functions */

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
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int pat9126ja_readreg(FAR struct pat9126ja_dev_s *priv,
                             uint8_t regaddr, FAR uint8_t *regval)
{
  struct i2c_config_s config;

  /* Set up the I2C configuration */

  config.frequency = priv->config->freq;
  config.address   = priv->config->addr;
  config.addrlen   = PAT9126JA_I2C_ADDR_LEN;

  /* I2c read one byte */

  return i2c_writeread(priv->config->i2c, &config, &regaddr,
                       sizeof(regaddr), regval, sizeof(*regval));
}

/****************************************************************************
 * Name: pat9126ja_writereg
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
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int pat9126ja_writereg(FAR struct pat9126ja_dev_s *priv,
                              uint8_t regaddr, uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t tx_buff[2];

  /* Set up the I2C configuration */

  config.frequency = priv->config->freq;
  config.address   = priv->config->addr;
  config.addrlen   = PAT9126JA_I2C_ADDR_LEN;

  /* Setup tx buffer */

  tx_buff[0] = regaddr;
  tx_buff[1] = regval;

  /* I2c reg write one byte */

  return i2c_write(priv->config->i2c, &config, tx_buff, 2);
}

/****************************************************************************
 * Name: pat9126ja_updatereg
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
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int pat9126ja_updatereg(FAR struct pat9126ja_dev_s *priv,
                               uint8_t regaddr, uint8_t mask,
                               uint8_t regval)
{
  uint8_t old_value;
  uint8_t new_value;
  int ret;

  /* Read old value of the register */

  ret = pat9126ja_readreg(priv, regaddr, &old_value);
  if (ret < 0)
    {
      snerr("Failed to read register: %d\n", ret);
      return ret;
    }

  /* Setup new value of the register */

  new_value = (old_value & ~mask) | (regval & mask);
  if (new_value == old_value)
    {
      return ret;
    }

  /* Write new value to the register */

  return pat9126ja_writereg(priv, regaddr, new_value);
}

/* Sensor handle functions */

/****************************************************************************
 * Name: pat9126ja_readdevid
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

static int pat9126ja_readdevid(FAR struct pat9126ja_dev_s *priv)
{
  uint8_t regval;
  int ret;

  ret = pat9126ja_readreg(priv, PAT9126JA_REG_WHOAMI, &regval);
  if (ret < 0)
    {
      snerr("Failed to read device id: %d\n", ret);
      return ret;
    }

  if (regval != PAT9126JA_WHOAMI_VALUE)
    {
      snerr("Wrong device ID: %x\n", regval);
      return -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: pat9126ja_writeprotect
 *
 * Description:
 *   Enable/disble write protect for registers beyond address 0x09.
 *
 * Input Parameters:
 *   priv   - Device struct.
 *   enable - Enable/disble value.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int pat9126ja_writeprotect(FAR struct pat9126ja_dev_s *priv,
                                  uint8_t enable)
{
  int ret;

  if (enable)
    {
      ret = pat9126ja_updatereg(priv, PAT9126JA_REG_W_PROTECT,
                                PAT9126JA_MASK_W_PROTECT,
                                PAT9126JA_ENABLE_W_PROTECT);
      if (ret < 0)
        {
          snerr("Failed to enable write protect: %d\n", ret);
        }
    }
  else
    {
      ret = pat9126ja_updatereg(priv, PAT9126JA_REG_W_PROTECT,
                                PAT9126JA_MASK_W_PROTECT,
                                PAT9126JA_DISABLE_W_PROTECT);
      if (ret < 0)
        {
          snerr("Failed to disable write protect: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: pat9126ja_reset
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

static int pat9126ja_reset(FAR struct pat9126ja_dev_s *priv)
{
  uint8_t count = 0;
  uint8_t regval;
  uint8_t status;
  int ret;

  do
    {
      /* software reset */

      pat9126ja_writereg(priv, PAT9126JA_REG_CONFIG,
                               PAT9126JA_CONFIG_RESET);

      /* Delay for reset booting */

      up_mdelay(PAT9126JA_RESET_DELAY);

      /* Check reset is done */

      ret = pat9126ja_readreg(priv, PAT9126JA_REG_CONFIG, &regval);
      if (ret < 0)
        {
          snerr("Failed to read reset status: %d\n", ret);
          return ret;
        }

      status = (PAT9126JA_CONFIG_RESET_OK == regval);
      count++;

      /* Check reset is timeout */

      if (count > PAT9126JA_RESET_MAX_COUNT)
        {
          snerr("Reset timeout: %d\n", count);
          return -EIO;
        }
    }
  while (!status);

  return ret;
}

/****************************************************************************
 * Name: pat9126ja_setmode
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

static int pat9126ja_setmode(FAR struct pat9126ja_dev_s *priv,
                             enum pat9126ja_mode_e mode)
{
  uint8_t regval;
  int ret;

  /* If the chip is powerdown, need to exit this state */

  ret = pat9126ja_readreg(priv, PAT9126JA_REG_CONFIG, &regval);
  if (ret < 0)
    {
      snerr("Failied to read config register: %d\n", ret);
    }

  if ((regval & PAT9126JA_MASK_CONFIG_PD_ENH)
      >> PAT9126JA_SHIFT_CONFIG_PD_ENH)
    {
      ret = pat9126ja_writereg(priv, PAT9126JA_REG_CONFIG,
                               PAT9126JA_CONFIG_RESET_OK);
      if (ret < 0)
        {
          snerr("Failed to make chip exit powerdown: %d\n", ret);
        }
    }

  switch (mode)
    {
      case PAT9126JA_MODE_NORMAL:
        {
          ret = pat9126ja_writereg(priv, PAT9126JA_REG_MODE,
                                   PAT9126JA_MODE_WAKEUP_TO_RUN);
          if (ret < 0)
            {
              snerr("Failed to make chip wake up to run: %d\n", ret);
            }
        }
        break;

      case PAT9126JA_MODE_LOWPOWER1:
        {
          ret = pat9126ja_writereg(priv, PAT9126JA_REG_MODE,
                                   PAT9126JA_MODE_EN_SLP1_DIS_SLP2);
          if (ret < 0)
            {
              snerr("Failed to make chip into lowpower1 mode: %d\n", ret);
            }
        }
        break;

      case PAT9126JA_MODE_LOWPOWER2:
        {
          ret = pat9126ja_writereg(priv, PAT9126JA_REG_MODE,
                                   PAT9126JA_MODE_EN_SLP1_SLP2);
          if (ret < 0)
            {
              snerr("Failed to make chip into lowpower2 mode: %d\n", ret);
            }
        }
        break;

      case PAT9126JA_MODE_POWERDOWN:
        {
          ret = pat9126ja_writereg(priv, PAT9126JA_REG_CONFIG,
                                   PAT9126JA_CONFIG_POWERDOWN);
          if (ret < 0)
            {
              snerr("Failied to make chip into powerdown state: %d\n", ret);
            }
        }
        break;

      default:
        {
          snerr("Mode type is not supported\n");
          return -EINVAL;
        }
   }

  return ret;
}

/****************************************************************************
 * Name: pat9126ja_isready
 *
 * Description:
 *   Read the sensor data ready flag.
 *
 * Input Parameters:
 *   priv  - Device struct.
 *   value - Data ready flag.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int pat9126ja_isready(FAR struct pat9126ja_dev_s *priv,
        FAR uint8_t *value)
{
  uint8_t status;
  int ret;

  ret = pat9126ja_readreg(priv, PAT9126JA_REG_MOTION_STATUS, &status);
  if (ret < 0)
    {
      snerr("Failed to read motion status: %d\n", ret);
      return ret;
    }

  *value = (status & PAT9126JA_MASK_MOTION_STATUS_M)
         >> PAT9126JA_SHIFT_MOTION_STATUS_M;

  return ret;
}

/****************************************************************************
 * Name: pat9126ja_getdata
 *
 * Description:
 *   Read the ots data.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   ots - Store event data.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int pat9126ja_getdata(FAR struct pat9126ja_dev_s *priv,
                             FAR struct sensor_ots *ots)
{
  uint8_t x_low;
  uint8_t y_low;
  uint8_t xy_hi;
  int16_t x_hi;
  int16_t y_hi;
  int ret;

  ret = pat9126ja_readreg(priv, PAT9126JA_REG_DELTA_X_LO, &x_low);
  if (ret < 0)
    {
      snerr("Failed to read delta_x_lo: %d\n", ret);
      return ret;
    }

  ret = pat9126ja_readreg(priv, PAT9126JA_REG_DELTA_Y_LO, &y_low);
  if (ret < 0)
    {
      snerr("Failed to read delta_y_lo: %d\n", ret);
      return ret;
    }

  ret = pat9126ja_readreg(priv, PAT9126JA_REG_DELTA_XY_HI, &xy_hi);
  if (ret < 0)
    {
      snerr("Failed to read xy_hi: %d\n", ret);
      return ret;
    }

  x_hi = ((int16_t)xy_hi << 4) & 0xf00;
  if (x_hi & 0x800)
    {
      x_hi |= 0xf000;
    }

  y_hi = ((int16_t)xy_hi << 8) & 0xf00;
  if (y_hi & 0x800)
    {
      y_hi |= 0xf000;
    }

  priv->dev.x_position -= (x_hi | x_low);
  ots->x = priv->dev.x_position;
  ots->y = -(y_hi | y_low);

  return ret;
}

/****************************************************************************
 * Name: pat9126ja_read_push
 *
 * Description:
 *   Read the ots data.
 *   Then push data to upper half.
 *
 * Input Parameters:
 *   priv - Device struct.
 *   push - Flag of pushing data to upper half.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int pat9126ja_read_push(FAR struct pat9126ja_dev_s *priv, bool push)
{
  struct sensor_ots ots;
  uint8_t status;
  int ret;

  /* Check data is ready */

  ret = pat9126ja_isready(priv, &status);
  if (ret)
    {
      snerr("Failed to check is ready: %d\n", ret);
      return ret;
    }

  /* If ready, get data */

  if (status)
    {
      ret = pat9126ja_getdata(priv, &ots);
      if (ret < 0)
        {
          snerr("Failed to get data: %d\n", ret);
          return ret;
        }
    }

  /* Add timestamp */

  ots.timestamp = priv->dev.timestamp;

  /* If there is any data, check the data is needed pushing to upper */

  if (push && status)
    {
      priv->dev.lower.push_event(priv->dev.lower.priv, &ots,
                                 sizeof(struct sensor_ots));
    }

  return ret;
}

/****************************************************************************
 * Name: pat9126ja_initchip
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

static int pat9126ja_initchip(FAR struct pat9126ja_dev_s *priv)
{
  int ret;

  /* Read chip id */

  ret = pat9126ja_readdevid(priv);
  if (ret < 0)
    {
      snerr("Failed to reading chip id: %d\n", ret);
      return ret;
    }

  /* Software reset */

  ret = pat9126ja_reset(priv);
  if (ret < 0)
    {
      snerr("Failed to reset chip: %d\n", ret);
      return ret;
    }

  /* Swith to bank0 */

  ret = pat9126ja_writereg(priv, PAT9126JA_REG_BANK, PAT9126JA_BANK0);
  if (ret < 0)
    {
      snerr("Failed to swicth to bank0: %d\n", ret);
      return ret;
    }

  /* Disable write protect */

  ret = pat9126ja_writeprotect(priv, PAT9126JA_DISABLE);
  if (ret < 0)
    {
      snerr("Failed to disable write protect: %d\n", ret);
      return ret;
    }

  /* Set X-axis resolution, deponding on application */

  ret = pat9126ja_writereg(priv, PAT9126JA_REG_RES_X,
                           priv->dev.x_resolution);
  if (ret < 0)
    {
      snerr("Failed to set X-axis resolution: %d\n", ret);
      return ret;
    }

  /* Set Y-axis resolution, deponding on application */

  ret = pat9126ja_writereg(priv, PAT9126JA_REG_RES_Y,
                           PAT9126JA_Y_RESOLUTION);
  if (ret < 0)
    {
      snerr("Failed to set Y-axis resolution: %d\n", ret);
      return ret;
    }

  /* Set X/Y data format: 12-bit */

  ret = pat9126ja_writereg(priv, PAT9126JA_REG_DATAFORMAT,
                           PAT9126JA_DATAFORMAT_12_BIT);
  if (ret < 0)
    {
      snerr("Failed to set 12-bit X/Y data format: %d\n", ret);
      return ret;
    }

  /* Set the chip power: low voltage segment */

  ret = pat9126ja_writereg(priv, PAT9126JA_REG_LOWVOLT_CONFIG,
                           PAT9126JA_LOWVOLT_SEGMENT);
  if (ret < 0)
    {
      snerr("Failed to set the chip for low voltage segment: %d\n", ret);
      return ret;
    }

  /* Set other features beyond the datasheet */

  ret = pat9126ja_writereg(priv, PAT9126JA_REG_FEATURE1,
                           PAT9126JA_FEATURE1_VALUE);
  if (ret < 0)
    {
      snerr("Failed to set feature1 register: %d\n", ret);
      return ret;
    }

  ret = pat9126ja_writereg(priv, PAT9126JA_REG_FEATURE2,
                           PAT9126JA_FEATURE2_VALUE);
  if (ret < 0)
    {
      snerr("Failed to set feature2 register: %d\n", ret);
      return ret;
    }

  ret = pat9126ja_writereg(priv, PAT9126JA_REG_FEATURE3,
                           PAT9126JA_FEATURE3_VALUE);
  if (ret < 0)
    {
      snerr("Failed to set feature3 register: %d\n", ret);
    }

  return ret;
}

/* Sensor ops functions */

/****************************************************************************
 * Name: pat9126ja_activate
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

static int pat9126ja_activate(FAR struct file *filep,
                              FAR struct sensor_lowerhalf_s *lower,
                              bool enable)
{
  FAR struct pat9126ja_dev_s *priv = (FAR struct pat9126ja_dev_s *)lower;
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Check activated flag is updated */

  if (priv->dev.activated == enable)
    {
      return OK;
    }

  if (enable)
    {
      /* If enabled, read out but don't push, only used for clearing data */

      ret = pat9126ja_read_push(priv, false);
      if (ret < 0)
        {
          snerr("ERROR: Failed to read push: %d\n", ret);
          return ret;
        }

      ret = pat9126ja_setmode(priv, PAT9126JA_MODE_NORMAL);
      if (ret < 0)
        {
          snerr("ERROR: Failed to set chip run in normal mode: %d\n", ret);
          return ret;
        }
    }
  else
    {
      /* If disabled, set chip enter into lowpower mode */

      ret = pat9126ja_setmode(priv, PAT9126JA_MODE_POWERDOWN);
      if (ret < 0)
        {
          snerr("ERROR: Failed to set chip powerdown: %d\n", ret);
          return ret;
        }

      work_cancel(HPWORK, &priv->work);
    }

  /* Enable/disable interrupt */

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                  IOEXPANDER_OPTION_INTCFG,
                  enable ? (FAR void *)IOEXPANDER_VAL_FALLING :
                  (FAR void *)IOEXPANDER_VAL_DISABLE);
  priv->dev.activated = enable;

  return ret;
}

/****************************************************************************
 * Name: pat9126ja_selftest
 *
 * Description:
 *   Mainly used in the self-test link, including device ID inspection
 *   and device functional inspection.
 *
 * Input Parameters:
 *   filep - The pointer of file, represents each user using the sensor.
 *   lower - The instance of lower half sensor driver.
 *   arg   - The parameters associated with cmd.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *   -ENOTTY    - The cmd don't support.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/
#ifdef CONFIG_FACTEST_SENSORS_PAT9126JA
static int pat9126ja_selftest(FAR struct file *filep,
                              FAR struct sensor_lowerhalf_s *lower,
                              unsigned long arg)
{
  FAR struct pat9126ja_dev_s *priv = (FAR struct pat9126ja_dev_s *)lower;
  int ret = -ENOTTY;

  DEBUGASSERT(lower != NULL);

  /* Process ioctl commands */

  switch (arg)
    {
      case SNIOC_SIMPLE_CHECK:     /* Simple communication check */
        {
          ret = pat9126ja_readdevid(priv);
        }
        break;

      default:                     /* Other cmd tag */
        {
          snerr("ERROR: Cmd is not supported: %d\n", ret);
        }
        break;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: pat9126ja_set_calibvalue
 *
 * Description:
 *   The calibration value to be written in the dedicated registers. At each
 *   power-on, so that the values read from the sensor are already corrected.
 *   When the device is calibrated, the absolute accuracy will be better than
 *   before.
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

static int pat9126ja_set_calibvalue(FAR struct file *filep,
                                    FAR struct sensor_lowerhalf_s *lower,
                                    unsigned long arg)
{
  FAR struct pat9126ja_dev_s *priv = (FAR struct pat9126ja_dev_s *)lower;
  uint8_t x_res;
  int ret;

  DEBUGASSERT(lower != NULL);

  x_res = (uint8_t)atoi((FAR char *)arg);

  /* Set resolution according to target value */

  ret = pat9126ja_writereg(priv, PAT9126JA_REG_RES_X, x_res);
  if (ret < 0)
    {
      snerr("Failed to set X-axis resolution: %d\n", ret);
    }
  else
    {
      priv->dev.x_resolution = x_res;
    }

  return ret;
}

/****************************************************************************
 * Name: pat9126ja_calibrate
 *
 * Description:
 *   This operation can trigger the calibration operation, and if the
 *   calibration operation is short-lived, the calibration result value can
 *   be obtained at the same time, the calibration value to be written in or
 *   the non-volatile memory of the sensor or dedicated registers. When the
 *   upper application calibration is completed, the current calibration
 *   value of the sensor needs to be obtained and backed up, so that the last
 *   calibration value can be directly obtained after power-on.
 *
 * Input Parameters:
 *   filep - The pointer of file, represents each user using the sensor.
 *   lower - The instance of lower half sensor driver.
 *   arg   - The parameters associated with calibration value.
 *
 * Returned Value:
 *   Zero (OK) on success.
 *
 ****************************************************************************/

#ifdef CONFIG_FACTEST_SENSORS_PAT9126JA
static int pat9126ja_calibrate(FAR struct file *filep,
                               FAR struct sensor_lowerhalf_s *lower,
                               unsigned long arg)
{
  FAR struct pat9126ja_dev_s *priv = (FAR struct pat9126ja_dev_s *)lower;

  DEBUGASSERT(lower != NULL);

  sprintf((FAR char *)arg, "%d", priv->dev.x_resolution);

  return OK;
}
#endif

/****************************************************************************
 * Name: pat9126ja_control
 *
 * Description:
 *   With this method, the user can set some special config for the sensor,
 *   such as changing the custom mode, setting the custom resolution, reset,
 *   etc, which are all parsed and implemented by lower half driver.
 *
 * Input Parameters:
 *   lower      - The instance of lower half sensor driver.
 *   cmd        - The special cmd for sensor.
 *   arg        - The parameters associated with cmd.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *   -ENOTTY    - The cmd don't support.
 *   -EINVAL    - Failed to match sensor type.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/
#ifdef CONFIG_FACTEST_SENSORS_PAT9126JA
static int pat9126ja_control(FAR struct sensor_lowerhalf_s *lower,
                             int cmd, unsigned long arg)
{
  FAR struct pat9126ja_dev_s *priv = (FAR struct pat9126ja_dev_s *)lower;
  FAR struct sensor_ioctl_s *ioctl = (FAR struct sensor_ioctl_s *)arg;
  int ret = OK;

  DEBUGASSERT(lower != NULL);

  /* Process ioctl commands. */

  switch (cmd)
    {
      case SNIOC_DISPLACE_INFO:       /* Displacement information */
        {
          sprintf((FAR char *)ioctl->data, "%d", priv->dev.x_position);
        }
        break;

      default:                        /* Other cmd tag */
        {
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}
#endif

/* Sensor interrupt functions */

/****************************************************************************
 * Name: pat9126ja_interrupt_handler
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
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int pat9126ja_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                       ioe_pinset_t pinset, FAR void *arg)
{
  /* This function should be called upon a rising edge on the LSM6DSO
   * new data interrupt pin since it signals that new data has
   * been measured.
   */

  FAR struct pat9126ja_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  /* Task the worker with retrieving the latest sensor data. We should not
   * do this in a interrupt since it might take too long. Also we cannot lock
   * the I2C bus from within an interrupt.
   */

  DEBUGASSERT(priv->work.worker == NULL);

  work_queue(LPWORK, &priv->work, pat9126ja_worker, priv, 0);

  /* Enable polling */

  priv->dev.polling_count = 0;

  /* Disable interrupt */

  IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);

  return OK;
}

/****************************************************************************
 * Name: pat9126ja_worker
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

static void pat9126ja_worker(FAR void *arg)
{
  FAR struct pat9126ja_dev_s *priv = arg;
  bool pin_val;

  /* Sanity check */

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Get the timestamp */

  priv->dev.timestamp = sensor_get_timestamp();

  /* Check interrupt pin status */

  IOEXP_READPIN(priv->config->ioedev, priv->config->pin, &pin_val);

  /* Prepare for next reading */

  work_queue(LPWORK, &priv->work, pat9126ja_worker, priv,
             PAT9126JA_POLLING_TIME / USEC_PER_TICK);

  if (!pin_val)
    {
      /* Read the latest sensor data and push to uppe half */

      pat9126ja_read_push(priv, true);
    }
  else
    {
      priv->dev.polling_count++;

      /* Check interrupt is completed */

      if (priv->dev.polling_count > PAT9126JA_POLLING_COUNT)
        {
          /* Stop polling */

          work_cancel(HPWORK, &priv->work);

          /* Enable interrupt */

          IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                          IOEXPANDER_OPTION_INTCFG,
                          (FAR void *)IOEXPANDER_VAL_FALLING);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pat9126ja_register
 *
 * Description:
 *   Register the pat9126ja character device as 'devno'
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

int pat9126ja_register(int devno,
                       FAR const struct pat9126ja_config_s *config)
{
  FAR struct pat9126ja_dev_s *priv;
  FAR void *ioephandle;
  int ret;

  /* Sanity check */

  DEBUGASSERT(config != NULL);

  /* Initialize the PAT9126JA device structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->dev.lower.type     = SENSOR_TYPE_OTS;
  priv->dev.lower.nbuffer  = PAT9126JA_DEFAULT_BUFFER_NUMBER;
  priv->dev.lower.ops      = &g_pat9126ja_ops;
  priv->dev.x_resolution   = PAT9126JA_X_RESOLUTION;
  priv->config             = config;

  /* Initialize chip and enter into lowpower mode */

  ret = pat9126ja_initchip(priv);
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

  ioephandle = IOEP_ATTACH(priv->config->ioedev, priv->config->pin,
                           pat9126ja_interrupt_handler, (void *)priv);
  if (ioephandle == NULL)
    {
      ret = -EIO;
      snerr("ERROR: Failed to attach: %d\n", ret);
      goto err;
    }

  ret = IOEXP_SETOPTION(priv->config->ioedev, priv->config->pin,
                        IOEXPANDER_OPTION_INTCFG,
                        (FAR void *)IOEXPANDER_VAL_DISABLE);
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
      goto irq_err;
    }

  return ret;

irq_err:
  IOEP_DETACH(priv->config->ioedev, pat9126ja_interrupt_handler);

err:
  kmm_free(priv);
  return ret;
}
