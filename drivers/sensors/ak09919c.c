/****************************************************************************
 * drivers/sensors/ak09919c/ak09919c.c
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
#include <assert.h>

#include <nuttx/nuttx.h>
#include <nuttx/wqueue.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ak09919c.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AK09919C_DEVID                  0x0E48     /* Device ID. */

/* Mag resolution. */

#define AK09919C_RESOLUTION             0.15f      /* uT/LSB. */

/* default ODR */

#define AK09919C_DEFAULT_ODR            10000      /* default 100Hz. */

#define AK09919C_VECTOR_REMAP           2          /* Vector remap of lsm6dso. */

/* Register address. */

#define AK09919C_REG_WIA1               0x00       /* Company ID. */
#define AK09919C_REG_WIA2               0x01       /* Device ID. */
#define AK09919C_REG_ST1                0x10       /* Status 1 register. */
#define AK09919C_REG_HXH                0x11       /* X-axis data high byte. */
#define AK09919C_REG_HXL                0x12       /* X-axis data low byte. */
#define AK09919C_REG_HYH                0x13       /* Y-axis data high byte. */
#define AK09919C_REG_HYL                0x14       /* Y-axis data low byte. */
#define AK09919C_REG_HZH                0x15       /* Z-axis data high byte. */
#define AK09919C_REG_HZL                0x16       /* Z-axis data low byte. */
#define AK09919C_REG_TMPS               0x17       /* Dummy. */
#define AK09919C_REG_ST2                0x18       /* Status 2 register. */
#define AK09919C_REG_CNTL1              0x30       /* Control settings 1 register. */
#define AK09919C_REG_CNTL2              0x31       /* Control settings 2 register. */
#define AK09919C_REG_CNTL3              0x32       /* Control settings 3 register. */

/* Noise Suppression Filter. */

#define AK09919C_NSF_NONE               0x00       /* No noise suppression filter. */
#define AK09919C_NSF_LOW                0x01       /* Low noise suppression filter. */
#define AK09919C_NSF_MIDDLE             0x02       /* Middle noise suppression filter. */
#define AK09919C_NSF_HIGH               0x03       /* High noise suppression filter. */

/* Power mode. */

#define AK09919C_POWER_DOWN_MODE        0x00       /* Powerdown mode(default). */
#define AK09919C_SINGLESHOT_MODE        0x01       /* Single-shot mode. */
#define AK09919C_CONTINUOUS_MODE1       0x02       /* Continuous conversion mode 1. */
#define AK09919C_CONTINUOUS_MODE2       0x04       /* Continuous conversion mode 2. */
#define AK09919C_CONTINUOUS_MODE3       0x06       /* Continuous conversion mode 3. */
#define AK09919C_CONTINUOUS_MODE4       0x08       /* Continuous conversion mode 4. */
#define AK09919C_CONTINUOUS_MODE5       0x0E       /* Continuous conversion mode 5. */
#define AK09919C_SELF_TEST_MODE         0x10       /* Self test mode. */

/* single mode register value threshold. */

#define AK09919C_LOLIMIT_SNG_ST1        1          /* low limit of st1 in single mode. */
#define AK09919C_HILIMIT_SNG_ST1        1          /* high limit of st1 in single mode. */
#define AK09919C_LOLIMIT_SNG_HX         -32751     /* low limit of x-axis in single mode. */
#define AK09919C_HILIMIT_SNG_HX         32751      /* high limit of x-axis in single mode. */
#define AK09919C_LOLIMIT_SNG_HY         -32751     /* low limit of y-axis in single mode. */
#define AK09919C_HILIMIT_SNG_HY         32751      /* high limit of y-axis in single mode. */
#define AK09919C_LOLIMIT_SNG_HZ         -32751     /* low limit of z-axis in single mode. */
#define AK09919C_HILIMIT_SNG_HZ         32751      /* high limit of z-axis in single mode. */
#define AK09919C_LOLIMIT_SNG_ST2        4          /* low limit of st2 in single mode. */
#define AK09919C_HILIMIT_SNG_ST2        116        /* high limit of st2 in single mode. */

/* selftest mode register value threshold. */

#define AK09919C_LOLIMIT_SLF_ST1        1          /* low limit of st1 in selftest mode. */
#define AK09919C_HILIMIT_SLF_ST1        1          /* high limit of st1 in selftest mode. */
#define AK09919C_LOLIMIT_SLF_HX         -200       /* low limit of x-axis in selftest mode. */
#define AK09919C_HILIMIT_SLF_HX         200        /* high limit of x-axis in selftest mode. */
#define AK09919C_LOLIMIT_SLF_HY         -200       /* low limit of y-axis in selftest mode. */
#define AK09919C_HILIMIT_SLF_HY         200        /* high limit of y-axis in selftest mode. */
#define AK09919C_LOLIMIT_SLF_HZ         -1000      /* low limit of z-axis in selftest mode. */
#define AK09919C_HILIMIT_SLF_HZ         -150       /* high limit of z-axis in selftest mode. */
#define AK09919C_LOLIMIT_SLF_ST2        4          /* low limit of st2 in selftest mode. */
#define AK09919C_HILIMIT_SLF_ST2        116        /* high limit of st2 in selftest mode. */

/* Reset control bit. */

#define AK09919C_SOFT_RESET             0x01

/* Device delay(us) control. */

#define AK09919C_SOFT_RESET_DELAY       1000       /* Soft reset delay time(us). */
#define AK09919C_MODE_SWITCH_DELAY      8000       /* Mode switching delay time(us). */

/* Noise suppresion level control bit. */

#define AK09919C_NOISE_SUPPRESION       0x60

#define SET_BITSLICE(s, v, offset, mask)         \
do                                               \
  {                                              \
    s &= mask;                                   \
    s |= (v << offset) & mask;                   \
  }                                              \
while(0)                                         \

#define MAKE_S16(u8h, u8l)                       \
        (int16_t)(((uint16_t)(u8h) << 8) | (uint16_t)(u8l))

#define PARAMETER_VERIFY(data, lolimit, hilimit) \
        ( data >= lolimit && data <= hilimit ) ? 0 : -EINVAL

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Structure for ak09919C device. */

struct ak09919c_dev_s
{
  struct sensor_lowerhalf_s lower;            /* The lower half driver. */
  FAR const struct ak09919c_config_s *config; /* The board config function. */
  bool activated;                             /* Sensor working state. */
  unsigned long interval;                     /* Sensor sample interval. */
  uint8_t workmode;                           /* Sensor work mode. */
  struct work_s work;                         /* Work queue for reading. */
};

/* Structure for ak09919C odr. */

struct ak09919c_odr_s
{
  uint8_t regval;                             /* the data of register. */
  unsigned long odr;                          /* the unit is us. */
};

/* Structure for ak09919c data threshold. */

struct ak09919c_threshold_s
{
  uint8_t st1_lolimit;                        /* low limit of st1. */
  uint8_t st1_hilimit;                        /* high limit of st1. */
  uint8_t st2_lolimit;                        /* low limit of st2. */
  uint8_t st2_hilimit;                        /* high limit of st2. */
  int xdata_lolimit;                          /* low limit of x-axis data. */
  int xdata_hilimit;                          /* high limit of x-axis data. */
  int ydata_lolimit;                          /* low limit of y-axis data. */
  int ydata_hilimit;                          /* high limit of y-axis data. */
  int zdata_lolimit;                          /* low limit of z-axis data. */
  int zdata_hilimit;                          /* high limit of z-axis data. */
};

/* Structure for ak09919c data. */

struct ak09919c_magdata_s
{
  uint8_t st1;                                /* data of AK09919C_REG_ST1. */
  uint8_t xdata_hi;                           /* data of AK09919C_REG_HXH. */
  uint8_t xdata_lo;                           /* data of AK09919C_REG_HXL. */
  uint8_t ydata_hi;                           /* data of AK09919C_REG_HYH. */
  uint8_t ydata_lo;                           /* data of AK09919C_REG_HYL. */
  uint8_t zdata_hi;                           /* data of AK09919C_REG_HZH. */
  uint8_t zdata_lo;                           /* data of AK09919C_REG_HZL. */
  uint8_t res;                                /* data of AK09919C_REG_TMPS. */
  uint8_t st2;                                /* data of AK09919C_REG_ST2. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C functions. */

static int ak09919c_i2c_readreg(FAR struct ak09919c_dev_s *priv,
                                uint8_t regaddr, FAR uint8_t *regval);
static int ak09919c_i2c_writereg(FAR struct ak09919c_dev_s *priv,
                                 uint8_t regaddr, uint8_t regval);
static int ak09919c_i2c_read(FAR struct ak09919c_dev_s *priv,
                             uint8_t regaddr, FAR uint8_t *buffer,
                             uint32_t cnt);

/* Sensor handle functions. */

static int ak09919c_enable(FAR struct ak09919c_dev_s *priv, bool enable);
static int ak09919c_init(FAR struct ak09919c_dev_s *priv);
static int ak09919c_checkid(FAR struct ak09919c_dev_s *priv);
static int ak09919c_checkdev(FAR struct ak09919c_dev_s *priv);
static int ak09919c_setmode(FAR struct ak09919c_dev_s *priv, uint8_t mode);
static int ak09919c_setnoisefilter(FAR struct ak09919c_dev_s *priv,
                                   uint32_t nsf);
static int ak09919c_readmag(FAR struct ak09919c_dev_s *priv,
                            FAR struct sensor_mag *data);
static int ak09919c_softreset(FAR struct ak09919c_dev_s *priv);
static int ak09919c_findodr(FAR unsigned long *expect_period_us);
static int ak09919c_verifyparam(FAR struct ak09919c_magdata_s *magdata,
                                FAR const struct ak09919c_threshold_s
                                *threshold);

/* Sensor ops functions. */

static int ak09919c_activate(FAR struct file *filep,
                             FAR struct sensor_lowerhalf_s *lower,
                             bool enable);
static int ak09919c_set_interval(FAR struct file *filp,
                                 FAR struct sensor_lowerhalf_s *lower,
                                 FAR unsigned long *interval_us);
static int ak09919c_selftest(FAR struct file *filep,
                             FAR struct sensor_lowerhalf_s *lower,
                             unsigned long arg);

/* Sensor poll functions. */

static void ak09919c_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ak09919c_odr_s g_ak09919c_odr[] =
{
  {AK09919C_CONTINUOUS_MODE4, 10000},       /* Sampling interval is 10ms. */
  {AK09919C_CONTINUOUS_MODE3, 20000},       /* Sampling interval is 20ms. */
  {AK09919C_CONTINUOUS_MODE2, 50000},       /* Sampling interval is 50ms. */
  {AK09919C_CONTINUOUS_MODE1, 100000},      /* Sampling interval is 100ms. */
  {AK09919C_CONTINUOUS_MODE5, 200000},      /* Sampling interval is 200ms. */
};

static const struct sensor_ops_s g_ak09919c_ops =
{
  .activate = ak09919c_activate,             /* Enable/disable snesor. */
  .selftest = ak09919c_selftest,             /* Sensor selftest function. */
  .set_interval = ak09919c_set_interval,     /* Set output data period. */
};

static const struct ak09919c_threshold_s g_ak09919c_sngthr =
{
  .st1_lolimit = AK09919C_LOLIMIT_SNG_ST1,  /* low limit of st1 in single mode. */
  .st1_hilimit = AK09919C_HILIMIT_SNG_ST1,  /* high limit of st1 in single mode. */
  .st2_lolimit = AK09919C_LOLIMIT_SNG_ST2,  /* low limit of st2 in single mode. */
  .st2_hilimit = AK09919C_HILIMIT_SNG_ST2,  /* high limit of st2 in single mode. */
  .xdata_lolimit = AK09919C_LOLIMIT_SNG_HX, /* low limit of x-axis in single mode. */
  .xdata_hilimit = AK09919C_HILIMIT_SNG_HX, /* high limit of x-axis in single mode. */
  .ydata_lolimit = AK09919C_LOLIMIT_SNG_HY, /* low limit of y-axis in single mode. */
  .ydata_hilimit = AK09919C_HILIMIT_SNG_HY, /* high limit of y-axis in single mode. */
  .zdata_lolimit = AK09919C_LOLIMIT_SNG_HZ, /* low limit of z-axis in single mode. */
  .zdata_hilimit = AK09919C_HILIMIT_SNG_HZ  /* high limit of z-axis in single mode. */
};

static const struct ak09919c_threshold_s g_ak09919c_slfthr =
{
  .st1_lolimit = AK09919C_LOLIMIT_SLF_ST1,  /* low limit of st1 in selftest mode. */
  .st1_hilimit = AK09919C_HILIMIT_SLF_ST1,  /* high limit of st1 in selftest mode. */
  .st2_lolimit = AK09919C_LOLIMIT_SLF_ST2,  /* low limit of st2 in selftest mode. */
  .st2_hilimit = AK09919C_HILIMIT_SLF_ST2,  /* high limit of st2 in selftest mode. */
  .xdata_lolimit = AK09919C_LOLIMIT_SLF_HX, /* low limit of x-axis in selftest mode. */
  .xdata_hilimit = AK09919C_HILIMIT_SLF_HX, /* high limit of x-axis in selftest mode. */
  .ydata_lolimit = AK09919C_LOLIMIT_SLF_HY, /* low limit of y-axis in selftest mode. */
  .ydata_hilimit = AK09919C_HILIMIT_SLF_HY, /* high limit of y-axis in selftest mode. */
  .zdata_lolimit = AK09919C_LOLIMIT_SLF_HZ, /* low limit of z-axis in selftest mode. */
  .zdata_hilimit = AK09919C_HILIMIT_SLF_HZ  /* high limit of z-axis in selftest mode. */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* I2C functions */

/****************************************************************************
 * Name: ak09919c_i2c_read
 *
 * Description:
 *   Read mag data
 *
 * Input Parameters
 *   priv     -Device struct
 *   regaddr  -Register address
 *   regval   -Register value
 *   cnt      -Data number
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_i2c_read(FAR struct ak09919c_dev_s *priv,
                             uint8_t regaddr, FAR uint8_t *regval,
                             uint32_t cnt)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->config->freq;
  msg[0].addr      = priv->config->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->config->freq;
  msg[1].addr      = priv->config->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = cnt;

  ret = I2C_TRANSFER(priv->config->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_i2c_readreg
 *
 * Description:
 *   Read 8-bit ak09919c register
 *
 * Input Parameters
 *   priv     -Device struct
 *   regaddr  -Register address
 *   regval   -Register value
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_i2c_readreg(FAR struct ak09919c_dev_s *priv,
                                uint8_t regaddr, FAR uint8_t *regval)
{
  return ak09919c_i2c_read(priv, regaddr, regval, 1);
}

/****************************************************************************
 * Name: ak09919c_i2c_writereg
 *
 * Description:
 *   Write 8-bit ak09919c register
 *
 * Input Parameters
 *   priv     -Device struct
 *   regaddr  -Register address
 *   regval   -To be write value
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_i2c_writereg(FAR struct ak09919c_dev_s *priv,
                                 uint8_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg;
  int ret;
  uint8_t txbuffer[2];

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg.frequency = priv->config->freq;
  msg.addr      = priv->config->addr;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = 2;

  ret = I2C_TRANSFER(priv->config->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/* Sensor handle functions */

/****************************************************************************
 * Name: ak09919c_setmode
 *
 * Description:
 *   Set work mode for AK09919C
 *
 * Input Parameters
 *   priv     -Device struct
 *   mode     -Work mode
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_setmode(FAR struct ak09919c_dev_s *priv,
                            uint8_t mode)
{
  int ret;

  ret = ak09919c_i2c_writereg(priv, AK09919C_REG_CNTL2, mode);
  if (ret < 0)
    {
      snerr("Failed to set work mode: %d\n", ret);
    }
  else
    {
      up_udelay(AK09919C_MODE_SWITCH_DELAY);
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_setnoisefilter
 *
 * Description:
 *   Set AK09919c noise suppression filter
 *
 * Input Parameters
 *   priv     -Device struct
 *   nsf     -noise suppression filter
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_setnoisefilter(FAR struct ak09919c_dev_s *priv,
                                   uint32_t nsf)
{
  int ret;
  uint8_t ctrl1;

  ret = ak09919c_i2c_readreg(priv, AK09919C_REG_CNTL1, &ctrl1);
  if (ret < 0)
    {
      snerr("Failed to set noise filter: %d\n", ret);
    }

  SET_BITSLICE(ctrl1, nsf, 5, AK09919C_NOISE_SUPPRESION);

  ret = ak09919c_i2c_writereg(priv, AK09919C_REG_CNTL1, ctrl1);
  if (ret < 0)
    {
      snerr("Failed to set noise filter: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_readmag
 *
 * Description:
 *   Read mag data
 *
 * Input Parameters
 *   priv     -Device struct
 *   data     -mag data buffer
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_readmag(FAR struct ak09919c_dev_s *priv,
                            FAR struct sensor_mag *data)
{
  int ret;
  uint8_t state;
  uint8_t buffer[8];
  int16_t raw_data[3];

  /* AK09919C_REG_ST1 must be read separately before reading data. */

  ret = ak09919c_i2c_readreg(priv, AK09919C_REG_ST1, &state);
  if (ret < 0)
    {
      snerr("Failed to read drdy: %d\n", ret);
      return ret;
    }

  /* New data can be unlocked only when AK09919C_REG_ST2(8th byte) is read. */

  ret = ak09919c_i2c_read(priv, AK09919C_REG_HXH,
                          buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("Failed to read mag: %d\n", ret);
      return ret;
    }

  raw_data[0] = MAKE_S16(buffer[0], buffer[1]);
  raw_data[1] = MAKE_S16(buffer[2], buffer[3]);
  raw_data[2] = MAKE_S16(buffer[4], buffer[5]);

  sensor_remap_vector_raw16(raw_data, raw_data, AK09919C_VECTOR_REMAP);

  data->x = AK09919C_RESOLUTION * raw_data[0];
  data->y = AK09919C_RESOLUTION * raw_data[1];
  data->z = AK09919C_RESOLUTION * raw_data[2];

  return ret;
}

/****************************************************************************
 * Name: ak09919c_checkid
 *
 * Description:
 *   Read and verify the AK09919c chip ID
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_checkid(FAR struct ak09919c_dev_s *priv)
{
  int ret;
  uint16_t devid;

  ret = ak09919c_i2c_read(priv, AK09919C_REG_WIA1, (uint8_t *)&devid, 2);
  if (ret < 0)
    {
      snerr("Failed to read device id: %d\n", ret);
      return ret;
    }

  if (devid != AK09919C_DEVID)
    {
      snerr("Wrong Device ID! %02x\n", devid);
      ret = -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_softreset
 *
 * Description:
 *   Reset AK09919c device
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_softreset(FAR struct ak09919c_dev_s *priv)
{
  int ret;

  ret = ak09919c_i2c_writereg(priv, AK09919C_REG_CNTL3,
                              AK09919C_SOFT_RESET);
  if (ret < 0)
    {
      snerr("Failed to reset AK09919C: %d\n", ret);
    }
  else
    {
      up_udelay(AK09919C_SOFT_RESET_DELAY);
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_verifyparam
 *
 * Description:
 *   Parameter verification
 *
 * Input Parameters
 *   magdata   - Data of mag
 *   threshold - Threshold of mag data
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_verifyparam(FAR struct ak09919c_magdata_s *magdata,
                                FAR const struct ak09919c_threshold_s
                                *threshold)
{
  int xdata;
  int ydata;
  int zdata;
  int ret;

  xdata = MAKE_S16(magdata->xdata_hi, magdata->xdata_lo);
  ydata = MAKE_S16(magdata->ydata_hi, magdata->ydata_lo);
  zdata = MAKE_S16(magdata->zdata_hi, magdata->zdata_lo);

  ret = PARAMETER_VERIFY(magdata->st1,
                         threshold->st1_lolimit,
                         threshold->st1_hilimit);
  if (ret < 0)
    {
      snerr("ST1 data is abnormal: %d\n", ret);
      return ret;
    }

  ret = PARAMETER_VERIFY(magdata->st2,
                         threshold->st2_lolimit,
                         threshold->st2_hilimit);
  if (ret < 0)
    {
      snerr("ST2 data is abnormal: %d\n", ret);
      return ret;
    }

  ret = PARAMETER_VERIFY(xdata,
                         threshold->xdata_lolimit,
                         threshold->xdata_hilimit);
  if (ret < 0)
    {
      snerr("X-axis data is abnormal: %d\n", ret);
      return ret;
    }

  ret = PARAMETER_VERIFY(ydata,
                         threshold->ydata_lolimit,
                         threshold->ydata_hilimit);
  if (ret < 0)
    {
      snerr("Y-axis data is abnormal: %d\n", ret);
      return ret;
    }

  ret = PARAMETER_VERIFY(zdata,
                         threshold->zdata_lolimit,
                         threshold->zdata_hilimit);
  if (ret < 0)
    {
      snerr("Z-axis data is abnormal: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_enable
 *
 * Description:
 *   AK09919c control on/off
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_enable(FAR struct ak09919c_dev_s *priv, bool enable)
{
  int ret;

  if (enable)
    {
      /* Reset soft device. */

      ret = ak09919c_softreset(priv);
      if (ret < 0)
        {
          snerr("Failed reset device: %d\n", ret);
          return ret;
        }

      /* Set noise filter. */

      ret = ak09919c_setnoisefilter(priv, AK09919C_NSF_LOW);
      if (ret < 0)
        {
          snerr("Failed set noise filter: %d\n", ret);
          return ret;
        }

      /* Set work mode. */

      ret = ak09919c_setmode(priv, priv->workmode);
      if (ret < 0)
        {
          snerr("Failed set work mode: %d\n", ret);
          return ret;
        }

      work_queue(HPWORK, &priv->work,
                 ak09919c_worker, priv,
                 priv->interval / USEC_PER_TICK);
    }
  else
    {
      work_cancel(HPWORK, &priv->work);

      /* Reset soft device. */

      ret = ak09919c_softreset(priv);
      if (ret < 0)
        {
          snerr("Failed reset device: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_findodr
 *
 * Description:
 *   Find the period that matches best.
 *
 * Input Parameters
 *   expect_period_us  -the time(expext) between report data, in us.
 *
 * Returned Value
 *   Index of odr param.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_findodr(FAR unsigned long *expect_period_us)
{
  int i;
  int len = sizeof(g_ak09919c_odr) / sizeof(struct ak09919c_odr_s);

  for (i = 0; i < len; i++)
    {
      if (*expect_period_us <= g_ak09919c_odr[i].odr)
        {
          return i;
        }
    }

  return len - 1;
}

/****************************************************************************
 * Name: ak09919c_init
 *
 * Description:
 *   AK09919c init
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_init(FAR struct ak09919c_dev_s *priv)
{
  int ret;

  /* Reset soft device. */

  ret = ak09919c_softreset(priv);
  if (ret < 0)
    {
      snerr("Failed reset device: %d\n", ret);
      return ret;
    }

  /* Set noise suppression level. */

  ret = ak09919c_setnoisefilter(priv, AK09919C_NSF_LOW);
  if (ret < 0)
    {
      snerr("Failed set Noise suppression filter: %d\n", ret);
      return ret;
    }

  /* Set power down mode. */

  ret = ak09919c_setmode(priv, AK09919C_POWER_DOWN_MODE);
  if (ret < 0)
    {
      snerr("Failed set work mode: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_checkdev
 *
 * Description:
 *   Check if the device is working properly.
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_checkdev(FAR struct ak09919c_dev_s *priv)
{
  struct ak09919c_magdata_s magdata;
  int ret;

  /* Reset soft device. */

  ret = ak09919c_softreset(priv);
  if (ret < 0)
    {
      snerr("Failed reset device: %d\n", ret);
      return ret;
    }

  /* Set to single measurement mode. */

  ret = ak09919c_setmode(priv, AK09919C_SINGLESHOT_MODE);
  if (ret < 0)
    {
      snerr("Failed set single measurement mode: %d\n", ret);
      return ret;
    }

  /* AK09919C_REG_ST1 must be read separately before reading data. */

  ret = ak09919c_i2c_readreg(priv, AK09919C_REG_ST1, &magdata.st1);
  if (ret < 0)
    {
      snerr("Failed to read state: %d\n", ret);
      return ret;
    }

  /* New data can be unlocked only when AK09919C_REG_ST2(8th byte) is read. */

  ret = ak09919c_i2c_read(priv, AK09919C_REG_HXH,
                          &magdata.xdata_hi, 8);
  if (ret < 0)
    {
      snerr("Failed to read mag: %d\n", ret);
      return ret;
    }

  /* Check whether single mode data is right. */

  ret = ak09919c_verifyparam(&magdata,
                             &g_ak09919c_sngthr);
  if (ret < 0)
    {
      snerr("The data of single mode is error: %d\n", ret);
      return ret;
    }

  /* Set to selftest mode. */

  ret = ak09919c_setmode(priv, AK09919C_SELF_TEST_MODE);
  if (ret < 0)
    {
      snerr("Failed set selftest mode: %d\n", ret);
      return ret;
    }

  /* AK09919C_REG_ST1 must be read separately before reading data. */

  ret = ak09919c_i2c_readreg(priv, AK09919C_REG_ST1, &magdata.st1);
  if (ret < 0)
    {
      snerr("Failed to read state: %d\n", ret);
      return ret;
    }

  /* New data can be unlocked only when AK09919C_REG_ST2(8th byte) is read. */

  ret = ak09919c_i2c_read(priv, AK09919C_REG_HXH,
                          &magdata.xdata_hi, 8);
  if (ret < 0)
    {
      snerr("Failed to read mag: %d\n", ret);
      return ret;
    }

  /* Check whether selftest mode data is right. */

  ret = ak09919c_verifyparam(&magdata,
                             &g_ak09919c_slfthr);
  if (ret < 0)
    {
      snerr("The data of selftest mode is error: %d\n", ret);
      return ret;
    }

  return ret;
}

/* Sensor ops functions */

/****************************************************************************
 * Name: ak09919c_set_interval
 *
 * Description:
 *   Set ODR
 *
 * Input Parameters
 *   filep       - The pointer of file, represents each user using the sensor.
 *   lower       - The instance of lower half sensor driver.
 *   interval_us - Sample interval.
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_set_interval(FAR struct file *filp,
                                 FAR struct sensor_lowerhalf_s *lower,
                                 FAR unsigned long *interval_us)
{
  FAR struct ak09919c_dev_s *priv = (FAR struct ak09919c_dev_s *)lower;
  int ret;
  int idx;

  DEBUGASSERT(priv != NULL);

  /* Find the period that matches best. */

  idx = ak09919c_findodr(interval_us);
  priv->interval = g_ak09919c_odr[idx].odr;
  priv->workmode = g_ak09919c_odr[idx].regval;

  ret = ak09919c_setmode(priv, priv->workmode);
  if (ret < 0)
    {
      snerr("Failed set work mode: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_activate
 *
 * Description:
 *   Enable or disable sensor device.
 *
 * Input Parameters
 *   filep  - The pointer of file, represents each user using the sensor.
 *   lower  - The instance of lower half sensor driver.
 *   enable - true(enable) and false(disable).
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ak09919c_activate(FAR struct file *filep,
                             FAR struct sensor_lowerhalf_s *lower,
                             bool enable)
{
  FAR struct ak09919c_dev_s *priv = (FAR struct ak09919c_dev_s *)lower;
  int ret;

  if (lower->type != SENSOR_TYPE_MAGNETIC_FIELD)
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  if (priv->activated != enable)
    {
      ret = ak09919c_enable(priv, enable);
      if (ret < 0)
        {
          snerr("Failed to enable mag sensor: %d\n", ret);
          return ret;
        }

      priv->activated = enable;
    }

  return OK;
}

/****************************************************************************
 * Name: ak09919c_selftest
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

static int ak09919c_selftest(FAR struct file *filep,
                             FAR struct sensor_lowerhalf_s *lower,
                             unsigned long arg)
{
  FAR struct ak09919c_dev_s *priv = (FAR struct ak09919c_dev_s *)lower;
  int ret;

  DEBUGASSERT(lower != NULL);

  /* Process ioctl commands. */

  switch (arg)
    {
      case SNIOC_SIMPLE_CHECK:    /* Simple communication check. */
        {
          ret = ak09919c_checkid(priv);
          if (ret < 0)
            {
              snerr("Failed to get DeviceID: %d\n", ret);
            }
        }
        break;

      case SNIOC_FULL_CHECK:      /* Fully functional check. */
        {
          ret = ak09919c_checkdev(priv);
          if (ret < 0)
            {
              snerr("Failed to selftest: %d\n", ret);
            }
        }
        break;

      default:                    /* Other cmd tag. */
        {
          ret = -ENOTTY;
          snerr("The cmd don't support: %d\n", ret);
        }
        break;
    }

    return ret;
}

/****************************************************************************
 * Name: ak09919c_worker
 *
 * Description:
 *   Polling the DRDY flag according to 1/5 of the sampling time,
 *   and read the data if DRDY is set
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static void ak09919c_worker(FAR void *arg)
{
  FAR struct ak09919c_dev_s *priv = arg;
  struct sensor_mag tmp;

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp. */

  tmp.timestamp = sensor_get_timestamp();

  /* Set work queue. */

  work_queue(HPWORK, &priv->work, ak09919c_worker,
             priv, priv->interval / USEC_PER_TICK);

  if (ak09919c_readmag(priv, &tmp) >= 0)
    {
      priv->lower.push_event(priv->lower.priv, &tmp,
                             sizeof(struct sensor_mag));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ak09919c_register
 *
 * Description:
 *   Register the AK09919C character device as 'devpath'.
 *
 * Input Parameters:
 *   devno   - The device number, used to build the device path
 *             as /dev/sensor/gyro_uncalN
 *   config  - configuration for the L3GD20 driver.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int ak09919c_register(int devno, FAR const struct ak09919c_config_s *config)
{
  FAR struct ak09919c_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(config != NULL);

  /* Initialize the AK09919C device structure */

  priv = kmm_zalloc(sizeof(struct ak09919c_dev_s));
  if (priv == NULL)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->config = config;
  priv->lower.type = SENSOR_TYPE_MAGNETIC_FIELD;
  priv->lower.nbuffer = CONFIG_SENSORS_AK09919C_BUFFER_NUMBER;
  priv->lower.uncalibrated = true;
  priv->lower.ops = &g_ak09919c_ops;
  priv->interval = AK09919C_DEFAULT_ODR;
  priv->workmode = AK09919C_CONTINUOUS_MODE4;

  /* Check Device ID */

  ret = ak09919c_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      goto err;
    }

  /* Init the device */

  ret = ak09919c_init(priv);
  if (ret < 0)
    {
      snerr("Failed to initialize physical device ak0991c:%d\n", ret);
      goto err;
    }

  /* Register the sensor driver */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("Failed to register driver:%d\n", ret);
      goto err;
    }

  return ret;

err:
  kmm_free(priv);
  return ret;
}
