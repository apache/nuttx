/****************************************************************************
 * drivers/sensors/opt3007.c
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
#include <nuttx/sensors/opt3007.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPT3007_DEFAULT_INTERVAL   100000    /* Default conversion interval */
#define OPT3007_AUTOSCALE_MODE     0x0c      /* Automatic Full-Scale Mode */
#define OPT3007_MF_ID              0x5449    /* Manufacturer ID */
#define OPT3007_DEVICE_ID          0x3001    /* Device ID */

/* Mode of conversion operation */

#define OPT3007_SHUTDOWN_MODE      0x00      /* Shutdown mode(default) */
#define OPT3007_SINGLESHOT_MODE    0x01      /* Single-shot mode */
#define OPT3007_CONTINUOUS_MODE    0x02      /* Continuous conversions */

/* Conversiontime */

#define OPT3007_CONVERSION_100MS   0x00      /* Conversion time 100ms */
#define OPT3007_CONVERSION_800MS   0x01      /* Conversion time 800ms */
#define OPT3007_INTERVAL_100MS     100000    /* Interval time 100ms in us */
#define OPT3007_INTERVAL_800MS     800000    /* Interval time 800ms in us */

/* Interrupt mode */

#define OPT3007_CONVERTEND_MODE    0x0c      /* Interrupt when convert end */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct opt3007_devreg_s
{
  uint8_t msb;                               /* Register segment start bit */
  uint8_t lsb;                               /* Register segment end bit */
  uint8_t address;                           /* Register address */
};

typedef struct opt3007_devreg_s opt3007_devreg_t;

struct opt3007_registers_s
{
  opt3007_devreg_t r;                        /* Result */
  opt3007_devreg_t e;                        /* Exponent bits */
  opt3007_devreg_t rn;                       /* Range number field */
  opt3007_devreg_t ct;                       /* Conversion time field */
  opt3007_devreg_t m;                        /* Conversion operation mode */
  opt3007_devreg_t ovf;                      /* Overflow flag field */
  opt3007_devreg_t crf;                      /* Conversion ready field */
  opt3007_devreg_t fh;                       /* Flag high field */
  opt3007_devreg_t fl;                       /* Flag low field */
  opt3007_devreg_t l;                        /* Latch field */
  opt3007_devreg_t pol;                      /* Polarity field */
  opt3007_devreg_t me;                       /* Mask exponent field */
  opt3007_devreg_t fc;                       /* Fault count field */
  opt3007_devreg_t le;                       /* Low-Limit exponent bits */
  opt3007_devreg_t tl;                       /* Low-Limit result */
  opt3007_devreg_t he;                       /* High-Limit exponent bits */
  opt3007_devreg_t th;                       /* High-Limit result */
  opt3007_devreg_t id;                       /* Manufacturer ID (5449h) */
  opt3007_devreg_t did;                      /* Device ID (3001h) */
};

typedef struct opt3007_registers_s opt3007_registers_t;

struct opt3007_dev_s
{
  /* sensor_lowerhalf_s must be in the first line. */

  struct sensor_lowerhalf_s lower;           /* Lower half sensor driver */
  bool activated;                            /* Sensor working state */
  unsigned long interval;                    /* Sensor acquisition interval */
  FAR const struct opt3007_config_s *config; /* The board config function */
  FAR const opt3007_registers_t *devreg;     /* opt3007 device register */
  struct work_s work;                        /* Interrupt handler worker */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C functions */

static int opt3007_i2c_read(FAR struct opt3007_dev_s *priv,
                            uint8_t regaddr,
                            FAR uint16_t *regval);
static int opt3007_i2c_write(FAR struct opt3007_dev_s *priv,
                             uint8_t regaddr, uint16_t value);

/* Sensor handle functions */

static int opt3007_readmfid(FAR struct opt3007_dev_s *priv,
                            FAR const opt3007_registers_t *devreg);
static int opt3007_readdevid(FAR struct opt3007_dev_s *priv,
                             FAR const opt3007_registers_t *devreg);
static int opt3007_enable(FAR struct opt3007_dev_s *priv,
                          bool enable);
static void opt3007_findodr(FAR unsigned long *expect_period_us);
static int opt3007_setrange(FAR struct opt3007_dev_s *priv,
                            FAR const opt3007_registers_t *devreg,
                            uint16_t value);
static int opt3007_setoperation(FAR struct opt3007_dev_s *priv,
                                FAR const opt3007_registers_t *devreg,
                                uint16_t value);
static int opt3007_setconversiontime(FAR struct opt3007_dev_s *priv,
                                     FAR const opt3007_registers_t *devreg,
                                     uint16_t value);
static int opt3007_setintmode(FAR struct opt3007_dev_s *priv,
                              FAR const opt3007_registers_t *devreg);
static int opt3007_readlux(FAR struct opt3007_dev_s *priv,
                           FAR float *lux);

/* Host control functions */

static int opt3007_writereg(FAR struct opt3007_dev_s *priv,
                            FAR const opt3007_devreg_t *devreg,
                            uint16_t regval);
static int opt3007_readreg(FAR struct opt3007_dev_s *priv,
                           FAR const opt3007_devreg_t *devreg,
                           FAR uint16_t *regval);

/* Sensor ops functions */

static int opt3007_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *period_us);
static int opt3007_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);
static int opt3007_selftest(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            unsigned long arg);

/* Sensor poll functions */

static void opt3007_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_opt3007_ops =
{
  .activate = opt3007_activate,             /* Enable/disable sensor. */
  .set_interval = opt3007_set_interval,     /* Set output data period. */
  .selftest = opt3007_selftest              /* Sensor selftest function */
};

static const opt3007_registers_t g_opt3007_devreg =
{
  .e.address = 0,
  .e.msb = 15,
  .e.lsb = 12,

  .r.address = 0,
  .r.msb = 11,
  .r.lsb = 0,

  .rn.address = 1,
  .rn.msb = 15,
  .rn.lsb = 12,

  .ct.address = 1,
  .ct.msb = 11,
  .ct.lsb = 11,

  .m.address = 1,
  .m.msb = 10,
  .m.lsb = 9,

  .ovf.address = 1,
  .ovf.msb = 8,
  .ovf.lsb = 8,

  .crf.address = 1,
  .crf.msb = 7,
  .crf.lsb = 7,

  .fh.address = 1,
  .fh.msb = 6,
  .fh.lsb = 6,

  .fl.address = 1,
  .fl.msb = 5,
  .fl.lsb = 5,

  .l.address = 1,
  .l.msb = 4,
  .l.lsb = 4,

  .pol.address = 1,
  .pol.msb = 3,
  .pol.lsb = 3,

  .me.address = 1,
  .me.msb = 2,
  .me.lsb = 2,

  .fc.address = 1,
  .fc.msb = 1,
  .fc.lsb = 0,

  .le.address = 2,
  .le.msb = 15,
  .le.lsb = 12,

  .tl.address = 2,
  .tl.msb = 11,
  .tl.lsb = 0,

  .he.address = 3,
  .he.msb = 15,
  .he.lsb = 12,

  .th.address = 3,
  .th.msb = 11,
  .th.lsb = 0,

  .id.address = 0x7e,
  .id.msb = 15,
  .id.lsb = 0,

  .did.address = 0x7f,
  .did.msb = 15,
  .did.lsb = 0,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* I2C functions */

/****************************************************************************
 * Name: opt3007_i2c_read
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

static int opt3007_i2c_read(FAR struct opt3007_dev_s *priv, uint8_t regaddr,
                            FAR uint16_t *regval)
{
  struct i2c_config_s config;
  int ret;
  uint8_t buffer[2];

  /* Set up the I2C configuration */

  config.frequency = priv->config->freq;
  config.address = priv->config->addr;
  config.addrlen = 7;

  /* Write 8 bits to device, then read 16-bits from the device */

  ret = i2c_writeread(priv->config->i2c, &config, &regaddr, 1, buffer, 2);
  if (ret < 0)
    {
      snerr("I2C writeread failed: %d\n", ret);
      return ret;
    }

  /* Copy the content of the buffer to the location of the uint16_t pointer */

  *regval = (uint16_t)((buffer[0] << 8) | (buffer[1]));

  return ret;
}

/****************************************************************************
 * Name: opt3007_i2c_write
 *
 * Description:
 *   write 16-bit register.
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

static int opt3007_i2c_write(FAR struct opt3007_dev_s *priv,
                             uint8_t regaddr, uint16_t value)
{
  struct i2c_config_s config;
  int ret;
  uint8_t buf[3];

  DEBUGASSERT(priv != NULL);

  /* Set up the I2C configuration. */

  config.frequency = priv->config->freq;
  config.address = priv->config->addr;
  config.addrlen = 7;

  /* Set up the Modbus write request. */

  buf[0] = regaddr;
  buf[1] = (value >> 8) & 0xff;
  buf[2] = value & 0xff;

  /* Write the Modbus write request. */

  ret = i2c_write(priv->config->i2c, &config, buf, sizeof(buf));
  if (ret < 0)
    {
      snerr("I2C write failed: %d\n", ret);
    }

  return ret;
}

/* Sensor handle functions */

/****************************************************************************
 * Name: opt3007_readmfid
 *
 * Description:
 *   Read the Manufacturer ID.  The manufacturer ID reads 5449h.
 *   In ASCII code, this register reads TI.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   devreg  - Sensor register struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int opt3007_readmfid(FAR struct opt3007_dev_s *priv,
                            FAR const opt3007_registers_t *devreg)
{
  int ret;
  uint16_t regval;

  ret = opt3007_readreg(priv, &devreg->id, &regval);
  if (ret < 0 || regval != OPT3007_MF_ID)
    {
      snerr("Wrong manufacturer ID: %d\n", regval);
      ret = -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: opt3007_readdevid
 *
 * Description:
 *   Read the device ID.  The device ID reads 3001h.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   devreg  - Sensor register struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int opt3007_readdevid(FAR struct opt3007_dev_s *priv,
                             FAR const opt3007_registers_t *devreg)
{
  int ret;
  uint16_t regval;

  ret = opt3007_readreg(priv, &devreg->did, &regval);
  if (ret < 0 || regval != OPT3007_DEVICE_ID)
    {
      snerr("Wrong device ID: %d\n", regval);
      ret = -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: opt3007_enable
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

static int opt3007_enable(FAR struct opt3007_dev_s *priv,
                          bool enable)
{
  int ret;

  if (enable)
    {
      /* Set Range. */

      ret = opt3007_setrange(priv, priv->devreg, OPT3007_AUTOSCALE_MODE);
      if (ret < 0)
        {
          snerr("Failed set range: %d\n", ret);
          return ret;
        }

      /* Set Mode to Continuous conversions. */

      ret = opt3007_setintmode(priv, priv->devreg);
      if (ret < 0)
        {
          snerr("Failed setintmode: %d\n", ret);
          return ret;
        }

      ret = opt3007_setoperation(priv, priv->devreg,
                                 OPT3007_CONTINUOUS_MODE);
      if (ret < 0)
        {
          snerr("Failed set operation: %d\n", ret);
          return ret;
        }

      work_queue(HPWORK, &priv->work,
                 opt3007_worker, priv,
                 priv->interval / USEC_PER_TICK);
    }
  else
    {
      /* Set to Shut Down */

      work_cancel(HPWORK, &priv->work);
      ret = opt3007_setoperation(priv, priv->devreg, OPT3007_SHUTDOWN_MODE);
      if (ret < 0)
        {
          snerr("Failed set operation Shut Down: %d\n", ret);
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: opt3007_findodr
 *
 * Description:
 *   Find the period that matches best.
 *
 * Input Parameters:
 *   expect_period_us  - the time(expect) between report data, in us.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void opt3007_findodr(FAR unsigned long *expect_period_us)
{
  if (*expect_period_us < OPT3007_INTERVAL_800MS)
    {
      if (*expect_period_us > 0)
        {
          *expect_period_us = OPT3007_INTERVAL_100MS;
        }
    }
  else
    {
      *expect_period_us = OPT3007_INTERVAL_800MS;
    }
}

/****************************************************************************
 * Name: opt3007_setrange
 *
 * Description:
 *   Set the sensor to shotdown mode.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   devreg  - Sensor register struct.
 *   value   - Register value.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int opt3007_setrange(FAR struct opt3007_dev_s *priv,
                            FAR const opt3007_registers_t *devreg,
                            uint16_t value)
{
  return opt3007_writereg(priv, &devreg->rn, value);
}

/****************************************************************************
 * Name: opt3007_setoperation
 *
 * Description:
 *   Set the sensor to shotdown mode.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   devreg  - Sensor register struct.
 *   value   - Mode of conversion operation.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   00 = Shutdown(default)
 *   01 = Single-shot
 *   10, 11 = Continuousconversions
 *
 ****************************************************************************/

static int opt3007_setoperation(FAR struct opt3007_dev_s *priv,
                                FAR const opt3007_registers_t *devreg,
                                uint16_t value)
{
  return opt3007_writereg(priv, &devreg->m, value);
}

/****************************************************************************
 * Name: opt3007_setconversiontime
 *
 * Description:
 *   Set the sensor to shotdown mode.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   devreg  - Sensor register struct.
 *   value   - Value of conversion time.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   00 = 100 ms
 *   01 = 800 ms
 *
 ****************************************************************************/

static int opt3007_setconversiontime(FAR struct opt3007_dev_s *priv,
                                     FAR const opt3007_registers_t *devreg,
                                     uint16_t value)
{
  return opt3007_writereg(priv, &devreg->ct, value);
}

/****************************************************************************
 * Name: opt3007_setintmode
 *
 * Description:
 *   Set the sensor to shotdown mode.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   devreg  - Sensor register struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   when the two threshold low register MSBs are set to 11b.
 *
 ****************************************************************************/

static int opt3007_setintmode(FAR struct opt3007_dev_s *priv,
                              FAR const opt3007_registers_t *devreg)
{
  return opt3007_writereg(priv, &devreg->le, OPT3007_CONVERTEND_MODE);
}

/****************************************************************************
 * Name: opt3007_readlux
 *
 * Description:
 *   Set the sensor to shotdown mode.
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

static int opt3007_readlux(FAR struct opt3007_dev_s *priv,
                           FAR float *lux)
{
  int ret;
  uint16_t regmask;
  uint16_t readvalue;
  uint16_t mantissa;
  uint8_t exp;

  ret = opt3007_i2c_read(priv, priv->devreg->r.address, &readvalue);
  if (ret < 0)
    {
      snerr("Failed to read lux: %d\n", ret);
      return ret;
    }

  /* Masks the bits of the register value for mantissa */

  regmask = ((0xffff << (priv->devreg->r.msb + 1)) ^
             (0xffff << priv->devreg->r.lsb)) & 0xffff;

  mantissa = readvalue & regmask;
  exp = readvalue >> priv->devreg->e.lsb;
  *lux = (0.01f) * (1 << exp) * mantissa;

  /* Clear interrupt of the sensor */

  ret = opt3007_i2c_read(priv, priv->devreg->crf.address, &readvalue);
  if (ret < 0)
    {
      snerr("Failed to clear interrupt of the sensor: %d\n", ret);
      return ret;
    }

  return ret;
}

/* Host control functions */

/****************************************************************************
 * Name: opt3007_writereg
 *
 * Description:
 *   Write data to opt3007 register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   devreg  - Sensor register struct.
 *   regval  - Data to write
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int opt3007_writereg(FAR struct opt3007_dev_s *priv,
                            FAR const opt3007_devreg_t *devreg,
                            uint16_t regval)
{
  int ret;
  uint16_t readvalue;
  uint16_t writevalue;
  uint16_t regmask;

  ret = opt3007_i2c_read(priv, devreg->address, &readvalue);
  if (ret < 0)
    {
      snerr("Failed to read register: %d\n", ret);
      return ret;
    }

  /* Masks the bits for the new value to be written. */

  regmask = (~((0xffff << (devreg->msb + 1)) ^
            (0xffff << devreg->lsb))) & 0xffff;

  /* Creates a new register value to be written to h/w
   * based on created mask and data to be written.
   */

  writevalue = (readvalue & regmask) |
               ((regval & (~regmask >> devreg->lsb)) << devreg->lsb);
  if (readvalue != writevalue)
    {
      ret = opt3007_i2c_write(priv, devreg->address, writevalue);
      if (ret < 0)
        {
          snerr("Failed to write register: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: opt3007_readreg
 *
 * Description:
 *   Read data to opt3007 register.
 *
 * Input Parameters:
 *   priv    - Device struct.
 *   devreg  - Sensor register struct.
 *   regval  - Data to write
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int opt3007_readreg(FAR struct opt3007_dev_s *priv,
                           FAR const opt3007_devreg_t *devreg,
                           FAR uint16_t *regval)
{
  uint16_t regmask;
  int ret;

  ret = opt3007_i2c_read(priv, devreg->address, regval);
  if (ret < 0)
    {
      snerr("Failed to read register: %d\n", ret);
      return ret;
    }

  /* Masks the bits for the register value to be reported
   * based on register positional information.
   */

  regmask = (~((0xffff << (devreg->msb + 1)) ^
            (0xffff << devreg->lsb))) & 0xffff;

  /* Assembles the value of the register */

  *regval |= ((*regval & regmask) >> devreg->lsb);

  return ret;
}

/* Sensor ops functions */

/****************************************************************************
 * Name: opt3007_set_interval
 *
 * Description:
 *   Set the sensor output data period in microseconds for a given sensor.
 *   If *period_us > max_delay it will be truncated to max_delay and if
 *   *period_us < min_delay it will be replaced by min_delay.
 *
 * Input Parameters:
 *   lower      - The instance of lower half sensor driver.
 *   filep      - The pointer of file, represents each user using the sensor.
 *   period_us  - The time between report data, in us. It may by overwrite
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

static int opt3007_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *period_us)
{
  FAR struct opt3007_dev_s *priv = (FAR struct opt3007_dev_s *)lower;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Find the period that matches best.  */

  opt3007_findodr(period_us);

  switch (*period_us)
    {
      case OPT3007_INTERVAL_100MS:
        ret = opt3007_setconversiontime(priv, priv->devreg,
                                        OPT3007_CONVERSION_100MS);
        break;

      case OPT3007_INTERVAL_800MS:
        ret = opt3007_setconversiontime(priv, priv->devreg,
                                        OPT3007_CONVERSION_800MS);
        break;

      default:
        ret = -EINVAL;
        break;
    }

  if (ret < 0)
    {
      snerr("Failed to set interval: %d\n", ret);
      return ret;
    }

  priv->interval = *period_us;
  return ret;
}

/****************************************************************************
 * Name: opt3007_activate
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

static int opt3007_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct opt3007_dev_s *priv = (FAR struct opt3007_dev_s *)lower;
  int ret;

  if (lower->type == SENSOR_TYPE_LIGHT)
    {
      if (priv->activated != enable)
        {
          ret = opt3007_enable(priv, enable);
          if (ret < 0)
            {
              snerr("Failed to enable light sensor: %d\n", ret);
              return ret;
            }

          priv->activated = enable;
        }
    }
  else
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: opt3007_selftest
 *
 * Selftest allows for the testing of the mechanical and electrical
 * portions of the sensors. When the selftest is activated, the
 * electronics cause the sensors to be actuated and produce an output
 * signal. The output signal is used to observe the selftest response.
 * When the selftest response exceeds the min/max values,
 * the part is deemed to have failed selftest.
 *
 * Input Parameters:
 *   lower      - The instance of lower half sensor driver.
 *   filep      - The pointer of file, represents each user using the sensor.
 *   arg        - The parameters associated with selftest.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int opt3007_selftest(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            unsigned long arg)
{
  FAR struct opt3007_dev_s * priv = (FAR struct opt3007_dev_s *)lower;
  int ret;

  DEBUGASSERT(lower != NULL);

  /* Process ioctl commands. */

  switch (arg)
    {
      case SNIOC_SIMPLE_CHECK:      /* Simple check tag */
        {
          /* Read device ID. */

          ret = opt3007_readdevid(priv, priv->devreg);
        }
        break;

      default:                      /* Other cmd tag */
        {
          ret = -ENOTTY;
          snerr("ERROR: The cmd don't support: %d\n", ret);
        }
        break;
    }

  return ret;
}

/* Sensor poll functions */

/****************************************************************************
 * Name: opt3007_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data.
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

static void opt3007_worker(FAR void *arg)
{
  FAR struct opt3007_dev_s *priv = arg;
  struct sensor_light light;

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp. */

  light.timestamp = sensor_get_timestamp();

  /* Set work queue. */

  work_queue(HPWORK, &priv->work, opt3007_worker, priv,
             priv->interval / USEC_PER_TICK);

  /* Read out the latest sensor data */

  if (opt3007_readlux(priv, &light.light) == 0)
    {
      /* push data to upper half driver */

      priv->lower.push_event(priv->lower.priv, &light, sizeof(light));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: opt3007_register
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

int opt3007_register(int devno, FAR const struct opt3007_config_s *config)
{
  FAR struct opt3007_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(config != NULL);

  /* Initialize the OPT3007 device structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->config = config;
  priv->devreg = &g_opt3007_devreg;
  priv->lower.ops = &g_opt3007_ops;
  priv->lower.type = SENSOR_TYPE_LIGHT;
  priv->lower.uncalibrated = true;
  priv->interval = OPT3007_DEFAULT_INTERVAL;
  priv->lower.nbuffer = CONFIG_SENSORS_OPT3007_BUFFER_NUMBER;

  /* Read and verify the manufacturerid */

  ret = opt3007_readmfid(priv, priv->devreg);
  if (ret < 0)
    {
      snerr("Failed to verify the ManufacturerID: %d\n", ret);
      goto err;
    }

  /* Read and verify the deviceid */

  ret = opt3007_readdevid(priv, priv->devreg);
  if (ret < 0)
    {
      snerr("Failed to verify the DeviceID: %d\n", ret);
      goto err;
    }

  ret = opt3007_setoperation(priv, priv->devreg, OPT3007_SHUTDOWN_MODE);
  if (ret < 0)
    {
      snerr("Failed to switch off sensor: %d\n", ret);
      goto err;
    }

  /* Register the character driver */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      goto err;
    }

  return ret;

err:
  kmm_free(priv);
  return ret;
}
