/****************************************************************************
 * drivers/sensors/sx9373.c
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
#include <stdio.h>
#include <fcntl.h>

#include <nuttx/nuttx.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/sx9373.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SX9373_WHOAMI_VALUE           0x937327 /* Device ID. */
#define SX9373_INIT_DELAY             50000    /* Init delay time(us). */
#define SX9373_MAX_RETRY              3        /* Max number of retries. */
#define SX9373_CH_NUM                 3        /* Number of channel. */

/* Device state setup. */

#define SX9373_IRQ_SOURCE             0x4000   /* Current interrupt events. */
#define SX9373_IRQ_DISABLE            0x4004   /* Disable interrupt*/
#define SX9373_DEVICE_RESET           0x4240   /* Issues a softwre reset on the device. */
#define SX9373_COMMAND                0x4280   /* Start and stop sensing. */

/* Device information. */

#define SX9373_DEVICE_INFO            0x42cc

/* Status registers. */

#define SX9373_DEVICE_STATUS_A        0x8000
#define SX9373_DEVICE_STATUS_B        0x8004
#define SX9373_DEVICE_STATUS_C        0x8008

/* Channel enable control. */

#define SX9373_GENERAL_SETUP          0x8024

/* Four channels AFE parameter setup. */

#define SX9373_AFE_PARAM_PH0          0x8028
#define SX9373_AFE_PARAM_PH1          0x8034
#define SX9373_AFE_PARAM_PH2          0x8040

/* Four channels AFE connection setup. */

#define SX9373_AFE_CS_USAGE_PH0	      0x8030
#define SX9373_AFE_CS_USAGE_PH1	      0x803c
#define SX9373_AFE_CS_USAGE_PH2	      0x8048

/* Proximity threshold. */

#define SX9373_PROX_THRESH_PH0        0x8098
#define SX9373_PROX_THRESH_PH1        0x80b8
#define SX9373_PROX_THRESH_PH2        0x80d8

/* Offset value readback. */

#define SX9373_OFFSET_PH0             0x802c
#define SX9373_OFFSET_PH1             0x8038
#define SX9373_OFFSET_PH2             0x8044

/* Useful value readback. */

#define SX9373_USEFUL_PH0             0x81f0
#define SX9373_USEFUL_PH1             0x81f4
#define SX9373_USEFUL_PH2             0x81f8

/* Average value readback. */

#define SX9373_AVERAGE_PH0            0x8210
#define SX9373_AVERAGE_PH1            0x8214
#define SX9373_AVERAGE_PH2            0x8218

/* Diff value readback. */

#define SX9373_DIFF_PH0               0x8230
#define SX9373_DIFF_PH1               0x8234
#define SX9373_DIFF_PH2               0x8238

/* Channel0 status register mask. */

#define SX9373_CH0PROX1_MASK          1 << 24
#define SX9373_CH0PROX2_MASK          1 << 16
#define SX9373_CH0PROX3_MASK          1 << 8
#define SX9373_CH0PROX4_MASK          1 << 0

/* Channels of the SX9373 used. */

#define SX9373_MAIN_CHANNEL           0        /* Main channel. */
#define SX9373_REF_CHANNEL            1        /* Reference channel. */
#define SX9373_SHIELD_CHANNEL         2        /* Shielded channel. */

/* Control opcode. */

#define SX9373_PHASE_CONTROL          0x000f   /* Phase control opcode. */
#define SX9373_COMPENSATION_CONTROL   0x000e   /* Compensation control opcode. */
#define SX9373_ENTER_CONTROL          0x000d   /* Enter control opcode. */
#define SX9373_EXIT_CONTROL           0x000c   /* Exit control opcode. */
#define SX9373_RESET_CONTROL          0x00de   /* Reset control opcode. */

/* Work queue polling time. */

#define SX9373_POLL_INTERVAL          200000   /* Poll interval(us). */

/* Read the maximum length of a line in the configuration file. */

#define SX9373_MAX_READLINE           30

/* Raw data need to read three channels, otherwise only one. */

#ifdef CONFIG_SENSOR_SX9373_RAWDATA
#define SX9373_CH_VALID               3
#else
#define SX9373_CH_VALID               1
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Structure for param information. */

struct sx9373_param_s
{
  uint16_t reg;           /* Address of register. */
  uint32_t val;           /* Value of register. */
};

/* Enumerate  of channel state. */

enum prox_state_e
{
  PROX_STATE_0,
  PROX_STATE_1,
  PROX_STATE_2,
  PROX_STATE_3,
  PROX_STATE_4,
};

/* Structure of rawdata. */

struct sx9373_rawdata_s
{
  int32_t usef;              /* Valid data after low pass filter. */
  int32_t aver;              /* Valid data after averaging filter. */
  int32_t diff;              /* Difference between useful and average. */
  int32_t offset;            /* Actual detected cap change value. */
  int32_t status;            /* Data of status registers. */
};

/* SX9373 that need to be initialized to config values. */

static const struct sx9373_param_s g_sx9373_default[] =
{
  {SX9373_AFE_PARAM_PH0, 0x0000446e},    /* AFE_PARAM_PH0. */
  {SX9373_AFE_PARAM_PH1, 0x0000446e},    /* AFE_PARAM_PH1. */
  {SX9373_AFE_PARAM_PH2, 0x0000446e},    /* AFE_PARAM_PH2. */
  {SX9373_AFE_CS_USAGE_PH0, 0x000001f5}, /* REG_AFEPH_PH0. */
  {SX9373_AFE_CS_USAGE_PH1, 0x000001f7}, /* REG_AFEPH_PH1. */
  {SX9373_AFE_CS_USAGE_PH2, 0x00000177}, /* REG_AFEPH_PH2. */
  {SX9373_PROX_THRESH_PH0, 0x00008e8e},  /* prox threshold ph0. */
  {SX9373_PROX_THRESH_PH1, 0x00000000},  /* prox threshold ph1. */
  {SX9373_PROX_THRESH_PH2, 0x00000000},  /* prox threshold ph2. */
  {SX9373_GENERAL_SETUP, 0x00000707},    /* PHEN. */
};

/* Sensor struct */

struct sx9373_channel_s
{
  /* sensor_lowerhalf_s must be in the first line. */

  struct sensor_lowerhalf_s lower;       /* Lower half sensor driver */
  FAR struct sx9373_dev_s *dev;          /* Point to the device struct */
  bool activated;                        /* If it's activated now. */
};

/* Structure for sx9373 device. */

struct sx9373_dev_s
{
  struct sx9373_channel_s cap_ch[SX9373_CH_VALID];  /* Infomartion of channel. */
  FAR const struct sx9373_config_s *config;         /* Pointer to the cfg struct. */
  uint64_t timestamp;                               /* Units is microseconds. */
  struct work_s work;                               /* Work queue for read data. */
  FAR const char *file_path;                        /* File path of parameter. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C functions. */

static int sx9373_i2c_read(FAR struct sx9373_dev_s *priv,
                           uint16_t regaddr, FAR uint32_t *regval);
static int sx9373_i2c_write(FAR struct sx9373_dev_s *priv,
                            uint16_t regaddr, uint32_t regval);

/* Sensor handle functions. */

static int sx9373_checkid(FAR struct sx9373_dev_s *priv);
static int sx9373_disable_irq (FAR struct sx9373_dev_s *priv);
static int sx9373_softreset(FAR struct sx9373_dev_s *priv);
static int sx9373_suspend(FAR struct sx9373_dev_s *priv);
static int sx9373_resume(FAR struct sx9373_dev_s *priv);
static int sx9373_enable(FAR struct sx9373_dev_s *priv, bool enable);
static int sx9373_init(FAR struct sx9373_dev_s *priv);
static int sx9373_setparam(FAR struct sx9373_dev_s *priv,
                           FAR const struct sx9373_param_s *param, int len);
static int sx9373_readstate(FAR struct sx9373_dev_s *priv,
                            FAR struct sx9373_rawdata_s *rawdata);
#ifdef CONFIG_SENSOR_SX9373_RAWDATA
static int sx9373_read_rawdata(FAR struct sx9373_dev_s *priv,
                               FAR struct sx9373_rawdata_s *rawdata,
                               int channel);
#endif
static int sx9373_read_cfg_param(FAR struct file *file,
                                 FAR char *buf, int len);
static int sx9373_write_cfg_param(FAR struct sx9373_dev_s *priv,
                                  FAR struct file *file);
static int sx9373_config_device(FAR struct sx9373_dev_s *priv);

/* Sensor ops functions. */

static int sx9373_activate(FAR struct file *filep,
                           FAR struct sensor_lowerhalf_s *lower,
                           bool enable);
static int sx9373_selftest(FAR struct file *filep,
                           FAR struct sensor_lowerhalf_s *lower,
                           unsigned long arg);
static int sx9373_calibrate(FAR struct file *filep,
                            FAR struct sensor_lowerhalf_s *lower,
                            unsigned long arg);

/* Sensor poll functions. */

static void sx9373_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sx9373_ops =
{
  .activate = sx9373_activate,               /* Enable/disable snesor. */
  .selftest = sx9373_selftest,               /* Sensor selftest function. */
  .calibrate = sx9373_calibrate,             /* Calibration Operations. */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* I2C functions */

/****************************************************************************
 * Name: sx9373_i2c_read
 *
 * Description:
 *   Read 32-bit register.
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

static int sx9373_i2c_read(FAR struct sx9373_dev_s *priv,
                           uint16_t regaddr, FAR uint32_t *regval)
{
  struct i2c_msg_s msg[2];
  uint8_t w_buf[2];
  uint8_t buf[4];
  int ret;

  w_buf[0] = (uint8_t)(regaddr >> 8);
  w_buf[1] = (uint8_t)regaddr;

  msg[0].frequency = priv->config->freq;
  msg[0].addr      = priv->config->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = w_buf;
  msg[0].length    = 2;

  msg[1].frequency = priv->config->freq;
  msg[1].addr      = priv->config->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buf;
  msg[1].length    = 4;

  ret = I2C_TRANSFER(priv->config->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
      return ret;
    }

  *regval = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
            ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3]);

  return ret;
}

/****************************************************************************
 * Name: sx9373_i2c_write
 *
 * Description:
 *   write 32-bit register.
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

static int sx9373_i2c_write(FAR struct sx9373_dev_s *priv,
                            uint16_t regaddr, uint32_t regval)
{
  struct i2c_msg_s msg;
  int ret;
  uint8_t w_buf[6];

  w_buf[0] = (uint8_t)(regaddr >> 8);
  w_buf[1] = (uint8_t)regaddr;
  w_buf[2] = (uint8_t)(regval >> 24);
  w_buf[3] = (uint8_t)(regval >> 16);
  w_buf[4] = (uint8_t)(regval >> 8);
  w_buf[5] = (uint8_t)regval;

  msg.frequency = priv->config->freq;
  msg.addr      = priv->config->addr;
  msg.flags     = 0;
  msg.buffer    = w_buf;
  msg.length    = 6;

  ret = I2C_TRANSFER(priv->config->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sx9373_checkid
 *
 * Description:
 *   Read and verify the SX9373 chip ID
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_checkid(FAR struct sx9373_dev_s *priv)
{
  int ret;
  uint32_t devid;

  ret = sx9373_i2c_read(priv, SX9373_DEVICE_INFO, &devid);
  if (ret < 0 || devid != SX9373_WHOAMI_VALUE)
    {
      snerr("Wrong Device ID: %ld\n", devid);
      ret = -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: sx9373_disable_irq
 *
 * Description:
 *   disable SX9373 irq
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_disable_irq(FAR struct sx9373_dev_s *priv)
{
  int ret;
  int value;

  /* read SX9373_IRQ_SOURCE to pull up irq pin, reduce leakage */

  ret = sx9373_i2c_read(priv, SX9373_IRQ_SOURCE, (FAR uint32_t *)&value);
  if (ret < 0)
    {
      snerr("Failed to clear IRQ: %d\n", ret);
    }

  ret = sx9373_i2c_write(priv, SX9373_IRQ_DISABLE, 0x0);
  if (ret < 0)
    {
      snerr("disable irq: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sx9373_softreset
 *
 * Description:
 *   Reset SX9373 device
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_softreset(FAR struct sx9373_dev_s *priv)
{
  int ret;

  ret = sx9373_i2c_write(priv, SX9373_DEVICE_RESET, SX9373_RESET_CONTROL);
  if (ret < 0)
    {
      snerr("Failed to reset SX9373: %d\n", ret);
    }
  else
    {
      nxsig_usleep(50000);
    }

  ret = sx9373_disable_irq(priv);
  if (ret < 0)
    {
      snerr("Failed to disable irq SX9373: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sx9373_setparam
 *
 * Description:
 *   Set device parameters of sx9373
 *
 * Input Parameters
 *   priv     -Device struct
 *   param    -Parameters of cap sensor
 *   len      -Length of parameter
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_setparam(FAR struct sx9373_dev_s *priv,
                           FAR const struct sx9373_param_s *param, int len)
{
  int ret;
  int i;

  for (i = 0 ; i < len; i++)
    {
      ret = sx9373_i2c_write(priv, param[i].reg, param[i].val);
      if (ret < 0)
        {
          snerr("Failed to set param: %d\n", ret);
          return ret;
        }
    }

  ret = sx9373_i2c_write(priv, SX9373_COMMAND, SX9373_PHASE_CONTROL);
  if (ret < 0)
    {
      snerr("Failed to set param: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: sx9373_readstate
 *
 * Description:
 *   Read the status of device detection .
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_readstate(FAR struct sx9373_dev_s *priv,
                            FAR struct sx9373_rawdata_s *rawdata)
{
  uint32_t buf;
  int ret;

  ret = sx9373_i2c_read(priv, SX9373_DEVICE_STATUS_A, &buf);
  if (ret < 0)
    {
      snerr("Failed to read status register: %d\n", ret);
      return ret;
    }
  else
    {
      if (buf & SX9373_CH0PROX4_MASK)
        {
          rawdata->status = PROX_STATE_4;
        }
      else if (buf & SX9373_CH0PROX3_MASK)
        {
          rawdata->status = PROX_STATE_3;
        }
      else if (buf & SX9373_CH0PROX2_MASK)
        {
          rawdata->status = PROX_STATE_2;
        }
      else if (buf & SX9373_CH0PROX1_MASK)
        {
          rawdata->status = PROX_STATE_1;
        }
      else
        {
          rawdata->status = PROX_STATE_0;
        }
    }

  return ret;
}

#ifdef CONFIG_SENSOR_SX9373_RAWDATA

/****************************************************************************
 * Name: sx9373_read_rawdata
 *
 * Description:
 *   Set device parameters of sx9373
 *
 * Input Parameters
 *   priv     -Device struct
 *   rawdata  -Rawdata of sx9373
 *   channel  -Data channel of sx9373
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_read_rawdata(FAR struct sx9373_dev_s *priv,
                               FAR struct sx9373_rawdata_s *rawdata,
                               int channel)
{
  uint32_t buf;
  int ret;

  ret = sx9373_i2c_read(priv, SX9373_USEFUL_PH0 + 4 * channel, &buf);
  if (ret < 0)
    {
      snerr("Failed to read useful data: %d\n", ret);
      return ret;
    }
  else
    {
      rawdata->usef = (int32_t)buf >> 10;
    }

  ret = sx9373_i2c_read(priv, SX9373_AVERAGE_PH0 + 4 * channel, &buf);
  if (ret < 0)
    {
      snerr("Failed to read average data: %d\n", ret);
      return ret;
    }
  else
    {
      rawdata->aver = (int32_t)buf >> 10;
    }

  ret = sx9373_i2c_read(priv, SX9373_DIFF_PH0 + 4 * channel, &buf);
  if (ret < 0)
    {
      snerr("Failed to read diff data: %d\n", ret);
      return ret;
    }
  else
    {
      rawdata->diff = (int32_t)buf >> 10;
    }

  ret = sx9373_i2c_read(priv, SX9373_OFFSET_PH0 + 4 * channel, &buf);
  if (ret < 0)
    {
      snerr("Failed to read offest data: %d\n", ret);
      return ret;
    }
  else
    {
      rawdata->offset = (int32_t)(buf & 0x3fff);
    }

  return ret;
}

#endif

/****************************************************************************
 * Name: sx9373_suspend
 *
 * Description:
 *   Suspend the device, make the device dormant.
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_suspend(FAR struct sx9373_dev_s *priv)
{
  int ret;

  ret = sx9373_i2c_write(priv, SX9373_COMMAND,
                         SX9373_ENTER_CONTROL);
  if (ret < 0)
    {
      snerr("Failed to suspend device: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sx9373_resume
 *
 * Description:
 *   Wake up the device to make it work.
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_resume(FAR struct sx9373_dev_s *priv)
{
  int ret;

  ret = sx9373_i2c_write(priv, SX9373_COMMAND,
                         SX9373_EXIT_CONTROL);
  if (ret < 0)
    {
      snerr("Failed to resume device: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sx9373_enable
 *
 * Description:
 *   SX9373 control on/off
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_enable(FAR struct sx9373_dev_s *priv, bool enable)
{
  int ret;

  if (enable)
    {
      /* Resume the device. */

      ret = sx9373_resume(priv);
      if (ret < 0)
        {
          snerr("Failed to resume device: %d\n", ret);
          return ret;
        }

      work_queue(HPWORK, &priv->work, sx9373_worker, priv,
                 SX9373_POLL_INTERVAL / USEC_PER_TICK);
    }
  else
    {
      work_cancel(HPWORK, &priv->work);

      /* Suspend the device. */

      ret = sx9373_suspend(priv);
      if (ret < 0)
        {
          snerr("Failed to suspend device: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sx9373_init
 *
 * Description:
 *   Device initialization
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_init(FAR struct sx9373_dev_s *priv)
{
  int len = sizeof(g_sx9373_default) / sizeof(struct sx9373_param_s);
  int ret;

  /* Soft reset device. */

  ret = sx9373_softreset(priv);
  if (ret < 0)
    {
      snerr("Failed to reset device: %d\n", ret);
      return ret;
    }

  /* set phase parameter. */

  ret = sx9373_setparam(priv, g_sx9373_default, len);
  if (ret < 0)
    {
      snerr("Failed to set param: %d\n", ret);
      return ret;
    }

  /* suspend the device. */

  ret = sx9373_enable(priv, false);
  if (ret < 0)
    {
      snerr("Failed to suspend device: %d\n", ret);
    }

  return ret;
}

/* Sensor ops functions */

/****************************************************************************
 * Name: sx9373_activate
 *
 * Description:
 *   Enable or disable sensor device.
 *
 * Input Parameters
 *   filep  - The pointer of file, represents each user using the sensor.
 *   lower  - The instance of lower half sensor driver.
 *   enable - True(enable) and false(disable).
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_activate(FAR struct file *filep,
                           FAR struct sensor_lowerhalf_s *lower,
                           bool enable)
{
  FAR struct sx9373_channel_s *cap_ch = (FAR struct sx9373_channel_s *)lower;
  FAR struct sx9373_dev_s *priv = cap_ch->dev;
  int idx;
  int ret;

  if (lower->type != SENSOR_TYPE_CAP)
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  if (!enable)
    {
      cap_ch->activated = enable;
    }

  for (idx = 0; idx < SX9373_CH_VALID; idx++)
    {
      if (priv->cap_ch[idx].activated)
        {
          break;
        }
    }

  if (idx == SX9373_CH_VALID)
    {
      ret = sx9373_enable(priv, enable);
      if (ret < 0)
        {
          snerr("Failed to enable cap sensor: %d\n", ret);
          return ret;
        }
    }

  cap_ch->activated = enable;

  return OK;
}

/****************************************************************************
 * Name: sx9373_selftest
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

static int sx9373_selftest(FAR struct file *filep,
                           FAR struct sensor_lowerhalf_s *lower,
                           unsigned long arg)
{
  FAR struct sx9373_channel_s *cap_ch = (FAR struct sx9373_channel_s *)lower;
  FAR struct sx9373_dev_s *priv = cap_ch->dev;
  int ret = -ENOTTY;

  DEBUGASSERT(lower != NULL);

  /* Process ioctl commands. */

  switch (arg)
    {
      case SNIOC_SIMPLE_CHECK:       /* Simple communication check. */
        {
          ret = sx9373_checkid(priv);
          if (ret < 0)
            {
              snerr("Failed to get DeviceID: %d\n", ret);
            }
        }
        break;

      default:                       /* Other cmd tag. */
        {
          snerr("The cmd don't support: %d\n", ret);
        }
        break;
    }

    return ret;
}

/****************************************************************************
 * Name: sx9373_calibrate
 *
 * Description:
 * Trigger clear calibration in an empty state.
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

static int sx9373_calibrate(FAR struct file *filep,
                            FAR struct sensor_lowerhalf_s *lower,
                            unsigned long arg)
{
  FAR struct sx9373_channel_s *cap_ch = (FAR struct sx9373_channel_s *)lower;
  FAR struct sx9373_dev_s *priv = cap_ch->dev;
  FAR struct sx9373_param_s param;
  int ret;

  DEBUGASSERT(lower != NULL);

  param.reg = SX9373_COMMAND;
  param.val = SX9373_COMPENSATION_CONTROL;

  ret = sx9373_setparam(priv, &param, 1);
  if (ret < 0)
    {
      snerr("Failed to set param: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sx9373_read_cfg_param
 *
 * Description:
 *   Read device parameters of sx9373
 *
 * Input Parameters
 *   file     -File path of parameters
 *   buf      -Buffer of parameters
 *   len      -Length of read
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_read_cfg_param(FAR struct file *file,
                                 FAR char *buf, int len)
{
  int i;

  len = file_read(file, buf, len);
  if (len <= 0)
    {
      len = file_read(file, buf, len);
      if (len <= 0)
        {
          snerr("Failed to read parameters: %d\n", -EINVAL);
          return -EINVAL;
        }
    }

  for (i = 0; i < len; i++)
    {
      if (buf[i] == '\n')
        {
          file_seek(file, i - len + 1, SEEK_CUR);
          buf[i + 1] = '\0';
          break;
        }
    }

  return i + 1;
}

/****************************************************************************
 * Name: sx9373_write_cfg_param
 *
 * Description:
 *   Device function configuration
 *
 * Input Parameters
 *   priv     -Device struct
 *   file     -Config information
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_write_cfg_param(FAR struct sx9373_dev_s *priv,
                                  FAR struct file *file)
{
  char buf[SX9373_MAX_READLINE];
  struct sx9373_param_s param;
  int count;
  int ret;
  int i;

  /* Read head information of config file. */

  ret = sx9373_read_cfg_param(file, buf, sizeof(buf));
  if (ret < 0)
    {
      snerr("Failed to read param: %d\n", ret);
      return ret;
    }
  else
    {
      if (!strstr(buf, "Device: sx9373"))
        {
          snerr("Failed to read head information: %d\n", -EINVAL);
          return -EINVAL;
        }
    }

  /* Read parameter length of config file. */

  ret = sx9373_read_cfg_param(file, buf, sizeof(buf));
  if (ret < 0)
    {
      snerr("Failed to read param: %d\n", ret);
      return ret;
    }
  else
    {
      sscanf(buf, "Length: %d\n", &count);
    }

  /* Set phase parameter. */

  for (i = 0; i < count; i++)
    {
      ret = sx9373_read_cfg_param(file, buf, sizeof(buf));
      if (ret < 0)
        {
          snerr("Failed to read param: %d\n", ret);
          return ret;
        }
      else
        {
          sscanf(buf, "%hx, %lx\n", &param.reg, &param.val);
          ret = sx9373_setparam(priv, &param, 1);
          if (ret < 0)
            {
              snerr("Failed to set param: %d\n", ret);
              return ret;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sx9373_config_device
 *
 * Description:
 *   It is mainly used to initialize the device. Read the config file and
 *   initialize the device.
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

static int sx9373_config_device(FAR struct sx9373_dev_s *priv)
{
  struct file file;
  int ret;

  /* Open file of config parameter. */

  ret = file_open(&file, priv->file_path, O_RDONLY);
  if (ret < 0)
    {
      snerr("Failed to open file:%s, err:%d\n", priv->file_path, ret);
      return ret;
    }

  /* Configure the device. */

  ret = sx9373_write_cfg_param(priv, &file);
  if (ret < 0)
    {
      snerr("Failed to config device: %d\n", ret);
    }

  file_close(&file);

  return ret;
}

/* Sensor poll functions */

/****************************************************************************
 * Name: sx9373_worker
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

static void sx9373_worker(FAR void *arg)
{
  FAR struct sx9373_dev_s *priv = arg;
  struct sx9373_rawdata_s rawdata;
  struct sensor_cap cap;
  int ret;

  DEBUGASSERT(priv != NULL);

  memset(&cap, 0, sizeof(struct sensor_cap));
  cap.timestamp = sensor_get_timestamp();

  /* Set work queue. */

  work_queue(HPWORK, &priv->work, sx9373_worker,
             priv, SX9373_POLL_INTERVAL / USEC_PER_TICK);

  ret = sx9373_readstate(priv, &rawdata);
  if (ret < 0)
    {
      snerr("Failed to read state: %d\n", ret);
      return;
    }
  else
    {
      cap.status = rawdata.status;
    }

#ifdef CONFIG_SENSOR_SX9373_RAWDATA

  /* Obtain raw data. */

  for (int i = 0; i < SX9373_CH_NUM; i++)
    {
      ret = sx9373_read_rawdata(priv, &rawdata, i);
      if (ret < 0)
        {
          snerr("Failed to read rawdata: %d\n", ret);
          return;
        }
      else
        {
          memcpy(&cap.rawdata, &rawdata, sizeof(cap.rawdata));

          /* push data to upper half driver */

          priv->cap_ch[i].lower.push_event(
            priv->cap_ch[i].lower.priv, &cap, sizeof(cap));
        }
    }

#else

  /* push data to upper half driver */

  priv->cap_ch[SX9373_MAIN_CHANNEL].lower.push_event(
    priv->cap_ch[SX9373_MAIN_CHANNEL].lower.priv, &cap, sizeof(cap));

#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx9373_register
 *
 * Description:
 *   Register the SX9373 character device as 'devpath'.
 *
 * Input Parameters:
 *   devno   - The device number, used to build the device path
 *             as /dev/sensor/proxN
 *   config  - configuration for the sx9373 driver.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sx9373_register(int devno, FAR const struct sx9373_config_s *config)
{
  FAR struct sx9373_dev_s *priv;
  int idx;
  int ret;

  /* Sanity check. */

  DEBUGASSERT(config != NULL);

  /* Initialize the SX9373 device structure. */

  priv = kmm_zalloc(sizeof(struct sx9373_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      ret = -ENOMEM;
      goto err;
    }

  priv->config = config;
  priv->file_path = config->file_path;

  for (idx = 0; idx < SX9373_CH_VALID; idx++)
    {
      priv->cap_ch[idx].dev = priv;
      priv->cap_ch[idx].lower.type = SENSOR_TYPE_CAP;
      priv->cap_ch[idx].lower.ops = &g_sx9373_ops;
      priv->cap_ch[idx].lower.nbuffer =
        CONFIG_SENSORS_SX9373_BUFFER_NUMBER;
    }

  /* Check Device ID. */

  ret = sx9373_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      goto err;
    }

  /* Init the device. */

  ret = sx9373_init(priv);
  if (ret < 0)
    {
      snerr("Failed to initialize physical device sx9373:%d\n", ret);
      goto err;
    }

  /* Start config for sensor. */

  ret = sx9373_config_device(priv);
  if (ret < 0)
    {
      snerr("Failed to config sx9373 sensor: %d\n", ret);
      goto err;
    }

  /* Register the sensor driver. */

  for (idx = 0; idx < SX9373_CH_VALID; idx++)
    {
      ret = sensor_register((&(priv->cap_ch[idx].lower)),
                            devno * SX9373_CH_VALID + idx);
      if (ret < 0)
        {
          snerr("Failed to register CAP%d driver: %d\n", idx, ret);

          /* Unregister all registered cap sensors */

          idx--;
          for (; idx >= 0; idx--)
            {
              sensor_unregister((&(priv->cap_ch[idx].lower)),
                                devno * SX9373_CH_VALID + idx);
            }

          goto err;
        }
    }

  return ret;

err:
  kmm_free(priv);
  return ret;
}
