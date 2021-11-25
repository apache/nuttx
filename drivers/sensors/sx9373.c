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

#include <nuttx/nuttx.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/sx9373.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SX9373_WHOAMI_VALUE           0x937327 /* Device ID */
#define SX9373_CHANNEL_NUM            2        /* Number of channel */
#define SX9373_MAX_RETRY              3        /* Clear interrupt attempts. */
#define SX9373_POWERON_DELAY          3000     /* Power-on delay time(us). */

/* Interrupt control. */

#define SX9373_IRQ_SOURCE             0x4000   /* Current interrupt events. */
#define SX9373_IRQ_MASK_A             0x4004   /* Interrupt event configuration. */
#define SX9373_IRQ_MASK_B             0x800c   /* Interrupt event configuration. */
#define SX9373_IRQ_SETUP              0x4008   /* Interrupt pin configuration. */

/* Device state setup. */

#define SX9373_DEVICE_RESET           0x4240   /* Issues a softwre reset on the device. */
#define SX9373_COMMAND                0x4280   /* Start and stop sensing. */
#define SX9373_CMMAND_BUSY            0x4284   /* Flag corresponding to command register */

/* Pin customization. */

#define SX9373_PIN_SETUP_A            0x42c0
#define SX9373_PIN_SETUP_B            0x42c4

/* Device information. */

#define SX9373_DEVICE_INFO            0x42cc

/* Status registers. */

#define SX9373_DEVICE_STATUS_A        0x8000
#define SX9373_DEVICE_STATUS_B        0x8004
#define SX9373_DEVICE_STATUS_C        0x8008

/* Status output setup. */

#define SX9373_STATUS_OUTPUT_0        0x8010
#define SX9373_STATUS_OUTPUT_1        0x8014
#define SX9373_STATUS_OUTPUT_2        0x8018
#define SX9373_STATUS_OUTPUT_3        0x801c

/* Scan frequency setup. */

#define SX9373_SCAN_PERIOD_SETUP      0x8020

/* Channel enable control. */

#define SX9373_GENERAL_SETUP          0x8024

/* Four channels AFE parameter setup. */

#define SX9373_AFE_PARAM_PH0          0x8028
#define SX9373_AFE_PARAM_PH1          0x8034
#define SX9373_AFE_PARAM_PH2          0x8040
#define SX9373_AFE_PARAM_PH3          0x804c

/* Four channels AFE connection setup. */

#define SX9373_AFE_CS_USAGE_PH0	      0x8030
#define SX9373_AFE_CS_USAGE_PH1	      0x803c
#define SX9373_AFE_CS_USAGE_PH2	      0x8048
#define SX9373_AFE_CS_USAGE_PH3	      0x8054

/* Four channels ADC, RAW filter and debounce filter. */

#define SX9373_FILTER_SETUP_A_PH0     0x8088
#define SX9373_FILTER_SETUP_A_PH1     0x80a8
#define SX9373_FILTER_SETUP_A_PH2     0x80c8
#define SX9373_FILTER_SETUP_A_PH3     0x80e8

/* Four channels average and usefilter filter. */

#define SX9373_FILTER_SETUP_B_PH0     0x808c
#define SX9373_FILTER_SETUP_B_PH1     0x80ac
#define SX9373_FILTER_SETUP_B_PH2     0x80cc
#define SX9373_FILTER_SETUP_B_PH3     0x80ec

/* Filter setup C. */

#define SX9373_USE_FLT_SETUP_PH0      0x8090
#define SX9373_USE_FLT_SETUP_PH1      0x80b0
#define SX9373_USE_FLT_SETUP_PH2      0x80d0
#define SX9373_USE_FLT_SETUP_PH3      0x80f0

/* Filter setup D. */

#define SX9373_ADC_QUICK_FILTER_0     0x81e0
#define SX9373_ADC_QUICK_FILTER_1     0x81e4
#define SX9373_ADC_QUICK_FILTER_2     0x81e8
#define SX9373_ADC_QUICK_FILTER_3     0x81ec

/* Steady and saturation setup. */

#define SX9373_STEADY_SATURATION_PH0  0x809c
#define SX9373_STEADY_SATURATION_PH1  0x80bc
#define SX9373_STEADY_SATURATION_PH2  0x80dc
#define SX9373_STEADY_SATURATION_PH3  0x80fc

/* Failure threshold setup. */

#define SX9373_FAILURE_THRESHOLD_PH0  0x80a4
#define SX9373_FAILURE_THRESHOLD_PH1  0x80c4
#define SX9373_FAILURE_THRESHOLD_PH2  0x80e4
#define SX9373_FAILURE_THRESHOLD_PH3  0x8104

/* Proximity threshold. */

#define SX9373_PROX_THRESH_PH0        0x8098
#define SX9373_PROX_THRESH_PH1        0x80b8
#define SX9373_PROX_THRESH_PH2        0x80d8
#define SX9373_PROX_THRESH_PH3        0x80f8

/* Startup setup. */

#define SX9373_STARTUP_PH0            0x8094
#define SX9373_STARTUP_PH1            0x80B4
#define SX9373_STARTUP_PH2            0x80D4
#define SX9373_STARTUP_PH3            0x80F4

/* Reference correction setup A. */

#define SX9373_REF_CORRECTION_PH0     0x80a0
#define SX9373_REF_CORRECTION_PH1     0x80c0
#define SX9373_REF_CORRECTION_PH2     0x80e0
#define SX9373_REF_CORRECTION_PH3     0x8100

/* Reference correction setup B. */

#define SX9373_REF_ENGINE_1_CONFIG    0x8188
#define SX9373_REF_ENGINE_2_CONFIG    0x818c
#define SX9373_REF_ENGINE_3_CONFIG    0x8190
#define SX9373_REF_ENGINE_4_CONFIG    0x8194

/* Smart human sensing setup A. */

#define SX9373_ENGINE_1_CONFIG        0x8198
#define SX9373_ENGINE_1_X0            0x819c
#define SX9373_ENGINE_1_X1            0x81a0
#define SX9373_ENGINE_1_X2            0x81a4
#define SX9373_ENGINE_1_X3            0x81a8
#define SX9373_ENGINE_1_Y0            0x81aC
#define SX9373_ENGINE_1_Y1            0x81b0
#define SX9373_ENGINE_1_Y2            0x81b4
#define SX9373_ENGINE_1_Y3            0x81b8

/* Smart human sensing setup B. */

#define SX9373_ENGINE_2_CONFIG        0x81bC
#define SX9373_ENGINE_2_X0            0x81c0
#define SX9373_ENGINE_2_X1            0x81c4
#define SX9373_ENGINE_2_X2            0x81c8
#define SX9373_ENGINE_2_X3            0x81cc
#define SX9373_ENGINE_2_Y0            0x81d0
#define SX9373_ENGINE_2_Y1            0x81d4
#define SX9373_ENGINE_2_Y2            0x81d8
#define SX9373_ENGINE_2_Y3            0x81dc

/* Offset value readback. */

#define SX9373_OFFSET_PH0             0x802c
#define SX9373_OFFSET_PH1             0x8038
#define SX9373_OFFSET_PH2             0x8044
#define SX9373_OFFSET_PH3             0x8050

/* Useful value readback. */

#define SX9373_USEFUL_PH0             0x81f0
#define SX9373_USEFUL_PH1             0x81f4
#define SX9373_USEFUL_PH2             0x81f8
#define SX9373_USEFUL_PH3             0x81fc

/* UseFilter value readback. */

#define SX9373_USEFILTER_PH0          0x8250
#define SX9373_USEFILTER_PH1          0x8254
#define SX9373_USEFILTER_PH2          0x8258
#define SX9373_USEFILTER_PH3          0x825c

/* Average value readback. */

#define SX9373_AVERAGE_PH0            0x8210
#define SX9373_AVERAGE_PH1            0x8214
#define SX9373_AVERAGE_PH2            0x8218
#define SX9373_AVERAGE_PH3            0x821c

/* Diff value readback. */

#define SX9373_DIFF_PH0               0x8230
#define SX9373_DIFF_PH1               0x8234
#define SX9373_DIFF_PH2               0x8238
#define SX9373_DIFF_PH3               0x823c

/* Control opcode . */

#define SX9373_PHASE_CONTROL          0x000f   /* Phase control opcode. */
#define SX9373_COMPENSATION_CONTROL   0x000e   /* Compensation control opcode. */
#define SX9373_ENTER_CONTROL          0x000d   /* Enter control opcode. */
#define SX9373_EXIT_CONTROL           0x000c   /* Exit control opcode. */
#define SX9373_RESET_CONTROL          0x00de   /* Reset control opcode. */

/* Factory test instructions. */

#define SX9373_SIMPLE_CHECK           0x00     /* Simple communication check. */

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Enumerate of detection level. */

enum sx9373_level_e
{
  SX9373_LEVEL_0,
  SX9373_LEVEL_1,
  SX9373_LEVEL_2,
  SX9373_LEVEL_3,
  SX9373_LEVEL_4
};

/* Structure for param information. */

struct sx9373_param_s
{
  uint16_t reg;           /* Address of register. */
  uint32_t val;           /* Value of register. */
};

/* Structure for status information of each button. */

struct sx9373_channel_s
{
  uint32_t prox1_mask;                 /* Shield bit1 for each channel. */
  uint32_t prox2_mask;                 /* Shield bit2 for each channel. */
  uint32_t prox3_mask;                 /* Shield bit3 for each channel. */
  uint32_t prox4_mask;                 /* Shield bit4 for each channel. */
  enum sx9373_level_e level;           /* Current detection level. */
};

/* SX9373 that need to be initialized to config values. */

static const struct sx9373_param_s g_sx9373_param[] =
{
  {SX9373_GENERAL_SETUP, 0x00000300},       /* PHEN. */
  {SX9373_IRQ_MASK_A, 0x0000007f},          /* irq mask. */
  {SX9373_AFE_PARAM_PH0, 0x0000085c},       /* AFE_PARAM_PH0. */
  {SX9373_AFE_PARAM_PH1, 0x0000085c},       /* AFE_PARAM_PH1. */
  {SX9373_AFE_CS_USAGE_PH0, 0x00fff9fd},    /* REG_AFEPH_PH0. */
  {SX9373_AFE_CS_USAGE_PH1, 0x00fff9ef},    /* REG_AFEPH_PH1. */
  {SX9373_PROX_THRESH_PH0, 0xc8ad8d64},     /* prox threshold ph0. */
  {SX9373_PROX_THRESH_PH1, 0xc8ad8d64},     /* prox threshold ph1. */
  {SX9373_GENERAL_SETUP, 0x00000303},       /* PHEN. */
};

/* SX9373 information of channels. */

static struct sx9373_channel_s g_sx9373_channel[SX9373_CHANNEL_NUM] =
{
  /* Capture channel. */

  {
    .prox1_mask = 1 << 24,
    .prox2_mask = 1 << 16,
    .prox3_mask = 1 << 8,
    .prox4_mask = 1 << 0,
    .level = SX9373_LEVEL_0,
  },

  /* Reference channel. */

  {
    .prox1_mask = 1 << 25,
    .prox2_mask = 1 << 17,
    .prox3_mask = 1 << 9,
    .prox4_mask = 1 << 1,
    .level = SX9373_LEVEL_0,
  }
};

/* Structure for sx9373 device. */

struct sx9373_dev_s
{
  struct sensor_lowerhalf_s lower;          /* Lower half driver. */
  FAR const struct sx9373_config_s *config; /* Pointer to the cfg struct. */
  uint64_t timestamp;                       /* Units is microseconds. */
  bool activated;                           /* Sensor working state. */
  struct work_s work;                       /* Work queue. */
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
static int sx9373_clearirq(FAR struct sx9373_dev_s *priv);
static int sx9373_softreset(FAR struct sx9373_dev_s *priv);
static int sx9373_setparam(FAR struct sx9373_dev_s *priv);
static int sx9373_suspend(FAR struct sx9373_dev_s *priv);
static int sx9373_resume(FAR struct sx9373_dev_s *priv);
static int sx9373_readstate(FAR struct sx9373_dev_s *priv);
static int sx9373_enable(FAR struct sx9373_dev_s *priv, bool enable);
static int sx9373_init(FAR struct sx9373_dev_s *priv);

/* Sensor ops functions. */

static int sx9373_activate(FAR struct sensor_lowerhalf_s *lower,
                           bool enable);
static int sx9373_selftest(FAR struct sensor_lowerhalf_s *lower,
                           unsigned long arg);

/* Sensor interrupt functions. */

static int sx9373_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset, FAR void *arg);
static void sx9373_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sx9373_ops =
{
  .activate = sx9373_activate,     /* Enable/disable snesor. */
  .selftest = sx9373_selftest      /* Sensor selftest function. */
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
 * Name: sx9373_clearirq
 *
 * Description:
 *   Clear interrupt status of SX9373
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

static int sx9373_clearirq(FAR struct sx9373_dev_s *priv)
{
  uint32_t state;

  return sx9373_i2c_read(priv, SX9373_IRQ_SOURCE, &state);
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
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_setparam(FAR struct sx9373_dev_s *priv)
{
  int ret;
  int i;
  int len = sizeof(g_sx9373_param) / sizeof(struct sx9373_param_s);

  for (i = 0 ; i < len; i++)
    {
      ret = sx9373_i2c_write(priv, g_sx9373_param[i].reg,
                             g_sx9373_param[i].val);
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
      return ret;
    }

  IOEXP_SETOPTION(priv->config->ioe, priv->config->pin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);

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

  IOEXP_SETOPTION(priv->config->ioe, priv->config->pin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_FALLING);

  ret = sx9373_i2c_write(priv, SX9373_COMMAND,
                         SX9373_EXIT_CONTROL);
  if (ret < 0)
    {
      snerr("Failed to resume device: %d\n", ret);
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

static int sx9373_readstate(FAR struct sx9373_dev_s *priv)
{
  uint32_t prox_state;
  int ret;
  int i;

  ret = sx9373_i2c_read(priv, SX9373_DEVICE_STATUS_A, &prox_state);
  if (ret < 0)
    {
      snerr("Failed to read state: %d\n", ret);
      return ret;
    }

  for (i = 0; i < SX9373_CHANNEL_NUM; i++)
    {
      if (prox_state & g_sx9373_channel[i].prox4_mask)
        {
          g_sx9373_channel[i].level = SX9373_LEVEL_4;
        }
      else if (prox_state & g_sx9373_channel[i].prox3_mask)
        {
          g_sx9373_channel[i].level = SX9373_LEVEL_3;
        }
      else if (prox_state & g_sx9373_channel[i].prox2_mask)
        {
          g_sx9373_channel[i].level = SX9373_LEVEL_2;
        }
      else if (prox_state & g_sx9373_channel[i].prox1_mask)
        {
          g_sx9373_channel[i].level = SX9373_LEVEL_1;
        }
      else
        {
          g_sx9373_channel[i].level = SX9373_LEVEL_0;
        }
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
      ret = sx9373_resume(priv);
      if (ret < 0)
        {
          snerr("Failed to resume device: %d\n", ret);
          return ret;
        }

      /* Make sure no interrupts are pending. */

      ret = sx9373_clearirq(priv);
      if (ret < 0)
        {
          snerr("Failed to read state: %d\n", ret);
        }
    }
  else
    {
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
  int ret;

  /* Soft reset device. */

  ret = sx9373_softreset(priv);
  if (ret < 0)
    {
      snerr("Failed to reset device: %d\n", ret);
      return ret;
    }

  /* set phase parameter. */

  ret = sx9373_setparam(priv);
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
 *   Enable or disable sensor device
 *
 * Input Parameters
 *   lower     -The instance of lower half sensor driver
 *   enable    -true(enable) and false(disable)
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int sx9373_activate(FAR struct sensor_lowerhalf_s *lower,
                           bool enable)
{
  FAR struct sx9373_dev_s *priv = (FAR struct sx9373_dev_s *)lower;
  int ret;

  if (lower->type != SENSOR_TYPE_PROXIMITY)
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  if (priv->activated != enable)
    {
      ret = sx9373_enable(priv, enable);
      if (ret < 0)
        {
          snerr("Failed to enable cap sensor: %d\n", ret);
          return ret;
        }

      priv->activated = enable;
    }

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

static int sx9373_selftest(FAR struct sensor_lowerhalf_s *lower,
                           unsigned long arg)
{
  FAR struct sx9373_dev_s *priv = (FAR struct sx9373_dev_s *)lower;
  int ret = -ENOTTY;

  DEBUGASSERT(lower != NULL);

  /* Process ioctl commands. */

  switch (arg)
    {
      case SX9373_SIMPLE_CHECK:      /* Simple communication check. */
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

/* Sensor interrupt functions */

/****************************************************************************
 * Name: sx9373_interrupt_handler
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

static int sx9373_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset, FAR void *arg)
{
  /* This function should be called upon a rising edge on the SX9373 new
   * data interrupt pin since it signals that new data has been measured.
   */

  FAR struct sx9373_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp */

  priv->timestamp = sensor_get_timestamp();

  /* Task the worker with retrieving the latest sensor data. We should not
   * do this in a interrupt since it might take too long. Also we cannot lock
   * the I2C bus from within an interrupt.
   */

  IOEXP_SETOPTION(priv->config->ioe, priv->config->pin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_DISABLE);

  work_queue(LPWORK, &priv->work, sx9373_worker, priv, 0);

  return OK;
}

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
  struct sensor_event_prox prox;
  uint8_t cnt = 0;

  DEBUGASSERT(priv != NULL);

  /* Make sure no interrupts are pending. */

  while (cnt++ < SX9373_MAX_RETRY)
    {
      if (sx9373_clearirq(priv) >= 0)
        {
          break;
        }
    }

  DEBUGASSERT(cnt != SX9373_MAX_RETRY);

  IOEXP_SETOPTION(priv->config->ioe, priv->config->pin,
                  IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_FALLING);

  if (sx9373_readstate(priv) >= 0)
    {
      prox.timestamp = priv->timestamp;
      prox.proximity = (float)g_sx9373_channel[0].level;

      /* push data to upper half driver */

      priv->lower.push_event(priv->lower.priv, &prox, sizeof(prox));
    }
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
  FAR void *ioephanle;
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
  priv->lower.type = SENSOR_TYPE_PROXIMITY;
  priv->lower.ops = &g_sx9373_ops;
  priv->lower.buffer_number = CONFIG_SENSORS_SX9373_BUFFER_NUMBER;

  /* Wait 3ms for stable power supply. */

  up_udelay(SX9373_POWERON_DELAY);

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

  /* Interrupt register */

  ret = IOEXP_SETDIRECTION(priv->config->ioe, priv->config->pin,
                           IOEXPANDER_DIRECTION_IN);
  if (ret < 0)
    {
      snerr("Failed to set direction: %d\n", ret);
      goto err;
    }

  ioephanle = IOEP_ATTACH(priv->config->ioe, priv->config->pin,
                          sx9373_interrupt_handler, priv);
  if (ioephanle == NULL)
    {
      snerr("Failed to attach: %d\n", ret);
      ret = -EIO;
      goto err;
    }

  ret = IOEXP_SETOPTION(priv->config->ioe, priv->config->pin,
                        IOEXPANDER_OPTION_INTCFG,
                        (FAR void *)IOEXPANDER_VAL_DISABLE);
  if (ret < 0)
    {
      snerr("Failed to set option: %d\n", ret);
      IOEP_DETACH(priv->config->ioe, sx9373_interrupt_handler);
      goto err;
    }

  /* Register the sensor driver. */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      IOEP_DETACH(priv->config->ioe, sx9373_interrupt_handler);
      goto err;
    }

  return ret;

err:
  kmm_free(priv);
  return ret;
}
