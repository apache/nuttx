/****************************************************************************
 * drivers/motor/drv2624.c
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
#include <assert.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>

#include <nuttx/motor/motor.h>
#include <nuttx/motor/drv2624.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DRV2624_RAM_SIZE                           (1024)

#define DRV2624_R0X00                              (0x00)
#define DRV2624_R0X00_CHIP_ID_REV                  (0x03)

#define DRV2624_R0X01_STATUS                       (0x01)
#define DRV2624_R0X01_STATUS_DIAG_RESULT_MSK       (0x80)
#define DRV2624_R0X01_STATUS_DIAG_RESULT_OK        (0x00)
#define DRV2624_R0X01_STATUS_DIAG_RESULT_NOK       (0x80)
#define DRV2624_R0X01_STATUS_PRG_ERROR_MSK         (0x10)
#define DRV2624_R0X01_STATUS_PROCESS_DONE_MSK      (0x08)
#define DRV2624_R0X01_STATUS_UVLO_MSK              (0x04)
#define DRV2624_R0X01_STATUS_OVER_TEMP_MSK         (0x02)
#define DRV2624_R0X01_STATUS_OC_DETECT_MSK         (0x01)

#define DRV2624_R00X02_INT_ENABLE                  (0x02)
#define DRV2624_R00X02_INT_MASK_ALL                (0x1f)
#define DRV2624_R00X02_INT_ENABLE_ALL              (0x00)
#define DRV2624_R00X02_INT_ENABLE_CRITICAL         (0x08)

#define DRV2624_R0X03_DIAG_Z                       (0x03)
#define DRV2624_R0X04_VBAT                         (0x04)

#define DRV2624_R0X05_LRA_PERIOD_H                 (0x05)
#define DRV2624_R0X05_LRA_PERIOD_H_MSK             (0x03)

#define DRV2624_R0X06_LRA_PERIOD_L                 (0x06)

#define DRV2624_R0X07                              (0x07)
#define DRV2624_R0X07_TRIG_PIN_FUNC_SFT            (2)
#define DRV2624_R0X07_TRIG_PIN_FUNC_MSK            (0x02 << 2)
#define DRV2624_R0X07_TRIG_PIN_FUNC_EXTERNAL_PULSE (0)
#define DRV2624_R0X07_TRIG_PIN_FUNC_INT            (0x02 << 2)
#define DRV2624_R0X07_MODE_MSK                     (0x3)
#define DRV2624_R0X07_MODE_AUTO_LVL_CALIB_RTN      (0x3)
#define DRV2624_R0X07_MODE_DIAG_MODE               (0x2)
#define DRV2624_R0X07_MODE_WVFRM_SEQ_MODE          (0x1)
#define DRV2624_R0X07_MODE_RTP_MODE                (0x0)

#define DRV2624_R0X08                              (0x08)
#define DRV2624_R0X08_LRA_ERM_MSK                  (0x80)
#define DRV2624_R0X08_LRA_ERM_SFT                  (7)
#define DRV2624_R0X08_LRA                          (1 << 7)
#define DRV2624_R0X08_CTL_LOOP_MSK                 (0x40)
#define DRV2624_R0X08_CTL_LOOP_SFT                 (6)
#define DRV2624_R0X08_CTL_LOOP_CLOSED_LOOP         (0)
#define DRV2624_R0X08_CTL_LOOP_OPEN_LOOP           (1 << 6)
#define DRV2624_R0X08_HYBRID_LOOP_MSK              (0x20)
#define DRV2624_R0X08_HYBRID_LOOP_SFT              (5)
#define DRV2624_R0X08_AUTO_BRK_OL_MSK              (0x10)
#define DRV2624_R0X08_AUTO_BRK_OL_SFT              (4)
#define DRV2624_R0X08_AUTO_BRK_OL_EN               (0x10)
#define DRV2624_R0X08_AUTO_BRK_INTO_STBY_MSK       (0x08)
#define DRV2624_R0X08_AUTO_BRK_INTO_STBY_SFT       (3)

#define DRV2624_R0X09                              (0x09)
#define DRV2624_R0X09_UVLO_THRES_MSK               (0x07)
#define DRV2624_R0X09_UVLO_THRES_3_2V              (0x07)

#define DRV2624_R0X0A_BAT_LIFE_EXT_LVL1            (0x0a)
#define DRV2624_R0X0B_BAT_LIFE_EXT_LVL2            (0x0b)

#define DRV2624_R0X0C_GO                           (0x0c)
#define DRV2624_R0X0C_GO_MSK                       (0x01)
#define DRV2624_R0X0C_GO_BIT                       (0x01)
#define DRV2624_R0X0C_NGO_BIT                      (0x00)

#define DRV2624_R0X0D                              (0x0d)
#define DRV2624_R0X0D_PLAYBACK_INTERVAL_MSK        (0x20)
#define DRV2624_R0X0D_PLAYBACK_INTERVAL_SFT        (5)
#define DRV2624_R0X0D_PLAYBACK_INTERVAL_1MS        (0x01 << 5)
#define DRV2624_R0X0D_PLAYBACK_INTERVAL_5MS        (0x00 << 5)
#define DRV2624_R0X0D_DIG_MEM_GAIN_MASK            (0x03)
#define DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_100    (0x0)
#define DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_75     (0x1)
#define DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_50     (0x2)
#define DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_25     (0x3)

#define DRV2624_R0X0E_RTP_INPUT                    (0x0e)
#define DRV2624_R0X0E_DEFAULT_VALUE                (0x7f)

#define DRV2624_R0X0F_SEQ1                         (0x0f)
#define DRV2624_R0X10_SEQ2                         (0x10)
#define DRV2624_R0X11_SEQ3                         (0x11)
#define DRV2624_R0X12_SEQ4                         (0x12)
#define DRV2624_R0X13_SEQ5                         (0x13)
#define DRV2624_R0X14_SEQ6                         (0x14)
#define DRV2624_R0X15_SEQ7                         (0x15)
#define DRV2624_R0X16_SEQ8                         (0x16)
#define DRV2624_R0X17_WAV1_4_SEQ_LOOP              (0x17)
#define DRV2624_R0X18_WAV5_8_SEQ_LOOP              (0x18)
#define DRV2624_R0X19_MAIN_LOOP                    (0x19)

#define DRV2624_WAVE_ID_INDEX0                     (0x00)

#define DRV2624_SEQ_NO_LOOP                        (0x00)
#define DRV2624_SEQ_LOOP_ONCE                      (0x01)
#define DRV2624_SEQ_LOOP_TWICE                     (0x02)
#define DRV2624_SEQ_LOOP_TRIPPLE                   (0x03)

#define DRV2624_MAIN_NO_LOOP                       (0x00)
#define DRV2624_MAIN_LOOP_ONCE                     (0x01)
#define DRV2624_MAIN_LOOP_TWICE                    (0x02)
#define DRV2624_MAIN_LOOP_3_TIMES                  (0x03)
#define DRV2624_MAIN_LOOP_4_TIMES                  (0x04)
#define DRV2624_MAIN_LOOP_5_TIMES                  (0x05)
#define DRV2624_MAIN_LOOP_6_TIMES                  (0x06)
#define DRV2624_MAIN_LOOP_INFINITELY               (0x07)

#define DRV2624_R0X1A_ODT                          (0x1a)
#define DRV2624_R0X1B_SPT                          (0x1b)
#define DRV2624_R0X1C_SNT                          (0x1c)
#define DRV2624_R0X1D_BRT                          (0x1d)

#define DRV2624_R0X1F_RATED_VOLTAGE                (0x1f)
#define DRV2624_R0X20_OD_CLAMP                     (0x20)
#define DRV2624_R0X21_CAL_COMP                     (0x21)
#define DRV2624_R0X22_CAL_BEMF                     (0x22)
#define DRV2624_R0X23_CAL_GAIN                     (0x23)
#define DRV2624_R0X23_BEMF_GAIN_MSK                (0x03)
#define DRV2624_R0X23_BEMF_GAIN_30X_LRA            (0x03)

#define DRV2624_R0X24_RATED_VOLTAGE_CLAMP          (0x24)
#define DRV2624_R0X25_OD_CLAMP_LVL1                (0x25)
#define DRV2624_R0X26_OD_CLAMP_LVL2                (0x26)

#define DRV2624_R0X27                              (0x27)
#define DRV2624_R0X27_DRIVE_TIME_MSK               (0x1f)
#define DRV2624_R0X27_LRA_MIN_FREQ_SEL_SFT         (0x07)
#define DRV2624_R0X27_LRA_MIN_FREQ_SEL_45HZ        (0x01 << 7)
#define DRV2624_R0X27_LRA_MIN_FREQ_SEL_MSK         (0x80)

#define DRV2624_R0X28                              (0x28)

#define DRV2624_R0X29                              (0x29)
#define DRV2624_R0X29_SAMPLE_TIME_MSK              (0x0c)
#define DRV2624_R0X29_SAMPLE_TIME_250us            (0x08)

#define DRV2624_R0X2A                              (0x2a)
#define DRV2624_R0X2A_CAL_TIME_MSK                 (0x03)
#define DRV2624_R0X2A_CAL_TIME_250MS               (0x00)
#define DRV2624_R0X2A_CAL_TIME_500MS               (0x01)
#define DRV2624_R0X2A_CAL_TIME_1000MS              (0x02)
#define DRV2624_R0X2A_CAL_TIME_TRIGGER_CRTLD       (0x03)

#define DRV2624_R0X2C                              (0x2c)
#define DRV2624_R0X2C_LRA_WAVE_SHAPE_MSK           (0x01)

#define DRV2624_R0X2E_OL_LRA_PERIOD_H              (0x2e)
#define DRV2624_R0X2E_OL_LRA_PERIOD_H_MSK          (0x03)

#define DRV2624_R0X2F_OL_LRA_PERIOD_L              (0x2f)
#define DRV2624_R0X30_CURRENT_K                    (0x30)

#define DRV2624_R0XFD_RAM_ADDR_UPPER               (0xfd)
#define DRV2624_R0XFE_RAM_ADDR_LOWER               (0xfe)
#define DRV2624_R0XFF_RAM_DATA                     (0xff)

#define DRV2624_GO_BIT_CHECK_INTERVAL              (5*1000)
#define DRV2624_GO_BIT_MAX_RETRY_CNT               (10)
#define DRV2624_DIAG_PROCESS_WAITTIME              (600*1000)
#define DRV2624_CALIB_PROCESS_WAITTIME             (600*1000)

#define DRV2624_FW_HEAD_LENGTH                     (10)
#define MAX_RETRIES                                (5)
/****************************************************************************
 * Private Types
 ****************************************************************************/

struct drv2624_calib_s
{
  uint8_t finished;
  uint8_t calcomp;
  uint8_t calbemf;
  uint8_t calgain;
  uint8_t lra_msb;
  uint8_t lra_lsb;
  int calibrated_f0;
};

struct drv2624_diag_s
{
  uint8_t finished;
  uint8_t diagz;
  uint8_t diagk;
};

struct drv2624_data_s
{
  struct drv2624_calib_s calidata;
  struct drv2624_diag_s diagdata;
};

struct drv2624_dev_s
{
  struct motor_lowerhalf_s lower;            /* The struct of lower half */
  FAR const struct drv2624_config_s *config; /* The board config function */
  struct drv2624_data_s data;                /* The struct of data */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C functions */

static int drv2624_i2c_read(FAR struct drv2624_dev_s *priv, uint8_t addr,
                            FAR uint8_t *val, uint32_t cnt);
static int drv2624_i2c_readreg(FAR struct drv2624_dev_s *priv,
                               uint8_t addr, FAR uint8_t *val);
static int drv2624_i2c_writereg(FAR struct drv2624_dev_s *priv,
                                uint8_t addr, uint8_t val);
static int drv2624_i2c_write_bits(FAR struct drv2624_dev_s *priv,
                                  uint8_t addr, uint8_t mask, uint8_t val);

/* control functions */

static int drv2624_set_go_bit(FAR struct drv2624_dev_s *priv, uint8_t val);
static int drv2624_reg_dump(FAR struct drv2624_dev_s *priv,
                            uint8_t start_addr, uint8_t end_addr);
static int drv2624_start_auto_calibrate(FAR struct drv2624_dev_s *priv);
static int drv2624_get_calibration_result(FAR struct drv2624_dev_s *priv,
                                          unsigned long arg);
static int drv2624_calibration(FAR struct drv2624_dev_s *priv,
                               unsigned long arg);
static int drv2624_set_calib_param(FAR struct drv2624_dev_s *priv,
                                   unsigned long arg);
static int drv2624_start_auto_diagnose(FAR struct drv2624_dev_s *priv);
static int drv2624_get_diag_result(FAR struct drv2624_dev_s *priv);
static int drv2624_ram_init(FAR struct drv2624_dev_s *priv);
static int drv2624_checkid(FAR struct drv2624_dev_s *priv);
static int drv2624_common_setting(FAR struct drv2624_dev_s *priv);

/* vibrator ops functions */

static int drv2624_setup(FAR struct motor_lowerhalf_s *dev);
static int drv2624_shutdown(FAR struct motor_lowerhalf_s *dev);
static int drv2624_stop(FAR struct motor_lowerhalf_s *dev);
static int drv2624_start(FAR struct motor_lowerhalf_s *dev);
static int drv2624_setparam(FAR struct motor_lowerhalf_s *dev,
                            FAR struct motor_params_s *param);
static int drv2624_setmode(FAR struct motor_lowerhalf_s *dev, uint8_t mode);
static int drv2624_setlimit(FAR struct motor_lowerhalf_s *dev,
                            FAR struct motor_limits_s *limits);
static int drv2624_setfault(FAR struct motor_lowerhalf_s *dev,
                            uint8_t fault);
static int drv2624_getstate(FAR struct motor_lowerhalf_s *dev,
                            FAR struct motor_state_s *state);
static int drv2624_getfault(FAR struct motor_lowerhalf_s *dev,
                            FAR uint8_t *fault);
static int drv2624_clearfault(FAR struct motor_lowerhalf_s *dev,
                              uint8_t fault);
static int drv2624_ioctl(FAR struct motor_lowerhalf_s *dev, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_drv2624_rambin[] =
{
  /* revision, waveform numbers */

  0x03,

  /* header section, start address H,L,config byte */

  0x8a, 0x00, 0x06,
  0xf4, 0x01, 0x01,
  0xa5, 0x00, 0x03,

  /* waveform data content */

  0x00, 0x00, 0x0a, 0x0a, 0x00, 0x14, 0x6a, 0x00,
  0x1e, 0x0a, 0x30, 0x15, 0x45, 0x22, 0x73, 0x10,
  0x20, 0x3d, 0x18, 0x06, 0x20, 0x16, 0x10, 0x26,
  0x15, 0x15, 0x20, 0x18, 0x11, 0x14, 0xc1, 0x26,
  0x28, 0x37, 0x01, 0x10, 0x22, 0x17, 0x14, 0x21,
};

static const struct motor_ops_s g_drv2624_ops =
{
  .setup       = drv2624_setup,
  .shutdown    = drv2624_shutdown,
  .stop        = drv2624_stop,
  .start       = drv2624_start,
  .params_set  = drv2624_setparam,
  .mode_set    = drv2624_setmode,
  .limits_set  = drv2624_setlimit,
  .fault_set   = drv2624_setfault,
  .state_get   = drv2624_getstate,
  .fault_get   = drv2624_getfault,
  .fault_clear = drv2624_clearfault,
  .ioctl       = drv2624_ioctl,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* I2C functions */

/****************************************************************************
 * Name: drv2624_i2c_read
 *
 * Description:
 *   Read data
 *
 * Input Parameters
 *   priv - Device struct
 *   addr - Register address
 *   val  - Register value
 *   cnt  - Data number
 *
 * Returned Value
 *   return OK if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int drv2624_i2c_read(FAR struct drv2624_dev_s *priv, uint8_t addr,
                            FAR uint8_t *val, uint32_t cnt)
{
  struct i2c_msg_s msg[2];
  int ret;
  int retries;

  msg[0].frequency = priv->config->freq;
  msg[0].addr      = priv->config->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &addr;
  msg[0].length    = 1;

  msg[1].frequency = priv->config->freq;
  msg[1].addr      = priv->config->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = val;
  msg[1].length    = cnt;

  for (retries = 0; retries < MAX_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->config->i2c, msg, 2);
      if (ret >= 0)
        {
          break;
        }
      else
        {
          mtrerr("I2C_TRANSFER failed: %d\n", ret);
        }
    }

  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: drv2624_i2c_readreg
 *
 * Description:
 *   Read 8-bit drv2624 register
 *
 * Input Parameters
 *   priv - Device struct
 *   addr - Register address
 *   val  - Register value
 *
 * Returned Value
 *   return OK if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int drv2624_i2c_readreg(FAR struct drv2624_dev_s *priv, uint8_t addr,
                               FAR uint8_t *val)
{
  return drv2624_i2c_read(priv, addr, val, 1);
}

/****************************************************************************
 * Name: drv2624_i2c_writereg
 *
 * Description:
 *   Write 8-bit drv2624 register
 *
 * Input Parameters
 *   priv  - Device struct
 *   addr  - Register address
 *   val   - To be write value
 *
 * Returned Value
 *   return OK if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int drv2624_i2c_writereg(FAR struct drv2624_dev_s *priv, uint8_t addr,
                                uint8_t val)
{
  struct  i2c_msg_s msg;
  int ret;
  uint8_t txbuffer[2];
  int retries;

  txbuffer[0]   = addr;
  txbuffer[1]   = val;

  msg.frequency = priv->config->freq;
  msg.addr      = priv->config->addr;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = 2;

  for (retries = 0; retries < MAX_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->config->i2c, &msg, 1);
      if (ret >= 0)
        {
          break;
        }
      else
        {
          mtrerr("I2C_TRANSFER failed ret = %d\n", ret);
        }
    }

  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: drv2624_i2c_write_bits
 *
 * Description:
 *   Write bits of drv2624 register
 *
 * Input Parameters
 *   priv  -Device struct
 *   addr  -Register address
 *   mask  -mask the bits to be written
 *   val   -value to be written
 *
 * Returned Value
 *   return OK if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int drv2624_i2c_write_bits(FAR struct drv2624_dev_s *priv,
                                  uint8_t addr, uint8_t mask, uint8_t val)
{
  uint8_t regval;
  int ret;

  ret = drv2624_i2c_readreg(priv, addr, &regval);
  if (ret < 0)
    {
      mtrerr("I2C read reg 0x%02x failed ret = %d\n", addr, ret);
      return ret;
    }

  regval = (regval & ~mask) | (mask & val);
  ret = drv2624_i2c_writereg(priv, addr, regval);
  if (ret < 0)
    {
      mtrerr("I2C write reg 0x%02x failed ret = %d\n", addr, ret);
    }

  return ret;
}

/* control functions */

/****************************************************************************
 * Name: drv2624_set_go_bit
 *
 * Description:
 *   write go bit registor
 *
 ****************************************************************************/

static int drv2624_set_go_bit(FAR struct drv2624_dev_s *priv, uint8_t val)
{
  int ret;
  int retry = DRV2624_GO_BIT_MAX_RETRY_CNT;
  uint8_t regval;

  val &= DRV2624_R0X0C_GO_MSK;

  ret = drv2624_i2c_writereg(priv, DRV2624_R0X0C_GO, val);
  if (ret < 0)
    {
      return ret;
    }

  usleep(DRV2624_GO_BIT_CHECK_INTERVAL);

  ret = drv2624_i2c_readreg(priv, DRV2624_R0X0C_GO, &regval);
  if (ret < 0)
    {
      return ret;
    }

  while (((regval & DRV2624_R0X0C_GO_MSK) != val) && (retry > 0))
    {
      ret = drv2624_i2c_readreg(priv, DRV2624_R0X0C_GO, &regval);
      usleep(DRV2624_GO_BIT_CHECK_INTERVAL);
      retry--;
    }

  return ret;
}

/****************************************************************************
 * Name: drv2624_common_setting
 *
 * Description:
 *   set common regsitor value, not changed often
 *
 ****************************************************************************/

static int drv2624_common_setting(FAR struct drv2624_dev_s *priv)
{
  int ret;

  /* waveseq mode setting */

  ret = drv2624_i2c_writereg(priv, DRV2624_R0X19_MAIN_LOOP,
                             DRV2624_MAIN_NO_LOOP);
  if (ret < 0)
    {
      return ret;
    }

  ret = drv2624_i2c_writereg(priv, DRV2624_R0X17_WAV1_4_SEQ_LOOP,
                             DRV2624_SEQ_NO_LOOP);
  if (ret < 0)
    {
      return ret;
    }

  ret = drv2624_i2c_writereg(priv, DRV2624_R0X18_WAV5_8_SEQ_LOOP,
                             DRV2624_SEQ_NO_LOOP);
  if (ret < 0)
    {
      return ret;
    }

  ret = drv2624_i2c_writereg(priv, DRV2624_R0X10_SEQ2,
                             DRV2624_WAVE_ID_INDEX0);
  if (ret < 0)
    {
      return ret;
    }

  /* control reg common setting */

  ret = drv2624_i2c_writereg(priv, DRV2624_R0X07, 0x59);
  if (ret < 0)
    {
      return ret;
    }

  ret = drv2624_i2c_writereg(priv, DRV2624_R0X08, 0x80);
  if (ret < 0)
    {
      return ret;
    }

  ret = drv2624_i2c_writereg(priv, DRV2624_R0X09,
                             DRV2624_R0X09_UVLO_THRES_3_2V);
  if (ret < 0)
    {
      return ret;
    }

  ret = drv2624_i2c_writereg(priv, DRV2624_R0X1F_RATED_VOLTAGE, 0x46);
  if (ret < 0)
    {
      return ret;
    }

  ret = drv2624_i2c_writereg(priv, DRV2624_R0X20_OD_CLAMP, 0xa4);
  if (ret < 0)
    {
      return ret;
    }

  return drv2624_i2c_write_bits(priv, DRV2624_R0X2A,
                                DRV2624_R0X2A_CAL_TIME_MSK,
                                DRV2624_R0X2A_CAL_TIME_500MS);
}

/****************************************************************************
 * Name: drv2624_reg_dump
 *
 * Description:
 *   dump all register values for selftest if needed
 *
 * Input Parameters
 *   priv       - Device struct
 *   start_addr - dump from this addr
 *   end_addr   - dump to this addr
 *
 * Returned Value
 *   return OK if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int drv2624_reg_dump(FAR struct drv2624_dev_s *priv,
                            uint8_t start_addr, uint8_t end_addr)
{
  FAR uint8_t *reg_buf;

  if (end_addr <= start_addr)
    {
      return -EINVAL;
    }

  reg_buf = kmm_zalloc(end_addr - start_addr + 1);
  if (NULL == reg_buf)
    {
      return -ENOMEM;
    }

  for (uint8_t i = 0; i <= (end_addr - start_addr); i++)
    {
      drv2624_i2c_readreg(priv, (start_addr + i), &reg_buf[i]);
      mtrinfo("[0x%2x]=0x%2x\n", (start_addr + i), reg_buf[i]);
    }

  kmm_free(reg_buf);

  return OK;
}

/****************************************************************************
 * Name: drv2624_start_auto_calibrate
 *
 * Description:
 *   start calibration process
 *
 ****************************************************************************/

static int drv2624_start_auto_calibrate(FAR struct drv2624_dev_s *priv)
{
  int ret;

  ret = drv2624_i2c_write_bits(priv, DRV2624_R0X07, DRV2624_R0X07_MODE_MSK,
                               DRV2624_R0X07_MODE_AUTO_LVL_CALIB_RTN);
  if (ret < 0)
    {
      return ret;
    }

  return drv2624_set_go_bit(priv, DRV2624_R0X0C_GO_BIT);
}

/****************************************************************************
 * Name: drv2624_get_calibration_result
 *
 * Description :
 *   get calibration result after calibration process finish
 *
 ****************************************************************************/

static int drv2624_get_calibration_result(FAR struct drv2624_dev_s *priv,
                                          unsigned long arg)
{
  int ret;
  uint8_t val;
  FAR char *calibdata = (FAR char *)arg;

  DEBUGASSERT(priv != NULL && calibdata != NULL);

  ret = drv2624_i2c_readreg(priv, DRV2624_R0X01_STATUS, &val);
  if (ret < 0)
    {
      return ret;
    }

  if ((val & DRV2624_R0X01_STATUS_PROCESS_DONE_MSK) == 0)
    {
      mtrerr("calibration fail, process not finish,need wait more time\n");
      return -EAGAIN;
    }

  if (val & DRV2624_R0X01_STATUS_DIAG_RESULT_NOK)
    {
      mtrerr("calibration fail, check connection or enviroment\n");
      return -EAGAIN;
    }
  else
    {
      ret = drv2624_i2c_readreg(priv, DRV2624_R0X21_CAL_COMP,
                                &priv->data.calidata.calcomp);
      if (ret < 0)
        {
          return ret;
        }

      ret = drv2624_i2c_readreg(priv, DRV2624_R0X22_CAL_BEMF,
                                &priv->data.calidata.calbemf);
      if (ret < 0)
        {
          return ret;
        }

      ret = drv2624_i2c_readreg(priv, DRV2624_R0X23_CAL_GAIN,
                                &priv->data.calidata.calgain);
      if (ret < 0)
        {
          return ret;
        }

      ret = drv2624_i2c_readreg(priv, DRV2624_R0X05_LRA_PERIOD_H,
                                &priv->data.calidata.lra_msb);
      if (ret < 0)
        {
          return ret;
        }

      ret = drv2624_i2c_readreg(priv, DRV2624_R0X06_LRA_PERIOD_L,
                                &priv->data.calidata.lra_lsb);
      if (ret < 0)
        {
          return ret;
        }

      priv->data.calidata.calibrated_f0 =
          100000000 / ((priv->data.calidata.lra_msb << 8 |
                        priv->data.calidata.lra_lsb) * 2439);

      mtrinfo("reg 0x21 = 0x%x, 0x22 = 0x%x, 0x23 = 0x%x, "
              "calibrated f0 = %d\n",
              priv->data.calidata.calcomp,
              priv->data.calidata.calbemf,
              priv->data.calidata.calgain,
              priv->data.calidata.calibrated_f0);

      priv->data.calidata.finished = 1;

      sprintf(calibdata, "%03d,%03d,%03d,%03d\n",
              priv->data.calidata.finished,
              priv->data.calidata.calcomp,
              priv->data.calidata.calbemf,
              priv->data.calidata.calgain);

      return ret;
    }
}

/****************************************************************************
 * Name: drv2624_calibration
 *
 * Description:
 *  start calibration process and get its result
 *
 ****************************************************************************/

static int drv2624_calibration(FAR struct drv2624_dev_s *priv,
                               unsigned long arg)
{
  int ret;

  ret = drv2624_start_auto_calibrate(priv);
  if (ret < 0)
    {
      mtrerr("drv2624_start_auto_calibrate err ret=%d\n", ret);
      return ret;
    }

  usleep(DRV2624_CALIB_PROCESS_WAITTIME);

  ret = drv2624_get_calibration_result(priv, arg);
  if (ret < 0)
    {
      mtrerr("get calibration result err ret=%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: drv2624_set_calib_param
 *
 * Description:
 *   rewrite calibration data into reg if calibration succeed
 *
 ****************************************************************************/

static int drv2624_set_calib_param(FAR struct drv2624_dev_s *priv,
                                   unsigned long arg)
{
  FAR char *calibdata = (FAR char *)arg;
  struct drv2624_calib_s pdata;

  DEBUGASSERT(priv != NULL && calibdata != NULL);

  pdata.finished = atoi(&calibdata[0]);
  pdata.calcomp = atoi(&calibdata[4]);
  pdata.calbemf = atoi(&calibdata[8]) ;
  pdata.calgain = atoi(&calibdata[12]);

  mtrinfo("finish = %d, reg 0x21 = 0x%x, 0x22 = 0x%x, 0x23 = 0x%x\n",
          pdata.finished, pdata.calcomp, pdata.calbemf, pdata.calgain);

  if (pdata.finished == 1)
    {
      drv2624_i2c_writereg(priv, DRV2624_R0X21_CAL_COMP, pdata.calcomp);
      drv2624_i2c_writereg(priv, DRV2624_R0X22_CAL_BEMF, pdata.calbemf);
      drv2624_i2c_writereg(priv, DRV2624_R0X23_CAL_GAIN, pdata.calgain);
    }

  return OK;
}

/****************************************************************************
 * Name: drv2624_start_auto_diagnose
 *
 * Description:
 *   start auto diagnose process
 *
 ****************************************************************************/

static int drv2624_start_auto_diagnose(FAR struct drv2624_dev_s *priv)
{
  int ret;

  ret = drv2624_i2c_write_bits(priv, DRV2624_R0X07, DRV2624_R0X07_MODE_MSK,
                               DRV2624_R0X07_MODE_DIAG_MODE);
  if (ret < 0)
    {
      return ret;
    }

  return drv2624_set_go_bit(priv, DRV2624_R0X0C_GO_BIT);
}

/****************************************************************************
 * Name: drv2624_get_diag_result
 *
 * Description :
 *   get diagnose result after diagnose process finish
 *
 ****************************************************************************/

static int drv2624_get_diag_result(FAR struct drv2624_dev_s *priv)
{
  int ohm;
  uint8_t val;
  int ret = OK;

  drv2624_i2c_readreg(priv, DRV2624_R0X01_STATUS, &val);

  if (val & DRV2624_R0X01_STATUS_PROCESS_DONE_MSK)
    {
      drv2624_i2c_readreg(priv, DRV2624_R0X03_DIAG_Z,
                          &priv->data.diagdata.diagz);
      drv2624_i2c_readreg(priv, DRV2624_R0X30_CURRENT_K,
                          &priv->data.diagdata.diagk);
      ohm = 478.43f * (priv->data.diagdata.diagz) /
            (4 * priv->data.diagdata.diagk + 719);

      mtrinfo("resistance = %d ohm\n", ohm);
      if (ohm  == 0)
        {
          mtrerr("ERROR, open circuit detected!!\n");
          ret = -MOTOR_FAULT_OTHER;
        }
    }

  if (val & DRV2624_R0X01_STATUS_OC_DETECT_MSK)
    {
      mtrerr("ERROR, Over Current detected!!\n");
      ret = -MOTOR_FAULT_OVERCURRENT;
    }

  if (val & DRV2624_R0X01_STATUS_OVER_TEMP_MSK)
    {
      mtrerr("ERROR, Over Temperature detected!!\n");
      ret = -MOTOR_FAULT_OVERTEMP;
    }

  if (val & DRV2624_R0X01_STATUS_UVLO_MSK)
    {
      mtrerr("ERROR, VDD drop observed!!\n");
      ret = -MOTOR_FAULT_OTHER;
    }

  return ret;
}

/****************************************************************************
 * Name: drv2624_ram_init
 *
 * Description:
 *   write ram data into drv2624 chip
 *
 ****************************************************************************/

static int drv2624_ram_init(FAR struct drv2624_dev_s *priv)
{
  int size;
  int i;
  int ret;

  size = sizeof(g_drv2624_rambin);

  if (size < DRV2624_RAM_SIZE)
    {
      ret = drv2624_i2c_writereg(priv, DRV2624_R0XFD_RAM_ADDR_UPPER, 0);
      if (ret < 0)
        {
          return ret;
        }

      ret = drv2624_i2c_writereg(priv, DRV2624_R0XFE_RAM_ADDR_LOWER, 0);
      if (ret < 0)
        {
          return ret;
        }

      for (i = DRV2624_FW_HEAD_LENGTH; i < size; i++)
        {
          ret = drv2624_i2c_writereg(priv, DRV2624_R0XFF_RAM_DATA,
                                     g_drv2624_rambin[i]);
          if (ret < 0)
            {
              return ret;
            }
        }
    }
  else
    {
      mtrerr("firmware size too big\n");
      ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: drv2624_checkid
 *
 * Description:
 *   Read and verify the drv2624 chip ID
 *
 ****************************************************************************/

static int drv2624_checkid(FAR struct drv2624_dev_s *priv)
{
  int ret;
  uint8_t devid;

  ret = drv2624_i2c_readreg(priv, DRV2624_R0X00, &devid);
  if (ret < 0 || devid != DRV2624_R0X00_CHIP_ID_REV)
    {
      mtrerr("Wrong Device ID! %02x\n", devid);
      ret = -ENODEV;
    }

  return ret;
}

/* motor ops functions */

/****************************************************************************
 * Name: drv2624_setup
 *
 * Description:
 *   drv2624 initilize, power on
 *
 ****************************************************************************/

static int drv2624_setup(FAR struct motor_lowerhalf_s *dev)
{
  int ret;
  FAR struct drv2624_dev_s *priv = (FAR struct drv2624_dev_s *)dev;

  DEBUGASSERT(dev != NULL);

  /* TODO:set motor enable pin on */

  ret = drv2624_checkid(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = drv2624_ram_init(priv);
  if (ret < 0)
    {
      return ret;
    }

  return drv2624_common_setting(priv);
}

/****************************************************************************
 * Name: drv2624_shutdown
 *
 * Description:
 *   drv2624 shutdown, power off
 *
 ****************************************************************************/

static int drv2624_shutdown(FAR struct motor_lowerhalf_s *dev)
{
  /* TODO:set motor enable pin off */

  return OK;
}

/****************************************************************************
 * Name: drv2624_stop
 *
 * Description:
 *   set gobit 0,stop play
 *
 ****************************************************************************/

static int drv2624_stop(FAR struct motor_lowerhalf_s *dev)
{
  FAR struct drv2624_dev_s *priv = (FAR struct drv2624_dev_s *)dev;

  DEBUGASSERT(dev != NULL);

  return drv2624_set_go_bit(priv, DRV2624_R0X0C_NGO_BIT);
}

/****************************************************************************
 * Name: drv2624_start
 *
 * Description:
 *   set go bit 1, start play
 *
 ****************************************************************************/

static int drv2624_start(FAR struct motor_lowerhalf_s *dev)
{
  FAR struct drv2624_dev_s *priv = (FAR struct drv2624_dev_s *)dev;

  DEBUGASSERT(dev != NULL);

  return drv2624_set_go_bit(priv, DRV2624_R0X0C_GO_BIT);
}

/****************************************************************************
 * Name: drv2624_setparam
 *
 * Description:
 *   set drv2624 used parameters
 *
 ****************************************************************************/

static int drv2624_setparam(FAR struct motor_lowerhalf_s  *dev,
                            FAR struct motor_params_s *param)
{
  int ret = -EINVAL;
  FAR struct drv2624_dev_s *priv = (FAR struct drv2624_dev_s *)dev;
  uint8_t strength;
  uint8_t rtpin;

  DEBUGASSERT(dev != NULL && param != NULL);

  if (param->pattern.patternid >= 0)
    {
      ret = drv2624_i2c_writereg(priv, DRV2624_R0X0F_SEQ1,
                                param->pattern.patternid);
    }

  if (param->pattern.strength > 0 && param->pattern.strength <= 0.25)
    {
      strength = DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_25;
      ret = drv2624_i2c_write_bits(priv, DRV2624_R0X0D,
                              DRV2624_R0X0D_DIG_MEM_GAIN_MASK, strength);
    }
  else if(param->pattern.strength > 0.25 && param->pattern.strength <= 0.5)
    {
      strength = DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_50;
      ret = drv2624_i2c_write_bits(priv, DRV2624_R0X0D,
                              DRV2624_R0X0D_DIG_MEM_GAIN_MASK, strength);
    }
  else if(param->pattern.strength > 0.5 && param->pattern.strength <= 0.75)
    {
      strength = DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_75;
      ret = drv2624_i2c_write_bits(priv, DRV2624_R0X0D,
                              DRV2624_R0X0D_DIG_MEM_GAIN_MASK, strength);
    }
  else if(param->pattern.strength > 0.75 && param->pattern.strength <= 1)
    {
      strength = DRV2624_R0X0D_DIG_MEM_GAIN_STRENGTH_100;
      ret = drv2624_i2c_write_bits(priv, DRV2624_R0X0D,
                              DRV2624_R0X0D_DIG_MEM_GAIN_MASK, strength);
    }

  if (param->force > 0.0 || param->force <= 1.0)
    {
      rtpin = (param->force * 10 * 127) / 10.f;
      ret = drv2624_i2c_writereg(priv, DRV2624_R0X0E_RTP_INPUT, rtpin);
    }

  return ret;
}

/****************************************************************************
 * Name: drv2624_setmode
 *
 * Description:
 *   set play mode
 *
 ****************************************************************************/

static int drv2624_setmode(FAR struct motor_lowerhalf_s *dev, uint8_t mode)
{
  int ret;
  FAR struct drv2624_dev_s *priv = (FAR struct drv2624_dev_s *)dev;

  DEBUGASSERT(dev != NULL);

  /* Only rtp mode and wave seq mode supported */

  if ((mode != MOTOR_OPMODE_FORCE) && (mode != MOTOR_OPMODE_PATTERN))
    {
      mtrerr("ERROR:  Unsupported play mode %d!\n", mode);
      return -EINVAL;
    }

  priv->lower.opmode = mode;
  if (mode == MOTOR_OPMODE_FORCE)
    {
      ret = drv2624_i2c_write_bits(priv, DRV2624_R0X07,
                                   DRV2624_R0X07_MODE_MSK,
                                   DRV2624_R0X07_MODE_RTP_MODE);
    }
  else if (mode == MOTOR_OPMODE_PATTERN)
    {
      ret = drv2624_i2c_write_bits(priv, DRV2624_R0X07,
                                   DRV2624_R0X07_MODE_MSK,
                                   DRV2624_R0X07_MODE_WVFRM_SEQ_MODE);
    }

  return ret;
}

/****************************************************************************
 * Name: drv2624_setlimit
 *
 * Description:
 *   set drv2624 limit value
 *
 ****************************************************************************/

static int drv2624_setlimit(FAR struct motor_lowerhalf_s *dev,
                            FAR struct motor_limits_s *limits)
{
  FAR struct drv2624_dev_s *priv = (FAR struct drv2624_dev_s *)dev;

  DEBUGASSERT(dev != NULL && limits != NULL);

  /* Set limit */

  priv->lower.limits.force = limits->force;

  /* Lock limits */

  priv->lower.limits.lock = true;

  return OK;
}

/****************************************************************************
 * Name: drv2624_setfault
 *
 * Description:
 *   not used now, just keep the interface
 *
 ****************************************************************************/

static int drv2624_setfault(FAR struct motor_lowerhalf_s *dev,
                            uint8_t fault)
{
  return OK;
}

/****************************************************************************
 * Name: drv2624_getstate
 *
 * Description:
 *   not used now, just keep the interface
 *
 ****************************************************************************/

static int drv2624_getstate(FAR struct motor_lowerhalf_s *dev,
                            FAR struct motor_state_s *state)
{
  return OK;
}

/****************************************************************************
 * Name: drv2624_getfault
 *
 * Description:
 *   start diagnose process and get the diagnose result
 *
 ****************************************************************************/

static int drv2624_getfault(FAR struct motor_lowerhalf_s *dev,
                            FAR uint8_t *fault)
{
  int ret;
  FAR struct drv2624_dev_s *priv = (FAR struct drv2624_dev_s *)dev;

  DEBUGASSERT(dev != NULL && fault != NULL);

  ret = drv2624_start_auto_diagnose(priv);
  if (ret < 0)
    {
      mtrerr("start diagnose err ret=%d\n", ret);
      return ret;
    }

  usleep(DRV2624_DIAG_PROCESS_WAITTIME);

  *fault = drv2624_get_diag_result(priv);

  return ret;
}

/****************************************************************************
 * Name: drv2624_clearfault
 *
 * Description:
 *   not used now, just keep the interface
 *
 ****************************************************************************/

static int drv2624_clearfault(FAR struct motor_lowerhalf_s *dev,
                              uint8_t fault)
{
  return OK;
}

/****************************************************************************
 * Name: drv2624_ioctl
 *
 * Description:
 *   ioctrl function used in drv2624
 *
 ****************************************************************************/

static int drv2624_ioctl(FAR struct motor_lowerhalf_s *dev, int cmd,
                         unsigned long arg)
{
  int ret;
  FAR struct drv2624_dev_s *priv = (FAR struct drv2624_dev_s *)dev;

  DEBUGASSERT(dev != NULL);

  switch (cmd)
    {
      case MTRIOC_SELFTEST:
        {
          ret = drv2624_reg_dump(priv, DRV2624_R0X00,
                                 DRV2624_R0X30_CURRENT_K);
          break;
        }

      case MTRIOC_SET_CALIBDATA:
        {
          ret = drv2624_set_calib_param(priv, arg);
          break;
        }

      case MTRIOC_CALIBRATE:
        {
          ret = drv2624_calibration(priv, arg);
          break;
        }

      default:
        {
          mtrerr("undefined ioctrl cmd: %d\n", cmd);
          ret = -EINVAL;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv2624_register
 *
 * Description:
 *   Register the drv2624 character device as 'devpath'
 *
 * Input Parameters:
 *   devname - The name of driver register. E.g., "/dev/lra0"
 *   config  - the board config of drv2624
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

int drv2624_register(FAR const char *devname,
                     FAR const struct drv2624_config_s *config)
{
  int ret;
  FAR struct drv2624_dev_s *priv;

  DEBUGASSERT(devname != NULL && config != NULL);

  /* Initialize the drv2624 device structure */

  priv = kmm_zalloc(sizeof(struct drv2624_dev_s));
  if (NULL == priv)
    {
      mtrerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->config = config;
  priv->lower.ops = &g_drv2624_ops;

  /* Check Device ID */

  ret = drv2624_checkid(priv);
  if (ret < 0)
    {
      mtrerr("Failed to register driver: %d\n", ret);
      goto err;
    }

  ret = motor_register(devname, &priv->lower);
  if (ret < 0)
    {
      mtrerr("Failed to register driver:%d\n", ret);
      goto err;
    }

  return ret;

err:
  kmm_free(priv);
  return ret;
}
