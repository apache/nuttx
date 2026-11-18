/****************************************************************************
 * drivers/video/imx335.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Sony IMX335 5 MP image sensor, 2-lane MIPI CSI-2, RAW10 RGGB Bayer.
 *
 * All-pixel scan, 2 data lanes, 10 bits, 30 frames per second with the
 * default VMAX of 4500 lines.
 *
 * The sensor only outputs the full 2592x1944 RAW10 frame
 * (V4L2_PIX_FMT_SRGGB10P).  A capture interface with an image signal
 * processor (for example the STM32N6 DCMIPP) demosaics and scales it and
 * registers its own image sensor in front of this one.  There is no
 * automatic exposure in the sensor; the exposure time and the gain can be
 * changed with V4L2_CID_EXPOSURE_ABSOLUTE and V4L2_CID_GAIN.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imx335.h>
#include <nuttx/video/video.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX335_I2C_ADDR          0x1a
#define IMX335_I2C_FREQ          400000

#define IMX335_WIDTH             2592
#define IMX335_HEIGHT            1944

/* Frame timing: VMAX (reset value, never changed) lines per frame and
 * the HMAX line length for 30 frames per second.
 */

#define IMX335_FPS               30
#define IMX335_VMAX              4500
#define IMX335_HMAX              0x0226
#define IMX335_SHR0_MIN          9

/* Gain in 0.3 dB steps, 0 to 72 dB */

#define IMX335_GAIN_MAX          240

/* Registers */

#define IMX335_REG_STANDBY       0x3000  /* Bit 0: standby */
#define IMX335_REG_ID            0x3912  /* Reads 0 (ST IMX335_CHIP_ID) */
#define IMX335_REG_REGHOLD       0x3001  /* Bit 0: hold register updates */
#define IMX335_REG_VMAX          0x3030  /* 24 bits: lines per frame */
#define IMX335_REG_HMAX          0x3034  /* 16 bits: line length */
#define IMX335_REG_SHR0          0x3058  /* 24 bits: shutter line */
#define IMX335_REG_GAIN          0x30e8  /* 16 bits: gain, 0.3 dB steps */

#define IMX335_STANDBY           0x01
#define IMX335_OPERATING         0x00

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx335_reg_s
{
  uint16_t addr;
  uint8_t  val;
};

struct imx335_inck_s
{
  uint32_t freq;
  struct imx335_reg_s regs[7];
};

struct imx335_dev_s
{
  struct imgsensor_s sensor;          /* Must be first */
  FAR struct i2c_master_s *i2c;       /* I2C bus */
  mutex_t lock;                       /* Serializes the sensor access */
  FAR const struct imx335_inck_s *inck;
  uint32_t exposure;                  /* Exposure time in 100 us units */
  uint32_t gain;                      /* Gain in 0.3 dB steps */
  bool streaming;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool imx335_is_available(FAR struct imgsensor_s *sensor);
static int imx335_init(FAR struct imgsensor_s *sensor);
static int imx335_uninit(FAR struct imgsensor_s *sensor);
static FAR const char *
imx335_get_driver_name(FAR struct imgsensor_s *sensor);
static int imx335_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval);
static int imx335_start_capture(FAR struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval);
static int imx335_stop_capture(FAR struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type);
static int imx335_get_frame_interval(FAR struct imgsensor_s *sensor,
                                     imgsensor_stream_type_t type,
                                     FAR imgsensor_interval_t *interval);
static int imx335_get_supported_value(FAR struct imgsensor_s *sensor,
                                      uint32_t id,
                                      FAR imgsensor_supported_value_t *val);
static int imx335_get_value(FAR struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            FAR imgsensor_value_t *value);
static int imx335_set_value(FAR struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Initial settings with the sensor in standby, then the fixed value
 * registers.  Multi-byte registers are little endian.
 */

static const struct imx335_reg_s g_imx335_init_regs[] =
{
  { 0x3000, 0x01 },
  { 0x3002, 0x00 },
  { 0x302c, 0x3c },
  { 0x302d, 0x00 },
  { 0x302e, 0x20 },
  { 0x302f, 0x0a },
  { 0x3056, 0x98 },
  { 0x3057, 0x07 },
  { 0x3074, 0xc8 },
  { 0x3075, 0x00 },
  { 0x3076, 0x30 },
  { 0x3077, 0x0f },
  { 0x31a1, 0x00 },
  { 0x3288, 0x21 },
  { 0x328a, 0x02 },
  { 0x3414, 0x05 },
  { 0x3416, 0x18 },
  { 0x3648, 0x01 },
  { 0x364a, 0x04 },
  { 0x364c, 0x04 },
  { 0x3678, 0x01 },
  { 0x367c, 0x31 },
  { 0x367e, 0x31 },
  { 0x3706, 0x10 },
  { 0x3708, 0x03 },
  { 0x3714, 0x02 },
  { 0x3715, 0x02 },
  { 0x3716, 0x01 },
  { 0x3717, 0x03 },
  { 0x371c, 0x3d },
  { 0x371d, 0x3f },
  { 0x372c, 0x00 },
  { 0x372d, 0x00 },
  { 0x372e, 0x46 },
  { 0x372f, 0x00 },
  { 0x3730, 0x89 },
  { 0x3731, 0x00 },
  { 0x3732, 0x08 },
  { 0x3733, 0x01 },
  { 0x3734, 0xfe },
  { 0x3735, 0x05 },
  { 0x3740, 0x02 },
  { 0x375d, 0x00 },
  { 0x375e, 0x00 },
  { 0x375f, 0x11 },
  { 0x3760, 0x01 },
  { 0x3768, 0x1b },
  { 0x3769, 0x1b },
  { 0x376a, 0x1b },
  { 0x376b, 0x1b },
  { 0x376c, 0x1a },
  { 0x376d, 0x17 },
  { 0x376e, 0x0f },
  { 0x3776, 0x00 },
  { 0x3777, 0x00 },
  { 0x3778, 0x46 },
  { 0x3779, 0x00 },
  { 0x377a, 0x89 },
  { 0x377b, 0x00 },
  { 0x377c, 0x08 },
  { 0x377d, 0x01 },
  { 0x377e, 0x23 },
  { 0x377f, 0x02 },
  { 0x3780, 0xd9 },
  { 0x3781, 0x03 },
  { 0x3782, 0xf5 },
  { 0x3783, 0x06 },
  { 0x3784, 0xa5 },
  { 0x3788, 0x0f },
  { 0x378a, 0xd9 },
  { 0x378b, 0x03 },
  { 0x378c, 0xeb },
  { 0x378d, 0x05 },
  { 0x378e, 0x87 },
  { 0x378f, 0x06 },
  { 0x3790, 0xf5 },
  { 0x3792, 0x43 },
  { 0x3794, 0x7a },
  { 0x3796, 0xa1 },
  { 0x37b0, 0x36 },
  { 0x3a00, 0x01 },
};

/* No binning (the 2592x1944 cropping window of the init list stays), the
 * 30 frames per second line length and 2 lanes with 10 bits.
 */

static const struct imx335_reg_s g_imx335_mode_regs[] =
{
  { 0x3199, 0x00 },
  { 0x3300, 0x00 },
  { 0x3078, 0x01 },
  { 0x3079, 0x02 },
  { 0x307a, 0xff },
  { 0x307b, 0x02 },
  { 0x307c, 0x00 },
  { 0x307d, 0x00 },
  { 0x307e, 0x00 },
  { 0x307f, 0x00 },
  { 0x3080, 0x01 },
  { 0x3081, 0x02 },
  { 0x3082, 0xff },
  { 0x3083, 0x02 },
  { 0x3084, 0x00 },
  { 0x3085, 0x00 },
  { 0x3086, 0x00 },
  { 0x3087, 0x00 },
  { 0x30a4, 0x33 },
  { 0x30a8, 0x10 },
  { 0x30a9, 0x04 },
  { 0x30ac, 0x00 },
  { 0x30ad, 0x00 },
  { 0x30b0, 0x10 },
  { 0x30b1, 0x08 },
  { 0x30b4, 0x00 },
  { 0x30b5, 0x00 },
  { 0x30b6, 0x00 },
  { 0x30b7, 0x00 },
  { 0x3112, 0x08 },
  { 0x3113, 0x00 },
  { 0x3116, 0x08 },
  { 0x3117, 0x00 },
  { 0x3001, 0x01 },
  { 0x3034, IMX335_HMAX & 0xff },
  { 0x3035, IMX335_HMAX >> 8 },
  { 0x3001, 0x00 },
  { 0x3050, 0x00 },
  { 0x319d, 0x00 },
  { 0x341c, 0xff },
  { 0x341d, 0x01 },
  { 0x3a01, 0x01 },
};

/* Input clock (INCK) dependent settings */

static const struct imx335_inck_s g_imx335_inck[] =
{
  {
    74250000,
    {
      { 0x300c, 0xb6 },
      { 0x300d, 0x7f },
      { 0x314c, 0x80 },
      { 0x314d, 0x00 },
      { 0x315a, 0x03 },
      { 0x3168, 0x68 },
      { 0x316a, 0x7f },
    },
  },
  {
    27000000,
    {
      { 0x300c, 0x42 },
      { 0x300d, 0x2e },
      { 0x314c, 0xb0 },
      { 0x314d, 0x00 },
      { 0x315a, 0x02 },
      { 0x3168, 0x8f },
      { 0x316a, 0x7e },
    },
  },
  {
    24000000,
    {
      { 0x300c, 0x3b },
      { 0x300d, 0x2a },
      { 0x314c, 0xc6 },
      { 0x314d, 0x00 },
      { 0x315a, 0x02 },
      { 0x3168, 0xa0 },
      { 0x316a, 0x7e },
    },
  },
  {
    18000000,
    {
      { 0x300c, 0x2d },
      { 0x300d, 0x1f },
      { 0x314c, 0x84 },
      { 0x314d, 0x00 },
      { 0x315a, 0x01 },
      { 0x3168, 0x6b },
      { 0x316a, 0x7d },
    },
  },
  {
    6000000,
    {
      { 0x300c, 0x0f },
      { 0x300d, 0x0b },
      { 0x314c, 0xc6 },
      { 0x314d, 0x00 },
      { 0x315a, 0x00 },
      { 0x3168, 0xa0 },
      { 0x316a, 0x7c },
    },
  },
};

static const struct imgsensor_ops_s g_imx335_ops =
{
  .is_available           = imx335_is_available,
  .init                   = imx335_init,
  .uninit                 = imx335_uninit,
  .get_driver_name        = imx335_get_driver_name,
  .validate_frame_setting = imx335_validate_frame_setting,
  .start_capture          = imx335_start_capture,
  .stop_capture           = imx335_stop_capture,
  .get_frame_interval     = imx335_get_frame_interval,
  .get_supported_value    = imx335_get_supported_value,
  .get_value              = imx335_get_value,
  .set_value              = imx335_set_value,
};

static const struct v4l2_fmtdesc g_imx335_fmtdescs[] =
{
  {
    .pixelformat = V4L2_PIX_FMT_SRGGB10P,
    .description = "RAW10 RGGB",
  },
};

static const struct v4l2_frmsizeenum g_imx335_frmsizes[] =
{
  {
    .type = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete =
      {
        .width  = IMX335_WIDTH,
        .height = IMX335_HEIGHT,
      },
  },
};

static const struct v4l2_frmivalenum g_imx335_frmintervals[] =
{
  {
    .type = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete =
      {
        .numerator   = 1,
        .denominator = IMX335_FPS,
      },
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx335_putreg
 ****************************************************************************/

static int imx335_putreg(FAR struct imx335_dev_s *priv, uint16_t regaddr,
                         uint32_t regval, uint8_t len)
{
  struct i2c_msg_s msg;
  uint8_t buf[5];
  int ret;
  int i;

  /* 16-bit register address, multi-byte values are little endian */

  buf[0] = regaddr >> 8;
  buf[1] = regaddr & 0xff;
  for (i = 0; i < len; i++)
    {
      buf[2 + i] = (regval >> (8 * i)) & 0xff;
    }

  msg.frequency = IMX335_I2C_FREQ;
  msg.addr      = IMX335_I2C_ADDR;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 2 + len;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: write 0x%04x failed: %d\n", regaddr, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: imx335_getreg
 ****************************************************************************/

static int imx335_getreg(FAR struct imx335_dev_s *priv, uint16_t regaddr,
                         FAR uint8_t *regval)
{
  struct i2c_msg_s msg[2];
  uint8_t buf[2];

  buf[0] = regaddr >> 8;
  buf[1] = regaddr & 0xff;

  msg[0].frequency = IMX335_I2C_FREQ;
  msg[0].addr      = IMX335_I2C_ADDR;
  msg[0].flags     = 0;
  msg[0].buffer    = buf;
  msg[0].length    = 2;

  msg[1].frequency = IMX335_I2C_FREQ;
  msg[1].addr      = IMX335_I2C_ADDR;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = 1;

  return I2C_TRANSFER(priv->i2c, msg, 2);
}

/****************************************************************************
 * Name: imx335_putreglist
 ****************************************************************************/

static int imx335_putreglist(FAR struct imx335_dev_s *priv,
                             FAR const struct imx335_reg_s *regs,
                             size_t nregs)
{
  size_t i;
  int ret;

  for (i = 0; i < nregs; i++)
    {
      ret = imx335_putreg(priv, regs[i].addr, regs[i].val, 1);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: imx335_setexposure
 *
 * Description:
 *   Program the shutter for an exposure time in 100 us units.  One line
 *   lasts 1 / (VMAX * fps) seconds.
 *
 ****************************************************************************/

static int imx335_setexposure(FAR struct imx335_dev_s *priv,
                              uint32_t exposure)
{
  uint32_t lines;
  uint32_t shr;
  int ret;

  lines = exposure * IMX335_VMAX * IMX335_FPS / 10000;
  if (lines < 1)
    {
      lines = 1;
    }

  if (lines > IMX335_VMAX - IMX335_SHR0_MIN)
    {
      lines = IMX335_VMAX - IMX335_SHR0_MIN;
    }

  shr = IMX335_VMAX - lines;

  ret = imx335_putreg(priv, IMX335_REG_REGHOLD, 1, 1);
  if (ret >= 0)
    {
      ret = imx335_putreg(priv, IMX335_REG_SHR0, shr, 3);
      imx335_putreg(priv, IMX335_REG_REGHOLD, 0, 1);
    }

  if (ret >= 0)
    {
      priv->exposure = exposure;
    }

  return ret;
}

/****************************************************************************
 * Name: imx335_setgain
 ****************************************************************************/

static int imx335_setgain(FAR struct imx335_dev_s *priv, uint32_t gain)
{
  int ret;

  ret = imx335_putreg(priv, IMX335_REG_REGHOLD, 1, 1);
  if (ret >= 0)
    {
      ret = imx335_putreg(priv, IMX335_REG_GAIN, gain, 2);
      imx335_putreg(priv, IMX335_REG_REGHOLD, 0, 1);
    }

  if (ret >= 0)
    {
      priv->gain = gain;
    }

  return ret;
}

/****************************************************************************
 * Name: imx335_is_available
 ****************************************************************************/

static bool imx335_is_available(FAR struct imgsensor_s *sensor)
{
  FAR struct imx335_dev_s *priv = (FAR struct imx335_dev_s *)sensor;
  uint8_t vmax[3];
  uint8_t id = 0xff;
  int ret;
  int i;

  /* The ID register reads 0, so VMAX is checked too: it keeps its reset
   * value of 4500 lines as the driver never changes it.
   */

  nxmutex_lock(&priv->lock);
  ret = imx335_getreg(priv, IMX335_REG_ID, &id);
  for (i = 0; i < 3 && ret >= 0; i++)
    {
      ret = imx335_getreg(priv, IMX335_REG_VMAX + i, &vmax[i]);
    }

  nxmutex_unlock(&priv->lock);
  return ret >= 0 && id == 0 &&
         (vmax[0] | (vmax[1] << 8) | ((uint32_t)vmax[2] << 16)) ==
         IMX335_VMAX;
}

/****************************************************************************
 * Name: imx335_init
 ****************************************************************************/

static int imx335_init(FAR struct imgsensor_s *sensor)
{
  FAR struct imx335_dev_s *priv = (FAR struct imx335_dev_s *)sensor;
  int ret;

  nxmutex_lock(&priv->lock);
  ret = imx335_putreglist(priv, g_imx335_init_regs,
                          nitems(g_imx335_init_regs));
  if (ret >= 0)
    {
      ret = imx335_putreglist(priv, g_imx335_mode_regs,
                              nitems(g_imx335_mode_regs));
    }

  if (ret >= 0)
    {
      ret = imx335_putreglist(priv, priv->inck->regs,
                              nitems(priv->inck->regs));
    }

  if (ret >= 0)
    {
      ret = imx335_setgain(priv, priv->gain);
    }

  if (ret >= 0)
    {
      ret = imx335_setexposure(priv, priv->exposure);
    }

  if (ret < 0)
    {
      snerr("ERROR: IMX335 initialization failed: %d\n", ret);
    }

  priv->streaming = false;
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: imx335_uninit
 ****************************************************************************/

static int imx335_uninit(FAR struct imgsensor_s *sensor)
{
  FAR struct imx335_dev_s *priv = (FAR struct imx335_dev_s *)sensor;
  int ret;

  nxmutex_lock(&priv->lock);
  priv->streaming = false;
  ret = imx335_putreg(priv, IMX335_REG_STANDBY, IMX335_STANDBY, 1);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: imx335_get_driver_name
 ****************************************************************************/

static FAR const char *
imx335_get_driver_name(FAR struct imgsensor_s *sensor)
{
  return "IMX335";
}

/****************************************************************************
 * Name: imx335_validate_frame_setting
 ****************************************************************************/

static int imx335_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval)
{
  FAR imgsensor_format_t *fmt;

  if (nr_datafmts != 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  fmt = &datafmts[IMGSENSOR_FMT_MAIN];
  if (fmt->pixelformat != IMGSENSOR_PIX_FMT_SRGGB10P ||
      fmt->width != IMX335_WIDTH || fmt->height != IMX335_HEIGHT)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: imx335_start_capture
 ****************************************************************************/

static int imx335_start_capture(FAR struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval)
{
  FAR struct imx335_dev_s *priv = (FAR struct imx335_dev_s *)sensor;
  int ret = OK;

  nxmutex_lock(&priv->lock);
  if (!priv->streaming)
    {
      ret = imx335_putreg(priv, IMX335_REG_STANDBY, IMX335_OPERATING, 1);
      if (ret >= 0)
        {
          /* Settling time after leaving standby, as in ST's driver */

          nxsig_usleep(20 * USEC_PER_MSEC);
          priv->streaming = true;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: imx335_stop_capture
 ****************************************************************************/

static int imx335_stop_capture(FAR struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type)
{
  FAR struct imx335_dev_s *priv = (FAR struct imx335_dev_s *)sensor;
  int ret = OK;

  /* The capture framework may stop the stream from the frame interrupt
   * when it runs out of buffers.  I2C is not possible there; keep the
   * sensor running, the capture interface drops the frames.
   */

  if (up_interrupt_context())
    {
      return OK;
    }

  nxmutex_lock(&priv->lock);
  if (priv->streaming)
    {
      priv->streaming = false;
      ret = imx335_putreg(priv, IMX335_REG_STANDBY, IMX335_STANDBY, 1);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: imx335_get_frame_interval
 ****************************************************************************/

static int imx335_get_frame_interval(FAR struct imgsensor_s *sensor,
                                     imgsensor_stream_type_t type,
                                     FAR imgsensor_interval_t *interval)
{
  interval->numerator   = 1;
  interval->denominator = IMX335_FPS;
  return OK;
}

/****************************************************************************
 * Name: imx335_get_supported_value
 ****************************************************************************/

static int imx335_get_supported_value(FAR struct imgsensor_s *sensor,
                                      uint32_t id,
                                      FAR imgsensor_supported_value_t *val)
{
  switch (id)
    {
      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        val->type                  = IMGSENSOR_CTRL_TYPE_INTEGER;
        val->u.range.minimum       = 1;
        val->u.range.maximum       = 10000 / IMX335_FPS;
        val->u.range.step          = 1;
        val->u.range.default_value = CONFIG_VIDEO_IMX335_EXPOSURE;
        return OK;

      case IMGSENSOR_ID_GAIN:
        val->type                  = IMGSENSOR_CTRL_TYPE_INTEGER;
        val->u.range.minimum       = 0;
        val->u.range.maximum       = IMX335_GAIN_MAX;
        val->u.range.step          = 1;
        val->u.range.default_value = CONFIG_VIDEO_IMX335_GAIN;
        return OK;

      default:
        return -EINVAL;
    }
}

/****************************************************************************
 * Name: imx335_get_value
 ****************************************************************************/

static int imx335_get_value(FAR struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            FAR imgsensor_value_t *value)
{
  FAR struct imx335_dev_s *priv = (FAR struct imx335_dev_s *)sensor;
  int ret = OK;

  nxmutex_lock(&priv->lock);
  switch (id)
    {
      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        value->value32 = priv->exposure;
        break;

      case IMGSENSOR_ID_GAIN:
        value->value32 = priv->gain;
        break;

      default:
        ret = -EINVAL;
        break;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: imx335_set_value
 ****************************************************************************/

static int imx335_set_value(FAR struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t value)
{
  FAR struct imx335_dev_s *priv = (FAR struct imx335_dev_s *)sensor;
  int ret;

  switch (id)
    {
      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        if (value.value32 < 1 || value.value32 > 10000 / IMX335_FPS)
          {
            return -EINVAL;
          }

        nxmutex_lock(&priv->lock);
        ret = imx335_setexposure(priv, value.value32);
        nxmutex_unlock(&priv->lock);
        return ret;

      case IMGSENSOR_ID_GAIN:
        if (value.value32 < 0 || value.value32 > IMX335_GAIN_MAX)
          {
            return -EINVAL;
          }

        nxmutex_lock(&priv->lock);
        ret = imx335_setgain(priv, value.value32);
        nxmutex_unlock(&priv->lock);
        return ret;

      default:
        return -EINVAL;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx335_initialize
 ****************************************************************************/

FAR struct imgsensor_s *imx335_initialize(FAR struct i2c_master_s *i2c,
                                          uint32_t inck)
{
  FAR struct imx335_dev_s *priv;
  int i;

  if (i2c == NULL)
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(struct imx335_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  for (i = 0; i < nitems(g_imx335_inck); i++)
    {
      if (g_imx335_inck[i].freq == inck)
        {
          priv->inck = &g_imx335_inck[i];
          break;
        }
    }

  if (priv->inck == NULL)
    {
      snerr("ERROR: unsupported input clock %" PRIu32 "\n", inck);
      kmm_free(priv);
      return NULL;
    }

  priv->i2c      = i2c;
  nxmutex_init(&priv->lock);
  priv->exposure = CONFIG_VIDEO_IMX335_EXPOSURE;
  priv->gain     = CONFIG_VIDEO_IMX335_GAIN;

  priv->sensor.ops              = &g_imx335_ops;
  priv->sensor.fmtdescs         = g_imx335_fmtdescs;
  priv->sensor.fmtdescs_num     = nitems(g_imx335_fmtdescs);
  priv->sensor.frmsizes         = g_imx335_frmsizes;
  priv->sensor.frmsizes_num     = nitems(g_imx335_frmsizes);
  priv->sensor.frmintervals     = g_imx335_frmintervals;
  priv->sensor.frmintervals_num = nitems(g_imx335_frmintervals);

  return &priv->sensor;
}
