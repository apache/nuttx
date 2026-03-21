/****************************************************************************
 * drivers/video/gc0308.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/arch.h>
#include <nuttx/video/video.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GC0308_I2C_ADDR         0x21
#define GC0308_I2C_FREQ         100000

/* GC0308 Register Addresses */

#define GC0308_REG_CHIP_ID      0x00
#define GC0308_CHIP_ID_VAL      0x9b

/* Page 0 registers */

#define GC0308_REG_RESET        0xfe  /* Page/reset register */
#define GC0308_REG_HBLANK_H     0x01
#define GC0308_REG_HBLANK_L     0x02
#define GC0308_REG_VBLANK_H     0x03
#define GC0308_REG_VBLANK_L     0x04
#define GC0308_REG_SH_DELAY     0x05
#define GC0308_REG_ROW_START_H  0x06
#define GC0308_REG_ROW_START_L  0x07
#define GC0308_REG_COL_START_H  0x08
#define GC0308_REG_COL_START_L  0x09
#define GC0308_REG_WIN_HEIGHT_H 0x0a
#define GC0308_REG_WIN_HEIGHT_L 0x0b
#define GC0308_REG_WIN_WIDTH_H  0x0c
#define GC0308_REG_WIN_WIDTH_L  0x0d
#define GC0308_REG_VS_ST        0x0e
#define GC0308_REG_VS_ET        0x0f
#define GC0308_REG_VB_HB        0x10
#define GC0308_REG_RSH_WIDTH    0x11
#define GC0308_REG_TSP_WIDTH    0x12
#define GC0308_REG_SAMPLE_HOLD  0x13
#define GC0308_REG_CISCTL_MODE1 0x14
#define GC0308_REG_CISCTL_MODE2 0x15
#define GC0308_REG_CISCTL_MODE3 0x16
#define GC0308_REG_CISCTL_MODE4 0x17

/* Output format (Page 0) */

#define GC0308_REG_OUTPUT_FMT   0x24  /* Bits[3:0]: output format select */
#define GC0308_REG_OUT_FORMAT   0x44
#define GC0308_REG_OUT_EN       0x45
#define GC0308_REG_SYNC_MODE    0x46

/* Analog & bias */

#define GC0308_REG_ANALOG_MODE1 0x1a
#define GC0308_REG_ANALOG_MODE2 0x1b

/* Exposure */

#define GC0308_REG_EXP_H        0x03
#define GC0308_REG_EXP_L        0x04

/* Gain */

#define GC0308_REG_GLOBAL_GAIN  0x50

/* Crop */

#define GC0308_REG_CROP_WIN_MODE 0x46
#define GC0308_REG_CROP_Y1_H    0x47
#define GC0308_REG_CROP_Y1_L    0x48
#define GC0308_REG_CROP_X1_H    0x49
#define GC0308_REG_CROP_X1_L    0x4a
#define GC0308_REG_CROP_WIN_H_H 0x4b
#define GC0308_REG_CROP_WIN_H_L 0x4c
#define GC0308_REG_CROP_WIN_W_H 0x4d
#define GC0308_REG_CROP_WIN_W_L 0x4e

/* Subsample (Page 0) */

#define GC0308_REG_SUBSAMPLE    0x59
#define GC0308_REG_SUB_MODE     0x5a
#define GC0308_REG_SUB_ROW_N1   0x5b
#define GC0308_REG_SUB_ROW_N2   0x5c
#define GC0308_REG_SUB_COL_N1   0x5d
#define GC0308_REG_SUB_COL_N2   0x5e

/* Subsample control (Page 1) */

#define GC0308_P1_REG_SUB_CTRL    0x53  /* Bit7: subsample enable */
#define GC0308_P1_REG_SUB_RATIO   0x54  /* H[7:4] V[3:0] ratio */
#define GC0308_P1_REG_SUB_EN      0x55  /* Bit0: subsample output enable */
#define GC0308_P1_REG_SUB_HOFF    0x56  /* H offset */
#define GC0308_P1_REG_SUB_VOFF    0x57  /* V offset */
#define GC0308_P1_REG_SUB_HSIZE   0x58  /* H size adjust */
#define GC0308_P1_REG_SUB_VSIZE   0x59  /* V size adjust */

/* Native sensor resolution (VGA) — used as default when caller passes 0 */

#define GC0308_NATIVE_WIDTH   640
#define GC0308_NATIVE_HEIGHT  480

/* Mirror/flip */

#define GC0308_REG_CISCTL_MODE1_MIRROR  0x14

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gc0308_reg_s
{
  uint8_t addr;
  uint8_t val;
};

struct gc0308_dev_s
{
  struct imgsensor_s sensor;
  struct i2c_master_s *i2c;
  uint16_t width;
  uint16_t height;
  uint32_t pixelformat;
  struct v4l2_frmsizeenum frmsizes;
  bool streaming;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool gc0308_is_available(struct imgsensor_s *sensor);
static int gc0308_init(struct imgsensor_s *sensor);
static int gc0308_uninit(struct imgsensor_s *sensor);
static const char *gc0308_get_driver_name(struct imgsensor_s *sensor);
static int gc0308_validate_frame_setting(struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         imgsensor_format_t *datafmts,
                                         imgsensor_interval_t *interval);
static int gc0308_start_capture(struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                imgsensor_format_t *datafmts,
                                imgsensor_interval_t *interval);
static int gc0308_stop_capture(struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type);
static int gc0308_get_supported_value(struct imgsensor_s *sensor,
                                      uint32_t id,
                                      imgsensor_supported_value_t *value);
static int gc0308_get_value(struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t *value);
static int gc0308_set_value(struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* GC0308 initialization register table from espressif/esp32-camera vendor
 * driver. This is the proven register set for GC0308 VGA output.
 */

static const struct gc0308_reg_s g_gc0308_init_regs[] =
{
  { 0xfe, 0x00 },
  { 0xec, 0x20 },
  { 0x05, 0x00 },
  { 0x06, 0x00 },
  { 0x07, 0x00 },
  { 0x08, 0x00 },
  { 0x09, 0x01 },
  { 0x0a, 0xe8 },
  { 0x0b, 0x02 },
  { 0x0c, 0x88 },
  { 0x0d, 0x02 },
  { 0x0e, 0x02 },
  { 0x10, 0x26 },
  { 0x11, 0x0d },
  { 0x12, 0x2a },
  { 0x13, 0x00 },
  { 0x14, 0x10 },
  { 0x15, 0x0a },
  { 0x16, 0x05 },
  { 0x17, 0x01 },
  { 0x18, 0x44 },
  { 0x19, 0x44 },
  { 0x1a, 0x2a },
  { 0x1b, 0x00 },
  { 0x1c, 0x49 },
  { 0x1d, 0x9a },
  { 0x1e, 0x61 },
  { 0x1f, 0x00 },
  { 0x20, 0x7f },
  { 0x21, 0xfa },
  { 0x22, 0x57 },
  { 0x24, 0xa2 },  /* YCbYCr output format */
  { 0x25, 0x0f },
  { 0x26, 0x03 },
  { 0x28, 0x00 },
  { 0x2d, 0x0a },
  { 0x2f, 0x01 },
  { 0x30, 0xf7 },
  { 0x31, 0x50 },
  { 0x32, 0x00 },
  { 0x33, 0x28 },
  { 0x34, 0x2a },
  { 0x35, 0x28 },
  { 0x39, 0x04 },
  { 0x3a, 0x20 },
  { 0x3b, 0x20 },
  { 0x3c, 0x00 },
  { 0x3d, 0x00 },
  { 0x3e, 0x00 },
  { 0x3f, 0x00 },
  { 0x50, 0x14 },  /* Global gain */
  { 0x52, 0x41 },
  { 0x53, 0x80 },
  { 0x54, 0x80 },
  { 0x55, 0x80 },
  { 0x56, 0x80 },
  { 0x5a, 0x56 },  /* AWB R gain */
  { 0x5b, 0x40 },  /* AWB G gain */
  { 0x5c, 0x4a },  /* AWB B gain */
  { 0x8b, 0x20 },
  { 0x8c, 0x20 },
  { 0x8d, 0x20 },
  { 0x8e, 0x14 },
  { 0x8f, 0x10 },
  { 0x90, 0x14 },
  { 0x91, 0x3c },
  { 0x92, 0x50 },
  { 0x5d, 0x12 },
  { 0x5e, 0x1a },
  { 0x5f, 0x24 },
  { 0x60, 0x07 },
  { 0x61, 0x15 },
  { 0x62, 0x08 },
  { 0x64, 0x03 },
  { 0x66, 0xe8 },
  { 0x67, 0x86 },
  { 0x68, 0x82 },
  { 0x69, 0x18 },
  { 0x6a, 0x0f },
  { 0x6b, 0x00 },
  { 0x6c, 0x5f },
  { 0x6d, 0x8f },
  { 0x6e, 0x55 },
  { 0x6f, 0x38 },
  { 0x70, 0x15 },
  { 0x71, 0x33 },
  { 0x72, 0xdc },
  { 0x73, 0x00 },
  { 0x74, 0x02 },
  { 0x75, 0x3f },
  { 0x76, 0x02 },
  { 0x77, 0x38 },
  { 0x78, 0x88 },
  { 0x79, 0x81 },
  { 0x7a, 0x81 },
  { 0x7b, 0x22 },
  { 0x7c, 0xff },
  { 0x93, 0x48 },  /* Color matrix */
  { 0x94, 0x02 },
  { 0x95, 0x07 },
  { 0x96, 0xe0 },
  { 0x97, 0x40 },
  { 0x98, 0xf0 },
  { 0xb1, 0x40 },  /* Saturation */
  { 0xb2, 0x40 },
  { 0xb3, 0x40 },
  { 0xb6, 0xe0 },
  { 0xbd, 0x38 },
  { 0xbe, 0x36 },
  { 0xd0, 0xcb },  /* AEC */
  { 0xd1, 0x10 },
  { 0xd2, 0x90 },
  { 0xd3, 0x48 },
  { 0xd5, 0xf2 },
  { 0xd6, 0x16 },
  { 0xdb, 0x92 },
  { 0xdc, 0xa5 },
  { 0xdf, 0x23 },
  { 0xd9, 0x00 },
  { 0xda, 0x00 },
  { 0xe0, 0x09 },
  { 0xed, 0x04 },
  { 0xee, 0xa0 },
  { 0xef, 0x40 },
  { 0x80, 0x03 },
  { 0x9f, 0x10 },
  { 0xa0, 0x20 },
  { 0xa1, 0x38 },
  { 0xa2, 0x4e },
  { 0xa3, 0x63 },
  { 0xa4, 0x76 },
  { 0xa5, 0x87 },
  { 0xa6, 0xa2 },
  { 0xa7, 0xb8 },
  { 0xa8, 0xca },
  { 0xa9, 0xd8 },
  { 0xaa, 0xe3 },
  { 0xab, 0xeb },
  { 0xac, 0xf0 },
  { 0xad, 0xf8 },
  { 0xae, 0xfd },
  { 0xaf, 0xff },
  { 0xc0, 0x00 },
  { 0xc1, 0x10 },
  { 0xc2, 0x1c },
  { 0xc3, 0x30 },
  { 0xc4, 0x43 },
  { 0xc5, 0x54 },
  { 0xc6, 0x65 },
  { 0xc7, 0x75 },
  { 0xc8, 0x93 },
  { 0xc9, 0xb0 },
  { 0xca, 0xcb },
  { 0xcb, 0xe6 },
  { 0xcc, 0xff },
  { 0xf0, 0x02 },
  { 0xf1, 0x01 },
  { 0xf2, 0x02 },
  { 0xf3, 0x30 },
  { 0xf7, 0x04 },
  { 0xf8, 0x02 },
  { 0xf9, 0x9f },
  { 0xfa, 0x78 },
  { 0xfe, 0x01 },
  { 0x00, 0xf5 },
  { 0x02, 0x20 },
  { 0x04, 0x10 },
  { 0x05, 0x08 },
  { 0x06, 0x20 },
  { 0x08, 0x0a },
  { 0x0a, 0xa0 },
  { 0x0b, 0x60 },
  { 0x0c, 0x08 },
  { 0x0e, 0x44 },
  { 0x0f, 0x32 },
  { 0x10, 0x41 },
  { 0x11, 0x37 },
  { 0x12, 0x22 },
  { 0x13, 0x19 },
  { 0x14, 0x44 },
  { 0x15, 0x44 },
  { 0x16, 0xc2 },
  { 0x17, 0xa8 },
  { 0x18, 0x18 },
  { 0x19, 0x50 },
  { 0x1a, 0xd8 },
  { 0x1b, 0xf5 },
  { 0x70, 0x40 },
  { 0x71, 0x58 },
  { 0x72, 0x30 },
  { 0x73, 0x48 },
  { 0x74, 0x20 },
  { 0x75, 0x60 },
  { 0x77, 0x20 },
  { 0x78, 0x32 },
  { 0x30, 0x03 },
  { 0x31, 0x40 },
  { 0x32, 0x10 },
  { 0x33, 0xe0 },
  { 0x34, 0xe0 },
  { 0x35, 0x00 },
  { 0x36, 0x80 },
  { 0x37, 0x00 },
  { 0x38, 0x04 },
  { 0x39, 0x09 },
  { 0x3a, 0x12 },
  { 0x3b, 0x1c },
  { 0x3c, 0x28 },
  { 0x3d, 0x31 },
  { 0x3e, 0x44 },
  { 0x3f, 0x57 },
  { 0x40, 0x6c },
  { 0x41, 0x81 },
  { 0x42, 0x94 },
  { 0x43, 0xa7 },
  { 0x44, 0xb8 },
  { 0x45, 0xd6 },
  { 0x46, 0xee },
  { 0x47, 0x0d },
  { 0x62, 0xf7 },
  { 0x63, 0x68 },
  { 0x64, 0xd3 },
  { 0x65, 0xd3 },
  { 0x66, 0x60 },
  { 0xfe, 0x00 },
  { 0x01, 0x32 },
  { 0x02, 0x0c },
  { 0x0f, 0x01 },
  { 0xe2, 0x00 },
  { 0xe3, 0x78 },
  { 0xe4, 0x00 },
  { 0xe5, 0xfe },
  { 0xe6, 0x01 },
  { 0xe7, 0xe0 },
  { 0xe8, 0x01 },
  { 0xe9, 0xe0 },
  { 0xea, 0x01 },
  { 0xeb, 0xe0 },
  { 0xfe, 0x00 },
};

static const struct imgsensor_ops_s g_gc0308_ops =
{
  .is_available           = gc0308_is_available,
  .init                   = gc0308_init,
  .uninit                 = gc0308_uninit,
  .get_driver_name        = gc0308_get_driver_name,
  .validate_frame_setting = gc0308_validate_frame_setting,
  .start_capture          = gc0308_start_capture,
  .stop_capture           = gc0308_stop_capture,
  .get_supported_value    = gc0308_get_supported_value,
  .get_value              = gc0308_get_value,
  .set_value              = gc0308_set_value,
};

static const struct v4l2_fmtdesc g_gc0308_fmtdescs[] =
{
  {
    .pixelformat = V4L2_PIX_FMT_YUYV,
    .description = "YUV 4:2:2 (YUYV)",
  },
  {
    .pixelformat = V4L2_PIX_FMT_RGB565,
    .description = "RGB565",
  },
};

static const struct v4l2_frmivalenum g_gc0308_frmintervals[] =
{
  {
    .type = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete =
      {
        .numerator   = 1,
        .denominator = 30,
      },
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gc0308_putreg
 ****************************************************************************/

static int gc0308_putreg(struct i2c_master_s *i2c,
                         uint8_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg;
  uint8_t buf[2];
  int ret;

  buf[0] = regaddr;
  buf[1] = regval;

  msg.frequency = GC0308_I2C_FREQ;
  msg.addr      = GC0308_I2C_ADDR;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 2;

  ret = I2C_TRANSFER(i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C write to 0x%02x failed: %d\n", regaddr, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: gc0308_getreg
 ****************************************************************************/

static int gc0308_getreg(struct i2c_master_s *i2c,
                         uint8_t regaddr, uint8_t *regval)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = GC0308_I2C_FREQ;
  msg[0].addr      = GC0308_I2C_ADDR;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = GC0308_I2C_FREQ;
  msg[1].addr      = GC0308_I2C_ADDR;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C read from 0x%02x failed: %d\n", regaddr, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: gc0308_modreg
 *
 * Description:
 *   Read-modify-write an 8-bit register.
 *
 ****************************************************************************/

static int gc0308_modreg(struct i2c_master_s *i2c,
                         uint8_t regaddr,
                         uint8_t clearbits, uint8_t setbits)
{
  uint8_t regval;
  int ret;

  ret = gc0308_getreg(i2c, regaddr, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval = (regval & ~clearbits) | setbits;

  return gc0308_putreg(i2c, regaddr, regval);
}

/****************************************************************************
 * Name: gc0308_putreglist
 ****************************************************************************/

static int gc0308_putreglist(struct i2c_master_s *i2c,
                             const struct gc0308_reg_s *reglist,
                             size_t nentries)
{
  size_t i;
  int ret;

  for (i = 0; i < nentries; i++)
    {
      ret = gc0308_putreg(i2c, reglist[i].addr, reglist[i].val);
      if (ret < 0)
        {
          snerr("GC0308 write[%d] 0x%02x=0x%02x FAILED: %d\n",
                (int)i, reglist[i].addr, reglist[i].val, ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: gc0308_is_available
 ****************************************************************************/

static bool gc0308_is_available(struct imgsensor_s *sensor)
{
  struct gc0308_dev_s *priv = (struct gc0308_dev_s *)sensor;
  uint8_t id = 0;
  int ret;

  /* Select page 0 */

  gc0308_putreg(priv->i2c, GC0308_REG_RESET, 0x00);

  ret = gc0308_getreg(priv->i2c, GC0308_REG_CHIP_ID, &id);
  if (ret < 0)
    {
      return false;
    }

  sninfo("GC0308 chip ID: 0x%02x (expected 0x%02x)\n",
         id, GC0308_CHIP_ID_VAL);

  return (id == GC0308_CHIP_ID_VAL);
}

/****************************************************************************
 * Name: gc0308_init
 ****************************************************************************/

static int gc0308_init(struct imgsensor_s *sensor)
{
  struct gc0308_dev_s *priv = (struct gc0308_dev_s *)sensor;
  uint8_t id = 0;
  uint8_t fmt = 0;
  int ret;

  /* Software reset per vendor: write 0xf0 to reg 0xfe, wait 80ms */

  ret = gc0308_putreg(priv->i2c, 0xfe, 0xf0);
  if (ret < 0)
    {
      snerr("GC0308 soft reset failed: %d\n", ret);
      return ret;
    }

  up_mdelay(80);

  /* Write vendor initialization register table */

  ret = gc0308_putreglist(priv->i2c, g_gc0308_init_regs,
                          nitems(g_gc0308_init_regs));
  if (ret < 0)
    {
      snerr("GC0308 init regs failed: %d\n", ret);
      return ret;
    }

  up_mdelay(80);

  /* Set default RGB565 output format: reg 0x24 bits[3:0] = 6 */

  ret = gc0308_putreg(priv->i2c, GC0308_REG_RESET, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  ret = gc0308_modreg(priv->i2c, GC0308_REG_OUTPUT_FMT, 0x0f, 0x06);
  priv->pixelformat = IMGSENSOR_PIX_FMT_RGB565;
  if (ret < 0)
    {
      return ret;
    }

  /* Configure subsample for non-VGA resolutions */

  if (priv->width < 640 || priv->height < 480)
    {
      uint8_t ratio;

      /* Switch to page 1 */

      ret = gc0308_putreg(priv->i2c, GC0308_REG_RESET, 0x01);
      if (ret < 0)
        {
          return ret;
        }

      ret = gc0308_modreg(priv->i2c, GC0308_P1_REG_SUB_CTRL, 0, 0x80);
      ret |= gc0308_modreg(priv->i2c, GC0308_P1_REG_SUB_EN, 0, 0x01);

      if (priv->width <= 160)
        {
          ratio = 0x44;  /* 1/4 */
        }
      else
        {
          ratio = 0x22;  /* 1/2 */
        }

      ret |= gc0308_putreg(priv->i2c, GC0308_P1_REG_SUB_RATIO, ratio);
      ret |= gc0308_putreg(priv->i2c, GC0308_P1_REG_SUB_HOFF,  0x00);
      ret |= gc0308_putreg(priv->i2c, GC0308_P1_REG_SUB_VOFF,  0x00);
      ret |= gc0308_putreg(priv->i2c, GC0308_P1_REG_SUB_HSIZE, 0x00);
      ret |= gc0308_putreg(priv->i2c, GC0308_P1_REG_SUB_VSIZE, 0x00);

      /* Back to page 0 */

      ret |= gc0308_putreg(priv->i2c, GC0308_REG_RESET, 0x00);
    }

  /* Debug: verify key registers */

  gc0308_getreg(priv->i2c, 0x00, &id);
  gc0308_getreg(priv->i2c, 0x24, &fmt);
  syslog(LOG_INFO, "GC0308 init done: id=0x%02x fmt=0x%02x ret=%d\n",
         id, fmt, ret);

  return ret;
}

/****************************************************************************
 * Name: gc0308_uninit
 ****************************************************************************/

static int gc0308_uninit(struct imgsensor_s *sensor)
{
  struct gc0308_dev_s *priv = (struct gc0308_dev_s *)sensor;

  /* Soft reset per vendor */

  gc0308_putreg(priv->i2c, GC0308_REG_RESET, 0xf0);
  priv->streaming = false;

  return OK;
}

/****************************************************************************
 * Name: gc0308_get_driver_name
 ****************************************************************************/

static const char *gc0308_get_driver_name(struct imgsensor_s *sensor)
{
  return "GC0308";
}

/****************************************************************************
 * Name: gc0308_validate_frame_setting
 ****************************************************************************/

static int gc0308_validate_frame_setting(struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         imgsensor_format_t *datafmts,
                                         imgsensor_interval_t *interval)
{
  struct gc0308_dev_s *priv = (struct gc0308_dev_s *)sensor;

  if (nr_datafmts < 1 || !datafmts)
    {
      return -EINVAL;
    }

  if (datafmts[IMGSENSOR_FMT_MAIN].pixelformat !=
      IMGSENSOR_PIX_FMT_YUYV &&
      datafmts[IMGSENSOR_FMT_MAIN].pixelformat !=
      IMGSENSOR_PIX_FMT_RGB565)
    {
      return -EINVAL;
    }

  if (datafmts[IMGSENSOR_FMT_MAIN].width != priv->width ||
      datafmts[IMGSENSOR_FMT_MAIN].height != priv->height)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: gc0308_start_capture
 ****************************************************************************/

static int gc0308_start_capture(struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                imgsensor_format_t *datafmts,
                                imgsensor_interval_t *interval)
{
  struct gc0308_dev_s *priv = (struct gc0308_dev_s *)sensor;
  uint8_t fmtval;
  int ret;

  if (priv->streaming)
    {
      return -EBUSY;
    }

  /* Configure output format register based on requested pixel format */

  if (datafmts[IMGSENSOR_FMT_MAIN].pixelformat == IMGSENSOR_PIX_FMT_RGB565)
    {
      fmtval = 0x06;  /* RGB565 */
    }
  else
    {
      fmtval = 0x02;  /* YCbCr422 */
    }

  ret = gc0308_modreg(priv->i2c, GC0308_REG_OUTPUT_FMT, 0x0f, fmtval);
  if (ret < 0)
    {
      return ret;
    }

  priv->pixelformat = datafmts[IMGSENSOR_FMT_MAIN].pixelformat;
  priv->streaming = true;
  return OK;
}

/****************************************************************************
 * Name: gc0308_stop_capture
 ****************************************************************************/

static int gc0308_stop_capture(struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type)
{
  struct gc0308_dev_s *priv = (struct gc0308_dev_s *)sensor;

  priv->streaming = false;
  return OK;
}

/****************************************************************************
 * Name: gc0308_get_supported_value
 ****************************************************************************/

static int gc0308_get_supported_value(struct imgsensor_s *sensor,
                                      uint32_t id,
                                      imgsensor_supported_value_t *value)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: gc0308_get_value
 ****************************************************************************/

static int gc0308_get_value(struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t *value)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: gc0308_set_value
 ****************************************************************************/

static int gc0308_set_value(struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t value)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gc0308_initialize
 *
 * Description:
 *   Initialize the GC0308 camera sensor driver.
 *
 * Input Parameters:
 *   i2c    - I2C bus device
 *   width  - Desired frame width  (0 = VGA 640)
 *   height - Desired frame height (0 = VGA 480)
 *
 * Returned Value:
 *   Pointer to imgsensor_s on success; NULL on failure.
 *
 ****************************************************************************/

struct imgsensor_s *gc0308_initialize(struct i2c_master_s *i2c,
                                      uint16_t width,
                                      uint16_t height)
{
  struct gc0308_dev_s *priv;

  if (!i2c)
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(struct gc0308_dev_s));
  if (!priv)
    {
      return NULL;
    }

  priv->i2c       = i2c;
  priv->streaming = false;
  priv->width     = width  ? width  : GC0308_NATIVE_WIDTH;
  priv->height    = height ? height : GC0308_NATIVE_HEIGHT;

  priv->frmsizes.type             = V4L2_FRMSIZE_TYPE_DISCRETE;
  priv->frmsizes.discrete.width   = priv->width;
  priv->frmsizes.discrete.height  = priv->height;

  priv->sensor.ops              = &g_gc0308_ops;
  priv->sensor.fmtdescs         = g_gc0308_fmtdescs;
  priv->sensor.fmtdescs_num     = nitems(g_gc0308_fmtdescs);
  priv->sensor.frmsizes         = &priv->frmsizes;
  priv->sensor.frmsizes_num     = 1;
  priv->sensor.frmintervals     = g_gc0308_frmintervals;
  priv->sensor.frmintervals_num = nitems(g_gc0308_frmintervals);

  return &priv->sensor;
}
