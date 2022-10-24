/****************************************************************************
 * drivers/video/isx012.c
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

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mutex.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>
#include <arch/irq.h>

#include <nuttx/video/isx012.h>
#include "isx012_reg.h"
#include "isx012_range.h"
#include <nuttx/video/imgsensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following macro is enabled because
 * it is to make stable startup. (other case)
 */

/* #define ISX012_NOT_USE_NSTBY */

/* The following macro is disabled because it is to see detailed control. */

/* #define ISX012_CHECK_IN_DETAIL */

/* Skip invalid frame because it occurs first due to the spec of isx012. */

#define ISX012_FRAME_SKIP_EN

#define OUT_HSIZE_QVGA           (320)
#define OUT_VSIZE_QVGA           (240)
#define OUT_HSIZE_VGA            (640)
#define OUT_VSIZE_VGA            (480)
#define OUT_HSIZE_HD            (1280)
#define OUT_VSIZE_HD             (720)
#define OUT_HSIZE_QUADVGA       (1280)
#define OUT_VSIZE_QUADVGA        (960)
#define OUT_HSIZE_FULLHD        (1920)
#define OUT_VSIZE_FULLHD        (1080)
#define OUT_HSIZE_3M            (2048)
#define OUT_VSIZE_3M            (1536)
#define OUT_HSIZE_5M            (2560)
#define OUT_VSIZE_5M            (1920)

#define OUT_YUV_VSIZE_MIN         (64)
#define OUT_YUV_HSIZE_MIN         (96)
#define OUT_JPG_VSIZE_MIN         (64)
#define OUT_JPG_HSIZE_MIN         (96)
#define OUT_YUV_15FPS_VSIZE_MAX  (600)
#define OUT_YUV_15FPS_HSIZE_MAX  (800)
#define OUT_YUV_30FPS_VSIZE_MAX  (600)
#define OUT_YUV_30FPS_HSIZE_MAX  (800)
#define OUT_YUV_60FPS_VSIZE_MAX  (480)
#define OUT_YUV_60FPS_HSIZE_MAX  (640)
#define OUT_YUV_120FPS_VSIZE_MAX (240)
#define OUT_YUV_120FPS_HSIZE_MAX (320)
#define OUT_JPG_15FPS_VSIZE_MAX (1944)
#define OUT_JPG_15FPS_HSIZE_MAX (2592)
#define OUT_JPG_30FPS_VSIZE_MAX  (960)
#define OUT_JPG_30FPS_HSIZE_MAX (1280)
#define OUT_JPG_60FPS_VSIZE_MAX  (480)
#define OUT_JPG_60FPS_HSIZE_MAX  (640)
#define OUT_JPG_120FPS_VSIZE_MAX (240)
#define OUT_JPG_120FPS_HSIZE_MAX (320)

#define OUT_YUVINT_30FPS_VSIZE_MAX  (240)
#define OUT_YUVINT_30FPS_HSIZE_MAX  (400)
#define OUT_JPGINT_30FPS_VSIZE_MAX  (960)
#define OUT_JPGINT_30FPS_HSIZE_MAX (1280)
#define OUT_JPGINT_15FPS_VSIZE_MAX (1224)
#define OUT_JPGINT_15FPS_HSIZE_MAX (1632)

#define VINT_TIMEOUT                (400) /* ms */
#define VINT_WAIT_TIME                (5) /* ms */
#define VINT_DELAY_TIME               (0) /* ms */
#define CAMERA_MODE_TIMEOUT         (800) /* ms */
#define CAMERA_MODE_WAIT_TIME        (10) /* ms */
#define CAMERA_MODE_DELAY_TIME        (0) /* ms */
#define DEVICE_STATE_TIMEOUT        (100) /* ms */
#define DEVICE_STATE_WAIT_TIME        (1) /* ms */
#define DEVICE_STATE_DELAY_TIME       (2) /* ms */

#define I2CFREQ_STANDARD         (100000) /* Standard mode : 100kHz */
#define I2CFREQ_FAST             (400000) /* Fast mode     : 400kHz */

#define ISX012_SIZE_STEP              (2)

#define CXC_RGB_DATA_UNIT_NUM        (27)
#define CXC_RGB_DATA_UNIT_SIZE        (7)
#define CXC_GRB_DATA_UNIT_NUM        (27)
#define CXC_GRB_DATA_UNIT_SIZE        (7)
#define SHD_RGB_DATA_UNIT_NUM        (27)
#define SHD_RGB_DATA_UNIT_SIZE       (11)
#define SHD_GRB_DATA_UNIT_NUM        (27)
#define SHD_GRB_DATA_UNIT_SIZE       (11)
#define SHD_R1_DATA_UNIT_NUM         (14)
#define SHD_R1_DATA_UNIT_SIZE        (11)
#define SHD_R2_DATA_UNIT_NUM         (14)
#define SHD_R2_DATA_UNIT_SIZE        (11)
#define SHD_B2_DATA_UNIT_NUM         (14)
#define SHD_B2_DATA_UNIT_SIZE        (11)

#define ISX012_ELEMS_3APARAM         (3)

#define VALIDATE_VALUE(val, min, max, step) (((val >= min) && \
                                              (val <= max) && \
                                              (((val - min) % step) == 0) ? \
                                              OK : -EINVAL))

#define ISX012_CHIPID_L (0x0000c460)
#define ISX012_CHIPID_H (0x00005516)

#define BASE_HSIZE_FOR_CLIP_OFFSET (2592)
#define BASE_VSIZE_FOR_CLIP_OFFSET (1944)

#define ZOOM_UNIT        (0x0100)
#define CLIP_OFFSET_UNIT (0x0010)
#define CLIP_SIZE_UNIT   (8)
#define RESCALE_FOR_CLIP(v, a, b)  (((v) * (a)) / (b))

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum isx012_state_e
{
  STATE_ISX012_PRESLEEP,
  STATE_ISX012_SLEEP,
  STATE_ISX012_ACTIVE,
  STATE_ISX012_POWEROFF,
};

typedef enum isx012_state_e isx012_state_t;

struct isx012_reg_s
{
  uint16_t regaddr;
  uint16_t regval;
  uint8_t  regsize;
};

typedef struct isx012_reg_s isx012_reg_t;

struct isx012_rect_s
{
  int32_t left;
  int32_t top;
  uint32_t width;
  uint32_t height;
};

typedef struct isx012_rect_s isx012_rect_t;

struct isx012_dev_s
{
  mutex_t                 i2c_lock;
  FAR struct i2c_master_s *i2c;        /* I2C interface */
  uint8_t                 i2c_addr;    /* I2C address */
  int                     i2c_freq;    /* Frequency */
  isx012_state_t          state;       /* ISX012 status */
  uint8_t                 mode;        /* ISX012 mode */
  isx012_rect_t           clip_video;  /* Clip information for VIDEO */
  isx012_rect_t           clip_still;  /* Clip information for STILL */
};

typedef struct isx012_dev_s isx012_dev_t;

#define ARRAY_NENTRIES(a) (sizeof(a)/sizeof(a[0]))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* register operations */

static uint16_t isx012_getreg(FAR isx012_dev_t *priv,
                              uint16_t regaddr, uint16_t regsize);
static int     isx012_putreg(isx012_dev_t *priv, uint16_t regaddr,
                             uint16_t regval, uint16_t regsize);
static int     isx012_putreglist(isx012_dev_t *priv,
                         FAR const isx012_reg_t *reglist, size_t nentries);
#ifdef ISX012_CHECK_IN_DETAIL
static int     isx012_putregs(isx012_dev_t *priv, uint16_t regaddr,
                              FAR uint8_t *regvals, uint8_t regsize);
static int     isx012_chipid(FAR struct i2c_master_s *i2c);
#endif

static int isx012_chk_int_state(isx012_dev_t *priv,
                                uint8_t  sts, uint32_t delay_time,
                                uint32_t wait_time, uint32_t timeout);
static int isx012_set_mode_param(isx012_dev_t *priv,
                                 imgsensor_stream_type_t type,
                                 uint8_t nr_fmt,
                                 FAR imgsensor_format_t *fmt,
                                 FAR imgsensor_interval_t *interval);
static int isx012_change_camera_mode(isx012_dev_t *priv, uint8_t mode);
static int isx012_change_device_state(isx012_dev_t *priv,
                                      isx012_state_t state);
static int isx012_replace_frameinterval_to_regval
                (FAR imgsensor_interval_t *interval);
static int8_t isx012_get_maximum_fps
                (uint8_t nr_datafmt,
                 FAR imgsensor_format_t *datafmt);
static int isx012_set_shd(isx012_dev_t *priv);
static bool is_movie_needed(uint8_t fmt, uint8_t fps);

/* image sensor device operations interface */

static bool isx012_is_available(void);
static int isx012_init(void);
static int isx012_uninit(void);
static FAR const char *isx012_get_driver_name(void);
static int isx012_validate_frame_setting(imgsensor_stream_type_t type,
                                         uint8_t nr_datafmt,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval);
static int isx012_start_capture(imgsensor_stream_type_t type,
                                uint8_t nr_datafmt,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval);
static int isx012_stop_capture(imgsensor_stream_type_t type);
static int isx012_get_supported_value
             (uint32_t id, FAR imgsensor_supported_value_t *value);
static int isx012_get_value
             (uint32_t id, uint32_t size, FAR imgsensor_value_t *value);
static int isx012_set_value
             (uint32_t id, uint32_t size, imgsensor_value_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static isx012_dev_t g_isx012_private =
{
  NXMUTEX_INITIALIZER,
};

#ifndef ISX012_NOT_USE_NSTBY
static const isx012_reg_t g_isx012_presleep[] =
{
  {PLL_CKSEL, 0x00, 0x01}, /* PLL_CKSEL */
  {SRCCK_DIV, 0x00, 0x01}, /* SRCCK_DIV */
  {INCK_SET,  0x17, 0x01}, /* INCK_SET */
};
#define ISX012_PRESLEEP_NENTRIES ARRAY_NENTRIES(g_isx012_presleep)
#endif

static const isx012_reg_t g_isx012_def_init[] =
{
#ifdef ISX012_NOT_USE_NSTBY
  {PLL_CKSEL,         0x00, 0x01},
  {SRCCK_DIV,         0x00, 0x01},
#endif
  {DRIVABILITY,       0xaa, 0x01},
  {VIFCONFIG,       0x0200, 0x02},
  {YUVCONFIG_TN,    0xff0a, 0x02},
  {ILCODELEN,         0x00, 0x01},
  {AFMODE_MONI,       0x01, 0x01},
  {YUVCONFIG,       0xff6a, 0x02},
  {VIF_REC601_Y,    0x10fe, 0x02},
  {VIF_REC601_C,    0x10f0, 0x02},
  {HSENS_MODE_SEL,    0x11, 0x01},
  {VIF_CLKCONFIG1,    0x30, 0x01},
  {VIF_CLKCONFIG2,    0x30, 0x01},
  {VIF_CLKCONFIG3,    0x30, 0x01},
  {VIF_CLKCONFIG4,    0x30, 0x01},
  {VIF_CLKCONFIG5,    0x30, 0x01},
  {VIF_CLKCONFIG6,    0x30, 0x01},
  {VIF_CLKCONFIG7,    0x30, 0x01},
  {VIF_CLKCONFIG8,    0x30, 0x01},
  {VIF_CLKCONFIG9,    0x30, 0x01},
  {VIF_CLKCONFIG10,   0x30, 0x01},
  {VIF_CLKCONFIG11,   0x30, 0x01},
  {VIF_CLKCONFIG12,   0x30, 0x01},
  {VIF_CLKCONFIG13,   0x11, 0x01},
  {VIF_CLKCONFIG14,   0x11, 0x01},
  {VIF_CLKCONFIG15,   0x11, 0x01},
  {VIF_CLKCONFIG16,   0x11, 0x01},
#ifdef ISX012_NOT_USE_NSTBY
  {INCK_SET,          0x17, 0x01}, /* INCK_SET */
#endif
  {FRM_FIX_SN1_2,     0xff, 0x01}, /* Fix framerate */
  {FAST_MODECHG_EN,   0x01, 0x01},
  {FAST_SHT_MODE_SEL, 0x01, 0x01},
  {CAP_HALF_AE_CTRL,  0x07, 0x01}, /* HAFREL=HIGHSPEED, CAP=Auto  */
  {HALF_AWB_CTRL,     0x01, 0x01},
  {AESPEED_FAST,      0x0f, 0x01},
  {FASTMOVE_TIMEOUT,  0x2d, 0x01},
  {YGAMMA_MODE,       0x01, 0x01},
  {INT_QLTY2,         0x50, 0x01},
  {JPEG_PRED_MODE,    0x00, 0x01},
};

#define ISX012_RESET_NENTRIES ARRAY_NENTRIES(g_isx012_def_init)

static const uint8_t g_isx012_cxc_rgb_data[CXC_RGB_DATA_UNIT_NUM]
                                          [CXC_RGB_DATA_UNIT_SIZE] =
{
  {0x01, 0x43, 0xc0, 0xf0, 0x4f, 0xfc, 0x13},  /* CXC_RGB_UNIT0  */
  {0x80, 0x44, 0x20, 0x21, 0x48, 0x04, 0x0e},  /* CXC_RGB_UNIT1  */
  {0x81, 0x43, 0xc0, 0x10, 0x30, 0xfc, 0x13},  /* CXC_RGB_UNIT2  */
  {0xff, 0x04, 0x20, 0x11, 0x48, 0x08, 0x12},  /* CXC_RGB_UNIT3  */
  {0x81, 0x43, 0xe0, 0x20, 0x48, 0x08, 0x12},  /* CXC_RGB_UNIT4  */
  {0x80, 0x03, 0xe0, 0x00, 0x38, 0x04, 0x10},  /* CXC_RGB_UNIT5  */
  {0x01, 0x84, 0x20, 0x21, 0x48, 0x04, 0x10},  /* CXC_RGB_UNIT6  */
  {0x01, 0x04, 0xc0, 0x10, 0x20, 0x00, 0x08},  /* CXC_RGB_UNIT7  */
  {0x81, 0x82, 0xc0, 0x20, 0x38, 0x08, 0x0e},  /* CXC_RGB_UNIT8  */
  {0x01, 0x43, 0xc0, 0x10, 0x20, 0x04, 0x04},  /* CXC_RGB_UNIT9  */
  {0x01, 0x41, 0x40, 0x10, 0x20, 0x08, 0x0a},  /* CXC_RGB_UNIT10 */
  {0x82, 0x82, 0x80, 0x20, 0x20, 0x04, 0x04},  /* CXC_RGB_UNIT11 */
  {0x82, 0x80, 0x20, 0x20, 0x08, 0x04, 0x06},  /* CXC_RGB_UNIT12 */
  {0x81, 0x42, 0xa0, 0x10, 0x20, 0x04, 0x08},  /* CXC_RGB_UNIT13 */
  {0x81, 0x80, 0x00, 0x00, 0x00, 0x04, 0x00},  /* CXC_RGB_UNIT14 */
  {0x01, 0x41, 0x80, 0x10, 0x20, 0x00, 0x08},  /* CXC_RGB_UNIT15 */
  {0x00, 0x42, 0x20, 0x20, 0x08, 0x08, 0x00},  /* CXC_RGB_UNIT16 */
  {0x82, 0xc0, 0x40, 0x20, 0x20, 0x08, 0x08},  /* CXC_RGB_UNIT17 */
  {0x80, 0x02, 0xa0, 0x10, 0x20, 0x08, 0x04},  /* CXC_RGB_UNIT18 */
  {0x02, 0x81, 0x60, 0x30, 0x20, 0x08, 0x0a},  /* CXC_RGB_UNIT19 */
  {0x82, 0x42, 0xc0, 0x10, 0x30, 0x04, 0x0a},  /* CXC_RGB_UNIT20 */
  {0x03, 0xc3, 0xa0, 0x40, 0x28, 0x0c, 0x0a},  /* CXC_RGB_UNIT21 */
  {0x03, 0xc3, 0xc0, 0x20, 0x20, 0x08, 0x08},  /* CXC_RGB_UNIT22 */
  {0x82, 0xc2, 0xc0, 0x30, 0x40, 0x10, 0x0e},  /* CXC_RGB_UNIT23 */
  {0x84, 0x03, 0xa1, 0x40, 0x28, 0x08, 0x08},  /* CXC_RGB_UNIT24 */
  {0x02, 0x82, 0xa0, 0x30, 0x30, 0x0c, 0x10},  /* CXC_RGB_UNIT25 */
  {0x84, 0x03, 0xe1, 0x40, 0x28, 0x10, 0x0a},  /* CXC_RGB_UNIT26 */
};

static const uint8_t g_isx012_cxc_grb_data[CXC_GRB_DATA_UNIT_NUM]
                                          [CXC_GRB_DATA_UNIT_SIZE] =
{
  {0x00, 0x3d, 0x40, 0x0f, 0xc0, 0x03, 0xf2},  /* CXC_GRB_UNIT0  */
  {0x80, 0x7c, 0x80, 0x1f, 0xd8, 0x03, 0xf0},  /* CXC_GRB_UNIT1  */
  {0x00, 0x3c, 0x40, 0x0f, 0xd0, 0x03, 0xf0},  /* CXC_GRB_UNIT2  */
  {0x80, 0x3c, 0x20, 0x1f, 0xe0, 0x07, 0xf6},  /* CXC_GRB_UNIT3  */
  {0x00, 0x3c, 0x00, 0x1f, 0xd0, 0x07, 0xf4},  /* CXC_GRB_UNIT4  */
  {0x00, 0x3d, 0x40, 0x0f, 0xc8, 0x03, 0xf2},  /* CXC_GRB_UNIT5  */
  {0x80, 0xfc, 0x5f, 0xff, 0xd7, 0x07, 0xf4},  /* CXC_GRB_UNIT6  */
  {0x01, 0x7d, 0x40, 0x0f, 0xd0, 0xff, 0xf3},  /* CXC_GRB_UNIT7  */
  {0x7f, 0xfd, 0x3f, 0x0f, 0xc8, 0x03, 0xf2},  /* CXC_GRB_UNIT8  */
  {0x81, 0x7c, 0x20, 0x0f, 0xd0, 0xff, 0xf7},  /* CXC_GRB_UNIT9  */
  {0x7e, 0xfe, 0x5f, 0x0f, 0xd8, 0x03, 0xf6},  /* CXC_GRB_UNIT10 */
  {0x80, 0xbd, 0xa0, 0x2f, 0xe8, 0x07, 0xfa},  /* CXC_GRB_UNIT11 */
  {0x80, 0xfe, 0xbf, 0x0f, 0xe8, 0xff, 0xf9},  /* CXC_GRB_UNIT12 */
  {0x00, 0x3e, 0x80, 0x3f, 0xe8, 0x0f, 0xfa},  /* CXC_GRB_UNIT13 */
  {0x02, 0x40, 0xe0, 0x0f, 0xf8, 0x03, 0xfe},  /* CXC_GRB_UNIT14 */
  {0x80, 0x7f, 0xe0, 0x1f, 0xf8, 0x17, 0xfe},  /* CXC_GRB_UNIT15 */
  {0x85, 0xff, 0xe0, 0x2f, 0x08, 0x04, 0x04},  /* CXC_GRB_UNIT16 */
  {0x81, 0x40, 0x20, 0x20, 0x00, 0x08, 0x00},  /* CXC_GRB_UNIT17 */
  {0x84, 0x00, 0x21, 0x30, 0x10, 0x0c, 0x06},  /* CXC_GRB_UNIT18 */
  {0x02, 0x82, 0x40, 0x20, 0x10, 0x0c, 0x02},  /* CXC_GRB_UNIT19 */
  {0x83, 0x00, 0x21, 0x40, 0x08, 0x10, 0x06},  /* CXC_GRB_UNIT20 */
  {0x83, 0x82, 0xa0, 0x20, 0x20, 0x08, 0x08},  /* CXC_GRB_UNIT21 */
  {0x02, 0x81, 0x40, 0x30, 0x18, 0x0c, 0x06},  /* CXC_GRB_UNIT22 */
  {0x03, 0x81, 0x80, 0x10, 0x20, 0x04, 0x08},  /* CXC_GRB_UNIT23 */
  {0x82, 0x82, 0x80, 0x20, 0x20, 0x0c, 0x06},  /* CXC_GRB_UNIT24 */
  {0x83, 0xc1, 0x40, 0x20, 0x20, 0x04, 0x08},  /* CXC_GRB_UNIT25 */
  {0x01, 0x82, 0xa0, 0x20, 0x20, 0x08, 0x08},  /* CXC_GRB_UNIT26 */
};

static const uint8_t g_isx012_shd_rgb_data[SHD_RGB_DATA_UNIT_NUM]
                                          [SHD_RGB_DATA_UNIT_SIZE] =
{
  {0xf1, 0x59, 0x52, 0x7b, 0x98, 0xc4, 0x9d, 0x23, 0x29, 0x87, 0x46}, /* SHD_RGB_UNIT0  */
  {0xc6, 0x81, 0xd1, 0x70, 0x56, 0xe4, 0x9c, 0x1b, 0x6d, 0x07, 0x48}, /* SHD_RGB_UNIT1  */
  {0xdd, 0xf1, 0x51, 0x7d, 0xa8, 0xb4, 0x1e, 0x25, 0x49, 0xc7, 0x46}, /* SHD_RGB_UNIT2  */
  {0xbd, 0xf1, 0x50, 0x6d, 0x2a, 0x44, 0x1b, 0x0a, 0x01, 0x87, 0x44}, /* SHD_RGB_UNIT3  */
  {0xd0, 0xa9, 0x51, 0x77, 0x84, 0xd4, 0x9d, 0x1f, 0x2d, 0xc7, 0x44}, /* SHD_RGB_UNIT4  */
  {0xa8, 0xa9, 0xcf, 0x62, 0x98, 0xa3, 0x17, 0xdb, 0xfc, 0x05, 0x38}, /* SHD_RGB_UNIT5  */
  {0x90, 0xe1, 0x8e, 0x6a, 0x08, 0xc4, 0x9b, 0x0e, 0x11, 0x07, 0x43}, /* SHD_RGB_UNIT6  */
  {0xac, 0xa9, 0x4f, 0x5d, 0x4e, 0x13, 0x15, 0xb9, 0xf8, 0x44, 0x2b}, /* SHD_RGB_UNIT7  */
  {0x44, 0x21, 0xcb, 0x56, 0x0e, 0x63, 0x98, 0xe3, 0x78, 0x86, 0x3d}, /* SHD_RGB_UNIT8  */
  {0xab, 0x81, 0x4f, 0x62, 0x7c, 0xc3, 0x94, 0xb4, 0x98, 0x84, 0x26}, /* SHD_RGB_UNIT9  */
  {0x14, 0xe9, 0x48, 0x46, 0x4a, 0x12, 0x93, 0xa4, 0x84, 0xc5, 0x31}, /* SHD_RGB_UNIT10 */
  {0x81, 0xe9, 0x4d, 0x67, 0xac, 0x73, 0x17, 0xd0, 0xdc, 0x24, 0x29}, /* SHD_RGB_UNIT11 */
  {0x12, 0xb9, 0x08, 0x40, 0x02, 0x52, 0x10, 0x84, 0x6c, 0x64, 0x25}, /* SHD_RGB_UNIT12 */
  {0x4c, 0x91, 0xcb, 0x5b, 0x4c, 0xe3, 0x19, 0xec, 0xdc, 0x05, 0x34}, /* SHD_RGB_UNIT13 */
  {0x37, 0x39, 0x8a, 0x44, 0x2a, 0x02, 0x10, 0x80, 0x14, 0xe4, 0x20}, /* SHD_RGB_UNIT14 */
  {0x1c, 0x51, 0x49, 0x53, 0xe4, 0x02, 0x17, 0xd3, 0xb8, 0xe6, 0x3d}, /* SHD_RGB_UNIT15 */
  {0x8b, 0xd9, 0x8d, 0x53, 0xc8, 0x72, 0x12, 0x98, 0x50, 0x24, 0x23}, /* SHD_RGB_UNIT16 */
  {0x19, 0x11, 0x89, 0x4c, 0x8c, 0x32, 0x16, 0xc7, 0x14, 0x06, 0x38}, /* SHD_RGB_UNIT17 */
  {0xca, 0xc1, 0x10, 0x6c, 0xe0, 0x83, 0x97, 0xd0, 0x4c, 0xa5, 0x2d}, /* SHD_RGB_UNIT18 */
  {0x3e, 0x99, 0x0a, 0x51, 0xbc, 0xc2, 0x15, 0xc2, 0x28, 0x26, 0x39}, /* SHD_RGB_UNIT19 */
  {0xa5, 0x89, 0x0f, 0x7b, 0x8c, 0x64, 0x9d, 0x14, 0xb9, 0x46, 0x3e}, /* SHD_RGB_UNIT20 */
  {0x8f, 0x41, 0xce, 0x5e, 0x5e, 0x03, 0x98, 0xdc, 0x50, 0xe6, 0x3a}, /* SHD_RGB_UNIT21 */
  {0xb4, 0x49, 0x90, 0x72, 0x50, 0x74, 0xa1, 0x3a, 0x05, 0x88, 0x4b}, /* SHD_RGB_UNIT22 */
  {0xe1, 0xd1, 0x91, 0x71, 0x38, 0xc4, 0x1b, 0x0a, 0xed, 0x86, 0x42}, /* SHD_RGB_UNIT23 */
  {0xcb, 0x49, 0xd1, 0x78, 0x86, 0x74, 0x9f, 0x2d, 0xb9, 0x88, 0x51}, /* SHD_RGB_UNIT24 */
  {0x11, 0x62, 0x93, 0x7c, 0x9c, 0x94, 0x1d, 0x1b, 0x41, 0x67, 0x46}, /* SHD_RGB_UNIT25 */
  {0xcf, 0x81, 0x91, 0x77, 0x82, 0x54, 0x9f, 0x2a, 0x21, 0xa8, 0x4d}, /* SHD_RGB_UNIT26 */
};

static const uint8_t g_isx012_shd_grb_data[SHD_GRB_DATA_UNIT_NUM]
                                          [SHD_GRB_DATA_UNIT_SIZE] =
{
  {0xe8, 0xa9, 0x0f, 0x78, 0xe4, 0x13, 0x9d, 0xf0, 0x04, 0xe7, 0x39}, /* SHD_GRB_UNIT0  */
  {0xbd, 0x51, 0x0e, 0x6f, 0x94, 0x63, 0x1c, 0xea, 0x4c, 0x27, 0x3c}, /* SHD_GRB_UNIT1  */
  {0xd7, 0x19, 0x4f, 0x7a, 0xf4, 0xd3, 0x1d, 0xf7, 0x20, 0xe7, 0x3a}, /* SHD_GRB_UNIT2  */
  {0xb6, 0x11, 0x0e, 0x6c, 0x76, 0x03, 0x9b, 0xdd, 0xf0, 0x06, 0x39}, /* SHD_GRB_UNIT3  */
  {0xc9, 0xc1, 0x8e, 0x75, 0xc8, 0xe3, 0x9c, 0xef, 0xf8, 0xa6, 0x39}, /* SHD_GRB_UNIT4  */
  {0xa0, 0x69, 0x0d, 0x62, 0x20, 0x93, 0x97, 0xbf, 0xf4, 0xa5, 0x30}, /* SHD_GRB_UNIT5  */
  {0x8c, 0xb1, 0x8c, 0x68, 0x60, 0x13, 0x9b, 0xe0, 0xcc, 0xa6, 0x38}, /* SHD_GRB_UNIT6  */
  {0x9f, 0x71, 0x0d, 0x5c, 0xf4, 0x12, 0x15, 0xab, 0x00, 0x65, 0x28}, /* SHD_GRB_UNIT7  */
  {0x44, 0x41, 0x0a, 0x56, 0xbc, 0x02, 0x98, 0xc4, 0x50, 0x26, 0x34}, /* SHD_GRB_UNIT8  */
  {0x9a, 0x59, 0x4d, 0x5f, 0x16, 0x83, 0x14, 0xa8, 0x9c, 0x64, 0x25}, /* SHD_GRB_UNIT9  */
  {0x15, 0xc1, 0xc8, 0x46, 0x38, 0x22, 0x13, 0x9a, 0x74, 0x65, 0x2c}, /* SHD_GRB_UNIT10 */
  {0x78, 0x11, 0x4c, 0x63, 0x36, 0xb3, 0x96, 0xbb, 0xcc, 0x44, 0x27}, /* SHD_GRB_UNIT11 */
  {0x11, 0xa1, 0x48, 0x40, 0x04, 0x72, 0x10, 0x83, 0x70, 0x84, 0x23}, /* SHD_GRB_UNIT12 */
  {0x4a, 0x69, 0xca, 0x59, 0xe0, 0xf2, 0x98, 0xcc, 0xb0, 0xa5, 0x2e}, /* SHD_GRB_UNIT13 */
  {0x33, 0xc1, 0x09, 0x44, 0x24, 0x02, 0x10, 0x80, 0x14, 0x84, 0x20}, /* SHD_GRB_UNIT14 */
  {0x1b, 0xd1, 0x48, 0x52, 0x98, 0x72, 0x96, 0xb7, 0x8c, 0x06, 0x35}, /* SHD_GRB_UNIT15 */
  {0x81, 0x39, 0xcc, 0x51, 0x96, 0x32, 0x92, 0x92, 0x48, 0x44, 0x22}, /* SHD_GRB_UNIT16 */
  {0x17, 0xb9, 0x48, 0x4b, 0x5e, 0xa2, 0x15, 0xb0, 0xd8, 0x45, 0x30}, /* SHD_GRB_UNIT17 */
  {0xc0, 0x19, 0xce, 0x69, 0x56, 0x23, 0x97, 0xba, 0x38, 0x05, 0x2a}, /* SHD_GRB_UNIT18 */
  {0x3b, 0xe1, 0x09, 0x50, 0x82, 0x42, 0x95, 0xac, 0xf8, 0x05, 0x31}, /* SHD_GRB_UNIT19 */
  {0x94, 0x19, 0x4d, 0x78, 0xca, 0xe3, 0x9c, 0xe8, 0xa8, 0xa6, 0x35}, /* SHD_GRB_UNIT20 */
  {0x8b, 0x71, 0xcc, 0x5d, 0xf8, 0xa2, 0x97, 0xc0, 0x24, 0xa6, 0x32}, /* SHD_GRB_UNIT21 */
  {0xa4, 0xb1, 0x8d, 0x6d, 0x96, 0xd3, 0xa0, 0x09, 0xe1, 0xa7, 0x3f}, /* SHD_GRB_UNIT22 */
  {0xde, 0x09, 0xcf, 0x70, 0x92, 0x73, 0x9b, 0xe0, 0xcc, 0x06, 0x38}, /* SHD_GRB_UNIT23 */
  {0xc0, 0x89, 0x4e, 0x74, 0xcc, 0x13, 0x1e, 0xfc, 0x84, 0x48, 0x45}, /* SHD_GRB_UNIT24 */
  {0x06, 0x7a, 0xd0, 0x7a, 0xe6, 0x33, 0x1d, 0xef, 0x24, 0x07, 0x3b}, /* SHD_GRB_UNIT25 */
  {0xc4, 0xb1, 0x0e, 0x74, 0xca, 0x33, 0x1e, 0xfc, 0xc4, 0x07, 0x41}, /* SHD_GRB_UNIT26 */
};

static const uint8_t g_isx012_shd_r1_data[SHD_R1_DATA_UNIT_NUM]
                                         [SHD_R1_DATA_UNIT_SIZE] =
{
  {0x10, 0x92, 0x10, 0x82, 0xf8, 0x43, 0x1f, 0xfb, 0xf0, 0xe7, 0x40}, /* SHD_R1_UNIT0   */
  {0x07, 0x92, 0xd0, 0x82, 0xec, 0x33, 0x9e, 0xed, 0x68, 0xe7, 0x3c}, /* SHD_R1_UNIT1   */
  {0xfa, 0x21, 0xd0, 0x7e, 0xce, 0xa3, 0x1b, 0xcd, 0x20, 0xe6, 0x31}, /* SHD_R1_UNIT2   */
  {0xa6, 0x69, 0xce, 0x78, 0xbc, 0xa3, 0x1b, 0xbe, 0x44, 0x25, 0x28}, /* SHD_R1_UNIT3   */
  {0x45, 0x19, 0xcb, 0x65, 0x78, 0xe3, 0x1b, 0xc8, 0x3c, 0xa5, 0x24}, /* SHD_R1_UNIT4   */
  {0x15, 0xc1, 0x48, 0x4d, 0xd6, 0x72, 0x99, 0xd3, 0xdc, 0x25, 0x27}, /* SHD_R1_UNIT5   */
  {0x11, 0x01, 0x08, 0x41, 0x42, 0x42, 0x15, 0xc1, 0xa4, 0x06, 0x2f}, /* SHD_R1_UNIT6   */
  {0x39, 0x89, 0x08, 0x40, 0x0a, 0x22, 0x12, 0xab, 0x0c, 0x26, 0x38}, /* SHD_R1_UNIT7   */
  {0x91, 0x71, 0x4a, 0x49, 0x2c, 0xa2, 0x11, 0x9c, 0xc4, 0xa5, 0x33}, /* SHD_R1_UNIT8   */
  {0xe2, 0xe1, 0x4d, 0x5f, 0xa4, 0x22, 0x94, 0xa3, 0xa0, 0x05, 0x34}, /* SHD_R1_UNIT9   */
  {0xc7, 0x41, 0x50, 0x7c, 0x7e, 0xd3, 0x19, 0xc5, 0x48, 0x86, 0x35}, /* SHD_R1_UNIT10  */
  {0xda, 0xa9, 0xcf, 0x8c, 0x42, 0x24, 0x20, 0xf5, 0x8c, 0x67, 0x3c}, /* SHD_R1_UNIT11  */
  {0xf6, 0x89, 0xd0, 0x88, 0x90, 0x34, 0x23, 0x0b, 0x15, 0xa8, 0x3f}, /* SHD_R1_UNIT12  */
  {0x00, 0x72, 0x10, 0x89, 0x68, 0x04, 0x69, 0x00, 0x00, 0x19, 0x26}, /* SHD_R1_UNIT13  */
};

static const uint8_t g_isx012_shd_r2_data[SHD_R2_DATA_UNIT_NUM]
                                         [SHD_R2_DATA_UNIT_SIZE] =
{
  {0x3a, 0xe2, 0x11, 0x8c, 0x42, 0x74, 0xa1, 0x0c, 0x89, 0x08, 0x46}, /* SHD_R2_UNIT0   */
  {0x30, 0xe2, 0xd1, 0x8c, 0x36, 0x54, 0x20, 0xfe, 0xec, 0x47, 0x41}, /* SHD_R2_UNIT1   */
  {0x20, 0x5a, 0x91, 0x88, 0x16, 0x94, 0x1d, 0xda, 0x80, 0x26, 0x35}, /* SHD_R2_UNIT2   */
  {0xc2, 0x69, 0x0f, 0x81, 0x00, 0x94, 0x9d, 0xc9, 0x84, 0xe5, 0x29}, /* SHD_R2_UNIT3   */
  {0x54, 0xb1, 0x0b, 0x6c, 0xb2, 0xb3, 0x9d, 0xd4, 0x74, 0x85, 0x25}, /* SHD_R2_UNIT4   */
  {0x1a, 0xf1, 0x08, 0x50, 0xfc, 0xe2, 0x1a, 0xe0, 0x2c, 0x66, 0x28}, /* SHD_R2_UNIT5   */
  {0x14, 0x01, 0x88, 0x41, 0x4e, 0x32, 0x16, 0xcb, 0x08, 0x87, 0x31}, /* SHD_R2_UNIT6   */
  {0x42, 0x99, 0x08, 0x40, 0x0c, 0x72, 0x92, 0xb1, 0x58, 0x86, 0x3b}, /* SHD_R2_UNIT7   */
  {0xa8, 0xd9, 0xca, 0x4a, 0x32, 0xe2, 0x91, 0xa0, 0x04, 0x66, 0x36}, /* SHD_R2_UNIT8   */
  {0x02, 0xc2, 0x4e, 0x64, 0xbe, 0xd2, 0x94, 0xa9, 0xe0, 0xc5, 0x36}, /* SHD_R2_UNIT9   */
  {0xe1, 0x61, 0x91, 0x84, 0xb6, 0x43, 0x9b, 0xcf, 0x9c, 0x66, 0x38}, /* SHD_R2_UNIT10  */
  {0xf6, 0xa1, 0x50, 0x97, 0x8e, 0x34, 0x22, 0x04, 0x01, 0x08, 0x40}, /* SHD_R2_UNIT11  */
  {0x15, 0x9a, 0x51, 0x92, 0xf2, 0xd4, 0xa5, 0x1d, 0x99, 0xa8, 0x43}, /* SHD_R2_UNIT12  */
  {0x21, 0x82, 0x91, 0x92, 0xbe, 0xf4, 0x9e, 0xf3, 0x4c, 0x87, 0x38}, /* SHD_R2_UNIT13  */
};

static const uint8_t g_isx012_shd_b2_data[SHD_B2_DATA_UNIT_NUM]
                                         [SHD_B2_DATA_UNIT_SIZE] =
{
  {0xef, 0x39, 0xcf, 0x74, 0x88, 0xb3, 0x1b, 0xdf, 0x20, 0x47, 0x3b}, /* SHD_B2_UNIT0   */
  {0xdf, 0x59, 0xcf, 0x77, 0x8c, 0x43, 0x1b, 0xd7, 0xb8, 0x46, 0x37}, /* SHD_B2_UNIT1   */
  {0xcc, 0xc1, 0x0e, 0x73, 0x78, 0xa3, 0x99, 0xc1, 0xd0, 0x25, 0x2f}, /* SHD_B2_UNIT2   */
  {0x87, 0x09, 0x0d, 0x6c, 0x64, 0x93, 0x99, 0xb6, 0x30, 0xc5, 0x27}, /* SHD_B2_UNIT3   */
  {0x3f, 0xb1, 0x0a, 0x5f, 0x2a, 0x93, 0x99, 0xbc, 0x1c, 0x85, 0x24}, /* SHD_B2_UNIT4   */
  {0x16, 0xc9, 0x48, 0x4c, 0xb6, 0x92, 0x17, 0xc4, 0x94, 0x85, 0x26}, /* SHD_B2_UNIT5   */
  {0x10, 0x09, 0x88, 0x41, 0x3a, 0x52, 0x94, 0xb2, 0x2c, 0xc6, 0x2c}, /* SHD_B2_UNIT6   */
  {0x33, 0x79, 0x08, 0x40, 0x08, 0xc2, 0x11, 0xa2, 0x94, 0x65, 0x34}, /* SHD_B2_UNIT7   */
  {0x7e, 0x39, 0x4a, 0x48, 0x26, 0x52, 0x91, 0x96, 0x64, 0x05, 0x2f}, /* SHD_B2_UNIT8   */
  {0xbf, 0x09, 0x8d, 0x5b, 0x92, 0xa2, 0x93, 0x9d, 0x4c, 0x65, 0x2f}, /* SHD_B2_UNIT9   */
  {0x95, 0xf9, 0x0e, 0x73, 0x48, 0x63, 0x98, 0xb9, 0xd8, 0xa5, 0x30}, /* SHD_B2_UNIT10  */
  {0xa5, 0xb1, 0x8d, 0x83, 0xf4, 0xa3, 0x1d, 0xe0, 0xd0, 0x06, 0x36}, /* SHD_B2_UNIT11  */
  {0xbe, 0xa9, 0x4e, 0x79, 0x50, 0xd4, 0x20, 0xf6, 0x54, 0xa7, 0x38}, /* SHD_B2_UNIT12  */
  {0xc5, 0x91, 0xce, 0x7a, 0xf4, 0x03, 0x44, 0x00, 0x60, 0x60, 0x00}, /* SHD_B2_UNIT13  */
};

static const isx012_reg_t g_isx012_shd_thresholds[] =
{
  {SHD_INP_TH_HB_H_R2, 0x1478, 2},
  {SHD_INP_TH_HB_L_R2, 0x1380, 2},
  {SHD_INP_TH_LB_H_R2, 0x10cc, 2},
  {SHD_INP_TH_LB_L_R2, 0x1004, 2},
  {SHD_INP_TH_HB_H_RB, 0x10cc, 2},
  {SHD_INP_TH_HB_L_RB, 0x1004, 2},
  {SHD_INP_TH_LB_H_RB, 0x0000, 2},
  {SHD_INP_TH_LB_L_RB, 0x0000, 2},
};

#define ISX012_SHD_THRESHOLDS_NENTRIES ARRAY_NENTRIES(g_isx012_shd_thresholds)

static const isx012_reg_t g_isx012_shd_wb[] =
{
  {NORMR,              0x1101, 2},
  {NORMB,              0x0f7b, 2},
  {AWBPRER,            0x0147, 2},
  {AWBPREB,            0x022a, 2},
  {SHD_PRER_OFFSET_R2, 0x001b, 2},
  {SHD_PRER_OFFSET_RB, 0x000b, 2},
  {SHD_PREB_OFFSET_RB, 0x0003, 2},
};

#define ISX012_SHD_WB_NENTRIES ARRAY_NENTRIES(g_isx012_shd_wb)

static int32_t g_isx012_colorfx_actual[] =
{
  IMGSENSOR_COLORFX_NONE,
  IMGSENSOR_COLORFX_BW,
  IMGSENSOR_COLORFX_SEPIA,
  IMGSENSOR_COLORFX_NEGATIVE,
  IMGSENSOR_COLORFX_SKETCH,
  IMGSENSOR_COLORFX_SOLARIZATION,
  IMGSENSOR_COLORFX_PASTEL
};

static uint8_t g_isx012_colorfx_regval[] =
{
  REGVAL_EFFECT_NONE,
  REGVAL_EFFECT_MONOTONE,
  REGVAL_EFFECT_SEPIA,
  REGVAL_EFFECT_NEGPOS,
  REGVAL_EFFECT_SKETCH,
  REGVAL_EFFECT_SOLARIZATION,
  REGVAL_EFFECT_PASTEL
};

static int32_t g_isx012_presetwb_actual[] =
{
  IMGSENSOR_WHITE_BALANCE_AUTO,
  IMGSENSOR_WHITE_BALANCE_INCANDESCENT,
  IMGSENSOR_WHITE_BALANCE_FLUORESCENT,
  IMGSENSOR_WHITE_BALANCE_DAYLIGHT,
  IMGSENSOR_WHITE_BALANCE_CLOUDY,
  IMGSENSOR_WHITE_BALANCE_SHADE
};

static uint8_t g_isx012_presetwb_regval[] =
{
  REGVAL_AWB_ATM,
  REGVAL_AWB_LIGHTBULB,
  REGVAL_AWB_FLUORESCENTLIGHT,
  REGVAL_AWB_CLEARWEATHER,
  REGVAL_AWB_CLOUDYWEATHER,
  REGVAL_AWB_SHADE
};

static int32_t g_isx012_photometry_actual[] =
{
  IMGSENSOR_EXPOSURE_METERING_AVERAGE,
  IMGSENSOR_EXPOSURE_METERING_CENTER_WEIGHTED,
  IMGSENSOR_EXPOSURE_METERING_SPOT,
  IMGSENSOR_EXPOSURE_METERING_MATRIX
};

static uint8_t g_isx012_photometry_regval[] =
{
  REGVAL_PHOTOMETRY_AVERAGE,
  REGVAL_PHOTOMETRY_CENTERWEIGHT,
  REGVAL_PHOTOMETRY_SPOT,
  REGVAL_PHOTOMETRY_MULTIPATTERN
};

static int32_t g_isx012_iso_actual[] =
{
  25 * 1000,
  32 * 1000,
  40 * 1000,
  50 * 1000,
  64 * 1000,
  80 * 1000,
  100 * 1000,
  125 * 1000,
  160 * 1000,
  200 * 1000,
  250 * 1000,
  320 * 1000,
  400 * 1000,
  500 * 1000,
  640 * 1000,
  800 * 1000,
  1000 * 1000,
  1250 * 1000,
  1600 * 1000
};

static uint8_t g_isx012_iso_regval[] =
{
  REGVAL_ISO_25,
  REGVAL_ISO_32,
  REGVAL_ISO_40,
  REGVAL_ISO_50,
  REGVAL_ISO_64,
  REGVAL_ISO_80,
  REGVAL_ISO_100,
  REGVAL_ISO_125,
  REGVAL_ISO_160,
  REGVAL_ISO_200,
  REGVAL_ISO_250,
  REGVAL_ISO_320,
  REGVAL_ISO_400,
  REGVAL_ISO_500,
  REGVAL_ISO_640,
  REGVAL_ISO_800,
  REGVAL_ISO_1000,
  REGVAL_ISO_1250,
  REGVAL_ISO_1600
};

static struct imgsensor_ops_s g_isx012_ops =
{
  isx012_is_available,                  /* is HW available */
  isx012_init,                          /* init */
  isx012_uninit,                        /* uninit */
  isx012_get_driver_name,               /* get driver name */
  isx012_validate_frame_setting,        /* validate_frame_setting */
  isx012_start_capture,                 /* start_capture */
  isx012_stop_capture,                  /* stop_capture */
  NULL,                                 /* get_frame_interval */
  isx012_get_supported_value,           /* get_supported_value */
  isx012_get_value,                     /* get_value */
  isx012_set_value                      /* set_value */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t isx012_getreg(FAR isx012_dev_t *priv,
                              uint16_t regaddr, uint16_t regsize)
{
  struct i2c_config_s config;
  volatile uint16_t regval = 0;
  volatile uint8_t buffer[2];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->i2c_freq;
  config.address   = priv->i2c_addr;
  config.addrlen   = 7;
  buffer[0] = regaddr >> 8;
  buffer[1] = regaddr & 0xff;

  nxmutex_lock(&g_isx012_private.i2c_lock);

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, (FAR uint8_t *)buffer, 2);
  if (ret < 0)
    {
      verr("i2c_write failed: %d\n", ret);
    }
  else
    {
      /* Restart and read 16bits from the register */

      ret = i2c_read(priv->i2c, &config, (FAR uint8_t *)buffer, regsize);
      if (ret < 0)
        {
          verr("i2c_read failed: %d\n", ret);
        }
    }

  nxmutex_unlock(&g_isx012_private.i2c_lock);

  if (ret >= 0)
    {
      memcpy((FAR uint8_t *)&regval, (FAR uint8_t *)buffer, regsize);
    }

  return regval;
}

static int isx012_putreg(isx012_dev_t *priv,
                         uint16_t regaddr, uint16_t regval, uint16_t regsize)
{
  struct i2c_config_s config;
  volatile uint8_t buffer[4];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->i2c_freq;
  config.address   = priv->i2c_addr;
  config.addrlen   = 7;

  /* Set up for the transfer */

  buffer[0] = regaddr >> 8;   /* RegAddr Hi */
  buffer[1] = regaddr & 0xff; /* RegAddr Low */

  memcpy((FAR uint8_t *)&buffer[2], (FAR uint8_t *)&regval, regsize);

  nxmutex_lock(&g_isx012_private.i2c_lock);

  /* And do it */

  ret = i2c_write(priv->i2c, &config,
                 (FAR uint8_t *)buffer, regsize + 2);
  if (ret < 0)
    {
      verr("i2c_write failed: %d\n", ret);
    }

  nxmutex_unlock(&g_isx012_private.i2c_lock);
  return ret;
}

static int isx012_putreglist(isx012_dev_t *priv,
                             FAR const isx012_reg_t *reglist,
                             size_t nentries)
{
  FAR const isx012_reg_t *entry;
  int ret = OK;

  for (entry = reglist; nentries > 0; nentries--, entry++)
    {
      ret = isx012_putreg(priv, entry->regaddr,
                          entry->regval, entry->regsize);
      if (ret < 0)
        {
          verr("isx012_putreg failed: %d\n", ret);
          return ret;
        }
    }

  return ret;
}

static int isx012_chk_int_state(isx012_dev_t *priv,
                                uint8_t sts, uint32_t delay_time,
                                uint32_t wait_time, uint32_t timeout)
{
  int ret = 0;
  volatile uint8_t data;
  uint32_t time = 0;

  nxsig_usleep(delay_time * 1000);
  while (time < timeout)
    {
      data = isx012_getreg(priv, INTSTS0, sizeof(data));
      data = data & sts;
      if (data != 0)
        {
          ret = isx012_putreg(priv, INTCLR0, data, sizeof(data));
          return ret;
        }

      nxsig_usleep(wait_time * 1000);
      time += wait_time;
    }

  return ERROR;
}

static int isx012_replace_fmt_to_regval(uint8_t nr_fmt,
                                        FAR imgsensor_format_t *fmt)
{
  int ret;

  if (fmt == NULL)
    {
      return -EINVAL;
    }

  switch (fmt[IMGSENSOR_FMT_MAIN].pixelformat)
    {
      case IMGSENSOR_PIX_FMT_UYVY:
        ret = REGVAL_OUTFMT_YUV;
        break;

      case IMGSENSOR_PIX_FMT_RGB565:
        ret = REGVAL_OUTFMT_RGB;
        break;

      case IMGSENSOR_PIX_FMT_JPEG:
        ret = REGVAL_OUTFMT_JPEG;
        break;

      case IMGSENSOR_PIX_FMT_JPEG_WITH_SUBIMG:
        if (nr_fmt == 1)
          {
            ret = REGVAL_OUTFMT_JPEG;
          }
        else
          {
            ret = REGVAL_OUTFMT_INTERLEAVE;
          }

        break;

      default:  /* Unsupported format */

        ret = -EINVAL;
    }

  return ret;
}

static bool is_movie_needed(uint8_t fmt, uint8_t fps)
{
  bool need = true;

  if ((fmt == IMGSENSOR_PIX_FMT_UYVY) ||
      (fmt == IMGSENSOR_PIX_FMT_RGB565))
    {
      if (fps >= REGVAL_FPSTYPE_30FPS)   /* This means fps <= 30 */
        {
          need = false;
        }
    }

  return need;
}

static void resize_for_clip(uint8_t nr_fmt,
                            FAR imgsensor_format_t *fmt,
                            FAR isx012_rect_t *clip,
                            FAR uint16_t *w,
                            FAR uint16_t *h,
                            FAR uint16_t *s_w,
                            FAR uint16_t *s_h)
{
  ASSERT(fmt && clip && w && h && s_w && s_h);

  *w = (clip->width  == 0) ? fmt[IMGSENSOR_FMT_MAIN].width  : clip->width;
  *h = (clip->height == 0) ? fmt[IMGSENSOR_FMT_MAIN].height : clip->height;

  if (nr_fmt > 1)
    {
      *s_w = fmt[IMGSENSOR_FMT_SUB].width;
      if (clip->width > 0)
        {
          *s_w = (uint32_t)*s_w * clip->width /
                 fmt[IMGSENSOR_FMT_MAIN].width;
        }

      *s_h = fmt[IMGSENSOR_FMT_SUB].height;
      if (clip->height > 0)
        {
          *s_h = (uint32_t)*s_h * clip->height /
                 fmt[IMGSENSOR_FMT_MAIN].height;
        }
    }
}

static void calc_clip_regval(uint16_t pos,
                             uint16_t clip_sz,
                             uint16_t frm_sz,
                             uint16_t basis_sz,
                             FAR uint32_t *ratio,
                             FAR int32_t  *offset)
{
  DEBUGASSERT(ratio && offset);

  *ratio = ZOOM_UNIT;
  *offset = 0;

  if (clip_sz != 0)
    {
      /* Clip by setting zoom up. */

      *ratio *= frm_sz;
      *ratio /= clip_sz;

      /* Applications' request pos means position from the upper left corner.
       * On the other hand, ISX012's register means the center of the image,
       * which has the maximum size of the sensor.
       */

      *offset = CLIP_OFFSET_UNIT;
      *offset *= (int16_t)(pos + (clip_sz / 2) - (frm_sz / 2));
      *offset *= basis_sz;
      *offset /= (int32_t)frm_sz;
    }
}

static bool is_clipped(FAR isx012_rect_t *clip)
{
  DEBUGASSERT(clip);

  if ((clip->left   == 0) &&
      (clip->top    == 0) &&
      (clip->width  == 0) &&
      (clip->height == 0))
    {
      return false;
    }

  return true;
}

static void activate_clip(FAR isx012_dev_t *priv,
                          uint16_t w,
                          uint16_t h,
                          FAR isx012_rect_t *clip)
{
  uint8_t  hvfree = 0;
  uint32_t r_x    = ZOOM_UNIT;
  uint32_t r_y    = ZOOM_UNIT;
  int32_t  x      = 0;
  int32_t  y      = 0;

  DEBUGASSERT(priv && clip);

  if (is_clipped(clip))
    {
      hvfree = 1;

      calc_clip_regval
        (clip->left, clip->width,  w, BASE_HSIZE_FOR_CLIP_OFFSET, &r_x, &x);
      calc_clip_regval
        (clip->top,  clip->height, h, BASE_VSIZE_FOR_CLIP_OFFSET, &r_y, &y);

      if (w * 3 > h * 4)
        {
          /* In case that aspect ratio is longer horizontally than 4:3,
           * re-scaling vertical component setting.
           */

          r_y = RESCALE_FOR_CLIP(r_y, w * 3, h * 4);
          y   = RESCALE_FOR_CLIP(y,   h * 4, w * 3);
        }
      else if (w * 3 < h * 4)
        {
          /* In case that aspect ratio is longer vertically than 4:3,
           * re-scaling horizontal component setting.
           */

          r_x = RESCALE_FOR_CLIP(r_x, h * 4, w * 3);
          x   = RESCALE_FOR_CLIP(x,   w * 3, h * 4);
        }
    }

  isx012_putreg(priv, HVFREEZOOM, hvfree, 1);
  isx012_putreg(priv, EZOOM_MAG,  ZOOM_UNIT,     sizeof(uint16_t));
  isx012_putreg(priv, EZOOM_HMAG, (uint16_t)r_x, sizeof(uint16_t));
  isx012_putreg(priv, EZOOM_VMAG, (uint16_t)r_y, sizeof(uint16_t));
  isx012_putreg(priv, OFFSET_X, (int16_t)x, sizeof(int16_t));
  isx012_putreg(priv, OFFSET_Y, (int16_t)y, sizeof(int16_t));
}

static int isx012_set_mode_param(FAR isx012_dev_t *priv,
                                 imgsensor_stream_type_t type,
                                 uint8_t nr_fmt,
                                 FAR imgsensor_format_t *fmt,
                                 FAR imgsensor_interval_t *interval)
{
  int ret = 0;
  int fmt_val = isx012_replace_fmt_to_regval(nr_fmt, fmt);
  int fps_val = isx012_replace_frameinterval_to_regval(interval);
  uint16_t fps_addr;
  uint16_t fmt_addr;
  uint16_t smode_addr;
  uint16_t hsize_addr;
  uint16_t vsize_addr;
  uint8_t smode;
  uint8_t mode;
  FAR isx012_rect_t *clip;
  uint16_t w = 0;
  uint16_t h = 0;
  uint16_t s_w = 0;
  uint16_t s_h = 0;

  /* Get register address for type  */

  if (type == IMGSENSOR_STREAM_TYPE_VIDEO)
    {
      if (is_movie_needed(fmt_val, fps_val))
        {
          if (priv->mode == REGVAL_MODESEL_HREL)
            {
              /* In Half release state,
               * the setting which need movie mode is prohibited.
               */

              return -EPERM;
            }

          fps_addr   = FPSTYPE_MOVIE;
          fmt_addr   = OUTFMT_MOVIE;
          smode_addr = SENSMODE_MOVIE;
          hsize_addr = HSIZE_MOVIE;
          vsize_addr = VSIZE_MOVIE;
          mode       = REGVAL_MODESEL_MOV;
        }
      else
        {
          fps_addr   = FPSTYPE_MONI;
          fmt_addr   = OUTFMT_MONI;
          smode_addr = SENSMODE_MONI;
          hsize_addr = HSIZE_MONI;
          vsize_addr = VSIZE_MONI;
          mode       = REGVAL_MODESEL_MON;
        }

      clip = &priv->clip_video;
    }
  else
    {
      fps_addr   = FPSTYPE_CAP;
      fmt_addr   = OUTFMT_CAP;
      smode_addr = SENSMODE_CAP;
      hsize_addr = HSIZE_CAP;
      vsize_addr = VSIZE_CAP;
      mode       = REGVAL_MODESEL_CAP;

      clip = &priv->clip_still;
    }

  ret = isx012_putreg(priv, fps_addr, fps_val, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, fmt_addr, fmt_val, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  switch (fps_val)
    {
      case REGVAL_FPSTYPE_120FPS:
        smode = REGVAL_SENSMODE_1_8;
        break;

      case REGVAL_FPSTYPE_60FPS:
        smode = REGVAL_SENSMODE_1_4;
        break;

      case REGVAL_FPSTYPE_30FPS:
        smode = REGVAL_SENSMODE_1_2;
        break;

      default:
        smode = REGVAL_SENSMODE_ALLPIX;
        break;
    }

  ret = isx012_putreg(priv, smode_addr, smode, sizeof(uint8_t));
  if (ret < 0)
    {
      return ret;
    }

  resize_for_clip(nr_fmt, fmt, clip, &w, &h, &s_w, &s_h);
  activate_clip(priv,
                fmt[IMGSENSOR_FMT_MAIN].width,
                fmt[IMGSENSOR_FMT_MAIN].height,
                clip);

  ret = isx012_putreg(priv, hsize_addr, w, sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = isx012_putreg(priv, vsize_addr, h, sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  if (fmt_val == REGVAL_OUTFMT_INTERLEAVE)
    {
      ret = isx012_putreg(priv, HSIZE_TN, s_w, sizeof(uint16_t));
      if (ret < 0)
        {
          return ret;
        }

      ret = isx012_putreg(priv, VSIZE_TN, s_h, sizeof(uint16_t));
      if (ret < 0)
        {
          return ret;
        }
    }

  if (priv->state != STATE_ISX012_ACTIVE)
    {
      isx012_change_device_state(priv, STATE_ISX012_ACTIVE);
    }

  ret = isx012_change_camera_mode(priv, mode);
  if (ret == OK)
    {
      priv->mode = mode;
    }

  return ret;
}

/****************************************************************************
 * isx012_change_camera_mode
 ****************************************************************************/

static int isx012_change_camera_mode(isx012_dev_t *priv, uint8_t mode)
{
  int      ret = 0;
  uint16_t fmt_addr;
  uint8_t  fmt;
  uint32_t vifmode;
#ifdef ISX012_FRAME_SKIP_EN
  uint8_t mask_num;
  int i;
#endif /* ISX012_FRAME_SKIP_EN */

  if (priv->state != STATE_ISX012_ACTIVE)
    {
      return -EPERM;
    }

  switch (mode)
    {
      case REGVAL_MODESEL_MON:
      case REGVAL_MODESEL_HREL:
        fmt_addr = OUTFMT_MONI;
        break;

      case REGVAL_MODESEL_MOV:
        fmt_addr = OUTFMT_MOVIE;
        break;

      case REGVAL_MODESEL_CAP:
        fmt_addr = OUTFMT_CAP;
        break;

      default:
        return -EPERM;
    }

  fmt = isx012_getreg(priv, fmt_addr, 1);

  switch (fmt) /* mode parallel */
    {
      case REGVAL_OUTFMT_YUV:
        vifmode = REGVAL_VIFMODE_YUV_PARALLEL;
        break;
      case REGVAL_OUTFMT_JPEG:
        vifmode = REGVAL_VIFMODE_JPEG_PARALLEL;
        break;
      case REGVAL_OUTFMT_INTERLEAVE:
        vifmode = REGVAL_VIFMODE_INTERLEAVE_PARALLEL;
        break;
      case REGVAL_OUTFMT_RGB:
        vifmode = REGVAL_VIFMODE_RGB_PARALLEL;
        break;
      default:
        vifmode = REGVAL_VIFMODE_YUV_PARALLEL;
        break;
    }

  ret = isx012_putreg(priv, VIFMODE, vifmode, sizeof(vifmode));
  if (ret < 0)
    {
      return ret;
    }

  if (mode != isx012_getreg(priv, MODESEL, sizeof(mode)))
    {
      isx012_putreg(priv, INTCLR0, CM_CHANGED_STS, 1);

      ret = isx012_putreg(priv, MODESEL, mode, sizeof(mode));
      if (ret < 0)
        {
          return ret;
        }

      /* Wait CM_CHANGED */

      ret = isx012_chk_int_state(priv,
                                 CM_CHANGED_STS,
                                 CAMERA_MODE_DELAY_TIME,
                                 CAMERA_MODE_WAIT_TIME,
                                 CAMERA_MODE_TIMEOUT);
      if (ret != 0)
        {
          return ret;
        }
    }

#ifdef ISX012_FRAME_SKIP_EN
  if (mode != REGVAL_MODESEL_HREL)
    {
      isx012_putreg(priv, INTCLR0, VINT_STS, 1);
      mask_num = isx012_getreg(priv, RO_MASK_NUM, sizeof(mask_num));
      for (i = 0; i < mask_num; i++)
        {
          /* Wait Next VINT */

          ret = isx012_chk_int_state(priv, VINT_STS, VINT_DELAY_TIME,
                                           VINT_WAIT_TIME, VINT_TIMEOUT);
          if (ret != 0)
            {
              return ret;
            }
        }
    }
#endif /* ISX012_FRAME_SKIP_EN */

  return OK;
}

/****************************************************************************
 * isx012_change_device_state
 ****************************************************************************/

static int isx012_change_device_state(isx012_dev_t *priv,
                                      isx012_state_t state)
{
  int ret = 0;
#ifdef ISX012_FRAME_SKIP_EN
  int i;
  uint8_t mute_cnt;
#endif /* ISX012_FRAME_SKIP_EN */

  if (priv->state == STATE_ISX012_PRESLEEP || priv->state == state)
    {
      return -EPERM;
    }

  switch (state)
    {
      case STATE_ISX012_SLEEP:
        isx012_putreg(priv, INTCLR0, OM_CHANGED_STS, 1);
        board_isx012_set_sleep(1);
        break;
      case STATE_ISX012_ACTIVE:
        isx012_putreg(priv, INTCLR0, OM_CHANGED_STS | CM_CHANGED_STS, 1);
        board_isx012_release_sleep();
        break;
      case STATE_ISX012_PRESLEEP:
        return -EPERM;
      default:
        return -EPERM;
    }

  /* Wait OM_CHANGED */

  ret = isx012_chk_int_state(priv, OM_CHANGED_STS,
                                   DEVICE_STATE_DELAY_TIME,
                                   DEVICE_STATE_WAIT_TIME,
                                   DEVICE_STATE_TIMEOUT);
  if (ret != 0)
    {
      return ret;
    }

  priv->state = state;

  if (state == STATE_ISX012_ACTIVE)
    {
      /* Wait CM_CHANGED -> Monitoring */

      ret = isx012_chk_int_state(priv, CM_CHANGED_STS,
                                       CAMERA_MODE_DELAY_TIME,
                                       CAMERA_MODE_WAIT_TIME,
                                       CAMERA_MODE_TIMEOUT);
      if (ret != 0)
        {
          return ret;
        }

#ifdef ISX012_FRAME_SKIP_EN
      mute_cnt = isx012_getreg(priv, MUTECNT, sizeof(mute_cnt));
      isx012_putreg(priv, INTCLR0, VINT_STS, 1);
      for (i = 0; i < mute_cnt; i++)
        {
          /* Wait Next VINT */

          ret = isx012_chk_int_state(priv, VINT_STS, VINT_DELAY_TIME,
                                           VINT_WAIT_TIME, VINT_TIMEOUT);
          if (ret != 0)
            {
              return ret;
            }
        }
#endif  /* ISX012_FRAME_SKIP_EN */
    }

  priv->mode = REGVAL_MODESEL_MON;

  return OK;
}

int init_isx012(FAR struct isx012_dev_s *priv)
{
  int ret;

#ifdef ISX012_NOT_USE_NSTBY
  board_isx012_release_sleep();
  board_isx012_release_reset();
  nxsig_usleep(6000);
#else
  board_isx012_release_reset();
  nxsig_usleep(6000);
#endif

#ifdef ISX012_CHECK_IN_DETAIL
  /* check the chip id */

  ret = isx012_chipid(priv);
  if (ret < 0)
    {
      verr("isx012_chipid failed: %d\n", ret);
      board_isx012_set_reset();
      return ret;
    }
#endif

  /* Wait OM_CHANGED Power OFF -> PreSleep */

  ret = isx012_chk_int_state(priv, OM_CHANGED_STS, DEVICE_STATE_DELAY_TIME,
                             DEVICE_STATE_WAIT_TIME, DEVICE_STATE_TIMEOUT);
  if (ret != OK)
    {
      verr("OM_CHANGED_STS(PreSleep) is Not occurred: %d\n", ret);
      return ret;
    }

  priv->state = STATE_ISX012_PRESLEEP;

#ifndef ISX012_NOT_USE_NSTBY
  /* set the isx012 clock */

  /* Write INCK_SET register ISX012 change state PreSleep -> Sleep */

  ret = isx012_putreglist(priv, g_isx012_presleep, ISX012_PRESLEEP_NENTRIES);
  if (ret != OK)
    {
      verr("isx012_putreglist(INCK_SET) failed: %d\n", ret);
      return ret;
    }

  /* Wait OM_CHANGED PreSleep -> Sleep */

  ret = isx012_chk_int_state(priv, OM_CHANGED_STS, DEVICE_STATE_DELAY_TIME,
                             DEVICE_STATE_WAIT_TIME, DEVICE_STATE_TIMEOUT);
  if (ret != OK)
    {
      verr("OM_CHANGED_STS(Sleep) is Not occurred: %d\n", ret);
      return ret;
    }
#endif

  priv->state = STATE_ISX012_SLEEP;
  priv->i2c_freq = I2CFREQ_FAST;

  /* initialize the isx012 hardware */

  ret = isx012_putreglist(priv, g_isx012_def_init, ISX012_RESET_NENTRIES);
  if (ret < 0)
    {
      verr("isx012_putreglist failed: %d\n", ret);
      board_isx012_set_reset();
      return ret;
    }

  /* Set shading adjustment */

  ret = isx012_set_shd(priv);
  if (ret < 0)
    {
      verr("isx012_set_shd failed: %d\n", ret);
      board_isx012_set_reset();
      return ret;
    }

  return ret;
}

static bool isx012_is_available(void)
{
  bool ret;

  isx012_init();

  /* Try to access via I2C.
   * Select DEVICESTS register, which has positive value.
   */

  ret = (isx012_getreg(&g_isx012_private, DEVICESTS, 1) == DEVICESTS_SLEEP);

  isx012_uninit();

  return ret;
}

static int isx012_init(void)
{
  FAR struct isx012_dev_s *priv = &g_isx012_private;
  int ret = 0;

  priv->i2c      = board_isx012_initialize();
  priv->i2c_addr = ISX012_I2C_SLV_ADDR;
  priv->i2c_freq = I2CFREQ_STANDARD;

  ret = board_isx012_power_on();
  if (ret < 0)
    {
      verr("Failed to power on %d\n", ret);
      return ret;
    }

  ret = init_isx012(priv);
  if (ret < 0)
    {
      verr("Failed to init_isx012 %d\n", ret);
      board_isx012_set_reset();
      board_isx012_power_off();
      return ret;
    }

  return ret;
}

static int isx012_uninit(void)
{
  FAR struct isx012_dev_s *priv = &g_isx012_private;

  int ret = 0;

  if (priv->state == STATE_ISX012_ACTIVE)
    {
      board_isx012_set_sleep(1);
    }

  board_isx012_set_reset();

  ret = board_isx012_power_off();
  if (ret < 0)
    {
      verr("Failed to power off %d\n", ret);
      return ret;
    }

  board_isx012_uninitialize(priv->i2c);

  priv->i2c_freq = I2CFREQ_STANDARD;
  priv->state    = STATE_ISX012_POWEROFF;

  return ret;
}

static FAR const char *isx012_get_driver_name(void)
{
  return "ISX012";
}

static int8_t isx012_get_maximum_fps(uint8_t nr_fmt,
                                     FAR imgsensor_format_t *fmt)
{
  int8_t max_fps = REGVAL_FPSTYPE_120FPS;
  uint16_t main_w;
  uint16_t main_h;
  uint16_t sub_w;
  uint16_t sub_h;

  main_w = fmt[IMGSENSOR_FMT_MAIN].width;
  main_h = fmt[IMGSENSOR_FMT_MAIN].height;

  switch (fmt[IMGSENSOR_FMT_MAIN].pixelformat)
    {
      case IMGSENSOR_PIX_FMT_UYVY:                /* YUV 4:2:2 */
      case IMGSENSOR_PIX_FMT_RGB565:              /* RGB565 */

        if ((main_w < OUT_YUV_HSIZE_MIN) ||
            (main_h < OUT_YUV_VSIZE_MIN) ||
            (main_w > OUT_YUV_15FPS_HSIZE_MAX) ||
            (main_h > OUT_YUV_15FPS_VSIZE_MAX))
          {
            /* IN frame size is out of range */

            return -EINVAL;
          }
        else if ((main_w <= OUT_YUV_120FPS_HSIZE_MAX) &&
                 (main_h <= OUT_YUV_120FPS_VSIZE_MAX))
          {
            max_fps = REGVAL_FPSTYPE_120FPS;
          }
        else
          {
            max_fps = REGVAL_FPSTYPE_60FPS;
          }

        break;

      case IMGSENSOR_PIX_FMT_JPEG:                /* JPEG */

        if ((main_w < OUT_JPG_HSIZE_MIN) ||
            (main_h < OUT_JPG_VSIZE_MIN) ||
            (main_w > OUT_JPG_15FPS_HSIZE_MAX) ||
            (main_h > OUT_JPG_15FPS_VSIZE_MAX))
          {
            /* IN frame size is out of range */

            return -EINVAL;
          }
        else if ((main_w <= OUT_JPG_120FPS_HSIZE_MAX) &&
                 (main_h <= OUT_JPG_120FPS_VSIZE_MAX))
          {
            max_fps = REGVAL_FPSTYPE_120FPS;
          }
        else if ((main_w <= OUT_JPG_60FPS_HSIZE_MAX) &&
                 (main_h <= OUT_JPG_60FPS_VSIZE_MAX))
          {
            max_fps = REGVAL_FPSTYPE_60FPS;
          }
        else if ((main_w <= OUT_JPG_30FPS_HSIZE_MAX) &&
                 (main_h <= OUT_JPG_30FPS_VSIZE_MAX))
          {
            max_fps = REGVAL_FPSTYPE_30FPS;
          }
        else
          {
            max_fps = REGVAL_FPSTYPE_15FPS;
          }

        break;

      case IMGSENSOR_PIX_FMT_JPEG_WITH_SUBIMG: /* JPEG + sub image */

        if (nr_fmt == 1)
          {
            sub_w = OUT_YUV_HSIZE_MIN;
            sub_h = OUT_YUV_VSIZE_MIN;
          }
        else
          {
            if (fmt[IMGSENSOR_FMT_SUB].pixelformat
                != IMGSENSOR_PIX_FMT_UYVY)
              {
                /* Unsupported pixel format */

                return -EINVAL;
              }

            sub_w  = fmt[IMGSENSOR_FMT_SUB].width;
            sub_h  = fmt[IMGSENSOR_FMT_SUB].height;
          }

        if ((main_w < OUT_JPG_HSIZE_MIN) ||
            (main_h < OUT_JPG_VSIZE_MIN) ||
            (main_w > OUT_JPGINT_15FPS_HSIZE_MAX) ||
            (main_h > OUT_JPGINT_15FPS_VSIZE_MAX) ||
            (sub_w  < OUT_YUV_HSIZE_MIN) ||
            (sub_h  < OUT_YUV_VSIZE_MIN) ||
            (sub_w  > OUT_YUVINT_30FPS_HSIZE_MAX) ||
            (sub_h  > OUT_YUVINT_30FPS_VSIZE_MAX))
          {
            /* IN frame size is out of range */

            return -EINVAL;
          }
        else if ((main_w <= OUT_JPGINT_30FPS_HSIZE_MAX) &&
                 (main_h <= OUT_JPGINT_30FPS_VSIZE_MAX))
          {
            max_fps = REGVAL_FPSTYPE_30FPS;
          }
        else
          {
            max_fps = REGVAL_FPSTYPE_15FPS;
          }

        break;

      default:
        return -EINVAL;
    }

  return max_fps;
}

static int isx012_replace_frameinterval_to_regval
           (FAR imgsensor_interval_t *interval)
{
  /* Avoid multiplication overflow */

  if ((interval->denominator * 2) / 2 != interval->denominator)
    {
      return -EINVAL;
    }

  /* Avoid division by zero */

  if (interval->numerator == 0)
    {
      return -EINVAL;
    }

  /* Support only 1/x or 2/x. */

  if (((interval->denominator * 2) % interval->numerator) != 0)
    {
      return -EINVAL;
    }

  /* Switch by FPS * 2 */

  switch ((interval->denominator * 2) / interval->numerator)
    {
      case 240 : /* 120FPS */
        return REGVAL_FPSTYPE_120FPS;

      case 120 : /* 60FPS */
        return REGVAL_FPSTYPE_60FPS;

      case 60 :  /* 30FPS */
        return REGVAL_FPSTYPE_30FPS;

      case 30 :  /* 15FPS */
        return REGVAL_FPSTYPE_15FPS;

      case 20 :  /* 10FPS */
        return REGVAL_FPSTYPE_10FPS;

      case 15 :  /* 7.5FPS */
        return REGVAL_FPSTYPE_7_5FPS;

      case 12 :  /* 6FPS */
        return REGVAL_FPSTYPE_6FPS;

      case 10 :  /* 5FPS */
        return REGVAL_FPSTYPE_5FPS;

      default :
        return -EINVAL;
    }
}

static int isx012_validate_frame_setting(imgsensor_stream_type_t type,
                                         uint8_t nr_fmt,
                                         FAR imgsensor_format_t *fmt,
                                         FAR imgsensor_interval_t *interval)
{
  int max_fps;
  int arg_fps;

  if ((fmt == NULL) ||
      (interval == NULL))
    {
      return -EINVAL;
    }

  if ((nr_fmt < 1) || (nr_fmt > 2))
    {
      return -EINVAL;
    }

  max_fps = isx012_get_maximum_fps(nr_fmt, fmt);
  if (max_fps == -EINVAL)
    {
      return -EINVAL;
    }

  arg_fps = isx012_replace_frameinterval_to_regval(interval);
  if (arg_fps == -EINVAL)
    {
      return -EINVAL;
    }

  if (max_fps > arg_fps)
    {
      return -EINVAL;
    }

  return OK;
}

static int isx012_start_capture(imgsensor_stream_type_t type,
                                uint8_t nr_fmt,
                                FAR imgsensor_format_t *fmt,
                                FAR imgsensor_interval_t *interval)
{
  int ret;

  FAR struct isx012_dev_s *priv = &g_isx012_private;

  ret = isx012_validate_frame_setting(type, nr_fmt, fmt, interval);
  if (ret != OK)
    {
      return ret;
    }

  return isx012_set_mode_param(priv, type, nr_fmt, fmt, interval);
}

static int isx012_stop_capture(imgsensor_stream_type_t type)
{
  return OK;
}

static int isx012_get_supported_value
             (uint32_t id, FAR imgsensor_supported_value_t *value)
{
  int ret = OK;
  imgsensor_capability_range_t *range = &value->u.range;
  imgsensor_capability_discrete_t *discrete = &value->u.discrete;
  imgsensor_capability_elems_t *elems = &value->u.elems;

  ASSERT(value);

  switch (id)
    {
      case IMGSENSOR_ID_BRIGHTNESS:
        value->type          = IMGSENSOR_CTRL_TYPE_INTEGER;
        range->minimum       = ISX012_MIN_BRIGHTNESS;
        range->maximum       = ISX012_MAX_BRIGHTNESS;
        range->step          = ISX012_STEP_BRIGHTNESS;
        range->default_value = ISX012_DEF_BRIGHTNESS;

        break;

      case IMGSENSOR_ID_CONTRAST:
        value->type          = IMGSENSOR_CTRL_TYPE_U8FIXEDPOINT_Q7;
        range->minimum       = ISX012_MIN_CONTRAST;
        range->maximum       = ISX012_MAX_CONTRAST;
        range->step          = ISX012_STEP_CONTRAST;
        range->default_value = ISX012_DEF_CONTRAST;

        break;

      case IMGSENSOR_ID_SATURATION:
        value->type          = IMGSENSOR_CTRL_TYPE_INTEGER;
        range->minimum       = ISX012_MIN_SATURATION;
        range->maximum       = ISX012_MAX_SATURATION;
        range->step          = ISX012_STEP_SATURATION;
        range->default_value = ISX012_DEF_SATURATION;

        break;

      case IMGSENSOR_ID_HUE:
        value->type          = IMGSENSOR_CTRL_TYPE_INTEGER;
        range->minimum       = ISX012_MIN_HUE;
        range->maximum       = ISX012_MAX_HUE;
        range->step          = ISX012_STEP_HUE;
        range->default_value = ISX012_DEF_HUE;

        break;

      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
        value->type          = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        range->minimum       = ISX012_MIN_AUTOWB;
        range->maximum       = ISX012_MAX_AUTOWB;
        range->step          = ISX012_STEP_AUTOWB;
        range->default_value = ISX012_DEF_AUTOWB;

        break;
      case IMGSENSOR_ID_GAMMA_CURVE:
        value->type          = IMGSENSOR_CTRL_TYPE_U16;
        elems->minimum       = ISX012_MIN_GAMMACURVE;
        elems->maximum       = ISX012_MAX_GAMMACURVE;
        elems->step          = ISX012_STEP_GAMMACURVE;
        elems->nr_elems      = ISX012_ELEMS_GAMMACURVE;

        break;

      case IMGSENSOR_ID_EXPOSURE:
        value->type          = IMGSENSOR_CTRL_TYPE_INTEGER_TIMES_3;
        range->minimum       = ISX012_MIN_EXPOSURE;
        range->maximum       = ISX012_MAX_EXPOSURE;
        range->step          = ISX012_STEP_EXPOSURE;
        range->default_value = ISX012_DEF_EXPOSURE;

        break;

      case IMGSENSOR_ID_HFLIP_VIDEO:
        value->type          = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        range->minimum       = ISX012_MIN_HFLIP;
        range->maximum       = ISX012_MAX_HFLIP;
        range->step          = ISX012_STEP_HFLIP;
        range->default_value = ISX012_DEF_HFLIP;

        break;

      case IMGSENSOR_ID_VFLIP_VIDEO:
        value->type          = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        range->minimum       = ISX012_MIN_VFLIP;
        range->maximum       = ISX012_MAX_VFLIP;
        range->step          = ISX012_STEP_VFLIP;
        range->default_value = ISX012_DEF_VFLIP;

        break;

      case IMGSENSOR_ID_HFLIP_STILL:
        value->type          = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        range->minimum       = ISX012_MIN_HFLIP_STILL;
        range->maximum       = ISX012_MAX_HFLIP_STILL;
        range->step          = ISX012_STEP_HFLIP_STILL;
        range->default_value = ISX012_DEF_HFLIP_STILL;

        break;

      case IMGSENSOR_ID_VFLIP_STILL:
        value->type          = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        range->minimum       = ISX012_MIN_VFLIP_STILL;
        range->maximum       = ISX012_MAX_VFLIP_STILL;
        range->step          = ISX012_STEP_VFLIP_STILL;
        range->default_value = ISX012_DEF_VFLIP_STILL;

        break;

      case IMGSENSOR_ID_SHARPNESS:
        value->type          = IMGSENSOR_CTRL_TYPE_INTEGER;
        range->minimum       = ISX012_MIN_SHARPNESS;
        range->maximum       = ISX012_MAX_SHARPNESS;
        range->step          = ISX012_STEP_SHARPNESS;
        range->default_value = ISX012_DEF_SHARPNESS;

        break;

      case IMGSENSOR_ID_COLOR_KILLER:
        value->type          = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        range->minimum       = ISX012_MIN_COLORKILLER;
        range->maximum       = ISX012_MAX_COLORKILLER;
        range->step          = ISX012_STEP_COLORKILLER;
        range->default_value = ISX012_DEF_COLORKILLER;

        break;

      case IMGSENSOR_ID_COLORFX:
        value->type = IMGSENSOR_CTRL_TYPE_INTEGER_MENU;
        discrete->nr_values
          = ARRAY_NENTRIES(g_isx012_colorfx_actual);
        discrete->values = g_isx012_colorfx_actual;
        discrete->default_value = IMGSENSOR_COLORFX_NONE;

        break;

      case IMGSENSOR_ID_EXPOSURE_AUTO:
        value->type          = IMGSENSOR_CTRL_TYPE_INTEGER;
        range->minimum       = ISX012_MIN_EXPOSUREAUTO;
        range->maximum       = ISX012_MAX_EXPOSUREAUTO;
        range->step          = ISX012_STEP_EXPOSUREAUTO;
        range->default_value = ISX012_DEF_EXPOSUREAUTO;

        break;

      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        value->type          = IMGSENSOR_CTRL_TYPE_INTEGER;
        range->minimum       = ISX012_MIN_EXPOSURETIME;
        range->maximum       = ISX012_MAX_EXPOSURETIME;
        range->step          = ISX012_STEP_EXPOSURETIME;
        range->default_value = ISX012_DEF_EXPOSURETIME;

        break;

      case IMGSENSOR_ID_EXPOSURE_METERING:
        value->type = IMGSENSOR_CTRL_TYPE_INTEGER_MENU;
        discrete->nr_values
          = ARRAY_NENTRIES(g_isx012_photometry_actual);
        discrete->values = g_isx012_photometry_actual;
        discrete->default_value
          = IMGSENSOR_EXPOSURE_METERING_AVERAGE;

        break;

      case IMGSENSOR_ID_AUTO_N_PRESET_WB:
        value->type = IMGSENSOR_CTRL_TYPE_INTEGER_MENU;
        discrete->nr_values = ARRAY_NENTRIES(g_isx012_presetwb_actual);
        discrete->values = g_isx012_presetwb_actual;
        discrete->default_value = IMGSENSOR_WHITE_BALANCE_AUTO;

        break;

      case IMGSENSOR_ID_WIDE_DYNAMIC_RANGE:
        value->type          = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        range->minimum       = ISX012_MIN_YGAMMA;
        range->maximum       = ISX012_MAX_YGAMMA;
        range->step          = ISX012_STEP_YGAMMA;
        range->default_value = ISX012_DEF_YGAMMA;

        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY:
        value->type             = IMGSENSOR_CTRL_TYPE_INTEGER_MENU;
        discrete->nr_values     = ARRAY_NENTRIES(g_isx012_iso_actual);
        discrete->values        = g_isx012_iso_actual;
        discrete->default_value = 0;

        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY_AUTO:
        value->type          = IMGSENSOR_CTRL_TYPE_INTEGER;
        range->minimum       = ISX012_MIN_ISOAUTO;
        range->maximum       = ISX012_MAX_ISOAUTO;
        range->step          = ISX012_STEP_ISOAUTO;
        range->default_value = ISX012_DEF_ISOAUTO;

        break;

      case IMGSENSOR_ID_3A_LOCK:
        value->type          = IMGSENSOR_CTRL_TYPE_BITMASK;
        range->minimum       = ISX012_MIN_3ALOCK;
        range->maximum       = ISX012_MAX_3ALOCK;
        range->step          = ISX012_STEP_3ALOCK;
        range->default_value = ISX012_DEF_3ALOCK;

        break;

      case IMGSENSOR_ID_3A_PARAMETER:
        value->type          = IMGSENSOR_CTRL_TYPE_U16;
        elems->minimum       = 0;
        elems->maximum       = 65535;
        elems->step          = 1;
        elems->nr_elems      = ISX012_ELEMS_3APARAM;

        break;

      case IMGSENSOR_ID_3A_STATUS:
        value->type          = IMGSENSOR_CTRL_TYPE_INTEGER;
        range->minimum       = 0;
        range->maximum       = 3;
        range->step          = 1;
        range->default_value = 3;

        break;

      case IMGSENSOR_ID_JPEG_QUALITY:
        value->type          = IMGSENSOR_CTRL_TYPE_INTEGER;
        range->minimum       = ISX012_MIN_JPGQUALITY;
        range->maximum       = ISX012_MAX_JPGQUALITY;
        range->step          = ISX012_STEP_JPGQUALITY;
        range->default_value = ISX012_DEF_JPGQUALITY;

        break;

      default: /* Unsupported parameter */
        ret = -EINVAL;

        break;
    }

  return ret;
}

static int isx012_get_value(uint32_t id,
                            uint32_t size,
                            FAR imgsensor_value_t *value)
{
  FAR struct isx012_dev_s *priv = &g_isx012_private;
  uint16_t   readvalue;
  uint8_t    cnt;
  uint8_t    threea_enable;
  uint16_t   read_src;
  uint16_t   *read_dst;
  int        ret = OK;
  uint16_t   exposure_time_lsb;
  uint16_t   exposure_time_msb;

  ASSERT(value);

  switch (id)
    {
      case IMGSENSOR_ID_BRIGHTNESS:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_BRIGHTNESS,
                                  ISX012_SIZE_BRIGHTNESS);

        value->value32 = (int32_t)(int8_t)(0x00ff & readvalue);
        break;

      case IMGSENSOR_ID_CONTRAST:
        value->value32 = isx012_getreg(priv,
                                       ISX012_REG_CONTRAST,
                                       ISX012_SIZE_CONTRAST);
        break;

      case IMGSENSOR_ID_SATURATION:
        value->value32 = isx012_getreg(priv,
                                       ISX012_REG_SATURATION,
                                       ISX012_SIZE_SATURATION);
        break;

      case IMGSENSOR_ID_HUE:
        value->value32 = isx012_getreg(priv,
                                       ISX012_REG_HUE,
                                       ISX012_SIZE_HUE);
        break;

      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_AUTOWB,
                                  ISX012_SIZE_AUTOWB);

        /* Convert to video driver's value */

        value->value32 = (readvalue & REGVAL_CPUEXT_BIT_AWBSTOP) ? 0 : 1;

        break;

      case IMGSENSOR_ID_GAMMA_CURVE:
        if (value->p_u16 == NULL)
          {
            return -EINVAL;
          }

        if (size != ISX012_ELEMS_GAMMACURVE * sizeof(uint16_t))
          {
            return -EINVAL;
          }

        read_src = ISX012_REG_GAMMACURVE;
        read_dst = value->p_u16;

        for (cnt = 0; cnt < ISX012_ELEMS_GAMMACURVE; cnt++)
          {
            *read_dst = isx012_getreg(priv,
                                      read_src,
                                      ISX012_SIZE_GAMMACURVE);
            read_src += ISX012_SIZE_GAMMACURVE;
            read_dst++;
          }

        break;

      case IMGSENSOR_ID_EXPOSURE:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_EXPOSURE,
                                  ISX012_SIZE_EXPOSURE);

        value->value32 = (int32_t)(int8_t)(0x00ff & readvalue);

        break;

      case IMGSENSOR_ID_HFLIP_VIDEO:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_HFLIP,
                                  ISX012_SIZE_HFLIP);

        value->value32 = (readvalue & REGVAL_READVECT_BIT_H) ? 1 : 0;

        break;

      case IMGSENSOR_ID_VFLIP_VIDEO:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_VFLIP,
                                  ISX012_SIZE_VFLIP);

        value->value32 = (readvalue & REGVAL_READVECT_BIT_V) ? 1 : 0;

        break;

      case IMGSENSOR_ID_HFLIP_STILL:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_HFLIP_STILL,
                                  ISX012_SIZE_HFLIP_STILL);

        value->value32 = (readvalue & REGVAL_READVECT_BIT_H) ? 1 : 0;

        break;

      case IMGSENSOR_ID_VFLIP_STILL:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_VFLIP_STILL,
                                  ISX012_SIZE_VFLIP_STILL);

        value->value32 = (readvalue & REGVAL_READVECT_BIT_V) ? 1 : 0;

        break;

      case IMGSENSOR_ID_SHARPNESS:
        value->value32 = isx012_getreg(priv,
                                       ISX012_REG_SHARPNESS,
                                       ISX012_SIZE_SHARPNESS);
        break;

      case IMGSENSOR_ID_COLOR_KILLER:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_COLORKILLER,
                                  ISX012_SIZE_COLORKILLER);

        value->value32 = (readvalue == REGVAL_EFFECT_MONOTONE) ? 1 : 0;

        break;

      case IMGSENSOR_ID_COLORFX:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_COLOREFFECT,
                                  ISX012_SIZE_COLOREFFECT);

        ret = -EINVAL;
        for (cnt = 0; cnt < ARRAY_NENTRIES(g_isx012_colorfx_regval); cnt++)
          {
            if (g_isx012_colorfx_regval[cnt] == readvalue)
              {
                value->value32 = g_isx012_colorfx_actual[cnt];
                ret = OK;
                break;
              }
          }

        break;

      case IMGSENSOR_ID_EXPOSURE_AUTO:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_EXPOSURETIME,
                                  ISX012_SIZE_EXPOSURETIME);

        value->value32 = readvalue ?
                         IMGSENSOR_EXPOSURE_MANUAL : IMGSENSOR_EXPOSURE_AUTO;

        break;

      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        value->value32 = isx012_getreg(priv,
                                       ISX012_REG_EXPOSURETIME,
                                       ISX012_SIZE_EXPOSURETIME);

        if (value->value32 == REGVAL_EXPOSURETIME_AUTO)
          {
            exposure_time_lsb = isx012_getreg
                                (priv,
                                 ISX012_REG_EXPOSUREAUTOVALUE_LSB,
                                 ISX012_SIZE_EXPOSUREAUTOVALUE);
            exposure_time_msb = isx012_getreg
                                (priv,
                                 ISX012_REG_EXPOSUREAUTOVALUE_MSB,
                                 ISX012_SIZE_EXPOSUREAUTOVALUE);

            value->value32 = (uint16_t)(((exposure_time_msb << 16)
                                         | exposure_time_lsb)
                                        / ISX012_UNIT_EXPOSURETIME_US);
          }
        break;

      case IMGSENSOR_ID_AUTO_N_PRESET_WB:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_PRESETWB,
                                  ISX012_SIZE_PRESETWB);

        for (cnt = 0; cnt < ARRAY_NENTRIES(g_isx012_presetwb_regval); cnt++)
          {
            if (g_isx012_presetwb_regval[cnt] == readvalue)
              {
                value->value32 = g_isx012_presetwb_actual[cnt];
                ret = OK;
                break;
              }
          }

        break;

      case IMGSENSOR_ID_WIDE_DYNAMIC_RANGE:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_YGAMMA,
                                  ISX012_SIZE_YGAMMA);
        value->value32 = readvalue ? 0 : 1;

        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_ISOAUTOVALUE,
                                  ISX012_SIZE_ISOAUTOVALUE);

        ret = -EINVAL;
        for (cnt = 0; cnt < ARRAY_NENTRIES(g_isx012_iso_regval); cnt++)
          {
            if (g_isx012_iso_regval[cnt] == readvalue)
              {
                value->value32 = g_isx012_iso_actual[cnt];
                ret = OK;
                break;
              }
          }

        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY_AUTO:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_ISOAUTO,
                                  ISX012_SIZE_ISOAUTO);

        value->value32 = (readvalue == REGVAL_ISO_AUTO) ?
                         IMGSENSOR_ISO_SENSITIVITY_AUTO :
                         IMGSENSOR_ISO_SENSITIVITY_MANUAL;

        break;

      case IMGSENSOR_ID_EXPOSURE_METERING:
        readvalue = isx012_getreg(priv,
                                  ISX012_REG_PHOTOMETRY,
                                  ISX012_SIZE_PHOTOMETRY);

        ret = -EINVAL;
        for (cnt = 0;
             cnt < ARRAY_NENTRIES(g_isx012_photometry_regval);
             cnt++)
          {
            if (g_isx012_photometry_regval[cnt] == readvalue)
              {
                value->value32 = g_isx012_photometry_actual[cnt];
                ret = OK;
                break;
              }
          }

        break;

      case IMGSENSOR_ID_3A_PARAMETER:
        if (value->p_u16 == NULL)
          {
            return -EINVAL;
          }

        if (size != ISX012_ELEMS_3APARAM * sizeof(uint16_t))
          {
            return -EINVAL;
          }

        /* Get AWB parameter */

        value->p_u16[0] = isx012_getreg(priv, RATIO_R, 2);
        value->p_u16[1] = isx012_getreg(priv, RATIO_B, 2);

        /* Get AE parameter */

        value->p_u16[2] = isx012_getreg(priv, AELEVEL, 2);

        break;

      case IMGSENSOR_ID_3A_STATUS:

        /* Initialize returned status */

        value->value32 = IMGSENSOR_3A_STATUS_STABLE;

        /* Get AWB/AE enable or not */

        threea_enable = isx012_getreg(priv, CPUEXT, 1);

        if ((threea_enable & REGVAL_CPUEXT_BIT_AWBSTOP)
               != REGVAL_CPUEXT_BIT_AWBSTOP)
          {
            readvalue = isx012_getreg(priv, AWBSTS, 1);
            if (readvalue != REGVAL_AWBSTS_STOP) /* AWB is not stopped */
              {
                value->value32 |= IMGSENSOR_3A_STATUS_AWB_OPERATING;
              }
          }

        if ((threea_enable & REGVAL_CPUEXT_BIT_AESTOP)
               != REGVAL_CPUEXT_BIT_AESTOP)
          {
            readvalue = isx012_getreg(priv, AESTS, 1);
            if (readvalue != REGVAL_AESTS_STOP) /* AE is not stopped */
              {
                value->value32 |= IMGSENSOR_3A_STATUS_AE_OPERATING;
              }
          }
        break;

      case IMGSENSOR_ID_JPEG_QUALITY:
        value->value32 = isx012_getreg(priv,
                                       ISX012_REG_JPGQUALITY,
                                       ISX012_SIZE_JPGQUALITY);
        break;

      default: /* Unsupported id */

        ret = -EINVAL;
        break;
    }

  return ret;
}

static bool validate_clip_setting(uint32_t sz, FAR uint32_t *clip)
{
  bool ret = false;
  uint32_t w;
  uint32_t h;

  DEBUGASSERT(clip);

  if (sz != IMGSENSOR_CLIP_NELEM * sizeof(uint32_t))
    {
      return ret;
    }

  w = clip[IMGSENSOR_CLIP_INDEX_WIDTH];
  h = clip[IMGSENSOR_CLIP_INDEX_HEIGHT];

  if ((w % CLIP_SIZE_UNIT == 0) && (h % CLIP_SIZE_UNIT == 0))
    {
      ret = true;
    }

  return ret;
}

static int set_clip(uint32_t size,
                    FAR uint32_t *val,
                    FAR isx012_rect_t *target)
{
  DEBUGASSERT(target);

  if (!validate_clip_setting(size, val))
    {
      return -EINVAL;
    }

  target->left   = (int32_t)val[IMGSENSOR_CLIP_INDEX_LEFT];
  target->top    = (int32_t)val[IMGSENSOR_CLIP_INDEX_TOP];
  target->width  = val[IMGSENSOR_CLIP_INDEX_WIDTH];
  target->height = val[IMGSENSOR_CLIP_INDEX_HEIGHT];

  return OK;
}

static int isx012_set_value(uint32_t id,
                            uint32_t size,
                            FAR imgsensor_value_t value)
{
  FAR struct isx012_dev_s *priv = &g_isx012_private;
  int       ret = -EINVAL;
  uint8_t   cnt;
  uint16_t  *write_src;
  uint16_t  write_dst;
  uint16_t  regval;
  uint16_t  exposure_time_lsb;
  uint16_t  exposure_time_msb;

  switch (id)
    {
      case IMGSENSOR_ID_BRIGHTNESS:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_BRIGHTNESS,
                             ISX012_MAX_BRIGHTNESS,
                             ISX012_STEP_BRIGHTNESS);
        if (ret != OK)
          {
            break;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_BRIGHTNESS,
                            value.value32,
                            ISX012_SIZE_BRIGHTNESS);
        break;

      case IMGSENSOR_ID_CONTRAST:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_CONTRAST,
                             ISX012_MAX_CONTRAST,
                             ISX012_STEP_CONTRAST);
        if (ret != OK)
          {
            break;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_CONTRAST,
                            value.value32,
                            ISX012_SIZE_CONTRAST);
        break;

      case IMGSENSOR_ID_SATURATION:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_SATURATION,
                             ISX012_MAX_SATURATION,
                             ISX012_STEP_SATURATION);
        if (ret != OK)
          {
            break;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_SATURATION,
                            value.value32,
                            ISX012_SIZE_SATURATION);
        break;

      case IMGSENSOR_ID_HUE:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_HUE,
                             ISX012_MAX_HUE,
                             ISX012_STEP_HUE);
        if (ret != OK)
          {
            break;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_HUE,
                            value.value32,
                            ISX012_SIZE_HUE);
        break;

      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_AUTOWB,
                             ISX012_MAX_AUTOWB,
                             ISX012_STEP_AUTOWB);
        if (ret != OK)
          {
            break;
          }

        regval = isx012_getreg(priv,
                               ISX012_REG_AUTOWB,
                               ISX012_SIZE_AUTOWB);

        if (value.value32)
          {
            /* Because true means setting auto white balance
             * turn off the stop bit
             */

            regval &= ~REGVAL_CPUEXT_BIT_AWBSTOP;
          }
        else
          {
            /* Because false means stopping auto white balance,
             * turn on the stop bit.
             */

            regval |= REGVAL_CPUEXT_BIT_AWBSTOP;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_AUTOWB,
                            regval,
                            ISX012_SIZE_AUTOWB);
        break;

      case IMGSENSOR_ID_GAMMA_CURVE:
        if (value.p_u16 == NULL)
          {
            return -EINVAL;
          }

        if (size != ISX012_ELEMS_GAMMACURVE * 2)
          {
            return -EINVAL;
          }

        write_src = value.p_u16;
        write_dst = ISX012_REG_GAMMACURVE;

        for (cnt = 0; cnt < ISX012_ELEMS_GAMMACURVE; cnt++)
          {
            ret = VALIDATE_VALUE(*write_src,
                                 ISX012_MIN_GAMMACURVE,
                                 ISX012_MAX_GAMMACURVE,
                                 ISX012_STEP_GAMMACURVE);
            if (ret != OK)
              {
                break;
              }

            ret = isx012_putreg(priv,
                                write_dst,
                                *write_src,
                                ISX012_SIZE_GAMMACURVE);

            write_src++;
            write_dst += ISX012_SIZE_GAMMACURVE;
          }

        break;

      case IMGSENSOR_ID_EXPOSURE:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_EXPOSURE,
                             ISX012_MAX_EXPOSURE,
                             ISX012_STEP_EXPOSURE);
        if (ret != OK)
          {
            break;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_EXPOSURE,
                            value.value32,
                            ISX012_SIZE_EXPOSURE);

        break;

      case IMGSENSOR_ID_HFLIP_VIDEO:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_HFLIP,
                             ISX012_MAX_HFLIP,
                             ISX012_STEP_HFLIP);
        if (ret != OK)
          {
            break;
          }

        regval = isx012_getreg(priv,
                               ISX012_REG_HFLIP,
                               ISX012_SIZE_HFLIP);

        if (value.value32)
          {
            regval |= REGVAL_READVECT_BIT_H;
          }
        else
          {
            regval &= ~REGVAL_READVECT_BIT_H;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_HFLIP,
                            regval,
                            ISX012_SIZE_HFLIP);
        break;

      case IMGSENSOR_ID_VFLIP_VIDEO:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_VFLIP,
                             ISX012_MAX_VFLIP,
                             ISX012_STEP_VFLIP);
        if (ret != OK)
          {
            break;
          }

        regval = isx012_getreg(priv,
                               ISX012_REG_VFLIP,
                               ISX012_SIZE_VFLIP);

        if (value.value32)
          {
            regval |= REGVAL_READVECT_BIT_V;
          }
        else
          {
            regval &= ~REGVAL_READVECT_BIT_V;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_VFLIP,
                            regval,
                            ISX012_SIZE_VFLIP);
        break;

      case IMGSENSOR_ID_HFLIP_STILL:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_HFLIP_STILL,
                             ISX012_MAX_HFLIP_STILL,
                             ISX012_STEP_HFLIP_STILL);
        if (ret != OK)
          {
            break;
          }

        regval = isx012_getreg(priv,
                               ISX012_REG_HFLIP_STILL,
                               ISX012_SIZE_HFLIP_STILL);

        if (value.value32)
          {
            regval |= REGVAL_READVECT_BIT_H;
          }
        else
          {
            regval &= ~REGVAL_READVECT_BIT_H;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_HFLIP_STILL,
                            regval,
                            ISX012_SIZE_HFLIP_STILL);

        break;

      case IMGSENSOR_ID_VFLIP_STILL:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_VFLIP_STILL,
                             ISX012_MAX_VFLIP_STILL,
                             ISX012_STEP_VFLIP_STILL);
        if (ret != OK)
          {
            break;
          }

        regval = isx012_getreg(priv,
                               ISX012_REG_VFLIP_STILL,
                               ISX012_SIZE_VFLIP_STILL);

        if (value.value32)
          {
            regval |= REGVAL_READVECT_BIT_V;
          }
        else
          {
            regval &= ~REGVAL_READVECT_BIT_V;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_VFLIP_STILL,
                            regval,
                            ISX012_SIZE_VFLIP_STILL);
        break;

      case IMGSENSOR_ID_SHARPNESS:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_SHARPNESS,
                             ISX012_MAX_SHARPNESS,
                             ISX012_STEP_SHARPNESS);
        if (ret != OK)
          {
            break;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_SHARPNESS,
                            value.value32,
                            ISX012_SIZE_SHARPNESS);
        break;

      case IMGSENSOR_ID_COLOR_KILLER:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_COLORKILLER,
                             ISX012_MAX_COLORKILLER,
                             ISX012_STEP_COLORKILLER);
        if (ret != OK)
          {
            break;
          }

        ret = isx012_putreg
                (priv,
                 ISX012_REG_COLORKILLER,
                 value.value32 ? REGVAL_EFFECT_MONOTONE : REGVAL_EFFECT_NONE,
                 ISX012_SIZE_COLORKILLER);

        break;

      case IMGSENSOR_ID_COLORFX:
        for (cnt = 0; cnt < ARRAY_NENTRIES(g_isx012_colorfx_actual); cnt++)
          {
            if (g_isx012_colorfx_actual[cnt] == value.value32)
              {
                ret = isx012_putreg(priv,
                                    ISX012_REG_COLOREFFECT,
                                    g_isx012_colorfx_regval[cnt],
                                    ISX012_SIZE_COLOREFFECT);
                break;
              }
          }

        break;

      case IMGSENSOR_ID_EXPOSURE_AUTO:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_EXPOSUREAUTO,
                             ISX012_MAX_EXPOSUREAUTO,
                             ISX012_STEP_EXPOSUREAUTO);
        if (ret != OK)
          {
            break;
          }

        if (value.value32 == IMGSENSOR_EXPOSURE_AUTO)
          {
            /* Register is the same as IMGSENSOR_ID_EXPOSURE_ABSOLUTE.
             * If this register value = REGVAL_EXPOSURETIME_AUTO(=0),
             *  it means auto. Otherwise, it means manual.
             */

            ret = isx012_putreg(priv,
                                ISX012_REG_EXPOSURETIME,
                                REGVAL_EXPOSURETIME_AUTO,
                                ISX012_SIZE_EXPOSURETIME);
          }
        else
          {
            /* In manual case, read current value of register which
             * value adjusted automatically by ISX012 HW is set to.
             * It has 32bits length which is composed of LSB 16bits
             *  and MSB 16bits.
             */

            exposure_time_lsb = isx012_getreg
                                (priv,
                                 ISX012_REG_EXPOSUREAUTOVALUE_LSB,
                                 ISX012_SIZE_EXPOSUREAUTOVALUE);
            exposure_time_msb = isx012_getreg
                                (priv,
                                 ISX012_REG_EXPOSUREAUTOVALUE_MSB,
                                 ISX012_SIZE_EXPOSUREAUTOVALUE);

            /* Register value adjusted automatically by ISX012 HW
             *  has the different unit from manual value register.
             *   automatic value register : 1   microsec unit
             *   manual    value register : 100 microsec unit
             */

            regval = (uint16_t)(((exposure_time_msb << 16)
                                  | exposure_time_lsb)
                                 / ISX012_UNIT_EXPOSURETIME_US);
            ret = isx012_putreg(priv,
                                ISX012_REG_EXPOSURETIME,
                                regval,
                                ISX012_SIZE_EXPOSURETIME);
          }

        break;

      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_EXPOSURETIME,
                             ISX012_MAX_EXPOSURETIME,
                             ISX012_STEP_EXPOSURETIME);
        if (ret != OK)
          {
            break;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_EXPOSURETIME,
                            value.value32,
                            ISX012_SIZE_EXPOSURETIME);
        break;

      case IMGSENSOR_ID_WIDE_DYNAMIC_RANGE:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_YGAMMA,
                             ISX012_MAX_YGAMMA,
                             ISX012_STEP_YGAMMA);
        if (ret != OK)
          {
            break;
          }

        if (value.value32)
          {
            regval = REGVAL_YGAMMA_AUTO;
          }
        else
          {
            regval = REGVAL_YGAMMA_OFF;
          }

        ret = isx012_putreg
                (priv,
                 ISX012_REG_YGAMMA,
                 value.value32 ? REGVAL_YGAMMA_AUTO : REGVAL_YGAMMA_OFF,
                 ISX012_SIZE_YGAMMA);

        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY:
        for (cnt = 0; cnt < ARRAY_NENTRIES(g_isx012_iso_actual); cnt++)
          {
            if (g_isx012_iso_actual[cnt]
                 == value.value32)
              {
                ret = isx012_putreg(priv,
                                    ISX012_REG_ISO,
                                    g_isx012_iso_regval[cnt],
                                    ISX012_SIZE_ISO);
                break;
              }
          }

        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY_AUTO:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_ISOAUTO,
                             ISX012_MAX_ISOAUTO,
                             ISX012_STEP_ISOAUTO);
        if (ret != OK)
          {
            break;
          }

        if (value.value32 == IMGSENSOR_ISO_SENSITIVITY_AUTO)
          {
            ret = isx012_putreg(priv,
                                ISX012_REG_ISOAUTO,
                                REGVAL_ISO_AUTO,
                                ISX012_SIZE_ISOAUTO);
          }
        else
          {
            /* In manual case, read auto adjust value and set it */

            regval = isx012_getreg(priv,
                                   ISX012_REG_ISOAUTOVALUE,
                                   ISX012_SIZE_ISOAUTOVALUE);
            ret = isx012_putreg(priv,
                                ISX012_REG_ISO,
                                regval,
                                ISX012_SIZE_ISO);
          }

        break;

      case IMGSENSOR_ID_EXPOSURE_METERING:
        for (cnt = 0;
             cnt < ARRAY_NENTRIES(g_isx012_photometry_actual);
             cnt++)
          {
            if (g_isx012_photometry_actual[cnt]
                 == value.value32)
              {
                ret = isx012_putreg(priv,
                                    ISX012_REG_PHOTOMETRY,
                                    g_isx012_photometry_regval[cnt],
                                    ISX012_SIZE_PHOTOMETRY);
                break;
              }
          }

        break;

      case IMGSENSOR_ID_AUTO_N_PRESET_WB:
        for (cnt = 0;
             cnt < ARRAY_NENTRIES(g_isx012_presetwb_actual);
             cnt++)
          {
            if (g_isx012_presetwb_actual[cnt] == value.value32)
              {
                ret = isx012_putreg(priv,
                                    ISX012_REG_PRESETWB,
                                    g_isx012_presetwb_regval[cnt],
                                    ISX012_SIZE_PRESETWB);
                break;
              }
          }

        break;

      case IMGSENSOR_ID_3A_LOCK:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_3ALOCK,
                             ISX012_MAX_3ALOCK,
                             ISX012_STEP_3ALOCK);
        if (ret != OK)
          {
            break;
          }

        regval = 0;

        if ((value.value32 & IMGSENSOR_LOCK_EXPOSURE)
              == IMGSENSOR_LOCK_EXPOSURE)
          {
            regval |= REGVAL_CPUEXT_BIT_AESTOP;
          }

        if ((value.value32 & IMGSENSOR_LOCK_WHITE_BALANCE)
              == IMGSENSOR_LOCK_WHITE_BALANCE)
          {
            regval |= REGVAL_CPUEXT_BIT_AWBSTOP;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_3ALOCK,
                            regval,
                            ISX012_SIZE_3ALOCK);

        break;

      case IMGSENSOR_ID_3A_PARAMETER:

        /* AWB parameter : red */

        ret = isx012_putreg(priv, INIT_CONT_INR, value.p_u16[0], 2);
        ret = isx012_putreg(priv, INIT_CONT_OUTR, value.p_u16[0], 2);

        /* AWB parameter : blue */

        ret = isx012_putreg(priv, INIT_CONT_INB, value.p_u16[1], 2);
        ret = isx012_putreg(priv, INIT_CONT_OUTB, value.p_u16[1], 2);

        /* AE parameter */

        ret = isx012_putreg(priv, AE_START_LEVEL, value.p_u16[2], 2);

        break;

      case IMGSENSOR_ID_JPEG_QUALITY:
        ret = VALIDATE_VALUE(value.value32,
                             ISX012_MIN_JPGQUALITY,
                             ISX012_MAX_JPGQUALITY,
                             ISX012_STEP_JPGQUALITY);
        if (ret != OK)
          {
            break;
          }

        ret = isx012_putreg(priv,
                            ISX012_REG_JPGQUALITY,
                            value.value32,
                            ISX012_SIZE_JPGQUALITY);
        break;

      case IMGSENSOR_ID_CLIP_VIDEO:
        ret = set_clip(size, value.p_u32, &priv->clip_video);

        break;

      case IMGSENSOR_ID_CLIP_STILL:
        ret = set_clip(size, value.p_u32, &priv->clip_still);

        break;

      default: /* Unsupported control id */

        break;
    }

  return ret;
}

static int isx012_set_shd(FAR isx012_dev_t *priv)
{
  int ret;
  int unit_cnt;
  int size_cnt;

  /* At first, disable CXC and SHD */

  ret = isx012_putreg(priv, SHD_EN, 0x50, 1);
  if (ret < 0)
    {
      verr("isx012_putreg(disable CXC/SHD) failed: %d\n", ret);
      return ret;
    }

  /* Set CXC Validity */

  ret = isx012_putreg(priv, CXC_VALID, 0x8282, 2);
  if (ret < 0)
    {
      verr("isx012_putreg(CXC_VALID) failed: %d\n", ret);
      return ret;
    }

  /* Set CXC R Gb data */

  for (unit_cnt = 0; unit_cnt < CXC_RGB_DATA_UNIT_NUM; unit_cnt++)
    {
      for (size_cnt = 0; size_cnt < CXC_RGB_DATA_UNIT_SIZE; size_cnt++)
        {
          ret = isx012_putreg(priv,
                              CXC_RGB_UNIT(unit_cnt, size_cnt),
                              g_isx012_cxc_rgb_data[unit_cnt][size_cnt],
                              1);
          if (ret < 0)
            {
              verr("isx012_putreg(CXC R Gb) failed: %d\n", ret);
              return ret;
            }
        }
    }

  /* Set CXC G Rb data */

  for (unit_cnt = 0; unit_cnt < CXC_GRB_DATA_UNIT_NUM; unit_cnt++)
    {
      for (size_cnt = 0; size_cnt < CXC_GRB_DATA_UNIT_SIZE; size_cnt++)
        {
          ret = isx012_putreg(priv,
                              CXC_GRB_UNIT(unit_cnt, size_cnt),
                              g_isx012_cxc_grb_data[unit_cnt][size_cnt],
                              1);
          if (ret < 0)
            {
              verr("isx012_putreg(CXC G Rb) failed: %d\n", ret);
              return ret;
            }
        }
    }

  /* Set SHD Validity */

  ret = isx012_putreg(priv, SHD_VALID, 0x9191, 2);
  if (ret < 0)
    {
      verr("isx012_putreg(SHD_VALID) failed: %d\n", ret);
      return ret;
    }

  /* Set SHD R Gb data */

  for (unit_cnt = 0; unit_cnt < SHD_RGB_DATA_UNIT_NUM; unit_cnt++)
    {
      for (size_cnt = 0; size_cnt < SHD_RGB_DATA_UNIT_SIZE; size_cnt++)
        {
          ret = isx012_putreg(priv,
                              SHD_RGB_UNIT(unit_cnt, size_cnt),
                              g_isx012_shd_rgb_data[unit_cnt][size_cnt],
                              1);
          if (ret < 0)
            {
              verr("isx012_putreg(SHD R Gb) failed: %d\n", ret);
              return ret;
            }
        }
    }

  /* Set SHD G Rb data */

  for (unit_cnt = 0; unit_cnt < SHD_GRB_DATA_UNIT_NUM; unit_cnt++)
    {
      for (size_cnt = 0; size_cnt < SHD_GRB_DATA_UNIT_SIZE; size_cnt++)
        {
          ret = isx012_putreg(priv,
                              SHD_GRB_UNIT(unit_cnt, size_cnt),
                              g_isx012_shd_grb_data[unit_cnt][size_cnt],
                              1);
          if (ret < 0)
            {
              verr("isx012_putreg(SHD G Rb) failed: %d\n", ret);
              return ret;
            }
        }
    }

  /* Set SHD R1 data */

  for (unit_cnt = 0; unit_cnt < SHD_R1_DATA_UNIT_NUM; unit_cnt++)
    {
      for (size_cnt = 0; size_cnt < SHD_R1_DATA_UNIT_SIZE; size_cnt++)
        {
          ret = isx012_putreg(priv,
                              SHD_R1_UNIT(unit_cnt, size_cnt),
                              g_isx012_shd_r1_data[unit_cnt][size_cnt],
                              1);
          if (ret < 0)
            {
              verr("isx012_putreg(SHD R1) failed: %d\n", ret);
              return ret;
            }
        }
    }

  /* Set SHD R2 data */

  for (unit_cnt = 0; unit_cnt < SHD_R2_DATA_UNIT_NUM; unit_cnt++)
    {
      for (size_cnt = 0; size_cnt < SHD_R2_DATA_UNIT_SIZE; size_cnt++)
        {
          ret = isx012_putreg(priv,
                              SHD_R2_UNIT(unit_cnt, size_cnt),
                              g_isx012_shd_r2_data[unit_cnt][size_cnt],
                              1);
          if (ret < 0)
            {
              verr("isx012_putreg(SHD R2) failed: %d\n", ret);
              return ret;
            }
        }
    }

  /* Set SHD B2 data */

  for (unit_cnt = 0; unit_cnt < SHD_B2_DATA_UNIT_NUM; unit_cnt++)
    {
      for (size_cnt = 0; size_cnt < SHD_B2_DATA_UNIT_SIZE; size_cnt++)
        {
          ret = isx012_putreg(priv,
                              SHD_B2_UNIT(unit_cnt, size_cnt),
                              g_isx012_shd_b2_data[unit_cnt][size_cnt],
                              1);
          if (ret < 0)
            {
              verr("isx012_putreg(SHD B2) failed: %d\n", ret);
              return ret;
            }
        }
    }

  /* Set SHD thresholds data */

  ret = isx012_putreglist(priv, g_isx012_shd_thresholds,
                          ISX012_SHD_THRESHOLDS_NENTRIES);
  if (ret < 0)
    {
      verr("isx012_putreglist failed(SHD thresholds): %d\n", ret);
      board_isx012_set_reset();
      return ret;
    }

  /* Set SHD white balance data */

  ret = isx012_putreglist(priv, g_isx012_shd_wb, ISX012_SHD_WB_NENTRIES);
  if (ret < 0)
    {
      verr("isx012_putreglist(SHD white balance) failed: %d\n", ret);
      board_isx012_set_reset();
      return ret;
    }

  /* Enable CXC and SHD */

  ret = isx012_putreg(priv, SHD_EN, 0x57, 1);
  if (ret < 0)
    {
      verr("isx012_putreg(enable CXC/SHD) failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int isx012_initialize(void)
{
  int ret;
  FAR struct isx012_dev_s *priv = &g_isx012_private;

  /* Register image sensor operations variable */

  ret = imgsensor_register(&g_isx012_ops);
  if (ret != OK)
    {
      verr("Failed to register ops to video driver.\n");
      return ret;
    }

  /* Initialize other information */

  priv->state = STATE_ISX012_POWEROFF;
  return OK;
}

int isx012_uninitialize(void)
{
  /* No procedure */

  return OK;
}

#ifdef CONFIG_VIDEO_ISX012_REGDEBUG
int isx012_read_register(uint16_t addr, FAR uint8_t *buf, uint8_t size)
{
  uint16_t buf16;

  if (buf == NULL)
    {
      return -EINVAL;
    }

  if (size > 2)
    {
      return -EINVAL;
    }

  buf16 = isx012_getreg(&g_isx012_private, addr, size);
  memcpy(buf, &buf16, size);
  return OK;
}
#endif
