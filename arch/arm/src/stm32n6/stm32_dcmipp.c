/****************************************************************************
 * arch/arm/src/stm32n6/stm32_dcmipp.c
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

/* DCMIPP capture interface for raw Bayer MIPI CSI-2 sensors.
 *
 * Only pipe1 (the main pipe) is used:
 *
 *   CSI-2 VC0 -> black level -> exposure (white balance gains) ->
 *   demosaicing -> colour correction -> decimation -> downsize ->
 *   gamma -> RGB565 pixel packer -> memory
 *
 * The raw sensor is not registered with V4L2 directly: the DCMIPP
 * registers an image sensor of its own that reports the RGB565 output
 * formats and sizes, forwards the other controls to the raw sensor and
 * implements V4L2_CID_EXPOSURE_AUTO and V4L2_CID_AUTO_WHITE_BALANCE.
 *
 * The black level, colour matrix and initial white balance gains come
 * from the board.  The statistics unit averages R, G and B after the
 * demosaicing over the whole frame.  From these means a
 * worker adjusts the sensor exposure time and gain towards a target
 * luminance (CONFIG_STM32_DCMIPP_AE) and the red and blue gains for a grey
 * world (CONFIG_STM32_DCMIPP_AWB).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <sys/video_controls.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/video/video.h>
#include <nuttx/wqueue.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/imgsensor.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/stm32n6xxx_dcmipp.h"
#include "hardware/stm32n6xxx_rcc.h"
#include "stm32_csi.h"
#include "stm32_dcmipp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum downsize factor; larger reductions use the decimation first */

#define DCMIPP_DSIZE_MAX         8

/* Demosaicing filter strengths: 0, no sensor specific tuning */

#define DCMIPP_DM_PEAK           0
#define DCMIPP_DM_LINEV          0
#define DCMIPP_DM_LINEH          0
#define DCMIPP_DM_EDGE           0

#if defined(CONFIG_STM32_DCMIPP_AE) || defined(CONFIG_STM32_DCMIPP_AWB)
#  define HAVE_DCMIPP_AUTO
#endif

/* Pipe1 interrupts: the frame start (vertical sync), the frame end and
 * the errors.
 */

#define DCMIPP_P1_ERRS           (DCMIPP_CMINT_ATXERR | DCMIPP_CMINT_P1OVR)
#define DCMIPP_P1_INTS           (DCMIPP_P1_ERRS | DCMIPP_CMINT_P1FRAME | \
                                  DCMIPP_CMINT_P1VSYNC)

/* Automatic exposure: dead band around the target (8-bit luminance) and
 * limits of the exposure change in one step (Q8).  The gain table holds
 * 2^(n / 20) in Q8.
 */

#define DCMIPP_AE_TOLERANCE      4
#define DCMIPP_AE_RATIO_MIN      64
#define DCMIPP_AE_RATIO_MAX      1024
#define DCMIPP_AE_POW2_STEPS     20

/* Automatic white balance: gain limits (128 = 1.0), usable green mean
 * (8-bit) and dead band (1/32 of the gain).
 */

#define DCMIPP_AWB_GAIN_MIN      64
#define DCMIPP_AWB_GAIN_MAX      512
#define DCMIPP_AWB_GREEN_MIN     8
#define DCMIPP_AWB_GREEN_MAX     224
#define DCMIPP_AWB_DEADBAND      32

/* Timeout waiting for the end of a capture, in milliseconds */

#define DCMIPP_STOP_TIMEOUT      100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_dcmipp_s
{
  struct imgdata_s data;              /* Must be first */
  struct imgsensor_s isp;             /* Sensor seen by V4L2 */
  struct v4l2_frmsizeenum frmsizes[3];
  FAR struct imgsensor_s *sensor;     /* Raw sensor */
  FAR const struct stm32_dcmipp_config_s *config;
  imgdata_capture_t callback;         /* Frame completion callback */
  FAR void *arg;                      /* Callback argument */
  uintptr_t buf;                      /* Buffer for the next frame or 0 */
  uint32_t framesize;                 /* Bytes in one output frame */
  bool capturing;
  bool armed;                         /* Capture requested into buf */
  bool active;                        /* Frame being written into buf */
  bool frameerr;                      /* Error during the active frame */
#ifdef HAVE_DCMIPP_AUTO
  struct work_s work;                 /* Statistics processing */
  uint32_t stat[3];                   /* R, G, B sums / 256 */
  uint32_t statpixels;                /* Pixels in the statistics window */
  uint16_t gain[3];                   /* White balance gains, 128 = 1.0 */
  volatile uint8_t skip;              /* Frames before the next statistics */
  volatile bool busy;                 /* Statistics being processed */
#endif
#ifdef CONFIG_STM32_DCMIPP_AWB
  volatile bool awbauto;              /* V4L2_CID_AUTO_WHITE_BALANCE */
#endif
#ifdef CONFIG_STM32_DCMIPP_AE
  mutex_t aelock;                     /* AE against manual settings */
  volatile bool aeauto;               /* V4L2_CID_EXPOSURE_AUTO is auto */
  bool ae;                            /* Exposure time control available */
  bool aegain;                        /* Gain control available */
  int32_t expmin;                     /* Exposure time range */
  int32_t expmax;
  int32_t gainmin;                    /* Gain range */
  int32_t gainmax;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int dcmipp_init(FAR struct imgdata_s *data);
static int dcmipp_uninit(FAR struct imgdata_s *data);
static int dcmipp_set_buf(FAR struct imgdata_s *data,
                          uint8_t nr_datafmts,
                          FAR imgdata_format_t *datafmts,
                          FAR uint8_t *addr, uint32_t size);
static int dcmipp_validate_frame_setting(FAR struct imgdata_s *data,
                                         uint8_t nr_datafmts,
                                         FAR imgdata_format_t *datafmts,
                                         FAR imgdata_interval_t *interval);
static int dcmipp_start_capture(FAR struct imgdata_s *data,
                                uint8_t nr_datafmts,
                                FAR imgdata_format_t *datafmts,
                                FAR imgdata_interval_t *interval,
                                FAR imgdata_capture_t callback,
                                FAR void *arg);
static int dcmipp_stop_capture(FAR struct imgdata_s *data);

static bool dcmipp_isp_is_available(FAR struct imgsensor_s *sensor);
static int dcmipp_isp_init(FAR struct imgsensor_s *sensor);
static int dcmipp_isp_uninit(FAR struct imgsensor_s *sensor);
static FAR const char *
dcmipp_isp_get_driver_name(FAR struct imgsensor_s *sensor);
static int dcmipp_isp_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                             imgsensor_stream_type_t type,
                                             uint8_t nr_datafmts,
                                             FAR imgsensor_format_t *fmts,
                                             FAR imgsensor_interval_t *ival);
static int dcmipp_isp_start_capture(FAR struct imgsensor_s *sensor,
                                    imgsensor_stream_type_t type,
                                    uint8_t nr_datafmts,
                                    FAR imgsensor_format_t *fmts,
                                    FAR imgsensor_interval_t *ival);
static int dcmipp_isp_stop_capture(FAR struct imgsensor_s *sensor,
                                   imgsensor_stream_type_t type);
static int dcmipp_isp_get_frame_interval(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         FAR imgsensor_interval_t *ival);
static int
dcmipp_isp_get_supported_value(FAR struct imgsensor_s *sensor, uint32_t id,
                               FAR imgsensor_supported_value_t *val);
static int dcmipp_isp_get_value(FAR struct imgsensor_s *sensor,
                                uint32_t id, uint32_t size,
                                FAR imgsensor_value_t *value);
static int dcmipp_isp_set_value(FAR struct imgsensor_s *sensor,
                                uint32_t id, uint32_t size,
                                imgsensor_value_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imgdata_ops_s g_dcmipp_ops =
{
  .init                   = dcmipp_init,
  .uninit                 = dcmipp_uninit,
  .set_buf                = dcmipp_set_buf,
  .validate_frame_setting = dcmipp_validate_frame_setting,
  .start_capture          = dcmipp_start_capture,
  .stop_capture           = dcmipp_stop_capture,
};

static const struct imgsensor_ops_s g_dcmipp_isp_ops =
{
  .is_available           = dcmipp_isp_is_available,
  .init                   = dcmipp_isp_init,
  .uninit                 = dcmipp_isp_uninit,
  .get_driver_name        = dcmipp_isp_get_driver_name,
  .validate_frame_setting = dcmipp_isp_validate_frame_setting,
  .start_capture          = dcmipp_isp_start_capture,
  .stop_capture           = dcmipp_isp_stop_capture,
  .get_frame_interval     = dcmipp_isp_get_frame_interval,
  .get_supported_value    = dcmipp_isp_get_supported_value,
  .get_value              = dcmipp_isp_get_value,
  .set_value              = dcmipp_isp_set_value,
};

/* Pipe1 output format */

static const struct v4l2_fmtdesc g_dcmipp_fmtdescs[] =
{
  {
    .pixelformat = V4L2_PIX_FMT_RGB565,
    .description = "RGB565",
  },
};

static struct stm32_dcmipp_s g_dcmipp =
{
  .data =
    {
      .ops = &g_dcmipp_ops,
    },
  .isp =
    {
      .ops = &g_dcmipp_isp_ops,
    },
};

#ifdef CONFIG_STM32_DCMIPP_AE
static const uint16_t g_dcmipp_pow2[DCMIPP_AE_POW2_STEPS] =
{
  256, 265, 274, 284, 294, 304, 315, 326, 338, 350,
  362, 375, 388, 402, 416, 431, 446, 461, 478, 495
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dcmipp_gain
 *
 * Description:
 *   Convert a gain (128 = 1.0) to the exposure block shift and multiplier
 *   fields: gain = MULT * 2^SHF / 128 with MULT < 256.
 *
 ****************************************************************************/

static uint32_t dcmipp_gain(uint32_t gain, unsigned int multshift,
                            unsigned int shfshift)
{
  uint32_t shift = 0;

  while (gain >= 256 && shift < 7)
    {
      gain >>= 1;
      shift++;
    }

  return ((gain & 0xff) << multshift) | (shift << shfshift);
}

/****************************************************************************
 * Name: dcmipp_coef
 *
 * Description:
 *   Pack two signed colour conversion coefficients (256 = 1.0).
 *
 ****************************************************************************/

static uint32_t dcmipp_coef(int16_t low, int16_t high)
{
  uint32_t regval;

  regval  = ((uint32_t)low & DCMIPP_P1CC_COEF_MASK) << DCMIPP_P1CC_LOW_SHIFT;
  regval |= ((uint32_t)high & DCMIPP_P1CC_COEF_MASK) <<
            DCMIPP_P1CC_HIGH_SHIFT;
  return regval;
}

/****************************************************************************
 * Name: dcmipp_setwb
 *
 * Description:
 *   Program the white balance gains (128 = 1.0) of the exposure block.
 *
 ****************************************************************************/

static void dcmipp_setwb(FAR const uint16_t *gain)
{
  putreg32(dcmipp_gain(gain[1], DCMIPP_P1EXCR2_MULTG_SHIFT,
                       DCMIPP_P1EXCR2_SHFG_SHIFT) |
           dcmipp_gain(gain[2], DCMIPP_P1EXCR2_MULTB_SHIFT,
                       DCMIPP_P1EXCR2_SHFB_SHIFT),
           STM32_DCMIPP_P1EXCR2);
  putreg32(dcmipp_gain(gain[0], DCMIPP_P1EXCR1_MULTR_SHIFT,
                       DCMIPP_P1EXCR1_SHFR_SHIFT) | DCMIPP_P1EXCR1_ENABLE,
           STM32_DCMIPP_P1EXCR1);
}

/****************************************************************************
 * Name: dcmipp_configisp
 *
 * Description:
 *   Program the static pipe1 image processing.
 *
 ****************************************************************************/

static void dcmipp_configisp(FAR const struct stm32_dcmipp_config_s *cfg)
{
  uint32_t regval;

  /* Black level */

  if (cfg->blacklevel != 0)
    {
      regval = DCMIPP_P1BLCCR_ENABLE |
               ((uint32_t)cfg->blacklevel << DCMIPP_P1BLCCR_BLCR_SHIFT) |
               ((uint32_t)cfg->blacklevel << DCMIPP_P1BLCCR_BLCG_SHIFT) |
               ((uint32_t)cfg->blacklevel << DCMIPP_P1BLCCR_BLCB_SHIFT);
    }
  else
    {
      regval = 0;
    }

  putreg32(regval, STM32_DCMIPP_P1BLCCR);

  /* White balance gains in the exposure block */

  dcmipp_setwb(cfg->gain);

  /* Demosaicing */

  putreg32(DCMIPP_P1DMCR_ENABLE |
           ((uint32_t)cfg->bayer << DCMIPP_P1DMCR_TYPE_SHIFT) |
           (DCMIPP_DM_PEAK << DCMIPP_P1DMCR_PEAK_SHIFT) |
           (DCMIPP_DM_LINEV << DCMIPP_P1DMCR_LINEV_SHIFT) |
           (DCMIPP_DM_LINEH << DCMIPP_P1DMCR_LINEH_SHIFT) |
           ((uint32_t)DCMIPP_DM_EDGE << DCMIPP_P1DMCR_EDGE_SHIFT),
           STM32_DCMIPP_P1DMCR);

  /* Colour correction matrix, no offsets and no clamping */

  putreg32(dcmipp_coef(cfg->ccm[0][0], cfg->ccm[0][1]),
           STM32_DCMIPP_P1CCRR1);
  putreg32(dcmipp_coef(cfg->ccm[0][2], 0), STM32_DCMIPP_P1CCRR2);
  putreg32(dcmipp_coef(cfg->ccm[1][0], cfg->ccm[1][1]),
           STM32_DCMIPP_P1CCGR1);
  putreg32(dcmipp_coef(cfg->ccm[1][2], 0), STM32_DCMIPP_P1CCGR2);
  putreg32(dcmipp_coef(cfg->ccm[2][0], cfg->ccm[2][1]),
           STM32_DCMIPP_P1CCBR1);
  putreg32(dcmipp_coef(cfg->ccm[2][2], 0), STM32_DCMIPP_P1CCBR2);
  putreg32(DCMIPP_P1CCCR_ENABLE, STM32_DCMIPP_P1CCCR);

  /* Gamma */

  putreg32(cfg->gamma ? DCMIPP_P1GMCR_ENABLE : 0, STM32_DCMIPP_P1GMCR);
}

#ifdef HAVE_DCMIPP_AUTO
/****************************************************************************
 * Name: dcmipp_configstats
 *
 * Description:
 *   Average R, G and B after the demosaicing over the whole sensor frame
 *   (average metering).  The 24-bit sums / 256 hold a full 5 MP frame.
 *
 ****************************************************************************/

static void dcmipp_configstats(FAR struct stm32_dcmipp_s *priv)
{
  FAR const struct stm32_dcmipp_config_s *cfg = priv->config;
  uint32_t width  = cfg->width & ~1;
  uint32_t height = cfg->height & ~1;

  putreg32(0, STM32_DCMIPP_P1STSTR);
  putreg32((width << DCMIPP_P1STSZR_HSIZE_SHIFT) |
           (height << DCMIPP_P1STSZR_VSIZE_SHIFT) | DCMIPP_P1STSZR_CROPEN,
           STM32_DCMIPP_P1STSZR);

  putreg32(DCMIPP_P1STCR_ENABLE | DCMIPP_P1STCR_SRC_POSTR |
           DCMIPP_P1STCR_AVER_ALL, STM32_DCMIPP_P1ST1CR);
  putreg32(DCMIPP_P1STCR_ENABLE | DCMIPP_P1STCR_SRC_POSTG |
           DCMIPP_P1STCR_AVER_ALL, STM32_DCMIPP_P1ST2CR);
  putreg32(DCMIPP_P1STCR_ENABLE | DCMIPP_P1STCR_SRC_POSTB |
           DCMIPP_P1STCR_AVER_ALL, STM32_DCMIPP_P1ST3CR);

  priv->statpixels = width * height;
}
#endif

#ifdef CONFIG_STM32_DCMIPP_AE
/****************************************************************************
 * Name: dcmipp_aetotal
 *
 * Description:
 *   Exposure time times the linear sensor gain, Q8.
 *
 ****************************************************************************/

static uint64_t dcmipp_aetotal(FAR struct stm32_dcmipp_s *priv,
                               int32_t exposure, int32_t gain)
{
  uint32_t steps = priv->config->gainsteps;
  uint32_t shift;
  uint32_t g;

  exposure = MIN(MAX(exposure, priv->expmin), priv->expmax);
  if (!priv->aegain)
    {
      return (uint64_t)exposure << 8;
    }

  g     = MIN(MAX(gain, priv->gainmin), priv->gainmax) - priv->gainmin;
  shift = MIN(g / steps, 31);

  return ((uint64_t)exposure *
          g_dcmipp_pow2[(g % steps) * DCMIPP_AE_POW2_STEPS / steps]) <<
         shift;
}

/****************************************************************************
 * Name: dcmipp_aesplit
 *
 * Description:
 *   Split an exposure time times gain product (Q8) into the sensor
 *   settings.  The exposure time is used first, the gain only adds noise.
 *
 ****************************************************************************/

static void dcmipp_aesplit(FAR struct stm32_dcmipp_s *priv, uint64_t total,
                           FAR int32_t *exposure, FAR int32_t *gain)
{
  uint32_t steps = priv->config->gainsteps;
  uint64_t lin;
  int32_t g = 0;
  int i;

  if (!priv->aegain || total <= ((uint64_t)priv->expmax << 8))
    {
      *exposure = MIN(MAX((int32_t)((total + 128) >> 8), priv->expmin),
                      priv->expmax);
      *gain     = priv->aegain ? priv->gainmin : 0;
      return;
    }

  /* Linear gain (Q8) at the longest exposure time, then in sensor steps */

  *exposure = priv->expmax;
  lin       = total / priv->expmax;

  while (lin >= 512 && g < priv->gainmax - priv->gainmin)
    {
      lin >>= 1;
      g    += steps;
    }

  for (i = DCMIPP_AE_POW2_STEPS - 1; i > 0 && g_dcmipp_pow2[i] > lin; i--)
    {
    }

  g    += i * steps / DCMIPP_AE_POW2_STEPS;
  *gain = MIN(priv->gainmin + g, priv->gainmax);
}

/****************************************************************************
 * Name: dcmipp_ae
 *
 * Description:
 *   Move the sensor exposure half way to the one giving the target mean
 *   luminance.  Nothing changes inside the dead band.
 *
 ****************************************************************************/

static void dcmipp_ae(FAR struct stm32_dcmipp_s *priv,
                      FAR const uint32_t *avg)
{
  imgsensor_value_t exposure;
  imgsensor_value_t gain;
  uint32_t target = CONFIG_STM32_DCMIPP_AE_TARGET << 8;
  uint64_t total;
  uint32_t ratio;
  uint32_t lum;
  int32_t newexp;
  int32_t newgain;

  if (!priv->ae)
    {
      return;
    }

  /* BT.601 luminance of the means, 1/256 units */

  lum = (77 * avg[0] + 150 * avg[1] + 29 * avg[2]) >> 8;
  if (abs((int32_t)lum - (int32_t)target) <= (DCMIPP_AE_TOLERANCE << 8))
    {
      return;
    }

  gain.value32 = 0;
  if (IMGSENSOR_GET_VALUE(priv->sensor, IMGSENSOR_ID_EXPOSURE_ABSOLUTE,
                          sizeof(int32_t), &exposure) < 0 ||
      (priv->aegain &&
       IMGSENSOR_GET_VALUE(priv->sensor, IMGSENSOR_ID_GAIN,
                           sizeof(int32_t), &gain) < 0))
    {
      return;
    }

  /* Exposure ratio towards the target (Q8), limited.  Take half of it, or
   * all of it when half does not change the coarse sensor settings.
   */

  ratio = lum > 0 ? (target << 8) / lum : DCMIPP_AE_RATIO_MAX;
  ratio = MIN(MAX(ratio, DCMIPP_AE_RATIO_MIN), DCMIPP_AE_RATIO_MAX);
  total = dcmipp_aetotal(priv, exposure.value32, gain.value32);

  dcmipp_aesplit(priv, (total * ((ratio + 256) / 2)) >> 8,
                 &newexp, &newgain);
  if (newexp == exposure.value32 && newgain == gain.value32)
    {
      dcmipp_aesplit(priv, (total * ratio) >> 8, &newexp, &newgain);
    }

  if (newexp != exposure.value32)
    {
      exposure.value32 = newexp;
      IMGSENSOR_SET_VALUE(priv->sensor, IMGSENSOR_ID_EXPOSURE_ABSOLUTE,
                          sizeof(int32_t), exposure);
    }

  if (priv->aegain && newgain != gain.value32)
    {
      gain.value32 = newgain;
      IMGSENSOR_SET_VALUE(priv->sensor, IMGSENSOR_ID_GAIN,
                          sizeof(int32_t), gain);
    }
}
#endif

#ifdef CONFIG_STM32_DCMIPP_AWB
/****************************************************************************
 * Name: dcmipp_awb
 *
 * Description:
 *   Grey world white balance: move the red and blue gains half way to the
 *   ones matching the red and blue means to the green one.  Dark and
 *   saturated frames are skipped.
 *
 ****************************************************************************/

static void dcmipp_awb(FAR struct stm32_dcmipp_s *priv,
                       FAR const uint32_t *avg)
{
  bool update = false;
  uint32_t target;
  uint32_t gain;
  int i;

  if (avg[1] < (DCMIPP_AWB_GREEN_MIN << 8) ||
      avg[1] > (DCMIPP_AWB_GREEN_MAX << 8))
    {
      return;
    }

  for (i = 0; i < 3; i += 2)
    {
      gain   = priv->gain[i];
      target = avg[i] > 0 ? gain * avg[1] / avg[i] : DCMIPP_AWB_GAIN_MAX;

      if (abs((int32_t)target - (int32_t)gain) * DCMIPP_AWB_DEADBAND <=
          gain)
        {
          continue;
        }

      gain = MIN(MAX((gain + target) / 2, DCMIPP_AWB_GAIN_MIN),
                 DCMIPP_AWB_GAIN_MAX);
      if (gain != priv->gain[i])
        {
          priv->gain[i] = gain;
          update        = true;
        }
    }

  if (update)
    {
      dcmipp_setwb(priv->gain);
    }
}
#endif

#ifdef HAVE_DCMIPP_AUTO
/****************************************************************************
 * Name: dcmipp_worker
 *
 * Description:
 *   Run the automatic exposure and white balance on the latest statistics.
 *
 ****************************************************************************/

static void dcmipp_worker(FAR void *arg)
{
  FAR struct stm32_dcmipp_s *priv = arg;
  uint32_t avg[3];
  int i;

  if (priv->capturing && priv->statpixels > 0)
    {
      /* Means of the 8-bit components, 1/256 units */

      for (i = 0; i < 3; i++)
        {
          avg[i] = ((uint64_t)priv->stat[i] << 16) / priv->statpixels;
        }

#ifdef CONFIG_STM32_DCMIPP_AE
      nxmutex_lock(&priv->aelock);
      if (priv->aeauto)
        {
          dcmipp_ae(priv, avg);
        }

      nxmutex_unlock(&priv->aelock);
#endif
#ifdef CONFIG_STM32_DCMIPP_AWB
      if (priv->awbauto)
        {
          dcmipp_awb(priv, avg);
        }
#endif
    }

  /* Wait for frames with the new settings */

  priv->skip = CONFIG_STM32_DCMIPP_ISP_INTERVAL;
  priv->busy = false;
}

/****************************************************************************
 * Name: dcmipp_stats
 *
 * Description:
 *   Latch the statistics of the previous frame at the vertical sync and
 *   defer their processing to the worker.
 *
 ****************************************************************************/

static void dcmipp_stats(FAR struct stm32_dcmipp_s *priv)
{
  if (priv->skip > 0)
    {
      priv->skip--;
      return;
    }

  if (priv->busy)
    {
      return;
    }

  priv->stat[0] = getreg32(STM32_DCMIPP_P1ST1SR) & DCMIPP_P1STSR_ACCU_MASK;
  priv->stat[1] = getreg32(STM32_DCMIPP_P1ST2SR) & DCMIPP_P1STSR_ACCU_MASK;
  priv->stat[2] = getreg32(STM32_DCMIPP_P1ST3SR) & DCMIPP_P1STSR_ACCU_MASK;
  priv->busy    = true;

  work_queue(LPWORK, &priv->work, dcmipp_worker, priv, 0);
}

/****************************************************************************
 * Name: dcmipp_autostart
 *
 * Description:
 *   Prepare the automatic exposure and white balance for a new capture.
 *
 ****************************************************************************/

static void dcmipp_autostart(FAR struct stm32_dcmipp_s *priv)
{
#ifdef CONFIG_STM32_DCMIPP_AE
  imgsensor_supported_value_t val;

  /* Take the exposure time and gain ranges from the sensor */

  priv->ae     = false;
  priv->aegain = false;

  if (priv->sensor != NULL &&
      IMGSENSOR_GET_SUPPORTED_VALUE(priv->sensor,
                                    IMGSENSOR_ID_EXPOSURE_ABSOLUTE,
                                    &val) >= 0)
    {
      priv->expmin = MAX(val.u.range.minimum, 1);
      priv->expmax = val.u.range.maximum;
      priv->ae     = priv->expmax > priv->expmin;
    }

  if (priv->ae && priv->config->gainsteps > 0 &&
      IMGSENSOR_GET_SUPPORTED_VALUE(priv->sensor, IMGSENSOR_ID_GAIN,
                                    &val) >= 0)
    {
      priv->gainmin = val.u.range.minimum;
      priv->gainmax = val.u.range.maximum;
      priv->aegain  = priv->gainmax > priv->gainmin;
    }
#endif

  /* The first frames come from the pipe start */

  priv->skip = CONFIG_STM32_DCMIPP_ISP_INTERVAL;
}
#endif

/****************************************************************************
 * Name: dcmipp_configscaler
 *
 * Description:
 *   Scale the sensor frame to the output size: decimate by powers of two
 *   until the remaining ratio fits the downsizer.
 *
 ****************************************************************************/

static void dcmipp_configscaler(FAR const struct stm32_dcmipp_config_s *cfg,
                                uint32_t width, uint32_t height)
{
  uint32_t srcw = cfg->width;
  uint32_t srch = cfg->height;
  uint32_t hdec = 0;
  uint32_t vdec = 0;
  uint32_t hratio;
  uint32_t vratio;
  uint32_t hdiv;
  uint32_t vdiv;

  /* Crop to the sensor frame, so that extra lines or columns sent by the
   * sensor never reach the memory.
   */

  putreg32(0, STM32_DCMIPP_P1CRSTR);
  putreg32((srcw << DCMIPP_P1CRSZR_HSIZE_SHIFT) |
           (srch << DCMIPP_P1CRSZR_VSIZE_SHIFT) | DCMIPP_P1CRSZR_ENABLE,
           STM32_DCMIPP_P1CRSZR);

  while (width * DCMIPP_DSIZE_MAX < srcw && hdec < 3)
    {
      hdec++;
      srcw /= 2;
    }

  while (height * DCMIPP_DSIZE_MAX < srch && vdec < 3)
    {
      vdec++;
      srch /= 2;
    }

  if (hdec != 0 || vdec != 0)
    {
      putreg32(DCMIPP_P1DCCR_ENABLE | DCMIPP_P1DCCR_HDEC(hdec) |
               DCMIPP_P1DCCR_VDEC(vdec), STM32_DCMIPP_P1DCCR);
    }
  else
    {
      putreg32(0, STM32_DCMIPP_P1DCCR);
    }

  if (srcw == width && srch == height)
    {
      putreg32(0, STM32_DCMIPP_P1DSCR);
      return;
    }

  /* Ratio: 8192 * src / dst; division factor: 1024 * dst / src */

  hratio = MIN(srcw * 8192 / width, 65535);
  vratio = MIN(srch * 8192 / height, 65535);
  hdiv   = MIN(width * 1024 / srcw, 1023);
  vdiv   = MIN(height * 1024 / srch, 1023);

  putreg32((hratio << DCMIPP_P1DSRTIOR_HRATIO_SHIFT) |
           (vratio << DCMIPP_P1DSRTIOR_VRATIO_SHIFT),
           STM32_DCMIPP_P1DSRTIOR);
  putreg32((width << DCMIPP_P1DSSZR_HSIZE_SHIFT) |
           (height << DCMIPP_P1DSSZR_VSIZE_SHIFT), STM32_DCMIPP_P1DSSZR);
  putreg32(DCMIPP_P1DSCR_ENABLE | (hdiv << DCMIPP_P1DSCR_HDIV_SHIFT) |
           (vdiv << DCMIPP_P1DSCR_VDIV_SHIFT), STM32_DCMIPP_P1DSCR);
}

/****************************************************************************
 * Name: dcmipp_disablepipe
 *
 * Description:
 *   Let the frame being captured end, then disable pipe1.
 *
 ****************************************************************************/

static void dcmipp_disablepipe(void)
{
  int i;

  for (i = 0; i < DCMIPP_STOP_TIMEOUT; i++)
    {
      if ((getreg32(STM32_DCMIPP_CMSR1) & DCMIPP_CMSR1_P1CPTACT) == 0)
        {
          break;
        }

      nxsig_usleep(1000);
    }

  modifyreg32(STM32_DCMIPP_P1FSCR, DCMIPP_P1FSCR_PIPEN, 0);
}

/****************************************************************************
 * Name: dcmipp_arm
 *
 * Description:
 *   Request the capture of the next frame into priv->buf.  The pipe takes
 *   the address and the request at the frame start.
 *
 ****************************************************************************/

static void dcmipp_arm(FAR struct stm32_dcmipp_s *priv)
{
  putreg32(priv->buf, STM32_DCMIPP_P1PPM0AR1);
  modifyreg32(STM32_DCMIPP_P1FCTCR, 0, DCMIPP_P1FCTCR_CPTREQ);
  priv->armed = true;
}

/****************************************************************************
 * Name: dcmipp_done
 *
 * Description:
 *   Hand the frame in priv->buf to the capture framework.  Its callback
 *   gives the next buffer with set_buf() or stops the capture.
 *
 ****************************************************************************/

static void dcmipp_done(FAR struct stm32_dcmipp_s *priv, bool err)
{
  struct timespec ts;
  struct timeval tv;

  priv->buf = 0;
  if (!priv->capturing || priv->callback == NULL)
    {
      return;
    }

  /* One clean and invalidate of the whole data cache drops the lines the
   * CPU may have fetched from this buffer during the transfer and writes
   * back any dirty line of the next one before the DCMIPP writes it.  Its
   * cost is bounded by the cache size, not by the frame size.
   */

  up_flush_dcache_all();

  clock_systime_timespec(&ts);
  tv.tv_sec  = ts.tv_sec;
  tv.tv_usec = ts.tv_nsec / 1000;

  priv->callback(err ? EIO : 0, err ? 0 : priv->framesize, &tv, priv->arg);
}

/****************************************************************************
 * Name: dcmipp_interrupt
 *
 * Description:
 *   Each buffer gets one frame: the capture request is withdrawn at the
 *   frame start (vertical sync), where the pipe takes it with the address,
 *   and is renewed by set_buf() with the next buffer.  A frame starting
 *   before that is not captured, so the DCMIPP never writes a buffer
 *   handed back to the application.
 *
 ****************************************************************************/

static int dcmipp_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct stm32_dcmipp_s *priv = arg;
  bool armed = priv->armed;
  uint32_t status;

  /* A frame start in this status precedes a request made while handling
   * it, so only a request made before counts (armed).
   */

  status = getreg32(STM32_DCMIPP_CMSR2) & getreg32(STM32_DCMIPP_CMIER);
  putreg32(status, STM32_DCMIPP_CMFCR);

  if ((status & DCMIPP_P1_ERRS) != 0)
    {
      priv->frameerr = true;
      verr("ERROR: DCMIPP %s\n", (status & DCMIPP_CMINT_ATXERR) != 0 ?
           "AXI transfer error" : "pipe1 overrun");
    }

  if ((status & DCMIPP_CMINT_P1FRAME) != 0)
    {
      if (priv->active)
        {
          if (stm32_csi_errors())
            {
              priv->frameerr = true;
            }

          priv->active = false;
          dcmipp_done(priv, priv->frameerr);
        }
      else if (priv->armed)
        {
          /* The frame start was missed: the pipe may already write the
           * next frame into the same buffer.
           */

          modifyreg32(STM32_DCMIPP_P1FCTCR, DCMIPP_P1FCTCR_CPTREQ, 0);
          priv->armed = false;
          dcmipp_done(priv, true);
        }
    }

  if ((status & DCMIPP_CMINT_P1VSYNC) != 0 && priv->capturing)
    {
#ifdef HAVE_DCMIPP_AUTO
      dcmipp_stats(priv);
#endif

      /* A frame without frame end until the next start is lost */

      if (priv->active)
        {
          priv->active = false;
          dcmipp_done(priv, true);
        }
      else if (armed && priv->armed)
        {
          modifyreg32(STM32_DCMIPP_P1FCTCR, DCMIPP_P1FCTCR_CPTREQ, 0);
          priv->armed    = false;
          priv->active   = true;
          priv->frameerr = false;

          /* Drop the CSI-2 errors seen before this frame */

          stm32_csi_errors();
        }
    }

  return OK;
}

/****************************************************************************
 * Name: dcmipp_init
 ****************************************************************************/

static int dcmipp_init(FAR struct imgdata_s *data)
{
  FAR struct stm32_dcmipp_s *priv = (FAR struct stm32_dcmipp_s *)data;
  FAR const struct stm32_dcmipp_config_s *cfg = priv->config;
  int ret;

  /* Reset the DCMIPP and the CSI-2 host */

  putreg32(RCC_APB5RSTR_DCMIPPRST | RCC_APB5RSTR_CSIRST,
           STM32_RCC_APB5RSTSR);
  putreg32(RCC_APB5RSTR_DCMIPPRST | RCC_APB5RSTR_CSIRST,
           STM32_RCC_APB5RSTCR);

  /* Serial input, pipe1 takes the raw data type of virtual channel 0 */

  putreg32(0, STM32_DCMIPP_PRCR);
  putreg32(DCMIPP_CMCR_INSEL, STM32_DCMIPP_CMCR);
  putreg32(DCMIPP_P1FSCR_DTIDA(DCMIPP_DT_RAW8 + (cfg->bpp - 8) / 2) |
           DCMIPP_P1FSCR_DTMODE_DTIDA | DCMIPP_P1FSCR_VC(0),
           STM32_DCMIPP_P1FSCR);

  dcmipp_configisp(cfg);

#ifdef HAVE_DCMIPP_AUTO
  dcmipp_configstats(priv);
  priv->gain[0] = cfg->gain[0];
  priv->gain[1] = cfg->gain[1];
  priv->gain[2] = cfg->gain[2];
  priv->busy    = false;
#endif

  /* Start the D-PHY and virtual channel 0 of the CSI-2 host */

  ret = stm32_csi_start(cfg->lanes, cfg->bitrate, cfg->bpp);
  if (ret < 0)
    {
      return ret;
    }

  priv->capturing = false;
  priv->armed     = false;
  priv->active    = false;
  priv->buf       = 0;
  priv->callback  = NULL;
  up_enable_irq(STM32_IRQ_DCMIPP);
  return OK;
}

/****************************************************************************
 * Name: dcmipp_uninit
 ****************************************************************************/

static int dcmipp_uninit(FAR struct imgdata_s *data)
{
  dcmipp_stop_capture(data);
  up_disable_irq(STM32_IRQ_DCMIPP);
  stm32_csi_stop();
  return OK;
}

/****************************************************************************
 * Name: dcmipp_set_buf
 ****************************************************************************/

static int dcmipp_set_buf(FAR struct imgdata_s *data,
                          uint8_t nr_datafmts,
                          FAR imgdata_format_t *datafmts,
                          FAR uint8_t *addr, uint32_t size)
{
  FAR struct stm32_dcmipp_s *priv = (FAR struct stm32_dcmipp_s *)data;
  uint32_t framesize;
  irqstate_t flags;

  framesize = datafmts[IMGDATA_FMT_MAIN].width *
              datafmts[IMGDATA_FMT_MAIN].height * 2;

  /* The buffer must be cache-line aligned so that the cache maintenance
   * cannot touch unrelated data.
   */

  if (addr == NULL ||
      ((uintptr_t)addr & (ARMV8M_DCACHE_LINESIZE - 1)) != 0 ||
      size < framesize)
    {
      return -EINVAL;
    }

  /* No dirty line may be evicted over the frame.  From the frame interrupt
   * dcmipp_done() has already cleaned the whole cache.
   */

  if (!up_interrupt_context())
    {
      up_flush_dcache((uintptr_t)addr, (uintptr_t)addr + framesize);
    }

  flags = enter_critical_section();

  priv->buf       = (uintptr_t)addr;
  priv->framesize = framesize;
  if (priv->capturing)
    {
      dcmipp_arm(priv);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: dcmipp_validate_frame_setting
 ****************************************************************************/

static int dcmipp_validate_frame_setting(FAR struct imgdata_s *data,
                                         uint8_t nr_datafmts,
                                         FAR imgdata_format_t *datafmts,
                                         FAR imgdata_interval_t *interval)
{
  FAR struct stm32_dcmipp_s *priv = (FAR struct stm32_dcmipp_s *)data;
  FAR const struct stm32_dcmipp_config_s *cfg = priv->config;
  FAR imgdata_format_t *fmt;

  if (nr_datafmts != 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  fmt = &datafmts[IMGDATA_FMT_MAIN];
  if (fmt->pixelformat != IMGDATA_PIX_FMT_RGB565)
    {
      return -EINVAL;
    }

  /* The line pitch must be a multiple of 16 bytes; at most 1/64 of the
   * sensor size (1/8 decimation and 1/8 downsize).
   */

  if ((fmt->width & 7) != 0 || fmt->width > cfg->width ||
      fmt->height > cfg->height ||
      fmt->width * 64 < cfg->width || fmt->height * 64 < cfg->height)
    {
      return -EINVAL;
    }

  /* Frames are whole cache lines, so that consecutive buffers never share
   * one.
   */

  if ((fmt->width * fmt->height * 2) % ARMV8M_DCACHE_LINESIZE != 0)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: dcmipp_start_capture
 ****************************************************************************/

static int dcmipp_start_capture(FAR struct imgdata_s *data,
                                uint8_t nr_datafmts,
                                FAR imgdata_format_t *datafmts,
                                FAR imgdata_interval_t *interval,
                                FAR imgdata_capture_t callback,
                                FAR void *arg)
{
  FAR struct stm32_dcmipp_s *priv = (FAR struct stm32_dcmipp_s *)data;
  FAR imgdata_format_t *fmt = &datafmts[IMGDATA_FMT_MAIN];
  irqstate_t flags;
  int ret;

  ret = dcmipp_validate_frame_setting(data, nr_datafmts, datafmts,
                                      interval);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->buf == 0 || priv->framesize != fmt->width * fmt->height * 2)
    {
      return -EINVAL;
    }

  /* A stop from the frame interrupt leaves the pipe enabled */

  if ((getreg32(STM32_DCMIPP_P1FSCR) & DCMIPP_P1FSCR_PIPEN) != 0)
    {
      dcmipp_disablepipe();
    }

  dcmipp_configscaler(priv->config, fmt->width, fmt->height);

  putreg32(DCMIPP_P1PPCR_FORMAT_RGB565, STM32_DCMIPP_P1PPCR);
  putreg32(fmt->width * 2, STM32_DCMIPP_P1PPM0PR);

#ifdef HAVE_DCMIPP_AUTO
  dcmipp_autostart(priv);
#endif

  flags = enter_critical_section();

  priv->callback  = callback;
  priv->arg       = arg;
  priv->capturing = true;
  priv->active    = false;

  /* Continuous mode, one frame per buffer (see dcmipp_interrupt()) */

  putreg32(0, STM32_DCMIPP_P1FCTCR);
  putreg32(DCMIPP_P1_INTS, STM32_DCMIPP_CMFCR);
  modifyreg32(STM32_DCMIPP_CMIER, 0, DCMIPP_P1_INTS);
  modifyreg32(STM32_DCMIPP_P1FSCR, 0, DCMIPP_P1FSCR_PIPEN);
  dcmipp_arm(priv);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: dcmipp_stop_capture
 ****************************************************************************/

static int dcmipp_stop_capture(FAR struct imgdata_s *data)
{
  FAR struct stm32_dcmipp_s *priv = (FAR struct stm32_dcmipp_s *)data;
  irqstate_t flags;

  flags = enter_critical_section();

  priv->capturing = false;
  priv->armed     = false;
  priv->active    = false;
  priv->buf       = 0;
  modifyreg32(STM32_DCMIPP_P1FCTCR, DCMIPP_P1FCTCR_CPTREQ, 0);
  modifyreg32(STM32_DCMIPP_CMIER, DCMIPP_P1_INTS, 0);
  putreg32(DCMIPP_P1_INTS, STM32_DCMIPP_CMFCR);

  leave_critical_section(flags);

  /* The capture ends with the current frame.  The capture framework may
   * stop from the frame interrupt; dcmipp_start_capture() then disables
   * the pipe.
   */

  if (up_interrupt_context())
    {
#ifdef HAVE_DCMIPP_AUTO
      work_cancel(LPWORK, &priv->work);
      priv->busy = false;
#endif
    }
  else
    {
#ifdef HAVE_DCMIPP_AUTO
      work_cancel_sync(LPWORK, &priv->work);
      priv->busy = false;
#endif
      dcmipp_disablepipe();
    }

  return OK;
}

/****************************************************************************
 * Name: dcmipp_isp_sensorfmt
 *
 * Description:
 *   Replace the pipe1 output format by the raw sensor frame.
 *
 ****************************************************************************/

static int dcmipp_isp_sensorfmt(FAR struct stm32_dcmipp_s *priv,
                                uint8_t nr_datafmts,
                                FAR const imgsensor_format_t *fmts,
                                FAR imgsensor_format_t *raw)
{
  FAR const struct stm32_dcmipp_config_s *cfg = priv->config;

  if (nr_datafmts != 1 || fmts == NULL ||
      fmts[IMGSENSOR_FMT_MAIN].pixelformat != IMGSENSOR_PIX_FMT_RGB565 ||
      fmts[IMGSENSOR_FMT_MAIN].width > cfg->width ||
      fmts[IMGSENSOR_FMT_MAIN].height > cfg->height)
    {
      return -EINVAL;
    }

  raw->width       = cfg->width;
  raw->height      = cfg->height;
  raw->pixelformat = cfg->pixelformat;
  return OK;
}

/****************************************************************************
 * Name: dcmipp_isp_is_available
 ****************************************************************************/

static bool dcmipp_isp_is_available(FAR struct imgsensor_s *sensor)
{
  return IMGSENSOR_IS_AVAILABLE(g_dcmipp.sensor);
}

/****************************************************************************
 * Name: dcmipp_isp_init
 ****************************************************************************/

static int dcmipp_isp_init(FAR struct imgsensor_s *sensor)
{
  return IMGSENSOR_INIT(g_dcmipp.sensor);
}

/****************************************************************************
 * Name: dcmipp_isp_uninit
 ****************************************************************************/

static int dcmipp_isp_uninit(FAR struct imgsensor_s *sensor)
{
  return IMGSENSOR_UNINIT(g_dcmipp.sensor);
}

/****************************************************************************
 * Name: dcmipp_isp_get_driver_name
 ****************************************************************************/

static FAR const char *
dcmipp_isp_get_driver_name(FAR struct imgsensor_s *sensor)
{
  return IMGSENSOR_GET_DRIVER_NAME(g_dcmipp.sensor);
}

/****************************************************************************
 * Name: dcmipp_isp_validate_frame_setting
 ****************************************************************************/

static int dcmipp_isp_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                             imgsensor_stream_type_t type,
                                             uint8_t nr_datafmts,
                                             FAR imgsensor_format_t *fmts,
                                             FAR imgsensor_interval_t *ival)
{
  imgsensor_format_t raw;
  int ret;

  ret = dcmipp_isp_sensorfmt(&g_dcmipp, nr_datafmts, fmts, &raw);
  if (ret < 0)
    {
      return ret;
    }

  return IMGSENSOR_VALIDATE_FRAME_SETTING(g_dcmipp.sensor, type, 1, &raw,
                                          ival);
}

/****************************************************************************
 * Name: dcmipp_isp_start_capture
 ****************************************************************************/

static int dcmipp_isp_start_capture(FAR struct imgsensor_s *sensor,
                                    imgsensor_stream_type_t type,
                                    uint8_t nr_datafmts,
                                    FAR imgsensor_format_t *fmts,
                                    FAR imgsensor_interval_t *ival)
{
  imgsensor_format_t raw;
  int ret;

  ret = dcmipp_isp_sensorfmt(&g_dcmipp, nr_datafmts, fmts, &raw);
  if (ret < 0)
    {
      return ret;
    }

  return IMGSENSOR_START_CAPTURE(g_dcmipp.sensor, type, 1, &raw, ival);
}

/****************************************************************************
 * Name: dcmipp_isp_stop_capture
 ****************************************************************************/

static int dcmipp_isp_stop_capture(FAR struct imgsensor_s *sensor,
                                   imgsensor_stream_type_t type)
{
  return IMGSENSOR_STOP_CAPTURE(g_dcmipp.sensor, type);
}

/****************************************************************************
 * Name: dcmipp_isp_get_frame_interval
 ****************************************************************************/

static int dcmipp_isp_get_frame_interval(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         FAR imgsensor_interval_t *ival)
{
  return IMGSENSOR_GET_FRAME_INTERVAL(g_dcmipp.sensor, type, ival);
}

/****************************************************************************
 * Name: dcmipp_isp_get_supported_value
 ****************************************************************************/

static int
dcmipp_isp_get_supported_value(FAR struct imgsensor_s *sensor, uint32_t id,
                               FAR imgsensor_supported_value_t *val)
{
  switch (id)
    {
#ifdef CONFIG_STM32_DCMIPP_AE
      case IMGSENSOR_ID_EXPOSURE_AUTO:
        val->type                  = IMGSENSOR_CTRL_TYPE_INTEGER;
        val->u.range.minimum       = IMGSENSOR_EXPOSURE_AUTO;
        val->u.range.maximum       = IMGSENSOR_EXPOSURE_MANUAL;
        val->u.range.step          = 1;
        val->u.range.default_value = IMGSENSOR_EXPOSURE_AUTO;
        return OK;
#endif

#ifdef CONFIG_STM32_DCMIPP_AWB
      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
        val->type                  = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        val->u.range.minimum       = 0;
        val->u.range.maximum       = 1;
        val->u.range.step          = 1;
        val->u.range.default_value = 1;
        return OK;
#endif

      default:
        return IMGSENSOR_GET_SUPPORTED_VALUE(g_dcmipp.sensor, id, val);
    }
}

/****************************************************************************
 * Name: dcmipp_isp_get_value
 ****************************************************************************/

static int dcmipp_isp_get_value(FAR struct imgsensor_s *sensor,
                                uint32_t id, uint32_t size,
                                FAR imgsensor_value_t *value)
{
  switch (id)
    {
#ifdef CONFIG_STM32_DCMIPP_AE
      case IMGSENSOR_ID_EXPOSURE_AUTO:
        value->value32 = g_dcmipp.aeauto ? IMGSENSOR_EXPOSURE_AUTO :
                                           IMGSENSOR_EXPOSURE_MANUAL;
        return OK;
#endif

#ifdef CONFIG_STM32_DCMIPP_AWB
      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
        value->value32 = g_dcmipp.awbauto;
        return OK;
#endif

      default:
        return IMGSENSOR_GET_VALUE(g_dcmipp.sensor, id, size, value);
    }
}

/****************************************************************************
 * Name: dcmipp_isp_set_value
 *
 * Description:
 *   With the automatic exposure on, the exposure time and the gain belong
 *   to it; with it off, the values set by the application are kept.
 *
 ****************************************************************************/

static int dcmipp_isp_set_value(FAR struct imgsensor_s *sensor,
                                uint32_t id, uint32_t size,
                                imgsensor_value_t value)
{
#ifdef CONFIG_STM32_DCMIPP_AE
  int ret;
#endif

  switch (id)
    {
#ifdef CONFIG_STM32_DCMIPP_AE
      case IMGSENSOR_ID_EXPOSURE_AUTO:
        if (value.value32 != IMGSENSOR_EXPOSURE_AUTO &&
            value.value32 != IMGSENSOR_EXPOSURE_MANUAL)
          {
            return -EINVAL;
          }

        nxmutex_lock(&g_dcmipp.aelock);
        g_dcmipp.aeauto = value.value32 == IMGSENSOR_EXPOSURE_AUTO;
        nxmutex_unlock(&g_dcmipp.aelock);
        return OK;

      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
      case IMGSENSOR_ID_GAIN:
        nxmutex_lock(&g_dcmipp.aelock);
        ret = g_dcmipp.aeauto ? -EBUSY :
              IMGSENSOR_SET_VALUE(g_dcmipp.sensor, id, size, value);
        nxmutex_unlock(&g_dcmipp.aelock);
        return ret;
#endif

#ifdef CONFIG_STM32_DCMIPP_AWB
      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
        g_dcmipp.awbauto = value.value32 != 0;
        return OK;
#endif

      default:
        break;
    }

  return IMGSENSOR_SET_VALUE(g_dcmipp.sensor, id, size, value);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dcmipp_initialize
 ****************************************************************************/

int stm32_dcmipp_initialize(FAR const struct stm32_dcmipp_config_s *config,
                            FAR struct imgsensor_s *sensor)
{
  FAR struct stm32_dcmipp_s *priv = &g_dcmipp;
  int ret;
  int i;

  if (config == NULL || sensor == NULL || config->bpp < 8 ||
      config->bpp > 14 || (config->bpp & 1) != 0 ||
      config->bayer > STM32_DCMIPP_BAYER_BGGR)
    {
      return -EINVAL;
    }

  priv->config = config;
  priv->sensor = sensor;
#ifdef CONFIG_STM32_DCMIPP_AE
  nxmutex_init(&priv->aelock);
  priv->aeauto = true;
#endif
#ifdef CONFIG_STM32_DCMIPP_AWB
  priv->awbauto = true;
#endif

  /* QVGA, VGA and the sensor size; any size accepted by
   * dcmipp_validate_frame_setting() can be set.
   */

  for (i = 0; i < nitems(priv->frmsizes); i++)
    {
      priv->frmsizes[i].type = V4L2_FRMSIZE_TYPE_DISCRETE;
    }

  priv->frmsizes[0].discrete.width  = VIDEO_HSIZE_QVGA;
  priv->frmsizes[0].discrete.height = VIDEO_VSIZE_QVGA;
  priv->frmsizes[1].discrete.width  = VIDEO_HSIZE_VGA;
  priv->frmsizes[1].discrete.height = VIDEO_VSIZE_VGA;
  priv->frmsizes[2].discrete.width  = config->width;
  priv->frmsizes[2].discrete.height = config->height;

  priv->isp.fmtdescs         = g_dcmipp_fmtdescs;
  priv->isp.fmtdescs_num     = nitems(g_dcmipp_fmtdescs);
  priv->isp.frmsizes         = priv->frmsizes;
  priv->isp.frmsizes_num     = nitems(priv->frmsizes);
  priv->isp.frmintervals     = sensor->frmintervals;
  priv->isp.frmintervals_num = sensor->frmintervals_num;

  ret = irq_attach(STM32_IRQ_DCMIPP, dcmipp_interrupt, priv);
  if (ret < 0)
    {
      return ret;
    }

  imgdata_register(&priv->data);
  return imgsensor_register(&priv->isp);
}
