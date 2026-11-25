/****************************************************************************
 * arch/arm/src/stm32n6/stm32_dcmipp.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_DCMIPP_H
#define __ARCH_ARM_SRC_STM32N6_STM32_DCMIPP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/video/imgdata.h>
#include <nuttx/video/imgsensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bayer patterns of the raw sensor data */

#define STM32_DCMIPP_BAYER_RGGB  0
#define STM32_DCMIPP_BAYER_GRBG  1
#define STM32_DCMIPP_BAYER_GBRG  2
#define STM32_DCMIPP_BAYER_BGGR  3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Description of the raw Bayer CSI-2 sensor attached to the DCMIPP and the
 * image signal processing applied in pipe1.  The white balance gains are
 * the initial ones with CONFIG_STM32_DCMIPP_AWB.
 */

struct stm32_dcmipp_config_s
{
  uint16_t width;          /* Sensor frame width in pixels */
  uint16_t height;         /* Sensor frame height in lines */
  uint32_t pixelformat;    /* Sensor format, IMGSENSOR_PIX_FMT_xxx */
  uint8_t  lanes;          /* CSI-2 data lanes (1 or 2) */
  uint16_t bitrate;        /* CSI-2 bit rate per lane in Mbit/s */
  uint8_t  bpp;            /* Raw bits per pixel (8, 10, 12 or 14) */
  uint8_t  bayer;          /* STM32_DCMIPP_BAYER_xxx */
  uint8_t  blacklevel;     /* Black level subtracted (8-bit scale) */
  uint16_t gain[3];        /* R, G, B white balance gains, 128 = 1.0 */
  int16_t  ccm[3][3];      /* Colour correction matrix, 256 = 1.0 */
  bool     gamma;          /* Enable the gamma correction */
  uint8_t  gainsteps;      /* Sensor gain steps doubling the gain (AE),
                            * 0: exposure time only */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dcmipp_initialize
 *
 * Description:
 *   Initialize the DCMIPP and CSI-2 host capture interface.  Pipe1
 *   demosaics the raw stream and outputs RGB565 frames scaled to the
 *   requested size.  The DCMIPP registers itself as the image data and,
 *   in front of the raw sensor, as the image sensor of the capture
 *   framework; do not register the raw sensor.
 *
 * Input Parameters:
 *   config - Sensor and image processing description; must stay valid
 *   sensor - Raw sensor
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_dcmipp_initialize(FAR const struct stm32_dcmipp_config_s *config,
                            FAR struct imgsensor_s *sensor);

#endif /* __ARCH_ARM_SRC_STM32N6_STM32_DCMIPP_H */
