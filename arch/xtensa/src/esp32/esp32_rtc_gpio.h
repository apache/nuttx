/****************************************************************************
 * arch/xtensa/src/esp32/esp32_rtc_gpio.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_RTC_GPIO_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_RTC_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/esp32_rtc_io.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEFAULT_RTC_PU_BIT          (1 << 27)
#define DEFAULT_RTC_PD_BIT          (1 << 28)
#define SPECIAL_RTC_PU_BIT          (1 << 22)
#define SPECIAL_RTC_PD_BIT          (1 << 23)

/* RTC GPIO Index */

#define RTCIO_GPIO0_IDX         0  /* RTCIO_IDX_0 */

#define RTCIO_GPIO2_IDX         1  /* RTCIO_IDX_2 */

#define RTCIO_GPIO4_IDX         2  /* RTCIO_IDX_4 */

#define RTCIO_GPIO12_IDX        3  /* RTCIO_IDX_12 */

#define RTCIO_GPIO13_IDX        4  /* RTCIO_IDX_13 */

#define RTCIO_GPIO14_IDX        5  /* RTCIO_IDX_14 */

#define RTCIO_GPIO15_IDX        6  /* RTCIO_IDX_15 */

#define RTCIO_GPIO25_IDX        7  /* RTCIO_IDX_25 */

#define RTCIO_GPIO26_IDX        8  /* RTCIO_IDX_26 */

#define RTCIO_GPIO27_IDX        9  /* RTCIO_IDX_27 */

#define RTCIO_GPIO32_IDX        10  /* RTCIO_IDX_32 */

#define RTCIO_GPIO33_IDX        11  /* RTCIO_IDX_33 */

#define RTCIO_GPIO34_IDX        12  /* RTCIO_IDX_34 */

#define RTCIO_GPIO35_IDX        13  /* RTCIO_IDX_35 */

#define RTCIO_GPIO36_IDX        14  /* RTCIO_IDX_36 */

#define RTCIO_GPIO37_IDX        15  /* RTCIO_IDX_37 */

#define RTCIO_GPIO38_IDX        16  /* RTCIO_IDX_38 */

#define RTCIO_GPIO39_IDX        17  /* RTCIO_IDX_39 */

#define RTC_GPIO_NUMBER         18

/****************************************************************************
 * Public Data
 ****************************************************************************/

const int g_rtc_io_num_map[GPIO_PIN_COUNT + 1] =
{
    RTCIO_GPIO0_IDX,        /* GPIO0 */
    -1,                     /* GPIO1 not supported */
    RTCIO_GPIO2_IDX,        /* GPIO2 */
    -1,                     /* GPIO3 not supported */
    RTCIO_GPIO4_IDX,        /* GPIO4 */
    -1,                     /* GPIO5 not supported */
    -1,                     /* GPIO6 not supported */
    -1,                     /* GPIO7 not supported */
    -1,                     /* GPIO8 not supported */
    -1,                     /* GPIO9 not supported */
    -1,                     /* GPIO10 not supported */
    -1,                     /* GPIO11 not supported */
    RTCIO_GPIO12_IDX,       /* GPIO12 */
    RTCIO_GPIO13_IDX,       /* GPIO13 */
    RTCIO_GPIO14_IDX,       /* GPIO14 */
    RTCIO_GPIO15_IDX,       /* GPIO15 */
    -1,                     /* GPIO16 not supported */
    -1,                     /* GPIO17 not supported */
    -1,                     /* GPIO18 not supported */
    -1,                     /* GPIO19 not supported */
    -1,                     /* GPIO20 not supported */
    -1,                     /* GPIO21 not supported */
    -1,                     /* GPIO22 not supported */
    -1,                     /* GPIO23 not supported */
    -1,                     /* GPIO24 not supported */
    RTCIO_GPIO25_IDX,       /* GPIO25 */
    RTCIO_GPIO26_IDX,       /* GPIO26 */
    RTCIO_GPIO27_IDX,       /* GPIO27 */
    -1,                     /* GPIO28 not supported */
    -1,                     /* GPIO29 not supported */
    -1,                     /* GPIO30 not supported */
    -1,                     /* GPIO31 not supported */
    RTCIO_GPIO32_IDX,       /* GPIO32 */
    RTCIO_GPIO33_IDX,       /* GPIO33 */
    RTCIO_GPIO34_IDX,       /* GPIO34 */
    RTCIO_GPIO35_IDX,       /* GPIO35 */
    RTCIO_GPIO36_IDX,       /* GPIO36 */
    RTCIO_GPIO37_IDX,       /* GPIO37 */
    RTCIO_GPIO38_IDX,       /* GPIO38 */
    RTCIO_GPIO39_IDX        /* GPIO39 */
};

const int g_rtc_io_desc[RTC_GPIO_NUMBER] =
{
  RTC_IO_TOUCH_PAD1_REG,
  RTC_IO_TOUCH_PAD2_REG,
  RTC_IO_TOUCH_PAD0_REG,
  RTC_IO_TOUCH_PAD5_REG,
  RTC_IO_TOUCH_PAD4_REG,
  RTC_IO_TOUCH_PAD6_REG,
  RTC_IO_TOUCH_PAD3_REG,
  RTC_IO_PAD_DAC1_REG,
  RTC_IO_PAD_DAC2_REG,
  RTC_IO_TOUCH_PAD7_REG,
  RTC_IO_XTAL_32K_PAD_REG,
  RTC_IO_XTAL_32K_PAD_REG,
  RTC_IO_ADC_PAD_REG,
  RTC_IO_ADC_PAD_REG,
  RTC_IO_SENSOR_PADS_REG,
  RTC_IO_SENSOR_PADS_REG,
  RTC_IO_SENSOR_PADS_REG,
  RTC_IO_SENSOR_PADS_REG
};

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_RTC_GPIO_H */
