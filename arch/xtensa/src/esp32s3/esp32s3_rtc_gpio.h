/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_rtc_gpio.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_RTC_GPIO_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_RTC_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "hardware/esp32s3_rtc_io.h"
#include "hardware/esp32s3_rtccntl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTCIO_PIN_FUNC                  0

#define RTC_MODE_SHIFT                  0
#define RTC_MODE_MASK                   (7 << RTC_MODE_SHIFT)
#  define RTC_INPUT                     (1 << (RTC_MODE_SHIFT + 0))
#  define RTC_OUTPUT                    (1 << (RTC_MODE_SHIFT + 1))

#define RTC_PULL_SHIFT                  2
#define RTC_PULL_MASK                   (7 << RTC_PULL_SHIFT)
#  define RTC_PULLUP                    (1 << (RTC_PULL_SHIFT + 0))
#  define RTC_PULLDOWN                  (1 << (RTC_PULL_SHIFT + 1))
#  define RTC_OPEN_DRAIN                (1 << (RTC_PULL_SHIFT + 2))

#define RTC_FUNCTION_SHIFT              5
#define RTC_FUNCTION_MASK               (3 << RTC_FUNCTION_SHIFT)
#  define RTC_FUNCTION_RTCIO            (1 << RTC_FUNCTION_SHIFT)
#  define RTC_FUNCTION_DIGITAL          (2 << RTC_FUNCTION_SHIFT)

#define RTC_DRIVE_SHIFT                 7
#define RTC_DRIVE_MASK                  (7 << RTC_DRIVE_SHIFT)
#  define RTC_DRIVE_0                   (1 << RTC_DRIVE_SHIFT)
#  define RTC_DRIVE_1                   (2 << RTC_DRIVE_SHIFT)
#  define RTC_DRIVE_2                   (3 << RTC_DRIVE_SHIFT)
#  define RTC_DRIVE_3                   (4 << RTC_DRIVE_SHIFT)

#define RTC_INPUT_PULLUP                (RTC_INPUT | RTC_PULLUP)
#define RTC_INPUT_PULLDOWN              (RTC_INPUT | RTC_PULLDOWN)
#define RTC_OUTPUT_OPEN_DRAIN           (RTC_OUTPUT | RTC_OPEN_DRAIN)
#define RTC_INPUT_FUNCTION_RTCIO        (RTC_INPUT | RTC_FUNCTION_RTCIO)
#define RTC_INPUT_FUNCTION_DIGITAL      (RTC_INPUT | RTC_FUNCTION_DIGITAL)
#define RTC_OUTPUT_FUNCTION_RTCIO       (RTC_OUTPUT | RTC_FUNCTION_RTCIO)
#define RTC_OUTPUT_FUNCTION_DIGITAL     (RTC_OUTPUT | RTC_FUNCTION_DIGITAL)

/* RTC GPIO channels */

#define RTC_GPIO_NUMBER             22

#define RTCIO_GPIO0_CHANNEL         0   /* RTCIO_CHANNEL_0 */
#define RTCIO_CHANNEL_0_GPIO_NUM    0

#define RTCIO_GPIO1_CHANNEL         1   /* RTCIO_CHANNEL_1 */
#define RTCIO_CHANNEL_1_GPIO_NUM    1

#define RTCIO_GPIO2_CHANNEL         2   /* RTCIO_CHANNEL_2 */
#define RTCIO_CHANNEL_2_GPIO_NUM    2

#define RTCIO_GPIO3_CHANNEL         3   /* RTCIO_CHANNEL_3 */
#define RTCIO_CHANNEL_3_GPIO_NUM    3

#define RTCIO_GPIO4_CHANNEL         4   /* RTCIO_CHANNEL_4 */
#define RTCIO_CHANNEL_4_GPIO_NUM    4

#define RTCIO_GPIO5_CHANNEL         5   /* RTCIO_CHANNEL_5 */
#define RTCIO_CHANNEL_5_GPIO_NUM    5

#define RTCIO_GPIO6_CHANNEL         6   /* RTCIO_CHANNEL_6 */
#define RTCIO_CHANNEL_6_GPIO_NUM    6

#define RTCIO_GPIO7_CHANNEL         7   /* RTCIO_CHANNEL_7 */
#define RTCIO_CHANNEL_7_GPIO_NUM    7

#define RTCIO_GPIO8_CHANNEL         8   /* RTCIO_CHANNEL_8 */
#define RTCIO_CHANNEL_8_GPIO_NUM    8

#define RTCIO_GPIO9_CHANNEL         9   /* RTCIO_CHANNEL_9 */
#define RTCIO_CHANNEL_9_GPIO_NUM    9

#define RTCIO_GPIO10_CHANNEL        10   /* RTCIO_CHANNEL_10 */
#define RTCIO_CHANNEL_10_GPIO_NUM   10

#define RTCIO_GPIO11_CHANNEL        11   /* RTCIO_CHANNEL_11 */
#define RTCIO_CHANNEL_11_GPIO_NUM   11

#define RTCIO_GPIO12_CHANNEL        12   /* RTCIO_CHANNEL_12 */
#define RTCIO_CHANNEL_12_GPIO_NUM   12

#define RTCIO_GPIO13_CHANNEL        13   /* RTCIO_CHANNEL_13 */
#define RTCIO_CHANNEL_13_GPIO_NUM   13

#define RTCIO_GPIO14_CHANNEL        14   /* RTCIO_CHANNEL_14 */
#define RTCIO_CHANNEL_14_GPIO_NUM   14

#define RTCIO_GPIO15_CHANNEL        15   /* RTCIO_CHANNEL_15 */
#define RTCIO_CHANNEL_15_GPIO_NUM   15

#define RTCIO_GPIO16_CHANNEL        16   /* RTCIO_CHANNEL_16 */
#define RTCIO_CHANNEL_16_GPIO_NUM   16

#define RTCIO_GPIO17_CHANNEL        17   /* RTCIO_CHANNEL_17 */
#define RTCIO_CHANNEL_17_GPIO_NUM   17

#define RTCIO_GPIO18_CHANNEL        18   /* RTCIO_CHANNEL_18 */
#define RTCIO_CHANNEL_18_GPIO_NUM   18

#define RTCIO_GPIO19_CHANNEL        19   /* RTCIO_CHANNEL_19 */
#define RTCIO_CHANNEL_19_GPIO_NUM   19

#define RTCIO_GPIO20_CHANNEL        20   /* RTCIO_CHANNEL_20 */
#define RTCIO_CHANNEL_20_GPIO_NUM   20

#define RTCIO_GPIO21_CHANNEL        21   /* RTCIO_CHANNEL_21 */
#define RTCIO_CHANNEL_21_GPIO_NUM   21

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct rtc_io_desc_s
{
  uint32_t reg;        /* Register of RTC pad, or 0 if not an RTC GPIO */
  uint32_t mux;        /* Bit mask for selecting digital pad or RTC pad */
  uint32_t func;       /* Shift of pad function (FUN_SEL) field */
  uint32_t ie;         /* Mask of input enable */
  uint32_t pullup;     /* Mask of pullup enable */
  uint32_t pulldown;   /* Mask of pulldown enable */
  uint32_t slpsel;     /* If slpsel bit is set, slpie will be used as pad
                        * input enabled signal in sleep mode
                        */
  uint32_t slpie;      /* Mask of input enable in sleep mode */
  uint32_t slpoe;      /* Mask of output enable in sleep mode */
  uint32_t hold;       /* Mask of hold enable */
  uint32_t hold_force; /* Mask of hold_force bit for RTC IO in
                        * RTC_CNTL_HOLD_REG
                        */
  uint32_t drv_v;      /* Mask of drive capability */
  uint32_t drv_s;      /* Offset of drive capability */
  int rtc_num;         /* GPIO number (corresponds to RTC pad) */
};

/* Must be big enough to hold the above encodings */

typedef uint16_t rtcio_pinattr_t;
typedef struct rtc_io_desc_s rtc_io_desc_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

static const rtc_io_desc_t g_rtc_io_desc[RTC_GPIO_NUMBER] =
{
  /* REG
   * MUX select
   * Function select
   * Input enable
   * Pullup
   * Pulldown
   * Sleep select
   * Sleep input enable
   * PAD hold
   * Pad force hold
   * Mask of drive capability Offset
   * GPIO number
   */

  {
    RTCIO_TOUCH_PAD0_REG,
    RTCIO_TOUCH_PAD0_MUX_SEL_M,
    RTCIO_TOUCH_PAD0_FUN_SEL_S,
    RTCIO_TOUCH_PAD0_FUN_IE_M,
    RTCIO_TOUCH_PAD0_RUE_M,
    RTCIO_TOUCH_PAD0_RDE_M,
    RTCIO_TOUCH_PAD0_SLP_SEL_M,
    RTCIO_TOUCH_PAD0_SLP_IE_M,
    RTCIO_TOUCH_PAD0_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD0_HOLD_M,
    RTCIO_TOUCH_PAD0_DRV_V,
    RTCIO_TOUCH_PAD0_DRV_S,
    RTCIO_CHANNEL_0_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD1_REG,
    RTCIO_TOUCH_PAD1_MUX_SEL_M,
    RTCIO_TOUCH_PAD1_FUN_SEL_S,
    RTCIO_TOUCH_PAD1_FUN_IE_M,
    RTCIO_TOUCH_PAD1_RUE_M,
    RTCIO_TOUCH_PAD1_RDE_M,
    RTCIO_TOUCH_PAD1_SLP_SEL_M,
    RTCIO_TOUCH_PAD1_SLP_IE_M,
    RTCIO_TOUCH_PAD1_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD1_HOLD_M,
    RTCIO_TOUCH_PAD1_DRV_V,
    RTCIO_TOUCH_PAD1_DRV_S,
    RTCIO_CHANNEL_1_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD2_REG,
    RTCIO_TOUCH_PAD2_MUX_SEL_M,
    RTCIO_TOUCH_PAD2_FUN_SEL_S,
    RTCIO_TOUCH_PAD2_FUN_IE_M,
    RTCIO_TOUCH_PAD2_RUE_M,
    RTCIO_TOUCH_PAD2_RDE_M,
    RTCIO_TOUCH_PAD2_SLP_SEL_M,
    RTCIO_TOUCH_PAD2_SLP_IE_M,
    RTCIO_TOUCH_PAD2_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD2_HOLD_M,
    RTCIO_TOUCH_PAD2_DRV_V,
    RTCIO_TOUCH_PAD2_DRV_S,
    RTCIO_CHANNEL_2_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD3_REG,
    RTCIO_TOUCH_PAD3_MUX_SEL_M,
    RTCIO_TOUCH_PAD3_FUN_SEL_S,
    RTCIO_TOUCH_PAD3_FUN_IE_M,
    RTCIO_TOUCH_PAD3_RUE_M,
    RTCIO_TOUCH_PAD3_RDE_M,
    RTCIO_TOUCH_PAD3_SLP_SEL_M,
    RTCIO_TOUCH_PAD3_SLP_IE_M,
    RTCIO_TOUCH_PAD3_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD3_HOLD_M,
    RTCIO_TOUCH_PAD3_DRV_V,
    RTCIO_TOUCH_PAD3_DRV_S,
    RTCIO_CHANNEL_3_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD4_REG,
    RTCIO_TOUCH_PAD4_MUX_SEL_M,
    RTCIO_TOUCH_PAD4_FUN_SEL_S,
    RTCIO_TOUCH_PAD4_FUN_IE_M,
    RTCIO_TOUCH_PAD4_RUE_M,
    RTCIO_TOUCH_PAD4_RDE_M,
    RTCIO_TOUCH_PAD4_SLP_SEL_M,
    RTCIO_TOUCH_PAD4_SLP_IE_M,
    RTCIO_TOUCH_PAD4_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD4_HOLD_M,
    RTCIO_TOUCH_PAD4_DRV_V,
    RTCIO_TOUCH_PAD4_DRV_S,
    RTCIO_CHANNEL_4_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD5_REG,
    RTCIO_TOUCH_PAD5_MUX_SEL_M,
    RTCIO_TOUCH_PAD5_FUN_SEL_S,
    RTCIO_TOUCH_PAD5_FUN_IE_M,
    RTCIO_TOUCH_PAD5_RUE_M,
    RTCIO_TOUCH_PAD5_RDE_M,
    RTCIO_TOUCH_PAD5_SLP_SEL_M,
    RTCIO_TOUCH_PAD5_SLP_IE_M,
    RTCIO_TOUCH_PAD5_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD5_HOLD_M,
    RTCIO_TOUCH_PAD5_DRV_V,
    RTCIO_TOUCH_PAD5_DRV_S,
    RTCIO_CHANNEL_5_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD6_REG,
    RTCIO_TOUCH_PAD6_MUX_SEL_M,
    RTCIO_TOUCH_PAD6_FUN_SEL_S,
    RTCIO_TOUCH_PAD6_FUN_IE_M,
    RTCIO_TOUCH_PAD6_RUE_M,
    RTCIO_TOUCH_PAD6_RDE_M,
    RTCIO_TOUCH_PAD6_SLP_SEL_M,
    RTCIO_TOUCH_PAD6_SLP_IE_M,
    RTCIO_TOUCH_PAD6_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD6_HOLD_M,
    RTCIO_TOUCH_PAD6_DRV_V,
    RTCIO_TOUCH_PAD6_DRV_S,
    RTCIO_CHANNEL_6_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD7_REG,
    RTCIO_TOUCH_PAD7_MUX_SEL_M,
    RTCIO_TOUCH_PAD7_FUN_SEL_S,
    RTCIO_TOUCH_PAD7_FUN_IE_M,
    RTCIO_TOUCH_PAD7_RUE_M,
    RTCIO_TOUCH_PAD7_RDE_M,
    RTCIO_TOUCH_PAD7_SLP_SEL_M,
    RTCIO_TOUCH_PAD7_SLP_IE_M,
    RTCIO_TOUCH_PAD7_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD7_HOLD_M,
    RTCIO_TOUCH_PAD7_DRV_V,
    RTCIO_TOUCH_PAD7_DRV_S,
    RTCIO_CHANNEL_7_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD8_REG,
    RTCIO_TOUCH_PAD8_MUX_SEL_M,
    RTCIO_TOUCH_PAD8_FUN_SEL_S,
    RTCIO_TOUCH_PAD8_FUN_IE_M,
    RTCIO_TOUCH_PAD8_RUE_M,
    RTCIO_TOUCH_PAD8_RDE_M,
    RTCIO_TOUCH_PAD8_SLP_SEL_M,
    RTCIO_TOUCH_PAD8_SLP_IE_M,
    RTCIO_TOUCH_PAD8_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD8_HOLD_M,
    RTCIO_TOUCH_PAD8_DRV_V,
    RTCIO_TOUCH_PAD8_DRV_S,
    RTCIO_CHANNEL_8_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD9_REG,
    RTCIO_TOUCH_PAD9_MUX_SEL_M,
    RTCIO_TOUCH_PAD9_FUN_SEL_S,
    RTCIO_TOUCH_PAD9_FUN_IE_M,
    RTCIO_TOUCH_PAD9_RUE_M,
    RTCIO_TOUCH_PAD9_RDE_M,
    RTCIO_TOUCH_PAD9_SLP_SEL_M,
    RTCIO_TOUCH_PAD9_SLP_IE_M,
    RTCIO_TOUCH_PAD9_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD9_HOLD_M,
    RTCIO_TOUCH_PAD9_DRV_V,
    RTCIO_TOUCH_PAD9_DRV_S,
    RTCIO_CHANNEL_9_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD10_REG,
    RTCIO_TOUCH_PAD10_MUX_SEL_M,
    RTCIO_TOUCH_PAD10_FUN_SEL_S,
    RTCIO_TOUCH_PAD10_FUN_IE_M,
    RTCIO_TOUCH_PAD10_RUE_M,
    RTCIO_TOUCH_PAD10_RDE_M,
    RTCIO_TOUCH_PAD10_SLP_SEL_M,
    RTCIO_TOUCH_PAD10_SLP_IE_M,
    RTCIO_TOUCH_PAD10_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD10_HOLD_M,
    RTCIO_TOUCH_PAD10_DRV_V,
    RTCIO_TOUCH_PAD10_DRV_S,
    RTCIO_CHANNEL_10_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD11_REG,
    RTCIO_TOUCH_PAD11_MUX_SEL_M,
    RTCIO_TOUCH_PAD11_FUN_SEL_S,
    RTCIO_TOUCH_PAD11_FUN_IE_M,
    RTCIO_TOUCH_PAD11_RUE_M,
    RTCIO_TOUCH_PAD11_RDE_M,
    RTCIO_TOUCH_PAD11_SLP_SEL_M,
    RTCIO_TOUCH_PAD11_SLP_IE_M,
    RTCIO_TOUCH_PAD11_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD11_HOLD_M,
    RTCIO_TOUCH_PAD11_DRV_V,
    RTCIO_TOUCH_PAD11_DRV_S,
    RTCIO_CHANNEL_11_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD12_REG,
    RTCIO_TOUCH_PAD12_MUX_SEL_M,
    RTCIO_TOUCH_PAD12_FUN_SEL_S,
    RTCIO_TOUCH_PAD12_FUN_IE_M,
    RTCIO_TOUCH_PAD12_RUE_M,
    RTCIO_TOUCH_PAD12_RDE_M,
    RTCIO_TOUCH_PAD12_SLP_SEL_M,
    RTCIO_TOUCH_PAD12_SLP_IE_M,
    RTCIO_TOUCH_PAD12_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD12_HOLD_M,
    RTCIO_TOUCH_PAD12_DRV_V,
    RTCIO_TOUCH_PAD12_DRV_S,
    RTCIO_CHANNEL_12_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD13_REG,
    RTCIO_TOUCH_PAD13_MUX_SEL_M,
    RTCIO_TOUCH_PAD13_FUN_SEL_S,
    RTCIO_TOUCH_PAD13_FUN_IE_M,
    RTCIO_TOUCH_PAD13_RUE_M,
    RTCIO_TOUCH_PAD13_RDE_M,
    RTCIO_TOUCH_PAD13_SLP_SEL_M,
    RTCIO_TOUCH_PAD13_SLP_IE_M,
    RTCIO_TOUCH_PAD13_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD13_HOLD_M,
    RTCIO_TOUCH_PAD13_DRV_V,
    RTCIO_TOUCH_PAD13_DRV_S,
    RTCIO_CHANNEL_13_GPIO_NUM
  },
  {
    RTCIO_TOUCH_PAD14_REG,
    RTCIO_TOUCH_PAD14_MUX_SEL_M,
    RTCIO_TOUCH_PAD14_FUN_SEL_S,
    RTCIO_TOUCH_PAD14_FUN_IE_M,
    RTCIO_TOUCH_PAD14_RUE_M,
    RTCIO_TOUCH_PAD14_RDE_M,
    RTCIO_TOUCH_PAD14_SLP_SEL_M,
    RTCIO_TOUCH_PAD14_SLP_IE_M,
    RTCIO_TOUCH_PAD14_SLP_OE_M,
    0,
    RTC_CNTL_TOUCH_PAD14_HOLD_M,
    RTCIO_TOUCH_PAD14_DRV_V,
    RTCIO_TOUCH_PAD14_DRV_S,
    RTCIO_CHANNEL_14_GPIO_NUM
  },
  {
    RTCIO_XTAL_32P_PAD_REG,
    RTCIO_X32P_MUX_SEL_M,
    RTCIO_X32P_FUN_SEL_S,
    RTCIO_X32P_FUN_IE_M,
    RTCIO_X32P_RUE_M,
    RTCIO_X32P_RDE_M,
    RTCIO_X32P_SLP_SEL_M,
    RTCIO_X32P_SLP_IE_M,
    RTCIO_X32P_SLP_OE_M,
    0,
    RTC_CNTL_X32P_HOLD_M,
    RTCIO_X32P_DRV_V,
    RTCIO_X32P_DRV_S,
    RTCIO_CHANNEL_15_GPIO_NUM
  },
  {
    RTCIO_XTAL_32N_PAD_REG,
    RTCIO_X32N_MUX_SEL_M,
    RTCIO_X32N_FUN_SEL_S,
    RTCIO_X32N_FUN_IE_M,
    RTCIO_X32N_RUE_M,
    RTCIO_X32N_RDE_M,
    RTCIO_X32N_SLP_SEL_M,
    RTCIO_X32N_SLP_IE_M,
    RTCIO_X32N_SLP_OE_M,
    0,
    RTC_CNTL_X32N_HOLD_M,
    RTCIO_X32N_DRV_V,
    RTCIO_X32N_DRV_S,
    RTCIO_CHANNEL_16_GPIO_NUM
  },
  {
    RTCIO_PAD_DAC1_REG,
    RTCIO_PDAC1_MUX_SEL_M,
    RTCIO_PDAC1_FUN_SEL_S,
    RTCIO_PDAC1_FUN_IE_M,
    RTCIO_PDAC1_RUE_M,
    RTCIO_PDAC1_RDE_M,
    RTCIO_PDAC1_SLP_SEL_M,
    RTCIO_PDAC1_SLP_IE_M,
    RTCIO_PDAC1_SLP_OE_M,
    0,
    RTC_CNTL_PDAC1_HOLD_M,
    RTCIO_PDAC1_DRV_V,
    RTCIO_PDAC1_DRV_S,
    RTCIO_CHANNEL_17_GPIO_NUM
  },
  {
    RTCIO_PAD_DAC2_REG,
    RTCIO_PDAC2_MUX_SEL_M,
    RTCIO_PDAC2_FUN_SEL_S,
    RTCIO_PDAC2_FUN_IE_M,
    RTCIO_PDAC2_RUE_M,
    RTCIO_PDAC2_RDE_M,
    RTCIO_PDAC2_SLP_SEL_M,
    RTCIO_PDAC2_SLP_IE_M,
    RTCIO_PDAC2_SLP_OE_M,
    0,
    RTC_CNTL_PDAC2_HOLD_M,
    RTCIO_PDAC2_DRV_V,
    RTCIO_PDAC2_DRV_S,
    RTCIO_CHANNEL_18_GPIO_NUM
  },
  {
    RTCIO_RTC_PAD19_REG,
    RTCIO_RTC_PAD19_MUX_SEL_M,
    RTCIO_RTC_PAD19_FUN_SEL_S,
    RTCIO_RTC_PAD19_FUN_IE_M,
    RTCIO_RTC_PAD19_RUE_M,
    RTCIO_RTC_PAD19_RDE_M,
    RTCIO_RTC_PAD19_SLP_SEL_M,
    RTCIO_RTC_PAD19_SLP_IE_M,
    RTCIO_RTC_PAD19_SLP_OE_M,
    0,
    RTC_CNTL_RTC_PAD19_HOLD_M,
    RTCIO_RTC_PAD19_DRV_V,
    RTCIO_RTC_PAD19_DRV_S,
    RTCIO_CHANNEL_19_GPIO_NUM
  },
  {
    RTCIO_RTC_PAD20_REG,
    RTCIO_RTC_PAD20_MUX_SEL_M,
    RTCIO_RTC_PAD20_FUN_SEL_S,
    RTCIO_RTC_PAD20_FUN_IE_M,
    RTCIO_RTC_PAD20_RUE_M,
    RTCIO_RTC_PAD20_RDE_M,
    RTCIO_RTC_PAD20_SLP_SEL_M,
    RTCIO_RTC_PAD20_SLP_IE_M,
    RTCIO_RTC_PAD20_SLP_OE_M,
    0,
    RTC_CNTL_RTC_PAD20_HOLD_M,
    RTCIO_RTC_PAD20_DRV_V,
    RTCIO_RTC_PAD20_DRV_S,
    RTCIO_CHANNEL_20_GPIO_NUM
  },
  {
    RTCIO_RTC_PAD21_REG,
    RTCIO_RTC_PAD21_MUX_SEL_M,
    RTCIO_RTC_PAD21_FUN_SEL_S,
    RTCIO_RTC_PAD21_FUN_IE_M,
    RTCIO_RTC_PAD21_RUE_M,
    RTCIO_RTC_PAD21_RDE_M,
    RTCIO_RTC_PAD21_SLP_SEL_M,
    RTCIO_RTC_PAD21_SLP_IE_M,
    RTCIO_RTC_PAD21_SLP_OE_M,
    0,
    RTC_CNTL_RTC_PAD21_HOLD_M,
    RTCIO_RTC_PAD21_DRV_V,
    RTCIO_RTC_PAD21_DRV_S,
    RTCIO_CHANNEL_21_GPIO_NUM
  }
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_configrtcio
 *
 * Description:
 *   Configure a RTCIO pin based on encoded pin attributes.
 *
 * Input Parameters:
 *   rtcio_num     - RTCIO pin to be configured.
 *   attr          - Attributes to be configured for the selected RTCIO pin.
 *                   The following attributes are accepted:
 *                   - Direction (OUTPUT or INPUT)
 *                   - Pull (PULLUP, PULLDOWN or OPENDRAIN)
 *                   - Function (if not provided, assume function RTCIO by
 *                     default)
 *                   - Drive strength (if not provided, assume DRIVE_2 by
 *                     default)
 *
 * Returned Value:
 *   Zero (OK) on success, or -1 (ERROR) in case of failure.
 *
 ****************************************************************************/

int esp32s3_configrtcio(int rtcio_num, rtcio_pinattr_t attr);

/****************************************************************************
 * Name: esp32s3_rtcioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   RTC IRQs.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_RTCIO_IRQ
void esp32s3_rtcioirqinitialize(void);
#else
#  define esp32s3_rtcioirqinitialize()
#endif

/****************************************************************************
 * Name: esp32s3_rtcioirqenable
 *
 * Description:
 *   Enable the interrupt for the specified RTC peripheral IRQ
 *
 * Input Parameters:
 *   irq - The IRQ number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_RTCIO_IRQ
void esp32s3_rtcioirqenable(int irq);
#else
#  define esp32s3_rtcioirqenable(irq)
#endif

/****************************************************************************
 * Name: esp32s3_rtcioirqdisable
 *
 * Description:
 *   Disable the interrupt for the specified RTC peripheral IRQ.
 *
 * Input Parameters:
 *   irq - The IRQ number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_RTCIO_IRQ
void esp32s3_rtcioirqdisable(int irq);
#else
#  define esp32s3_rtcioirqdisable(irq)
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_RTC_GPIO_H */
