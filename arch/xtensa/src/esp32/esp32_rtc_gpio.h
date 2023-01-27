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

#include "hardware/esp32_gpio.h"
#include "hardware/esp32_rtc_io.h"
#include "hardware/esp32_rtccntl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEFAULT_RTC_PU_BIT          (1 << 27)
#define DEFAULT_RTC_PD_BIT          (1 << 28)
#define SPECIAL_RTC_PU_BIT          (1 << 22)
#define SPECIAL_RTC_PD_BIT          (1 << 23)

#define RTCIO_PIN_FUNC                  0

#define RTC_MODE_SHIFT                  0
#define RTC_MODE_MASK                   (3 << RTC_MODE_SHIFT)
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

#define RTC_GPIO_NUMBER             18

#define RTCIO_GPIO36_CHANNEL        0    /* RTCIO_CHANNEL_0 */
#define RTCIO_CHANNEL_0_GPIO_NUM    36

#define RTCIO_GPIO37_CHANNEL        1    /* RTCIO_CHANNEL_1 */
#define RTCIO_CHANNEL_1_GPIO_NUM    37

#define RTCIO_GPIO38_CHANNEL        2    /* RTCIO_CHANNEL_2 */
#define RTCIO_CHANNEL_2_GPIO_NUM    38

#define RTCIO_GPIO39_CHANNEL        3    /* RTCIO_CHANNEL_3 */
#define RTCIO_CHANNEL_3_GPIO_NUM    39

#define RTCIO_GPIO34_CHANNEL        4    /* RTCIO_CHANNEL_4 */
#define RTCIO_CHANNEL_4_GPIO_NUM    34

#define RTCIO_GPIO35_CHANNEL        5    /* RTCIO_CHANNEL_5 */
#define RTCIO_CHANNEL_5_GPIO_NUM    35

#define RTCIO_GPIO25_CHANNEL        6    /* RTCIO_CHANNEL_6 */
#define RTCIO_CHANNEL_6_GPIO_NUM    25

#define RTCIO_GPIO26_CHANNEL        7    /* RTCIO_CHANNEL_7 */
#define RTCIO_CHANNEL_7_GPIO_NUM    26

#define RTCIO_GPIO33_CHANNEL        8    /* RTCIO_CHANNEL_8 */
#define RTCIO_CHANNEL_8_GPIO_NUM    33

#define RTCIO_GPIO32_CHANNEL        9    /* RTCIO_CHANNEL_9 */
#define RTCIO_CHANNEL_9_GPIO_NUM    32

#define RTCIO_GPIO4_CHANNEL         10   /* RTCIO_CHANNEL_10 */
#define RTCIO_CHANNEL_10_GPIO_NUM   4

#define RTCIO_GPIO0_CHANNEL         11   /* RTCIO_CHANNEL_11 */
#define RTCIO_CHANNEL_11_GPIO_NUM   0

#define RTCIO_GPIO2_CHANNEL         12   /* RTCIO_CHANNEL_12 */
#define RTCIO_CHANNEL_12_GPIO_NUM   2

#define RTCIO_GPIO15_CHANNEL        13   /* RTCIO_CHANNEL_13 */
#define RTCIO_CHANNEL_13_GPIO_NUM   15

#define RTCIO_GPIO13_CHANNEL        14   /* RTCIO_CHANNEL_14 */
#define RTCIO_CHANNEL_14_GPIO_NUM   13

#define RTCIO_GPIO12_CHANNEL        15   /* RTCIO_CHANNEL_15 */
#define RTCIO_CHANNEL_15_GPIO_NUM   12

#define RTCIO_GPIO14_CHANNEL        16   /* RTCIO_CHANNEL_16 */
#define RTCIO_CHANNEL_16_GPIO_NUM   14

#define RTCIO_GPIO27_CHANNEL        17   /* RTCIO_CHANNEL_17 */
#define RTCIO_CHANNEL_17_GPIO_NUM   27

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct rtc_io_desc_s
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
} rtc_io_desc_t;

/* Must be big enough to hold the pin attributes */

typedef uint16_t rtcio_pinattr_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

static const int g_gpio_to_rtcio_map[GPIO_PIN_COUNT + 1] =
{
  RTCIO_GPIO0_CHANNEL,        /* GPIO0 */
  -1,                         /* GPIO1 not supported */
  RTCIO_GPIO2_CHANNEL,        /* GPIO2 */
  -1,                         /* GPIO3 not supported */
  RTCIO_GPIO4_CHANNEL,        /* GPIO4 */
  -1,                         /* GPIO5 not supported */
  -1,                         /* GPIO6 not supported */
  -1,                         /* GPIO7 not supported */
  -1,                         /* GPIO8 not supported */
  -1,                         /* GPIO9 not supported */
  -1,                         /* GPIO10 not supported */
  -1,                         /* GPIO11 not supported */
  RTCIO_GPIO12_CHANNEL,       /* GPIO12 */
  RTCIO_GPIO13_CHANNEL,       /* GPIO13 */
  RTCIO_GPIO14_CHANNEL,       /* GPIO14 */
  RTCIO_GPIO15_CHANNEL,       /* GPIO15 */
  -1,                         /* GPIO16 not supported */
  -1,                         /* GPIO17 not supported */
  -1,                         /* GPIO18 not supported */
  -1,                         /* GPIO19 not supported */
  -1,                         /* GPIO20 not supported */
  -1,                         /* GPIO21 not supported */
  -1,                         /* GPIO22 not supported */
  -1,                         /* GPIO23 not supported */
  -1,                         /* GPIO24 not supported */
  RTCIO_GPIO25_CHANNEL,       /* GPIO25 */
  RTCIO_GPIO26_CHANNEL,       /* GPIO26 */
  RTCIO_GPIO27_CHANNEL,       /* GPIO27 */
  -1,                         /* GPIO28 not supported */
  -1,                         /* GPIO29 not supported */
  -1,                         /* GPIO30 not supported */
  -1,                         /* GPIO31 not supported */
  RTCIO_GPIO32_CHANNEL,       /* GPIO32 */
  RTCIO_GPIO33_CHANNEL,       /* GPIO33 */
  RTCIO_GPIO34_CHANNEL,       /* GPIO34 */
  RTCIO_GPIO35_CHANNEL,       /* GPIO35 */
  RTCIO_GPIO36_CHANNEL,       /* GPIO36 */
  RTCIO_GPIO37_CHANNEL,       /* GPIO37 */
  RTCIO_GPIO38_CHANNEL,       /* GPIO38 */
  RTCIO_GPIO39_CHANNEL        /* GPIO39 */
};

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
    RTC_IO_SENSOR_PADS_REG,
    RTC_IO_SENSE1_MUX_SEL_M,
    RTC_IO_SENSE1_FUN_SEL_S,
    RTC_IO_SENSE1_FUN_IE_M,
    0,
    0,
    RTC_IO_SENSE1_SLP_SEL_M,
    RTC_IO_SENSE1_SLP_IE_M,
    0,
    RTC_IO_SENSE1_HOLD_M,
    RTC_CNTL_SENSE1_HOLD_FORCE_M,
    0,
    0,
    RTCIO_CHANNEL_0_GPIO_NUM
  },
  {
    RTC_IO_SENSOR_PADS_REG,
    RTC_IO_SENSE2_MUX_SEL_M,
    RTC_IO_SENSE2_FUN_SEL_S,
    RTC_IO_SENSE2_FUN_IE_M,
    0,
    0,
    RTC_IO_SENSE2_SLP_SEL_M,
    RTC_IO_SENSE2_SLP_IE_M,
    0,
    RTC_IO_SENSE2_HOLD_M,
    RTC_CNTL_SENSE2_HOLD_FORCE_M,
    0,
    0,
    RTCIO_CHANNEL_1_GPIO_NUM
  },
  {
    RTC_IO_SENSOR_PADS_REG,
    RTC_IO_SENSE3_MUX_SEL_M,
    RTC_IO_SENSE3_FUN_SEL_S,
    RTC_IO_SENSE3_FUN_IE_M,
    0,
    0,
    RTC_IO_SENSE3_SLP_SEL_M,
    RTC_IO_SENSE3_SLP_IE_M,
    0,
    RTC_IO_SENSE3_HOLD_M,
    RTC_CNTL_SENSE3_HOLD_FORCE_M,
    0,
    0,
    RTCIO_CHANNEL_2_GPIO_NUM
  },
  {
    RTC_IO_SENSOR_PADS_REG,
    RTC_IO_SENSE4_MUX_SEL_M,
    RTC_IO_SENSE4_FUN_SEL_S,
    RTC_IO_SENSE4_FUN_IE_M,
    0,
    0,
    RTC_IO_SENSE4_SLP_SEL_M,
    RTC_IO_SENSE4_SLP_IE_M,
    0,
    RTC_IO_SENSE4_HOLD_M,
    RTC_CNTL_SENSE4_HOLD_FORCE_M,
    0,
    0,
    RTCIO_CHANNEL_3_GPIO_NUM
  },
  {
    RTC_IO_ADC_PAD_REG,
    RTC_IO_ADC1_MUX_SEL_M,
    RTC_IO_ADC1_FUN_SEL_S,
    RTC_IO_ADC1_FUN_IE_M,
    0,
    0,
    RTC_IO_ADC1_SLP_SEL_M,
    RTC_IO_ADC1_SLP_IE_M,
    0,
    RTC_IO_ADC1_HOLD_M,
    RTC_CNTL_ADC1_HOLD_FORCE_M,
    0,
    0,
    RTCIO_CHANNEL_4_GPIO_NUM
  },
  {
    RTC_IO_ADC_PAD_REG,
    RTC_IO_ADC2_MUX_SEL_M,
    RTC_IO_ADC2_FUN_SEL_S,
    RTC_IO_ADC2_FUN_IE_M,
    0,
    0,
    RTC_IO_ADC2_SLP_SEL_M,
    RTC_IO_ADC2_SLP_IE_M,
    0,
    RTC_IO_ADC2_HOLD_M,
    RTC_CNTL_ADC2_HOLD_FORCE_M,
    0,
    0,
    RTCIO_CHANNEL_5_GPIO_NUM
  },
  {
    RTC_IO_PAD_DAC1_REG,
    RTC_IO_PDAC1_MUX_SEL_M,
    RTC_IO_PDAC1_FUN_SEL_S,
    RTC_IO_PDAC1_FUN_IE_M,
    RTC_IO_PDAC1_RUE_M,
    RTC_IO_PDAC1_RDE_M,
    RTC_IO_PDAC1_SLP_SEL_M,
    RTC_IO_PDAC1_SLP_IE_M,
    0,
    RTC_IO_PDAC1_HOLD_M,
    RTC_CNTL_PDAC1_HOLD_FORCE_M,
    RTC_IO_PDAC1_DRV_V,
    RTC_IO_PDAC1_DRV_S,
    RTCIO_CHANNEL_6_GPIO_NUM
  },
  {
    RTC_IO_PAD_DAC2_REG,
    RTC_IO_PDAC2_MUX_SEL_M,
    RTC_IO_PDAC2_FUN_SEL_S,
    RTC_IO_PDAC2_FUN_IE_M,
    RTC_IO_PDAC2_RUE_M,
    RTC_IO_PDAC2_RDE_M,
    RTC_IO_PDAC2_SLP_SEL_M,
    RTC_IO_PDAC2_SLP_IE_M,
    0,
    RTC_IO_PDAC2_HOLD_M,
    RTC_CNTL_PDAC2_HOLD_FORCE_M,
    RTC_IO_PDAC2_DRV_V,
    RTC_IO_PDAC2_DRV_S,
    RTCIO_CHANNEL_7_GPIO_NUM
  },
  {
    RTC_IO_XTAL_32K_PAD_REG,
    RTC_IO_X32N_MUX_SEL_M,
    RTC_IO_X32N_FUN_SEL_S,
    RTC_IO_X32N_FUN_IE_M,
    RTC_IO_X32N_RUE_M,
    RTC_IO_X32N_RDE_M,
    RTC_IO_X32N_SLP_SEL_M,
    RTC_IO_X32N_SLP_IE_M,
    0,
    RTC_IO_X32N_HOLD_M,
    RTC_CNTL_X32N_HOLD_FORCE_M,
    RTC_IO_X32N_DRV_V,
    RTC_IO_X32N_DRV_S,
    RTCIO_CHANNEL_8_GPIO_NUM
  },
  {
    RTC_IO_XTAL_32K_PAD_REG,
    RTC_IO_X32P_MUX_SEL_M,
    RTC_IO_X32P_FUN_SEL_S,
    RTC_IO_X32P_FUN_IE_M,
    RTC_IO_X32P_RUE_M,
    RTC_IO_X32P_RDE_M,
    RTC_IO_X32P_SLP_SEL_M,
    RTC_IO_X32P_SLP_IE_M,
    0,
    RTC_IO_X32P_HOLD_M,
    RTC_CNTL_X32P_HOLD_FORCE_M,
    RTC_IO_X32P_DRV_V,
    RTC_IO_X32P_DRV_S,
    RTCIO_CHANNEL_9_GPIO_NUM
  },
  {
    RTC_IO_TOUCH_PAD0_REG,
    RTC_IO_TOUCH_PAD0_MUX_SEL_M,
    RTC_IO_TOUCH_PAD0_FUN_SEL_S,
    RTC_IO_TOUCH_PAD0_FUN_IE_M,
    RTC_IO_TOUCH_PAD0_RUE_M,
    RTC_IO_TOUCH_PAD0_RDE_M,
    RTC_IO_TOUCH_PAD0_SLP_SEL_M,
    RTC_IO_TOUCH_PAD0_SLP_IE_M,
    0,
    RTC_IO_TOUCH_PAD0_HOLD_M,
    RTC_CNTL_TOUCH_PAD0_HOLD_FORCE_M,
    RTC_IO_TOUCH_PAD0_DRV_V,
    RTC_IO_TOUCH_PAD0_DRV_S,
    RTCIO_CHANNEL_10_GPIO_NUM
  },
  {
    RTC_IO_TOUCH_PAD1_REG,
    RTC_IO_TOUCH_PAD1_MUX_SEL_M,
    RTC_IO_TOUCH_PAD1_FUN_SEL_S,
    RTC_IO_TOUCH_PAD1_FUN_IE_M,
    RTC_IO_TOUCH_PAD1_RUE_M,
    RTC_IO_TOUCH_PAD1_RDE_M,
    RTC_IO_TOUCH_PAD1_SLP_SEL_M,
    RTC_IO_TOUCH_PAD1_SLP_IE_M,
    0,
    RTC_IO_TOUCH_PAD1_HOLD_M,
    RTC_CNTL_TOUCH_PAD1_HOLD_FORCE_M,
    RTC_IO_TOUCH_PAD1_DRV_V,
    RTC_IO_TOUCH_PAD1_DRV_S,
    RTCIO_CHANNEL_11_GPIO_NUM
  },
  {
    RTC_IO_TOUCH_PAD2_REG,
    RTC_IO_TOUCH_PAD2_MUX_SEL_M,
    RTC_IO_TOUCH_PAD2_FUN_SEL_S,
    RTC_IO_TOUCH_PAD2_FUN_IE_M,
    RTC_IO_TOUCH_PAD2_RUE_M,
    RTC_IO_TOUCH_PAD2_RDE_M,
    RTC_IO_TOUCH_PAD2_SLP_SEL_M,
    RTC_IO_TOUCH_PAD2_SLP_IE_M,
    0,
    RTC_IO_TOUCH_PAD2_HOLD_M,
    RTC_CNTL_TOUCH_PAD2_HOLD_FORCE_M,
    RTC_IO_TOUCH_PAD2_DRV_V,
    RTC_IO_TOUCH_PAD2_DRV_S,
    RTCIO_CHANNEL_12_GPIO_NUM
  },
  {
    RTC_IO_TOUCH_PAD3_REG,
    RTC_IO_TOUCH_PAD3_MUX_SEL_M,
    RTC_IO_TOUCH_PAD3_FUN_SEL_S,
    RTC_IO_TOUCH_PAD3_FUN_IE_M,
    RTC_IO_TOUCH_PAD3_RUE_M,
    RTC_IO_TOUCH_PAD3_RDE_M,
    RTC_IO_TOUCH_PAD3_SLP_SEL_M,
    RTC_IO_TOUCH_PAD3_SLP_IE_M,
    0,
    RTC_IO_TOUCH_PAD3_HOLD_M,
    RTC_CNTL_TOUCH_PAD3_HOLD_FORCE_M,
    RTC_IO_TOUCH_PAD3_DRV_V,
    RTC_IO_TOUCH_PAD3_DRV_S,
    RTCIO_CHANNEL_13_GPIO_NUM
  },
  {
    RTC_IO_TOUCH_PAD4_REG,
    RTC_IO_TOUCH_PAD4_MUX_SEL_M,
    RTC_IO_TOUCH_PAD4_FUN_SEL_S,
    RTC_IO_TOUCH_PAD4_FUN_IE_M,
    RTC_IO_TOUCH_PAD4_RUE_M,
    RTC_IO_TOUCH_PAD4_RDE_M,
    RTC_IO_TOUCH_PAD4_SLP_SEL_M,
    RTC_IO_TOUCH_PAD4_SLP_IE_M,
    0,
    RTC_IO_TOUCH_PAD4_HOLD_M,
    RTC_CNTL_TOUCH_PAD4_HOLD_FORCE_M,
    RTC_IO_TOUCH_PAD4_DRV_V,
    RTC_IO_TOUCH_PAD4_DRV_S,
    RTCIO_CHANNEL_14_GPIO_NUM
  },
  {
    RTC_IO_TOUCH_PAD5_REG,
    RTC_IO_TOUCH_PAD5_MUX_SEL_M,
    RTC_IO_TOUCH_PAD5_FUN_SEL_S,
    RTC_IO_TOUCH_PAD5_FUN_IE_M,
    RTC_IO_TOUCH_PAD5_RUE_M,
    RTC_IO_TOUCH_PAD5_RDE_M,
    RTC_IO_TOUCH_PAD5_SLP_SEL_M,
    RTC_IO_TOUCH_PAD5_SLP_IE_M,
    0,
    RTC_IO_TOUCH_PAD5_HOLD_M,
    RTC_CNTL_TOUCH_PAD5_HOLD_FORCE_M,
    RTC_IO_TOUCH_PAD5_DRV_V,
    RTC_IO_TOUCH_PAD5_DRV_S,
    RTCIO_CHANNEL_15_GPIO_NUM
  },
  {
    RTC_IO_TOUCH_PAD6_REG,
    RTC_IO_TOUCH_PAD6_MUX_SEL_M,
    RTC_IO_TOUCH_PAD6_FUN_SEL_S,
    RTC_IO_TOUCH_PAD6_FUN_IE_M,
    RTC_IO_TOUCH_PAD6_RUE_M,
    RTC_IO_TOUCH_PAD6_RDE_M,
    RTC_IO_TOUCH_PAD6_SLP_SEL_M,
    RTC_IO_TOUCH_PAD6_SLP_IE_M,
    0,
    RTC_IO_TOUCH_PAD6_HOLD_M,
    RTC_CNTL_TOUCH_PAD6_HOLD_FORCE_M,
    RTC_IO_TOUCH_PAD6_DRV_V,
    RTC_IO_TOUCH_PAD6_DRV_S,
    RTCIO_CHANNEL_16_GPIO_NUM
  },
  {
    RTC_IO_TOUCH_PAD7_REG,
    RTC_IO_TOUCH_PAD7_MUX_SEL_M,
    RTC_IO_TOUCH_PAD7_FUN_SEL_S,
    RTC_IO_TOUCH_PAD7_FUN_IE_M,
    RTC_IO_TOUCH_PAD7_RUE_M,
    RTC_IO_TOUCH_PAD7_RDE_M,
    RTC_IO_TOUCH_PAD7_SLP_SEL_M,
    RTC_IO_TOUCH_PAD7_SLP_IE_M,
    0,
    RTC_IO_TOUCH_PAD7_HOLD_M,
    RTC_CNTL_TOUCH_PAD7_HOLD_FORCE_M,
    RTC_IO_TOUCH_PAD7_DRV_V,
    RTC_IO_TOUCH_PAD7_DRV_S,
    RTCIO_CHANNEL_17_GPIO_NUM
  }
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_configrtcio
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

int esp32_configrtcio(int rtcio_num, rtcio_pinattr_t attr);

/****************************************************************************
 * Name: esp32_rtcioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   RTC IRQs.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_RTCIO_IRQ
void esp32_rtcioirqinitialize(void);
#else
#  define esp32_rtcioirqinitialize()
#endif

/****************************************************************************
 * Name: esp32_rtcioirqenable
 *
 * Description:
 *   Enable the interrupt for the specified RTC peripheral IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_RTCIO_IRQ
void esp32_rtcioirqenable(int irq);
#else
#  define esp32_rtcioirqenable(irq)
#endif

/****************************************************************************
 * Name: esp32_rtcioirqdisable
 *
 * Description:
 *   Disable the interrupt for the specified RTC peripheral IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_RTCIO_IRQ
void esp32_rtcioirqdisable(int irq);
#else
#  define esp32_rtcioirqdisable(irq)
#endif

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_RTC_GPIO_H */
