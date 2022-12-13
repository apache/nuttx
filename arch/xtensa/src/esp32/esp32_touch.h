/****************************************************************************
 * arch/xtensa/src/esp32/esp32_touch.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_TOUCH_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_TOUCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp32_touch_lowerhalf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TOUCH_BIT_MASK_ALL                  ((1<<TOUCH_SENSOR_PINS)-1)
#define TOUCH_SLOPE_DEFAULT                 (TOUCH_SLOPE_7)
#define TOUCH_TIE_OPT_DEFAULT               (TOUCH_TIE_OPT_LOW)
#define TOUCH_BIT_MASK_MAX                  (TOUCH_BIT_MASK_ALL)
#define TOUCH_HIGH_VOLTAGE_THRESHOLD        (TOUCH_HVOLT_2V7)
#define TOUCH_LOW_VOLTAGE_THRESHOLD         (TOUCH_LVOLT_0V5)
#define TOUCH_ATTEN_VOLTAGE_THRESHOLD       (TOUCH_HVOLT_ATTEN_0V5)
#define TOUCH_THRESHOLD_MAX                 (0)
#define TOUCH_SLEEP_CYCLE_DEFAULT           (0x1000)
#define TOUCH_MEASURE_CYCLE_DEFAULT         (0x7fff)
#define TOUCH_FSM_MODE_DEFAULT              (TOUCH_FSM_MODE_SW)
#define TOUCH_TRIGGER_MODE_DEFAULT          (TOUCH_TRIGGER_BELOW)
#define TOUCH_TRIGGER_SOURCE_DEFAULT        (TOUCH_TRIGGER_SOURCE_SET1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct touch_config_s
{
  enum touch_high_volt_e refh;
  enum touch_low_volt_e refl;
  enum touch_volt_atten_e atten;
  enum touch_cnt_slope_e slope;
  enum touch_tie_opt_e tie_opt;
  enum touch_fsm_mode_e fsm_mode;
  uint16_t interrupt_threshold;
  uint16_t logic_threshold;
  uint32_t filter_period;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

static const uint8_t touch_channel_to_gpio[] =
{
  TOUCH_PAD_NUM0_GPIO_NUM,
  TOUCH_PAD_NUM1_GPIO_NUM,
  TOUCH_PAD_NUM2_GPIO_NUM,
  TOUCH_PAD_NUM3_GPIO_NUM,
  TOUCH_PAD_NUM4_GPIO_NUM,
  TOUCH_PAD_NUM5_GPIO_NUM,
  TOUCH_PAD_NUM6_GPIO_NUM,
  TOUCH_PAD_NUM7_GPIO_NUM,
  TOUCH_PAD_NUM8_GPIO_NUM,
  TOUCH_PAD_NUM9_GPIO_NUM
};

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_configtouch
 *
 * Description:
 *   Configure a touch pad channel.
 *
 * Input Parameters:
 *   tp - The touch pad channel;
 *   config - The touch pad configuration structure.
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

int esp32_configtouch(enum touch_pad_e tp, struct touch_config_s config);

/****************************************************************************
 * Name: esp32_touchread
 *
 * Description:
 *   Read a touch pad channel.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   0 if touch pad pressed, 1 if released.
 *
 ****************************************************************************/

bool esp32_touchread(enum touch_pad_e tp);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_TOUCH_H */
