/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_touch.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_TOUCH_H
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_TOUCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-preprocessor Definitions
 ****************************************************************************/

#define TOUCH_SENSOR_PINS           10
#define TOUCH_MEASURE_WAIT_MAX      (0xff)

/* Touch channel to GPIO */

#define TOUCH_PAD_NUM0_GPIO_NUM     4
#define TOUCH_PAD_NUM1_GPIO_NUM     0
#define TOUCH_PAD_NUM2_GPIO_NUM     2
#define TOUCH_PAD_NUM3_GPIO_NUM     15
#define TOUCH_PAD_NUM4_GPIO_NUM     13
#define TOUCH_PAD_NUM5_GPIO_NUM     12
#define TOUCH_PAD_NUM6_GPIO_NUM     14
#define TOUCH_PAD_NUM7_GPIO_NUM     27
#define TOUCH_PAD_NUM8_GPIO_NUM     33
#define TOUCH_PAD_NUM9_GPIO_NUM     32

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Touch pad channel */

enum touch_pad_e
{
  TOUCH_PAD_NUM0,     /* Touch pad channel 0 is GPIO4  */
  TOUCH_PAD_NUM1,     /* Touch pad channel 1 is GPIO0  */
  TOUCH_PAD_NUM2,     /* Touch pad channel 2 is GPIO2  */
  TOUCH_PAD_NUM3,     /* Touch pad channel 3 is GPIO15 */
  TOUCH_PAD_NUM4,     /* Touch pad channel 4 is GPIO13 */
  TOUCH_PAD_NUM5,     /* Touch pad channel 5 is GPIO12 */
  TOUCH_PAD_NUM6,     /* Touch pad channel 6 is GPIO14 */
  TOUCH_PAD_NUM7,     /* Touch pad channel 7 is GPIO27 */
  TOUCH_PAD_NUM8,     /* Touch pad channel 8 is GPIO33 */
  TOUCH_PAD_NUM9      /* Touch pad channel 9 is GPIO32 */
};

/* Touch sensor high reference voltage */

enum touch_high_volt_e
{
  TOUCH_HVOLT_2V4,    /* Touch sensor high reference voltage, 2.4V  */
  TOUCH_HVOLT_2V5,    /* Touch sensor high reference voltage, 2.5V  */
  TOUCH_HVOLT_2V6,    /* Touch sensor high reference voltage, 2.6V  */
  TOUCH_HVOLT_2V7     /* Touch sensor high reference voltage, 2.7V  */
};

/* Touch sensor low reference voltage */

enum touch_low_volt_e
{
  TOUCH_LVOLT_0V5,    /* Touch sensor low reference voltage, 0.5V  */
  TOUCH_LVOLT_0V6,    /* Touch sensor low reference voltage, 0.6V  */
  TOUCH_LVOLT_0V7,    /* Touch sensor low reference voltage, 0.7V  */
  TOUCH_LVOLT_0V8     /* Touch sensor low reference voltage, 0.8V  */
};

/* Touch sensor high reference voltage attenuation */

enum touch_volt_atten_e
{
  TOUCH_HVOLT_ATTEN_1V5,  /* 1.5V attenuation  */
  TOUCH_HVOLT_ATTEN_1V,   /* 1.0V attenuation  */
  TOUCH_HVOLT_ATTEN_0V5,  /* 0.5V attenuation  */
  TOUCH_HVOLT_ATTEN_0V    /*   0V attenuation  */
};

/* Touch sensor charge/discharge speed */

enum touch_cnt_slope_e
{
  TOUCH_SLOPE_0,       /* Touch sensor charge/discharge speed, always zero */
  TOUCH_SLOPE_1,       /* Touch sensor charge/discharge speed, slowest     */
  TOUCH_SLOPE_2,       /* Touch sensor charge/discharge speed              */
  TOUCH_SLOPE_3,       /* Touch sensor charge/discharge speed              */
  TOUCH_SLOPE_4,       /* Touch sensor charge/discharge speed              */
  TOUCH_SLOPE_5,       /* Touch sensor charge/discharge speed              */
  TOUCH_SLOPE_6,       /* Touch sensor charge/discharge speed              */
  TOUCH_SLOPE_7        /* Touch sensor charge/discharge speed, fast        */
};

/* Touch sensor initial charge level */

enum touch_tie_opt_e
{
  TOUCH_TIE_OPT_LOW,    /* Initial level of charging voltage, low level */
  TOUCH_TIE_OPT_HIGH    /* Initial level of charging voltage, high level */
};

/* Touch sensor FSM mode */

enum touch_fsm_mode_e
{
  TOUCH_FSM_MODE_TIMER,     /* To start touch FSM by timer */
  TOUCH_FSM_MODE_SW         /* To start touch FSM by software trigger */
};

/* Touch sensor touch IRQ trigger */

enum touch_trigger_mode_e
{
  TOUCH_TRIGGER_BELOW,     /* Interrupt if value is less than threshold. */
  TOUCH_TRIGGER_ABOVE      /* Interrupt if value is larger than threshold. */
};

/* Touch sensor wakeup IRQ trigger */

enum touch_trigger_src_e
{
  TOUCH_TRIGGER_SOURCE_BOTH,  /* Interrupt if SET1 and SET2 are "touched" */
  TOUCH_TRIGGER_SOURCE_SET1   /* Interrupt if SET1 is "touched" */
};

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_TOUCH_H */
