/****************************************************************************
 * arch/xtensa/src/esp32s2/hardware/esp32s2_touch.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_TOUCH_H
#define __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_TOUCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-preprocessor Definitions
 ****************************************************************************/

#define TOUCH_SENSOR_PINS           15
#define TOUCH_MEASURE_WAIT_MAX      (0xff)
#define TOUCH_THRESHOLD_NO_USE      (0)

/* The waterproof function includes a shielded channel (TOUCH_PAD_NUM14) */

#define TOUCH_SHIELD_CHANNEL        (14)

/* T0 is an internal channel that does not have a corresponding external
 * GPIO. T0 will work simultaneously with the measured channel Tn. Finally,
 * the actual measured value of Tn is the value after subtracting lower bits
 * of T0.
 */

#define TOUCH_DENOISE_CHANNEL       (0)

/* Touch channel to GPIO */

/* Note: T0 is an internal channel that does not have a corresponding
 * external GPIO.
 */

#define TOUCH_PAD_NUM1_CHANNEL_NUM     1
#define TOUCH_PAD_NUM2_CHANNEL_NUM     2
#define TOUCH_PAD_NUM3_CHANNEL_NUM     3
#define TOUCH_PAD_NUM4_CHANNEL_NUM     4
#define TOUCH_PAD_NUM5_CHANNEL_NUM     5
#define TOUCH_PAD_NUM6_CHANNEL_NUM     6
#define TOUCH_PAD_NUM7_CHANNEL_NUM     7
#define TOUCH_PAD_NUM8_CHANNEL_NUM     8
#define TOUCH_PAD_NUM9_CHANNEL_NUM     9
#define TOUCH_PAD_NUM10_CHANNEL_NUM    10
#define TOUCH_PAD_NUM11_CHANNEL_NUM    11
#define TOUCH_PAD_NUM12_CHANNEL_NUM    12
#define TOUCH_PAD_NUM13_CHANNEL_NUM    13
#define TOUCH_PAD_NUM14_CHANNEL_NUM    14

#define TOUCH_PAD_NUM1_GPIO_NUM     1
#define TOUCH_PAD_NUM2_GPIO_NUM     2
#define TOUCH_PAD_NUM3_GPIO_NUM     3
#define TOUCH_PAD_NUM4_GPIO_NUM     4
#define TOUCH_PAD_NUM5_GPIO_NUM     5
#define TOUCH_PAD_NUM6_GPIO_NUM     6
#define TOUCH_PAD_NUM7_GPIO_NUM     7
#define TOUCH_PAD_NUM8_GPIO_NUM     8
#define TOUCH_PAD_NUM9_GPIO_NUM     9
#define TOUCH_PAD_NUM10_GPIO_NUM    10
#define TOUCH_PAD_NUM11_GPIO_NUM    11
#define TOUCH_PAD_NUM12_GPIO_NUM    12
#define TOUCH_PAD_NUM13_GPIO_NUM    13
#define TOUCH_PAD_NUM14_GPIO_NUM    14

#define TOUCH_BIT_MASK_ALL          ((1 << TOUCH_SENSOR_PINS) - 1)
#define TOUCH_INTR_MASK_ALL         (TOUCH_INTR_MASK_TIMEOUT | \
                                     TOUCH_INTR_MASK_SCAN_DONE | \
                                     TOUCH_INTR_MASK_INACTIVE | \
                                     TOUCH_INTR_MASK_ACTIVE | \
                                     TOUCH_INTR_MASK_DONE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Touch pad channel */

enum touch_pad_e
{
  TOUCH_PAD_NUM0,     /* Touch pad channel 0 is GPIO0 (NC)   */
  TOUCH_PAD_NUM1,     /* Touch pad channel 1 is GPIO1        */
  TOUCH_PAD_NUM2,     /* Touch pad channel 2 is GPIO2        */
  TOUCH_PAD_NUM3,     /* Touch pad channel 3 is GPIO3        */
  TOUCH_PAD_NUM4,     /* Touch pad channel 4 is GPIO4        */
  TOUCH_PAD_NUM5,     /* Touch pad channel 5 is GPIO5        */
  TOUCH_PAD_NUM6,     /* Touch pad channel 6 is GPIO6        */
  TOUCH_PAD_NUM7,     /* Touch pad channel 7 is GPIO7        */
  TOUCH_PAD_NUM8,     /* Touch pad channel 8 is GPIO8        */
  TOUCH_PAD_NUM9,     /* Touch pad channel 9 is GPIO9        */
  TOUCH_PAD_NUM10,    /* Touch pad channel 10 is GPIO10      */
  TOUCH_PAD_NUM11,    /* Touch pad channel 11 is GPIO11      */
  TOUCH_PAD_NUM12,    /* Touch pad channel 12 is GPIO12      */
  TOUCH_PAD_NUM13,    /* Touch pad channel 13 is GPIO13      */
  TOUCH_PAD_NUM14,    /* Touch pad channel 14 is GPIO14      */
  TOUCH_PAD_ALL       /* Used only by specific functions     */
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

enum touch_intr_mask_e
{
  TOUCH_INTR_MASK_DONE = BIT(0),      /* Measurement done for one of the enabled channels. */
  TOUCH_INTR_MASK_ACTIVE = BIT(1),    /* Active for one of the enabled channels. */
  TOUCH_INTR_MASK_INACTIVE = BIT(2),  /* Inactive for one of the enabled channels. */
  TOUCH_INTR_MASK_SCAN_DONE = BIT(3), /* Measurement done for all the enabled channels. */
  TOUCH_INTR_MASK_TIMEOUT = BIT(4)    /* Timeout for one of the enabled channels. */
};

enum touch_denoise_grade_e
{
  TOUCH_DENOISE_BIT12,    /* Denoise range is 12bit */
  TOUCH_DENOISE_BIT10,    /* Denoise range is 10bit */
  TOUCH_DENOISE_BIT8,     /* Denoise range is 8bit */
  TOUCH_DENOISE_BIT4      /* Denoise range is 4bit */
};

enum touch_denoise_cap_e
{
  TOUCH_DENOISE_CAP_L0,   /* Denoise channel internal reference capacitance is 5pf */
  TOUCH_DENOISE_CAP_L1,   /* Denoise channel internal reference capacitance is 6.4pf */
  TOUCH_DENOISE_CAP_L2,   /* Denoise channel internal reference capacitance is 7.8pf */
  TOUCH_DENOISE_CAP_L3,   /* Denoise channel internal reference capacitance is 9.2pf */
  TOUCH_DENOISE_CAP_L4,   /* Denoise channel internal reference capacitance is 10.6pf */
  TOUCH_DENOISE_CAP_L5,   /* Denoise channel internal reference capacitance is 12.0pf */
  TOUCH_DENOISE_CAP_L6,   /* Denoise channel internal reference capacitance is 13.4pf */
  TOUCH_DENOISE_CAP_L7    /* Denoise channel internal reference capacitance is 14.8pf */
};

/* Touch channel idle state configuration */

enum touch_conn_type_e
{
  TOUCH_CONN_HIGHZ,   /* Idle status of touch channel is high resistance state */
  TOUCH_CONN_GND      /* Idle status of touch channel is ground connection */
};

/* Touch channel IIR filter coefficient configuration.
 *
 * On ESP32-S2, there is an error in the IIR calculation. The magnitude of
 * the error is twice the filter coefficient. So please select a smaller
 * filter coefficient on the basis of meeting the filtering requirements.
 * Recommended filter coefficient selection `IIR_16`.
 */

enum touch_filter_mode_e
{
  TOUCH_FILTER_IIR_4,     /* The filter mode is first-order IIR filter. The coefficient is 4. */
  TOUCH_FILTER_IIR_8,     /* The filter mode is first-order IIR filter. The coefficient is 8. */
  TOUCH_FILTER_IIR_16,    /* The filter mode is first-order IIR filter. The coefficient is 16 (Typical value). */
  TOUCH_FILTER_IIR_32,    /* The filter mode is first-order IIR filter. The coefficient is 32. */
  TOUCH_FILTER_IIR_64,    /* The filter mode is first-order IIR filter. The coefficient is 64. */
  TOUCH_FILTER_IIR_128,   /* The filter mode is first-order IIR filter. The coefficient is 128. */
  TOUCH_FILTER_IIR_256,   /* The filter mode is first-order IIR filter. The coefficient is 256. */
  TOUCH_FILTER_JITTER     /* The filter mode is jitter filter */
};

/* Level of filter applied on the original data against large noise
 * interference.
 *
 * On ESP32-S2, there is an error in the IIR calculation. The magnitude of
 * the error is twice the filter coefficient. So please select a smaller
 * filter coefficient on the basis of meeting the filtering requirements.
 * Recommended filter coefficient selection `IIR_2`.
 */

enum touch_smooth_mode_e
{
  TOUCH_SMOOTH_OFF,   /* No filtering of raw data. */
  TOUCH_SMOOTH_IIR_2, /* Filter the raw data. The coefficient is 2 (Typical value). */
  TOUCH_SMOOTH_IIR_4, /* Filter the raw data. The coefficient is 4. */
  TOUCH_SMOOTH_IIR_8  /* Filter the raw data. The coefficient is 8. */
};

/* Touch sensor shield channel drive capability level */

enum touch_shield_driver_e
{
  TOUCH_SHIELD_DRV_L0,    /* The max equivalent capacitance in shield channel is 40pf */
  TOUCH_SHIELD_DRV_L1,    /* The max equivalent capacitance in shield channel is 80pf */
  TOUCH_SHIELD_DRV_L2,    /* The max equivalent capacitance in shield channel is 120pf */
  TOUCH_SHIELD_DRV_L3,    /* The max equivalent capacitance in shield channel is 160pf */
  TOUCH_SHIELD_DRV_L4,    /* The max equivalent capacitance in shield channel is 200pf */
  TOUCH_SHIELD_DRV_L5,    /* The max equivalent capacitance in shield channel is 240pf */
  TOUCH_SHIELD_DRV_L6,    /* The max equivalent capacitance in shield channel is 280pf */
  TOUCH_SHIELD_DRV_L7     /* The max equivalent capacitance in shield channel is 320pf */
};

#endif /* __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_TOUCH_H */
