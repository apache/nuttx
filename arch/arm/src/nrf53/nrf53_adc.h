/****************************************************************************
 * arch/arm/src/nrf53/nrf53_adc.h
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

#ifndef __ARCH_ARM_SRC_NRF53_NRF53_ADC_H
#define __ARCH_ARM_SRC_NRF53_NRF53_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#include <nuttx/analog/adc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ADC input */

enum nrf53_adc_ain_e
{
  NRF53_ADC_IN_NC       = 0,   /* Not connected */
  NRF53_ADC_IN_IN0      = 1,   /* Analog input 0 */
  NRF53_ADC_IN_IN1      = 2,   /* Analog input 1 */
  NRF53_ADC_IN_IN2      = 3,   /* Analog input 2 */
  NRF53_ADC_IN_IN3      = 4,   /* Analog input 3 */
  NRF53_ADC_IN_IN4      = 5,   /* Analog input 4 */
  NRF53_ADC_IN_IN5      = 6,   /* Analog input 5 */
  NRF53_ADC_IN_IN6      = 7,   /* Analog input 6 */
  NRF53_ADC_IN_IN7      = 8,   /* Analog input 7 */
  NRF53_ADC_IN_VDD      = 9,   /* VDD */
  NRF53_ADC_IN_VDDHDIV5 = 10,  /* VDDH/5 */
};

/* Resistor control */

enum nrf53_adc_res_e
{
  NRF53_ADC_RES_BYPASS   = 0,   /* Bypass resistor ladder */
  NRF53_ADC_RES_PULLDOWN = 1,   /* Pull-down to GND */
  NRF53_ADC_RES_PULLUP   = 2,   /* Pull-up to VDD */
  NRF53_ADC_RES_VDD_2    = 3    /* Set input at VDD/2 */
};

/* Gain control */

enum nrf53_adc_gain_e
{
  NRF53_ADC_GAIN_1_6 = 0,       /* 1/6 */
  NRF53_ADC_GAIN_1_5 = 1,       /* 1/5 */
  NRF53_ADC_GAIN_1_4 = 2,       /* 1/4 */
  NRF53_ADC_GAIN_1_3 = 3,       /* 1/3 */
  NRF53_ADC_GAIN_1_2 = 4,       /* 1/2 */
  NRF53_ADC_GAIN_1   = 5,       /* 1 */
  NRF53_ADC_GAIN_2   = 6,       /* 2 */
  NRF53_ADC_GAIN_4   = 7        /* 4 */
};

/* Reference control */

enum nrf53_adc_refsel_e
{
  NRF53_ADC_REFSEL_INTERNAL = 0, /* Internal reference (0.6V) */
  NRF53_ADC_REFSEL_VDD_4    = 1  /* VDD/4 as reference */
};

/* Acquisition time control */

enum nrf53_adc_tacq_e
{
  NRF53_ADC_TACQ_3US  = 0,      /* 3 us */
  NRF53_ADC_TACQ_5US  = 1,      /* 5 us */
  NRF53_ADC_TACQ_10US = 2,      /* 10 us */
  NRF53_ADC_TACQ_15US = 3,      /* 15 us */
  NRF53_ADC_TACQ_20US = 4,      /* 20 us */
  NRF53_ADC_TACQ_40US = 5       /* 40 us */
};

/* ADC mode control */

enum nrf53_adc_mode_e
{
  NRF53_ADC_MODE_SE   = 0,      /* Single-ended mode */
  NRF53_ADC_MODE_DIFF = 1       /* Differentail mode */
};

/* ADC burst control */

enum nrf53_adc_burst_e
{
  NRF53_ADC_BURST_DISABLE = 0,  /* Disable burst mode */
  NRF53_ADC_BURST_ENABLE  = 1   /* Enable burst mode */
};

/* NRF53 ADC channel configuration */

struct nrf53_adc_channel_s
{
  uint32_t p_psel;              /* P pin */
  uint32_t n_psel;              /* N pin */
#ifdef CONFIG_NRF53_SAADC_LIMITS
  uint16_t limith;              /* High limit */
  uint16_t limitl;              /* Low limit */
#endif
  uint8_t resp:2;               /* Positive chan resistor */
  uint8_t resn:2;               /* Negative chan resistor */
  uint8_t gain:3;               /* Gain control */
  uint8_t refsel:1;             /* Reference control */
  uint8_t tacq:3;               /* Acquisition time */
  uint8_t mode:1;               /* Singe-ended or differential mode */
  uint8_t burst:1;              /* Burst mode */
  uint8_t _res:3;               /* Reserved */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_adcinitialize
 *
 * Description:
 *   Initialize the ADC. See nrf53_adc.c for more details.
 *
 * Input Parameters:
 *   chanlist  - channels configuration
 *   nchannels - number of channels
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *nrf53_adcinitialize(
    const struct nrf53_adc_channel_s *chan,
    int channels);

#endif  /* __ARCH_ARM_SRC_NRF53_NRF53_ADC_H */
