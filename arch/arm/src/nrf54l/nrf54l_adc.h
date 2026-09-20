/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_adc.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_NRF54L_ADC_H
#define __ARCH_ARM_SRC_NRF54L_NRF54L_ADC_H

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

enum nrf54l_adc_ain_e
{
  NRF54L_ADC_IN_NC       = 0,   /* Not connected */
  NRF54L_ADC_IN_IN0      = 1,   /* Analog input 0 */
  NRF54L_ADC_IN_IN1      = 2,   /* Analog input 1 */
  NRF54L_ADC_IN_IN2      = 3,   /* Analog input 2 */
  NRF54L_ADC_IN_IN3      = 4,   /* Analog input 3 */
  NRF54L_ADC_IN_IN4      = 5,   /* Analog input 4 */
  NRF54L_ADC_IN_IN5      = 6,   /* Analog input 5 */
  NRF54L_ADC_IN_IN6      = 7,   /* Analog input 6 */
  NRF54L_ADC_IN_IN7      = 8,   /* Analog input 7 */
  NRF54L_ADC_IN_VDD      = 9,   /* VDD */
  NRF54L_ADC_IN_AVDD     = 10,  /* Internal analog supply */
  NRF54L_ADC_IN_DVDD     = 11,  /* Internal digital supply */
};

/* Gain control */

enum nrf54l_adc_gain_e
{
  NRF54L_ADC_GAIN_2   = 0,       /* 2 */
  NRF54L_ADC_GAIN_1   = 1,       /* 1 */
  NRF54L_ADC_GAIN_2_3 = 2,       /* 2/3 */
  NRF54L_ADC_GAIN_1_2 = 3,       /* 1/2 */
  NRF54L_ADC_GAIN_2_5 = 4,       /* 2/5 */
  NRF54L_ADC_GAIN_1_3 = 5,       /* 1/3 */
  NRF54L_ADC_GAIN_2_7 = 6,       /* 2/7 */
  NRF54L_ADC_GAIN_1_4 = 7        /* 1/4 */
};

/* Reference control */

enum nrf54l_adc_refsel_e
{
  NRF54L_ADC_REFSEL_INTERNAL = 0, /* Internal reference (0.9V) */
  NRF54L_ADC_REFSEL_EXTERNAL = 1  /* External reference */
};

/* Acquisition time control */

enum nrf54l_adc_tacq_e
{
  NRF54L_ADC_TACQ_3US  = 0,      /* 3 us */
  NRF54L_ADC_TACQ_5US  = 1,      /* 5 us */
  NRF54L_ADC_TACQ_10US = 2,      /* 10 us */
  NRF54L_ADC_TACQ_15US = 3,      /* 15 us */
  NRF54L_ADC_TACQ_20US = 4,      /* 20 us */
  NRF54L_ADC_TACQ_40US = 5       /* 40 us */
};

/* ADC mode control */

enum nrf54l_adc_mode_e
{
  NRF54L_ADC_MODE_SE   = 0,      /* Single-ended mode */
  NRF54L_ADC_MODE_DIFF = 1       /* Differential mode */
};

/* ADC burst control */

enum nrf54l_adc_burst_e
{
  NRF54L_ADC_BURST_DISABLE = 0,  /* Disable burst mode */
  NRF54L_ADC_BURST_ENABLE  = 1   /* Enable burst mode */
};

/* NRF54L ADC channel configuration */

struct nrf54l_adc_channel_s
{
  uint32_t p_psel;              /* P pin */
  uint32_t n_psel;              /* N pin */
#ifdef CONFIG_NRF54L_SAADC_LIMITS
  int16_t limith;               /* High limit */
  int16_t limitl;               /* Low limit */
#endif
  uint8_t gain:3;               /* Gain control */
  uint8_t refsel:1;             /* Reference control */
  uint8_t tacq:3;               /* Acquisition time */
  uint8_t mode:1;               /* Single-ended or differential mode */
  uint8_t burst:1;              /* Burst mode */
  uint8_t _res:7;               /* Reserved */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_adcinitialize
 *
 * Description:
 *   Initialize the ADC with a list of channel configurations. External
 *   analog and reference pins must be disconnected from digital inputs,
 *   outputs and pulls by the board before opening the device.
 *
 * Input Parameters:
 *   chan     - Channel configurations
 *   channels - Number of channels
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *nrf54l_adcinitialize(
    const struct nrf54l_adc_channel_s *chan,
    int channels);

#endif /* __ARCH_ARM_SRC_NRF54L_NRF54L_ADC_H */
