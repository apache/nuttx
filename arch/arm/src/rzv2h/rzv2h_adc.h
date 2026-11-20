/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_adc.h
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

#ifndef __ARCH_ARM_SRC_RZV2H_RZV2H_ADC_H
#define __ARCH_ARM_SRC_RZV2H_RZV2H_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <arch/chip/adc.h>

struct adc_dev_s;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RZV2H_ADC_CHANNEL_BIT(n) (UINT32_C(1) << (n))
#define RZV2H_ADC_CHANNEL_MASK \
  ((UINT32_C(1) << RZV2H_ADC_NCHANNELS) - UINT32_C(1))

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Return the single ADC_E unit 0 lower-half instance. Bits 0 through 7 of
 * channel_mask select ANI000 through ANI007.
 */

struct adc_dev_s *rzv2h_adc_initialize(uint32_t channel_mask);

#endif /* __ARCH_ARM_SRC_RZV2H_RZV2H_ADC_H */
