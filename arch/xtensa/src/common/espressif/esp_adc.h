/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_adc.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSI_ESP_ADC_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSI_ESP_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP_ADC_MAX_CHANNELS 10

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: esp_adc_initialize
 *
 * Description:
 *   This function initializes the specified ADC device with the provided
 *   configuration.
 *
 * Input Parameters:
 *   adc_num      - The ADC unit number.
 *   channel_list - List of channels to be configured for the ADC unit.
 *
 * Returned Value:
 *   Returns a valid pointer to the ADC device structure on success; NULL on
 *   any failure.
 *
 ****************************************************************************/

struct adc_dev_s *esp_adc_initialize(int adc_num,
                                     const uint8_t *channel_list);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSI_ESP_ADC_H */
