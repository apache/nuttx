/****************************************************************************
 * arch/arm64/src/imx9/imx9_sar_adc.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_SAR_ADC_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_SAR_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
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
 * Name: imx9_sar_adc_init
 *
 * Description:
 *   Initialize the i.MX9 SAR ADC block for the channels selected in the
 *   channel mask.
 *
 * Input Parameters:
 *   channel_mask - Bitmask of ADC channels that callers intend to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 ****************************************************************************/

int imx9_sar_adc_init(uint32_t channel_mask);

/****************************************************************************
 * Name: imx9_sar_adc_deinit
 *
 * Description:
 *   Deinitialize the i.MX9 SAR ADC block.
 ****************************************************************************/

void imx9_sar_adc_deinit(void);

/****************************************************************************
 * Name: imx9_sar_adc_read_channel
 *
 * Description:
 *   Perform a one-shot conversion on the selected ADC channel and return the
 *   raw result.
 *
 * Input Parameters:
 *   channel - ADC channel to convert
 *   value   - Location to receive the raw ADC result
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 ****************************************************************************/

int imx9_sar_adc_read_channel(uint8_t channel, uint32_t *value);

/****************************************************************************
 * Name: imx9_sar_adc_is_initialized
 *
 * Description:
 *   Report whether the i.MX9 SAR ADC block has been initialized.
 *
 * Returned Value:
 *   true if the SAR ADC is initialized; false otherwise.
 ****************************************************************************/

bool imx9_sar_adc_is_initialized(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_SAR_ADC_H */
