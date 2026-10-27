/****************************************************************************
 * arch/arm/src/gd32w51x/gd32w51x_adc.h
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

#ifndef __ARCH_ARM_SRC_GD32W51X_GD32W51X_ADC_H
#define __ARCH_ARM_SRC_GD32W51X_GD32W51X_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/adc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The analog input pins are configured by the board logic in board.h with
 * the following names.  Channel 9 (temperature sensor) and channel 10
 * (internal reference voltage) are internal and have no pin:
 *
 *   GPIO_ADC_IN0 ... GPIO_ADC_IN8
 */

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: gd32_adc_initialize
 *
 * Description:
 *   Initialize the ADC.  The GD32W51x has a single ADC, so 'intf' must be
 *   zero.
 *
 * Input Parameters:
 *   intf       - Must be 0
 *   chanlist   - The list of channels (0..10) to convert, in the order of
 *                the conversion sequence
 *   nchannels  - Number of channels of chanlist, at most 9
 *
 * Returned Value:
 *   On success, a pointer to the ADC device structure is returned.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

struct adc_dev_s *gd32_adc_initialize(int intf, const uint8_t *chanlist,
                                      int nchannels);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32W51X_GD32W51X_ADC_H */
