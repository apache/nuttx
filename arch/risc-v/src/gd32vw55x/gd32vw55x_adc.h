/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_adc.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_ADC_H
#define __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_ADC_H

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

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: gd32_adc_initialize
 *
 * Description:
 *   Initialize the ADC.  The GD32VW55x has a single ADC, so 'intf' must be
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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_ADC_H */
