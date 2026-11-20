/****************************************************************************
 * arch/arm/include/rzv2h/adc.h
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

#ifndef __ARCH_ARM_INCLUDE_RZV2H_ADC_H
#define __ARCH_ARM_INCLUDE_RZV2H_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RZV2H_ADC_NCHANNELS 8

/* Return accumulated Window A events and atomically clear the stored state.
 * The argument is a pointer to struct rzv2h_adc_window_status_s.
 */

#define ANIOC_RZV2H_WINDOW_A_STATUS _ANIOC(AN_RZV2H_FIRST)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rzv2h_adc_window_status_s
{
  /* Bit n is set when channel n has matched since the previous status
   * read.
   */

  uint32_t pending_mask;

  /* Sum of all channel matches since the previous status read. */

  uint32_t total_events;

  /* Number of matches for each channel since the previous status read. */

  uint32_t channel_events[RZV2H_ADC_NCHANNELS];
};

#endif /* __ARCH_ARM_INCLUDE_RZV2H_ADC_H */
