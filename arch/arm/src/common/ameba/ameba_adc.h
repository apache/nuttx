/****************************************************************************
 * arch/arm/src/common/ameba/ameba_adc.h
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

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_ADC_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Ameba ADC is a single converter with one channel-switch list: the
 * board hands ameba_adc_register() a set of channels to sample and, for each
 * external channel, the analog pad it is wired to.  A single /dev/adc0
 * device then samples the whole list on every ANIOC_TRIGGER (the SDK does
 * not enable the hardware software-trigger path on this chip, so the driver
 * reads one sweep of the list through the auto channel-switch FIFO -- see
 * ameba_adc.c).
 *
 * Channels the board wires to an external pad give that pad here (encoded
 * with the AMEBA_PA()/AMEBA_PB() PinName codes the GPIO driver uses); the
 * chip's internal channels (temperature, VBAT, ...) have no pad and carry
 * AMEBA_ADC_PIN_NC.
 */

/* Sentinel pad code for an internal ADC channel that has no analog pad. */

#define AMEBA_ADC_PIN_NC      0xff

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ameba_adc_register
 *
 * Description:
 *   Configure the Ameba ADC for a list of channels and register it with the
 *   NuttX ADC character driver at the given path (typically "/dev/adc0").
 *   Every channel in the list is sampled, in list order, on each
 *   ANIOC_TRIGGER; the raw conversion value of each is delivered to the
 *   upper half tagged with the hardware channel number.
 *
 * Input Parameters:
 *   path     - The device path to register, e.g. "/dev/adc0".
 *   channels - Array of hardware ADC channel numbers to sample.
 *   pins     - Array giving, for each channel, the analog pad it is wired to
 *              (encoded with AMEBA_PA()/AMEBA_PB()), or AMEBA_ADC_PIN_NC for
 *              an internal channel that has no pad.
 *   nchan    - Number of entries in channels/pins (clamped to the
 *              converter's channel-list length).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ameba_adc_register(const char *path, const uint8_t *channels,
                       const uint8_t *pins, unsigned int nchan);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_ADC_H */
