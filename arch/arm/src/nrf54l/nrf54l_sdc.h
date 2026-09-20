/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_sdc.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_NRF54L_SDC_H
#define __ARCH_ARM_SRC_NRF54L_NRF54L_SDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_BLUETOOTH
#  define NRF54L_SDC_CENTRAL_COUNT (CONFIG_BLUETOOTH_MAX_CONN - CONFIG_NRF54L_SDC_PERIPHERAL_COUNT)
#else
#  define NRF54L_SDC_CENTRAL_COUNT (CONFIG_NRF54L_SDC_MAX_COUNT - CONFIG_NRF54L_SDC_PERIPHERAL_COUNT)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int nrf54l_sdc_initialize(void);
uint8_t nrf54l_sdc_reset(void);
size_t nrf54l_sdc_command(const uint8_t *command, uint8_t *event);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_NRF54L_NRF54L_SDC_H */
