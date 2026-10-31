/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_dppi.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_DPPI_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_DPPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets for DPPIC ***********************************************/

#define NRF54L_DPPIC_TASKS_CHGEN_OFFSET(n)      (0x0000 + ((n) * 8)) /* Enable channel group n */
#define NRF54L_DPPIC_TASKS_CHGDIS_OFFSET(n)     (0x0004 + ((n) * 8)) /* Disable channel group n */
#define NRF54L_DPPIC_SUBSCRIBE_CHGEN_OFFSET(n)  (0x0080 + ((n) * 8)) /* Subscribe to CHG[n].EN */
#define NRF54L_DPPIC_SUBSCRIBE_CHGDIS_OFFSET(n) (0x0084 + ((n) * 8)) /* Subscribe to CHG[n].DIS */
#define NRF54L_DPPIC_CHEN_OFFSET                0x0500               /* Channel enable */
#define NRF54L_DPPIC_CHENSET_OFFSET             0x0504               /* Channel enable set */
#define NRF54L_DPPIC_CHENCLR_OFFSET             0x0508               /* Channel enable clear */
#define NRF54L_DPPIC_CHG_OFFSET(n)              (0x0800 + ((n) * 4)) /* Channel group n */

/* Register bit definitions *************************************************/

#define DPPIC_CHEN_CH(n)                        (1 << (n))           /* Channel n */
#define DPPIC_SUBSCRIBE_CHIDX_MASK              (0xff)               /* Bits 0-7: DPPI channel */
#define DPPIC_SUBSCRIBE_EN                      (1 << 31)            /* Bit 31: Enable subscription */

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_DPPI_H */
