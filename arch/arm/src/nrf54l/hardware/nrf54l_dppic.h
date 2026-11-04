/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_dppic.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_DPPIC_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_DPPIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets for DPPIC ***********************************************/

#define NRF54L_DPPIC_CHEN_OFFSET     0x0500 /* Channel enable */
#define NRF54L_DPPIC_CHENSET_OFFSET  0x0504 /* Channel enable set */
#define NRF54L_DPPIC_CHENCLR_OFFSET  0x0508 /* Channel enable clear */

/* Register Bitfield Definitions for DPPIC **********************************/

#define DPPIC_CHEN_CH(x)             (1 << (x)) /* Channel x */

/* PUBLISH and SUBSCRIBE registers of all peripherals */

#define DPPI_CHIDX_SHIFT             (0)        /* Bits 0-7: Channel index */
#define DPPI_CHIDX_MASK              (0xff << DPPI_CHIDX_SHIFT)
#define DPPI_EN                      (1 << 31)  /* Bit 31: Enable */

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_DPPIC_H */
