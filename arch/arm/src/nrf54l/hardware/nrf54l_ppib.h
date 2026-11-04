/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_ppib.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_PPIB_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_PPIB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets for PPIB ************************************************/

#define NRF54L_PPIB_SUBSCRIBE_SEND_OFFSET(n)  (0x0080 + ((n) * 4)) /* Subscribe to SEND[n] */
#define NRF54L_PPIB_PUBLISH_RECEIVE_OFFSET(n) (0x0180 + ((n) * 4)) /* Publish RECEIVE[n] */
#define NRF54L_PPIB_OVERFLOW_SEND_OFFSET      0x0400               /* SEND task overflow status */

/* Register bit definitions *************************************************/

#define PPIB_SUBSCRIBE_CHIDX_MASK             (0xff)               /* Bits 0-7: DPPI channel */
#define PPIB_SUBSCRIBE_EN                     (1 << 31)            /* Bit 31: Enable subscription */
#define PPIB_PUBLISH_CHIDX_MASK               (0xff)               /* Bits 0-7: DPPI channel */
#define PPIB_PUBLISH_EN                       (1 << 31)            /* Bit 31: Enable publishing */

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_PPIB_H */
