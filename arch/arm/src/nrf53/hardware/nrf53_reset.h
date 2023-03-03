/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_reset.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_RESET_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_RESET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF53_RESET_RESETREAS_OFFSET        0x400  /* Reset reason */
#define NRF53_RESET_NETWORK_FORCEOFF_OFFSET 0x614  /* Force off power and clock in network core */

/* Register definitions *****************************************************/

#define NRF53_RESET_RESETREAS         (NRF53_RESET_BASE + NRF53_RESET_RESETREAS_OFFSET)
#define NRF53_RESET_NETWORK_FORCEOFF  (NRF53_RESET_BASE + NRF53_RESET_NETWORK_FORCEOFF_OFFSET)

/* Register bit definitions *************************************************/

#define RESET_NETWORK_FORCEOFF_RELEASE (0 << 0) /* 0b: Release force off signal */
#define RESET_NETWORK_FORCEOFF_HOLD    (1 << 0) /* 1b: Hold force off signal */

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_RESET_H */
