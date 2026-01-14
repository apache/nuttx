/****************************************************************************
 * arch/arm/src/nrf91/hardware/nrf91_regulators.h
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

#ifndef __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_REGULATORS_H
#define __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_REGULATORS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf91_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF91_REGULATORS_SYSTEMOFF_OFFSET     0x500  /* System OFF register */
#define NRF91_REGULATORS_EXTPOFCON_OFFSET     0x514  /* External power failure warning configuration */
#define NRF91_REGULATORS_DCDCEN_OFFSET        0x578  /* Enable DC/DC mode of the main voltage regulator */

/* Register definitions *****************************************************/

#define NRF91_REGULATORS_SYSTEMOFF            (NRF91_REGULATORS_BASE + NRF91_REGULATORS_SYSTEMOFF_OFFSET)
#define NRF91_REGULATORS_EXTPOFCON            (NRF91_REGULATORS_BASE + NRF91_REGULATORS_EXTPOFCON_OFFSET)
#define NRF91_REGULATORS_DCDCEN               (NRF91_REGULATORS_BASE + NRF91_REGULATORS_DCDCEN_OFFSET)

/* Register bit definitions *************************************************/

#define REGULATORS_DCDCEN_ENABLE              (1 << 0)

#endif /* __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_REGULATORS_H */
