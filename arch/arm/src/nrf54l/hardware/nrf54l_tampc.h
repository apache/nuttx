/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_tampc.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_TAMPC_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_TAMPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF54L_TAMPC_DBGEN_OFFSET    0x0500 /* Invasive debug protection */
#define NRF54L_TAMPC_NIDEN_OFFSET    0x0508 /* Non-invasive debug protection */
#define NRF54L_TAMPC_SPIDEN_OFFSET   0x0510 /* Secure invasive debug protection */
#define NRF54L_TAMPC_SPNIDEN_OFFSET  0x0518 /* Secure non-invasive debug protection */
#define NRF54L_TAMPC_AP_DBGEN_OFFSET 0x0700 /* Access port debug protection */

/* Register addresses *******************************************************/

#define NRF54L_TAMPC_DBGEN    (NRF54L_TAMPC_BASE + NRF54L_TAMPC_DBGEN_OFFSET)
#define NRF54L_TAMPC_NIDEN    (NRF54L_TAMPC_BASE + NRF54L_TAMPC_NIDEN_OFFSET)
#define NRF54L_TAMPC_SPIDEN   (NRF54L_TAMPC_BASE + NRF54L_TAMPC_SPIDEN_OFFSET)
#define NRF54L_TAMPC_SPNIDEN  (NRF54L_TAMPC_BASE + NRF54L_TAMPC_SPNIDEN_OFFSET)
#define NRF54L_TAMPC_AP_DBGEN (NRF54L_TAMPC_BASE + NRF54L_TAMPC_AP_DBGEN_OFFSET)

/* Register bit definitions *************************************************/

#define TAMPC_KEY    (0x50fa0000)
#define TAMPC_UNLOCK (TAMPC_KEY | 0xf0)
#define TAMPC_OPEN   (TAMPC_KEY | 1)
#define TAMPC_LOCKED (1 << 1)

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_TAMPC_H */
