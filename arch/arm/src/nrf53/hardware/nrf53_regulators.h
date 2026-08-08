/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_regulators.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_REGULATORS_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_REGULATORS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF53_REGULATORS_MAINREGSTATUS_OFFSET 0x428  /* Main supply status */
#define NRF53_REGULATORS_SYSTEMOFF_OFFSET     0x500  /* System OFF register */
#define NRF53_REGULATORS_POFCON_OFFSET        0x510  /* Power-fail comparator configuration */
#define NRF53_REGULATORS_MAINDCDCEN_OFFSET    0x704  /* DC/DC enable register for VREGMAIN */
#define NRF53_REGULATORS_RADIODCDCEN_OFFSET   0x904  /* DC/DC enable register for VREGRADIO */
#define NRF53_REGULATORS_HDCDCEN_OFFSET       0xb00  /* DC/DC enable register for VREGH */

/* Register definitions *****************************************************/

#define NRF53_REGULATORS_MAINREGSTATUS        (NRF53_REGULATORS_BASE + NRF53_REGULATORS_MAINREGSTATUS_OFFSET)
#define NRF53_REGULATORS_SYSTEMOFF            (NRF53_REGULATORS_BASE + NRF53_REGULATORS_SYSTEMOFF_OFFSET)
#define NRF53_REGULATORS_POFCON               (NRF53_REGULATORS_BASE + NRF53_REGULATORS_POFCON_OFFSET)
#define NRF53_REGULATORS_MAINDCDCEN           (NRF53_REGULATORS_BASE + NRF53_REGULATORS_MAINDCDCEN_OFFSET)
#define NRF53_REGULATORS_RADIODCDCEN          (NRF53_REGULATORS_BASE + NRF53_REGULATORS_RADIODCDCEN_OFFSET)
#define NRF53_REGULATORS_HDCDCEN              (NRF53_REGULATORS_BASE + NRF53_REGULATORS_HDCDCEN_OFFSET)

/* Register bit definitions *************************************************/

#define REGULATORS_DCDCEN_ENABLE              (1 << 0)

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_REGULATORS_H */
