/****************************************************************************
 * arch/arm/src/mx8mp/hardware/mx8mp_rdc.h
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

/* Reference:
 *   "i.MX 8M Plus Applications Processor Reference Manual",
 *   Document Number: IMX8MPRM Rev. 1, 06/2021. NXP
 */

#ifndef __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_RDC_H
#define __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_RDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/mx8mp_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RDC Register Addresses ***************************************************/

#define RDC_VIR                      (MX8M_RDC + 0)
#define RDC_STAT                     (MX8M_RDC + 0x024)
#define RDC_INTCTRL                  (MX8M_RDC + 0x024)
#define RDC_INTSTAT                  (MX8M_RDC + 0x02C)

/* RDC Common Register Offsets **********************************************/

#define RDC_DID_MASK                 0xf
#define RDC_DID                      (getreg32(RDC_STAT) & RDC_DID_MASK)

#endif /* __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_RDC_H */
