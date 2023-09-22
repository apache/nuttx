/****************************************************************************
 * arch/arm/src/mx8mp/hardware/mx8mp_gpc.h
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

#ifndef __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_GPC_H
#define __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_GPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/mx8mp_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPC Register Addresses ***************************************************/

#define GPC_PU_PGC_SW_PUP_REQ        (MX8M_GPC + 0x0D8)
#define GPC_PGC_CPU_M7_MAPPING       (MX8M_GPC + 0x1D0)
#define GPC_PU_PWRHSK                (MX8M_GPC + 0x190)

/* GPC PU Register Offsets **************************************************/

#define AUDIOMIX_DOMAIN              (1 << 7)
#define AUDIOMIX_SW_PUP_REQ          (1 << 5)
#define GPC_AUDIOMIX_NOC_PWRDNREQN   (1 << 15)
#define GPC_AUDIOMIX_PWRDNACKN       (1 << 31)

#endif /* __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_GPC_H */
