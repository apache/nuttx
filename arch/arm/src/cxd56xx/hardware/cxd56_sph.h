/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_sph.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SPH_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SPH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD56_SPH_REQ(n)    (CXD56_SPH_BASE + ((n) * 16) + 0)
#define CXD56_SPH_STS(n)    (CXD56_SPH_BASE + ((n) * 16) + 4)
#define CXD56_SPH_RESET(n)  (CXD56_SPH_BASE + ((n) * 4))

/* Hardware semaphore request */

#define REQ_UNLOCK              0
#define REQ_LOCK                1
#define REQ_RESERVE             2
#define REQ_INTRCLR             3

/* Hardware semaphore status [3:0] */

#define STATE_IDLE              0
#define STATE_LOCKED            1
#define STATE_LOCKEDANDRESERVED 2

#define STS_STATE(sts)  ((sts) & 0xf)
#define LOCK_OWNER(sts) (((sts) >> 16) & 0x1f)
#define RESV_OWNER(sts) (((sts) >> 24) & 0x1f)

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_SPH_H */
