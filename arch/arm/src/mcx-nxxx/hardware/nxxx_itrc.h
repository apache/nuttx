/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/nxxx_itrc.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_ITRC_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_ITRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/nxxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define NXXX_ITRC_OUT4_SEL0_OFFSET       0x0028
#define NXXX_ITRC_OUT4_SEL1_OFFSET       0x002c

/* Register Addresses *******************************************************/

#define NXXX_ITRC_OUT4_SEL0              (NXXX_ITRC0_BASE + NXXX_ITRC_OUT4_SEL0_OFFSET)
#define NXXX_ITRC_OUT4_SEL1              (NXXX_ITRC0_BASE + NXXX_ITRC_OUT4_SEL1_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* OUT_SEL IN0_SELn - digital glitch detector (dGDET), bits [1:0] */

#define ITRC_OUT_SEL_IN0_SELN_SHIFT      (0)
#define ITRC_OUT_SEL_IN0_SELN_MASK       (0x3u << ITRC_OUT_SEL_IN0_SELN_SHIFT)
#define ITRC_OUT_SEL_IN0_SELN(x)         (((x) << ITRC_OUT_SEL_IN0_SELN_SHIFT) & \
                                          ITRC_OUT_SEL_IN0_SELN_MASK)
#define ITRC_OUT_SEL_IN0_SELN_NONE       ITRC_OUT_SEL_IN0_SELN(0x2)

/* OUT_SEL IN9_SELn - analog glitch detector (aGDET), bits [19:18] */

#define ITRC_OUT_SEL_IN9_SELN_SHIFT      (18)
#define ITRC_OUT_SEL_IN9_SELN_MASK       (0x3u << ITRC_OUT_SEL_IN9_SELN_SHIFT)
#define ITRC_OUT_SEL_IN9_SELN(x)         (((x) << ITRC_OUT_SEL_IN9_SELN_SHIFT) & \
                                          ITRC_OUT_SEL_IN9_SELN_MASK)
#define ITRC_OUT_SEL_IN9_SELN_NONE       ITRC_OUT_SEL_IN9_SELN(0x2)

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_ITRC_H */
