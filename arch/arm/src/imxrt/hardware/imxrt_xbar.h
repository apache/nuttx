/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_xbar.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_XBAR_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_XBAR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "hardware/imxrt_memorymap.h"

#if defined(CONFIG_ARCH_FAMILY_IMXRT102x)
#  include "hardware/rt102x/imxrt102x_xbar.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT105x)
#  include "hardware/rt105x/imxrt105x_xbar.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT106x)
#  include "hardware/rt106x/imxrt106x_xbar.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT117x)
#  include "hardware/rt117x/imxrt117x_xbar.h"
#else
#  error Unrecognized i.MX RT architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_SEL_PER_REG    2

#define IMXRT_SEL1_SHIFTS    8    /* Bits 8-14: Input (XBARA_INn) to be muxed to
                                   *   XBARA_OUTm */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_XBAR_H */
