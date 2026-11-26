/****************************************************************************
 * arch/arm/src/efr32/hardware/efr32_msc.h
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

#ifndef __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_MSC_H
#define __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_MSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/efr32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define EFR32_MSC_READCTRL_OFFSET  0x0004  /* Read control */
#define EFR32_MSC_LOCK_OFFSET      0x003c  /* Configuration lock */

/* Register addresses *******************************************************/

#define EFR32_MSC_READCTRL         (EFR32_MSC_BASE + EFR32_MSC_READCTRL_OFFSET)
#define EFR32_MSC_LOCK             (EFR32_MSC_BASE + EFR32_MSC_LOCK_OFFSET)

/* Register bit definitions *************************************************/

/* Read control: flash wait states */

#define MSC_READCTRL_MODE_SHIFT    (20)
#define MSC_READCTRL_MODE_MASK     (0x3 << MSC_READCTRL_MODE_SHIFT)
#  define MSC_READCTRL_MODE_WS0    (0 << MSC_READCTRL_MODE_SHIFT)
#  define MSC_READCTRL_MODE_WS1    (1 << MSC_READCTRL_MODE_SHIFT)
#  define MSC_READCTRL_MODE_WS2    (2 << MSC_READCTRL_MODE_SHIFT)
#  define MSC_READCTRL_MODE_WS3    (3 << MSC_READCTRL_MODE_SHIFT)

/* Configuration lock */

#define MSC_LOCK_LOCKKEY_UNLOCK    0x1b71

#endif /* __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_MSC_H */
