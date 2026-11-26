/****************************************************************************
 * arch/arm/src/efr32/hardware/efr32_hfrco.h
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

#ifndef __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_HFRCO_H
#define __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_HFRCO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/efr32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define EFR32_HFRCO_CTRL_OFFSET    0x0004  /* Control */
#define EFR32_HFRCO_CAL_OFFSET     0x0008  /* Calibration */
#define EFR32_HFRCO_STATUS_OFFSET  0x000c  /* Status */
#define EFR32_HFRCO_LOCK_OFFSET    0x001c  /* Configuration lock */

/* Register addresses *******************************************************/

#define EFR32_HFRCO0_CTRL          (EFR32_HFRCO0_BASE + EFR32_HFRCO_CTRL_OFFSET)
#define EFR32_HFRCO0_CAL           (EFR32_HFRCO0_BASE + EFR32_HFRCO_CAL_OFFSET)
#define EFR32_HFRCO0_STATUS        (EFR32_HFRCO0_BASE + EFR32_HFRCO_STATUS_OFFSET)
#define EFR32_HFRCO0_LOCK          (EFR32_HFRCO0_BASE + EFR32_HFRCO_LOCK_OFFSET)

/* Register bit definitions *************************************************/

/* Calibration */

#define HFRCO_CAL_TUNING_SHIFT     (0)
#define HFRCO_CAL_TUNING_MASK      (0x7f << HFRCO_CAL_TUNING_SHIFT)
#define HFRCO_CAL_FREQRANGE_SHIFT  (16)
#define HFRCO_CAL_FREQRANGE_MASK   (0x1f << HFRCO_CAL_FREQRANGE_SHIFT)
#define HFRCO_CAL_CLKDIV_SHIFT     (24)
#define HFRCO_CAL_CLKDIV_MASK      (0x3 << HFRCO_CAL_CLKDIV_SHIFT)

/* Status */

#define HFRCO_STATUS_RDY           (1 << 0)
#define HFRCO_STATUS_FREQBSY       (1 << 1)
#define HFRCO_STATUS_SYNCBUSY      (1 << 2)
#define HFRCO_STATUS_ENS           (1 << 16)
#define HFRCO_STATUS_LOCK          (1 << 31)

#endif /* __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_HFRCO_H */
