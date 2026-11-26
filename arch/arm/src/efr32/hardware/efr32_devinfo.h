/****************************************************************************
 * arch/arm/src/efr32/hardware/efr32_devinfo.h
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

#ifndef __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_DEVINFO_H
#define __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_DEVINFO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/efr32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device information page (read only, factory programmed) */

#define EFR32_DEVINFO_PART_OFFSET            0x0004  /* Part information */
#define EFR32_DEVINFO_EUI64L_OFFSET          0x0048  /* EUI-64, low word */
#define EFR32_DEVINFO_EUI64H_OFFSET          0x004c  /* EUI-64, high word */
#define EFR32_DEVINFO_HFRCODPLLCAL_OFFSET(n) (0x0058 + 4 * (n))

#define EFR32_DEVINFO_PART         (EFR32_DEVINFO_BASE + EFR32_DEVINFO_PART_OFFSET)
#define EFR32_DEVINFO_EUI64L       (EFR32_DEVINFO_BASE + EFR32_DEVINFO_EUI64L_OFFSET)
#define EFR32_DEVINFO_EUI64H       (EFR32_DEVINFO_BASE + EFR32_DEVINFO_EUI64H_OFFSET)
#define EFR32_DEVINFO_HFRCODPLLCAL(n) \
  (EFR32_DEVINFO_BASE + EFR32_DEVINFO_HFRCODPLLCAL_OFFSET(n))

/* HFRCODPLLCAL index of the 80 MHz band, used for 72.6 to 80 MHz */

#define EFR32_DEVINFO_HFRCODPLLCAL_80MHZ     16

/* User data page: the HFXO tuning value written by the kit's factory
 * programming (0xffff when not present).
 */

#define EFR32_USERDATA_MFG_CTUNE   (EFR32_USERDATA_BASE + 0x0100)

#endif /* __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_DEVINFO_H */
