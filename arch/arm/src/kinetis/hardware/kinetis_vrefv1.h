/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_vrefv1.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_VREFV1_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_VREFV1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_VREF_TRM_OFFSET   0x0000 /* VREF Trim Register */
#define KINETIS_VREF_SC_OFFSET    0x0001 /* VREF Status and Control Register */

/* Register Addresses *******************************************************/

#define KINETIS_VREF_TRM          (KINETIS_VREF_BASE+KINETIS_VREF_TRM_OFFSET)
#define KINETIS_VREF_SC           (KINETIS_VREF_BASE+KINETIS_VREF_SC_OFFSET)

/* Register Bit Definitions *************************************************/

/* VREF Trim Register (8-bit) */

#define VREF_TRM_SHIFT            (0)       /* Bits 0-5: Trim bits */
#define VREF_TRM_MASK             (63 << VREF_TRM_SHIFT)
                                            /* Bits 6-7: Reserved */

/* VREF Status and Control Register (8-bit) */

#define VREF_SC_MODE_LV_SHIFT     (0)       /* Bits 0-1: Buffer Mode selection */
#define VREF_SC_MODE_LV_MASK      (3 << VREF_SC_MODE_LV_SHIFT)
#  define VREF_SC_MODE_LV_BANDGAP (0 << VREF_SC_MODE_LV_SHIFT) /* Bandgap on only */
#  define VREF_SC_MODE_LV_LOWPWR  (1 << VREF_SC_MODE_LV_SHIFT) /* Low-power buffer enabled */
#  define VREF_SC_MODE_LV_TIGHT   (2 << VREF_SC_MODE_LV_SHIFT) /* Tight-regulation buffer enabled */

#define VREF_SC_VREFST            (1 << 2)  /* Bit 2:  Internal Voltage Reference stable */
                                            /* Bits 3-5: Reserved */
#define VREF_SC_REGEN             (1 << 6)  /* Bit 6:  Regulator enable */
#define VREF_SC_VREFEN            (1 << 7)  /* Bit 7:  Internal Voltage Reference enable */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_VREFV1_H */
