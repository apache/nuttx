/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_osc.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_OSC_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_OSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_OSC_CR_OFFSET  0x0000 /* OSC Control Register */
#define KINETIS_OSC_DIV_OFFSET 0x0002 /* OSC CLock divider register */

/* Register Addresses *******************************************************/

#define KINETIS_OSC_CR         (KINETIS_OSC_BASE+KINETIS_OSC_CR_OFFSET)
#define KINETIS_OSC_DIV        (KINETIS_OSC_BASE+KINETIS_OSC_DIV_OFFSET)

/* Register Bit Definitions *************************************************/

/* OSC Control Register (8-bit) */

#define OSC_CR_ERCLKEN         (1 << 7)  /* Bit 7:  External Reference Enable */
                                         /* Bit 6:  Reserved */
#define OSC_CR_EREFSTEN        (1 << 5)  /* Bit 5:  External Reference Stop Enable */
                                         /* Bit 4:  Reserved */
#define OSC_CR_SC2P            (1 << 3)  /* Bit 3:  Oscillator 2 pF Capacitor Load Configure */
#define OSC_CR_SC4P            (1 << 2)  /* Bit 2:  Oscillator 4 pF Capacitor Load Configure */
#define OSC_CR_SC8P            (1 << 1)  /* Bit 1:  Oscillator 8 pF Capacitor Load Configure */
#define OSC_CR_SC16P           (1 << 0)  /* Bit 0:  Oscillator 16 pF Capacitor Load Configure */

/* OSC Control Register (8-bit) */

                                         /* Bits 0-5:  Reserved */
#define OSC_DIV_ERPS_SHIFT      6        /* Bits 6-7:  ERCLK prescaler */
#define OSC_DIV_ERPS_MASK       (3 << OSC_DIV_ERPS_SHIFT)
#  define OSC_DIV_ERPS_DIV1     (0 << OSC_DIV_ERPS_SHIFT) /* The divisor ratio is 1 */
#  define OSC_DIV_ERPS_DIV2     (1 << OSC_DIV_ERPS_SHIFT) /* The divisor ratio is 2 */
#  define OSC_DIV_ERPS_DIV3     (2 << OSC_DIV_ERPS_SHIFT) /* The divisor ratio is 4 */
#  define OSC_DIV_ERPS_DIV8     (3 << OSC_DIV_ERPS_SHIFT) /* The divisor ratio is 8 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_OSC_H */
