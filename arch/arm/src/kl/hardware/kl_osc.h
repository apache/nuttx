/****************************************************************************
 * arch/arm/src/kl/hardware/kl_osc.h
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

#ifndef __ARCH_ARM_SRC_KL_HARDWARE_KL_OSC_H
#define __ARCH_ARM_SRC_KL_HARDWARE_KL_OSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KL_OSC_CR_OFFSET       0x0000 /* OSC Control Register */

/* Register Addresses *******************************************************/

#define KL_OSC_CR              (KL_OSC_BASE+KL_OSC_CR_OFFSET)

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

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KL_HARDWARE_KL_OSC_H */
