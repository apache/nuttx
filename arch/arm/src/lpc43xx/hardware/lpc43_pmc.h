/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc43_pmc.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_PMC_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_PMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define LPC43_PD0_SLEEP0_HWENA_OFFSET 0x0000 /* Hardware sleep event enable register */
#define LPC43_PD0_SLEEP0_MODE_OFFSET  0x001c /* Power-down mode control register */

/* Register Addresses *******************************************************/

#define LPC43_PD0_SLEEP0_HWENA        (LPC43_PMC_BASE+LPC43_PD0_SLEEP0_HWENA_OFFSET)
#define LPC43_PD0_SLEEP0_MODE         (LPC43_PMC_BASE+LPC43_PD0_SLEEP0_MODE_OFFSET)

/* Register Bit Definitions *************************************************/

/* Hardware sleep event enable register */

#define PD0_SLEEP0_HWENA              (1 << 0)  /* Bit 0:  Enable power down mode */
                                                /* Bits 1-31:  Reserved */

/* Power-down mode control register */

#define PD0_DEEP_SLEEP_MODE           0x003000aa
#define PD0_PWRDOWN_MODE              0x0030fcba
#define PD0_DEEP_PWRDOWN_MODE         0x0030ff7f

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_PMC_H */
