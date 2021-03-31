/****************************************************************************
 * arch/avr/src/at32uc3/at32uc3_wdt.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_WDT_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define AVR32_WDT_CTRL_OFFSET 0x00 /* Control Register */
#define AVR32_WDT_CLR_OFFSET  0x04 /* Clear Register */

/* Register Addresses *******************************************************/

#define AVR32_WDT_CTRL        (AVR32_WDT_BASE+AVR32_WDT_CTRL_OFFSET)
#define AVR32_WDT_CLR         (AVR32_WDT_BASE+AVR32_WDT_CLR_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* Control Register Bit-field Definitions */

#define WDT_CTRL_EN          (1 << 0) /* Bit 0: WDT Enable */
#define WDT_CTRL_PSEL_SHIFT  (18)     /* Bits 8-12: Prescale Select */
#define WDT_CTRL_PSEL_MASK   (0x1f << WDT_CTRL_PSEL_SHIFT)
#define WDT_CTRL_KEY_SHIFT   (24)     /* Bits 24-31: Write protection key */
#define WDT_CTRL_KEY_MASK    (0xff << WDT_CTRL_KEY_SHIFT)

/* Clear Register Bit-field Definitions.  These registers have no bit fields:
 * "Writing periodically any value to this field when the WDT is enabled,
 * within the watchdog time-out period, will prevent a watchdog reset."
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_WDT_H */
