/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc43_dac.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_DAC_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define LPC43_DAC_CR_OFFSET     0x0000 /* D/A Converter Register */
#define LPC43_DAC_CTRL_OFFSET   0x0004 /* DAC Control register */
#define LPC43_DAC_CNTVAL_OFFSET 0x0008 /* DAC Counter Value register */

/* Register addresses *******************************************************/

#define LPC43_DAC_CR            (LPC43_DAC_BASE+LPC43_DAC_CR_OFFSET)
#define LPC43_DAC_CTRL          (LPC43_DAC_BASE+LPC43_DAC_CTRL_OFFSET)
#define LPC43_DAC_CNTVAL        (LPC43_DAC_BASE+LPC43_DAC_CNTVAL_OFFSET)

/* Register bit definitions *************************************************/

/* D/A Converter Register */

                                          /* Bits 0-5: Reserved */
#define DAC_CR_VALUE_SHIFT      (6)       /* Bits 6-15: Controls voltage on the AOUT pin */
#define DAC_CR_VALUE_MASK       (0x3ff << DAC_CR_VALUE_SHIFT)
#define DAC_CR_BIAS             (1 << 16) /* Bit 16: Controls DAC settling time */
                                          /* Bits 17-31: Reserved */

/* DAC Control register */

#define DAC_CTRL_INTDMAREQ      (1 << 0) /* Bit 0: Timer timed out */
#define DAC_CTRL_DBLBUFEN       (1 << 1) /* Bit 1: Enable DACR double-buffering */
#define DAC_CTRL_CNTEN          (1 << 2) /* Bit 2: Enable timeout counter */
#define DAC_CTRL_DMAEN          (1 << 3) /* Bit 3: Enable DMA access */
                                         /* Bits 4-31: Reserved */

/* DAC Counter Value register */

#define DAC_CNTVAL_SHIFT        (0)      /* Bits 0-15: Reload value for DAC interrupt/DMA timer */
#define DAC_CNTVAL_MASK         (0xffff << DAC_CNTVAL_SHIFT)
                                         /* Bits 8-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_DAC_H */
